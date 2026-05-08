from __future__ import annotations

import argparse
import heapq
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from rclpy.utilities import remove_ros_args


Point2D = tuple[float, float]


@dataclass(frozen=True)
class RouteEdge:
    edge_id: int
    start_id: int
    end_id: int
    coordinates: list[Point2D]
    metadata: dict[str, Any]


@dataclass(frozen=True)
class RouteGraph:
    nodes: dict[int, Point2D]
    edges: list[RouteEdge]


def _distance(a: Point2D, b: Point2D) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _polyline_length(points: list[Point2D]) -> float:
    return sum(_distance(start, end) for start, end in zip(points, points[1:]))


def _risk_weight(metadata: dict[str, Any]) -> float:
    risk = str(metadata.get('risk', '')).lower()
    if 'physical_dynamic_obstacle' in risk:
        return 4.0
    if 'ramp' in risk:
        return 1.6
    if 'complex' in risk:
        return 1.3
    return 1.0


def _load_route_graph(path: Path) -> RouteGraph:
    data = json.loads(path.read_text(encoding='utf-8'))
    nodes: dict[int, Point2D] = {}
    edges: list[RouteEdge] = []
    for feature in data.get('features', []):
        geometry = feature.get('geometry', {})
        properties = feature.get('properties', {})
        if geometry.get('type') == 'Point':
            node_id = int(properties['id'])
            x, y = geometry['coordinates'][:2]
            nodes[node_id] = (float(x), float(y))
        elif geometry.get('type') == 'MultiLineString':
            coordinates = [
                (float(x), float(y))
                for x, y, *_ in geometry['coordinates'][0]
            ]
            edges.append(
                RouteEdge(
                    edge_id=int(properties['id']),
                    start_id=int(properties['startid']),
                    end_id=int(properties['endid']),
                    coordinates=coordinates,
                    metadata=dict(properties.get('metadata', {})),
                )
            )
    return RouteGraph(nodes=nodes, edges=edges)


def _load_map_metadata(path: Path) -> dict[str, Any]:
    map_data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    return {
        'map': path.as_posix(),
        'image': str(map_data.get('image', '')),
        'resolution': float(map_data.get('resolution', 0.0)),
        'origin': map_data.get('origin', [0.0, 0.0, 0.0]),
    }


def _adjacency(graph: RouteGraph) -> dict[int, list[tuple[int, RouteEdge]]]:
    adjacency: dict[int, list[tuple[int, RouteEdge]]] = {
        node_id: [] for node_id in graph.nodes
    }
    for edge in graph.edges:
        adjacency.setdefault(edge.start_id, []).append((edge.end_id, edge))
        adjacency.setdefault(edge.end_id, []).append((edge.start_id, edge))
    return adjacency


def _edge_coordinates(edge: RouteEdge, source_id: int) -> list[Point2D]:
    if source_id == edge.start_id:
        return edge.coordinates
    return list(reversed(edge.coordinates))


def _shortest_route(
    graph: RouteGraph,
    start_id: int,
    goal_id: int,
    mode: str,
) -> tuple[list[int], list[Point2D], list[int], list[RouteEdge], float, float]:
    adjacency = _adjacency(graph)
    queue: list[tuple[float, int]] = [(0.0, start_id)]
    costs: dict[int, float] = {start_id: 0.0}
    previous: dict[int, tuple[int, RouteEdge]] = {}
    while queue:
        current_cost, current_id = heapq.heappop(queue)
        if current_id == goal_id:
            break
        if current_cost > costs.get(current_id, math.inf):
            continue
        for next_id, edge in adjacency.get(current_id, []):
            length = _polyline_length(edge.coordinates)
            weight = _risk_weight(edge.metadata) if mode != 'baseline' else 1.0
            next_cost = current_cost + length * weight
            if next_cost < costs.get(next_id, math.inf):
                costs[next_id] = next_cost
                previous[next_id] = (current_id, edge)
                heapq.heappush(queue, (next_cost, next_id))
    if goal_id not in costs:
        raise RuntimeError(f'no route found from {start_id} to {goal_id}')

    node_ids = [goal_id]
    edge_sequence: list[tuple[int, RouteEdge]] = []
    current_id = goal_id
    while current_id != start_id:
        parent_id, edge = previous[current_id]
        edge_sequence.append((parent_id, edge))
        current_id = parent_id
        node_ids.append(current_id)
    node_ids.reverse()
    edge_sequence.reverse()

    waypoints: list[Point2D] = []
    edge_ids: list[int] = []
    selected_edges: list[RouteEdge] = []
    for source_id, edge in edge_sequence:
        edge_ids.append(edge.edge_id)
        selected_edges.append(edge)
        segment = _edge_coordinates(edge, source_id)
        if waypoints and segment and waypoints[-1] == segment[0]:
            waypoints.extend(segment[1:])
        else:
            waypoints.extend(segment)
    route_length = _polyline_length(waypoints)
    return node_ids, waypoints, edge_ids, selected_edges, route_length, costs[goal_id]


def _risk_exposure(edges: list[RouteEdge]) -> float:
    return sum(
        _polyline_length(edge.coordinates) * (_risk_weight(edge.metadata) - 1.0)
        for edge in edges
    )


def _resample_waypoints(points: list[Point2D], spacing: float) -> list[Point2D]:
    if len(points) < 2 or spacing <= 0.0:
        return points
    sampled = [points[0]]
    carry = 0.0
    for start, end in zip(points, points[1:]):
        segment_length = _distance(start, end)
        if segment_length == 0.0:
            continue
        direction = (
            (end[0] - start[0]) / segment_length,
            (end[1] - start[1]) / segment_length,
        )
        position = spacing - carry
        while position < segment_length:
            sampled.append((
                start[0] + direction[0] * position,
                start[1] + direction[1] * position,
            ))
            position += spacing
        carry = segment_length - (position - spacing)
    if sampled[-1] != points[-1]:
        sampled.append(points[-1])
    return sampled


def _candidate(
    graph: RouteGraph,
    start_id: int,
    goal_id: int,
    planner_id: str,
    mode: str,
    waypoint_spacing: float,
) -> dict[str, Any]:
    node_ids, points, edge_ids, edges, route_length, score = _shortest_route(
        graph,
        start_id,
        goal_id,
        mode,
    )
    if mode == 'rl_safety_shield':
        points = _resample_waypoints(points, waypoint_spacing)
    risk_score = score - route_length
    return {
        'planner_id': planner_id,
        'runtime_status': (
            'implemented_runtime_baseline'
            if mode == 'baseline'
            else 'research_surrogate_not_trained_runtime'
        ),
        'route_node_ids': node_ids,
        'route_edge_ids': edge_ids,
        'waypoints': [
            {'x': round(x, 4), 'y': round(y, 4), 'yaw': 0.0}
            for x, y in points
        ],
        'path_length_m': round(route_length, 4),
        'risk_adjusted_score': round(score, 4),
        'risk_penalty_m': round(risk_score, 4),
        'route_risk_exposure_m': round(_risk_exposure(edges), 4),
    }


def build_candidate_report(
    map_path: Path,
    route_graph_path: Path,
    start_id: int,
    goal_id: int,
    waypoint_spacing: float = 0.8,
) -> dict[str, Any]:
    graph = _load_route_graph(route_graph_path)
    if start_id not in graph.nodes:
        raise RuntimeError(f'unknown start_id: {start_id}')
    if goal_id not in graph.nodes:
        raise RuntimeError(f'unknown goal_id: {goal_id}')
    candidates = [
        _candidate(
            graph,
            start_id,
            goal_id,
            'nav2_baseline_route',
            'baseline',
            waypoint_spacing,
        ),
        _candidate(
            graph,
            start_id,
            goal_id,
            'pct_style_risk_weighted_route',
            'pct_style',
            waypoint_spacing,
        ),
        _candidate(
            graph,
            start_id,
            goal_id,
            'rl_safety_shield_waypoints',
            'rl_safety_shield',
            waypoint_spacing,
        ),
    ]
    return {
        'schema': 'airos_advanced_planner_candidate.v1',
        'map_metadata': _load_map_metadata(map_path),
        'route_graph': route_graph_path.as_posix(),
        'start_id': start_id,
        'goal_id': goal_id,
        'candidate_count': len(candidates),
        'candidates': candidates,
        'research_boundary': (
            'PCT-style and RL-style entries are deterministic comparison '
            'surrogates for interface hardening; they are not trained PCT/RL '
            'planner runtimes.'
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Generate advanced planner comparison candidates.'
    )
    parser.add_argument('--map', required=True)
    parser.add_argument('--route-graph', required=True)
    parser.add_argument('--start-id', type=int, required=True)
    parser.add_argument('--goal-id', type=int, required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--waypoint-spacing', type=float, default=0.8)
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    report = build_candidate_report(
        Path(args.map).resolve(),
        Path(args.route_graph).resolve(),
        args.start_id,
        args.goal_id,
        args.waypoint_spacing,
    )
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + '\n',
        encoding='utf-8',
    )
    print(f'wrote advanced planner candidates: {output_path}')


if __name__ == '__main__':
    main()
