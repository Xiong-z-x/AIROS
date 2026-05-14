from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path as PathMsg
from nav2_msgs.action import FollowPath, NavigateThroughPoses
from nav2_msgs.msg import SpeedLimit
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from airos_experiments.scan_emulator import (
    OdomAnchor,
    Pose2D,
    _map_pose_from_anchor,
    _pose_from_initial_pose,
    _pose_from_odom,
)
from airos_experiments.sdf_geometry import (
    BoxCollision,
    CloudPoint,
    CollisionGeometry,
    inverse_transform_point,
    iter_obstacle_geometries,
    iter_traversable_boxes,
    load_collision_geometries,
    sample_box_top,
)
from airos_experiments.slam_traversability_graph import (
    build_slam_graph_from_pointcloud,
    build_slam_graph_from_points,
)

_ACTIVE_GOAL_STATUSES = {
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
}


def _goal_is_active(status: int) -> bool:
    return status in _ACTIVE_GOAL_STATUSES


@dataclass
class TerrainNode:
    index: int
    x: float
    y: float
    z: float
    surface_label: str
    edge_margin: float
    surface_local_x: float = 0.0
    surface_local_y: float = 0.0
    surface_half_x: float = 0.0
    surface_half_y: float = 0.0
    surface_width_axis: str = 'y'


@dataclass(frozen=True)
class TerrainGraph:
    nodes: list[TerrainNode]
    adjacency: list[list[tuple[int, float]]]
    terrain_cloud: list[CloudPoint]


def build_terrain_graph(
    world_file: Path,
    grid_resolution: float = 0.40,
    terrain_cloud_resolution: Optional[float] = None,
    robot_radius: float = 0.35,
    support_margin: float = 0.45,
    max_slope_grade: float = 0.55,
    max_step_height: float = 0.34,
    max_surface_transition_height: float = 0.12,
) -> TerrainGraph:
    geometries = load_collision_geometries(world_file)
    traversable_boxes = list(iter_traversable_boxes(geometries))
    obstacles = list(iter_obstacle_geometries(geometries))
    nodes: list[TerrainNode] = []
    terrain_cloud: list[CloudPoint] = []
    cloud_resolution = (
        terrain_cloud_resolution
        if terrain_cloud_resolution and terrain_cloud_resolution > 0.0
        else grid_resolution
    )

    for box in traversable_boxes:
        label = box.label
        margin = _surface_support_margin(label, support_margin)
        terrain_cloud.extend(sample_box_top(box, cloud_resolution, margin=0.0))
        for x, y, z, _ in sample_box_top(box, grid_resolution, margin=margin):
            if _blocked_by_obstacle(
                (x, y, z),
                obstacles,
                clearance=robot_radius,
                current_surface=box,
            ):
                continue
            edge_margin = _surface_edge_margin(box, (x, y, z))
            local = inverse_transform_point(box.transform, (x, y, z))
            nodes.append(
                TerrainNode(
                    index=len(nodes),
                    x=x,
                    y=y,
                    z=z,
                    surface_label=label,
                    edge_margin=edge_margin,
                    surface_local_x=local[0],
                    surface_local_y=local[1],
                    surface_half_x=box.size[0] / 2.0,
                    surface_half_y=box.size[1] / 2.0,
                    surface_width_axis=_box_width_axis(box),
                )
            )

    adjacency = _build_adjacency(
        nodes,
        grid_resolution=grid_resolution,
        max_slope_grade=max_slope_grade,
        max_step_height=max_step_height,
        max_surface_transition_height=max_surface_transition_height,
    )
    return TerrainGraph(nodes=nodes, adjacency=adjacency, terrain_cloud=terrain_cloud)


def build_slam_terrain_graph_from_points(
    points: list[tuple[float, float, float]],
    grid_resolution: float = 0.25,
    robot_radius: float = 0.35,
    support_margin: float = 0.45,
    max_slope_grade: float = 0.55,
    max_step_height: float = 0.34,
    max_surface_transition_height: float = 0.12,
    min_cell_points: int = 2,
    vertical_layer_gap: float = 0.18,
) -> TerrainGraph:
    slam_graph = build_slam_graph_from_points(
        points,
        grid_resolution=grid_resolution,
        max_slope_grade=max_slope_grade,
        max_step_height=max_step_height,
        max_surface_transition_height=max(
            max_surface_transition_height,
            max_step_height,
        ),
        min_cell_points=min_cell_points,
        vertical_layer_gap=vertical_layer_gap,
        obstacle_clearance=robot_radius,
    )
    return _terrain_graph_from_slam_graph(slam_graph)


def build_slam_terrain_graph_from_pointcloud(
    msg: PointCloud2,
    grid_resolution: float = 0.25,
    robot_radius: float = 0.35,
    support_margin: float = 0.45,
    max_slope_grade: float = 0.55,
    max_step_height: float = 0.34,
    max_surface_transition_height: float = 0.12,
    min_cell_points: int = 2,
    vertical_layer_gap: float = 0.18,
    max_points: int = 180000,
) -> TerrainGraph:
    slam_graph = build_slam_graph_from_pointcloud(
        msg,
        grid_resolution=grid_resolution,
        max_slope_grade=max_slope_grade,
        max_step_height=max_step_height,
        max_surface_transition_height=max_surface_transition_height,
        min_cell_points=min_cell_points,
        vertical_layer_gap=vertical_layer_gap,
        max_points=max_points,
        obstacle_clearance=robot_radius,
    )
    return _terrain_graph_from_slam_graph(slam_graph)


def _terrain_graph_from_slam_graph(slam_graph) -> TerrainGraph:
    nodes = [
        TerrainNode(
            index=node.index,
            x=node.x,
            y=node.y,
            z=node.z,
            surface_label=node.label.value,
            edge_margin=node.edge_margin,
            surface_local_x=node.surface_local_x,
            surface_local_y=node.surface_local_y,
            surface_half_x=node.surface_half_x,
            surface_half_y=node.surface_half_y,
            surface_width_axis=node.surface_width_axis,
        )
        for node in slam_graph.nodes
    ]
    return TerrainGraph(
        nodes=nodes,
        adjacency=slam_graph.adjacency,
        terrain_cloud=slam_graph.terrain_cloud,
    )


def plan_terrain_path(
    graph: TerrainGraph,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    start_z: float = 0.0,
    goal_z_policy: str = 'highest',
    max_goal_xy_distance: float = math.inf,
    goal_min_z: Optional[float] = None,
    blocked_points: Optional[list[tuple[float, float]]] = None,
    obstacle_clearance: float = 0.0,
) -> list[TerrainNode]:
    for policy in _goal_search_policies(goal_z_policy):
        path = _plan_terrain_path_once(
            graph,
            start_xy,
            goal_xy,
            start_z,
            policy,
            max_goal_xy_distance,
            goal_min_z,
            blocked_points or [],
            obstacle_clearance,
        )
        if path:
            return path
    return []


def plan_slam_frontier_path(
    graph: TerrainGraph,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    start_z: float = 0.0,
    min_path_distance: float = 1.0,
    max_path_distance: float = math.inf,
    blocked_points: Optional[list[tuple[float, float]]] = None,
    obstacle_clearance: float = 0.0,
    avoid_points: Optional[list[tuple[float, float]]] = None,
    avoid_clearance: float = 0.0,
    target_z: Optional[float] = None,
) -> list[TerrainNode]:
    if not graph.nodes:
        return []
    start_index = _nearest_reachable_start_node(
        graph,
        start_xy,
        start_z,
    )
    if start_index is None:
        return []

    distances = [math.inf] * len(graph.nodes)
    parents: list[Optional[int]] = [None] * len(graph.nodes)
    blocked_nodes = _nodes_near_blocked_points(
        graph.nodes,
        blocked_points or [],
        obstacle_clearance,
    )
    avoided_frontier_nodes = _nodes_near_blocked_points(
        graph.nodes,
        avoid_points or [],
        avoid_clearance,
    )
    blocked_nodes.discard(start_index)
    distances[start_index] = 0.0
    queue: list[tuple[float, int]] = [(0.0, start_index)]
    visited: list[int] = []

    while queue:
        current_distance, current = heapq.heappop(queue)
        if current_distance > distances[current]:
            continue
        if current in blocked_nodes:
            continue
        visited.append(current)
        for neighbor, edge_cost in graph.adjacency[current]:
            if neighbor in blocked_nodes:
                continue
            next_distance = current_distance + edge_cost
            if next_distance >= distances[neighbor]:
                continue
            distances[neighbor] = next_distance
            parents[neighbor] = current
            heapq.heappush(queue, (next_distance, neighbor))

    min_distance = max(0.0, min_path_distance)
    max_distance = max(min_distance, max_path_distance)
    candidates = [
        index
        for index in visited
        if distances[index] >= min_distance
        and index not in avoided_frontier_nodes
    ]
    if not candidates:
        return []
    bounded_candidates = [
        index
        for index in candidates
        if distances[index] <= max_distance
    ]
    if bounded_candidates:
        candidates = bounded_candidates
    if any(
        _frontier_makes_vertical_progress(
            graph.nodes[index],
            start_z=start_z,
            target_z=target_z,
        )
        for index in candidates
    ):
        high_goal_candidates = [
            index
            for index in candidates
            if not _frontier_low_under_high_goal(
                graph.nodes[index],
                goal_xy=goal_xy,
                start_z=start_z,
                target_z=target_z,
            )
        ]
        if high_goal_candidates:
            candidates = high_goal_candidates

    attractor_goal_xy = _frontier_elevation_entry_attractor_xy(
        graph.nodes,
        start_xy=start_xy,
        start_z=start_z,
        goal_xy=goal_xy,
        target_z=target_z,
    ) or _frontier_high_attractor_xy(
        graph.nodes,
        start_xy=start_xy,
        start_z=start_z,
        goal_xy=goal_xy,
        target_z=target_z,
    )
    scoring_goal_xy = attractor_goal_xy or goal_xy
    goal_distance_from_start = math.hypot(
        scoring_goal_xy[0] - start_xy[0],
        scoring_goal_xy[1] - start_xy[1],
    )
    goal_progress_by_index = {
        index: _goal_progress(
            start_xy,
            scoring_goal_xy,
            (graph.nodes[index].x, graph.nodes[index].y),
            goal_distance_from_start,
        )
        for index in candidates
    }
    best_goal_progress = max(goal_progress_by_index.values())
    if attractor_goal_xy is not None and best_goal_progress <= 0.0:
        scoring_goal_xy = goal_xy
        goal_distance_from_start = math.hypot(
            scoring_goal_xy[0] - start_xy[0],
            scoring_goal_xy[1] - start_xy[1],
        )
        goal_progress_by_index = {
            index: _goal_progress(
                start_xy,
                scoring_goal_xy,
                (graph.nodes[index].x, graph.nodes[index].y),
                goal_distance_from_start,
            )
            for index in candidates
        }
        best_goal_progress = max(goal_progress_by_index.values())
    final_goal_distance_from_start = math.hypot(
        goal_xy[0] - start_xy[0],
        goal_xy[1] - start_xy[1],
    )
    best_index = min(
        candidates,
        key=lambda index: (
            _frontier_reverse_penalty(
                graph.nodes[index],
                start_xy=start_xy,
                goal_xy=goal_xy,
                target_z=target_z,
                goal_distance_from_start=final_goal_distance_from_start,
            ),
            _frontier_goal_progress_penalty(
                goal_progress_by_index[index],
                best_goal_progress=best_goal_progress,
                goal_distance_from_start=goal_distance_from_start,
                start_z=start_z,
                target_z=target_z,
            ),
            _frontier_vertical_priority(
                graph.nodes[index],
                start_z=start_z,
                target_z=target_z,
            ),
            math.hypot(
                graph.nodes[index].x - scoring_goal_xy[0],
                graph.nodes[index].y - scoring_goal_xy[1],
            ),
            -_goal_progress(
                start_xy,
                scoring_goal_xy,
                (graph.nodes[index].x, graph.nodes[index].y),
                goal_distance_from_start,
            ),
        ),
    )
    best_progress = _goal_progress(
        start_xy,
        scoring_goal_xy,
        (graph.nodes[best_index].x, graph.nodes[best_index].y),
        goal_distance_from_start,
    )
    if best_progress <= 0.0 and not _frontier_makes_vertical_progress(
        graph.nodes[best_index],
        start_z=start_z,
        target_z=target_z,
    ):
        if attractor_goal_xy is not None and scoring_goal_xy != goal_xy:
            return plan_slam_frontier_path(
                graph,
                start_xy,
                goal_xy,
                start_z=start_z,
                min_path_distance=min_path_distance,
                max_path_distance=max_path_distance,
                blocked_points=blocked_points,
                obstacle_clearance=obstacle_clearance,
                avoid_points=avoid_points,
                avoid_clearance=avoid_clearance,
                target_z=None,
            )
        return []

    path: list[TerrainNode] = []
    cursor: Optional[int] = best_index
    while cursor is not None:
        path.append(graph.nodes[cursor])
        cursor = parents[cursor]
    path.reverse()
    return path


def _frontier_goal_progress_penalty(
    progress: float,
    *,
    best_goal_progress: float,
    goal_distance_from_start: float,
    start_z: float,
    target_z: Optional[float],
) -> int:
    if target_z is None or target_z <= start_z + 0.45:
        return 0
    if best_goal_progress <= 0.0:
        return 0
    slack = min(3.0, max(2.0, goal_distance_from_start * 0.10))
    return 1 if progress < best_goal_progress - slack else 0


def _frontier_low_under_high_goal(
    node: TerrainNode,
    *,
    goal_xy: tuple[float, float],
    start_z: float,
    target_z: Optional[float],
) -> bool:
    if target_z is None or target_z <= start_z + 0.45:
        return False
    if node.z >= target_z - 0.45:
        return False
    return math.hypot(node.x - goal_xy[0], node.y - goal_xy[1]) <= 4.0


def _frontier_elevation_entry_attractor_xy(
    nodes: list[TerrainNode],
    *,
    start_xy: tuple[float, float],
    start_z: float,
    goal_xy: tuple[float, float],
    target_z: Optional[float],
) -> Optional[tuple[float, float]]:
    if target_z is None or target_z <= start_z + 0.45:
        return None
    high_threshold = min(target_z, start_z + 0.75)
    low_threshold = start_z + 0.25
    high_nodes = [
        node
        for node in nodes
        if node.z >= high_threshold
        and _vertical_entry_label_priority(node) == 0
    ]
    low_nodes = [node for node in nodes if node.z <= low_threshold]
    if not high_nodes or not low_nodes:
        return None
    goal_distance = math.hypot(goal_xy[0] - start_xy[0], goal_xy[1] - start_xy[1])
    if goal_distance <= 1e-6:
        return None
    min_progress = min(4.0, max(1.5, goal_distance * 0.12))
    lateral_limit = min(7.0, max(4.0, goal_distance * 0.28))
    entry_candidates = []
    for high in high_nodes:
        high_progress = _goal_progress(
            start_xy,
            goal_xy,
            (high.x, high.y),
            goal_distance,
        )
        high_lateral = _goal_corridor_lateral_offset(
            start_xy,
            goal_xy,
            (high.x, high.y),
            goal_distance,
        )
        if high_progress < min_progress or high_lateral > lateral_limit:
            continue
        for low in low_nodes:
            horizontal = math.hypot(high.x - low.x, high.y - low.y)
            if horizontal < 1.0 or horizontal > 8.0:
                continue
            dz = high.z - low.z
            if dz < 0.25:
                continue
            grade = dz / max(horizontal, 1e-6)
            if grade > 0.70:
                continue
            entry_progress = _goal_progress(
                start_xy,
                goal_xy,
                (low.x, low.y),
                goal_distance,
            )
            if max(entry_progress, high_progress) < -1.0:
                continue
            low_lateral = _goal_corridor_lateral_offset(
                start_xy,
                goal_xy,
                (low.x, low.y),
                goal_distance,
            )
            if min(low_lateral, high_lateral) > lateral_limit:
                continue
            entry_candidates.append(
                (
                    low,
                    high,
                    min(low_lateral, high_lateral),
                    entry_progress,
                    grade,
                )
            )
    if not entry_candidates:
        return None
    entry, _, _, _, _ = min(
        entry_candidates,
        key=lambda item: (
            _vertical_entry_label_priority(item[1]),
            item[2],
            abs(item[4] - 0.25),
            math.hypot(item[0].x - start_xy[0], item[0].y - start_xy[1]),
            -item[3],
        ),
    )
    return (entry.x, entry.y)


def _frontier_high_attractor_xy(
    nodes: list[TerrainNode],
    *,
    start_xy: tuple[float, float],
    start_z: float,
    goal_xy: tuple[float, float],
    target_z: Optional[float],
) -> Optional[tuple[float, float]]:
    if target_z is None or target_z <= start_z + 0.45:
        return None
    high_threshold = min(target_z, start_z + 0.45)
    high_nodes = [node for node in nodes if node.z >= high_threshold]
    if not high_nodes:
        return None
    near_goal_attractor = min(
        high_nodes,
        key=lambda node: (
            math.hypot(node.x - goal_xy[0], node.y - goal_xy[1]),
            abs(node.z - target_z),
        ),
    )
    if math.hypot(near_goal_attractor.x - goal_xy[0], near_goal_attractor.y - goal_xy[1]) <= 6.0:
        return (near_goal_attractor.x, near_goal_attractor.y)

    goal_distance = math.hypot(goal_xy[0] - start_xy[0], goal_xy[1] - start_xy[1])
    if goal_distance <= 1e-6:
        return None
    min_progress = min(4.0, max(1.5, goal_distance * 0.12))
    lateral_limit = min(5.0, max(3.0, goal_distance * 0.22))
    corridor_nodes = []
    for node in high_nodes:
        progress = _goal_progress(
            start_xy,
            goal_xy,
            (node.x, node.y),
            goal_distance,
        )
        if progress < min_progress:
            continue
        lateral = _goal_corridor_lateral_offset(
            start_xy,
            goal_xy,
            (node.x, node.y),
            goal_distance,
        )
        if lateral > lateral_limit:
            continue
        corridor_nodes.append((node, progress, lateral))
    if not corridor_nodes:
        return None
    attractor, _, _ = min(
        corridor_nodes,
        key=lambda item: (
            _vertical_entry_label_priority(item[0]),
            item[2],
            -item[1],
            abs(item[0].z - target_z),
        ),
    )
    return (attractor.x, attractor.y)


def _goal_corridor_lateral_offset(
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    candidate_xy: tuple[float, float],
    goal_distance_from_start: float,
) -> float:
    if goal_distance_from_start <= 1e-6:
        return 0.0
    ux = (goal_xy[0] - start_xy[0]) / goal_distance_from_start
    uy = (goal_xy[1] - start_xy[1]) / goal_distance_from_start
    dx = candidate_xy[0] - start_xy[0]
    dy = candidate_xy[1] - start_xy[1]
    return abs(dx * uy - dy * ux)


def _vertical_entry_label_priority(node: TerrainNode) -> int:
    if 'ramp' in node.surface_label or 'step' in node.surface_label:
        return 0
    if 'deck' in node.surface_label or 'platform' in node.surface_label:
        return 1
    return 2


def _frontier_reverse_penalty(
    node: TerrainNode,
    *,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    target_z: Optional[float],
    goal_distance_from_start: float,
) -> int:
    if target_z is None or goal_distance_from_start <= 1e-6:
        return 0
    progress = _goal_progress(
        start_xy,
        goal_xy,
        (node.x, node.y),
        goal_distance_from_start,
    )
    return 1 if progress < 0.0 else 0


def _frontier_makes_vertical_progress(
    node: TerrainNode,
    *,
    start_z: float,
    target_z: Optional[float],
) -> bool:
    if target_z is None or target_z <= start_z + 0.45:
        return False
    return node.z >= start_z + 0.20


def _frontier_vertical_priority(
    node: TerrainNode,
    *,
    start_z: float,
    target_z: Optional[float],
) -> tuple[int, float]:
    if target_z is None or target_z <= start_z + 0.45:
        return (0, 0.0)
    if node.z < start_z + 0.20:
        return (1, 0.0)
    if node.z >= min(target_z, start_z + 0.45):
        return (0, 0.0)
    return (0, max(0.0, target_z - node.z))


def _nodes_near_blocked_points(
    nodes: list[TerrainNode],
    blocked_points: list[tuple[float, float]],
    obstacle_clearance: float,
) -> set[int]:
    if not blocked_points or obstacle_clearance <= 0.0:
        return set()
    clearance_sq = obstacle_clearance * obstacle_clearance
    blocked: set[int] = set()
    for node in nodes:
        for point_x, point_y in blocked_points:
            dx = node.x - point_x
            dy = node.y - point_y
            if dx * dx + dy * dy <= clearance_sq:
                blocked.add(node.index)
                break
    return blocked


def _goal_progress(
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    candidate_xy: tuple[float, float],
    goal_distance_from_start: float,
) -> float:
    if goal_distance_from_start <= 1e-6:
        return 0.0
    ux = (goal_xy[0] - start_xy[0]) / goal_distance_from_start
    uy = (goal_xy[1] - start_xy[1]) / goal_distance_from_start
    return (candidate_xy[0] - start_xy[0]) * ux + (
        candidate_xy[1] - start_xy[1]
    ) * uy


def _nearest_reachable_start_node(
    graph: TerrainGraph,
    xy: tuple[float, float],
    z_reference: float,
) -> Optional[int]:
    if not graph.nodes:
        return None
    component_ids, component_sizes = _weak_components(graph.adjacency)
    nearby = sorted(
        graph.nodes,
        key=lambda node: math.hypot(node.x - xy[0], node.y - xy[1]),
    )[:120]
    candidates = [
        node
        for node in nearby
        if graph.adjacency[node.index]
    ]
    if not candidates:
        candidates = nearby
    max_candidate_component = max(
        component_sizes[component_ids[node.index]]
        for node in candidates
    )
    if max_candidate_component >= 2:
        stable_candidates = [
            node
            for node in candidates
            if component_sizes[component_ids[node.index]]
            >= max(2, int(max_candidate_component * 0.50))
        ]
        if stable_candidates:
            candidates = stable_candidates
    return min(
        candidates,
        key=lambda node: (
            math.hypot(node.x - xy[0], node.y - xy[1])
            + abs(node.z - z_reference) * 0.6
        ),
    ).index


def _goal_search_policies(goal_z_policy: str) -> list[str]:
    if goal_z_policy == 'adaptive':
        return ['highest', 'nearest_z', 'lowest']
    if goal_z_policy in {'highest', 'lowest', 'nearest_z'}:
        return [goal_z_policy]
    return ['nearest_z']


def _goal_candidate_indexes(
    nodes: list[TerrainNode],
    goal_xy: tuple[float, float],
    *,
    z_reference: float,
    policy: str,
    max_goal_xy_distance: float,
    goal_min_z: Optional[float],
) -> list[int]:
    if not nodes:
        return []
    nearest = sorted(
        nodes,
        key=lambda node: math.hypot(node.x - goal_xy[0], node.y - goal_xy[1]),
    )
    if math.isfinite(max_goal_xy_distance):
        candidates = [
            node
            for node in nearest
            if math.hypot(node.x - goal_xy[0], node.y - goal_xy[1])
            <= max(0.0, max_goal_xy_distance)
        ]
    elif policy in {'highest', 'lowest'}:
        min_xy = math.hypot(nearest[0].x - goal_xy[0], nearest[0].y - goal_xy[1])
        candidates = [
            node
            for node in nearest[:240]
            if math.hypot(node.x - goal_xy[0], node.y - goal_xy[1])
            <= min_xy + 0.75
        ]
    else:
        candidates = nearest[:240]
    if goal_min_z is not None:
        candidates = [node for node in candidates if node.z >= goal_min_z]
    if policy == 'highest':
        candidates.sort(
            key=lambda node: (
                -node.z,
                math.hypot(node.x - goal_xy[0], node.y - goal_xy[1]),
            )
        )
    elif policy == 'lowest':
        candidates.sort(
            key=lambda node: (
                node.z,
                math.hypot(node.x - goal_xy[0], node.y - goal_xy[1]),
            )
        )
    else:
        candidates.sort(
            key=lambda node: (
                math.hypot(node.x - goal_xy[0], node.y - goal_xy[1])
                + abs(node.z - z_reference) * 0.6,
                math.hypot(node.x - goal_xy[0], node.y - goal_xy[1]),
            )
        )
    return [node.index for node in candidates]


def _plan_terrain_path_once(
    graph: TerrainGraph,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    start_z: float,
    goal_z_policy: str,
    max_goal_xy_distance: float,
    goal_min_z: Optional[float],
    blocked_points: list[tuple[float, float]],
    obstacle_clearance: float,
) -> list[TerrainNode]:
    if not graph.nodes:
        return []
    start_index = _nearest_reachable_start_node(
        graph,
        start_xy,
        start_z,
    )
    goal_indexes = _goal_candidate_indexes(
        graph.nodes,
        goal_xy,
        z_reference=start_z,
        policy=goal_z_policy,
        max_goal_xy_distance=max_goal_xy_distance,
        goal_min_z=goal_min_z,
    )
    if start_index is None or not goal_indexes:
        return []

    blocked_nodes = _nodes_near_blocked_points(
        graph.nodes,
        blocked_points,
        obstacle_clearance,
    )
    blocked_nodes.discard(start_index)
    for goal_index in goal_indexes:
        if start_index == goal_index:
            return [graph.nodes[start_index]]

        candidate_blocked_nodes = set(blocked_nodes)
        candidate_blocked_nodes.discard(goal_index)

        distances = [math.inf] * len(graph.nodes)
        parents: list[Optional[int]] = [None] * len(graph.nodes)
        distances[start_index] = 0.0
        queue: list[tuple[float, int]] = [
            (
                _heuristic(graph.nodes[start_index], graph.nodes[goal_index]),
                start_index,
            )
        ]

        while queue:
            _, current = heapq.heappop(queue)
            if current == goal_index:
                break
            current_distance = distances[current]
            if not math.isfinite(current_distance):
                continue
            for neighbor, edge_cost in graph.adjacency[current]:
                if neighbor in candidate_blocked_nodes:
                    continue
                next_distance = current_distance + edge_cost
                if next_distance >= distances[neighbor]:
                    continue
                distances[neighbor] = next_distance
                parents[neighbor] = current
                priority = next_distance + _heuristic(
                    graph.nodes[neighbor],
                    graph.nodes[goal_index],
                )
                heapq.heappush(queue, (priority, neighbor))

        if parents[goal_index] is None:
            continue

        path: list[TerrainNode] = []
        cursor: Optional[int] = goal_index
        while cursor is not None:
            path.append(graph.nodes[cursor])
            cursor = parents[cursor]
        path.reverse()
        return path
    return []


def _terrain_graph_route_diagnostics(
    graph: TerrainGraph,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    start_z: float,
    goal_z_policy: str,
) -> str:
    if not graph.nodes:
        return 'terrain graph diagnostics: graph is empty'

    start_index = _nearest_node(
        graph.nodes,
        start_xy,
        z_reference=start_z,
        policy='nearest_z',
    )
    goal_index = _nearest_node(
        graph.nodes,
        goal_xy,
        z_reference=start_z,
        policy=goal_z_policy,
    )
    if start_index is None or goal_index is None:
        return 'terrain graph diagnostics: start or goal nearest node missing'

    component_ids, component_sizes = _weak_components(graph.adjacency)
    start_node = graph.nodes[start_index]
    goal_node = graph.nodes[goal_index]
    start_component = component_ids[start_index]
    goal_component = component_ids[goal_index]
    xs = [node.x for node in graph.nodes]
    ys = [node.y for node in graph.nodes]
    zs = [node.z for node in graph.nodes]

    return (
        'terrain graph diagnostics: '
        f'nodes={len(graph.nodes)} '
        f'edges={sum(len(edges) for edges in graph.adjacency)} '
        f'bounds=x[{min(xs):.2f},{max(xs):.2f}] '
        f'y[{min(ys):.2f},{max(ys):.2f}] '
        f'z[{min(zs):.2f},{max(zs):.2f}] '
        f'start_node=({start_node.x:.2f},{start_node.y:.2f},'
        f'{start_node.z:.2f},{start_node.surface_label},'
        f'degree={len(graph.adjacency[start_index])},'
        f'component={start_component},'
        f'component_size={component_sizes[start_component]}) '
        f'goal_node=({goal_node.x:.2f},{goal_node.y:.2f},'
        f'{goal_node.z:.2f},{goal_node.surface_label},'
        f'degree={len(graph.adjacency[goal_index])},'
        f'component={goal_component},'
        f'component_size={component_sizes[goal_component]}) '
        f'largest_components={sorted(component_sizes, reverse=True)[:5]}'
    )


def _goal_target_z(
    graph: TerrainGraph,
    goal_xy: tuple[float, float],
    start_z: float,
    goal_z_policy: str,
) -> Optional[float]:
    policy = _goal_search_policies(goal_z_policy)[0]
    goal_index = _nearest_node(
        graph.nodes,
        goal_xy,
        z_reference=start_z,
        policy=policy,
    )
    if goal_index is None:
        return None
    return graph.nodes[goal_index].z


def should_keep_pending_slam_goal(
    graph: TerrainGraph,
    *,
    terrain_map_source: str,
    frontier_replan_enabled: bool,
) -> bool:
    return terrain_map_source == 'slam_cloud' and frontier_replan_enabled


def should_hold_active_frontier_path(
    *,
    active_path: list[TerrainNode],
    current_xy: tuple[float, float],
    goal_tolerance: float,
    execution_active: bool = True,
    active_final_goal_xy: Optional[tuple[float, float]] = None,
    final_goal_xy: Optional[tuple[float, float]] = None,
    final_goal_tolerance: float = 0.05,
) -> bool:
    if not active_path:
        return False
    if not execution_active:
        return False
    if active_final_goal_xy is not None and final_goal_xy is not None:
        final_goal_distance = math.hypot(
            active_final_goal_xy[0] - final_goal_xy[0],
            active_final_goal_xy[1] - final_goal_xy[1],
        )
        if final_goal_distance > max(0.0, final_goal_tolerance):
            return False
    goal = active_path[-1]
    goal_distance = math.hypot(goal.x - current_xy[0], goal.y - current_xy[1])
    return goal_distance > max(0.0, goal_tolerance)


def should_release_stalled_frontier_path(
    *,
    active_path: list[TerrainNode],
    commanded_motion: bool,
    current_xy: tuple[float, float],
    monitor_start_xy: Optional[tuple[float, float]],
    elapsed_sec: float,
    min_progress: float,
    timeout_sec: float,
    goal_xy: Optional[tuple[float, float]] = None,
) -> bool:
    if not active_path or not commanded_motion or monitor_start_xy is None:
        return False
    if elapsed_sec < _frontier_effective_stall_timeout(active_path, timeout_sec):
        return False
    progress = _tracking_progress(
        current_xy=current_xy,
        monitor_start_xy=monitor_start_xy,
        goal_xy=goal_xy,
    )
    return progress < max(0.0, min_progress)


def _frontier_effective_stall_timeout(
    active_path: list[TerrainNode],
    timeout_sec: float,
) -> float:
    base_timeout = max(0.0, timeout_sec)
    if len(active_path) < 2:
        return base_timeout
    return max(base_timeout, min(45.0, _frontier_path_xy_length(active_path) * 4.0))


def _frontier_path_xy_length(active_path: list[TerrainNode]) -> float:
    length = 0.0
    for previous, current in zip(active_path, active_path[1:]):
        length += math.hypot(current.x - previous.x, current.y - previous.y)
    return length


def should_refresh_frontier_stall_monitor(
    *,
    current_xy: tuple[float, float],
    monitor_start_xy: Optional[tuple[float, float]],
    min_progress: float,
    goal_xy: Optional[tuple[float, float]] = None,
) -> bool:
    if monitor_start_xy is None:
        return False
    progress = _tracking_progress(
        current_xy=current_xy,
        monitor_start_xy=monitor_start_xy,
        goal_xy=goal_xy,
    )
    return progress >= max(0.0, min_progress)


def _tracking_progress(
    *,
    current_xy: tuple[float, float],
    monitor_start_xy: tuple[float, float],
    goal_xy: Optional[tuple[float, float]] = None,
) -> float:
    if goal_xy is None:
        return math.hypot(
            current_xy[0] - monitor_start_xy[0],
            current_xy[1] - monitor_start_xy[1],
        )
    start_distance = math.hypot(
        goal_xy[0] - monitor_start_xy[0],
        goal_xy[1] - monitor_start_xy[1],
    )
    current_distance = math.hypot(
        goal_xy[0] - current_xy[0],
        goal_xy[1] - current_xy[1],
    )
    return start_distance - current_distance


def advance_direct_target_index(
    path: list[TerrainNode],
    current_index: int,
    current_xy: tuple[float, float],
    waypoint_tolerance: float,
    current_z: Optional[float] = None,
    z_tolerance: float = math.inf,
) -> int:
    if not path:
        return 0
    index = min(max(current_index, 0), len(path) - 1)
    tolerance = max(0.0, waypoint_tolerance)

    def distance_at(candidate_index: int) -> float:
        candidate = path[candidate_index]
        xy_distance = math.hypot(
            candidate.x - current_xy[0],
            candidate.y - current_xy[1],
        )
        return xy_distance + _direct_height_error(
            candidate,
            current_z=current_z,
            z_tolerance=z_tolerance,
        )

    while index < len(path) - 1 and _direct_node_reached(
        path[index],
        current_xy=current_xy,
        current_z=current_z,
        xy_tolerance=tolerance,
        z_tolerance=z_tolerance,
    ):
        index += 1

    best_index = index
    best_distance = distance_at(index)
    for candidate_index in range(index + 1, len(path)):
        if _direct_height_error(
            path[candidate_index],
            current_z=current_z,
            z_tolerance=z_tolerance,
        ) > 0.0:
            continue
        candidate_distance = distance_at(candidate_index)
        if candidate_distance + 0.05 < best_distance:
            best_index = candidate_index
            best_distance = candidate_distance

    index = best_index
    while index < len(path) - 1 and _direct_node_reached(
        path[index],
        current_xy=current_xy,
        current_z=current_z,
        xy_tolerance=tolerance,
        z_tolerance=z_tolerance,
    ):
        index += 1
    return index


def _direct_height_error(
    node: TerrainNode,
    *,
    current_z: Optional[float],
    z_tolerance: float,
) -> float:
    if current_z is None or not math.isfinite(z_tolerance):
        return 0.0
    return max(0.0, abs(node.z - current_z) - max(0.0, z_tolerance))


def _direct_node_reached(
    node: TerrainNode,
    *,
    current_xy: tuple[float, float],
    current_z: Optional[float],
    xy_tolerance: float,
    z_tolerance: float,
) -> bool:
    xy_distance = math.hypot(node.x - current_xy[0], node.y - current_xy[1])
    if xy_distance > max(0.0, xy_tolerance):
        return False
    return _direct_height_error(
        node,
        current_z=current_z,
        z_tolerance=z_tolerance,
    ) <= 0.0


def direct_tracking_start_clearance(
    *,
    follow_path_start_clearance: float,
    start_waypoint_clearance: float,
    direct_waypoint_tolerance: float,
    direct_lookahead_dist: float,
) -> float:
    return max(
        0.0,
        follow_path_start_clearance,
        start_waypoint_clearance,
        direct_waypoint_tolerance,
        direct_lookahead_dist,
    )


def drop_regressive_start_waypoints(
    path: list[TerrainNode],
    *,
    start_xy: tuple[float, float],
    final_goal_xy: tuple[float, float],
    regression_tolerance: float,
    current_z: Optional[float] = None,
    z_tolerance: float = math.inf,
) -> list[TerrainNode]:
    if len(path) <= 1:
        return path
    if current_z is not None and any(
        _direct_height_error(node, current_z=current_z, z_tolerance=z_tolerance)
        > 0.0
        for node in path
    ):
        return path
    start_goal_distance = math.hypot(
        final_goal_xy[0] - start_xy[0],
        final_goal_xy[1] - start_xy[1],
    )
    tolerance = max(0.0, regression_tolerance)
    keep_index = 0
    for index, node in enumerate(path[:-1]):
        node_goal_distance = math.hypot(
            final_goal_xy[0] - node.x,
            final_goal_xy[1] - node.y,
        )
        if node_goal_distance > start_goal_distance + tolerance:
            keep_index = index + 1
            continue
        break
    return path[keep_index:] or [path[-1]]


def select_stall_tracking_goal(
    *,
    direct_path: list[TerrainNode],
    active_frontier_path: list[TerrainNode],
    direct_target_index: int,
) -> Optional[TerrainNode]:
    if active_frontier_path:
        return active_frontier_path[-1]
    if not direct_path:
        return None
    tracking_index = min(
        max(direct_target_index, 0),
        len(direct_path) - 1,
    )
    return direct_path[tracking_index]


def should_reject_regressive_frontier_path(
    *,
    candidate_goal_distance: float,
    best_goal_distance: Optional[float],
    regression_tolerance: float,
) -> bool:
    if best_goal_distance is None:
        return False
    return candidate_goal_distance > best_goal_distance + max(0.0, regression_tolerance)


def _weak_components(
    adjacency: list[list[tuple[int, float]]],
) -> tuple[list[int], list[int]]:
    undirected = [set() for _ in adjacency]
    for index, edges in enumerate(adjacency):
        for other, _ in edges:
            undirected[index].add(other)
            undirected[other].add(index)

    component_ids = [-1] * len(adjacency)
    component_sizes: list[int] = []
    for index in range(len(adjacency)):
        if component_ids[index] >= 0:
            continue
        component_id = len(component_sizes)
        queue = [index]
        component_ids[index] = component_id
        size = 0
        while queue:
            current = queue.pop()
            size += 1
            for other in undirected[current]:
                if component_ids[other] >= 0:
                    continue
                component_ids[other] = component_id
                queue.append(other)
        component_sizes.append(size)
    return component_ids, component_sizes


class TerrainPctPlanner(Node):
    def __init__(self) -> None:
        super().__init__('terrain_pct_planner')
        self.declare_parameter('world_file', '')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('initial_pose_topic', '/initialpose')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('path_topic', '/pct_path')
        self.declare_parameter(
            'terrain_cloud_topic',
            '/terrain_traversability_cloud',
        )
        self.declare_parameter('terrain_map_source', 'sdf')
        self.declare_parameter('slam_map_topic', '/Laser_map')
        self.declare_parameter('slam_map_max_points', 180000)
        self.declare_parameter('slam_grid_resolution', 0.25)
        self.declare_parameter('slam_min_cell_points', 2)
        self.declare_parameter('slam_vertical_layer_gap', 0.18)
        self.declare_parameter('slam_rebuild_period_sec', 3.0)
        self.declare_parameter('use_initial_pose_anchor', True)
        self.declare_parameter('grid_resolution', 0.40)
        self.declare_parameter('terrain_cloud_resolution', 0.0)
        self.declare_parameter('robot_radius', 0.35)
        self.declare_parameter('support_margin', 0.45)
        self.declare_parameter('max_slope_grade', 0.55)
        self.declare_parameter('max_step_height', 0.34)
        self.declare_parameter('max_surface_transition_height', 0.12)
        self.declare_parameter('goal_z_policy', 'highest')
        self.declare_parameter('goal_min_z', -1.0)
        self.declare_parameter('send_nav2_goals', True)
        self.declare_parameter('nav_execution_mode', 'direct')
        self.declare_parameter('waypoint_spacing', 0.90)
        self.declare_parameter('start_waypoint_clearance', 0.45)
        self.declare_parameter('follow_path_start_clearance', 0.12)
        self.declare_parameter('slope_speed_limit', 0.14)
        self.declare_parameter('flat_speed_limit', 0.22)
        self.declare_parameter('slope_speed_grade_threshold', 0.08)
        self.declare_parameter('direct_cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('direct_control_frequency', 15.0)
        self.declare_parameter('direct_lookahead_dist', 0.45)
        self.declare_parameter('direct_waypoint_tolerance', 0.24)
        self.declare_parameter('direct_goal_tolerance', 0.30)
        self.declare_parameter('direct_z_tolerance', 0.45)
        self.declare_parameter('direct_heading_gain', 1.4)
        self.declare_parameter('direct_max_linear_speed', 0.20)
        self.declare_parameter('direct_min_linear_speed', 0.035)
        self.declare_parameter('direct_max_angular_speed', 0.45)
        self.declare_parameter('direct_max_heading_error_for_forward', 1.25)
        self.declare_parameter('initial_surface_z_hint', -1.0)
        self.declare_parameter('initial_surface_hint_radius', 0.75)
        self.declare_parameter('last_path_surface_hint_radius', 0.75)
        self.declare_parameter('terrain_publish_period_sec', 4.0)
        self.declare_parameter('duplicate_goal_xy_tolerance', 0.05)
        self.declare_parameter('duplicate_goal_time_sec', 1.5)
        self.declare_parameter('goal_snap_max_distance', 1.0)
        self.declare_parameter('frontier_replan_enabled', True)
        self.declare_parameter('frontier_min_path_distance', 1.0)
        self.declare_parameter('frontier_max_path_distance', 2.0)
        self.declare_parameter('frontier_obstacle_scan_topic', '/scan')
        self.declare_parameter('frontier_obstacle_clearance', 0.45)
        self.declare_parameter('frontier_obstacle_range_max', 3.0)
        self.declare_parameter('frontier_stall_timeout_sec', 8.0)
        self.declare_parameter('frontier_stall_min_progress', 0.20)
        self.declare_parameter('frontier_failed_clearance', 1.6)
        self.declare_parameter('frontier_goal_regression_tolerance', 1.5)

        world_file = Path(str(self.get_parameter('world_file').value))
        self._world_frame = str(self.get_parameter('world_frame').value)
        self._terrain_map_source = str(
            self.get_parameter('terrain_map_source').value
        )
        if self._terrain_map_source not in {'sdf', 'slam_cloud'}:
            raise ValueError(
                "terrain_map_source must be 'sdf' or 'slam_cloud', "
                f"got {self._terrain_map_source!r}"
            )
        self._goal_z_policy = str(self.get_parameter('goal_z_policy').value)
        goal_min_z = float(self.get_parameter('goal_min_z').value)
        self._goal_min_z: Optional[float] = (
            goal_min_z if goal_min_z >= 0.0 else None
        )
        self._send_nav2_goals = bool(
            self.get_parameter('send_nav2_goals').value
        )
        self._nav_execution_mode = str(
            self.get_parameter('nav_execution_mode').value
        )
        if self._nav_execution_mode not in {'direct', 'follow_path', 'waypoints'}:
            raise ValueError(
                "nav_execution_mode must be 'direct', 'follow_path' or 'waypoints', "
                f"got {self._nav_execution_mode!r}"
            )
        self._waypoint_spacing = float(
            self.get_parameter('waypoint_spacing').value
        )
        self._start_waypoint_clearance = float(
            self.get_parameter('start_waypoint_clearance').value
        )
        self._follow_path_start_clearance = float(
            self.get_parameter('follow_path_start_clearance').value
        )
        self._slope_speed_limit = float(
            self.get_parameter('slope_speed_limit').value
        )
        self._flat_speed_limit = float(
            self.get_parameter('flat_speed_limit').value
        )
        self._slope_speed_grade_threshold = float(
            self.get_parameter('slope_speed_grade_threshold').value
        )
        self._direct_lookahead_dist = float(
            self.get_parameter('direct_lookahead_dist').value
        )
        self._direct_waypoint_tolerance = float(
            self.get_parameter('direct_waypoint_tolerance').value
        )
        self._direct_goal_tolerance = float(
            self.get_parameter('direct_goal_tolerance').value
        )
        self._direct_z_tolerance = float(
            self.get_parameter('direct_z_tolerance').value
        )
        self._direct_heading_gain = float(
            self.get_parameter('direct_heading_gain').value
        )
        self._direct_max_linear_speed = float(
            self.get_parameter('direct_max_linear_speed').value
        )
        self._direct_min_linear_speed = float(
            self.get_parameter('direct_min_linear_speed').value
        )
        self._direct_max_angular_speed = float(
            self.get_parameter('direct_max_angular_speed').value
        )
        self._direct_max_heading_error_for_forward = float(
            self.get_parameter('direct_max_heading_error_for_forward').value
        )
        self._initial_surface_z_hint = float(
            self.get_parameter('initial_surface_z_hint').value
        )
        self._initial_surface_hint_radius = float(
            self.get_parameter('initial_surface_hint_radius').value
        )
        self._last_path_surface_hint_radius = float(
            self.get_parameter('last_path_surface_hint_radius').value
        )
        self._duplicate_goal_xy_tolerance = float(
            self.get_parameter('duplicate_goal_xy_tolerance').value
        )
        self._duplicate_goal_time_sec = float(
            self.get_parameter('duplicate_goal_time_sec').value
        )
        self._goal_snap_max_distance = float(
            self.get_parameter('goal_snap_max_distance').value
        )
        self._frontier_replan_enabled = bool(
            self.get_parameter('frontier_replan_enabled').value
        )
        self._frontier_min_path_distance = float(
            self.get_parameter('frontier_min_path_distance').value
        )
        self._frontier_max_path_distance = float(
            self.get_parameter('frontier_max_path_distance').value
        )
        self._frontier_obstacle_clearance = float(
            self.get_parameter('frontier_obstacle_clearance').value
        )
        self._frontier_obstacle_range_max = float(
            self.get_parameter('frontier_obstacle_range_max').value
        )
        self._frontier_stall_timeout_sec = float(
            self.get_parameter('frontier_stall_timeout_sec').value
        )
        self._frontier_stall_min_progress = float(
            self.get_parameter('frontier_stall_min_progress').value
        )
        self._frontier_failed_clearance = float(
            self.get_parameter('frontier_failed_clearance').value
        )
        self._frontier_goal_regression_tolerance = float(
            self.get_parameter('frontier_goal_regression_tolerance').value
        )
        self._frontier_target_z: Optional[float] = None
        self._use_initial_pose_anchor = bool(
            self.get_parameter('use_initial_pose_anchor').value
        )
        live_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._world_file = world_file
        self._grid_resolution = float(self.get_parameter('grid_resolution').value)
        self._terrain_cloud_resolution = float(
            self.get_parameter('terrain_cloud_resolution').value
        )
        self._robot_radius = float(self.get_parameter('robot_radius').value)
        self._support_margin = float(self.get_parameter('support_margin').value)
        self._max_slope_grade = float(self.get_parameter('max_slope_grade').value)
        self._max_step_height = float(self.get_parameter('max_step_height').value)
        self._max_surface_transition_height = float(
            self.get_parameter('max_surface_transition_height').value
        )
        self._slam_map_topic = str(self.get_parameter('slam_map_topic').value)
        self._slam_map_max_points = int(
            self.get_parameter('slam_map_max_points').value
        )
        self._slam_grid_resolution = float(
            self.get_parameter('slam_grid_resolution').value
        )
        self._slam_min_cell_points = int(
            self.get_parameter('slam_min_cell_points').value
        )
        self._slam_vertical_layer_gap = float(
            self.get_parameter('slam_vertical_layer_gap').value
        )
        self._slam_rebuild_period_sec = max(
            float(self.get_parameter('slam_rebuild_period_sec').value),
            1.0,
        )
        self._graph = TerrainGraph(nodes=[], adjacency=[], terrain_cloud=[])
        self._slam_map_msg: Optional[PointCloud2] = None
        self._scan_msg: Optional[LaserScan] = None
        self._last_slam_map_stamp: Optional[tuple[int, int]] = None
        self._last_slam_graph_stamp: Optional[tuple[int, int]] = None
        self._slam_graph_rebuild_in_progress = False
        self._slam_map_timer = None
        if self._terrain_map_source == 'slam_cloud':
            self._slam_subscription = self.create_subscription(
                PointCloud2,
                self._slam_map_topic,
                self._slam_map_callback,
                live_qos,
            )
            self._slam_map_timer = self.create_timer(
                self._slam_rebuild_period_sec,
                self._rebuild_slam_graph,
            )
        else:
            self._graph = build_terrain_graph(
                world_file,
                grid_resolution=self._grid_resolution,
                terrain_cloud_resolution=self._terrain_cloud_resolution,
                robot_radius=self._robot_radius,
                support_margin=self._support_margin,
                max_slope_grade=self._max_slope_grade,
                max_step_height=self._max_step_height,
                max_surface_transition_height=self._max_surface_transition_height,
            )
        self._odom_subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_callback,
            live_qos,
        )
        self._scan_subscription = self.create_subscription(
            LaserScan,
            str(self.get_parameter('frontier_obstacle_scan_topic').value),
            self._scan_callback,
            live_qos,
        )
        self._initial_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter('initial_pose_topic').value),
            self._initial_pose_callback,
            10,
        )
        self._goal_subscription = self.create_subscription(
            PoseStamped,
            str(self.get_parameter('goal_topic').value),
            self._goal_callback,
            live_qos,
        )
        self._path_publisher = self.create_publisher(
            PathMsg,
            str(self.get_parameter('path_topic').value),
            live_qos,
        )
        self._terrain_cloud_publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('terrain_cloud_topic').value),
            latched_qos,
        )
        self._waypoint_action_client = ActionClient(
            self,
            NavigateThroughPoses,
            '/navigate_through_poses',
        )
        self._follow_path_action_client = ActionClient(
            self,
            FollowPath,
            '/follow_path',
        )
        self._speed_limit_publisher = self.create_publisher(
            SpeedLimit,
            '/speed_limit',
            1,
        )
        self._direct_cmd_vel_publisher = self.create_publisher(
            Twist,
            str(self.get_parameter('direct_cmd_vel_topic').value),
            10,
        )

        self._odom_msg: Optional[Odometry] = None
        self._pending_initial_pose: Optional[Pose2D] = None
        self._odom_anchor: Optional[OdomAnchor] = None
        self._active_nav_goal: Optional[ClientGoalHandle] = None
        self._pending_nav_goal: Optional[
            tuple[FollowPath.Goal | NavigateThroughPoses.Goal, int, int, str]
        ] = None
        self._initial_planner_xy: Optional[tuple[float, float]] = None
        self._last_goal_xy: Optional[tuple[float, float]] = None
        self._last_goal_time_ns: Optional[int] = None
        self._pending_final_goal_xy: Optional[tuple[float, float]] = None
        self._last_planned_path: list[TerrainNode] = []
        self._active_frontier_path: list[TerrainNode] = []
        self._active_frontier_final_goal_xy: Optional[tuple[float, float]] = None
        self._frontier_progress_final_goal_xy: Optional[tuple[float, float]] = None
        self._frontier_best_goal_distance: Optional[float] = None
        self._frontier_stall_start_xy: Optional[tuple[float, float]] = None
        self._frontier_stall_start_time_ns: Optional[int] = None
        self._frontier_stall_commanded_motion = False
        self._frontier_avoid_points: list[tuple[float, float]] = []
        self._direct_path: list[TerrainNode] = []
        self._direct_final_goal_xy: Optional[tuple[float, float]] = None
        self._direct_target_index = 0
        self._direct_speed_limit = 0.0
        self._terrain_timer = self.create_timer(
            max(float(self.get_parameter('terrain_publish_period_sec').value), 0.5),
            self._publish_terrain_cloud,
        )
        self._direct_timer = self.create_timer(
            1.0
            / max(
                float(self.get_parameter('direct_control_frequency').value),
                1.0,
            ),
            self._direct_control_tick,
        )
        self.get_logger().info(
            'terrain pct-style planner ready: '
            f'nodes={len(self._graph.nodes)} '
            f'edges={sum(len(edges) for edges in self._graph.adjacency)} '
            f'world_file={world_file} '
            f'terrain_map_source={self._terrain_map_source}'
        )

    def _odom_callback(self, msg: Odometry) -> None:
        self._odom_msg = msg
        if self._pending_initial_pose is not None:
            self._set_odom_anchor(self._pending_initial_pose, msg)
            self._pending_initial_pose = None

    def _slam_map_callback(self, msg: PointCloud2) -> None:
        self._slam_map_msg = msg
        self._last_slam_map_stamp = (
            int(msg.header.stamp.sec),
            int(msg.header.stamp.nanosec),
        )

    def _scan_callback(self, msg: LaserScan) -> None:
        self._scan_msg = msg

    def _rebuild_slam_graph(self) -> None:
        if self._terrain_map_source != 'slam_cloud':
            return
        if self._slam_map_msg is None:
            return
        if self._slam_graph_rebuild_in_progress:
            return
        if (
            self._last_slam_map_stamp is not None
            and self._last_slam_map_stamp == self._last_slam_graph_stamp
        ):
            return

        self._slam_graph_rebuild_in_progress = True
        try:
            graph = build_slam_terrain_graph_from_pointcloud(
                self._slam_map_msg,
                grid_resolution=self._slam_grid_resolution,
                robot_radius=self._robot_radius,
                support_margin=self._support_margin,
                max_slope_grade=self._max_slope_grade,
                max_step_height=self._max_step_height,
                max_surface_transition_height=self._max_surface_transition_height,
                min_cell_points=self._slam_min_cell_points,
                vertical_layer_gap=self._slam_vertical_layer_gap,
                max_points=self._slam_map_max_points,
            )
            self._graph = graph
            self._last_slam_graph_stamp = self._last_slam_map_stamp
        finally:
            self._slam_graph_rebuild_in_progress = False
        self.get_logger().info(
            'rebuilt terrain graph from FAST-LIO map: '
            f'nodes={len(self._graph.nodes)} '
            f'edges={sum(len(edges) for edges in self._graph.adjacency)}'
        )
        self._try_pending_final_goal()

    def _initial_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        if not self._use_initial_pose_anchor:
            return
        initial_pose = _pose_from_initial_pose(msg)
        if self._odom_msg is None:
            self._pending_initial_pose = initial_pose
            return
        self._set_odom_anchor(initial_pose, self._odom_msg)

    def _set_odom_anchor(self, map_pose: Pose2D, odom_msg: Odometry) -> None:
        self._odom_anchor = OdomAnchor(
            map_pose=map_pose,
            odom_pose=_pose_from_odom(odom_msg),
        )

    def _current_pose(self) -> tuple[float, float, float]:
        x, y, _, z = self._current_planar_pose()
        return x, y, z

    def _current_planar_pose(self) -> tuple[float, float, float, float]:
        if self._odom_msg is None:
            return 0.0, 0.0, 0.0, 0.0
        pose = _pose_from_odom(self._odom_msg)
        if self._odom_anchor is not None:
            pose = _map_pose_from_anchor(pose, self._odom_anchor)
        z = float(self._odom_msg.pose.pose.position.z)
        return pose.x, pose.y, pose.yaw, z

    def _goal_callback(self, msg: PoseStamped) -> None:
        start_x, start_y, start_z = self._current_pose()
        if self._initial_planner_xy is None:
            self._initial_planner_xy = (start_x, start_y)
        terrain_start_z = self._terrain_surface_z_for_pose(
            (start_x, start_y),
            start_z,
        )
        goal_x = float(msg.pose.position.x)
        goal_y = float(msg.pose.position.y)
        self._frontier_target_z = _goal_target_z(
            self._graph,
            (goal_x, goal_y),
            terrain_start_z,
            self._goal_z_policy,
        )
        if self._goal_min_z is not None:
            self._frontier_target_z = max(
                self._frontier_target_z or self._goal_min_z,
                self._goal_min_z,
            )
        if self._is_duplicate_goal((goal_x, goal_y)):
            return
        path = plan_terrain_path(
            self._graph,
            (start_x, start_y),
            (goal_x, goal_y),
            start_z=terrain_start_z,
            goal_z_policy=self._goal_z_policy,
            max_goal_xy_distance=self._goal_snap_max_distance,
            goal_min_z=self._goal_min_z,
        )
        if not path:
            self.get_logger().warning(
                'terrain planner failed to find a traversable route: '
                f'start=({start_x:.2f},{start_y:.2f}) '
                f'goal=({goal_x:.2f},{goal_y:.2f})'
            )
            diagnostics = _terrain_graph_route_diagnostics(
                self._graph,
                (start_x, start_y),
                (goal_x, goal_y),
                terrain_start_z,
                self._goal_z_policy,
            )
            if diagnostics:
                self.get_logger().warning(diagnostics)
            if should_keep_pending_slam_goal(
                self._graph,
                terrain_map_source=self._terrain_map_source,
                frontier_replan_enabled=self._frontier_replan_enabled,
            ):
                self._pending_final_goal_xy = (goal_x, goal_y)
            self._plan_frontier_toward_goal(
                start_xy=(start_x, start_y),
                start_z=terrain_start_z,
                final_goal_xy=(goal_x, goal_y),
            )
            return
        self._pending_final_goal_xy = None
        self._active_frontier_path = []
        self._active_frontier_final_goal_xy = None
        self._reset_frontier_stall_monitor()
        self._last_planned_path = path
        self._publish_and_execute_path(path, msg)

    def _try_pending_final_goal(self) -> None:
        if self._pending_final_goal_xy is None or self._odom_msg is None:
            return
        start_x, start_y, start_z = self._current_pose()
        if self._initial_planner_xy is None:
            self._initial_planner_xy = (start_x, start_y)
        terrain_start_z = self._terrain_surface_z_for_pose(
            (start_x, start_y),
            start_z,
        )
        goal_x, goal_y = self._pending_final_goal_xy
        path = plan_terrain_path(
            self._graph,
            (start_x, start_y),
            (goal_x, goal_y),
            start_z=terrain_start_z,
            goal_z_policy=self._goal_z_policy,
            max_goal_xy_distance=self._goal_snap_max_distance,
            goal_min_z=self._goal_min_z,
        )
        if not path:
            self._plan_frontier_toward_goal(
                start_xy=(start_x, start_y),
                start_z=terrain_start_z,
                final_goal_xy=(goal_x, goal_y),
            )
            return
        msg = PoseStamped()
        msg.header.frame_id = self._world_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = goal_x
        msg.pose.position.y = goal_y
        msg.pose.orientation.w = 1.0
        self._pending_final_goal_xy = None
        self._active_frontier_path = []
        self._active_frontier_final_goal_xy = None
        self._reset_frontier_stall_monitor()
        self._last_planned_path = path
        self.get_logger().info(
            'pending final goal became reachable after FAST-LIO map update: '
            f'goal=({goal_x:.2f},{goal_y:.2f}) path_nodes={len(path)}'
        )
        self._publish_and_execute_path(path, msg)

    def _terrain_surface_z_for_pose(
        self,
        current_xy: tuple[float, float],
        odom_z: float,
    ) -> float:
        terrain_start_z = _surface_z_reference(
            odom_z=odom_z,
            current_xy=current_xy,
            initial_xy=self._initial_planner_xy,
            initial_surface_z_hint=self._initial_surface_z_hint,
            initial_surface_hint_radius=self._initial_surface_hint_radius,
            last_path=self._last_planned_path,
            last_path_surface_hint_radius=self._last_path_surface_hint_radius,
        )
        terrain_start_z = (
            _surface_height_at_xy(
                self._graph.nodes,
                current_xy,
                z_hint=terrain_start_z,
                max_xy_distance=self._initial_surface_hint_radius,
            )
            or terrain_start_z
        )
        return terrain_start_z

    def _plan_frontier_toward_goal(
        self,
        *,
        start_xy: tuple[float, float],
        start_z: float,
        final_goal_xy: tuple[float, float],
    ) -> None:
        if not self._frontier_replan_enabled or self._terrain_map_source != 'slam_cloud':
            return
        self._reset_frontier_progress_if_new_goal(final_goal_xy)
        if should_hold_active_frontier_path(
            active_path=self._active_frontier_path,
            current_xy=start_xy,
            goal_tolerance=self._direct_goal_tolerance,
            execution_active=bool(self._direct_path),
            active_final_goal_xy=self._active_frontier_final_goal_xy,
            final_goal_xy=final_goal_xy,
            final_goal_tolerance=self._duplicate_goal_xy_tolerance,
        ):
            return
        self._active_frontier_path = []
        self._active_frontier_final_goal_xy = None
        self._reset_frontier_stall_monitor()
        path = self._find_non_regressive_frontier_path(
            start_xy=start_xy,
            start_z=start_z,
            final_goal_xy=final_goal_xy,
        )
        if not path:
            return
        self._pending_final_goal_xy = final_goal_xy
        self._last_planned_path = path
        self._active_frontier_path = path
        self._active_frontier_final_goal_xy = final_goal_xy
        self._frontier_stall_start_xy = start_xy
        self._frontier_stall_start_time_ns = self.get_clock().now().nanoseconds
        self._frontier_stall_commanded_motion = False
        msg = PoseStamped()
        msg.header.frame_id = self._world_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = path[-1].x
        msg.pose.position.y = path[-1].y
        msg.pose.orientation.w = 1.0
        self.get_logger().info(
            'published FAST-LIO exploration frontier path toward pending goal: '
            f'frontier=({path[-1].x:.2f},{path[-1].y:.2f}) '
            f'final_goal=({final_goal_xy[0]:.2f},{final_goal_xy[1]:.2f}) '
            f'path_nodes={len(path)}'
        )
        self._publish_and_execute_path(path, msg)

    def _find_non_regressive_frontier_path(
        self,
        *,
        start_xy: tuple[float, float],
        start_z: float,
        final_goal_xy: tuple[float, float],
    ) -> list[TerrainNode]:
        avoid_points = list(self._frontier_avoid_points)
        blocked_points = self._frontier_blocked_points_from_scan()
        for _ in range(8):
            path = plan_slam_frontier_path(
                self._graph,
                start_xy,
                final_goal_xy,
                start_z=start_z,
                min_path_distance=self._frontier_min_path_distance,
                max_path_distance=self._frontier_max_path_distance,
                blocked_points=blocked_points,
                obstacle_clearance=self._frontier_obstacle_clearance,
                avoid_points=avoid_points,
                avoid_clearance=self._frontier_failed_clearance,
                target_z=self._frontier_target_z,
            )
            if not path:
                self._frontier_avoid_points = avoid_points[-8:]
                return []
            frontier = path[-1]
            candidate_distance = math.hypot(
                frontier.x - final_goal_xy[0],
                frontier.y - final_goal_xy[1],
            )
            if not should_reject_regressive_frontier_path(
                candidate_goal_distance=candidate_distance,
                best_goal_distance=self._frontier_best_goal_distance,
                regression_tolerance=self._frontier_goal_regression_tolerance,
            ):
                self._frontier_avoid_points = avoid_points[-8:]
                return path
            avoid_points.append((frontier.x, frontier.y))
        self._frontier_avoid_points = avoid_points[-8:]
        return []

    def _reset_frontier_progress_if_new_goal(
        self,
        final_goal_xy: tuple[float, float],
    ) -> None:
        if self._frontier_progress_final_goal_xy is None:
            self._frontier_progress_final_goal_xy = final_goal_xy
            self._frontier_best_goal_distance = None
            return
        goal_distance = math.hypot(
            self._frontier_progress_final_goal_xy[0] - final_goal_xy[0],
            self._frontier_progress_final_goal_xy[1] - final_goal_xy[1],
        )
        if goal_distance <= max(0.0, self._duplicate_goal_xy_tolerance):
            return
        self._frontier_progress_final_goal_xy = final_goal_xy
        self._frontier_best_goal_distance = None

    def _remember_frontier_progress(
        self,
        frontier: TerrainNode,
        final_goal_xy: tuple[float, float],
    ) -> None:
        distance = math.hypot(
            frontier.x - final_goal_xy[0],
            frontier.y - final_goal_xy[1],
        )
        if (
            self._frontier_best_goal_distance is None
            or distance < self._frontier_best_goal_distance
        ):
            self._frontier_best_goal_distance = distance

    def _frontier_blocked_points_from_scan(self) -> list[tuple[float, float]]:
        if self._scan_msg is None:
            return []
        base_x, base_y, base_yaw, _ = self._current_planar_pose()
        range_limit = max(0.0, self._frontier_obstacle_range_max)
        if range_limit <= 0.0:
            return []
        blocked: list[tuple[float, float]] = []
        angle = float(self._scan_msg.angle_min)
        increment = float(self._scan_msg.angle_increment)
        max_range = min(float(self._scan_msg.range_max), range_limit)
        for distance in self._scan_msg.ranges:
            if (
                math.isfinite(distance)
                and distance >= float(self._scan_msg.range_min)
                and distance <= max_range
            ):
                world_angle = base_yaw + angle
                blocked.append(
                    (
                        base_x + math.cos(world_angle) * distance,
                        base_y + math.sin(world_angle) * distance,
                    )
                )
            angle += increment
        return blocked

    def _publish_and_execute_path(
        self,
        path: list[TerrainNode],
        original_goal: PoseStamped,
    ) -> None:
        self._publish_path(path)
        if not self._send_nav2_goals:
            return
        if self._nav_execution_mode == 'direct':
            self._start_direct_tracking(
                path,
                final_goal_xy=(
                    float(original_goal.pose.position.x),
                    float(original_goal.pose.position.y),
                ),
            )
        elif self._nav_execution_mode == 'follow_path':
            self._send_follow_path_goal(path)
        else:
            self._send_waypoint_goal(path, original_goal)

    def _is_duplicate_goal(self, goal_xy: tuple[float, float]) -> bool:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_goal_xy is None or self._last_goal_time_ns is None:
            self._last_goal_xy = goal_xy
            self._last_goal_time_ns = now_ns
            return False
        distance = math.hypot(
            goal_xy[0] - self._last_goal_xy[0],
            goal_xy[1] - self._last_goal_xy[1],
        )
        elapsed_sec = (now_ns - self._last_goal_time_ns) / 1_000_000_000.0
        duplicate = (
            distance <= max(0.0, self._duplicate_goal_xy_tolerance)
            and elapsed_sec <= max(0.0, self._duplicate_goal_time_sec)
        )
        if not duplicate:
            self._last_goal_xy = goal_xy
            self._last_goal_time_ns = now_ns
        return duplicate

    def _publish_path(self, path: list[TerrainNode]) -> None:
        msg = PathMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._world_frame
        msg.poses = [
            self._pose_stamped_for_node(node, path, index, msg.header.stamp)
            for index, node in enumerate(path)
        ]
        self._path_publisher.publish(msg)

    def _send_waypoint_goal(
        self,
        path: list[TerrainNode],
        original_goal: PoseStamped,
    ) -> None:
        if not self._waypoint_action_client.server_is_ready():
            if not self._waypoint_action_client.wait_for_server(timeout_sec=0.1):
                self.get_logger().warning(
                    'navigate_through_poses server is not ready; '
                    'published /pct_path only'
                )
                return
        goal_msg = NavigateThroughPoses.Goal()
        reduced_path = _waypoint_path(path, self._waypoint_spacing)
        start_x, start_y, _ = self._current_pose()
        nav_path = _waypoints_after_start_clearance(
            reduced_path,
            (start_x, start_y),
            self._start_waypoint_clearance,
        )
        stamp = self.get_clock().now().to_msg()
        poses = [
            self._pose_stamped_for_node(node, nav_path, index, stamp)
            for index, node in enumerate(nav_path)
        ]
        if poses:
            poses[-1].pose.orientation = original_goal.pose.orientation
        goal_msg.poses = poses
        self._dispatch_or_cancel_active(
            goal_msg,
            len(poses),
            len(path),
            mode='waypoints',
        )

    def _send_follow_path_goal(self, path: list[TerrainNode]) -> None:
        if not self._follow_path_action_client.server_is_ready():
            if not self._follow_path_action_client.wait_for_server(timeout_sec=0.1):
                self.get_logger().warning(
                    'follow_path server is not ready; published /pct_path only'
                )
                return
        filtered_path = _waypoints_after_start_clearance(
            path,
            self._current_pose()[:2],
            self._follow_path_start_clearance,
        )
        path_msg = PathMsg()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self._world_frame
        path_msg.poses = [
            self._pose_stamped_for_node(node, filtered_path, index, path_msg.header.stamp)
            for index, node in enumerate(filtered_path)
        ]
        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg
        goal_msg.controller_id = 'FollowPath'
        goal_msg.goal_checker_id = 'general_goal_checker'
        self._publish_speed_limit(path)
        self._dispatch_or_cancel_active(
            goal_msg,
            len(path_msg.poses),
            len(path),
            mode='follow_path',
        )

    def _start_direct_tracking(
        self,
        path: list[TerrainNode],
        *,
        final_goal_xy: Optional[tuple[float, float]] = None,
    ) -> None:
        start_x, start_y, _, start_z = self._current_planar_pose()
        start_xy = (start_x, start_y)
        start_clearance = direct_tracking_start_clearance(
            follow_path_start_clearance=self._follow_path_start_clearance,
            start_waypoint_clearance=self._start_waypoint_clearance,
            direct_waypoint_tolerance=self._direct_waypoint_tolerance,
            direct_lookahead_dist=self._direct_lookahead_dist,
        )
        direct_path = _waypoints_after_start_clearance(
            path,
            start_xy,
            start_clearance,
            current_z=start_z,
            z_tolerance=self._direct_z_tolerance,
        )
        if final_goal_xy is not None:
            direct_path = drop_regressive_start_waypoints(
                direct_path,
                start_xy=start_xy,
                final_goal_xy=final_goal_xy,
                regression_tolerance=self._direct_waypoint_tolerance,
                current_z=start_z,
                z_tolerance=self._direct_z_tolerance,
            )
        self._publish_speed_limit(path)
        self._direct_path = direct_path
        self._direct_final_goal_xy = final_goal_xy
        self._direct_target_index = 0
        self._direct_speed_limit = self._direct_surface_speed_limit()
        if self._direct_path:
            self._frontier_stall_start_xy = start_xy
            self._frontier_stall_start_time_ns = self.get_clock().now().nanoseconds
            self._frontier_stall_commanded_motion = False
        self.get_logger().info(
            'started terrain-guided direct tracking: '
            f'poses={len(direct_path)} path_nodes={len(path)}'
        )

    def _direct_control_tick(self) -> None:
        if not self._direct_path:
            return

        current_x, current_y, current_yaw, current_z = self._current_planar_pose()
        goal = self._direct_path[-1]
        if _direct_node_reached(
            goal,
            current_xy=(current_x, current_y),
            current_z=current_z,
            xy_tolerance=self._direct_goal_tolerance,
            z_tolerance=self._direct_z_tolerance,
        ):
            if (
                self._active_frontier_path
                and self._pending_final_goal_xy is not None
            ):
                self._remember_frontier_progress(goal, self._pending_final_goal_xy)
            self._direct_path = []
            self._direct_final_goal_xy = None
            self._direct_target_index = 0
            if not should_hold_active_frontier_path(
                active_path=self._active_frontier_path,
                current_xy=(current_x, current_y),
                goal_tolerance=self._direct_goal_tolerance,
            ):
                self._active_frontier_path = []
                self._active_frontier_final_goal_xy = None
                self._reset_frontier_stall_monitor()
            self._publish_direct_stop()
            self.get_logger().info('terrain direct tracking goal reached')
            return

        self._advance_direct_target(current_x, current_y, current_z)
        if self._release_stalled_frontier_if_needed((current_x, current_y)):
            return
        self._direct_speed_limit = self._direct_surface_speed_limit()
        self._publish_speed_limit_for_direct_target()
        target = self._direct_lookahead_target(current_x, current_y, current_z)
        dx = target.x - current_x
        dy = target.y - current_y
        target_distance = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)
        heading_error = _normalize_angle(desired_yaw - current_yaw)
        twist = Twist()
        twist.linear.x = _direct_linear_speed(
            speed_limit=self._direct_speed_limit,
            max_linear_speed=self._direct_max_linear_speed,
            min_linear_speed=self._direct_min_linear_speed,
            heading_error=heading_error,
            max_heading_error_for_forward=(
                self._direct_max_heading_error_for_forward
            ),
            target_distance=target_distance,
            slow_radius=max(self._direct_lookahead_dist, 0.05),
        )
        twist.angular.z = _clamp(
            self._direct_heading_gain * heading_error,
            -self._direct_max_angular_speed,
            self._direct_max_angular_speed,
        )
        if direct_command_requests_translation(twist.linear.x):
            if not self._frontier_stall_commanded_motion:
                self._frontier_stall_start_xy = (current_x, current_y)
                self._frontier_stall_start_time_ns = (
                    self.get_clock().now().nanoseconds
                )
            self._frontier_stall_commanded_motion = True
        self._direct_cmd_vel_publisher.publish(twist)

    def _release_stalled_frontier_if_needed(
        self,
        current_xy: tuple[float, float],
    ) -> bool:
        if self._frontier_stall_start_time_ns is None:
            return False
        now_ns = self.get_clock().now().nanoseconds
        was_frontier_path = bool(self._active_frontier_path)
        tracking_path = (
            self._active_frontier_path
            if was_frontier_path
            else self._direct_path
        )
        if not tracking_path:
            return False
        tracking_goal = select_stall_tracking_goal(
            direct_path=self._direct_path,
            active_frontier_path=self._active_frontier_path,
            direct_target_index=self._direct_target_index,
        )
        if tracking_goal is None:
            return False
        tracking_goal_xy = (tracking_goal.x, tracking_goal.y)
        if should_refresh_frontier_stall_monitor(
            current_xy=current_xy,
            monitor_start_xy=self._frontier_stall_start_xy,
            min_progress=self._frontier_stall_min_progress,
            goal_xy=tracking_goal_xy,
        ):
            self._frontier_stall_start_xy = current_xy
            self._frontier_stall_start_time_ns = now_ns
            self._frontier_stall_commanded_motion = False
            return False
        elapsed_sec = (now_ns - self._frontier_stall_start_time_ns) / 1_000_000_000.0
        if not should_release_stalled_frontier_path(
            active_path=tracking_path,
            commanded_motion=self._frontier_stall_commanded_motion,
            current_xy=current_xy,
            monitor_start_xy=self._frontier_stall_start_xy,
            elapsed_sec=elapsed_sec,
            min_progress=self._frontier_stall_min_progress,
            timeout_sec=self._frontier_stall_timeout_sec,
            goal_xy=tracking_goal_xy,
        ):
            return False
        stalled_goal = tracking_goal
        if was_frontier_path:
            self._frontier_avoid_points.append((stalled_goal.x, stalled_goal.y))
            self._frontier_avoid_points = self._frontier_avoid_points[-8:]
            self._active_frontier_path = []
            self._active_frontier_final_goal_xy = None
        else:
            self._pending_final_goal_xy = self._direct_final_goal_xy or tracking_goal_xy
        self._direct_path = []
        self._direct_final_goal_xy = None
        self._direct_target_index = 0
        self._reset_frontier_stall_monitor()
        self._publish_direct_stop()
        if was_frontier_path:
            self.get_logger().warning(
                'released stalled FAST-LIO frontier path: '
                f'frontier=({stalled_goal.x:.2f},{stalled_goal.y:.2f})'
            )
        else:
            self.get_logger().warning(
                'released stalled FAST-LIO direct path for replanning: '
                f'goal=({stalled_goal.x:.2f},{stalled_goal.y:.2f})'
            )
        if self._pending_final_goal_xy is not None:
            self._try_pending_final_goal()
        return True

    def _reset_frontier_stall_monitor(self) -> None:
        self._frontier_stall_start_xy = None
        self._frontier_stall_start_time_ns = None
        self._frontier_stall_commanded_motion = False

    def _advance_direct_target(
        self,
        current_x: float,
        current_y: float,
        current_z: float,
    ) -> None:
        self._direct_target_index = advance_direct_target_index(
            self._direct_path,
            self._direct_target_index,
            (current_x, current_y),
            self._direct_waypoint_tolerance,
            current_z=current_z,
            z_tolerance=self._direct_z_tolerance,
        )

    def _direct_lookahead_target(
        self,
        current_x: float,
        current_y: float,
        current_z: float,
    ) -> TerrainNode:
        target = self._direct_path[self._direct_target_index]
        target_surface = target.surface_label
        for index in range(self._direct_target_index, len(self._direct_path)):
            candidate = self._direct_path[index]
            if candidate.surface_label != target_surface:
                if index > self._direct_target_index:
                    return target
                continue
            distance = math.hypot(candidate.x - current_x, candidate.y - current_y)
            if _direct_height_error(
                candidate,
                current_z=current_z,
                z_tolerance=self._direct_z_tolerance,
            ) > 0.0:
                return target
            if distance >= max(0.0, self._direct_lookahead_dist):
                return candidate
            target = candidate
        return target

    def _publish_direct_stop(self) -> None:
        self._direct_cmd_vel_publisher.publish(Twist())

    def _direct_surface_speed_limit(self) -> float:
        if not self._direct_path:
            return max(0.01, self._flat_speed_limit)
        surface_label = self._direct_path[self._direct_target_index].surface_label
        return _surface_speed_limit_for_label(
            surface_label,
            slope_speed_limit=self._slope_speed_limit,
            flat_speed_limit=self._flat_speed_limit,
        )

    def _publish_speed_limit_for_direct_target(self) -> None:
        if not self._direct_path:
            return
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._world_frame
        msg.percentage = False
        msg.speed_limit = self._direct_speed_limit
        self._speed_limit_publisher.publish(msg)

    def _publish_speed_limit(self, path: list[TerrainNode]) -> None:
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._world_frame
        msg.percentage = False
        msg.speed_limit = _path_speed_limit(
            path,
            slope_speed_limit=self._slope_speed_limit,
            flat_speed_limit=self._flat_speed_limit,
            slope_grade_threshold=self._slope_speed_grade_threshold,
        )
        self._speed_limit_publisher.publish(msg)

    def _dispatch_or_cancel_active(
        self,
        goal_msg: FollowPath.Goal | NavigateThroughPoses.Goal,
        waypoint_count: int,
        path_node_count: int,
        mode: str,
    ) -> None:
        if self._active_nav_goal is not None and _goal_is_active(
            self._active_nav_goal.status
        ):
            self._pending_nav_goal = (
                goal_msg,
                waypoint_count,
                path_node_count,
                mode,
            )
            cancel_future = self._active_nav_goal.cancel_goal_async()
            cancel_future.add_done_callback(self._on_previous_goal_cancelled)
            self.get_logger().info(
                'cancelled previous terrain-guided navigation goal before '
                'sending a new one'
            )
            return

        self._dispatch_nav_goal(goal_msg, waypoint_count, path_node_count, mode)

    def _dispatch_nav_goal(
        self,
        goal_msg: FollowPath.Goal | NavigateThroughPoses.Goal,
        waypoint_count: int,
        path_node_count: int,
        mode: str,
    ) -> None:
        if mode == 'follow_path':
            client = self._follow_path_action_client
        else:
            client = self._waypoint_action_client
        send_future = client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_goal_response)
        self.get_logger().info(
            f'sent terrain-guided {mode} goal: '
            f'poses={waypoint_count} path_nodes={path_node_count}'
        )

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning(
                'terrain-guided navigation goal was rejected'
            )
            return
        self._active_nav_goal = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        self._active_nav_goal = None
        if self._pending_nav_goal is not None:
            pending_goal, waypoint_count, path_node_count, mode = self._pending_nav_goal
            self._pending_nav_goal = None
            self._dispatch_nav_goal(
                pending_goal,
                waypoint_count,
                path_node_count,
                mode,
            )

    def _on_previous_goal_cancelled(self, future) -> None:
        self._active_nav_goal = None
        if self._pending_nav_goal is None:
            return
        pending_goal, waypoint_count, path_node_count, mode = self._pending_nav_goal
        self._pending_nav_goal = None
        self._dispatch_nav_goal(
            pending_goal,
            waypoint_count,
            path_node_count,
            mode,
        )

    def _pose_stamped_for_node(
        self,
        node: TerrainNode,
        path: list[TerrainNode],
        index: int,
        stamp,
    ) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self._world_frame
        pose.pose.position.x = node.x
        pose.pose.position.y = node.y
        pose.pose.position.z = node.z
        next_node = path[min(index + 1, len(path) - 1)]
        yaw = math.atan2(next_node.y - node.y, next_node.x - node.x)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _publish_terrain_cloud(self) -> None:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._world_frame
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name='intensity',
                offset=12,
                datatype=PointField.FLOAT32,
                count=1,
            ),
        ]
        self._terrain_cloud_publisher.publish(
            point_cloud2.create_cloud(header, fields, self._graph.terrain_cloud)
        )


def _build_adjacency(
    nodes: list[TerrainNode],
    grid_resolution: float,
    max_slope_grade: float,
    max_step_height: float,
    max_surface_transition_height: float,
) -> list[list[tuple[int, float]]]:
    adjacency: list[list[tuple[int, float]]] = [[] for _ in nodes]
    bins: dict[tuple[int, int], list[int]] = {}
    for node in nodes:
        key = _bin_key(node.x, node.y, grid_resolution)
        bins.setdefault(key, []).append(node.index)

    neighbor_radius = grid_resolution * 1.65
    for node in nodes:
        key_x, key_y = _bin_key(node.x, node.y, grid_resolution)
        candidates: set[int] = set()
        for bx in range(key_x - 2, key_x + 3):
            for by in range(key_y - 2, key_y + 3):
                candidates.update(bins.get((bx, by), []))
        for other_index in candidates:
            if other_index == node.index:
                continue
            other = nodes[other_index]
            horizontal = math.hypot(other.x - node.x, other.y - node.y)
            if horizontal < grid_resolution * 0.35:
                continue
            if horizontal > neighbor_radius:
                continue
            dz = abs(other.z - node.z)
            grade = dz / max(horizontal, 1e-6)
            step_height_limit = max_step_height
            if (
                'stair' in node.surface_label
                or 'step' in node.surface_label
                or 'stair' in other.surface_label
                or 'step' in other.surface_label
            ):
                step_height_limit = max(step_height_limit, 0.25)
            if dz > step_height_limit or grade > max_slope_grade:
                continue
            surface_changed = other.surface_label != node.surface_label
            if (
                surface_changed
                and dz > max(0.0, max_surface_transition_height)
            ):
                continue
            if surface_changed and not _valid_surface_transition(node, other):
                continue
            slope_cost = 1.0 + grade * 1.8
            edge_risk = _edge_risk(node) + _edge_risk(other)
            transition_cost = grid_resolution if surface_changed else 0.0
            cost = math.sqrt(horizontal * horizontal + dz * dz) * slope_cost
            adjacency[node.index].append(
                (other_index, cost + edge_risk + transition_cost)
            )
    return adjacency


def _valid_surface_transition(node: TerrainNode, other: TerrainNode) -> bool:
    if _is_slam_surface_label(node.surface_label) or _is_slam_surface_label(
        other.surface_label
    ):
        return True
    if _is_ramp_label(node.surface_label) and not _is_ramp_label(
        other.surface_label
    ):
        return _is_ramp_entry_node(node)
    if _is_ramp_label(other.surface_label) and not _is_ramp_label(
        node.surface_label
    ):
        return _is_ramp_entry_node(other)
    return True


def _blocked_by_obstacle(
    point: tuple[float, float, float],
    obstacles: list[CollisionGeometry],
    clearance: float,
    current_surface: BoxCollision,
) -> bool:
    for obstacle in obstacles:
        if isinstance(obstacle, BoxCollision):
            if obstacle.model_name == current_surface.model_name:
                continue
            local = inverse_transform_point(obstacle.transform, point)
            half_x = obstacle.size[0] / 2.0 + clearance
            half_y = obstacle.size[1] / 2.0 + clearance
            z_margin = obstacle.size[2] / 2.0 + 0.45
            if (
                abs(local[0]) <= half_x
                and abs(local[1]) <= half_y
                and abs(local[2]) <= z_margin
            ):
                return True
            continue
        local = inverse_transform_point(obstacle.transform, point)
        if math.hypot(local[0], local[1]) <= obstacle.radius + clearance:
            if abs(local[2]) <= obstacle.length / 2.0 + 0.45:
                return True
    return False


def _surface_edge_margin(
    box: BoxCollision,
    point: tuple[float, float, float],
) -> float:
    local = inverse_transform_point(box.transform, point)
    return min(
        box.size[0] / 2.0 - abs(local[0]),
        box.size[1] / 2.0 - abs(local[1]),
    )


def _edge_risk(node: TerrainNode) -> float:
    if _is_floor_label(node.surface_label):
        return 0.0
    return max(0.0, 0.65 - node.edge_margin) * 1.2


def _nearest_node(
    nodes: list[TerrainNode],
    xy: tuple[float, float],
    z_reference: float,
    policy: str,
) -> Optional[int]:
    nearby = sorted(
        nodes,
        key=lambda node: math.hypot(node.x - xy[0], node.y - xy[1]),
    )[:80]
    if not nearby:
        return None
    if policy == 'highest':
        min_xy = math.hypot(nearby[0].x - xy[0], nearby[0].y - xy[1])
        candidates = [
            node
            for node in nearby
            if math.hypot(node.x - xy[0], node.y - xy[1]) <= min_xy + 0.75
        ]
        return max(candidates, key=lambda node: node.z).index
    if policy == 'lowest':
        min_xy = math.hypot(nearby[0].x - xy[0], nearby[0].y - xy[1])
        candidates = [
            node
            for node in nearby
            if math.hypot(node.x - xy[0], node.y - xy[1]) <= min_xy + 0.75
        ]
        return min(candidates, key=lambda node: node.z).index
    return min(
        nearby,
        key=lambda node: (
            math.hypot(node.x - xy[0], node.y - xy[1])
            + abs(node.z - z_reference) * 0.6
        ),
    ).index


def _thin_path(path: list[TerrainNode]) -> list[TerrainNode]:
    if len(path) <= 2:
        return path
    reduced = [path[0]]
    previous_heading: Optional[float] = None
    distance_since_keep = 0.0
    for index in range(1, len(path) - 1):
        last = path[index - 1]
        current = path[index]
        next_node = path[index + 1]
        distance_since_keep += math.hypot(current.x - last.x, current.y - last.y)
        heading = math.atan2(next_node.y - current.y, next_node.x - current.x)
        heading_change = (
            abs(_normalize_angle(heading - previous_heading))
            if previous_heading is not None
            else 0.0
        )
        if distance_since_keep >= 0.8 or heading_change >= 0.35:
            reduced.append(current)
            distance_since_keep = 0.0
        previous_heading = heading
    reduced.append(path[-1])
    return reduced


def _waypoint_path(
    path: list[TerrainNode],
    spacing: float,
) -> list[TerrainNode]:
    if len(path) <= 2:
        return path
    waypoints = [path[0]]
    distance = 0.0
    for index in range(1, len(path) - 1):
        last = path[index - 1]
        current = path[index]
        distance += math.hypot(current.x - last.x, current.y - last.y)
        vertical_change = abs(current.z - waypoints[-1].z)
        surface_changed = current.surface_label != waypoints[-1].surface_label
        if distance >= spacing or vertical_change >= 0.16 or surface_changed:
            waypoints.append(current)
            distance = 0.0
    waypoints.append(path[-1])
    return waypoints


def _waypoints_after_start_clearance(
    path: list[TerrainNode],
    start_xy: tuple[float, float],
    clearance_radius: float,
    current_z: Optional[float] = None,
    z_tolerance: float = math.inf,
) -> list[TerrainNode]:
    if len(path) <= 1:
        return path
    filtered: list[TerrainNode] = []
    for node in path:
        if _direct_node_reached(
            node,
            current_xy=start_xy,
            current_z=current_z,
            xy_tolerance=clearance_radius,
            z_tolerance=z_tolerance,
        ):
            continue
        filtered.append(node)
    return filtered or [path[-1]]


def _path_speed_limit(
    path: list[TerrainNode],
    slope_speed_limit: float,
    flat_speed_limit: float,
    slope_grade_threshold: float,
) -> float:
    if _path_has_slope_grade(path, slope_grade_threshold):
        return max(0.01, slope_speed_limit)
    return max(0.01, flat_speed_limit)


def _surface_speed_limit_for_label(
    surface_label: str,
    slope_speed_limit: float,
    flat_speed_limit: float,
) -> float:
    if _is_slope_label(surface_label):
        return max(0.01, slope_speed_limit)
    return max(0.01, flat_speed_limit)


def _path_has_slope_grade(
    path: list[TerrainNode],
    slope_grade_threshold: float,
) -> bool:
    for first, second in zip(path, path[1:]):
        horizontal = math.hypot(second.x - first.x, second.y - first.y)
        if horizontal <= 1e-6:
            continue
        grade = abs(second.z - first.z) / horizontal
        if grade >= max(0.0, slope_grade_threshold):
            return True
    return False


def _surface_z_reference(
    odom_z: float,
    current_xy: tuple[float, float],
    initial_xy: Optional[tuple[float, float]],
    initial_surface_z_hint: float,
    initial_surface_hint_radius: float,
    last_path: list[TerrainNode],
    last_path_surface_hint_radius: float,
) -> float:
    if abs(odom_z) > 0.05:
        return odom_z

    last_path_z = _nearest_path_surface_z(
        last_path,
        current_xy,
        max(0.0, last_path_surface_hint_radius),
    )
    if last_path_z is not None:
        return last_path_z

    if initial_xy is not None and initial_surface_z_hint >= 0.0:
        distance_from_initial = math.hypot(
            current_xy[0] - initial_xy[0],
            current_xy[1] - initial_xy[1],
        )
        if distance_from_initial <= max(0.0, initial_surface_hint_radius):
            return initial_surface_z_hint

    return odom_z


def _nearest_path_surface_z(
    path: list[TerrainNode],
    xy: tuple[float, float],
    max_distance: float,
) -> Optional[float]:
    if not path:
        return None
    nearest = min(path, key=lambda node: math.hypot(node.x - xy[0], node.y - xy[1]))
    distance = math.hypot(nearest.x - xy[0], nearest.y - xy[1])
    if distance <= max_distance:
        return nearest.z
    return None


def _surface_height_at_xy(
    nodes: list[TerrainNode],
    xy: tuple[float, float],
    z_hint: float,
    max_xy_distance: float = 0.75,
) -> Optional[float]:
    if z_hint < 0.0:
        return None
    candidates = [
        node
        for node in nodes
        if math.hypot(node.x - xy[0], node.y - xy[1])
        <= max(0.0, max_xy_distance)
    ]
    if not candidates:
        return None
    selected = min(
        candidates,
        key=lambda node: (
            math.hypot(node.x - xy[0], node.y - xy[1])
            + abs(node.z - z_hint) * 0.6
        ),
    )
    return selected.z


def _heuristic(node: TerrainNode, goal: TerrainNode) -> float:
    return math.sqrt(
        (node.x - goal.x) ** 2
        + (node.y - goal.y) ** 2
        + (node.z - goal.z) ** 2
    )


def _bin_key(x: float, y: float, resolution: float) -> tuple[int, int]:
    return int(round(x / resolution)), int(round(y / resolution))


def _is_floor_label(label: str) -> bool:
    return 'floor' in label or 'ground' in label


def _is_slope_label(label: str) -> bool:
    return 'ramp' in label or 'slope' in label or 'stair' in label


def _is_ramp_label(label: str) -> bool:
    return 'ramp' in label or 'slope' in label


def _is_slam_surface_label(label: str) -> bool:
    return label.startswith('slam_')


def _is_ramp_entry_node(node: TerrainNode) -> bool:
    if not _is_ramp_label(node.surface_label):
        return False
    local_width = (
        node.surface_local_x
        if node.surface_width_axis == 'x'
        else node.surface_local_y
    )
    half_width = (
        node.surface_half_x
        if node.surface_width_axis == 'x'
        else node.surface_half_y
    )
    if half_width <= 0.0:
        return node.edge_margin >= 0.40
    return abs(local_width) <= half_width * 0.35


def _box_width_axis(box: BoxCollision) -> str:
    return 'x' if box.size[0] <= box.size[1] else 'y'


def _surface_support_margin(label: str, support_margin: float) -> float:
    if _is_floor_label(label):
        return 0.0
    if 'stair' in label or 'step' in label:
        return min(max(0.0, support_margin), 0.08)
    return support_margin


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def _direct_linear_speed(
    speed_limit: float,
    max_linear_speed: float,
    min_linear_speed: float,
    heading_error: float,
    max_heading_error_for_forward: float,
    target_distance: float,
    slow_radius: float,
) -> float:
    capped_speed = min(max(speed_limit, 0.0), max_linear_speed)
    if capped_speed <= 0.0:
        return 0.0
    if target_distance <= 0.03:
        return 0.0
    heading_limit = max(max_heading_error_for_forward, 1e-6)
    if abs(heading_error) >= heading_limit:
        return 0.0
    heading_scale = max(
        0.0,
        1.0 - abs(heading_error) / heading_limit,
    )
    distance_scale = min(max(target_distance / max(slow_radius, 1e-6), 0.0), 1.0)
    scaled_speed = capped_speed * max(heading_scale, 0.20) * distance_scale
    return max(min_linear_speed, scaled_speed)


def direct_command_requests_translation(
    linear_x: float,
    min_linear_x: float = 0.01,
) -> bool:
    return abs(linear_x) > max(0.0, min_linear_x)


def main() -> None:
    rclpy.init()
    node = TerrainPctPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
