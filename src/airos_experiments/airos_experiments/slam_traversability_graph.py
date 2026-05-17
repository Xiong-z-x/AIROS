from __future__ import annotations

import heapq
import math
import struct
from collections import defaultdict
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from sensor_msgs.msg import PointCloud2

from airos_experiments.sdf_geometry import CloudPoint, terrain_intensity


class TraversabilityLabel(str, Enum):
    FLOOR = 'slam_floor'
    RAMP = 'slam_ramp'
    STEP = 'slam_step'
    PLATFORM = 'slam_deck'


@dataclass
class SlamTerrainNode:
    index: int
    x: float
    y: float
    z: float
    label: TraversabilityLabel
    edge_margin: float = 0.0
    surface_local_x: float = 0.0
    surface_local_y: float = 0.0
    surface_half_x: float = 0.0
    surface_half_y: float = 0.0
    surface_width_axis: str = 'y'


@dataclass(frozen=True)
class SlamTerrainGraph:
    nodes: list[SlamTerrainNode]
    adjacency: list[list[tuple[int, float]]]
    terrain_cloud: list[CloudPoint]


def build_slam_graph_from_pointcloud(
    msg: PointCloud2,
    grid_resolution: float = 0.25,
    max_slope_grade: float = 0.55,
    max_step_height: float = 0.34,
    max_surface_transition_height: float = 0.12,
    min_cell_points: int = 2,
    vertical_layer_gap: float = 0.18,
    max_points: int = 180000,
    obstacle_clearance: float = 0.0,
    vertical_obstacle_min_height: float = 0.45,
) -> SlamTerrainGraph:
    return build_slam_graph_from_points(
        sample_xyz_points(msg, max_points=max_points),
        grid_resolution=grid_resolution,
        max_slope_grade=max_slope_grade,
        max_step_height=max_step_height,
        max_surface_transition_height=max_surface_transition_height,
        min_cell_points=min_cell_points,
        vertical_layer_gap=vertical_layer_gap,
        obstacle_clearance=obstacle_clearance,
        vertical_obstacle_min_height=vertical_obstacle_min_height,
    )


def build_slam_graph_from_points(
    points: list[tuple[float, float, float]],
    grid_resolution: float = 0.25,
    max_slope_grade: float = 0.55,
    max_step_height: float = 0.34,
    max_surface_transition_height: float = 0.12,
    min_cell_points: int = 2,
    vertical_layer_gap: float = 0.18,
    obstacle_clearance: float = 0.0,
    vertical_obstacle_min_height: float = 0.45,
) -> SlamTerrainGraph:
    if not points:
        return SlamTerrainGraph(nodes=[], adjacency=[], terrain_cloud=[])

    bins: dict[tuple[int, int], list[tuple[float, float, float]]] = defaultdict(list)
    for x, y, z in points:
        bins[_bin_key(x, y, grid_resolution)].append((x, y, z))

    obstacle_bins = _vertical_obstacle_bins(
        bins,
        min_cell_points=min_cell_points,
        vertical_layer_gap=vertical_layer_gap,
        vertical_obstacle_min_height=vertical_obstacle_min_height,
    )
    obstacle_clearance = max(0.0, obstacle_clearance)
    nodes: list[SlamTerrainNode] = []
    for (bin_x, bin_y), cell_points in bins.items():
        if len(cell_points) < max(1, min_cell_points):
            continue
        cell_points.sort(key=lambda point: point[2])
        clusters = _height_clusters(cell_points, vertical_layer_gap)
        blocked_low_clusters = _blocked_low_cluster_indexes(
            clusters,
            max_step_height=max_step_height,
            min_cell_points=min_cell_points,
        )
        for cluster_index, cluster in enumerate(clusters):
            if len(cluster) < max(1, min_cell_points):
                continue
            if cluster_index in blocked_low_clusters:
                continue
            node = _node_from_cluster(
                cluster,
                index=len(nodes),
                cell_center=(bin_x * grid_resolution, bin_y * grid_resolution),
                grid_resolution=grid_resolution,
            )
            if _xy_near_obstacle_bins(
                node.x,
                node.y,
                obstacle_bins,
                grid_resolution,
                obstacle_clearance,
            ):
                continue
            nodes.append(node)

    _classify_nodes(
        nodes,
        grid_resolution=grid_resolution,
        max_slope_grade=max_slope_grade,
        max_step_height=max_step_height,
    )
    adjacency = _build_adjacency(
        nodes,
        grid_resolution=grid_resolution,
        max_slope_grade=max_slope_grade,
        max_step_height=max_step_height,
        max_surface_transition_height=max(
            max_surface_transition_height,
            max_step_height,
        ),
        obstacle_bins=obstacle_bins,
        obstacle_clearance=obstacle_clearance,
    )
    terrain_cloud = [
        (node.x, node.y, node.z, terrain_intensity(node.label.value))
        for node in nodes
    ]
    return SlamTerrainGraph(
        nodes=nodes,
        adjacency=adjacency,
        terrain_cloud=terrain_cloud,
    )


def plan_slam_graph_path(
    graph: SlamTerrainGraph,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    start_z: float = 0.0,
    goal_z_policy: str = 'highest',
) -> list[SlamTerrainNode]:
    if not graph.nodes:
        return []
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
        return []
    if start_index == goal_index:
        return [graph.nodes[start_index]]

    distances = [math.inf] * len(graph.nodes)
    parents: list[Optional[int]] = [None] * len(graph.nodes)
    distances[start_index] = 0.0
    queue: list[tuple[float, int]] = [
        (_heuristic(graph.nodes[start_index], graph.nodes[goal_index]), start_index)
    ]

    while queue:
        _, current = heapq.heappop(queue)
        if current == goal_index:
            break
        current_distance = distances[current]
        if not math.isfinite(current_distance):
            continue
        for neighbor, edge_cost in graph.adjacency[current]:
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
        return []

    path: list[SlamTerrainNode] = []
    cursor: Optional[int] = goal_index
    while cursor is not None:
        path.append(graph.nodes[cursor])
        cursor = parents[cursor]
    path.reverse()
    return path


def sample_xyz_points(
    msg: PointCloud2,
    *,
    max_points: int,
) -> list[tuple[float, float, float]]:
    offsets = {field.name: field.offset for field in msg.fields}
    if not {'x', 'y', 'z'}.issubset(offsets):
        return []
    point_step = int(msg.point_step)
    total = int(msg.width) * int(msg.height)
    if point_step <= 0 or total <= 0:
        return []
    stride = 1
    if max_points > 0 and total > max_points:
        stride = int(math.ceil(total / max_points))

    endian = '>' if msg.is_bigendian else '<'
    points: list[tuple[float, float, float]] = []
    for index in range(0, total, stride):
        base = index * point_step
        try:
            x = struct.unpack_from(endian + 'f', msg.data, base + offsets['x'])[0]
            y = struct.unpack_from(endian + 'f', msg.data, base + offsets['y'])[0]
            z = struct.unpack_from(endian + 'f', msg.data, base + offsets['z'])[0]
        except struct.error:
            break
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            points.append((float(x), float(y), float(z)))
    return points


def _height_clusters(
    points: list[tuple[float, float, float]],
    vertical_layer_gap: float,
) -> list[list[tuple[float, float, float]]]:
    clusters: list[list[tuple[float, float, float]]] = []
    current: list[tuple[float, float, float]] = [points[0]]
    for point in points[1:]:
        if abs(point[2] - current[-1][2]) <= max(0.01, vertical_layer_gap):
            current.append(point)
            continue
        clusters.append(current)
        current = [point]
    clusters.append(current)
    return clusters


def _vertical_obstacle_bins(
    bins: dict[tuple[int, int], list[tuple[float, float, float]]],
    *,
    min_cell_points: int,
    vertical_layer_gap: float,
    vertical_obstacle_min_height: float,
) -> dict[tuple[int, int], tuple[float, float]]:
    obstacle_bins: dict[tuple[int, int], tuple[float, float]] = {}
    min_points = max(2, min_cell_points)
    min_height = max(0.18, vertical_obstacle_min_height)
    for key, cell_points in bins.items():
        if len(cell_points) < min_points:
            continue
        sorted_points = sorted(cell_points, key=lambda point: point[2])
        vertical_clusters = []
        for cluster in _height_clusters(sorted_points, vertical_layer_gap):
            if len(cluster) < min_points:
                continue
            z_values = [point[2] for point in cluster]
            if max(z_values) - min(z_values) >= min_height:
                vertical_clusters.append(cluster)
        if not vertical_clusters:
            continue
        high_points = [point for cluster in vertical_clusters for point in cluster]
        obstacle_bins[key] = (
            sum(point[0] for point in high_points) / len(high_points),
            sum(point[1] for point in high_points) / len(high_points),
        )
    return obstacle_bins


def _blocked_low_cluster_indexes(
    clusters: list[list[tuple[float, float, float]]],
    *,
    max_step_height: float,
    min_cell_points: int,
) -> set[int]:
    blocked: set[int] = set()
    if len(clusters) < 2:
        return blocked
    min_points = max(1, min_cell_points)
    for lower_index, lower in enumerate(clusters[:-1]):
        if len(lower) < min_points:
            continue
        lower_z = sum(point[2] for point in lower) / len(lower)
        stacked_points = 0
        stacked_layers = 0
        top_z = lower_z
        max_upper_span = 0.0
        for upper in clusters[lower_index + 1:]:
            if len(upper) < min_points:
                continue
            upper_z = sum(point[2] for point in upper) / len(upper)
            if upper_z <= lower_z + max(0.18, max_step_height * 0.55):
                continue
            upper_zs = [point[2] for point in upper]
            max_upper_span = max(max_upper_span, max(upper_zs) - min(upper_zs))
            stacked_layers += 1
            stacked_points += len(upper)
            top_z = max(top_z, upper_z)
        if (
            stacked_points >= max(2, min_points * 2)
            and top_z - lower_z >= max(0.45, max_step_height * 1.2)
            and max_upper_span >= max(0.10, max_step_height * 0.25)
        ):
            blocked.add(lower_index)
            continue
        if (
            stacked_layers >= 2
            and stacked_points >= max(3, min_points * 3)
            and top_z - lower_z >= max(0.55, max_step_height * 1.5)
        ):
            blocked.add(lower_index)
    return blocked


def _node_from_cluster(
    cluster: list[tuple[float, float, float]],
    *,
    index: int,
    cell_center: tuple[float, float],
    grid_resolution: float,
) -> SlamTerrainNode:
    xs = [point[0] for point in cluster]
    ys = [point[1] for point in cluster]
    zs = [point[2] for point in cluster]
    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)
    mean_z = sum(zs) / len(zs)
    local_x = mean_x - cell_center[0]
    local_y = mean_y - cell_center[1]
    half_x = max(abs(x - cell_center[0]) for x in xs)
    half_y = max(abs(y - cell_center[1]) for y in ys)
    return SlamTerrainNode(
        index=index,
        x=mean_x,
        y=mean_y,
        z=mean_z,
        label=TraversabilityLabel.FLOOR,
        edge_margin=max(
            0.0,
            min(
                grid_resolution / 2.0 - abs(local_x),
                grid_resolution / 2.0 - abs(local_y),
            ),
        ),
        surface_local_x=local_x,
        surface_local_y=local_y,
        surface_half_x=max(half_x, grid_resolution / 4.0),
        surface_half_y=max(half_y, grid_resolution / 4.0),
    )


def _classify_nodes(
    nodes: list[SlamTerrainNode],
    grid_resolution: float,
    max_slope_grade: float,
    max_step_height: float,
) -> None:
    bins: dict[tuple[int, int], list[int]] = defaultdict(list)
    for node in nodes:
        bins[_bin_key(node.x, node.y, grid_resolution)].append(node.index)

    search_radius = grid_resolution * 1.8
    flat_height_limit = max(0.12, max_step_height * 0.5)
    ramp_grade_limit = max(0.08, max_slope_grade * 0.35)
    platform_height = flat_height_limit + max_step_height

    for node in nodes:
        neighbors = _neighbor_nodes(node, nodes, bins, grid_resolution, search_radius)
        if not neighbors:
            node.label = (
                TraversabilityLabel.FLOOR
                if node.z <= flat_height_limit
                else TraversabilityLabel.PLATFORM
            )
            continue

        local_grades: list[float] = []
        local_dzs: list[float] = []
        for neighbor in neighbors:
            horizontal = math.hypot(neighbor.x - node.x, neighbor.y - node.y)
            if horizontal <= 1e-6:
                continue
            dz = abs(neighbor.z - node.z)
            local_dzs.append(dz)
            local_grades.append(dz / horizontal)

        if not local_grades:
            node.label = (
                TraversabilityLabel.FLOOR
                if node.z <= flat_height_limit
                else TraversabilityLabel.PLATFORM
            )
            continue

        mean_grade = sum(local_grades) / len(local_grades)
        max_dz = max(local_dzs)
        if mean_grade >= ramp_grade_limit and max_dz <= max_step_height * 1.2:
            node.label = TraversabilityLabel.RAMP
        elif max_dz >= max_step_height:
            node.label = TraversabilityLabel.STEP
        elif node.z <= flat_height_limit:
            node.label = TraversabilityLabel.FLOOR
        elif node.z >= platform_height:
            node.label = TraversabilityLabel.PLATFORM
        else:
            node.label = TraversabilityLabel.RAMP


def _neighbor_nodes(
    node: SlamTerrainNode,
    nodes: list[SlamTerrainNode],
    bins: dict[tuple[int, int], list[int]],
    grid_resolution: float,
    search_radius: float,
) -> list[SlamTerrainNode]:
    candidates: set[int] = set()
    key_x, key_y = _bin_key(node.x, node.y, grid_resolution)
    bin_radius = max(1, int(math.ceil(search_radius / max(grid_resolution, 1e-6))))
    for bx in range(key_x - bin_radius, key_x + bin_radius + 1):
        for by in range(key_y - bin_radius, key_y + bin_radius + 1):
            candidates.update(bins.get((bx, by), []))
    candidates.discard(node.index)
    return [
        nodes[index]
        for index in candidates
        if math.hypot(nodes[index].x - node.x, nodes[index].y - node.y)
        <= search_radius
    ]


def _build_adjacency(
    nodes: list[SlamTerrainNode],
    grid_resolution: float,
    max_slope_grade: float,
    max_step_height: float,
    max_surface_transition_height: float,
    obstacle_bins: dict[tuple[int, int], tuple[float, float]],
    obstacle_clearance: float,
) -> list[list[tuple[int, float]]]:
    adjacency: list[list[tuple[int, float]]] = [[] for _ in nodes]
    bins: dict[tuple[int, int], list[int]] = defaultdict(list)
    for node in nodes:
        bins[_bin_key(node.x, node.y, grid_resolution)].append(node.index)

    neighbor_radius = _slam_neighbor_radius(
        grid_resolution=grid_resolution,
        max_slope_grade=max_slope_grade,
        max_step_height=max_step_height,
    )
    for node in nodes:
        for other in _neighbor_nodes(
            node,
            nodes,
            bins,
            grid_resolution,
            neighbor_radius,
        ):
            horizontal = math.hypot(other.x - node.x, other.y - node.y)
            if horizontal < grid_resolution * 0.35:
                continue
            dz = abs(other.z - node.z)
            grade = dz / max(horizontal, 1e-6)
            if dz > max_step_height or grade > max_slope_grade:
                continue
            surface_changed = other.label != node.label
            if surface_changed and dz > max(0.0, max_surface_transition_height):
                continue
            if _edge_near_obstacle_bins(
                node.x,
                node.y,
                other.x,
                other.y,
                obstacle_bins,
                grid_resolution,
                obstacle_clearance,
            ):
                continue
            cost = math.sqrt(horizontal * horizontal + dz * dz) * (1.0 + grade * 1.8)
            if surface_changed:
                cost += grid_resolution
            adjacency[node.index].append((other.index, cost))
    _add_sparse_slope_bridges(
        adjacency,
        nodes,
        bins,
        grid_resolution=grid_resolution,
        max_slope_grade=max_slope_grade,
        max_step_height=max_step_height,
        max_surface_transition_height=max_surface_transition_height,
        obstacle_bins=obstacle_bins,
        obstacle_clearance=obstacle_clearance,
    )
    return adjacency


def _add_sparse_slope_bridges(
    adjacency: list[list[tuple[int, float]]],
    nodes: list[SlamTerrainNode],
    bins: dict[tuple[int, int], list[int]],
    *,
    grid_resolution: float,
    max_slope_grade: float,
    max_step_height: float,
    max_surface_transition_height: float,
    obstacle_bins: dict[tuple[int, int], tuple[float, float]],
    obstacle_clearance: float,
) -> None:
    sparse_radius = _slam_sparse_bridge_radius(
        grid_resolution=grid_resolution,
        max_slope_grade=max_slope_grade,
        max_step_height=max_step_height,
    )
    for node in nodes:
        for other in _neighbor_nodes(
            node,
            nodes,
            bins,
            grid_resolution,
            sparse_radius,
        ):
            if other.index <= node.index:
                continue
            if _has_edge(adjacency[node.index], other.index):
                continue
            bridge_cost = _sparse_slope_bridge_cost(
                node,
                other,
                grid_resolution=grid_resolution,
                max_slope_grade=max_slope_grade,
                max_surface_transition_height=max_surface_transition_height,
                obstacle_bins=obstacle_bins,
                obstacle_clearance=obstacle_clearance,
            )
            if bridge_cost is None:
                continue
            adjacency[node.index].append((other.index, bridge_cost))
            adjacency[other.index].append((node.index, bridge_cost))
    _add_component_sparse_step_bridges(
        adjacency,
        nodes,
        bins,
        grid_resolution=grid_resolution,
        max_slope_grade=max_slope_grade,
        max_surface_transition_height=max_surface_transition_height,
        obstacle_bins=obstacle_bins,
        obstacle_clearance=obstacle_clearance,
        search_radius=max(sparse_radius, grid_resolution * 14.0),
    )


def _add_component_sparse_step_bridges(
    adjacency: list[list[tuple[int, float]]],
    nodes: list[SlamTerrainNode],
    bins: dict[tuple[int, int], list[int]],
    *,
    grid_resolution: float,
    max_slope_grade: float,
    max_surface_transition_height: float,
    obstacle_bins: dict[tuple[int, int], tuple[float, float]],
    obstacle_clearance: float,
    search_radius: float,
) -> None:
    component_ids = _adjacency_component_ids(adjacency)
    best_pairs: dict[tuple[int, int], tuple[float, int, int]] = {}
    for node in nodes:
        if node.label == TraversabilityLabel.FLOOR:
            continue
        for other in _neighbor_nodes(
            node,
            nodes,
            bins,
            grid_resolution,
            search_radius,
        ):
            if other.index <= node.index or other.label == TraversabilityLabel.FLOOR:
                continue
            first_component = component_ids[node.index]
            second_component = component_ids[other.index]
            if first_component == second_component:
                continue
            horizontal = math.hypot(other.x - node.x, other.y - node.y)
            if horizontal < grid_resolution * 3.0:
                continue
            dz = abs(other.z - node.z)
            grade = dz / max(horizontal, 1e-6)
            if grade > max_slope_grade:
                continue
            if not _sparse_bridge_uses_vertical_structure(
                node,
                other,
                min_vertical_change=max(0.06, max_surface_transition_height * 0.25),
            ):
                continue
            component_key = tuple(sorted((first_component, second_component)))
            score = math.sqrt(horizontal * horizontal + dz * dz)
            previous = best_pairs.get(component_key)
            if previous is None or score < previous[0]:
                best_pairs[component_key] = (score, node.index, other.index)
    for _, first_index, second_index in sorted(best_pairs.values()):
        if _has_edge(adjacency[first_index], second_index):
            continue
        bridge_cost = _sparse_slope_bridge_cost(
            nodes[first_index],
            nodes[second_index],
            grid_resolution=grid_resolution,
            max_slope_grade=max_slope_grade,
            max_surface_transition_height=max_surface_transition_height,
            obstacle_bins=obstacle_bins,
            obstacle_clearance=obstacle_clearance,
        )
        if bridge_cost is None:
            continue
        adjacency[first_index].append((second_index, bridge_cost))
        adjacency[second_index].append((first_index, bridge_cost))


def _adjacency_component_ids(
    adjacency: list[list[tuple[int, float]]],
) -> list[int]:
    component_ids = [-1] * len(adjacency)
    component_id = 0
    for start in range(len(adjacency)):
        if component_ids[start] >= 0:
            continue
        stack = [start]
        component_ids[start] = component_id
        while stack:
            current = stack.pop()
            for neighbor, _ in adjacency[current]:
                if component_ids[neighbor] >= 0:
                    continue
                component_ids[neighbor] = component_id
                stack.append(neighbor)
        component_id += 1
    return component_ids


def _sparse_slope_bridge_cost(
    node: SlamTerrainNode,
    other: SlamTerrainNode,
    *,
    grid_resolution: float,
    max_slope_grade: float,
    max_surface_transition_height: float,
    obstacle_bins: dict[tuple[int, int], tuple[float, float]],
    obstacle_clearance: float,
) -> Optional[float]:
    horizontal = math.hypot(other.x - node.x, other.y - node.y)
    if horizontal < grid_resolution * 0.85:
        return None
    dz = abs(other.z - node.z)
    grade = dz / max(horizontal, 1e-6)
    if grade > max_slope_grade:
        return None
    if not _sparse_bridge_uses_vertical_structure(
        node,
        other,
        min_vertical_change=max(0.06, max_surface_transition_height * 0.25),
    ):
        return None
    if _edge_near_obstacle_bins(
        node.x,
        node.y,
        other.x,
        other.y,
        obstacle_bins,
        grid_resolution,
        max(obstacle_clearance, grid_resolution * 0.60),
    ):
        return None
    surface_changed = other.label != node.label
    cost = math.sqrt(horizontal * horizontal + dz * dz) * (1.0 + grade * 2.2)
    if surface_changed:
        cost += grid_resolution * 1.5
    return cost


def _sparse_bridge_uses_vertical_structure(
    node: SlamTerrainNode,
    other: SlamTerrainNode,
    *,
    min_vertical_change: float,
) -> bool:
    if (
        (node.label == TraversabilityLabel.FLOOR or other.label == TraversabilityLabel.FLOOR)
        and abs(other.z - node.z) > max(min_vertical_change, 0.20)
    ):
        return False
    if abs(other.z - node.z) >= min_vertical_change:
        return True
    if min(node.z, other.z) < 0.45:
        return False
    non_floor_labels = {
        TraversabilityLabel.RAMP,
        TraversabilityLabel.STEP,
        TraversabilityLabel.PLATFORM,
    }
    return node.label in non_floor_labels and other.label in non_floor_labels


def _has_edge(edges: list[tuple[int, float]], target_index: int) -> bool:
    return any(index == target_index for index, _ in edges)


def _xy_near_obstacle_bins(
    x: float,
    y: float,
    obstacle_bins: dict[tuple[int, int], tuple[float, float]],
    grid_resolution: float,
    obstacle_clearance: float,
) -> bool:
    if not obstacle_bins or obstacle_clearance <= 0.0:
        return False
    bin_x, bin_y = _bin_key(x, y, grid_resolution)
    bin_radius = max(
        1,
        int(math.ceil(obstacle_clearance / max(grid_resolution, 1e-6))) + 1,
    )
    for bx in range(bin_x - bin_radius, bin_x + bin_radius + 1):
        for by in range(bin_y - bin_radius, bin_y + bin_radius + 1):
            obstacle_xy = obstacle_bins.get((bx, by))
            if obstacle_xy is None:
                continue
            if math.hypot(x - obstacle_xy[0], y - obstacle_xy[1]) <= obstacle_clearance:
                return True
    return False


def _edge_near_obstacle_bins(
    start_x: float,
    start_y: float,
    end_x: float,
    end_y: float,
    obstacle_bins: dict[tuple[int, int], tuple[float, float]],
    grid_resolution: float,
    obstacle_clearance: float,
) -> bool:
    if not obstacle_bins or obstacle_clearance <= 0.0:
        return False
    mid_x = (start_x + end_x) * 0.5
    mid_y = (start_y + end_y) * 0.5
    bin_x, bin_y = _bin_key(mid_x, mid_y, grid_resolution)
    segment_radius = math.hypot(end_x - start_x, end_y - start_y) * 0.5
    bin_radius = max(
        1,
        int(
            math.ceil(
                (obstacle_clearance + segment_radius)
                / max(grid_resolution, 1e-6)
            )
        )
        + 1,
    )
    for bx in range(bin_x - bin_radius, bin_x + bin_radius + 1):
        for by in range(bin_y - bin_radius, bin_y + bin_radius + 1):
            obstacle_xy = obstacle_bins.get((bx, by))
            if obstacle_xy is None:
                continue
            distance = _point_to_segment_distance(
                obstacle_xy[0],
                obstacle_xy[1],
                start_x,
                start_y,
                end_x,
                end_y,
            )
            if distance <= obstacle_clearance:
                return True
    return False


def _point_to_segment_distance(
    point_x: float,
    point_y: float,
    start_x: float,
    start_y: float,
    end_x: float,
    end_y: float,
) -> float:
    dx = end_x - start_x
    dy = end_y - start_y
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return math.hypot(point_x - start_x, point_y - start_y)
    t = ((point_x - start_x) * dx + (point_y - start_y) * dy) / length_sq
    t = max(0.0, min(1.0, t))
    closest_x = start_x + t * dx
    closest_y = start_y + t * dy
    return math.hypot(point_x - closest_x, point_y - closest_y)


def _slam_neighbor_radius(
    *,
    grid_resolution: float,
    max_slope_grade: float,
    max_step_height: float,
) -> float:
    step_limited_radius = max_step_height / max(max_slope_grade, 1e-6)
    return max(grid_resolution * 1.65, min(grid_resolution * 2.25, step_limited_radius))


def _slam_sparse_bridge_radius(
    *,
    grid_resolution: float,
    max_slope_grade: float,
    max_step_height: float,
) -> float:
    step_limited_radius = max_step_height / max(max_slope_grade, 1e-6)
    return max(
        grid_resolution * 3.25,
        min(grid_resolution * 4.5, step_limited_radius * 1.7),
    )


def _nearest_node(
    nodes: list[SlamTerrainNode],
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


def _heuristic(node: SlamTerrainNode, goal: SlamTerrainNode) -> float:
    return math.sqrt(
        (node.x - goal.x) ** 2
        + (node.y - goal.y) ** 2
        + (node.z - goal.z) ** 2
    )


def _bin_key(x: float, y: float, resolution: float) -> tuple[int, int]:
    return int(round(x / resolution)), int(round(y / resolution))
