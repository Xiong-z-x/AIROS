from __future__ import annotations

import struct
from pathlib import Path

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from airos_experiments.sdf_geometry import sample_world_cloud
from airos_experiments.slam_traversability_graph import (
    TraversabilityLabel,
    build_slam_graph_from_points,
    build_slam_graph_from_pointcloud,
    plan_slam_graph_path,
)
from airos_experiments.terrain_pct_planner import (
    TerrainGraph,
    TerrainNode,
    build_slam_terrain_graph_from_pointcloud,
    should_hold_active_frontier_path,
    should_release_stalled_frontier_path,
    should_keep_pending_slam_goal,
    plan_slam_frontier_path,
    plan_terrain_path,
)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _large_world() -> Path:
    return _repo_root() / 'src/airos_sim/worlds/large_multilevel_complex_static.sdf'


def _xyz_pointcloud(points: list[tuple[float, float, float]]) -> PointCloud2:
    msg = PointCloud2()
    msg.header = Header(frame_id='map')
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = b''.join(struct.pack('<fff', *point) for point in points)
    return msg


def _floor_ramp_platform_points() -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for x in (0.0, 0.25, 0.50):
        for y in (-0.12, 0.0, 0.12):
            points.append((x, y, 0.0))
    for x, z in (
        (0.75, 0.10),
        (1.00, 0.22),
        (1.25, 0.34),
        (1.50, 0.48),
        (1.75, 0.64),
        (2.00, 0.82),
        (2.25, 1.00),
    ):
        for y in (-0.18, -0.06, 0.06, 0.18):
            points.append((x, y, z))
    for x in (2.50, 2.75, 3.00):
        for y in (-0.12, 0.0, 0.12):
            points.append((x, y, 1.00))
    return points


def test_slam_graph_routes_floor_to_platform_through_ramp() -> None:
    graph = build_slam_graph_from_pointcloud(
        _xyz_pointcloud(_floor_ramp_platform_points()),
        grid_resolution=0.25,
        min_cell_points=1,
        vertical_layer_gap=0.10,
        max_slope_grade=0.75,
        max_step_height=0.34,
    )
    path = plan_slam_graph_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(2.75, 0.0),
        start_z=0.0,
        goal_z_policy='highest',
    )
    labels = {node.label for node in path}

    assert path
    assert path[0].z <= 0.05
    assert path[-1].z >= 0.95
    assert TraversabilityLabel.FLOOR in labels
    assert TraversabilityLabel.RAMP in labels
    assert TraversabilityLabel.PLATFORM in labels


def test_slam_graph_bridges_sparse_ramp_samples_with_safe_grade() -> None:
    points: list[tuple[float, float, float]] = []
    for x, z in (
        (0.0, 0.0),
        (0.6, 0.24),
        (1.2, 0.48),
        (1.8, 0.72),
        (2.4, 0.96),
    ):
        for y in (-0.06, 0.06):
            points.append((x, y, z))

    graph = build_slam_graph_from_pointcloud(
        _xyz_pointcloud(points),
        grid_resolution=0.30,
        min_cell_points=2,
        vertical_layer_gap=0.10,
        max_slope_grade=0.58,
        max_step_height=0.36,
        max_surface_transition_height=0.12,
    )
    path = plan_slam_graph_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(2.4, 0.0),
        start_z=0.0,
        goal_z_policy='highest',
    )

    assert path
    assert path[0].z <= 0.05
    assert path[-1].z >= 0.90


def test_slam_graph_rejects_direct_platform_edge_drop() -> None:
    points: list[tuple[float, float, float]] = []
    for x in (0.0, 0.25, 0.50):
        for y in (-0.10, 0.0, 0.10):
            points.append((x, y, 1.00))
    for y in (-0.10, 0.0, 0.10):
        points.append((0.50, y, 0.00))

    graph = build_slam_graph_from_pointcloud(
        _xyz_pointcloud(points),
        grid_resolution=0.25,
        min_cell_points=1,
        vertical_layer_gap=0.10,
        max_slope_grade=0.75,
        max_step_height=0.34,
    )
    path = plan_slam_graph_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(0.50, 0.0),
        start_z=1.0,
        goal_z_policy='lowest',
    )

    assert path == []


def test_slam_graph_routes_large_world_spawn_to_third_level() -> None:
    points = [
        (x, y, z)
        for x, y, z, _ in sample_world_cloud(
            _large_world(),
            spacing=0.16,
            include_dynamic=False,
        )
    ]
    graph = build_slam_graph_from_points(
        points,
        grid_resolution=0.25,
        max_slope_grade=0.58,
        max_step_height=0.36,
        max_surface_transition_height=0.12,
        min_cell_points=2,
        vertical_layer_gap=0.18,
    )
    path = plan_slam_graph_path(
        graph,
        start_xy=(0.0, -10.0),
        goal_xy=(6.0, 13.0),
        start_z=0.0,
        goal_z_policy='highest',
    )
    labels = {node.label for node in path}

    assert len(graph.nodes) > 20000
    assert path
    assert path[0].z < 0.10
    assert path[-1].z > 2.0
    assert max(node.z for node in path) > 2.0
    assert TraversabilityLabel.FLOOR in labels
    assert TraversabilityLabel.STEP in labels


def test_slam_graph_routes_sparse_same_level_floor_from_spawn() -> None:
    points = [
        (x, y, z)
        for x, y, z, _ in sample_world_cloud(
            _large_world(),
            spacing=0.16,
            include_dynamic=False,
        )
    ]
    graph = build_slam_graph_from_points(
        points,
        grid_resolution=0.25,
        max_slope_grade=0.58,
        max_step_height=0.36,
        max_surface_transition_height=0.12,
        min_cell_points=2,
        vertical_layer_gap=0.18,
    )
    path = plan_slam_graph_path(
        graph,
        start_xy=(0.0, -10.0),
        goal_xy=(2.0, -8.0),
        start_z=0.0,
        goal_z_policy='highest',
    )

    assert path
    assert path[0].z < 0.10
    assert path[-1].z < 0.10


def test_terrain_planner_adaptive_goal_policy_falls_back_to_reachable_floor() -> None:
    points: list[tuple[float, float, float]] = []
    for x in (0.0, 0.25, 0.50, 0.75, 1.00):
        for y in (-0.10, 0.0, 0.10):
            points.append((x, y, 0.0))
    for y in (-0.10, 0.0, 0.10):
        points.append((1.00, y, 1.70))

    graph = build_slam_terrain_graph_from_pointcloud(
        _xyz_pointcloud(points),
        grid_resolution=0.25,
        min_cell_points=1,
        vertical_layer_gap=0.10,
        max_slope_grade=0.75,
        max_step_height=0.34,
    )
    highest_path = plan_terrain_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(1.0, 0.0),
        start_z=0.0,
        goal_z_policy='highest',
    )
    adaptive_path = plan_terrain_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(1.0, 0.0),
        start_z=0.0,
        goal_z_policy='adaptive',
    )

    assert highest_path == []
    assert adaptive_path
    assert adaptive_path[-1].z < 0.10


def test_terrain_planner_rejects_goal_outside_slam_map_coverage() -> None:
    points: list[tuple[float, float, float]] = []
    for x in (0.0, 0.25, 0.50, 0.75, 1.00):
        for y in (-0.10, 0.0, 0.10):
            points.append((x, y, 0.0))

    graph = build_slam_terrain_graph_from_pointcloud(
        _xyz_pointcloud(points),
        grid_resolution=0.25,
        min_cell_points=1,
        vertical_layer_gap=0.10,
        max_slope_grade=0.75,
        max_step_height=0.34,
    )
    path = plan_terrain_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(4.0, 0.0),
        start_z=0.0,
        goal_z_policy='adaptive',
        max_goal_xy_distance=1.0,
    )

    assert path == []


def test_terrain_planner_rejects_low_goal_when_high_goal_required() -> None:
    nodes = [
        TerrainNode(0, 0.0, 0.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(1, 1.0, 0.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(2, 2.0, 0.0, 0.0, 'slam_floor', 1.0),
    ]
    graph = TerrainGraph(
        nodes=nodes,
        adjacency=[
            [(1, 1.0)],
            [(0, 1.0), (2, 1.0)],
            [(1, 1.0)],
        ],
        terrain_cloud=[],
    )

    path = plan_terrain_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(2.0, 0.0),
        start_z=0.0,
        goal_z_policy='highest',
        goal_min_z=1.2,
    )

    assert path == []


def test_terrain_planner_accepts_high_goal_when_required_layer_is_reachable() -> None:
    nodes = [
        TerrainNode(0, 0.0, 0.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(1, 1.0, 0.0, 0.4, 'slam_ramp', 1.0),
        TerrainNode(2, 2.0, 0.0, 0.8, 'slam_ramp', 1.0),
        TerrainNode(3, 3.0, 0.0, 1.2, 'slam_deck', 1.0),
    ]
    graph = TerrainGraph(
        nodes=nodes,
        adjacency=[
            [(1, 1.0)],
            [(0, 1.0), (2, 1.0)],
            [(1, 1.0), (3, 1.0)],
            [(2, 1.0)],
        ],
        terrain_cloud=[],
    )

    path = plan_terrain_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(3.0, 0.0),
        start_z=0.0,
        goal_z_policy='highest',
        goal_min_z=1.0,
    )

    assert [node.index for node in path] == [0, 1, 2, 3]


def test_empty_slam_graph_failure_keeps_goal_pending_for_rebuild() -> None:
    graph = TerrainGraph(nodes=[], adjacency=[], terrain_cloud=[])

    assert should_keep_pending_slam_goal(
        graph,
        terrain_map_source='slam_cloud',
        frontier_replan_enabled=True,
    )


def test_sdf_terrain_failure_does_not_keep_goal_pending() -> None:
    graph = TerrainGraph(nodes=[], adjacency=[], terrain_cloud=[])

    assert not should_keep_pending_slam_goal(
        graph,
        terrain_map_source='sdf',
        frontier_replan_enabled=True,
    )


def test_slam_frontier_path_stays_in_reachable_component_toward_goal() -> None:
    nodes = [
        TerrainNode(0, 0.0, 0.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(1, 1.0, 0.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(2, 2.0, 0.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(3, 8.0, 0.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(4, 9.0, 0.0, 0.0, 'slam_floor', 1.0),
    ]
    graph = TerrainGraph(
        nodes=nodes,
        adjacency=[
            [(1, 1.0)],
            [(0, 1.0), (2, 1.0)],
            [(1, 1.0)],
            [(4, 1.0)],
            [(3, 1.0)],
        ],
        terrain_cloud=[],
    )

    final_path = plan_terrain_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(9.0, 0.0),
        start_z=0.0,
        goal_z_policy='adaptive',
        max_goal_xy_distance=1.0,
    )
    frontier_path = plan_slam_frontier_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(9.0, 0.0),
        start_z=0.0,
        min_path_distance=1.0,
    )

    assert final_path == []
    assert [node.index for node in frontier_path] == [0, 1, 2]


def test_slam_frontier_path_requires_progress_from_start() -> None:
    nodes = [
        TerrainNode(0, 0.0, 0.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(1, 0.2, 0.0, 0.0, 'slam_floor', 1.0),
    ]
    graph = TerrainGraph(
        nodes=nodes,
        adjacency=[[(1, 0.2)], [(0, 0.2)]],
        terrain_cloud=[],
    )

    path = plan_slam_frontier_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(10.0, 0.0),
        start_z=0.0,
        min_path_distance=1.0,
    )

    assert path == []


def test_slam_frontier_path_ignores_isolated_start_outlier() -> None:
    nodes = [
        TerrainNode(0, 0.0, 0.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(1, 0.25, 0.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(2, 0.75, 0.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(3, 1.25, 0.0, 0.0, 'slam_floor', 1.0),
    ]
    graph = TerrainGraph(
        nodes=nodes,
        adjacency=[
            [],
            [(2, 0.5)],
            [(1, 0.5), (3, 0.5)],
            [(2, 0.5)],
        ],
        terrain_cloud=[],
    )

    path = plan_slam_frontier_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(5.0, 0.0),
        start_z=0.0,
        min_path_distance=0.25,
        max_path_distance=1.0,
    )

    assert [node.index for node in path] == [1, 2, 3]


def test_terrain_path_uses_stable_start_component_over_near_artifact() -> None:
    artifact_nodes = [
        TerrainNode(0, 0.0, 0.0, -0.05, 'slam_floor', 1.0),
        TerrainNode(1, 0.2, 0.0, -0.05, 'slam_floor', 1.0),
        TerrainNode(2, 0.4, 0.0, -0.05, 'slam_floor', 1.0),
        TerrainNode(3, -0.2, 0.0, -0.05, 'slam_floor', 1.0),
        TerrainNode(4, -0.4, 0.0, -0.05, 'slam_floor', 1.0),
        TerrainNode(5, -0.6, 0.0, -0.05, 'slam_floor', 1.0),
        TerrainNode(6, -0.8, 0.0, -0.05, 'slam_floor', 1.0),
    ]
    stable_nodes = [
        TerrainNode(index, 1.2 + (index - 7) * 0.25, 0.0, -0.30, 'slam_floor', 1.0)
        for index in range(7, 32)
    ]
    nodes = artifact_nodes + stable_nodes
    adjacency: list[list[tuple[int, float]]] = [[] for _ in nodes]
    adjacency[0].append((1, 0.2))
    adjacency[1].extend([(0, 0.2), (2, 0.2)])
    adjacency[2].append((1, 0.2))
    adjacency[0].append((3, 0.2))
    adjacency[3].extend([(0, 0.2), (4, 0.2)])
    adjacency[4].extend([(3, 0.2), (5, 0.2)])
    adjacency[5].extend([(4, 0.2), (6, 0.2)])
    adjacency[6].append((5, 0.2))
    for index in range(7, 31):
        adjacency[index].append((index + 1, 0.25))
        adjacency[index + 1].append((index, 0.25))
    graph = TerrainGraph(nodes=nodes, adjacency=adjacency, terrain_cloud=[])

    path = plan_terrain_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(7.0, 0.0),
        start_z=0.0,
        goal_z_policy='nearest_z',
        max_goal_xy_distance=1.0,
    )

    assert path
    assert path[0].index == 7
    assert path[-1].index == 30


def test_slam_frontier_path_uses_stable_start_component_over_near_artifact() -> None:
    artifact_nodes = [
        TerrainNode(0, 0.0, 0.0, -0.05, 'slam_floor', 1.0),
        TerrainNode(1, 0.2, 0.0, -0.05, 'slam_floor', 1.0),
        TerrainNode(2, 0.4, 0.0, -0.05, 'slam_floor', 1.0),
        TerrainNode(3, -0.2, 0.0, -0.05, 'slam_floor', 1.0),
        TerrainNode(4, -0.4, 0.0, -0.05, 'slam_floor', 1.0),
        TerrainNode(5, -0.6, 0.0, -0.05, 'slam_floor', 1.0),
        TerrainNode(6, -0.8, 0.0, -0.05, 'slam_floor', 1.0),
    ]
    stable_nodes = [
        TerrainNode(index, 1.2 + (index - 7) * 0.25, 0.0, -0.30, 'slam_floor', 1.0)
        for index in range(7, 32)
    ]
    nodes = artifact_nodes + stable_nodes
    adjacency: list[list[tuple[int, float]]] = [[] for _ in nodes]
    adjacency[0].append((1, 0.2))
    adjacency[1].extend([(0, 0.2), (2, 0.2)])
    adjacency[2].append((1, 0.2))
    adjacency[0].append((3, 0.2))
    adjacency[3].extend([(0, 0.2), (4, 0.2)])
    adjacency[4].extend([(3, 0.2), (5, 0.2)])
    adjacency[5].extend([(4, 0.2), (6, 0.2)])
    adjacency[6].append((5, 0.2))
    for index in range(7, 31):
        adjacency[index].append((index + 1, 0.25))
        adjacency[index + 1].append((index, 0.25))
    graph = TerrainGraph(nodes=nodes, adjacency=adjacency, terrain_cloud=[])

    path = plan_slam_frontier_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(7.0, 0.0),
        start_z=0.0,
        min_path_distance=0.25,
        max_path_distance=1.0,
    )

    assert path
    assert path[0].index == 7


def test_slam_frontier_path_limits_exploration_to_short_rolling_step() -> None:
    nodes = [
        TerrainNode(index, float(index), 0.0, 0.0, 'slam_floor', 1.0)
        for index in range(9)
    ]
    adjacency: list[list[tuple[int, float]]] = [[] for _ in nodes]
    for index in range(len(nodes) - 1):
        adjacency[index].append((index + 1, 1.0))
        adjacency[index + 1].append((index, 1.0))
    graph = TerrainGraph(nodes=nodes, adjacency=adjacency, terrain_cloud=[])

    path = plan_slam_frontier_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(20.0, 0.0),
        start_z=0.0,
        min_path_distance=1.0,
        max_path_distance=2.5,
    )

    assert [node.index for node in path] == [0, 1, 2]


def test_slam_frontier_path_prefers_forward_progress_over_goal_distance() -> None:
    nodes = [
        TerrainNode(0, 0.0, 0.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(1, 0.0, 1.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(2, 1.0, 0.1, 0.0, 'slam_floor', 1.0),
        TerrainNode(3, 2.0, 0.2, 0.0, 'slam_floor', 1.0),
    ]
    graph = TerrainGraph(
        nodes=nodes,
        adjacency=[
            [(1, 1.0), (2, 1.0)],
            [(0, 1.0)],
            [(0, 1.0), (3, 1.0)],
            [(2, 1.0)],
        ],
        terrain_cloud=[],
    )

    path = plan_slam_frontier_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(10.0, 2.0),
        start_z=0.0,
        min_path_distance=1.0,
        max_path_distance=2.0,
    )

    assert [node.index for node in path] == [0, 2, 3]


def test_slam_frontier_path_prefers_vertical_progress_for_high_goal() -> None:
    nodes = [
        TerrainNode(0, 0.0, 0.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(1, 1.0, 0.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(2, 2.0, 0.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(3, 0.0, -1.0, 0.35, 'slam_ramp', 1.0),
        TerrainNode(4, 0.0, -2.0, 0.70, 'slam_ramp', 1.0),
        TerrainNode(5, 2.0, 0.0, 2.0, 'slam_deck', 1.0),
    ]
    graph = TerrainGraph(
        nodes=nodes,
        adjacency=[
            [(1, 1.0), (3, 1.1)],
            [(0, 1.0), (2, 1.0)],
            [(1, 1.0)],
            [(0, 1.1), (4, 1.1)],
            [(3, 1.1)],
            [],
        ],
        terrain_cloud=[],
    )

    path = plan_slam_frontier_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(2.0, 0.0),
        start_z=0.0,
        min_path_distance=1.0,
        max_path_distance=3.0,
        target_z=2.0,
    )

    assert [node.index for node in path] == [0, 3, 4]


def test_slam_frontier_path_ignores_small_height_noise_for_high_goal() -> None:
    nodes = [
        TerrainNode(0, 0.0, 0.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(1, 0.5, -0.2, 0.19, 'slam_floor', 1.0),
        TerrainNode(2, 2.0, 0.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(3, 4.0, 0.0, 0.0, 'slam_floor', 1.0),
    ]
    graph = TerrainGraph(
        nodes=nodes,
        adjacency=[
            [(1, 0.6), (2, 2.0)],
            [(0, 0.6)],
            [(0, 2.0), (3, 2.0)],
            [(2, 2.0)],
        ],
        terrain_cloud=[],
    )

    path = plan_slam_frontier_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(8.0, 0.0),
        start_z=0.0,
        min_path_distance=0.25,
        max_path_distance=5.0,
        target_z=1.6,
    )

    assert [node.index for node in path] == [0, 2, 3]


def test_active_frontier_path_is_held_until_reached() -> None:
    path = [
        TerrainNode(0, 0.0, 0.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(1, 2.0, 0.0, 0.0, 'slam_floor', 1.0),
    ]

    assert should_hold_active_frontier_path(
        active_path=path,
        current_xy=(0.25, 0.0),
        goal_tolerance=0.35,
    )


def test_active_frontier_path_releases_after_goal_is_reached() -> None:
    path = [
        TerrainNode(0, 0.0, 0.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(1, 2.0, 0.0, 0.0, 'slam_floor', 1.0),
    ]

    assert not should_hold_active_frontier_path(
        active_path=path,
        current_xy=(1.9, 0.0),
        goal_tolerance=0.35,
    )


def test_active_frontier_path_releases_when_final_goal_changes() -> None:
    path = [
        TerrainNode(0, 0.0, 0.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(1, 2.0, 0.0, 0.0, 'slam_floor', 1.0),
    ]

    assert not should_hold_active_frontier_path(
        active_path=path,
        current_xy=(0.25, 0.0),
        goal_tolerance=0.35,
        active_final_goal_xy=(6.0, 13.0),
        final_goal_xy=(-6.0, 13.0),
        final_goal_tolerance=0.05,
    )


def test_inactive_frontier_execution_does_not_hold_stale_path() -> None:
    path = [
        TerrainNode(0, 0.0, 0.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(1, 2.0, 0.0, 0.0, 'slam_floor', 1.0),
    ]

    assert not should_hold_active_frontier_path(
        active_path=path,
        current_xy=(0.25, 0.0),
        goal_tolerance=0.35,
        execution_active=False,
    )


def test_stalled_frontier_releases_after_commanded_motion_without_odom_progress() -> None:
    assert should_release_stalled_frontier_path(
        active_path=[
            TerrainNode(0, 0.0, 0.0, 0.0, 'slam_floor', 1.0),
            TerrainNode(1, 2.0, 0.0, 0.0, 'slam_floor', 1.0),
        ],
        commanded_motion=True,
        current_xy=(0.02, 0.01),
        monitor_start_xy=(0.0, 0.0),
        elapsed_sec=9.0,
        min_progress=0.20,
        timeout_sec=8.0,
    )


def test_stalled_frontier_keeps_active_path_when_odom_progresses() -> None:
    assert not should_release_stalled_frontier_path(
        active_path=[
            TerrainNode(0, 0.0, 0.0, 0.0, 'slam_floor', 1.0),
            TerrainNode(1, 2.0, 0.0, 0.0, 'slam_floor', 1.0),
        ],
        commanded_motion=True,
        current_xy=(0.35, 0.01),
        monitor_start_xy=(0.0, 0.0),
        elapsed_sec=9.0,
        min_progress=0.20,
        timeout_sec=8.0,
    )


def test_slam_frontier_path_uses_live_obstacle_points_to_avoid_wall_gap() -> None:
    nodes = [
        TerrainNode(0, 0.0, -1.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(1, 0.0, 0.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(2, 0.0, 1.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(3, -0.7, -0.5, 0.0, 'slam_floor', 1.0),
        TerrainNode(4, -0.7, 0.5, 0.0, 'slam_floor', 1.0),
    ]
    graph = TerrainGraph(
        nodes=nodes,
        adjacency=[
            [(1, 1.0), (3, 0.9)],
            [(0, 1.0), (2, 1.0)],
            [(1, 1.0), (4, 0.9)],
            [(0, 0.9), (4, 1.0)],
            [(3, 1.0), (2, 0.9)],
        ],
        terrain_cloud=[],
    )

    path = plan_slam_frontier_path(
        graph,
        start_xy=(0.0, -1.0),
        goal_xy=(0.0, 2.0),
        start_z=0.0,
        min_path_distance=1.0,
        max_path_distance=3.0,
        blocked_points=[(0.0, 0.0)],
        obstacle_clearance=0.35,
    )

    assert [node.index for node in path] == [0, 3, 4, 2]


def test_slam_frontier_path_avoids_recently_failed_frontier_region() -> None:
    nodes = [
        TerrainNode(0, 0.0, -10.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(1, 1.0, -9.8, 0.0, 'slam_floor', 1.0),
        TerrainNode(2, 2.0, -9.6, 0.0, 'slam_floor', 1.0),
        TerrainNode(3, -1.0, -8.8, 0.0, 'slam_floor', 1.0),
        TerrainNode(4, -2.0, -7.8, 0.0, 'slam_floor', 1.0),
    ]
    graph = TerrainGraph(
        nodes=nodes,
        adjacency=[
            [(1, 1.0), (3, 1.4)],
            [(0, 1.0), (2, 1.0)],
            [(1, 1.0)],
            [(0, 1.4), (4, 1.4)],
            [(3, 1.4)],
        ],
        terrain_cloud=[],
    )

    path = plan_slam_frontier_path(
        graph,
        start_xy=(0.0, -10.0),
        goal_xy=(6.0, 13.0),
        start_z=0.0,
        min_path_distance=0.5,
        max_path_distance=3.0,
        avoid_points=[(1.8, -9.6)],
        avoid_clearance=1.4,
    )

    assert [node.index for node in path] == [0, 3, 4]


def test_slam_frontier_path_tracks_mapped_high_attractor_for_high_goal() -> None:
    nodes = [
        TerrainNode(0, 0.0, -10.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(1, 2.0, -9.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(2, -2.0, -9.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(3, -4.0, -8.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(4, -4.0, -4.0, 1.7, 'slam_deck', 1.0),
    ]
    graph = TerrainGraph(
        nodes=nodes,
        adjacency=[
            [(1, 2.2), (2, 2.2)],
            [(0, 2.2)],
            [(0, 2.2), (3, 2.2)],
            [(2, 2.2)],
            [],
        ],
        terrain_cloud=[],
    )

    path = plan_slam_frontier_path(
        graph,
        start_xy=(0.0, -10.0),
        goal_xy=(6.0, 13.0),
        start_z=0.0,
        min_path_distance=1.0,
        max_path_distance=5.0,
        target_z=1.6,
    )

    assert [node.index for node in path] == [0, 2, 3]


def test_slam_frontier_path_rejects_reverse_low_bump_for_high_goal() -> None:
    nodes = [
        TerrainNode(0, -4.8, -10.6, 0.0, 'slam_floor', 1.0),
        TerrainNode(1, -6.0, -10.1, 0.50, 'slam_step', 1.0),
        TerrainNode(2, -3.7, -8.8, 0.0, 'slam_floor', 1.0),
        TerrainNode(3, -2.8, -7.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(4, 5.2, 10.8, 2.0, 'slam_deck', 1.0),
    ]
    graph = TerrainGraph(
        nodes=nodes,
        adjacency=[
            [(1, 1.4), (2, 2.2)],
            [(0, 1.4)],
            [(0, 2.2), (3, 2.0)],
            [(2, 2.0)],
            [],
        ],
        terrain_cloud=[],
    )

    path = plan_slam_frontier_path(
        graph,
        start_xy=(-4.8, -10.6),
        goal_xy=(6.0, 13.0),
        start_z=0.0,
        min_path_distance=0.25,
        max_path_distance=10.0,
        target_z=1.6,
    )

    assert [node.index for node in path] == [0, 2, 3]


def test_slam_graph_routes_around_vertical_obstacle_cells() -> None:
    points: list[tuple[float, float, float]] = []
    for x in (0.0, 0.25, 0.50, 0.75, 1.00):
        points.append((x, 0.0, 0.0))
    for z in (0.20, 0.45, 0.70, 0.95, 1.20):
        for dx in (-0.03, 0.0, 0.03):
            for dy in (-0.03, 0.0, 0.03):
                points.append((0.50 + dx, dy, z))

    graph = build_slam_graph_from_pointcloud(
        _xyz_pointcloud(points),
        grid_resolution=0.25,
        min_cell_points=1,
        vertical_layer_gap=0.10,
        max_slope_grade=0.75,
        max_step_height=0.34,
    )
    path = plan_slam_graph_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(1.0, 0.0),
        start_z=0.0,
        goal_z_policy='nearest_z',
    )

    assert path == []


def test_slam_graph_rejects_sparse_edges_through_wall_base_cell() -> None:
    points: list[tuple[float, float, float]] = [
        (-0.05, -0.25, 0.0),
        (0.05, -0.25, 0.0),
        (-0.05, 0.25, 0.0),
        (0.05, 0.25, 0.0),
        (-0.05, 0.0, 0.0),
        (0.05, 0.0, 0.0),
    ]
    for z in (0.42, 0.50, 0.58):
        for x in (-0.04, 0.04):
            points.append((x, 0.0, z))

    graph = build_slam_graph_from_pointcloud(
        _xyz_pointcloud(points),
        grid_resolution=0.25,
        min_cell_points=1,
        vertical_layer_gap=0.18,
        max_slope_grade=0.75,
        max_step_height=0.34,
    )
    path = plan_slam_graph_path(
        graph,
        start_xy=(0.0, -0.25),
        goal_xy=(0.0, 0.25),
        start_z=0.0,
        goal_z_policy='nearest_z',
    )

    assert path == []
