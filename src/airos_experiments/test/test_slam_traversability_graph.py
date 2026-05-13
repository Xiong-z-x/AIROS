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
