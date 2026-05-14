from __future__ import annotations

import math
import struct
from pathlib import Path

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from airos_experiments.sdf_geometry import (
    BoxCollision,
    inverse_transform_point,
    load_collision_geometries,
    sample_world_cloud,
)
from airos_experiments.terrain_pct_planner import (
    TerrainNode,
    _direct_linear_speed,
    _goal_is_active,
    _path_speed_limit,
    _surface_height_at_xy,
    _surface_speed_limit_for_label,
    _surface_z_reference,
    _waypoint_path,
    _waypoints_after_start_clearance,
    build_slam_terrain_graph_from_pointcloud,
    build_slam_terrain_graph_from_points,
    build_terrain_graph,
    direct_tracking_start_clearance,
    plan_terrain_path,
)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _realistic_world() -> Path:
    return _repo_root() / 'src/airos_sim/worlds/realistic_multilevel_ramp.sdf'


def _large_world() -> Path:
    return _repo_root() / 'src/airos_sim/worlds/large_multilevel_complex_static.sdf'


def _box_by_model(world_file: Path, model_name: str) -> BoxCollision:
    for geometry in load_collision_geometries(world_file):
        if isinstance(geometry, BoxCollision) and geometry.model_name == model_name:
            return geometry
    raise AssertionError(f'missing box model {model_name}')


def _floor_ramp_deck_points() -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for x in (-0.24, 0.0, 0.24):
        for y in (-0.12, 0.0, 0.12):
            points.append((x, y, 0.0))
    for x, z in (
        (0.50, 0.05),
        (0.75, 0.14),
        (1.00, 0.24),
        (1.25, 0.34),
        (1.50, 0.48),
        (1.75, 0.66),
        (2.00, 0.84),
        (2.25, 1.00),
    ):
        for y in (-0.18, -0.06, 0.06, 0.18):
            points.append((x, y, z))
    for x in (2.50, 2.75, 3.00):
        for y in (-0.12, 0.0, 0.12):
            points.append((x, y, 1.00))
    return points


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


def test_slam_terrain_graph_from_points_supports_floor_ramp_and_deck() -> None:
    points = _floor_ramp_deck_points()

    graph = build_slam_terrain_graph_from_points(
        points,
        grid_resolution=0.25,
        max_slope_grade=0.60,
        max_step_height=0.34,
        max_surface_transition_height=0.20,
        min_cell_points=1,
        vertical_layer_gap=0.10,
    )
    labels = {node.surface_label for node in graph.nodes}
    path = plan_terrain_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(2.75, 0.0),
        start_z=0.0,
        goal_z_policy='highest',
    )

    assert graph.nodes
    assert path
    assert any('floor' in label for label in labels)
    assert any('ramp' in label for label in labels)
    assert any('deck' in label for label in labels)
    assert path[0].z <= 0.05
    assert path[-1].z >= 0.95
    assert any('ramp' in node.surface_label for node in path)


def test_slam_terrain_graph_from_pointcloud_accepts_laser_map_xyz_fields() -> None:
    graph = build_slam_terrain_graph_from_pointcloud(
        _xyz_pointcloud(_floor_ramp_deck_points()),
        grid_resolution=0.25,
        max_slope_grade=0.60,
        max_step_height=0.34,
        max_surface_transition_height=0.20,
        min_cell_points=1,
        vertical_layer_gap=0.10,
        max_points=1000,
    )
    path = plan_terrain_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(2.75, 0.0),
        start_z=0.0,
        goal_z_policy='highest',
    )

    assert path
    assert path[0].z <= 0.05
    assert path[-1].z >= 0.95
    assert any('ramp' in node.surface_label for node in path)


def test_slam_terrain_graph_allows_step_sized_slam_surface_transitions() -> None:
    points: list[tuple[float, float, float]] = []
    for x, z in (
        (0.00, 0.00),
        (0.25, 0.00),
        (0.50, 0.30),
        (0.75, 0.60),
        (1.00, 0.90),
        (1.25, 1.20),
    ):
        for y in (-0.10, 0.0, 0.10):
            points.append((x, y, z))

    graph = build_slam_terrain_graph_from_points(
        points,
        grid_resolution=0.25,
        max_slope_grade=1.25,
        max_step_height=0.36,
        max_surface_transition_height=0.12,
        min_cell_points=1,
        vertical_layer_gap=0.10,
    )
    path = plan_terrain_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(1.25, 0.0),
        start_z=0.0,
        goal_z_policy='highest',
    )

    assert path
    assert path[-1].z >= 1.15
    assert any(
        'ramp' in node.surface_label or 'step' in node.surface_label
        for node in path
    )


def test_sdf_surface_cloud_includes_ramp_and_mezzanine_deck() -> None:
    points = sample_world_cloud(_realistic_world(), spacing=0.25)

    ramp_points = [
        point
        for point in points
        if -2.7 <= point[0] <= 0.7
        and -3.6 <= point[1] <= 3.6
        and point[3] >= 130.0
    ]
    deck_points = [
        point
        for point in points
        if -2.4 <= point[0] <= 6.0
        and 2.1 <= point[1] <= 7.5
        and 0.60 <= point[2] <= 0.72
        and point[3] >= 120.0
    ]

    assert len(points) > 20000
    assert len(ramp_points) > 300
    assert max(point[2] for point in ramp_points) - min(
        point[2] for point in ramp_points
    ) > 0.55
    assert min(point[2] for point in ramp_points) >= 0.0
    assert len(deck_points) > 250


def test_large_multilevel_world_has_dense_3d_structure_and_vertical_connectors() -> None:
    points = sample_world_cloud(_large_world(), spacing=0.20)

    high_points = [point for point in points if point[2] > 1.75]
    stair_points = [point for point in points if point[3] >= 145.0]
    ramp_points = [point for point in points if point[3] >= 130.0]
    obstacle_points = [
        point
        for point in points
        if 85.0 <= point[3] <= 100.0 and point[2] > 0.25
    ]

    assert len(points) > 50000
    assert high_points
    assert max(point[2] for point in points) > 2.2
    assert len(stair_points) > 400
    assert len(ramp_points) > 900
    assert len(obstacle_points) > 8000


def test_pct_routes_from_floor_to_third_level_through_stairs_and_ramps() -> None:
    graph = build_terrain_graph(
        _large_world(),
        grid_resolution=0.25,
        max_slope_grade=0.62,
        max_step_height=0.20,
        max_surface_transition_height=0.14,
    )
    path = plan_terrain_path(
        graph,
        start_xy=(0.0, -10.0),
        goal_xy=(6.0, 13.0),
        start_z=0.0,
        goal_z_policy='highest',
    )
    labels = {node.surface_label for node in path}

    assert len(graph.nodes) > 6000
    assert path
    assert path[0].z < 0.10
    assert path[-1].z > 2.0
    assert max(node.z for node in path) > 2.0
    assert any('lower_access_ramp' in label for label in labels)
    assert any('stair' in label or 'step' in label for label in labels)
    assert any('third_floor_deck' in label for label in labels)

    for first, second in zip(path, path[1:]):
        horizontal = math.hypot(second.x - first.x, second.y - first.y)
        dz = abs(second.z - first.z)
        assert dz <= 0.20
        assert dz / max(horizontal, 1e-6) <= 0.62


def test_large_world_ramp_transitions_use_center_entry_corridor() -> None:
    ramp_box = _box_by_model(_large_world(), 'lower_access_ramp')
    half_width = ramp_box.size[1] / 2.0
    graph = build_terrain_graph(
        _large_world(),
        grid_resolution=0.25,
        max_slope_grade=0.62,
        max_step_height=0.36,
        max_surface_transition_height=0.14,
    )
    path = plan_terrain_path(
        graph,
        start_xy=(0.0, -10.0),
        goal_xy=(6.0, 13.0),
        start_z=0.0,
        goal_z_policy='highest',
    )
    ramp_transitions = [
        (first, second)
        for first, second in zip(path, path[1:])
        if first.surface_label != second.surface_label
        and (
            'lower_access_ramp' in first.surface_label
            or 'lower_access_ramp' in second.surface_label
        )
    ]

    assert ramp_transitions
    for first, second in ramp_transitions:
        ramp_node = (
            first if 'lower_access_ramp' in first.surface_label else second
        )
        local = inverse_transform_point(
            ramp_box.transform,
            (ramp_node.x, ramp_node.y, ramp_node.z),
        )
        assert abs(local[1]) <= half_width * 0.35
        assert abs(ramp_node.surface_local_y) <= ramp_node.surface_half_y * 0.35


def test_visual_cloud_omits_vertical_sides_of_traversable_decks() -> None:
    deck_box = _box_by_model(_large_world(), 'second_floor_deck')
    points = sample_world_cloud(_large_world(), spacing=0.20)
    side_like_deck_points = []
    for point in points:
        if point[3] != 120.0:
            continue
        local = inverse_transform_point(deck_box.transform, point[:3])
        inside_xy = (
            abs(local[0]) <= deck_box.size[0] / 2.0 + 1e-6
            and abs(local[1]) <= deck_box.size[1] / 2.0 + 1e-6
        )
        below_top = local[2] < deck_box.size[2] / 2.0 - 0.02
        above_bottom = local[2] > -deck_box.size[2] / 2.0 - 1e-6
        if inside_xy and below_top and above_bottom:
            side_like_deck_points.append(point)

    assert side_like_deck_points == []


def test_terrain_pct_planner_routes_from_floor_over_ramp_to_upper_deck() -> None:
    graph = build_terrain_graph(_realistic_world(), grid_resolution=0.40)
    path = plan_terrain_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(2.0, 6.8),
        start_z=0.0,
        goal_z_policy='highest',
    )
    labels = {node.surface_label for node in path}

    assert len(graph.nodes) > 1000
    assert path
    assert any('wide_access_ramp' in label for label in labels)
    assert any('mezzanine_deck_visual' in label for label in labels)
    assert max(node.z for node in path) > 0.60
    assert path[-1].z > 0.60

    for first, second in zip(path, path[1:]):
        horizontal = math.hypot(second.x - first.x, second.y - first.y)
        dz = abs(second.z - first.z)
        assert dz <= 0.36
        assert dz / max(horizontal, 1e-6) <= 0.58


def test_terrain_planner_preserves_ramp_start_from_body_height() -> None:
    graph = build_terrain_graph(_realistic_world(), grid_resolution=0.40)
    path = plan_terrain_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(1.2, 0.0),
        start_z=0.26,
        goal_z_policy='highest',
    )
    waypoints = _waypoint_path(path, spacing=0.90)

    assert waypoints
    assert 'wide_access_ramp' in waypoints[0].surface_label
    assert waypoints[0].z > 0.30
    assert waypoints[-1].surface_label.startswith('floor')
    assert waypoints[-1].z < 0.05


def test_terrain_planner_does_not_drop_from_ramp_edge_to_floor() -> None:
    graph = build_terrain_graph(_realistic_world(), grid_resolution=0.25)
    path = plan_terrain_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(1.2, 0.0),
        start_z=0.42,
        goal_z_policy='highest',
    )

    assert path
    assert 'wide_access_ramp' in path[0].surface_label

    for first, second in zip(path, path[1:]):
        first_is_ramp = 'wide_access_ramp' in first.surface_label
        second_is_floor = second.surface_label.startswith('floor')
        if first_is_ramp and second_is_floor:
            assert first.z <= 0.12


def test_initial_surface_hint_uses_world_surface_not_body_spawn_height() -> None:
    graph = build_terrain_graph(_realistic_world(), grid_resolution=0.25)

    surface_z = _surface_height_at_xy(graph.nodes, (0.0, 0.0), z_hint=0.42)

    assert surface_z is not None
    assert 0.33 <= surface_z <= 0.36


def test_terrain_planner_uses_ramp_surface_height_for_default_spawn() -> None:
    graph = build_terrain_graph(_realistic_world(), grid_resolution=0.40)
    path = plan_terrain_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(1.2, 0.0),
        start_z=0.37,
        goal_z_policy='highest',
    )

    assert path
    assert 'wide_access_ramp' in path[0].surface_label
    assert path[0].z > 0.34
    assert path[-1].surface_label.startswith('floor')


def test_nav_waypoints_drop_ramp_start_node_inside_clearance_radius() -> None:
    path = [
        TerrainNode(0, -0.15, -0.06, 0.37, 'wide_access_ramp/link/collision', 1.0),
        TerrainNode(1, 0.20, -1.00, 0.0, 'floor/link/collision', 1.0),
        TerrainNode(2, 1.00, 0.00, 0.0, 'floor/link/collision', 1.0),
    ]

    waypoints = _waypoints_after_start_clearance(
        path,
        start_xy=(0.0, 0.0),
        clearance_radius=0.25,
    )

    assert waypoints == path[1:]


def test_nav_waypoints_drop_all_near_start_nodes_but_keep_floor_descent_target() -> None:
    path = [
        TerrainNode(0, -0.15, -0.06, 0.37, 'wide_access_ramp/link/collision', 1.0),
        TerrainNode(1, -0.15, -0.46, 0.31, 'wide_access_ramp/link/collision', 1.0),
        TerrainNode(2, 0.20, -1.00, 0.0, 'floor/link/collision', 1.0),
        TerrainNode(3, 1.00, -0.20, 0.0, 'floor/link/collision', 1.0),
    ]

    waypoints = _waypoints_after_start_clearance(
        path,
        start_xy=(0.0, 0.0),
        clearance_radius=0.75,
    )

    assert waypoints == path[2:]


def test_direct_clearance_keeps_nearby_high_waypoint_until_height_is_reached() -> None:
    path = [
        TerrainNode(0, 0.10, 0.00, 2.05, 'slam_deck', 1.0),
        TerrainNode(1, 1.00, 0.00, 2.10, 'slam_deck', 1.0),
    ]

    waypoints = _waypoints_after_start_clearance(
        path,
        start_xy=(0.0, 0.0),
        clearance_radius=0.75,
        current_z=0.45,
        z_tolerance=0.45,
    )

    assert waypoints == path


def test_follow_path_keeps_nearby_ramp_nodes_for_safe_descent() -> None:
    path = [
        TerrainNode(0, -0.15, -0.06, 0.37, 'wide_access_ramp/link/collision', 1.0),
        TerrainNode(1, -0.15, -0.46, 0.31, 'wide_access_ramp/link/collision', 1.0),
        TerrainNode(2, 0.20, -1.00, 0.0, 'floor/link/collision', 1.0),
        TerrainNode(3, 1.00, -0.20, 0.0, 'floor/link/collision', 1.0),
    ]

    follow_path = _waypoints_after_start_clearance(
        path,
        start_xy=(0.0, 0.0),
        clearance_radius=0.12,
    )

    assert follow_path == path


def test_direct_tracking_uses_wide_start_clearance_to_skip_stale_waypoints() -> None:
    assert direct_tracking_start_clearance(
        follow_path_start_clearance=0.12,
        start_waypoint_clearance=0.75,
        direct_waypoint_tolerance=0.42,
        direct_lookahead_dist=0.45,
    ) == 0.75


def test_slope_path_uses_lower_speed_limit() -> None:
    slope_path = [
        TerrainNode(0, -0.15, -0.06, 0.37, 'wide_access_ramp/link/collision', 1.0),
        TerrainNode(1, -0.15, -0.46, 0.31, 'wide_access_ramp/link/collision', 1.0),
    ]
    flat_path = [
        TerrainNode(0, 0.20, -1.00, 0.0, 'floor/link/collision', 1.0),
        TerrainNode(1, 1.00, -1.00, 0.0, 'floor/link/collision', 1.0),
    ]

    assert _path_speed_limit(
        slope_path,
        slope_speed_limit=0.12,
        flat_speed_limit=0.22,
        slope_grade_threshold=0.08,
    ) == 0.12
    assert _path_speed_limit(
        flat_path,
        slope_speed_limit=0.12,
        flat_speed_limit=0.22,
        slope_grade_threshold=0.08,
    ) == 0.22


def test_direct_tracking_stops_forward_speed_for_large_heading_error() -> None:
    large_heading_speed = _direct_linear_speed(
        speed_limit=0.14,
        max_linear_speed=0.20,
        min_linear_speed=0.035,
        heading_error=1.40,
        max_heading_error_for_forward=1.25,
        target_distance=0.60,
        slow_radius=0.45,
    )

    assert large_heading_speed == 0.0
    assert _direct_linear_speed(
        speed_limit=0.14,
        max_linear_speed=0.20,
        min_linear_speed=0.035,
        heading_error=0.30,
        max_heading_error_for_forward=1.25,
        target_distance=0.60,
        slow_radius=0.45,
    ) > 0.08


def test_surface_speed_limit_uses_slope_label() -> None:
    assert _surface_speed_limit_for_label(
        'wide_access_ramp/link/collision',
        slope_speed_limit=0.14,
        flat_speed_limit=0.22,
    ) == 0.14
    assert _surface_speed_limit_for_label(
        'floor/link/collision',
        slope_speed_limit=0.14,
        flat_speed_limit=0.22,
    ) == 0.22


def test_surface_z_reference_uses_spawn_hint_until_robot_leaves_initial_area() -> None:
    assert _surface_z_reference(
        odom_z=0.0,
        current_xy=(0.10, 0.20),
        initial_xy=(0.0, 0.0),
        initial_surface_z_hint=0.26,
        initial_surface_hint_radius=0.75,
        last_path=[],
        last_path_surface_hint_radius=0.75,
    ) == 0.26
    assert _surface_z_reference(
        odom_z=0.0,
        current_xy=(2.0, 0.0),
        initial_xy=(0.0, 0.0),
        initial_surface_z_hint=0.26,
        initial_surface_hint_radius=0.75,
        last_path=[],
        last_path_surface_hint_radius=0.75,
    ) == 0.0


def test_surface_z_reference_uses_last_path_when_odom_is_flat() -> None:
    path = [
        TerrainNode(0, -0.15, -0.06, 0.37, 'wide_access_ramp/link/collision', 1.0),
        TerrainNode(1, 0.20, -1.00, 0.0, 'floor/link/collision', 1.0),
    ]

    assert _surface_z_reference(
        odom_z=0.0,
        current_xy=(-0.10, -0.05),
        initial_xy=(0.0, 0.0),
        initial_surface_z_hint=0.26,
        initial_surface_hint_radius=0.75,
        last_path=path,
        last_path_surface_hint_radius=0.75,
    ) == 0.37


def test_terrain_planner_distinguishes_active_nav_goal_statuses() -> None:
    assert _goal_is_active(1)
    assert _goal_is_active(2)
    assert _goal_is_active(3)
    assert not _goal_is_active(4)
    assert not _goal_is_active(5)
    assert not _goal_is_active(6)


def test_terrain_planner_cancels_previous_nav_goal_before_new_one() -> None:
    source = (
        _repo_root()
        / 'src/airos_experiments/airos_experiments/terrain_pct_planner.py'
    ).read_text(encoding='utf-8')

    assert 'cancel_goal_async' in source
    assert '_pending_nav_goal' in source
    assert 'duplicate_goal_xy_tolerance' in source
    assert 'duplicate_goal_time_sec' in source
    assert 'terrain-guided navigation goal was rejected' in source
