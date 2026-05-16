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
    transform_point,
)
from airos_experiments.terrain_pct_planner import (
    TerrainGraph,
    TerrainNode,
    TerrainPctPlanner,
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
    advance_direct_target_index,
    direct_tracking_progress_z,
    direct_tracking_start_clearance,
    direct_tracking_gate_z,
    drop_regressive_start_waypoints,
    plan_terrain_path,
    should_defer_pending_final_goal_for_active_frontier,
    should_reject_regressive_final_path,
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


def test_final_high_path_rejects_deck_edge_step_without_ramp_approach() -> None:
    nodes = [
        TerrainNode(0, 0.0, 0.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(1, 0.8, 0.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(2, 1.5, 0.0, 0.74, 'slam_step', 1.0),
        TerrainNode(3, 2.3, 0.0, 1.2, 'slam_deck', 1.0),
        TerrainNode(4, 0.0, 1.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(5, 0.8, 1.0, 0.25, 'slam_ramp', 1.0),
        TerrainNode(6, 1.6, 1.0, 0.55, 'slam_ramp', 1.0),
        TerrainNode(7, 2.3, 1.0, 1.2, 'slam_deck', 1.0),
    ]
    graph = TerrainGraph(
        nodes=nodes,
        adjacency=[
            [(1, 0.8), (4, 1.0)],
            [(0, 0.8), (2, 0.9)],
            [(1, 0.9), (3, 1.0)],
            [(2, 1.0)],
            [(0, 1.0), (5, 1.0)],
            [(4, 1.0), (6, 1.0)],
            [(5, 1.0), (7, 1.0)],
            [(6, 1.0)],
        ],
        terrain_cloud=[],
    )

    path = plan_terrain_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(2.3, 0.5),
        start_z=0.0,
        goal_z_policy='highest',
        goal_min_z=1.0,
    )

    assert [node.index for node in path] == [0, 4, 5, 6, 7]


def test_final_high_path_rejects_step_drop_after_high_entry() -> None:
    nodes = [
        TerrainNode(0, 0.0, 0.0, 0.0, 'slam_floor', 1.0),
        TerrainNode(1, 0.8, 0.0, 0.25, 'slam_ramp', 1.0),
        TerrainNode(2, 1.6, 0.0, 0.55, 'slam_ramp', 1.0),
        TerrainNode(3, 2.4, 0.0, 0.82, 'slam_step', 1.0),
        TerrainNode(4, 3.2, 0.0, -0.02, 'slam_step', 1.0),
        TerrainNode(5, 4.0, 0.0, 1.2, 'slam_deck', 1.0),
        TerrainNode(6, 0.8, 1.0, 0.25, 'slam_ramp', 1.0),
        TerrainNode(7, 1.6, 1.0, 0.55, 'slam_ramp', 1.0),
        TerrainNode(8, 2.4, 1.0, 0.82, 'slam_step', 1.0),
        TerrainNode(9, 3.2, 1.0, 1.05, 'slam_step', 1.0),
        TerrainNode(10, 4.0, 1.0, 1.2, 'slam_deck', 1.0),
    ]
    graph = TerrainGraph(
        nodes=nodes,
        adjacency=[
            [(1, 0.8), (6, 1.3)],
            [(0, 0.8), (2, 0.8)],
            [(1, 0.8), (3, 0.8)],
            [(2, 0.8), (4, 0.8)],
            [(3, 0.8), (5, 0.8)],
            [(4, 0.8)],
            [(0, 1.3), (7, 0.8)],
            [(6, 0.8), (8, 0.8)],
            [(7, 0.8), (9, 0.8)],
            [(8, 0.8), (10, 0.8)],
            [(9, 0.8)],
        ],
        terrain_cloud=[],
    )

    path = plan_terrain_path(
        graph,
        start_xy=(0.0, 0.0),
        goal_xy=(4.0, 0.5),
        start_z=0.0,
        goal_z_policy='highest',
        goal_min_z=1.0,
    )

    assert [node.index for node in path] == [0, 6, 7, 8, 9, 10]


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


def test_large_multilevel_lower_ramp_physically_connects_landings() -> None:
    lower_ramp = _box_by_model(_large_world(), 'lower_access_ramp')
    lower_landing = _box_by_model(_large_world(), 'ramp_lower_landing')
    upper_landing = _box_by_model(_large_world(), 'ramp_upper_landing')
    deck = _box_by_model(_large_world(), 'second_floor_deck')

    ramp_half_x = lower_ramp.size[0] / 2.0
    ramp_top_z = lower_ramp.size[2] / 2.0
    lower_start = transform_point(
        lower_ramp.transform,
        (-ramp_half_x, 0.0, ramp_top_z),
    )
    upper_end = transform_point(
        lower_ramp.transform,
        (ramp_half_x, 0.0, ramp_top_z),
    )
    lower_landing_top = transform_point(
        lower_landing.transform,
        (0.0, 0.0, lower_landing.size[2] / 2.0),
    )
    upper_landing_top = transform_point(
        upper_landing.transform,
        (0.0, 0.0, upper_landing.size[2] / 2.0),
    )
    deck_top = transform_point(deck.transform, (0.0, 0.0, deck.size[2] / 2.0))

    assert abs(lower_start[2] - lower_landing_top[2]) <= 0.12
    assert abs(upper_end[2] - upper_landing_top[2]) <= 0.12
    assert abs(upper_landing_top[2] - deck_top[2]) <= 0.12


def test_large_multilevel_lower_ramp_has_robot_body_clearance() -> None:
    lower_ramp = _box_by_model(_large_world(), 'lower_access_ramp')
    deck = _box_by_model(_large_world(), 'second_floor_deck')
    robot_body_height = 0.18
    body_clearance_margin = 0.10
    required_clearance = robot_body_height + body_clearance_margin

    ramp_half_x = lower_ramp.size[0] / 2.0
    ramp_top_z = lower_ramp.size[2] / 2.0
    deck_bottom_local_z = -deck.size[2] / 2.0
    for fraction in (0.0, 0.25, 0.50, 0.75, 1.0):
        ramp_local_x = -ramp_half_x + fraction * lower_ramp.size[0]
        ramp_top = transform_point(
            lower_ramp.transform,
            (ramp_local_x, 0.0, ramp_top_z),
        )
        deck_local = inverse_transform_point(deck.transform, ramp_top)
        deck_covers_ramp_center = (
            abs(deck_local[0]) <= deck.size[0] / 2.0
            and abs(deck_local[1]) <= deck.size[1] / 2.0
        )
        if not deck_covers_ramp_center:
            continue
        deck_bottom = transform_point(
            deck.transform,
            (deck_local[0], deck_local[1], deck_bottom_local_z),
        )
        assert deck_bottom[2] - ramp_top[2] >= required_clearance


def test_large_multilevel_upper_landing_starts_at_ramp_top() -> None:
    lower_ramp = _box_by_model(_large_world(), 'lower_access_ramp')
    upper_landing = _box_by_model(_large_world(), 'ramp_upper_landing')

    landing_leading_edge = transform_point(
        upper_landing.transform,
        (0.0, -upper_landing.size[1] / 2.0, upper_landing.size[2] / 2.0),
    )
    ramp_local = inverse_transform_point(lower_ramp.transform, landing_leading_edge)
    ramp_top_at_landing_edge = transform_point(
        lower_ramp.transform,
        (ramp_local[0], 0.0, lower_ramp.size[2] / 2.0),
    )

    assert abs(ramp_local[1]) <= lower_ramp.size[1] / 2.0
    assert abs(landing_leading_edge[2] - ramp_top_at_landing_edge[2]) <= 0.05


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


def test_direct_tracking_drops_regressive_low_prefix_without_skipping_high_entry() -> None:
    path = [
        TerrainNode(0, 3.26, 1.20, 0.01, 'slam_floor', 1.0),
        TerrainNode(1, 4.93, 1.37, 0.14, 'slam_step', 1.0),
        TerrainNode(2, 4.70, 5.30, 0.90, 'slam_ramp', 1.0),
        TerrainNode(3, 6.00, 13.00, 2.20, 'slam_deck', 1.0),
    ]

    filtered = drop_regressive_start_waypoints(
        path,
        start_xy=(4.37, 1.45),
        final_goal_xy=(6.0, 13.0),
        regression_tolerance=0.42,
        current_z=0.30,
        z_tolerance=0.45,
    )

    assert filtered == path[1:]


def test_direct_tracking_drops_regressive_low_ramp_prefix_before_high_entry() -> None:
    path = [
        TerrainNode(0, 3.20, -12.20, 0.05, 'slam_ramp', 1.0),
        TerrainNode(1, 4.90, -11.40, 0.36, 'slam_ramp', 1.0),
        TerrainNode(2, -4.70, -6.30, 0.10, 'slam_floor', 1.0),
        TerrainNode(3, -4.70, -3.40, 0.42, 'slam_ramp', 1.0),
        TerrainNode(4, 6.00, 13.00, 2.20, 'slam_deck', 1.0),
    ]

    filtered = drop_regressive_start_waypoints(
        path,
        start_xy=(5.31, -11.35),
        final_goal_xy=(6.0, 13.0),
        regression_tolerance=0.42,
        current_z=0.22,
        z_tolerance=0.45,
    )

    assert filtered == path[2:]


def test_pending_final_goal_waits_for_active_frontier_endpoint() -> None:
    active_frontier_path = [
        TerrainNode(0, -3.40, -3.96, 0.13, 'slam_ramp', 1.0),
        TerrainNode(1, -3.60, -0.83, 0.42, 'slam_ramp', 1.0),
    ]

    assert should_defer_pending_final_goal_for_active_frontier(
        active_path=active_frontier_path,
        current_xy=(-3.15, -1.67),
        goal_tolerance=0.30,
        execution_active=True,
        active_final_goal_xy=(6.0, 13.0),
        final_goal_xy=(6.0, 13.0),
        final_goal_tolerance=0.05,
    )


def test_high_final_path_rejects_large_initial_goal_regression() -> None:
    path = [
        TerrainNode(0, -3.15, -1.67, 0.31, 'slam_ramp', 1.0),
        TerrainNode(1, 7.18, -11.29, 0.65, 'slam_ramp', 1.0),
        TerrainNode(2, 6.00, 13.00, 2.20, 'slam_deck', 1.0),
    ]

    assert should_reject_regressive_final_path(
        path,
        start_xy=(-3.15, -1.67),
        final_goal_xy=(6.0, 13.0),
        start_z=0.31,
        regression_tolerance=1.5,
    )


def test_high_final_path_rejects_low_floor_detour_before_ramp_entry() -> None:
    path = [
        TerrainNode(0, -1.00, -2.80, 0.00, 'slam_floor', 1.0),
        TerrainNode(1, 2.20, -3.05, -0.14, 'slam_floor', 1.0),
        TerrainNode(2, 3.59, -0.89, -0.35, 'slam_floor', 1.0),
        TerrainNode(3, 4.82, 2.09, -0.67, 'slam_ramp', 1.0),
        TerrainNode(4, 4.76, 4.80, 0.43, 'slam_step', 1.0),
        TerrainNode(5, 0.40, 3.60, 0.65, 'slam_deck', 1.0),
    ]

    assert should_reject_regressive_final_path(
        path,
        start_xy=(-1.0, -2.8),
        final_goal_xy=(0.4, 3.6),
        start_z=0.0,
        regression_tolerance=1.5,
    )


def test_direct_tracking_gate_uses_surface_height_for_high_waypoint_progress() -> None:
    terrain_nodes = [
        TerrainNode(100, 0.34, 0.35, -0.02, 'slam_floor', 1.0),
        TerrainNode(101, 0.58, 0.64, 0.76, 'slam_deck', 1.0),
    ]
    direct_path = [
        TerrainNode(10, 0.58, 0.64, 0.76, 'slam_deck', 1.0),
        TerrainNode(11, 1.46, 1.14, 0.77, 'slam_deck', 1.0),
    ]

    gate_z = direct_tracking_gate_z(
        terrain_nodes,
        current_xy=(0.34, 0.35),
        odom_z=0.34,
    )

    assert gate_z < 0.05
    assert advance_direct_target_index(
        direct_path,
        0,
        (0.34, 0.35),
        0.42,
        current_z=gate_z,
        z_tolerance=0.45,
    ) == 0


def test_direct_tracking_holds_high_deck_waypoint_until_physical_height_progress() -> None:
    direct_path = [
        TerrainNode(10, 6.20, 5.97, 1.20, 'slam_deck', 1.0),
        TerrainNode(11, 6.24, 6.48, 1.24, 'slam_deck', 1.0),
    ]

    assert advance_direct_target_index(
        direct_path,
        0,
        (6.20, 5.96),
        0.42,
        current_z=0.80,
        z_tolerance=0.45,
    ) == 0


def test_high_surface_gate_does_not_prove_physical_height_progress() -> None:
    direct_path = [
        TerrainNode(10, -5.93, 3.62, 0.76, 'slam_deck', 1.0),
        TerrainNode(11, -7.21, 3.33, 0.83, 'slam_step', 1.0),
    ]
    surface_gate_z = 0.76
    physical_z = 0.36

    progress_z = direct_tracking_progress_z(
        direct_path,
        current_index=0,
        physical_z=physical_z,
        surface_gate_z=surface_gate_z,
    )

    assert progress_z == physical_z
    assert advance_direct_target_index(
        direct_path,
        0,
        (-5.93, 3.62),
        0.42,
        current_z=progress_z,
        z_tolerance=0.45,
    ) == 0


def test_high_step_requires_physical_height_before_waypoint_progress() -> None:
    direct_path = [
        TerrainNode(10, -2.72, 0.63, 0.82, 'slam_step', 1.0),
        TerrainNode(11, -2.10, 1.17, -0.02, 'slam_step', 1.0),
    ]
    surface_gate_z = 0.82
    physical_z = 0.33

    progress_z = direct_tracking_progress_z(
        direct_path,
        current_index=0,
        physical_z=physical_z,
        surface_gate_z=surface_gate_z,
    )

    assert progress_z == physical_z
    assert advance_direct_target_index(
        direct_path,
        0,
        (-2.72, 0.63),
        0.42,
        current_z=progress_z,
        z_tolerance=0.45,
    ) == 0


def test_direct_tracking_lookahead_pushes_past_xy_reached_height_debt_step() -> None:
    direct_path = [
        TerrainNode(10, -3.93, 2.14, 0.52, 'slam_step', 1.0),
        TerrainNode(11, -4.18, 2.10, 0.52, 'slam_step', 1.0),
        TerrainNode(12, -4.55, 2.22, 0.58, 'slam_step', 1.0),
    ]
    planner = object.__new__(TerrainPctPlanner)
    planner._direct_path = direct_path
    planner._direct_target_index = 0
    planner._direct_lookahead_dist = 0.45
    planner._direct_waypoint_tolerance = 0.42
    planner._direct_z_tolerance = 0.45

    target = TerrainPctPlanner._direct_lookahead_target(
        planner,
        -3.93,
        2.14,
        0.26,
    )

    assert target.index == 12
    assert advance_direct_target_index(
        direct_path,
        0,
        (-3.93, 2.14),
        0.42,
        current_z=0.26,
        z_tolerance=0.45,
    ) == 0


def test_direct_tracking_height_debt_lookahead_does_not_cross_surface_change() -> None:
    direct_path = [
        TerrainNode(10, -3.93, 2.14, 0.52, 'slam_step', 1.0),
        TerrainNode(11, -4.25, 2.23, 0.58, 'slam_ramp', 1.0),
        TerrainNode(12, -4.65, 2.36, 0.68, 'slam_ramp', 1.0),
    ]
    planner = object.__new__(TerrainPctPlanner)
    planner._direct_path = direct_path
    planner._direct_target_index = 0
    planner._direct_lookahead_dist = 0.45
    planner._direct_waypoint_tolerance = 0.42
    planner._direct_z_tolerance = 0.45

    target = TerrainPctPlanner._direct_lookahead_target(
        planner,
        -3.93,
        2.14,
        0.26,
    )

    assert target.index == 10


def test_direct_tracking_height_debt_lookahead_prefers_forward_path_tangent() -> None:
    direct_path = [
        TerrainNode(10, -4.29, 3.35, 0.72, 'slam_step', 1.0),
        TerrainNode(11, -4.12, 3.90, 0.73, 'slam_step', 1.0),
        TerrainNode(12, -3.97, 4.46, 0.74, 'slam_step', 1.0),
    ]
    planner = object.__new__(TerrainPctPlanner)
    planner._direct_path = direct_path
    planner._direct_target_index = 0
    planner._direct_lookahead_dist = 0.45
    planner._direct_waypoint_tolerance = 0.42
    planner._direct_z_tolerance = 0.45

    target = TerrainPctPlanner._direct_lookahead_target(
        planner,
        -4.05,
        3.70,
        0.40,
    )

    assert target.index == 12
    assert advance_direct_target_index(
        direct_path,
        0,
        (-4.05, 3.70),
        0.42,
        current_z=0.40,
        z_tolerance=0.45,
    ) == 0


def test_direct_tracking_height_debt_lookahead_ignores_following_zigzag() -> None:
    direct_path = [
        TerrainNode(10, -4.19, 4.16, 0.70, 'slam_step', 1.0),
        TerrainNode(11, -3.95, 5.10, 0.86, 'slam_step', 1.0),
        TerrainNode(12, -4.12, 4.70, 0.88, 'slam_step', 1.0),
    ]
    planner = object.__new__(TerrainPctPlanner)
    planner._direct_path = direct_path
    planner._direct_target_index = 0
    planner._direct_lookahead_dist = 0.45
    planner._direct_waypoint_tolerance = 0.42
    planner._direct_z_tolerance = 0.45

    target = TerrainPctPlanner._direct_lookahead_target(
        planner,
        -4.18,
        4.64,
        0.39,
    )

    assert target.index == 11
    assert advance_direct_target_index(
        direct_path,
        0,
        (-4.18, 4.64),
        0.42,
        current_z=0.39,
        z_tolerance=0.45,
    ) == 0


def test_direct_tracking_skips_low_ramp_waypoint_after_height_progress() -> None:
    direct_path = [
        TerrainNode(10, -3.39, -0.97, 0.13, 'slam_ramp', 1.0),
        TerrainNode(11, -3.41, -0.66, 0.19, 'slam_ramp', 1.0),
        TerrainNode(12, -3.30, 0.10, 0.55, 'slam_step', 1.0),
    ]

    assert advance_direct_target_index(
        direct_path,
        0,
        (-3.34, -1.43),
        0.42,
        current_z=0.48,
        z_tolerance=0.45,
    ) == 1


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
    assert _surface_speed_limit_for_label(
        'ramp_upper_landing/link/collision',
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


def test_direct_tracking_diagnostics_log_target_robot_and_command_state() -> None:
    source = (
        _repo_root()
        / 'src/airos_experiments/airos_experiments/terrain_pct_planner.py'
    ).read_text(encoding='utf-8')

    assert "direct_diagnostics_period_sec" in source
    assert "_maybe_log_direct_diagnostics(" in source
    assert "direct tracking diagnostics: " in source
    for field in (
        "index=",
        "target=(",
        "robot=(",
        "gate_z=",
        "surface=",
        "heading_error=",
        "speed_limit=",
        "cmd=(",
    ):
        assert field in source


def test_direct_tracking_requires_final_goal_xy_for_off_graph_endpoint() -> None:
    from airos_experiments import terrain_pct_planner as planner

    endpoint = TerrainNode(10, 7.81, -9.55, 0.0, 'slam_floor', 1.0)
    reaches_goal = getattr(planner, 'direct_tracking_reaches_goal', None)

    assert reaches_goal is not None

    assert not reaches_goal(
        endpoint,
        current_xy=(7.81, -9.55),
        current_z=0.0,
        final_goal_xy=(8.0, -9.0),
        xy_tolerance=0.30,
        z_tolerance=0.45,
    )


def test_direct_tracking_accepts_final_goal_xy_inside_tolerance() -> None:
    from airos_experiments import terrain_pct_planner as planner

    endpoint = TerrainNode(10, 1.88, -9.19, 0.0, 'slam_floor', 1.0)

    assert planner.direct_tracking_reaches_goal(
        endpoint,
        current_xy=(1.88, -9.19),
        current_z=0.0,
        final_goal_xy=(1.9, -9.2),
        xy_tolerance=0.30,
        z_tolerance=0.45,
    )


def test_direct_tracking_control_separates_surface_gate_from_progress_z() -> None:
    source = (
        _repo_root()
        / 'src/airos_experiments/airos_experiments/terrain_pct_planner.py'
    ).read_text(encoding='utf-8')

    assert "gate_z = self._direct_gate_z(current_x, current_y, current_z)" in source
    assert (
        "progress_z = direct_tracking_progress_z("
        in source
    )
    assert "self._advance_direct_target(current_x, current_y, progress_z)" in source
    assert (
        "target = self._direct_lookahead_target(current_x, current_y, progress_z)"
        in source
    )
    assert "self._graph.nodes" in source
    assert "self._terrain_graph" not in source


def test_slam_rebuild_does_not_block_direct_control_executor() -> None:
    source = (
        _repo_root()
        / 'src/airos_experiments/airos_experiments/terrain_pct_planner.py'
    ).read_text(encoding='utf-8')

    assert "from concurrent.futures import Future, ThreadPoolExecutor" in source
    assert "self._slam_rebuild_executor = ThreadPoolExecutor(" in source
    assert "max_workers=1" in source
    assert "self._slam_graph_future" in source
    assert "_queue_slam_graph_rebuild" in source
    assert "_apply_finished_slam_graph_rebuild" in source
    assert "graph = build_slam_terrain_graph_from_pointcloud(" not in source.split(
        "    def _rebuild_slam_graph("
    )[1].split("    def _initial_pose_callback(")[0]
    main_section = source.split("def main() -> None:")[1]
    assert "MultiThreadedExecutor" in main_section
    assert "rclpy.spin(node)" not in main_section


def test_goal_callback_logs_received_and_duplicate_terrain_goals() -> None:
    source = (
        _repo_root()
        / 'src/airos_experiments/airos_experiments/terrain_pct_planner.py'
    ).read_text(encoding='utf-8')

    assert "received terrain goal: " in source
    assert "ignored duplicate terrain goal: " in source
    assert source.index("ignored duplicate terrain goal: ") < source.index(
        "path = plan_terrain_path("
    )
