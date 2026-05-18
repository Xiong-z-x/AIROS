from __future__ import annotations

from pathlib import Path

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

from airos_experiments.planner_comparison_node import (
    GridMap,
    _RrtStarPlanner,
    _overlay_scan_obstacles,
    _grid_astar_path,
    _path_collision_free,
    _repair_path_on_grid,
    _shortcut_path,
    _value_iteration_path,
    _CoarseGrid,
)


def _open_grid() -> GridMap:
    msg = OccupancyGrid()
    msg.info.width = 20
    msg.info.height = 20
    msg.info.resolution = 0.2
    msg.info.origin.position.x = -2.0
    msg.info.origin.position.y = -2.0
    msg.data = [0] * (msg.info.width * msg.info.height)
    return GridMap.from_msg(msg, occupied_threshold=65, robot_radius_m=0.01)


def test_scan_overlay_marks_fast_lio_projected_obstacle_in_planning_grid():
    grid = _open_grid()
    scan = LaserScan()
    scan.header.frame_id = 'base_footprint'
    scan.angle_min = 0.0
    scan.angle_increment = 0.0
    scan.range_min = 0.08
    scan.range_max = 4.5
    scan.ranges = [1.0]

    overlay = _overlay_scan_obstacles(
        grid,
        scan,
        base_pose=(0.0, 0.0, 0.0),
        obstacle_radius_m=0.25,
    )

    assert overlay.occupied(overlay.world_to_grid((1.0, 0.0)))
    assert not grid.occupied(grid.world_to_grid((1.0, 0.0)))


def test_online_slam_unknown_cells_are_not_treated_as_obstacles_by_default():
    msg = OccupancyGrid()
    msg.info.width = 6
    msg.info.height = 6
    msg.info.resolution = 0.2
    msg.data = [-1] * (msg.info.width * msg.info.height)
    msg.data[3 * msg.info.width + 3] = 100

    grid = GridMap.from_msg(msg, occupied_threshold=65, robot_radius_m=0.01)

    assert not grid.raw_occupied((0, 0))
    assert not grid.occupied((0, 0))
    assert grid.raw_occupied((3, 3))
    assert grid.occupied((3, 3))


def test_value_iteration_q_path_reaches_goal_on_open_grid():
    grid = _open_grid()
    coarse = _CoarseGrid(grid, step=1)

    path, expanded = _value_iteration_path(
        coarse,
        start=(2, 2),
        goal=(17, 17),
        max_iterations=3000,
        discount=0.96,
    )

    assert path[0] == (2, 2)
    assert path[-1] == (17, 17)
    assert len(path) >= 2
    assert expanded > 0


def test_rrt_star_returns_collision_free_path_on_open_grid():
    grid = _open_grid()
    planner = _RrtStarPlanner(
        grid,
        step_m=0.5,
        goal_sample_rate=0.35,
        rewire_radius_m=0.8,
        max_samples=800,
        rng=__import__('random').Random(4),
    )

    path, expanded, message = planner.plan((-1.4, -1.4), (1.4, 1.4))

    assert message == 'ok'
    assert len(path) >= 2
    assert expanded > 0
    assert path[0] != path[-1]
    assert all(grid.free(grid.world_to_grid(point)) for point in path)
    assert _path_collision_free(grid, path)


def test_path_collision_free_rejects_segment_through_obstacle():
    msg = OccupancyGrid()
    msg.info.width = 20
    msg.info.height = 20
    msg.info.resolution = 0.2
    msg.info.origin.position.x = -2.0
    msg.info.origin.position.y = -2.0
    data = [0] * (msg.info.width * msg.info.height)
    for y in range(20):
        data[y * 20 + 10] = 100
    msg.data = data
    grid = GridMap.from_msg(msg, occupied_threshold=65, robot_radius_m=0.01)

    assert not _path_collision_free(grid, [(-1.5, 0.0), (1.5, 0.0)])


def test_shortcut_path_keeps_segments_collision_free():
    msg = OccupancyGrid()
    msg.info.width = 30
    msg.info.height = 30
    msg.info.resolution = 0.2
    msg.info.origin.position.x = -3.0
    msg.info.origin.position.y = -3.0
    data = [0] * (msg.info.width * msg.info.height)
    for y in range(30):
        if y not in {14, 15, 16}:
            data[y * 30 + 15] = 100
    msg.data = data
    grid = GridMap.from_msg(msg, occupied_threshold=65, robot_radius_m=0.01)

    path = [(-2.5, -2.5), (-0.3, 0.1), (0.1, 0.1), (2.5, 2.5)]
    shortened = _shortcut_path(grid, path)

    assert _path_collision_free(grid, shortened)
    assert len(shortened) <= len(path)


def test_repair_path_on_grid_replaces_blocked_segment_with_fine_path():
    msg = OccupancyGrid()
    msg.info.width = 30
    msg.info.height = 30
    msg.info.resolution = 0.2
    msg.info.origin.position.x = -3.0
    msg.info.origin.position.y = -3.0
    data = [0] * (msg.info.width * msg.info.height)
    for y in range(30):
        if y not in {14, 15, 16}:
            data[y * 30 + 15] = 100
    msg.data = data
    grid = GridMap.from_msg(msg, occupied_threshold=65, robot_radius_m=0.01)
    coarse_path = [(-2.5, -2.0), (2.5, -2.0)]

    repaired, changed = _repair_path_on_grid(grid, coarse_path, max_radius_cells=6)

    assert changed
    assert _path_collision_free(grid, repaired)
    assert len(repaired) > len(coarse_path)


def test_smac_style_astar_fallback_reaches_goal_on_open_grid():
    grid = _open_grid()

    path, expanded = _grid_astar_path(
        grid,
        start=(-1.4, -1.4),
        goal=(1.4, 1.4),
        max_radius_cells=5,
    )

    assert len(path) >= 2
    assert expanded > 0
    assert all(grid.free(grid.world_to_grid(point)) for point in path)


def test_planner_comparison_artifacts_are_present():
    assert Path('src/airos_experiments/launch/planner_comparison.launch.py').is_file()
    assert Path('src/airos_nav/config/nav2_planner_comparison.yaml').is_file()
    assert Path('src/airos_sim/worlds/single_floor_planner_showcase.sdf').is_file()
    assert Path('src/airos_nav/maps/single_floor_planner_showcase.yaml').is_file()


def test_planner_comparison_defaults_to_full_slam_and_motion_chain():
    nav_launch = Path('src/airos_nav/launch/nav.launch.py').read_text(encoding='utf-8')
    comparison_launch = Path(
        'src/airos_experiments/launch/planner_comparison.launch.py'
    ).read_text(encoding='utf-8')

    assert "' in ('full', 'planner_only')" in nav_launch
    assert "' in ('full', 'controller_only')" in nav_launch
    assert "DeclareLaunchArgument('nav_stack_mode', default_value='full')" in comparison_launch
    assert (
        "DeclareLaunchArgument('localization', default_value='slam_toolbox_mapping')"
        in comparison_launch
    )
    assert "DeclareLaunchArgument('slam_nav_startup', default_value='gated')" in comparison_launch
    assert "DeclareLaunchArgument('fast_lio_debug', default_value='true')" in comparison_launch
    assert (
        "DeclareLaunchArgument('colorized_pointcloud', default_value='true')"
        in comparison_launch
    )
    assert (
        "DeclareLaunchArgument('execute_primary_motion', default_value='true')"
        in comparison_launch
    )
    assert (
        "DeclareLaunchArgument('enable_navigate_to_pose_bridge', default_value='false')"
        in comparison_launch
    )
    assert 'single_floor_planner_showcase.yaml' in comparison_launch
    assert 'planner_comparison_node' in comparison_launch


def test_planner_comparison_accepts_rviz_goal_topic_and_nav2_action_without_cmd_vel():
    source = Path(
        'src/airos_experiments/airos_experiments/planner_comparison_node.py'
    ).read_text(encoding='utf-8')

    assert "self.declare_parameter('goal_topic', '/goal_pose')" in source
    assert 'NavigateToPose' in source
    assert "'navigate_to_pose'" in source
    assert 'self._start_planner_comparison(' in source
    assert 'execute_primary_nav2_goal' in source
    assert 'create_publisher(Twist' not in source
    assert "'/cmd_vel'" not in source


def test_planner_comparison_projects_fast_lio_cloud_into_slam_scan_overlay():
    launch_text = Path(
        'src/airos_experiments/launch/planner_comparison.launch.py'
    ).read_text(encoding='utf-8')

    assert "package='fast_lio'" in launch_text
    assert "executable='fastlio_mapping'" in launch_text
    assert "executable='fast_lio_map_aligner'" in launch_text
    assert "'input_topic': '/Laser_map'" in launch_text
    assert "'output_topic': '/Laser_map_world'" in launch_text
    assert "name='fast_lio_registered_aligner'" in launch_text
    assert "'input_topic': '/cloud_registered'" in launch_text
    assert "'output_topic': '/cloud_registered_world'" in launch_text
    assert "'input_topic': '/Laser_map_world'" in launch_text
    assert "'output_topic': '/Laser_map_colored'" in launch_text
    assert "executable='slam_scan_projector'" in launch_text
    assert "'cloud_topic': '/cloud_registered_world'" in launch_text
    assert "'scan_topic': '/slam_scan'" in launch_text
    assert "'use_slam_scan_overlay': True" in launch_text


def test_planner_comparison_motion_execution_can_still_be_overridden():
    launch_text = Path(
        'src/airos_experiments/launch/planner_comparison.launch.py'
    ).read_text(encoding='utf-8')

    assert "DeclareLaunchArgument('execute_primary_motion', default_value='true')" in (
        launch_text
    )
    assert "'execute_primary_nav2_goal': LaunchConfiguration('execute_primary_motion')" in (
        launch_text
    )
    assert 'enable_navigate_to_pose_bridge' in launch_text


def test_nav2_full_params_expose_theta_star_for_comparison_display():
    for params_path in (
        'src/airos_nav/config/nav2_params.yaml',
        'src/airos_nav/config/nav2_research_profile.yaml',
    ):
        params_text = Path(params_path).read_text(encoding='utf-8')

        assert '- GridBased' in params_text
        assert '- ThetaStar' in params_text
        assert 'plugin: nav2_smac_planner/SmacPlanner2D' in params_text
        assert 'plugin: nav2_theta_star_planner/ThetaStarPlanner' in params_text


def test_nav_launch_keeps_slam_toolbox_params_separate_from_nav2_params():
    launch_text = Path('src/airos_nav/launch/nav.launch.py').read_text(
        encoding='utf-8'
    )

    assert "'slam_toolbox_mapping.yaml'" in launch_text
    assert "'slam_toolbox_localization.yaml'" in launch_text
    slam_mapping_section = launch_text.split('    slam_mapping = ')[1].split(
        '    map_only_manager = '
    )[0]
    assert "'params_file': os.path.join(" in slam_mapping_section
    assert "'slam_toolbox_mapping.yaml'" in slam_mapping_section
