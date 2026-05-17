from __future__ import annotations

from pathlib import Path

import yaml


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _read_text(relative_path: str) -> str:
    return (_repo_root() / relative_path).read_text(encoding='utf-8')


def _rviz_display(name: str) -> dict:
    rviz_config = yaml.safe_load(
        _read_text('src/airos_nav/rviz/nav.rviz')
    )
    displays = rviz_config['Visualization Manager']['Displays']
    return next(display for display in displays if display.get('Name') == name)


def test_fast_lio_visual_launch_leaves_laser_map_to_fast_lio_only() -> None:
    launch_text = _read_text(
        'src/airos_experiments/launch/visual_fast_lio_navigation.launch.py'
    )

    assert "'pointcloud_registered': 'false'" in launch_text
    assert "'pointcloud_map': 'false'" in launch_text
    assert "'localization': LaunchConfiguration('localization')" in launch_text
    assert (
        "DeclareLaunchArgument('localization', default_value='slam_toolbox_mapping')"
        in launch_text
    )
    assert "DeclareLaunchArgument('slam_nav_startup', default_value='gated')" in launch_text
    assert 'def _fast_lio_localization_enabled()' in launch_text
    assert "' != 'slam_toolbox_mapping'" in launch_text
    assert 'condition=IfCondition(_fast_lio_localization_enabled())' in launch_text
    assert "DeclareLaunchArgument('fast_lio_debug', default_value='true')" in launch_text
    assert "executable='fast_lio_localization_bridge'" in launch_text
    assert "'fast_lio_odom_topic': '/Odometry'" in launch_text
    assert "'wheel_odom_topic': '/odom'" in launch_text
    assert "'aligned_odom_topic': '/fast_lio_odom_world'" in launch_text
    assert 'static_map_to_odom' not in launch_text
    assert "executable='pointcloud_colorizer'" in launch_text
    assert "DeclareLaunchArgument('colorized_pointcloud', default_value='true')" in launch_text
    assert "DeclareLaunchArgument('dense_visual_pointcloud', default_value='false')" in launch_text
    assert "DeclareLaunchArgument('pointcloud_spacing', default_value='0.06')" in launch_text
    assert "DeclareLaunchArgument('max_live_points', default_value='180000')" in launch_text
    assert (
        "DeclareLaunchArgument('fast_lio_pointcloud_spacing', default_value='0.16')"
    ) in launch_text
    assert (
        "DeclareLaunchArgument('fast_lio_max_live_points', default_value='12000')"
    ) in launch_text
    assert "executable='terrain_pct_planner'" in launch_text
    assert "DeclareLaunchArgument('terrain_planner', default_value='false')" in launch_text
    assert "DeclareLaunchArgument('terrain_map_source', default_value='slam_cloud')" in launch_text
    assert (
        "DeclareLaunchArgument('collision_scan_topic', default_value='/scan')"
    ) in launch_text
    assert "DeclareLaunchArgument('terrain_goal_min_z', default_value='-1.0')" in launch_text
    assert "DeclareLaunchArgument('slam_map_max_points', default_value='180000')" in launch_text
    assert "DeclareLaunchArgument('slam_grid_resolution', default_value='0.30')" in launch_text
    assert "DeclareLaunchArgument('slam_min_cell_points', default_value='2')" in launch_text
    assert "DeclareLaunchArgument('slam_vertical_layer_gap', default_value='0.18')" in launch_text
    assert "DeclareLaunchArgument('slam_rebuild_period_sec', default_value='3.0')" in launch_text
    assert "DeclareLaunchArgument('use_route', default_value='false')" in launch_text
    assert "DeclareLaunchArgument('nav_stack_mode', default_value='full')" in launch_text
    assert "DeclareLaunchArgument('dynamic_obstacles', default_value='false')" in launch_text
    assert (
        "DeclareLaunchArgument('world', default_value='single_floor_complex_large')"
    ) in launch_text
    assert "'point_spacing': LaunchConfiguration('fast_lio_pointcloud_spacing')" in launch_text
    assert "'max_live_points': LaunchConfiguration('fast_lio_max_live_points')" in launch_text
    assert "name='dense_building_pointcloud'" in launch_text
    assert "'registered_cloud_topic': '/dense_visual_cloud'" in launch_text
    assert "'point_spacing': LaunchConfiguration('pointcloud_spacing')" in launch_text
    assert "'max_live_points': LaunchConfiguration('max_live_points')" in launch_text
    assert "'goal_topic': '/terrain_goal_pose'" in launch_text
    assert "'terrain_map_source': LaunchConfiguration('terrain_map_source')" in launch_text
    assert "'slam_map_topic': '/Laser_map_world'" in launch_text
    assert "'slam_map_max_points': LaunchConfiguration('slam_map_max_points')" in launch_text
    assert "'slam_grid_resolution': LaunchConfiguration('slam_grid_resolution')" in launch_text
    assert "'slam_min_cell_points': LaunchConfiguration('slam_min_cell_points')" in launch_text
    assert (
        "'slam_vertical_layer_gap': LaunchConfiguration('slam_vertical_layer_gap')"
    ) in launch_text
    assert (
        "'slam_rebuild_period_sec': LaunchConfiguration('slam_rebuild_period_sec')"
    ) in launch_text
    assert "DeclareLaunchArgument('terrain_odom_topic', default_value='/fast_lio_odom_world')" in launch_text
    assert "executable='slam_scan_projector'" in launch_text
    assert "name='fast_lio_registered_aligner'" in launch_text
    assert "'input_topic': '/cloud_registered'" in launch_text
    assert "'output_topic': '/cloud_registered_world'" in launch_text
    assert "'cloud_topic': '/cloud_registered_world'" in launch_text
    assert "'pose_source': 'tf'" in launch_text
    assert "'map_frame': 'map'" in launch_text
    assert "'base_frame': 'base_footprint'" in launch_text
    assert "'scan_topic': '/slam_scan'" in launch_text
    assert "'min_z': 0.45" in launch_text
    assert "'surface_estimate_radius': 0.75" in launch_text
    assert "'surface_estimate_min_points': 3" in launch_text
    assert "'collision_scan_topic': LaunchConfiguration('collision_scan_topic')" in launch_text
    assert (
        "DeclareLaunchArgument('terrain_goal_z_policy', default_value='nearest_z')"
    ) in launch_text
    assert "DeclareLaunchArgument('terrain_goal_max_z', default_value='-1.0')" in launch_text
    assert "'goal_z_policy': LaunchConfiguration('terrain_goal_z_policy')" in launch_text
    assert "'goal_min_z': LaunchConfiguration('terrain_goal_min_z')" in launch_text
    assert "'goal_max_z': LaunchConfiguration('terrain_goal_max_z')" in launch_text
    assert "'goal_snap_max_distance': 2.0" in launch_text
    assert "'frontier_replan_enabled': True" in launch_text
    assert "'frontier_min_path_distance': 0.25" in launch_text
    assert "'frontier_max_path_distance': 14.0" in launch_text
    assert "'frontier_obstacle_scan_topic': '/slam_scan'" in launch_text
    assert "'frontier_obstacle_clearance': 0.45" in launch_text
    assert "'frontier_obstacle_range_max': 3.0" in launch_text
    assert "'frontier_stall_timeout_sec': 8.0" in launch_text
    assert "'frontier_stall_min_progress': 0.20" in launch_text
    assert "'frontier_failed_clearance': 1.6" in launch_text
    assert "'frontier_goal_regression_tolerance': 1.5" in launch_text
    assert "'nav_execution_mode': LaunchConfiguration('terrain_execution_mode')" in launch_text
    assert "'nav_stack_mode': LaunchConfiguration('nav_stack_mode')" in launch_text
    assert "DeclareLaunchArgument('terrain_execution_mode', default_value='direct')" in launch_text
    assert "'grid_resolution': 0.25" in launch_text
    assert "'terrain_cloud_resolution': 0.10" in launch_text
    assert "'max_step_height': 0.50" in launch_text
    assert "'odom_topic': '/Odometry'" not in launch_text
    assert "'odom_topic': LaunchConfiguration('terrain_odom_topic')" in launch_text
    terrain_planner_section = launch_text.split(
        '    terrain_planner = Node('
    )[1].split('    return LaunchDescription([')[0]
    assert "'odom_topic': '/odom'" not in terrain_planner_section
    assert "'odom_topic': LaunchConfiguration('terrain_odom_topic')" in (
        terrain_planner_section
    )
    assert "'use_initial_pose_anchor': False" in launch_text
    assert "'direct_cmd_vel_topic': '/cmd_vel_nav'" in launch_text
    assert "'direct_waypoint_tolerance': 0.42" in launch_text
    assert "'direct_goal_tolerance': 0.30" in launch_text
    assert "'direct_z_tolerance': 0.45" in launch_text
    assert "'direct_max_linear_speed': 0.30" in launch_text
    assert "'direct_max_angular_speed': 0.45" in launch_text
    assert "'single_floor_complex_large_static.sdf'" in launch_text
    assert "'start_waypoint_clearance': 0.75" in launch_text
    assert "'follow_path_start_clearance': 0.12" in launch_text
    assert "'slope_speed_limit': 0.16" in launch_text
    assert "'flat_speed_limit': 0.32" in launch_text
    assert "'initial_surface_z_hint': LaunchConfiguration('robot_spawn_z')" in launch_text
    assert "DeclareLaunchArgument('robot_spawn_y', default_value='0.0')" in launch_text
    assert "DeclareLaunchArgument('robot_spawn_z', default_value='0.26')" in launch_text
    assert "DeclareLaunchArgument('robot_spawn_yaw', default_value='0.0')" in launch_text
    assert "'min_visible_z': 0.08" in launch_text
    assert "'max_points': 800000" in launch_text


def test_frontier_progress_is_committed_after_direct_tracking_reaches_goal() -> None:
    planner_text = _read_text(
        'src/airos_experiments/airos_experiments/terrain_pct_planner.py'
    )
    frontier_plan_section = planner_text.split(
        '    def _plan_frontier_toward_goal('
    )[1].split('    def _find_non_regressive_frontier_path(')[0]
    direct_reached_section = planner_text.split(
        '        if direct_tracking_reaches_goal('
    )[1].split('        self._advance_direct_target')[0]

    assert '_remember_frontier_progress' not in frontier_plan_section
    assert '_remember_frontier_progress(goal, self._pending_final_goal_xy)' in (
        direct_reached_section
    )


def test_direct_tracking_goal_reach_requires_original_final_goal() -> None:
    planner_text = _read_text(
        'src/airos_experiments/airos_experiments/terrain_pct_planner.py'
    )
    direct_tick_section = planner_text.split(
        '    def _direct_control_tick(self) -> None:'
    )[1].split('    def _maybe_log_direct_diagnostics(')[0]

    assert 'direct_tracking_reaches_goal(' in direct_tick_section
    assert 'final_goal_xy=self._direct_final_goal_xy' in direct_tick_section
    assert '_direct_node_reached(\n            goal,' not in direct_tick_section


def test_goal_callback_uses_pose_z_as_floor_aware_goal_constraint() -> None:
    planner_text = _read_text(
        'src/airos_experiments/airos_experiments/terrain_pct_planner.py'
    )
    goal_callback = planner_text.split(
        '    def _goal_callback(self, msg: PoseStamped) -> None:'
    )[1].split('    def _try_pending_final_goal(self) -> None:')[0]

    assert 'goal_z = float(msg.pose.position.z)' in goal_callback
    assert 'effective_goal_min_z' in goal_callback
    assert 'goal_min_z=effective_goal_min_z' in goal_callback
    assert 'target_z=' in goal_callback


def test_goal_callback_rejects_regressive_high_final_path_before_execution() -> None:
    planner_text = _read_text(
        'src/airos_experiments/airos_experiments/terrain_pct_planner.py'
    )
    goal_callback = planner_text.split(
        '    def _goal_callback(self, msg: PoseStamped) -> None:'
    )[1].split('    def _try_pending_final_goal(self) -> None:')[0]

    rejection_index = goal_callback.index('should_reject_regressive_final_path(')
    execution_index = goal_callback.index('_publish_and_execute_path(path, msg)')
    assert rejection_index < execution_index
    assert 'self._plan_frontier_toward_goal(' in (
        goal_callback[rejection_index:execution_index]
    )


def test_direct_stall_monitor_tracks_current_waypoint_not_path_endpoint() -> None:
    planner_text = _read_text(
        'src/airos_experiments/airos_experiments/terrain_pct_planner.py'
    )
    direct_tick_section = planner_text.split(
        '    def _direct_control_tick(self) -> None:'
    )[1].split('    def _release_stalled_frontier_if_needed(')[0]
    release_section = planner_text.split(
        '    def _release_stalled_frontier_if_needed('
    )[1].split('    def _reset_frontier_stall_monitor(')[0]

    assert direct_tick_section.index('self._advance_direct_target(') < (
        direct_tick_section.index('self._release_stalled_frontier_if_needed(')
    )
    assert 'tracking_goal = select_stall_tracking_goal(' in release_section
    assert 'active_frontier_path=self._active_frontier_path' in release_section
    assert 'direct_target_index=self._direct_target_index' in release_section


def test_sim_launch_defaults_to_native_gazebo_sensor_source() -> None:
    launch_text = _read_text('src/airos_sim/launch/sim.launch.py')
    visual_launch_text = _read_text(
        'src/airos_experiments/launch/visual_fast_lio_navigation.launch.py'
    )

    assert "DeclareLaunchArgument('sensor_source', default_value='native')" in launch_text
    assert "sensor_source = LaunchConfiguration('sensor_source')" in launch_text
    assert "native_sensor_enabled = sensor_source == 'native'" in launch_text
    assert "emulated_sensor_enabled = sensor_source == 'emulated'" in launch_text
    assert "not native_sensor_enabled" in launch_text
    assert (
        "ros_topic_name in {'/scan', '/livox/lidar', '/livox/lidar_points'}"
        in launch_text
    )
    assert "'lidar_topic': '/livox/lidar_points'" in launch_text
    assert "'sensor_source': LaunchConfiguration('sensor_source')" in visual_launch_text
    assert "DeclareLaunchArgument('sensor_source', default_value='native')" in visual_launch_text


def test_fast_lio_sim_profile_preserves_dense_building_pointclouds() -> None:
    fast_lio_config = yaml.safe_load(
        _read_text('src/fast_lio/config/airos_sim.yaml')
    )
    params = fast_lio_config['/**']['ros__parameters']

    assert params['point_filter_num'] == 1
    assert params['filter_size_surf'] == 0.16
    assert params['filter_size_map'] == 0.16
    assert params['publish']['dense_publish_en'] is False
    assert params['publish']['scan_publish_en'] is True


def test_native_gazebo_lidar_sensors_are_declared_on_robot() -> None:
    urdf_text = _read_text(
        'src/airos_go2w_description/urdf/go2w_nav_eq.urdf.xacro'
    )

    assert '<sensor name="nav_lidar_2d" type="gpu_lidar">' in urdf_text
    assert '<topic>/scan</topic>' in urdf_text
    assert '<horizontal><samples>720</samples>' in urdf_text
    assert '<vertical><samples>1</samples>' in urdf_text
    assert '<sensor name="fast_lio_lidar_3d" type="gpu_lidar">' in urdf_text
    assert '<topic>/livox/lidar</topic>' in urdf_text
    assert '<vertical><samples>16</samples>' in urdf_text
    assert '<gz_frame_id>livox_frame</gz_frame_id>' in urdf_text


def test_bridge_exports_native_gazebo_scan_and_pointcloud() -> None:
    bridge_config = yaml.safe_load(
        _read_text('src/airos_sim/config/ros_gz_bridge.yaml')
    )
    by_topic = {entry['topic_name']: entry for entry in bridge_config}
    by_ros_topic = {
        entry.get('ros_topic_name', entry['topic_name']): entry
        for entry in bridge_config
    }

    assert by_topic['/scan']['ros_type_name'] == 'sensor_msgs/msg/LaserScan'
    assert by_topic['/scan']['gz_type_name'] == 'ignition.msgs.LaserScan'
    assert by_topic['/scan']['direction'] == 'GZ_TO_ROS'

    livox_cloud = by_ros_topic['/livox/lidar_points']
    assert livox_cloud['topic_name'] == '/livox/lidar/points'
    assert livox_cloud['ros_type_name'] == 'sensor_msgs/msg/PointCloud2'
    assert livox_cloud['gz_type_name'] == 'ignition.msgs.PointCloudPacked'
    assert livox_cloud['direction'] == 'GZ_TO_ROS'


def test_bridge_remaps_native_pointcloud_to_fast_lio_topic() -> None:
    launch_text = _read_text('src/airos_sim/launch/sim.launch.py')

    assert 'bridge_remaps = []' in launch_text
    assert "entry.get('ros_topic_name', gz_topic_name)" in launch_text
    assert "entry.get('topic_name')" in launch_text
    assert "'--ros-args'" in launch_text
    assert "'-r'" in launch_text


def test_external_map_manager_can_be_disabled_for_fast_lio_launch() -> None:
    nav_launch_text = _read_text('src/airos_nav/launch/nav.launch.py')

    assert 'external_map_manager = LaunchConfiguration' in nav_launch_text
    assert "DeclareLaunchArgument('external_map_manager', default_value='true')" in nav_launch_text
    assert '_external_map_manager_enabled(localization, external_map_manager)' in nav_launch_text
    assert "collision_scan_topic = LaunchConfiguration('collision_scan_topic')" in nav_launch_text
    assert (
        "DeclareLaunchArgument('collision_scan_topic', default_value='/scan')"
    ) in nav_launch_text
    assert "{'scan.topic': collision_scan_topic}" in nav_launch_text


def test_visual_launches_use_safe_rviz_wrapper() -> None:
    launch_paths = [
        'src/airos_nav/launch/nav.launch.py',
        'src/airos_experiments/launch/visual_fast_lio_navigation.launch.py',
        'src/airos_experiments/launch/visual_navigation.launch.py',
        'src/airos_experiments/launch/visual_slam_mapping.launch.py',
    ]

    for launch_path in launch_paths:
        launch_text = _read_text(launch_path)
        assert "package='airos_experiments'" in launch_text
        assert "executable='rviz2_safe'" in launch_text


def test_visual_launches_do_not_force_global_hardware_opengl() -> None:
    launch_paths = [
        'src/airos_experiments/launch/visual_fast_lio_navigation.launch.py',
        'src/airos_experiments/launch/visual_navigation.launch.py',
        'src/airos_experiments/launch/visual_slam_mapping.launch.py',
    ]

    for launch_path in launch_paths:
        launch_text = _read_text(launch_path)
        assert "SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '0')" not in launch_text
        assert "SetEnvironmentVariable('__GL_SYNC_TO_VBLANK', '0')" not in launch_text
        assert "SetEnvironmentVariable('vblank_mode', '0')" not in launch_text


def test_sim_launch_defaults_gazebo_to_wsl_stable_rendering() -> None:
    launch_text = _read_text('src/airos_sim/launch/sim.launch.py')
    visual_launch_text = _read_text(
        'src/airos_experiments/launch/visual_fast_lio_navigation.launch.py'
    )
    world_text = _read_text('src/airos_sim/worlds/single_floor_lab.sdf')

    assert (
        "DeclareLaunchArgument('gazebo_rendering_mode', default_value='wsl_stable')"
        in launch_text
    )
    assert "DeclareLaunchArgument('robot_spawn_z', default_value='0.26')" in launch_text
    assert (
        "'gazebo_rendering_mode': LaunchConfiguration('gazebo_rendering_mode')"
        in visual_launch_text
    )
    assert "'robot_spawn_z': LaunchConfiguration('robot_spawn_z')" in visual_launch_text
    assert "'LIBGL_ALWAYS_SOFTWARE': '1'" in launch_text
    assert "'GALLIUM_DRIVER': 'llvmpipe'" in launch_text
    assert "'QT_QPA_PLATFORM': 'xcb'" in launch_text
    assert "'WAYLAND_DISPLAY': ''" in launch_text
    assert "'__GL_SYNC_TO_VBLANK': '1'" in launch_text
    assert "'vblank_mode': '1'" in launch_text
    assert '--render-engine ogre2' in launch_text
    assert '--render-engine-gui ogre2' in launch_text
    assert '<render_engine>ogre2</render_engine>' in world_text


def test_safe_rviz_wrapper_is_signal_isolated() -> None:
    wrapper = _read_text(
        'src/airos_experiments/airos_experiments/rviz2_safe.py'
    )

    assert 'start_new_session=True' in wrapper
    assert 'os.killpg(_child.pid, signal.SIGTERM)' in wrapper
    assert 'os.killpg(_child.pid, signal.SIGKILL)' in wrapper
    assert 'if _shutdown_requested:' in wrapper


def test_visual_navigation_does_not_publish_simulated_laser_map() -> None:
    launch_text = _read_text(
        'src/airos_experiments/launch/visual_navigation.launch.py'
    )

    assert "'pointcloud_map': 'false'" in launch_text


def test_nav_rviz_prefers_nav2_map_and_shows_aligned_slam_cloud() -> None:
    nav_map = _rviz_display('Nav2 Map /map')
    laser_map = _rviz_display('PointCloud Map /Laser_map')
    colorized_map = _rviz_display('Colorized PointCloud Map /Laser_map_colored')
    terrain_cloud = _rviz_display(
        'Terrain Traversability Cloud /terrain_traversability_cloud'
    )
    fast_lio_cost_scan = _rviz_display('FAST-LIO Cost Scan /slam_scan')
    dense_building_cloud = _rviz_display(
        'Dense Building Cloud /dense_visual_cloud'
    )
    registered_cloud = _rviz_display('Registered Cloud /cloud_registered')
    livox_cloud = _rviz_display('Livox Raw Cloud /livox/lidar_points')

    assert nav_map['Enabled'] is True
    assert nav_map['Value'] is True
    assert nav_map['Topic']['Value'] == '/map'

    assert laser_map['Enabled'] is False
    assert laser_map['Value'] is False
    assert laser_map['Style'] == 'Points'
    assert laser_map['Decay Time'] == 0

    assert colorized_map['Enabled'] is True
    assert colorized_map['Value'] is True
    assert colorized_map['Color Transformer'] == 'RGB8'
    assert colorized_map['Topic']['Value'] == '/Laser_map_colored'
    assert colorized_map['Decay Time'] == 0
    assert colorized_map['Style'] == 'Points'
    assert colorized_map['Size (Pixels)'] == 2
    assert colorized_map['Size (m)'] == 0.02

    assert terrain_cloud['Enabled'] is False
    assert terrain_cloud['Value'] is False
    assert terrain_cloud['Topic']['Value'] == '/terrain_traversability_cloud'
    assert terrain_cloud['Topic']['Durability Policy'] == 'Transient Local'
    assert terrain_cloud['Style'] == 'Points'
    assert terrain_cloud['Size (Pixels)'] == 2
    assert terrain_cloud['Size (m)'] == 0.02

    assert fast_lio_cost_scan['Enabled'] is True
    assert fast_lio_cost_scan['Value'] is True
    assert fast_lio_cost_scan['Topic']['Value'] == '/slam_scan'
    assert fast_lio_cost_scan['Color Transformer'] == 'FlatColor'
    assert fast_lio_cost_scan['Style'] == 'Flat Squares'

    assert dense_building_cloud['Enabled'] is False
    assert dense_building_cloud['Value'] is False
    assert dense_building_cloud['Topic']['Value'] == '/dense_visual_cloud'
    assert dense_building_cloud['Color Transformer'] == 'AxisColor'
    assert dense_building_cloud['Style'] == 'Points'
    assert dense_building_cloud['Size (Pixels)'] == 1
    assert dense_building_cloud['Size (m)'] == 0.012
    assert dense_building_cloud['Decay Time'] == 0

    assert registered_cloud['Enabled'] is False
    assert registered_cloud['Value'] is False
    assert registered_cloud['Color Transformer'] == 'AxisColor'
    assert registered_cloud['Style'] == 'Points'
    assert registered_cloud['Size (Pixels)'] == 2
    assert registered_cloud['Size (m)'] == 0.02
    assert registered_cloud['Decay Time'] == 0
    assert registered_cloud['Topic']['Depth'] == 1

    for live_cloud in (livox_cloud,):
        assert live_cloud['Enabled'] is False
        assert live_cloud['Value'] is False
        assert live_cloud['Style'] == 'Points'
        assert live_cloud['Decay Time'] == 0
        assert live_cloud['Topic']['Depth'] == 1


def test_nav_rviz_uses_nav2_goal_and_hides_pct_path_by_default() -> None:
    rviz_config = yaml.safe_load(_read_text('src/airos_nav/rviz/nav.rviz'))
    tools = rviz_config['Visualization Manager']['Tools']
    tool_classes = {tool['Class'] for tool in tools}
    terrain_path = _rviz_display('Terrain PCT Path /pct_path')
    dynamic_obstacles = _rviz_display('Dynamic Obstacles')

    assert 'nav2_rviz_plugins/GoalTool' in tool_classes
    assert 'rviz_default_plugins/SetGoal' not in tool_classes
    assert any(
        panel['Class'] == 'nav2_rviz_plugins/Navigation 2'
        for panel in rviz_config['Panels']
    )
    assert terrain_path['Enabled'] is False
    assert terrain_path['Value'] is False
    assert terrain_path['Topic']['Value'] == '/pct_path'
    assert dynamic_obstacles['Enabled'] is False
    assert dynamic_obstacles['Topic']['Value'] == '/dynamic_obstacles/markers'
    assert dynamic_obstacles['Value'] is False


def test_mppi_commands_fit_safe_velocity_chain() -> None:
    nav_params = yaml.safe_load(
        _read_text('src/airos_nav/config/nav2_params.yaml')
    )
    controller_params = nav_params['controller_server']['ros__parameters']
    follow_path = controller_params['FollowPath']
    smoother_params = nav_params['velocity_smoother']['ros__parameters']

    assert follow_path['plugin'] == 'nav2_mppi_controller::MPPIController'
    assert follow_path['motion_model'] == 'DiffDrive'
    assert follow_path['vx_max'] <= smoother_params['max_velocity'][0]
    assert follow_path['wz_max'] <= smoother_params['max_velocity'][2]
    assert follow_path['vx_min'] >= 0.0
    assert 'CostCritic' in follow_path['critics']
    assert 'PreferForwardCritic' in follow_path['critics']


def test_nav2_planner_defaults_to_smac2d_for_single_floor() -> None:
    nav_params = yaml.safe_load(
        _read_text('src/airos_nav/config/nav2_params.yaml')
    )
    bt_params = nav_params['bt_navigator']['ros__parameters']
    grid_based = nav_params['planner_server']['ros__parameters']['GridBased']
    global_costmap = nav_params['global_costmap']['global_costmap']['ros__parameters']
    local_costmap = nav_params['local_costmap']['local_costmap']['ros__parameters']
    static_layer = global_costmap['static_layer']
    global_obstacle_layer = global_costmap['obstacle_layer']
    local_obstacle_layer = local_costmap['obstacle_layer']

    assert bt_params['bt_loop_duration'] == 50
    assert bt_params['default_server_timeout'] == 1000
    assert bt_params['default_cancel_timeout'] == 1000
    assert grid_based['plugin'] == 'nav2_smac_planner/SmacPlanner2D'
    assert grid_based['allow_unknown'] is True
    assert grid_based['use_final_approach_orientation'] is False
    assert global_costmap['track_unknown_space'] is False
    assert static_layer['map_subscribe_transient_local'] is True
    assert static_layer['subscribe_to_updates'] is True
    assert global_obstacle_layer['observation_sources'] == (
        'scan fast_lio_slam_scan'
    )
    assert local_obstacle_layer['observation_sources'] == (
        'scan fast_lio_slam_scan'
    )
    assert global_obstacle_layer['fast_lio_slam_scan']['topic'] == '/slam_scan'
    assert local_obstacle_layer['fast_lio_slam_scan']['topic'] == '/slam_scan'
    assert global_obstacle_layer['fast_lio_slam_scan']['marking'] is True
    assert global_obstacle_layer['fast_lio_slam_scan']['clearing'] is False


def test_airos_nav_depends_on_mppi_and_smac_planner() -> None:
    package_xml = _read_text('src/airos_nav/package.xml')

    assert 'nav2_mppi_controller' in package_xml
    assert 'nav2_smac_planner' in package_xml


def test_fast_lio_sim_extrinsic_matches_republished_livox_imu_frame() -> None:
    fast_lio_params = yaml.safe_load(
        _read_text('src/fast_lio/config/airos_sim.yaml')
    )['/**']['ros__parameters']
    sim_launch = _read_text('src/airos_sim/launch/sim.launch.py')

    assert "'output_topic': '/livox/imu'" in sim_launch
    assert "'frame_id': 'livox_frame'" in sim_launch
    assert fast_lio_params['common']['imu_topic'] == '/livox/imu'
    assert fast_lio_params['mapping']['extrinsic_est_en'] is False
    assert fast_lio_params['mapping']['extrinsic_T'] == [0.0, 0.0, 0.0]
    assert fast_lio_params['mapping']['extrinsic_R'] == [
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    ]


def test_fast_lio_shutdown_uses_rclcpp_signal_handling() -> None:
    source = _read_text('src/fast_lio/src/laserMapping.cpp')

    assert 'signal(SIGINT, SigHandle)' not in source
    assert 'if (fp != nullptr)' in source
