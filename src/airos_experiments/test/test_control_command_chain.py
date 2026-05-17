from __future__ import annotations

from pathlib import Path

import yaml


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _read_text(relative_path: str) -> str:
    return (_repo_root() / relative_path).read_text(encoding='utf-8')


def test_control_launch_has_no_cmd_vel_bypass_to_base_controller() -> None:
    launch_text = _read_text('src/airos_control/launch/control.launch.py')

    assert "'/cmd_vel'" not in launch_text
    assert "'/diff_drive_controller/cmd_vel_unstamped'" not in launch_text
    assert 'cmd_vel_relay' not in launch_text


def test_control_launch_can_spawn_champ_legged_controllers() -> None:
    launch_text = _read_text('src/airos_control/launch/control.launch.py')
    ros_control = yaml.safe_load(
        _read_text('src/unitree_go2_sim/config/ros_control/ros_control.yaml')
    )
    controller_params = ros_control['controller_manager']['ros__parameters']

    assert "robot_mobility_profile must be 'wheeled' or 'legged_champ'" in launch_text
    assert "robot_mobility_profile == 'legged_champ'" in launch_text
    assert 'joint_states_controller' in launch_text
    assert 'joint_group_effort_controller' in launch_text
    assert controller_params['update_rate'] == 100
    assert (
        controller_params['joint_group_effort_controller']['type']
        == 'joint_trajectory_controller/JointTrajectoryController'
    )
    joints = ros_control['joint_group_effort_controller']['ros__parameters']['joints']
    assert len(joints) == 12
    assert all(
        f'{leg}_{joint}_joint' in joints
        for leg in ['lf', 'rf', 'lh', 'rh']
        for joint in ['hip', 'upper_leg', 'lower_leg']
    )


def test_nav_launch_lifecycle_manages_collision_monitor_inline() -> None:
    launch_text = _read_text('src/airos_nav/launch/nav.launch.py')

    assert 'collision_monitor_node.launch.py' not in launch_text
    assert "package='nav2_collision_monitor'" in launch_text
    assert "executable='collision_monitor'" in launch_text
    assert "name='lifecycle_manager_collision_monitor'" in launch_text
    assert "condition=IfCondition(_full_stack_enabled(nav_stack_mode))" in launch_text
    assert "'node_names': ['collision_monitor']" in launch_text


def test_nav_launch_can_run_controller_only_for_pct_execution() -> None:
    launch_text = _read_text('src/airos_nav/launch/nav.launch.py')

    assert "DeclareLaunchArgument('nav_stack_mode', default_value='full')" in launch_text
    assert (
        "nav_stack_mode must be 'full', 'controller_only' or 'safety_only'"
        in launch_text
    )
    assert "name='controller_only_lifecycle_activator'" in launch_text
    assert "name='safety_only_lifecycle_activator'" in launch_text
    assert "'controller_server'," in launch_text
    assert "'velocity_smoother'," in launch_text
    assert "'collision_monitor'," in launch_text
    assert "'node_names': ['velocity_smoother', 'collision_monitor']" in launch_text
    assert "condition=IfCondition(_full_stack_enabled(nav_stack_mode))" in launch_text


def test_lifecycle_activator_retries_service_availability() -> None:
    source = _read_text(
        'src/airos_experiments/airos_experiments/lifecycle_activator.py'
    )

    activate_node = source.split('    def _activate_node(')[1].split(
        '    def _wait_for_lifecycle_services('
    )[0]
    wait_helper = source.split('    def _wait_for_lifecycle_services(')[1].split(
        '    def _get_state('
    )[0]

    assert '_wait_for_lifecycle_services(' in activate_node
    assert 'for _ in range(max(self._attempts, 1)):' in wait_helper
    assert 'get_state.wait_for_service' in wait_helper
    assert 'change_state.wait_for_service' in wait_helper
    assert 'return True' in wait_helper


def test_cleanup_script_stops_fast_lio_scan_projector() -> None:
    cleanup_script = _read_text('scripts/cleanup_airos_runtime.sh')

    assert "'slam_scan_projector'" in cleanup_script
    assert 'slam_scan_projector' in cleanup_script.split("leftover_pattern=")[1]


def test_cleanup_script_stops_nav2_map_saver_leftovers() -> None:
    cleanup_script = _read_text('scripts/cleanup_airos_runtime.sh')

    assert "'nav2_map_server/map_saver_server'" in cleanup_script
    assert 'nav2_' in cleanup_script.split("leftover_pattern=")[1]


def test_cleanup_script_does_not_kill_its_calling_shell_by_pattern() -> None:
    cleanup_script = _read_text('scripts/cleanup_airos_runtime.sh')

    assert 'protected_pids' in cleanup_script
    assert 'pgrep -af "$pattern"' in cleanup_script
    assert 'pkill -f' not in cleanup_script


def test_fast_lio_single_floor_demo_script_uses_physical_odom_acceptance() -> None:
    demo_script = _read_text('scripts/run_fast_lio_single_floor_demo.sh')

    assert 'cleanup_airos_runtime.sh' in demo_script
    assert 'DEMO_TARGET="${DEMO_TARGET:-near_goal}"' in demo_script
    assert 'near_goal)' in demo_script
    assert 'long_corridor)' in demo_script
    assert 'GOAL_X="${GOAL_X:-1.9}"' in demo_script
    assert 'GOAL_X="${GOAL_X:-8.0}"' in demo_script
    assert 'GOAL_Y="${GOAL_Y:--9.0}"' in demo_script
    assert 'visual_fast_lio_navigation.launch.py' in demo_script
    assert 'terrain_goal_z_policy:=nearest_z' in demo_script
    assert 'terrain_goal_min_z:=-1.0' in demo_script
    assert 'TERRAIN_GOAL_MAX_Z="${TERRAIN_GOAL_MAX_Z:-0.45}"' in demo_script
    assert 'terrain_goal_max_z:="$TERRAIN_GOAL_MAX_Z"' in demo_script
    assert 'TERRAIN_MAX_STEP_HEIGHT="${TERRAIN_MAX_STEP_HEIGHT:-0.50}"' in demo_script
    assert (
        'TERRAIN_DIRECT_LOOKAHEAD_DIST="${TERRAIN_DIRECT_LOOKAHEAD_DIST:-0.45}"'
    ) in demo_script
    assert (
        'TERRAIN_DIRECT_WAYPOINT_TOLERANCE="${TERRAIN_DIRECT_WAYPOINT_TOLERANCE:-0.42}"'
    ) in demo_script
    assert (
        'TERRAIN_FOLLOW_PATH_START_CLEARANCE="${TERRAIN_FOLLOW_PATH_START_CLEARANCE:-0.12}"'
    ) in demo_script
    assert 'terrain_max_step_height:="$TERRAIN_MAX_STEP_HEIGHT"' in demo_script
    assert (
        'terrain_direct_lookahead_dist:="$TERRAIN_DIRECT_LOOKAHEAD_DIST"'
    ) in demo_script
    assert (
        'terrain_direct_waypoint_tolerance:="$TERRAIN_DIRECT_WAYPOINT_TOLERANCE"'
    ) in demo_script
    assert (
        'terrain_follow_path_start_clearance:="$TERRAIN_FOLLOW_PATH_START_CLEARANCE"'
    ) in demo_script
    assert 'ROBOT_MOBILITY_PROFILE="${ROBOT_MOBILITY_PROFILE:-legged_champ}"' in demo_script
    assert 'LEGGED_CHAMP_CONTROLLER="${LEGGED_CHAMP_CONTROLLER:-true}"' in demo_script
    assert 'LEGGED_CMD_VEL_TOPIC="${LEGGED_CMD_VEL_TOPIC:-/cmd_vel_champ}"' in demo_script
    assert (
        'CMD_VEL_OUT_TOPIC="${CMD_VEL_OUT_TOPIC:-/cmd_vel_champ}"'
    ) in demo_script
    assert 'robot_mobility_profile:="$ROBOT_MOBILITY_PROFILE"' in demo_script
    assert 'legged_champ_controller:="$LEGGED_CHAMP_CONTROLLER"' in demo_script
    assert 'legged_cmd_vel_topic:="$LEGGED_CMD_VEL_TOPIC"' in demo_script
    assert 'cmd_vel_out_topic:="$CMD_VEL_OUT_TOPIC"' in demo_script
    assert 'terrain_odom_topic:=/odom' in demo_script
    assert 'cross_level_evidence_probe' in demo_script
    assert '--times "$GOAL_PUBLISH_COUNT"' in demo_script
    assert '--rate "$GOAL_PUBLISH_RATE_HZ"' in demo_script
    assert 'wheel_goal_xy_distance' in demo_script
    assert 'gazebo_goal_xy_distance' in demo_script
    assert 'planner_received_goal' in demo_script
    assert 'planner_started_direct_tracking' in demo_script
    assert 'planner_reached_goal' in demo_script
    assert 'direct_diagnostics_seen' in demo_script
    assert 'ACCEPTANCE_TOLERANCE_M' in demo_script
    assert 'MAX_LOG_RUNS_TO_KEEP' in demo_script
    assert 'rm -rf --' in demo_script


def test_gazebo_bridge_does_not_accept_direct_cmd_vel() -> None:
    bridge_entries = yaml.safe_load(
        _read_text('src/airos_sim/config/ros_gz_bridge.yaml')
    )

    bridged_ros_topics = {
        entry.get('ros_topic_name') or entry.get('topic_name')
        for entry in bridge_entries
    }
    assert '/cmd_vel' not in bridged_ros_topics


def test_base_controller_limits_match_nav2_safe_chain() -> None:
    controllers = yaml.safe_load(
        _read_text('src/airos_control/config/go2w_controllers.yaml')
    )
    robot_model = _read_text(
        'src/airos_go2w_description/urdf/go2w_nav_eq.urdf.xacro'
    )
    diff_drive = controllers['diff_drive_controller']['ros__parameters']

    assert diff_drive['wheel_separation'] == 0.72
    assert '<xacro:property name="wheel_y" value="0.36"/>' in robot_model
    assert '<xacro:property name="base_z" value="0.18"/>' in robot_model
    assert '<xacro:property name="body_mass" value="14.0"/>' in robot_model
    assert '<mu1>3.0</mu1>' in robot_model
    assert '<mu2>3.0</mu2>' in robot_model
    assert 0.30 <= diff_drive['linear']['x']['max_velocity'] <= 0.32
    assert diff_drive['linear']['x']['min_velocity'] >= 0.0
    assert 0.20 <= diff_drive['linear']['x']['max_acceleration'] <= 0.22
    assert 0.50 <= diff_drive['angular']['z']['max_velocity'] <= 0.55
    assert diff_drive['angular']['z']['min_velocity'] >= -0.55
    assert 0.50 <= diff_drive['angular']['z']['max_acceleration'] <= 0.55


def test_sim_odom_is_gazebo_truth_not_wheel_integrator() -> None:
    control_launch = _read_text('src/airos_control/launch/control.launch.py')
    controllers = yaml.safe_load(
        _read_text('src/airos_control/config/go2w_controllers.yaml')
    )
    bridge_entries = yaml.safe_load(
        _read_text('src/airos_sim/config/ros_gz_bridge.yaml')
    )
    robot_model = _read_text(
        'src/airos_go2w_description/urdf/go2w_nav_eq.urdf.xacro'
    )

    diff_drive = controllers['diff_drive_controller']['ros__parameters']

    assert 'odom_relay' not in control_launch
    assert "'/diff_drive_controller/odom'" not in control_launch
    assert diff_drive['enable_odom_tf'] is False
    assert 'ignition::gazebo::systems::OdometryPublisher' in robot_model
    assert '<odom_frame>odom</odom_frame>' in robot_model
    assert '<robot_base_frame>base_footprint</robot_base_frame>' in robot_model
    assert '<dimensions>2</dimensions>' in robot_model
    assert '<odom_topic>/odom</odom_topic>' in robot_model
    assert '<tf_topic>/tf</tf_topic>' in robot_model

    assert any(
        entry.get('topic_name') == '/odom'
        and entry.get('ros_type_name') == 'nav_msgs/msg/Odometry'
        and entry.get('gz_type_name') == 'ignition.msgs.Odometry'
        and entry.get('direction') == 'GZ_TO_ROS'
        for entry in bridge_entries
    )
    assert any(
        entry.get('topic_name') == '/tf'
        and entry.get('ros_type_name') == 'tf2_msgs/msg/TFMessage'
        and entry.get('gz_type_name') == 'ignition.msgs.Pose_V'
        and entry.get('direction') == 'GZ_TO_ROS'
        for entry in bridge_entries
    )


def test_legged_go2_description_uses_champ_frames_and_fortress_plugins() -> None:
    sim_launch = _read_text('src/airos_sim/launch/sim.launch.py')
    visual_launch = _read_text(
        'src/airos_experiments/launch/visual_fast_lio_navigation.launch.py'
    )
    nav_launch = _read_text('src/airos_nav/launch/nav.launch.py')
    go2_urdf = _read_text(
        'src/unitree_go2_description/urdf/unitree_go2_robot.xacro'
    )
    go2_gazebo = _read_text(
        'src/unitree_go2_description/urdf/unitree_go2_gazebo.xacro'
    )
    gait = yaml.safe_load(_read_text('src/unitree_go2_sim/config/gait/gait.yaml'))
    gait_params = gait['/**']['ros__parameters']['gait']

    assert "robot_mobility_profile', default_value='legged_champ'" in sim_launch
    assert "legged_champ_controller', default_value='true'" in sim_launch
    assert "robot_mobility_profile must be 'wheeled' or 'legged_champ'" in sim_launch
    assert "robot_name = 'unitree_go2_legged'" in sim_launch
    assert "executable='quadruped_controller_node'" in sim_launch
    assert "executable='state_estimation_node'" in sim_launch
    assert (
        "legged_cmd_vel_topic = LaunchConfiguration('legged_cmd_vel_topic').perform(context)"
        in sim_launch
    )
    assert "remappings=[('/cmd_vel/smooth', legged_cmd_vel_topic)]" in sim_launch
    assert "'legged_cmd_vel_topic': LaunchConfiguration('legged_cmd_vel_topic')" in visual_launch
    assert "DeclareLaunchArgument('legged_cmd_vel_topic', default_value='/cmd_vel_champ')" in visual_launch
    assert "'cmd_vel_out_topic': LaunchConfiguration('cmd_vel_out_topic')" in visual_launch
    assert "'cmd_vel_out_topic': cmd_vel_out_topic" in nav_launch
    assert 'name="base_footprint"' in go2_urdf
    assert 'name="base_footprint_joint"' in go2_urdf
    assert '<xacro:arg name="enable_d455" default="false"/>' in go2_urdf
    assert '<xacro:if value="$(arg enable_d455)">' in go2_urdf
    assert 'ignition::gazebo::systems::OdometryPublisher' in go2_gazebo
    assert '<robot_base_frame>base_footprint</robot_base_frame>' in go2_gazebo
    assert gait_params['max_linear_velocity_x'] == 0.3
    assert gait_params['max_angular_velocity_z'] == 0.5
    assert gait_params['swing_height'] == 0.04
    assert gait_params['nominal_height'] == 0.225


def test_nav2_uses_rotation_shim_over_conservative_pure_pursuit() -> None:
    nav_params = yaml.safe_load(
        _read_text('src/airos_nav/config/nav2_params.yaml')
    )
    controller = nav_params['controller_server']['ros__parameters']
    follow_path = controller['FollowPath']
    progress_checker = controller['progress_checker']
    smoother = nav_params['velocity_smoother']['ros__parameters']
    collision_monitor = nav_params['collision_monitor']['ros__parameters']

    assert controller['controller_plugins'] == ['FollowPath']
    assert follow_path['plugin'] == (
        'nav2_rotation_shim_controller::RotationShimController'
    )
    assert follow_path['primary_controller'] == (
        'nav2_regulated_pure_pursuit_controller::'
        'RegulatedPurePursuitController'
    )
    assert 0.22 <= follow_path['desired_linear_vel'] <= 0.24
    assert follow_path['use_rotate_to_heading'] is False
    assert follow_path['allow_reversing'] is False
    assert 0.55 <= follow_path['angular_dist_threshold'] <= 0.9
    assert (
        0.25 <= follow_path['angular_disengage_threshold']
        < follow_path['angular_dist_threshold']
    )
    assert 0.30 <= smoother['max_velocity'][0] <= 0.32
    assert smoother['min_velocity'][0] >= 0.0
    assert 0.20 <= smoother['max_accel'][0] <= 0.22
    assert 0.12 <= progress_checker['required_movement_radius'] <= 0.25
    assert progress_checker['movement_time_allowance'] >= 18.0
    assert follow_path['use_collision_detection'] is False
    assert follow_path['cost_scaling_dist'] <= 0.35
    assert collision_monitor['StopZone']['max_points'] >= 5
    assert collision_monitor['SlowZone']['max_points'] >= 4
    assert collision_monitor['SlowZone']['slowdown_ratio'] >= 0.55


def test_nav2_uses_base_footprint_for_ground_pose_control() -> None:
    nav_params = yaml.safe_load(
        _read_text('src/airos_nav/config/nav2_params.yaml')
    )

    assert nav_params['amcl']['ros__parameters']['base_frame_id'] == 'base_footprint'
    assert nav_params['bt_navigator']['ros__parameters']['robot_base_frame'] == 'base_footprint'
    assert (
        nav_params['local_costmap']['local_costmap']['ros__parameters']['robot_base_frame']
        == 'base_footprint'
    )
    assert (
        nav_params['global_costmap']['global_costmap']['ros__parameters']['robot_base_frame']
        == 'base_footprint'
    )
    assert nav_params['behavior_server']['ros__parameters']['robot_base_frame'] == 'base_footprint'
    assert nav_params['collision_monitor']['ros__parameters']['base_frame_id'] == 'base_footprint'
