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
    diff_drive = controllers['diff_drive_controller']['ros__parameters']

    assert diff_drive['linear']['x']['max_velocity'] <= 0.24
    assert diff_drive['linear']['x']['min_velocity'] >= 0.0
    assert diff_drive['angular']['z']['max_velocity'] <= 0.55
    assert diff_drive['angular']['z']['min_velocity'] >= -0.55


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


def test_nav2_uses_rotation_shim_over_conservative_pure_pursuit() -> None:
    nav_params = yaml.safe_load(
        _read_text('src/airos_nav/config/nav2_params.yaml')
    )
    controller = nav_params['controller_server']['ros__parameters']
    follow_path = controller['FollowPath']
    smoother = nav_params['velocity_smoother']['ros__parameters']

    assert controller['controller_plugins'] == ['FollowPath']
    assert follow_path['plugin'] == (
        'nav2_rotation_shim_controller::RotationShimController'
    )
    assert follow_path['primary_controller'] == (
        'nav2_regulated_pure_pursuit_controller::'
        'RegulatedPurePursuitController'
    )
    assert follow_path['desired_linear_vel'] <= 0.24
    assert follow_path['use_rotate_to_heading'] is False
    assert follow_path['allow_reversing'] is False
    assert 0.55 <= follow_path['angular_dist_threshold'] <= 0.9
    assert 0.25 <= follow_path['angular_disengage_threshold'] < follow_path['angular_dist_threshold']
    assert smoother['max_velocity'][0] <= 0.24
    assert smoother['min_velocity'][0] >= 0.0


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
