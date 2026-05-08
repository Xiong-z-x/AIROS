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
    assert "'localization': 'static'" in launch_text
    assert 'fast_lio_localization_bridge' not in launch_text
    assert 'static_map_to_odom' not in launch_text


def test_external_map_manager_can_be_disabled_for_fast_lio_launch() -> None:
    nav_launch_text = _read_text('src/airos_nav/launch/nav.launch.py')

    assert 'external_map_manager = LaunchConfiguration' in nav_launch_text
    assert "DeclareLaunchArgument('external_map_manager', default_value='true')" in nav_launch_text
    assert '_external_map_manager_enabled(localization, external_map_manager)' in nav_launch_text


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

    assert "DeclareLaunchArgument('gazebo_rendering_mode', default_value='wsl_stable')" in launch_text
    assert "'gazebo_rendering_mode': LaunchConfiguration('gazebo_rendering_mode')" in visual_launch_text
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


def test_nav_rviz_keeps_live_clouds_single_frame_and_disabled() -> None:
    laser_map = _rviz_display('PointCloud Map /Laser_map')
    registered_cloud = _rviz_display('Registered Cloud /cloud_registered')
    livox_cloud = _rviz_display('Livox Cloud /livox/lidar')

    assert laser_map['Enabled'] is True
    assert laser_map['Style'] == 'Points'
    assert laser_map['Decay Time'] == 0

    for live_cloud in (registered_cloud, livox_cloud):
        assert live_cloud['Enabled'] is False
        assert live_cloud['Value'] is False
        assert live_cloud['Style'] == 'Points'
        assert live_cloud['Decay Time'] == 0
        assert live_cloud['Topic']['Depth'] == 1


def test_rotation_shim_over_rpp_commands_fit_safe_velocity_chain() -> None:
    nav_params = yaml.safe_load(
        _read_text('src/airos_nav/config/nav2_params.yaml')
    )
    controller_params = nav_params['controller_server']['ros__parameters']
    follow_path = controller_params['FollowPath']
    smoother_params = nav_params['velocity_smoother']['ros__parameters']

    assert follow_path['plugin'] == (
        'nav2_rotation_shim_controller::RotationShimController'
    )
    assert follow_path['primary_controller'] == (
        'nav2_regulated_pure_pursuit_controller::'
        'RegulatedPurePursuitController'
    )
    assert follow_path['desired_linear_vel'] <= smoother_params['max_velocity'][0]
    assert (
        follow_path['rotate_to_heading_angular_vel']
        <= smoother_params['max_velocity'][2]
    )
    assert follow_path['allow_reversing'] is False
    assert follow_path['use_rotate_to_heading'] is False


def test_airos_nav_depends_on_pure_pursuit_controller_not_mppi() -> None:
    package_xml = _read_text('src/airos_nav/package.xml')

    assert 'nav2_regulated_pure_pursuit_controller' in package_xml
    assert 'nav2_mppi_controller' not in package_xml


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
