import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _gazebo_env_for_rendering_mode(rendering_mode: str) -> dict[str, str]:
    if rendering_mode == 'hardware':
        return {
            'LIBGL_ALWAYS_SOFTWARE': '0',
            '__GL_SYNC_TO_VBLANK': '0',
            'vblank_mode': '0',
        }
    if rendering_mode == 'wsl_stable':
        return {
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'GALLIUM_DRIVER': 'llvmpipe',
            'QT_QPA_PLATFORM': 'xcb',
            'WAYLAND_DISPLAY': '',
            '__GL_SYNC_TO_VBLANK': '1',
            'vblank_mode': '1',
        }
    return {}


def _gazebo_actions(world_file: str, gui: bool, rendering_mode: str):
    gz_args = (
        f'-r --render-engine ogre2 --render-engine-gui ogre2 {world_file}'
        if gui
        else f'-r -s --render-engine ogre2 {world_file}'
    )
    actions = [
        SetEnvironmentVariable(name=name, value=value)
        for name, value in _gazebo_env_for_rendering_mode(rendering_mode).items()
    ]
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py',
                )
            ),
            launch_arguments={'gz_args': gz_args}.items(),
        )
    )
    return GroupAction(scoped=True, actions=actions)


def _resource_path_with(existing_value: str, *paths: str) -> str:
    entries = []
    for path in paths:
        if path and path not in entries:
            entries.append(path)
    for path in existing_value.split(os.pathsep):
        if path and path not in entries:
            entries.append(path)
    return os.pathsep.join(entries)


def _launch_setup(context, *args, **kwargs):
    pkg_sim = get_package_share_directory('airos_sim')
    pkg_desc = get_package_share_directory('airos_go2w_description')
    pkg_control = get_package_share_directory('airos_control')
    pkg_go2_desc = get_package_share_directory('unitree_go2_description')
    pkg_go2_sim = get_package_share_directory('unitree_go2_sim')

    gui = LaunchConfiguration('gui').perform(context).lower() in {'true', '1', 'yes'}
    world_name = LaunchConfiguration('world').perform(context)
    world_files = {
        'single_floor_lab': 'single_floor_lab.sdf',
        'advanced_indoor_ramp': 'advanced_indoor_ramp.sdf',
        'realistic_multilevel_ramp': 'realistic_multilevel_ramp.sdf',
        'large_multilevel_complex': 'large_multilevel_complex.sdf',
    }
    static_world_files = {
        'advanced_indoor_ramp': 'advanced_indoor_ramp_static.sdf',
        'realistic_multilevel_ramp': 'realistic_multilevel_ramp_static.sdf',
        'large_multilevel_complex': 'large_multilevel_complex_static.sdf',
    }
    if world_name not in world_files:
        raise RuntimeError(
            'world must be one of '
            f"{sorted(world_files)}, got {world_name!r}"
        )
    dynamic_obstacles = (
        LaunchConfiguration('dynamic_obstacles').perform(context).lower()
        in {'true', '1', 'yes'}
    )
    point_spacing = float(LaunchConfiguration('point_spacing').perform(context))
    max_live_points = int(LaunchConfiguration('max_live_points').perform(context))
    physical_dynamic_obstacles = (
        LaunchConfiguration('physical_dynamic_obstacles').perform(context).lower()
        in {'true', '1', 'yes'}
    )
    open_source_scene_assets = (
        LaunchConfiguration('open_source_scene_assets').perform(context).lower()
        in {'true', '1', 'yes'}
    )
    robot_visual_profile = LaunchConfiguration('robot_visual_profile').perform(context)
    if robot_visual_profile not in {'analytic', 'reference_mesh'}:
        raise RuntimeError(
            "robot_visual_profile must be 'analytic' or 'reference_mesh', "
            f"got {robot_visual_profile!r}"
        )
    robot_mobility_profile = LaunchConfiguration('robot_mobility_profile').perform(context)
    if robot_mobility_profile not in {'wheeled', 'legged_champ'}:
        raise RuntimeError(
            "robot_mobility_profile must be 'wheeled' or 'legged_champ', "
            f"got {robot_mobility_profile!r}"
        )
    sensor_source = LaunchConfiguration('sensor_source').perform(context).lower()
    if sensor_source not in {'native', 'emulated'}:
        raise RuntimeError(
            "sensor_source must be 'native' or 'emulated', "
            f"got {sensor_source!r}"
        )
    native_sensor_enabled = sensor_source == 'native'
    emulated_sensor_enabled = sensor_source == 'emulated'
    dynamic_obstacle_seed = int(LaunchConfiguration('dynamic_obstacle_seed').perform(context))
    pointcloud_enabled = (
        LaunchConfiguration('pointcloud').perform(context).lower()
        in {'true', '1', 'yes'}
    )
    pointcloud_registered = (
        LaunchConfiguration('pointcloud_registered').perform(context).lower()
        in {'true', '1', 'yes'}
    )
    pointcloud_map = (
        LaunchConfiguration('pointcloud_map').perform(context).lower()
        in {'true', '1', 'yes'}
    )
    gazebo_rendering_mode = LaunchConfiguration('gazebo_rendering_mode').perform(context)
    robot_spawn_x = LaunchConfiguration('robot_spawn_x').perform(context)
    robot_spawn_y = LaunchConfiguration('robot_spawn_y').perform(context)
    robot_spawn_z = LaunchConfiguration('robot_spawn_z').perform(context)
    robot_spawn_yaw = LaunchConfiguration('robot_spawn_yaw').perform(context)
    legged_cmd_vel_topic = LaunchConfiguration('legged_cmd_vel_topic').perform(context)

    world_filename = (
        world_files[world_name]
        if physical_dynamic_obstacles
        else static_world_files.get(world_name, world_files[world_name])
    )
    world_file = os.path.join(pkg_sim, 'worlds', world_filename)
    bridge_config = os.path.join(pkg_sim, 'config', 'ros_gz_bridge.yaml')
    wheeled_xacro_file = os.path.join(pkg_desc, 'urdf', 'go2w_nav_eq.urdf.xacro')
    wheeled_controller_yaml = os.path.join(pkg_control, 'config', 'go2w_controllers.yaml')
    go2_xacro_file = os.path.join(
        pkg_go2_desc,
        'urdf',
        'unitree_go2_robot.xacro',
    )
    go2_controller_yaml = os.path.join(
        pkg_go2_sim,
        'config',
        'ros_control',
        'ros_control.yaml',
    )
    rviz_config = os.path.join(pkg_desc, 'rviz', 'model.rviz')

    if robot_mobility_profile == 'legged_champ':
        robot_name = 'unitree_go2_legged'
        robot_description = xacro.process_file(
            go2_xacro_file,
            mappings={'robot_controllers': go2_controller_yaml},
        ).toxml()
    else:
        robot_name = 'go2w_nav_eq'
        robot_description = xacro.process_file(
            wheeled_xacro_file,
            mappings={
                'controller_config': wheeled_controller_yaml,
                'visual_profile': robot_visual_profile,
            },
        ).toxml()

    bridge_specs = []
    bridge_remaps = []
    with open(bridge_config, 'r', encoding='utf-8') as stream:
        bridge_entries = yaml.safe_load(stream) or []
    for entry in bridge_entries:
        direction = str(entry.get('direction', 'BIDIRECTIONAL')).upper()
        gz_topic_name = str(entry.get('topic_name'))
        ros_topic_name = str(entry.get('ros_topic_name', gz_topic_name))
        if (
            not native_sensor_enabled
            and ros_topic_name in {'/scan', '/livox/lidar', '/livox/lidar_points'}
        ):
            continue
        if direction == 'GZ_TO_ROS':
            bridge_specs.append(
                f"{gz_topic_name}@{entry['ros_type_name']}[{entry['gz_type_name']}"
            )
        elif direction == 'ROS_TO_GZ':
            bridge_specs.append(
                f"{ros_topic_name}@{entry['ros_type_name']}]"
                f"{entry['gz_type_name']}"
            )
        else:
            bridge_specs.append(
                f"{gz_topic_name}@{entry['ros_type_name']}@{entry['gz_type_name']}"
            )
        if ros_topic_name != gz_topic_name:
            bridge_remaps.extend(['--ros-args', '-r', f'{gz_topic_name}:={ros_topic_name}'])

    gazebo = _gazebo_actions(world_file, gui, gazebo_rendering_mode)
    gz_resource_path = _resource_path_with(
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        pkg_sim,
        pkg_desc,
        pkg_go2_desc,
        os.path.dirname(pkg_sim),
        os.path.dirname(pkg_desc),
        os.path.dirname(pkg_go2_desc),
    )
    ign_resource_path = _resource_path_with(
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
        pkg_sim,
        pkg_desc,
        pkg_go2_desc,
        os.path.dirname(pkg_sim),
        os.path.dirname(pkg_desc),
        os.path.dirname(pkg_go2_desc),
    )

    spawn_open_source_building = Node(
        condition=IfCondition(LaunchConfiguration('open_source_scene_assets')),
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'open_source_building_reference',
            '-file', os.path.join(
                pkg_sim,
                'models',
                'open_source_building',
                'model.sdf',
            ),
            '-x', '-7.5',
            '-y', '5.5',
            '-z', '0.02',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', robot_name,
            '-param', 'robot_description',
            '-x', robot_spawn_x,
            '-y', robot_spawn_y,
            '-z', robot_spawn_z,
            '-R', '0.0',
            '-P', '0.0',
            '-Y', robot_spawn_yaw,
        ],
        parameters=[{'robot_description': robot_description}],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=bridge_specs + bridge_remaps,
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_control, 'launch', 'control.launch.py')
        ),
        launch_arguments={
            'robot_mobility_profile': robot_mobility_profile,
        }.items(),
    )

    go2_joints_config = os.path.join(pkg_go2_sim, 'config', 'joints', 'joints.yaml')
    go2_links_config = os.path.join(pkg_go2_sim, 'config', 'links', 'links.yaml')
    go2_gait_config = os.path.join(pkg_go2_sim, 'config', 'gait', 'gait.yaml')
    legged_quadruped_controller = Node(
        condition=IfCondition(LaunchConfiguration('legged_champ_controller')),
        package='champ_base',
        executable='quadruped_controller_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'gazebo': True},
            {'publish_joint_states': True},
            {'publish_joint_control': True},
            {'publish_foot_contacts': False},
            {
                'joint_controller_topic': (
                    'joint_group_effort_controller/joint_trajectory'
                )
            },
            {'urdf': robot_description},
            go2_joints_config,
            go2_links_config,
            go2_gait_config,
            {'hardware_connected': False},
            {'close_loop_odom': True},
        ],
        remappings=[('/cmd_vel/smooth', legged_cmd_vel_topic)],
    )

    legged_state_estimator = Node(
        condition=IfCondition(LaunchConfiguration('legged_champ_controller')),
        package='champ_base',
        executable='state_estimation_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'orientation_from_imu': True},
            {'urdf': robot_description},
            go2_joints_config,
            go2_links_config,
            go2_gait_config,
        ],
    )

    scan_emulator = Node(
        package='airos_experiments',
        executable='scan_emulator',
        name='scan_emulator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'world_file': world_file,
            'odom_topic': '/odom',
            'scan_topic': '/scan',
            'scan_frame': 'lidar_link',
            'sample_count': 360,
            'publish_rate_hz': 6.0,
            'dynamic_obstacles_enabled': dynamic_obstacles,
            'dynamic_obstacle_seed': dynamic_obstacle_seed,
            'include_dynamic_models': physical_dynamic_obstacles,
        }],
    )

    dynamic_marker_emulator = Node(
        package='airos_experiments',
        executable='scan_emulator',
        name='dynamic_obstacle_marker_emulator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'world_file': world_file,
            'odom_topic': '/odom',
            'scan_topic': '/scan_dynamic_overlay',
            'scan_frame': 'lidar_link',
            'sample_count': 72,
            'publish_rate_hz': 2.0,
            'dynamic_obstacles_enabled': True,
            'dynamic_obstacle_seed': dynamic_obstacle_seed,
            'include_dynamic_models': physical_dynamic_obstacles,
        }],
    )

    pointcloud_emulator = Node(
        package='airos_experiments',
        executable='pointcloud_emulator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'world_file': world_file,
            'odom_topic': '/odom',
            'lidar_topic': '/livox/lidar_points',
            'registered_cloud_topic': '/cloud_registered',
            'map_cloud_topic': '/Laser_map',
            'publish_registered_cloud': pointcloud_registered,
            'publish_map_cloud': pointcloud_map,
            'lidar_frame': 'livox_frame',
            'world_frame': 'map',
            'publish_rate_hz': 3.0,
            'map_publish_rate_hz': 0.25,
            'range_max': 12.0,
            'point_spacing': point_spacing,
            'max_live_points': max_live_points,
            'include_dynamic_models': physical_dynamic_obstacles,
        }],
    )

    livox_imu_relay = Node(
        package='airos_experiments',
        executable='imu_republisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/imu',
            'output_topic': '/livox/imu',
            'frame_id': 'livox_frame',
        }],
    )

    livox_custom_bridge = Node(
        package='airos_experiments',
        executable='livox_custom_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/livox/lidar_points',
            'output_topic': '/livox/lidar',
            'scan_line': 16,
            'scan_period_us': 100000,
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}],
    )

    delayed_sensor_nodes = []
    if emulated_sensor_enabled:
        delayed_sensor_nodes.append(scan_emulator)
        if pointcloud_enabled:
            delayed_sensor_nodes.append(pointcloud_emulator)
    elif dynamic_obstacles:
        delayed_sensor_nodes.append(dynamic_marker_emulator)
    if pointcloud_enabled:
        delayed_sensor_nodes.append(livox_custom_bridge)
    if pointcloud_enabled:
        delayed_sensor_nodes.append(livox_imu_relay)

    dynamic_obstacle_triggers = []
    if physical_dynamic_obstacles:
        dynamic_obstacle_triggers.append(
            TimerAction(
                period=7.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            'ign',
                            'topic',
                            '-t',
                            f'/airos/{world_name}/start_dynamic_obstacles',
                            '-m',
                            'ignition.msgs.Empty',
                            '-p',
                            ' ',
                        ],
                        output='screen',
                    ),
                ],
            )
        )

    return [
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=gz_resource_path,
        ),
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=ign_resource_path,
        ),
        gazebo,
        robot_state_publisher,
        TimerAction(period=2.0, actions=[spawn_robot]),
        TimerAction(
            period=2.5,
            actions=[spawn_open_source_building] if open_source_scene_assets else [],
        ),
        TimerAction(period=3.0, actions=[bridge]),
        TimerAction(period=4.0, actions=[control]),
        TimerAction(
            period=5.0,
            actions=(
                [legged_quadruped_controller, legged_state_estimator]
                if robot_mobility_profile == 'legged_champ'
                else []
            ),
        ),
        TimerAction(
            period=6.0,
            actions=delayed_sensor_nodes,
        ),
        *dynamic_obstacle_triggers,
        rviz,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('world', default_value='single_floor_lab'),
        DeclareLaunchArgument('sensor_source', default_value='native'),
        DeclareLaunchArgument('dynamic_obstacles', default_value='false'),
        DeclareLaunchArgument('physical_dynamic_obstacles', default_value='false'),
        DeclareLaunchArgument('open_source_scene_assets', default_value='false'),
        DeclareLaunchArgument('robot_mobility_profile', default_value='legged_champ'),
        DeclareLaunchArgument('legged_champ_controller', default_value='true'),
        DeclareLaunchArgument('legged_cmd_vel_topic', default_value='/cmd_vel_champ'),
        DeclareLaunchArgument('robot_visual_profile', default_value='analytic'),
        DeclareLaunchArgument('dynamic_obstacle_seed', default_value='0'),
        DeclareLaunchArgument('pointcloud', default_value='true'),
        DeclareLaunchArgument('pointcloud_registered', default_value='true'),
        DeclareLaunchArgument('pointcloud_map', default_value='true'),
        DeclareLaunchArgument('point_spacing', default_value='0.06'),
        DeclareLaunchArgument('max_live_points', default_value='180000'),
        DeclareLaunchArgument('gazebo_rendering_mode', default_value='wsl_stable'),
        DeclareLaunchArgument('robot_spawn_x', default_value='0.0'),
        DeclareLaunchArgument('robot_spawn_y', default_value='0.0'),
        DeclareLaunchArgument('robot_spawn_z', default_value='0.375'),
        DeclareLaunchArgument('robot_spawn_yaw', default_value='0.0'),
        OpaqueFunction(function=_launch_setup),
    ])
