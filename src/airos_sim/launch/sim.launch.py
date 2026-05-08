import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
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
    gz_args = f"-r --render-engine ogre2 --render-engine-gui ogre2 {world_file}" if gui else f"-r -s --render-engine ogre2 {world_file}"
    actions = [
        SetEnvironmentVariable(name=name, value=value)
        for name, value in _gazebo_env_for_rendering_mode(rendering_mode).items()
    ]
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': gz_args}.items(),
        )
    )
    return GroupAction(scoped=True, actions=actions)


def _launch_setup(context, *args, **kwargs):
    pkg_sim = get_package_share_directory('airos_sim')
    pkg_desc = get_package_share_directory('airos_go2w_description')
    pkg_control = get_package_share_directory('airos_control')

    gui = LaunchConfiguration('gui').perform(context).lower() in {'true', '1', 'yes'}
    dynamic_obstacles = (
        LaunchConfiguration('dynamic_obstacles').perform(context).lower()
        in {'true', '1', 'yes'}
    )
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

    world_file = os.path.join(pkg_sim, 'worlds', 'single_floor_lab.sdf')
    bridge_config = os.path.join(pkg_sim, 'config', 'ros_gz_bridge.yaml')
    xacro_file = os.path.join(pkg_desc, 'urdf', 'go2w_nav_eq.urdf.xacro')
    controller_yaml = os.path.join(pkg_control, 'config', 'go2w_controllers.yaml')
    rviz_config = os.path.join(pkg_desc, 'rviz', 'model.rviz')

    robot_description = xacro.process_file(
        xacro_file,
        mappings={'controller_config': controller_yaml},
    ).toxml()

    bridge_specs = []
    with open(bridge_config, 'r', encoding='utf-8') as stream:
        bridge_entries = yaml.safe_load(stream) or []
    for entry in bridge_entries:
        direction = str(entry.get('direction', 'BIDIRECTIONAL')).upper()
        if direction == 'GZ_TO_ROS':
            bridge_specs.append(
                f"{entry['topic_name']}@{entry['ros_type_name']}[{entry['gz_type_name']}"
            )
        elif direction == 'ROS_TO_GZ':
            bridge_specs.append(
                f"{entry['ros_topic_name']}@{entry['ros_type_name']}]"
                f"{entry['gz_type_name']}"
            )
        else:
            bridge_specs.append(
                f"{entry['topic_name']}@{entry['ros_type_name']}@{entry['gz_type_name']}"
            )

    gazebo = _gazebo_actions(world_file, gui, gazebo_rendering_mode)

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
            '-name', 'go2w_nav_eq',
            '-param', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.24',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
        ],
        parameters=[{'robot_description': robot_description}],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=bridge_specs,
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_control, 'launch', 'control.launch.py')
        ),
    )

    scan_emulator = Node(
        package='airos_experiments',
        executable='scan_emulator',
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
            'lidar_topic': '/livox/lidar',
            'registered_cloud_topic': '/cloud_registered',
            'map_cloud_topic': '/Laser_map',
            'publish_registered_cloud': pointcloud_registered,
            'publish_map_cloud': pointcloud_map,
            'lidar_frame': 'livox_frame',
            'world_frame': 'map',
            'publish_rate_hz': 3.0,
            'map_publish_rate_hz': 0.25,
            'range_max': 12.0,
            'point_spacing': 0.35,
            'max_live_points': 3200,
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

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}],
    )

    delayed_sensor_nodes = [scan_emulator]
    if pointcloud_enabled:
        delayed_sensor_nodes.extend([pointcloud_emulator, livox_imu_relay])

    return [
        gazebo,
        robot_state_publisher,
        TimerAction(period=2.0, actions=[spawn_robot]),
        TimerAction(period=3.0, actions=[bridge]),
        TimerAction(period=4.0, actions=[control]),
        TimerAction(
            period=6.0,
            actions=delayed_sensor_nodes,
        ),
        rviz,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('dynamic_obstacles', default_value='false'),
        DeclareLaunchArgument('dynamic_obstacle_seed', default_value='0'),
        DeclareLaunchArgument('pointcloud', default_value='true'),
        DeclareLaunchArgument('pointcloud_registered', default_value='true'),
        DeclareLaunchArgument('pointcloud_map', default_value='true'),
        DeclareLaunchArgument('gazebo_rendering_mode', default_value='wsl_stable'),
        OpaqueFunction(function=_launch_setup),
    ])
