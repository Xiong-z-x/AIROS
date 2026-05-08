import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_launch = os.path.join(
        get_package_share_directory('airos_sim'),
        'launch',
        'sim.launch.py',
    )
    nav_launch = os.path.join(
        get_package_share_directory('airos_nav'),
        'launch',
        'nav.launch.py',
    )
    fast_lio_config = os.path.join(
        get_package_share_directory('fast_lio'),
        'config',
        'airos_sim.yaml',
    )
    rviz_config = os.path.join(
        get_package_share_directory('airos_nav'),
        'rviz',
        'nav.rviz',
    )

    sim = GroupAction(
        scoped=True,
        launch_configurations={
            'gui': LaunchConfiguration('gui'),
            'rviz': 'false',
            'dynamic_obstacles': 'false',
            'pointcloud': 'true',
            'pointcloud_registered': 'false',
            'pointcloud_map': 'false',
            'gazebo_rendering_mode': LaunchConfiguration('gazebo_rendering_mode'),
        },
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
            ),
        ],
    )

    fast_lio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[fast_lio_config, {'use_sim_time': True}],
    )

    lifecycle_activator = Node(
        package='airos_experiments',
        executable='lifecycle_activator',
        name='fast_lio_lifecycle_activator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'node_names': ['/map_server', '/route_server'],
            'attempts': 10,
            'service_timeout_sec': 3.0,
            'poll_period_sec': 1.0,
        }],
    )

    map_to_fast_lio_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_fast_lio_map',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'fast_lio_map',
        ],
    )

    nav = GroupAction(
        scoped=True,
        launch_configurations={
            'rviz': 'false',
            'use_route': LaunchConfiguration('use_route'),
            'localization': 'static',
            'log_level': LaunchConfiguration('log_level'),
        },
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_launch),
            ),
        ],
    )

    rviz = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='airos_experiments',
        executable='rviz2_safe',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_route', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='warn'),
        DeclareLaunchArgument('gazebo_rendering_mode', default_value='wsl_stable'),
        sim,
        TimerAction(period=10.0, actions=[map_to_fast_lio_map, fast_lio]),
        TimerAction(period=15.0, actions=[nav]),
        TimerAction(period=24.0, actions=[lifecycle_activator]),
        TimerAction(period=32.0, actions=[rviz]),
    ])
