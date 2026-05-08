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
            'dynamic_obstacles': LaunchConfiguration('dynamic_obstacles'),
            'dynamic_obstacle_seed': LaunchConfiguration(
                'dynamic_obstacle_seed'
            ),
            'pointcloud': 'true',
            'pointcloud_registered': 'true',
            'pointcloud_map': 'false',
            'gazebo_rendering_mode': LaunchConfiguration('gazebo_rendering_mode'),
        },
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
            ),
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
        DeclareLaunchArgument('dynamic_obstacles', default_value='false'),
        DeclareLaunchArgument('dynamic_obstacle_seed', default_value='1'),
        DeclareLaunchArgument('log_level', default_value='warn'),
        DeclareLaunchArgument('gazebo_rendering_mode', default_value='wsl_stable'),
        sim,
        TimerAction(period=10.0, actions=[nav]),
        TimerAction(period=24.0, actions=[rviz]),
    ])
