import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_launch = os.path.join(
        get_package_share_directory('airos_sim'),
        'launch',
        'sim.launch.py',
    )
    mapping_launch = os.path.join(
        get_package_share_directory('airos_slam'),
        'launch',
        'mapping.launch.py',
    )
    rviz_config = os.path.join(
        get_package_share_directory('airos_nav'),
        'rviz',
        'nav.rviz',
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'rviz': 'false',
            'dynamic_obstacles': 'false',
            'pointcloud': 'true',
            'gazebo_rendering_mode': LaunchConfiguration('gazebo_rendering_mode'),
        }.items(),
    )
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_launch),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )
    rviz = Node(
        package='airos_experiments',
        executable='rviz2_safe',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('gazebo_rendering_mode', default_value='wsl_stable'),
        sim,
        TimerAction(period=8.0, actions=[mapping]),
        TimerAction(period=10.0, actions=[rviz]),
    ])
