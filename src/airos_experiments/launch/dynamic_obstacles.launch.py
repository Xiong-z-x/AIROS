import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_launch = os.path.join(
        get_package_share_directory('airos_sim'),
        'launch',
        'sim.launch.py',
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
            'dynamic_obstacles': LaunchConfiguration('dynamic_obstacles'),
            'dynamic_obstacle_seed': LaunchConfiguration('dynamic_obstacle_seed'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('dynamic_obstacles', default_value='false'),
        DeclareLaunchArgument('dynamic_obstacle_seed', default_value='1'),
        sim,
    ])
