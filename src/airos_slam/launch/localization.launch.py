import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    posegraph = LaunchConfiguration('posegraph').perform(context)
    params = [
        LaunchConfiguration('params_file'),
        {'use_sim_time': LaunchConfiguration('use_sim_time')},
    ]
    if posegraph:
        params.append({'map_file_name': posegraph})

    return [
        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=params,
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
        )
    ]


def generate_launch_description():
    pkg_slam = get_package_share_directory('airos_slam')
    pkg_nav = get_package_share_directory('airos_nav')
    default_params = os.path.join(pkg_slam, 'config', 'slam_toolbox_localization.yaml')
    default_posegraph = os.path.join(pkg_nav, 'maps', 'single_floor_lab_slam')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('posegraph', default_value=default_posegraph),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
