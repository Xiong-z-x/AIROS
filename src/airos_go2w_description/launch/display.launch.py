import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('airos_go2w_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'go2w_nav_eq.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'model.rviz')
    controller_yaml = os.path.join(
        get_package_share_directory('airos_control'),
        'config',
        'go2w_controllers.yaml',
    )

    robot_description = xacro.process_file(
        xacro_file,
        mappings={
            'controller_config': controller_yaml,
            'visual_profile': LaunchConfiguration('visual_profile').perform(
                context
            ),
        },
    ).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
        output='screen',
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return [
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('visual_profile', default_value='analytic'),
        OpaqueFunction(function=_launch_setup),
    ])
