from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def _launch_setup(context, *args, **kwargs):
    robot_mobility_profile = LaunchConfiguration('robot_mobility_profile').perform(
        context
    )
    if robot_mobility_profile not in {'wheeled', 'legged_champ'}:
        raise RuntimeError(
            "robot_mobility_profile must be 'wheeled' or 'legged_champ', "
            f"got {robot_mobility_profile!r}"
        )

    if robot_mobility_profile == 'legged_champ':
        return [
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    '--controller-manager-timeout',
                    '120',
                    'joint_states_controller',
                    '--controller-manager',
                    '/controller_manager',
                ],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    '--controller-manager-timeout',
                    '120',
                    'joint_group_effort_controller',
                    '--controller-manager',
                    '/controller_manager',
                ],
                output='screen',
            ),
        ]

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )

    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )

    return [joint_state_broadcaster, diff_drive_controller]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_mobility_profile', default_value='legged_champ'),
        OpaqueFunction(function=_launch_setup),
    ])
