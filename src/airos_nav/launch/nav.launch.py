import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def _is(value: LaunchConfiguration, expected: str) -> PythonExpression:
    return PythonExpression(["'", value, "' == '", expected, "'"])


def _route_manager_enabled(
    use_route: LaunchConfiguration,
    localization: LaunchConfiguration,
) -> PythonExpression:
    return PythonExpression([
        "'",
        use_route,
        "'.lower() == 'true' and '",
        localization,
        "' != 'external'",
    ])


def _external_map_manager_enabled(
    localization: LaunchConfiguration,
    external_map_manager: LaunchConfiguration,
) -> PythonExpression:
    return PythonExpression([
        "'",
        localization,
        "' == 'external' and '",
        external_map_manager,
        "'.lower() == 'true'",
    ])


def generate_launch_description():
    pkg_nav = get_package_share_directory('airos_nav')
    pkg_slam = get_package_share_directory('airos_slam')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    route_graph = LaunchConfiguration('route_graph')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('rviz')
    localization = LaunchConfiguration('localization')
    use_route = LaunchConfiguration('use_route')
    external_map_manager = LaunchConfiguration('external_map_manager')
    log_level = LaunchConfiguration('log_level')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={
                'use_sim_time': use_sim_time,
                'yaml_filename': map_file,
                'graph_filepath': route_graph,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    navigation_lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    amcl = Node(
        condition=IfCondition(_is(localization, 'amcl')),
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    localization_manager = Node(
        condition=IfCondition(_is(localization, 'amcl')),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['map_server', 'amcl']},
        ],
    )

    slam_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_slam, 'launch', 'localization.launch.py')),
        condition=IfCondition(_is(localization, 'slam_toolbox')),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    map_only_manager = Node(
        condition=IfCondition(_is(localization, 'slam_toolbox')),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['map_server']},
        ],
    )

    static_map_to_odom = Node(
        condition=IfCondition(_is(localization, 'static')),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
    )

    static_map_manager = Node(
        condition=IfCondition(_is(localization, 'static')),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_static_map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['map_server']},
        ],
    )

    external_localization_map_manager = Node(
        condition=IfCondition(
            _external_map_manager_enabled(localization, external_map_manager)
        ),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_external_map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['map_server']},
        ],
    )

    navigation_nodes = GroupAction([
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel_smoothed')],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': autostart},
                {'node_names': navigation_lifecycle_nodes},
            ],
        ),
    ])

    collision_monitor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_collision_monitor'), 'launch', 'collision_monitor_node.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items(),
    )

    route_server = GroupAction(
        condition=IfCondition(use_route),
        actions=[
            Node(
                package='nav2_route',
                executable='route_server',
                name='route_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                condition=IfCondition(
                    _route_manager_enabled(use_route, localization)
                ),
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_route',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': ['route_server']},
                ],
            ),
        ],
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package='airos_experiments',
        executable='rviz2_safe',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_nav, 'rviz', 'nav.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg_nav, 'maps', 'single_floor_lab.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg_nav, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('route_graph', default_value=os.path.join(pkg_nav, 'routes', 'single_floor_lab_route.geojson')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('localization', default_value='amcl'),
        DeclareLaunchArgument('use_route', default_value='false'),
        DeclareLaunchArgument('external_map_manager', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        map_server,
        amcl,
        localization_manager,
        slam_localization,
        map_only_manager,
        static_map_to_odom,
        static_map_manager,
        external_localization_map_manager,
        navigation_nodes,
        collision_monitor,
        route_server,
        rviz,
    ])
