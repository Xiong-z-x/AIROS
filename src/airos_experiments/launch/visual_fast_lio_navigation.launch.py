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
    pkg_nav = get_package_share_directory('airos_nav')
    pkg_sim = get_package_share_directory('airos_sim')
    sim_launch = os.path.join(
        pkg_sim,
        'launch',
        'sim.launch.py',
    )
    nav_launch = os.path.join(
        pkg_nav,
        'launch',
        'nav.launch.py',
    )
    fast_lio_config = os.path.join(
        get_package_share_directory('fast_lio'),
        'config',
        'airos_sim.yaml',
    )
    rviz_config = os.path.join(
        pkg_nav,
        'rviz',
        'nav.rviz',
    )

    sim = GroupAction(
        scoped=True,
        launch_configurations={
            'gui': LaunchConfiguration('gui'),
            'rviz': 'false',
            'world': LaunchConfiguration('world'),
            'dynamic_obstacles': LaunchConfiguration('dynamic_obstacles'),
            'physical_dynamic_obstacles': LaunchConfiguration(
                'physical_dynamic_obstacles'
            ),
            'open_source_scene_assets': LaunchConfiguration(
                'open_source_scene_assets'
            ),
            'robot_visual_profile': LaunchConfiguration('robot_visual_profile'),
            'sensor_source': LaunchConfiguration('sensor_source'),
            'pointcloud': 'true',
            'pointcloud_registered': 'false',
            'pointcloud_map': 'false',
            'point_spacing': LaunchConfiguration('pointcloud_spacing'),
            'max_live_points': LaunchConfiguration('max_live_points'),
            'gazebo_rendering_mode': LaunchConfiguration('gazebo_rendering_mode'),
            'robot_spawn_x': LaunchConfiguration('robot_spawn_x'),
            'robot_spawn_y': LaunchConfiguration('robot_spawn_y'),
            'robot_spawn_z': LaunchConfiguration('robot_spawn_z'),
            'robot_spawn_yaw': LaunchConfiguration('robot_spawn_yaw'),
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
            'map': LaunchConfiguration('map'),
            'route_graph': LaunchConfiguration('route_graph'),
            'planner_profile': LaunchConfiguration('planner_profile'),
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

    pointcloud_colorizer = Node(
        condition=IfCondition(LaunchConfiguration('colorized_pointcloud')),
        package='airos_experiments',
        executable='pointcloud_colorizer',
        name='laser_map_colorizer',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/Laser_map',
            'output_topic': '/Laser_map_colored',
            'min_z': -0.40,
            'max_z': 2.20,
            'min_visible_z': 0.08,
            'max_points': 220000,
        }],
    )

    terrain_planner = Node(
        condition=IfCondition(LaunchConfiguration('terrain_planner')),
        package='airos_experiments',
        executable='terrain_pct_planner',
        name='terrain_pct_planner',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'world_file': LaunchConfiguration('terrain_world_file'),
            'world_frame': 'map',
            'odom_topic': '/odom',
            'goal_topic': '/goal_pose',
            'path_topic': '/pct_path',
            'terrain_cloud_topic': '/terrain_traversability_cloud',
            'grid_resolution': 0.40,
            'robot_radius': 0.35,
            'support_margin': 0.45,
            'max_slope_grade': 0.58,
            'max_step_height': 0.36,
            'goal_z_policy': 'highest',
            'send_nav2_goals': LaunchConfiguration('terrain_send_nav2_goals'),
            'waypoint_spacing': 0.90,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('world', default_value='realistic_multilevel_ramp'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                pkg_nav,
                'maps',
                'realistic_multilevel_ramp.yaml',
            ),
        ),
        DeclareLaunchArgument(
            'route_graph',
            default_value=os.path.join(
                pkg_nav,
                'routes',
                'realistic_multilevel_ramp_route.geojson',
            ),
        ),
        DeclareLaunchArgument('use_route', default_value='true'),
        DeclareLaunchArgument('planner_profile', default_value='baseline'),
        DeclareLaunchArgument('dynamic_obstacles', default_value='false'),
        DeclareLaunchArgument('physical_dynamic_obstacles', default_value='false'),
        DeclareLaunchArgument('open_source_scene_assets', default_value='false'),
        DeclareLaunchArgument('robot_visual_profile', default_value='analytic'),
        DeclareLaunchArgument('log_level', default_value='warn'),
        DeclareLaunchArgument('sensor_source', default_value='native'),
        DeclareLaunchArgument('gazebo_rendering_mode', default_value='wsl_stable'),
        DeclareLaunchArgument('colorized_pointcloud', default_value='true'),
        DeclareLaunchArgument('pointcloud_spacing', default_value='0.12'),
        DeclareLaunchArgument('max_live_points', default_value='22000'),
        DeclareLaunchArgument('terrain_planner', default_value='true'),
        DeclareLaunchArgument(
            'terrain_world_file',
            default_value=os.path.join(
                pkg_sim,
                'worlds',
                'realistic_multilevel_ramp.sdf',
            ),
        ),
        DeclareLaunchArgument('terrain_send_nav2_goals', default_value='true'),
        DeclareLaunchArgument('robot_spawn_x', default_value='0.0'),
        DeclareLaunchArgument('robot_spawn_y', default_value='0.0'),
        DeclareLaunchArgument('robot_spawn_z', default_value='0.26'),
        DeclareLaunchArgument('robot_spawn_yaw', default_value='0.0'),
        sim,
        TimerAction(period=10.0, actions=[map_to_fast_lio_map, fast_lio]),
        TimerAction(period=15.0, actions=[nav]),
        TimerAction(period=24.0, actions=[lifecycle_activator]),
        TimerAction(period=26.0, actions=[pointcloud_colorizer]),
        TimerAction(period=28.0, actions=[terrain_planner]),
        TimerAction(period=32.0, actions=[rviz]),
    ])
