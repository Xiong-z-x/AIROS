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
            'point_spacing': LaunchConfiguration('fast_lio_pointcloud_spacing'),
            'max_live_points': LaunchConfiguration('fast_lio_max_live_points'),
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
        condition=IfCondition(LaunchConfiguration('use_route')),
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
            'nav_stack_mode': LaunchConfiguration('nav_stack_mode'),
            'external_map_manager': 'false',
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
            'max_points': 800000,
        }],
    )

    dense_building_cloud = Node(
        condition=IfCondition(LaunchConfiguration('dense_visual_pointcloud')),
        package='airos_experiments',
        executable='pointcloud_emulator',
        name='dense_building_pointcloud',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'world_file': LaunchConfiguration('terrain_world_file'),
            'odom_topic': '/odom',
            'lidar_topic': '/dense_visual_cloud_local',
            'registered_cloud_topic': '/dense_visual_cloud',
            'map_cloud_topic': '/dense_visual_cloud_map',
            'publish_registered_cloud': True,
            'publish_map_cloud': False,
            'lidar_frame': 'livox_frame',
            'world_frame': 'map',
            'publish_rate_hz': 1.0,
            'map_publish_rate_hz': 0.10,
            'range_max': 30.0,
            'point_spacing': LaunchConfiguration('pointcloud_spacing'),
            'max_live_points': LaunchConfiguration('max_live_points'),
            'include_dynamic_models': LaunchConfiguration('physical_dynamic_obstacles'),
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
            'goal_topic': '/terrain_goal_pose',
            'path_topic': '/pct_path',
            'terrain_cloud_topic': '/terrain_traversability_cloud',
            'terrain_map_source': LaunchConfiguration('terrain_map_source'),
            'slam_map_topic': '/Laser_map',
            'slam_map_max_points': LaunchConfiguration('slam_map_max_points'),
            'slam_grid_resolution': LaunchConfiguration('slam_grid_resolution'),
            'slam_min_cell_points': LaunchConfiguration('slam_min_cell_points'),
            'slam_vertical_layer_gap': LaunchConfiguration('slam_vertical_layer_gap'),
            'slam_rebuild_period_sec': LaunchConfiguration('slam_rebuild_period_sec'),
            'grid_resolution': 0.25,
            'terrain_cloud_resolution': 0.10,
            'robot_radius': 0.35,
            'support_margin': 0.45,
            'max_slope_grade': 0.58,
            'max_step_height': 0.36,
            'goal_z_policy': 'adaptive',
            'goal_snap_max_distance': 1.0,
            'send_nav2_goals': LaunchConfiguration('terrain_send_nav2_goals'),
            'nav_execution_mode': LaunchConfiguration('terrain_execution_mode'),
            'direct_cmd_vel_topic': '/cmd_vel_nav',
            'direct_lookahead_dist': 0.45,
            'direct_waypoint_tolerance': 0.24,
            'direct_goal_tolerance': 0.30,
            'direct_heading_gain': 1.4,
            'direct_max_linear_speed': 0.16,
            'direct_min_linear_speed': 0.035,
            'direct_max_angular_speed': 0.28,
            'direct_max_heading_error_for_forward': 1.25,
            'waypoint_spacing': 0.90,
            'start_waypoint_clearance': 0.75,
            'follow_path_start_clearance': 0.12,
            'slope_speed_limit': 0.09,
            'flat_speed_limit': 0.18,
            'slope_speed_grade_threshold': 0.08,
            'initial_surface_z_hint': LaunchConfiguration('robot_spawn_z'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('world', default_value='large_multilevel_complex'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                pkg_nav,
                'maps',
                'large_multilevel_complex.yaml',
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
        DeclareLaunchArgument('use_route', default_value='false'),
        DeclareLaunchArgument('planner_profile', default_value='baseline'),
        DeclareLaunchArgument('dynamic_obstacles', default_value='false'),
        DeclareLaunchArgument('physical_dynamic_obstacles', default_value='false'),
        DeclareLaunchArgument('open_source_scene_assets', default_value='false'),
        DeclareLaunchArgument('robot_visual_profile', default_value='analytic'),
        DeclareLaunchArgument('log_level', default_value='warn'),
        DeclareLaunchArgument('sensor_source', default_value='native'),
        DeclareLaunchArgument('gazebo_rendering_mode', default_value='wsl_stable'),
        DeclareLaunchArgument('colorized_pointcloud', default_value='true'),
        DeclareLaunchArgument('dense_visual_pointcloud', default_value='true'),
        DeclareLaunchArgument('pointcloud_spacing', default_value='0.06'),
        DeclareLaunchArgument('max_live_points', default_value='180000'),
        DeclareLaunchArgument('fast_lio_pointcloud_spacing', default_value='0.16'),
        DeclareLaunchArgument('fast_lio_max_live_points', default_value='12000'),
        DeclareLaunchArgument('terrain_map_source', default_value='slam_cloud'),
        DeclareLaunchArgument('slam_map_max_points', default_value='80000'),
        DeclareLaunchArgument('slam_grid_resolution', default_value='0.25'),
        DeclareLaunchArgument('slam_min_cell_points', default_value='2'),
        DeclareLaunchArgument('slam_vertical_layer_gap', default_value='0.18'),
        DeclareLaunchArgument('slam_rebuild_period_sec', default_value='3.0'),
        DeclareLaunchArgument('terrain_planner', default_value='true'),
        DeclareLaunchArgument(
            'terrain_world_file',
            default_value=os.path.join(
                pkg_sim,
                'worlds',
                'large_multilevel_complex_static.sdf',
            ),
        ),
        DeclareLaunchArgument('terrain_send_nav2_goals', default_value='true'),
        DeclareLaunchArgument('terrain_execution_mode', default_value='direct'),
        DeclareLaunchArgument('nav_stack_mode', default_value='safety_only'),
        DeclareLaunchArgument('robot_spawn_x', default_value='0.0'),
        DeclareLaunchArgument('robot_spawn_y', default_value='-10.0'),
        DeclareLaunchArgument('robot_spawn_z', default_value='0.26'),
        DeclareLaunchArgument('robot_spawn_yaw', default_value='-1.5708'),
        sim,
        TimerAction(period=10.0, actions=[map_to_fast_lio_map, fast_lio]),
        TimerAction(period=15.0, actions=[nav]),
        TimerAction(
            period=24.0,
            actions=[lifecycle_activator],
            condition=IfCondition(LaunchConfiguration('use_route')),
        ),
        TimerAction(period=26.0, actions=[pointcloud_colorizer]),
        TimerAction(period=26.0, actions=[dense_building_cloud]),
        TimerAction(period=28.0, actions=[terrain_planner]),
        TimerAction(period=32.0, actions=[rviz]),
    ])
