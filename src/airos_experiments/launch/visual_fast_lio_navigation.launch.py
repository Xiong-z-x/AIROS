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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _fast_lio_localization_enabled() -> PythonExpression:
    return PythonExpression([
        "'",
        LaunchConfiguration('fast_lio_debug'),
        "'.lower() == 'true' and '",
        LaunchConfiguration('localization'),
        "' != 'slam_toolbox_mapping'",
    ])


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
            'collision_scan_topic': LaunchConfiguration('collision_scan_topic'),
            'external_map_manager': 'false',
            'localization': LaunchConfiguration('localization'),
            'log_level': LaunchConfiguration('log_level'),
            'slam_nav_startup': LaunchConfiguration('slam_nav_startup'),
        },
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_launch),
            ),
        ],
    )

    fast_lio_localization_bridge = Node(
        package='airos_experiments',
        executable='fast_lio_localization_bridge',
        name='fast_lio_localization_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'fast_lio_odom_topic': '/Odometry',
            'wheel_odom_topic': '/odom',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'aligned_odom_topic': '/fast_lio_odom_world',
            'publish_rate_hz': 20.0,
            'max_source_age_sec': 0.8,
            'spawn_x': LaunchConfiguration('robot_spawn_x'),
            'spawn_y': LaunchConfiguration('robot_spawn_y'),
            'spawn_z': LaunchConfiguration('robot_spawn_z'),
            'spawn_yaw': LaunchConfiguration('robot_spawn_yaw'),
        }],
    )

    fast_lio_map_aligner = Node(
        package='airos_experiments',
        executable='fast_lio_map_aligner',
        name='fast_lio_map_aligner',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/Laser_map',
            'output_topic': '/Laser_map_world',
            'output_frame': 'map',
            'max_points': 800000,
            'spawn_x': LaunchConfiguration('robot_spawn_x'),
            'spawn_y': LaunchConfiguration('robot_spawn_y'),
            'spawn_z': LaunchConfiguration('robot_spawn_z'),
            'spawn_yaw': LaunchConfiguration('robot_spawn_yaw'),
        }],
    )

    fast_lio_registered_aligner = Node(
        package='airos_experiments',
        executable='fast_lio_map_aligner',
        name='fast_lio_registered_aligner',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/cloud_registered',
            'output_topic': '/cloud_registered_world',
            'output_frame': 'map',
            'max_points': 60000,
            'spawn_x': LaunchConfiguration('robot_spawn_x'),
            'spawn_y': LaunchConfiguration('robot_spawn_y'),
            'spawn_z': LaunchConfiguration('robot_spawn_z'),
            'spawn_yaw': LaunchConfiguration('robot_spawn_yaw'),
        }],
    )

    slam_scan_projector = Node(
        package='airos_experiments',
        executable='slam_scan_projector',
        name='slam_scan_projector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'cloud_topic': '/cloud_registered_world',
            'odom_topic': LaunchConfiguration('terrain_odom_topic'),
            'pose_source': 'tf',
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'scan_topic': '/slam_scan',
            'scan_frame': 'base_footprint',
            'publish_rate_hz': 6.0,
            'angle_min': -3.141592653589793,
            'angle_max': 3.141592653589793,
            'angle_increment': 0.017453292519943295,
            'range_min': 0.08,
            'range_max': 4.5,
            'min_z': 0.45,
            'max_z': 1.40,
            'max_points': 60000,
            'surface_estimate_radius': 0.75,
            'surface_estimate_min_points': 3,
        }],
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
            'input_topic': '/Laser_map_world',
            'output_topic': '/Laser_map_colored',
            'min_z': -0.40,
            'max_z': 2.20,
            'min_visible_z': 0.03,
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
            'odom_topic': '/fast_lio_odom_world',
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
            'odom_topic': LaunchConfiguration('terrain_odom_topic'),
            'use_initial_pose_anchor': False,
            'goal_topic': '/terrain_goal_pose',
            'path_topic': '/pct_path',
            'terrain_cloud_topic': '/terrain_traversability_cloud',
            'terrain_map_source': LaunchConfiguration('terrain_map_source'),
            'slam_map_topic': '/Laser_map_world',
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
            'max_step_height': 0.50,
            'goal_z_policy': LaunchConfiguration('terrain_goal_z_policy'),
            'goal_min_z': LaunchConfiguration('terrain_goal_min_z'),
            'goal_max_z': LaunchConfiguration('terrain_goal_max_z'),
            'goal_snap_max_distance': 2.0,
            'frontier_replan_enabled': True,
            'frontier_min_path_distance': 0.25,
            'frontier_max_path_distance': 14.0,
            'frontier_obstacle_scan_topic': '/slam_scan',
            'frontier_obstacle_clearance': 0.45,
            'frontier_obstacle_range_max': 3.0,
            'frontier_stall_timeout_sec': 8.0,
            'frontier_stall_min_progress': 0.20,
            'frontier_failed_clearance': 1.6,
            'frontier_goal_regression_tolerance': 1.5,
            'send_nav2_goals': LaunchConfiguration('terrain_send_nav2_goals'),
            'nav_execution_mode': LaunchConfiguration('terrain_execution_mode'),
            'direct_cmd_vel_topic': '/cmd_vel_nav',
            'direct_lookahead_dist': 0.45,
            'direct_waypoint_tolerance': 0.42,
            'direct_goal_tolerance': 0.30,
            'direct_z_tolerance': 0.45,
            'direct_heading_gain': 1.4,
            'direct_max_linear_speed': 0.30,
            'direct_min_linear_speed': 0.035,
            'direct_max_angular_speed': 0.45,
            'direct_max_heading_error_for_forward': 1.25,
            'waypoint_spacing': 0.90,
            'start_waypoint_clearance': 0.75,
            'follow_path_start_clearance': 0.12,
            'slope_speed_limit': 0.16,
            'flat_speed_limit': 0.32,
            'slope_speed_grade_threshold': 0.08,
            'initial_surface_z_hint': LaunchConfiguration('robot_spawn_z'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('world', default_value='single_floor_complex_large'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                pkg_nav,
                'maps',
                'single_floor_complex_large.yaml',
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
        DeclareLaunchArgument('planner_profile', default_value='research'),
        DeclareLaunchArgument('localization', default_value='slam_toolbox_mapping'),
        DeclareLaunchArgument('slam_nav_startup', default_value='gated'),
        DeclareLaunchArgument('dynamic_obstacles', default_value='false'),
        DeclareLaunchArgument('physical_dynamic_obstacles', default_value='false'),
        DeclareLaunchArgument('open_source_scene_assets', default_value='false'),
        DeclareLaunchArgument('robot_visual_profile', default_value='reference_mesh'),
        DeclareLaunchArgument('log_level', default_value='warn'),
        DeclareLaunchArgument('sensor_source', default_value='native'),
        DeclareLaunchArgument('gazebo_rendering_mode', default_value='wsl_stable'),
        DeclareLaunchArgument('fast_lio_debug', default_value='true'),
        DeclareLaunchArgument('colorized_pointcloud', default_value='true'),
        DeclareLaunchArgument('dense_visual_pointcloud', default_value='false'),
        DeclareLaunchArgument('pointcloud_spacing', default_value='0.06'),
        DeclareLaunchArgument('max_live_points', default_value='180000'),
        DeclareLaunchArgument('fast_lio_pointcloud_spacing', default_value='0.16'),
        DeclareLaunchArgument('fast_lio_max_live_points', default_value='12000'),
        DeclareLaunchArgument('terrain_map_source', default_value='slam_cloud'),
        DeclareLaunchArgument('collision_scan_topic', default_value='/scan'),
        DeclareLaunchArgument('terrain_goal_z_policy', default_value='nearest_z'),
        DeclareLaunchArgument('terrain_goal_min_z', default_value='-1.0'),
        DeclareLaunchArgument('terrain_goal_max_z', default_value='-1.0'),
        DeclareLaunchArgument('slam_map_max_points', default_value='180000'),
        DeclareLaunchArgument('slam_grid_resolution', default_value='0.30'),
        DeclareLaunchArgument('slam_min_cell_points', default_value='2'),
        DeclareLaunchArgument('slam_vertical_layer_gap', default_value='0.18'),
        DeclareLaunchArgument('slam_rebuild_period_sec', default_value='3.0'),
        DeclareLaunchArgument('terrain_odom_topic', default_value='/fast_lio_odom_world'),
        DeclareLaunchArgument('terrain_planner', default_value='false'),
        DeclareLaunchArgument(
            'terrain_world_file',
            default_value=os.path.join(
                pkg_sim,
                'worlds',
                'single_floor_complex_large_static.sdf',
            ),
        ),
        DeclareLaunchArgument('terrain_send_nav2_goals', default_value='true'),
        DeclareLaunchArgument('terrain_execution_mode', default_value='direct'),
        DeclareLaunchArgument('nav_stack_mode', default_value='full'),
        DeclareLaunchArgument('robot_spawn_x', default_value='0.0'),
        DeclareLaunchArgument('robot_spawn_y', default_value='0.0'),
        DeclareLaunchArgument('robot_spawn_z', default_value='0.26'),
        DeclareLaunchArgument('robot_spawn_yaw', default_value='0.0'),
        sim,
        TimerAction(
            period=10.0,
            actions=[map_to_fast_lio_map, fast_lio],
            condition=IfCondition(LaunchConfiguration('fast_lio_debug')),
        ),
        TimerAction(
            period=13.0,
            actions=[fast_lio_localization_bridge],
            condition=IfCondition(_fast_lio_localization_enabled()),
        ),
        TimerAction(period=15.0, actions=[nav]),
        TimerAction(
            period=24.0,
            actions=[lifecycle_activator],
            condition=IfCondition(LaunchConfiguration('use_route')),
        ),
        TimerAction(
            period=24.0,
            actions=[fast_lio_map_aligner, fast_lio_registered_aligner],
            condition=IfCondition(LaunchConfiguration('fast_lio_debug')),
        ),
        TimerAction(
            period=25.0,
            actions=[slam_scan_projector],
            condition=IfCondition(LaunchConfiguration('fast_lio_debug')),
        ),
        TimerAction(period=26.0, actions=[pointcloud_colorizer]),
        TimerAction(period=26.0, actions=[dense_building_cloud]),
        TimerAction(period=28.0, actions=[terrain_planner]),
        TimerAction(period=32.0, actions=[rviz]),
    ])
