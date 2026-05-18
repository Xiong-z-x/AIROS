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
    pkg_fast_lio = get_package_share_directory('fast_lio')

    sim_launch = os.path.join(pkg_sim, 'launch', 'sim.launch.py')
    nav_launch = os.path.join(pkg_nav, 'launch', 'nav.launch.py')
    rviz_config = os.path.join(pkg_nav, 'rviz', 'nav.rviz')
    nav_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    fast_lio_config = os.path.join(pkg_fast_lio, 'config', 'airos_sim.yaml')

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
        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_launch))],
    )

    fast_lio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[fast_lio_config, {'use_sim_time': True}],
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
            'planner_profile': LaunchConfiguration('planner_profile'),
            'params_file': nav_params,
            'nav_stack_mode': LaunchConfiguration('nav_stack_mode'),
            'localization': LaunchConfiguration('localization'),
            'external_map_manager': 'false',
            'collision_scan_topic': LaunchConfiguration('collision_scan_topic'),
            'slam_nav_startup': LaunchConfiguration('slam_nav_startup'),
            'log_level': LaunchConfiguration('log_level'),
        },
        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(nav_launch))],
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

    planner_compare = Node(
        package='airos_experiments',
        executable='planner_comparison_node',
        name='planner_comparison_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'map_topic': '/map',
            'goal_topic': '/goal_pose',
            'global_frame': 'map',
            'base_frame': 'base_footprint',
            'robot_radius_m': 0.43,
            'occupied_threshold': 65,
            'unknown_is_occupied': False,
            'slam_scan_topic': '/slam_scan',
            'use_slam_scan_overlay': True,
            'slam_scan_max_age_sec': 2.0,
            'slam_scan_obstacle_radius_m': 0.25,
            'snap_radius_m': 1.2,
            'q_grid_step': 4,
            'q_max_iterations': 60000,
            'q_discount': 0.96,
            'rrt_max_samples': 2600,
            'rrt_step_m': 0.65,
            'rrt_goal_sample_rate': 0.12,
            'rrt_rewire_radius_m': 1.4,
            'rrt_attempts': 3,
            'random_seed': 7,
            'animate_paths': True,
            'publish_metrics': LaunchConfiguration('publish_metrics'),
            'path_animation_rate_hz': 14.0,
            'path_animation_spacing_m': 0.12,
            'path_animation_points_per_tick': 3,
            'enable_navigate_to_pose_bridge': LaunchConfiguration(
                'enable_navigate_to_pose_bridge'
            ),
            'execute_primary_nav2_goal': LaunchConfiguration('execute_primary_motion'),
            'navigate_to_pose_action_name': 'navigate_to_pose',
        }],
    )

    dense_world_cloud = Node(
        condition=IfCondition(LaunchConfiguration('dense_visual_pointcloud')),
        package='airos_experiments',
        executable='pointcloud_emulator',
        name='planner_showcase_dense_world_cloud',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'world_file': os.path.join(
                pkg_sim,
                'worlds',
                'single_floor_planner_showcase_static.sdf',
            ),
            'odom_topic': '/odom',
            'lidar_topic': '/planner_showcase/livox_points_local',
            'registered_cloud_topic': '/dense_visual_cloud',
            'map_cloud_topic': '/dense_visual_cloud_map',
            'publish_registered_cloud': True,
            'publish_map_cloud': True,
            'world_frame': 'map',
            'lidar_frame': 'livox_frame',
            'use_initial_pose_anchor': False,
            'point_spacing': 0.08,
            'range_max': 40.0,
            'publish_rate_hz': 1.0,
            'map_publish_rate_hz': 0.5,
            'max_live_points': 220000,
            'include_dynamic_models': False,
        }],
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

    slam_scan_projector = Node(
        package='airos_experiments',
        executable='slam_scan_projector',
        name='planner_showcase_slam_scan_projector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'cloud_topic': '/cloud_registered_world',
            'odom_topic': '/fast_lio_odom_world',
            'pose_source': 'tf',
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'scan_topic': '/slam_scan',
            'scan_frame': 'base_footprint',
            'publish_rate_hz': 4.0,
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

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('world', default_value='single_floor_planner_showcase'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                pkg_nav,
                'maps',
                'single_floor_planner_showcase.yaml',
            ),
        ),
        DeclareLaunchArgument('robot_visual_profile', default_value='analytic'),
        DeclareLaunchArgument('sensor_source', default_value='native'),
        DeclareLaunchArgument('gazebo_rendering_mode', default_value='wsl_stable'),
        DeclareLaunchArgument('robot_spawn_x', default_value='0.0'),
        DeclareLaunchArgument('robot_spawn_y', default_value='0.0'),
        DeclareLaunchArgument('robot_spawn_z', default_value='0.26'),
        DeclareLaunchArgument('robot_spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('log_level', default_value='warn'),
        DeclareLaunchArgument('use_route', default_value='false'),
        DeclareLaunchArgument('planner_profile', default_value='research'),
        DeclareLaunchArgument('localization', default_value='slam_toolbox_mapping'),
        DeclareLaunchArgument('slam_nav_startup', default_value='gated'),
        DeclareLaunchArgument('dynamic_obstacles', default_value='false'),
        DeclareLaunchArgument('physical_dynamic_obstacles', default_value='false'),
        DeclareLaunchArgument('open_source_scene_assets', default_value='false'),
        DeclareLaunchArgument('fast_lio_debug', default_value='true'),
        DeclareLaunchArgument('colorized_pointcloud', default_value='true'),
        DeclareLaunchArgument('dense_visual_pointcloud', default_value='false'),
        DeclareLaunchArgument('fast_lio_pointcloud_spacing', default_value='0.16'),
        DeclareLaunchArgument('fast_lio_max_live_points', default_value='12000'),
        DeclareLaunchArgument('collision_scan_topic', default_value='/scan'),
        DeclareLaunchArgument('nav_stack_mode', default_value='full'),
        DeclareLaunchArgument('execute_primary_motion', default_value='true'),
        DeclareLaunchArgument('publish_metrics', default_value='false'),
        DeclareLaunchArgument('enable_navigate_to_pose_bridge', default_value='false'),
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
            actions=[fast_lio_map_aligner, fast_lio_registered_aligner],
            condition=IfCondition(LaunchConfiguration('fast_lio_debug')),
        ),
        TimerAction(
            period=25.0,
            actions=[slam_scan_projector],
            condition=IfCondition(LaunchConfiguration('fast_lio_debug')),
        ),
        TimerAction(period=26.0, actions=[pointcloud_colorizer]),
        TimerAction(period=26.0, actions=[dense_world_cloud]),
        TimerAction(period=30.0, actions=[planner_compare]),
        TimerAction(period=32.0, actions=[rviz]),
    ])
