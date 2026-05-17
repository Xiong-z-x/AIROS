import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('airos_nav')
    pkg_sim = get_package_share_directory('airos_sim')

    sim_launch = os.path.join(pkg_sim, 'launch', 'sim.launch.py')
    nav_launch = os.path.join(pkg_nav, 'launch', 'nav.launch.py')
    rviz_config = os.path.join(pkg_nav, 'rviz', 'nav.rviz')
    planner_params = os.path.join(
        pkg_nav,
        'config',
        'nav2_planner_comparison.yaml',
    )

    sim = GroupAction(
        scoped=True,
        launch_configurations={
            'gui': LaunchConfiguration('gui'),
            'rviz': 'false',
            'world': LaunchConfiguration('world'),
            'dynamic_obstacles': 'false',
            'physical_dynamic_obstacles': 'false',
            'open_source_scene_assets': 'false',
            'robot_visual_profile': LaunchConfiguration('robot_visual_profile'),
            'sensor_source': LaunchConfiguration('sensor_source'),
            'pointcloud': 'true',
            'pointcloud_registered': 'false',
            'pointcloud_map': 'false',
            'point_spacing': '0.10',
            'max_live_points': '12000',
            'gazebo_rendering_mode': LaunchConfiguration('gazebo_rendering_mode'),
            'robot_spawn_x': LaunchConfiguration('robot_spawn_x'),
            'robot_spawn_y': LaunchConfiguration('robot_spawn_y'),
            'robot_spawn_z': LaunchConfiguration('robot_spawn_z'),
            'robot_spawn_yaw': LaunchConfiguration('robot_spawn_yaw'),
        },
        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_launch))],
    )

    nav = GroupAction(
        scoped=True,
        launch_configurations={
            'rviz': 'false',
            'use_route': 'false',
            'map': LaunchConfiguration('map'),
            'planner_profile': 'baseline',
            'params_file': planner_params,
            'nav_stack_mode': 'planner_only',
            'localization': 'static',
            'external_map_manager': 'true',
            'collision_scan_topic': '/scan',
            'slam_nav_startup': 'autostart',
            'log_level': LaunchConfiguration('log_level'),
        },
        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(nav_launch))],
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
            'snap_radius_m': 1.2,
            'q_grid_step': 4,
            'q_max_iterations': 60000,
            'q_discount': 0.96,
            'rrt_max_samples': 2600,
            'rrt_step_m': 0.65,
            'rrt_goal_sample_rate': 0.12,
            'rrt_rewire_radius_m': 1.4,
            'random_seed': 7,
            'animate_paths': True,
            'path_animation_rate_hz': 14.0,
            'path_animation_spacing_m': 0.12,
            'path_animation_points_per_tick': 3,
        }],
    )

    dense_world_cloud = Node(
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
            'registered_cloud_topic': '/planner_showcase/dense_world_cloud',
            'map_cloud_topic': '/planner_showcase/dense_map_cloud',
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

    dense_cloud_colorizer = Node(
        package='airos_experiments',
        executable='pointcloud_colorizer',
        name='planner_showcase_cloud_colorizer',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/planner_showcase/dense_map_cloud',
            'output_topic': '/Laser_map_colored',
            'min_z': -0.40,
            'max_z': 2.20,
            'min_visible_z': 0.06,
            'max_points': 900000,
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
        sim,
        TimerAction(period=5.0, actions=[nav]),
        TimerAction(period=9.0, actions=[dense_world_cloud]),
        TimerAction(period=10.0, actions=[dense_cloud_colorizer]),
        TimerAction(period=16.0, actions=[planner_compare]),
        TimerAction(period=18.0, actions=[rviz]),
    ])
