from setuptools import find_packages, setup

package_name = 'airos_experiments'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/dynamic_obstacles.launch.py',
            'launch/visual_fast_lio_navigation.launch.py',
            'launch/visual_slam_mapping.launch.py',
            'launch/visual_navigation.launch.py',
        ]),
        ('share/' + package_name + '/missions', [
            'missions/advanced_indoor_ramp_missions.yaml',
            'missions/large_complex_go2w_missions.yaml',
            'missions/realistic_multilevel_ramp_missions.yaml',
            'missions/single_floor_lab_missions.yaml',
        ]),
        ('share/' + package_name + '/scripts', [
            'scripts/run_clean_nav_batch.py',
            'scripts/run_nav_trials.py',
            'scripts/summarize_trials.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xiongzx',
    maintainer_email='3133903229@qq.com',
    description='Experiment and reporting package for AIROS navigation trials.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'generate_world_map = airos_experiments.world_map_generator:main',
            'fast_lio_map_aligner = airos_experiments.fast_lio_map_aligner:main',
            'fast_lio_localization_bridge = airos_experiments.fast_lio_localization_bridge:main',
            'generate_advanced_planner_candidates = '
            'airos_experiments.advanced_planner_candidate:main',
            'cross_level_evidence_probe = '
            'airos_experiments.cross_level_evidence_probe:main',
            'imu_republisher = airos_experiments.imu_republisher:main',
            'initial_pose_publisher = airos_experiments.initial_pose_publisher:main',
            'lifecycle_activator = airos_experiments.lifecycle_activator:main',
            'livox_custom_bridge = airos_experiments.livox_custom_bridge:main',
            'nav_chain_smoke_probe = airos_experiments.nav_chain_smoke_probe:main',
            'pointcloud_colorizer = airos_experiments.pointcloud_colorizer:main',
            'pointcloud_emulator = airos_experiments.pointcloud_emulator:main',
            'publish_terrain_goal = airos_experiments.terrain_goal_publisher:main',
            'rviz2_safe = airos_experiments.rviz2_safe:main',
            'verify_route_graph = airos_experiments.route_graph_verifier:main',
            'run_clean_nav_batch = airos_experiments.clean_batch_runner:main',
            'run_nav_trials = airos_experiments.nav_trial_runner:main',
            'save_slam_map = airos_experiments.slam_map_saver:main',
            'scan_emulator = airos_experiments.scan_emulator:main',
            'slam_nav_coordinator = airos_experiments.slam_nav_coordinator:main',
            'slam_scan_projector = airos_experiments.slam_scan_projector:main',
            'summarize_trials = airos_experiments.metrics_summarizer:main',
            'terrain_pct_planner = airos_experiments.terrain_pct_planner:main',
        ],
    },
)
