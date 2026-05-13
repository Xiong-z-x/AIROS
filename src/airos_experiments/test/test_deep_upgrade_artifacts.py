from __future__ import annotations

from pathlib import Path

import yaml


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _read_text(relative_path: str) -> str:
    return (_repo_root() / relative_path).read_text(encoding='utf-8')


def test_sim_launch_can_select_advanced_world_and_physical_obstacles() -> None:
    launch_text = _read_text('src/airos_sim/launch/sim.launch.py')
    world_text = _read_text('src/airos_sim/worlds/advanced_indoor_ramp.sdf')
    static_world_text = _read_text(
        'src/airos_sim/worlds/advanced_indoor_ramp_static.sdf'
    )
    realistic_world_text = _read_text(
        'src/airos_sim/worlds/realistic_multilevel_ramp.sdf'
    )
    static_realistic_world_text = _read_text(
        'src/airos_sim/worlds/realistic_multilevel_ramp_static.sdf'
    )

    assert "DeclareLaunchArgument('world', default_value='single_floor_lab')" in launch_text
    assert "advanced_indoor_ramp.sdf" in launch_text
    assert "advanced_indoor_ramp_static.sdf" in launch_text
    assert "realistic_multilevel_ramp.sdf" in launch_text
    assert "realistic_multilevel_ramp_static.sdf" in launch_text
    assert "physical_dynamic_obstacles" in launch_text
    assert "open_source_scene_assets" in launch_text
    assert "robot_visual_profile" in launch_text
    assert "DeclareLaunchArgument('robot_spawn_z', default_value='0.26')" in launch_text
    assert "dynamic_obstacle_marker_emulator" in launch_text
    assert "'scan_topic': '/scan_dynamic_overlay'" in launch_text
    assert "DeclareLaunchArgument('point_spacing', default_value='0.06')" in launch_text
    assert "DeclareLaunchArgument('max_live_points', default_value='180000')" in launch_text
    assert "float(LaunchConfiguration('point_spacing').perform(context))" in launch_text
    assert "int(LaunchConfiguration('max_live_points').perform(context))" in launch_text
    assert "'include_dynamic_models': physical_dynamic_obstacles" in launch_text
    assert "os.path.dirname(pkg_sim)" in launch_text
    assert "os.path.dirname(pkg_desc)" in launch_text
    assert "moving_pedestrian" in world_text
    assert "inspection_cart_dynamic" in world_text
    assert "triggered-publisher" in world_text
    assert "moving_pedestrian" not in static_world_text
    assert "inspection_cart_dynamic" not in static_world_text
    assert "triggered-publisher" not in static_world_text
    assert "ramp_main" in world_text
    assert "wide_access_ramp" in realistic_world_text
    assert "mezzanine_deck_visual" in realistic_world_text
    assert "moving_pedestrian" not in static_realistic_world_text
    assert "inspection_cart_dynamic" not in static_realistic_world_text
    assert "triggered-publisher" not in static_realistic_world_text
    assert "/airos/realistic_multilevel_ramp/start_dynamic_obstacles" in realistic_world_text


def test_open_source_reference_assets_are_installed_and_licensed() -> None:
    sim_cmake = _read_text('src/airos_sim/CMakeLists.txt')
    building_model = _read_text('src/airos_sim/models/open_source_building/model.sdf')
    robot_urdf = _read_text(
        'src/airos_go2w_description/urdf/go2w_nav_eq.urdf.xacro'
    )
    license_text = _read_text('docs/third_party_3d_dog_navi_ros2_AFL-3.0_LICENSE')

    assert 'DIRECTORY config launch models worlds' in sim_cmake
    assert 'Building.dae' in building_model
    assert 'open_source_go2w_reference/base.dae' in robot_urdf
    assert 'open_source_go2w_reference/${' in robot_urdf
    assert 'open_source_go2w_reference/mid-360-scaled.dae' in robot_urdf
    assert 'scale="0.001 0.001 0.001"' in robot_urdf
    assert 'Academic Free License' in license_text


def test_advanced_world_has_matching_nav_map_and_missions() -> None:
    map_yaml = yaml.safe_load(
        _read_text('src/airos_nav/maps/advanced_indoor_ramp.yaml')
    )
    mission_text = _read_text(
        'src/airos_experiments/missions/advanced_indoor_ramp_missions.yaml'
    )
    route_text = _read_text('src/airos_nav/routes/advanced_indoor_ramp_route.geojson')

    assert map_yaml['image'] == 'advanced_indoor_ramp.pgm'
    assert map_yaml['resolution'] == 0.05
    assert 'ramp_entry_to_upper_observation' in mission_text
    assert 'advanced_indoor_ramp_route' in route_text


def test_realistic_multilevel_world_has_matching_nav_artifacts() -> None:
    map_yaml = yaml.safe_load(
        _read_text('src/airos_nav/maps/realistic_multilevel_ramp.yaml')
    )
    mission_text = _read_text(
        'src/airos_experiments/missions/realistic_multilevel_ramp_missions.yaml'
    )
    route_text = _read_text(
        'src/airos_nav/routes/realistic_multilevel_ramp_route.geojson'
    )
    setup_text = _read_text('src/airos_experiments/setup.py')

    assert map_yaml['image'] == 'realistic_multilevel_ramp.pgm'
    assert map_yaml['resolution'] == 0.05
    assert 'multilevel_upper_lab_long_goal' in mission_text
    assert 'dynamic_crossing_to_south_service' in mission_text
    assert 'realistic_multilevel_ramp_route' in route_text
    assert 'realistic_multilevel_ramp_missions.yaml' in setup_text


def test_nav_launch_exposes_planner_profile_and_advanced_defaults() -> None:
    nav_launch = _read_text('src/airos_nav/launch/nav.launch.py')
    profile_text = _read_text('src/airos_nav/config/nav2_research_profile.yaml')
    readme_text = _read_text('docs/advanced_planning_research_profile.md')
    setup_text = _read_text('src/airos_experiments/setup.py')
    candidate_text = _read_text(
        'src/airos_experiments/airos_experiments/advanced_planner_candidate.py'
    )

    assert "DeclareLaunchArgument('planner_profile', default_value='baseline')" in nav_launch
    assert "nav2_research_profile.yaml" in nav_launch
    assert "planner_profile" in nav_launch
    assert 'nav2_mppi_controller::MPPIController' in profile_text
    assert 'movement_time_allowance: 25.0' in profile_text
    assert 'max_path_occupancy_ratio: 0.20' in profile_text
    assert 'generate_advanced_planner_candidates' in setup_text
    assert 'terrain_pct_planner' in setup_text
    assert 'airos_advanced_planner_candidate.v1' in candidate_text
    assert 'research_surrogate_not_trained_runtime' in candidate_text
    assert 'PCT-planner' in readme_text
    assert '强化学习' in readme_text


def test_clean_runner_can_pass_world_map_route_and_planner_profile() -> None:
    runner_text = _read_text(
        'src/airos_experiments/airos_experiments/clean_batch_runner.py'
    )

    assert "parser.add_argument('--world'" in runner_text
    assert "'--map'" in runner_text
    assert "parser.add_argument('--planner-profile'" in runner_text
    assert "parser.add_argument('--open-source-scene-assets'" in runner_text
    assert "parser.add_argument('--robot-visual-profile'" in runner_text
    assert "world:={args.world}" in runner_text
    assert "open_source_scene_assets:=" in runner_text
    assert "robot_visual_profile:={args.robot_visual_profile}" in runner_text
    assert "map:={args.map}" in runner_text
    assert "planner_profile:={args.planner_profile}" in runner_text
    assert "use_route:=true" in runner_text
