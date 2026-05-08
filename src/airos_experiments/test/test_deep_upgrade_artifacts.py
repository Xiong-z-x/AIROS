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

    assert "DeclareLaunchArgument('world', default_value='single_floor_lab')" in launch_text
    assert "advanced_indoor_ramp.sdf" in launch_text
    assert "physical_dynamic_obstacles" in launch_text
    assert "moving_pedestrian" in world_text
    assert "inspection_cart_dynamic" in world_text
    assert "triggered-publisher" in world_text
    assert "ramp_main" in world_text


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


def test_nav_launch_exposes_planner_profile_and_advanced_defaults() -> None:
    nav_launch = _read_text('src/airos_nav/launch/nav.launch.py')
    profile_text = _read_text('src/airos_nav/config/nav2_research_profile.yaml')
    readme_text = _read_text('docs/advanced_planning_research_profile.md')

    assert "DeclareLaunchArgument('planner_profile', default_value='baseline')" in nav_launch
    assert "nav2_research_profile.yaml" in nav_launch
    assert "planner_profile" in nav_launch
    assert 'nav2_mppi_controller::MPPIController' in profile_text
    assert 'PCT-planner' in readme_text
    assert '强化学习' in readme_text


def test_clean_runner_can_pass_world_map_route_and_planner_profile() -> None:
    runner_text = _read_text(
        'src/airos_experiments/airos_experiments/clean_batch_runner.py'
    )

    assert "parser.add_argument('--world'" in runner_text
    assert "'--map'" in runner_text
    assert "parser.add_argument('--planner-profile'" in runner_text
    assert "world:={args.world}" in runner_text
    assert "map:={args.map}" in runner_text
    assert "planner_profile:={args.planner_profile}" in runner_text
    assert "use_route:=true" in runner_text
