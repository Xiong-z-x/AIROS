from __future__ import annotations

from pathlib import Path

from airos_experiments.advanced_planner_candidate import build_candidate_report


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def test_advanced_planner_candidates_keep_runtime_boundary() -> None:
    root = _repo_root()

    report = build_candidate_report(
        root / 'src/airos_nav/maps/advanced_indoor_ramp.yaml',
        root / 'src/airos_nav/routes/advanced_indoor_ramp_route.geojson',
        start_id=1,
        goal_id=3,
    )

    assert report['schema'] == 'airos_advanced_planner_candidate.v1'
    assert report['candidate_count'] == 3
    planners = {
        candidate['planner_id']: candidate
        for candidate in report['candidates']
    }
    assert planners['nav2_baseline_route']['runtime_status'] == (
        'implemented_runtime_baseline'
    )
    assert planners['pct_style_risk_weighted_route']['runtime_status'] == (
        'research_surrogate_not_trained_runtime'
    )
    assert planners['rl_safety_shield_waypoints']['runtime_status'] == (
        'research_surrogate_not_trained_runtime'
    )
    assert planners['nav2_baseline_route']['route_edge_ids'] == [201, 202]
    assert planners['pct_style_risk_weighted_route']['risk_penalty_m'] > 0.0
    assert len(planners['rl_safety_shield_waypoints']['waypoints']) > len(
        planners['nav2_baseline_route']['waypoints']
    )
    assert 'not trained PCT/RL planner runtimes' in report['research_boundary']


def test_advanced_planner_avoids_dynamic_obstacle_route_when_available() -> None:
    root = _repo_root()

    report = build_candidate_report(
        root / 'src/airos_nav/maps/advanced_indoor_ramp.yaml',
        root / 'src/airos_nav/routes/advanced_indoor_ramp_route.geojson',
        start_id=5,
        goal_id=4,
    )

    planners = {
        candidate['planner_id']: candidate
        for candidate in report['candidates']
    }
    baseline = planners['nav2_baseline_route']
    pct_style = planners['pct_style_risk_weighted_route']

    assert baseline['route_edge_ids'] == [204, 203]
    assert pct_style['route_edge_ids'] == [205]
    assert pct_style['route_risk_exposure_m'] < baseline['route_risk_exposure_m']
    assert pct_style['path_length_m'] > baseline['path_length_m']
