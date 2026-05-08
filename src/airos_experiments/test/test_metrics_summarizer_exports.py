import json
from pathlib import Path

from airos_experiments.metrics_summarizer import export_report_artifacts
from airos_experiments.nav_trial_runner import (
    Mission,
    _load_route_graph,
    _route_waypoints,
)


def _write_jsonl(path: Path, rows: list[dict[str, object]]) -> None:
    path.write_text(
        ''.join(json.dumps(row) + '\n' for row in rows),
        encoding='utf-8',
    )


def test_export_report_artifacts_writes_outputs(tmp_path: Path) -> None:
    input_path = tmp_path / 'trials.jsonl'
    csv_path = tmp_path / 'summary.csv'
    markdown_path = tmp_path / 'summary.md'
    figures_dir = tmp_path / 'figures'
    _write_jsonl(
        input_path,
        [
            {
                'mission_id': 'mission_a',
                'route_id': 'route_a',
                'success': True,
                'elapsed_sec': 1.0,
                'path_length_m': 2.0,
                'emergency_stop_count': 0,
                'collision_count': 0,
                'minimum_obstacle_distance_m': 0.5,
                'mean_cmd_period_sec': 0.05,
                'max_cmd_period_sec': 0.07,
            },
            {
                'mission_id': 'mission_a',
                'route_id': 'route_a',
                'success': True,
                'elapsed_sec': 3.0,
                'path_length_m': 4.0,
                'emergency_stop_count': 0,
                'collision_count': 0,
                'minimum_obstacle_distance_m': 0.4,
                'mean_cmd_period_sec': 0.06,
                'max_cmd_period_sec': 0.08,
            },
        ],
    )

    summary = export_report_artifacts(
        input_path,
        csv_path,
        markdown_path,
        figures_dir,
    )

    assert summary['trial_count'] == 2
    csv_text = csv_path.read_text(encoding='utf-8')
    assert 'mission_a,route_a,2,2,1.0,2.0,1.0,3.0,0.4,0.08' in csv_text
    assert 'Success: 2/2' in markdown_path.read_text(encoding='utf-8')
    assert (figures_dir / 'mean_elapsed_sec.svg').exists()
    assert (figures_dir / 'mean_path_length_m.svg').exists()


def test_route_waypoints_follow_geojson_edges() -> None:
    src_root = Path(__file__).resolve().parents[2]
    graph = _load_route_graph(
        src_root / 'airos_nav/routes/single_floor_lab_route.geojson'
    )
    mission = Mission(
        mission_id='lab_start_to_task_a',
        start_pose=(0.0, 0.0, 0.0),
        goal_pose=(2.25, 2.35, 1.57),
        route_id='start_to_task_a',
        dynamic_obstacle_seed=1,
        speed_limit=0.35,
        expected_timeout_sec=90.0,
    )

    waypoints = _route_waypoints(graph, mission)

    assert [(round(x, 2), round(y, 2)) for x, y, _ in waypoints] == [
        (2.10, -1.60),
        (2.25, 2.35),
    ]
    assert round(waypoints[-1][2], 2) == 1.57
