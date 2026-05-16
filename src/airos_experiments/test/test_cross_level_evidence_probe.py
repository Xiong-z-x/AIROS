from __future__ import annotations

import subprocess
from pathlib import Path

from airos_experiments.cross_level_evidence_probe import _run_ign_pose_query


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _read_text(relative_path: str) -> str:
    return (_repo_root() / relative_path).read_text(encoding='utf-8')


def test_cross_level_evidence_probe_is_exposed_as_console_script() -> None:
    setup_text = _read_text('src/airos_experiments/setup.py')

    assert 'cross_level_evidence_probe = ' in setup_text
    assert 'airos_experiments.cross_level_evidence_probe:main' in setup_text


def test_terrain_goal_publisher_is_exposed_as_console_script() -> None:
    setup_text = _read_text('src/airos_experiments/setup.py')

    assert 'publish_terrain_goal = ' in setup_text
    assert 'airos_experiments.terrain_goal_publisher:main' in setup_text


def test_terrain_goal_publisher_preserves_3d_goal_fields() -> None:
    publisher_text = _read_text(
        'src/airos_experiments/airos_experiments/terrain_goal_publisher.py'
    )

    assert "parser.add_argument('--z', type=float" in publisher_text
    assert "parser.add_argument('--frame-id', default='map')" in publisher_text
    assert "parser.add_argument('--topic', default='/terrain_goal_pose')" in publisher_text
    assert 'msg.pose.position.z = float(args.z)' in publisher_text
    assert 'publish_count' in publisher_text
    assert 'rate_hz' in publisher_text


def test_cross_level_evidence_probe_records_required_fields() -> None:
    probe_text = _read_text(
        'src/airos_experiments/airos_experiments/cross_level_evidence_probe.py'
    )

    for field in (
        'laser_map_points',
        'pct_path_poses',
        'pct_path_max_z',
        'cmd_vel_nav_norm',
        'cmd_vel_nav_count',
        'cmd_vel_nav_age_sec',
        'cmd_vel_smoothed_norm',
        'cmd_vel_smoothed_count',
        'cmd_vel_smoothed_age_sec',
        'base_cmd_norm',
        'base_cmd_count',
        'base_cmd_age_sec',
        'fast_lio_xyz',
        'fast_lio_goal_xy_distance',
        'wheel_odom_xyz',
        'wheel_goal_xy_distance',
        'gazebo_xyz',
        'gazebo_goal_xy_distance',
        'goal_xyz',
    ):
        assert field in probe_text

    assert "self.declare_parameter('wheel_odom_topic', '/odom')" in probe_text
    assert 'self._wheel_odom_callback' in probe_text
    assert "'ign'," in probe_text
    assert "'--json-output'," in probe_text
    assert "'/world/{world}/pose/info'" in probe_text
    assert "pose.get('name') != entity" in probe_text
    assert 'result.stdout.splitlines()' in probe_text
    assert "parser.add_argument('--goal-z', type=float, default=2.2)" in probe_text
    assert 'goal_xyz=(args.goal_x, args.goal_y, args.goal_z)' in probe_text


def test_ign_pose_query_timeout_returns_none(monkeypatch) -> None:
    def _timeout_run(*args, **kwargs):  # noqa: ANN001, ANN002, ANN003
        raise subprocess.TimeoutExpired(cmd=kwargs.get('args', 'ign'), timeout=3.0)

    monkeypatch.setattr(subprocess, 'run', _timeout_run)

    assert _run_ign_pose_query('large_multilevel_complex', 'go2w_nav_eq', 3.0) is None
