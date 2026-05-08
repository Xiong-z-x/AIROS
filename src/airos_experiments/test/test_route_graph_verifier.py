from __future__ import annotations

import subprocess

from airos_experiments.route_graph_verifier import _run


def test_run_converts_timeout_to_return_code(monkeypatch) -> None:
    def _raise_timeout(*args, **kwargs):
        raise subprocess.TimeoutExpired(
            cmd=['ros2', 'action', 'list', '-t'],
            timeout=2.0,
        )

    monkeypatch.setattr(subprocess, 'run', _raise_timeout)

    result = _run(['ros2', 'action', 'list', '-t'], 2.0)

    assert result.returncode == 124
    assert result.stdout == ''
    assert result.stderr == ''
