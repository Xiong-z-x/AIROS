from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Any

import yaml
from rclpy.utilities import remove_ros_args


def _write_route_params(
    path: Path,
    graph_path: Path,
    use_sim_time: bool,
) -> None:
    params: dict[str, Any] = {
        'route_server': {
            'ros__parameters': {
                'use_sim_time': use_sim_time,
                'route_frame': 'map',
                'graph_filepath': str(graph_path),
                'graph_file_loader': 'geojson_graph_file_loader',
                'graph_file_saver': 'geojson_graph_file_saver',
                'geojson_graph_file_loader': {
                    'plugin': 'nav2_route::GeoJsonGraphFileLoader',
                },
                'geojson_graph_file_saver': {
                    'plugin': 'nav2_route::GeoJsonGraphFileSaver',
                },
                'costmap_topic': 'global_costmap/costmap_raw',
                'edge_cost_functions': ['DistanceScorer'],
                'DistanceScorer': {
                    'plugin': 'nav2_route::DistanceScorer',
                },
                'operations': ['AdjustSpeedLimit'],
                'AdjustSpeedLimit': {
                    'plugin': 'nav2_route::AdjustSpeedLimit',
                    'speed_tag': 'speed_limit',
                    'speed_limit_topic': 'speed_limit',
                },
            },
        },
        'lifecycle_manager_route': {
            'ros__parameters': {
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['route_server'],
            },
        },
    }
    path.write_text(yaml.safe_dump(params, sort_keys=False), encoding='utf-8')


def _start_process(
    command: list[str],
    log_path: Path,
) -> subprocess.Popen[str]:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_stream = log_path.open('w', encoding='utf-8')
    process = subprocess.Popen(
        command,
        stdout=log_stream,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    process._airos_log_stream = log_stream  # type: ignore[attr-defined]
    return process


def _stop_process(process: subprocess.Popen[str]) -> None:
    if process.poll() is None:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                break
            try:
                process.wait(timeout=1.5)
                break
            except subprocess.TimeoutExpired:
                continue
    log_stream = getattr(process, '_airos_log_stream', None)
    if log_stream is not None:
        log_stream.close()


def _run(
    command: list[str],
    timeout_sec: float,
) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_sec,
        )
    except subprocess.TimeoutExpired as exc:
        return subprocess.CompletedProcess(
            command,
            124,
            stdout=exc.stdout if isinstance(exc.stdout, str) else '',
            stderr=exc.stderr if isinstance(exc.stderr, str) else '',
        )


def _wait_for_active(log_path: Path, timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if log_path.exists():
            log_text = log_path.read_text(
                encoding='utf-8',
                errors='replace',
            )
            if 'Managed nodes are active' in log_text:
                return True
            if 'Activating' in log_text and 'Creating bond' in log_text:
                return True
        result = _run(
            ['ros2', 'action', 'list', '-t'],
            2.0,
        )
        if (
            result.returncode == 0
            and '/compute_route [nav2_msgs/action/ComputeRoute]' in result.stdout
        ):
            return True
        time.sleep(0.5)
    return False


def verify_route_graph(
    graph_path: Path,
    start_id: int,
    goal_id: int,
    timeout_sec: float,
    log_dir: Path,
) -> bool:
    with tempfile.TemporaryDirectory(prefix='airos_route_') as tmp:
        params_path = Path(tmp) / 'route_params.yaml'
        _write_route_params(params_path, graph_path, use_sim_time=False)
        server_log = log_dir / 'route_server.log'
        manager_log = log_dir / 'route_lifecycle_manager.log'
        server = _start_process(
            [
                'ros2',
                'run',
                'nav2_route',
                'route_server',
                '--ros-args',
                '--params-file',
                str(params_path),
            ],
            server_log,
        )
        manager = _start_process(
            [
                'ros2',
                'run',
                'nav2_lifecycle_manager',
                'lifecycle_manager',
                '--ros-args',
                '-r',
                '__node:=lifecycle_manager_route',
                '--params-file',
                str(params_path),
            ],
            manager_log,
        )
        try:
            if not _wait_for_active(manager_log, timeout_sec):
                return False
            goal = (
                f'{{start_id: {start_id}, goal_id: {goal_id}, '
                'use_start: false, use_poses: false}'
            )
            result = _run(
                [
                    'ros2',
                    'action',
                    'send_goal',
                    '/compute_route',
                    'nav2_msgs/action/ComputeRoute',
                    goal,
                ],
                timeout_sec,
            )
            output = result.stdout + result.stderr
            (log_dir / 'compute_route_goal.txt').write_text(
                output,
                encoding='utf-8',
            )
            return (
                result.returncode == 0
                and 'Goal finished with status: SUCCEEDED' in output
                and 'Route found with' in (
                    log_dir / 'route_server.log'
                ).read_text(encoding='utf-8', errors='replace')
            )
        finally:
            _stop_process(manager)
            _stop_process(server)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            'Verify AIROS nav2_route graph loading and route computation.'
        ),
    )
    parser.add_argument(
        '--graph',
        default='src/airos_nav/routes/single_floor_lab_route.geojson',
    )
    parser.add_argument('--start-id', type=int, default=1)
    parser.add_argument('--goal-id', type=int, default=4)
    parser.add_argument('--timeout-sec', type=float, default=25.0)
    parser.add_argument('--log-dir', default='log/route_graph_verifier')
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    ok = verify_route_graph(
        Path(args.graph).resolve(),
        args.start_id,
        args.goal_id,
        args.timeout_sec,
        Path(args.log_dir),
    )
    if not ok:
        raise SystemExit('route graph verification failed')
    print('route graph verification passed')


if __name__ == '__main__':
    main()
