from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

from rclpy.utilities import remove_ros_args

from airos_experiments.nav_trial_runner import _load_missions

ROUTE_WAYPOINT_USE_ROUTE_ARGUMENT = 'use_route:=true'


def _open_log(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    return path.open('w', encoding='utf-8')


def _terminate_process(
    process: subprocess.Popen[str],
    timeout_sec: float,
) -> None:
    if process.poll() is not None:
        return

    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=timeout_sec)
        return
    except subprocess.TimeoutExpired:
        pass

    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=timeout_sec)
        return
    except subprocess.TimeoutExpired:
        pass

    os.killpg(process.pid, signal.SIGKILL)
    process.wait(timeout=timeout_sec)


def _process_group_has_members(pgid: int) -> bool:
    result = subprocess.run(
        ['ps', '-eo', 'pgid='],
        check=False,
        capture_output=True,
        text=True,
    )
    return any(
        line.strip() == str(pgid)
        for line in result.stdout.splitlines()
    )


def _wait_for_process_group_exit(pgid: int, timeout_sec: float) -> None:
    deadline = time.monotonic() + max(0.0, timeout_sec)
    while time.monotonic() < deadline:
        if not _process_group_has_members(pgid):
            return
        time.sleep(0.2)
    if _process_group_has_members(pgid):
        os.killpg(pgid, signal.SIGKILL)


def _start_process(
    command: list[str],
    log_path: Path,
    env: dict[str, str],
) -> subprocess.Popen[str]:
    log_stream = _open_log(log_path)
    process = subprocess.Popen(
        command,
        stdout=log_stream,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
        start_new_session=True,
    )
    process._airos_log_stream = log_stream  # type: ignore[attr-defined]
    return process


def _close_process_log(process: subprocess.Popen[str]) -> None:
    log_stream = getattr(process, '_airos_log_stream', None)
    if log_stream is not None:
        log_stream.close()


def _wait_for_startup(seconds: float) -> None:
    deadline = time.monotonic() + max(0.0, seconds)
    while time.monotonic() < deadline:
        time.sleep(min(0.5, deadline - time.monotonic()))


def _run_trial_command(
    mission_file: Path,
    mission_id: str,
    output: Path,
    timeout_sec: float,
    env: dict[str, str],
    use_route_waypoints: bool,
    route_graph: str,
) -> tuple[int, str, str]:
    command = [
        'ros2',
        'run',
        'airos_experiments',
        'run_nav_trials',
        '--mission',
        str(mission_file),
        '--mission-id',
        mission_id,
        '--count',
        '1',
        '--reset-sim',
        '--output',
        str(output),
        '--ros-args',
        '-p',
        'use_sim_time:=true',
    ]
    if use_route_waypoints:
        command.extend([
            '-p',
            'use_route_waypoints:=true',
            '-p',
            f'route_graph_path:={route_graph}',
        ])
    try:
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_sec,
            env=env,
        )
        return result.returncode, result.stdout, result.stderr
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout if isinstance(exc.stdout, str) else ''
        stderr = exc.stderr if isinstance(exc.stderr, str) else ''
        return 124, stdout, stderr


def _fallback_result(
    mission_id: str,
    reason: str,
    returncode: int,
    stdout: str,
    stderr: str,
) -> dict[str, Any]:
    return {
        'mission_id': mission_id,
        'status': 0,
        'success': False,
        'reason': reason,
        'elapsed_sec': None,
        'path_length_m': None,
        'emergency_stop_count': None,
        'collision_count': None,
        'collision_metric_source': 'scan_range_threshold',
        'minimum_obstacle_distance_m': None,
        'mean_cmd_period_sec': None,
        'max_cmd_period_sec': None,
        'runner_returncode': returncode,
        'runner_stdout_tail': stdout[-1200:],
        'runner_stderr_tail': stderr[-1200:],
    }


def _append_jsonl(path: Path, result: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(result, ensure_ascii=False) + '\n')


def _read_first_jsonl(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    for line in path.read_text(encoding='utf-8').splitlines():
        if line.strip():
            return json.loads(line)
    return None


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            'Run AIROS missions with a clean sim/nav process per trial.'
        )
    )
    parser.add_argument('--mission', required=True)
    parser.add_argument('--mission-id', action='append', default=[])
    parser.add_argument('--count', type=int, default=1)
    parser.add_argument(
        '--output',
        default='log/airos_nav_trials_clean_batch.jsonl',
    )
    parser.add_argument('--log-dir', default='log/clean_batch')
    parser.add_argument('--sim-startup-sec', type=float, default=14.0)
    parser.add_argument('--nav-startup-sec', type=float, default=12.0)
    parser.add_argument('--trial-timeout-sec', type=float, default=130.0)
    parser.add_argument('--shutdown-timeout-sec', type=float, default=8.0)
    parser.add_argument('--attempts', type=int, default=1)
    parser.add_argument('--world', default='single_floor_lab')
    parser.add_argument(
        '--map',
        default='src/airos_nav/maps/single_floor_lab.yaml',
    )
    parser.add_argument('--planner-profile', default='baseline')
    parser.add_argument('--use-route-waypoints', action='store_true')
    parser.add_argument(
        '--route-graph',
        default='src/airos_nav/routes/single_floor_lab_route.geojson',
    )
    parser.add_argument('--dynamic-obstacles', action='store_true')
    parser.add_argument('--physical-dynamic-obstacles', action='store_true')
    parser.add_argument('--open-source-scene-assets', action='store_true')
    parser.add_argument('--robot-visual-profile', default='analytic')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--rviz', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    mission_file = Path(args.mission)
    missions = _load_missions(mission_file)
    if args.mission_id:
        selected = set(args.mission_id)
        missions = [
            mission for mission in missions if mission.mission_id in selected
        ]
        missing = selected - {mission.mission_id for mission in missions}
        if missing:
            raise RuntimeError(f'unknown mission_id: {sorted(missing)}')

    trial_missions = [
        missions[index % len(missions)]
        for index in range(max(0, args.count))
    ]
    if args.dry_run:
        print(
            json.dumps(
                {
                    'trials': len(trial_missions),
                    'mission_ids': [
                        mission.mission_id for mission in trial_missions
                    ],
                    'mode': 'clean_process_per_trial',
                },
                ensure_ascii=False,
            )
        )
        return

    output = Path(args.output)
    if output.exists():
        output.unlink()
    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()

    attempts = max(1, args.attempts)
    for index, mission in enumerate(trial_missions, start=1):
        mission_id = mission.mission_id
        final_result: dict[str, Any] | None = None
        final_returncode = 1
        final_sim_log = ''
        final_nav_log = ''
        for attempt in range(1, attempts + 1):
            sim_log = log_dir / f'{index:03d}_{mission_id}_a{attempt}_sim.log'
            nav_log = log_dir / f'{index:03d}_{mission_id}_a{attempt}_nav.log'
            attempt_output = (
                log_dir / f'{index:03d}_{mission_id}_a{attempt}.jsonl'
            )
            if attempt_output.exists():
                attempt_output.unlink()
            sim_process = _start_process(
                [
                    'ros2',
                    'launch',
                    'airos_sim',
                    'sim.launch.py',
                    f'world:={args.world}',
                    f'gui:={str(args.gui).lower()}',
                    f'rviz:={str(args.rviz).lower()}',
                    'dynamic_obstacles:='
                    f'{str(args.dynamic_obstacles).lower()}',
                    'physical_dynamic_obstacles:='
                    f'{str(args.physical_dynamic_obstacles).lower()}',
                    'open_source_scene_assets:='
                    f'{str(args.open_source_scene_assets).lower()}',
                    f'robot_visual_profile:={args.robot_visual_profile}',
                    f'dynamic_obstacle_seed:={mission.dynamic_obstacle_seed}',
                ],
                sim_log,
                env,
            )
            nav_process: subprocess.Popen[str] | None = None
            try:
                _wait_for_startup(args.sim_startup_sec)
                nav_process = _start_process(
                    [
                        'ros2',
                        'launch',
                        'airos_nav',
                        'nav.launch.py',
                        'rviz:=false',
                        f'map:={args.map}',
                        f'route_graph:={args.route_graph}',
                        f'planner_profile:={args.planner_profile}',
                        f'use_route:={str(args.use_route_waypoints).lower()}',
                    ],
                    nav_log,
                    env,
                )
                _wait_for_startup(args.nav_startup_sec)
                returncode, stdout, stderr = _run_trial_command(
                    mission_file,
                    mission_id,
                    attempt_output,
                    args.trial_timeout_sec,
                    env,
                    args.use_route_waypoints,
                    args.route_graph,
                )
                final_returncode = returncode
                final_sim_log = sim_log.as_posix()
                final_nav_log = nav_log.as_posix()
                if returncode == 0:
                    final_result = _read_first_jsonl(attempt_output)
                else:
                    final_result = _fallback_result(
                        mission_id,
                        'runner_failed',
                        returncode,
                        stdout,
                        stderr,
                    )
                if final_result is not None and final_result.get('success'):
                    break
            finally:
                if nav_process is not None:
                    nav_pgid = nav_process.pid
                    _terminate_process(
                        nav_process,
                        args.shutdown_timeout_sec,
                    )
                    _wait_for_process_group_exit(
                        nav_pgid,
                        args.shutdown_timeout_sec,
                    )
                    _close_process_log(nav_process)
                sim_pgid = sim_process.pid
                _terminate_process(sim_process, args.shutdown_timeout_sec)
                _wait_for_process_group_exit(
                    sim_pgid,
                    args.shutdown_timeout_sec,
                )
                _close_process_log(sim_process)

        if final_result is None:
            raise RuntimeError(f'no result produced for {mission_id}')
        _append_jsonl(output, final_result)
        print(
            json.dumps(
                {
                    'trial_index': index,
                    'mission_id': mission_id,
                    'attempts': attempts,
                    'success': bool(final_result.get('success')),
                    'reason': final_result.get('reason'),
                    'execution_mode': final_result.get('execution_mode'),
                    'runner_returncode': final_returncode,
                    'sim_log': final_sim_log,
                    'nav_log': final_nav_log,
                },
                ensure_ascii=False,
            )
        )


if __name__ == '__main__':
    main()
