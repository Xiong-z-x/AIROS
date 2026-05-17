#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

DEMO_TARGET="${DEMO_TARGET:-near_goal}"
case "$DEMO_TARGET" in
  near_goal)
    GOAL_X="${GOAL_X:-1.9}"
    GOAL_Y="${GOAL_Y:--9.2}"
    GOAL_Z="${GOAL_Z:-0.0}"
    ;;
  long_corridor)
    GOAL_X="${GOAL_X:-8.0}"
    GOAL_Y="${GOAL_Y:--9.0}"
    GOAL_Z="${GOAL_Z:-0.0}"
    ;;
  custom)
    GOAL_X="${GOAL_X:?GOAL_X is required when DEMO_TARGET=custom}"
    GOAL_Y="${GOAL_Y:?GOAL_Y is required when DEMO_TARGET=custom}"
    GOAL_Z="${GOAL_Z:-0.0}"
    ;;
  *)
    printf 'Unknown DEMO_TARGET=%s. Use near_goal, long_corridor, or custom.\n' "$DEMO_TARGET" >&2
    exit 2
    ;;
esac
PROBE_DURATION_SEC="${PROBE_DURATION_SEC:-55}"
PROBE_SAMPLE_PERIOD_SEC="${PROBE_SAMPLE_PERIOD_SEC:-2}"
STARTUP_WAIT_SEC="${STARTUP_WAIT_SEC:-35}"
GOAL_PUBLISH_COUNT="${GOAL_PUBLISH_COUNT:-5}"
GOAL_PUBLISH_RATE_HZ="${GOAL_PUBLISH_RATE_HZ:-1}"
ACCEPTANCE_TOLERANCE_M="${ACCEPTANCE_TOLERANCE_M:-0.30}"
TERRAIN_GOAL_MAX_Z="${TERRAIN_GOAL_MAX_Z:-0.45}"
MAX_LOG_RUNS_TO_KEEP="${MAX_LOG_RUNS_TO_KEEP:-8}"
RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
LOG_ROOT="${LOG_ROOT:-log/fast_lio_single_floor_demo}"
RUN_DIR="$LOG_ROOT/$RUN_ID"
LAUNCH_LOG="$RUN_DIR/launch.log"
PROBE_LOG="$RUN_DIR/probe_${DEMO_TARGET}_goal_${GOAL_X}_${GOAL_Y}.jsonl"

DRY_RUN=false
if [[ "${1:-}" == "--dry-run" ]]; then
  DRY_RUN=true
fi

run() {
  printf '+ %s\n' "$*"
  if [[ "$DRY_RUN" == "false" ]]; then
    "$@"
  fi
}

cleanup_old_runs() {
  [[ "$MAX_LOG_RUNS_TO_KEEP" =~ ^[0-9]+$ ]] || return 0
  if (( MAX_LOG_RUNS_TO_KEEP <= 0 )); then
    return 0
  fi
  mkdir -p "$LOG_ROOT"
  mapfile -t old_runs < <(
    find "$LOG_ROOT" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' \
      | sort -rn \
      | awk -v keep="$MAX_LOG_RUNS_TO_KEEP" 'NR > keep {print $2}'
  )
  for old_run in "${old_runs[@]}"; do
    run rm -rf -- "$old_run"
  done
}

publish_goal() {
  run ros2 topic pub \
    --times "$GOAL_PUBLISH_COUNT" \
    --rate "$GOAL_PUBLISH_RATE_HZ" \
    /terrain_goal_pose \
    geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: ${GOAL_X}, y: ${GOAL_Y}, z: ${GOAL_Z}}, orientation: {w: 1.0}}}"
}

summarize_probe() {
  python3 - "$PROBE_LOG" "$LAUNCH_LOG" "$ACCEPTANCE_TOLERANCE_M" <<'PY'
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

probe_path = Path(sys.argv[1])
launch_log_path = Path(sys.argv[2])
tolerance = float(sys.argv[3])
rows = [json.loads(line) for line in probe_path.read_text().splitlines() if line.strip()]
if not rows:
    raise SystemExit('probe log is empty')
launch_log = launch_log_path.read_text(errors='replace')

def max_value(name: str) -> float | int | None:
    values = [row.get(name) for row in rows if row.get(name) is not None]
    return max(values) if values else None

def contains_launch_evidence(pattern: str) -> bool:
    return pattern in launch_log

last = rows[-1]
wheel_goal_xy_distance = last.get('wheel_goal_xy_distance')
gazebo_goal_xy_distance = last.get('gazebo_goal_xy_distance')
fast_lio_goal_xy_distance = last.get('fast_lio_goal_xy_distance')
planner_received_goal = contains_launch_evidence('received terrain goal:')
planner_started_direct_tracking = contains_launch_evidence(
    'started terrain-guided direct tracking:'
)
planner_reached_goal = contains_launch_evidence('terrain direct tracking goal reached')
direct_diagnostics_seen = contains_launch_evidence('direct tracking diagnostics:')
accepted = (
    gazebo_goal_xy_distance is not None
    and math.isfinite(float(gazebo_goal_xy_distance))
    and float(gazebo_goal_xy_distance) <= tolerance
    and planner_received_goal
    and planner_started_direct_tracking
    and planner_reached_goal
)
summary = {
    'accepted': accepted,
    'acceptance_tolerance_m': tolerance,
    'samples': len(rows),
    'laser_map_points_max': max_value('laser_map_points'),
    'pct_path_poses_max': max_value('pct_path_poses'),
    'pct_path_max_z': max_value('pct_path_max_z'),
    'cmd_vel_nav_count_max': max_value('cmd_vel_nav_count'),
    'cmd_vel_smoothed_count_max': max_value('cmd_vel_smoothed_count'),
    'base_cmd_count_max': max_value('base_cmd_count'),
    'planner_received_goal': planner_received_goal,
    'planner_started_direct_tracking': planner_started_direct_tracking,
    'planner_reached_goal': planner_reached_goal,
    'direct_diagnostics_seen': direct_diagnostics_seen,
    'fast_lio_goal_xy_distance': fast_lio_goal_xy_distance,
    'wheel_goal_xy_distance': wheel_goal_xy_distance,
    'gazebo_goal_xy_distance': gazebo_goal_xy_distance,
}
print(json.dumps(summary, ensure_ascii=False, sort_keys=True))
if not accepted:
    raise SystemExit(2)
PY
}

if [[ "$DRY_RUN" == "true" ]]; then
  printf '[DRY-RUN] FAST-LIO/PCT single-floor demo target=%s goal=(%s,%s,%s) would write %s\n' \
    "$DEMO_TARGET" "$GOAL_X" "$GOAL_Y" "$GOAL_Z" "$RUN_DIR"
fi

cleanup_old_runs
run bash scripts/cleanup_airos_runtime.sh
run mkdir -p "$RUN_DIR"

if [[ "$DRY_RUN" == "true" ]]; then
  printf '+ ros2 launch airos_experiments visual_fast_lio_navigation.launch.py ... >%s 2>&1 &\n' "$LAUNCH_LOG"
  printf '+ sleep %s\n' "$STARTUP_WAIT_SEC"
  publish_goal
  printf '+ ros2 run airos_experiments cross_level_evidence_probe --output %s ...\n' "$PROBE_LOG"
  printf '+ summarize probe with gazebo_goal_xy_distance <= %s\n' "$ACCEPTANCE_TOLERANCE_M"
  exit 0
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -u

run ros2 launch airos_experiments visual_fast_lio_navigation.launch.py \
  gui:=false \
  rviz:=false \
  terrain_send_nav2_goals:=true \
  terrain_execution_mode:=direct \
  terrain_goal_z_policy:=nearest_z \
  terrain_goal_min_z:=-1.0 \
  terrain_goal_max_z:="$TERRAIN_GOAL_MAX_Z" \
  terrain_odom_topic:=/odom \
  log_level:=info \
  >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

finish() {
  if [[ "${LAUNCH_PID:-}" =~ ^[0-9]+$ ]]; then
    kill "$LAUNCH_PID" 2>/dev/null || true
  fi
  bash scripts/cleanup_airos_runtime.sh >/dev/null 2>&1 || true
}
trap finish EXIT

run sleep "$STARTUP_WAIT_SEC"
publish_goal
run ros2 run airos_experiments cross_level_evidence_probe \
  --output "$PROBE_LOG" \
  --duration-sec "$PROBE_DURATION_SEC" \
  --sample-period-sec "$PROBE_SAMPLE_PERIOD_SEC" \
  --goal-x "$GOAL_X" \
  --goal-y "$GOAL_Y"

summarize_probe
