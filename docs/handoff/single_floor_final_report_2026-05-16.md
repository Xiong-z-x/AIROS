# Single-Floor Final Report - 2026-05-16

Status: final single-floor display baseline. This report intentionally stops
before cross-level completion work.

## Final Scope

Current fixed result:

- Fortress + Go2W surrogate + FAST-LIO2 + PCT-style terrain planner.
- Single-floor SLAM map growth, path planning, direct execution, safety chain,
  and Gazebo physical arrival are accepted.
- Cross-level interfaces are left in place, but cross-level physical ascent is
  not claimed as complete.

Do not spend more time on cross-level runtime in this handoff unless the user
explicitly reopens that phase.

## Fixed Demo Command

Use this as the canonical single-floor result:

```bash
DEMO_TARGET=long_corridor bash scripts/run_fast_lio_single_floor_demo.sh
```

Fallback quick smoke:

```bash
DEMO_TARGET=near_goal bash scripts/run_fast_lio_single_floor_demo.sh
```

The script clears runtime leftovers, starts
`visual_fast_lio_navigation.launch.py` headless, publishes repeated
`/terrain_goal_pose` goals, samples SLAM/path/command/Gazebo evidence, and
accepts only when Gazebo physical distance is within `0.30m`.

## Accepted Evidence

### long_corridor

Evidence path:
`log/fast_lio_single_floor_demo/long_corridor_after_scan_index_20260516_223440/`

- Goal: `(8.0, -9.0, 0.0)`.
- `/Laser_map_world` max sampled points: `351188`.
- `/pct_path_poses_max`: `10`.
- `/pct_path_max_z`: `-0.044546`.
- `/cmd_vel_nav_count_max`: `573`.
- `/cmd_vel_smoothed_count_max`: `645`.
- base command count max: `660`.
- final FAST-LIO goal XY distance: `0.201657m`.
- final wheel/Gazebo goal XY distance: `0.294605m`.
- launch log records:
  - `received terrain goal`
  - `started terrain-guided direct tracking`
  - `pending final goal became reachable after FAST-LIO map update`
  - `terrain direct tracking goal reached`

This is the best current display result.

### near_goal

Evidence path:
`log/fast_lio_single_floor_demo/near_goal_after_scan_index_20260516_223232/`

- Goal: `(1.9, -9.2, 0.0)`.
- `/Laser_map_world` max sampled points: `248275`.
- `/cmd_vel_nav_count_max`: `200`.
- `/cmd_vel_smoothed_count_max`: `235`.
- base command count max: `235`.
- final FAST-LIO goal XY distance: `0.257631m`.
- final wheel/Gazebo goal XY distance: `0.222402m`.
- launch log records repeated goal reception, direct tracking start, and
  `terrain direct tracking goal reached`.

Use this only when a shorter, faster smoke is needed.

## Current Planning Algorithm

Fact:

- The current runtime planner is `terrain_pct_planner.py`.
- It is PCT-style, not upstream CUDA PCT-planner and not RL.
- It builds a traversability graph from live FAST-LIO SLAM point cloud
  `/Laser_map_world`.
- It publishes `/pct_path`.
- Direct terrain tracking consumes the path and publishes `/cmd_vel_nav`.
- Nav2 is used in `safety_only` mode for `velocity_smoother` and
  `collision_monitor`, not as the main global planner for the FAST-LIO path.

Single-floor display parameters that must stay fixed:

- `terrain_goal_z_policy:=nearest_z`
- `terrain_goal_min_z:=-1.0`
- `terrain_goal_max_z:=0.45`
- `terrain_odom_topic:=/odom`

These prevent low-floor goals from being attracted to high structures and make
Gazebo/wheel odom the physical acceptance basis.

## Interfaces Left For Later Work

### Goal Interface

- Topic: `/terrain_goal_pose`
- Type: `geometry_msgs/msg/PoseStamped`
- Current single-floor use: z can be `0.0`, with low-floor z-window from the
  launch/script profile.
- Cross-level future use: z must be meaningful. Do not use a pure 2D goal tool
  to claim high-floor target selection.

Existing helper:

```bash
ros2 run airos_experiments publish_terrain_goal --x X --y Y --z Z
```

### Launch Interface

Main entry:

```bash
ros2 launch airos_experiments visual_fast_lio_navigation.launch.py ...
```

Important parameters:

- `terrain_map_source`
- `slam_map_topic`
- `path_topic`
- `terrain_goal_z_policy`
- `terrain_goal_min_z`
- `terrain_goal_max_z`
- `terrain_odom_topic`
- `nav_stack_mode`
- `terrain_execution_mode`

### Evidence Topics

Keep these for future verification:

- `/Laser_map_world`
- `/pct_path`
- `/cmd_vel_nav`
- `/cmd_vel_smoothed`
- `/diff_drive_controller/cmd_vel_unstamped`
- `/odom`
- `/fast_lio_odom_world`
- Gazebo model pose

## Difficulties And Specific Problems Found

1. Single-floor goals were initially vulnerable to high-structure attraction.
   Fix: use `nearest_z` plus `terrain_goal_max_z:=0.45` for the display profile.

2. FAST-LIO aligned odom and Gazebo/wheel odom can disagree. A planner
   `goal reached` event based only on aligned odom is not enough for physical
   acceptance. Fix for display: `terrain_odom_topic:=/odom` and acceptance by
   Gazebo/wheel distance.

3. One-shot volatile goal publishing can be missed during ROS discovery or graph
   initialization. Fix: the demo script publishes the goal several times.

4. `/pct_path` may be transient after direct tracking reaches the goal. For
   accepted single-floor runs, use both probe data and launch log evidence.

5. The long single-floor target was previously borderline because reachable
   SLAM graph nodes could stop just outside the final tolerance. Current accepted
   run proves the fixed profile reaches `0.294605m`, under the `0.30m`
   threshold. Do not loosen this threshold to hide failures.

6. `/slam_scan` freshness previously risked collision monitor source timeout
   because projector support filtering was too expensive. It was fixed with
   spatial support bins; do not revert to per-point full-cloud scans.

7. `collision_monitor` lifecycle activation previously had a startup race.
   `lifecycle_activator` now waits/retries service availability.

8. Runtime cleanup previously missed `nav2_map_server/map_saver_server`.
   `scripts/cleanup_airos_runtime.sh` now terminates it.

9. Cross-level high `/pct_path` is not the same as physical climb. Several runs
   produced high paths and fresh command chains while Gazebo z stayed near
   ground level.

10. The Go2W surrogate is still wheel-equivalent for physics. It is acceptable
    for the fixed single-floor display, but it is a poor basis for stair/ramp
    physical claims.

11. Recent cross-level debugging exposed low-floor detour and frontier-entry
    issues. Those are documented as future work only. They are not part of this
    final single-floor baseline.

## Current Agent Situation Summary

This handoff was narrowed by the user to one deliverable: freeze the best
single-floor navigation result, leave clean interfaces, and stop.

What is fixed now:

- Best current display result is `long_corridor`, with Gazebo/wheel final XY
  distance `0.294605m` under the fixed `0.30m` threshold.
- Short fallback result is `near_goal`, with Gazebo/wheel final XY distance
  `0.222402m`.
- The canonical command is
  `DEMO_TARGET=long_corridor bash scripts/run_fast_lio_single_floor_demo.sh`.
- The interface surface is left through `/terrain_goal_pose`,
  `visual_fast_lio_navigation.launch.py` parameters, `/pct_path`, command
  topics, odom topics, and Gazebo pose evidence.

What must not be overstated:

- This is not a completed cross-level navigation result.
- High `/pct_path` evidence is not physical ascent evidence.
- Current Go2W physics is wheel-equivalent and should not be used to claim
  stair/ramp capability.
- FAST-LIO odom and Gazebo/wheel odom can diverge, so physical acceptance must
  stay based on `/odom` plus Gazebo pose.

Current work state:

- No further runtime smoke should be run for this handoff unless the user
  explicitly reopens runtime verification.
- No new planner rewrite should be started from this state.
- If another model continues, it should read this report first and either
  polish the single-floor demo presentation or start a clean, separately scoped
  physical climbing branch.
- The main remaining risk is presentation polish, not single-floor evidence:
  the current result is usable, but the worktree may still contain dirty
  experimental cross-level files from prior attempts.

## Files To Treat As Baseline

- `scripts/run_fast_lio_single_floor_demo.sh`
- `src/airos_experiments/launch/visual_fast_lio_navigation.launch.py`
- `src/airos_experiments/airos_experiments/terrain_pct_planner.py`
- `src/airos_experiments/airos_experiments/slam_scan_projector.py`
- `scripts/cleanup_airos_runtime.sh`
- `README.md`
- this report

## What Not To Do Next

- Do not run more cross-level smoke tests in the current handoff.
- Do not claim cross-level completion.
- Do not import a full external Unitree stack into this dirty worktree just to
  try climbing.
- Do not switch back to SDF truth planning for the FAST-LIO demo.
- Do not relax the `0.30m` single-floor acceptance threshold.

## Final Recommendation

Freeze the current single-floor result as the project display baseline. Future
work should start from a clean branch or clean handoff with one of these scoped
goals:

1. polish the single-floor demo presentation,
2. replace the physical model/control for climbing in a separate branch,
3. or build a small cross-level map with physical-z acceptance before returning
   to the large multilevel scene.
