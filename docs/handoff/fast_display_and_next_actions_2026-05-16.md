# Fast Display And Next Actions - 2026-05-16

Status: action-preparation snapshot. This file does not claim new runtime
success; it summarizes the fastest current delivery route and the next bounded
work packages.

## Goal Order

1. Ship a visible single-floor result first:
   FAST-LIO2 map growth -> SLAM traversability graph -> PCT-style `/pct_path`
   -> direct terrain tracking -> smoother/collision monitor -> Gazebo physical
   arrival.
2. Keep cross-level navigation as a second-stage branch until physical ascent is
   proven with Gazebo pose z, not just high `/pct_path`.
3. Use external/open-source assets or code only when they reduce time to a
   verified result. Do not import a large upstream stack unless the interface
   and acceptance test are clear.

## Current Best Demo

Use this command for the current strongest display path:

```bash
DEMO_TARGET=long_corridor bash scripts/run_fast_lio_single_floor_demo.sh
```

Latest accepted evidence:

- `log/fast_lio_single_floor_demo/near_goal_after_scan_index_20260516_223232/`
  reached the Gazebo/wheel goal within `0.222402m`.
- `log/fast_lio_single_floor_demo/long_corridor_after_scan_index_20260516_223440/`
  reached the Gazebo/wheel goal within `0.294605m`, produced 10 `/pct_path`
  poses, and kept `/pct_path_max_z=-0.044546`.
- Both runs had live `/Laser_map_world` growth, nonzero command propagation to
  the base, and zero `/slam_scan` stale warnings.

This is the fastest credible user-facing result. It proves the advanced
single-floor SLAM/planning/execution chain in Gazebo. It does not prove
cross-level navigation.

## Cross-Level Stop Condition

Do not repeat long cross-level trials on the same wheel surrogate and same
large map unless one of these inputs changes first:

1. a smaller multilevel smoke world with an explicit physical-z acceptance,
2. a ramp-capable execution/model branch,
3. or a targeted ramp-entry/direct-tracking fix validated by a short ramp smoke
   test before the full cross-level run.

Latest cross-level evidence:

- `log/cross_level_after_single_floor_refresh_20260516_223943/`:
  `/pct_path_max_z=2.138636`, base command `3337`, Gazebo z stayed
  `-0.005001`.
- `log/cross_level_after_zigzag_lookahead_fix_20260516_225624/`:
  `/pct_path_max_z=2.249888`, base command `2686`, Gazebo z stayed
  `-0.005001`.

Fact: high path generation, live SLAM growth, `/slam_scan` freshness, and the
command chain are not the current main blockers.

Inference: the remaining blocker is physical execution around ramp entry,
surface support, heading/approach, or the current surrogate/model interaction
with the large multilevel scene.

## Recommended Next Work Packages

### Package A: Demo Polish, Highest Priority

Purpose: make the current single-floor result easy to run and present.

Actions:

- Keep `scripts/run_fast_lio_single_floor_demo.sh` as the canonical command.
- Add only small display/readme polish if needed.
- Do not change planner internals unless the demo script regresses.

Acceptance:

- `DEMO_TARGET=long_corridor bash scripts/run_fast_lio_single_floor_demo.sh`
  returns `accepted: true`.
- Final Gazebo distance is `<= 0.30m`.
- `/Laser_map_world`, `/pct_path`, `/cmd_vel_nav`, smoothed command, and base
  command all have evidence in the run log.

### Package B: Cross-Level Future Work, Deferred

Purpose: keep the known cross-level gap documented without making this handoff
spend more runtime on it.

Actions:

- Do not run another cross-level smoke in the current handoff.
- If cross-level work is reopened later, start from a clean branch and define a
  small physical-z acceptance target before any runtime trial.
- Reuse the retained interfaces (`/terrain_goal_pose`, `/pct_path`, odom,
  command topics, and Gazebo pose evidence) instead of adding another ad hoc
  runner in the dirty baseline.

Acceptance:

- High `/pct_path` appears from `/Laser_map_world`.
- Gazebo pose z crosses the landing threshold and stays there long enough to be
  sampled.
- The path does not jump across stairs, walls, unsupported deck edges, or
  unmapped voids.

Latest negative evidence:

- `log/fast_lio_multilevel_smoke/quick_multilevel_smoke_20260516_232258/`
  used the far upper-lab goal `(7.2,7.4,0.9)`. It produced live map growth and
  base commands, but `/pct_path_max_z` only reached `0.235028`, Gazebo z max was
  `0.335047`, and final goal distance stayed `2.332921m`.
- `log/fast_lio_multilevel_smoke/ramp_entry_smoke_20260516_232735/` retargeted
  the smoke to the ramp/entry goal `(0.4,3.6,0.65)`. The planner received the
  goal and started direct tracking, command counts reached
  `/cmd_vel_nav=801`, smoother `861`, base `874`, and `/Laser_map_world`
  reached `291189` sampled points. It still failed: `/pct_path` was not sampled,
  Gazebo z max was only `0.060886`, and final goal distance was `4.120866m`.
- `log/fast_lio_multilevel_smoke/ramp_corridor_guard_20260516_233809/` started
  the evidence probe before publishing the goal, so `/pct_path` was sampled.
  It produced `/Laser_map_world=271731`, `/pct_path_poses=33`,
  `/pct_path_max_z=0.984888`, `/cmd_vel_nav=697`, smoother/base `769`, but
  Gazebo z max was only `0.067521` and final goal distance was `5.047544m`.
  Launch logs show one regressive high path was deferred, then a later direct
  path still began with low-floor target `(2.20,-3.05,-0.14)` instead of a
  physical ramp-corridor entry.

Inference: the smaller smoke has now reproduced the core cross-level execution
gap without the large-map runtime cost. The first static guard was only
partially effective: it deferred one bad final path but did not prevent the
next accepted path from driving along a low-floor detour. The immediate next
fix should apply the same low-floor corridor rejection to the initial final-goal
acceptance path, not only to pending-goal replanning, before another runtime
attempt.

Current cleanup note: the temporary multilevel smoke runner is intentionally
not part of the final single-floor baseline. Keep the evidence and problem
summary, but do not ask the next model to run a removed helper script.

### Package C: Model/Control Branch, Only If Needed

Purpose: improve physical climbing capability without destabilizing the current
demo.

Actions:

- Keep current wheel-equivalent model as the stable baseline.
- Evaluate a ramp-capable surrogate or Unitree/Go2W-style model in a separate
  branch/scope.
- Treat imported visual meshes as visual-only unless a verified control and
  contact model is also present.

Acceptance:

- A direct-command ramp smoke test raises Gazebo z.
- The model remains controllable through the existing ROS 2 command chain or a
  documented adapter.
- No claim of cross-level autonomy is made until the full SLAM/planning/control
  chain reaches the high goal physically.

## Reporting Rules

Use these labels in all status updates:

- Fact: observed from current code, launch/config, tests, or runtime logs.
- Inference: derived from facts but not directly observed.
- Pending: not yet verified.

Never write that cross-level navigation is complete unless evidence includes:

- `/Laser_map_world` growth,
- `/pct_path max z > 2.0` for the current large multilevel goal,
- command propagation through `/cmd_vel_nav`, smoother, collision monitor, and
  base controller,
- and Gazebo physical pose z reaching the high platform with final goal
  distance accepted.
