# 2026-05-16 Cross-Level Execution Gap Snapshot

Status: current diagnostic snapshot for the next model.

## Fact

- The active chain remains `/Laser_map_world -> SLAM traversability graph -> /pct_path -> direct terrain tracking -> velocity_smoother -> collision_monitor -> diff_drive_controller`.
- `visual_fast_lio_navigation.launch.py` is still the main entry point for this chain.
- A single-floor Nav2 clean batch passed 4 of 4 `single_floor_lab` missions on 2026-05-16 with zero emergency stops and zero collision events.
- `log/cross_level_full_20260516_113311/probe.jsonl` contains a 360-second headless run after publishing `/terrain_goal_pose` at `(6.0, 13.0, 2.2)`.
- In that run, `/Laser_map_world` reached `791828` sampled points.
- In that run, `/pct_path` reached `pct_path_max_z: 2.138301`, so high path generation was reproduced.
- In that run, `/cmd_vel_nav` was nonzero in 71 of 72 probe samples, `/cmd_vel_smoothed` was nonzero in 38 of 72, and base controller command was nonzero in 37 of 72.
- The final probe sample recorded:
  - `fast_lio_xyz: [2.975368, 1.832708, 0.360271]`
  - `gazebo_xyz: [2.8349, 1.541415, -0.005001]`
  - `gazebo_goal_xy_distance: 11.887684`
- `launch.log` shows the final high path became reachable with `path_nodes=37`.
- Direct tracking diagnostics then advanced to high-surface waypoints, including `slam_deck` targets around z `0.76-0.78` and a later `slam_ramp` target around z `0.50`, while Gazebo physical z stayed near `-0.005`.
- A later probe identified one non-physical graph issue: the first high waypoint
  after the low-floor segment could be a sparse `slam_deck` edge target near
  `(0.60, 0.85, 0.75)`, which maps to `second_floor_deck`, not to a ramp or
  stair entry.
- `test_slam_graph_rejects_sparse_floor_to_deck_edge_jump` now covers this
  failure mode. `_sparse_bridge_uses_vertical_structure` rejects sparse floor
  bridges whose endpoint height jump exceeds the small transition threshold,
  while preserving sparse ramp and non-floor step-landing bridge tests.
- `direct_tracking_gate_z` now gates direct waypoint progress with nearby terrain
  surface height instead of blindly using aligned odom z.
- After the sparse-bridge guard, `log/cross_level_bridge_guard_20260516_122021/`
  reproduced high path generation with `pct_path_max_z: 2.065411`.
- In that run, the first high waypoint was a `slam_ramp` target
  `(-2.39, 0.83, 0.73)`, not the earlier `second_floor_deck` shortcut.
- The same run produced partial physical climb evidence:
  `max_gazebo_z: 0.217646`, but the final Gazebo z returned to about ground
  level and the final goal distance remained `15.566393m`.
- A post-run log/probe correlation showed that, after `/pct_path` max z exceeded
  `2.0`, `/cmd_vel_nav` stayed nonzero in 70/71 command samples, but
  `/cmd_vel_smoothed` and base command were zero in 32/71 samples. Around the
  Gazebo z peak at `elapsed_sec: 320.902`, `/cmd_vel_nav_norm` was `0.45`,
  `/cmd_vel_smoothed_norm` and `base_cmd_norm` were `0.146667`.
- The same correlation showed repeated `collision_monitor` SlowZone events on
  the ramp segment and direct tracking slope `speed_limit=0.16`.
- `cross_level_evidence_probe` now records per-command-topic message counts and
  message age fields (`cmd_vel_nav_count`, `cmd_vel_nav_age_sec`,
  `cmd_vel_smoothed_count`, `cmd_vel_smoothed_age_sec`, `base_cmd_count`,
  `base_cmd_age_sec`) so the next runtime run can distinguish real command
  timeout/dropout from 5-second sampling artifacts.
- `log/cross_level_cmd_age_20260516_124501/` used the enhanced probe. It
  reproduced high path generation with `pct_path_max_z: 2.074105`, but Gazebo z
  stayed near `-0.005001`.
- In that run, `/cmd_vel_nav` message age reached `3.91s`,
  `/cmd_vel_smoothed` and base command age reached `2.257s`, and
  `/cmd_vel_smoothed`/base command were zero in 38 of 71 samples. This exceeds
  `velocity_smoother.velocity_timeout: 1.0`, so at least part of the zero-command
  behavior is due to upstream direct command starvation, not only collision
  monitor slowdown.
- `terrain_pct_planner.py` now queues SLAM point-cloud graph rebuilds in a
  single background `ThreadPoolExecutor` and runs the node with a
  `MultiThreadedExecutor`, so direct control timer callbacks are not blocked by
  synchronous FAST-LIO graph rebuilding.
- `log/cross_level_async_rebuild_20260516_125937/` is the first runtime after
  the background rebuild change. In a 180-second probe it reproduced high path
  generation with `pct_path_max_z: 2.188031`; no high-path sample had command age
  above `1.0s`. `/cmd_vel_nav_age_sec` max was `1.716s` overall, but high-path
  execution stayed below the smoother timeout.
- The same post-fix runtime did not produce physical climb. Gazebo z stayed near
  `-0.005001`, while FAST-LIO aligned z reached about `0.362515`.
- In the post-fix runtime, direct tracking later targeted high `slam_deck`
  waypoints such as `(6.20, 5.97, 1.20)` while the robot's aligned z stayed
  around `0.36` and `gate_z` near `0.03`. This is a new local failure signature:
  command starvation was improved, but high waypoint physical progress gating is
  still insufficient.
- `test_direct_tracking_holds_high_deck_waypoint_until_physical_height_progress`
  now covers this failure signature. `_direct_height_error` uses a stricter
  `0.25m` z tolerance for high `slam_deck` nodes, so a deck waypoint cannot be
  considered reached while physical/gated height remains well below the deck.
- `log/cross_level_deck_gate_20260516_130933/` is the first runtime after the
  stricter high-deck gate. It reproduced high path generation with
  `pct_path_max_z: 2.068655`; all command ages stayed well below `1.0s`
  (`cmd_vel_nav_age_sec` max `0.18s`, smoother/base max about `0.065s`).
- The same runtime still did not climb: Gazebo z stayed near `-0.005001`.
  Direct tracking no longer jumped through the high deck sequence; it stalled
  and replanned around the first high target `(-0.90,1.54,0.72)` and later
  tracked ramp/step approach nodes such as `(6.03,3.20,0.71)` with `gate_z`
  near ground level.
- A yaw/pitch-aware SDF collision check of the latest stall targets showed that
  the requested final goal `(6.0,13.0,2.2)` lies on the `third_floor_deck` /
  `upper_ramp_landing` top surface, but the latest intermediate high targets do
  not describe a valid climb entry. `(-0.90,1.54,0.72)` is inside the
  `second_floor_deck` XY footprint but about `0.22m` below its top surface, and
  `(6.03,3.20,0.71)` is near the `lab_table_10` top/edge rather than a real
  ramp, stair, or deck entry.
- `test_slam_frontier_path_rejects_isolated_obstacle_step_ahead` now covers the
  latter failure signature. `plan_slam_frontier_path` filters obstacle-like low
  `slam_step` frontier endpoints for high-floor goals when other reachable
  candidates exist, so a table-like high surface cannot outrank the continuing
  low-floor goal corridor.
- Static regression after that frontier guard passed:
  `127 passed in 33.55s` for visual config, SLAM graph, terrain planner,
  control command chain, and cross-level evidence probe tests.
- `log/cross_level_frontier_step_guard_20260516_133146/` is the runtime after
  the obstacle-like `slam_step` frontier guard. It still produced a high path:
  `/Laser_map_world` reached `750899` sampled points and `/pct_path` reached
  `pct_path_max_z: 2.187763`.
- In that runtime, command freshness stayed healthy after the high path appeared:
  `cmd_vel_nav_age_sec` max was `0.084s`, `cmd_vel_smoothed_age_sec` max was
  `0.064s`, and base command age max was `0.065s`.
- The same runtime still did not climb physically. The final Gazebo pose was
  `[4.590421, 0.406641, -0.005001]`, and final
  `gazebo_goal_xy_distance` remained `12.672001m`.
- The first high final path in that runtime stalled around
  `index=9/35 target=(0.64,1.52,0.74) ... surface=slam_step`, with
  `gate_z` near `-0.01` and repeated rotation commands. A later replan still
  included `target=(6.06,3.21,0.85) ... surface=slam_step`.
- SDF correlation of the new stall target showed `(0.64,1.52,0.74)` is inside
  the `second_floor_deck` XY footprint but below the deck top, not a validated
  ramp/stair entry. The earlier table-like attraction was reduced, but a
  nonphysical deck-edge/step-entry problem remains in the final high path.
- `test_final_high_path_rejects_deck_edge_step_without_ramp_approach` now covers
  the remaining final-path transition defect. `plan_terrain_path` skips a high
  candidate path if it first reaches a high `slam_step` / `slam_deck` entry
  without prior ramp/stair/step ascent evidence, and can then select another
  reachable high candidate.
- Static regression after that final-path guard passed: focused terrain/SLAM
  graph tests reported `96 passed in 32.90s`; the required visual config, SLAM
  graph, terrain planner, control chain, and cross-level probe gate reported
  `128 passed in 33.65s`; `git diff --check` passed; `colcon build
  --symlink-install` finished 8 packages.
- `log/cross_level_final_transition_guard_20260516_140148/` is the runtime after
  the final-path transition guard. It still produced live map growth and high
  paths: `/Laser_map_world` reached `791963` sampled points and `/pct_path`
  reached `pct_path_max_z: 2.630439`.
- In that runtime, command freshness remained mostly healthy: final
  `cmd_vel_nav_age_sec` was `0.008s`, smoother age `0.0s`, base command age
  `0.059s`. One sample reached `cmd_vel_nav_age_sec: 1.189s`, while downstream
  smoother/base command ages stayed below `0.2s`.
- The same runtime produced partial physical climb but still failed cross-level
  arrival. Gazebo z peaked at `0.154583` around `elapsed_sec: 140.401`, then
  returned to about ground level; the final Gazebo pose was
  `[2.749582, 2.918139, -0.005001]` and final goal XY distance remained
  `10.592881m`.
- The post-guard direct tracking sequence no longer stalled first at the
  `(0.64,1.52,0.74)` deck-edge signature. The first final high path progressed
  along `slam_ramp` targets up to about `target_z=0.19` and `gate_z=0.48`, then
  released a stalled direct path around `goal=(-3.41,-0.66)`. A later final high
  path reached `target=(1.18,2.44,0.77) ... surface=slam_step` with physical
  Gazebo z back near ground.
- A later geometry audit found a physical SDF defect: the original
  `lower_access_ramp` pose did not make the ramp top surface continuous with
  `ramp_lower_landing` and `ramp_upper_landing` / `second_floor_deck`.
  `test_large_multilevel_lower_ramp_physically_connects_landings` now covers
  this connection.
- Both `large_multilevel_complex.sdf` and `large_multilevel_complex_static.sdf`
  now set `lower_access_ramp` to center z `0.440` and pitch `-0.0678`, matching
  the intended lower-to-upper landing height span.
- The same geometry slice exposed a graph constraint gap: different ramp-like
  surfaces could connect without satisfying the ramp-entry corridor. The terrain
  planner now requires `_is_ramp_entry_node` for ramp-to-ramp surface changes.
- Static verification after the SDF geometry and ramp-to-ramp guard passed:
  focused ramp geometry/path tests `3 passed in 2.29s`; visual config, SLAM
  graph, terrain planner, control command chain, and cross-level probe gate
  `130 passed in 33.37s`; `git diff --check` passed; `colcon build
  --symlink-install` finished 8 packages.
- `log/cross_level_ramp_geometry_20260516_155258/` is the runtime after the SDF
  ramp geometry fix. It reproduced high path generation: `/Laser_map_world`
  reached `789725` sampled points and `/pct_path` reached
  `pct_path_max_z: 2.212123`; the first high path sample appeared at
  `elapsed_sec: 40.21`.
- In that runtime, command freshness was healthy:
  `cmd_vel_nav_age_sec` max `0.091s`, `cmd_vel_smoothed_age_sec` max `0.061s`,
  and base command age max `0.075s`.
- The same runtime still did not physically climb. Gazebo z never exceeded
  `-0.005001`; the minimum Gazebo XY distance to the final goal was
  `11.624816m`; the final Gazebo pose was
  `[-6.940314, 3.194232, -0.005001]`.
- The run did move beyond the earlier low-entry stall: the final path progressed
  through `slam_ramp` targets and then into high `slam_deck` / `slam_step`
  targets. This is progress in the execution boundary, not cross-level success.
- The latest direct-tracking logs show the remaining mismatch more sharply:
  surface `gate_z` can rise near high ramp/deck/step nodes while Gazebo physical
  z stays at ground height. High deck/step waypoint advancement must not use
  surface height as proof of physical climb.
- `test_high_surface_gate_does_not_prove_physical_height_progress` now covers
  that latest mismatch. It reproduces a high `slam_deck` / `slam_step` path with
  `surface_gate_z=0.76` and low robot z `0.36`, and requires progress z to stay
  at the robot z.
- `terrain_pct_planner.py` now separates surface gate height from waypoint
  progress height via `direct_tracking_progress_z`. For high `slam_deck` /
  `slam_step` nodes, direct target advancement and lookahead use robot z instead
  of surface `gate_z`; the surface `gate_z` remains available for diagnostics.
- Static verification after this progress-gate slice passed:
  `test_terrain_pointcloud_planner.py` reported `35 passed in 3.73s`; visual
  config, SLAM graph, terrain planner, control command chain, and cross-level
  probe gate reported `131 passed in 39.82s`.

## Inference

- The current blocker is no longer high `/pct_path` generation.
- The command chain is not globally broken, because command flow reaches the base controller in many samples.
- The strongest current hypothesis is a physical execution/height-source mismatch: direct tracking progresses through high-surface waypoints using the aligned SLAM odometry frame, but the Gazebo body does not physically climb.
- The sparse floor-to-deck shortcut was one real graph defect and has been
  regression-covered, but it was not the whole cross-level problem.
- Remaining hypotheses are narrower: ramp-entry geometry/surface labeling may
  still be imperfect, direct tracking may need a physical progress gate around
  ramp nodes, downstream smoothing/collision may intermittently drop base
  commands, and the surrogate model may not sustain ascent.
- The command-drop hypothesis is stronger than before, but not yet proven. The
  existing 5-second probe snapshots can show zero-valued last commands; they
  cannot prove whether those zeros were sustained timeouts, instantaneous stop
  samples, or valid smoother/collision outputs without the new count/age fields.
- The enhanced probe proves one concrete timeout path: direct command publication
  can be delayed long enough for velocity_smoother to time out. The background
  rebuild change should be treated as a targeted fix for command starvation, not
  as proof that physical cross-level ascent is solved.
- The post-fix run shifts the leading blocker toward high-surface waypoint
  progression and physical-height/contact gating. Direct execution can now keep
  commands fresh, but the tracker can still approach high deck waypoints while
  Gazebo pose remains on the ground plane.
- The stricter high-deck z gate should reduce false progression across deck
  waypoints, but it has not yet been runtime-validated. It may expose a new stall
  at the ramp-to-deck transition if the current surrogate cannot physically gain
  height.
- Runtime now supports that inference: the false-progression symptom is reduced,
  but the robot still fails to gain physical height. The active blocker has moved
  to ramp/step entry geometry, contact/friction/vehicle capability, or requiring
  a physically capable profile before accepting deck transition.
- The latest SDF correlation narrows the geometry side of the blocker: one
  post-deck-gate frontier path was attracted to obstacle/table-like `slam_step`
  structure rather than a validated ramp/stair entry. This was a planner
  selection defect, not proof that all remaining physical ascent failures come
  from the surrogate model.
- Runtime after that filter keeps command age low and preserves high path
  generation, so the leading blocker is now more specifically final-path
  transition validity: the path can still enter high `slam_step` / deck-edge
  artifacts without a physically climbable ramp/stair approach.
- The final-path transition guard moves the runtime signature forward: it can
  induce a real, small ramp climb, but the robot does not sustain ascent or
  transition to the next high step/deck segment. The next blocker is therefore
  ramp-ascent persistence and high-step entry execution, not the earlier
  `(0.64,1.52,0.74)` deck-edge shortcut.
- The SDF ramp geometry fix removed one physical-world inconsistency and moved
  the robot farther through the ramp corridor in XY, but it did not produce
  Gazebo z climb. The next static guard now addresses the distinction between
  terrain-surface height used for local tracking and physical-height progress
  required to accept high deck/step waypoint advancement. Runtime still needs to
  prove whether this guard produces a useful hold, a physical climb, or exposes
  the next ramp/contact/model limitation.

## Pending

- Runtime-validate the new physical-height progress gate for high deck/step
  advancement.
- If the robot now holds at high deck/step entry without Gazebo z rise, next
  candidates are stricter ramp-to-deck transition checks, ramp-only approach
  shaping, contact/friction/model capability diagnostics, or using a more
  capable Unitree-style footed profile once the current surrogate evidence is
  exhausted.
- If behavior changes are needed, add a targeted failing test before editing `terrain_pct_planner.py` or `slam_traversability_graph.py`.

## Next Minimal Diagnostic Step

1. Rerun the headless high-goal chain and require all three: `/pct_path` max z
   above `2.0`, command ages below the smoother timeout, and Gazebo pose z
   actually climbing to the high deck target.
2. Inspect whether the new progress gate causes a useful hold at the ramp/deck
   boundary or whether the robot still diverts to a high `slam_step` artifact.
3. Correlate the next `slam_step` target with SDF geometry before changing graph
   logic; do not assume every high `slam_step` is the same defect.
