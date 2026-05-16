# Next Model Cautions

Status: required caution sheet for the next conversation.
Last updated: 2026-05-16.

## Easy Mistakes

- Mistaking SDF-derived tests for the active runtime truth. The active FAST-LIO2 demo plans from `/Laser_map_world`.
- Mistaking a high `/pct_path` for physical high-deck arrival. Path generation is accepted; physical arrival is not.
- Reopening the single-floor Nav2 planner as the first suspect without checking
  the latest evidence. On 2026-05-16, `single_floor_lab` clean batch completed
  all four missions with `success: true`, zero emergency stops, and zero
  collision events. Current evidence does not justify changing the single-floor
  planner before cross-level diagnostics.
- Reading older documents and concluding that FAST-LIO/PCT high path generation is still impossible. That was true earlier, not after the latest sparse bridge and frontier fixes.
- Changing only one speed limit. The actual speed chain spans `terrain_pct_planner`, `velocity_smoother`, `diff_drive_controller`, and Nav2 RPP parameters.
- Letting Nav2 full-stack control the FAST-LIO terrain path. Current FAST-LIO launch defaults to `nav_stack_mode:=safety_only` and `terrain_execution_mode:=direct`.
- Using `gz sim`. This workspace is pinned to Fortress and `ign gazebo`.
- Trusting WSL/Gazebo GUI behavior without cleaning old processes first.
- Deleting or committing root-level PDF/DOCX course materials. They are user materials, not source.

## Modules Requiring Care

- `terrain_pct_planner.py`: large file with planning, frontier, direct-control, and ROS action responsibilities. Keep edits small and test-backed.
- `slam_traversability_graph.py`: graph connectivity fixes can accidentally create wall-crossing paths. Always run sparse bridge and wall-base rejection tests together.
- `visual_fast_lio_navigation.launch.py`: launch defaults encode the current system contract. Updating runtime behavior without updating launch tests will confuse future agents.
- `nav2_params.yaml` and `go2w_controllers.yaml`: speed and acceleration limits must remain consistent.
- `slam_scan_projector.py`: `/slam_scan` is local safety input. Do not convert the accumulated `/Laser_map_world` into StopZone truth.

## Historical Failure Patterns

- Long frontier paths caused the robot to chase stale low-floor goals while the SLAM map kept changing.
- Over-aggressive blocked-node use on final SLAM graph planning prevented high-floor final paths.
- Planned-but-not-reached frontiers tightened the regression gate too early.
- Sparse FAST-LIO ramp/stair samples split high-floor graphs into disconnected islands.
- Direct tracking skipped or dropped cross-level waypoints by XY closeness while the robot was still low.
- Collision monitor or controller launch wiring can silently stop base commands if lifecycle activation is wrong.
- A final-goal path guard can appear to work in pending-goal replanning while
  the initial goal callback still accepts a later low-floor detour. Check both
  code paths before rerunning cross-level smoke.

## 2026-05-16 Single-Floor Gate

- Fact: `bash scripts/cleanup_airos_runtime.sh` returned `[PASS] AIROS runtime processes cleaned.`
- Fact: `run_clean_nav_batch --dry-run` enumerated `lab_start_to_task_a`,
  `lab_start_to_task_b`, `lab_door_passage`, and `lab_return_point`.
- Fact: targeted static gate passed: `117 passed`.
- Fact: `git diff --check` passed.
- Fact: `colcon build --symlink-install` finished 8 packages.
- Fact: runtime baseline
  `log/single_floor_baseline_20260516.jsonl` records all four single-floor
  missions as `success: true`, `execution_mode: navigate_to_pose`, with
  `emergency_stop_count: 0` and `collision_count: 0`.
- Inference: current `SmacPlannerHybrid` + `REEDS_SHEPP` and RotationShim/RPP
  with `allow_reversing: false` is executable for the current single-floor
  mission set, despite being a possible mismatch candidate for future tighter
  missions.
- Pending: route-waypoint mode and dynamic-obstacle single-floor variants were
  not rerun in this gate.

## 2026-05-16 Fast Display Priority

- Fact: the user's current priority is fast demonstrable results with good
  effect. The minimum target is complex single-floor navigation with SLAM,
  planning, and execution evidence. Algorithm upgrades and physical cross-floor
  climbing are secondary until the display baseline is stable.
- Fact: `log/fast_show_single_floor_clean_batch_20260516.jsonl` is the latest
  fast display baseline. `lab_door_passage` finished successfully in `15.303s`,
  with path length `2.664m`, `emergency_stop_count: 0`, `collision_count: 0`,
  and minimum obstacle distance `1.078m`.
- Fact: `visual_fast_lio_navigation.launch.py` now exposes
  `terrain_goal_z_policy`. The default remains `highest` for cross-level goals.
  Single-floor FAST-LIO/PCT display runs should explicitly pass
  `terrain_goal_z_policy:=nearest_z terrain_goal_min_z:=-1.0`.
- Fact: target `(8.0,-6.8,0.0)` is not reliable for single-floor FAST-LIO/PCT
  display in `large_multilevel_complex`; the SLAM graph selected high-structure
  candidates around `target_z≈1.96` and later `target_z≈1.24`.
- Fact: target `(8.0,-9.0,0.0)` produced a low-level FAST-LIO/PCT path with
  `/pct_path_max_z≈0.226738`, `/Laser_map_world` growth from `449105` to
  `599135` sampled points, and fresh `/cmd_vel_nav`, `/cmd_vel_smoothed`, and
  base commands. It did not prove physical arrival; Gazebo goal distance only
  improved from `3.984403m` to `3.937942m`.
- Fact: target `(1.9,-9.2,0.0)` is the current best FAST-LIO/PCT single-floor
  display smoke. In `log/single_floor_fast_lio_demo_20260516_1652/probe_goal_1p9_-9p2.jsonl`,
  `/Laser_map_world` grew from `267848` to `401742` sampled points,
  `/pct_path_max_z` stayed low at about `-0.073576`, command counts reached
  `/cmd_vel_nav=228`, `/cmd_vel_smoothed=263`, and base command `263`, and
  Gazebo goal distance dropped from `2.061553m` to `0.603608m`.
- Fact: the same launch log recorded `terrain direct tracking goal reached` for
  `(1.9,-9.2)`. This is a usable FAST-LIO/PCT single-floor smoke, but not
  precise physical-goal acceptance because final Gazebo distance stayed above
  the configured direct goal tolerance `0.30m`.
- Fact: `scripts/cleanup_airos_runtime.sh` now terminates
  `nav2_map_server/map_saver_server`; `test_cleanup_script_stops_nav2_map_saver_leftovers`
  covers this residual-process risk.
- Inference: for a near-term demo, use stable Nav2 clean batch as the user-facing
  "works now" result, and use FAST-LIO/PCT as the advanced SLAM-planning layer
  with its direct-execution limitation stated plainly.
- Pending: if the next step is improving display quality, tighten FAST-LIO/PCT
  direct final-goal convergence against Gazebo ground truth for the
  `(1.9,-9.2)` class of low-level targets before returning to cross-level
  physical ascent.

## 2026-05-16 Cross-Level Diagnostics

- Fact: `cross_level_evidence_probe` was added as a diagnostic-only sampler for
  `/Laser_map_world`, `/pct_path`, `/cmd_vel_nav`, `/cmd_vel_smoothed`, base
  command, `/fast_lio_odom_world`, and Gazebo pose.
- Fact: `terrain_pct_planner.py` now logs direct tracking diagnostics containing
  target index, target xyz, robot xyz, surface label, heading error, speed
  limit, and command.
- Fact: a fresh run reproduced live map growth and high path generation:
  `log/cross_level_evidence_probe_final_20260516.jsonl` reached
  `laser_map_points: 710863` and `pct_path_max_z: 2.339499`.
- Fact: the same run did not prove physical climb. Gazebo pose z stayed near
  `-0.005`; `gazebo_goal_xy_distance` remained about `11.93m` in the final
  probe sample.
- Fact: a longer 360-second diagnostic window in
  `log/cross_level_full_20260516_113311/` again reproduced the gap:
  `probe.jsonl` has 72 samples, `laser_map_points` up to `791828`,
  `pct_path_max_z: 2.138301`, and nonzero base-controller commands in 37 of 72
  samples. The final sample still had Gazebo z at `-0.005001` and
  `gazebo_goal_xy_distance: 11.887684`.
- Fact: first frontier direct-tracking diagnostics showed low-floor exploration
  toward `slam_floor` waypoints, with robot z around `0.26` in aligned odom and
  commands transitioning from rotation-only to forward motion.
- Fact: during the final high path, direct tracking advanced to high-surface
  waypoints while Gazebo physical height did not rise. Representative log lines
  include `index=10/35 target=(0.58,0.64,0.76) ... surface=slam_deck` with
  robot aligned-odom z around `0.32-0.34`, and later
  `index=13/35 target=(4.14,2.29,0.50) ... surface=slam_ramp` while Gazebo pose
  remained near ground level.
- Fact: `log/cross_level_gate_full_20260516_120724/` showed that a local
  surface-height gate alone did not solve physical ascent. It prevented some
  aligned-odom height overclaiming, but could still be attracted to nearby high
  surfaces.
- Fact: `log/cross_level_bridge_guard_20260516_122021/` is the newest runtime
  evidence after rejecting sparse floor-to-deck edge jumps. It still generated a
  high `/pct_path` with `pct_path_max_z: 2.065411`, changed the first high target
  to `slam_ramp` at `(-2.39,0.83,0.73)`, and produced partial Gazebo climb
  (`max_gazebo_z: 0.217646`) without reaching the high goal.
- Fact: post-run correlation of that log showed slope tracking at
  `speed_limit=0.16`, repeated `collision_monitor` SlowZone events near the ramp,
  and many 5-second probe samples where `/cmd_vel_nav` was nonzero while
  `/cmd_vel_smoothed` and base command were zero.
- Fact: `cross_level_evidence_probe` now records command message counts and age
  fields for `/cmd_vel_nav`, `/cmd_vel_smoothed`, and base command. Use these
  fields in the next runtime run before changing safety or speed parameters.
- Fact: `log/cross_level_cmd_age_20260516_124501/` used those fields and showed
  `/cmd_vel_nav_age_sec` up to `3.91s`, downstream command age up to `2.257s`,
  and zero downstream commands in 38 of 71 samples while high `/pct_path` existed.
- Fact: `terrain_pct_planner.py` now offloads SLAM graph rebuilds to a single
  background `ThreadPoolExecutor` and uses `MultiThreadedExecutor` in `main()`
  so direct command publication is not blocked by synchronous point-cloud graph
  construction.
- Fact: `log/cross_level_async_rebuild_20260516_125937/` verified that command
  age improved after the executor change. It produced high `/pct_path`
  (`pct_path_max_z: 2.188031`) and no high-path sample had command age above
  `1.0s`.
- Fact: the same run still did not physically climb. Gazebo z stayed near
  `-0.005001`, while direct tracking reached high `slam_deck` targets around
  `(6.20,5.97,1.20)` with aligned odom z around `0.36` and `gate_z` near `0.03`.
- Fact: `test_direct_tracking_holds_high_deck_waypoint_until_physical_height_progress`
  now covers high-deck false progression. High `slam_deck` targets use a
  stricter z reach tolerance, so low physical/gated height cannot advance through
  deck waypoints.
- Fact: `log/cross_level_deck_gate_20260516_130933/` runtime-validated the next
  boundary: high path still appeared (`pct_path_max_z: 2.068655`) and command
  ages stayed below `1.0s`, but Gazebo z stayed near ground. The run stalled and
  replanned around high ramp/step targets instead of falsely advancing through
  deck nodes.
- Fact: SDF collision correlation after that run showed that the requested final
  goal `(6.0,13.0,2.2)` is on the real `third_floor_deck` /
  `upper_ramp_landing`, but one later intermediate target `(6.03,3.20,0.71)`
  is near the `lab_table_10` top/edge, not a real ramp/stair/deck entry.
- Fact: `test_slam_frontier_path_rejects_isolated_obstacle_step_ahead` now
  covers this obstacle-like `slam_step` frontier failure. The frontier planner
  filters low `slam_step` endpoints for high-floor goals when other reachable
  candidates exist.
- Fact: the targeted static gate including visual config, SLAM graph, terrain
  planner, control chain, and cross-level probe tests passed with
  `127 passed in 33.55s`.
- Fact: `log/cross_level_frontier_step_guard_20260516_133146/` runtime-validated
  the obstacle-like `slam_step` frontier guard only partially. The run still
  generated a high path (`pct_path_max_z: 2.187763`) and kept command ages low
  (`cmd_vel_nav_age_sec` max `0.084s`, smoother/base command age max about
  `0.065s`), but Gazebo z stayed near `-0.005001`.
- Fact: the first high final path in that run stalled at
  `target=(0.64,1.52,0.74) ... surface=slam_step` with `gate_z` near ground;
  SDF correlation places that target inside the `second_floor_deck` XY footprint
  but below the deck top, not on a validated ramp/stair entry.
- Fact: `test_final_high_path_rejects_deck_edge_step_without_ramp_approach` now
  covers that final-path transition defect. `plan_terrain_path` can skip a high
  candidate path that first reaches high `slam_step` / `slam_deck` without prior
  ramp/stair/step ascent evidence and continue to another high candidate.
- Fact: static regression after that guard passed: focused terrain/SLAM graph
  tests `96 passed in 32.90s`; required static gate `128 passed in 33.65s`;
  `git diff --check` passed; `colcon build --symlink-install` finished 8
  packages.
- Fact: `log/cross_level_final_transition_guard_20260516_140148/` preserved high
  path generation (`pct_path_max_z: 2.630439`) and command freshness, and
  produced partial physical climb (`gazebo_z_max: 0.154583`). It still did not
  reach the high deck; final Gazebo z returned to about `-0.005001` and final
  goal XY distance was `10.592881m`.
- Fact: the new runtime no longer first stalls at the earlier
  `(0.64,1.52,0.74)` deck-edge target. It progressed along `slam_ramp` targets
  up to about `target_z=0.19` / `gate_z=0.48`, released a stalled direct path
  around `goal=(-3.41,-0.66)`, and later encountered a high `slam_step` target
  near `(1.18,2.44,0.77)`.
- Fact: a later SDF audit found that the original `lower_access_ramp` pose did
  not physically connect `ramp_lower_landing` to `ramp_upper_landing` /
  `second_floor_deck`. `test_large_multilevel_lower_ramp_physically_connects_landings`
  now covers this, and both large multilevel worlds set the ramp center z to
  `0.440` with pitch `-0.0678`.
- Fact: the same slice fixed a ramp-to-ramp transition gap. Different ramp-like
  surfaces now require both endpoints to satisfy the ramp-entry corridor before
  an edge is accepted.
- Fact: static verification after the SDF/ramp-to-ramp slice passed: focused
  tests `3 passed in 2.29s`; visual config, SLAM graph, terrain planner, control
  chain, and cross-level probe gate `130 passed in 33.37s`; `git diff --check`
  passed; `colcon build --symlink-install` finished 8 packages.
- Fact: `log/cross_level_ramp_geometry_20260516_155258/` is the latest runtime
  after that geometry fix. It preserved high path generation
  (`pct_path_max_z: 2.212123`) and command freshness (`cmd_vel_nav_age_sec` max
  `0.091s`, smoother max `0.061s`, base max `0.075s`).
- Fact: the same runtime still failed physical climb. Gazebo z stayed at
  `-0.005001`, minimum final-goal XY distance was `11.624816m`, and final Gazebo
  pose was `[-6.940314, 3.194232, -0.005001]`.
- Fact: direct tracking then progressed from `slam_ramp` into high `slam_deck`
  / `slam_step` targets while Gazebo z remained at ground height. Do not treat
  terrain surface `gate_z` as proof of physical high-deck arrival.
- Fact: `test_high_surface_gate_does_not_prove_physical_height_progress` now
  covers that mismatch. `direct_tracking_progress_z` keeps high `slam_deck` /
  `slam_step` waypoint progress tied to robot z when surface `gate_z` is high
  but robot z remains low.
- Fact: after this progress-gate slice, focused terrain planner tests reported
  `35 passed in 3.73s`; visual config, SLAM graph, terrain planner, control
  chain, and cross-level probe gate reported `131 passed in 39.82s`.
- Fact: after direct ramp physics succeeded but main-chain cross-level still
  stayed low, a new static guard was added:
  `test_slam_frontier_path_prefers_ramp_entry_over_isolated_step_pair`.
  It captures the case where a high-floor frontier was pulled toward a low
  `slam_step` pseudo-entry around `(4.9,1.4,0.14)` instead of the real lower
  ramp corridor. `terrain_pct_planner` now prioritizes continuous ramp/stair
  vertical progress over isolated step-pair attractors during high-floor
  frontier entry scoring.
- Fact: the guard passed under the required affected gate:
  `148 passed in 27.55s`; `git diff --check` passed; `colcon build --symlink-install`
  finished 8 packages.
- Pending: this has not yet been runtime-proven. The next bounded cross-level
  run must check whether the first frontier/direct target actually shifts
  toward the lower ramp corridor and whether Gazebo pose z starts sustained
  climb.
- Fact: the first bounded runtime after that change was
  `log/cross_level_after_frontier_entry_fix_20260516_203725/`. It preserved
  high path generation (`pct_path_max_z_max=2.064855`) and fresh command chain
  (`cmd_vel_nav_age_sec_max=0.077s`, smoother/base age about `0.06s`) but still
  did not climb (`gazebo_z_max=-0.005001`).
- Fact: that runtime showed a new path-semantics issue: after an initially high
  path, direct execution spent a long time at low-height `slam_ramp` targets
  around `(6.02,-11.40,0.36)`, which are far from the true high-goal approach.
  Later frontier eventually shifted to `(-5.93,0.55)`, closer to the true lower
  ramp side, but too late to pass the run.
- Fact: `test_direct_tracking_drops_regressive_low_ramp_prefix_before_high_entry`
  now covers low-height ramp/slope prefixes that make too little progress toward
  the high final goal. The latest affected static gate is `149 passed in
  27.56s`, `git diff --check` passed, and `colcon build --symlink-install`
  finished 8 packages.
- Fact: `test_pending_final_goal_waits_for_active_frontier_endpoint` and
  `test_high_final_path_rejects_large_initial_goal_regression` now cover the
  next final-path failure: while an active frontier is still being executed, do
  not switch to a high final path whose early low-floor segment moves farther
  away from the final goal.
- Fact: the latest affected static gate is now `151 passed in 27.47s`;
  `git diff --check` passed; `colcon build --symlink-install` finished 8
  packages.
- Fact: `log/cross_level_after_final_regression_guard_20260516_210455/`
  runtime-proved that the old early switch to the far final target did not
  recur. The launch log shows a frontier path toward `frontier=(-3.56,1.20)`
  and direct tracking advanced to `target=(-3.56,1.20,0.46) surface=slam_ramp`.
- Fact: the same runtime did not physically climb. `gazebo_z_max` stayed about
  `-0.005`, and collision monitor reported `Robot to stop due to StopZone
  polygon` near the lower-ramp edge before the stalled frontier was released.
- Fact: that run's probe summary can show `pct_path_poses_max=0` because the
  probe started after goal publication and `/pct_path` is not latched. Use
  `launch.log` as the path-publication fact source for that run.
- Inference: the next blocker is ramp-edge / support-margin / local
  `/slam_scan` StopZone interaction, not high `/pct_path` generation, command
  freshness, or the 3D goal tool.
- Pending: do not relax or disable collision monitor as the first move. First
  add ramp-center/support-margin scoring or `/slam_scan` StopZone diagnostics
  so the path does not hug ramp edges or cross unsupported stair/platform gaps.
- Fact: user priority changed on 2026-05-16. The immediate product goal is fast
  demonstrable complex single-floor SLAM + planning + execution quality. Cross
  floor physical ascent remains important but is no longer the only active
  thread.
- Fact: the first FAST-LIO/PCT single-floor smoke at
  `log/single_floor_fast_lio_demo_20260516_1652/` moved Gazebo from
  `[0.0,-10.0,-0.005001]` to `[1.719182,-9.775888,-0.005001]` toward
  `(1.9,-9.2)`, but final Gazebo distance was still `0.603608m`.
- Fact: the enhanced probe run
  `log/single_floor_fast_lio_demo_20260516_1703/probe_goal_1p9_-9p2.jsonl`
  showed why: `/fast_lio_odom_world` reached about `0.125m` from the goal while
  wheel odom / Gazebo stayed about `0.569m` away. Do not treat FAST-LIO aligned
  odom alone as physical single-floor goal acceptance.
- Fact: `visual_fast_lio_navigation.launch.py` now has a `terrain_odom_topic`
  launch argument. The default remains `/fast_lio_odom_world`; for quick
  physical single-floor demonstration, run with `terrain_odom_topic:=/odom`
  plus `terrain_goal_z_policy:=nearest_z terrain_goal_min_z:=-1.0`.
- Fact: the current best FAST-LIO/PCT single-floor demo evidence is
  `log/single_floor_fast_lio_demo_20260516_odom/`. After repeating the goal with
  `ros2 topic pub --times 5 --rate 1`, terrain planner logged
  `received terrain goal`, `started terrain-guided direct tracking`,
  `poses=6 path_nodes=7`, and `terrain direct tracking goal reached`.
- Fact: in
  `log/single_floor_fast_lio_demo_20260516_odom/probe_goal_1p9_-9p2_after_pub.jsonl`,
  `/Laser_map_world` reached `631671` points and final wheel/Gazebo goal
  distance to `(1.9,-9.2)` was `0.214862m`, below the current direct goal
  tolerance `0.30m`.
- Fact: a one-shot `ros2 topic pub --once` can be lost or arrive before the
  planner is ready. The failure mode is a growing `/Laser_map_world` with
  `/pct_path` still empty and command counts at 0. Use repeated publishing or a
  scripted readiness gate for demos.
- Fact: `scripts/run_fast_lio_single_floor_demo.sh` is now the preferred
  one-command FAST-LIO/PCT single-floor demo harness. It runs headless with
  `terrain_goal_z_policy:=nearest_z`, `terrain_goal_min_z:=-1.0`, and
  `terrain_odom_topic:=/odom`, publishes the goal repeatedly, runs
  `cross_level_evidence_probe`, and summarizes acceptance from launch-log
  planner evidence plus Gazebo/wheel goal distance.
- Fact: the script smoke
  `log/fast_lio_single_floor_demo/smoke_20260516_172800/` returned
  `accepted: true` with `laser_map_points_max: 198858`,
  `cmd_vel_nav_count_max: 202`, and wheel/Gazebo final goal distance
  `0.20666m`.
- Fact: the probe in that script smoke did not sample a non-empty `/pct_path`
  because direct tracking had already completed and cleared the path. Use the
  launch log evidence (`received terrain goal`, `started terrain-guided direct
  tracking`, `terrain direct tracking goal reached`) to avoid a false negative.
- Fact: the demo harness now supports `DEMO_TARGET=near_goal|long_corridor|custom`.
  `near_goal` is the default accepted target. `long_corridor` is intentionally
  exploratory and must not be treated as accepted yet.
- Fact: `DEMO_TARGET=long_corridor` run
  `log/fast_lio_single_floor_demo/long_corridor_20260516_173459/` produced a
  low `/pct_path` (`pct_path_poses_max=11`, `pct_path_max_z=-0.027427`) and a
  healthy command chain (`cmd_vel_nav_count_max=464`), but final Gazebo distance
  was `0.313639m`, above the `0.30m` acceptance threshold.
- Fact: the same long-corridor launch log initially reported `target_z≈1.237`
  before later low-floor tracking, so `nearest_z` alone does not fully prevent
  high-structure attraction for longer single-floor targets.
- Inference: the next root-cause work should focus on the transition from
  exploration/frontier execution to final high `/pct_path` physical execution,
  especially the mismatch between SLAM/aligned odom height, physical Gazebo body
  height, and high-surface waypoint advancement. Downstream smoothing/collision
  monitor behavior still matters, but it is not the only observed gap.
- Inference: one blocker, sparse floor-to-deck shortcutting, is now covered by
  `test_slam_graph_rejects_sparse_floor_to_deck_edge_jump`. The remaining
  blocker is likely around sustaining ramp contact/ascent and command delivery
  during the ramp segment, not high path generation.
- Inference: command delivery through smoother/collision is now a high-priority
  suspect, but the old 5-second snapshots alone cannot prove sustained timeout or
  dropout. Treat this as a hypothesis until count/age evidence confirms it.
- Inference: command starvation from synchronous SLAM graph rebuild was one real
  blocker. The next run must verify whether the executor fix keeps command age
  below the smoother timeout; do not assume it solves ramp climbing until Gazebo
  pose confirms ascent.
- Inference: after command starvation is reduced, the leading blocker is
  high-surface waypoint progression without physical height/contact confirmation.
  Do not spend the next turn retuning velocity timeout unless new evidence shows
  command age regressed.
- Inference: the stricter high-deck gate is a necessary guard, not final proof.
  The next runtime may show a useful stall at the ramp/deck boundary; treat that
  as evidence for ramp-entry or surrogate-physics work, not as a regression in
  high path generation.
- Inference: that useful stall has now appeared. The next root-cause thread
  should focus on ramp/step physical entry and model capability, not on
  `/cmd_vel_nav` freshness unless new evidence regresses.
- Inference: one post-deck-gate stall was partly caused by frontier attraction
  to obstacle/table-like high structure. Treat the new filter as a targeted
  geometry-selection guard that still needs runtime validation; do not claim it
  proves physical ascent.
- Inference: after that runtime validation, command freshness and high path
  generation are not the leading suspects. The next defect is narrower: final
  high paths can still enter high `slam_step` / deck-edge artifacts without a
  physically climbable ramp/stair-continuous approach.
- Inference: after the final-path guard, the leading defect moved again. The
  current evidence supports ramp-ascent persistence and high-step entry
  execution as the next thread, not reverting the final-path guard or retuning
  velocity timeout first.
- Inference: after the SDF ramp geometry fix, the leading defect narrowed to
  surface-height gating versus physical-height progress. A static guard now
  covers high deck/step false progression; runtime still must show whether this
  produces climb, useful holding behavior, or exposes a model/contact limitation.
- Inference: for fast demos, `/odom`-based terrain execution is a practical
  acceptance harness for physical single-floor convergence, while the default
  FAST-LIO odom path should remain the cross-level/SLAM mainline until a cleaner
  estimator/ground-truth split is implemented.
- Inference: before pursuing longer single-floor demos, add a strict low-floor
  z-window or goal-candidate constraint for demo mode. Do not merely increase
  the `0.30m` acceptance threshold to make `long_corridor` pass.
- Fact: the strict single-floor z-window is now implemented. `plan_terrain_path`
  accepts `goal_max_z`, `visual_fast_lio_navigation.launch.py` exposes
  `terrain_goal_max_z`, and `scripts/run_fast_lio_single_floor_demo.sh` defaults
  to `TERRAIN_GOAL_MAX_Z=0.45`.
- Fact: `log/fast_lio_single_floor_demo/long_corridor_zwindow_20260516_174832/`
  proves the z-window removed the earlier high-structure target snap:
  `target_z=0.45`, `pct_path_max_z=-0.030729`, and command counts were healthy.
  It still did not pass strict physical acceptance: wheel/Gazebo final distance
  was `0.302173m`, just above `0.30m`.
- Fact: `append_direct_final_goal` now adds a final direct-tracking waypoint at
  the user goal only when the final graph node is within direct goal tolerance.
  This is a safety guard: do not expand it into arbitrary off-graph chasing.
- Fact: `log/fast_lio_single_floor_demo/long_corridor_finalsnap_20260516_175457/`
  still failed (`accepted: false`, final wheel/Gazebo distance `0.71327m`).
  The final reachable graph node was around `(7.81,-9.55,0.00)`, too far from
  `(8.0,-9.0)` for the snap guard, so the tracker correctly did not extrapolate.
- Fact: the current best one-command FAST-LIO/PCT demo remains the default
  `near_goal`. `log/fast_lio_single_floor_demo/near_goal_after_zwindow_snap_20260516_175830/`
  passed with `accepted: true`, `laser_map_points_max=220390`,
  `cmd_vel_nav_count_max=173`, and wheel/Gazebo distance `0.223881m`.
- Fact: direct tracking completion is now final-goal-aware. It can only report
  final success when both the tracked graph endpoint and the original user goal
  are within direct goal tolerance; this prevents off-graph endpoints from being
  misreported as final arrival.
- Fact: after that guard, the long single-floor target passed:
  `log/fast_lio_single_floor_demo/long_corridor_goal_guard_20260516_184353/`
  had `accepted: true`, `laser_map_points_max=263068`,
  `pct_path_max_z=-0.032202`, `cmd_vel_nav_count_max=487`, and wheel/Gazebo
  distance `0.266004m` to `(8.0,-9.0)`.
- Inference: the current fast display baseline can use `long_corridor` as the
  stronger single-floor FAST-LIO/PCT example and keep `near_goal` as a quick
  smoke test.
- Pending: before resuming cross-level physical navigation, update the goal
  tool/goal semantics. A 2D RViz-style goal is not enough to unambiguously mark
  target floor or height; use a 3D/floor-aware goal path and log selected z/layer.
- Fact: the 3D goal publication slice is now implemented. Use
  `ros2 run airos_experiments publish_terrain_goal --x 6.0 --y 13.0 --z 2.2 --publish-count 5 --rate-hz 1`
  for cross-level goals, not a one-shot 2D/volatile goal. The planner treats a
  high PoseStamped z as an effective goal layer constraint, and the evidence
  probe records `goal_xyz`.
- Pending: cross-level acceptance must reject paths that cut across stairs,
  platform edges, or map holes. Check route surface labels, ramp/stair
  continuity, slope/step sequence, support footprint margin, `/pct_path` z, and
  Gazebo/odom physical height together.
- Pending: monitor SLAM mapping and relocalization explicitly in cross-level
  runs. Compare `/Laser_map_world`, `/fast_lio_odom_world`, `/odom`, and Gazebo
  pose; do not accept FAST-LIO aligned odom alone as proof of physical arrival.
- Fact: `log/cross_level_current_reprobe_20260516_181159/` revalidated the
  active cross-level gap after the single-floor work. `/Laser_map_world` reached
  `655392` sampled points, `/pct_path_max_z` reached `2.070835`, and command
  counts reached `/cmd_vel_nav=2530`, smoother/base `2829`. Gazebo z stayed near
  `-0.005001` and wheel `/odom` z stayed `0.0`, so physical climb is still not
  accepted.
- Fact: that run exposed a path-safety defect: final high paths could enter a
  high `slam_step` waypoint and later continue through low/negative-z
  `slam_step` nodes. The robot then rotated near the high step and replanned
  without physical height gain.
- Fact: `test_final_high_path_rejects_step_drop_after_high_entry` and
  `_invalid_final_high_drop_node` now cover this. The fix rejects high-goal paths
  that drop sharply after entering the high corridor. It has static coverage
  and runtime coverage, but no physical ascent acceptance.
- Fact: `log/cross_level_after_high_drop_guard_20260516_182151/` preserved live
  SLAM growth and high path generation after that guard:
  `laser_map_points_max=635571`, `pct_path_max_z=2.206236`, and command counts
  reached `/cmd_vel_nav=2491`, smoother `2759`, base `2658`.
- Fact: the same run still did not physically climb. FAST-LIO aligned z reached
  about `0.343199`, but wheel `/odom` stayed at `z=0.0`, Gazebo z stayed near
  `-0.005`, and Gazebo final-goal distance remained about `11.67m`.
- Fact: the next observed stall is around the ramp-to-deck transition: direct
  tracking reaches a high `slam_deck` target near `(6.59,4.84,1.07)` while the
  robot remains near `(6.3,1.85,0.34)` in aligned odom and at ground-level
  Gazebo height, then releases the stalled direct path.
- Fact: after the 3D goal publisher was added, two headless cross-level runs
  reproduced the same remaining gap with explicit target floor evidence:
  `log/cross_level_3d_goal_20260516_190046/` reached
  `goal_xyz_last=[6.0,13.0,2.2]`, `pct_path_max_z=2.275243`, and
  `laser_map_points_max=674569`; `log/cross_level_3d_goal_long_20260516_191019/`
  reached `pct_path_max_z=2.344505`, `laser_map_points_max=785990`, and fresh
  command ages below about `0.14s`. Both runs still had wheel `/odom` z at `0.0`
  and Gazebo z near `-0.005`.
- Inference: repeating longer cross-level runs without changing diagnostics is
  low value. First verify whether the current `go2w_nav_eq` diff-drive surrogate
  can physically climb a simple ramp in Gazebo; if that fails, the next work is a
  ramp-capable display mode or model/control replacement branch, not more
  planner tuning.
- Fact: the simple-ramp concern has now been narrowed. Direct Gazebo physics
  smoke on the repaired lower ramp succeeded:
  `log/lower_ramp_physics_after_landing_fix_20260516_195621/` reached
  `gazebo_z_max=0.934999` and `gazebo_y_max=12.240317`.
- Fact: wheel `/odom` z staying `0.0` is not by itself a climb-failure proof in
  this repo, because the current Ignition odometry publisher is 2D. Use Gazebo
  pose z as the primary physical-height acceptance signal until odometry is
  changed.
- Fact: after the ramp/landing SDF and landing-classification fixes, the main
  FAST-LIO/PCT/direct chain still produced high paths but did not physically
  climb:
  `cross_level_after_landing_fix_goal_ok_20260516_200724` reached
  `pct_path_max_z=2.341184`, and
  `cross_level_after_regressive_prefix_fix_20260516_201433` reached
  `pct_path_max_z=2.124622`, while Gazebo z remained about `-0.005001`.
- Inference: the next blocker is more likely SLAM graph/frontier entry
  selection around low `slam_step` pseudo-entries near `(4.9,1.4,0.14)` and true
  ramp-entry approach, not a blanket surrogate inability to climb ramps.
- Pending: prove whether the immediate blocker is ramp-to-deck target selection,
  continuous ramp/stair support, physical-height/ramp-contact gating,
  safety-layer limiting, or surrogate physical limits.
- Pending: package the `/odom` single-floor FAST-LIO/PCT run into a reusable
  one-command demo and extend it from the short `(1.9,-9.2)` smoke to a longer
  complex single-floor route without reintroducing high-structure snapping.
- Fact: latest direct execution TDD changed only command lookahead, not waypoint
  acceptance. When the current `slam_step` waypoint has physical-height debt and
  XY distance is within waypoint tolerance, `_direct_lookahead_target` may select
  a later waypoint on the same surface label so the base keeps moving. It must
  not cross a surface-label change in this mode.
- Fact: evidence probe now treats an `ign topic /pose/info` timeout as
  `gazebo_xyz=None` for that sample instead of aborting the whole run. Do not
  use an empty `probe.jsonl` as navigation evidence.
- Fact: `log/cross_level_after_height_debt_lookahead_20260516_215319/` reached
  `/pct_path_max_z=2.178018`, `/Laser_map_world` sampled max `792874`, and fresh
  command counts `/cmd_vel_nav=3101`, smoother `3502`, base `3496`. Gazebo z
  still stayed at `-0.005001`, so physical cross-level acceptance remains false.
- Fact: this run progressed farther than the previous StopZone/near-target
  stalls. Direct diagnostics show low `slam_ramp` execution, a reached frontier,
  and later `slam_step` targets around `(-4.29,3.35,0.72)` and
  `(-3.97,4.46,0.74)`.
- Inference: the active execution bottleneck has shifted to unstable step/ramp
  lookahead heading. The target alternates between adjacent high-debt
  `slam_step` candidates, heading error becomes large, and `_direct_linear_speed`
  returns zero even while commands remain fresh.
- Pending: next work should stabilize direct step/ramp target selection using a
  path-tangent/forward-progress heading rule or curvature gate. Keep the
  physical-height waypoint gate; do not bypass it to claim arrival.
- Fact: a path-tangent/forward-progress refinement was added after that
  diagnosis, but the immediate follow-up run
  `log/cross_level_after_tangent_lookahead_20260516_220735/` is not valid
  cross-level motion evidence because `collision_monitor` did not become active
  and `base_cmd_count` stayed at `0`.
- Fact: the lifecycle activator now retries lifecycle service availability.
  `log/control_chain_after_lifecycle_retry_20260516_221346/` restored command
  propagation with `/cmd_vel_nav=915`, smoother `1016`, and base command `1015`;
  Gazebo goal distance improved from `24.517079m` to `18.280812m`.
- Fact: the same short run did not yet generate a high path sample
  (`/pct_path_max_z=0.475012`) and did not climb (`Gazebo z=-0.005001`).
  Treat it as control-chain recovery evidence only.
- Pending: collision monitor repeatedly warned that `/slam_scan` source
  timestamps differed from node time by about one second and ignored the source.
  Diagnose scan freshness/projector load/source timeout before running another
  long cross-level attempt; do not simply turn off collision monitoring.
- Fact: `/slam_scan` freshness has a first fix. The projector's supported-ramp
  filter now builds a local spatial support index and avoids the previous
  per-point full-cloud scan. `log/scan_freshness_after_support_index_20260516_222625/`
  had `slam_scan_stale_warn_count=0` with base command count `1033`.
- Fact: that run is not cross-level acceptance evidence. It sampled
  `/pct_path_max_z=0.470178` and Gazebo z stayed `-0.005001`.
- Pending: next runtime should be a longer cross-level run now that
  lifecycle activation and `/slam_scan` freshness are healthy. Acceptance still
  requires high `/pct_path max z > 2.0` plus Gazebo pose z rising to the high
  deck, not just command motion.
- Fact: single-floor demo readiness has been refreshed after the latest fixes.
  `near_goal_after_scan_index_20260516_223232` accepted with final Gazebo
  distance `0.222402m`, and
  `long_corridor_after_scan_index_20260516_223440` accepted with final Gazebo
  distance `0.294605m`, `/Laser_map_world` max `351188`, `/pct_path` max z
  `-0.044546`, and no `/slam_scan` stale warnings.
- Inference: if a quick public/demo result is needed, use the single-floor
  `scripts/run_fast_lio_single_floor_demo.sh` chain first. It is currently
  better verified than cross-level physical ascent.
- Pending: do not spend another long cycle doubting the single-floor baseline
  unless these scripts regress. The next hard problem is still physical
  cross-level ascent and high-floor arrival.
- Fact: two post-single-floor cross-level runs still failed physical ascent:
  `cross_level_after_single_floor_refresh_20260516_223943` had
  `/pct_path_max_z=2.138636`, base command `3337`, stale warnings `0`, but
  Gazebo z `-0.005001`; `cross_level_after_zigzag_lookahead_fix_20260516_225624`
  had `/pct_path_max_z=2.249888`, base command `2686`, stale warnings `0`, and
  Gazebo z `-0.005001`.
- Inference: stop running repeated long cross-level trials on the same wheel
  surrogate without changing model/execution/map. The fast path is to ship the
  single-floor demo now, then make cross-level a separate, smaller branch:
  either a ramp-capable surrogate/footed model or a simpler validated multilevel
  map with explicit physical-z acceptance.

## What To Check First

1. `git status --short`
2. `docs/handoff/pre_migration_handoff_report.md`
3. `src/airos_experiments/launch/visual_fast_lio_navigation.launch.py`
4. `src/airos_experiments/airos_experiments/terrain_pct_planner.py`
5. `src/airos_experiments/airos_experiments/slam_traversability_graph.py`
6. Targeted tests for visual config, SLAM graph, terrain planner, and control command chain.

## Evidence Discipline

Use these labels in future reports:

- Fact: directly observed in code, command output, tests, or runtime logs.
- Inference: derived from facts but not directly observed.
- Pending: plausible but not yet verified.

Never write "done" for a cross-level navigation feature unless the evidence
includes both high `/pct_path` and physical pose/odom climbing to the high deck.

## Documentation Discipline

- Add new status snapshots under `docs/handoff/` when the project crosses a real milestone.
- Keep `README.md` concise and point detailed state to handoff docs.
- Treat older docs as background unless they are updated with the current date and evidence.
- Record failed probes, not only successful runs. This project repeatedly improved by preserving negative evidence.
