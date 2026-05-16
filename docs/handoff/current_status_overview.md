# AIROS Current Status Overview

Status: current factual snapshot for handoff.
Last updated: 2026-05-16.

## Project Goal

AIROS is a ROS 2 Humble + Ignition Gazebo Fortress autonomous navigation
prototype for a Go2W-style wheel-legged navigation surrogate. The current
project direction is no longer only a flat Nav2 demo: the active line is
FAST-LIO2 SLAM mapping, SLAM-cloud traversability extraction, PCT-style
cross-level path generation, and a safety-gated motion chain in Gazebo/RViz.

## Current Runtime Architecture

```text
Gazebo Fortress large_multilevel_complex_static.sdf
  -> native /scan and /livox/lidar_points
  -> livox_custom_bridge publishes /livox/lidar CustomMsg
  -> fastlio_mapping publishes /cloud_registered, /Laser_map, /Odometry
  -> fast_lio_map_aligner publishes /Laser_map_world and /cloud_registered_world
  -> slam_scan_projector publishes local /slam_scan
  -> terrain_pct_planner builds SLAM-cloud terrain graph from /Laser_map_world
  -> /pct_path and direct /cmd_vel_nav
  -> velocity_smoother + collision_monitor
  -> diff_drive_controller
```

The default FAST-LIO visual launch uses:

```bash
ros2 launch airos_experiments visual_fast_lio_navigation.launch.py \
  gui:=true \
  rviz:=true \
  terrain_send_nav2_goals:=true \
  terrain_execution_mode:=direct
```

## Current Verified Facts

- `visual_fast_lio_navigation.launch.py` defaults to `terrain_map_source:=slam_cloud`.
- The terrain planner subscribes to `/Laser_map_world`, not raw SDF terrain, in the active FAST-LIO route.
- `/Laser_map_colored`, `/terrain_traversability_cloud`, and `/pct_path` remain RViz-facing outputs.
- `nav_stack_mode:=safety_only` is the default for the FAST-LIO visual launch, so Nav2 is mainly the safety and smoothing layer for direct PCT execution.
- SDF geometry is still used for Gazebo worlds, map generation, deterministic tests, and the older SDF planner mode.
- Current speed chain was raised for responsiveness: direct planner `0.30 m/s`, flat speed limit `0.32 m/s`, slope speed limit `0.16 m/s`, base linear limit `0.32 m/s`, base angular limit `0.55 rad/s`.

## Latest Online Acceptance Evidence

The latest stable single-floor FAST-LIO/PCT-style demo evidence is:

```text
log/fast_lio_single_floor_demo/long_corridor_goal_guard_20260516_184353/
accepted: true
laser_map_points_max: 263068
cmd_vel_nav_count_max: 487
pct_path_max_z: -0.032202
wheel/gazebo distance to (8.0, -9.0): 0.266004 m
launch log: received terrain goal, started terrain-guided direct tracking,
            terrain direct tracking goal reached
```

This is the current best display path for the advanced SLAM -> terrain planning
-> direct execution chain. The previous shorter `near_goal` target remains a
quick smoke test:
`log/fast_lio_single_floor_demo/near_goal_after_zwindow_snap_20260516_175830/`
passed with wheel/Gazebo distance `0.223881 m`.

The cross-level line still generates high paths from live FAST-LIO2 maps. An
earlier online FAST-LIO2 run after sparse component bridge and speed-chain
updates produced:

```text
READY True
HIGH_SEEN True
PATH_UPDATES 3
PATH_LAST endpoint approx (6.97, 11.42, 2.155)
PATH_MAXZ_OVERALL 2.155
CMD_COUNT 797
CMD_NONZERO 796
CMD_MAX_LINEAR 0.1999766994
CMD_MAX_ANGULAR 0.28
ODOM_DXY 6.49 m
CLOUD_FIRST points 69075
CLOUD_LAST points 594777
```

This proves the live FAST-LIO2 SLAM map can grow, reconnect enough of the
cross-level structure, and produce a high `/pct_path` from `/Laser_map_world`.
It also proves the motion command chain is active.

This does not prove physical arrival on the high deck. The remaining limitation
is execution quality: the robot still needs a dedicated ramp/stair approach and
3D waypoint following improvement before high-deck arrival can be claimed.

The newest bounded cross-level reprobe after the final high-drop guard is:

```text
log/cross_level_after_high_drop_guard_20260516_182151/
laser_map_points_max: 635571
pct_path_max_z: 2.206236
first high path elapsed: 85.108 s
cmd counts: /cmd_vel_nav=2491, /cmd_vel_smoothed=2759, base=2658
fast_lio_z_max: 0.343199
wheel_odom_z: 0.0
gazebo_z: about -0.005
gazebo final goal distance: about 11.67 m
```

Fact: high `/pct_path` generation and command publication remain alive.
Fact: Gazebo and wheel odom still do not show physical climb.
Inference: after the high-drop guard, the next narrow blocker is ramp-to-deck
transition or high-deck target execution around targets such as
`(6.59, 4.84, 1.07)`, where direct tracking keeps commanding motion but the
surrogate remains near `z=0.34` aligned odom and ground-level Gazebo height.

Cross-level goal publication is now 3D/floor-aware at the tool level:

```bash
ros2 run airos_experiments publish_terrain_goal \
  --x 6.0 --y 13.0 --z 2.2 \
  --publish-count 5 --rate-hz 1
```

The terrain planner uses a high `PoseStamped.pose.position.z` as an effective
minimum goal layer constraint for that goal. `cross_level_evidence_probe` also
records `goal_xyz` so later runtime logs preserve the intended target floor.

The newest 3D-goal runtime evidence is:

```text
log/cross_level_3d_goal_20260516_190046/
goal_xyz_last: [6.0, 13.0, 2.2]
laser_map_points_max: 674569
pct_path_max_z: 2.275243
cmd counts: /cmd_vel_nav=2513, /cmd_vel_smoothed=2845, base=2830
fast_lio_z_max: 0.432377
wheel_odom_z_max: 0.0
gazebo_z_max: -0.005001

log/cross_level_3d_goal_long_20260516_191019/
goal_xyz_last: [6.0, 13.0, 2.2]
laser_map_points_max: 785990
pct_path_max_z: 2.344505
cmd counts: /cmd_vel_nav=4240, /cmd_vel_smoothed=4734, base=4734
cmd age max: about 0.14 s / 0.061 s / 0.071 s
fast_lio_z_max: 0.36647
wheel_odom_z_max: 0.0
gazebo_z_max: -0.004997
gazebo final goal distance: 15.572861 m
```

Fact: the 3D goal tool, live FAST-LIO map growth, high `/pct_path`, and fresh
command chain all work in the cross-level run.
Fact: wheel odom and Gazebo pose still do not physically climb.
Inference: the next narrow blocker is physical execution around ramp/deck/step
contact or the current diff-drive surrogate capability, not goal publication or
high-path generation.

The newest ramp-physics and main-chain recheck changes that interpretation:

```text
log/lower_ramp_physics_after_landing_fix_20260516_195621/
gazebo_z_max: 0.934999
gazebo_y_max: 12.240317
accepted_upper_landing: true

log/cross_level_after_landing_fix_goal_ok_20260516_200724/
laser_map_points_max: 542955
pct_path_max_z: 2.341184
cmd counts: /cmd_vel_nav=2132, smoother=2380, base=2395
gazebo_z_max: -0.005001

log/cross_level_after_regressive_prefix_fix_20260516_201433/
laser_map_points_max: 482482
pct_path_max_z: 2.124622
cmd counts: /cmd_vel_nav=1761, smoother=1962, base=1962
gazebo_z_max: -0.005001
```

Fact: the repaired lower ramp is physically climbable by the current surrogate
when commanded directly.
Fact: the current wheel `/odom` z remains `0.0` because the odometry publisher is
2D; use Gazebo pose z as primary physical climb evidence unless odometry is
changed to 3D.
Fact: the main FAST-LIO/PCT/direct chain still generates high `/pct_path` and
fresh commands but does not drive the robot onto the physical lower ramp.
Inference: the next narrow blocker is SLAM graph/frontier entry selection around
low `slam_step` pseudo-entries near `(4.9, 1.4, 0.14)`, not a total ramp-physics
failure.

The newest static frontier-entry fix tightens that blocker:

```text
test_slam_frontier_path_prefers_ramp_entry_over_isolated_step_pair
Result: passed

python3 -m pytest \
  src/airos_experiments/test/test_visual_pointcloud_config.py \
  src/airos_experiments/test/test_slam_traversability_graph.py \
  src/airos_experiments/test/test_terrain_pointcloud_planner.py \
  src/airos_experiments/test/test_control_command_chain.py \
  src/airos_experiments/test/test_cross_level_evidence_probe.py -q
Result: 148 passed in 27.55s

git diff --check
Result: no output

colcon build --symlink-install
Result: 8 packages finished [1.15s]
```

Fact: high-floor frontier entry scoring now prefers continuous ramp/stair
vertical progress over an isolated step-pair attractor when both are visible.
Fact: this is a static planner safety fix only.
Pending: rerun one bounded cross-level runtime with the 3D goal to see whether
the robot now approaches the real lower ramp corridor instead of the low
`slam_step` pseudo-entry.

That bounded runtime was run once:

```text
log/cross_level_after_frontier_entry_fix_20260516_203725/
samples: 42
laser_map_points_max: 796963
pct_path_max_z_max: 2.064855
cmd counts: /cmd_vel_nav=2981, smoother=3284, base=3284
cmd age max: about 0.077 s / 0.062 s / 0.060 s
gazebo_z_max: -0.005001
gazebo final goal distance: 22.014349 m
accepted_high_path: true
accepted_physical_high: false
```

Fact: high `/pct_path` generation and command-chain freshness still work.
Fact: Gazebo pose still did not climb.
Fact: the later frontier target did shift toward `frontier=(-5.93,0.55)`, closer
to the true lower-ramp side, but earlier direct execution still spent a long
time on low-height `slam_ramp` targets near `(6.02,-11.40,0.36)`, far from the
valid high-goal approach.
Inference: after isolated step-pair filtering, another static path semantics
gap remained: low-height `slam_ramp` prefixes with little/no progress toward
the high goal can delay or reverse the run before the true ramp corridor is
used.

The next static fixes after that runtime add:

```text
test_direct_tracking_drops_regressive_low_ramp_prefix_before_high_entry
Result: passed

test_pending_final_goal_waits_for_active_frontier_endpoint
Result: passed

test_high_final_path_rejects_large_initial_goal_regression
Result: passed

python3 -m pytest \
  src/airos_experiments/test/test_visual_pointcloud_config.py \
  src/airos_experiments/test/test_slam_traversability_graph.py \
  src/airos_experiments/test/test_terrain_pointcloud_planner.py \
  src/airos_experiments/test/test_control_command_chain.py \
  src/airos_experiments/test/test_cross_level_evidence_probe.py -q
Result: 151 passed in 27.47s

git diff --check
Result: no output

colcon build --symlink-install
Result: 8 packages finished [0.84s]
```

Fact: direct tracking now drops low-height ramp/slope prefixes that fail to make
at least the regression tolerance of progress toward the final high goal, while
preserving high-floor ramp/deck detours.
Fact: final-goal planning now defers while an active frontier path is still
being executed, and rejects high final paths whose early low-floor prefix moves
substantially farther from the final high goal.

The bounded runtime after these guards was:

```text
log/cross_level_after_final_regression_guard_20260516_210455/
laser_map_points_max: 793376
cmd counts: /cmd_vel_nav=2142, smoother=2362, base=1538
fast_lio_z_max: 0.261517
wheel_odom_z_max: 0.0
gazebo_z_max: -0.005
gazebo final goal distance: 15.872381 m
accepted_physical_high: false
```

Fact: the old early switch to the far high final target did not recur. Launch
logs show the planner published a frontier path toward `frontier=(-3.56,1.20)`
and direct tracking advanced to a `slam_ramp` target near
`(-3.56,1.20,0.46)`.
Fact: collision monitor then reported `Robot to stop due to StopZone polygon`;
the stalled frontier was later released, and Gazebo pose z stayed at ground
height.
Fact: the runtime probe started after the goal publication, and `/pct_path` is
not latched. Therefore `pct_path_poses_max=0` in that summary is a sampling
artifact, not evidence that no path was published.
Inference: the current cross-level blocker has shifted from high-path
generation and command freshness to physical execution near the lower-ramp
edge, local safety scan / StopZone interaction, and insufficient ramp-center or
support-margin bias.
Pending: add diagnostics or scoring for ramp centerline/support margin before
changing safety limits; do not disable collision monitor to force a climb.

## Current Code State

There are intentional uncommitted changes in these areas:

- `src/airos_experiments/airos_experiments/slam_traversability_graph.py`: constrained component sparse step bridges for sparse FAST-LIO non-floor samples.
- `src/airos_experiments/airos_experiments/terrain_pct_planner.py`: ramp/step entry attractor scoring, frontier behavior fixes, single-floor goal z-window, PoseStamped.z high-floor goal constraint, direct final-goal snap guard, final-goal-aware direct completion, final high-path high-drop rejection, low-ramp-prefix filtering, and final-path regression guard.
- `src/airos_experiments/airos_experiments/terrain_goal_publisher.py`: repeated 3D terrain goal publisher for cross-level targets.
- `src/airos_experiments/launch/visual_fast_lio_navigation.launch.py`: faster direct tracking, larger SLAM map sample cap, higher step allowance, longer bounded frontier distance.
- `src/airos_control/config/go2w_controllers.yaml`: higher base velocity and acceleration limits.
- `src/airos_nav/config/nav2_params.yaml`: higher RPP/smoother velocity chain.
- Tests under `src/airos_experiments/test/`: regression coverage for the above.

## Latest Cross-Level Execution Snapshot

After the ramp support-margin and `/slam_scan` ramp-surface filtering work, the
direct tracker still stalled when the robot was XY-close to a high-ish
`slam_step` waypoint but physical height had not caught up. The latest TDD fix
keeps waypoint completion gated on physical height, but lets the command
lookahead target move to the next same-surface `slam_step` waypoint so the base
keeps pushing uphill instead of spinning on a near-zero-distance target.

New runtime evidence:

```text
log/cross_level_after_height_debt_lookahead_20260516_215319/
goal_xyz: [6.0, 13.0, 2.2]
samples: 44
/Laser_map_world sampled max points: 792874
/pct_path max z: 2.178018
cmd counts: /cmd_vel_nav=3101, smoother=3502, base=3496
cmd ages at end: 0.034s / 0.032s / 0.028s
fast_lio_xyz max z: 0.400256
wheel /odom max z: 0.0
Gazebo pose max z: -0.005001
Gazebo goal distance min: 13.662475m, final: 13.720846m
accepted_physical_high: false
```

Fact: the run again proves live SLAM growth, high `/pct_path`, and fresh command
chain.
Fact: the height-debt lookahead change improved execution distance: direct logs
show the robot progressed through low `slam_ramp`, reached the next frontier,
and later targeted `slam_step` waypoints around `(-4.29,3.35,0.72)` /
`(-3.97,4.46,0.74)`.
Fact: physical cross-level navigation is still not accepted. Gazebo z stayed at
ground height, and the final direct path was released as stalled.
Inference: the current active blocker is no longer only StopZone or near-zero
target distance. The direct target now oscillates between adjacent `slam_step`
lookahead candidates with large heading error, causing repeated zero linear
velocity despite fresh commands. The next fix should stabilize uphill step/ramp
heading selection or add path-tangent/curvature gating before changing safety
limits.

Follow-up control-chain evidence:

```text
log/cross_level_after_tangent_lookahead_20260516_220735/
Result: invalid for cross-level motion diagnosis
Reason: /cmd_vel_nav and /cmd_vel_smoothed were published, but base command
count stayed at 0 because collision_monitor lifecycle services were not ready
when lifecycle_activator made its single service wait.

log/control_chain_after_lifecycle_retry_20260516_221346/
samples: 14
/Laser_map_world sampled max points: 342888
/pct_path sampled max z: 0.475012
cmd counts: /cmd_vel_nav=915, smoother=1016, base=1015
Gazebo goal distance: 24.517079m -> 18.280812m
Gazebo pose max z: -0.005001
```

Fact: lifecycle activation now retries service availability and restored the
base command publisher in `safety_only` mode.
Fact: the short follow-up run proves XY motion and command propagation are back,
but it does not prove high `/pct_path` generation or physical ascent in that
specific sample window.
Pending: collision monitor repeatedly warned that `/slam_scan` timestamps were
about one second away from current node time and ignored the scan source. Before
the next long cross-level run, diagnose `/slam_scan` freshness/projector cost or
the collision-monitor source timeout instead of disabling the safety layer.

Follow-up `/slam_scan` freshness evidence:

```text
log/scan_freshness_after_support_index_20260516_222625/
samples: 14
slam_scan stale warnings: 0
/Laser_map_world sampled max points: 378853
/pct_path sampled max z: 0.470178
cmd counts: /cmd_vel_nav=918, smoother=994, base=1033
cmd ages at end: 0.020s / 0.039s / 0.039s
Gazebo goal distance: 17.844465m
Gazebo pose max z: -0.005001
```

Fact: the stale warning was caused by projector compute cost, not by a missing
command publisher. The ramp-surface support filter now uses a local spatial
support index instead of scanning the full sampled cloud for every point.
Fact: the short run proves collision monitor scan freshness recovered and base
command propagation remains healthy.
Pending: this run is still only a control/safety-chain recovery check. It did
not sample a high `/pct_path`, and Gazebo z stayed at ground height.

## Latest Single-Floor Demo Snapshot

After the lifecycle and `/slam_scan` freshness fixes, the single-floor
FAST-LIO/PCT/direct chain was rechecked with the reusable demo script.

```text
log/fast_lio_single_floor_demo/near_goal_after_scan_index_20260516_223232/
accepted: true
samples: 28
/Laser_map_world sampled max points: 248275
cmd counts: /cmd_vel_nav=200, smoother=235, base=235
final wheel/Gazebo goal distance: 0.222402m / 0.222402m
slam_scan stale warnings: 0

log/fast_lio_single_floor_demo/long_corridor_after_scan_index_20260516_223440/
accepted: true
samples: 48
/Laser_map_world sampled max points: 351188
/pct_path poses max: 10
/pct_path max z: -0.044546
cmd counts: /cmd_vel_nav=573, smoother=645, base=660
final wheel/Gazebo goal distance: 0.294605m / 0.294605m
slam_scan stale warnings: 0
```

Fact: both near-goal and longer single-floor routes reached the physical Gazebo
goal tolerance with live FAST-LIO map growth, direct tracking, command
propagation, and no `/slam_scan` freshness warnings.
Fact: the long-corridor run exercised the final-goal-after-map-update path:
launch logs recorded `pending final goal became reachable after FAST-LIO map
update`, then direct tracking reached the goal.
Inference: the fastest demonstrable result should use
`scripts/run_fast_lio_single_floor_demo.sh` as the current stable showcase while
cross-level physical ascent remains under active diagnosis.
Pending: this single-floor evidence does not prove cross-level physical ascent.
The next cross-level run should start from the now-healthy single-floor/control
baseline.

## Latest Cross-Level Stop Point

After the single-floor refresh, two cross-level bounded runs were used to check
whether the latest direct lookahead changes were enough to produce physical
ascent. They were not.

```text
log/cross_level_after_single_floor_refresh_20260516_223943/
samples: 42
/Laser_map_world sampled max points: 793960
/pct_path max z: 2.138636
cmd counts: /cmd_vel_nav=2977, smoother=3352, base=3337
slam_scan stale warnings: 0
Gazebo pose max z: -0.005001
Gazebo goal distance min/final: 12.201398m / 12.321149m
accepted_high_path: true
accepted_physical_high: false

log/cross_level_after_zigzag_lookahead_fix_20260516_225624/
samples: 34
/Laser_map_world sampled max points: 621554
/pct_path max z: 2.249888
cmd counts: /cmd_vel_nav=2335, smoother=2686, base=2686
slam_scan stale warnings: 0
Gazebo pose max z: -0.005001
Gazebo goal distance min/final: 11.985970m / 11.985970m
accepted_high_path: true
accepted_physical_high: false
direct_stalled: true
```

Fact: the current architecture still reliably produces live SLAM growth, high
`/pct_path`, and a healthy command chain.
Fact: the current Go2W surrogate still did not physically climb in Gazebo during
these runs. Do not continue claiming or implying cross-level completion.
Fact: the latest direct lookahead fix reduced one observed same-surface
lookahead zigzag case, but the next run still stalled around a low `slam_ramp`
target near `(5.12,1.43,0.46)` while Gazebo z stayed at ground height.
Inference: for fast visible progress, stop spending long cycles on this wheel
surrogate cross-level run. Treat cross-level as a second-stage branch requiring
either a ramp-capable execution/model update or a simpler validated multilevel
demo map before claiming high-floor arrival.

## Current Verification Snapshot

Latest verification completed on 2026-05-16 after the height-debt direct
lookahead, path-tangent lookahead, evidence-probe timeout, lifecycle
service-retry, `/slam_scan` support-index fix, and same-surface zigzag
lookahead guard:

```text
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest \
  src/airos_experiments/test/test_slam_scan_projector.py \
  src/airos_experiments/test/test_visual_pointcloud_config.py \
  src/airos_experiments/test/test_slam_traversability_graph.py \
  src/airos_experiments/test/test_terrain_pointcloud_planner.py \
  src/airos_experiments/test/test_control_command_chain.py \
  src/airos_experiments/test/test_cross_level_evidence_probe.py -q
Result: 166 passed in 28.19s

git diff --check
Result: no output

colcon build --symlink-install
Result: 8 packages finished [0.94s]

Focused checks:
python3 -m pytest src/airos_experiments/test/test_terrain_pointcloud_planner.py -q
Result: 49 passed in 3.21s

python3 -m pytest src/airos_experiments/test/test_slam_scan_projector.py -q
Result: 8 passed in 0.17s

bash scripts/cleanup_airos_runtime.sh
Result: [PASS] AIROS runtime processes cleaned.
```

Before a handoff commit or a new phase, run the final gate in
`pre_migration_handoff_report.md`.

## Non-Source Artifacts

- `build/`, `install/`, `log/`, `.pytest_cache/`, and `results/` are generated or ignored artifacts.
- Root-level PDF/DOCX files are user course/reference materials, not source.
- `docs/third_party_3d_dog_navi_ros2_AFL-3.0_LICENSE` documents the imported visual asset license.
