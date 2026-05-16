# 2026-05-16 Ramp Physics And SLAM Entry Gap

## Fact

- Direct Gazebo physics smoke now proves the `go2w_nav_eq` surrogate can climb
  the repaired lower ramp in `large_multilevel_complex`.
- Evidence:

```text
log/lower_ramp_physics_after_landing_fix_20260516_195621/summary.json
samples: 38
gazebo_z_first: 0.094382
gazebo_z_max: 0.934999
gazebo_y_first: -6.600012
gazebo_y_max: 12.240317
accepted_upper_landing: true
base_cmd_count_max: 1299
wheel_z_max: 0.0
```

- `wheel_z_max=0.0` is expected for the current wheel odometry chain because
  the Ignition odometry publisher is configured as 2D. Use Gazebo pose z as the
  primary physical climb evidence unless the odometry publisher is changed to
  3D.
- The lower-ramp physical blocker had two scene/config roots:
  - `second_floor_deck` originally covered the lower ramp and left insufficient
    body clearance.
  - `ramp_upper_landing` originally started too early, creating an abrupt
    leading edge; after moving it, `ramp_upper_landing` was also found to be
    misclassified as a ramp because its name contains `ramp`.
- Current world fixes are in both
  `src/airos_sim/worlds/large_multilevel_complex.sdf` and
  `src/airos_sim/worlds/large_multilevel_complex_static.sdf`.
- Current code fix: `_is_ramp_label` / `_is_slope_label` no longer classify
  labels containing `landing` as slope/ramp surfaces.

## Runtime Rechecks

The corrected 3D goal command is:

```bash
ros2 run airos_experiments publish_terrain_goal \
  --x 6.0 --y 13.0 --z 2.2 \
  --frame-id map --publish-count 20 --rate-hz 2
```

Do not use `--stamp-now`; that flag does not exist.

Two bounded headless `visual_fast_lio_navigation.launch.py` runs after the ramp
physics fix show the main chain is still not physically crossing floors:

```text
log/cross_level_after_landing_fix_goal_ok_20260516_200724/summary.json
laser_map_points_max: 542955
pct_path_max_z: 2.341184
cmd_vel_nav_count_max: 2132
base_cmd_count_max: 2395
gazebo_z_max: -0.005001
gazebo_last: [4.102172, 1.016865, -0.005001]
accepted_high_path: true
accepted_physical_high: false

log/cross_level_after_regressive_prefix_fix_20260516_201433/summary.json
laser_map_points_max: 482482
pct_path_max_z: 2.124622
cmd_vel_nav_count_max: 1761
base_cmd_count_max: 1962
gazebo_z_max: -0.005001
gazebo_last: [4.037324, 1.374671, -0.005001]
accepted_high_path: true
accepted_physical_high: false
```

## Inference

- The current remaining blocker is not "the surrogate cannot climb any ramp".
  Direct base command on the repaired lower ramp physically reaches the upper
  landing.
- The main-chain blocker is now more likely SLAM graph / frontier entry
  selection and direct tracking around a low `slam_step` pseudo-entry near
  `(4.9, 1.4, 0.14)`, which is far from the true lower ramp corridor around
  `x=-4.7`.
- The final high `/pct_path` appears late and remains valid as a high-path
  generation proof, but direct tracking still does not guide the robot onto the
  physical lower ramp.

## Pending

- Add a planner-side guard or scoring change so high-floor frontier approach
  prefers the true low end of a detected ramp/stair sequence and rejects low,
  isolated `slam_step` bumps that do not provide sustained vertical progress.
- Preserve existing sparse bridge and wall-base rejection tests when editing
  `slam_traversability_graph.py`.
- When editing `terrain_pct_planner.py`, keep running frontier, direct tracking,
  and visual launch config tests.
- Physical cross-level acceptance still requires Gazebo pose to reach high
  elevation and approach the target floor; high `/pct_path` alone is not enough.
