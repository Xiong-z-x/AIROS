# Planner Comparison Showcase

Date: 2026-05-17

## Purpose

This showcase extends the accepted single-floor SLAM/Nav2/FAST-LIO display with
a planner-only comparison mode. It is designed for report/demo use: the robot
stays still, RViz receives a goal, and four planning algorithms publish their
paths and metrics at the same time.

## Stage 1 Baseline

The accepted baseline is recorded as:

- tag: `phase1-perfect-slam-nav-fastlio-costmap`
- commit: `826e525 Fuse FAST-LIO projected scan into Nav2 costmaps`
- report: `docs/handoff/phase1_perfected_baseline_2026-05-17.md`

## Complex Single-Floor Scene

New scene files:

- `src/airos_sim/worlds/single_floor_planner_showcase.sdf`
- `src/airos_sim/worlds/single_floor_planner_showcase_static.sdf`
- `src/airos_nav/maps/single_floor_planner_showcase.yaml`

The scene keeps the previous large single-floor structure and adds office desks,
chairs, shelves, columns, static pedestrian-like cylinders, a narrow passage,
and a U-shaped trap area. These obstacles are simple SDF geometry so the demo is
fast to reproduce and does not depend on external meshes.

## SLAM Mapping Display

Use this mode when presenting online mapping and FAST-LIO evidence:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch airos_experiments visual_fast_lio_navigation.launch.py \
  world:=single_floor_planner_showcase \
  map:=src/airos_nav/maps/single_floor_planner_showcase.yaml \
  terrain_world_file:=src/airos_sim/worlds/single_floor_planner_showcase_static.sdf \
  gui:=true rviz:=true log_level:=warn
```

If the map needs to be regenerated from online SLAM:

```bash
ros2 run airos_experiments save_slam_map \
  --prefix src/airos_nav/maps/single_floor_planner_showcase_slam \
  --posegraph src/airos_nav/maps/single_floor_planner_showcase_slam \
  --timeout-sec 20
```

## Planner-Only Comparison

Use this mode after the scene/map is ready:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch airos_experiments planner_comparison.launch.py \
  gui:=true rviz:=true log_level:=warn
```

In RViz, use `2D Goal Pose`. The comparison node publishes:

- `/planner_compare/smac_path`
- `/planner_compare/theta_star_path`
- `/planner_compare/q_learning_path`
- `/planner_compare/rrt_star_path`
- `/planner_compare/metrics_markers`
- `/planner_compare/metrics_summary`

The comparison mode does not send Nav2 navigation goals and does not publish
`/cmd_vel`. It starts only map/planner infrastructure plus the comparison node.

## Algorithm Roles

- SmacPlanner2D: Nav2 production-grade grid planner baseline.
- Theta*: Nav2 official any-angle planner, useful for showing straighter routes.
- Q-learning: deterministic grid RL baseline using a reward/value policy on the
  occupancy map; fast and reproducible, not a deep RL training claim.
- RRT*: seeded sampling planner for showing exploration behavior in complex
  obstacle layouts.

## Known Boundaries

- The saved seed map is generated from SDF geometry for repeatable planner
  comparison. A live SLAM map can be saved later and swapped in with the same
  launch argument.
- The Gazebo diff-drive controller still exists because the robot model is
  spawned, but planner comparison mode does not start the Nav2 controller stack
  and does not publish velocity commands.
- This work is single-floor only. Cross-level and footed-model experiments are
  intentionally outside this showcase.
