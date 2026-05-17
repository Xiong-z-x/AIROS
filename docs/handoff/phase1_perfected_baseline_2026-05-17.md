# Phase 1 Perfected Baseline

Date: 2026-05-17
Git tag: `phase1-perfect-slam-nav-fastlio-costmap`
Commit: `826e525 Fuse FAST-LIO projected scan into Nav2 costmaps`

## Baseline Meaning

This tag records the first stable single-floor demonstration baseline after the
SLAM/Nav2/FAST-LIO visualization and costmap chain was accepted in RViz.

The baseline is intended as a recovery point before the next planner-comparison
phase. If later planner experiments become unstable, restore this exact state by
checking out the tag instead of guessing which intermediate commit was stable.

## Current Runtime Chain

- Simulator and robot: `airos_sim` single-floor Gazebo world with the wheeled
  Go2-style model.
- Online mapping: `slam_toolbox` publishes the 2D occupancy map on `/map`.
- FAST-LIO evidence: aligned colorized point cloud is shown on
  `/Laser_map_colored`.
- FAST-LIO-to-costmap bridge: `slam_scan_projector` projects the FAST-LIO cloud
  into `/slam_scan`.
- Nav2 costmaps: both local and global costmap obstacle layers consume `/scan`
  and `/slam_scan`.
- Startup ordering: `slam_nav_coordinator` gates Nav2 lifecycle startup until
  the online map covers the robot.
- Default RViz display includes `/map`, `/Laser_map_colored`, `/slam_scan`,
  `/scan`, robot TF/model, and `/plan`.

## Launch Command

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch airos_experiments visual_fast_lio_navigation.launch.py \
  gui:=true rviz:=true log_level:=warn
```

Default launch values at this baseline:

- `world:=single_floor_complex_large`
- `localization:=slam_toolbox_mapping`
- `slam_nav_startup:=gated`
- `planner_profile:=research`
- `fast_lio_debug:=true`
- `colorized_pointcloud:=true`
- `nav_stack_mode:=full`

## Recovery

Safe inspection checkout:

```bash
git switch --detach phase1-perfect-slam-nav-fastlio-costmap
```

Branch from the stable baseline:

```bash
git switch -c recover-phase1 phase1-perfect-slam-nav-fastlio-costmap
```

Do not use destructive reset commands unless the user explicitly requests that
the current working tree be discarded.

## Known Boundaries

- This baseline is a single-floor SLAM/Nav2/FAST-LIO costmap demonstration.
- It does not claim cross-floor or stair/ramp physical navigation completion.
- The robot is still the stable wheeled model; the footed model experiment was
  reverted because its TF/RobotModel chain was unstable.
- In WSL/RViz, OpenGL shader warnings may appear and are not by themselves a
  navigation-chain failure.
