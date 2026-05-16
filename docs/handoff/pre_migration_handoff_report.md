# AIROS Pre-Migration Handoff Report

Status: authoritative migration report.
Last updated: 2026-05-15.

## Total Goal

Build a reproducible AIROS autonomous navigation prototype for a Go2W-style
robot surrogate in Ubuntu 22.04 WSL2, ROS 2 Humble, and Ignition Gazebo
Fortress. The current technical direction is:

```text
FAST-LIO2 SLAM map
  -> SLAM-cloud traversability graph
  -> PCT-style cross-level /pct_path
  -> direct terrain tracking through Nav2 safety/smoothing chain
  -> Gazebo motion evidence
```

The older flat Nav2 + route graph baseline remains valuable and verified, but
the next phase should continue from the FAST-LIO2 SLAM/PCT execution line.

## Current Phase Position

Completed and stable enough for handoff:

- ROS/Gazebo/WSL environment baseline.
- Go2W-style navigation equivalent model and control chain.
- Flat single-floor Nav2 acceptance and clean-runner evidence.
- FAST-LIO2 sensor chain and RViz map visualization.
- SLAM-cloud terrain graph and `/pct_path` generation from `/Laser_map_world`.
- High-floor `/pct_path` generation from a live FAST-LIO2 map.
- Faster but still bounded motion chain for the visual demo.

Not completed:

- Physical high-deck arrival in Gazebo.
- Full cross-level route-constrained batch.
- Upstream CUDA PCT-planner or trained RL runtime integration.

## Environment And Dependencies

Current baseline:

```text
OS: Ubuntu 22.04.5 LTS on WSL2
Kernel: 6.6.87.2-microsoft-standard-WSL2
ROS: ROS 2 Humble
Gazebo: Ignition Gazebo Fortress 6.16.0
Nav2: 1.1.20
slam_toolbox: 2.6.10
ros_gz: 0.244.23
gz_ros2_control: 0.7.18
```

Use `ign gazebo`, not `gz sim`. For GUI, keep the WSL-stable render path unless
explicitly testing rendering:

```text
--render-engine ogre --render-engine-gui ogre
```

## Key Modules

- `src/airos_sim`: Gazebo worlds, native sensors, bridge config, static/dynamic world variants.
- `src/airos_go2w_description`: Go2W-style URDF/Xacro and optional visual meshes.
- `src/airos_control`: diff-drive controller and base velocity limits.
- `src/fast_lio`: FAST-LIO2 package and AIROS sim config.
- `src/livox_ros_driver2`: message compatibility for Livox `CustomMsg`.
- `src/airos_nav`: Nav2 params, safety-only/controller/full stack launch, maps, route graphs, RViz config.
- `src/airos_experiments`: bridges, planners, graph builders, test probes, launch orchestration, metric tools.

## Current Technical Route

The active FAST-LIO2 launch is:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch airos_experiments visual_fast_lio_navigation.launch.py \
  gui:=true \
  rviz:=true \
  terrain_send_nav2_goals:=true \
  terrain_execution_mode:=direct
```

Important defaults:

- `world:=large_multilevel_complex`
- `terrain_map_source:=slam_cloud`
- `slam_map_topic:=/Laser_map_world`
- `path_topic:=/pct_path`
- `goal_topic:=/terrain_goal_pose`
- `nav_stack_mode:=safety_only`
- `terrain_execution_mode:=direct`
- `slam_map_max_points:=180000`
- `frontier_max_path_distance:=14.0`
- `max_step_height:=0.50`
- `direct_max_linear_speed:=0.30`
- `direct_max_angular_speed:=0.45`
- `flat_speed_limit:=0.32`
- `slope_speed_limit:=0.16`

## Completed Work To Date

- Project scaffold and ROS packages are in place.
- Environment gate scripts and docs exist.
- Static Nav2 single-floor clean-runner acceptance has historical 20/20 evidence.
- Route graph compute and single route-waypoint smoke have passed historically.
- Dynamic scan-layer smoke has passed historically.
- FAST-LIO2 takes Livox-style CustomMsg and IMU inputs and publishes map/odom outputs.
- `/Laser_map_world` alignment and `/cloud_registered_world` local projection exist.
- `terrain_pct_planner` can build from SDF or SLAM cloud, but the active demo uses SLAM cloud.
- `slam_traversability_graph.py` owns SLAM-cloud terrain extraction and graph connectivity.
- Sparse non-floor bridge logic and ramp/step frontier entry logic are regression-covered.
- Runtime evidence shows a high `/pct_path` with max z above 2.0 from live FAST-LIO2 data.

## Current True State

Accepted:

- FAST-LIO2 SLAM-cloud map can feed the PCT-style graph.
- `/pct_path` can be produced from `/Laser_map_world`, including high-floor path output after map growth.
- The motion command chain is active and produces non-zero `cmd_vel` and odometry movement.
- The code builds and targeted tests pass for the latest speed-chain and planner changes.

Not accepted:

- The robot has not been proven to physically climb to and stop on the high deck.
- Full advanced-world route-constrained batch remains future work.
- The upstream PCT-planner CUDA pipeline and RL planner are not installed as stable runtime backends.

## Fixed During This Sealing Pass

- Runtime residual processes were cleaned before verification.
- Handoff package was created under `docs/handoff/`.
- Stale planning files were marked as historical and linked to the current handoff entry.
- Documentation now distinguishes high `/pct_path` generation from physical high-deck arrival.
- Directory/source boundaries are recorded to avoid committing generated logs or user course materials.

## Remaining Limits

- Physical high-deck execution is the first real next task.
- The direct terrain tracker is still a 2D velocity controller following 3D path intent; it needs stronger ramp-entry and height-aware execution.
- FAST-LIO map quality varies by sensor mode, motion path, and WSL/Gazebo timing.
- Safety scan projection helps local blocking but must not be over-applied to final SLAM graph planning.
- The generated `log/` and `results/` evidence should be treated as local artifacts unless explicitly curated.

## Next Starting Task

Start with physical execution of the already generated high `/pct_path`:

1. Re-run the FAST-LIO visual chain headless.
2. Publish target `(6.0, 13.0, 2.2)` with the 3D goal helper:
   `ros2 run airos_experiments publish_terrain_goal --x 6.0 --y 13.0 --z 2.2 --publish-count 5 --rate-hz 1`.
3. Confirm `/pct_path` reaches `z > 2.0`.
4. Instrument direct tracking with ramp-entry, path index, robot pose, local slope, and command output.
5. Fix why the robot follows low-floor progress but does not climb the high path.

Do not restart by rebuilding the whole planning stack. The planning map and
high path generation are now far enough along; the bottleneck is execution.

## Final Gate Before Handoff Commit

Executed on 2026-05-15:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest \
  src/airos_experiments/test/test_visual_pointcloud_config.py \
  src/airos_experiments/test/test_slam_traversability_graph.py \
  src/airos_experiments/test/test_terrain_pointcloud_planner.py \
  src/airos_experiments/test/test_control_command_chain.py -q
git diff --check
colcon build --symlink-install
bash scripts/cleanup_airos_runtime.sh
```

Observed results:

```text
pytest: 117 passed in 29.96s
git diff --check: no output
colcon build: 8 packages finished [1.28s]
cleanup: [PASS] AIROS runtime processes cleaned.
```
