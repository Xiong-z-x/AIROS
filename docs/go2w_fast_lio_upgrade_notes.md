# Go2W FAST-LIO2 Visual Navigation Upgrade Notes

Date: 2026-05-07

## Goal

This note records the upgrade from the original flat Nav2 demonstration toward
a Go2W-style visual navigation demo similar in visible effect and topic surface
to `https://github.com/ypat999/3d_dog_navi_ros2`.

Target effect:

- Go2W four-wheel legged robot visible in Gazebo and RViz.
- Larger and more complex indoor map.
- RViz without Map, TF, RobotModel, point-cloud, or QoS status errors.
- Visible point-cloud map.
- FAST-LIO2 consumes Livox-style LiDAR and IMU topics.
- RViz 2D Goal / Nav2 action generates a path and drives the robot in Gazebo.
- WSL2 keeps the stable Fortress + OGRE GPU-rendered GUI baseline.

## Reference Repository Findings

The `ypat999/3d_dog_navi_ros2` repository is used as an interface and
target-effect reference, not as a directly reusable runtime for this WSL2
Fortress project.

Relevant topic surface:

```text
/livox/lidar
/livox/imu
/cloud_registered
/Laser_map
/odom
/cmd_vel
```

Imported or mirrored concepts:

- FAST-LIO2 ROS 2 fork role: `fast_lio`, executable `fastlio_mapping`.
- Livox CustomMsg ABI.
- RViz point-cloud map topics `/Laser_map` and `/cloud_registered`.
- Nav2 command flow through `/cmd_vel`.

Not imported as hardware runtime:

- Full `livox_ros_driver2` hardware driver, because it requires Livox SDK2
  shared libraries and real device configuration.
- PCT/ego planner or full CHAMP-style legged gait stack.

## Implemented Artifacts

FAST-LIO2 and Livox compatibility:

```text
src/fast_lio/
src/fast_lio/config/airos_sim.yaml
src/livox_ros_driver2/msg/CustomMsg.msg
src/livox_ros_driver2/msg/CustomPoint.msg
```

Simulation and sensor bridge:

```text
src/airos_sim/worlds/single_floor_lab.sdf
src/airos_sim/launch/sim.launch.py
src/airos_experiments/airos_experiments/pointcloud_emulator.py
src/airos_experiments/airos_experiments/imu_republisher.py
src/airos_experiments/airos_experiments/scan_emulator.py
```

Visual launch entries:

```text
src/airos_experiments/launch/visual_navigation.launch.py
src/airos_experiments/launch/visual_fast_lio_navigation.launch.py
```

RViz and control:

```text
src/airos_nav/rviz/nav.rviz
src/airos_go2w_description/rviz/model.rviz
src/airos_control/config/go2w_controllers.yaml
```

## Runtime Design

Stable visual navigation:

```text
Gazebo /odom + /imu
  -> scan_emulator publishes /scan
  -> pointcloud_emulator publishes /livox/lidar, /cloud_registered, /Laser_map
  -> Nav2 plans and controls /cmd_vel
  -> RViz displays TF, Go2W, /scan, /cloud_registered, /Laser_map, /plan
```

FAST-LIO2 visual navigation:

```text
Gazebo /odom + /imu
  -> imu_republisher publishes /livox/imu
  -> Gazebo native point cloud publishes /livox/lidar_points
  -> livox_custom_bridge publishes Livox CustomMsg /livox/lidar
  -> fastlio_mapping publishes /cloud_registered, /Laser_map, /Odometry
  -> fast_lio_localization_bridge publishes dynamic map->odom
  -> Nav2 plans and controls /cmd_vel
  -> RViz displays FAST-LIO point clouds and Nav2 path/control state
```

Important boundary:

```text
FAST-LIO2 proves the 3D point-cloud mapping and odometry side of the demo.
The stable visual launch keeps the static map->odom and wheel-odometry control
chain for goal execution. The FAST-LIO visual launch adds an external bridge
that publishes dynamic map->odom into Nav2.
```

## Fixes Applied

RViz auto-start:

- Top-level visual launch files isolate child launch `rviz=false`
  configurations inside scoped groups.
- `sim.launch.py` resolves the `pointcloud` switch inside `_launch_setup()` so
  returned nodes do not lose launch configuration scope.
- RViz launch is delayed until the TF/control chain is normally available.

RobotModel and TF:

- `diff_drive_controller` now publishes `odom -> base_footprint`.
- `robot_state_publisher` publishes the fixed URDF tree
  `base_footprint -> base_link -> sensor/wheel/leg links`.
- RobotModel `/robot_description` QoS is `Reliable + Transient Local`, matching
  `robot_state_publisher` and avoiding late-join warnings.

Static TF:

- `map -> odom` and `map -> fast_lio_map` static transforms use new-style
  `static_transform_publisher` arguments.

RViz display:

- `/Laser_map` RViz display uses Volatile durability, compatible with FAST-LIO2
  and the emulator map cloud.
- `/scan` is published Reliable so RViz, Nav2 and collision monitor can all
  receive it.
- Nav2 `/map` is restored in the RViz config, but it is kept disabled by
  default on this WSLg machine because enabling it triggers the known
  `indexed_8bit_image` GLSL link bug. The point-cloud map remains the active
  fallback.

Performance:

- Gazebo GUI keeps `--render-engine ogre --render-engine-gui ogre`.
- `LIBGL_ALWAYS_SOFTWARE=0`, `__GL_SYNC_TO_VBLANK=0`, and `vblank_mode=0` are
  set by the visual launches.
- Point-cloud and scan emulators keep bounded publish rates and point counts.

## Verification Evidence

Stable visual navigation:

```text
Command:
ros2 launch airos_experiments visual_navigation.launch.py \
  gui:=false rviz:=true dynamic_obstacles:=false use_route:=true log_level:=warn

Evidence:
rviz2 process started by launch.
RViz OpenGL version: 4.2.
/Laser_map width: 10496
/cloud_registered width: 5100
map -> base_link TF available.
```

FAST-LIO2 visual navigation:

```text
Command:
ros2 launch airos_experiments visual_fast_lio_navigation.launch.py \
  gui:=false rviz:=true use_route:=true log_level:=warn

Evidence:
rviz2 process started by launch.
RViz OpenGL version: 4.2.
/livox/imu publisher count: 1
/livox/lidar publisher count: 1
/scan publisher count: 1
/scan subscription count: 4
/cloud_registered width: 1525
/Laser_map width: 92963
/Odometry frame_id: fast_lio_map
/Odometry child_frame_id: fast_lio_body
No incompatible QoS warnings in the final FAST-LIO2 verification log.
```

RobotModel audit:

```text
URDF links: 14
map-frame reachable links: 14
missing links: 0
/robot_description publisher QoS: Reliable + Transient Local
/robot_description RViz subscriber QoS: Reliable + Transient Local
/odom child_frame_id: base_footprint
RViz log audit: no RobotModel, No transform, GLSL, or QoS errors.
```

Planning and control:

```text
ComputePathToPose goal: map (2.0, -1.5)
Result: SUCCEEDED
Path poses: 37

NavigateToPose goal: map (2.0, -1.5)
Result: SUCCEEDED
Final /odom approximately: x=1.962, y=-1.413
Final FAST-LIO /Odometry approximately: x=1.972, y=-1.414

RobotModel regression navigation:
NavigateToPose goal: map (1.0, -0.5)
Result: SUCCEEDED
Final /odom approximately: x=0.984, y=-0.437
```

FAST-LIO2 warning note:

```text
One transient "Sensor data lost" warning appeared during a longer navigation
run and was immediately followed by "Lidar sensor data recovered!".
/cloud_registered kept publishing at about 4.94 Hz after the warning.
```

## Commands

Stable visual demo:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch airos_experiments visual_navigation.launch.py \
  gui:=true \
  rviz:=true \
  dynamic_obstacles:=false \
  use_route:=true
```

FAST-LIO2 visual demo:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch airos_experiments visual_fast_lio_navigation.launch.py \
  gui:=true \
  rviz:=true \
  use_route:=true
```

Manual path planning smoke:

```bash
ros2 action send_goal /compute_path_to_pose \
  nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 2.0, y: -1.5, z: 0.0}, orientation: {w: 1.0}}}, planner_id: GridBased}"
```

Manual navigation smoke:

```bash
ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: -1.5, z: 0.0}, orientation: {w: 1.0}}}}"
```

## 2026-05-09 Advanced Runtime Update

- `sensor_source:=native` now keeps the raw Gazebo point cloud on
  `/livox/lidar_points` and converts it to Livox `CustomMsg` on
  `/livox/lidar` for FAST-LIO.
- `advanced_indoor_ramp` adds a larger indoor scene with ramp geometry,
  matching Nav2 seed map, route graph, missions, and Gazebo physical moving
  obstacles.
- `open_source_scene_assets:=true` and `robot_visual_profile:=reference_mesh`
  add optional AFL-3.0 Building / Go2W reference meshes from
  `ypat999/3d_dog_navi_ros2`.
- `planner_profile:=research` starts a separate Nav2 research profile using
  MPPI controller settings while the original baseline profile remains
  selectable.
- RViz shows the raw Gazebo cloud on `/livox/lidar_points`; FAST-LIO owns
  `/cloud_registered` and `/Laser_map`.

## Remaining Limitations

- WSL2/Fortress native Gazebo LiDAR is usable only in the software-stable
  rendering mode on this machine. Hardware OpenGL mode still crashes in OGRE2,
  so it is not the stable default for this project.
- `/livox/lidar` is a Livox `CustomMsg` converted from the Gazebo native
  `/livox/lidar_points` point cloud. It is not real hardware packet timing.
- `src/livox_ros_driver2` is message-only compatibility, not a real Livox
  hardware driver.
- Full PCT-planner and reinforcement-learning planners are documented as
  research targets, not promoted to the stable runtime.
- Optional Go2W reference meshes are visual-only. There is still no full Unitree
  low-level legged controller, CHAMP gait, or official Go2W hardware interface.
- FAST-LIO2 localization is connected to Nav2 in the external-localization
  visual launch through a bridge that publishes dynamic `map -> odom`. The
  stable visual launch still keeps the static `map -> odom` chain as the
  fallback.

## 2026-05-10 Terrain Cloud and Cross-Level Planning Update

Root cause of the missing ramp cloud:

- The previous `pointcloud_emulator` used the 2D scan obstacle parser and only
  sampled vertical obstacle side walls.
- Traversable horizontal or sloped surfaces such as `floor`,
  `wide_access_ramp`, and `mezzanine_deck_visual` were not sampled, so
  FAST-LIO2 could not map the ramp even when `/livox/lidar` was publishing.

Implemented fix:

- `sdf_geometry.py` parses SDF model/link/collision poses, including roll,
  pitch, and yaw.
- `pointcloud_emulator` now samples box top surfaces, side surfaces, and
  cylinder surfaces from SDF collision geometry.
- The ramp, floor, and mezzanine deck now enter `/livox/lidar_points`, the
  Livox `CustomMsg` bridge, FAST-LIO2, `/cloud_registered`, `/Laser_map`, and
  the RViz structural-color `/Laser_map_colored` display when
  `sensor_source:=emulated` is used.
- `terrain_pct_planner` parses the same traversable SDF surfaces, builds a
  height-aware terrain graph, publishes `/terrain_traversability_cloud` and
  `/pct_path`, then sends terrain-guided `NavigateThroughPoses` waypoints.
- The visual demo RViz goal tool publishes to `/terrain_goal_pose`, not
  Nav2's default `/goal_pose`. This prevents `bt_navigator` from also starting
  a direct `NavigateToPose` action while the terrain planner sends
  `NavigateThroughPoses`.
- Terrain-guided Nav2 goals also drop the first waypoint when it lies inside
  `start_waypoint_clearance:=0.75` of the current pose. This avoids sending a
  near-zero first waypoint that can make the controller rotate locally instead
  of progressing toward the real path. The graph start itself still uses the
  current odometry z when available. If Gazebo publishes flat odometry
  `z == 0`, the planner uses `initial_surface_z_hint:=robot_spawn_z` near the
  initial pose and the last planned terrain path nearby, so a robot spawned on
  the ramp plans from the ramp and can descend before going to a lower-floor
  target.
- `collision_monitor` is launched inline from `airos_nav/launch/nav.launch.py`
  with `lifecycle_manager_collision_monitor`, instead of including Nav2's
  default collision-monitor launch. This avoids an inactive collision monitor
  that subscribes to `/cmd_vel_smoothed` but does not reliably forward commands
  to `/diff_drive_controller/cmd_vel_unstamped`.
- Baseline Nav2 progress checking is tuned for the slow visual quadruped
  surrogate: `required_movement_radius:=0.18` and
  `movement_time_allowance:=20.0`. The controller still reports real stalls,
  but no longer aborts normal low-speed turning immediately.
- The default visual emulated cloud now uses `pointcloud_spacing:=0.16` and
  keeps up to 12k live LiDAR points. `/Laser_map_colored` keeps up to 220k
  sampled map points and hides points below `z=0.08` so the first-floor ground
  does not dominate the SLAM-map view.

Dynamic obstacle visibility:

- `visual_fast_lio_navigation.launch.py` exposes
  `dynamic_obstacles:=true|false` and now defaults the visual demo to `false`.
- In native sensor mode a lightweight marker-only scan emulator publishes
  `/dynamic_obstacles/markers` without taking over `/scan`.
- Gazebo physical moving obstacles are still controlled separately by
  `physical_dynamic_obstacles:=true`.
- The module is intentionally off by default because the current software marker
  overlay does not necessarily correspond to visible Gazebo moving bodies. Treat
  it as a later dynamic-obstacle task, not part of the default SLAM demo.

Current command for the most stable dense terrain-cloud demo on WSL2:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch airos_experiments visual_fast_lio_navigation.launch.py \
  gui:=true \
  rviz:=true \
  world:=realistic_multilevel_ramp \
  map:=src/airos_nav/maps/realistic_multilevel_ramp.yaml \
  route_graph:=src/airos_nav/routes/realistic_multilevel_ramp_route.geojson \
  sensor_source:=emulated \
  terrain_planner:=true \
  pointcloud_spacing:=0.16 \
  dynamic_obstacles:=false \
  colorized_pointcloud:=true
```

Current command for trying Gazebo native GPU LiDAR on WSL2:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch airos_experiments visual_fast_lio_navigation.launch.py \
  gui:=true \
  rviz:=true \
  world:=realistic_multilevel_ramp \
  map:=src/airos_nav/maps/realistic_multilevel_ramp.yaml \
  route_graph:=src/airos_nav/routes/realistic_multilevel_ramp_route.geojson \
  sensor_source:=native \
  gazebo_rendering_mode:=hardware \
  terrain_planner:=true \
  dynamic_obstacles:=false \
  physical_dynamic_obstacles:=false
```

2026-05-10 WSL native LiDAR test result:

- `sensor_source:=native gazebo_rendering_mode:=hardware` crashed Gazebo with
  `Ogre::UnimplementedException` in `GL3PlusTextureGpu::copyTo`.
- `sensor_source:=native gazebo_rendering_mode:=wsl_stable` did not crash in
  headless sim smoke. It published `/scan` at about 9.9 Hz and
  `/livox/lidar_points` as a 720 x 16 `PointCloud2`.
- Therefore the stable WSL path remains `sensor_source:=emulated` for the
  dense FAST-LIO terrain demo, while native LiDAR is selectable for raw-sensor
  experiments under `wsl_stable`.

## 2026-05-14 FAST-LIO2 SLAM-Cloud Planning Status

Implemented:

- `terrain_pct_planner` now supports `terrain_map_source:=slam_cloud`.
- In SLAM-cloud mode it subscribes to FAST-LIO2 `/Laser_map`, samples XYZ
  `PointCloud2` data into a height-aware terrain graph, publishes
  `/terrain_traversability_cloud`, and uses that graph for `/pct_path`.
- SLAM-cloud graph rebuilding is throttled by `slam_rebuild_period_sec`
  instead of rebuilding every second. The default FAST-LIO visual launch now
  samples up to 80k map points per rebuild, which keeps the planner responsive
  enough for RViz goal testing in WSL.
- SDF terrain planning and SLAM-cloud planning use different transition rules:
  SDF surfaces retain the explicit ramp-entry metadata, while SLAM-derived
  surfaces use slope/step limits because the point cloud has no SDF box
  metadata.
- `slam_traversability_graph.py` now separates SLAM-cloud traversability
  extraction and graph construction from `terrain_pct_planner.py`. The planner
  keeps Nav2/direct execution responsibilities, while the new module owns
  `/Laser_map` point sampling, floor/ramp/platform/step labeling, and graph
  connectivity.
- The default SLAM-cloud planner uses `slam_min_cell_points:=2` to avoid
  fragmenting sparse FAST-LIO clouds into tiny disconnected cells.
- The visual FAST-LIO planner launch uses `goal_z_policy:=adaptive` and
  `goal_snap_max_distance:=1.0`, so same-level goals can fall back from an
  unreachable high point to a reachable floor point, while targets outside the
  current SLAM-map coverage are rejected instead of producing a false path.
- If the final goal is not yet reachable in the current SLAM graph,
  `terrain_pct_planner` can publish a FAST-LIO exploration-frontier path inside
  the reachable component and keep the original final goal pending for later
  `/Laser_map` rebuilds. If a rebuild still cannot reach the final goal, the
  planner refreshes the frontier path from the current robot pose.
- SLAM-cloud graph construction now filters low surface clusters under
  multi-layer vertical point stacks, so obstacle bases are not treated as
  traversable ground in narrow point-cloud passages.
- Regression coverage includes synthetic floor -> ramp -> platform routing,
  direct platform-edge drop rejection, sparse same-level routing from the spawn
  area, large-world spawn -> third-level routing on a complete sampled point
  cloud, adaptive goal-height fallback, out-of-coverage goal rejection, and
  reachable-component frontier planning, plus vertical-obstacle cell rejection.

Runtime smoke evidence:

- Launch command:
  `ros2 launch airos_experiments visual_fast_lio_navigation.launch.py gui:=false rviz:=false sensor_source:=emulated terrain_map_source:=slam_cloud terrain_send_nav2_goals:=false dynamic_obstacles:=false physical_dynamic_obstacles:=false log_level:=warn`
- FAST-LIO2 published `/Laser_map`.
- `terrain_pct_planner` rebuilt the FAST-LIO terrain graph with 5716 nodes and
  about 31700 directed edges.
- Publishing `/terrain_goal_pose` at `(2.0, -8.0)` produced `/pct_path` with
  11 poses, starting near `(-0.127, -10.002, -0.400)` and ending near
  `(1.953, -7.995, -0.399)`.
- Publishing `/terrain_goal_pose` at `(6.0, 13.0)` produced no `/pct_path`.
  This is the intended behavior for the current map state because the nearest
  SLAM graph point was outside the 1.0 m goal snap limit and in a disconnected
  component.
- With FAST-LIO exploration frontier enabled, publishing `/terrain_goal_pose` at
  `(6.0, 13.0)` produced a frontier `/pct_path` with 85 poses from
  `(-0.127, -10.002, -0.400)` to `(4.832, 10.958, -0.400)` inside the current
  reachable SLAM component.
- A refresh smoke kept `(6.0, 13.0)` pending and observed multiple subsequent
  `/pct_path` frontier events after FAST-LIO graph rebuilds, including path
  endpoints near `(0.005, 12.096)`, `(4.752, 10.718)`, and `(4.907, 11.286)`.
- With `terrain_send_nav2_goals:=true` and `terrain_execution_mode:=direct`,
  the same frontier goal started direct tracking and produced `/cmd_vel_nav`
  (`linear.x=0.0350`, `angular.z=0.2800` in the smoke probe).

Remaining limitation:

- This is not yet a fully accepted cross-level FAST-LIO2 navigation chain.
  The complete sampled point-cloud graph can route from the spawn area to the
  third level in tests, but the live FAST-LIO local map at startup does not yet
  cover and connect the high-floor goal region. The current runtime can move
  toward a reachable frontier and keep the final goal pending, but it still
  needs an end-to-end acceptance run proving that motion expands the FAST-LIO
  map enough to replan and reach the requested high floor.
