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
