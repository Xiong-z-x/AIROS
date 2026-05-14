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
- In SLAM-cloud mode it subscribes to aligned FAST-LIO2 `/Laser_map_world`, samples XYZ
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
- For high-floor goals, the frontier selector now looks for mapped high
  structures already present in the FAST-LIO graph and uses the nearest high
  structure to bias the next reachable frontier. This prevents the robot from
  blindly moving along the final XY direction when the live map contains a
  lateral ramp or upper-deck hint.
- FAST-LIO exploration-frontier paths are bounded by
  `frontier_min_path_distance:=0.25` and `frontier_max_path_distance:=10.0` in the
  default visual launch. This prevents the controller from chasing a far
  frontier in one command before the live SLAM map has expanded.
- Frontier planning fuses live obstacle points through
  `frontier_obstacle_scan_topic:=/slam_scan`,
  `frontier_obstacle_clearance:=0.45`, and
  `frontier_obstacle_range_max:=3.0`. This is intentionally local and
  temporary: `/Laser_map_world` remains the accumulated SLAM planning map, while
  `/slam_scan` is projected from aligned FAST-LIO2 current-frame points on
  `/cloud_registered_world`.
- `/slam_scan` estimates the local surface height before applying the vertical
  obstacle band. This keeps ramp/floor surfaces out of the 2D safety scan and
  avoids using historical accumulated map points as real-time StopZone inputs.
- SLAM-cloud graph construction now filters low surface clusters under
  multi-layer or vertically thick point stacks, so obstacle bases are not
  treated as traversable ground in narrow point-cloud passages.
- Regression coverage includes synthetic floor -> ramp -> platform routing,
  direct platform-edge drop rejection, sparse same-level routing from the spawn
  area, large-world spawn -> third-level routing on a complete sampled point
  cloud, adaptive goal-height fallback, out-of-coverage goal rejection, and
  reachable-component frontier planning, high-structure-biased frontier
  selection, plus vertical-obstacle cell rejection.

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
- A later 75 s direct-control probe proved the partial closed loop is real but
  not yet complete: FAST-LIO graph nodes grew from about 5659 to about 7300,
  `/pct_path` refreshed 22 times, `/cmd_vel_nav` produced 847 non-zero command
  samples, and odometry moved about 1.20 m. The run then exposed the next
  bottleneck: long frontier paths near 80 nodes repeatedly led into the
  `/scan` collision monitor's SlowZone/StopZone. The planner now uses bounded
  rolling frontier paths to address that specific failure mode.
- A later root-cause probe showed the lower-floor wall was visible in the raw
  `/livox/lidar_points` current frame but not retained as high wall points in
  `/cloud_registered` or `/Laser_map` near the robot. The scan-fused frontier
  smoke then produced 12 short `/pct_path` refreshes, 416 non-zero
  `/cmd_vel_nav` samples, about 1.124 m odometry motion, and a minimum scan
  clearance of about 0.478 m instead of the previous 0.151 m stop-zone case.
- A 2026-05-14 high-floor frontier probe improved the live FAST-LIO chain but
  still did not complete it. In a 300 s run before the latest scoring fix,
  `/Laser_map_world` grew from 227790 to 657345 points, the terrain cloud grew
  from 1080 to 4622 points, `/cmd_vel_nav` produced 3512 non-zero samples, and
  odometry travelled about 25.5 m. The best frontier reached `(4.27, 8.23)`,
  about 5.07 m from the requested `(6.0, 13.0)` goal, but max path height was
  only about 0.085 m and collision monitoring stopped progress before the high
  floor became connected.
- The same probe exposed two planner defects that were fixed afterward:
  lateral high bumps could outrank a better final-goal corridor, and the
  frontier stall monitor only checked total progress from the original
  frontier start. Regression tests now cover both cases. A short 180 s rerun
  after fixing remote high-attractor gating produced frontiers
  `(-1.30,-7.57) -> (0.89,-0.91) -> (4.57,2.76)`, grew
  `/Laser_map_world` from 186364 to 417805 points, grew terrain cloud from
  1074 to 3529 points, produced 2228 non-zero `/cmd_vel_nav` samples, and
  moved odometry about 20.2 m. This confirms improved goal-directed FAST-LIO
  exploration, not final cross-level acceptance.
- A later regression-gated run kept frontier choices from moving far away from
  the final goal after a better frontier had already been found. The first
  240 s window produced monotonic frontier progress
  `(-1.29,-7.57) -> (0.56,-0.96) -> (3.34,2.41) -> (3.03,6.34) -> (5.08,9.92)`,
  grew `/Laser_map_world` from 203615 to 523897 points, grew terrain cloud from
  1071 to 4761 points, produced 2854 non-zero `/cmd_vel_nav` samples, and moved
  odometry about 25.4 m. The continued run then published a high-floor
  `/pct_path` ending near `(5.78,12.93,2.18)`, only about 0.23 m in XY from the
  requested `(6.0,13.0)` target. This proves the live FAST-LIO map can become
  connected enough for a cross-level high-floor path, but the follow-up 360 s
  monitor saw no new `/pct_path`, only 0.16 m additional odometry motion, and
  minimum odometry-to-goal distance about 5.29 m. Therefore the path-generation
  side improved substantially, while final motion-to-goal acceptance still
  failed.
- A subsequent 600 s runtime with local-surface `/slam_scan` projection showed
  that the safety scan fix reduced immediate false stops but did not complete
  high-floor navigation. `/Laser_map_world` grew from 93095 to 615983 points,
  terrain cloud grew from 1078 to 2947 points, `/cmd_vel_nav`,
  `/cmd_vel_smoothed`, and base controller commands all had non-zero samples,
  and odometry travelled about 19.0 m. However the best `/pct_path` endpoint was
  still about 8.11 m from `(6.0,13.0)`, max path height was only about 0.16 m,
  and odometry ended near `(-0.36,0.22)`. Logs showed repeated stalled frontier
  releases around the same low-floor region.
- The frontier progress gate now commits "best progress" only after direct
  tracking actually reaches a frontier. Planned-but-stalled frontiers no longer
  permanently tighten the regression gate, preventing one false good-looking
  path from trapping later exploration around the same low-floor pocket.
- Direct tracking stall detection now measures progress toward the active
  direct waypoint instead of raw displacement or whole-path endpoint distance.
  Final direct paths can be released and replanned if they stall after a pending
  FAST-LIO goal becomes reachable. Direct target advancement also skips passed
  waypoints by selecting a closer remaining waypoint, reducing repeated pursuit
  of small local points after the robot drifts off the planned line.
- A 360 s emulated FAST-LIO run after these direct-tracking fixes generated
  high-floor `/pct_path` endpoints close to the requested `(6.0,13.0)` target:
  best endpoint `(5.73,13.22,2.05)` with XY error about `0.35 m`, and terrain
  cloud growth from about 2466 to 8650 points. The base command chain remained
  active and odometry travelled about 17.2 m, but the robot still did not reach
  the high-floor goal; odometry ended near `(3.85,1.75)` with minimum
  odometry-to-goal distance about `11.45 m`.
- Applying local `/scan` obstacle blocking directly to final SLAM-cloud path
  planning was tested and deliberately not enabled by default: it prevented the
  high-floor final path from being produced in a 300 s run, leaving only low
  frontier paths. The final planner keeps the optional blocked-node API for
  future targeted use, while the default final path remains driven by the
  FAST-LIO traversability graph.
- 2026-05-14 follow-up root-cause result: the failure is no longer "FAST-LIO2
  cannot build high-floor points." A live snapshot contained high points around
  the third-floor goal, but the raw SLAM traversability graph split ramp/stair
  and deck samples into isolated components. The graph now adds constrained
  sparse slope bridges for sparse FAST-LIO ramp/stair samples, while keeping
  obstacle-base and wall-crossing regression tests. The planner also tries
  multiple goal candidates within the snap radius, so a disconnected high
  island does not block a reachable high-floor candidate nearby.
- With those changes, an offline rebuild from the live `/Laser_map_world`
  snapshot produced a high-floor plan (`path_nodes=21`, endpoint z about
  `2.14`, max path z about `2.25`, endpoint XY error about `1.72 m` with
  `goal_snap_max_distance:=2.0`). A restarted runtime then logged repeated
  `pending final goal became reachable` events and published high `/pct_path`
  instances with max z about `2.35`. Therefore the SLAM-map to PCT-path part
  is now demonstrated in live FAST-LIO data.
- The remaining blocker is physical execution of the high-floor path. In the
  same runtime, FAST-LIO aligned odometry approached the high-goal XY area
  (`min_goal_xy` about `0.28 m`), but `fast_lio_odom_world.z` stayed below
  about `0.65 m`, and Gazebo ground-truth pose stayed near `z=-0.005`. This
  proves the robot is not climbing to the high deck even when a high `/pct_path`
  exists. Next work should focus on terrain-aware execution of 3D waypoints
  and ramp/stair approach constraints, not on claiming FAST-LIO/PCT completion.

Remaining limitation:

- This is not yet a fully accepted cross-level FAST-LIO2 navigation chain.
  FAST-LIO2 SLAM map generation and high-floor PCT path generation have runtime
  evidence, but end-to-end motion acceptance has not passed because the robot
  still fails to physically climb from the low floor to the high deck.
