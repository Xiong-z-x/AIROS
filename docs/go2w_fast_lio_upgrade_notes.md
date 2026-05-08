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
  -> pointcloud_emulator publishes /livox/lidar
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

## Remaining Limitations

- No Gazebo GPU LiDAR in the current WSL2/Fortress/OGRE baseline.
- `/livox/lidar` is generated by a ROS-side point-cloud emulator.
- `src/livox_ros_driver2` is message-only compatibility, not a real Livox
  hardware driver.
- No physical moving obstacles in Gazebo; dynamic obstacles remain scan-layer
  emulation.
- No full Unitree low-level legged controller, CHAMP gait, or official Go2W
  hardware interface.
- FAST-LIO2 localization is connected to Nav2 in the external-localization
  visual launch through a bridge that publishes dynamic `map -> odom`. The
  stable visual launch still keeps the static `map -> odom` chain as the
  fallback.
