# AIROS 平面单层自主导航阶段执行计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:executing-plans` to execute this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 构建一个可复现的 ROS 2 Humble + Gazebo Fortress + Go2W 风格导航等效体 + slam_toolbox + Nav2 自主导航演示系统。  

**Architecture:** 先冻结环境与 GPU/OpenGL 渲染事实，再建立 Go2W 导航等效体和 Gazebo/ROS 控制闭环，随后接入 slam_toolbox 建图、Nav2 route graph、Smac Hybrid-A*、MPPI、collision_monitor 和实验统计。  

**Tech Stack:** Ubuntu 22.04 WSL2, ROS 2 Humble, Ignition Gazebo Fortress 6.16.0, ros_gz, gz_ros2_control, slam_toolbox, Nav2 1.1.20, RViz2, Python 3.10, CMake/Colcon.

**Current Status (2026-05-07):**

```text
Task 1: complete. GPU/OpenGL/Gazebo GUI gate and ROS/Nav2 package gate pass.
Task 2-5: complete for the current WSL2/Fortress baseline. Six ROS packages build, Go2W navigation equivalent displays, Gazebo spawn/control/IMU/odom/joint chain runs, and /cmd_vel changes odom.
Task 6: implemented. slam_toolbox mapping/localization configs and launch files exist; smoke map and posegraph are stored as single_floor_lab_slam.*.
Task 7: implemented and launch-verified. map_server, amcl, controller_server, planner_server, and bt_navigator reached active state; /navigate_to_pose and related Nav2 actions are available.
Task 8: implemented as a route graph artifact, route_server config, and route-waypoint execution mode. `verify_route_graph` validated nav2_route graph loading and `/compute_route` from node 1 to node 4 on 2026-05-07. A clean route-waypoint smoke also succeeded with `execution_mode=navigate_through_poses`; full route-constrained batch validation remains a future hardening item.
Task 9: implemented as a ROS-side scan-layer dynamic obstacle emulator with RViz markers and collision_monitor wiring. A dynamic clean smoke succeeded for `lab_start_to_task_a`; logs show `dynamic=True`, collision_monitor slowdown/recovery, and `Goal succeeded`. It is not a Gazebo physical moving-cylinder model.
Task 10: implemented and clean-runner validated. Metrics include success, elapsed time, path length, emergency stop count, scan-threshold collision estimate, minimum scan distance, and cmd output period. The clean process-per-trial 20-run acceptance passed on 2026-05-07, and report artifacts are exported under `results/`.
Task 11: implemented as course-ready outlines: `docs/report_outline.md`, `docs/ppt_outline.md`, and `docs/video_script.md`.
Native sensor note: `sim.launch.py` defaults to `sensor_source:=native`. `/scan` is bridged from Gazebo `gpu_lidar` as `LaserScan`, and Gazebo `gpu_lidar` `PointCloudPacked` output is bridged from `/livox/lidar/points` into ROS `/livox/lidar_points`, then converted to Livox `CustomMsg` on `/livox/lidar` for FAST-LIO2. The ROS-side emulators remain available as `sensor_source:=emulated` fallback.
FAST-LIO2 visual upgrade: implemented on 2026-05-09. `visual_fast_lio_navigation.launch.py` launches Gazebo, FAST-LIO2, Nav2, and RViz. Native `/scan`, `/livox/lidar`, `/cloud_registered`, `/Laser_map`, and `/Odometry` are live. RViz RobotModel now uses Transient Local robot_description QoS, all 14 robot links are reachable from map, and RViz keeps the Nav2 `/map` display entry plus the point-cloud map display.
Runtime smoke: one clean static Nav2 mission succeeded on 2026-05-07. The runner recorded status=4, success=true, elapsed=14.811s, path_length=3.39m, minimum_obstacle_distance=0.58m, mean_cmd_period=0.049s, max_cmd_period=0.0696s. Output files: log/airos_nav_trials_smoke_success_candidate.jsonl and log/airos_nav_trials_smoke_success_summary.json.
Clean-runner smoke: run_clean_nav_batch succeeded for lab_start_to_task_a on 2026-05-07. It starts sim/nav per trial, waits for map -> base_link TF, runs one mission, and cleans the launch process groups. The runner recorded status=4, success=true, elapsed=68.781s, path_length=3.566m, minimum_obstacle_distance=0.58m, mean_cmd_period=0.0493s, max_cmd_period=0.1011s. Output files: log/airos_nav_trials_clean_smoke_tf_gate.jsonl and log/airos_nav_trials_clean_smoke_tf_gate_summary.json.
Clean batch probe: run_clean_nav_batch first completed a 4-task probe on 2026-05-07 with 3/4 success. lab_door_passage failed because navigate_to_pose was not ready inside the previous fixed wait window. After adding a configurable action-server wait and structured action_server_unavailable result, lab_door_passage was rerun alone and succeeded with status=4, elapsed=18.757s, path_length=3.517m, minimum_obstacle_distance=0.544m.
Clean batch rerun: after the action-server wait fix, run_clean_nav_batch completed the 4-task set with 4/4 success on 2026-05-07. Summary: success_rate=1.0, mean_elapsed=19.479s, mean_path_length=3.182m, total_emergency_stop_count=0, collision_count=0, minimum_obstacle_distance=0.405m, mean_cmd_period=0.0537s, max_cmd_period=4.4887s. Evidence files: log/airos_nav_trials_clean_batch4_action_wait.jsonl and log/airos_nav_trials_clean_batch4_action_wait_summary.json.
20-run acceptance: run_clean_nav_batch completed 20 fixed trials on 2026-05-07, cycling the four missions five times each. Summary: trial_count=20, success_count=20, success_rate=1.0, mean_elapsed=19.195s, mean_path_length=3.102m, total_emergency_stop_count=0, collision_count=0, minimum_obstacle_distance=0.389m, mean_cmd_period=0.0506s, max_cmd_period=4.4604s. Evidence files: log/airos_nav_trials_clean_batch20_action_wait.jsonl and log/airos_nav_trials_clean_batch20_action_wait_summary.json. Residual warning: some return_point runs still recover from transient "Starting point in lethal space" planner warnings before succeeding; keep this as a map/footprint hardening item, not a failed acceptance.
Route graph verification: `ros2 run airos_experiments verify_route_graph --graph src/airos_nav/routes/single_floor_lab_route.geojson --start-id 1 --goal-id 4` passed on 2026-05-07. Evidence files: log/route_graph_verifier/route_server.log and log/route_graph_verifier/compute_route_goal.txt. The returned route used nodes 1 -> 3 -> 4 and edges 103 -> 105.
Route-waypoint smoke: `run_clean_nav_batch --use-route-waypoints` succeeded for lab_start_to_task_a on 2026-05-07. Summary: success_rate=1.0, execution_mode=navigate_through_poses, elapsed=24.402s, path_length=7.509m, collision_count=0, minimum_obstacle_distance=0.52m. Evidence files: log/airos_nav_trials_clean_route_waypoint_smoke.jsonl and log/airos_nav_trials_clean_route_waypoint_smoke_summary.json.
Dynamic smoke: `run_clean_nav_batch --dynamic-obstacles --attempts 2` succeeded for lab_start_to_task_a on 2026-05-07. Summary: success_rate=1.0, elapsed=14.963s, path_length=3.351m, collision_count=0, minimum_obstacle_distance=0.504m. Evidence files: log/airos_nav_trials_clean_dynamic_smoke_retry.jsonl and log/airos_nav_trials_clean_dynamic_smoke_retry_summary.json. Nav log shows collision_monitor slowdown/recovery and final `Goal succeeded`.
Report artifacts: `results/single_floor_lab_summary.csv`, `results/single_floor_lab_summary.md`, `results/figures/mean_elapsed_sec.svg`, and `results/figures/mean_path_length_m.svg` were generated from the 20-run JSONL.
Batch-trial note: use run_nav_trials --reset-sim for count > 1 so Ignition Gazebo resets the physical model pose before each mission. Publishing /initialpose alone only resets AMCL and is not a valid physical reset. For reliable current evidence, prefer run_clean_nav_batch because it avoids long-session Nav2 state carryover.
Reset probe: Ignition service /world/single_floor_lab/set_pose is present and returned data: true for go2w_nav_eq on 2026-05-07.
Batch-trial limitation: repeated missions in one long Nav2 session are not yet stable. Even with --reset-sim and costmap clearing, planner/controller recovery state can leave the robot in lethal-space or no-progress loops in the narrow seed map. The accepted batch evidence is therefore the clean process-per-trial runner, not the long-session loop.
```

---

## File Structure

最终仓库结构按下列责任边界创建：

```text
docs/
  AIROS_autonomous_navigation_technical_route.md
  AIROS_phased_execution_plan.md
  environment_baseline.md

scripts/
  check_gpu_gazebo_stack.sh
  check_ros_nav_stack.sh

src/
  airos_go2w_description/
    urdf/go2w_nav_eq.urdf.xacro
    meshes/
    rviz/model.rviz
    launch/display.launch.py

  airos_sim/
    worlds/single_floor_lab.sdf
    config/ros_gz_bridge.yaml
    launch/sim.launch.py

  airos_control/
    config/go2w_controllers.yaml
    launch/control.launch.py

  airos_slam/
    config/slam_toolbox_mapping.yaml
    config/slam_toolbox_localization.yaml
    launch/mapping.launch.py
    launch/localization.launch.py

  airos_nav/
    config/nav2_params.yaml
    maps/
    routes/
    rviz/nav.rviz
    launch/nav.launch.py

  airos_experiments/
    missions/single_floor_lab_missions.yaml
    scripts/run_nav_trials.py
    scripts/summarize_trials.py
    launch/dynamic_obstacles.launch.py
```

---

## Task 1: Environment Baseline And GPU Gate

**Files:**
- Create: `docs/environment_baseline.md`
- Create: `scripts/check_gpu_gazebo_stack.sh`
- Create: `scripts/check_ros_nav_stack.sh`

- [ ] **Step 1: Write GPU/Gazebo check script**

Create `scripts/check_gpu_gazebo_stack.sh` with executable Bash logic that prints:

```bash
#!/usr/bin/env bash
set -euo pipefail

echo "[GPU] nvidia-smi"
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader

echo "[OpenGL] glxinfo -B"
glxinfo -B | sed -n '1,40p'

echo "[Gazebo] Ignition Gazebo"
ign gazebo --versions
```

- [ ] **Step 2: Write ROS/Nav2 package check script**

Create `scripts/check_ros_nav_stack.sh` with:

```bash
#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash

required=(
  slam_toolbox
  nav2_route
  nav2_mppi_controller
  nav2_smac_planner
  nav2_collision_monitor
  nav2_velocity_smoother
  ros_gz_bridge
  ros_gz_sim
)

for pkg in "${required[@]}"; do
  ros2 pkg prefix "$pkg" >/dev/null
  echo "[OK] $pkg"
done
```

- [ ] **Step 3: Run checks**

Run:

```bash
chmod +x scripts/check_gpu_gazebo_stack.sh scripts/check_ros_nav_stack.sh
scripts/check_gpu_gazebo_stack.sh
scripts/check_ros_nav_stack.sh
```

Expected:

```text
nvidia-smi sees RTX 3050
glxinfo reports direct rendering and Accelerated: yes
ign gazebo reports 6.16.0
Gazebo GUI render probe stays alive for 8s with ogre
all required ROS packages print [OK]
```

- [ ] **Step 4: Record baseline**

Write `docs/environment_baseline.md` with the command output summary and the final decision:

```text
Use ROS 2 Humble + Ignition Gazebo Fortress.
Use ign gazebo, not gz sim.
Use WSLg OpenGL/D3D12 GPU rendering.
Use forced Gazebo GUI render args: --render-engine ogre --render-engine-gui ogre.
Do not migrate to Harmonic for this project stage.
```

---

## Task 2: Workspace And Package Scaffold

**Files:**
- Create: `src/airos_go2w_description/`
- Create: `src/airos_sim/`
- Create: `src/airos_control/`
- Create: `src/airos_slam/`
- Create: `src/airos_nav/`
- Create: `src/airos_experiments/`

- [ ] **Step 1: Create ROS packages**

Run:

```bash
mkdir -p src
cd src
ros2 pkg create airos_go2w_description --build-type ament_cmake
ros2 pkg create airos_sim --build-type ament_cmake
ros2 pkg create airos_control --build-type ament_cmake
ros2 pkg create airos_slam --build-type ament_cmake
ros2 pkg create airos_nav --build-type ament_cmake
ros2 pkg create airos_experiments --build-type ament_python
```

- [ ] **Step 2: Add resource folders**

Run:

```bash
mkdir -p \
  airos_go2w_description/urdf airos_go2w_description/meshes airos_go2w_description/rviz airos_go2w_description/launch \
  airos_sim/worlds airos_sim/config airos_sim/launch \
  airos_control/config airos_control/launch \
  airos_slam/config airos_slam/launch \
  airos_nav/config airos_nav/maps airos_nav/routes airos_nav/rviz airos_nav/launch \
  airos_experiments/missions airos_experiments/scripts airos_experiments/launch
```

- [ ] **Step 3: Build empty workspace**

Run:

```bash
cd /home/xiongzx/AIROS
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Expected: all six packages build successfully.

---

## Task 3: Go2W Navigation Equivalent Model

**Files:**
- Create: `src/airos_go2w_description/urdf/go2w_nav_eq.urdf.xacro`
- Create: `src/airos_go2w_description/launch/display.launch.py`
- Create: `src/airos_go2w_description/rviz/model.rviz`

- [ ] **Step 1: Implement Xacro model**

Model content must include:

```text
base_link box collision
four continuous wheel joints
fixed lidar_link
fixed imu_link
fixed camera_link
inertial mass near 18 kg
simple box/cylinder collision geometry
```

- [ ] **Step 2: Implement display launch**

Launch must start:

```text
xacro processing
robot_state_publisher
joint_state_publisher_gui
rviz2 with model.rviz
```

- [ ] **Step 3: Validate model**

Run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch airos_go2w_description display.launch.py
```

Expected:

```text
RViz2 shows base_link, lidar_link, imu_link, camera_link, and four wheel links.
No missing mesh or TF errors.
```

---

## Task 4: Gazebo World, Spawn, And Bridge

**Files:**
- Create: `src/airos_sim/worlds/single_floor_lab.sdf`
- Create: `src/airos_sim/config/ros_gz_bridge.yaml`
- Create: `src/airos_sim/launch/sim.launch.py`

- [ ] **Step 1: Create single-floor world**

World must contain:

```text
ground plane
office or lab walls
one L-shaped corridor
one narrow door-like passage
three static obstacles
low visual complexity
no high-poly meshes
```

- [ ] **Step 2: Create bridge config**

Bridge topics:

```text
/clock
/scan
/imu
/odom
/joint_states
/cmd_vel
```

- [ ] **Step 3: Create simulation launch**

Launch must start:

```text
ign gazebo --render-engine ogre --render-engine-gui ogre single_floor_lab.sdf
robot spawn
ros_gz_bridge
robot_state_publisher
```

- [ ] **Step 4: Validate Gazebo + RViz coexistence**

Run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch airos_sim sim.launch.py gui:=true
```

Expected:

```text
Gazebo GUI opens with GPU/OpenGL renderer using --render-engine ogre --render-engine-gui ogre.
Robot appears in the world.
/clock, /scan, /imu, /joint_states are visible in ros2 topic list.
```

---

## Task 5: Control Chain

**Files:**
- Create: `src/airos_control/config/go2w_controllers.yaml`
- Create: `src/airos_control/launch/control.launch.py`
- Modify: `src/airos_go2w_description/urdf/go2w_nav_eq.urdf.xacro`
- Modify: `src/airos_sim/launch/sim.launch.py`

- [ ] **Step 1: Add ros2_control tags**

The Xacro must expose four wheel joints with velocity command interfaces and position/velocity state interfaces.

- [ ] **Step 2: Add controller config**

Use grouped left/right wheel control:

```text
controller_manager
joint_state_broadcaster
diff_drive_controller
left_wheel_names: [fl_wheel_joint, rl_wheel_joint]
right_wheel_names: [fr_wheel_joint, rr_wheel_joint]
cmd_vel_timeout: 0.5
publish_rate: 50.0
```

- [ ] **Step 3: Validate low-speed motion**

Run:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}, angular: {z: 0.0}}" --rate 5
```

Expected:

```text
Robot moves forward in Gazebo.
/odom changes continuously.
RViz2 TF follows motion.
```

---

## Task 6: SLAM Mapping

**Files:**
- Create: `src/airos_slam/config/slam_toolbox_mapping.yaml`
- Create: `src/airos_slam/config/slam_toolbox_localization.yaml`
- Create: `src/airos_slam/launch/mapping.launch.py`
- Create: `src/airos_slam/launch/localization.launch.py`

- [ ] **Step 1: Add mapping config**

Set:

```text
mode: mapping
map_frame: map
odom_frame: odom
base_frame: base_link
scan_topic: /scan
resolution: 0.05
use_scan_matching: true
do_loop_closing: true
```

- [ ] **Step 2: Launch mapping**

Run:

```bash
ros2 launch airos_slam mapping.launch.py
```

Expected:

```text
/map publishes occupancy grid.
map -> odom transform exists.
RViz2 shows growing map.
```

- [ ] **Step 3: Save map and pose graph**

Save into:

```text
src/airos_nav/maps/single_floor_lab_slam.yaml
src/airos_nav/maps/single_floor_lab_slam.pgm
src/airos_nav/maps/single_floor_lab_slam.posegraph
src/airos_nav/maps/single_floor_lab_slam.data
```

The default Nav2 map remains `single_floor_lab.yaml` / `.pgm`, generated from the SDF as a full 12m x 12m seed map. The SLAM smoke map is intentionally separated as `single_floor_lab_slam.*` because its observed coverage can be smaller than the full navigation map during short WSL smoke runs.

- [ ] **Step 4: Validate localization mode**

Run localization launch with saved pose graph.

Expected:

```text
Robot localizes on saved map after restart.
map -> odom remains stable.
```

---

## Task 7: Nav2 Static Navigation

**Files:**
- Create: `src/airos_nav/config/nav2_params.yaml`
- Create: `src/airos_nav/launch/nav.launch.py`
- Create: `src/airos_nav/rviz/nav.rviz`

- [ ] **Step 1: Configure Nav2 servers**

Enable:

```text
map_server
slam_toolbox localization output
planner_server with SmacPlannerHybrid
controller_server with MPPIController
bt_navigator
behavior_server
lifecycle_manager
velocity_smoother
collision_monitor
```

- [ ] **Step 2: Configure costmaps**

Use:

```text
static_layer
obstacle_layer from /scan
inflation_layer
robot footprint matching Go2W equivalent body
```

- [ ] **Step 3: Run static navigation**

Run:

```bash
ros2 launch airos_nav nav.launch.py map:=src/airos_nav/maps/single_floor_lab.yaml
```

Expected:

```text
RViz2 can send goal.
Global path appears.
Local trajectory appears.
Robot reaches target without collision in static world.
```

---

## Task 8: Route Tool And Route Graph

**Files:**
- Create: `src/airos_nav/routes/single_floor_lab_route.geojson`
- Modify: `src/airos_nav/launch/nav.launch.py`
- Modify: `src/airos_nav/config/nav2_params.yaml`

- [ ] **Step 1: Start route tool**

Run:

```bash
ros2 launch nav2_rviz_plugins route_tool.launch.py yaml_filename:=src/airos_nav/maps/single_floor_lab.yaml
```

- [ ] **Step 2: Create route graph**

Create nodes for:

```text
start zone
corridor turn
door passage
task point A
task point B
return point
```

Save as:

```text
src/airos_nav/routes/single_floor_lab_route.geojson
```

- [ ] **Step 3: Enable route server**

Configure `nav2_route` to load the route graph and route goals through Nav2.

- [ ] **Step 4: Validate route navigation**

Expected:

```text
RViz2 shows route graph.
Route request produces route-constrained path.
Robot follows route graph to target.
```

---

## Task 9: Dynamic Obstacles And Safety

**Files:**
- Create: `src/airos_experiments/launch/dynamic_obstacles.launch.py`
- Create: `src/airos_experiments/missions/single_floor_lab_missions.yaml`
- Modify: `src/airos_nav/config/nav2_params.yaml`

- [ ] **Step 1: Add dynamic obstacle source**

Use the current WSL/Fortress-safe ROS-side scan emulator to inject simple moving circular obstacles:

```text
crossing pedestrian cylinder
slow moving cart
temporary blocker near door
```

Current implementation publishes `/dynamic_obstacles/markers` for RViz when
explicitly enabled. It is off by default because the module still needs a
Gazebo-visible physical-obstacle pass. It does not spawn Gazebo physical moving
bodies; that is deferred until the renderer/sensor path is migrated or
stabilized.

- [ ] **Step 2: Configure collision_monitor**

Use at least:

```text
stop zone
slowdown zone
/scan source
base frame: base_link
cmd_vel input/output chain connected after velocity_smoother
```

- [ ] **Step 3: Validate emergency behavior**

Expected:

```text
Obstacle enters stop zone.
collision_monitor suppresses unsafe cmd_vel.
Robot slows or stops before contact.
```

---

## Task 10: Experiments And Metrics

**Files:**
- Create: `src/airos_experiments/scripts/run_nav_trials.py`
- Create: `src/airos_experiments/scripts/summarize_trials.py`
- Create: `src/airos_experiments/missions/single_floor_lab_missions.yaml`

- [ ] **Step 1: Define mission set**

Mission YAML must include:

```text
mission_id
start_pose
goal_pose
route_id
dynamic_obstacle_seed
speed_limit
expected_timeout_sec
```

- [ ] **Step 2: Run 20 fixed trials**

Run:

```bash
ros2 run airos_experiments run_nav_trials --mission src/airos_experiments/missions/single_floor_lab_missions.yaml --count 20
```

- [ ] **Step 3: Summarize metrics**

Generate:

```text
success rate
collision count
minimum obstacle distance
task time
path length
emergency stop count
cmd output period as the controller-cycle proxy
CPU/memory/GPU notes
```

Current implementation records the first six items plus `mean_cmd_period_sec` and `max_cmd_period_sec`. CPU/memory/GPU notes are still documented manually from environment checks rather than sampled by the runner.

- [ ] **Step 4: Export report artifacts**

Write:

```text
results/single_floor_lab_summary.csv
results/single_floor_lab_summary.md
results/figures/
```

---

## Task 11: Demo And Course Deliverables

**Files:**
- Create: `docs/report_outline.md`
- Create: `docs/ppt_outline.md`
- Create: `docs/video_script.md`

- [ ] **Step 1: Write report outline**

Use the fixed structure from `docs/AIROS_autonomous_navigation_technical_route.md`.

- [ ] **Step 2: Write PPT outline**

Include:

```text
system architecture
GPU/OpenGL evidence
Go2W equivalent model
SLAM mapping
route graph
Nav2 path planning
MPPI dynamic avoidance
ablation results
```

- [ ] **Step 3: Write video script**

Follow:

```text
Gazebo + RViz2 startup
SLAM mapping
map save/load
route graph annotation
navigation task
dynamic obstacle avoidance
metrics summary
```

---

## Execution Rule

Implementation must proceed in task order. Do not tune Nav2 before Gazebo/TF/sensors are verified. Do not add RGB-D, 3D LiDAR, FAST-LIO2, Cartographer, Harmonic, or full legged gait before this flat single-floor mainline passes the Task 10 trial gate.
