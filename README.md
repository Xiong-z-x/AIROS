# AIROS 自主导航原型

AIROS 是当前 WSL2 / Ubuntu 22.04 / ROS 2 Humble / Gazebo Fortress 环境下的平面单层自主导航原型。主线目标是用 Go2W 风格导航等效体完成 SLAM 建图、定位、Nav2 静态导航、动态障碍安全测试和实验指标统计。

## 当前能力

当前仓库已经具备：

- Go2W 风格导航等效体：`base_link`、四轮、腿部视觉外形、`lidar_link`、`livox_frame`、`imu_link`、`camera_link`。
- 可选开源 Go2W 视觉网格：`robot_visual_profile:=reference_mesh` 会加载 AFL 3.0 许可参考仓库中的 Go2W body、轮子和 Mid360 mesh，但碰撞和控制仍保持当前稳定等效体。
- Gazebo Fortress 单层实验室世界：24m x 24m 轻量墙体、走廊、门洞和静态障碍。
- 控制链：`/cmd_vel -> velocity_smoother -> collision_monitor -> diff_drive_controller -> /odom -> TF`。
- Gazebo 原生 `/scan`：`gpu_lidar` 直接输出 `LaserScan`，桥接给 RViz、Nav2 和 collision monitor。
- Gazebo 原生 Livox 风格点云：`gpu_lidar` 输出 `PointCloudPacked`，先桥接到 ROS 的 `/livox/lidar_points`，再转换为 Livox `CustomMsg` `/livox/lidar` 给 FAST-LIO2 输入。
- `/cloud_registered`、`/Laser_map`、`/Odometry` 由 FAST-LIO2 输出，用于 RViz 点云地图显示和外部定位桥接。
- FAST-LIO2 仿真链路：集成 `fast_lio`，使用 `/livox/lidar` 和 `/livox/imu` 输出 `/cloud_registered`、`/Laser_map`、`/Odometry`。
- FAST-LIO2 地形规划入口：`terrain_pct_planner` 可在 `terrain_map_source:=slam_cloud` 下直接订阅 `/Laser_map`，从 FAST-LIO2 点云地图抽取 `/terrain_traversability_cloud`，并为已连通的已建图区域发布 `/pct_path`。
- `slam_toolbox` 建图和 localization 配置。
- Nav2：AMCL、SmacPlannerHybrid、MPPI、BT Navigator、behavior server、velocity smoother、collision monitor。
- Route graph：`src/airos_nav/routes/single_floor_lab_route.geojson`。
- 实验 runner：固定任务、初始位姿发布、路径长度、急停、scan 距离、collision 阈值估计、cmd 输出周期统计。
- Clean batch runner：每个 trial 自动干净启动 sim/nav、等待 `map -> base_link` 定位 TF、执行一个 mission、写 JSONL、清理进程组；支持 transient 失败重试。
- Route graph verifier：独立验证 `nav2_route` 能加载 GeoJSON 并返回 `/compute_route` 路线。
- Route waypoint 模式：根据 GeoJSON route graph 生成 waypoint 序列，通过 `navigate_through_poses` 执行。
- 高级场景入口：`advanced_indoor_ramp` 提供更复杂室内结构、坡道区域、匹配 Nav2 seed map、route graph 和任务文件。
- 可选开源 Building 场景视觉资产：`open_source_scene_assets:=true` 会叠加 AFL 3.0 许可参考仓库中的 Building mesh，作为高级展示视觉层。
- Gazebo 物理动态障碍实验入口：高级场景可启用 `physical_dynamic_obstacles:=true`，由 Gazebo 动态模型和 velocity-control 系统运动。
- 规划对比入口：`planner_profile:=baseline|research`，默认保留稳定 baseline，research profile 用于 MPPI/复杂场景对比。
- 结果导出：从 JSONL 生成 `results/single_floor_lab_summary.csv`、Markdown 摘要和 SVG 图表。

当前限制：

- `/scan` 和 `/livox/lidar` 现在默认来自 Gazebo 原生传感器链路；`sensor_source:=emulated` 仍可回退到 ROS 侧模拟器。
- `src/livox_ros_driver2` 当前是 FAST-LIO2 编译所需的 CustomMsg 消息兼容包，不是 Livox 硬件驱动。
- 开源 Building/Go2W mesh 只作为可选视觉资产接入；不导入参考仓库的 Gazebo Garden、CHAMP 或低层足式控制栈。
- FAST-LIO2 在仿真中用于 3D 点云建图和里程计输出；`visual_fast_lio_navigation.launch.py` 通过外部定位桥接把 FAST-LIO 结果接入 Nav2，稳定版 `visual_navigation.launch.py` 仍保留静态 `map -> odom` 控制链。
- 动态障碍有两条链路：`dynamic_obstacles:=true` 是 ROS scan-layer 回退实验；`physical_dynamic_obstacles:=true` 是 Gazebo 物理动态模型实验入口。
- 当前环境使用 Gazebo `ogre` 后端；本机 WSLg 下 `ogre2` 不作为稳定主线。
- Clean runner 已完成 20 次固定任务验收；长 Nav2 会话内循环仍不作为稳定主线。
- Route graph 已完成 ComputeRoute 验证和单任务 route-waypoint 执行 smoke；完整 route-constrained batch 仍未验收。
- 动态障碍已完成 scan-layer smoke；Gazebo 物理动态障碍已作为高级场景实验入口接入。
- Gazebo 物理动态障碍已接入高级场景入口；仍需要单独 runtime smoke，不替代 20 次稳定验收。
- FAST-LIO2 可视化导航 smoke：`visual_fast_lio_navigation.launch.py` 能自动启动 Gazebo server、FAST-LIO2、Nav2 和 RViz；`/livox/imu` 与 `/livox/lidar` 均为单发布者，`/cloud_registered`、`/Laser_map`、`/Odometry` 有数据；`NavigateToPose` 到 `(2.0, -1.5)` 成功，最终 `/odom` 约 `(1.962, -1.413)`。
- FAST-LIO2 `/Laser_map` 到 terrain planner 的链路已验证为同层可规划：无 GUI smoke 中 `/Laser_map` 生成 5716 个地形节点、约 35660 条边，`/terrain_goal_pose` 到 `(2.0, 2.0)` 可发布 46 个 pose 的 `/pct_path`。跨楼层路径仍未完成验收：当前 FAST-LIO2 点云图存在高程点，但目标到高层时仍可能不可达，说明楼层/坡道可通行语义提取还需要继续升级。

最近运行证据：

- 静态 Nav2 smoke 任务成功：`success=true`，耗时 14.811s，路径 3.39m，最小障碍距离 0.58m。
- 结果文件：`log/airos_nav_trials_smoke_success_candidate.jsonl`。
- 摘要文件：`log/airos_nav_trials_smoke_success_summary.json`。
- Clean runner 单任务 smoke 成功：`status=4`，`success=true`，耗时 68.781s，路径 3.566m，最小障碍距离 0.58m。
- 结果文件：`log/airos_nav_trials_clean_smoke_tf_gate.jsonl`。
- 摘要文件：`log/airos_nav_trials_clean_smoke_tf_gate_summary.json`。
- Clean runner 4 任务小批量复测：4/4 成功，`success_rate=1.0`，平均耗时 19.479s，平均路径 3.182m，最小障碍距离 0.405m。
- 小批量文件：`log/airos_nav_trials_clean_batch4_action_wait.jsonl`。
- 小批量摘要：`log/airos_nav_trials_clean_batch4_action_wait_summary.json`。
- Clean runner 20 次固定任务验收：20/20 成功，四个 mission 各 5 次，`success_rate=1.0`，平均耗时 19.195s，平均路径 3.102m，最小障碍距离 0.389m。
- 20 次验收文件：`log/airos_nav_trials_clean_batch20_action_wait.jsonl`。
- 20 次验收摘要：`log/airos_nav_trials_clean_batch20_action_wait_summary.json`。
- Route graph 验证通过：`verify_route_graph` 成功加载 `single_floor_lab_route.geojson`，`/compute_route` 从 node 1 到 node 4 返回 nodes `1 -> 3 -> 4`、edges `103 -> 105`。
- Route 验证日志：`log/route_graph_verifier/route_server.log`、`log/route_graph_verifier/compute_route_goal.txt`。
- Route waypoint clean smoke 成功：`lab_start_to_task_a`，`execution_mode=navigate_through_poses`，耗时 24.402s，路径 7.509m，最小障碍距离 0.52m。
- Route waypoint 文件：`log/airos_nav_trials_clean_route_waypoint_smoke.jsonl`。
- Route waypoint 摘要：`log/airos_nav_trials_clean_route_waypoint_smoke_summary.json`。
- 动态障碍 clean smoke 成功：`lab_start_to_task_a`，`success=true`，耗时 14.963s，路径 3.351m，最小障碍距离 0.504m；日志显示 collision_monitor slowdown/recovery 后 `Goal succeeded`。
- 动态 smoke 文件：`log/airos_nav_trials_clean_dynamic_smoke_retry.jsonl`。
- 动态 smoke 摘要：`log/airos_nav_trials_clean_dynamic_smoke_retry_summary.json`。
- 报告产物：`results/single_floor_lab_summary.csv`、`results/single_floor_lab_summary.md`、`results/figures/`。
- 较早 4 任务 probe 为 3/4，失败项是 Nav2 action server 启动等待不足；已通过 action-server 等待修复并复测通过。
- 失败项复测文件：`log/airos_nav_trials_clean_door_tf_gate_retry.jsonl`。

## 快速运行

环境检查：

```bash
scripts/check_gpu_gazebo_stack.sh
scripts/check_ros_nav_stack.sh
```

构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

清理残留仿真进程：

```bash
./scripts/cleanup_airos_runtime.sh
```

启动仿真：

```bash
ros2 launch airos_sim sim.launch.py \
  gui:=false \
  rviz:=false \
  sensor_source:=native
```

启动稳定可视化导航链路：

```bash
ros2 launch airos_experiments visual_navigation.launch.py \
  gui:=true \
  rviz:=true \
  dynamic_obstacles:=false \
  use_route:=true
```

启动 FAST-LIO2 可视化导航链路：

```bash
ros2 launch airos_experiments visual_fast_lio_navigation.launch.py \
  gui:=true \
  rviz:=true \
  sensor_source:=native \
  use_route:=true
```

启动高级场景 + FAST-LIO2 + 物理动态障碍 + 研究 profile：

```bash
ros2 launch airos_experiments visual_fast_lio_navigation.launch.py \
  gui:=true \
  rviz:=true \
  world:=advanced_indoor_ramp \
  map:=src/airos_nav/maps/advanced_indoor_ramp.yaml \
  route_graph:=src/airos_nav/routes/advanced_indoor_ramp_route.geojson \
  planner_profile:=research \
  physical_dynamic_obstacles:=true \
  open_source_scene_assets:=true \
  robot_visual_profile:=reference_mesh \
  sensor_source:=native \
  use_route:=true
```

单独查看参考 Go2W mesh：

```bash
ros2 launch airos_go2w_description display.launch.py \
  visual_profile:=reference_mesh
```

启动 Nav2：

```bash
ros2 launch airos_nav nav.launch.py rviz:=false use_route:=false
```

运行一次任务 dry-run：

```bash
ros2 run airos_experiments run_nav_trials \
  --mission src/airos_experiments/missions/single_floor_lab_missions.yaml \
  --count 1 \
  --dry-run \
  --ros-args -p use_sim_time:=true
```

连续批量任务需要重置 Gazebo 物理模型，避免只重置 AMCL 位姿：

```bash
ros2 run airos_experiments run_nav_trials \
  --mission src/airos_experiments/missions/single_floor_lab_missions.yaml \
  --count 4 \
  --reset-sim \
  --output log/airos_nav_trials.jsonl \
  --ros-args -p use_sim_time:=true
```

当前更稳的方式是每次干净启动后跑一个指定任务：

```bash
ros2 run airos_experiments run_clean_nav_batch \
  --mission src/airos_experiments/missions/single_floor_lab_missions.yaml \
  --mission-id lab_start_to_task_a \
  --count 1 \
  --attempts 2 \
  --output log/airos_nav_trials_clean_batch.jsonl \
  --log-dir log/clean_batch
```

这个入口会为每个 trial 重启仿真和 Nav2，并等待定位 TF。它比在同一个 Nav2 会话里循环 `run_nav_trials --count N` 更适合当前 WSL2/Fortress 环境。

运行 route waypoint smoke：

```bash
ros2 run airos_experiments run_clean_nav_batch \
  --mission src/airos_experiments/missions/single_floor_lab_missions.yaml \
  --mission-id lab_start_to_task_a \
  --count 1 \
  --use-route-waypoints \
  --route-graph src/airos_nav/routes/single_floor_lab_route.geojson \
  --attempts 2 \
  --sim-startup-sec 18 \
  --nav-startup-sec 18 \
  --output log/airos_nav_trials_clean_route_waypoint_smoke.jsonl \
  --log-dir log/clean_route_waypoint_smoke
```

运行动态障碍仿真入口：

```bash
ros2 launch airos_experiments dynamic_obstacles.launch.py gui:=false rviz:=false
```

运行动态障碍 clean smoke：

```bash
ros2 run airos_experiments run_clean_nav_batch \
  --mission src/airos_experiments/missions/single_floor_lab_missions.yaml \
  --mission-id lab_start_to_task_a \
  --count 1 \
  --dynamic-obstacles \
  --attempts 2 \
  --sim-startup-sec 18 \
  --nav-startup-sec 18 \
  --output log/airos_nav_trials_clean_dynamic_smoke.jsonl \
  --log-dir log/clean_dynamic_smoke
```

运行高级场景 route-constrained clean smoke：

```bash
ros2 run airos_experiments run_clean_nav_batch \
  --mission src/airos_experiments/missions/advanced_indoor_ramp_missions.yaml \
  --mission-id ramp_entry_to_upper_observation \
  --world advanced_indoor_ramp \
  --map src/airos_nav/maps/advanced_indoor_ramp.yaml \
  --route-graph src/airos_nav/routes/advanced_indoor_ramp_route.geojson \
  --planner-profile baseline \
  --open-source-scene-assets \
  --robot-visual-profile reference_mesh \
  --use-route-waypoints \
  --attempts 2 \
  --sim-startup-sec 18 \
  --nav-startup-sec 18 \
  --output log/airos_nav_trials_advanced_route_smoke.jsonl \
  --log-dir log/advanced_route_smoke
```

验证 route graph：

```bash
ros2 run airos_experiments verify_route_graph \
  --graph src/airos_nav/routes/single_floor_lab_route.geojson \
  --start-id 1 \
  --goal-id 4
```

生成任务统计摘要：

```bash
ros2 run airos_experiments summarize_trials \
  --input log/airos_nav_trials.jsonl \
  --output log/airos_nav_trials_summary.json
```

导出报告产物：

```bash
ros2 run airos_experiments summarize_trials \
  --input log/airos_nav_trials_clean_batch20_action_wait.jsonl \
  --output log/airos_nav_trials_clean_batch20_action_wait_summary.json \
  --csv-output results/single_floor_lab_summary.csv \
  --markdown-output results/single_floor_lab_summary.md \
  --figures-dir results/figures
```

## 地图文件

- `src/airos_nav/maps/single_floor_lab.yaml`：默认 Nav2 seed map，从 SDF 静态障碍生成，覆盖 24m x 24m。
- `src/airos_nav/maps/single_floor_lab_slam.yaml`：SLAM smoke map，来自 `slam_toolbox` 保存结果，仅用于建图/定位链路证明。
- `src/airos_nav/maps/single_floor_lab_slam.posegraph` 和 `.data`：SLAM localization 输入。

## 关键文档

- `docs/environment_baseline.md`：当前 WSL2 / GPU / Gazebo / ROS 基线。
- `docs/AIROS_phased_execution_plan.md`：阶段目标、验收标准和当前进展。
- `docs/AIROS_autonomous_navigation_technical_route.md`：技术路线、边界和后续实验设计。
- `docs/go2w_fast_lio_upgrade_notes.md`：Go2W、点云地图、FAST-LIO2、RViz/Gazebo 修复和验证证据。
- `docs/advanced_planning_research_profile.md`：高级场景、MPPI 研究配置、PCT/RL 接入边界。
- `docs/report_outline.md`：课程报告提纲。
- `docs/ppt_outline.md`：PPT 提纲。
- `docs/video_script.md`：演示视频脚本。
