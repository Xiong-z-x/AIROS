# AIROS 自主导航实践完整技术路线

状态：已自我审批  
日期：2026-05-07  
主线目标：在 Ubuntu 22.04 / ROS 2 Humble / Gazebo Fortress 环境中，实现平面单层场景下的 Go2W 风格轮足机器人 SLAM 建图、路线标定、Nav2 自主导航、动态避障与可复现实验验证。

> 状态说明：本文是早期平面单层技术路线基线。当前项目已进入 FAST-LIO2 SLAM 点云、SLAM-cloud traversability graph、高层 `/pct_path` 生成和迁移前封板阶段；最新接手入口见 `docs/handoff/README.md`。若本文与当前代码/测试或 `docs/handoff/` 不一致，以当前代码/测试和 `docs/handoff/` 为准。

## 0. 最终技术决策

本项目只采用一条主线，不再并行维护版本或算法分支。

| 决策项 | 最终选择 | 理由 |
|---|---|---|
| 操作系统 | Ubuntu 22.04.5 LTS in WSL2 | 已安装，符合 ROS 2 Humble 官方目标平台。 |
| ROS 发行版 | ROS 2 Humble | 当前已安装，Nav2 / slam_toolbox / ros_gz 组件齐全。 |
| Gazebo | Ignition Gazebo Fortress 6.16.0 | Humble 官方配对 Fortress；本机已安装 `ignition-gazebo6`、`ros-humble-ros-gz`、`gz_ros2_control`。 |
| GPU 策略 | 使用 WSLg OpenGL/D3D12 GPU 渲染；Gazebo GUI 强制 `--render-engine ogre --render-engine-gui ogre` | 本机 `glxinfo -B` 已确认 `direct rendering: Yes`、`Accelerated: yes`、renderer 为 `D3D12 (NVIDIA GeForce RTX 3050 Laptop GPU)`；Gazebo GUI 的 `ogre` 后端真实启动探针已通过；默认 `ogre2` 在当前 WSLg/D3D12 环境会触发 OGRE 异常，不作为主线渲染后端。 |
| Gazebo 命令族 | Fortress 使用 `ign gazebo`，不是 `gz sim` | 本机 `gz` 命令不存在，`ign gazebo --versions` 为 `6.16.0`。 |
| 机器人平台 | Go2W 风格“导航等效体” | 保留 Go2W 尺寸、质量、轮组、传感器、footprint 和 TF 语义；不在第一阶段做完整腿部步态动力学。 |
| 主 SLAM | `slam_toolbox` | 本机已装 `ros-humble-slam-toolbox 2.6.10`；支持建图、保存地图、序列化 pose graph、localization mode 和 RViz 工具链。 |
| 全局规划 | Nav2 `SmacPlannerHybrid` | 支持非圆形/legged/car-like 机器人，能表达 Go2W 等效体的转弯半径与 footprint 约束。 |
| 路线标定 | Nav2 Route Tool + `nav2_route` | 本机已装 `nav2_route 1.1.20`；RViz 内创建/编辑 route graph，满足“工具标定目标点/路线”的展示要求。 |
| 局部控制 | Nav2 MPPI Controller | 预测式局部控制，支持 adaptive collision avoidance，适合动态障碍物响应与消融实验。 |
| 安全层 | `collision_monitor` + `velocity_smoother` | 独立于 planner/controller 的安全兜底与速度平滑，降低 WSL2/Gazebo 抖动带来的碰撞风险。 |
| 传感器主线 | 2D LiDAR + IMU + 轮里程计 | 足够支撑平面单层 SLAM/Nav2，避免当前 8GB 级内存和 4GB 显存环境被 3D 点云/RGB-D 压垮。当前 WSLg/OGRE 组合下 `/scan` 先由 ROS 侧 transitional raycast emulator 发布；真实 Gazebo GPU lidar 等渲染后端稳定后再切回。 |

不采用的方向：

- 不将 Gazebo Harmonic 作为当前主线。Humble + Harmonic 需要非官方 `ros-humble-ros-gzharmonic` 包，并会与现有 `ros-humble-ros-gz*` 冲突。
- 不将 FAST-LIO2 + 3D LiDAR + OctoMap 作为第一阶段主线。该路线研究味更强，但对点云吞吐、内存、TF 和 3D 到 2D 投影链路要求更高，不适合当前先交付平面单层导航。
- 不将 Cartographer 作为主实现。课程描述中的“Cartographer 等算法”允许同类 SLAM 算法；当前本机没有 `cartographer_ros`，且 `slam_toolbox` 与 Nav2 route tool 的复现链路更短。
- 不做完整 Go2W 轮腿动力学与步态控制。Unitree 官方 ROS/Gazebo 资料明确偏低层关节控制，Gazebo 仿真不提供完整高层 walking 控制；导航课题第一阶段只需要稳定的 `/cmd_vel -> odom -> TF -> Nav2` 闭环。

## 1. 阶段一：环境配置确定

目标：冻结可复现运行环境，确认 Gazebo 走 GPU/OpenGL 加速，避免后续把版本问题、渲染问题和算法问题混在一起。

### 1.1 固定软件版本

主线固定如下：

```text
OS: Ubuntu 22.04.5 LTS / WSL2
ROS_DISTRO: humble
Gazebo: Ignition Gazebo Fortress 6.16.0
Gazebo command: ign gazebo
Gazebo GUI render args: --render-engine ogre --render-engine-gui ogre
ros_gz: ros-humble-ros-gz 0.244.23
gz_ros2_control: ros-humble-gz-ros2-control 0.7.18
Nav2: 1.1.20
slam_toolbox: 2.6.10
```

现有本机组件已经满足主线开发，不需要迁移 Gazebo 版本。

### 1.2 GPU/Gazebo 判定标准

本项目把 GPU 可用性定义为：

1. `nvidia-smi` 能看到 NVIDIA GPU；
2. `glxinfo -B` 显示 `direct rendering: Yes`；
3. `glxinfo -B` 显示 `Accelerated: yes`；
4. OpenGL renderer 指向 `D3D12 (NVIDIA GeForce RTX 3050 Laptop GPU)`；
5. Gazebo GUI 可以打开轻量世界，RViz2 与 Gazebo 同时运行时系统不发生明显卡死。

已核对的当前事实：

```text
GPU: NVIDIA GeForce RTX 3050 Laptop GPU
Driver: 595.97
GPU memory: 4096 MiB
OpenGL renderer: D3D12 (NVIDIA GeForce RTX 3050 Laptop GPU)
OpenGL version: 4.2
Direct rendering: Yes
Accelerated: yes
```

工程判断：

- Gazebo GUI 渲染已经具备 GPU/OpenGL 加速路径。
- Gazebo 不以 CUDA 作为 GUI/传感器渲染的主要判据。
- 当前 WSLg OpenGL 4.2 + D3D12/NVIDIA 路径可以支撑 Fortress GUI，但必须强制使用 `ogre` 后端；`ogre2` 在本机实测触发 `Ogre::UnimplementedException`。
- 后续 Gazebo GUI 启动命令必须带 `--render-engine ogre --render-engine-gui ogre`，并继续轻量化模型和场景，不能堆高面数 mesh、RGB-D、高频 3D LiDAR。

### 1.3 环境交付物

阶段一完成后，仓库应有：

```text
docs/environment_baseline.md
scripts/check_gpu_gazebo_stack.sh
scripts/check_ros_nav_stack.sh
```

验收标准：

- `nvidia-smi` 和 `glxinfo -B` 输出被记录；
- `ign gazebo --versions` 输出 `6.16.0`；
- `ros2 pkg list` 中可见 `slam_toolbox`、`nav2_route`、`nav2_mppi_controller`、`nav2_smac_planner`、`ros_gz_bridge`；
- 明确记录使用 `ign gazebo`，不使用 `gz sim`；
- 明确记录 Gazebo GUI 使用 `--render-engine ogre --render-engine-gui ogre`。

## 2. 阶段二：地图/模型导入与运动控制链

目标：先让机器人在 Gazebo 中稳定出现、在 RViz2 中正确显示 TF、能通过 `/cmd_vel` 运动，并能发布 LiDAR/IMU/odom。

### 2.1 仓库包结构

采用 6 个 ROS 2 包，边界固定：

```text
airos_go2w_description/
  URDF/Xacro、mesh、robot_state_publisher、RViz 模型配置

airos_sim/
  Gazebo world、spawn、ros_gz bridge、传感器仿真配置

airos_control/
  ros2_control/gz_ros2_control、diff/skid 四轮速度控制、controller yaml

airos_slam/
  slam_toolbox mapping/localization 参数、地图保存与 pose graph 管理

airos_nav/
  Nav2 参数、route graph、RViz route tool 配置、地图与导航 bringup

airos_experiments/
  任务脚本、动态障碍脚本、rosbag2 记录、指标统计、报告图表生成
```

### 2.2 Go2W 导航等效体建模

模型原则：

- 外形尺寸采用 Go2W 官方规格近似：`0.70m x 0.43m x 0.50m`；
- 质量采用约 `18kg`；
- 轮胎半径按 7 英寸轮胎近似；
- `visual` 可以保留 Go2W 风格外观；
- `collision` 必须简化为 box/cylinder，禁止直接用高面数 mesh 做碰撞；
- 腿部先固定为半蹲轮式姿态，不进入控制闭环；
- Nav2 footprint 按真实占地矩形加安全余量建模。

TF 主干固定为：

```text
map -> odom -> base_link
base_link -> lidar_link
base_link -> imu_link
base_link -> camera_link
base_link -> fl_wheel_link / fr_wheel_link / rl_wheel_link / rr_wheel_link
```

### 2.3 Gazebo/ROS 桥接链

主链话题：

```text
Gazebo clock        -> /clock
ROS raycast emulator -> /scan
Gazebo IMU          -> /imu
Gazebo odom/control -> /odom
Gazebo joints       -> /joint_states
Nav2 command        -> /cmd_vel
Smoothed command    -> /cmd_vel_smoothed
```

控制链固定为：

```text
RViz/Nav2 goal
  -> bt_navigator / route_server
  -> planner_server(SmacPlannerHybrid)
  -> controller_server(MPPI)
  -> velocity_smoother
  -> collision_monitor safety gate
  -> ros2_control controller
  -> gz_ros2_control
  -> Gazebo wheel joints
  -> odom / joint_states / TF feedback
```

当前阶段二的 `/scan` 是过渡方案：`airos_experiments/scan_emulator` 从 `single_floor_lab.sdf` 读取静态障碍物，结合 `/odom` 做 2D ray cast 并发布 `sensor_msgs/LaserScan`。原因是本机 `ogre2` 会触发 `Ogre::UnimplementedException`，而 `gpu_lidar` 在 `ogre` 后端只发布全 0 range，不能作为 SLAM/Nav2 输入。

### 2.4 阶段二验收标准

- Gazebo 和 RViz2 同时运行；
- RViz2 中模型、LiDAR frame、IMU frame、wheel frames 全部可见；
- `/tf` 中存在稳定 `odom -> base_link`；
- `/scan`、`/imu`、`/odom`、`/joint_states` 正常发布；
- 向 `/cmd_vel` 发布低速指令时，机器人在 Gazebo 平面移动，RViz2 里 TF 同步变化；
- Gazebo GUI 使用 `ogre` + GPU/OpenGL 加速，不落入软件渲染。

## 3. 阶段三：SLAM 建图

目标：用 `slam_toolbox` 完成平面单层环境建图、地图保存、pose graph 序列化与 localization mode 验证。

### 3.1 SLAM 输入

固定输入：

```text
/scan
/odom
/tf
/tf_static
/clock
```

IMU 不直接喂给 `slam_toolbox`，而是通过里程计融合链影响 `odom -> base_link`。这样接口更清晰，SLAM 只负责 `map -> odom`。

### 3.2 slam_toolbox 模式

建图阶段：

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

定位阶段：

```text
mode: localization
加载已保存 pose graph
保留 rolling scan buffer
不再污染已确认地图
```

### 3.3 地图产物

地图文件统一放入：

```text
airos_nav/maps/
  single_floor_lab.yaml
  single_floor_lab.pgm
  single_floor_lab_slam.yaml
  single_floor_lab_slam.pgm
  single_floor_lab_slam.posegraph
  single_floor_lab_slam.data
```

命名原则：

```text
<scene_name>_<map_version>.<yaml|pgm|posegraph>
```

当前实现采用双地图策略：

- `single_floor_lab.yaml` / `.pgm` 是默认 Nav2 seed map，从 SDF 静态障碍生成，覆盖 12m x 12m；
- `single_floor_lab_slam.*` 是 `slam_toolbox` smoke 建图产物，用于证明 mapping、serialize 和 localization 链路；
- 短时 smoke SLAM 地图可能覆盖范围较小，不能替代默认 Nav2 seed map。

### 3.4 阶段三验收标准

- RViz2 中 `/map` 实时增长；
- `map -> odom -> base_link` 连续稳定；
- 保存 `.yaml/.pgm` 和 `.posegraph`；
- 重启后可加载地图进入 localization mode；
- 同一静态场景中至少完成 3 次建图，地图拓扑一致；
- 记录建图耗时、平均 CPU、峰值内存、闭环是否成功。

## 4. 阶段四：导航路径规划与避障

目标：在已建地图上，通过 RViz2 工具标定路线/目标点，让 Go2W 等效体在 Gazebo 中自主导航，并对动态障碍物做实时避障。

### 4.1 路线标定方式

使用 Nav2 Route Tool 作为主交互工具：

```text
ros2 launch nav2_rviz_plugins route_tool.launch.py yaml_filename:=<map.yaml>
```

路线图保存为：

```text
airos_nav/routes/single_floor_lab_route.geojson
```

Route graph 语义：

- node：可停靠点、转弯点、门洞点、任务点；
- edge：允许通行路线；
- metadata：路线限速、危险区域、狭窄区域、动态障碍高发区域。

普通 RViz2 `2D Goal Pose` 只作为人工验证入口；正式演示使用 route graph + goal。

### 4.2 Nav2 主参数

全局规划：

```text
planner: nav2_smac_planner::SmacPlannerHybrid
motion_model_for_search: REEDS_SHEPP
minimum_turning_radius: 按等效体低速转弯能力标定
angle_quantization_bins: 72
allow_unknown: true
```

局部控制：

```text
controller: nav2_mppi_controller::MPPIController
motion_model: diff_drive
controller_frequency: 20Hz 起步
batch_size: 1000-1200 起步
time_steps: 48
model_dt: 0.05
visualize: false
```

Costmap：

```text
static_layer
obstacle_layer
inflation_layer
footprint collision checking
```

安全与平滑：

```text
velocity_smoother
collision_monitor
```

### 4.3 动态障碍设置

当前阶段的动态障碍物在 ROS `/scan` 层注入，不在 Gazebo 中生成真实移动刚体。这样可以绕开当前 WSLg/Fortress/OGRE 传感器限制，同时保留对 Nav2 obstacle layer、MPPI 和 collision_monitor 的算法级压力测试。

动态障碍物只引入平面单层目标：

- 横穿走廊的人形圆柱；
- 慢速移动小车；
- 突发静态障碍；
- 狭窄门洞附近的局部阻塞。

障碍物评价不靠主观视频判断，统一记录：

```text
success / failure
collision count
minimum obstacle distance
emergency stop count
replan count
cmd output period as controller-cycle proxy
task completion time
path length
```

当前 runner 已记录：success/failure、任务耗时、路径长度、急停次数、scan 阈值碰撞估计、最小 scan 距离、cmd 输出平均/最大周期。`run_clean_nav_batch` 还会为每个 trial 干净启动 sim/nav、等待 `map -> base_link` 定位 TF、执行单个 mission、写 JSONL 并清理 launch 进程组；现在支持 `--attempts`，用于规避 WSL/Gazebo/Nav2 bringup 的偶发 transient 失败。replan count 和 Gazebo 物理接触计数尚未实现。

批量实验必须使用 `run_nav_trials --reset-sim`。原因是 `/initialpose` 只会重置 AMCL 的定位假设，不会移动 Gazebo 中的物理机器人；如果不重置模型位姿，多任务统计会把前一个任务终点错误当作下一个任务起点。

当前限制：连续多任务复用同一个 Nav2 会话仍不稳定，主要表现为 recovery 之后 planner 报 `Starting point in lethal space` 或 controller `Failed to make progress`。`run_nav_trials` 已支持 `--mission-id`；`run_clean_nav_batch` 是当前已验收的采集入口。后续若要恢复长会话连续批量，需要继续处理 Nav2 recovery/costmap 状态复用。

### 4.4 阶段四验收标准

- RViz2 同时显示 map、route graph、global path、local trajectory、costmap；
- Gazebo 显示机器人和世界；RViz2 显示 scan 层动态障碍 marker；
- 从 route graph 或 2D goal 触发任务后，机器人能沿路径运动；
- 动态障碍横穿时，机器人能减速、绕行或安全停车；
- `collision_monitor` 能在危险距离内截断速度；
- `velocity_smoother` 输出速度无明显抖动；
- 20 次固定任务成功率达到可报告水平。

当前状态：Nav2 静态主链已验证到 lifecycle active、action server 可见、`map -> base_link` TF 可查、`/scan` 约 9Hz。2026-05-07 的一次干净 static mission smoke 已成功到达目标，runner 记录 `success=true`、耗时 14.811s、路径 3.39m、最小障碍距离 0.58m。同日 `run_clean_nav_batch` 也完成了 `lab_start_to_task_a` 单任务，记录 `status=4`、`success=true`、耗时 68.781s、路径 3.566m、最小障碍距离 0.58m。4 任务 clean batch 初次 probe 为 3/4，失败项 `lab_door_passage` 是 Nav2 action server 等待不足；加长并结构化 action server 等待后，`lab_door_passage` 单独复测成功，并且 4 任务 clean batch 复测达到 4/4 成功，`success_rate=1.0`、平均耗时 19.479s、平均路径 3.182m、最小障碍距离 0.405m。最终 clean process-per-trial 20 次固定任务验收已完成，四个 mission 各 5 次，20/20 成功，`success_rate=1.0`、平均耗时 19.195s、平均路径 3.102m、最小障碍距离 0.389m、急停 0、scan 阈值碰撞 0。Route graph 计算链也已用 `verify_route_graph` 验证，`/compute_route` 从 node 1 到 node 4 返回 nodes `1 -> 3 -> 4`、edges `103 -> 105`。Route waypoint clean smoke 已通过，`execution_mode=navigate_through_poses`，说明机器人能按 route graph 节点序列执行单任务。动态障碍 clean smoke 已在 `lab_start_to_task_a` 上通过，日志显示 `dynamic=True`、collision_monitor slowdown/recovery 和 `Goal succeeded`。保留的工程风险是部分 `return_point` run 会先出现 `Starting point in lethal space` 警告再恢复成功，后续应继续放宽地图/footprint/inflation 裕度；完整 route-constrained batch 仍未验收。

## 5. 阶段五：创新与消融归因

目标：在不破坏主链可复现性的前提下，把创新放进“路线决策/风险代价/控制评价”，而不是用黑盒模型直接接管底盘。

### 5.1 最终创新点

本项目采用一个创新主点：

```text
Route graph + MPPI + risk-aware cost annotation
```

含义：

- 先用 `slam_toolbox` 生成地图；
- 再用 Nav2 Route Tool 在 RViz2 中标定路线图；
- 对 edge/node 添加风险元数据，例如窄门、动态障碍高发区、低速区；
- route server 用结构化路线约束全局行为；
- MPPI 在局部 costmap 中处理动态障碍；
- collision_monitor 做最后安全兜底。

这个创新点的优点：

- 不破坏 Nav2 主体；
- 可解释；
- 可复现；
- 可以做消融；
- 与课程要求的“任务定义-路线设计-基线对齐-消融归因”强相关。

### 5.2 消融实验

固定只做 4 组：

| 组别 | 路线层 | 全局规划 | 局部控制 | 安全层 | 目的 |
|---|---|---|---|---|---|
| A 基线 | 无 route graph，仅 2D Goal | Smac Hybrid | RPP | collision_monitor | 最低风险可运行基线 |
| B 主系统 | route graph | Smac Hybrid | MPPI | collision_monitor | 本项目最终展示系统 |
| C 控制消融 | route graph | Smac Hybrid | RPP | collision_monitor | 归因 MPPI 对动态避障的贡献 |
| D 安全消融 | route graph | Smac Hybrid | MPPI | 无 collision_monitor | 归因安全栅栏的必要性 |

每组固定：

- 同一地图；
- 同一起终点；
- 同一动态障碍脚本；
- 同一随机种子集合；
- 同一速度上限；
- 同一 footprint。

### 5.3 实验指标

SLAM 指标：

```text
建图时间
地图闭环成功率
地图可通行区域一致性
pose graph 保存/恢复成功率
```

导航指标：

```text
任务成功率
碰撞率
路径长度
任务完成时间
最小障碍物距离
急停次数
replan 次数
controller cycle time
CPU / memory / GPU 状态
```

展示指标：

```text
Gazebo + RViz2 同屏可视化
SLAM 地图生成过程
Route graph 标定过程
Global path / local trajectory / costmap
动态障碍避让过程
最终统计图表
```

## 6. 阶段六：报告、PPT 与视频组织

当前已生成：

- `docs/report_outline.md`
- `docs/ppt_outline.md`
- `docs/video_script.md`
- `results/single_floor_lab_summary.csv`
- `results/single_floor_lab_summary.md`
- `results/figures/mean_elapsed_sec.svg`
- `results/figures/mean_path_length_m.svg`

报告结构固定：

1. 课题背景与目标；
2. 环境与硬件约束；
3. 为什么选择 Humble + Fortress；
4. Gazebo GPU/OpenGL 事实核对；
5. Go2W 导航等效体建模；
6. SLAM 建图链路；
7. Nav2 路线规划与 MPPI 避障；
8. route graph 风险标注创新；
9. 实验设计与消融；
10. 结果图表；
11. 局限与后续升级。

PPT 结构固定：

```text
01 题目与最终效果
02 系统架构总图
03 环境/GPU事实
04 Go2W等效体建模
05 SLAM建图流程
06 Nav2路线规划流程
07 动态避障与MPPI
08 创新点：route graph + risk annotation
09 消融实验
10 结果与结论
```

视频脚本固定：

```text
00:00 Gazebo + RViz2 同时启动
00:20 Go2W 等效体与 TF 展示
00:40 slam_toolbox 建图
01:20 保存地图并加载 localization
01:40 RViz2 Route Tool 标定路线
02:20 发送导航任务
02:50 动态障碍出现
03:20 MPPI 绕行 / collision_monitor 安全停车
03:50 指标面板与结论
```

## 7. 实施顺序

后续实现必须按以下顺序推进：

1. 环境基线与 GPU/OpenGL 检查脚本；
2. Go2W 导航等效体 URDF/Xacro；
3. Gazebo 单层静态室内世界；
4. ros_gz bridge 与 ros2_control 控制链；
5. RViz2 模型、TF、传感器可视化；
6. slam_toolbox mapping；
7. 地图保存与 localization；
8. Nav2 静态导航；
9. Nav2 Route Tool 与 route graph；
10. MPPI 局部控制；
11. collision_monitor 与 velocity_smoother；
12. 动态障碍世界；
13. 消融实验脚本；
14. 报告/PPT/视频材料生成。

任何阶段失败时，只回退到当前阶段的最小闭环，不跨阶段调参。

## 8. 主要事实源

- Gazebo 与 ROS 2 兼容关系：<https://gazebosim.org/docs/harmonic/ros_installation/>
- Gazebo Classic 迁移文档中 Humble/Fortress 官方配对说明：<https://gazebosim.org/docs/harmonic/migrating_gazebo_classic_ros2_packages/>
- Gazebo headless rendering / GPU 传感器说明：<https://gazebosim.org/api/sim/9/headless_rendering.html>
- Gazebo OpenGL/Ogre2 troubleshooting：<https://gazebosim.org/docs/latest/troubleshooting/>
- `ros_gz` Humble/Fortress 包状态：<https://index.ros.org/r/ros_gz/>
- `slam_toolbox` Humble 文档：<https://docs.ros.org/en/ros2_packages/humble/api/slam_toolbox/index.html>
- Nav2 Route Server：<https://docs.ros.org/en/ros2_packages/humble/api/nav2_route/index.html>
- Nav2 Route Tool：<https://docs.nav2.org/tutorials/docs/route_server_tools/navigation2_route_tool.html>
- Nav2 Smac Hybrid-A*：<https://docs.nav2.org/configuration/packages/smac/configuring-smac-hybrid.html>
- Nav2 MPPI Controller：<https://docs.nav2.org/configuration/packages/configuring-mppic.html>
- Nav2 MPPI Humble API：<https://docs.ros.org/en/ros2_packages/humble/api/nav2_mppi_controller/>
- Nav2 Collision Monitor：<https://docs.nav2.org/configuration/packages/configuring-collision-monitor.html>
- Nav2 Velocity Smoother：<https://index.ros.org/p/nav2_velocity_smoother/>
- Unitree Go2W 官方规格：<https://www.unitree.com/mobile/go2-w>
- Unitree ROS 仿真说明：<https://github.com/unitreerobotics/unitree_ros>
- Unitree ROS2 官方仓库：<https://github.com/unitreerobotics/unitree_ros2>
