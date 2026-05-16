# AIROS 深度升级发现记录

> 状态：历史发现记录。当前迁移前封板事实源见 `docs/handoff/`。

## 在线调研

### ypat999/3d_dog_navi_ros2

- 仓库地址：https://github.com/ypat999/3d_dog_navi_ros2
- README 说明该项目面向 ROS 2 Humble 和 Unitree Go2W，包含 FAST-LIO2、PCT-planner、ego-planner、Go2W 模型、混合控制和 RViz 工具等目标。
- README 明确写有“建设中，目前只有手动控制”，因此不能把该仓库视为可直接稳定接入的完整自主导航实现。
- README 推荐 Gazebo Garden 8.10+ 或 Gazebo 11，并安装 `ros-humble-ros-gzgarden-bridge`；这与本项目当前 Gazebo Fortress 稳定基线不一致。
- 可借鉴内容：话题表面 `/livox/lidar`、`/livox/imu`、`/cloud_registered`、`/Laser_map`、Go2W + Mid360 模型定位、PCT/EGO 作为研究入口、坡道检测参数。

### Nav2 route server

- 官方文档：https://docs.ros.org/en/humble/p/nav2_route/
- route server 是预定义 navigation route graph 规划器，补充自由空间 planner。
- Humble 文档列出 route graph、方向边、Kd-tree 最近节点、动态 edge scorer、operation plugin、ComputeRoute/ComputeAndTrackRoute 等能力。
- 本项目已有 `verify_route_graph` 和 route waypoint smoke，后续硬化应继续基于 route graph + `navigate_through_poses`，而不是绕开 Nav2。

### 规划算法

- 当前最适合工程落地的路径：保留 Nav2 Smac/MPPI/RPP 基线，加入 planner/controller profile 对比和 route-constrained 任务批量门禁。
- PCT-planner 属于 3D/多楼层研究方向，依赖 GPU/CUDA/GTSAM/点云断层地图，短期应作为 experimental 入口和文档化接口，不应覆盖当前可跑通基线。
- RL 路径规划通常需要训练环境、策略模型、状态/动作接口和安全约束；短期只能落成接口/评估入口，不能声称完成强化学习自主规划。

## 本仓库事实

- 当前提交 `3ce7c1e` 已实现 Gazebo 原生 `/scan` 和初版 `/livox/lidar/points -> /livox/lidar` PointCloud2 桥接；本轮升级继续修正为 `/livox/lidar_points` PointCloud2 raw topic，再转换成 `/livox/lidar` Livox CustomMsg。
- 当前基线回退提交为 `5d756de`。
- 工作树中未跟踪 PDF/DOCX、`results/`、`ubuntu.txt` 属于用户资料或历史结果，不纳入本轮提交。
- `world_map_generator.py` 能从本仓库 SDF 的 box/cylinder collision 生成 Nav2 2D seed map，因此高级场景采用本地 SDF 生成法，避免直接复制 Garden world 后无法生成匹配地图。
- 本机 Gazebo Fortress 安装了 `ignition-gazebo-triggered-publisher-system` 和 `ignition-gazebo-velocity-control-system`，可用于 Gazebo 物理动态障碍。
- `3d_dog_navi_ros2` 的 Go2W 模型文件和 Building mesh 可作为长期视觉参考；直接复制整包会引入 Garden、CHAMP、多包控制栈和自定义 Livox 插件，短期风险过高。
- 已采用受控导入：只复制 AFL 3.0 许可的 Building mesh、Go2W body/wheel/Mid360 mesh 和 license 文本，作为可选视觉层；不迁入 Garden/CHAMP 控制栈。


## 2026-05-11 PCT 执行链发现

- 本项目当前默认 PCT 演示应解释为：PCT-style 三维地形规划器负责跨高度路径，Nav2 不再启动 planner_server/bt_navigator/route_server 参与全局规划。
- Nav2 `FollowPath` + RotationShim/RPP 对斜坡坡底急转不稳定：能下坡，但会因碰撞预测或路径剪空 abort，并表现为原地旋转/不出发。
- 默认关闭动态障碍必须从 Gazebo world 层移除物理动态模型；仅不触发 velocity-control 仍会让静态障碍占据路径。
- 更稳定的工程链路是 PCT planner 自己按三维路径做局部 pure-pursuit/waypoint 跟踪，输出 `/cmd_vel_nav`，再经过 Nav2 velocity_smoother 和 collision_monitor 到 base controller。

## 2026-05-15 迁移前封板发现

- 当前主路线是 `/Laser_map_world` -> SLAM-cloud traversability graph -> `/pct_path` -> direct terrain tracking -> velocity smoother -> collision monitor -> base controller。
- 高层 `/pct_path` 已有在线证据：`PATH_MAXZ_OVERALL=2.155`，但这只证明 SLAM map 到 PCT path 生成链路有效，不证明机器人物理到达高层平台。
- 下一阶段应直接聚焦高层路径执行：ramp-entry approach、3D waypoint gating、local slope direction、direct tracking stall/replan 和 safety chain 协同。

## 2026-05-16 跨层执行诊断发现

- 单层 Nav2 不是当前直接瓶颈：`single_floor_lab` 四个 clean batch mission
  已在 2026-05-16 全部成功。
- 高层 `/pct_path` 可复现，不应再把“没有生成高层路径”作为当前事实。
  最新 360 秒窗口 `log/cross_level_full_20260516_113311/probe.jsonl` 中
  `pct_path_max_z: 2.138301`。
- command chain 不是全局断路：同一窗口中 base controller command 非零采样为
  37/72。
- 关键未解问题是物理高度：最终 `fast_lio_xyz.z` 为 `0.360271`，
  但 Gazebo ground-truth z 仍为 `-0.005001`。这说明不能用 aligned odom
  的轻微 z 变化替代物理爬升验收。
- 当前最强假设是 direct tracking 的高层 waypoint 进展、SLAM/aligned odom
  高度和 Gazebo 物理高度之间存在不一致；仍需区分 ramp-entry 失败、
  height gating 错误、SLAM graph 非物理连边和 surrogate 物理能力限制。
- 已确认并修复一个非物理连边：稀疏 SLAM bridge 允许 `slam_floor`
  直接跳连到 `second_floor_deck` 顶面附近，例如 `(0.60, 0.85, 0.75)`。
  修复后该类 floor-to-deck edge jump 被单元测试拒绝。
- 修复后高路径仍可生成，且第一高目标转为 `slam_ramp`，说明高层路径生成
  没被整体破坏。
- 最新运行出现部分物理爬升：Gazebo z 最高 `0.217646`，但没有持续爬升到
  高层，最终距离目标仍约 `15.57m`。下一步应聚焦 ramp segment 的物理接触、
  command smoothing/collision 输出和坡道跟踪策略。
- `cross_level_bridge_guard_20260516_122021` 的后处理显示命令链存在新的强
  疑点：高路径有效后 `/cmd_vel_nav` 大多非零，但 `/cmd_vel_smoothed` 与 base
  command 在 5 秒 probe 样本中多次为 0；同时坡道段 direct 限速为 `0.16`，
  且 collision monitor 多次 SlowZone。旧 probe 不能区分持续掉线和采样瞬时
  0，因此下一次运行必须使用新增的 command count/age 字段。
- 增强 probe 已确认一个真实命令超时路径：`/cmd_vel_nav_age_sec` 可达到
  `3.91s`，超过 velocity_smoother `1.0s` timeout。直接原因更接近
  `terrain_pct_planner` 同步 SLAM graph rebuild 饿死 direct control timer，
  已改为后台 rebuild；下一步需用运行证据确认 command age 是否下降。
- 后台 rebuild 修复后的 180 秒验证显示 command age 在高路径阶段已低于
  smoother timeout，但物理高度仍未上升。新的强信号是 high deck waypoint
  progression：direct tracking 到达 `slam_deck` 高目标附近时，aligned odom z
  约 `0.36`、`gate_z` 约 `0.03`、Gazebo z 仍近地面。下一步应测试并修正
  高层 waypoint 进展条件，而不是继续扩大速度/timeout。
- 高 deck waypoint false progression 已有单元测试覆盖并修复：高
  `slam_deck` 节点现在使用更严格的 z tolerance，避免只因 XY 靠近就推进到
  后续 deck waypoint。该修复尚未 runtime 验证，下一次运行要观察是否在
  ramp/deck 边界形成可解释 stall 或开始物理爬升。
- deck gate 修复已 runtime 验证为“压住 false progression，但未解决物理爬升”：
  `cross_level_deck_gate_20260516_130933` 中高路径和命令链均正常，但 Gazebo z
  没有升高，系统在 ramp/step 高目标附近 stall/replan。下一步应从 ramp/step
  入口几何、接触/摩擦、或更合适的足式模型 profile 继续。
- 对 `cross_level_deck_gate_20260516_130933` 的最新 SDF 几何对照显示：最终
  goal `(6.0,13.0,2.2)` 的确落在 `third_floor_deck` /
  `upper_ramp_landing`，但中间高目标 `(6.03,3.20,0.71)` 更接近
  `lab_table_10` 顶面/边缘，不是真实 ramp/stair/deck 入口。
- 已新增并通过 `test_slam_frontier_path_rejects_isolated_obstacle_step_ahead`：
  high-floor frontier 不应让孤立低高度 `slam_step` 障碍/桌面端点压过继续前进
  的低层走廊候选。该修复仍需下一次 runtime 验证。
- frontier guard 后的运行 `cross_level_frontier_step_guard_20260516_133146`
  已验证高路径和命令 freshness 仍正常：`pct_path_max_z: 2.187763`，
  `/cmd_vel_nav_age_sec` 最大 `0.084s`，smoother/base command age 最大约
  `0.065s`。但 Gazebo z 仍为 `-0.005001`，且高路径卡在
  `(0.64,1.52,0.74)` `slam_step` 目标附近。SDF 对照显示该点位于
  `second_floor_deck` XY 范围但低于 deck 顶面，不是有效 ramp/stair 入口。
  下一步应补 final-path transition 约束测试，而不是继续扩大 command timeout。
- 已新增并通过 `test_final_high_path_rejects_deck_edge_step_without_ramp_approach`：
  final high path 不应在缺少 ramp/stair/step 连续爬升证据时先进入高
  `slam_step` / `slam_deck` 候选。补丁后静态门禁为 `128 passed in 33.65s`，
  `git diff --check` 通过，`colcon build --symlink-install` 8 packages 通过。
- final transition guard 后的运行
  `cross_level_final_transition_guard_20260516_140148` 中，高路径仍可生成，
  `pct_path_max_z: 2.630439`，命令链基本健康；Gazebo z 从地面短暂上升到
  `0.154583`，但随后回到约 `-0.005001`，未到达高层目标。新的瓶颈是
  ramp-ascent persistence / high-step entry execution，而不是最早的
  `(0.64,1.52,0.74)` deck-edge shortcut。
- 后续 SDF 几何核查发现一个物理环境缺陷：`lower_access_ramp` 的原始坡度/高度
  没有连续连接 `ramp_lower_landing` 与 `ramp_upper_landing`/`second_floor_deck`。
  已用 `test_large_multilevel_lower_ramp_physically_connects_landings` 锁定，并在
  `large_multilevel_complex.sdf` 与 `large_multilevel_complex_static.sdf` 中将
  `lower_access_ramp` pose 调整为中心 z `0.440`、pitch `-0.0678`。
- 同步发现并修复一个 ramp-to-ramp 过渡约束缺口：不同 ramp-like surface 之间
  也必须满足 ramp-entry corridor，避免从 landing 边缘斜切进入 ramp。聚焦测试
  通过，随后 visual config、SLAM graph、terrain planner、control chain 与
  cross-level probe 总门禁为 `130 passed in 33.37s`，`git diff --check` 通过，
  `colcon build --symlink-install` 完成 8 packages。
- SDF ramp 几何修正后的运行 `log/cross_level_ramp_geometry_20260516_155258/`
  复现了高层路径：`pct_path_max_z: 2.212123`，第一次高路径在
  `elapsed_sec: 40.21` 出现。命令 age 健康：`cmd_vel_nav_age_sec` 最大
  `0.091s`，`cmd_vel_smoothed_age_sec` 最大 `0.061s`，base command age 最大
  `0.075s`。
- 同一运行仍未通过物理跨层验收：Gazebo z 全程最高仍为 `-0.005001`，
  最小目标 XY 距离为 `11.624816m`，最终 Gazebo pose 为
  `[-6.940314, 3.194232, -0.005001]`。几何修正让执行不再卡在旧的低层入口
  附近，但没有证明真实爬升。
- 该运行日志显示 direct tracker 可沿 `slam_ramp` 进入高 `slam_deck` /
  `slam_step` waypoint 序列，而 Gazebo 物理 z 仍在地面。下一步应把 terrain
  surface `gate_z` 与“物理高度进展”分离，优先测试高 deck/step waypoint 是否
  仍可被高 surface gate 误判为已接近。
- 已按该证据新增并通过
  `test_high_surface_gate_does_not_prove_physical_height_progress`：高
  `slam_deck`/`slam_step` waypoint 进展不再直接使用 surface `gate_z` 作为
  物理高度证明。`terrain_pct_planner` 现在用 `direct_tracking_progress_z`
  在高 deck/step 节点上回落到原始 robot z；`gate_z` 仍保留用于 surface 估计和
  诊断日志。该修复完成静态验证，仍需下一次 headless runtime 验证。

## 2026-05-16 快速展示优先级发现

- 用户最新目标强调“尽可能快地出展示结果和较好效果”，最低目标是复杂单层
  SLAM 建图、路径规划和执行闭环；跨楼层物理爬升不再是当前唯一阻塞项。
- 稳定展示线应优先使用已验收的单层 Nav2 clean runner。最新复验
  `log/fast_show_single_floor_clean_batch_20260516.jsonl` 中 `lab_door_passage`
  成功，且 collision / emergency stop 均为 0。
- FAST-LIO/PCT 单层展示线需要显式使用
  `terrain_goal_z_policy:=nearest_z terrain_goal_min_z:=-1.0`。若沿用默认
  `goal_z_policy:=highest`，即使目标消息 z 为 0，也可能吸附到高结构候选，
  不适合作为单层展示参数。
- 在 `large_multilevel_complex` 中，目标 `(8.0,-6.8,0.0)` 会被 SLAM 图吸到
  高结构候选，首轮日志出现 `target_z≈1.96`，第二轮使用 `nearest_z` 后仍可能
  吸到约 `target_z≈1.24`，因此它不是可靠的单层展示目标。
- 目标 `(8.0,-9.0,0.0)` 在 `nearest_z` 下可生成低层路径：
  `/pct_path_max_z≈0.226738`，地图点数增长到 `599135`，命令链三路 age 均低于
  `0.1s`。但 Gazebo 物理目标距离几乎未改善，说明 direct tracker 或底盘物理
  执行仍有单层局部卡点。
- FAST-LIO/PCT 单层近目标 `(1.9,-9.2,0.0)` 已形成可展示 smoke：
  `log/single_floor_fast_lio_demo_20260516_1652/probe_goal_1p9_-9p2.jsonl`
  记录 `/Laser_map_world` 从 `267848` 点增长到 `401742` 点，低层
  `/pct_path_max_z≈-0.073576`，三路命令链有效，Gazebo 目标距离从
  `2.061553m` 降到 `0.603608m`。launch 日志记录
  `terrain direct tracking goal reached`。
- 该 FAST-LIO/PCT 近目标 smoke 可以作为“SLAM 点云建图 -> PCT-style 低层路径
  -> direct 执行 -> Gazebo 实际移动”的展示证据，但仍不是精确到点验收；最终
  Gazebo 目标距离大于 direct goal tolerance `0.30m`。
- 增强 probe 已定位该误差：旧 smoke 中 `/fast_lio_odom_world` 到目标约
  `0.125m`，但 wheel odom / Gazebo 到目标约 `0.569m`。因此
  `/fast_lio_odom_world` 可用于 FAST-LIO 主线执行，但不能单独作为单层实体
  展示验收标准。
- 新增 `terrain_odom_topic` 后，显式使用 `terrain_odom_topic:=/odom` 的
  单层复跑已形成当前最佳 FAST-LIO/PCT 展示证据：
  `log/single_floor_fast_lio_demo_20260516_odom/launch.log` 记录收到目标、
  生成 `poses=6 path_nodes=7` 并 `terrain direct tracking goal reached`；
  `probe_goal_1p9_-9p2_after_pub.jsonl` 记录 `/Laser_map_world` 增长到
  `631671` 点，wheel odom / Gazebo 最终到 `(1.9,-9.2)` 的距离均为
  `0.214862m`，低于 direct goal tolerance `0.30m`。
- 操作层坑点：一次性 `ros2 topic pub --once` 可能在 volatile topic discovery
  或 graph 初始化窗口内丢失，表现为地图增长但 `/pct_path` 和命令计数全为 0。
  快速展示应使用 `--times 5 --rate 1` 或脚本等待 subscriber/graph ready 后
  再发布目标。
- 下一步若继续提高 FAST-LIO/PCT 单层效果，应把展示脚本化、增加更长单层路线
  和路径形状约束，而不是继续换高层目标；不要用 `(8.0,-6.8)` 作为单层验收
  目标。
- 单层 FAST-LIO/PCT 展示已脚本化：
  `scripts/run_fast_lio_single_floor_demo.sh` 默认复用 `(1.9,-9.2)` 和 `/odom`
  实体验收链，保留 `--dry-run`，并限制 `MAX_LOG_RUNS_TO_KEEP` 以降低日志占用。
  脚本 acceptance 不是只看最后一帧 `/pct_path`，而是联合 launch log 的
  planner/direct tracking 证据和 probe 的 wheel/Gazebo 距离。
- 真实脚本 smoke `log/fast_lio_single_floor_demo/smoke_20260516_172800/`
  已通过：`accepted: true`、`laser_map_points_max: 198858`、
  `cmd_vel_nav_count_max: 202`、wheel/Gazebo 到目标距离均为 `0.20666m`。
  该运行中 probe 没采到 active `/pct_path`，原因是 tracker 已完成并清空路径；
  launch log 已补齐路径生成和 `goal reached` 证据。
- `long_corridor` 命名目标已加入 runner，但还不是验收目标。探索运行
  `log/fast_lio_single_floor_demo/long_corridor_20260516_173459/` 中低层路径和
  命令链都有效，Gazebo 距离从 `7.841325m` 降至 `0.313592m`，但未进入
  `0.30m` acceptance。该失败应保留为路径/终点判据调试证据。
- `long_corridor` 的新风险是目标候选层选择：launch log 早期记录
  `target_z≈1.237`，说明 `nearest_z` 对远目标仍可能被高结构影响。下一步
  应给单层 profile 增加 z-window / low-floor goal candidate 约束，而不是只
  扩大 goal tolerance。
- 运行清理链存在真实残留风险：clean batch 后出现
  `nav2_map_server/map_saver_server` 残留。已将其加入
  `scripts/cleanup_airos_runtime.sh` 终止列表，并加测试覆盖。
- 单层 z-window 是必要约束：`terrain_goal_z_policy:=nearest_z` 只能改变候选排序，
  不能保证远目标不受高结构影响。`terrain_goal_max_z:=0.45` 后，
  `long_corridor_zwindow_20260516_174832` 的首个目标为 `target_z=0.45`，
  `/pct_path_max_z=-0.030729`，说明层选择被压回低层。
- 不能通过放宽 `0.30m` 验收阈值来包装长单层通过。`long_corridor` 在 z-window
  后最优仍停在 `0.302173m`，属于边界负结果；末端 snap 修正也只允许在最后
  graph 节点到用户目标不超过 direct goal tolerance 时追加终点。
- `long_corridor_finalsnap_20260516_175457` 未通过不是命令链断裂：
  `cmd_vel_nav_count_max=540`、`base_cmd_count_max=657`，planner 也记录
  `terrain direct tracking goal reached`。更直接的证据是最终可达 graph 节点约
  `(7.81,-9.55,0.00)`，离用户目标 `(8.0,-9.0)` 太远，末端 snap 正确拒绝外推。
  后续应优化目标附近地图覆盖/目标候选选择/前沿继续推进，而不是让 tracker
  离开已建图支撑区域追点。
- 当前可展示 FAST-LIO/PCT 主线仍是 `near_goal`：新逻辑后
  `near_goal_after_zwindow_snap_20260516_175830` 通过，Gazebo/wheel 到目标
  `0.223881m`，并且 launch log 有目标接收、direct tracking 和 goal reached
  证据。
- 跨楼层阶段必须升级 goal 工具和验收语义：2D goal 只能表达 XY，不能可靠标定
  目标楼层/高度。高层目标应通过 3D/floor-aware goal 消息或明确的目标工具发布，
  并记录目标 z/floor 与选中 graph layer。
- 跨楼层路径不能只看几何最短或 RViz 线条；必须检查路径是否沿连续坡道/楼梯/平台
  支撑面前进，不能横跨楼梯、平台边缘或未建图空洞。验收应同时看 `/pct_path`
  高度、waypoint surface label、坡度/step 连续性、机器人支撑 footprint 余量和
  Gazebo/odom 实际高度。
- SLAM 建图和重定位是跨楼层硬约束：需要同时监测 `/Laser_map_world`、FAST-LIO
  odom、wheel `/odom` 与 Gazebo pose 的偏差；不能只因 FAST-LIO aligned odom
  接近目标就声称物理到达。

## 2026-05-16 跨层完成目标审计与外部资源发现

- 当前目标的完成条件必须拆成 5 个硬证据：live `/Laser_map_world` 增长、高层
  `/pct_path` max z > 2.0、direct execution 命令链 fresh、Gazebo/odom 实际 z
  上升到高层、并接近高层目标。前 3 项已有多轮证据，后 2 项仍未验收。
- 当前路径规划算法事实：不是 upstream PCT CUDA/RL，而是
  `terrain_pct_planner` 中的 PCT-style terrain graph planner。active runtime 从
  `/Laser_map_world` 构建 SLAM terrain graph，经 `/pct_path` 交给 direct terrain
  tracking，再进入 Nav2 smoother/collision safety chain。
- 单层展示已达到当前可复现最低目标：`near_goal_after_zwindow_snap_20260516_175830`
  证明 FAST-LIO SLAM map 增长、目标接收、direct tracking、命令链、Gazebo 实体
  到点均成立。`long_corridor` 仍是探索目标，不能作为展示验收。
- 外部 Unitree 资源只可作为后续模型替换输入，不能作为当前完成证据。官方
  `unitree_ros2` 面向 ROS 2 Humble/Go2 通信和 SDK 接入；官方 `unitree_ros`
  Gazebo 仿真包偏 ROS1，并提示 Gazebo 仿真不适合高层运动控制。结论：直接把
  surrogate 换成足式模型不会自动解决 Fortress 中的跨层物理爬升，必须先做接口
  适配和仿真控制验收。
- skills 检索结果：本机已有 `ros2-development`、`robotics-design-patterns`、
  `code-simplification` 等适配技能。外部 ROS2/robotics skill 安装量较低，暂不
  盲装；已安装高安装量 `getsentry/skills@code-simplifier` 到
  `~/.agents/skills/code-simplifier`，用于后续简化代码审查。
- 日志占用当前不大：`log` 约 54M、`build` 约 18M、`install` 约 2.9M。但
  `log/` 下有大量 colcon build 历史目录；若后续要清理空间，优先清旧 build
  log，而不是删除 handoff 证据或最新 runtime 证据。
- `cross_level_current_reprobe_20260516_181159` 重新确认当前跨层缺口：
  SLAM 增长、高层 `/pct_path` 和 direct 命令链都成立，但 Gazebo/wheel z 没有
  上升。不要再把主要精力放在 command timeout 或高路径是否生成上。
- 新的最小根因证据：direct log 在 `slam_step` 高节点 `(-2.72,0.63,0.82)`
  附近长时间 `cmd=(0.000,±0.450)` 旋转，随后释放并重规划到低 z `slam_step`
  序列。这表明路径语义允许“进入高 step 后又回落到低层 step/floor”的高度序列，
  这是会导致机器狗/替身摔落或无法真实爬升的路径错误。
- 已用 `_invalid_final_high_drop_node` 把该类路径作为 final high path 无效路径
  处理。该修复是 planner 安全约束，不是物理爬升完成证据；仍需下一轮 runtime
  验证是否改为保持在连续 ramp/stair/step 上，或暴露 surrogate 物理能力不足。
- high-drop guard 后的复验
  `log/cross_level_after_high_drop_guard_20260516_182151/` 说明该安全约束没有破坏
  高路径生成：`/Laser_map_world` 最大 `635571` 点，`/pct_path_max_z=2.206236`，
  三路命令链仍有大量消息计数。
- 同一复验也说明跨层完成条件仍未满足：wheel `/odom` z 为 `0.0`，Gazebo z
  约 `-0.005`，高层目标距离仍约 `11.67m`。这不是 command freshness 或高路径
  是否生成的问题。
- 最新强信号是 ramp-to-deck / high-deck execution：direct tracker 从低 ramp
  目标推进到 `target=(6.59,4.84,1.07) surface=slam_deck` 后，机器人仍在
  `y≈1.85, z≈0.34` aligned odom 附近徘徊并释放 stalled path。下一步应优先
  判断该 deck target 是否缺少连续坡道支撑、是否需要 floor-aware/3D goal
  约束，或当前 surrogate/contact 模型是否不能完成这段爬升。

## 2026-05-16 单层 long_corridor final-goal 语义修复

- `long_corridor_finalsnap_20260516_175457` 的失败根因不是命令链断路，而是
  direct tracker 只检查当前 graph endpoint 是否到达。该 endpoint 约为
  `(7.81,-9.55)`，离用户 final goal `(8.0,-9.0)` 仍超过 direct tolerance，
  因此不应被视为最终到达。
- 已新增 `direct_tracking_reaches_goal`：最终完成条件同时要求 tracked graph
  endpoint 和原始 final goal 都在 direct goal tolerance 内。该判断保持
  `append_direct_final_goal` 的安全边界，不允许 off-graph 追点。
- `long_corridor_goal_guard_20260516_184353` 通过说明这次修复没有制造假成功：
  运行中 pending final goal 后续随 FAST-LIO 地图更新变为可达，最终 direct path
  目标为 `(8.00,-9.00,-0.03)`，Gazebo/wheel 到用户目标 `0.266004m`，低于
  `0.30m` 验收阈值。
- 单层 FAST-LIO/PCT 展示现在有两级证据：`near_goal` 作为快速 smoke，
  `long_corridor` 作为更强的长低层路线展示。跨层物理爬升仍未验收，不能因此
  推断完整跨楼层自主导航完成。

## 2026-05-16 3D goal 与跨层准备发现

- 用户指出的 goal 工具风险已经进入工程约束：跨楼层目标不能再只靠 2D goal 或
  RViz 点击表达，因为它不能可靠标定目标楼层/高度。
- 已新增 `publish_terrain_goal`，用重复发布的 3D `PoseStamped` 表达
  `(x,y,z)` 目标；当前高层目标发布命令是
  `ros2 run airos_experiments publish_terrain_goal --x 6.0 --y 13.0 --z 2.2 --publish-count 5 --rate-hz 1`。
- `terrain_pct_planner` 已把高 `PoseStamped.pose.position.z` 作为目标楼层下界
  约束，避免高层任务被同 XY 的低层候选吸走。`cross_level_evidence_probe`
  现在记录 `goal_xyz`，后续 runtime 证据可明确区分“目标楼层”与“实际到达高度”。
- 该改动只解决目标表达和证据记录，不证明跨楼层物理爬升已完成。跨层验收仍必须
  同时看到高 `/pct_path`、连续坡道/楼梯/平台支撑路径、命令链 fresh、wheel
  `/odom` 和 Gazebo pose 实际上升。
- 路径安全边界保持不变：不得让路径横跨楼梯、平台边缘或未建图空洞。跨层阶段
  应检查 waypoint surface label、坡度/step 序列、support footprint margin、
  FAST-LIO 与 wheel/Gazebo pose 的偏差，避免 SLAM 建图或重定位漂移被误认为
  物理进展。

## 2026-05-16 3D goal runtime 后的跨层物理瓶颈

- 3D/floor-aware goal 工具已在 runtime 中生效：两轮跨层 headless 运行都记录
  `goal_xyz_last=[6.0,13.0,2.2]`，并再次生成高层 `/pct_path`。
- 180 秒运行 `cross_level_3d_goal_20260516_190046`：
  `/pct_path_max_z=2.275243`、`/Laser_map_world` 最大 `674569` 点、
  `/cmd_vel_nav=2513`、smoother `2845`、base command `2830`。但 wheel `/odom`
  z 为 `0.0`，Gazebo z 仍约 `-0.005001`。
- 300 秒运行 `cross_level_3d_goal_long_20260516_191019`：
  `/pct_path_max_z=2.344505`、`/Laser_map_world` 最大 `785990` 点，命令 age
  最大只有约 `0.14s/0.061s/0.071s`，说明 command freshness 已不是主要瓶颈。
  但 Gazebo z 最大仍约 `-0.004997`，wheel `/odom` z 仍为 `0.0`，最终目标
  XY 距离恶化到 `15.572861m`。
- 当前强推断：跨层未完成的主因更接近 physical execution/surrogate limitation
  或 ramp/contact 接触链，而不是 3D goal、SLAM map growth、高层 path generation
  或 command timeout。该推断仍需一个简单 ramp 物理 smoke 验证。
- 继续跨层阶段前必须保留两条安全约束：不得接受横跨楼梯/平台边缘/地图空洞的
  path；不得用 FAST-LIO aligned z 或 surface `gate_z` 替代 wheel/Gazebo 物理
  高度。SLAM 建图与重定位一致性应作为验收项，而不是事后解释项。

## 2026-05-16 ramp 物理通道与 SLAM 入口新证据

- 直接底盘命令 smoke 已证明修复后的 `large_multilevel_complex` lower ramp 可以让
  当前 `go2w_nav_eq` 替身物理爬升：
  `log/lower_ramp_physics_after_landing_fix_20260516_195621/summary.json`
  中 Gazebo z 从 `0.094382` 到最大 `0.934999`，Gazebo y 从 `-6.600012`
  到最大 `12.240317`，`accepted_upper_landing=true`。
- 该物理通道修复包含三点：`second_floor_deck` 不再覆盖下坡道并压住车体，
  `ramp_upper_landing` 后移到坡道末端附近，`terrain_pct_planner` 不再把
  含 `landing` 的平面承台按 ramp/slope 分类。
- wheel `/odom` z 仍为 `0.0` 是当前 2D odometry publisher 的预期结果；跨层物理
  验收必须以 Gazebo pose z 为主，除非后续把 odometry 改为 3D。
- 主链路复测仍未完成物理跨层：
  `cross_level_after_landing_fix_goal_ok_20260516_200724` 有
  `/Laser_map_world=542955`、`/pct_path_max_z=2.341184` 和 fresh command chain，
  但 Gazebo z 仍为 `-0.005001`；
  `cross_level_after_regressive_prefix_fix_20260516_201433` 有
  `/pct_path_max_z=2.124622`，Gazebo 最终约 `(4.04,1.37,-0.005)`。
- 当前更强推断：剩余瓶颈不是“替身完全不能爬坡”，而是 SLAM graph/frontier
  入口选择把机器人带到低层 `slam_step` 伪入口附近，而不是带到真实下坡道入口
  `x≈-4.7` 的 corridor。

## 2026-05-16 frontier ramp-entry 静态修复

- 新增 `test_slam_frontier_path_prefers_ramp_entry_over_isolated_step_pair`，
  复现运行日志里的高层 frontier 风险：低层/孤立 `slam_step` pair 位于目标走廊
  附近时，算法会被高层吸引点拉向错误低层走廊，而不是接近真实 lower ramp 入口。
- 根因不是高路径生成失败，也不是命令链断路，而是
  `_frontier_elevation_entry_attractor_xy` / entry-attractor scoring 在高层目标下
  对连续坡道入口约束过窄，并且有 entry attractor 时过度贴近低端入口，可能不
  继续沿可爬升节点推进。
- 修复后，高层 frontier entry scoring 优先连续 `ramp/stair` vertical progress，
  `step` 次之；同时对坡道/楼梯入口适度放宽 lateral corridor，并在 entry-attractor
  分支里优先保留实际高度进展。
- 静态验证：visual config、SLAM graph、terrain planner、control chain、
  cross-level probe 组合测试 `148 passed in 27.55s`；`git diff --check` 无输出；
  `colcon build --symlink-install` 完成 8 packages。
- Pending：该修复尚未 runtime 验证。下一轮跨层实验必须检查 first frontier/direct
  target 是否转向真实 lower ramp corridor、Gazebo pose z 是否持续上升、路径是否仍
  避免横跨楼梯/平台边缘/未建图空洞。

## 2026-05-16 frontier-entry 后 runtime 与低 ramp 前缀发现

- `log/cross_level_after_frontier_entry_fix_20260516_203725/` 复测显示：
  live SLAM 地图最大 `796963` 点，`/pct_path_max_z_max=2.064855`，三路命令
  age 最大约 `0.077s/0.062s/0.060s`。这再次证明高路径和命令链没有断。
- 同一运行仍未通过物理跨层验收：Gazebo z 最大仍为 `-0.005001`，最终到高层目标
  XY 距离 `22.014349m`。
- 该运行部分验证了上一静态修复方向：后期 frontier 出现 `(-5.93,0.55)`，更接近
  真实 lower ramp 一侧。但系统在此之前长时间执行低高度 `slam_ramp`
  `(6.02,-11.40,0.36)` 一类目标，方向明显背离高层目标，导致运行窗口被消耗。
- 已新增 `test_direct_tracking_drops_regressive_low_ramp_prefix_before_high_entry`，
  并让 direct tracking 丢弃低高度 ramp/slope 前缀中对高层 final goal 进展不足的
  waypoint，同时用 `test_direct_regression_drop_preserves_high_floor_detour` 保证真正
  的高层 ramp/deck detour 不被误删。
- 静态验证：visual config、SLAM graph、terrain planner、control chain、
  cross-level probe 组合测试 `149 passed in 27.56s`；`git diff --check` 无输出；
  `colcon build --symlink-install` 完成 8 packages。
- 后续又补了 final-path/frontier gate：active frontier 尚未抵达时不抢切 final
  high path；final high path 如果初段低层前缀明显远离最终高层目标，则继续
  exploration。对应新增
  `test_pending_final_goal_waits_for_active_frontier_endpoint` 和
  `test_high_final_path_rejects_large_initial_goal_regression`。
- 最新静态验证更新为：visual config、SLAM graph、terrain planner、control chain、
  cross-level probe 组合测试 `151 passed in 27.47s`；`git diff --check` 无输出；
  `colcon build --symlink-install` 完成 8 packages。
- `log/cross_level_after_final_regression_guard_20260516_210455/` 复测显示旧的
  远处高层 final target 抢切问题已被控制：launch log 记录系统发布
  `frontier=(-3.56,1.20)` 的 frontier path，并沿 `slam_ramp` 推进到
  `target=(-3.56,1.20,0.46)` 附近。
- 同一运行仍未通过物理跨层验收：Gazebo z 最大仍约 `-0.005`，最终到高层目标
  XY 距离 `15.872381m`；collision monitor 在坡道边缘附近触发 StopZone，随后
  frontier 被判定 stalled 并释放。
- 该 run 的 probe summary 中 `/pct_path` 样本为 0 是采样时序限制：probe 在 goal
  发布后启动，而 `/pct_path` 非 latched；不能据此判断 planner 没有发布路径。
- Fact：高路径生成、3D goal 表达、命令链 freshness 都不是当前最新瓶颈。
- Inference：下一步应集中在 ramp-center/support-margin scoring 或 `/slam_scan`
  StopZone 诊断，避免路径贴坡道边缘或跨未支撑区域；不应第一步关闭安全层。
- Pending：仍未证明机器人实际爬到高层平台，不能声称完整跨楼层导航完成。

## 2026-05-16 height-debt lookahead 与最新执行瓶颈

- 已补 direct tracking 的 height-debt lookahead 回归测试：
  `test_direct_tracking_lookahead_pushes_past_xy_reached_height_debt_step`
  和 `test_direct_tracking_height_debt_lookahead_does_not_cross_surface_change`。
  当前行为是：高 `slam_step` waypoint 的物理高度未达时，
  `advance_direct_target_index` 仍不推进索引；但若机器人 XY 已压在该点附近，
  命令 lookahead 可瞄向同一 surface 段的后续点，以避免目标距离近似 0 后
  线速度被压成 0。
- 已修 `cross_level_evidence_probe` 对 Gazebo pose 查询超时的脆弱性：
  `ign topic /pose/info` 超时时该样本写 `gazebo_xyz=None`，probe 不再整段退出。
  失败的空 `probe.jsonl` 不可作为导航结论。
- 最新静态门禁通过：`test_slam_scan_projector.py`、visual config、SLAM graph、
  terrain planner、control chain、cross-level probe 组合测试
  `162 passed in 46.38s`；`git diff --check` 无输出；
  `colcon build --symlink-install` 完成 8 packages。
- 最新运行 `log/cross_level_after_height_debt_lookahead_20260516_215319/`：
  `/Laser_map_world` sampled max `792874`，`/pct_path_max_z=2.178018`，三路命令
  计数 `/cmd_vel_nav=3101`、smoother `3502`、base `3496`，最终 command age
  `0.034s/0.032s/0.028s`。
- 同一运行仍未通过跨层物理验收：FAST-LIO aligned z 最高 `0.400256`，wheel
  `/odom` z 仍 `0.0`，Gazebo pose z 仍 `-0.005001`；Gazebo 到高层目标 XY 距离
  最小 `13.662475m`，最终 `13.720846m`。
- 该运行比上一轮推进更远：低 `slam_ramp` frontier 被执行并到达，后续生成
  `max z > 2.0` 的高层路径；但最后在 `slam_step` 段围绕
  `(-4.29,3.35,0.72)` 与 `(-3.97,4.46,0.74)` 一类目标摆动，heading error 大，
  direct 线速度反复为 0，最终 direct path stall/release。
- 当前推断：瓶颈已从 `/slam_scan` StopZone 和近距离 height-debt 死点，转移到
  step/ramp 段 lookahead heading 不稳定。下一步应做路径切向/前向进展约束或
  曲率门控，保留物理高度 gate，不应把未爬升的 surface `gate_z` 当作到达证据。

## 2026-05-16 path-tangent lookahead 后的控制链恢复与新阻塞

- 已补 step/ramp 段 path-tangent/forward-progress lookahead 约束，避免高
  height-debt 段选择位于路径切向后方的候选目标。
- `log/cross_level_after_tangent_lookahead_20260516_220735/` 不能用于评价该
  lookahead 的跨层效果：该 run 中 `/cmd_vel_nav` 与 `/cmd_vel_smoothed` 有消息，
  但 `/diff_drive_controller/cmd_vel_unstamped` 没有 publisher，base command 计数
  为 `0`。launch log 显示 `collision_monitor` lifecycle 服务在 activator 的单次
  等待窗口内不可用。
- 已修 `lifecycle_activator`：激活 lifecycle node 前会在限定次数内重试
  `get_state` 与 `change_state` 服务可用性。对应控制链测试已覆盖。
- 最新短跑 `log/control_chain_after_lifecycle_retry_20260516_221346/` 证明
  `safety_only` 模式下 base command 恢复：`/cmd_vel_nav=915`、smoother `1016`、
  base command `1015`，Gazebo 到目标距离从 `24.517079m` 降到 `18.280812m`。
- 同一短跑只应作为控制链恢复证据：`/pct_path_max_z` 样本最大只有 `0.475012`，
  Gazebo z 仍为 `-0.005001`，没有新的高层路径或物理爬升验收。
- 新阻塞：collision monitor 多次报告 `/slam_scan` 与当前 node time 相差约
  `1s` 并忽略 scan source。下一步应先定位 `slam_scan_projector` 处理成本、
  publish stamp、QoS/source timeout 的真实关系；不能把关闭 collision monitor
  当作修复。
- 根因已收窄并做第一修复：`slam_scan_projector` 的连续坡面支持过滤原来对每个
  候选点扫描完整 sampled cloud，复杂度近似 O(n^2)。在 `60000` 点配置下会拖慢
  rclpy 单线程回调和 scan 发布，触发 collision monitor source timeout。
- 修复方式：投影前一次性构建二维局部 support bins，坡面支持判断只查 0.9m
  邻域 bin。该改动不放宽 StopZone/SlowZone 几何，也不关闭 collision monitor。
- 回归验证：`test_slam_scan_projector.py` 新增源契约，防止 supported-ramp filter
  回退到 per-point full-cloud scan；完整目标静态门禁更新为 `165 passed in 27.46s`，
  `git diff --check` 无输出，`colcon build --symlink-install` 完成 8 packages。
- 短跑验证：
  `log/scan_freshness_after_support_index_20260516_222625/summary.json` 记录
  `slam_scan_stale_warn_count=0`、base command `1033`、`/cmd_vel_nav=918`，
  最终 command age `0.020s/0.039s/0.039s`。Gazebo XY 向目标移动，但
  `/pct_path_max_z=0.470178`、Gazebo z 仍 `-0.005001`。
- 当前结论：控制链和 `/slam_scan` freshness 已恢复到可继续长窗跨层复测的状态；
  物理高层到达仍未证明。

## 2026-05-16 单层展示链复核

- `scripts/run_fast_lio_single_floor_demo.sh` 复核 `near_goal` 通过：
  `log/fast_lio_single_floor_demo/near_goal_after_scan_index_20260516_223232/`
  记录 `accepted=true`，`/Laser_map_world` 最大 `248275` 点，三段命令计数
  `/cmd_vel_nav=200`、smoother/base `235`，最终 wheel/Gazebo 到目标距离均为
  `0.222402m`。
- 同一 `near_goal` run 的 launch log 记录收到 terrain goal、启动 direct tracking
  并 `terrain direct tracking goal reached`；`/slam_scan` stale warning 计数为 0。
- `long_corridor` 复核也通过：
  `log/fast_lio_single_floor_demo/long_corridor_after_scan_index_20260516_223440/`
  记录 `accepted=true`，`/Laser_map_world` 最大 `351188` 点，`/pct_path_poses=10`，
  `/pct_path_max_z=-0.044546`，三段命令计数 `/cmd_vel_nav=573`、smoother `645`、
  base `660`，最终 wheel/Gazebo 到目标距离均为 `0.294605m`。
- `long_corridor` launch log 记录 `pending final goal became reachable after
  FAST-LIO map update` 后再次启动 direct tracking 并到达目标，说明不是只靠近距离
  目标或初始局部路径通过。
- 当前可展示基线：单层 FAST-LIO2 map growth -> PCT-style `/pct_path` -> direct
  execution -> smoother/collision monitor/base -> Gazebo pose 到达目标已经复核通过。
- Pending：跨楼层仍未完成；不能把单层通过外推成高层平台到达。

## 2026-05-16 跨层阶段收敛结论

- `cross_level_after_single_floor_refresh_20260516_223943`：
  live SLAM map 最大 `793960` 点，`/pct_path_max_z=2.138636`，命令链
  `/cmd_vel_nav=2977`、smoother `3352`、base `3337`，`/slam_scan` stale warning
  为 0；但 Gazebo z 仍 `-0.005001`，到目标距离最小/最终约
  `12.201398m/12.321149m`。
- 针对该 run 中 same-surface high-debt step lookahead 前后抖动，已新增
  `test_direct_tracking_height_debt_lookahead_ignores_following_zigzag` 并修复
  `_candidate_is_behind_path_tangent` 的方向判定。静态门禁更新为
  `166 passed in 28.19s`，`git diff --check` 无输出，`colcon build` 完成
  8 packages `[0.94s]`。
- 修复后短窗跨层 run `cross_level_after_zigzag_lookahead_fix_20260516_225624`
  仍未爬升：`/Laser_map_world=621554`，`/pct_path_max_z=2.249888`，base command
  `2686`，stale warning 0；Gazebo z 仍 `-0.005001`，最终距离约
  `11.985970m`，并释放 stalled direct path。
- 新失败点已从 step 目标前后抖动转为低 `slam_ramp` 目标附近物理不前进：
  日志长期停在 `target=(5.12,1.43,0.46)`、`robot≈(4.37,1.42,0.33)`，
  命令约 `(0.128,-0.355)` 但 Gazebo 位置几乎不变。
- 当前工程决策：不要继续在同一轮式 surrogate + 同一跨层地图上反复长跑。最快
  展示路线是交付单层 FAST-LIO/PCT/direct 闭环；跨层改为单独阶段，优先考虑
  更适合物理爬升的模型/控制或更小的可验证 multilevel demo map。

## 2026-05-16 realistic_multilevel_ramp smoke 结论

- Fact: `scripts/run_fast_lio_multilevel_smoke.sh` 已建立为跨层短窗验收入口，
  默认世界为 `realistic_multilevel_ramp`，目标为 `(0.4,3.6,0.65)`，验收字段包括
  `/pct_path_max_z`、`gazebo_z_max`、base command count 和 Gazebo goal distance。
- Fact: 远端 upper-lab 目标短跑
  `quick_multilevel_smoke_20260516_232258` 有 live SLAM map 和命令链，但
  `/pct_path_max_z=0.235028`、`gazebo_z_max=0.335047`，未达到脚本阈值。
- Fact: 坡道入口目标短跑
  `ramp_entry_smoke_20260516_232735` 中 planner 收到 `(0.4,3.6,0.65)` 并启动
  direct tracking，命令链达到 `/cmd_vel_nav=801`、smoother `861`、base `874`，
  `/Laser_map_world=291189`；但 Gazebo z 最大仅 `0.060886`，最终距离 `4.120866m`。
- Fact: 该 run 的 launch log 显示路径首段目标偏向
  `(2.20,-3.05,-0.14)` 等 `slam_floor` 节点，而不是沿真实 `wide_access_ramp`
  中心线进入夹层。
- Fact: `ramp_corridor_guard_20260516_233809` 修正了 probe/goal 顺序并采到
  `/pct_path_max_z=0.984888`、`/pct_path_poses=33`，命令链也持续传播：
  `/cmd_vel_nav=697`、smoother/base `769`。
- Fact: 同一 run 仍未物理爬升，`gazebo_z_max=0.067521`，最终目标距离
  `5.047544m`。launch log 显示第一次可达高路径被
  `deferred pending final goal because the reachable high path initially
  regresses away from the goal` 拦住，但下一次接收目标后仍启动了一条
  `path_nodes=33` 的 direct path，首目标为低层
  `(2.20,-3.05,-0.14)` `slam_floor`。
- Inference: 当前跨层失败可先在 `realistic_multilevel_ramp` 上复现和修复，
  重点是 SLAM-cloud path selection / ramp-corridor entry，而不是继续消耗
  `large_multilevel_complex` 长窗运行。
- Pending: 需要把低层远端前缀拒绝逻辑覆盖到初始 final-goal 接受路径。当前 guard
  已能拦住 pending retry 中的一种坏路径，但还没有覆盖重复 goal 后的直接接受分支。
