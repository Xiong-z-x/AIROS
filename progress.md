# AIROS 深度升级进度记录

> 状态：历史进度记录。当前迁移前封板资料见 `docs/handoff/`。

## 2026-05-09

- 建立本轮深度升级规划文件：`task_plan.md`、`findings.md`、`progress.md`。
- 已确认当前 `main` 和 `origin/main` 均位于 `3ce7c1e feat: use native gazebo lidar sensors`。
- 已完成在线初筛：`3d_dog_navi_ros2` 可作为参考，但不能直接替换当前运行时；Nav2 route server 是 route-constrained 加固的主线。
- 写入失败测试 `test_deep_upgrade_artifacts.py`，初始 4 项失败：高级 world/map/profile/runner 参数均不存在。
- 已新增 `advanced_indoor_ramp.sdf`，并用 `world_map_generator.py` 生成 `advanced_indoor_ramp.yaml/.pgm`。
- 已新增 `advanced_indoor_ramp_missions.yaml`、`advanced_indoor_ramp_route.geojson` 和 `nav2_research_profile.yaml`。
- 已扩展 `sim.launch.py`、`nav.launch.py`、两个 visual launch 和 `clean_batch_runner.py`，让高级世界、物理动态障碍、地图、route、planner profile 可透传。
- `python3 -m pytest src/airos_experiments/test/test_deep_upgrade_artifacts.py -q` 已通过：4 passed。
- 组合静态测试已通过：`test_deep_upgrade_artifacts.py`、`test_visual_pointcloud_config.py`、`test_control_command_chain.py`、`test_scan_emulator_anchor.py` 共 28 passed。
- `colcon build --symlink-install --packages-select airos_sim airos_nav airos_experiments` 已通过。
- 高级世界 runtime smoke 已通过：`sim.launch.py world:=advanced_indoor_ramp sensor_source:=native physical_dynamic_obstacles:=true gui:=false rviz:=false` 启动成功；`/scan` 有 ranges，初版 `/livox/lidar` PointCloud2 width=720，Gazebo topics 包含 `/model/moving_pedestrian/cmd_vel` 与 `/model/inspection_cart_dynamic/cmd_vel`，两个 ros2_control controller active。
- `scripts/cleanup_airos_runtime.sh` 清理后返回 `[PASS] AIROS runtime processes cleaned.`。
- 修复 `verify_route_graph`：不再依赖易卡死的外部 `ros2 lifecycle get /route_server` 轮询，改为日志/action 可用性判定。
- 高级 route graph 验证通过：`verify_route_graph --graph src/airos_nav/routes/advanced_indoor_ramp_route.geojson --start-id 1 --goal-id 3 --log-dir log/advanced_route_graph_verifier` 输出 `route graph verification passed`。
- `test_flake8.py` 已收窄到源码目录，避免 build/install/生成消息和第三方 FAST-LIO launch 污染；本轮 flake8 通过。
- 完成用户要求的 Livox CustomMsg 链路修正：Gazebo `/livox/lidar/points` 先桥接为 ROS `/livox/lidar_points` PointCloud2，`livox_custom_bridge` 再发布 `/livox/lidar` Livox `CustomMsg`，FAST-LIO `airos_sim.yaml` 改为 `lidar_type: 1`。
- Runtime 证据：`ros2 topic type /livox/lidar` 返回 `livox_ros_driver2/msg/CustomMsg`，`/livox/lidar_points` 返回 `sensor_msgs/msg/PointCloud2`；`ros2 node info /fastlio_mapping` 显示订阅 `/livox/lidar: livox_ros_driver2/msg/CustomMsg`；FAST-LIO 日志显示 `p_pre->lidar_type 1` 和 `Initialize the map kdtree`。
- 修复 research profile 初始 bringup 问题：MPPI `model_dt` 与 10Hz controller 周期对齐，`controller_server`、`planner_server`、`bt_navigator`、`route_server`、`map_server` 最终均为 `active [3]`。
- 修复 `livox_custom_bridge` Ctrl-C 关停二次 shutdown；最终 launch 关停时该进程 cleanly exited。
- 修复 runtime 清理脚本：补杀 `static_transform_publisher`，并停止 ROS daemon 清理残留 graph 缓存。
- 最终验证通过：`colcon build --symlink-install --packages-select airos_experiments airos_nav airos_sim fast_lio livox_ros_driver2`；`python3 -m pytest ...` 32 passed；`git diff --check` 通过。
- 补齐完成度审计弱项：导入 `ypat999/3d_dog_navi_ros2` 的 AFL 3.0 Building / Go2W / Mid360 视觉资产为可选 profile，保留当前等效体碰撞和控制链。

- Added `generate_advanced_planner_candidates` as a concrete Nav2/PCT-style/RL-style route candidate artifact for advanced planner comparison while keeping PCT/RL marked as research surrogates.

## 2026-05-11

- 恢复当前 AIROS PCT/FAST-LIO/Nav2 仿真上下文，清理残留进程。
- 复现斜坡下坡目标：PCT 能生成三维路径，但 FollowPath 在坡底被 RPP/collision/路径剪空中止，机器人会过冲到 y≈-3 后原地转向。
- 已锁定根因之一：`physical_dynamic_obstacles:=false` 时原 world 仍残留 Gazebo 动态模型，默认路径会被未启用障碍影响。新增失败测试并准备静态 world。
- 决策更新：后续默认演示链路改为 PCT 负责三维全局路径和局部跟踪，Nav2 只保留 velocity smoother + collision_monitor safety-only，不再让 Nav2 FollowPath 控制器解释坡底急转路径。

## 2026-05-15

- 当前目标切换为迁移前封板与交接打包，不继续扩展新模块。
- `scripts/cleanup_airos_runtime.sh` 已运行并返回 `[PASS] AIROS runtime processes cleaned.`。
- 已建立 `docs/handoff/` 交接包，包含当前状态、风险清理、迁移前技术路线、下一模型警示和 copy-ready 初始化 prompt。
- 当前事实：live FAST-LIO2 `/Laser_map_world` 已能生成高层 `/pct_path`，在线证据包括 `PATH_MAXZ_OVERALL=2.155`、点云从 69075 增长到 594777、命令链与 odom 链路持续有效；物理到达高层平台仍未验收。
- 速度链已上调但仍保持保守：direct max linear `0.30`，flat limit `0.32`，slope limit `0.16`，controller linear max `0.32`，angular max `0.55`。

## 2026-05-16

- 单层 Nav2 clean batch 已验收：`log/single_floor_baseline_20260516.jsonl`
  中 4 个 `single_floor_lab` mission 均 `success: true`，且 emergency stop /
  collision 计数为 0。
- 已新增跨层只读诊断工具 `cross_level_evidence_probe`，并在 direct terrain
  tracker 中增加 index、目标 xyz、机器人 xyz、surface、speed limit、cmd 等日志。
- 360 秒跨层 headless 复现实验保存在
  `log/cross_level_full_20260516_113311/`：`/pct_path` 再次达到
  `pct_path_max_z: 2.138301`，base controller command 在 37/72 个采样中非零。
- 同一实验未通过物理跨层验收：最终 Gazebo pose z 仍为 `-0.005001`，
  `gazebo_goal_xy_distance` 仍为 `11.887684`。
- 当前下一步不是重做高层 path generation，而是定位 direct tracking 高层
  waypoint 进展与 Gazebo 物理高度不一致的根因。
- 已修复并覆盖一个真实图连边风险：SLAM sparse bridge 现在拒绝低层
  `slam_floor` 到高 `slam_deck` 的非物理稀疏跳连；新增
  `test_slam_graph_rejects_sparse_floor_to_deck_edge_jump`。
- 已新增 direct tracking `gate_z` 诊断/门控，避免仅凭 aligned odom z 判断
  高层 waypoint 进展。
- 最新 360 秒运行 `log/cross_level_bridge_guard_20260516_122021/` 中高路径仍
  可生成，`pct_path_max_z: 2.065411`，第一高目标已变为 `slam_ramp`
  `(-2.39,0.83,0.73)`；Gazebo z 曾上升到 `0.217646`，但最终仍未到高层目标。
- 已解析该运行的 ramp/high 时间窗：坡道段 direct speed limit 为 `0.16`，
  collision monitor 多次触发 SlowZone；5 秒 probe 中 `/cmd_vel_nav` 经常非零，
  但 `/cmd_vel_smoothed` 和 base command 多次为 0。该现象仍需用消息 count/age
  排除采样假象。
- 已增强 `cross_level_evidence_probe`，新增三路命令的 message count 和 age
  字段，为下一次运行判断 smoother/collision/base 命令是否持续掉线做准备。
- 增强运行 `log/cross_level_cmd_age_20260516_124501/` 复现了高路径
  `pct_path_max_z: 2.074105`，但 Gazebo z 未上升；同时捕获到
  `/cmd_vel_nav_age_sec` 最大 `3.91s`，超过 velocity smoother `1.0s`
  timeout，说明 direct command 发布会被阻塞/饿死。
- 已将 FAST-LIO SLAM graph rebuild 改为后台单线程执行，并用
  `MultiThreadedExecutor` 运行 `terrain_pct_planner`，目标是避免图重建阻塞
  direct control timer。
- 修复后 180 秒验证 `log/cross_level_async_rebuild_20260516_125937/`：
  高路径 `pct_path_max_z: 2.188031`，高路径阶段 command age 未超过 `1.0s`，
  但 Gazebo z 仍保持约 `-0.005001`。命令链饥饿问题已改善，物理跨层仍未完成。
- 新的直接瓶颈：tracker 能继续推进到高 `slam_deck` 目标
  `(6.20,5.97,1.20)`，但 Gazebo 物理高度未上升，说明下一步应处理高 waypoint
  物理高度/contact gating 或 ramp-only entry。
- 已新增并通过
  `test_direct_tracking_holds_high_deck_waypoint_until_physical_height_progress`；
  高 `slam_deck` waypoint 现在使用更严格的 z 到达容差，避免物理/gate 高度仍
  在地面附近时继续推进 deck waypoint。
- 修复后 runtime `log/cross_level_deck_gate_20260516_130933/`：高路径仍生成
  `pct_path_max_z: 2.068655`，命令 age 全程低于 `1.0s`，但 Gazebo z 仍未上升。
  false deck progression 已被压住，剩余瓶颈转为 ramp/step 物理入口或模型能力。
- 已完成该运行的 SDF 几何对照：最终高层 goal 在真实
  `third_floor_deck` / `upper_ramp_landing` 上，但中间目标
  `(6.03,3.20,0.71)` 靠近 `lab_table_10` 顶面/边缘，不是可接受 ramp/stair/deck
  入口。
- 已用 TDD 修复一个 frontier 选择风险：新增
  `test_slam_frontier_path_rejects_isolated_obstacle_step_ahead`，并在 high-floor
  frontier 中过滤可替代候选存在时的低高度孤立 `slam_step` 障碍端点。
- 静态目标门禁已通过：`python3 -m pytest` 覆盖 visual config、SLAM graph、
  terrain planner、control command chain、cross-level evidence probe，结果
  `127 passed in 33.55s`。
- obstacle-like `slam_step` frontier guard 后的 runtime
  `log/cross_level_frontier_step_guard_20260516_133146/`：高路径仍生成，
  `pct_path_max_z: 2.187763`；命令 age 保持低位，`cmd_vel_nav_age_sec`
  最大 `0.084s`、smoother/base command age 最大约 `0.065s`；但 Gazebo z
  仍未上升，最终为 `-0.005001`，最终目标 XY 距离 `12.672001m`。
- 该运行的新瓶颈是 final high path transition：系统卡在
  `(0.64,1.52,0.74)` `slam_step` 附近，SDF 对照显示这是
  `second_floor_deck` 边缘/下方高度特征，不是可接受 ramp/stair 入口。
- 已按 TDD 新增
  `test_final_high_path_rejects_deck_edge_step_without_ramp_approach`，并在
  `plan_terrain_path` final candidate 选择中跳过缺少 ramp/stair/step 连续爬升
  证据的高 `slam_step` / `slam_deck` 入口候选。
- 补丁后静态验证通过：terrain/SLAM graph 聚焦测试 `96 passed in 32.90s`；
  visual config、SLAM graph、terrain planner、control chain、cross-level probe
  总门禁 `128 passed in 33.65s`；`git diff --check` 通过；
  `colcon build --symlink-install` 完成 8 packages。
- runtime `log/cross_level_final_transition_guard_20260516_140148/`：live map
  增长到 `791963` 点，`pct_path_max_z: 2.630439`，命令 age 基本低于 timeout。
  Gazebo z 出现短暂物理爬升，最高 `0.154583`，但最终回到
  `-0.005001`，最终目标 XY 距离 `10.592881m`。这不是跨层到达验收，只能说明
  final transition guard 推进了问题边界。
- 新的未完成项：让机器人持续保持 ramp/ascent progress，并处理后续
  `(1.18,2.44,0.77)` 一类 high `slam_step` entry，而不是回到已经覆盖的
  `(0.64,1.52,0.74)` deck-edge shortcut。
- 已用 TDD 暴露并修正 `lower_access_ramp` SDF 几何缺陷：原坡道没有物理连续
  连接 lower/upper landing。`large_multilevel_complex.sdf` 与 static 版本现在
  使用 `lower_access_ramp` pose `z=0.440`、pitch `-0.0678`。
- 已补 ramp-to-ramp 过渡约束：不同 ramp-like surface 之间也要求
  ramp-entry corridor，避免 landing 边缘斜切 ramp。补丁后门禁为
  `130 passed in 33.37s`，`git diff --check` 通过，
  `colcon build --symlink-install` 完成 8 packages。
- ramp 几何修正后的 runtime
  `log/cross_level_ramp_geometry_20260516_155258/`：`/pct_path` 仍可达到
  `pct_path_max_z: 2.212123`，第一次高路径在 `elapsed_sec: 40.21` 出现；
  命令 age 均低于 `0.1s`。机器人在平面 XY 上推进到更靠近目标的位置
  (`gazebo_goal_xy_distance` 最小 `11.624816m`)，但 Gazebo z 全程仍为
  `-0.005001`，最终目标距离变为 `16.235911m`。
- 当前最新瓶颈：direct tracking 的 surface `gate_z` 能随 SLAM ramp/deck/step
  表面变化，但 Gazebo 物理高度没有上升。下一步不应回退高路径生成或重调
  command timeout，而应写测试区分 surface height gate 与 physical height
  progress，尤其是高 `slam_deck`/`slam_step` waypoint advancement。
- 已完成该最小修复的 TDD RED/GREEN：新增
  `test_high_surface_gate_does_not_prove_physical_height_progress` 和源码契约测试，
  并实现 `direct_tracking_progress_z`。高 `slam_deck`/`slam_step` 节点现在用
  robot z 判定 waypoint 进展，避免 surface `gate_z` 单独证明物理爬升。
- 修复后聚焦 terrain planner 测试为 `35 passed in 3.73s`；visual config、
  SLAM graph、terrain planner、control chain 与 cross-level probe 门禁为
  `131 passed in 39.82s`。该结果只证明静态逻辑约束有效，不证明 Gazebo 已
  爬到高层。
- 当前未完成项：让机器人持续沿 ramp/stair 物理上升并到达高层目标。
- 用户最新优先级已调整：先尽快形成可展示的复杂单层导航效果和
  SLAM/规划/执行闭环；算法升级和跨层物理爬升降为后续。不能继续把所有
  runtime 时间投入跨层坡道物理爬升。
- 为支持单层 FAST-LIO/PCT 展示，`visual_fast_lio_navigation.launch.py` 新增
  `terrain_goal_z_policy` launch 参数，默认仍为 `highest` 保持跨层目标行为；
  单层展示应显式使用 `terrain_goal_z_policy:=nearest_z terrain_goal_min_z:=-1.0`。
- 快速稳定展示线已复验：
  `log/fast_show_single_floor_clean_batch_20260516.jsonl` 中
  `lab_door_passage` clean batch 成功，`success: true`，耗时 `15.303s`，
  路径 `2.664m`，`emergency_stop_count: 0`，`collision_count: 0`，
  最小障碍距离 `1.078m`。
- FAST-LIO/PCT 单层展示线已有部分证据：
  `log/single_floor_fast_lio_demo_20260516_1637/probe_goal_8_-9.jsonl`
  中 `/Laser_map_world` 从 `449105` 点增长到 `599135` 点，低层
  `/pct_path` 最大高度约 `0.226738`，`/cmd_vel_nav`、`/cmd_vel_smoothed`
  和 base command 计数持续增长，三路 command age 均低于 `0.1s`。
- 同一 FAST-LIO/PCT 单层运行未形成可靠物理到达：Gazebo 目标距离仅从
  `3.984403m` 降到 `3.937942m`，说明低层 `/pct_path` 和命令链存在，但 direct
  执行在该局部目标附近几乎没有物理推进。短期展示应优先使用稳定 Nav2
  clean batch；FAST-LIO/PCT 单层线作为前沿链路展示和下一轮执行调试对象。
- `scripts/cleanup_airos_runtime.sh` 已补杀
  `nav2_map_server/map_saver_server`，避免 clean batch 后遗留 map saver /
  Nav2 lifecycle 进程污染下一次实验；新增
  `test_cleanup_script_stops_nav2_map_saver_leftovers` 覆盖该风险。
- FAST-LIO/PCT 单层近目标 smoke 已补齐：
  `log/single_floor_fast_lio_demo_20260516_1652/probe_goal_1p9_-9p2.jsonl`
  中 `/Laser_map_world` 从 `267848` 点增长到 `401742` 点，低层
  `/pct_path` 最大高度约 `-0.073576`，三路命令计数分别达到
  `/cmd_vel_nav=228`、`/cmd_vel_smoothed=263`、base command `263`。
- 同一运行中 Gazebo pose 从 `[0.0, -10.0, -0.005001]` 移动到
  `[1.719182, -9.775888, -0.005001]`，目标 `(1.9,-9.2)` 距离从 `2.061553m`
  降到 `0.603608m`；launch 日志记录 `terrain direct tracking goal reached`。
  这可作为 FAST-LIO/PCT 单层展示 smoke，但仍不是精确到点验收，因为最终
  Gazebo 目标距离大于 direct goal tolerance `0.30m`。
- 已用增强 probe 复核该误差来源：
  `log/single_floor_fast_lio_demo_20260516_1703/probe_goal_1p9_-9p2.jsonl`
  显示 direct tracker 使用的 `/fast_lio_odom_world` 已接近目标
  `0.125m`，但 wheel odom / Gazebo 仍约 `0.569m`。这说明旧 smoke 的
  `goal reached` 主要由 FAST-LIO aligned odom 判定，不能直接等同实体精确
  到点。
- 为快速单层展示新增 `terrain_odom_topic` launch 参数，默认仍为
  `/fast_lio_odom_world` 以保留当前 FAST-LIO 主线；展示验收可显式传
  `terrain_odom_topic:=/odom`，让 terrain planner 与 `/slam_scan` 投影使用
  实体 odom 作为位姿基准。
- `/odom` 单层复跑已通过近目标实体收敛：
  `log/single_floor_fast_lio_demo_20260516_odom/launch.log` 记录 graph 从
  `1129/11504` 增长到 `1462/16596` 左右，目标连发后出现
  `received terrain goal`、`started terrain-guided direct tracking`、
  `poses=6 path_nodes=7`、持续 direct diagnostics，并在日志行 219 出现
  `terrain direct tracking goal reached`。
- 同一 `/odom` 展示运行的 probe
  `log/single_floor_fast_lio_demo_20260516_odom/probe_goal_1p9_-9p2_after_pub.jsonl`
  记录 `/Laser_map_world` 增长到 `631671` 点，最终 wheel odom /
  Gazebo 到目标 `(1.9,-9.2)` 的 XY 距离均为 `0.214862m`，低于当前
  direct goal tolerance `0.30m`。这是目前最适合快速展示的 FAST-LIO/PCT
  单层闭环证据。
- 该运行还有一个操作坑点：首次 `ros2 topic pub --once` 在 planner 已启动但
  graph 仍初始化/ROS discovery 未稳定时未被 terrain planner 处理；后续用
  `--times 5 --rate 1` 连发目标后才可靠触发目标回调。展示脚本应避免只发
  一次 volatile goal。
- 已将当前最佳单层 FAST-LIO/PCT 展示链封装为
  `scripts/run_fast_lio_single_floor_demo.sh`。脚本默认清理运行态、启动
  `visual_fast_lio_navigation.launch.py` headless，使用
  `terrain_goal_z_policy:=nearest_z terrain_goal_min_z:=-1.0 terrain_odom_topic:=/odom`，
  连发 `/terrain_goal_pose`，运行 `cross_level_evidence_probe`，并用 launch log
  中的 `received terrain goal` / `started terrain-guided direct tracking` /
  `terrain direct tracking goal reached` 加 probe 的 Gazebo 距离共同判定。
- 脚本真实 smoke 已通过：
  `log/fast_lio_single_floor_demo/smoke_20260516_172800/` 中 launch log 记录
  graph rebuild、direct tracking diagnostics 和 goal reached；probe summary
  显示 `accepted: true`、`laser_map_points_max: 198858`、
  `cmd_vel_nav_count_max: 202`、wheel/Gazebo 到目标距离均为 `0.20666m`。
- 脚本 dry-run 已修复并通过，避免在 `--dry-run` 时因重定向和缺失 probe 文件
  误失败；ROS setup sourcing 也已在脚本中临时关闭 nounset，避免
  `AMENT_TRACE_SETUP_FILES` 未定义错误。
- `scripts/run_fast_lio_single_floor_demo.sh` 已增加 `DEMO_TARGET` 命名目标：
  `near_goal` 为默认已验收目标，`long_corridor` 为探索性长低层目标，`custom`
  允许手动传 `GOAL_X/GOAL_Y/GOAL_Z`。
- `long_corridor` 探索运行
  `log/fast_lio_single_floor_demo/long_corridor_20260516_173459/` 暴露了下一步
  单层规划问题：系统生成低层 `/pct_path`，`pct_path_poses_max=11`、
  `pct_path_max_z=-0.027427`，命令链有效，Gazebo 目标距离从 `7.841325m`
  降到最小 `0.313592m`，但略高于验收阈值 `0.30m`，脚本正确返回
  `accepted: false`。
- 同一运行的 launch log 显示早期 `target_z≈1.237`，说明 `nearest_z` 在
  `long_corridor` 这类目标上仍可能短暂吸附到高结构候选，后续才切回低层
  path。下一步应增加单层 demo 的目标候选 Z 偏差约束或专用 low-floor
  profile，而不是直接放宽 acceptance tolerance。
- 已增加单层 goal z-window：`terrain_pct_planner.plan_terrain_path` 支持
  `goal_max_z`，`visual_fast_lio_navigation.launch.py` 暴露
  `terrain_goal_max_z`，`scripts/run_fast_lio_single_floor_demo.sh` 默认使用
  `TERRAIN_GOAL_MAX_Z=0.45`。新增
  `test_terrain_planner_goal_max_z_keeps_single_floor_goal_low` 覆盖“同 XY
  高层候选存在时单层 profile 保持低层目标”。
- `long_corridor` z-window 复跑
  `log/fast_lio_single_floor_demo/long_corridor_zwindow_20260516_174832/`
  证明层选择问题已改善：launch log 首个目标为 `target_z=0.45`，
  probe 记录 `pct_path_max_z=-0.030729`、`cmd_vel_nav_count_max=523`，
  wheel/Gazebo 最终距离为 `0.302173m`。该结果仍略高于 `0.30m`，脚本正确
  返回 `accepted: false`。
- 为避免把验收阈值放宽成假通过，direct tracker 增加安全末端收口函数
  `append_direct_final_goal`：只有用户目标离最后一个 terrain graph 节点不超过
  direct goal tolerance 时，才追加同表面的精确终点；若目标离已建图支撑区域过远，
  不外推路径。相关测试覆盖近距离追加和远距离拒绝。
- `long_corridor` 末端收口复跑
  `log/fast_lio_single_floor_demo/long_corridor_finalsnap_20260516_175457/`
  仍未通过：`accepted: false`，最终 wheel/Gazebo 距离 `0.71327m`。
  launch log 显示最终低层节点约 `(7.81,-9.55,0.00)`，离用户目标
  `(8.0,-9.0)` 超过 snap 安全半径，因此正确没有外推到目标点。该目标继续作为
  后续长单层优化对象，不作为当前展示验收目标。
- 默认 `near_goal` 展示目标已在新逻辑后复验通过：
  `log/fast_lio_single_floor_demo/near_goal_after_zwindow_snap_20260516_175830/`
  中 summary 为 `accepted: true`，`laser_map_points_max=220390`，
  `cmd_vel_nav_count_max=173`，wheel/Gazebo 到 `(1.9,-9.2)` 的最终距离均为
  `0.223881m`，launch log 记录 `received terrain goal`、
  `started terrain-guided direct tracking` 和 `terrain direct tracking goal reached`。
- 已开始阶段 10：按用户“彻底完成跨层证据链，但先确认单层”要求做 completion
  audit。结论是目标尚未完成：单层 `near_goal` 已可展示，`long_corridor` 未验收；
  跨层仍缺 Gazebo/odom 实际上升和高层到达证据。
- 已按 `$find-skills` 检索技能。ROS2/robotics 外部技能安装量偏低，暂不引入；
  已安装 `getsentry/skills@code-simplifier` 到 `~/.agents/skills/code-simplifier`，
  作为后续代码简化补充。本机已有 `ros2-development`、`robotics-design-patterns`、
  `code-simplification` 可继续使用。
- 已记录模型替换边界：可查到的官方 Unitree 资料支持 Go2/ROS2 通信接入，但不能
  证明 Fortress 中直接换足式模型即可完成跨层爬升。后续若换模型，应作为独立分支
  先验收仿真控制接口和坡道/楼梯能力。
- 当前跨层复现实验已完成：
  `log/cross_level_current_reprobe_20260516_181159/` 中 `/Laser_map_world`
  从 `156487` 增长到 `655392` 点，`/pct_path_max_z` 达到 `2.070835`，
  三路命令计数分别达到 `/cmd_vel_nav=2530`、`/cmd_vel_smoothed=2829`、
  base command `2829`，命令 age 最大分别约 `0.261s/0.064s/0.073s`。
  但 Gazebo z 仍固定在约 `-0.005001`，wheel `/odom` z 仍为 `0.0`，未证明物理
  爬升；目标距离只降到 Gazebo `11.168999m`。
- 本次 log 将瓶颈进一步缩窄：direct diagnostics 在高目标路径中出现
  `slam_step` 节点 `(-2.72,0.63,0.82)` 附近长时间旋转/释放，随后重规划到包含
  低 z `slam_step` 节点的路径。这说明当前问题不是高路径生成或命令链 freshness，
  而是高目标路径中存在高 step 后回落到低 step/floor 的不安全高度序列。
- 已新增并通过
  `test_final_high_path_rejects_step_drop_after_high_entry`，并在
  `terrain_pct_planner` 中增加 `_invalid_final_high_drop_node`。高目标路径进入
  高层通道后若出现大幅回落，会屏蔽该节点并重算路径。聚焦测试和
  `test_slam_traversability_graph.py + test_terrain_pointcloud_planner.py` 已通过：
  `104 passed in 31.15s`。
- high-drop guard 后已完成一次 bounded runtime 复验：
  `log/cross_level_after_high_drop_guard_20260516_182151/` 中
  `/Laser_map_world` 从 `156430` 增长到 `635571` 点，
  `/pct_path_max_z` 达到 `2.206236`，第一次高路径在 `elapsed_sec=85.108`
  出现；三路命令计数分别达到 `/cmd_vel_nav=2491`、
  `/cmd_vel_smoothed=2759`、base command `2658`。
- 同一运行仍未通过物理跨层验收：FAST-LIO aligned z 最高约 `0.343199`，
  wheel `/odom` z 仍为 `0.0`，Gazebo z 仍约 `-0.005`，Gazebo 到高层目标
  最终距离约 `11.67m`。因此不能声称跨层导航完成。
- high-drop guard 已消除一类“高层进入后回落低层 step”的路径语义缺陷，但
  新瓶颈转移到 ramp-to-deck / high-deck waypoint 执行。direct diagnostics
  显示系统可推进到 `index=15/25 target=(6.59,4.84,1.07) surface=slam_deck`
  附近，机器人仍停留在约 `(6.3,1.85,0.34)` aligned odom / Gazebo 近地面，
  并最终释放 stalled direct path。
- 2026-05-16 最新静态门禁已通过：visual config、SLAM graph、terrain planner、
  control command chain 组合测试 `136 passed in 31.38s`；`git diff --check`
  无输出；`colcon build --symlink-install` 完成 8 packages。
- 已修复 `long_corridor` 单层长路线的 final-goal 语义：新增
  `direct_tracking_reaches_goal`，要求 direct tracker 在 tracked graph endpoint
  和原始用户 final goal 都进入 tolerance 后才算最终到达，避免离目标较远的
  graph 终点误报 goal reached。新增测试覆盖 off-graph endpoint 拒绝和
  tolerance 内接受，visual source-contract 也锁定 direct tick 必须传入
  `self._direct_final_goal_xy`。
- 修复后 `long_corridor` runtime 已通过：
  `log/fast_lio_single_floor_demo/long_corridor_goal_guard_20260516_184353/`
  中 summary 为 `accepted: true`，`/Laser_map_world` 最大 `263068` 点，
  `/pct_path_max_z=-0.032202`，三路命令计数分别为
  `/cmd_vel_nav=487`、`/cmd_vel_smoothed=548`、base command `548`，
  wheel/Gazebo 到 `(8.0,-9.0)` 的最终距离均为 `0.266004m`。
- 同一运行证明单层长路线仍保持 SLAM/PCT 主线：初始 frontier 路径推进后，
  launch log 记录 `pending final goal became reachable after FAST-LIO map update`
  和最终 `terrain direct tracking goal reached`。这不是放宽验收阈值得到的通过。
- 2026-05-16 最新静态门禁更新为：visual config、SLAM graph、terrain planner、
  control command chain 组合测试 `139 passed in 31.33s`；`git diff --check`
  无输出；`colcon build --symlink-install` 完成 8 packages `[1.21s]`。
- 已补跨楼层阶段的 3D/floor-aware goal 发布链：新增
  `ros2 run airos_experiments publish_terrain_goal --x 6.0 --y 13.0 --z 2.2 --publish-count 5 --rate-hz 1`
  作为高层目标发布方式，避免继续依赖一次性 2D/volatile goal。`terrain_pct_planner`
  现在会把高 `PoseStamped.pose.position.z` 解释为目标楼层约束；`cross_level_evidence_probe`
  记录 `goal_xyz`，方便后续证据链区分目标 XY 与目标楼层。
- 对应测试已补齐：publisher console script、3D goal 字段保真、probe `goal_xyz`
  字段、planner goal-z source contract 和 final-goal-aware direct completion 均有
  覆盖。
- 2026-05-16 最新静态门禁更新为：visual config、SLAM graph、terrain planner、
  control command chain、cross-level evidence probe 组合测试
  `144 passed in 31.14s`；`git diff --check` 无输出；
  `colcon build --symlink-install` 完成 8 packages `[0.89s]`。
- 当前下一步不是继续补 goal 表达，而是用上述 3D 工具做一轮有界跨层 runtime：
  确认 `/pct_path max z > 2.0` 后，重点看 ramp-to-deck/high-deck waypoint、
  local slope/step 连续性、`/cmd_vel_nav` 到 collision monitor/base 的输出、
  wheel `/odom` 与 Gazebo pose 是否真实上升。
- 3D/floor-aware goal 的有界跨层 runtime 已完成两轮。180 秒运行
  `log/cross_level_3d_goal_20260516_190046/` 记录
  `goal_xyz_last=[6.0,13.0,2.2]`、`/Laser_map_world` 最大 `674569` 点、
  `/pct_path_max_z=2.275243`、`/cmd_vel_nav=2513`、smoother `2845`、base
  command `2830`，但 wheel `/odom` z 仍为 `0.0`，Gazebo z 仍约 `-0.005001`。
- 300 秒长窗 `log/cross_level_3d_goal_long_20260516_191019/` 进一步确认：
  `goal_xyz_last=[6.0,13.0,2.2]`、`/Laser_map_world` 最大 `785990` 点、
  `/pct_path_max_z=2.344505`、`/cmd_vel_nav=4240`、smoother/base `4734`，三路
  command age 最大分别约 `0.14s/0.061s/0.071s`，命令链 freshness 健康。
  但 Gazebo z 最大仍约 `-0.004997`，wheel `/odom` z 仍为 `0.0`，最终 Gazebo
  到高层目标 XY 距离反而扩大到 `15.572861m`。
- 长窗 launch log 显示 direct tracking 从 `slam_ramp` 目标推进到
  `slam_step` 高节点，FAST-LIO aligned z 约 `0.35-0.37`，但 Gazebo 真实高度
  不上升。下一步应先做最小物理能力/接触诊断：确认当前
  `go2w_nav_eq` diff-drive surrogate 是否能在简单 ramp 上让 Gazebo z 上升；
  若不能，不应继续用长跨层实验消耗时间。
- 已完成该物理能力诊断并收窄结论：
  `log/lower_ramp_physics_after_landing_fix_20260516_195621/` 直接底盘命令让
  Gazebo z 达到 `0.934999`，说明修复后的下坡道物理通道可通，不能再把跨层失败
  简化为 surrogate 完全不能爬坡。
- 已修复 `large_multilevel_complex(.sdf|_static.sdf)` 的两处物理几何问题：
  `second_floor_deck` 不再覆盖 lower ramp，`ramp_upper_landing` 后移到坡道末端；
  同时修复 `terrain_pct_planner` 对 `landing` 的 ramp/slope 误分类。相关几何、
  路径和速度分类单测通过。
- 主链路两轮复测仍未通过物理跨层：
  `cross_level_after_landing_fix_goal_ok_20260516_200724` 与
  `cross_level_after_regressive_prefix_fix_20260516_201433` 都有高
  `/pct_path` 和命令链，但 Gazebo z 始终约 `-0.005001`。后者最终停在约
  `(4.04,1.37)`，对应低层 `slam_step` 伪入口区域。
- 已新增 `drop_regressive_start_waypoints` 回归测试并修复高路径出现后回追低层前缀
  的问题；复测显示这不是唯一瓶颈。下一步应集中在 SLAM frontier/ramp-entry
  选择，避免追低层孤立 `slam_step`。
- 已完成一个 SLAM frontier/ramp-entry 静态修复：新增
  `test_slam_frontier_path_prefers_ramp_entry_over_isolated_step_pair`，并调整
  `terrain_pct_planner` 的 high-floor frontier entry scoring，使连续 ramp/stair
  vertical progress 优先于孤立 step-pair attractor。该修复用于降低主链路追向
  `(4.9,1.4,0.14)` 一类低层伪入口的风险。
- 修复后门禁通过：visual config、SLAM graph、terrain planner、control chain、
  cross-level evidence probe 组合测试 `148 passed in 27.55s`；`git diff --check`
  无输出；`colcon build --symlink-install` 完成 8 packages。
- 当前仍未完成跨层验收：尚未复跑 runtime 证明机器人会进入真实 lower ramp
  corridor，也未证明 Gazebo pose 实际持续上升并到达高层目标。
- 已复跑 `log/cross_level_after_frontier_entry_fix_20260516_203725/`：高路径和命令
  链仍健康，`/Laser_map_world` 最大 `796963` 点，`/pct_path_max_z_max=2.064855`，
  command age 最大低于 `0.08s`；但 Gazebo z 仍为 `-0.005001`，跨层物理验收仍失败。
- 该运行显示后期 frontier 已转向 `(-5.93,0.55)`，但前面仍被低高度 `slam_ramp`
  目标拖到 `(6,-11)` 一带，导致没有进入真实坡道。已新增
  `test_direct_tracking_drops_regressive_low_ramp_prefix_before_high_entry` 并实现低
  ramp/slope 前缀进展过滤。
- 最新静态门禁：visual config、SLAM graph、terrain planner、control chain、
  cross-level evidence probe 组合测试 `149 passed in 27.56s`；`git diff --check`
  无输出；`colcon build --symlink-install` 完成 8 packages。
- 已继续补 final-path/frontier gate：active frontier 执行期间不抢切 final high
  path；final high path 若在进入高层前大幅远离最终目标，则拒绝并继续探索。
  新增 `test_pending_final_goal_waits_for_active_frontier_endpoint` 和
  `test_high_final_path_rejects_large_initial_goal_regression`。
- 最新门禁更新为：visual config、SLAM graph、terrain planner、control chain、
  cross-level evidence probe 组合测试 `151 passed in 27.47s`；`git diff --check`
  无输出；`colcon build --symlink-install` 完成 8 packages。
- 已复跑 `log/cross_level_after_final_regression_guard_20260516_210455/`：旧的远处
  final target 抢切没有复现，系统发布 `frontier=(-3.56,1.20)` 的 frontier path，
  direct tracker 推进到 `target=(-3.56,1.20,0.46) surface=slam_ramp`。
- 同一运行仍未完成物理跨层：Gazebo z 仍约 `-0.005`，最终高层目标 XY 距离约
  `15.872381m`，collision monitor 在坡道边缘附近触发 StopZone，随后 frontier
  stall/release。
- 当前下一步准备：保留安全层，优先做 ramp-center/support-margin scoring 或
  `/slam_scan` StopZone 诊断；不要回退到 SDF 真值规划，也不要把高 `/pct_path`
  说成物理高层到达。
- 已补 height-debt direct lookahead 修正：当前高 `slam_step` waypoint 物理高度
  未达时，索引仍不推进；但命令目标可在同一 surface 段向后续 waypoint 前看，
  防止 XY 已到而目标距离近似 0 后原地旋转。跨 surface 的 height-debt lookahead
  仍被拒绝，避免把路径横跨坡道/楼梯/平台边界。
- 已修 cross-level evidence probe：Gazebo pose 查询超时不再让 probe 崩溃，
  而是该样本 `gazebo_xyz=None`。空 `probe.jsonl` 必须先查 probe/launch 错误，
  不能作为导航失败或成功证据。
- 最新门禁更新为：`test_slam_scan_projector.py`、visual config、SLAM graph、
  terrain planner、control chain、cross-level evidence probe 组合测试
  `162 passed in 46.38s`；`git diff --check` 无输出；`colcon build --symlink-install`
  完成 8 packages `[1.59s]`。
- 已复跑 `log/cross_level_after_height_debt_lookahead_20260516_215319/`：
  `/Laser_map_world` sampled max `792874`，`/pct_path_max_z=2.178018`，
  `/cmd_vel_nav=3101`、smoother `3502`、base `3496`，命令 age 仍 fresh。
- 同一运行仍未完成物理跨层：FAST-LIO aligned z 最大 `0.400256`，wheel `/odom`
  z 仍 `0.0`，Gazebo z 仍 `-0.005001`；Gazebo 到 `(6.0,13.0,2.2)` 的 XY 距离
  最小 `13.662475m`、最终 `13.720846m`。
- 最新执行瓶颈：direct tracker 可推进到更靠近高层入口的 `slam_step` 区域，
  但在 `(-4.29,3.35,0.72)` 与 `(-3.97,4.46,0.74)` 等相邻 lookahead 目标间摆动，
  heading error 过大导致 `_direct_linear_speed` 多次输出 0，最终 direct path
  stalled/released。下一步应稳定 step/ramp 段的前向切向目标选择，而不是关闭安全层。
- 已补 step/ramp 段 path-tangent/forward-progress lookahead 约束和回归测试；
  后续第一轮 runtime `cross_level_after_tangent_lookahead_20260516_220735` 暴露的
  是控制链问题而不是有效跨层执行结果：collision monitor lifecycle 服务未及时
  可用，base command 计数为 `0`。
- 已修 lifecycle activator 的服务等待策略，改为限定次数重试
  `get_state`/`change_state` 服务。最新静态门禁更新为
  `164 passed in 29.46s`，`git diff --check` 无输出，
  `colcon build --symlink-install` 完成 8 packages `[0.93s]`。
- 最新短跑 `control_chain_after_lifecycle_retry_20260516_221346` 恢复 base command：
  `/cmd_vel_nav=915`、smoother `1016`、base `1015`，Gazebo 到目标距离从
  `24.517079m` 缩短到 `18.280812m`。该 run 没有高路径验收：
  `/pct_path_max_z=0.475012`，Gazebo z 仍为 `-0.005001`。
- 当前下一步准备：先处理 collision monitor 忽略 `/slam_scan` 的 timestamp 差异
  告警，再做下一轮更长跨层 runtime。安全层不能直接关闭；若调整 source timeout
  或 projector cost，必须用测试和短跑证明 base command 与扫描 freshness 同时健康。
- 已完成 `/slam_scan` freshness 的最小修复：`slam_scan_projector` 连续坡面支持
  判断改为局部 spatial support bins，避免每点全量扫 sampled cloud。没有改变
  collision monitor 的 StopZone/SlowZone 几何和 source timeout。
- 静态门禁更新为 `165 passed in 27.46s`，`git diff --check` 无输出，
  `colcon build --symlink-install` 完成 8 packages `[0.99s]`。
- 短窗验证 `scan_freshness_after_support_index_20260516_222625`：
  `slam_scan_stale_warn_count=0`，base command `1033`，`/cmd_vel_nav=918`，
  `cmd_vel_smoothed=994`；Gazebo 到目标距离有下降。该 run 仍不是跨层成功证据，
  因为 `/pct_path_max_z=0.470178` 且 Gazebo z 仍 `-0.005001`。
- 当前下一步：现在可以进入较长跨层 runtime 复测 path-tangent lookahead 的实际效果；
  接受条件仍是 `/pct_path max z > 2.0` 且 Gazebo pose 实际上升到高层。
- 已按用户优先级先复核单层展示链：
  `near_goal_after_scan_index_20260516_223232` 与
  `long_corridor_after_scan_index_20260516_223440` 都 `accepted=true`。
- `near_goal` 最终 Gazebo/wheel 距离均为 `0.222402m`，`/Laser_map_world`
  最大 `248275`，命令链 `/cmd_vel_nav=200`、smoother/base `235`。
- `long_corridor` 最终 Gazebo/wheel 距离均为 `0.294605m`，`/Laser_map_world`
  最大 `351188`，`/pct_path_poses=10`，`/pct_path_max_z=-0.044546`，命令链
  `/cmd_vel_nav=573`、smoother `645`、base `660`。
- 两个单层 run 的 `/slam_scan` stale warning 计数都是 0。当前单层主链可以作为
  快速展示基线；后续应转入跨层长窗复测，不再反复怀疑单层链路，除非脚本回归。
- 跨层复测已按收敛原则停止继续长跑：
  `cross_level_after_single_floor_refresh_20260516_223943` 和
  `cross_level_after_zigzag_lookahead_fix_20260516_225624` 都再次证明高
  `/pct_path`、live SLAM 和命令链，但 Gazebo z 均停在 `-0.005001`。
- 已修一个 direct lookahead 的局部抖动问题并通过 `166 passed`、`git diff --check`、
  `colcon build`；但修后跨层仍未物理爬升，说明继续在同一 surrogate 上长跑收益低。
- 当前执行策略：单层演示作为当前交付结果；跨层不再在本轮死磕，后续作为二阶段
  分支处理模型/控制或简化 multilevel map。

## 2026-05-16 最小 multilevel smoke 入口

- 已新增 `scripts/run_fast_lio_multilevel_smoke.sh`，默认使用已有
  `realistic_multilevel_ramp`、`realistic_multilevel_ramp_static.sdf`、
  `realistic_multilevel_ramp.yaml` 和 `realistic_multilevel_ramp_route.geojson`。
- 脚本复用当前 FAST-LIO/PCT/direct/safety 主链，重复发布 3D
  `/terrain_goal_pose`，用 `cross_level_evidence_probe` 采集
  `/Laser_map_world`、`/pct_path`、三段 cmd、`/odom` 和 Gazebo pose，并把
  `/pct_path` 高度、Gazebo z 和目标距离写入 `summary.json`。
- 静态脚本测试已加入
  `test_fast_lio_multilevel_smoke_script_uses_physical_z_acceptance`。
- 第一轮短跑 `log/fast_lio_multilevel_smoke/quick_multilevel_smoke_20260516_232258/`
  目标为远端 upper-lab `(7.2,7.4,0.9)`：live map 最大 `312701` 点，base command
  `1083`，但 `/pct_path_max_z=0.235028`，`gazebo_z_max=0.335047`，最终目标距离
  `2.332921m`，未通过。
- 第二轮短跑 `log/fast_lio_multilevel_smoke/ramp_entry_smoke_20260516_232735/`
  目标改为坡道入口/夹层目标 `(0.4,3.6,0.65)`：planner 收到目标并启动 direct
  tracking，`/Laser_map_world` 最大 `291189` 点，命令链 `/cmd_vel_nav=801`、
  smoother `861`、base `874`；但 `/pct_path` 未被 probe 采样，`gazebo_z_max`
  只有 `0.060886`，最终目标距离 `4.120866m`。
- 已修 multilevel smoke 的采样顺序：probe 先启动，再发布
  `/terrain_goal_pose`，避免 goal 很快被处理后 `/pct_path` 漏采。
- 第三轮短跑
  `log/fast_lio_multilevel_smoke/ramp_corridor_guard_20260516_233809/`
  采到了高路径：`/Laser_map_world=271731`、`/pct_path_poses=33`、
  `/pct_path_max_z=0.984888`，命令链 `/cmd_vel_nav=697`、smoother/base `769`。
  但它仍未通过物理爬升：`gazebo_z_max=0.067521`，最终目标距离 `5.047544m`。
- 同一 run 证明低层绕路 guard 只部分生效：planner 先记录
  `deferred pending final goal because the reachable high path initially
  regresses away from the goal`，随后又接受了一条 direct path，其首目标仍是
  `(2.20,-3.05,-0.14)` `slam_floor`，机器人沿低层走廊移动而没有进入实际坡道。
- 最新推断：更小 smoke 已复现跨层执行缺口，且成本明显低于大场景。下一步应把
  “高/夹层目标不得先走远端低层 floor detour” 的约束前移到初始 final-goal
  接受路径，而不只是 pending final goal retry；修复前不要继续重复 runtime。
