# AIROS 深度升级任务计划

> 状态：历史计划。当前迁移前封板与后续接手入口见 `docs/handoff/README.md`。
> 若本文与 `docs/handoff/` 或当前代码/测试不一致，以当前代码和 `docs/handoff/` 为准。

## 目标

在不破坏当前稳定基线的前提下，推进 AIROS 从单层平面导航原型升级到更接近 Go2W / FAST-LIO / 复杂场景 / route-constrained 的高级演示系统。

## 成功标准

1. FAST-LIO 输入链路默认使用 Gazebo 原生雷达点云，ROS 侧模拟器只作为 `sensor_source:=emulated` 回退。
2. RViz 保留 Nav2 地图、RobotModel、TF、LaserScan、Livox 原始点云、FAST-LIO 注册点云和点云地图显示，避免重复发布源造成重影。
3. 增加一个更复杂的高级场景入口，包含更丰富静态结构和 Gazebo 物理动态障碍；不删除现有 `single_floor_lab`。
4. 增加规划算法对比入口：保留当前 Nav2 基线，加入可切换的高级配置/研究入口；PCT/RL 不伪装成已完整落地。
5. Go2W 模型向真实 Unitree Go2W 参考靠近：保持当前可控轮式等效体，补充可验证的模型参数/传感器/坡道模式边界。
6. 加固 route-constrained：路线、任务点、批量任务有可复用验证入口。
7. 完成构建、少量关键测试、Git 提交和 GitHub 同步。

## 当前阶段

- [x] 阶段 0：保存当前稳定回退点。
- [x] 阶段 1：原生 Gazebo `/scan` 与 `/livox/lidar_points -> Livox CustomMsg /livox/lidar` 链路已实现。
- [x] 阶段 2：调研并筛选可接入参考项目、地图、规划算法。
- [x] 阶段 3：盘点本仓库当前传感器、RViz、地图、Nav2、route 链路。
- [x] 阶段 4：实现高级场景和物理动态障碍入口。
- [x] 阶段 5：实现规划配置对比和 route 批量硬化入口。
- [x] 阶段 6：更新文档、测试、提交并同步。
- [x] 阶段 7：FAST-LIO2 `/Laser_map_world` -> SLAM-cloud traversability graph -> 高层 `/pct_path` 生成已获得在线证据。
- [ ] 阶段 8：物理执行高层路径并到达高层平台仍未验收，是下一阶段直接起点。
- [x] 阶段 9：迁移前封板交接包已建立于 `docs/handoff/`。
- [x] 阶段 10：按当前目标重审证据链并推进最小闭环。已确认
  `near_goal` 和 `long_corridor` 都已有可展示 FAST-LIO/PCT 单层闭环证据；跨层
  headless 复现已再次证明 live SLAM 增长、高层 `/pct_path` 和 direct 命令链
  成立，但 Gazebo/odom 实际高层爬升仍未验收。
- [x] 阶段 11：后续行动准备。已补 3D/floor-aware goal 发布链
  `publish_terrain_goal`，并让 planner 使用高 `PoseStamped.z` 作为目标楼层约束；
  probe 已记录 `goal_xyz`。不得用 2D goal 或 RViz 线条替代跨楼层验收。
- [ ] 阶段 12：有界跨层执行诊断。3D/floor-aware goal 的两轮 headless runtime
  已完成：都确认 `(6.0,13.0,2.2)` 高层目标能驱动 `/pct_path max z > 2.0`
  和 fresh command chain，但 wheel `/odom` 与 Gazebo pose 仍没有实际上升。
  当前剩余问题不是 goal 表达或高路径生成，而是 ramp-to-deck/high-deck
  waypoint execution、连续坡道/楼梯支撑、SLAM 重定位一致性、或 surrogate
  物理能力限制；不得回退到 SDF 真值规划。
- [x] 阶段 13：物理坡道能力与场景几何诊断。已证明修复后的 lower ramp 可被当前
  `go2w_nav_eq` 直接底盘命令爬上二层承台；已修复 deck clearance、upper landing
  leading edge 和 landing/ramp 分类误判。
- [ ] 阶段 14：SLAM frontier/ramp-entry 选择修复。已补一个静态 guard，避免
  高层 frontier 被低层孤立 `slam_step` pair 吸引而压过真实 lower ramp corridor；
  runtime 复测显示后期 frontier 已向真实坡道侧移动，但仍被低高度 `slam_ramp`
  前缀拖住；已再补低 ramp/slope 前缀进展过滤，对应门禁 `149 passed`、
  `git diff --check`、`colcon build` 通过。随后又补 final-path/frontier gate，
  最新门禁为 `151 passed`、`git diff --check`、`colcon build` 通过；
  `cross_level_after_final_regression_guard_20260516_210455` 证明旧的远处 final
  target 抢切已被控制，但机器人在 `(-3.56,1.20,0.46)` 附近触发 StopZone，
  Gazebo z 仍未上升。后续又补 `/slam_scan` 连续坡面过滤和 height-debt
  direct lookahead，最新门禁 `162 passed`、`git diff --check`、`colcon build`
  通过；`cross_level_after_height_debt_lookahead_20260516_215319` 再次生成
  `/pct_path_max_z=2.178018`，命令链 fresh，并推进到更靠近高层入口的
  `slam_step` 区域，但 Gazebo z 仍为 `-0.005001`。下一步转向 step/ramp 段
  前向切向目标选择和 heading 稳定，不能关闭安全层硬冲，也不能把高 `/pct_path`
  说成物理到达。已补 path-tangent lookahead 后，第一次 runtime 暴露出
  collision monitor lifecycle 激活竞态，base command 为 0；已修
  `lifecycle_activator` 服务等待重试并通过最新门禁 `164 passed`、`git diff --check`
  与 `colcon build`。`control_chain_after_lifecycle_retry_20260516_221346` 证明
  base command 恢复，但该短跑没有高路径/爬升验收。阶段 14 的下一步是处理
  collision monitor 忽略 `/slam_scan` 的 timestamp freshness 告警，再做长窗跨层
  复测。已定位并修复 `/slam_scan` 投影中 supported-ramp filter 的近似 O(n^2)
  处理成本，改为局部 spatial support bins；最新门禁 `165 passed`、`git diff --check`
  与 `colcon build` 通过。`scan_freshness_after_support_index_20260516_222625`
  证明 stale warning 为 0 且 base command 仍通，但仍没有高路径/爬升验收。
- [x] 阶段 15：单层展示链复核。按用户“先确认单层导航没问题”的优先级，已用
  `scripts/run_fast_lio_single_floor_demo.sh` 复核 `near_goal` 和 `long_corridor`。
  `near_goal_after_scan_index_20260516_223232` 最终 Gazebo/wheel 目标距离均为
  `0.222402m`；`long_corridor_after_scan_index_20260516_223440` 最终 Gazebo/wheel
  目标距离均为 `0.294605m`，`/pct_path_poses=10`，`/pct_path_max_z=-0.044546`。
  两个 run 均 `accepted=true` 且 `/slam_scan` stale warning 为 0。当前展示优先级
  应使用单层 FAST-LIO/PCT/direct 全链路；跨层继续作为未验收阶段推进。
- [x] 阶段 16：跨层死磕停止点。已完成两轮 post-refresh 跨层验证：
  `cross_level_after_single_floor_refresh_20260516_223943` 与
  `cross_level_after_zigzag_lookahead_fix_20260516_225624` 均满足 live SLAM、
  高 `/pct_path` 和命令链，但均不满足 Gazebo 物理爬升。已修一个 step
  lookahead zigzag 局部问题并通过 `166 passed`、`git diff --check`、`colcon build`，
  但修后仍未爬升。按用户效率要求，不再在同一轮式 surrogate 上连续长跑；跨层
  后续应切到更适合爬升的模型/控制或更小的验证地图。
- [x] 阶段 17：最小 multilevel smoke 入口。已新增
  `scripts/run_fast_lio_multilevel_smoke.sh`，复用现有
  `realistic_multilevel_ramp`、`publish_terrain_goal` 和
  `cross_level_evidence_probe`，验收同时要求 `/pct_path` 高度、Gazebo z 和目标距离。
  两轮短跑均未通过物理爬升：远端 upper-lab 目标只得到
  `/pct_path_max_z=0.235028`、`gazebo_z_max=0.335047`；坡道入口目标虽然有
  `/Laser_map_world=291189` 和 base command `874`，但 `/pct_path` 未被采样、
  `gazebo_z_max=0.060886`。随后已修 probe 启动顺序并加入低层绕路静态 guard；
  `ramp_corridor_guard_20260516_233809` 采到 `/pct_path_max_z=0.984888`、
  `/pct_path_poses=33` 和 base command `769`，但 Gazebo z 仍只有 `0.067521`。
  该 run 说明 guard 只部分生效：pending final goal 的回归路径被拦住，但重复
  goal 后的初始 final-goal 接受分支仍可启动低层 `slam_floor` detour。该入口
  已经把跨层问题从“大场景长跑”收窄成更便宜的 ramp-corridor / SLAM path
  selection 问题；下一步不应继续长跑，应先把低层绕路拒绝逻辑前移到初始
  final-goal path acceptance。

## 决策边界

- 不直接迁移到 Gazebo Garden/Harmonic；当前工程主线仍是 WSL2 + Ubuntu 22.04 + ROS 2 Humble + Gazebo Fortress。
- 不把研究型算法包装成已完成产品。PCT/RL 若无法在本机稳定编译运行，只能作为 `research` 或 `experimental` profile 明确标注。
- 不删除当前 Nav2 基线、emulated sensor 回退、单层实验室地图或已有批量验证证据。
- 不提交用户资料 PDF/DOCX、历史结果图表或其他非本轮源码工件。
- 不把“可安装的 Unitree/Go2 通信或描述仓库”推断成“Fortress 中可直接完成
  跨层足式爬楼”。模型替换必须有仿真/控制接口和验收证据，否则只作为后续分支。
- 跨层阶段先用 `scripts/run_fast_lio_multilevel_smoke.sh` 做短窗验收，只有该
  smoke 出现 `/pct_path` 高度和 Gazebo z 同时过阈值后，才值得回到
  `large_multilevel_complex` 长窗验收。

## 当前实现证据

- 新增 `src/airos_sim/worlds/advanced_indoor_ramp.sdf`，包含复杂室内结构、坡道视觉/碰撞模型、`moving_pedestrian` 和 `inspection_cart_dynamic` Gazebo 动态模型。
- `sim.launch.py` 新增 `world:=single_floor_lab|advanced_indoor_ramp` 和 `physical_dynamic_obstacles:=true|false`。
- `sim.launch.py` 新增 `open_source_scene_assets:=true|false` 和 `robot_visual_profile:=analytic|reference_mesh`，用于可选开源 Building/Go2W 视觉资产。
- 新增 `src/airos_nav/maps/advanced_indoor_ramp.yaml/.pgm`、`src/airos_nav/routes/advanced_indoor_ramp_route.geojson`、`src/airos_experiments/missions/advanced_indoor_ramp_missions.yaml`。
- `nav.launch.py` 新增 `planner_profile:=baseline|research`，research profile 使用 `nav2_research_profile.yaml`。
- `generate_advanced_planner_candidates` 新增可运行的 Nav2/PCT-style/RL-style 候选路线对比报告入口，PCT/RL 明确标注为研究替身而不是训练完成的 runtime。
- `run_clean_nav_batch` 新增 `--world`、`--map`、`--route-graph`、`--planner-profile`、`--physical-dynamic-obstacles`，可用于高级场景 batch。
- 新增 `src/airos_experiments/test/test_deep_upgrade_artifacts.py` 锁定这些入口。
- 最终验证：32 个 Python/配置测试通过，5 个 ROS 包构建通过，高级 FAST-LIO + Nav2 research runtime smoke 通过，route graph verifier 通过。
