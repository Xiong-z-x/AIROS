# AIROS 深度升级任务计划

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

## 决策边界

- 不直接迁移到 Gazebo Garden/Harmonic；当前工程主线仍是 WSL2 + Ubuntu 22.04 + ROS 2 Humble + Gazebo Fortress。
- 不把研究型算法包装成已完成产品。PCT/RL 若无法在本机稳定编译运行，只能作为 `research` 或 `experimental` profile 明确标注。
- 不删除当前 Nav2 基线、emulated sensor 回退、单层实验室地图或已有批量验证证据。
- 不提交用户资料 PDF/DOCX、历史结果图表或其他非本轮源码工件。

## 当前实现证据

- 新增 `src/airos_sim/worlds/advanced_indoor_ramp.sdf`，包含复杂室内结构、坡道视觉/碰撞模型、`moving_pedestrian` 和 `inspection_cart_dynamic` Gazebo 动态模型。
- `sim.launch.py` 新增 `world:=single_floor_lab|advanced_indoor_ramp` 和 `physical_dynamic_obstacles:=true|false`。
- `sim.launch.py` 新增 `open_source_scene_assets:=true|false` 和 `robot_visual_profile:=analytic|reference_mesh`，用于可选开源 Building/Go2W 视觉资产。
- 新增 `src/airos_nav/maps/advanced_indoor_ramp.yaml/.pgm`、`src/airos_nav/routes/advanced_indoor_ramp_route.geojson`、`src/airos_experiments/missions/advanced_indoor_ramp_missions.yaml`。
- `nav.launch.py` 新增 `planner_profile:=baseline|research`，research profile 使用 `nav2_research_profile.yaml`。
- `run_clean_nav_batch` 新增 `--world`、`--map`、`--route-graph`、`--planner-profile`、`--physical-dynamic-obstacles`，可用于高级场景 batch。
- 新增 `src/airos_experiments/test/test_deep_upgrade_artifacts.py` 锁定这些入口。
- 最终验证：32 个 Python/配置测试通过，5 个 ROS 包构建通过，高级 FAST-LIO + Nav2 research runtime smoke 通过，route graph verifier 通过。
