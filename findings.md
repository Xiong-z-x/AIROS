# AIROS 深度升级发现记录

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
