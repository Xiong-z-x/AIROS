# AIROS 深度升级进度记录

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
