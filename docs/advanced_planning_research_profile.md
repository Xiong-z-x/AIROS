# Advanced Planning Research Profile

This profile keeps the current Nav2 baseline intact and adds a clearly marked
research entry for advanced planning comparison.

## What is implemented now

- `nav2_params.yaml` remains the stable baseline.
- `nav2_research_profile.yaml` is an experimental Nav2 profile for comparison.
- `nav.launch.py` exposes `planner_profile:=baseline|research` so the profile
  can be selected without deleting the baseline.
- `advanced_indoor_ramp.sdf` and `advanced_indoor_ramp_route.geojson` provide a
  more complex scene and route-constrained test surface.
- `open_source_scene_assets:=true` overlays the AFL-3.0 Building mesh from
  `ypat999/3d_dog_navi_ros2` as a visual-only scene asset.
- `robot_visual_profile:=reference_mesh` overlays AFL-3.0 Go2W body, wheel, and
  Mid360 meshes on the current verified navigation-equivalent robot.

## PCT-planner boundary

PCT-planner is the correct research direction for multi-floor point-cloud
terrain planning, but it is not directly promoted to the stable runtime in this
repository. The referenced `3d_dog_navi_ros2` project targets Gazebo Garden and
contains a large planner stack with CUDA/GTSAM-style dependencies. AIROS keeps
that path as an experimental integration target, not as a claimed completed
feature.

## 强化学习边界

强化学习路径规划需要训练环境、策略模型、状态/动作接口和安全约束。
当前仓库没有训练好的策略权重，也没有可复现训练流水线。因此本轮只保留
RL planner 的研究接口和对比位置，不声称已经完成强化学习自主规划。

## How to run the research profile

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
  sensor_source:=native
```
