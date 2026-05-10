# Advanced Planning Research Profile

This profile keeps the current Nav2 baseline intact and adds a clearly marked
research entry for advanced planning comparison.

## What is implemented now

- `nav2_params.yaml` remains the stable baseline.
- `nav2_research_profile.yaml` is an experimental Nav2 profile for comparison.
- `nav.launch.py` exposes `planner_profile:=baseline|research` so the profile
  can be selected without deleting the baseline.
- `terrain_pct_planner` is a ROS 2 runtime terrain planner. It parses the SDF
  collision geometry, samples floor/ramp/deck traversable surfaces, runs a
  PCT-style height-aware graph search, publishes `/pct_path`, and sends
  terrain-guided `NavigateThroughPoses` waypoints to Nav2.
- `pointcloud_emulator` now samples full 3D collision surfaces rather than only
  obstacle side walls. The ramp top, floor, and mezzanine deck therefore enter
  `/livox/lidar_points`, FAST-LIO2 input, `/cloud_registered`, and
  `/Laser_map`.
- `generate_advanced_planner_candidates` exports deterministic route candidates
  for `nav2_baseline_route`, `pct_style_risk_weighted_route`, and
  `rl_safety_shield_waypoints`. This gives the PCT/RL research path a concrete
  input/output contract without claiming a trained planner runtime.
- `advanced_indoor_ramp.sdf` and `advanced_indoor_ramp_route.geojson` provide a
  stair/ramp-focused scene and route-constrained test surface.
- `realistic_multilevel_ramp.sdf` adds a larger indoor multi-level ramp scene
  with shelves, workbenches, glass partitions, columns, crates, and triggerable
  physical dynamic obstacles.
- `open_source_scene_assets:=true` overlays the AFL-3.0 Building mesh from
  `ypat999/3d_dog_navi_ros2` as a visual-only scene asset.
- `robot_visual_profile:=reference_mesh` overlays AFL-3.0 Go2W body, wheel, and
  Mid360 meshes on the current verified navigation-equivalent robot. The Mid360
  DAE is scaled to millimeter units in URDF so it does not hide the robot in
  Gazebo.
- `pointcloud_colorizer` republishes `/Laser_map` as `/Laser_map_colored` with
  height-based RGB colors for RViz inspection.

## PCT-planner boundary

PCT-planner is the correct research direction for multi-floor point-cloud
terrain planning. The directly checked ROS 2 fork still needs CUDA toolkit,
CuPy, Open3D, NumPy/SciPy compatibility work, and native planner dependencies
before it can be built as a stable package in this WSL2 Humble workspace.
AIROS therefore implements a compatible runtime bridge first: terrain surface
sampling plus height-aware graph search and Nav2 waypoint execution. This is a
PCT-style terrain planner, not a claim that upstream PCT-planner CUDA
tomography is fully installed.

Current repository state:

- Keeps upstream PCT-planner as a research integration target.
- Provides `terrain_pct_planner` as the current runnable ROS 2 terrain planner.
- Provides a route-graph and map-backed candidate generator with risk-weighted
  scoring so PCT-like terrain costs can be compared against the Nav2 baseline.
- Marks PCT-style candidates as `research_surrogate_not_trained_runtime`.
- Does not claim CUDA tomography, learned traversability, or full kinodynamic
  legged planning as completed runtime.

## 强化学习边界

强化学习路径规划需要训练环境、策略模型、状态/动作接口和安全约束。
当前仓库没有训练好的策略权重，也没有可复现训练流水线。因此本轮只保留
RL planner 的研究接口和对比位置，不声称已经完成强化学习自主规划。

Current repository state:

- Exports an RL-style safety-shield waypoint candidate that densifies route
  waypoints for later policy/action filtering.
- Marks RL-style candidates as `research_surrogate_not_trained_runtime`.
- Does not claim a trained policy, reward model, or sim-to-real safety proof.

## Candidate generator

```bash
ros2 run airos_experiments generate_advanced_planner_candidates \
  --map src/airos_nav/maps/realistic_multilevel_ramp.yaml \
  --route-graph src/airos_nav/routes/realistic_multilevel_ramp_route.geojson \
  --start-id 1 \
  --goal-id 3 \
  --output log/advanced_planner_candidates.json
```

Output schema:

```text
airos_advanced_planner_candidate.v1
```

The report contains route node ids, edge ids, waypoints, path length,
risk-adjusted score, and route risk exposure for each candidate. It is intended
for regression checks and future replacement by real PCT/RL planner backends.

## How to run the research profile

```bash
ros2 launch airos_experiments visual_fast_lio_navigation.launch.py \
  gui:=true \
  rviz:=true \
  world:=realistic_multilevel_ramp \
  map:=src/airos_nav/maps/realistic_multilevel_ramp.yaml \
  route_graph:=src/airos_nav/routes/realistic_multilevel_ramp_route.geojson \
  planner_profile:=research \
  terrain_planner:=true \
  dynamic_obstacles:=true \
  physical_dynamic_obstacles:=true \
  open_source_scene_assets:=true \
  robot_visual_profile:=reference_mesh \
  sensor_source:=native \
  colorized_pointcloud:=true
```

RViz displays to inspect:

- `/Laser_map_colored`: height-colored FAST-LIO2 map cloud.
- `/cloud_registered`: current registered LiDAR cloud.
- `/terrain_traversability_cloud`: floor/ramp/deck surfaces used by the terrain
  planner.
- `/pct_path`: terrain-aware cross-level path sent as Nav2 waypoints.
- `/dynamic_obstacles/markers`: software dynamic obstacle overlay. Gazebo
  physical dynamic obstacles are triggered with `physical_dynamic_obstacles:=true`.
