# AIROS Autonomous Navigation Report Outline

## 1. Project Objective

Build a reproducible single-floor autonomous navigation system in the current
WSL2, ROS 2 Humble, and Gazebo Fortress environment. The target is a Go2W
navigation-equivalent robot that can map, localize, plan, avoid scan-layer
obstacles, execute fixed missions, and export quantitative results.

## 2. Environment Baseline

- OS: Ubuntu 22.04.5 LTS on WSL2.
- ROS: ROS 2 Humble.
- Gazebo: Ignition Gazebo Fortress 6.16.0.
- Renderer: OGRE through WSLg D3D12/NVIDIA OpenGL.
- Rejected path: OGRE2/Harmonic is not used in the current baseline.
- Evidence source: `docs/environment_baseline.md`.

## 3. System Architecture

- `airos_go2w_description`: Go2W navigation-equivalent URDF/Xacro model.
- `airos_sim`: Gazebo world, spawn, bridge, scan emulator integration.
- `airos_control`: differential drive control chain.
- `airos_slam`: slam_toolbox mapping and localization launch/config.
- `airos_nav`: Nav2 map, AMCL, Smac Hybrid-A*, MPPI, collision monitor,
  velocity smoother, route graph, and route server config.
- `airos_experiments`: mission definitions, clean batch runner, dynamic
  obstacle scan-layer emulator, route verifier, and metrics export.

## 4. Go2W Equivalent Model

Describe the simplification boundary: the model is a navigation-equivalent
mobile base for flat indoor planning, not a full legged dynamics model. Include
URDF frame tree, lidar frame, base footprint, and controller interfaces.

## 5. Mapping And Localization

- Mapping path: `slam_toolbox` produces the single-floor lab map.
- Localization path: AMCL on the saved map for fixed navigation trials.
- Key interfaces: `/map`, `/scan`, `/tf`, `/tf_static`, `/initialpose`.
- Current hardening: trial runner waits for `map -> base_link` before goal
  submission.

## 6. Route Graph And Task Semantics

- Route graph file: `src/airos_nav/routes/single_floor_lab_route.geojson`.
- Nodes represent start, door passage, task points, and return point.
- Edges carry route metadata such as `speed_limit` and `risk`.
- Route server is validated for `ComputeRoute` graph loading and route
  computation. Full route-constrained execution remains a future hardening
  item beyond the current 20-run navigation gate.

## 7. Navigation Stack

- Global planning: Smac Hybrid-A*.
- Local control: MPPI controller.
- Safety layer: `nav2_collision_monitor` on `/scan`.
- Command chain: Nav2 command, velocity smoother, collision monitor, and
  differential drive controller.
- Main validated entry: clean process-per-trial runner.

## 8. Dynamic Obstacle Handling

- Current implementation: ROS-side scan-layer moving obstacle emulator and RViz
  markers.
- Current limitation: obstacles are not Gazebo physical moving bodies and are
  not GPU LiDAR returns.
- Evidence expected from current baseline: scan-layer avoidance and safety
  metrics, not physical contact simulation.

## 9. Experiments

- Mission file:
  `src/airos_experiments/missions/single_floor_lab_missions.yaml`.
- Acceptance batch:
  `log/airos_nav_trials_clean_batch20_action_wait.jsonl`.
- Result summary:
  `results/single_floor_lab_summary.md`.
- Figures:
  `results/figures/mean_elapsed_sec.svg`,
  `results/figures/mean_path_length_m.svg`.

## 10. Results

Report the current 20-run clean batch:

- Trials: 20.
- Success: 20/20.
- Success rate: 1.0.
- Mean elapsed: 19.195 s.
- Mean path length: 3.102 m.
- Emergency stops: 0.
- Scan-threshold collisions: 0.
- Minimum obstacle distance: 0.389 m.

## 11. Limitations

- No stable OGRE2/GPU LiDAR path in the current WSL2/Fortress baseline.
- `/scan` is produced by the ROS-side emulator.
- Dynamic obstacles are scan-layer inputs, not Gazebo physical obstacles.
- Long multi-mission Nav2 reuse in one process is not the accepted path.
- Real robot deployment still requires hardware validation.

## 12. Conclusion

The current implementation meets the single-floor, fixed-mission, quantitative
navigation demonstration target in the constrained WSL2 environment. The next
technical step is not changing Gazebo versions, but hardening route-constrained
execution and dynamic obstacle acceptance evidence on top of the stable clean
runner.
