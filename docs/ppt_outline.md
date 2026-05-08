# AIROS PPT Outline

## 1. Title

AIROS: ROS 2 Humble and Gazebo Fortress Based Indoor Autonomous Navigation.

## 2. Problem And Constraints

- Goal: reproducible indoor autonomous navigation for a Go2W-equivalent robot.
- Constraint: WSL2 development environment.
- Decision: keep Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress, OGRE renderer.

## 3. Environment Evidence

- GPU/OpenGL renderer: D3D12 NVIDIA RTX 3050 through WSLg.
- Gazebo: Ignition Gazebo Fortress 6.16.0.
- Rejected path: OGRE2 is unstable on this machine.
- Evidence: `docs/environment_baseline.md`.

## 4. System Architecture

- Simulation: Gazebo world, robot spawn, ROS/Gazebo bridge.
- Robot model: Go2W navigation-equivalent URDF/Xacro.
- Navigation: Nav2 AMCL, Smac Hybrid-A*, MPPI, collision monitor.
- Experiment: mission runner, clean batch runner, metrics exporter.

## 5. Go2W Equivalent Model

- Frame tree and lidar frame.
- Differential-drive navigation equivalent.
- Scope boundary: flat indoor navigation, not full quadruped dynamics.

## 6. SLAM Mapping

- Mapping with `slam_toolbox`.
- Saved map and posegraph.
- Localization with AMCL on the saved map.

## 7. Route Graph

- Nav2 route graph: `single_floor_lab_route.geojson`.
- Route nodes: start, door, task points, return point.
- Edge metadata: risk and speed limits.
- Route server ComputeRoute verification is available.

## 8. Nav2 Planning And Control

- Smac Hybrid-A* global planning.
- MPPI local control.
- Velocity smoother and collision monitor.
- Trial runner gates on `map -> base_link` before sending goals.

## 9. Dynamic Avoidance

- Current method: ROS-side scan-layer dynamic obstacle emulator.
- Collision monitor consumes `/scan`.
- Limitation: not Gazebo physical obstacles and not GPU LiDAR.

## 10. Experiment Design

- Four fixed missions.
- Five repeats per mission.
- Clean process-per-trial execution to avoid stale Nav2 recovery state.
- Metrics: success, time, path length, emergency stops, scan collision
  estimate, minimum obstacle distance, command period.

## 11. Results

- 20/20 success.
- Success rate: 1.0.
- Mean elapsed: 19.195 s.
- Mean path: 3.102 m.
- Collision estimate: 0.
- Emergency stops: 0.

## 12. Ablation And Discussion

- Baseline without route-constrained execution: validated.
- Route graph computation: validated separately.
- Dynamic physical obstacle simulation: not claimed in current WSL baseline.
- Future ablation: route graph on/off, MPPI/RPP, collision monitor on/off.

## 13. Limitations

- WSL2 limits OGRE2/GPU LiDAR reliability.
- Scan emulator simplifies sensor physics.
- Real robot migration needs native/hardware validation.

## 14. Conclusion

The project has a stable single-floor navigation demonstration and a
repeatable metrics pipeline in the current environment.
