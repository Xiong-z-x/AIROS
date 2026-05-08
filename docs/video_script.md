# AIROS Demo Video Script

## 0:00 - 0:20 Title And Goal

Show the project title and state the target: reproducible indoor autonomous
navigation for a Go2W-equivalent robot in ROS 2 Humble and Gazebo Fortress.

## 0:20 - 0:45 Environment Baseline

Show `docs/environment_baseline.md`. Highlight WSL2, Gazebo Fortress, OGRE
renderer, and NVIDIA/D3D12 OpenGL evidence. State that OGRE2/GPU LiDAR is not
used in this baseline.

## 0:45 - 1:20 Gazebo And RViz Startup

Run the simulation launch. Show Gazebo world, robot model, RViz frames, map,
scan, costmaps, and robot pose.

## 1:20 - 1:55 SLAM Mapping Evidence

Show the saved map and posegraph files. Explain that `slam_toolbox` is used for
the mapping chain and AMCL is used for fixed-trial localization.

## 1:55 - 2:25 Route Graph Annotation

Open or display `single_floor_lab_route.geojson`. Show nodes for start, door,
task points, and return point. Mention edge metadata: speed limit and risk.

## 2:25 - 3:10 Navigation Task

Run one clean navigation mission. Show robot motion, global path, local control,
and final success status. Explain that the runner waits for `map -> base_link`
before sending the goal.

## 3:10 - 3:45 Dynamic Obstacle Layer

Launch with dynamic obstacle mode or show RViz markers. Explain that the
current baseline uses ROS-side scan-layer obstacles, not physical Gazebo moving
cylinders or GPU LiDAR returns.

## 3:45 - 4:25 Metrics Summary

Show `results/single_floor_lab_summary.md` and the SVG figures. Report:

- 20 fixed trials.
- 20 successes.
- Success rate 1.0.
- Mean elapsed 19.195 s.
- Mean path length 3.102 m.
- Emergency stops 0.
- Scan-threshold collisions 0.

## 4:25 - 5:00 Limitations And Next Step

State the real limitations: OGRE2/GPU LiDAR is not stable in the current WSL2
baseline; dynamic obstacles are scan-layer inputs; real robot deployment still
requires hardware validation. The next engineering step is route-constrained
execution hardening and dynamic obstacle acceptance evidence.
