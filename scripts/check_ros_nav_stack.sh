#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

required=(
  slam_toolbox
  nav2_route
  nav2_mppi_controller
  nav2_smac_planner
  nav2_collision_monitor
  nav2_velocity_smoother
  ros_gz_bridge
  ros_gz_sim
)

for pkg in "${required[@]}"; do
  ros2 pkg prefix "$pkg" >/dev/null
  printf '[OK] %s -> %s\n' "$pkg" "$(ros2 pkg prefix "$pkg")"
done

printf '\n[PASS] Required ROS 2 Humble navigation and Gazebo bridge packages are available.\n'
