#!/usr/bin/env bash
set -euo pipefail

patterns=(
  'ros2 launch airos_sim sim.launch.py'
  'ros2 launch airos_experiments visual_fast_lio_navigation.launch.py'
  'ros2 launch airos_experiments visual_navigation.launch.py'
  'ign gazebo'
  'parameter_bridge'
  'fastlio_mapping'
  'fast_lio_localization_bridge'
  'fast_lio_map_aligner'
  'tf2_ros/static_transform_publisher'
  'robot_state_publisher'
  'imu_republisher'
  'livox_custom_bridge'
  'scan_emulator'
  'pointcloud_emulator'
  'airos_experiments/pointcloud_colorizer'
  'airos_experiments/terrain_pct_planner'
  'nav2_lifecycle_manager/lifecycle_manager'
  'nav2_map_server/map_server'
  'nav2_amcl/amcl'
  'nav2_controller/controller_server'
  'nav2_smoother/smoother_server'
  'nav2_planner/planner_server'
  'nav2_behaviors/behavior_server'
  'nav2_bt_navigator/bt_navigator'
  'nav2_waypoint_follower/waypoint_follower'
  'nav2_velocity_smoother/velocity_smoother'
  'nav2_collision_monitor/collision_monitor'
  'nav2_route/route_server'
  'rviz2'
)

for pattern in "${patterns[@]}"; do
  pkill -f "$pattern" 2>/dev/null || true
done

sleep 1

for pattern in "${patterns[@]}"; do
  pkill -9 -f "$pattern" 2>/dev/null || true
done

sleep 1

if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop >/dev/null 2>&1 || true
fi

leftover_pattern='ign gazebo|parameter_bridge|ros2 launch airos|fastlio_mapping|fast_lio_localization_bridge|fast_lio_map_aligner|static_transform_publisher|robot_state_publisher|imu_republisher|livox_custom_bridge|scan_emulator|pointcloud_emulator|airos_experiments/pointcloud_colorizer|airos_experiments/terrain_pct_planner|nav2_|rviz2'
if pgrep -af "$leftover_pattern" >/tmp/airos_runtime_leftovers.txt; then
  cat /tmp/airos_runtime_leftovers.txt
  exit 1
fi

printf '[PASS] AIROS runtime processes cleaned.\n'
