#!/usr/bin/env bash
set -euo pipefail

patterns=(
  'ros2 launch airos_sim sim.launch.py'
  'ros2 launch airos_experiments visual_fast_lio_navigation.launch.py'
  'ros2 launch airos_experiments visual_navigation.launch.py'
  'ign gazebo'
  'parameter_bridge'
  'fastlio_mapping'
  'imu_republisher'
  'scan_emulator'
  'pointcloud_emulator'
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

if pgrep -af 'ign gazebo|parameter_bridge|ros2 launch airos|fastlio_mapping|imu_republisher|scan_emulator|pointcloud_emulator|rviz2' >/tmp/airos_runtime_leftovers.txt; then
  cat /tmp/airos_runtime_leftovers.txt
  exit 1
fi

printf '[PASS] AIROS runtime processes cleaned.\n'
