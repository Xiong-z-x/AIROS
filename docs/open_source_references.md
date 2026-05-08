# Open Source References

This project keeps the implementation aligned with the current WSL2, ROS 2
Humble, and Gazebo Fortress baseline. The references below were used as primary
or near-primary sources for package roles and interface choices.

## Navigation2

- Repository: <https://github.com/ros-navigation/navigation2>
- Nav2 documentation: <https://docs.nav2.org/>
- `NavigateThroughPoses` action API:
  <https://api.nav2.org/actions/humble/navigatethroughposes.html>

How it is used here:

- Nav2 provides AMCL, Smac Hybrid-A*, MPPI, BT Navigator, Waypoint Follower,
  Velocity Smoother, and Collision Monitor integration.
- `NavigateThroughPoses` is used by the route waypoint smoke path so the robot
  follows route-graph-derived intermediate poses instead of only a single final
  2D goal.

## Nav2 Route

- Humble package documentation:
  <https://docs.ros.org/en/ros2_packages/humble/api/nav2_route/index.html>
- Route Server tools documentation:
  <https://docs.nav2.org/tutorials/docs/route_server_tools.html>

How it is used here:

- `nav2_route` validates the GeoJSON route graph and returns graph-constrained
  `/compute_route` results.
- The project separately converts the route graph to `NavigateThroughPoses`
  waypoints for the current minimal route-following smoke test.

## Nav2 Collision Monitor

- Tutorial:
  <https://docs.nav2.org/tutorials/docs/using_collision_monitor.html>
- API documentation:
  <https://docs.ros.org/en/iron/p/nav2_collision_monitor/>

How it is used here:

- Collision Monitor consumes `/scan` and filters velocity commands after the
  velocity smoother.
- Current evidence shows slowdown and recovery in the dynamic scan-layer smoke
  test. This is not a hard real-time safety certification.

## ROS/Gazebo Bridge

- Repository: <https://github.com/gazebosim/ros_gz>
- API documentation: <https://docs.ros.org/en/ros2_packages/jazzy/api/ros_gz/>

How it is used here:

- `ros_gz_bridge` connects Gazebo Fortress topics and ROS 2 topics for the
  current simulation stack.

## SLAM Toolbox

- Repository: <https://github.com/SteveMacenski/slam_toolbox>

How it is used here:

- `slam_toolbox` provides the 2D mapping/localization development path and
  saved map/posegraph artifacts.

## 3D Dog Navigation Reference

- Repository: <https://github.com/ypat999/3d_dog_navi_ros2>

How it is used here:

- The repository is used as a target-effect and topic-interface reference for
  Go2W visual navigation, FAST-LIO2, Livox-style topics, point-cloud map
  displays, and `/cmd_vel` navigation control.
- The mirrored topic surface is `/livox/lidar`, `/livox/imu`,
  `/cloud_registered`, `/Laser_map`, `/odom`, and `/cmd_vel`.
- The current AIROS implementation does not vendor the full hardware Livox
  driver or PCT/ego planner runtime.
