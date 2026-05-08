# livox_ros_driver2 interface shim

This package intentionally provides only the Livox `CustomMsg` and
`CustomPoint` message ABI required by the imported `fast_lio` package.

It does not include the hardware Livox driver node because that driver depends
on Livox SDK2 and `liblivox_lidar_sdk_shared.so` installed under
`/usr/local/lib`. AIROS simulation publishes `/livox/lidar` as
`sensor_msgs/msg/PointCloud2`, so FAST-LIO is configured for the generic
PointCloud2 path.
