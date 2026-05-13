from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan, PointCloud2

from airos_experiments.scan_emulator import _yaw_from_quaternion
from airos_experiments.slam_traversability_graph import sample_xyz_points


def project_cloud_to_scan(
    cloud: PointCloud2,
    odom: Odometry,
    *,
    frame_id: str,
    angle_min: float,
    angle_max: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    min_z: float,
    max_z: float,
    max_points: int = 120000,
) -> LaserScan:
    scan = LaserScan()
    scan.header.stamp = cloud.header.stamp
    scan.header.frame_id = frame_id
    scan.angle_min = angle_min
    scan.angle_max = angle_max
    scan.angle_increment = angle_increment
    scan.time_increment = 0.0
    scan.scan_time = 0.0
    scan.range_min = range_min
    scan.range_max = range_max

    beam_count = max(1, int(math.floor((angle_max - angle_min) / angle_increment)) + 1)
    ranges = [math.inf] * beam_count

    base_x = float(odom.pose.pose.position.x)
    base_y = float(odom.pose.pose.position.y)
    base_z = float(odom.pose.pose.position.z)
    orientation = odom.pose.pose.orientation
    base_yaw = _yaw_from_quaternion(
        float(orientation.x),
        float(orientation.y),
        float(orientation.z),
        float(orientation.w),
    )
    cos_yaw = math.cos(-base_yaw)
    sin_yaw = math.sin(-base_yaw)

    for x, y, z in sample_xyz_points(cloud, max_points=max_points):
        relative_z = z - base_z
        if relative_z < min_z or relative_z > max_z:
            continue
        dx = x - base_x
        dy = y - base_y
        local_x = cos_yaw * dx - sin_yaw * dy
        local_y = sin_yaw * dx + cos_yaw * dy
        distance = math.hypot(local_x, local_y)
        if distance < range_min or distance > range_max:
            continue
        angle = math.atan2(local_y, local_x)
        if angle < angle_min or angle > angle_max:
            continue
        index = int(round((angle - angle_min) / angle_increment))
        if 0 <= index < beam_count and distance < ranges[index]:
            ranges[index] = float(distance)

    scan.ranges = ranges
    return scan


class SlamScanProjector(Node):
    def __init__(self) -> None:
        super().__init__('slam_scan_projector')
        self.declare_parameter('cloud_topic', '/Laser_map_world')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/slam_scan')
        self.declare_parameter('scan_frame', 'base_footprint')
        self.declare_parameter('publish_rate_hz', 6.0)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', math.radians(1.0))
        self.declare_parameter('range_min', 0.08)
        self.declare_parameter('range_max', 4.5)
        self.declare_parameter('min_z', 0.08)
        self.declare_parameter('max_z', 1.40)
        self.declare_parameter('max_points', 120000)

        self._cloud: PointCloud2 | None = None
        self._odom: Odometry | None = None
        self._scan_frame = str(self.get_parameter('scan_frame').value)
        self._angle_min = float(self.get_parameter('angle_min').value)
        self._angle_max = float(self.get_parameter('angle_max').value)
        self._angle_increment = float(self.get_parameter('angle_increment').value)
        self._range_min = float(self.get_parameter('range_min').value)
        self._range_max = float(self.get_parameter('range_max').value)
        self._min_z = float(self.get_parameter('min_z').value)
        self._max_z = float(self.get_parameter('max_z').value)
        self._max_points = int(self.get_parameter('max_points').value)

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            qos,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter('cloud_topic').value),
            self._cloud_callback,
            qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_callback,
            qos,
        )
        self.create_timer(
            1.0 / max(float(self.get_parameter('publish_rate_hz').value), 0.1),
            self._publish_scan,
        )
        self.get_logger().info(
            'projecting FAST-LIO map cloud to safety scan: '
            f'{self.get_parameter("cloud_topic").value} -> '
            f'{self.get_parameter("scan_topic").value}'
        )

    def _cloud_callback(self, msg: PointCloud2) -> None:
        self._cloud = msg

    def _odom_callback(self, msg: Odometry) -> None:
        self._odom = msg

    def _publish_scan(self) -> None:
        if self._cloud is None or self._odom is None:
            return
        scan = project_cloud_to_scan(
            self._cloud,
            self._odom,
            frame_id=self._scan_frame,
            angle_min=self._angle_min,
            angle_max=self._angle_max,
            angle_increment=self._angle_increment,
            range_min=self._range_min,
            range_max=self._range_max,
            min_z=self._min_z,
            max_z=self._max_z,
            max_points=self._max_points,
        )
        scan.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(scan)


def main() -> None:
    rclpy.init()
    node = SlamScanProjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
