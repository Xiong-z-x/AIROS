from __future__ import annotations

import math
import struct
from typing import Iterable

import rclpy
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2


def _field_offsets(msg: PointCloud2) -> dict[str, int]:
    return {field.name: field.offset for field in msg.fields}


def _iter_xyz_points(msg: PointCloud2) -> Iterable[tuple[float, float, float]]:
    offsets = _field_offsets(msg)
    required = {'x', 'y', 'z'}
    if not required.issubset(offsets):
        return
    point_step = int(msg.point_step)
    if point_step <= 0:
        return
    data = msg.data
    x_offset = offsets['x']
    y_offset = offsets['y']
    z_offset = offsets['z']
    endian = '>' if msg.is_bigendian else '<'
    total = int(msg.width) * int(msg.height)
    for index in range(total):
        base = index * point_step
        try:
            x = struct.unpack_from(endian + 'f', data, base + x_offset)[0]
            y = struct.unpack_from(endian + 'f', data, base + y_offset)[0]
            z = struct.unpack_from(endian + 'f', data, base + z_offset)[0]
        except struct.error:
            break
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            yield float(x), float(y), float(z)


class LivoxCustomBridge(Node):
    def __init__(self) -> None:
        super().__init__('livox_custom_bridge')
        self.declare_parameter('input_topic', '/livox/lidar_points')
        self.declare_parameter('output_topic', '/livox/lidar')
        self.declare_parameter('scan_line', 16)
        self.declare_parameter('scan_period_us', 100000)
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self._scan_line = max(1, int(self.get_parameter('scan_line').value))
        self._scan_period_us = max(
            1,
            int(self.get_parameter('scan_period_us').value),
        )
        qos = QoSProfile(depth=2)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self._publisher = self.create_publisher(CustomMsg, output_topic, qos)
        self._subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self._on_cloud,
            qos,
        )
        self.get_logger().info(
            f'livox custom bridge ready: {input_topic} -> {output_topic}'
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        points = list(_iter_xyz_points(msg))
        if not points:
            return
        livox_msg = CustomMsg()
        livox_msg.header = msg.header
        livox_msg.timebase = (
            int(msg.header.stamp.sec) * 1000000000
            + int(msg.header.stamp.nanosec)
        )
        livox_msg.lidar_id = 1
        livox_msg.rsvd = [0, 0, 0]
        livox_msg.point_num = len(points)
        point_count = max(1, len(points) - 1)
        livox_points: list[CustomPoint] = []
        for index, (x, y, z) in enumerate(points):
            point = CustomPoint()
            point.offset_time = int(index * self._scan_period_us / point_count)
            point.x = x
            point.y = y
            point.z = z
            point.reflectivity = 100
            point.tag = 0x10
            point.line = index % self._scan_line
            livox_points.append(point)
        livox_msg.points = livox_points
        self._publisher.publish(livox_msg)


def main() -> None:
    rclpy.init()
    node = LivoxCustomBridge()
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
