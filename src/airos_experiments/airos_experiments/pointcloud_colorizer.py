from __future__ import annotations

import math
import struct
from typing import Iterable

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _pack_rgb_float(red: int, green: int, blue: int) -> float:
    rgb_uint32 = (red << 16) | (green << 8) | blue
    return struct.unpack('f', struct.pack('I', rgb_uint32))[0]


def _height_rgb(z: float, min_z: float, max_z: float) -> tuple[int, int, int]:
    if max_z <= min_z:
        return 255, 255, 255
    t = _clamp((z - min_z) / (max_z - min_z), 0.0, 1.0)
    if t < 0.33:
        k = t / 0.33
        return int(40 * (1.0 - k) + 30 * k), int(120 * k), 255
    if t < 0.66:
        k = (t - 0.33) / 0.33
        return int(30 * (1.0 - k) + 255 * k), int(120 + 120 * k), int(255 * (1.0 - k))
    k = (t - 0.66) / 0.34
    return 255, int(240 * (1.0 - k) + 60 * k), int(40 * k)


def colorize_points(
    points: Iterable[tuple[float, float, float]],
    *,
    min_z: float,
    max_z: float,
) -> list[tuple[float, float, float, float]]:
    colored: list[tuple[float, float, float, float]] = []
    for x, y, z in points:
        red, green, blue = _height_rgb(float(z), min_z, max_z)
        colored.append((float(x), float(y), float(z), _pack_rgb_float(red, green, blue)))
    return colored


def sample_xyz_points(
    msg: PointCloud2,
    *,
    max_points: int,
) -> list[tuple[float, float, float]]:
    offsets = {field.name: field.offset for field in msg.fields}
    if not {'x', 'y', 'z'}.issubset(offsets):
        return []
    point_step = int(msg.point_step)
    total = int(msg.width) * int(msg.height)
    if point_step <= 0 or total <= 0:
        return []
    stride = 1
    if max_points > 0 and total > max_points:
        stride = int(math.ceil(total / max_points))

    endian = '>' if msg.is_bigendian else '<'
    x_offset = offsets['x']
    y_offset = offsets['y']
    z_offset = offsets['z']
    points: list[tuple[float, float, float]] = []
    for index in range(0, total, stride):
        base = index * point_step
        try:
            x = struct.unpack_from(endian + 'f', msg.data, base + x_offset)[0]
            y = struct.unpack_from(endian + 'f', msg.data, base + y_offset)[0]
            z = struct.unpack_from(endian + 'f', msg.data, base + z_offset)[0]
        except struct.error:
            break
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            points.append((float(x), float(y), float(z)))
    return points


class PointCloudColorizer(Node):
    def __init__(self) -> None:
        super().__init__('pointcloud_colorizer')
        self.declare_parameter('input_topic', '/Laser_map')
        self.declare_parameter('output_topic', '/Laser_map_colored')
        self.declare_parameter('min_z', -0.40)
        self.declare_parameter('max_z', 2.20)
        self.declare_parameter('max_points', 90000)

        self._input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self._min_z = float(self.get_parameter('min_z').value)
        self._max_z = float(self.get_parameter('max_z').value)
        self._max_points = int(self.get_parameter('max_points').value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        publish_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(PointCloud2, output_topic, publish_qos)
        self.create_subscription(PointCloud2, self._input_topic, self._on_cloud, qos)
        self.get_logger().info(
            f'colorizing point cloud {self._input_topic} -> {output_topic}'
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        sampled_points = sample_xyz_points(
            msg,
            max_points=self._max_points,
        )

        colored_points = colorize_points(
            sampled_points,
            min_z=self._min_z,
            max_z=self._max_z,
        )
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud = point_cloud2.create_cloud(msg.header, fields, colored_points)
        self._publisher.publish(cloud)


def main() -> None:
    rclpy.init()
    node = PointCloudColorizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
