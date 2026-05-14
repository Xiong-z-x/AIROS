from __future__ import annotations

import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

from airos_experiments.fast_lio_frame_alignment import (
    FrameAlignment,
    transform_point,
)


def aligned_xyz_points(
    msg: PointCloud2,
    alignment: FrameAlignment,
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
    points: list[tuple[float, float, float]] = []
    for index in range(0, total, stride):
        base = index * point_step
        try:
            x = struct.unpack_from(endian + 'f', msg.data, base + offsets['x'])[0]
            y = struct.unpack_from(endian + 'f', msg.data, base + offsets['y'])[0]
            z = struct.unpack_from(endian + 'f', msg.data, base + offsets['z'])[0]
        except struct.error:
            break
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            points.append(transform_point(float(x), float(y), float(z), alignment))
    return points


class FastLioMapAligner(Node):
    def __init__(self) -> None:
        super().__init__('fast_lio_map_aligner')
        self.declare_parameter('input_topic', '/Laser_map')
        self.declare_parameter('output_topic', '/Laser_map_world')
        self.declare_parameter('output_frame', 'map')
        self.declare_parameter('max_points', 800000)
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('spawn_z', 0.0)
        self.declare_parameter('spawn_yaw', 0.0)

        self._alignment = FrameAlignment(
            spawn_x=float(self.get_parameter('spawn_x').value),
            spawn_y=float(self.get_parameter('spawn_y').value),
            spawn_z=float(self.get_parameter('spawn_z').value),
            spawn_yaw=float(self.get_parameter('spawn_yaw').value),
        )
        self._output_frame = str(self.get_parameter('output_frame').value)
        self._max_points = int(self.get_parameter('max_points').value)

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('output_topic').value),
            qos,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter('input_topic').value),
            self._on_cloud,
            qos,
        )
        self.get_logger().info(
            'aligning FAST-LIO map to world frame: '
            f'{self.get_parameter("input_topic").value} -> '
            f'{self.get_parameter("output_topic").value}'
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        points = aligned_xyz_points(
            msg,
            self._alignment,
            max_points=self._max_points,
        )
        header = msg.header
        header.frame_id = self._output_frame
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        self._publisher.publish(point_cloud2.create_cloud(header, fields, points))


def main() -> None:
    rclpy.init()
    node = FastLioMapAligner()
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
