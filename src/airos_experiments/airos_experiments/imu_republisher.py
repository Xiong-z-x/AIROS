from __future__ import annotations

import copy

import rclpy
from builtin_interfaces.msg import Time as RosTime
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Imu


class ImuRepublisher(Node):
    def __init__(self) -> None:
        super().__init__('imu_republisher')

        self.declare_parameter('input_topic', '/imu')
        self.declare_parameter('output_topic', '/livox/imu')
        self.declare_parameter('frame_id', 'livox_frame')

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._last_stamp_ns = 0
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._publisher = self.create_publisher(
            Imu,
            str(self.get_parameter('output_topic').value),
            qos,
        )
        self._subscription = self.create_subscription(
            Imu,
            str(self.get_parameter('input_topic').value),
            self._callback,
            qos,
        )
        self.get_logger().info(
            'imu republisher ready: '
            f"{self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value}"
        )

    def _callback(self, msg: Imu) -> None:
        out = copy.deepcopy(msg)
        out.header.stamp = self._monotonic_stamp(msg.header.stamp)
        out.header.frame_id = self._frame_id
        self._publisher.publish(out)

    def _monotonic_stamp(self, stamp: RosTime) -> RosTime:
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if stamp_ns <= self._last_stamp_ns:
            stamp_ns = self._last_stamp_ns + 1
        self._last_stamp_ns = stamp_ns
        return RosTime(
            sec=int(stamp_ns // 1_000_000_000),
            nanosec=int(stamp_ns % 1_000_000_000),
        )


def main() -> None:
    rclpy.init()
    node = ImuRepublisher()
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
