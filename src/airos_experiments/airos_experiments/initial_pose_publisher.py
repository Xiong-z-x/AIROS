from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__('initial_pose_publisher')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('period_sec', 1.0)
        self.declare_parameter('publish_count', 6)

        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10,
        )
        self._sent = 0
        self.done = False
        period = max(float(self.get_parameter('period_sec').value), 0.1)
        self._timer = self.create_timer(period, self._publish)

    def _publish(self) -> None:
        yaw = float(self.get_parameter('yaw').value)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = str(self.get_parameter('frame_id').value)
        msg.pose.pose.position.x = float(self.get_parameter('x').value)
        msg.pose.pose.position.y = float(self.get_parameter('y').value)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self._publisher.publish(msg)
        self._sent += 1
        if self._sent >= int(self.get_parameter('publish_count').value):
            self.destroy_timer(self._timer)
            self.done = True


def main() -> None:
    rclpy.init()
    node = InitialPosePublisher()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
