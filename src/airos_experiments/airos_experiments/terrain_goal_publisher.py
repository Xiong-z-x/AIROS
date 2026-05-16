from __future__ import annotations

import argparse
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.utilities import remove_ros_args


def _build_goal(args: argparse.Namespace) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = str(args.frame_id)
    msg.pose.position.x = float(args.x)
    msg.pose.position.y = float(args.y)
    msg.pose.position.z = float(args.z)
    msg.pose.orientation.w = 1.0
    return msg


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Publish a repeated 3D terrain goal for FAST-LIO/PCT runs.',
    )
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--z', type=float, required=True)
    parser.add_argument('--frame-id', default='map')
    parser.add_argument('--topic', default='/terrain_goal_pose')
    parser.add_argument('--publish-count', type=int, default=5)
    parser.add_argument('--rate-hz', type=float, default=1.0)
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    rclpy.init(args=sys.argv)
    node = rclpy.create_node('terrain_goal_publisher')
    publisher = node.create_publisher(PoseStamped, str(args.topic), 10)
    period_sec = 1.0 / max(float(args.rate_hz), 1e-6)
    goal = _build_goal(args)
    try:
        for _ in range(max(1, int(args.publish_count))):
            goal.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(goal)
            rclpy.spin_once(node, timeout_sec=period_sec)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
