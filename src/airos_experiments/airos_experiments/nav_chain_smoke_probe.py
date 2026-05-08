from __future__ import annotations

import argparse
import json
import math
import time
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import LaserScan


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class NavChainSmokeProbe(Node):
    def __init__(self) -> None:
        super().__init__('nav_chain_smoke_probe')

        self.declare_parameter('goal_x', 0.25)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('relative_goal', True)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('compute_action', '/compute_path_to_pose')
        self.declare_parameter('navigate_action', '/navigate_to_pose')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_nav_topic', '/cmd_vel_nav')
        self.declare_parameter('cmd_vel_smoothed_topic', '/cmd_vel_smoothed')
        self.declare_parameter(
            'base_cmd_topic',
            '/diff_drive_controller/cmd_vel_unstamped',
        )
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('server_timeout_sec', 8.0)
        self.declare_parameter('result_timeout_sec', 15.0)
        self.declare_parameter('warmup_sec', 3.0)
        self.declare_parameter('initial_odom_timeout_sec', 8.0)
        self.declare_parameter('min_odom_delta_m', 0.01)
        self.declare_parameter('min_clearance_m', 0.30)

        self._compute_client = ActionClient(
            self,
            ComputePathToPose,
            str(self.get_parameter('compute_action').value),
        )
        self._navigate_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter('navigate_action').value),
        )
        self._odom_samples: list[tuple[float, float]] = []
        self._cmd_nav_samples: list[tuple[float, float]] = []
        self._cmd_smoothed_samples: list[tuple[float, float]] = []
        self._base_cmd_samples: list[tuple[float, float]] = []
        self._feedback_samples: list[dict[str, float | int]] = []
        self._min_scan_range = math.inf

        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_callback,
            20,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('cmd_vel_nav_topic').value),
            lambda msg: self._cmd_nav_samples.append(
                (float(msg.linear.x), float(msg.angular.z))
            ),
            20,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('cmd_vel_smoothed_topic').value),
            lambda msg: self._cmd_smoothed_samples.append(
                (float(msg.linear.x), float(msg.angular.z))
            ),
            20,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('base_cmd_topic').value),
            lambda msg: self._base_cmd_samples.append(
                (float(msg.linear.x), float(msg.angular.z))
            ),
            20,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self._scan_callback,
            20,
        )

    def run(self) -> dict[str, Any]:
        self._spin_for(float(self.get_parameter('warmup_sec').value))
        self._wait_for_initial_odom()
        initial_odom = self._odom_samples[-1] if self._odom_samples else None
        goal_pose = self._make_goal_pose(initial_odom)
        report: dict[str, Any] = {
            'goal': {
                'x': float(goal_pose.pose.position.x),
                'y': float(goal_pose.pose.position.y),
                'yaw': float(self.get_parameter('goal_yaw').value),
            },
            'relative_goal': bool(self.get_parameter('relative_goal').value),
            'initial_odom': initial_odom,
            'initial_min_scan_range_m': self._finite_scan_range(),
        }

        report.update(self._compute_path(goal_pose))
        report.update(self._navigate(goal_pose))
        self._spin_for(0.5)
        report.update(self._summarize_motion())
        report['final_min_scan_range_m'] = self._finite_scan_range()
        report['success'] = self._success(report)
        return report

    def _make_goal_pose(
        self,
        initial_odom: tuple[float, float] | None,
    ) -> PoseStamped:
        goal_x = float(self.get_parameter('goal_x').value)
        goal_y = float(self.get_parameter('goal_y').value)
        if bool(self.get_parameter('relative_goal').value) and initial_odom:
            goal_x += initial_odom[0]
            goal_y += initial_odom[1]
        qx, qy, qz, qw = _yaw_to_quaternion(
            float(self.get_parameter('goal_yaw').value)
        )
        pose = PoseStamped()
        pose.header.frame_id = str(self.get_parameter('global_frame').value)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _compute_path(self, goal_pose: PoseStamped) -> dict[str, Any]:
        timeout = float(self.get_parameter('server_timeout_sec').value)
        if not self._compute_client.wait_for_server(timeout_sec=timeout):
            return {'compute_server_ready': False}

        goal = ComputePathToPose.Goal()
        goal.goal = goal_pose
        goal.use_start = False
        send_future = self._compute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return {
                'compute_server_ready': True,
                'compute_goal_accepted': False,
            }

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=float(self.get_parameter('result_timeout_sec').value),
        )
        result = result_future.result()
        if result is None:
            return {
                'compute_server_ready': True,
                'compute_goal_accepted': True,
                'compute_result_ready': False,
            }

        poses = result.result.path.poses
        return {
            'compute_server_ready': True,
            'compute_goal_accepted': True,
            'compute_result_ready': True,
            'compute_status': int(result.status),
            'path_pose_count': len(poses),
        }

    def _navigate(self, goal_pose: PoseStamped) -> dict[str, Any]:
        timeout = float(self.get_parameter('server_timeout_sec').value)
        if not self._navigate_client.wait_for_server(timeout_sec=timeout):
            return {'navigate_server_ready': False}

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        send_future = self._navigate_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return {
                'navigate_server_ready': True,
                'navigate_goal_accepted': False,
            }

        result_future = handle.get_result_async()
        deadline = time.monotonic() + float(
            self.get_parameter('result_timeout_sec').value
        )
        while rclpy.ok() and time.monotonic() < deadline:
            if result_future.done():
                break
            rclpy.spin_once(self, timeout_sec=0.1)

        if not result_future.done():
            cancel_future = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            return {
                'navigate_server_ready': True,
                'navigate_goal_accepted': True,
                'navigate_result_ready': False,
            }

        result = result_future.result()
        return {
            'navigate_server_ready': True,
            'navigate_goal_accepted': True,
            'navigate_result_ready': True,
            'navigate_status': int(result.status) if result else None,
            'feedback_count': len(self._feedback_samples),
            'last_feedback': self._feedback_samples[-1]
            if self._feedback_samples else None,
        }

    def _feedback_callback(self, msg) -> None:
        feedback = msg.feedback
        pose = feedback.current_pose.pose.position
        self._feedback_samples.append({
            'x': float(pose.x),
            'y': float(pose.y),
            'distance_remaining': float(feedback.distance_remaining),
            'recoveries': int(feedback.number_of_recoveries),
        })

    def _summarize_motion(self) -> dict[str, Any]:
        return {
            'odom_sample_count': len(self._odom_samples),
            'odom_delta_m': self._odom_delta(),
            'cmd_nav_nonzero_count': self._nonzero_count(self._cmd_nav_samples),
            'cmd_smoothed_nonzero_count': self._nonzero_count(
                self._cmd_smoothed_samples
            ),
            'base_cmd_nonzero_count': self._nonzero_count(
                self._base_cmd_samples
            ),
        }

    def _success(self, report: dict[str, Any]) -> bool:
        min_clearance = float(self.get_parameter('min_clearance_m').value)
        return (
            report.get('compute_status') == GoalStatus.STATUS_SUCCEEDED
            and int(report.get('path_pose_count', 0)) > 1
            and report.get('navigate_status') == GoalStatus.STATUS_SUCCEEDED
            and float(report.get('odom_delta_m', 0.0)) >= float(
                self.get_parameter('min_odom_delta_m').value
            )
            and int(report.get('cmd_nav_nonzero_count', 0)) > 0
            and int(report.get('cmd_smoothed_nonzero_count', 0)) > 0
            and int(report.get('base_cmd_nonzero_count', 0)) > 0
            and (
                report.get('final_min_scan_range_m') is None
                or float(report['final_min_scan_range_m']) >= min_clearance
            )
        )

    def _odom_callback(self, msg: Odometry) -> None:
        self._odom_samples.append((
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        ))

    def _scan_callback(self, msg: LaserScan) -> None:
        ranges = [
            float(value)
            for value in msg.ranges
            if math.isfinite(value) and msg.range_min <= value <= msg.range_max
        ]
        if ranges:
            self._min_scan_range = min(self._min_scan_range, min(ranges))

    def _spin_for(self, duration_sec: float) -> None:
        deadline = time.monotonic() + max(0.0, duration_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _wait_for_initial_odom(self) -> None:
        deadline = time.monotonic() + float(
            self.get_parameter('initial_odom_timeout_sec').value
        )
        while rclpy.ok() and not self._odom_samples and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _odom_delta(self) -> float:
        if len(self._odom_samples) < 2:
            return 0.0
        start = self._odom_samples[0]
        end = self._odom_samples[-1]
        return math.hypot(end[0] - start[0], end[1] - start[1])

    @staticmethod
    def _nonzero_count(samples: list[tuple[float, float]]) -> int:
        return sum(
            1
            for linear, angular in samples
            if abs(linear) > 1e-4 or abs(angular) > 1e-4
        )

    def _finite_scan_range(self) -> float | None:
        if math.isfinite(self._min_scan_range):
            return self._min_scan_range
        return None


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Verify the AIROS Nav2 planning and control smoke chain.'
    )
    parser.add_argument('--goal-x', type=float, default=0.25)
    parser.add_argument('--goal-y', type=float, default=0.0)
    parser.add_argument('--goal-yaw', type=float, default=0.0)
    parser.add_argument(
        '--absolute-goal',
        action='store_true',
        help='Interpret goal coordinates in map frame instead of odom-relative.',
    )
    args = parser.parse_args(remove_ros_args()[1:])

    rclpy.init()
    node = NavChainSmokeProbe()
    node.set_parameters([
        rclpy.parameter.Parameter(
            'goal_x',
            rclpy.Parameter.Type.DOUBLE,
            args.goal_x,
        ),
        rclpy.parameter.Parameter(
            'goal_y',
            rclpy.Parameter.Type.DOUBLE,
            args.goal_y,
        ),
        rclpy.parameter.Parameter(
            'goal_yaw',
            rclpy.Parameter.Type.DOUBLE,
            args.goal_yaw,
        ),
        rclpy.parameter.Parameter(
            'relative_goal',
            rclpy.Parameter.Type.BOOL,
            not args.absolute_goal,
        ),
    ])
    try:
        report = node.run()
        print(json.dumps(report, ensure_ascii=False, sort_keys=True))
        if not report['success']:
            raise SystemExit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
