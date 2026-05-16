from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import PointCloud2


@dataclass
class ProbeSnapshot:
    elapsed_sec: float
    goal_xyz: tuple[float, float, float]
    laser_map_points: int | None
    pct_path_poses: int
    pct_path_max_z: float | None
    cmd_vel_nav_norm: float | None
    cmd_vel_nav_count: int
    cmd_vel_nav_age_sec: float | None
    cmd_vel_smoothed_norm: float | None
    cmd_vel_smoothed_count: int
    cmd_vel_smoothed_age_sec: float | None
    base_cmd_norm: float | None
    base_cmd_count: int
    base_cmd_age_sec: float | None
    fast_lio_xyz: tuple[float, float, float] | None
    fast_lio_goal_xy_distance: float | None
    wheel_odom_xyz: tuple[float, float, float] | None
    wheel_goal_xy_distance: float | None
    gazebo_xyz: tuple[float, float, float] | None
    gazebo_goal_xy_distance: float | None


def _twist_norm(msg: Twist | None) -> float | None:
    if msg is None:
        return None
    return round(
        abs(float(msg.linear.x)) + abs(float(msg.angular.z)),
        6,
    )


def _odom_xyz(msg: Odometry | None) -> tuple[float, float, float] | None:
    if msg is None:
        return None
    position = msg.pose.pose.position
    return (
        round(float(position.x), 6),
        round(float(position.y), 6),
        round(float(position.z), 6),
    )


def _goal_xy_distance(
    xyz: tuple[float, float, float] | None,
    goal_xy: tuple[float, float],
) -> float | None:
    if xyz is None:
        return None
    return round(math.hypot(xyz[0] - goal_xy[0], xyz[1] - goal_xy[1]), 6)


def _path_max_z(msg: NavPath | None) -> float | None:
    if msg is None or not msg.poses:
        return None
    return round(max(float(pose.pose.position.z) for pose in msg.poses), 6)


def _point_count(msg: PointCloud2 | None) -> int | None:
    if msg is None:
        return None
    return int(msg.width) * int(msg.height)


def _message_age_sec(now_monotonic: float, last_monotonic: float | None) -> float | None:
    if last_monotonic is None:
        return None
    return round(max(0.0, now_monotonic - last_monotonic), 3)


def _run_ign_pose_query(
    world: str,
    entity: str,
    timeout_sec: float,
) -> tuple[float, float, float] | None:
    try:
        result = subprocess.run(
            [
                'ign',
                'topic',
                '-e',
                '--json-output',
                '-n',
                '1',
                '-t',
                f'/world/{world}/pose/info',
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_sec,
        )
    except subprocess.TimeoutExpired:
        return None
    if result.returncode != 0 or not result.stdout.strip():
        return None

    payload = None
    for line in result.stdout.splitlines():
        if not line.strip():
            continue
        payload = json.loads(line)
        break
    if payload is None:
        return None
    for pose in payload.get('pose', []):
        if pose.get('name') != entity:
            continue
        position = pose.get('position', {})
        return (
            round(float(position.get('x', 0.0)), 6),
            round(float(position.get('y', 0.0)), 6),
            round(float(position.get('z', 0.0)), 6),
        )
    return None


class CrossLevelEvidenceProbe(Node):
    def __init__(self) -> None:
        super().__init__('cross_level_evidence_probe')

        self.declare_parameter('laser_map_topic', '/Laser_map_world')
        self.declare_parameter('path_topic', '/pct_path')
        self.declare_parameter('cmd_vel_nav_topic', '/cmd_vel_nav')
        self.declare_parameter('cmd_vel_smoothed_topic', '/cmd_vel_smoothed')
        self.declare_parameter(
            'base_cmd_topic',
            '/diff_drive_controller/cmd_vel_unstamped',
        )
        self.declare_parameter('fast_lio_odom_topic', '/fast_lio_odom_world')
        self.declare_parameter('wheel_odom_topic', '/odom')

        self._laser_map: PointCloud2 | None = None
        self._path: NavPath | None = None
        self._cmd_vel_nav: Twist | None = None
        self._cmd_vel_smoothed: Twist | None = None
        self._base_cmd: Twist | None = None
        self._fast_lio_odom: Odometry | None = None
        self._wheel_odom: Odometry | None = None
        self._cmd_vel_nav_count = 0
        self._cmd_vel_smoothed_count = 0
        self._base_cmd_count = 0
        self._cmd_vel_nav_last_monotonic: float | None = None
        self._cmd_vel_smoothed_last_monotonic: float | None = None
        self._base_cmd_last_monotonic: float | None = None

        self.create_subscription(
            PointCloud2,
            str(self.get_parameter('laser_map_topic').value),
            self._laser_map_callback,
            10,
        )
        self.create_subscription(
            NavPath,
            str(self.get_parameter('path_topic').value),
            self._path_callback,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('cmd_vel_nav_topic').value),
            self._cmd_vel_nav_callback,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('cmd_vel_smoothed_topic').value),
            self._cmd_vel_smoothed_callback,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('base_cmd_topic').value),
            self._base_cmd_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('fast_lio_odom_topic').value),
            self._fast_lio_odom_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('wheel_odom_topic').value),
            self._wheel_odom_callback,
            10,
        )

    def _laser_map_callback(self, msg: PointCloud2) -> None:
        self._laser_map = msg

    def _path_callback(self, msg: NavPath) -> None:
        self._path = msg

    def _cmd_vel_nav_callback(self, msg: Twist) -> None:
        self._cmd_vel_nav = msg
        self._cmd_vel_nav_count += 1
        self._cmd_vel_nav_last_monotonic = time.monotonic()

    def _cmd_vel_smoothed_callback(self, msg: Twist) -> None:
        self._cmd_vel_smoothed = msg
        self._cmd_vel_smoothed_count += 1
        self._cmd_vel_smoothed_last_monotonic = time.monotonic()

    def _base_cmd_callback(self, msg: Twist) -> None:
        self._base_cmd = msg
        self._base_cmd_count += 1
        self._base_cmd_last_monotonic = time.monotonic()

    def _fast_lio_odom_callback(self, msg: Odometry) -> None:
        self._fast_lio_odom = msg

    def _wheel_odom_callback(self, msg: Odometry) -> None:
        self._wheel_odom = msg

    def snapshot(
        self,
        elapsed_sec: float,
        world: str,
        entity: str,
        goal_xyz: tuple[float, float, float],
        ign_timeout_sec: float,
    ) -> ProbeSnapshot:
        now_monotonic = time.monotonic()
        gazebo_xyz = _run_ign_pose_query(world, entity, ign_timeout_sec)
        fast_lio_xyz = _odom_xyz(self._fast_lio_odom)
        wheel_odom_xyz = _odom_xyz(self._wheel_odom)
        goal_xy = (goal_xyz[0], goal_xyz[1])
        return ProbeSnapshot(
            elapsed_sec=round(elapsed_sec, 3),
            goal_xyz=(
                round(float(goal_xyz[0]), 6),
                round(float(goal_xyz[1]), 6),
                round(float(goal_xyz[2]), 6),
            ),
            laser_map_points=_point_count(self._laser_map),
            pct_path_poses=0 if self._path is None else len(self._path.poses),
            pct_path_max_z=_path_max_z(self._path),
            cmd_vel_nav_norm=_twist_norm(self._cmd_vel_nav),
            cmd_vel_nav_count=self._cmd_vel_nav_count,
            cmd_vel_nav_age_sec=_message_age_sec(
                now_monotonic,
                self._cmd_vel_nav_last_monotonic,
            ),
            cmd_vel_smoothed_norm=_twist_norm(self._cmd_vel_smoothed),
            cmd_vel_smoothed_count=self._cmd_vel_smoothed_count,
            cmd_vel_smoothed_age_sec=_message_age_sec(
                now_monotonic,
                self._cmd_vel_smoothed_last_monotonic,
            ),
            base_cmd_norm=_twist_norm(self._base_cmd),
            base_cmd_count=self._base_cmd_count,
            base_cmd_age_sec=_message_age_sec(
                now_monotonic,
                self._base_cmd_last_monotonic,
            ),
            fast_lio_xyz=fast_lio_xyz,
            fast_lio_goal_xy_distance=_goal_xy_distance(fast_lio_xyz, goal_xy),
            wheel_odom_xyz=wheel_odom_xyz,
            wheel_goal_xy_distance=_goal_xy_distance(wheel_odom_xyz, goal_xy),
            gazebo_xyz=gazebo_xyz,
            gazebo_goal_xy_distance=_goal_xy_distance(gazebo_xyz, goal_xy),
        )


def _snapshot_to_dict(snapshot: ProbeSnapshot) -> dict[str, Any]:
    return {
        'elapsed_sec': snapshot.elapsed_sec,
        'goal_xyz': snapshot.goal_xyz,
        'laser_map_points': snapshot.laser_map_points,
        'pct_path_poses': snapshot.pct_path_poses,
        'pct_path_max_z': snapshot.pct_path_max_z,
        'cmd_vel_nav_norm': snapshot.cmd_vel_nav_norm,
        'cmd_vel_nav_count': snapshot.cmd_vel_nav_count,
        'cmd_vel_nav_age_sec': snapshot.cmd_vel_nav_age_sec,
        'cmd_vel_smoothed_norm': snapshot.cmd_vel_smoothed_norm,
        'cmd_vel_smoothed_count': snapshot.cmd_vel_smoothed_count,
        'cmd_vel_smoothed_age_sec': snapshot.cmd_vel_smoothed_age_sec,
        'base_cmd_norm': snapshot.base_cmd_norm,
        'base_cmd_count': snapshot.base_cmd_count,
        'base_cmd_age_sec': snapshot.base_cmd_age_sec,
        'fast_lio_xyz': snapshot.fast_lio_xyz,
        'fast_lio_goal_xy_distance': snapshot.fast_lio_goal_xy_distance,
        'wheel_odom_xyz': snapshot.wheel_odom_xyz,
        'wheel_goal_xy_distance': snapshot.wheel_goal_xy_distance,
        'gazebo_xyz': snapshot.gazebo_xyz,
        'gazebo_goal_xy_distance': snapshot.gazebo_goal_xy_distance,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Collect bounded cross-level navigation evidence.',
    )
    parser.add_argument('--output', default='log/cross_level_evidence_probe.jsonl')
    parser.add_argument('--duration-sec', type=float, default=120.0)
    parser.add_argument('--sample-period-sec', type=float, default=5.0)
    parser.add_argument('--world', default='large_multilevel_complex')
    parser.add_argument('--entity', default='go2w_nav_eq')
    parser.add_argument('--goal-x', type=float, default=6.0)
    parser.add_argument('--goal-y', type=float, default=13.0)
    parser.add_argument('--goal-z', type=float, default=2.2)
    parser.add_argument('--ign-timeout-sec', type=float, default=3.0)
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=sys.argv)
    node = CrossLevelEvidenceProbe()
    start = time.monotonic()
    next_sample = start
    try:
        with output.open('w', encoding='utf-8') as stream:
            while rclpy.ok() and time.monotonic() - start <= args.duration_sec:
                now = time.monotonic()
                if now >= next_sample:
                    snapshot = node.snapshot(
                        elapsed_sec=now - start,
                        world=args.world,
                        entity=args.entity,
                        goal_xyz=(args.goal_x, args.goal_y, args.goal_z),
                        ign_timeout_sec=args.ign_timeout_sec,
                    )
                    line = json.dumps(_snapshot_to_dict(snapshot), ensure_ascii=False)
                    stream.write(line + '\n')
                    stream.flush()
                    print(line)
                    next_sample = now + max(0.1, args.sample_period_sec)
                rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
