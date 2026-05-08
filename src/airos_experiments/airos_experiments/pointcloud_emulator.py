from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

import rclpy
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from airos_experiments.scan_emulator import (
    CircleObstacle,
    OdomAnchor,
    Pose2D,
    RectObstacle,
    _load_obstacles,
    _map_pose_from_anchor,
    _pose_from_initial_pose,
    _pose_from_odom,
)

CloudPoint = tuple[float, float, float, float]


def _axis_samples(
    center: float,
    half_extent: float,
    spacing: float,
) -> list[float]:
    length = max(half_extent * 2.0, spacing)
    count = max(int(math.ceil(length / spacing)), 1)
    start = center - half_extent
    return [start + min(i * spacing, length) for i in range(count + 1)]


def _z_samples(height: float, spacing: float) -> list[float]:
    top = min(max(height, 0.2), 2.2)
    count = max(int(math.ceil(top / spacing)), 1)
    return [min(i * spacing, top) for i in range(count + 1)]


def _rect_points(obstacle: RectObstacle, spacing: float) -> list[CloudPoint]:
    points: list[CloudPoint] = []
    min_x = obstacle.cx - obstacle.hx
    max_x = obstacle.cx + obstacle.hx
    min_y = obstacle.cy - obstacle.hy
    max_y = obstacle.cy + obstacle.hy
    for z in _z_samples(obstacle.height, spacing):
        for x in _axis_samples(obstacle.cx, obstacle.hx, spacing):
            points.append((x, min_y, z, 80.0))
            points.append((x, max_y, z, 80.0))
        for y in _axis_samples(obstacle.cy, obstacle.hy, spacing):
            points.append((min_x, y, z, 80.0))
            points.append((max_x, y, z, 80.0))
    return points


def _circle_points(
    obstacle: CircleObstacle,
    spacing: float,
) -> list[CloudPoint]:
    points: list[CloudPoint] = []
    circumference = 2.0 * math.pi * obstacle.radius
    count = max(int(math.ceil(circumference / spacing)), 12)
    for z in _z_samples(obstacle.height, spacing):
        for index in range(count):
            angle = 2.0 * math.pi * index / count
            points.append((
                obstacle.cx + math.cos(angle) * obstacle.radius,
                obstacle.cy + math.sin(angle) * obstacle.radius,
                z,
                95.0,
            ))
    return points


def _obstacles_to_cloud(
    obstacles: list[RectObstacle | CircleObstacle],
    spacing: float,
) -> list[CloudPoint]:
    points: list[CloudPoint] = []
    for obstacle in obstacles:
        if isinstance(obstacle, RectObstacle):
            points.extend(_rect_points(obstacle, spacing))
        else:
            points.extend(_circle_points(obstacle, spacing))
    return points


class PointCloudEmulator(Node):
    def __init__(self) -> None:
        super().__init__('pointcloud_emulator')

        self.declare_parameter('world_file', '')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('lidar_frame', 'livox_frame')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('initial_pose_topic', '/initialpose')
        self.declare_parameter('lidar_topic', '/livox/lidar')
        self.declare_parameter('registered_cloud_topic', '/cloud_registered')
        self.declare_parameter('map_cloud_topic', '/Laser_map')
        self.declare_parameter('publish_registered_cloud', True)
        self.declare_parameter('publish_map_cloud', True)
        self.declare_parameter('use_initial_pose_anchor', True)
        self.declare_parameter('sensor_offset_x', 0.25)
        self.declare_parameter('sensor_offset_y', 0.0)
        self.declare_parameter('sensor_z', 0.40)
        self.declare_parameter('sensor_yaw_offset', 0.0)
        self.declare_parameter('point_spacing', 0.28)
        self.declare_parameter('range_max', 14.0)
        self.declare_parameter('horizontal_fov_rad', 6.28318530718)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('map_publish_rate_hz', 0.5)
        self.declare_parameter('max_live_points', 5000)

        world_file = Path(str(self.get_parameter('world_file').value))
        obstacles = (
            _load_obstacles(world_file)
            if world_file.as_posix()
            else []
        )
        spacing = float(self.get_parameter('point_spacing').value)
        self._map_points = _obstacles_to_cloud(obstacles, spacing)

        self._world_frame = str(self.get_parameter('world_frame').value)
        self._lidar_frame = str(self.get_parameter('lidar_frame').value)
        self._sensor_offset_x = float(
            self.get_parameter('sensor_offset_x').value
        )
        self._sensor_offset_y = float(
            self.get_parameter('sensor_offset_y').value
        )
        self._sensor_z = float(self.get_parameter('sensor_z').value)
        self._sensor_yaw_offset = float(
            self.get_parameter('sensor_yaw_offset').value
        )
        self._range_max = float(self.get_parameter('range_max').value)
        self._horizontal_fov_rad = float(
            self.get_parameter('horizontal_fov_rad').value
        )
        self._max_live_points = int(
            self.get_parameter('max_live_points').value
        )
        self._use_initial_pose_anchor = bool(
            self.get_parameter('use_initial_pose_anchor').value
        )
        self._publish_registered_cloud = bool(
            self.get_parameter('publish_registered_cloud').value
        )
        self._publish_map_cloud = bool(
            self.get_parameter('publish_map_cloud').value
        )

        live_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._odom_subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_callback,
            live_qos,
        )
        self._initial_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter('initial_pose_topic').value),
            self._initial_pose_callback,
            10,
        )
        self._lidar_publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('lidar_topic').value),
            live_qos,
        )
        self._registered_publisher = (
            self.create_publisher(
                PointCloud2,
                str(self.get_parameter('registered_cloud_topic').value),
                live_qos,
            )
            if self._publish_registered_cloud
            else None
        )
        self._map_publisher = (
            self.create_publisher(
                PointCloud2,
                str(self.get_parameter('map_cloud_topic').value),
                map_qos,
            )
            if self._publish_map_cloud
            else None
        )

        self._odom_msg: Optional[Odometry] = None
        self._pending_initial_pose: Optional[Pose2D] = None
        self._odom_anchor: Optional[OdomAnchor] = None
        self._last_stamp_ns = 0
        live_period = 1.0 / max(
            float(self.get_parameter('publish_rate_hz').value),
            0.1,
        )
        map_period = 1.0 / max(
            float(self.get_parameter('map_publish_rate_hz').value),
            0.05,
        )
        self._live_timer = self.create_timer(live_period, self._publish_live)
        self._map_timer = (
            self.create_timer(map_period, self._publish_map)
            if self._publish_map_cloud
            else None
        )
        self.get_logger().info(
            'pointcloud emulator ready: '
            f'points={len(self._map_points)} world_file={world_file} '
            f'registered={self._publish_registered_cloud} '
            f'map={self._publish_map_cloud}'
        )

    def _odom_callback(self, msg: Odometry) -> None:
        self._odom_msg = msg
        if self._pending_initial_pose is not None:
            self._set_odom_anchor(self._pending_initial_pose, msg)
            self._pending_initial_pose = None

    def _initial_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        if not self._use_initial_pose_anchor:
            return
        initial_pose = _pose_from_initial_pose(msg)
        if self._odom_msg is None:
            self._pending_initial_pose = initial_pose
            return
        self._set_odom_anchor(initial_pose, self._odom_msg)

    def _set_odom_anchor(self, map_pose: Pose2D, odom_msg: Odometry) -> None:
        self._odom_anchor = OdomAnchor(
            map_pose=map_pose,
            odom_pose=_pose_from_odom(odom_msg),
        )

    def _current_sensor_pose(self) -> tuple[float, float, float]:
        if self._odom_msg is None:
            return 0.0, 0.0, 0.0

        base_pose = _pose_from_odom(self._odom_msg)
        if self._odom_anchor is not None:
            base_pose = _map_pose_from_anchor(base_pose, self._odom_anchor)
        yaw = base_pose.yaw
        offset_x = (
            math.cos(yaw) * self._sensor_offset_x
            - math.sin(yaw) * self._sensor_offset_y
        )
        offset_y = (
            math.sin(yaw) * self._sensor_offset_x
            + math.cos(yaw) * self._sensor_offset_y
        )
        return (
            base_pose.x + offset_x,
            base_pose.y + offset_y,
            yaw + self._sensor_yaw_offset,
        )

    def _make_cloud(
        self,
        frame_id: str,
        points: list[CloudPoint],
    ) -> PointCloud2:
        header = Header()
        header.stamp = self._monotonic_stamp()
        header.frame_id = frame_id
        fields = [
            PointField(
                name='x',
                offset=0,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name='y',
                offset=4,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name='z',
                offset=8,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name='intensity',
                offset=12,
                datatype=PointField.FLOAT32,
                count=1,
            ),
        ]
        return point_cloud2.create_cloud(header, fields, points)

    def _monotonic_stamp(self) -> RosTime:
        stamp = self.get_clock().now().to_msg()
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if stamp_ns <= self._last_stamp_ns:
            stamp_ns = self._last_stamp_ns + 1
        self._last_stamp_ns = stamp_ns
        stamp.sec = int(stamp_ns // 1_000_000_000)
        stamp.nanosec = int(stamp_ns % 1_000_000_000)
        return stamp

    def _visible_points(self) -> tuple[list[CloudPoint], list[CloudPoint]]:
        sensor_x, sensor_y, sensor_yaw = self._current_sensor_pose()
        cos_yaw = math.cos(sensor_yaw)
        sin_yaw = math.sin(sensor_yaw)
        half_fov = self._horizontal_fov_rad / 2.0
        max_range_sq = self._range_max * self._range_max
        local: list[CloudPoint] = []
        world: list[CloudPoint] = []

        for x, y, z, intensity in self._map_points:
            dx = x - sensor_x
            dy = y - sensor_y
            range_sq = dx * dx + dy * dy
            if range_sq > max_range_sq:
                continue
            local_x = cos_yaw * dx + sin_yaw * dy
            local_y = -sin_yaw * dx + cos_yaw * dy
            if self._horizontal_fov_rad < (2.0 * math.pi - 1e-3):
                if abs(math.atan2(local_y, local_x)) > half_fov:
                    continue
            local.append((local_x, local_y, z - self._sensor_z, intensity))
            world.append((x, y, z, intensity))

        if len(local) > self._max_live_points:
            stride = max(len(local) // self._max_live_points, 1)
            local = local[::stride]
            world = world[::stride]
        return local, world

    def _publish_map(self) -> None:
        if self._map_publisher is None:
            return
        self._map_publisher.publish(
            self._make_cloud(self._world_frame, self._map_points)
        )

    def _publish_live(self) -> None:
        local_points, world_points = self._visible_points()
        self._lidar_publisher.publish(
            self._make_cloud(self._lidar_frame, local_points)
        )
        if self._registered_publisher is not None:
            self._registered_publisher.publish(
                self._make_cloud(self._world_frame, world_points)
            )


def main() -> None:
    rclpy.init()
    node = PointCloudEmulator()
    try:
        rclpy.spin(node)
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
