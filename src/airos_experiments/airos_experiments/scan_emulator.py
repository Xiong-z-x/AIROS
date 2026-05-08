from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class RectObstacle:
    cx: float
    cy: float
    hx: float
    hy: float
    height: float = 1.0


@dataclass(frozen=True)
class CircleObstacle:
    cx: float
    cy: float
    radius: float
    height: float = 1.0


Obstacle = RectObstacle | CircleObstacle


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class OdomAnchor:
    map_pose: Pose2D
    odom_pose: Pose2D


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _pose_from_odom(msg: Odometry) -> Pose2D:
    pose = msg.pose.pose
    return Pose2D(
        float(pose.position.x),
        float(pose.position.y),
        _yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
    )


def _pose_from_initial_pose(msg: PoseWithCovarianceStamped) -> Pose2D:
    pose = msg.pose.pose
    return Pose2D(
        float(pose.position.x),
        float(pose.position.y),
        _yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
    )


def _map_pose_from_anchor(current: Pose2D, anchor: OdomAnchor) -> Pose2D:
    yaw_offset = anchor.map_pose.yaw - anchor.odom_pose.yaw
    dx = current.x - anchor.odom_pose.x
    dy = current.y - anchor.odom_pose.y
    cos_yaw = math.cos(yaw_offset)
    sin_yaw = math.sin(yaw_offset)
    return Pose2D(
        anchor.map_pose.x + cos_yaw * dx - sin_yaw * dy,
        anchor.map_pose.y + sin_yaw * dx + cos_yaw * dy,
        _normalize_angle(current.yaw + yaw_offset),
    )


def _parse_xyz(text: Optional[str]) -> tuple[float, float, float]:
    if not text:
        return 0.0, 0.0, 0.0
    parts = [float(part) for part in text.split()]
    while len(parts) < 3:
        parts.append(0.0)
    return parts[0], parts[1], parts[2]


def _ray_rect_intersection(
    origin_x: float,
    origin_y: float,
    dir_x: float,
    dir_y: float,
    rect: RectObstacle,
) -> Optional[float]:
    min_x = rect.cx - rect.hx
    max_x = rect.cx + rect.hx
    min_y = rect.cy - rect.hy
    max_y = rect.cy + rect.hy

    t_min = -math.inf
    t_max = math.inf

    if abs(dir_x) < 1e-12:
        if origin_x < min_x or origin_x > max_x:
            return None
    else:
        t1 = (min_x - origin_x) / dir_x
        t2 = (max_x - origin_x) / dir_x
        t_min = max(t_min, min(t1, t2))
        t_max = min(t_max, max(t1, t2))

    if abs(dir_y) < 1e-12:
        if origin_y < min_y or origin_y > max_y:
            return None
    else:
        t1 = (min_y - origin_y) / dir_y
        t2 = (max_y - origin_y) / dir_y
        t_min = max(t_min, min(t1, t2))
        t_max = min(t_max, max(t1, t2))

    if t_max < max(t_min, 0.0):
        return None

    hit = t_min if t_min >= 0.0 else t_max
    return hit if hit >= 0.0 else None


def _ray_circle_intersection(
    origin_x: float,
    origin_y: float,
    dir_x: float,
    dir_y: float,
    circle: CircleObstacle,
) -> Optional[float]:
    rel_x = origin_x - circle.cx
    rel_y = origin_y - circle.cy

    a = dir_x * dir_x + dir_y * dir_y
    b = 2.0 * (rel_x * dir_x + rel_y * dir_y)
    c = rel_x * rel_x + rel_y * rel_y - circle.radius * circle.radius
    discriminant = b * b - 4.0 * a * c
    if discriminant < 0.0:
        return None

    root = math.sqrt(discriminant)
    t1 = (-b - root) / (2.0 * a)
    t2 = (-b + root) / (2.0 * a)
    candidates = [t for t in (t1, t2) if t >= 0.0]
    if not candidates:
        return None
    return min(candidates)


def _load_obstacles(world_file: Path) -> list[Obstacle]:
    try:
        root = ET.parse(world_file).getroot()
    except (ET.ParseError, FileNotFoundError):
        return []

    world = root.find('world')
    if world is None:
        return []

    obstacles: list[Obstacle] = []
    ignored_models = {'go2w_nav_eq', 'floor'}

    for model in world.findall('model'):
        name = model.get('name', '')
        if name in ignored_models:
            continue

        pose_text = model.findtext('pose')
        pos_x, pos_y, _ = _parse_xyz(pose_text)
        link = model.find('link')
        if link is None:
            continue

        collision = link.find('collision')
        if collision is None:
            continue
        geometry = collision.find('geometry')
        if geometry is None:
            continue

        box = geometry.find('box')
        if box is not None:
            size_x, size_y, size_z = _parse_xyz(box.findtext('size'))
            if size_z < 0.2:
                continue
            obstacles.append(
                RectObstacle(
                    pos_x,
                    pos_y,
                    size_x / 2.0,
                    size_y / 2.0,
                    size_z,
                )
            )
            continue

        cylinder = geometry.find('cylinder')
        if cylinder is not None:
            radius_text = cylinder.findtext('radius')
            length_text = cylinder.findtext('length')
            if radius_text is None or length_text is None:
                continue
            length = float(length_text)
            if length < 0.2:
                continue
            obstacles.append(
                CircleObstacle(pos_x, pos_y, float(radius_text), length)
            )

    return obstacles


class ScanEmulator(Node):
    def __init__(self) -> None:
        super().__init__('scan_emulator')

        self.declare_parameter('world_file', '')
        self.declare_parameter('scan_frame', 'lidar_link')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('initial_pose_topic', '/initialpose')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('use_initial_pose_anchor', True)
        self.declare_parameter('sensor_offset_x', 0.25)
        self.declare_parameter('sensor_offset_y', 0.0)
        self.declare_parameter('sensor_yaw_offset', 0.0)
        self.declare_parameter('angle_min', -1.396263)
        self.declare_parameter('angle_max', 1.396263)
        self.declare_parameter('range_min', 0.08)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('sample_count', 640)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('dynamic_obstacles_enabled', False)
        self.declare_parameter('dynamic_obstacle_seed', 0)
        self.declare_parameter(
            'dynamic_marker_topic',
            '/dynamic_obstacles/markers',
        )

        world_file = Path(self.get_parameter('world_file').value)
        self._obstacles = (
            _load_obstacles(world_file)
            if world_file.as_posix()
            else []
        )
        if not self._obstacles:
            self._obstacles = [
                RectObstacle(-3.0, 0.0, 0.05, 4.0),
                RectObstacle(3.0, 0.0, 0.05, 4.0),
                RectObstacle(0.0, 3.5, 3.0, 0.05),
                RectObstacle(0.0, -3.5, 3.0, 0.05),
                RectObstacle(-1.7, 1.15, 1.3, 0.06),
                RectObstacle(1.15, 0.0, 0.09, 0.575),
                RectObstacle(-0.55, 0.9, 0.225, 0.175),
                RectObstacle(0.9, -1.1, 0.175, 0.275),
                CircleObstacle(1.6, 1.25, 0.25),
            ]

        self._sensor_offset_x = float(
            self.get_parameter('sensor_offset_x').value
        )
        self._sensor_offset_y = float(
            self.get_parameter('sensor_offset_y').value
        )
        self._sensor_yaw_offset = float(
            self.get_parameter('sensor_yaw_offset').value
        )
        self._scan_frame = str(self.get_parameter('scan_frame').value)
        self._world_frame = str(self.get_parameter('world_frame').value)
        self._use_initial_pose_anchor = bool(
            self.get_parameter('use_initial_pose_anchor').value
        )
        self._angle_min = float(self.get_parameter('angle_min').value)
        self._angle_max = float(self.get_parameter('angle_max').value)
        self._range_min = float(self.get_parameter('range_min').value)
        self._range_max = float(self.get_parameter('range_max').value)
        self._sample_count = int(self.get_parameter('sample_count').value)
        self._publish_rate_hz = float(
            self.get_parameter('publish_rate_hz').value
        )
        self._time_step = (
            1.0 / self._publish_rate_hz
            if self._publish_rate_hz > 0.0
            else 0.1
        )
        self._dynamic_obstacles_enabled = bool(
            self.get_parameter('dynamic_obstacles_enabled').value
        )
        self._dynamic_obstacle_seed = int(
            self.get_parameter('dynamic_obstacle_seed').value
        )

        odom_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        scan_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._odom_subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_callback,
            odom_qos,
        )
        self._initial_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter('initial_pose_topic').value),
            self._initial_pose_callback,
            10,
        )
        self._scan_publisher = self.create_publisher(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            scan_qos,
        )
        self._marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('dynamic_marker_topic').value),
            1,
        )

        self._odom_msg: Optional[Odometry] = None
        self._pending_initial_pose: Optional[Pose2D] = None
        self._odom_anchor: Optional[OdomAnchor] = None
        self._timer = self.create_timer(self._time_step, self._publish_scan)
        self.get_logger().info(
            f'scan emulator ready: obstacles={len(self._obstacles)} '
            f'dynamic={self._dynamic_obstacles_enabled} '
            f'world_file={world_file}'
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
        sensor_x = base_pose.x + offset_x
        sensor_y = base_pose.y + offset_y
        sensor_yaw = yaw + self._sensor_yaw_offset
        return sensor_x, sensor_y, sensor_yaw

    def _dynamic_obstacles(self) -> list[CircleObstacle]:
        if not self._dynamic_obstacles_enabled:
            return []

        stamp = self.get_clock().now().nanoseconds / 1_000_000_000.0
        phase = (stamp + 0.73 * self._dynamic_obstacle_seed) % 24.0

        pedestrian_y = -2.2 + 4.4 * ((phase % 12.0) / 12.0)
        cart_x = -2.3 + 4.6 * (((phase + 5.0) % 18.0) / 18.0)
        obstacles = [
            CircleObstacle(-0.2, pedestrian_y, 0.22),
            CircleObstacle(cart_x, -1.65, 0.32),
        ]
        if 8.0 <= phase <= 15.0:
            obstacles.append(CircleObstacle(1.35, 0.45, 0.28))
        return obstacles

    def _publish_dynamic_markers(
        self,
        obstacles: list[CircleObstacle],
    ) -> None:
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for index, obstacle in enumerate(obstacles):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self._world_frame
            marker.ns = 'airos_dynamic_obstacles'
            marker.id = index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = obstacle.cx
            marker.pose.position.y = obstacle.cy
            marker.pose.position.z = 0.35
            marker.pose.orientation.w = 1.0
            diameter = obstacle.radius * 2.0
            marker.scale.x = diameter
            marker.scale.y = diameter
            marker.scale.z = 0.7
            marker.color.r = 0.95
            marker.color.g = 0.55
            marker.color.b = 0.08
            marker.color.a = 0.85
            marker.lifetime.sec = 1
            markers.markers.append(marker)
        self._marker_publisher.publish(markers)

    def _cast_ray(
        self,
        origin_x: float,
        origin_y: float,
        angle: float,
        obstacles: list[Obstacle],
    ) -> float:
        dir_x = math.cos(angle)
        dir_y = math.sin(angle)
        best = self._range_max
        for obstacle in obstacles:
            if isinstance(obstacle, RectObstacle):
                hit = _ray_rect_intersection(
                    origin_x,
                    origin_y,
                    dir_x,
                    dir_y,
                    obstacle,
                )
            else:
                hit = _ray_circle_intersection(
                    origin_x,
                    origin_y,
                    dir_x,
                    dir_y,
                    obstacle,
                )
            if hit is None:
                continue
            if self._range_min <= hit <= best:
                best = hit
        return max(self._range_min, min(best, self._range_max))

    def _publish_scan(self) -> None:
        sensor_x, sensor_y, sensor_yaw = self._current_sensor_pose()
        dynamic_obstacles = self._dynamic_obstacles()
        obstacles = self._obstacles + dynamic_obstacles
        angle_increment = (
            (self._angle_max - self._angle_min)
            / max(self._sample_count - 1, 1)
        )

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._scan_frame
        msg.angle_min = self._angle_min
        msg.angle_max = self._angle_max
        msg.angle_increment = angle_increment
        msg.time_increment = 0.0
        msg.scan_time = self._time_step
        msg.range_min = self._range_min
        msg.range_max = self._range_max
        msg.ranges = [
            self._cast_ray(
                sensor_x,
                sensor_y,
                sensor_yaw + self._angle_min + i * angle_increment,
                obstacles,
            )
            for i in range(self._sample_count)
        ]
        msg.intensities = []
        self._scan_publisher.publish(msg)
        self._publish_dynamic_markers(dynamic_obstacles)


def main() -> None:
    rclpy.init()
    node = ScanEmulator()
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
