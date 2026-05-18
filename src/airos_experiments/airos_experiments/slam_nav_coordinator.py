from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass(frozen=True)
class MapBounds:
    min_x: float
    min_y: float
    max_x: float
    max_y: float

    def contains(self, x: float, y: float, margin_m: float) -> bool:
        return (
            self.min_x + margin_m <= x <= self.max_x - margin_m
            and self.min_y + margin_m <= y <= self.max_y - margin_m
        )


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _occupancy_grid_bounds(grid: OccupancyGrid) -> MapBounds:
    origin = grid.info.origin
    yaw = _yaw_from_quaternion(
        origin.orientation.x,
        origin.orientation.y,
        origin.orientation.z,
        origin.orientation.w,
    )
    width_m = float(grid.info.width) * float(grid.info.resolution)
    height_m = float(grid.info.height) * float(grid.info.resolution)

    corners = (
        (0.0, 0.0),
        (width_m, 0.0),
        (0.0, height_m),
        (width_m, height_m),
    )
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    xs = []
    ys = []
    for x, y in corners:
        xs.append(origin.position.x + x * cos_yaw - y * sin_yaw)
        ys.append(origin.position.y + x * sin_yaw + y * cos_yaw)
    return MapBounds(min(xs), min(ys), max(xs), max(ys))


def _grid_is_usable(grid: OccupancyGrid, min_cells: int) -> bool:
    if grid.info.width <= 0 or grid.info.height <= 0:
        return False
    if grid.info.resolution <= 0.0:
        return False
    if int(grid.info.width) * int(grid.info.height) < min_cells:
        return False
    return True


class SlamNavCoordinator(Node):
    def __init__(self) -> None:
        super().__init__('slam_nav_coordinator')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter(
            'navigation_manager_service',
            '/lifecycle_manager_navigation/manage_nodes',
        )
        self.declare_parameter(
            'collision_manager_service',
            '/lifecycle_manager_collision_monitor/manage_nodes',
        )
        self.declare_parameter('startup_timeout_sec', 45.0)
        self.declare_parameter('poll_period_sec', 0.5)
        self.declare_parameter('map_edge_margin_m', 0.35)
        self.declare_parameter('min_map_cells', 100)

        self._map_topic = str(self.get_parameter('map_topic').value)
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._robot_frame = str(self.get_parameter('robot_frame').value)
        nav_service = str(self.get_parameter('navigation_manager_service').value)
        collision_service = str(self.get_parameter('collision_manager_service').value)
        self._startup_timeout_sec = float(
            self.get_parameter('startup_timeout_sec').value
        )
        self._poll_period_sec = float(self.get_parameter('poll_period_sec').value)
        self._map_edge_margin_m = float(
            self.get_parameter('map_edge_margin_m').value
        )
        self._min_map_cells = int(self.get_parameter('min_map_cells').value)

        self._latest_map: OccupancyGrid | None = None
        self._started = False
        self._pending_startups = []
        self._deadline = self.get_clock().now() + Duration(
            seconds=self._startup_timeout_sec
        )

        self._tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._nav_client = self.create_client(ManageLifecycleNodes, nav_service)
        self._collision_client = self.create_client(
            ManageLifecycleNodes,
            collision_service,
        )
        self._map_sub = self.create_subscription(
            OccupancyGrid,
            self._map_topic,
            self._on_map,
            10,
        )
        self._timer = self.create_timer(self._poll_period_sec, self._tick)

        self.get_logger().info(
            'waiting for online SLAM map before Nav2 startup: '
            f'map_topic={self._map_topic}, robot_frame={self._robot_frame}'
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg

    def _tick(self) -> None:
        if self._started:
            return

        now = self.get_clock().now()
        if now > self._deadline:
            self.get_logger().error(
                'timed out waiting for usable SLAM map and TF; Nav2 was not started'
            )
            self._started = True
            return

        if self._latest_map is None:
            return
        if not _grid_is_usable(self._latest_map, self._min_map_cells):
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException:
            return

        x = float(transform.transform.translation.x)
        y = float(transform.transform.translation.y)
        bounds = _occupancy_grid_bounds(self._latest_map)
        if not bounds.contains(x, y, self._map_edge_margin_m):
            self.get_logger().warn(
                'SLAM map received, but robot is outside usable bounds: '
                f'robot=({x:.2f}, {y:.2f}), '
                f'bounds=({bounds.min_x:.2f}, {bounds.min_y:.2f})-'
                f'({bounds.max_x:.2f}, {bounds.max_y:.2f})'
            )
            return

        self.get_logger().info(
            'SLAM map covers robot; starting Nav2 lifecycle managers: '
            f'robot=({x:.2f}, {y:.2f}), map_cells='
            f'{self._latest_map.info.width}x{self._latest_map.info.height}'
        )
        self._start_lifecycle_manager(self._nav_client, 'navigation')
        self._start_lifecycle_manager(self._collision_client, 'collision_monitor')
        self._started = True

    def _start_lifecycle_manager(self, client, label: str) -> None:
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f'{label} lifecycle manager service unavailable')
            return
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        future = client.call_async(request)
        self._pending_startups.append(future)
        future.add_done_callback(
            lambda completed_future: self._on_startup_complete(
                completed_future,
                label,
            )
        )

    def _on_startup_complete(self, future, label: str) -> None:
        if future in self._pending_startups:
            self._pending_startups.remove(future)
        result = future.result()
        if result is None:
            self.get_logger().warn(f'{label} lifecycle startup request failed')
            return
        if result.success:
            self.get_logger().info(f'{label} lifecycle startup accepted')
        else:
            self.get_logger().warn(f'{label} lifecycle startup rejected')


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SlamNavCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
