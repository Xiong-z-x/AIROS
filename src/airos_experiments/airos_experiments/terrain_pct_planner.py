from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path as PathMsg
from nav2_msgs.action import FollowPath, NavigateThroughPoses
from nav2_msgs.msg import SpeedLimit
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
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
    OdomAnchor,
    Pose2D,
    _map_pose_from_anchor,
    _pose_from_initial_pose,
    _pose_from_odom,
)
from airos_experiments.sdf_geometry import (
    BoxCollision,
    CloudPoint,
    CollisionGeometry,
    inverse_transform_point,
    iter_obstacle_geometries,
    iter_traversable_boxes,
    load_collision_geometries,
    sample_box_top,
)

_ACTIVE_GOAL_STATUSES = {
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
}


def _goal_is_active(status: int) -> bool:
    return status in _ACTIVE_GOAL_STATUSES


@dataclass(frozen=True)
class TerrainNode:
    index: int
    x: float
    y: float
    z: float
    surface_label: str
    edge_margin: float
    surface_local_x: float = 0.0
    surface_local_y: float = 0.0
    surface_half_x: float = 0.0
    surface_half_y: float = 0.0
    surface_width_axis: str = 'y'


@dataclass(frozen=True)
class TerrainGraph:
    nodes: list[TerrainNode]
    adjacency: list[list[tuple[int, float]]]
    terrain_cloud: list[CloudPoint]


def build_terrain_graph(
    world_file: Path,
    grid_resolution: float = 0.40,
    terrain_cloud_resolution: Optional[float] = None,
    robot_radius: float = 0.35,
    support_margin: float = 0.45,
    max_slope_grade: float = 0.55,
    max_step_height: float = 0.34,
    max_surface_transition_height: float = 0.12,
) -> TerrainGraph:
    geometries = load_collision_geometries(world_file)
    traversable_boxes = list(iter_traversable_boxes(geometries))
    obstacles = list(iter_obstacle_geometries(geometries))
    nodes: list[TerrainNode] = []
    terrain_cloud: list[CloudPoint] = []
    cloud_resolution = (
        terrain_cloud_resolution
        if terrain_cloud_resolution and terrain_cloud_resolution > 0.0
        else grid_resolution
    )

    for box in traversable_boxes:
        label = box.label
        margin = _surface_support_margin(label, support_margin)
        terrain_cloud.extend(sample_box_top(box, cloud_resolution, margin=0.0))
        for x, y, z, _ in sample_box_top(box, grid_resolution, margin=margin):
            if _blocked_by_obstacle(
                (x, y, z),
                obstacles,
                clearance=robot_radius,
                current_surface=box,
            ):
                continue
            edge_margin = _surface_edge_margin(box, (x, y, z))
            local = inverse_transform_point(box.transform, (x, y, z))
            nodes.append(
                TerrainNode(
                    index=len(nodes),
                    x=x,
                    y=y,
                    z=z,
                    surface_label=label,
                    edge_margin=edge_margin,
                    surface_local_x=local[0],
                    surface_local_y=local[1],
                    surface_half_x=box.size[0] / 2.0,
                    surface_half_y=box.size[1] / 2.0,
                    surface_width_axis=_box_width_axis(box),
                )
            )

    adjacency = _build_adjacency(
        nodes,
        grid_resolution=grid_resolution,
        max_slope_grade=max_slope_grade,
        max_step_height=max_step_height,
        max_surface_transition_height=max_surface_transition_height,
    )
    return TerrainGraph(nodes=nodes, adjacency=adjacency, terrain_cloud=terrain_cloud)


def plan_terrain_path(
    graph: TerrainGraph,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    start_z: float = 0.0,
    goal_z_policy: str = 'highest',
) -> list[TerrainNode]:
    if not graph.nodes:
        return []
    start_index = _nearest_node(
        graph.nodes,
        start_xy,
        z_reference=start_z,
        policy='nearest_z',
    )
    goal_index = _nearest_node(
        graph.nodes,
        goal_xy,
        z_reference=start_z,
        policy=goal_z_policy,
    )
    if start_index is None or goal_index is None:
        return []
    if start_index == goal_index:
        return [graph.nodes[start_index]]

    distances = [math.inf] * len(graph.nodes)
    parents: list[Optional[int]] = [None] * len(graph.nodes)
    distances[start_index] = 0.0
    queue: list[tuple[float, int]] = [
        (_heuristic(graph.nodes[start_index], graph.nodes[goal_index]), start_index)
    ]

    while queue:
        _, current = heapq.heappop(queue)
        if current == goal_index:
            break
        current_distance = distances[current]
        if not math.isfinite(current_distance):
            continue
        for neighbor, edge_cost in graph.adjacency[current]:
            next_distance = current_distance + edge_cost
            if next_distance >= distances[neighbor]:
                continue
            distances[neighbor] = next_distance
            parents[neighbor] = current
            priority = next_distance + _heuristic(
                graph.nodes[neighbor],
                graph.nodes[goal_index],
            )
            heapq.heappush(queue, (priority, neighbor))

    if parents[goal_index] is None:
        return []

    path: list[TerrainNode] = []
    cursor: Optional[int] = goal_index
    while cursor is not None:
        path.append(graph.nodes[cursor])
        cursor = parents[cursor]
    path.reverse()
    return path


class TerrainPctPlanner(Node):
    def __init__(self) -> None:
        super().__init__('terrain_pct_planner')
        self.declare_parameter('world_file', '')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('initial_pose_topic', '/initialpose')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('path_topic', '/pct_path')
        self.declare_parameter(
            'terrain_cloud_topic',
            '/terrain_traversability_cloud',
        )
        self.declare_parameter('use_initial_pose_anchor', True)
        self.declare_parameter('grid_resolution', 0.40)
        self.declare_parameter('terrain_cloud_resolution', 0.0)
        self.declare_parameter('robot_radius', 0.35)
        self.declare_parameter('support_margin', 0.45)
        self.declare_parameter('max_slope_grade', 0.55)
        self.declare_parameter('max_step_height', 0.34)
        self.declare_parameter('max_surface_transition_height', 0.12)
        self.declare_parameter('goal_z_policy', 'highest')
        self.declare_parameter('send_nav2_goals', True)
        self.declare_parameter('nav_execution_mode', 'direct')
        self.declare_parameter('waypoint_spacing', 0.90)
        self.declare_parameter('start_waypoint_clearance', 0.45)
        self.declare_parameter('follow_path_start_clearance', 0.12)
        self.declare_parameter('slope_speed_limit', 0.14)
        self.declare_parameter('flat_speed_limit', 0.22)
        self.declare_parameter('slope_speed_grade_threshold', 0.08)
        self.declare_parameter('direct_cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('direct_control_frequency', 15.0)
        self.declare_parameter('direct_lookahead_dist', 0.45)
        self.declare_parameter('direct_waypoint_tolerance', 0.24)
        self.declare_parameter('direct_goal_tolerance', 0.30)
        self.declare_parameter('direct_heading_gain', 1.4)
        self.declare_parameter('direct_max_linear_speed', 0.20)
        self.declare_parameter('direct_min_linear_speed', 0.035)
        self.declare_parameter('direct_max_angular_speed', 0.45)
        self.declare_parameter('direct_max_heading_error_for_forward', 1.25)
        self.declare_parameter('initial_surface_z_hint', -1.0)
        self.declare_parameter('initial_surface_hint_radius', 0.75)
        self.declare_parameter('last_path_surface_hint_radius', 0.75)
        self.declare_parameter('terrain_publish_period_sec', 4.0)
        self.declare_parameter('duplicate_goal_xy_tolerance', 0.05)
        self.declare_parameter('duplicate_goal_time_sec', 1.5)

        world_file = Path(str(self.get_parameter('world_file').value))
        self._world_frame = str(self.get_parameter('world_frame').value)
        self._goal_z_policy = str(self.get_parameter('goal_z_policy').value)
        self._send_nav2_goals = bool(
            self.get_parameter('send_nav2_goals').value
        )
        self._nav_execution_mode = str(
            self.get_parameter('nav_execution_mode').value
        )
        if self._nav_execution_mode not in {'direct', 'follow_path', 'waypoints'}:
            raise ValueError(
                "nav_execution_mode must be 'direct', 'follow_path' or 'waypoints', "
                f"got {self._nav_execution_mode!r}"
            )
        self._waypoint_spacing = float(
            self.get_parameter('waypoint_spacing').value
        )
        self._start_waypoint_clearance = float(
            self.get_parameter('start_waypoint_clearance').value
        )
        self._follow_path_start_clearance = float(
            self.get_parameter('follow_path_start_clearance').value
        )
        self._slope_speed_limit = float(
            self.get_parameter('slope_speed_limit').value
        )
        self._flat_speed_limit = float(
            self.get_parameter('flat_speed_limit').value
        )
        self._slope_speed_grade_threshold = float(
            self.get_parameter('slope_speed_grade_threshold').value
        )
        self._direct_lookahead_dist = float(
            self.get_parameter('direct_lookahead_dist').value
        )
        self._direct_waypoint_tolerance = float(
            self.get_parameter('direct_waypoint_tolerance').value
        )
        self._direct_goal_tolerance = float(
            self.get_parameter('direct_goal_tolerance').value
        )
        self._direct_heading_gain = float(
            self.get_parameter('direct_heading_gain').value
        )
        self._direct_max_linear_speed = float(
            self.get_parameter('direct_max_linear_speed').value
        )
        self._direct_min_linear_speed = float(
            self.get_parameter('direct_min_linear_speed').value
        )
        self._direct_max_angular_speed = float(
            self.get_parameter('direct_max_angular_speed').value
        )
        self._direct_max_heading_error_for_forward = float(
            self.get_parameter('direct_max_heading_error_for_forward').value
        )
        self._initial_surface_z_hint = float(
            self.get_parameter('initial_surface_z_hint').value
        )
        self._initial_surface_hint_radius = float(
            self.get_parameter('initial_surface_hint_radius').value
        )
        self._last_path_surface_hint_radius = float(
            self.get_parameter('last_path_surface_hint_radius').value
        )
        self._duplicate_goal_xy_tolerance = float(
            self.get_parameter('duplicate_goal_xy_tolerance').value
        )
        self._duplicate_goal_time_sec = float(
            self.get_parameter('duplicate_goal_time_sec').value
        )
        self._use_initial_pose_anchor = bool(
            self.get_parameter('use_initial_pose_anchor').value
        )
        self._graph = build_terrain_graph(
            world_file,
            grid_resolution=float(self.get_parameter('grid_resolution').value),
            terrain_cloud_resolution=float(
                self.get_parameter('terrain_cloud_resolution').value
            ),
            robot_radius=float(self.get_parameter('robot_radius').value),
            support_margin=float(self.get_parameter('support_margin').value),
            max_slope_grade=float(self.get_parameter('max_slope_grade').value),
            max_step_height=float(self.get_parameter('max_step_height').value),
            max_surface_transition_height=float(
                self.get_parameter('max_surface_transition_height').value
            ),
        )

        live_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
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
        self._goal_subscription = self.create_subscription(
            PoseStamped,
            str(self.get_parameter('goal_topic').value),
            self._goal_callback,
            live_qos,
        )
        self._path_publisher = self.create_publisher(
            PathMsg,
            str(self.get_parameter('path_topic').value),
            live_qos,
        )
        self._terrain_cloud_publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('terrain_cloud_topic').value),
            latched_qos,
        )
        self._waypoint_action_client = ActionClient(
            self,
            NavigateThroughPoses,
            '/navigate_through_poses',
        )
        self._follow_path_action_client = ActionClient(
            self,
            FollowPath,
            '/follow_path',
        )
        self._speed_limit_publisher = self.create_publisher(
            SpeedLimit,
            '/speed_limit',
            1,
        )
        self._direct_cmd_vel_publisher = self.create_publisher(
            Twist,
            str(self.get_parameter('direct_cmd_vel_topic').value),
            10,
        )

        self._odom_msg: Optional[Odometry] = None
        self._pending_initial_pose: Optional[Pose2D] = None
        self._odom_anchor: Optional[OdomAnchor] = None
        self._active_nav_goal: Optional[ClientGoalHandle] = None
        self._pending_nav_goal: Optional[
            tuple[FollowPath.Goal | NavigateThroughPoses.Goal, int, int, str]
        ] = None
        self._initial_planner_xy: Optional[tuple[float, float]] = None
        self._last_goal_xy: Optional[tuple[float, float]] = None
        self._last_goal_time_ns: Optional[int] = None
        self._last_planned_path: list[TerrainNode] = []
        self._direct_path: list[TerrainNode] = []
        self._direct_target_index = 0
        self._direct_speed_limit = 0.0
        self._terrain_timer = self.create_timer(
            max(float(self.get_parameter('terrain_publish_period_sec').value), 0.5),
            self._publish_terrain_cloud,
        )
        self._direct_timer = self.create_timer(
            1.0
            / max(
                float(self.get_parameter('direct_control_frequency').value),
                1.0,
            ),
            self._direct_control_tick,
        )
        self.get_logger().info(
            'terrain pct-style planner ready: '
            f'nodes={len(self._graph.nodes)} '
            f'edges={sum(len(edges) for edges in self._graph.adjacency)} '
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

    def _current_pose(self) -> tuple[float, float, float]:
        x, y, _, z = self._current_planar_pose()
        return x, y, z

    def _current_planar_pose(self) -> tuple[float, float, float, float]:
        if self._odom_msg is None:
            return 0.0, 0.0, 0.0, 0.0
        pose = _pose_from_odom(self._odom_msg)
        if self._odom_anchor is not None:
            pose = _map_pose_from_anchor(pose, self._odom_anchor)
        z = float(self._odom_msg.pose.pose.position.z)
        return pose.x, pose.y, pose.yaw, z

    def _goal_callback(self, msg: PoseStamped) -> None:
        start_x, start_y, start_z = self._current_pose()
        if self._initial_planner_xy is None:
            self._initial_planner_xy = (start_x, start_y)
        terrain_start_z = _surface_z_reference(
            odom_z=start_z,
            current_xy=(start_x, start_y),
            initial_xy=self._initial_planner_xy,
            initial_surface_z_hint=self._initial_surface_z_hint,
            initial_surface_hint_radius=self._initial_surface_hint_radius,
            last_path=self._last_planned_path,
            last_path_surface_hint_radius=self._last_path_surface_hint_radius,
        )
        terrain_start_z = (
            _surface_height_at_xy(
                self._graph.nodes,
                (start_x, start_y),
                z_hint=terrain_start_z,
                max_xy_distance=self._initial_surface_hint_radius,
            )
            or terrain_start_z
        )
        goal_x = float(msg.pose.position.x)
        goal_y = float(msg.pose.position.y)
        if self._is_duplicate_goal((goal_x, goal_y)):
            return
        path = plan_terrain_path(
            self._graph,
            (start_x, start_y),
            (goal_x, goal_y),
            start_z=terrain_start_z,
            goal_z_policy=self._goal_z_policy,
        )
        if not path:
            self.get_logger().warning(
                'terrain planner failed to find a traversable route: '
                f'start=({start_x:.2f},{start_y:.2f}) '
                f'goal=({goal_x:.2f},{goal_y:.2f})'
            )
            return
        self._last_planned_path = path
        self._publish_path(path)
        if self._send_nav2_goals:
            if self._nav_execution_mode == 'direct':
                self._start_direct_tracking(path)
            elif self._nav_execution_mode == 'follow_path':
                self._send_follow_path_goal(path)
            else:
                self._send_waypoint_goal(path, msg)

    def _is_duplicate_goal(self, goal_xy: tuple[float, float]) -> bool:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_goal_xy is None or self._last_goal_time_ns is None:
            self._last_goal_xy = goal_xy
            self._last_goal_time_ns = now_ns
            return False
        distance = math.hypot(
            goal_xy[0] - self._last_goal_xy[0],
            goal_xy[1] - self._last_goal_xy[1],
        )
        elapsed_sec = (now_ns - self._last_goal_time_ns) / 1_000_000_000.0
        duplicate = (
            distance <= max(0.0, self._duplicate_goal_xy_tolerance)
            and elapsed_sec <= max(0.0, self._duplicate_goal_time_sec)
        )
        if not duplicate:
            self._last_goal_xy = goal_xy
            self._last_goal_time_ns = now_ns
        return duplicate

    def _publish_path(self, path: list[TerrainNode]) -> None:
        msg = PathMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._world_frame
        msg.poses = [
            self._pose_stamped_for_node(node, path, index, msg.header.stamp)
            for index, node in enumerate(path)
        ]
        self._path_publisher.publish(msg)

    def _send_waypoint_goal(
        self,
        path: list[TerrainNode],
        original_goal: PoseStamped,
    ) -> None:
        if not self._waypoint_action_client.server_is_ready():
            if not self._waypoint_action_client.wait_for_server(timeout_sec=0.1):
                self.get_logger().warning(
                    'navigate_through_poses server is not ready; '
                    'published /pct_path only'
                )
                return
        goal_msg = NavigateThroughPoses.Goal()
        reduced_path = _waypoint_path(path, self._waypoint_spacing)
        start_x, start_y, _ = self._current_pose()
        nav_path = _waypoints_after_start_clearance(
            reduced_path,
            (start_x, start_y),
            self._start_waypoint_clearance,
        )
        stamp = self.get_clock().now().to_msg()
        poses = [
            self._pose_stamped_for_node(node, nav_path, index, stamp)
            for index, node in enumerate(nav_path)
        ]
        if poses:
            poses[-1].pose.orientation = original_goal.pose.orientation
        goal_msg.poses = poses
        self._dispatch_or_cancel_active(
            goal_msg,
            len(poses),
            len(path),
            mode='waypoints',
        )

    def _send_follow_path_goal(self, path: list[TerrainNode]) -> None:
        if not self._follow_path_action_client.server_is_ready():
            if not self._follow_path_action_client.wait_for_server(timeout_sec=0.1):
                self.get_logger().warning(
                    'follow_path server is not ready; published /pct_path only'
                )
                return
        filtered_path = _waypoints_after_start_clearance(
            path,
            self._current_pose()[:2],
            self._follow_path_start_clearance,
        )
        path_msg = PathMsg()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self._world_frame
        path_msg.poses = [
            self._pose_stamped_for_node(node, filtered_path, index, path_msg.header.stamp)
            for index, node in enumerate(filtered_path)
        ]
        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg
        goal_msg.controller_id = 'FollowPath'
        goal_msg.goal_checker_id = 'general_goal_checker'
        self._publish_speed_limit(path)
        self._dispatch_or_cancel_active(
            goal_msg,
            len(path_msg.poses),
            len(path),
            mode='follow_path',
        )

    def _start_direct_tracking(self, path: list[TerrainNode]) -> None:
        direct_path = _waypoints_after_start_clearance(
            path,
            self._current_pose()[:2],
            self._follow_path_start_clearance,
        )
        self._publish_speed_limit(path)
        self._direct_path = direct_path
        self._direct_target_index = 0
        self._direct_speed_limit = self._direct_surface_speed_limit()
        self.get_logger().info(
            'started terrain-guided direct tracking: '
            f'poses={len(direct_path)} path_nodes={len(path)}'
        )

    def _direct_control_tick(self) -> None:
        if not self._direct_path:
            return

        current_x, current_y, current_yaw, _ = self._current_planar_pose()
        goal = self._direct_path[-1]
        goal_distance = math.hypot(goal.x - current_x, goal.y - current_y)
        if goal_distance <= max(0.0, self._direct_goal_tolerance):
            self._direct_path = []
            self._direct_target_index = 0
            self._publish_direct_stop()
            self.get_logger().info('terrain direct tracking goal reached')
            return

        self._advance_direct_target(current_x, current_y)
        self._direct_speed_limit = self._direct_surface_speed_limit()
        self._publish_speed_limit_for_direct_target()
        target = self._direct_lookahead_target(current_x, current_y)
        dx = target.x - current_x
        dy = target.y - current_y
        target_distance = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)
        heading_error = _normalize_angle(desired_yaw - current_yaw)
        twist = Twist()
        twist.linear.x = _direct_linear_speed(
            speed_limit=self._direct_speed_limit,
            max_linear_speed=self._direct_max_linear_speed,
            min_linear_speed=self._direct_min_linear_speed,
            heading_error=heading_error,
            max_heading_error_for_forward=(
                self._direct_max_heading_error_for_forward
            ),
            target_distance=target_distance,
            slow_radius=max(self._direct_lookahead_dist, 0.05),
        )
        twist.angular.z = _clamp(
            self._direct_heading_gain * heading_error,
            -self._direct_max_angular_speed,
            self._direct_max_angular_speed,
        )
        self._direct_cmd_vel_publisher.publish(twist)

    def _advance_direct_target(self, current_x: float, current_y: float) -> None:
        while self._direct_target_index < len(self._direct_path) - 1:
            target = self._direct_path[self._direct_target_index]
            next_target = self._direct_path[self._direct_target_index + 1]
            target_distance = math.hypot(
                target.x - current_x,
                target.y - current_y,
            )
            next_distance = math.hypot(
                next_target.x - current_x,
                next_target.y - current_y,
            )
            if target_distance <= max(0.0, self._direct_waypoint_tolerance):
                self._direct_target_index += 1
                continue
            if next_distance + 0.05 < target_distance:
                self._direct_target_index += 1
                continue
            return

    def _direct_lookahead_target(
        self,
        current_x: float,
        current_y: float,
    ) -> TerrainNode:
        target = self._direct_path[self._direct_target_index]
        target_surface = target.surface_label
        for index in range(self._direct_target_index, len(self._direct_path)):
            candidate = self._direct_path[index]
            if candidate.surface_label != target_surface:
                if index > self._direct_target_index:
                    return target
                continue
            distance = math.hypot(candidate.x - current_x, candidate.y - current_y)
            if distance >= max(0.0, self._direct_lookahead_dist):
                return candidate
            target = candidate
        return target

    def _publish_direct_stop(self) -> None:
        self._direct_cmd_vel_publisher.publish(Twist())

    def _direct_surface_speed_limit(self) -> float:
        if not self._direct_path:
            return max(0.01, self._flat_speed_limit)
        surface_label = self._direct_path[self._direct_target_index].surface_label
        return _surface_speed_limit_for_label(
            surface_label,
            slope_speed_limit=self._slope_speed_limit,
            flat_speed_limit=self._flat_speed_limit,
        )

    def _publish_speed_limit_for_direct_target(self) -> None:
        if not self._direct_path:
            return
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._world_frame
        msg.percentage = False
        msg.speed_limit = self._direct_speed_limit
        self._speed_limit_publisher.publish(msg)

    def _publish_speed_limit(self, path: list[TerrainNode]) -> None:
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._world_frame
        msg.percentage = False
        msg.speed_limit = _path_speed_limit(
            path,
            slope_speed_limit=self._slope_speed_limit,
            flat_speed_limit=self._flat_speed_limit,
            slope_grade_threshold=self._slope_speed_grade_threshold,
        )
        self._speed_limit_publisher.publish(msg)

    def _dispatch_or_cancel_active(
        self,
        goal_msg: FollowPath.Goal | NavigateThroughPoses.Goal,
        waypoint_count: int,
        path_node_count: int,
        mode: str,
    ) -> None:
        if self._active_nav_goal is not None and _goal_is_active(
            self._active_nav_goal.status
        ):
            self._pending_nav_goal = (
                goal_msg,
                waypoint_count,
                path_node_count,
                mode,
            )
            cancel_future = self._active_nav_goal.cancel_goal_async()
            cancel_future.add_done_callback(self._on_previous_goal_cancelled)
            self.get_logger().info(
                'cancelled previous terrain-guided navigation goal before '
                'sending a new one'
            )
            return

        self._dispatch_nav_goal(goal_msg, waypoint_count, path_node_count, mode)

    def _dispatch_nav_goal(
        self,
        goal_msg: FollowPath.Goal | NavigateThroughPoses.Goal,
        waypoint_count: int,
        path_node_count: int,
        mode: str,
    ) -> None:
        if mode == 'follow_path':
            client = self._follow_path_action_client
        else:
            client = self._waypoint_action_client
        send_future = client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_goal_response)
        self.get_logger().info(
            f'sent terrain-guided {mode} goal: '
            f'poses={waypoint_count} path_nodes={path_node_count}'
        )

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning(
                'terrain-guided navigation goal was rejected'
            )
            return
        self._active_nav_goal = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        self._active_nav_goal = None
        if self._pending_nav_goal is not None:
            pending_goal, waypoint_count, path_node_count, mode = self._pending_nav_goal
            self._pending_nav_goal = None
            self._dispatch_nav_goal(
                pending_goal,
                waypoint_count,
                path_node_count,
                mode,
            )

    def _on_previous_goal_cancelled(self, future) -> None:
        self._active_nav_goal = None
        if self._pending_nav_goal is None:
            return
        pending_goal, waypoint_count, path_node_count, mode = self._pending_nav_goal
        self._pending_nav_goal = None
        self._dispatch_nav_goal(
            pending_goal,
            waypoint_count,
            path_node_count,
            mode,
        )

    def _pose_stamped_for_node(
        self,
        node: TerrainNode,
        path: list[TerrainNode],
        index: int,
        stamp,
    ) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self._world_frame
        pose.pose.position.x = node.x
        pose.pose.position.y = node.y
        pose.pose.position.z = node.z
        next_node = path[min(index + 1, len(path) - 1)]
        yaw = math.atan2(next_node.y - node.y, next_node.x - node.x)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _publish_terrain_cloud(self) -> None:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._world_frame
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name='intensity',
                offset=12,
                datatype=PointField.FLOAT32,
                count=1,
            ),
        ]
        self._terrain_cloud_publisher.publish(
            point_cloud2.create_cloud(header, fields, self._graph.terrain_cloud)
        )


def _build_adjacency(
    nodes: list[TerrainNode],
    grid_resolution: float,
    max_slope_grade: float,
    max_step_height: float,
    max_surface_transition_height: float,
) -> list[list[tuple[int, float]]]:
    adjacency: list[list[tuple[int, float]]] = [[] for _ in nodes]
    bins: dict[tuple[int, int], list[int]] = {}
    for node in nodes:
        key = _bin_key(node.x, node.y, grid_resolution)
        bins.setdefault(key, []).append(node.index)

    neighbor_radius = grid_resolution * 1.65
    for node in nodes:
        key_x, key_y = _bin_key(node.x, node.y, grid_resolution)
        candidates: set[int] = set()
        for bx in range(key_x - 2, key_x + 3):
            for by in range(key_y - 2, key_y + 3):
                candidates.update(bins.get((bx, by), []))
        for other_index in candidates:
            if other_index == node.index:
                continue
            other = nodes[other_index]
            horizontal = math.hypot(other.x - node.x, other.y - node.y)
            if horizontal < grid_resolution * 0.35:
                continue
            if horizontal > neighbor_radius:
                continue
            dz = abs(other.z - node.z)
            grade = dz / max(horizontal, 1e-6)
            step_height_limit = max_step_height
            if (
                'stair' in node.surface_label
                or 'step' in node.surface_label
                or 'stair' in other.surface_label
                or 'step' in other.surface_label
            ):
                step_height_limit = max(step_height_limit, 0.25)
            if dz > step_height_limit or grade > max_slope_grade:
                continue
            surface_changed = other.surface_label != node.surface_label
            if (
                surface_changed
                and dz > max(0.0, max_surface_transition_height)
            ):
                continue
            if surface_changed and not _valid_surface_transition(node, other):
                continue
            slope_cost = 1.0 + grade * 1.8
            edge_risk = _edge_risk(node) + _edge_risk(other)
            transition_cost = grid_resolution if surface_changed else 0.0
            cost = math.sqrt(horizontal * horizontal + dz * dz) * slope_cost
            adjacency[node.index].append(
                (other_index, cost + edge_risk + transition_cost)
            )
    return adjacency


def _valid_surface_transition(node: TerrainNode, other: TerrainNode) -> bool:
    if _is_ramp_label(node.surface_label) and not _is_ramp_label(
        other.surface_label
    ):
        return _is_ramp_entry_node(node)
    if _is_ramp_label(other.surface_label) and not _is_ramp_label(
        node.surface_label
    ):
        return _is_ramp_entry_node(other)
    return True


def _blocked_by_obstacle(
    point: tuple[float, float, float],
    obstacles: list[CollisionGeometry],
    clearance: float,
    current_surface: BoxCollision,
) -> bool:
    for obstacle in obstacles:
        if isinstance(obstacle, BoxCollision):
            if obstacle.model_name == current_surface.model_name:
                continue
            local = inverse_transform_point(obstacle.transform, point)
            half_x = obstacle.size[0] / 2.0 + clearance
            half_y = obstacle.size[1] / 2.0 + clearance
            z_margin = obstacle.size[2] / 2.0 + 0.45
            if (
                abs(local[0]) <= half_x
                and abs(local[1]) <= half_y
                and abs(local[2]) <= z_margin
            ):
                return True
            continue
        local = inverse_transform_point(obstacle.transform, point)
        if math.hypot(local[0], local[1]) <= obstacle.radius + clearance:
            if abs(local[2]) <= obstacle.length / 2.0 + 0.45:
                return True
    return False


def _surface_edge_margin(
    box: BoxCollision,
    point: tuple[float, float, float],
) -> float:
    local = inverse_transform_point(box.transform, point)
    return min(
        box.size[0] / 2.0 - abs(local[0]),
        box.size[1] / 2.0 - abs(local[1]),
    )


def _edge_risk(node: TerrainNode) -> float:
    if _is_floor_label(node.surface_label):
        return 0.0
    return max(0.0, 0.65 - node.edge_margin) * 1.2


def _nearest_node(
    nodes: list[TerrainNode],
    xy: tuple[float, float],
    z_reference: float,
    policy: str,
) -> Optional[int]:
    nearby = sorted(
        nodes,
        key=lambda node: math.hypot(node.x - xy[0], node.y - xy[1]),
    )[:80]
    if not nearby:
        return None
    if policy == 'highest':
        min_xy = math.hypot(nearby[0].x - xy[0], nearby[0].y - xy[1])
        candidates = [
            node
            for node in nearby
            if math.hypot(node.x - xy[0], node.y - xy[1]) <= min_xy + 0.75
        ]
        return max(candidates, key=lambda node: node.z).index
    return min(
        nearby,
        key=lambda node: (
            math.hypot(node.x - xy[0], node.y - xy[1])
            + abs(node.z - z_reference) * 0.6
        ),
    ).index


def _thin_path(path: list[TerrainNode]) -> list[TerrainNode]:
    if len(path) <= 2:
        return path
    reduced = [path[0]]
    previous_heading: Optional[float] = None
    distance_since_keep = 0.0
    for index in range(1, len(path) - 1):
        last = path[index - 1]
        current = path[index]
        next_node = path[index + 1]
        distance_since_keep += math.hypot(current.x - last.x, current.y - last.y)
        heading = math.atan2(next_node.y - current.y, next_node.x - current.x)
        heading_change = (
            abs(_normalize_angle(heading - previous_heading))
            if previous_heading is not None
            else 0.0
        )
        if distance_since_keep >= 0.8 or heading_change >= 0.35:
            reduced.append(current)
            distance_since_keep = 0.0
        previous_heading = heading
    reduced.append(path[-1])
    return reduced


def _waypoint_path(
    path: list[TerrainNode],
    spacing: float,
) -> list[TerrainNode]:
    if len(path) <= 2:
        return path
    waypoints = [path[0]]
    distance = 0.0
    for index in range(1, len(path) - 1):
        last = path[index - 1]
        current = path[index]
        distance += math.hypot(current.x - last.x, current.y - last.y)
        vertical_change = abs(current.z - waypoints[-1].z)
        surface_changed = current.surface_label != waypoints[-1].surface_label
        if distance >= spacing or vertical_change >= 0.16 or surface_changed:
            waypoints.append(current)
            distance = 0.0
    waypoints.append(path[-1])
    return waypoints


def _waypoints_after_start_clearance(
    path: list[TerrainNode],
    start_xy: tuple[float, float],
    clearance_radius: float,
) -> list[TerrainNode]:
    if len(path) <= 1:
        return path
    filtered: list[TerrainNode] = []
    for node in path:
        distance = math.hypot(node.x - start_xy[0], node.y - start_xy[1])
        if distance <= max(0.0, clearance_radius):
            continue
        filtered.append(node)
    return filtered or [path[-1]]


def _path_speed_limit(
    path: list[TerrainNode],
    slope_speed_limit: float,
    flat_speed_limit: float,
    slope_grade_threshold: float,
) -> float:
    if _path_has_slope_grade(path, slope_grade_threshold):
        return max(0.01, slope_speed_limit)
    return max(0.01, flat_speed_limit)


def _surface_speed_limit_for_label(
    surface_label: str,
    slope_speed_limit: float,
    flat_speed_limit: float,
) -> float:
    if _is_slope_label(surface_label):
        return max(0.01, slope_speed_limit)
    return max(0.01, flat_speed_limit)


def _path_has_slope_grade(
    path: list[TerrainNode],
    slope_grade_threshold: float,
) -> bool:
    for first, second in zip(path, path[1:]):
        horizontal = math.hypot(second.x - first.x, second.y - first.y)
        if horizontal <= 1e-6:
            continue
        grade = abs(second.z - first.z) / horizontal
        if grade >= max(0.0, slope_grade_threshold):
            return True
    return False


def _surface_z_reference(
    odom_z: float,
    current_xy: tuple[float, float],
    initial_xy: Optional[tuple[float, float]],
    initial_surface_z_hint: float,
    initial_surface_hint_radius: float,
    last_path: list[TerrainNode],
    last_path_surface_hint_radius: float,
) -> float:
    if abs(odom_z) > 0.05:
        return odom_z

    last_path_z = _nearest_path_surface_z(
        last_path,
        current_xy,
        max(0.0, last_path_surface_hint_radius),
    )
    if last_path_z is not None:
        return last_path_z

    if initial_xy is not None and initial_surface_z_hint >= 0.0:
        distance_from_initial = math.hypot(
            current_xy[0] - initial_xy[0],
            current_xy[1] - initial_xy[1],
        )
        if distance_from_initial <= max(0.0, initial_surface_hint_radius):
            return initial_surface_z_hint

    return odom_z


def _nearest_path_surface_z(
    path: list[TerrainNode],
    xy: tuple[float, float],
    max_distance: float,
) -> Optional[float]:
    if not path:
        return None
    nearest = min(path, key=lambda node: math.hypot(node.x - xy[0], node.y - xy[1]))
    distance = math.hypot(nearest.x - xy[0], nearest.y - xy[1])
    if distance <= max_distance:
        return nearest.z
    return None


def _surface_height_at_xy(
    nodes: list[TerrainNode],
    xy: tuple[float, float],
    z_hint: float,
    max_xy_distance: float = 0.75,
) -> Optional[float]:
    if z_hint < 0.0:
        return None
    candidates = [
        node
        for node in nodes
        if math.hypot(node.x - xy[0], node.y - xy[1])
        <= max(0.0, max_xy_distance)
    ]
    if not candidates:
        return None
    selected = min(
        candidates,
        key=lambda node: (
            math.hypot(node.x - xy[0], node.y - xy[1])
            + abs(node.z - z_hint) * 0.6
        ),
    )
    return selected.z


def _heuristic(node: TerrainNode, goal: TerrainNode) -> float:
    return math.sqrt(
        (node.x - goal.x) ** 2
        + (node.y - goal.y) ** 2
        + (node.z - goal.z) ** 2
    )


def _bin_key(x: float, y: float, resolution: float) -> tuple[int, int]:
    return int(round(x / resolution)), int(round(y / resolution))


def _is_floor_label(label: str) -> bool:
    return 'floor' in label or 'ground' in label


def _is_slope_label(label: str) -> bool:
    return 'ramp' in label or 'slope' in label or 'stair' in label


def _is_ramp_label(label: str) -> bool:
    return 'ramp' in label or 'slope' in label


def _is_ramp_entry_node(node: TerrainNode) -> bool:
    if not _is_ramp_label(node.surface_label):
        return False
    local_width = (
        node.surface_local_x
        if node.surface_width_axis == 'x'
        else node.surface_local_y
    )
    half_width = (
        node.surface_half_x
        if node.surface_width_axis == 'x'
        else node.surface_half_y
    )
    if half_width <= 0.0:
        return node.edge_margin >= 0.40
    return abs(local_width) <= half_width * 0.35


def _box_width_axis(box: BoxCollision) -> str:
    return 'x' if box.size[0] <= box.size[1] else 'y'


def _surface_support_margin(label: str, support_margin: float) -> float:
    if _is_floor_label(label):
        return 0.0
    if 'stair' in label or 'step' in label:
        return min(max(0.0, support_margin), 0.08)
    return support_margin


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def _direct_linear_speed(
    speed_limit: float,
    max_linear_speed: float,
    min_linear_speed: float,
    heading_error: float,
    max_heading_error_for_forward: float,
    target_distance: float,
    slow_radius: float,
) -> float:
    capped_speed = min(max(speed_limit, 0.0), max_linear_speed)
    if capped_speed <= 0.0:
        return 0.0
    if target_distance <= 0.03:
        return 0.0
    heading_limit = max(max_heading_error_for_forward, 1e-6)
    if abs(heading_error) >= heading_limit:
        return min(min_linear_speed, capped_speed)
    heading_scale = max(
        0.0,
        1.0 - abs(heading_error) / heading_limit,
    )
    distance_scale = min(max(target_distance / max(slow_radius, 1e-6), 0.0), 1.0)
    scaled_speed = capped_speed * max(heading_scale, 0.20) * distance_scale
    return max(min_linear_speed, scaled_speed)


def main() -> None:
    rclpy.init()
    node = TerrainPctPlanner()
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
