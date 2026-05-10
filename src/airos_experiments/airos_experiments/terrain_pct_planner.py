from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
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


@dataclass(frozen=True)
class TerrainNode:
    index: int
    x: float
    y: float
    z: float
    surface_label: str
    edge_margin: float


@dataclass(frozen=True)
class TerrainGraph:
    nodes: list[TerrainNode]
    adjacency: list[list[tuple[int, float]]]
    terrain_cloud: list[CloudPoint]


def build_terrain_graph(
    world_file: Path,
    grid_resolution: float = 0.40,
    robot_radius: float = 0.35,
    support_margin: float = 0.45,
    max_slope_grade: float = 0.55,
    max_step_height: float = 0.34,
) -> TerrainGraph:
    geometries = load_collision_geometries(world_file)
    traversable_boxes = list(iter_traversable_boxes(geometries))
    obstacles = list(iter_obstacle_geometries(geometries))
    nodes: list[TerrainNode] = []
    terrain_cloud: list[CloudPoint] = []

    for box in traversable_boxes:
        label = box.label
        margin = 0.0 if _is_floor_label(label) else support_margin
        terrain_cloud.extend(sample_box_top(box, grid_resolution, margin=0.0))
        for x, y, z, _ in sample_box_top(box, grid_resolution, margin=margin):
            if _blocked_by_obstacle(
                (x, y, z),
                obstacles,
                clearance=robot_radius,
                current_surface=box,
            ):
                continue
            edge_margin = _surface_edge_margin(box, (x, y, z))
            nodes.append(
                TerrainNode(
                    index=len(nodes),
                    x=x,
                    y=y,
                    z=z,
                    surface_label=label,
                    edge_margin=edge_margin,
                )
            )

    adjacency = _build_adjacency(
        nodes,
        grid_resolution=grid_resolution,
        max_slope_grade=max_slope_grade,
        max_step_height=max_step_height,
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
        self.declare_parameter('robot_radius', 0.35)
        self.declare_parameter('support_margin', 0.45)
        self.declare_parameter('max_slope_grade', 0.55)
        self.declare_parameter('max_step_height', 0.34)
        self.declare_parameter('goal_z_policy', 'highest')
        self.declare_parameter('send_nav2_goals', True)
        self.declare_parameter('waypoint_spacing', 0.90)
        self.declare_parameter('terrain_publish_period_sec', 4.0)

        world_file = Path(str(self.get_parameter('world_file').value))
        self._world_frame = str(self.get_parameter('world_frame').value)
        self._goal_z_policy = str(self.get_parameter('goal_z_policy').value)
        self._send_nav2_goals = bool(
            self.get_parameter('send_nav2_goals').value
        )
        self._waypoint_spacing = float(
            self.get_parameter('waypoint_spacing').value
        )
        self._use_initial_pose_anchor = bool(
            self.get_parameter('use_initial_pose_anchor').value
        )
        self._graph = build_terrain_graph(
            world_file,
            grid_resolution=float(self.get_parameter('grid_resolution').value),
            robot_radius=float(self.get_parameter('robot_radius').value),
            support_margin=float(self.get_parameter('support_margin').value),
            max_slope_grade=float(self.get_parameter('max_slope_grade').value),
            max_step_height=float(self.get_parameter('max_step_height').value),
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
        self._nav_action_client = ActionClient(
            self,
            NavigateThroughPoses,
            '/navigate_through_poses',
        )

        self._odom_msg: Optional[Odometry] = None
        self._pending_initial_pose: Optional[Pose2D] = None
        self._odom_anchor: Optional[OdomAnchor] = None
        self._terrain_timer = self.create_timer(
            max(float(self.get_parameter('terrain_publish_period_sec').value), 0.5),
            self._publish_terrain_cloud,
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
        if self._odom_msg is None:
            return 0.0, 0.0, 0.0
        pose = _pose_from_odom(self._odom_msg)
        if self._odom_anchor is not None:
            pose = _map_pose_from_anchor(pose, self._odom_anchor)
        z = float(self._odom_msg.pose.pose.position.z)
        return pose.x, pose.y, z

    def _goal_callback(self, msg: PoseStamped) -> None:
        start_x, start_y, start_z = self._current_pose()
        goal_x = float(msg.pose.position.x)
        goal_y = float(msg.pose.position.y)
        path = plan_terrain_path(
            self._graph,
            (start_x, start_y),
            (goal_x, goal_y),
            start_z=start_z,
            goal_z_policy=self._goal_z_policy,
        )
        if not path:
            self.get_logger().warning(
                'terrain planner failed to find a traversable route: '
                f'start=({start_x:.2f},{start_y:.2f}) '
                f'goal=({goal_x:.2f},{goal_y:.2f})'
            )
            return
        self._publish_path(path)
        if self._send_nav2_goals:
            self._send_waypoint_goal(path, msg)

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
        if not self._nav_action_client.server_is_ready():
            if not self._nav_action_client.wait_for_server(timeout_sec=0.1):
                self.get_logger().warning(
                    'navigate_through_poses server is not ready; '
                    'published /pct_path only'
                )
                return
        goal_msg = NavigateThroughPoses.Goal()
        reduced_path = _waypoint_path(path, self._waypoint_spacing)
        stamp = self.get_clock().now().to_msg()
        poses = [
            self._pose_stamped_for_node(node, reduced_path, index, stamp)
            for index, node in enumerate(reduced_path)
        ]
        if poses:
            poses[-1].pose.orientation = original_goal.pose.orientation
        goal_msg.poses = poses
        self._nav_action_client.send_goal_async(goal_msg)
        self.get_logger().info(
            'sent terrain-guided NavigateThroughPoses goal: '
            f'waypoints={len(poses)} path_nodes={len(path)}'
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
            if dz > max_step_height or grade > max_slope_grade:
                continue
            slope_cost = 1.0 + grade * 1.8
            edge_risk = _edge_risk(node) + _edge_risk(other)
            cost = math.sqrt(horizontal * horizontal + dz * dz) * slope_cost
            adjacency[node.index].append((other_index, cost + edge_risk))
    return adjacency


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


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


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
