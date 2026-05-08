from __future__ import annotations

import argparse
import heapq
import json
import math
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap, SetInitialPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.client import Client
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


@dataclass(frozen=True)
class Mission:
    mission_id: str
    start_pose: tuple[float, float, float]
    goal_pose: tuple[float, float, float]
    route_id: str
    dynamic_obstacle_seed: int
    speed_limit: float
    expected_timeout_sec: float


@dataclass(frozen=True)
class RouteGraph:
    nodes: dict[int, tuple[float, float]]
    edges: dict[int, list[tuple[int, float]]]


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _pose_tuple(value: list[float]) -> tuple[float, float, float]:
    if len(value) != 3:
        raise ValueError(f'pose must have 3 values, got {value}')
    return float(value[0]), float(value[1]), float(value[2])


def _load_missions(path: Path) -> list[Mission]:
    payload = yaml.safe_load(path.read_text(encoding='utf-8'))
    entries = payload['missions'] if isinstance(payload, dict) else payload
    missions: list[Mission] = []
    for entry in entries:
        missions.append(
            Mission(
                mission_id=str(entry['mission_id']),
                start_pose=_pose_tuple(entry['start_pose']),
                goal_pose=_pose_tuple(entry['goal_pose']),
                route_id=str(entry.get('route_id', '')),
                dynamic_obstacle_seed=int(
                    entry.get('dynamic_obstacle_seed', 0)
                ),
                speed_limit=float(entry.get('speed_limit', 0.35)),
                expected_timeout_sec=float(
                    entry.get('expected_timeout_sec', 90.0)
                ),
            )
        )
    return missions


def _load_route_graph(path: Path) -> RouteGraph:
    data = json.loads(path.read_text(encoding='utf-8'))
    nodes: dict[int, tuple[float, float]] = {}
    edges: dict[int, list[tuple[int, float]]] = {}
    for feature in data.get('features', []):
        properties = feature.get('properties', {})
        geometry = feature.get('geometry', {})
        feature_id = int(properties['id'])
        if geometry.get('type') == 'Point':
            x, y = geometry['coordinates'][:2]
            nodes[feature_id] = (float(x), float(y))
            edges.setdefault(feature_id, [])
        elif geometry.get('type') == 'MultiLineString':
            start_id = int(properties['startid'])
            end_id = int(properties['endid'])
            start = nodes.get(start_id)
            end = nodes.get(end_id)
            if start is None or end is None:
                continue
            cost = math.hypot(end[0] - start[0], end[1] - start[1])
            edges.setdefault(start_id, []).append((end_id, cost))
    return RouteGraph(nodes=nodes, edges=edges)


def _nearest_node(
    graph: RouteGraph,
    pose: tuple[float, float, float],
) -> int:
    if not graph.nodes:
        raise RuntimeError('route graph has no nodes')
    x, y, _ = pose
    return min(
        graph.nodes,
        key=lambda node_id: math.hypot(
            graph.nodes[node_id][0] - x,
            graph.nodes[node_id][1] - y,
        ),
    )


def _route_node_path(
    graph: RouteGraph,
    start_id: int,
    goal_id: int,
) -> list[int]:
    queue: list[tuple[float, int, list[int]]] = [(0.0, start_id, [start_id])]
    visited: set[int] = set()
    while queue:
        cost, node_id, path = heapq.heappop(queue)
        if node_id in visited:
            continue
        if node_id == goal_id:
            return path
        visited.add(node_id)
        for next_id, edge_cost in graph.edges.get(node_id, []):
            if next_id not in visited:
                heapq.heappush(
                    queue,
                    (cost + edge_cost, next_id, path + [next_id]),
                )
    raise RuntimeError(f'no route path from {start_id} to {goal_id}')


def _route_waypoints(
    graph: RouteGraph,
    mission: Mission,
) -> list[tuple[float, float, float]]:
    start_id = _nearest_node(graph, mission.start_pose)
    goal_id = _nearest_node(graph, mission.goal_pose)
    node_path = _route_node_path(graph, start_id, goal_id)
    waypoints: list[tuple[float, float, float]] = []
    for index, node_id in enumerate(node_path[1:], start=1):
        x, y = graph.nodes[node_id]
        if index == len(node_path) - 1:
            yaw = mission.goal_pose[2]
        else:
            next_x, next_y = graph.nodes[node_path[index + 1]]
            yaw = math.atan2(next_y - y, next_x - x)
        waypoints.append((x, y, yaw))
    return waypoints or [mission.goal_pose]


def _reset_sim_entity(
    world_name: str,
    entity_name: str,
    pose: tuple[float, float, float],
    entity_z: float,
    timeout_ms: int,
) -> None:
    x, y, yaw = pose
    qx, qy, qz, qw = _yaw_to_quaternion(yaw)
    request = (
        f'name: "{entity_name}", '
        f'position: {{x: {x}, y: {y}, z: {entity_z}}}, '
        f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}'
    )
    result = subprocess.run(
        [
            'ign',
            'service',
            '-s',
            f'/world/{world_name}/set_pose',
            '--reqtype',
            'ignition.msgs.Pose',
            '--reptype',
            'ignition.msgs.Boolean',
            '--timeout',
            str(timeout_ms),
            '--req',
            request,
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    output = f'{result.stdout}\n{result.stderr}'
    if result.returncode != 0 or 'true' not in output.lower():
        raise RuntimeError(
            f'failed to reset {entity_name} in {world_name}: {output.strip()}'
        )


class NavTrialRunner(Node):
    def __init__(self) -> None:
        super().__init__('nav_trial_runner')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter(
            'cmd_vel_topic',
            '/diff_drive_controller/cmd_vel_unstamped',
        )
        self.declare_parameter('cmd_vel_smoothed_topic', '/cmd_vel_smoothed')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('initial_pose_service', '/set_initial_pose')
        self.declare_parameter('initial_pose_service_timeout_sec', 5.0)
        self.declare_parameter('initial_pose_subscriber_timeout_sec', 10.0)
        self.declare_parameter('initial_pose_publish_count', 8)
        self.declare_parameter('initial_pose_publish_period_sec', 0.25)
        self.declare_parameter('initial_pose_stamp_backdate_sec', 0.25)
        self.declare_parameter('initial_pose_settle_sec', 4.0)
        self.declare_parameter('localization_timeout_sec', 25.0)
        self.declare_parameter('action_server_timeout_sec', 30.0)
        self.declare_parameter('goal_accept_timeout_sec', 30.0)
        self.declare_parameter('use_route_waypoints', False)
        self.declare_parameter(
            'route_graph_path',
            'src/airos_nav/routes/single_floor_lab_route.geojson',
        )
        self.declare_parameter('collision_range_threshold_m', 0.12)
        self.declare_parameter('collision_clear_threshold_m', 0.18)

        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._route_client = ActionClient(
            self,
            NavigateThroughPoses,
            'navigate_through_poses',
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            1,
        )
        self._set_initial_pose_client: Client = self.create_client(
            SetInitialPose,
            str(self.get_parameter('initial_pose_service').value),
        )
        self._clear_global_costmap_client: Client = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap',
        )
        self._clear_local_costmap_client: Client = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap',
        )
        self._odom_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_callback,
            10,
        )
        self._cmd_sub = self.create_subscription(
            Twist,
            str(self.get_parameter('cmd_vel_topic').value),
            self._cmd_callback,
            10,
        )
        self._smooth_sub = self.create_subscription(
            Twist,
            str(self.get_parameter('cmd_vel_smoothed_topic').value),
            self._smoothed_callback,
            10,
        )
        self._scan_sub = self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self._scan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self._last_xy: tuple[float, float] | None = None
        self._path_length = 0.0
        self._latest_cmd = 0.0
        self._latest_smoothed = 0.0
        self._stop_events = 0
        self._minimum_obstacle_distance = math.inf
        self._collision_events = 0
        self._collision_latched = False
        self._last_cmd_time: float | None = None
        self._cmd_periods: list[float] = []
        self._route_graph: RouteGraph | None = None
        self._execution_mode = 'navigate_to_pose'

    def run_mission(self, mission: Mission) -> dict[str, Any]:
        self._reset_metrics()
        self._publish_initial_pose(mission.start_pose)
        self._settle_after_initial_pose()
        if not self._wait_for_localization(mission.start_pose):
            return self._record(
                mission,
                time.monotonic(),
                GoalStatus.STATUS_ABORTED,
                'localization_unavailable',
            )
        self._clear_costmaps()

        start = time.monotonic()
        if self._use_route_waypoints():
            client = self._route_client
            action_name = 'navigate_through_poses'
        else:
            client = self._client
            action_name = 'navigate_to_pose'
        if not self._wait_for_action_server(client, action_name):
            return self._record(
                mission,
                start,
                GoalStatus.STATUS_ABORTED,
                'action_server_unavailable',
            )

        goal_msg = self._make_goal(mission)

        goal_handle = self._send_goal_when_ready(client, goal_msg)
        if goal_handle is None:
            return self._record(
                mission,
                start,
                GoalStatus.STATUS_ABORTED,
                'rejected',
            )

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            if time.monotonic() - start > mission.expected_timeout_sec:
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(
                    self,
                    cancel_future,
                    timeout_sec=2.0,
                )
                return self._record(
                    mission,
                    start,
                    GoalStatus.STATUS_CANCELED,
                    'timeout',
                )
            rclpy.spin_once(self, timeout_sec=0.1)

        result = result_future.result()
        status = (
            result.status
            if result is not None
            else GoalStatus.STATUS_UNKNOWN
        )
        return self._record(mission, start, status, 'finished')

    def _wait_for_action_server(self, client: ActionClient, name: str) -> bool:
        deadline = time.monotonic() + float(
            self.get_parameter('action_server_timeout_sec').value
        )
        while rclpy.ok() and time.monotonic() < deadline:
            if client.wait_for_server(timeout_sec=1.0):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().warning(f'{name} action server unavailable')
        return False

    def _reset_metrics(self) -> None:
        self._last_xy = None
        self._path_length = 0.0
        self._latest_cmd = 0.0
        self._latest_smoothed = 0.0
        self._stop_events = 0
        self._minimum_obstacle_distance = math.inf
        self._collision_events = 0
        self._collision_latched = False
        self._last_cmd_time = None
        self._cmd_periods = []

    def _record(
        self,
        mission: Mission,
        start: float,
        status: int,
        reason: str,
    ) -> dict[str, Any]:
        mean_cmd_period = (
            sum(self._cmd_periods) / len(self._cmd_periods)
            if self._cmd_periods
            else None
        )
        max_cmd_period = max(self._cmd_periods) if self._cmd_periods else None
        min_distance = (
            round(self._minimum_obstacle_distance, 3)
            if math.isfinite(self._minimum_obstacle_distance)
            else None
        )
        return {
            'mission_id': mission.mission_id,
            'route_id': mission.route_id,
            'status': int(status),
            'success': status == GoalStatus.STATUS_SUCCEEDED,
            'reason': reason,
            'elapsed_sec': round(time.monotonic() - start, 3),
            'path_length_m': round(self._path_length, 3),
            'emergency_stop_count': self._stop_events,
            'collision_count': self._collision_events,
            'collision_metric_source': 'scan_range_threshold',
            'minimum_obstacle_distance_m': min_distance,
            'mean_cmd_period_sec': (
                round(mean_cmd_period, 4) if mean_cmd_period else None
            ),
            'max_cmd_period_sec': (
                round(max_cmd_period, 4) if max_cmd_period else None
            ),
            'dynamic_obstacle_seed': mission.dynamic_obstacle_seed,
            'speed_limit': mission.speed_limit,
            'execution_mode': self._execution_mode,
        }

    def _publish_initial_pose(self, pose: tuple[float, float, float]) -> None:
        if self._set_initial_pose(pose):
            return
        self._wait_for_initial_pose_subscriber()
        publish_count = int(
            self.get_parameter('initial_pose_publish_count').value
        )
        publish_period = float(
            self.get_parameter('initial_pose_publish_period_sec').value
        )
        for _ in range(max(1, publish_count)):
            self._publish_initial_pose_once(pose)
            rclpy.spin_once(self, timeout_sec=max(0.0, publish_period))

    def _set_initial_pose(self, pose: tuple[float, float, float]) -> bool:
        timeout_sec = float(
            self.get_parameter('initial_pose_service_timeout_sec').value
        )
        if not self._set_initial_pose_client.wait_for_service(
            timeout_sec=timeout_sec
        ):
            return False
        request = SetInitialPose.Request()
        request.pose = self._make_initial_pose_msg(pose)
        future = self._set_initial_pose_client.call_async(request)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=timeout_sec,
        )
        return future.result() is not None

    def _wait_for_initial_pose_subscriber(self) -> bool:
        deadline = time.monotonic() + float(
            self.get_parameter('initial_pose_subscriber_timeout_sec').value
        )
        while rclpy.ok() and time.monotonic() < deadline:
            if self._initial_pose_pub.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().warning('initialpose subscriber not discovered')
        return False

    def _publish_initial_pose_once(
        self,
        pose: tuple[float, float, float],
    ) -> None:
        self._initial_pose_pub.publish(self._make_initial_pose_msg(pose))

    def _make_initial_pose_msg(
        self,
        pose: tuple[float, float, float],
    ) -> PoseWithCovarianceStamped:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self._backdated_stamp()
        msg.header.frame_id = str(self.get_parameter('global_frame').value)
        msg.pose.pose = self._make_pose(pose).pose
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        return msg

    def _backdated_stamp(self):
        stamp = self.get_clock().now().to_msg()
        backdate_nsec = int(
            max(
                0.0,
                float(
                    self.get_parameter(
                        'initial_pose_stamp_backdate_sec'
                    ).value
                ),
            )
            * 1_000_000_000
        )
        total_nsec = max(
            0,
            stamp.sec * 1_000_000_000 + stamp.nanosec - backdate_nsec,
        )
        stamp.sec = total_nsec // 1_000_000_000
        stamp.nanosec = total_nsec % 1_000_000_000
        return stamp

    def _wait_for_localization(
        self,
        pose: tuple[float, float, float],
    ) -> bool:
        deadline = time.monotonic() + float(
            self.get_parameter('localization_timeout_sec').value
        )
        publish_period = float(
            self.get_parameter('initial_pose_publish_period_sec').value
        )
        next_publish = time.monotonic()
        while rclpy.ok() and time.monotonic() < deadline:
            if self._localized():
                return True
            now = time.monotonic()
            if now >= next_publish:
                self._publish_initial_pose_once(pose)
                next_publish = now + max(0.1, publish_period)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().warning('map to base transform unavailable')
        return False

    def _localized(self) -> bool:
        return self._tf_buffer.can_transform(
            str(self.get_parameter('global_frame').value),
            str(self.get_parameter('base_frame').value),
            Time(),
            timeout=Duration(seconds=0.05),
        )

    def _settle_after_initial_pose(self) -> None:
        deadline = time.monotonic() + float(
            self.get_parameter('initial_pose_settle_sec').value
        )
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _clear_costmaps(self) -> None:
        self._clear_costmap(self._clear_global_costmap_client)
        self._clear_costmap(self._clear_local_costmap_client)

    def _clear_costmap(self, client: Client) -> None:
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning(
                f'costmap clear service unavailable: {client.srv_name}'
            )
            return
        future = client.call_async(ClearEntireCostmap.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None:
            self.get_logger().warning(
                f'costmap clear failed: {client.srv_name}'
            )

    def _send_goal_when_ready(self, client: ActionClient, goal_msg):
        deadline = time.monotonic() + float(
            self.get_parameter('goal_accept_timeout_sec').value
        )
        while rclpy.ok() and time.monotonic() < deadline:
            send_future = client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(
                self,
                send_future,
                timeout_sec=5.0,
            )
            goal_handle = send_future.result()
            if goal_handle is not None and goal_handle.accepted:
                return goal_handle
            rclpy.spin_once(self, timeout_sec=1.0)
        return None

    def _make_goal(self, mission: Mission):
        if not self._use_route_waypoints():
            self._execution_mode = 'navigate_to_pose'
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self._make_pose(mission.goal_pose)
            return goal_msg

        self._execution_mode = 'navigate_through_poses'
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = [
            self._make_pose(pose)
            for pose in _route_waypoints(self._get_route_graph(), mission)
        ]
        return goal_msg

    def _use_route_waypoints(self) -> bool:
        return bool(self.get_parameter('use_route_waypoints').value)

    def _get_route_graph(self) -> RouteGraph:
        if self._route_graph is None:
            self._route_graph = _load_route_graph(
                Path(str(self.get_parameter('route_graph_path').value))
            )
        return self._route_graph

    def _make_pose(self, pose: tuple[float, float, float]) -> PoseStamped:
        x, y, yaw = pose
        qx, qy, qz, qw = _yaw_to_quaternion(yaw)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def _odom_callback(self, msg: Odometry) -> None:
        current = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self._last_xy is not None:
            self._path_length += math.hypot(
                current[0] - self._last_xy[0],
                current[1] - self._last_xy[1],
            )
        self._last_xy = current

    def _cmd_callback(self, msg: Twist) -> None:
        now = time.monotonic()
        if self._last_cmd_time is not None:
            self._cmd_periods.append(now - self._last_cmd_time)
        self._last_cmd_time = now
        self._latest_cmd = abs(msg.linear.x) + abs(msg.angular.z)
        if self._latest_smoothed > 0.08 and self._latest_cmd < 0.01:
            self._stop_events += 1

    def _smoothed_callback(self, msg: Twist) -> None:
        self._latest_smoothed = abs(msg.linear.x) + abs(msg.angular.z)

    def _scan_callback(self, msg: LaserScan) -> None:
        valid_ranges = [
            value
            for value in msg.ranges
            if math.isfinite(value) and msg.range_min <= value <= msg.range_max
        ]
        if not valid_ranges:
            return

        current_min = min(valid_ranges)
        self._minimum_obstacle_distance = min(
            self._minimum_obstacle_distance,
            current_min,
        )
        collision_threshold = float(
            self.get_parameter('collision_range_threshold_m').value
        )
        clear_threshold = float(
            self.get_parameter('collision_clear_threshold_m').value
        )
        if current_min <= collision_threshold and not self._collision_latched:
            self._collision_events += 1
            self._collision_latched = True
        elif current_min >= clear_threshold:
            self._collision_latched = False


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Run fixed AIROS Nav2 missions.'
    )
    parser.add_argument('--mission', required=True)
    parser.add_argument('--mission-id', default='')
    parser.add_argument('--count', type=int, default=1)
    parser.add_argument('--output', default='log/airos_nav_trials.jsonl')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--reset-sim', action='store_true')
    parser.add_argument('--world-name', default='single_floor_lab')
    parser.add_argument('--entity-name', default='go2w_nav_eq')
    parser.add_argument('--entity-z', type=float, default=0.24)
    parser.add_argument('--reset-timeout-ms', type=int, default=3000)
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    missions = _load_missions(Path(args.mission))
    if args.mission_id:
        missions = [
            mission
            for mission in missions
            if mission.mission_id == args.mission_id
        ]
        if not missions:
            raise RuntimeError(f'unknown mission_id: {args.mission_id}')
    if args.dry_run:
        print(
            json.dumps(
                {
                    'missions': len(missions),
                    'mission_ids': [
                        mission.mission_id for mission in missions
                    ],
                },
                ensure_ascii=False,
            )
        )
        return

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=sys.argv)
    node = NavTrialRunner()
    try:
        with output.open('a', encoding='utf-8') as stream:
            for index in range(args.count):
                mission = missions[index % len(missions)]
                if args.reset_sim:
                    _reset_sim_entity(
                        args.world_name,
                        args.entity_name,
                        mission.start_pose,
                        args.entity_z,
                        args.reset_timeout_ms,
                    )
                result = node.run_mission(mission)
                stream.write(json.dumps(result, ensure_ascii=False) + '\n')
                stream.flush()
                print(json.dumps(result, ensure_ascii=False))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
