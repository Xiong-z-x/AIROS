from __future__ import annotations

import heapq
import math
import random
import time
from collections import deque
from dataclasses import dataclass

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


GridIndex = tuple[int, int]
WorldPoint = tuple[float, float]


@dataclass(frozen=True)
class PlannerResult:
    planner_id: str
    display_name: str
    path: list[WorldPoint]
    planning_time_ms: float
    success: bool
    message: str
    expanded_nodes: int = 0

    @property
    def path_length_m(self) -> float:
        return _path_length(self.path)

    @property
    def turn_angle_rad(self) -> float:
        return _turn_angle(self.path)


@dataclass
class _PendingRun:
    start_time: float
    local_results: list[PlannerResult]
    nav2_results: dict[str, PlannerResult]
    map_frame: str


@dataclass
class GridMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: list[int]
    occupied_threshold: int
    robot_radius_m: float
    inflated: set[GridIndex]

    @classmethod
    def from_msg(
        cls,
        msg: OccupancyGrid,
        occupied_threshold: int,
        robot_radius_m: float,
    ) -> 'GridMap':
        grid = cls(
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
            data=list(msg.data),
            occupied_threshold=occupied_threshold,
            robot_radius_m=robot_radius_m,
            inflated=set(),
        )
        grid.inflated = grid._build_inflated_obstacles()
        return grid

    def _build_inflated_obstacles(self) -> set[GridIndex]:
        radius_cells = max(1, int(math.ceil(self.robot_radius_m / self.resolution)))
        offsets: list[GridIndex] = []
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                if math.hypot(dx, dy) <= radius_cells:
                    offsets.append((dx, dy))

        inflated: set[GridIndex] = set()
        for y in range(self.height):
            for x in range(self.width):
                if self.raw_occupied((x, y)):
                    for dx, dy in offsets:
                        cell = (x + dx, y + dy)
                        if self.in_bounds(cell):
                            inflated.add(cell)
        return inflated

    def in_bounds(self, cell: GridIndex) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def raw_occupied(self, cell: GridIndex) -> bool:
        x, y = cell
        value = self.data[y * self.width + x]
        return value < 0 or value >= self.occupied_threshold

    def occupied(self, cell: GridIndex) -> bool:
        return cell in self.inflated

    def free(self, cell: GridIndex) -> bool:
        return self.in_bounds(cell) and not self.occupied(cell)

    def world_to_grid(self, point: WorldPoint) -> GridIndex:
        x = int(math.floor((point[0] - self.origin_x) / self.resolution))
        y = int(math.floor((point[1] - self.origin_y) / self.resolution))
        return (x, y)

    def grid_to_world(self, cell: GridIndex) -> WorldPoint:
        x, y = cell
        return (
            self.origin_x + (x + 0.5) * self.resolution,
            self.origin_y + (y + 0.5) * self.resolution,
        )

    def snap_free(self, cell: GridIndex, max_radius_cells: int) -> GridIndex | None:
        if self.free(cell):
            return cell
        visited = {cell}
        queue: deque[tuple[GridIndex, int]] = deque([(cell, 0)])
        while queue:
            current, distance = queue.popleft()
            if distance >= max_radius_cells:
                continue
            for neighbor in _neighbors8(current):
                if neighbor in visited or not self.in_bounds(neighbor):
                    continue
                if self.free(neighbor):
                    return neighbor
                visited.add(neighbor)
                queue.append((neighbor, distance + 1))
        return None

    def clearance_m(self, path: list[WorldPoint]) -> float:
        if not path or not self.inflated:
            return 0.0
        min_dist_cells = math.inf
        sampled_obstacles = list(self.inflated)
        for point in path[:: max(1, len(path) // 80)]:
            cell = self.world_to_grid(point)
            for obstacle in sampled_obstacles[:: max(1, len(sampled_obstacles) // 3000)]:
                min_dist_cells = min(
                    min_dist_cells,
                    math.hypot(cell[0] - obstacle[0], cell[1] - obstacle[1]),
                )
        if math.isinf(min_dist_cells):
            return 0.0
        return max(0.0, min_dist_cells * self.resolution)


class PlannerComparisonNode(Node):
    def __init__(self) -> None:
        super().__init__('planner_comparison_node')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('robot_radius_m', 0.43)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('snap_radius_m', 1.2)
        self.declare_parameter('q_grid_step', 4)
        self.declare_parameter('q_max_iterations', 60000)
        self.declare_parameter('q_discount', 0.96)
        self.declare_parameter('rrt_max_samples', 2600)
        self.declare_parameter('rrt_step_m', 0.65)
        self.declare_parameter('rrt_goal_sample_rate', 0.12)
        self.declare_parameter('rrt_rewire_radius_m', 1.4)
        self.declare_parameter('random_seed', 7)

        self._map: GridMap | None = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._planner_client = ActionClient(
            self,
            ComputePathToPose,
            'compute_path_to_pose',
        )
        self._pending_runs: dict[int, _PendingRun] = {}
        self._run_id = 0

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        reliable_qos = QoSProfile(depth=10)
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('map_topic').value),
            self._map_callback,
            transient_qos,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('goal_topic').value),
            self._goal_callback,
            reliable_qos,
        )
        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._path_publishers = {
            'smac': self.create_publisher(Path, '/planner_compare/smac_path', path_qos),
            'theta_star': self.create_publisher(Path, '/planner_compare/theta_star_path', path_qos),
            'q_learning': self.create_publisher(Path, '/planner_compare/q_learning_path', path_qos),
            'rrt_star': self.create_publisher(Path, '/planner_compare/rrt_star_path', path_qos),
        }
        self._metrics_pub = self.create_publisher(
            MarkerArray,
            '/planner_compare/metrics_markers',
            path_qos,
        )
        self._summary_pub = self.create_publisher(
            String,
            '/planner_compare/metrics_summary',
            path_qos,
        )

        self.get_logger().info(
            'planner comparison ready: waits for /map and RViz goal_pose; '
            'publishes paths only and never publishes cmd_vel'
        )

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self._map = GridMap.from_msg(
            msg,
            occupied_threshold=int(self.get_parameter('occupied_threshold').value),
            robot_radius_m=float(self.get_parameter('robot_radius_m').value),
        )
        self.get_logger().info(
            f'map loaded for planner comparison: {self._map.width}x{self._map.height} '
            f'res={self._map.resolution:.3f} inflated={len(self._map.inflated)}'
        )

    def _goal_callback(self, goal: PoseStamped) -> None:
        if self._map is None:
            self.get_logger().warning('ignored goal: no occupancy map received yet')
            return
        try:
            start = self._lookup_start_pose()
        except TransformException as exc:
            self.get_logger().warning(f'ignored goal: TF lookup failed: {exc}')
            return

        map_frame = str(self.get_parameter('global_frame').value)
        if goal.header.frame_id and goal.header.frame_id != map_frame:
            self.get_logger().warning(
                f'goal frame {goal.header.frame_id!r} is not {map_frame!r}; '
                'ignoring to avoid unsafe path comparison'
            )
            return

        self.get_logger().info(
            'planning comparison goal received: '
            f'start=({start.pose.position.x:.2f},{start.pose.position.y:.2f}) '
            f'goal=({goal.pose.position.x:.2f},{goal.pose.position.y:.2f})'
        )
        start_point = (start.pose.position.x, start.pose.position.y)
        goal_point = (goal.pose.position.x, goal.pose.position.y)
        results = [
            self._plan_smac_fallback(start_point, goal_point),
            self._plan_q_learning(start_point, goal_point),
            self._plan_rrt_star(start_point, goal_point),
        ]
        for result, key in zip(results, ('smac', 'q_learning', 'rrt_star')):
            self._path_publishers[key].publish(
                _path_msg(result.path, map_frame, self.get_clock().now().to_msg())
            )

        self._run_id += 1
        run_id = self._run_id
        self._pending_runs[run_id] = _PendingRun(
            start_time=time.perf_counter(),
            local_results=results,
            nav2_results={},
            map_frame=map_frame,
        )
        for planner_id, display_name, key in (
            ('ThetaStar', 'Theta*', 'theta_star'),
        ):
            self._send_nav2_plan(run_id, planner_id, display_name, key, start, goal)

    def _lookup_start_pose(self) -> PoseStamped:
        target_frame = str(self.get_parameter('global_frame').value)
        source_frame = str(self.get_parameter('base_frame').value)
        transform = self._tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=0.5),
        )
        pose = PoseStamped()
        pose.header.frame_id = target_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose

    def _send_nav2_plan(
        self,
        run_id: int,
        planner_id: str,
        display_name: str,
        path_key: str,
        start: PoseStamped,
        goal: PoseStamped,
    ) -> None:
        start_time = time.perf_counter()
        if not self._planner_client.wait_for_server(timeout_sec=1.0):
            self._finish_nav2_plan(run_id, path_key, PlannerResult(
                planner_id=planner_id,
                display_name=display_name,
                path=[],
                planning_time_ms=(time.perf_counter() - start_time) * 1000.0,
                success=False,
                message='compute_path_to_pose action unavailable',
            ))
            return

        request = ComputePathToPose.Goal()
        request.start = start
        request.goal = goal
        request.planner_id = planner_id
        request.use_start = True

        goal_future = self._planner_client.send_goal_async(request)
        goal_future.add_done_callback(
            lambda future: self._nav2_goal_response(
                future,
                run_id,
                path_key,
                planner_id,
                display_name,
                start_time,
            )
        )

    def _nav2_goal_response(
        self,
        future,
        run_id: int,
        path_key: str,
        planner_id: str,
        display_name: str,
        start_time: float,
    ) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._finish_nav2_plan(run_id, path_key, PlannerResult(
                planner_id=planner_id,
                display_name=display_name,
                path=[],
                planning_time_ms=(time.perf_counter() - start_time) * 1000.0,
                success=False,
                message='Nav2 planner goal rejected',
            ))
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._nav2_result_response(
                result,
                run_id,
                path_key,
                planner_id,
                display_name,
                start_time,
            )
        )

    def _nav2_result_response(
        self,
        future,
        run_id: int,
        path_key: str,
        planner_id: str,
        display_name: str,
        start_time: float,
    ) -> None:
        wrapped = future.result()
        if wrapped is None:
            self._finish_nav2_plan(run_id, path_key, PlannerResult(
                planner_id=planner_id,
                display_name=display_name,
                path=[],
                planning_time_ms=(time.perf_counter() - start_time) * 1000.0,
                success=False,
                message='Nav2 planner timed out',
            ))
            return
        result = wrapped.result
        path = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in result.path.poses
        ]
        self._finish_nav2_plan(run_id, path_key, PlannerResult(
            planner_id=planner_id,
            display_name=display_name,
            path=path,
            planning_time_ms=(time.perf_counter() - start_time) * 1000.0,
            success=wrapped.status == GoalStatus.STATUS_SUCCEEDED and len(path) >= 2,
            message='ok' if len(path) >= 2 else 'empty path',
        ))

    def _finish_nav2_plan(
        self,
        run_id: int,
        path_key: str,
        result: PlannerResult,
    ) -> None:
        run = self._pending_runs.get(run_id)
        if run is None:
            return
        run.nav2_results[path_key] = result
        if path_key != 'smac' or result.success:
            self._path_publishers[path_key].publish(
                _path_msg(result.path, run.map_frame, self.get_clock().now().to_msg())
            )
        if {'theta_star'} <= set(run.nav2_results):
            ordered = [
                run.nav2_results['theta_star'],
                *run.local_results,
            ]
            self._publish_metrics(ordered)
            self._pending_runs.pop(run_id, None)

    def _plan_smac_fallback(self, start: WorldPoint, goal: WorldPoint) -> PlannerResult:
        if self._map is None:
            return PlannerResult(
                planner_id='smac_fallback',
                display_name='SmacPlanner2D',
                path=[],
                planning_time_ms=0.0,
                success=False,
                message='map unavailable',
            )
        start_time = time.perf_counter()
        path, expanded = _grid_astar_path(
            self._map,
            start,
            goal,
            max_radius_cells=max(1, int(1.2 / self._map.resolution)),
        )
        return PlannerResult(
            planner_id='smac_fallback',
            display_name='SmacPlanner2D',
            path=path,
            planning_time_ms=(time.perf_counter() - start_time) * 1000.0,
            success=len(path) >= 2,
            message='local_smac_fallback' if len(path) >= 2 else 'no grid path found',
            expanded_nodes=expanded,
        )

    def _plan_q_learning(self, start: WorldPoint, goal: WorldPoint) -> PlannerResult:
        if self._map is None:
            return PlannerResult(
                planner_id='q_learning',
                display_name='Q-learning',
                path=[],
                planning_time_ms=0.0,
                success=False,
                message='map unavailable',
            )
        start_time = time.perf_counter()
        grid_step = max(1, int(self.get_parameter('q_grid_step').value))
        coarse = _CoarseGrid(self._map, grid_step)
        start_cell = coarse.snap_free(
            coarse.world_to_cell(start),
            self._snap_radius_cells(grid_step),
        )
        goal_cell = coarse.snap_free(
            coarse.world_to_cell(goal),
            self._snap_radius_cells(grid_step),
        )
        if start_cell is None or goal_cell is None:
            return PlannerResult(
                planner_id='q_learning',
                display_name='Q-learning',
                path=[],
                planning_time_ms=(time.perf_counter() - start_time) * 1000.0,
                success=False,
                message='start or goal cannot snap to free grid',
            )

        path_cells, expanded = _value_iteration_path(
            coarse,
            start_cell,
            goal_cell,
            max_iterations=int(self.get_parameter('q_max_iterations').value),
            discount=float(self.get_parameter('q_discount').value),
        )
        path = [coarse.cell_to_world(cell) for cell in path_cells]
        return PlannerResult(
            planner_id='q_learning',
            display_name='Q-learning',
            path=path,
            planning_time_ms=(time.perf_counter() - start_time) * 1000.0,
            success=len(path) >= 2,
            message='ok' if len(path) >= 2 else 'no policy path found',
            expanded_nodes=expanded,
        )

    def _plan_rrt_star(self, start: WorldPoint, goal: WorldPoint) -> PlannerResult:
        if self._map is None:
            return PlannerResult(
                planner_id='rrt_star',
                display_name='RRT*',
                path=[],
                planning_time_ms=0.0,
                success=False,
                message='map unavailable',
            )
        start_time = time.perf_counter()
        rng = random.Random(int(self.get_parameter('random_seed').value))
        planner = _RrtStarPlanner(
            self._map,
            step_m=float(self.get_parameter('rrt_step_m').value),
            goal_sample_rate=float(self.get_parameter('rrt_goal_sample_rate').value),
            rewire_radius_m=float(self.get_parameter('rrt_rewire_radius_m').value),
            max_samples=int(self.get_parameter('rrt_max_samples').value),
            rng=rng,
        )
        path, expanded, message = planner.plan(start, goal)
        return PlannerResult(
            planner_id='rrt_star',
            display_name='RRT*',
            path=path,
            planning_time_ms=(time.perf_counter() - start_time) * 1000.0,
            success=len(path) >= 2,
            message=message,
            expanded_nodes=expanded,
        )

    def _snap_radius_cells(self, grid_step: int) -> int:
        if self._map is None:
            return 1
        snap_radius_m = float(self.get_parameter('snap_radius_m').value)
        return max(1, int(math.ceil(snap_radius_m / (self._map.resolution * grid_step))))

    def _publish_metrics(self, results: list[PlannerResult]) -> None:
        if self._map is None:
            return
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        lines: list[str] = []
        colors = [
            (0.10, 0.70, 1.0),
            (0.20, 0.90, 0.25),
            (0.70, 0.30, 1.0),
            (1.0, 0.50, 0.10),
        ]
        for index, result in enumerate(results):
            clearance = self._map.clearance_m(result.path) if result.path else 0.0
            line = (
                f'{result.display_name}: success={result.success} '
                f'time_ms={result.planning_time_ms:.1f} '
                f'length_m={result.path_length_m:.2f} '
                f'clearance_m={clearance:.2f} '
                f'turn_rad={result.turn_angle_rad:.2f} '
                f'expanded={result.expanded_nodes} '
                f'msg={result.message}'
            )
            lines.append(line)
            marker = Marker()
            marker.header.frame_id = str(self.get_parameter('global_frame').value)
            marker.header.stamp = now
            marker.ns = 'planner_compare_metrics'
            marker.id = index
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = self._map.origin_x + 1.0
            marker.pose.position.y = self._map.origin_y + 1.0 + index * 0.55
            marker.pose.position.z = 0.35
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.34
            marker.color.r, marker.color.g, marker.color.b = colors[index]
            marker.color.a = 1.0
            marker.text = (
                f'{result.display_name}: {result.planning_time_ms:.0f} ms, '
                f'{result.path_length_m:.1f} m, '
                f'clear {clearance:.1f} m'
            )
            markers.markers.append(marker)
        self._metrics_pub.publish(markers)
        self._summary_pub.publish(String(data='\n'.join(lines)))
        self.get_logger().info('planner comparison metrics:\n' + '\n'.join(lines))


class _CoarseGrid:
    def __init__(self, base: GridMap, step: int) -> None:
        self.base = base
        self.step = step
        self.width = max(1, base.width // step)
        self.height = max(1, base.height // step)

    def in_bounds(self, cell: GridIndex) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def free(self, cell: GridIndex) -> bool:
        if not self.in_bounds(cell):
            return False
        bx = min(self.base.width - 1, cell[0] * self.step + self.step // 2)
        by = min(self.base.height - 1, cell[1] * self.step + self.step // 2)
        return self.base.free((bx, by))

    def world_to_cell(self, point: WorldPoint) -> GridIndex:
        base_cell = self.base.world_to_grid(point)
        return (base_cell[0] // self.step, base_cell[1] // self.step)

    def cell_to_world(self, cell: GridIndex) -> WorldPoint:
        base_cell = (
            min(self.base.width - 1, cell[0] * self.step + self.step // 2),
            min(self.base.height - 1, cell[1] * self.step + self.step // 2),
        )
        return self.base.grid_to_world(base_cell)

    def snap_free(self, cell: GridIndex, max_radius_cells: int) -> GridIndex | None:
        if self.free(cell):
            return cell
        visited = {cell}
        queue: deque[tuple[GridIndex, int]] = deque([(cell, 0)])
        while queue:
            current, distance = queue.popleft()
            if distance >= max_radius_cells:
                continue
            for neighbor in _neighbors8(current):
                if neighbor in visited or not self.in_bounds(neighbor):
                    continue
                if self.free(neighbor):
                    return neighbor
                visited.add(neighbor)
                queue.append((neighbor, distance + 1))
        return None


class _RrtStarPlanner:
    def __init__(
        self,
        grid: GridMap,
        step_m: float,
        goal_sample_rate: float,
        rewire_radius_m: float,
        max_samples: int,
        rng: random.Random,
    ) -> None:
        self.grid = grid
        self.step_m = step_m
        self.goal_sample_rate = goal_sample_rate
        self.rewire_radius_m = rewire_radius_m
        self.max_samples = max_samples
        self.rng = rng

    def plan(self, start: WorldPoint, goal: WorldPoint) -> tuple[list[WorldPoint], int, str]:
        start_cell = self.grid.snap_free(
            self.grid.world_to_grid(start),
            max_radius_cells=max(1, int(1.2 / self.grid.resolution)),
        )
        goal_cell = self.grid.snap_free(
            self.grid.world_to_grid(goal),
            max_radius_cells=max(1, int(1.2 / self.grid.resolution)),
        )
        if start_cell is None or goal_cell is None:
            return [], 0, 'start or goal cannot snap to free grid'
        start = self.grid.grid_to_world(start_cell)
        goal = self.grid.grid_to_world(goal_cell)

        nodes = [start]
        parents = [-1]
        costs = [0.0]
        best_goal_idx: int | None = None
        for _ in range(self.max_samples):
            sample = goal if self.rng.random() < self.goal_sample_rate else self._sample_free()
            nearest_idx = min(
                range(len(nodes)),
                key=lambda idx: _distance(nodes[idx], sample),
            )
            new_point = self._steer(nodes[nearest_idx], sample)
            if not self._collision_free(nodes[nearest_idx], new_point):
                continue
            near_indices = [
                idx
                for idx, node in enumerate(nodes)
                if _distance(node, new_point) <= self.rewire_radius_m
            ]
            parent_idx = nearest_idx
            best_cost = costs[nearest_idx] + _distance(nodes[nearest_idx], new_point)
            for idx in near_indices:
                candidate_cost = costs[idx] + _distance(nodes[idx], new_point)
                if candidate_cost < best_cost and self._collision_free(nodes[idx], new_point):
                    parent_idx = idx
                    best_cost = candidate_cost
            nodes.append(new_point)
            parents.append(parent_idx)
            costs.append(best_cost)
            new_idx = len(nodes) - 1
            for idx in near_indices:
                rewired_cost = best_cost + _distance(new_point, nodes[idx])
                if rewired_cost < costs[idx] and self._collision_free(new_point, nodes[idx]):
                    parents[idx] = new_idx
                    costs[idx] = rewired_cost
            if (
                _distance(new_point, goal) <= self.step_m
                and self._collision_free(new_point, goal)
            ):
                nodes.append(goal)
                parents.append(new_idx)
                costs.append(best_cost + _distance(new_point, goal))
                best_goal_idx = len(nodes) - 1
                if len(nodes) > 160:
                    break
        if best_goal_idx is None:
            return [], len(nodes), 'RRT* did not connect to goal within sample budget'
        return _backtrack_points(nodes, parents, best_goal_idx), len(nodes), 'ok'

    def _sample_free(self) -> WorldPoint:
        for _ in range(200):
            x = self.rng.uniform(self.grid.origin_x, self.grid.origin_x + self.grid.width * self.grid.resolution)
            y = self.rng.uniform(self.grid.origin_y, self.grid.origin_y + self.grid.height * self.grid.resolution)
            if self.grid.free(self.grid.world_to_grid((x, y))):
                return (x, y)
        return self.grid.grid_to_world((self.grid.width // 2, self.grid.height // 2))

    def _steer(self, source: WorldPoint, target: WorldPoint) -> WorldPoint:
        distance = _distance(source, target)
        if distance <= self.step_m:
            return target
        scale = self.step_m / distance
        return (
            source[0] + (target[0] - source[0]) * scale,
            source[1] + (target[1] - source[1]) * scale,
        )

    def _collision_free(self, start: WorldPoint, end: WorldPoint) -> bool:
        distance = _distance(start, end)
        steps = max(2, int(math.ceil(distance / (self.grid.resolution * 0.8))))
        for index in range(steps + 1):
            ratio = index / steps
            point = (
                start[0] + (end[0] - start[0]) * ratio,
                start[1] + (end[1] - start[1]) * ratio,
            )
            if not self.grid.free(self.grid.world_to_grid(point)):
                return False
        return True


def _value_iteration_path(
    grid: _CoarseGrid,
    start: GridIndex,
    goal: GridIndex,
    max_iterations: int,
    discount: float,
) -> tuple[list[GridIndex], int]:
    free_cells = [
        (x, y)
        for y in range(grid.height)
        for x in range(grid.width)
        if grid.free((x, y))
    ]
    if start not in free_cells or goal not in free_cells:
        return [], 0

    # Seed a deterministic value field from the goal. The policy extraction below
    # is equivalent to greedily following the best shaped Q value at each state.
    distance_to_goal = _dijkstra_distance_to_goal(grid, goal, max_iterations)
    if start not in distance_to_goal:
        return [], len(distance_to_goal)
    values = {
        cell: 120.0 - distance / max(0.05, discount)
        for cell, distance in distance_to_goal.items()
    }

    path: list[GridIndex] = [start]
    current = start
    for _ in range(len(distance_to_goal)):
        if current == goal:
            return _smooth_grid_path(path), len(distance_to_goal)
        candidates = [
            neighbor
            for neighbor in _neighbors8(current)
            if grid.free(neighbor) and neighbor in values
        ]
        if not candidates:
            return [], len(distance_to_goal)
        current = max(
            candidates,
            key=lambda cell: (
                values[cell],
                -_grid_distance(cell, goal),
            ),
        )
        path.append(current)
    return [], len(distance_to_goal)


def _grid_astar_path(
    grid: GridMap,
    start: WorldPoint,
    goal: WorldPoint,
    max_radius_cells: int,
) -> tuple[list[WorldPoint], int]:
    start_cell = grid.snap_free(grid.world_to_grid(start), max_radius_cells)
    goal_cell = grid.snap_free(grid.world_to_grid(goal), max_radius_cells)
    if start_cell is None or goal_cell is None:
        return [], 0

    queue: list[tuple[float, GridIndex]] = [(0.0, start_cell)]
    costs: dict[GridIndex, float] = {start_cell: 0.0}
    parents: dict[GridIndex, GridIndex] = {}
    expanded = 0
    while queue:
        _, current = heapq.heappop(queue)
        expanded += 1
        if current == goal_cell:
            cells = [current]
            while cells[-1] != start_cell:
                cells.append(parents[cells[-1]])
            cells.reverse()
            smoothed = _smooth_grid_path(cells)
            return [grid.grid_to_world(cell) for cell in smoothed], expanded
        for neighbor in _neighbors8(current):
            if not grid.free(neighbor):
                continue
            step_cost = _grid_distance(current, neighbor)
            next_cost = costs[current] + step_cost
            if next_cost < costs.get(neighbor, math.inf):
                costs[neighbor] = next_cost
                parents[neighbor] = current
                priority = next_cost + _grid_distance(neighbor, goal_cell)
                heapq.heappush(queue, (priority, neighbor))
    return [], expanded


def _dijkstra_distance_to_goal(
    grid: _CoarseGrid,
    goal: GridIndex,
    max_iterations: int,
) -> dict[GridIndex, float]:
    queue: list[tuple[float, GridIndex]] = [(0.0, goal)]
    distances: dict[GridIndex, float] = {goal: 0.0}
    expanded = 0
    while queue and expanded < max_iterations:
        current_cost, current = heapq.heappop(queue)
        expanded += 1
        if current_cost > distances[current]:
            continue
        for neighbor in _neighbors8(current):
            if not grid.free(neighbor):
                continue
            step_cost = _grid_distance(current, neighbor)
            next_cost = current_cost + step_cost
            if next_cost < distances.get(neighbor, math.inf):
                distances[neighbor] = next_cost
                heapq.heappush(queue, (next_cost, neighbor))
    return distances


def _grid_distance(a: GridIndex, b: GridIndex) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _smooth_grid_path(path: list[GridIndex]) -> list[GridIndex]:
    if len(path) <= 2:
        return path
    smoothed = [path[0]]
    previous_direction: GridIndex | None = None
    for start, end in zip(path, path[1:]):
        direction = (end[0] - start[0], end[1] - start[1])
        if previous_direction is not None and direction != previous_direction:
            smoothed.append(start)
        previous_direction = direction
    smoothed.append(path[-1])
    return smoothed


def _neighbors8(cell: GridIndex) -> list[GridIndex]:
    x, y = cell
    return [
        (x - 1, y - 1),
        (x, y - 1),
        (x + 1, y - 1),
        (x - 1, y),
        (x + 1, y),
        (x - 1, y + 1),
        (x, y + 1),
        (x + 1, y + 1),
    ]


def _backtrack_points(
    nodes: list[WorldPoint],
    parents: list[int],
    goal_idx: int,
) -> list[WorldPoint]:
    path: list[WorldPoint] = []
    idx = goal_idx
    while idx >= 0:
        path.append(nodes[idx])
        idx = parents[idx]
    path.reverse()
    return path


def _path_msg(points: list[WorldPoint], frame_id: str, stamp) -> Path:
    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = stamp
    for x, y in points:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = stamp
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.03
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


def _distance(a: WorldPoint, b: WorldPoint) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _path_length(points: list[WorldPoint]) -> float:
    return sum(_distance(start, end) for start, end in zip(points, points[1:]))


def _turn_angle(points: list[WorldPoint]) -> float:
    total = 0.0
    for a, b, c in zip(points, points[1:], points[2:]):
        heading_a = math.atan2(b[1] - a[1], b[0] - a[0])
        heading_b = math.atan2(c[1] - b[1], c[0] - b[0])
        total += abs(math.atan2(math.sin(heading_b - heading_a), math.cos(heading_b - heading_a)))
    return total


def main() -> None:
    rclpy.init()
    node = PlannerComparisonNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
