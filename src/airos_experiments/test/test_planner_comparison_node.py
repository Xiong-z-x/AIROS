from __future__ import annotations

from pathlib import Path

from nav_msgs.msg import OccupancyGrid

from airos_experiments.planner_comparison_node import (
    GridMap,
    _RrtStarPlanner,
    _grid_astar_path,
    _value_iteration_path,
    _CoarseGrid,
)


def _open_grid() -> GridMap:
    msg = OccupancyGrid()
    msg.info.width = 20
    msg.info.height = 20
    msg.info.resolution = 0.2
    msg.info.origin.position.x = -2.0
    msg.info.origin.position.y = -2.0
    msg.data = [0] * (msg.info.width * msg.info.height)
    return GridMap.from_msg(msg, occupied_threshold=65, robot_radius_m=0.01)


def test_value_iteration_q_path_reaches_goal_on_open_grid():
    grid = _open_grid()
    coarse = _CoarseGrid(grid, step=1)

    path, expanded = _value_iteration_path(
        coarse,
        start=(2, 2),
        goal=(17, 17),
        max_iterations=3000,
        discount=0.96,
    )

    assert path[0] == (2, 2)
    assert path[-1] == (17, 17)
    assert len(path) >= 2
    assert expanded > 0


def test_rrt_star_returns_collision_free_path_on_open_grid():
    grid = _open_grid()
    planner = _RrtStarPlanner(
        grid,
        step_m=0.5,
        goal_sample_rate=0.35,
        rewire_radius_m=0.8,
        max_samples=800,
        rng=__import__('random').Random(4),
    )

    path, expanded, message = planner.plan((-1.4, -1.4), (1.4, 1.4))

    assert message == 'ok'
    assert len(path) >= 2
    assert expanded > 0
    assert path[0] != path[-1]
    assert all(grid.free(grid.world_to_grid(point)) for point in path)


def test_smac_style_astar_fallback_reaches_goal_on_open_grid():
    grid = _open_grid()

    path, expanded = _grid_astar_path(
        grid,
        start=(-1.4, -1.4),
        goal=(1.4, 1.4),
        max_radius_cells=5,
    )

    assert len(path) >= 2
    assert expanded > 0
    assert all(grid.free(grid.world_to_grid(point)) for point in path)


def test_planner_comparison_artifacts_are_present():
    assert Path('src/airos_experiments/launch/planner_comparison.launch.py').is_file()
    assert Path('src/airos_nav/config/nav2_planner_comparison.yaml').is_file()
    assert Path('src/airos_sim/worlds/single_floor_planner_showcase.sdf').is_file()
    assert Path('src/airos_nav/maps/single_floor_planner_showcase.yaml').is_file()


def test_planner_only_launch_does_not_start_motion_servers():
    nav_launch = Path('src/airos_nav/launch/nav.launch.py').read_text(encoding='utf-8')
    comparison_launch = Path(
        'src/airos_experiments/launch/planner_comparison.launch.py'
    ).read_text(encoding='utf-8')

    assert "' in ('full', 'planner_only')" in nav_launch
    assert "' in ('full', 'controller_only')" in nav_launch
    assert "nav_stack_mode': 'planner_only'" in comparison_launch
    assert 'single_floor_planner_showcase.yaml' in comparison_launch
    assert 'planner_comparison_node' in comparison_launch


def test_planner_comparison_accepts_rviz_goal_topic_and_nav2_action_without_cmd_vel():
    source = Path(
        'src/airos_experiments/airos_experiments/planner_comparison_node.py'
    ).read_text(encoding='utf-8')

    assert "self.declare_parameter('goal_topic', '/goal_pose')" in source
    assert 'NavigateToPose' in source
    assert "'navigate_to_pose'" in source
    assert 'self._start_planner_comparison(' in source
    assert 'create_publisher(Twist' not in source
    assert "'/cmd_vel'" not in source
