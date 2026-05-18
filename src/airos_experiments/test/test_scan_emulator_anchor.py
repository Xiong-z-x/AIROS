import math
from pathlib import Path

from airos_experiments.scan_emulator import (
    OdomAnchor,
    Pose2D,
    RectObstacle,
    _load_obstacles,
    _map_pose_from_anchor,
    _ray_rect_intersection,
)
from airos_experiments.world_map_generator import _is_occupied


def test_map_pose_from_anchor_translates_odom_delta() -> None:
    anchor = OdomAnchor(
        map_pose=Pose2D(2.0, 3.0, 0.0),
        odom_pose=Pose2D(10.0, 20.0, 0.0),
    )

    result = _map_pose_from_anchor(Pose2D(10.5, 19.5, 0.0), anchor)

    assert result.x == 2.5
    assert result.y == 2.5
    assert result.yaw == 0.0


def test_map_pose_from_anchor_rotates_odom_delta() -> None:
    anchor = OdomAnchor(
        map_pose=Pose2D(1.0, 1.0, math.pi / 2.0),
        odom_pose=Pose2D(0.0, 0.0, 0.0),
    )

    result = _map_pose_from_anchor(Pose2D(1.0, 0.0, 0.0), anchor)

    assert math.isclose(result.x, 1.0, abs_tol=1e-12)
    assert math.isclose(result.y, 2.0, abs_tol=1e-12)
    assert math.isclose(result.yaw, math.pi / 2.0, abs_tol=1e-12)


def test_scan_obstacle_loader_excludes_dynamic_models_by_default() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    world = repo_root / 'src/airos_sim/worlds/realistic_multilevel_ramp.sdf'

    default_obstacles = _load_obstacles(world)
    dynamic_obstacles = _load_obstacles(world, include_dynamic_models=True)

    assert len(default_obstacles) == 17
    assert len(dynamic_obstacles) == 19


def test_rotated_box_pose_is_used_by_scan_and_generated_map() -> None:
    obstacle = RectObstacle(0.0, 0.0, 2.0, 0.2, 1.0, math.pi / 4.0)

    assert _is_occupied(
        math.sqrt(0.5),
        math.sqrt(0.5),
        [obstacle],
        inflate=0.0,
    )
    assert not _is_occupied(
        math.sqrt(0.5),
        -math.sqrt(0.5),
        [obstacle],
        inflate=0.0,
    )

    hit = _ray_rect_intersection(
        -3.0,
        -3.0,
        math.sqrt(0.5),
        math.sqrt(0.5),
        obstacle,
    )
    miss = _ray_rect_intersection(
        -3.0,
        3.0,
        1.0,
        0.0,
        obstacle,
    )

    assert hit is not None
    assert miss is None
