import math

from airos_experiments.scan_emulator import (
    OdomAnchor,
    Pose2D,
    _map_pose_from_anchor,
)


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
