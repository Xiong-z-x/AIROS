from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class FrameAlignment:
    spawn_x: float = 0.0
    spawn_y: float = 0.0
    spawn_z: float = 0.0
    spawn_yaw: float = 0.0


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def transform_point(
    x: float,
    y: float,
    z: float,
    alignment: FrameAlignment,
) -> tuple[float, float, float]:
    cos_yaw = math.cos(alignment.spawn_yaw)
    sin_yaw = math.sin(alignment.spawn_yaw)
    return (
        alignment.spawn_x + cos_yaw * x - sin_yaw * y,
        alignment.spawn_y + sin_yaw * x + cos_yaw * y,
        alignment.spawn_z + z,
    )


def transform_pose(
    pose: Pose2D,
    alignment: FrameAlignment,
) -> Pose2D:
    x, y, _ = transform_point(pose.x, pose.y, 0.0, alignment)
    return Pose2D(
        x=x,
        y=y,
        yaw=normalize_angle(alignment.spawn_yaw + pose.yaw),
    )

