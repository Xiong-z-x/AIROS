from __future__ import annotations

import math
from pathlib import Path

from nav_msgs.msg import Odometry

from airos_experiments.fast_lio_frame_alignment import (
    FrameAlignment,
    Pose2D,
    transform_point,
)
from airos_experiments.fast_lio_localization_bridge import (
    _aligned_odom_from_fast_lio,
    _compose_map_to_odom,
)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _read_text(relative_path: str) -> str:
    return (_repo_root() / relative_path).read_text(encoding='utf-8')


def test_spawn_alignment_transforms_fast_lio_local_points_to_world_map() -> None:
    alignment = FrameAlignment(
        spawn_x=0.0,
        spawn_y=-10.0,
        spawn_z=0.0,
        spawn_yaw=-math.pi / 2.0,
    )

    assert transform_point(0.0, 0.0, 0.0, alignment) == (0.0, -10.0, 0.0)

    x, y, z = transform_point(1.0, 0.0, 0.25, alignment)

    assert math.isclose(x, 0.0, abs_tol=1e-6)
    assert math.isclose(y, -11.0, abs_tol=1e-6)
    assert math.isclose(z, 0.25, abs_tol=1e-6)


def test_fast_lio_localization_bridge_offsets_map_to_spawn_pose() -> None:
    map_to_base = Pose2D(x=1.0, y=0.0, yaw=0.0)
    odom_to_base = Pose2D(x=0.0, y=-11.0, yaw=-math.pi / 2.0)

    map_to_odom = _compose_map_to_odom(
        map_to_base,
        odom_to_base,
        FrameAlignment(
            spawn_x=0.0,
            spawn_y=-10.0,
            spawn_z=0.0,
            spawn_yaw=-math.pi / 2.0,
        ),
    )

    assert math.isclose(map_to_odom.x, 0.0, abs_tol=1e-6)
    assert math.isclose(map_to_odom.y, 0.0, abs_tol=1e-6)
    assert math.isclose(map_to_odom.yaw, 0.0, abs_tol=1e-6)


def test_fast_lio_bridge_publishes_aligned_3d_pose_for_terrain_planner() -> None:
    fast_lio_odom = Odometry()
    fast_lio_odom.header.stamp.sec = 12
    fast_lio_odom.header.frame_id = 'fast_lio_map'
    fast_lio_odom.child_frame_id = 'fast_lio_body'
    fast_lio_odom.pose.pose.position.x = -23.0
    fast_lio_odom.pose.pose.position.y = 3.5
    fast_lio_odom.pose.pose.position.z = 1.75
    fast_lio_odom.pose.pose.orientation.z = math.sin(0.20 / 2.0)
    fast_lio_odom.pose.pose.orientation.w = math.cos(0.20 / 2.0)

    aligned = _aligned_odom_from_fast_lio(
        fast_lio_odom,
        alignment=FrameAlignment(
            spawn_x=0.0,
            spawn_y=-10.0,
            spawn_z=0.26,
            spawn_yaw=-math.pi / 2.0,
        ),
        map_frame='map',
        base_frame='base_footprint',
    )

    assert aligned.header.stamp.sec == 12
    assert aligned.header.frame_id == 'map'
    assert aligned.child_frame_id == 'base_footprint'
    assert math.isclose(aligned.pose.pose.position.x, 3.5, abs_tol=1e-6)
    assert math.isclose(aligned.pose.pose.position.y, 13.0, abs_tol=1e-6)
    assert math.isclose(aligned.pose.pose.position.z, 2.01, abs_tol=1e-6)


def test_fast_lio_visual_launch_aligns_local_map_before_planning() -> None:
    launch_text = _read_text(
        'src/airos_experiments/launch/visual_fast_lio_navigation.launch.py'
    )
    setup_py = _read_text('src/airos_experiments/setup.py')

    assert "executable='fast_lio_map_aligner'" in launch_text
    assert "'input_topic': '/Laser_map'" in launch_text
    assert "'output_topic': '/Laser_map_world'" in launch_text
    assert "'aligned_odom_topic': '/fast_lio_odom_world'" in launch_text
    assert "'odom_topic': '/fast_lio_odom_world'" in launch_text
    assert "'slam_map_topic': '/Laser_map_world'" in launch_text
    assert "'input_topic': '/Laser_map_world'" in launch_text
    assert "'spawn_x': LaunchConfiguration('robot_spawn_x')" in launch_text
    assert "'spawn_y': LaunchConfiguration('robot_spawn_y')" in launch_text
    assert "'spawn_yaw': LaunchConfiguration('robot_spawn_yaw')" in launch_text
    assert (
        'fast_lio_map_aligner = '
        'airos_experiments.fast_lio_map_aligner:main'
    ) in setup_py
