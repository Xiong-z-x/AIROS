from __future__ import annotations

import math
import struct
from pathlib import Path

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField

from airos_experiments.slam_scan_projector import project_cloud_to_scan


def _xyz_pointcloud(points: list[tuple[float, float, float]]) -> PointCloud2:
    msg = PointCloud2()
    msg.header.frame_id = 'map'
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = b''.join(struct.pack('<fff', *point) for point in points)
    return msg


def _odom(x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> Odometry:
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_footprint'
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


def _transform(x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> TransformStamped:
    msg = TransformStamped()
    msg.header.frame_id = 'map'
    msg.child_frame_id = 'base_footprint'
    msg.transform.translation.x = x
    msg.transform.translation.y = y
    msg.transform.rotation.z = math.sin(yaw / 2.0)
    msg.transform.rotation.w = math.cos(yaw / 2.0)
    return msg


def test_project_cloud_to_scan_uses_robot_relative_geometry() -> None:
    cloud = _xyz_pointcloud([
        (1.0, 0.0, 0.2),
        (3.0, 0.0, 0.2),
        (0.0, 2.0, 0.2),
    ])

    scan = project_cloud_to_scan(
        cloud,
        _odom(),
        frame_id='base_footprint',
        angle_min=-math.pi,
        angle_max=math.pi,
        angle_increment=math.pi / 4.0,
        range_min=0.05,
        range_max=5.0,
        min_z=-0.1,
        max_z=1.0,
    )

    forward_index = int(round((0.0 - scan.angle_min) / scan.angle_increment))
    left_index = int(round((math.pi / 2.0 - scan.angle_min) / scan.angle_increment))
    assert scan.header.frame_id == 'base_footprint'
    assert scan.ranges[forward_index] == 1.0
    assert scan.ranges[left_index] == 2.0


def test_project_cloud_to_scan_filters_floor_slope_and_high_ceiling_points() -> None:
    cloud = _xyz_pointcloud([
        (1.0, 0.0, -0.3),
        (1.5, 0.0, 0.2),
        (2.0, 0.0, 0.75),
        (0.5, 0.0, 2.5),
    ])

    scan = project_cloud_to_scan(
        cloud,
        _odom(),
        frame_id='base_footprint',
        angle_min=-math.pi / 2.0,
        angle_max=math.pi / 2.0,
        angle_increment=math.pi / 6.0,
        range_min=0.05,
        range_max=4.0,
        min_z=0.45,
        max_z=1.2,
    )

    forward_index = int(round((0.0 - scan.angle_min) / scan.angle_increment))
    assert scan.ranges[forward_index] == 2.0


def test_project_cloud_to_scan_uses_local_surface_when_odom_z_is_flat() -> None:
    cloud = _xyz_pointcloud([
        (0.0, 0.0, 1.00),
        (0.20, 0.10, 1.01),
        (-0.20, 0.10, 0.99),
        (1.00, 0.0, 1.00),
        (1.50, 0.0, 1.75),
    ])

    scan = project_cloud_to_scan(
        cloud,
        _odom(),
        frame_id='base_footprint',
        angle_min=-math.pi / 2.0,
        angle_max=math.pi / 2.0,
        angle_increment=math.pi / 6.0,
        range_min=0.05,
        range_max=4.0,
        min_z=0.45,
        max_z=1.2,
    )

    forward_index = int(round((0.0 - scan.angle_min) / scan.angle_increment))
    assert scan.ranges[forward_index] == 1.5


def test_project_cloud_to_scan_ignores_continuous_ramp_surface_points() -> None:
    cloud = _xyz_pointcloud([
        (0.00, 0.00, 0.00),
        (0.40, 0.00, 0.05),
        (0.80, 0.00, 0.12),
        (1.20, 0.00, 0.20),
        (1.60, 0.00, 0.30),
        (2.00, 0.00, 0.42),
        (2.40, 0.00, 0.56),
    ])

    scan = project_cloud_to_scan(
        cloud,
        _odom(),
        frame_id='base_footprint',
        angle_min=-math.pi / 2.0,
        angle_max=math.pi / 2.0,
        angle_increment=math.pi / 6.0,
        range_min=0.05,
        range_max=4.0,
        min_z=0.45,
        max_z=1.2,
    )

    forward_index = int(round((0.0 - scan.angle_min) / scan.angle_increment))
    assert math.isinf(scan.ranges[forward_index])


def test_project_cloud_to_scan_can_disable_supported_surface_filter() -> None:
    cloud = _xyz_pointcloud([
        (0.00, 0.00, 0.00),
        (0.40, 0.00, 0.05),
        (0.80, 0.00, 0.12),
        (1.20, 0.00, 0.20),
        (1.60, 0.00, 0.30),
        (2.00, 0.00, 0.42),
        (2.40, 0.00, 0.56),
    ])

    scan = project_cloud_to_scan(
        cloud,
        _odom(),
        frame_id='base_footprint',
        angle_min=-math.pi / 2.0,
        angle_max=math.pi / 2.0,
        angle_increment=math.pi / 6.0,
        range_min=0.05,
        range_max=4.0,
        min_z=0.45,
        max_z=1.2,
        filter_supported_surfaces=False,
    )

    forward_index = int(round((0.0 - scan.angle_min) / scan.angle_increment))
    assert math.isclose(scan.ranges[forward_index], 2.4, abs_tol=1e-6)


def test_project_cloud_to_scan_keeps_isolated_obstacle_above_ramp_surface() -> None:
    cloud = _xyz_pointcloud([
        (0.00, 0.00, 0.00),
        (0.40, 0.00, 0.05),
        (0.80, 0.00, 0.10),
        (2.00, 0.00, 0.70),
    ])

    scan = project_cloud_to_scan(
        cloud,
        _odom(),
        frame_id='base_footprint',
        angle_min=-math.pi / 2.0,
        angle_max=math.pi / 2.0,
        angle_increment=math.pi / 6.0,
        range_min=0.05,
        range_max=4.0,
        min_z=0.45,
        max_z=1.2,
    )

    forward_index = int(round((0.0 - scan.angle_min) / scan.angle_increment))
    assert scan.ranges[forward_index] == 2.0


def test_project_cloud_to_scan_uses_spatial_support_index_for_ramp_filter() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    source = (
        repo_root / 'src/airos_experiments/airos_experiments/slam_scan_projector.py'
    ).read_text(encoding='utf-8')
    projector = source.split('def project_cloud_to_scan(')[1].split(
        'def _build_lower_support_bins('
    )[0]
    support_check = source.split('def _is_supported_ramp_surface_point(')[1].split(
        'def _estimate_local_surface_z('
    )[0]

    assert 'support_bins = _build_lower_support_bins(points)' in projector
    assert 'support_bins=support_bins' in projector
    assert 'for other_x, other_y, other_z in points:' not in support_check


def test_project_cloud_to_scan_respects_odom_yaw() -> None:
    cloud = _xyz_pointcloud([
        (1.0, 1.0, 0.2),
    ])

    scan = project_cloud_to_scan(
        cloud,
        _odom(x=1.0, y=0.0, yaw=math.pi / 2.0),
        frame_id='base_footprint',
        angle_min=-math.pi,
        angle_max=math.pi,
        angle_increment=math.pi / 4.0,
        range_min=0.05,
        range_max=5.0,
        min_z=-0.1,
        max_z=1.0,
    )

    forward_index = int(round((0.0 - scan.angle_min) / scan.angle_increment))
    assert scan.ranges[forward_index] == 1.0


def test_project_cloud_to_scan_accepts_tf_pose_source() -> None:
    cloud = _xyz_pointcloud([
        (2.0, 1.0, 0.4),
    ])

    scan = project_cloud_to_scan(
        cloud,
        _transform(x=1.0, y=1.0, yaw=0.0),
        frame_id='base_footprint',
        angle_min=-math.pi,
        angle_max=math.pi,
        angle_increment=math.pi / 4.0,
        range_min=0.05,
        range_max=5.0,
        min_z=-0.1,
        max_z=1.0,
    )

    forward_index = int(round((0.0 - scan.angle_min) / scan.angle_increment))
    assert scan.ranges[forward_index] == 1.0


def test_slam_scan_projector_can_use_tf_pose_for_nav2_costmap_fusion() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    source = (
        repo_root / 'src/airos_experiments/airos_experiments/slam_scan_projector.py'
    ).read_text(encoding='utf-8')

    assert "self.declare_parameter('pose_source', 'odom')" in source
    assert "self.declare_parameter('map_frame', 'map')" in source
    assert "self.declare_parameter('base_frame', 'base_footprint')" in source
    assert "lookup_transform(" in source
    assert "pose_source must be 'odom' or 'tf'" in source


def test_slam_scan_projector_is_installed_as_console_script() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    setup_text = (repo_root / 'src/airos_experiments/setup.py').read_text(
        encoding='utf-8'
    )

    assert (
        'slam_scan_projector = airos_experiments.slam_scan_projector:main'
        in setup_text
    )
