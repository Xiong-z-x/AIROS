from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from tf2_ros import TransformBroadcaster

from airos_experiments.fast_lio_frame_alignment import (
    FrameAlignment,
    Pose2D,
    normalize_angle,
    transform_point,
    transform_pose,
)


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _pose2d_from_odom(msg: Odometry) -> Pose2D:
    pose = msg.pose.pose
    return Pose2D(
        float(pose.position.x),
        float(pose.position.y),
        _yaw_from_quaternion(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ),
    )


def _compose_map_to_odom(
    map_to_base: Pose2D,
    odom_to_base: Pose2D,
    alignment: FrameAlignment | None = None,
) -> Pose2D:
    if alignment is not None:
        map_to_base = transform_pose(map_to_base, alignment)
    yaw = normalize_angle(map_to_base.yaw - odom_to_base.yaw)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    odom_x_in_map = (
        map_to_base.x
        - cos_yaw * odom_to_base.x
        + sin_yaw * odom_to_base.y
    )
    odom_y_in_map = (
        map_to_base.y
        - sin_yaw * odom_to_base.x
        - cos_yaw * odom_to_base.y
    )
    return Pose2D(odom_x_in_map, odom_y_in_map, yaw)


def _aligned_odom_from_fast_lio(
    msg: Odometry,
    *,
    alignment: FrameAlignment,
    map_frame: str,
    base_frame: str,
) -> Odometry:
    pose = msg.pose.pose
    x, y, z = transform_point(
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
        alignment,
    )
    yaw = normalize_angle(
        alignment.spawn_yaw
        + _yaw_from_quaternion(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
    )
    qx, qy, qz, qw = _quaternion_from_yaw(yaw)

    aligned = Odometry()
    aligned.header.stamp = msg.header.stamp
    aligned.header.frame_id = map_frame
    aligned.child_frame_id = base_frame
    aligned.pose.pose.position.x = x
    aligned.pose.pose.position.y = y
    aligned.pose.pose.position.z = z
    aligned.pose.pose.orientation.x = qx
    aligned.pose.pose.orientation.y = qy
    aligned.pose.pose.orientation.z = qz
    aligned.pose.pose.orientation.w = qw
    aligned.pose.covariance = msg.pose.covariance
    aligned.twist = msg.twist
    return aligned


class FastLioLocalizationBridge(Node):
    def __init__(self) -> None:
        super().__init__('fast_lio_localization_bridge')

        self.declare_parameter('fast_lio_odom_topic', '/Odometry')
        self.declare_parameter('wheel_odom_topic', '/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('aligned_odom_topic', '/fast_lio_odom_world')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('max_source_age_sec', 0.8)
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('spawn_z', 0.0)
        self.declare_parameter('spawn_yaw', 0.0)

        self._map_frame = str(self.get_parameter('map_frame').value)
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._max_source_age_ns = int(
            float(self.get_parameter('max_source_age_sec').value)
            * 1_000_000_000
        )
        self._alignment = FrameAlignment(
            spawn_x=float(self.get_parameter('spawn_x').value),
            spawn_y=float(self.get_parameter('spawn_y').value),
            spawn_z=float(self.get_parameter('spawn_z').value),
            spawn_yaw=float(self.get_parameter('spawn_yaw').value),
        )

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._fast_lio_subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter('fast_lio_odom_topic').value),
            self._fast_lio_callback,
            qos,
        )
        self._wheel_odom_subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter('wheel_odom_topic').value),
            self._wheel_odom_callback,
            qos,
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self._aligned_odom_publisher = self.create_publisher(
            Odometry,
            str(self.get_parameter('aligned_odom_topic').value),
            qos,
        )
        self._fast_lio_odom: Optional[Odometry] = None
        self._wheel_odom: Optional[Odometry] = None
        period = 1.0 / max(
            float(self.get_parameter('publish_rate_hz').value),
            1.0,
        )
        self._timer = self.create_timer(period, self._publish_map_to_odom)
        self.get_logger().info(
            'fast_lio localization bridge ready: '
            f'{self.get_parameter("fast_lio_odom_topic").value} + '
            f'{self.get_parameter("wheel_odom_topic").value} -> '
            f'{self._map_frame}->{self._odom_frame}, '
            f'aligned odom={self.get_parameter("aligned_odom_topic").value}'
        )

    def destroy_node(self) -> bool:
        self.destroy_timer(self._timer)
        self.destroy_subscription(self._fast_lio_subscription)
        self.destroy_subscription(self._wheel_odom_subscription)
        self._tf_broadcaster = None
        return super().destroy_node()

    def _fast_lio_callback(self, msg: Odometry) -> None:
        self._fast_lio_odom = msg

    def _wheel_odom_callback(self, msg: Odometry) -> None:
        self._wheel_odom = msg

    def _stamp_age_ns(self, msg: Odometry) -> int:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        return self.get_clock().now().nanoseconds - stamp_ns

    def _sources_are_fresh(self) -> bool:
        if self._fast_lio_odom is None or self._wheel_odom is None:
            return False
        if self._max_source_age_ns <= 0:
            return True
        return (
            self._stamp_age_ns(self._fast_lio_odom) <= self._max_source_age_ns
            and self._stamp_age_ns(self._wheel_odom) <= self._max_source_age_ns
        )

    def _publish_map_to_odom(self) -> None:
        if not self._sources_are_fresh():
            return
        assert self._fast_lio_odom is not None
        assert self._wheel_odom is not None

        map_to_base = _pose2d_from_odom(self._fast_lio_odom)
        odom_to_base = _pose2d_from_odom(self._wheel_odom)
        map_to_odom = _compose_map_to_odom(
            map_to_base,
            odom_to_base,
            self._alignment,
        )
        qx, qy, qz, qw = _quaternion_from_yaw(map_to_odom.yaw)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._map_frame
        transform.child_frame_id = self._odom_frame
        transform.transform.translation.x = map_to_odom.x
        transform.transform.translation.y = map_to_odom.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(transform)
        self._aligned_odom_publisher.publish(
            _aligned_odom_from_fast_lio(
                self._fast_lio_odom,
                alignment=self._alignment,
                map_frame=self._map_frame,
                base_frame=self._base_frame,
            )
        )


def main() -> None:
    rclpy.init()
    node = FastLioLocalizationBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
