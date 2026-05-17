from __future__ import annotations

from control_msgs.msg import DynamicJointState
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def joint_state_from_dynamic(msg: DynamicJointState) -> JointState:
    joint_state = JointState()
    joint_state.header = msg.header
    joint_state.name = list(msg.joint_names)

    for interface in msg.interface_values:
        values_by_name = {
            name: float(value)
            for name, value in zip(interface.interface_names, interface.values)
        }
        joint_state.position.append(values_by_name.get('position', 0.0))
        joint_state.velocity.append(values_by_name.get('velocity', 0.0))
        joint_state.effort.append(values_by_name.get('effort', 0.0))

    return joint_state


class DynamicJointStateRelay(Node):
    def __init__(self) -> None:
        super().__init__('dynamic_joint_state_relay')
        self.declare_parameter('input_topic', '/dynamic_joint_states')
        self.declare_parameter('output_topic', '/joint_states')

        self._publisher = self.create_publisher(
            JointState,
            str(self.get_parameter('output_topic').value),
            10,
        )
        self.create_subscription(
            DynamicJointState,
            str(self.get_parameter('input_topic').value),
            self._on_dynamic_joint_state,
            10,
        )
        self.get_logger().info(
            'dynamic joint state relay ready: '
            f'{self.get_parameter("input_topic").value} -> '
            f'{self.get_parameter("output_topic").value}'
        )

    def _on_dynamic_joint_state(self, msg: DynamicJointState) -> None:
        self._publisher.publish(joint_state_from_dynamic(msg))


def main() -> None:
    rclpy.init()
    node = DynamicJointStateRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
