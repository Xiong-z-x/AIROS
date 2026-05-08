from __future__ import annotations

from typing import Iterable

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node


class LifecycleActivator(Node):
    def __init__(self) -> None:
        super().__init__('lifecycle_activator')
        self.declare_parameter('node_names', ['/map_server'])
        self.declare_parameter('attempts', 8)
        self.declare_parameter('service_timeout_sec', 2.0)
        self.declare_parameter('poll_period_sec', 1.0)

        names = self.get_parameter('node_names').value
        self._node_names = [str(name) for name in names]
        self._attempts = int(self.get_parameter('attempts').value)
        self._service_timeout_sec = float(
            self.get_parameter('service_timeout_sec').value
        )
        self._poll_period_sec = float(
            self.get_parameter('poll_period_sec').value
        )
        self.finished = False

    def activate_all(self) -> None:
        for node_name in self._node_names:
            self._activate_node(node_name)
        self.finished = True

    def _activate_node(self, node_name: str) -> None:
        get_state = self.create_client(GetState, f'{node_name}/get_state')
        change_state = self.create_client(ChangeState, f'{node_name}/change_state')

        if not get_state.wait_for_service(timeout_sec=self._service_timeout_sec):
            self.get_logger().warn(f'{node_name}: get_state service unavailable')
            return
        if not change_state.wait_for_service(timeout_sec=self._service_timeout_sec):
            self.get_logger().warn(
                f'{node_name}: change_state service unavailable'
            )
            return

        for _ in range(max(self._attempts, 1)):
            state = self._get_state(get_state)
            if state is None:
                self._sleep_once()
                continue
            if state.id == State.PRIMARY_STATE_ACTIVE:
                self.get_logger().info(f'{node_name}: already active')
                return
            if state.id == State.PRIMARY_STATE_UNCONFIGURED:
                self._change_state(
                    change_state,
                    Transition.TRANSITION_CONFIGURE,
                    node_name,
                    'configure',
                )
            if state.id in (
                State.PRIMARY_STATE_UNCONFIGURED,
                State.PRIMARY_STATE_INACTIVE,
            ):
                self._change_state(
                    change_state,
                    Transition.TRANSITION_ACTIVATE,
                    node_name,
                    'activate',
                )
            self._sleep_once()

        state = self._get_state(get_state)
        if state is None or state.id != State.PRIMARY_STATE_ACTIVE:
            label = state.label if state is not None else 'unknown'
            self.get_logger().warn(f'{node_name}: final state is {label}')

    def _get_state(self, client) -> State | None:
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=self._service_timeout_sec,
        )
        if not future.done() or future.result() is None:
            return None
        return future.result().current_state

    def _change_state(
        self,
        client,
        transition_id: int,
        node_name: str,
        label: str,
    ) -> None:
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=self._service_timeout_sec,
        )
        if not future.done() or future.result() is None:
            self.get_logger().warn(f'{node_name}: {label} request timed out')
            return
        if future.result().success:
            self.get_logger().info(f'{node_name}: {label} transition accepted')
        else:
            self.get_logger().warn(f'{node_name}: {label} transition rejected')

    def _sleep_once(self) -> None:
        end_time = self.get_clock().now().nanoseconds + int(
            self._poll_period_sec * 1_000_000_000
        )
        while rclpy.ok() and self.get_clock().now().nanoseconds < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LifecycleActivator()
    try:
        node.activate_all()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
