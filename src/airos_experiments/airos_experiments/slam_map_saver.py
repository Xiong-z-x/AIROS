from __future__ import annotations

import argparse
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from std_msgs.msg import String


class SlamMapSaver(Node):
    def __init__(self) -> None:
        super().__init__('slam_map_saver')
        self._save_map = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self._serialize = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')

    def save(self, map_prefix: Path, posegraph: Path, timeout_sec: float) -> bool:
        map_prefix.parent.mkdir(parents=True, exist_ok=True)
        if not self._wait_for_services(timeout_sec):
            return False

        save_request = SaveMap.Request()
        save_request.name = String(data=str(map_prefix))
        save_future = self._save_map.call_async(save_request)
        rclpy.spin_until_future_complete(self, save_future, timeout_sec=timeout_sec)
        save_response = save_future.result()
        if save_response is None or save_response.result != SaveMap.Response.RESULT_SUCCESS:
            self.get_logger().error('slam_toolbox save_map failed')
            return False

        serialize_request = SerializePoseGraph.Request()
        serialize_request.filename = str(posegraph)
        serialize_future = self._serialize.call_async(serialize_request)
        rclpy.spin_until_future_complete(self, serialize_future, timeout_sec=timeout_sec)
        serialize_response = serialize_future.result()
        if (
            serialize_response is None
            or serialize_response.result != SerializePoseGraph.Response.RESULT_SUCCESS
        ):
            self.get_logger().error('slam_toolbox serialize_map failed')
            return False

        self.get_logger().info(f'saved map={map_prefix} posegraph={posegraph}')
        return True

    def _wait_for_services(self, timeout_sec: float) -> bool:
        ok = True
        for client, name in (
            (self._save_map, '/slam_toolbox/save_map'),
            (self._serialize, '/slam_toolbox/serialize_map'),
        ):
            if not client.wait_for_service(timeout_sec=timeout_sec):
                self.get_logger().error(f'service unavailable: {name}')
                ok = False
        return ok


def main() -> None:
    parser = argparse.ArgumentParser(description='Save slam_toolbox map and pose graph.')
    parser.add_argument('--prefix', default='src/airos_nav/maps/single_floor_lab')
    parser.add_argument('--posegraph', default='src/airos_nav/maps/single_floor_lab')
    parser.add_argument('--timeout-sec', type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = SlamMapSaver()
    try:
        ok = node.save(Path(args.prefix), Path(args.posegraph), args.timeout_sec)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
