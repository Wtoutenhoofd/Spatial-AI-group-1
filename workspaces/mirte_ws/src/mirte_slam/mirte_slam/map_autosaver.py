#!/usr/bin/env python3
"""Periodically persist the slam_toolbox map to disk.

Calls the ``/slam_toolbox/save_map`` service on a fixed timer so a recent
occupancy-grid map (``<map_path>.pgm`` + ``<map_path>.yaml``) is always on disk,
even if the robot is powered off mid-run.

Parameters:
    map_path    (str)   Base path for the saved map (default ~/mirte_maps/map).
    save_period (float) Seconds between autosaves (default 10.0).
"""
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from slam_toolbox.srv import SaveMap


class MapAutosaver(Node):
    def __init__(self):
        super().__init__('map_autosaver')

        default_path = os.path.join(os.path.expanduser('~'), 'mirte_maps', 'map')
        self.declare_parameter('map_path', default_path)
        self.declare_parameter('save_period', 10.0)

        self.map_path = self.get_parameter('map_path').value
        period = self.get_parameter('save_period').value

        # save_map writes <map_path>.pgm/.yaml; make sure the directory exists.
        os.makedirs(os.path.dirname(self.map_path), exist_ok=True)

        self.client = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self.timer = self.create_timer(period, self._save)
        self.get_logger().info(
            f'Autosaving map every {period:g}s to {self.map_path}.pgm/.yaml'
        )

    def _save(self):
        if not self.client.service_is_ready():
            self.get_logger().warn(
                '/slam_toolbox/save_map not available yet; skipping this autosave'
            )
            return
        request = SaveMap.Request()
        request.name = String(data=self.map_path)
        self.client.call_async(request).add_done_callback(self._on_saved)

    def _on_saved(self, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Map autosave call failed: {exc}')
            return
        # SaveMap result: 0 == success; anything else means no map was written.
        result = getattr(response, 'result', 0)
        if result == 0:
            self.get_logger().info(f'Saved map to {self.map_path}.pgm/.yaml')
        else:
            self.get_logger().warn(
                f'save_map returned result={result} '
                '(no map yet? check /scan and the odom->base_link TF)'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MapAutosaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
