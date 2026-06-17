#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan


FRONT_CONE_HALF_ANGLE = 0.15  # radians (~8.6 degrees)


class FrontScanFilter(Node):

    def __init__(self):
        super().__init__("front_scan_filter")

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

        self.scan_pub = self.create_publisher(
            LaserScan,
            "/front_scan",
            10
        )

        self.get_logger().info(
            f"Publishing filtered scan ±{math.degrees(FRONT_CONE_HALF_ANGLE):.1f}° on /front_scan"
        )

    def scan_callback(self, msg: LaserScan):

        filtered = LaserScan()

        # Copy metadata
        filtered.header = msg.header
        filtered.angle_min = -FRONT_CONE_HALF_ANGLE
        filtered.angle_max = FRONT_CONE_HALF_ANGLE
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max

        angle = msg.angle_min

        for r in msg.ranges:

            if -FRONT_CONE_HALF_ANGLE <= angle <= FRONT_CONE_HALF_ANGLE:
                filtered.ranges.append(r)

            angle += msg.angle_increment

        self.scan_pub.publish(filtered)


def main(args=None):
    rclpy.init(args=args)

    node = FrontScanFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()