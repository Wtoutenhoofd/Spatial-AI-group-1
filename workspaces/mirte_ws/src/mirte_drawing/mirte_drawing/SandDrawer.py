"""
sand_drawer.py
--------------
ROS 2 node that controls the arm/gripper to draw a pattern in sand.

State machine role
──────────────────
  Activates on : DRAW_PATTERN  (published on /robot_state)
  Completes on : publishes DONE on /state_change

Subscriptions
  /robot_state   (std_msgs/String)    – current pipeline state
  /pattern_data  (std_msgs/String)    – JSON pattern from Module 3:
                                        {"type": "text"|"shape",
                                         "points": [[x, y], ...]}

Publications
  /state_change  (std_msgs/String)    – publishes DONE when drawing is complete
  /mirte_master_arm_controller/joint_trajectory
                 (trajectory_msgs/JointTrajectory) – arm joint commands
"""

import json

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SandDrawer(Node):

    def __init__(self) -> None:
        super().__init__("sand_drawer")

        self.state_pub = self.create_publisher(String, "/state_change", 10)
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            "/mirte_master_arm_controller/joint_trajectory",
            10,
        )

        self.create_subscription(String, "/robot_state", self._state_callback, 10)
        self.create_subscription(String, "/pattern_data", self._pattern_callback, 10)

        self.active = False
        self.pattern = None

    def _state_callback(self, msg: String) -> None:
        if msg.data == "DRAW_PATTERN" and not self.active:
            self.active = True
            self.get_logger().info("DRAW_PATTERN – starting sand drawing")
            self._draw()
        elif msg.data != "DRAW_PATTERN":
            self.active = False

    def _pattern_callback(self, msg: String) -> None:
        try:
            self.pattern = json.loads(msg.data)
            self.get_logger().info(f"Pattern received: {self.pattern.get('type', '?')}")
        except Exception as e:
            self.get_logger().warn(f"Pattern parse error: {e}")

    def _draw(self) -> None:
        if self.pattern is None:
            self.get_logger().warn("No pattern data received yet – cannot draw")
            return

        points = self.pattern.get("points", [])
        self.get_logger().info(f"Drawing {len(points)} points")

        # TODO: implement arm trajectory for each point
        # Example: for x, y in points:
        #     self._move_arm_to(x, y)

        self._signal_done()

    def _move_arm_to(self, x: float, y: float) -> None:
        """Send a single joint trajectory command to position the gripper."""
        traj = JointTrajectory()
        traj.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_joint",
        ]

        point = JointTrajectoryPoint()
        # TODO: replace with real inverse kinematics for (x, y)
        point.positions = [0.0, 0.0, 0.0, 0.0]
        point.time_from_start.sec = 2

        traj.points.append(point)
        self.arm_pub.publish(traj)

    def _signal_done(self) -> None:
        msg = String()
        msg.data = "DONE"
        self.state_pub.publish(msg)
        self.active = False
        self.get_logger().info("Drawing complete – signalling DONE")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SandDrawer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
