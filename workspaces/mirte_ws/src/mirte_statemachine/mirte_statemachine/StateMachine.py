"""
state_manager.py
----------------
ROS 2 node that owns and broadcasts the robot's top-level state.

Responsibilities
────────────────
  1. Raises the arm once at startup (allowing time for the arm controller
     to initialise) and then transitions to the first state in _NEXT_STATE.
  2. Publishes the current state on /robot_state at 2 Hz so all other
     nodes can always read the latest value.
  3. Listens on /state_change for transition requests from other nodes
     (e.g. VisionController signalling DONE).

Topics
──────
  Published
    /robot_state   (std_msgs/String) – current state, broadcast at 2 Hz
    /state_change  (std_msgs/String) – transition requests (this node also
                                       emits them to trigger itself)
  Subscribed
    /state_change  (std_msgs/String) – receives state transitions

State sequence
──────────────
  RAISE_ARM  →  (arm reaches position)  →  first key of _NEXT_STATE
            →  (DONE received)           →  next value in _NEXT_STATE
            →  …                         →  None  →  shutdown
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

ARM_RAISE_DURATION: int = 5
STARTUP_DELAY: float = 2.0

# Valid state transitions: maps each state to the state a DONE signal advances
# it to, or None if DONE means shut down.
# To change the task sequence, only edit this dict.
_NEXT_STATE: dict[str, str | None] = {
    "RAISE_ARM":        "TRACK_WHITEBOARD",
    "TRACK_WHITEBOARD": "TRACK_SANDPIT",
    "TRACK_SANDPIT":    None,   # DONE here → shutdown
}

# The first state entered after the arm raise – derived from the dict so it
# never needs to be updated separately.
_FIRST_STATE: str = next(iter(_NEXT_STATE))


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class StateManager(Node):

    # -- Lifecycle -----------------------------------------------------------

    def __init__(self) -> None:
        super().__init__("state_manager")

        # ── Publishers ───────────────────────────────────────────────────────
        self.state_pub = self.create_publisher(String, "/robot_state", 10)
        self.state_change_pub = self.create_publisher(String, "/state_change", 10)
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            "/mirte_master_arm_controller/joint_trajectory",
            10,
        )

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(String, "/state_change", self._state_change_callback, 10)

        # ── Internal state ───────────────────────────────────────────────────
        self.current_state: str = "RAISE_ARM"
        self._publish_state_change(self.current_state)

        # ── Timers ───────────────────────────────────────────────────────────
        self.create_timer(0.5, self._state_heartbeat_callback)
        self._start_timer = self.create_timer(STARTUP_DELAY, self._start_sequence)

    # -- State helpers --------------------------------------------------------

    def _publish_state_change(self, state: str) -> None:
        msg = String()
        msg.data = state
        self.state_change_pub.publish(msg)

    def _publish_current_state(self) -> None:
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)

    # -- Arm control ----------------------------------------------------------

    def _raise_arm(self) -> None:
        traj = JointTrajectory()
        traj.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_joint",
        ]
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, -1.57, 0.0]
        point.time_from_start.sec = ARM_RAISE_DURATION
        traj.points.append(point)
        self.arm_pub.publish(traj)
        self.get_logger().info("Arm raise command sent")

        self._arm_done_timer = self.create_timer(
            float(ARM_RAISE_DURATION),
            self._on_arm_raise_complete,
        )

    def _on_arm_raise_complete(self) -> None:
        self._arm_done_timer.cancel()
        # Read the first task state from the dict instead of hardcoding it
        self.current_state = _NEXT_STATE[_FIRST_STATE]
        msg = String()
        msg.data = self.current_state
        self.state_change_pub.publish(msg)
        self.get_logger().info(f"Arm raised – State -> {self.current_state}")

    # -- Callbacks ------------------------------------------------------------

    def _state_change_callback(self, msg: String) -> None:
        new_state = msg.data

        if new_state == "DONE":
            next_state = _NEXT_STATE.get(self.current_state)
            if next_state is None:
                self.get_logger().info(
                    f"DONE received in {self.current_state} – task complete, shutting down"
                )
                self._publish_current_state()
                raise SystemExit
            else:
                self.current_state = next_state
                self.get_logger().info(f"DONE received – State → {self.current_state}")
        else:
            self.current_state = new_state
            self.get_logger().info(f"State → {self.current_state}")

        self._publish_current_state()

    def _state_heartbeat_callback(self) -> None:
        self._publish_current_state()

    # -- Startup sequence -----------------------------------------------------

    def _start_sequence(self) -> None:
        self._start_timer.cancel()
        self.get_logger().info("Starting arm-raise sequence")
        self._raise_arm()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = StateManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()