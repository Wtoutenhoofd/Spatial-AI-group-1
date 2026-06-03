"""
sand_drawer.py
--------------
ROS 2 node that controls the arm/gripper to draw a pattern in sand.

State machine role
──────────────────
  Activates on : DRAW_PATTERN  (published on /robot_state)
  Completes on : publishes DONE on /state_change

Internal flow
──────────────
  IDLE
    │  DRAW_PATTERN received
    ▼
  PROBING  – arm moves down in small steps; monitors joint effort on
             /joint_states until resistance exceeds EFFORT_THRESHOLD,
             then records z_sand from the current joint angles
    │  contact detected
    ▼
  DRAWING  – sends all waypoints as a single JointTrajectory; a one-shot
             timer fires DONE after the estimated trajectory duration
    │  timer fires
    ▼
  IDLE  (publishes DONE on /state_change)

Subscriptions
  /robot_state   (std_msgs/String)      – current pipeline state
  /pattern_data  (std_msgs/String)      – JSON: {"type": "text"|"shape",
                                                  "points": [[x, y], ...]}
  /joint_states  (sensor_msgs/JointState) – effort feedback for probing

Publications
  /state_change  (std_msgs/String)      – DONE when drawing is complete
  /mirte_master_arm_controller/joint_trajectory
                 (trajectory_msgs/JointTrajectory) – arm commands
"""

import json
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ---------------------------------------------------------------------------
# Calibration constants  – measure on the real robot
# ---------------------------------------------------------------------------

L1: float = 0.10          # shoulder → elbow link length (m)
L2: float = 0.08          # elbow → pen tip link length (m)

PROBE_LIFT_START: float = 0.3    # shoulder_lift angle to begin probe (rad)
PROBE_LIFT_STEP:  float = 0.05   # how much to lower each step (rad)
PROBE_LIFT_MIN:   float = -0.8   # abort probe if this angle is reached
PROBE_INTERVAL:   float = 0.5    # seconds between probe steps

EFFORT_THRESHOLD: float = 0.5    # joint effort (Nm) that signals sand contact

DRAW_STEP_SEC: int = 2           # seconds per waypoint during drawing


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class SandDrawer(Node):

    def __init__(self) -> None:
        super().__init__("sand_drawer")

        # ── Publishers ───────────────────────────────────────────────────────
        self.state_pub = self.create_publisher(String, "/state_change", 10)
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            "/mirte_master_arm_controller/joint_trajectory",
            10,
        )

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(String, "/robot_state", self._state_callback, 10)
        self.create_subscription(String, "/pattern_data", self._pattern_callback, 10)
        self.create_subscription(JointState, "/joint_states", self._joint_state_callback, 10)

        # ── Internal state ───────────────────────────────────────────────────
        self._mode: str = "IDLE"       # IDLE | PROBING | DRAWING
        self.pattern: dict | None = None
        self.z_sand: float | None = None

        self._joint_positions: dict[str, float] = {}
        self._joint_efforts:   dict[str, float] = {}

        self._probe_lift: float = PROBE_LIFT_START
        self._probe_timer = None
        self._done_timer  = None

    # -- Subscriptions -------------------------------------------------------

    def _state_callback(self, msg: String) -> None:
        if msg.data == "DRAW_PATTERN" and self._mode == "IDLE":
            self.get_logger().info("DRAW_PATTERN – probing sand surface")
            self._mode = "PROBING"
            self._start_probe()
        elif msg.data != "DRAW_PATTERN" and self._mode != "IDLE":
            self._cancel_timers()
            self._mode = "IDLE"

    def _pattern_callback(self, msg: String) -> None:
        try:
            self.pattern = json.loads(msg.data)
            pts = len(self.pattern.get("points", []))
            self.get_logger().info(
                f"Pattern received: type={self.pattern.get('type','?')}  points={pts}"
            )
        except Exception as exc:
            self.get_logger().warn(f"Pattern parse error: {exc}")

    def _joint_state_callback(self, msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self._joint_positions[name] = msg.position[i]
            if i < len(msg.effort):
                self._joint_efforts[name] = msg.effort[i]

    # -- Probing -------------------------------------------------------------

    def _start_probe(self) -> None:
        self._probe_lift = PROBE_LIFT_START
        self._send_arm(pan=0.0, lift=self._probe_lift, elbow=0.0, duration=1)
        self._probe_timer = self.create_timer(PROBE_INTERVAL, self._probe_step)

    def _probe_step(self) -> None:
        if self._mode != "PROBING":
            self._probe_timer.cancel()
            return

        # Check effort on the two joints most likely to feel resistance
        effort = max(
            abs(self._joint_efforts.get("shoulder_lift_joint", 0.0)),
            abs(self._joint_efforts.get("elbow_joint", 0.0)),
        )

        if effort >= EFFORT_THRESHOLD:
            # Contact detected – derive z from current joint angles
            lift  = self._joint_positions.get("shoulder_lift_joint", self._probe_lift)
            elbow = self._joint_positions.get("elbow_joint", 0.0)
            self.z_sand = L1 * math.sin(lift) + L2 * math.sin(lift + elbow)

            self.get_logger().info(
                f"Sand contact: effort={effort:.2f} Nm  z_sand={self.z_sand:.3f} m"
            )
            self._probe_timer.cancel()
            self._mode = "DRAWING"
            self._draw()
            return

        # Lower the arm one step
        self._probe_lift -= PROBE_LIFT_STEP

        if self._probe_lift < PROBE_LIFT_MIN:
            self.get_logger().warn(
                "Probe reached minimum angle without contact – "
                "check EFFORT_THRESHOLD or robot/sand setup"
            )
            self._probe_timer.cancel()
            self._mode = "IDLE"
            return

        self.get_logger().info(
            f"Probing: lift={math.degrees(self._probe_lift):.1f}°  effort={effort:.2f}"
        )
        self._send_arm(pan=0.0, lift=self._probe_lift, elbow=0.0, duration=PROBE_INTERVAL)

    # -- Drawing -------------------------------------------------------------

    def _draw(self) -> None:
        if self.pattern is None:
            self.get_logger().warn("No pattern data – cannot draw")
            self._mode = "IDLE"
            return

        points = self.pattern.get("points", [])
        self.get_logger().info(
            f"Drawing {len(points)} points at z_sand={self.z_sand:.3f} m"
        )

        traj = JointTrajectory()
        traj.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_joint",
        ]

        t = DRAW_STEP_SEC
        for x, y in points:
            result = self._ik(x, y, self.z_sand)
            if result is None:
                self.get_logger().warn(f"Point ({x:.2f}, {y:.2f}) out of reach – skipping")
                continue

            pan, lift, elbow = result
            wp = JointTrajectoryPoint()
            wp.positions = [pan, lift, elbow, 0.0]
            wp.time_from_start.sec = t
            traj.points.append(wp)
            t += DRAW_STEP_SEC

        if not traj.points:
            self.get_logger().warn("No reachable points – nothing to draw")
            self._signal_done()
            return

        self.arm_pub.publish(traj)

        # Signal DONE after the full trajectory is estimated to complete
        self._done_timer = self.create_timer(float(t), self._on_draw_complete)

    def _on_draw_complete(self) -> None:
        self._done_timer.cancel()
        self._signal_done()

    # -- IK ------------------------------------------------------------------

    def _ik(self, x: float, y: float, z: float) -> tuple | None:
        """2-link analytical IK for shoulder_lift + elbow; shoulder_pan from atan2."""
        pan = math.atan2(y, x)
        r   = math.sqrt(x**2 + y**2)

        D = (r**2 + z**2 - L1**2 - L2**2) / (2 * L1 * L2)
        if abs(D) > 1.0:
            return None  # point out of reach

        elbow = math.atan2(-math.sqrt(1 - D**2), D)   # elbow-down solution
        lift  = math.atan2(z, r) - math.atan2(
            L2 * math.sin(elbow),
            L1 + L2 * math.cos(elbow),
        )
        return pan, lift, elbow

    # -- Helpers -------------------------------------------------------------

    def _send_arm(self, pan: float, lift: float, elbow: float, duration: float) -> None:
        traj = JointTrajectory()
        traj.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_joint",
        ]
        point = JointTrajectoryPoint()
        point.positions = [pan, lift, elbow, 0.0]
        point.time_from_start.sec = max(1, int(duration))
        traj.points.append(point)
        self.arm_pub.publish(traj)

    def _signal_done(self) -> None:
        msg = String()
        msg.data = "DONE"
        self.state_pub.publish(msg)
        self._mode = "IDLE"
        self.get_logger().info("Drawing complete – signalling DONE")

    def _cancel_timers(self) -> None:
        if self._probe_timer:
            self._probe_timer.cancel()
        if self._done_timer:
            self._done_timer.cancel()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

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
