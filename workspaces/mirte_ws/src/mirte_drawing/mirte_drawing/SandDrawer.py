"""
sand_drawer.py
--------------
ROS 2 node that draws text in sand by controlling the robot arm.

Flow
────
  IDLE
    │  DRAW_PATTERN received on /robot_state
    ▼
  PROBING  – arm steps down until joint effort exceeds EFFORT_THRESHOLD;
             records z_sand from current joint angles
    │  contact detected
    ▼
  DRAWING  – converts text from /whiteboard_text to arm waypoints using a
             built-in single-stroke font; sends full trajectory at once;
             pen lifts between letter strokes
    │  trajectory timer fires
    ▼
  IDLE  (publishes DONE on /state_change)

Subscriptions
  /robot_state     (std_msgs/String)       – pipeline state gate
  /whiteboard_text (std_msgs/String)       – JSON: {"text": "HELLO", ...}
  /joint_states    (sensor_msgs/JointState) – effort feedback for probing

Publications
  /state_change    (std_msgs/String)       – DONE when drawing complete
  /mirte_master_arm_controller/joint_trajectory
                   (trajectory_msgs/JointTrajectory)

Calibration
───────────
  L1, L2            – arm link lengths; measure on the real robot
  EFFORT_THRESHOLD  – press arm gently on a surface and read /joint_states
  DRAW_X_CENTER     – how far in front of the robot to draw (m)
"""

import json
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ---------------------------------------------------------------------------
# Calibration – adjust for the real robot
# ---------------------------------------------------------------------------

L1: float = 0.1378            # shoulder_lift → elbow link length (m)
L2: float = 0.1427            # elbow → wrist link length (m)

EFFORT_THRESHOLD: float = 0.5 # joint effort (Nm) that signals sand contact

# Set to True to skip probing and draw at a fixed z height (for testing)
SKIP_PROBING: bool = True
Z_SAND_FIXED: float = 0.05    # fixed z height when SKIP_PROBING is True (m)

PROBE_LIFT_START: float = 0.3  # shoulder_lift angle to start probing (rad)
PROBE_LIFT_STEP:  float = 0.05 # how much to lower each probe step (rad)
PROBE_LIFT_MIN:   float = -0.8 # abort if probe reaches this angle
PROBE_INTERVAL:   float = 0.5  # seconds between probe steps

# ---------------------------------------------------------------------------
# Drawing layout
# ---------------------------------------------------------------------------

DRAW_X_CENTER:  float = 0.20  # center x of drawing area (m in front of robot)
LETTER_WIDTH:   float = 0.075 # width of one letter in sand (m)
LETTER_HEIGHT:  float = 0.120 # height of one letter in sand (m)
LETTER_GAP:     float = 0.025 # gap between letters (m)
PEN_LIFT:       float = 0.020 # how much to raise pen between strokes (m)
DRAW_STEP_SEC:  int   = 2     # seconds per waypoint

# ---------------------------------------------------------------------------
# Single-stroke font
# ---------------------------------------------------------------------------
# Each letter is a list of strokes.
# Each stroke is a list of (x, y) points, normalised to [0-1] × [0-1].
#   x = 0 → left edge,   x = 1 → right edge  (maps to robot y-axis)
#   y = 0 → bottom,      y = 1 → top          (maps to robot x-axis)

STROKES: dict[str, list[list[tuple[float, float]]]] = {
    'A': [[(0.0,0.0),(0.5,1.0),(1.0,0.0)],
          [(0.2,0.4),(0.8,0.4)]],
    'B': [[(0.0,0.0),(0.0,1.0),(0.7,1.0),(0.9,0.85),(0.7,0.5),(0.0,0.5)],
          [(0.0,0.5),(0.7,0.5),(0.9,0.35),(0.7,0.0),(0.0,0.0)]],
    'C': [[(0.9,0.85),(0.7,1.0),(0.3,1.0),(0.0,0.75),(0.0,0.25),(0.3,0.0),(0.7,0.0),(0.9,0.15)]],
    'D': [[(0.0,0.0),(0.0,1.0),(0.6,1.0),(0.9,0.75),(0.9,0.25),(0.6,0.0),(0.0,0.0)]],
    'E': [[(1.0,1.0),(0.0,1.0),(0.0,0.0),(1.0,0.0)],
          [(0.0,0.5),(0.7,0.5)]],
    'F': [[(0.0,0.0),(0.0,1.0),(1.0,1.0)],
          [(0.0,0.5),(0.7,0.5)]],
    'G': [[(0.9,0.85),(0.7,1.0),(0.3,1.0),(0.0,0.75),(0.0,0.25),(0.3,0.0),(0.7,0.0),(0.9,0.25),(0.9,0.5),(0.5,0.5)]],
    'H': [[(0.0,0.0),(0.0,1.0)],
          [(0.0,0.5),(1.0,0.5)],
          [(1.0,1.0),(1.0,0.0)]],
    'I': [[(0.2,1.0),(0.8,1.0)],
          [(0.5,1.0),(0.5,0.0)],
          [(0.2,0.0),(0.8,0.0)]],
    'J': [[(0.2,0.25),(0.2,0.0),(0.8,0.0),(0.8,1.0)]],
    'K': [[(0.0,0.0),(0.0,1.0)],
          [(1.0,1.0),(0.0,0.5),(1.0,0.0)]],
    'L': [[(0.0,1.0),(0.0,0.0),(1.0,0.0)]],
    'M': [[(0.0,0.0),(0.0,1.0),(0.5,0.5),(1.0,1.0),(1.0,0.0)]],
    'N': [[(0.0,0.0),(0.0,1.0),(1.0,0.0),(1.0,1.0)]],
    'O': [[(0.3,0.0),(0.7,0.0),(0.9,0.25),(0.9,0.75),(0.7,1.0),(0.3,1.0),(0.1,0.75),(0.1,0.25),(0.3,0.0)]],
    'P': [[(0.0,0.0),(0.0,1.0),(0.7,1.0),(0.9,0.85),(0.9,0.65),(0.7,0.5),(0.0,0.5)]],
    'Q': [[(0.3,0.0),(0.7,0.0),(0.9,0.25),(0.9,0.75),(0.7,1.0),(0.3,1.0),(0.1,0.75),(0.1,0.25),(0.3,0.0)],
          [(0.6,0.2),(1.0,0.0)]],
    'R': [[(0.0,0.0),(0.0,1.0),(0.7,1.0),(0.9,0.85),(0.9,0.65),(0.7,0.5),(0.0,0.5)],
          [(0.5,0.5),(1.0,0.0)]],
    'S': [[(0.9,0.85),(0.7,1.0),(0.3,1.0),(0.1,0.85),(0.1,0.65),(0.5,0.5),(0.9,0.35),(0.9,0.15),(0.7,0.0),(0.3,0.0),(0.1,0.15)]],
    'T': [[(0.0,1.0),(1.0,1.0)],
          [(0.5,1.0),(0.5,0.0)]],
    'U': [[(0.0,1.0),(0.0,0.25),(0.3,0.0),(0.7,0.0),(1.0,0.25),(1.0,1.0)]],
    'V': [[(0.0,1.0),(0.5,0.0),(1.0,1.0)]],
    'W': [[(0.0,1.0),(0.25,0.0),(0.5,0.5),(0.75,0.0),(1.0,1.0)]],
    'X': [[(0.0,1.0),(1.0,0.0)],
          [(1.0,1.0),(0.0,0.0)]],
    'Y': [[(0.0,1.0),(0.5,0.5),(1.0,1.0)],
          [(0.5,0.5),(0.5,0.0)]],
    'Z': [[(0.0,1.0),(1.0,1.0),(0.0,0.0),(1.0,0.0)]],
    ' ': [],
}


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class SandDrawer(Node):

    def __init__(self) -> None:
        super().__init__("sand_drawer")

        # ── Publishers ───────────────────────────────────────────────────────
        self.state_pub = self.create_publisher(String, "/state_change", 10)
        self.arm_pub   = self.create_publisher(
            JointTrajectory,
            "/mirte_master_arm_controller/joint_trajectory",
            10,
        )

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(String,     "/robot_state",     self._state_callback,      10)
        self.create_subscription(String,     "/whiteboard_text", self._text_callback,        10)
        self.create_subscription(JointState, "/joint_states",    self._joint_state_callback, 10)

        # ── Internal state ───────────────────────────────────────────────────
        self._mode: str          = "IDLE"
        self._text: str          = ""
        self.z_sand: float | None = None

        self._joint_positions: dict[str, float] = {}
        self._joint_efforts:   dict[str, float] = {}

        self._probe_lift: float = PROBE_LIFT_START
        self._probe_timer = None
        self._done_timer  = None

    # -- Subscriptions -------------------------------------------------------

    def _state_callback(self, msg: String) -> None:
        if msg.data == "DRAW_PATTERN" and self._mode == "IDLE":
            if not self._text:
                self.get_logger().warn("No text received yet – waiting for /whiteboard_text")
                return
            if SKIP_PROBING:
                self.z_sand = Z_SAND_FIXED
                self.get_logger().info(
                    f"DRAW_PATTERN – skipping probe, z_sand={self.z_sand} m, "
                    f"will draw: {self._text!r}"
                )
                self._mode = "DRAWING"
                self._draw()
            else:
                self.get_logger().info(f"DRAW_PATTERN – probing sand, will draw: {self._text!r}")
                self._mode = "PROBING"
                self._start_probe()
        elif msg.data != "DRAW_PATTERN" and self._mode != "IDLE":
            self._cancel_timers()
            self._mode = "IDLE"

    def _text_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._text = data.get("text", "").upper().strip()
            self.get_logger().info(f"Text to draw: {self._text!r}")
        except Exception as exc:
            self.get_logger().warn(f"Text parse error: {exc}")

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

        effort = max(
            abs(self._joint_efforts.get("shoulder_lift_joint", 0.0)),
            abs(self._joint_efforts.get("elbow_joint", 0.0)),
        )

        if effort >= EFFORT_THRESHOLD:
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

        self._probe_lift -= PROBE_LIFT_STEP
        if self._probe_lift < PROBE_LIFT_MIN:
            self.get_logger().warn("Probe reached minimum angle without contact")
            self._probe_timer.cancel()
            self._mode = "IDLE"
            return

        self.get_logger().info(
            f"Probing: lift={math.degrees(self._probe_lift):.1f}°  effort={effort:.2f}"
        )
        self._send_arm(pan=0.0, lift=self._probe_lift, elbow=0.0, duration=PROBE_INTERVAL)

    # -- Drawing -------------------------------------------------------------

    def _draw(self) -> None:
        waypoints = self._text_to_waypoints(self._text)
        if not waypoints:
            self.get_logger().warn("No drawable characters in text")
            self._signal_done()
            return

        traj = JointTrajectory()
        traj.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint",
            "elbow_joint",        "wrist_joint",
        ]

        t = DRAW_STEP_SEC
        skipped = 0
        for x, y, z in waypoints:
            result = self._ik(x, y, z)
            if result is None:
                skipped += 1
                continue
            pan, lift, elbow = result
            wp = JointTrajectoryPoint()
            wp.positions = [pan, lift, elbow, 0.0]
            wp.time_from_start.sec = t
            traj.points.append(wp)
            t += DRAW_STEP_SEC

        if skipped:
            self.get_logger().warn(f"{skipped} waypoints out of reach – skipped")

        if not traj.points:
            self.get_logger().warn("All waypoints out of reach")
            self._signal_done()
            return

        self.get_logger().info(
            f"Drawing {len(traj.points)} waypoints  "
            f"(~{t} s)  text={self._text!r}"
        )
        self.arm_pub.publish(traj)
        self._done_timer = self.create_timer(float(t), self._on_draw_complete)

    def _text_to_waypoints(self, text: str) -> list[tuple[float, float, float]]:
        """Convert text to (x, y, z) robot-frame coordinates with pen-up moves."""
        n = len(text)
        total_y = n * LETTER_WIDTH + max(0, n - 1) * LETTER_GAP
        y_start = -total_y / 2  # centre the text on the y-axis

        waypoints: list[tuple[float, float, float]] = []

        for i, ch in enumerate(text):
            strokes = STROKES.get(ch, [])
            y_letter = y_start + i * (LETTER_WIDTH + LETTER_GAP)

            for stroke in strokes:
                # Pen up: move to first point of stroke at lift height
                lx, ly = stroke[0]
                wx, wy = self._letter_to_robot(lx, ly, y_letter)
                waypoints.append((wx, wy, self.z_sand + PEN_LIFT))

                # Pen down: draw each point in the stroke
                for lx, ly in stroke:
                    wx, wy = self._letter_to_robot(lx, ly, y_letter)
                    waypoints.append((wx, wy, self.z_sand))

        return waypoints

    def _letter_to_robot(
        self, lx: float, ly: float, y_offset: float
    ) -> tuple[float, float]:
        """
        Map normalised letter coordinates to robot (x, y).
          lx [0-1]: horizontal in letter  → robot y-axis (lateral)
          ly [0-1]: vertical in letter    → robot x-axis (forward)
        """
        x = DRAW_X_CENTER + (ly - 0.5) * LETTER_HEIGHT
        y = y_offset + lx * LETTER_WIDTH
        return x, y

    def _on_draw_complete(self) -> None:
        self._done_timer.cancel()
        self._signal_done()

    # -- IK ------------------------------------------------------------------

    def _ik(self, x: float, y: float, z: float) -> tuple | None:
        """Analytical 2-link IK; shoulder_pan from atan2."""
        pan = math.atan2(y, x)
        r   = math.sqrt(x**2 + y**2)

        D = (r**2 + z**2 - L1**2 - L2**2) / (2 * L1 * L2)
        if abs(D) > 1.0:
            return None

        elbow = math.atan2(-math.sqrt(1 - D**2), D)
        lift  = math.atan2(z, r) - math.atan2(
            L2 * math.sin(elbow),
            L1 + L2 * math.cos(elbow),
        )
        return -pan, -lift - math.pi / 3, elbow

    # -- Helpers -------------------------------------------------------------

    def _send_arm(self, pan: float, lift: float, elbow: float, duration: float) -> None:
        traj = JointTrajectory()
        traj.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint",
            "elbow_joint",        "wrist_joint",
        ]
        pt = JointTrajectoryPoint()
        pt.positions = [pan, lift, elbow, 0.0]
        pt.time_from_start.sec = max(1, int(duration))
        traj.points.append(pt)
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
