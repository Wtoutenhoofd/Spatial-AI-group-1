"""
sand_drawer.py
--------------
ROS 2 node that draws text in sand by controlling the Mirte Master arm.

Flow
────
  IDLE
    │  DRAW_PATTERN received on /robot_state
    ▼
  PROBING  – arm steps down until joint effort exceeds EFFORT_THRESHOLD;
             records wrist height from current joint angles
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

Kinematics (verified against the real Mirte Master URDF)
────────────────────────────────────────────────────────
  Chain: frame_link → shoulder_pan → shoulder_lift → elbow → wrist
  All arm joints are limited to ±90° (pi/2). This is the single most
  important constraint: any IK solution outside that range is rejected by
  the controller and the arm freezes in a wrong pose (e.g. up in the air).

  shoulder_lift = 0  → first link points straight UP (+z).
  Angles are measured FROM VERTICAL, so:
      horizontal reach = L*sin(angle)   vertical = L*cos(angle)

  The shoulder pivot sits SHOULDER_Z (=0.0881 m) above frame_link, while
  the ground is at GROUND_Z (=-0.0955 m). Because of the ±90° limits the
  WRIST cannot descend below z≈-0.054 m, i.e. it stays ~4 cm above the
  ground. The pen/gripper mounted below the wrist bridges that last gap,
  so we keep the wrist at WRIST_DRAW_HEIGHT and let the pen touch the sand.

  Net effect: the drawable area is a small band roughly 0.095–0.155 m in
  front of the robot. Keep the letters small (see layout constants below).

Calibration – adjust for your robot/sim
  WRIST_DRAW_HEIGHT  – wrist z (frame_link) while drawing; lower it if the
                       pen does not reach the sand, raise it if it digs in
  WRIST_DRAW_ANGLE   – wrist_joint angle that points the pen down
  PEN_LIFT           – how much to raise the wrist between strokes
  L1, L2             – link lengths (match the URDF: 0.1378 / 0.14265)
  EFFORT_THRESHOLD   – press arm gently on a surface and read /joint_states
"""

import json
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ---------------------------------------------------------------------------
# Arm geometry – taken directly from the Mirte Master URDF (arm.xacro)
# ---------------------------------------------------------------------------

L1: float = 0.1378            # shoulder_lift → elbow link length (m)
L2: float = 0.14265           # elbow → wrist link length (m)

SHOULDER_Z: float = 0.0881    # shoulder_lift pivot height above frame_link (m)
SHOULDER_Y: float = 0.079274  # forward offset from frame_link origin to pivot (m)
REACH_OFFSET: float = 0.00625 # small horizontal offset baked into the chain (m)

JOINT_LIMIT: float = math.pi / 2   # ±90° hard limit on every arm joint (rad)
GROUND_Z: float = -0.0955     # ground plane in frame_link coordinates (m)

EFFORT_THRESHOLD: float = 0.5 # joint effort (Nm) that signals sand contact

# ---------------------------------------------------------------------------
# Drawing height / pen
# ---------------------------------------------------------------------------
# Set to True to skip probing and draw at a fixed wrist height (for testing).
SKIP_PROBING: bool = True

# Wrist z (in frame_link) held while drawing. The pen/gripper extends below
# the wrist down to the sand at GROUND_Z. The wrist physically cannot go much
# below ~-0.05, so keep this near 0.0 and tune the pen length instead.
WRIST_DRAW_HEIGHT: float = 0.0
WRIST_DRAW_ANGLE:  float = 0.0   # wrist_joint angle to aim the pen downward (rad)

PEN_LIFT: float = 0.020       # how much to raise the wrist between strokes (m)

PROBE_LIFT_START: float = 1.2  # shoulder_lift angle to start probing (rad, <pi/2)
PROBE_LIFT_STEP:  float = 0.05 # how much to raise lift toward horizontal each step
PROBE_LIFT_MAX:   float = 1.55 # abort if probe reaches this angle (≈ pi/2)
PROBE_INTERVAL:   float = 0.5  # seconds between probe steps

# ---------------------------------------------------------------------------
# Drawing layout  (sized to the reachable band – keep letters small!)
# ---------------------------------------------------------------------------

DRAW_X_CENTER:  float = 0.115 # center forward distance of drawing area (m)
LETTER_HEIGHT:  float = 0.040 # letter extent in the forward direction (m)
LETTER_WIDTH:   float = 0.035 # letter extent in the lateral direction (m)
LETTER_GAP:     float = 0.015 # gap between letters (m)
DRAW_STEP_SEC:  int   = 2     # seconds per waypoint

# ---------------------------------------------------------------------------
# Single-stroke font
# ---------------------------------------------------------------------------
# Each letter is a list of strokes.
# Each stroke is a list of (x, y) points, normalised to [0-1] × [0-1].
#   x = 0 → left edge,   x = 1 → right edge  (maps to robot lateral axis)
#   y = 0 → bottom,      y = 1 → top          (maps to robot forward axis)

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
        self._mode: str             = "IDLE"
        self._text: str             = ""
        self._draw_z: float | None  = None   # wrist height held while drawing

        self._joint_positions: dict[str, float] = {}
        self._joint_efforts:   dict[str, float] = {}

        self._probe_lift: float = PROBE_LIFT_START
        self._probe_timer = None
        self._done_timer  = None

        self._selftest()

    # -- Subscriptions -------------------------------------------------------

    def _state_callback(self, msg: String) -> None:
        if msg.data == "DRAW_PATTERN" and self._mode == "IDLE":
            if not self._text:
                self.get_logger().warn("No text received yet – waiting for /whiteboard_text")
                return
            if SKIP_PROBING:
                self._draw_z = WRIST_DRAW_HEIGHT
                self.get_logger().info(
                    f"DRAW_PATTERN – skipping probe, wrist height={self._draw_z} m, "
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
    # Probing lowers the wrist by driving shoulder_lift toward horizontal
    # (pi/2). The wrist height for a given (lift, elbow) follows the same
    # from-vertical convention as the IK/FK below.

    def _start_probe(self) -> None:
        self._probe_lift = PROBE_LIFT_START
        self._send_arm(pan=0.0, lift=self._probe_lift, elbow=0.0,
                       wrist=WRIST_DRAW_ANGLE, duration=1)
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
            self._draw_z = SHOULDER_Z + L1 * math.cos(lift) + L2 * math.cos(lift + elbow)
            self.get_logger().info(
                f"Sand contact: effort={effort:.2f} Nm  wrist_z={self._draw_z:.3f} m"
            )
            self._probe_timer.cancel()
            self._mode = "DRAWING"
            self._draw()
            return

        self._probe_lift += PROBE_LIFT_STEP
        if self._probe_lift > PROBE_LIFT_MAX:
            self.get_logger().warn("Probe reached maximum angle without contact")
            self._probe_timer.cancel()
            self._mode = "IDLE"
            return

        self.get_logger().info(
            f"Probing: lift={math.degrees(self._probe_lift):.1f}°  effort={effort:.2f}"
        )
        self._send_arm(pan=0.0, lift=self._probe_lift, elbow=0.0,
                       wrist=WRIST_DRAW_ANGLE, duration=PROBE_INTERVAL)

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
        for fwd, lat, z in waypoints:
            result = self._ik(fwd, lat, z)
            if result is None:
                skipped += 1
                continue
            pan, lift, elbow, wrist = result
            wp = JointTrajectoryPoint()
            wp.positions = [pan, lift, elbow, wrist]
            wp.time_from_start.sec = t
            traj.points.append(wp)
            t += DRAW_STEP_SEC

        if skipped:
            self.get_logger().warn(
                f"{skipped}/{len(waypoints)} waypoints out of reach – skipped. "
                f"If many, shrink LETTER_* or move DRAW_X_CENTER into the "
                f"reachable band (see startup self-test)."
            )

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
        """Convert text to (forward, lateral, z) frame_link coords with pen-up moves."""
        n = len(text)
        total_w = n * LETTER_WIDTH + max(0, n - 1) * LETTER_GAP
        lat_start = -total_w / 2  # centre the text laterally

        waypoints: list[tuple[float, float, float]] = []

        for i, ch in enumerate(text):
            strokes = STROKES.get(ch, [])
            lat_letter = lat_start + i * (LETTER_WIDTH + LETTER_GAP)

            for stroke in strokes:
                # Pen up: move to first point of stroke at lift height
                lx, ly = stroke[0]
                fwd, lat = self._letter_to_robot(lx, ly, lat_letter)
                waypoints.append((fwd, lat, self._draw_z + PEN_LIFT))

                # Pen down: draw each point in the stroke
                for lx, ly in stroke:
                    fwd, lat = self._letter_to_robot(lx, ly, lat_letter)
                    waypoints.append((fwd, lat, self._draw_z))

        return waypoints

    def _letter_to_robot(
        self, lx: float, ly: float, lat_offset: float
    ) -> tuple[float, float]:
        """
        Map normalised letter coordinates to robot (forward, lateral).
          lx [0-1]: horizontal in letter  → robot lateral axis
          ly [0-1]: vertical in letter    → robot forward axis
        """
        fwd = DRAW_X_CENTER + (ly - 0.5) * LETTER_HEIGHT
        lat = lat_offset + lx * LETTER_WIDTH
        return fwd, lat

    def _on_draw_complete(self) -> None:
        self._done_timer.cancel()
        self._signal_done()

    # -- Forward / inverse kinematics ---------------------------------------
    # Derived and numerically verified against the real Mirte Master URDF.
    # Joint variables are sent to the controller directly (no extra negation).
    #   pan   = shoulder_pan_joint   lift = shoulder_lift_joint
    #   elbow = elbow_joint          wrist = wrist_joint
    # Angles for lift/elbow are measured from vertical (0 = link points up).

    def _fk_wrist(self, pan: float, lift: float, elbow: float) -> tuple[float, float, float]:
        """Forward kinematics to the wrist origin → (forward, lateral, z)."""
        s2, c2   = math.sin(lift), math.cos(lift)
        s23, c23 = math.sin(lift + elbow), math.cos(lift + elbow)
        reach = L1 * s2 + L2 * s23 - REACH_OFFSET
        lateral = -math.sin(pan) * reach
        forward = -SHOULDER_Y + math.cos(pan) * reach
        z = SHOULDER_Z + L1 * c2 + L2 * c23
        return forward, lateral, z

    def _ik(self, forward: float, lateral: float, z: float) -> tuple | None:
        """
        Inverse kinematics for the wrist origin.
        Returns (pan, lift, elbow, wrist) within ±90° limits, or None if the
        target is unreachable / would violate a joint limit.
        """
        dy = forward + SHOULDER_Y
        reach = math.hypot(lateral, dy)
        pan = math.atan2(-lateral, dy)

        a = reach + REACH_OFFSET          # horizontal component of the 2-link
        b = z - SHOULDER_Z                # vertical component (from pivot)

        cos_elbow = (a * a + b * b - L1 * L1 - L2 * L2) / (2 * L1 * L2)
        if abs(cos_elbow) > 1.0:
            return None                   # out of physical reach

        # Try both elbow configurations; keep the first within all joint limits.
        for sign in (-1.0, 1.0):
            elbow = math.atan2(sign * math.sqrt(max(0.0, 1.0 - cos_elbow ** 2)), cos_elbow)
            lift  = math.atan2(a, b) - math.atan2(
                L2 * math.sin(elbow),
                L1 + L2 * math.cos(elbow),
            )
            if (abs(pan)   <= JOINT_LIMIT and
                    abs(lift)  <= JOINT_LIMIT and
                    abs(elbow) <= JOINT_LIMIT):
                return pan, lift, elbow, WRIST_DRAW_ANGLE
        return None

    def _selftest(self) -> None:
        """Log a round-trip IK/FK check and the reachable forward band."""
        center = (DRAW_X_CENTER, 0.0, WRIST_DRAW_HEIGHT)
        sol = self._ik(*center)
        if sol is None:
            self.get_logger().error(
                f"IK self-test FAILED: draw center {center} is UNREACHABLE. "
                f"Move DRAW_X_CENTER into the reachable band below."
            )
        else:
            pan, lift, elbow, wrist = sol
            fwd, lat, z = self._fk_wrist(pan, lift, elbow)
            err = math.dist((fwd, lat, z), center)
            self.get_logger().info(
                f"IK self-test OK: center {center} → "
                f"pan={math.degrees(pan):.1f}° lift={math.degrees(lift):.1f}° "
                f"elbow={math.degrees(elbow):.1f}°  (round-trip err {err*1000:.2f} mm)"
            )

        lo = hi = None
        f = 0.05
        while f < 0.25:
            if self._ik(f, 0.0, WRIST_DRAW_HEIGHT) is not None:
                lo = f if lo is None else lo
                hi = f
            f += 0.005
        if lo is not None:
            self.get_logger().info(
                f"Reachable forward band at wrist z={WRIST_DRAW_HEIGHT}: "
                f"{lo:.3f}–{hi:.3f} m (lateral=0). Letters span "
                f"{DRAW_X_CENTER - LETTER_HEIGHT/2:.3f}–{DRAW_X_CENTER + LETTER_HEIGHT/2:.3f} m."
            )

    # -- Helpers -------------------------------------------------------------

    def _send_arm(self, pan: float, lift: float, elbow: float,
                  wrist: float, duration: float) -> None:
        traj = JointTrajectory()
        traj.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint",
            "elbow_joint",        "wrist_joint",
        ]
        pt = JointTrajectoryPoint()
        pt.positions = [pan, lift, elbow, wrist]
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
