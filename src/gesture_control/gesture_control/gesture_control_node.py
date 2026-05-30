"""
gesture_control_node.py
=======================
ROS 2 node that:
  1. Captures webcam frames using OpenCV.
  2. Tracks hand landmarks using MediaPipe Hands.
  3. Implements a clutch state machine (fist=ENGAGED, open=RELEASED).
  4. Maps 3D hand displacements to Cartesian robot end-effector targets.
  5. Solves IK using Damped Least Squares to find joint angles.
  6. Publishes JointState messages to /joint_states at 20 Hz.
  7. Draws a rich HUD overlay on the OpenCV video feed.

Coordinate Mapping (from hand space → robot base frame):
  Hand X (horizontal, normalized 0→1 left to right in mirrored frame)
    → Robot Y axis (left/right)  scale factor K_Y
  Hand Y (vertical, normalized 0→1 top to bottom)
    → Robot Z axis (up/down)     scale factor K_Z  (inverted: up = lower Y)
  Hand scale d = dist(WRIST, MIDDLE_MCP) in normalized image coords
    → Robot X axis (depth/reach) scale factor K_X
"""

import sys
import time
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time as RosTime

# MediaPipe (lazy import to allow the node to start before mediapipe is sourced)
try:
    import mediapipe as mp
    _MP_AVAILABLE = True
except ImportError:
    _MP_AVAILABLE = False

from gesture_control.kinematics import (
    forward_kinematics, solve_ik,
    HOME_JOINTS, HOME_EE_POS,
    JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# MediaPipe landmark indices
LM_WRIST        = 0
LM_INDEX_MCP    = 5
LM_MIDDLE_MCP   = 9
LM_RING_MCP     = 13
LM_PINKY_MCP    = 17
LM_INDEX_TIP    = 8
LM_MIDDLE_TIP   = 12
LM_RING_TIP     = 16
LM_PINKY_TIP    = 20
LM_THUMB_TIP    = 4

# Fist detection threshold: ratio of tip-to-wrist distance vs palm size.
# When all finger tips are close to the wrist (fist closed), ratio < threshold.
FIST_CLOSED_THRESHOLD = 1.60   # empirically tuned

# Cartesian workspace safety limits (metres, in robot base frame)
WS_X_MIN, WS_X_MAX =  0.10,  0.80   # forward reach
WS_Y_MIN, WS_Y_MAX = -0.75,  0.75   # left/right
WS_Z_MIN, WS_Z_MAX =  0.05,  0.85   # height

# Mapping sensitivity (metres per normalized-image-unit)
K_X =  2.50   # depth (hand scale → robot X)
K_Y =  2.00   # left/right
K_Z =  2.00   # up/down

# IK solver parameters
IK_MAX_ITERS   = 80
IK_TOLERANCE   = 1e-4
IK_DAMPING     = 0.05
IK_STEP_SIZE   = 0.55

# Smoothing factor for target positions (0 = no smoothing, 1 = frozen)
ALPHA = 0.45   # exponential moving average weight for new position

# Node loop rate
LOOP_HZ = 20

# HUD colours (BGR)
CLR_GREEN      = (80,  210,  80)
CLR_ORANGE     = (50,  170, 255)
CLR_RED        = (60,   60, 240)
CLR_WHITE      = (255, 255, 255)
CLR_BLACK      = (0,     0,   0)
CLR_YELLOW     = (30,  220, 220)
CLR_CYAN       = (220, 200,  50)
CLR_DARK_BG    = (30,   30,  30)
CLR_ACCENT_GRN = (100, 230, 140)
CLR_ACCENT_ORG = (80,  150, 240)

# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------

def _dist2d(lm_a, lm_b) -> float:
    """Euclidean distance between two MediaPipe NormalizedLandmark objects."""
    return math.hypot(lm_a.x - lm_b.x, lm_a.y - lm_b.y)


def _is_fist_closed(landmarks) -> bool:
    """
    Returns True when the hand is in a fist (all fingers curled).

    Strategy: compute the palm size (wrist→middle_mcp distance), then check
    if all four finger tips are within FIST_CLOSED_THRESHOLD × palm_size of
    the wrist. Thumb is intentionally excluded for robustness.
    """
    wrist  = landmarks[LM_WRIST]
    palm_d = _dist2d(wrist, landmarks[LM_MIDDLE_MCP]) + 1e-9

    tip_indices = [LM_INDEX_TIP, LM_MIDDLE_TIP, LM_RING_TIP, LM_PINKY_TIP]
    avg_ratio = sum(_dist2d(wrist, landmarks[i]) for i in tip_indices) / (4.0 * palm_d)
    return avg_ratio < FIST_CLOSED_THRESHOLD


def _get_hand_coords(landmarks):
    """
    Extract the three normalised hand-space coordinates used for mapping.

    Returns
    -------
    cx : float  — horizontal centre (normalised 0–1, flipped for mirror mode)
    cy : float  — vertical centre   (normalised 0–1)
    d  : float  — wrist→middle_mcp distance (proxy for depth)
    """
    wrist  = landmarks[LM_WRIST]
    m_mcp  = landmarks[LM_MIDDLE_MCP]
    cx = wrist.x          # already flipped by cv2.flip + mp.FLIP_HORIZONTAL
    cy = wrist.y
    d  = _dist2d(wrist, m_mcp)
    return cx, cy, d


def _clamp_workspace(x, y, z):
    """
    Clamp the Cartesian target to the safe workspace volume.

    Returns
    -------
    cx, cy, cz : clamped coordinates
    clamped    : bool — True if any axis was clamped
    """
    cx = float(np.clip(x, WS_X_MIN, WS_X_MAX))
    cy = float(np.clip(y, WS_Y_MIN, WS_Y_MAX))
    cz = float(np.clip(z, WS_Z_MIN, WS_Z_MAX))
    clamped = (cx != x) or (cy != y) or (cz != z)
    return cx, cy, cz, clamped


# ---------------------------------------------------------------------------
# HUD drawing helpers
# ---------------------------------------------------------------------------

def _draw_rounded_rect(img, x1, y1, x2, y2, color, radius=12, alpha=0.55):
    """Draw a semi-transparent rounded rectangle as a HUD panel."""
    overlay = img.copy()
    cv2.rectangle(overlay, (x1 + radius, y1), (x2 - radius, y2), color, -1)
    cv2.rectangle(overlay, (x1, y1 + radius), (x2, y2 - radius), color, -1)
    for cx, cy in [(x1+radius, y1+radius), (x2-radius, y1+radius),
                   (x1+radius, y2-radius), (x2-radius, y2-radius)]:
        cv2.circle(overlay, (cx, cy), radius, color, -1)
    cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)


def _put_text(img, text, pos, color, scale=0.65, thickness=1, bold=False):
    font = cv2.FONT_HERSHEY_DUPLEX
    if bold:
        cv2.putText(img, text, pos, font, scale, CLR_BLACK, thickness + 2, cv2.LINE_AA)
    cv2.putText(img, text, pos, font, scale, color, thickness, cv2.LINE_AA)


def _draw_hud(img, state, target, clamped, residual, q_joints, anchor_px=None, hand_px=None):
    """
    Render the full heads-up display on the frame.

    Parameters
    ----------
    img       : the OpenCV frame (modified in-place).
    state     : 'ENGAGED' or 'RELEASED'
    target    : (x, y, z) desired end-effector position in robot frame
    clamped   : bool — workspace limit was hit
    residual  : float — IK residual error in metres
    q_joints  : current joint angles (6,) for visual indicator
    anchor_px : (px, py) pixel position of the engagement anchor (or None)
    hand_px   : (px, py) pixel position of current hand centroid (or None)
    """
    h, w = img.shape[:2]
    engaged = (state == 'ENGAGED')

    # ---- Status panel (top-left) ----
    panel_color = (20, 80, 20) if engaged else (30, 60, 80)
    _draw_rounded_rect(img, 8, 8, 310, 130, panel_color, radius=14, alpha=0.65)

    state_color = CLR_ACCENT_GRN if engaged else CLR_ACCENT_ORG
    state_icon  = '●' if engaged else '○'
    _put_text(img, f'{state_icon} CLUTCH: {state}', (20, 38), state_color, scale=0.80, thickness=2, bold=True)
    _put_text(img, f'X:{target[0]:+.3f}  Y:{target[1]:+.3f}', (20, 68), CLR_WHITE, scale=0.60)
    _put_text(img, f'Z:{target[2]:+.3f}  IK err:{residual*1000:.1f}mm', (20, 92), CLR_WHITE, scale=0.60)

    # Fist instruction
    hint = 'Close fist to ENGAGE' if not engaged else 'Open fist to RELEASE'
    _put_text(img, hint, (20, 118), (180, 180, 180), scale=0.52)

    # ---- Workspace-limit warning ----
    if clamped:
        t = time.time()
        if int(t * 2) % 2 == 0:   # flash at 2 Hz
            _draw_rounded_rect(img, w//2 - 180, 8, w//2 + 180, 46, (0, 0, 160), radius=10, alpha=0.75)
            _put_text(img, '⚠  WORKSPACE LIMIT REACHED', (w//2 - 168, 34), CLR_RED, scale=0.60, thickness=2, bold=True)

    # ---- Joint angle mini-bar chart (bottom strip) ----
    bar_h = 28
    bar_y0 = h - bar_h - 8
    bar_x0 = 8
    bar_w_each = (w - 16) // 6
    for i in range(6):
        lo, hi = JOINT_LIMITS_LOWER[i], JOINT_LIMITS_UPPER[i]
        ratio = (q_joints[i] - lo) / (hi - lo + 1e-9)
        bx = bar_x0 + i * bar_w_each
        _draw_rounded_rect(img, bx, bar_y0, bx + bar_w_each - 4, bar_y0 + bar_h,
                           (50, 50, 50), radius=4, alpha=0.6)
        fill_w = max(4, int((bar_w_each - 8) * ratio))
        bar_fill_color = CLR_ACCENT_GRN if engaged else (100, 120, 100)
        cv2.rectangle(img, (bx + 4, bar_y0 + 5), (bx + 4 + fill_w, bar_y0 + bar_h - 5),
                      bar_fill_color, -1)
        _put_text(img, f'J{i+1}', (bx + 6, bar_y0 + bar_h - 7), CLR_WHITE, scale=0.38)

    # ---- Anchor crosshair + displacement vector ----
    if engaged and anchor_px and hand_px:
        ax, ay = anchor_px
        hx, hy = hand_px
        cv2.line(img, (ax - 14, ay), (ax + 14, ay), CLR_YELLOW, 2, cv2.LINE_AA)
        cv2.line(img, (ax, ay - 14), (ax, ay + 14), CLR_YELLOW, 2, cv2.LINE_AA)
        cv2.circle(img, (ax, ay), 6, CLR_YELLOW, -1)
        cv2.arrowedLine(img, (ax, ay), (hx, hy), CLR_CYAN, 2, cv2.LINE_AA, tipLength=0.15)

    # ---- Branding watermark ----
    _put_text(img, 'UR5e Gesture Control | ROS 2 Humble', (w - 318, h - 12),
              (100, 100, 100), scale=0.42)


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------

class GestureControlNode(Node):

    def __init__(self):
        super().__init__('gesture_control_node')

        # --- Parameters ---
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('loop_hz', LOOP_HZ)
        cam_idx  = self.get_parameter('camera_index').value
        loop_hz  = self.get_parameter('loop_hz').value

        # --- Publisher ---
        self.pub_joint_state = self.create_publisher(JointState, '/joint_states', 10)

        # --- State ---
        self.q_current  = HOME_JOINTS.copy()              # current joint angles
        self.ee_target  = HOME_EE_POS.copy()              # smoothed target EE pos
        self.ee_frozen  = HOME_EE_POS.copy()              # frozen baseline (used when engaged)
        self.clutch     = 'RELEASED'
        self.anchor_cx  = 0.0
        self.anchor_cy  = 0.0
        self.anchor_d   = 0.0
        self.last_residual = 0.0
        self.workspace_clamped = False

        # --- OpenCV + MediaPipe ---
        self.cap = cv2.VideoCapture(cam_idx)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera at index {cam_idx}. Check device.')
            sys.exit(1)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not _MP_AVAILABLE:
            self.get_logger().fatal('mediapipe not installed. Run: pip install --user mediapipe')
            sys.exit(1)

        mp_hands = mp.solutions.hands
        self.hands = mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            model_complexity=1,
            min_detection_confidence=0.65,
            min_tracking_confidence=0.55,
        )
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

        # --- Timer ---
        period = 1.0 / float(loop_hz)
        self.timer = self.create_timer(period, self._loop_callback)

        self.get_logger().info(
            f'GestureControlNode started — camera={cam_idx}, rate={loop_hz} Hz'
        )

    # -----------------------------------------------------------------------

    def _loop_callback(self):
        """Main control loop: capture → perceive → map → IK → publish → display."""

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn('Failed to read frame from camera.', throttle_duration_sec=2.0)
            self._publish_joints()
            return

        # Mirror horizontally so user's left = left on screen (intuitive)
        frame = cv2.flip(frame, 1)
        h, w = frame.shape[:2]

        # Convert BGR → RGB for MediaPipe
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        results = self.hands.process(rgb)
        rgb.flags.writeable = True

        anchor_px = None
        hand_px   = None

        if results.multi_hand_landmarks:
            hand_lm = results.multi_hand_landmarks[0]
            lm = hand_lm.landmark

            # Draw hand skeleton on frame
            self.mp_drawing.draw_landmarks(
                frame, hand_lm,
                mp.solutions.hands.HAND_CONNECTIONS,
                self.mp_drawing_styles.get_default_hand_landmarks_style(),
                self.mp_drawing_styles.get_default_hand_connections_style(),
            )

            fist = _is_fist_closed(lm)
            cx, cy, d = _get_hand_coords(lm)
            hand_px = (int(cx * w), int(cy * h))

            if fist:
                # ---- ENGAGED ----
                if self.clutch == 'RELEASED':
                    # Transition: record anchor
                    self.clutch    = 'ENGAGED'
                    self.anchor_cx = cx
                    self.anchor_cy = cy
                    self.anchor_d  = d
                    self.ee_frozen = self.ee_target.copy()
                    self.get_logger().info('Clutch ENGAGED')

                # Compute relative displacement from anchor
                delta_x =  K_X * (d  - self.anchor_d)
                delta_y = -K_Y * (cx - self.anchor_cx)   # right = +Y in robot
                delta_z = -K_Z * (cy - self.anchor_cy)   # up = +Z in robot

                raw_target = self.ee_frozen + np.array([delta_x, delta_y, delta_z])

                # Clamp to safe workspace
                tx, ty, tz, clamped = _clamp_workspace(*raw_target)
                self.workspace_clamped = clamped
                new_target = np.array([tx, ty, tz])

                # Exponential smoothing
                self.ee_target = ALPHA * self.ee_target + (1.0 - ALPHA) * new_target

                # Anchor pixel (for vector overlay)
                anchor_px = (int(self.anchor_cx * w), int(self.anchor_cy * h))

            else:
                # ---- RELEASED ----
                if self.clutch == 'ENGAGED':
                    # Transition: freeze at current position
                    self.clutch = 'RELEASED'
                    self.ee_frozen = self.ee_target.copy()
                    self.get_logger().info('Clutch RELEASED')
                self.workspace_clamped = False

        else:
            # No hand detected — stay released
            if self.clutch == 'ENGAGED':
                self.clutch = 'RELEASED'
                self.ee_frozen = self.ee_target.copy()
                self.get_logger().info('Hand lost — Clutch RELEASED')

        # ---- IK Solve ----
        q_sol, residual = solve_ik(
            self.ee_target,
            self.q_current,
            max_iters=IK_MAX_ITERS,
            tol=IK_TOLERANCE,
            damping=IK_DAMPING,
            step_size=IK_STEP_SIZE,
        )
        self.q_current = q_sol
        self.last_residual = residual

        # ---- Publish ----
        self._publish_joints()

        # ---- Draw HUD ----
        _draw_hud(
            frame,
            state=self.clutch,
            target=self.ee_target,
            clamped=self.workspace_clamped,
            residual=self.last_residual,
            q_joints=self.q_current,
            anchor_px=anchor_px,
            hand_px=hand_px,
        )

        cv2.imshow('UR5e Gesture Control', frame)
        cv2.waitKey(1)

    # -----------------------------------------------------------------------

    def _publish_joints(self):
        """Build and publish a JointState message with the current joint angles."""
        msg = JointState()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.name     = JOINT_NAMES
        msg.position = self.q_current.tolist()
        msg.velocity = [0.0] * 6
        msg.effort   = [0.0] * 6
        self.pub_joint_state.publish(msg)

    # -----------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info('Shutting down GestureControlNode.')
        self.cap.release()
        cv2.destroyAllWindows()
        self.hands.close()
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = GestureControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
