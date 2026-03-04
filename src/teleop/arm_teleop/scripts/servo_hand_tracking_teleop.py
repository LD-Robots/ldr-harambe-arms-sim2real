#!/usr/bin/env python3
"""
Dual-arm hand tracking teleop using Orbbec Gemini 336L + MediaPipe Hands + MoveIt Servo.

Tracks both palms in 3D using aligned depth camera, publishes PoseStamped targets
to two MoveIt Servo nodes (left/right arm), and maps finger curl to robot hand joints.

Prerequisites:
  - Full dual-arm system running (Gazebo + controllers)
  - Orbbec camera running: ros2 launch orbbec_camera gemini_330_series.launch.py depth_registration:=true
  - Launch via: ros2 launch arm_teleop servo_hand_tracking_teleop.launch.py

Controls (OpenCV window):
  e     - Toggle tracking on/off
  d     - Toggle debug overlay
  q/Esc - Quit
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field
from typing import Optional

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Quaternion
from visualization_msgs.msg import Marker
from image_geometry import PinholeCameraModel
from moveit_msgs.srv import ServoCommandType
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import SetBool
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# MediaPipe hand landmark indices
WRIST = 0
INDEX_MCP = 5
MIDDLE_MCP = 9
PINKY_MCP = 17

# Finger landmark groups: (MCP, PIP, DIP/TIP) for curl angle computation
FINGER_LANDMARKS = {
    "thumb": (2, 3, 4),    # thumb MCP, IP, TIP
    "index": (5, 6, 8),    # index MCP, PIP, TIP
    "middle": (9, 10, 12),
    "ring": (13, 14, 16),
    "pinky": (17, 18, 20),
}
# Thumb abduction: angle between thumb MCP direction and index MCP direction
THUMB_ABD_LANDMARKS = (1, 2, 5)  # thumb CMC, thumb MCP, index MCP

COMMAND_TYPE_POSE = 2

# EE frame names for TF lookup (current EE position)
EE_FRAMES = {
    "left": "urdf_l_wrist_assembly",
    "right": "urdf_r_wrist_assembly",
}


# Camera optical frame name (from Orbbec driver, child of camera_link)
CAMERA_OPTICAL_FRAME = "camera_color_optical_frame"

# Robot reference point (where the robot's "head" is, in urdf_base frame)
# Midpoint above shoulders: X=0.05 forward, Y=0 center, Z=0.55 above shoulders
ROBOT_HEAD_REF = np.array([0.05, 0.0, 0.55])


@dataclass
class HandState:
    """Smoothed state for one hand."""

    position: Optional[np.ndarray] = None
    orientation: Optional[np.ndarray] = None  # quaternion [x, y, z, w]
    finger_curls: dict = field(default_factory=dict)
    last_valid_position: Optional[np.ndarray] = None
    detected: bool = False


class ServoHandTrackingTeleop(Node):
    """Dual-arm hand tracking teleop node."""

    # Robot hand joint limits from SRDF close state
    FINGER_JOINT_MAX = {
        "thumb_proximal_yaw": 1.309,   # 75 degrees
        "thumb_proximal_pitch": 0.611,  # 35 degrees
        "index_proximal": 1.466,   # 84 degrees
        "middle_proximal": 1.466,  # 84 degrees
        "ring_proximal": 1.466,    # 84 degrees
        "pinky_proximal": 1.466,   # 84 degrees
    }

    LEFT_HAND_JOINTS = [
        "left_thumb_proximal_yaw_joint",
        "left_thumb_proximal_pitch_joint",
        "left_index_proximal_joint",
        "left_middle_proximal_joint",
        "left_ring_proximal_joint",
        "left_pinky_proximal_joint",
    ]

    RIGHT_HAND_JOINTS = [
        "right_thumb_proximal_yaw_joint",
        "right_thumb_proximal_pitch_joint",
        "right_index_proximal_joint",
        "right_middle_proximal_joint",
        "right_ring_proximal_joint",
        "right_pinky_proximal_joint",
    ]

    def __init__(self) -> None:
        super().__init__("servo_hand_tracking_teleop")

        self._cb_group = ReentrantCallbackGroup()

        # Parameters
        self._smoothing_alpha = float(
            self.declare_parameter("smoothing_alpha", 0.4).value
        )
        self._tracking_enabled = bool(
            self.declare_parameter("tracking_enabled", True).value
        )
        self._debug_mode = bool(
            self.declare_parameter("debug_mode", False).value
        )
        self._hand_publish_rate = float(
            self.declare_parameter("hand_publish_rate", 10.0).value
        )
        self._min_detection_confidence = float(
            self.declare_parameter("min_detection_confidence", 0.7).value
        )
        self._min_tracking_confidence = float(
            self.declare_parameter("min_tracking_confidence", 0.5).value
        )
        self._target_frame = str(
            self.declare_parameter("target_frame", "urdf_base").value
        )

        # State
        self._lock = threading.Lock()
        self._bridge = CvBridge()
        self._cam_model = PinholeCameraModel()
        self._cam_model_ready = False
        self._latest_color: Optional[np.ndarray] = None
        self._latest_depth: Optional[np.ndarray] = None
        self._left_hand = HandState()
        self._right_hand = HandState()
        self._head_position: Optional[np.ndarray] = None  # smoothed head 3D in camera frame
        self._head_position_base: Optional[np.ndarray] = None  # head in urdf_base frame
        self._servo_ready = {"left": False, "right": False}
        self._command_type_set = {"left": False, "right": False}

        # TF2
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # MediaPipe Hands
        self._mp_hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=self._min_detection_confidence,
            min_tracking_confidence=self._min_tracking_confidence,
        )
        self._mp_drawing = mp.solutions.drawing_utils

        # MediaPipe Face Detection (for head reference point)
        self._mp_face = mp.solutions.face_detection.FaceDetection(
            model_selection=0,  # 0=short range (<2m), 1=long range
            min_detection_confidence=0.5,
        )

        # --- Subscribers ---
        self.create_subscription(
            Image, "/camera/color/image_raw", self._color_cb, 5
        )
        self.create_subscription(
            Image, "/camera/depth/image_raw", self._depth_cb, 5
        )
        self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self._camera_info_cb, 5
        )

        # --- Publishers ---
        self._left_pose_pub = self.create_publisher(
            PoseStamped, "/left_servo_node/pose_target_cmds", 10
        )
        self._right_pose_pub = self.create_publisher(
            PoseStamped, "/right_servo_node/pose_target_cmds", 10
        )
        self._left_hand_pub = self.create_publisher(
            JointTrajectory, "/left_hand_controller/joint_trajectory", 10
        )
        self._right_hand_pub = self.create_publisher(
            JointTrajectory, "/right_hand_controller/joint_trajectory", 10
        )
        # Visualization markers for RViz
        self._left_marker_pub = self.create_publisher(
            Marker, "/left_target_marker", 10
        )
        self._right_marker_pub = self.create_publisher(
            Marker, "/right_target_marker", 10
        )
        self._head_marker_pub = self.create_publisher(
            Marker, "/head_ref_marker", 10
        )

        # --- Service clients ---
        self._left_cmd_type_client = self.create_client(
            ServoCommandType,
            "/left_servo_node/switch_command_type",
            callback_group=self._cb_group,
        )
        self._right_cmd_type_client = self.create_client(
            ServoCommandType,
            "/right_servo_node/switch_command_type",
            callback_group=self._cb_group,
        )
        self._left_pause_client = self.create_client(
            SetBool,
            "/left_servo_node/pause_servo",
            callback_group=self._cb_group,
        )
        self._right_pause_client = self.create_client(
            SetBool,
            "/right_servo_node/pause_servo",
            callback_group=self._cb_group,
        )

        # --- Timers (only for non-GUI tasks) ---
        self._finger_timer = self.create_timer(
            1.0 / self._hand_publish_rate, self._finger_loop
        )
        self._init_timer = self.create_timer(1.0, self._init_servo)

        # Control loop rate for main thread
        self._control_rate = 30.0
        # Store latest results for debug display
        self._latest_results = None

        self.get_logger().info(
            "Hand tracking teleop starting...\n"
            f"  Target frame: {self._target_frame}\n"
            f"  Smoothing alpha: {self._smoothing_alpha}\n"
            f"  Debug mode: {self._debug_mode}\n"
            "  Press 'e' to toggle tracking, 'q' to quit"
        )

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #

    def _color_cb(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
            with self._lock:
                self._latest_color = frame
        except Exception as e:
            self.get_logger().error(f"Color conversion error: {e}")

    def _depth_cb(self, msg: Image) -> None:
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, "passthrough")
            with self._lock:
                self._latest_depth = depth
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        if not self._cam_model_ready:
            self._cam_model.from_camera_info(msg)
            self._cam_model_ready = True
            self.get_logger().info("Camera model initialized")

    # ------------------------------------------------------------------ #
    # Servo initialization
    # ------------------------------------------------------------------ #

    def _init_servo(self) -> None:
        """Set command type to POSE and unpause both servo nodes."""
        all_set = all(self._command_type_set.values())
        if all_set:
            self._init_timer.cancel()
            return

        for side, client in [
            ("left", self._left_cmd_type_client),
            ("right", self._right_cmd_type_client),
        ]:
            if self._command_type_set[side]:
                continue
            if not client.service_is_ready():
                self.get_logger().info(
                    f"Waiting for {side}_servo_node/switch_command_type..."
                )
                continue

            request = ServoCommandType.Request()
            request.command_type = COMMAND_TYPE_POSE
            self.get_logger().info(f"Setting {side} servo to POSE mode...")
            future = client.call_async(request)
            future.add_done_callback(
                lambda f, s=side: self._cmd_type_done(f, s)
            )

    def _cmd_type_done(self, future, side: str) -> None:
        try:
            resp = future.result()
            if resp.success:
                self._command_type_set[side] = True
                self.get_logger().info(f"{side} servo POSE mode set, unpausing...")
                pause_client = (
                    self._left_pause_client if side == "left"
                    else self._right_pause_client
                )
                if pause_client.service_is_ready():
                    req = SetBool.Request()
                    req.data = False  # unpause
                    unpause_future = pause_client.call_async(req)
                    unpause_future.add_done_callback(
                        lambda f, s=side: self._unpause_done(f, s)
                    )
                else:
                    self.get_logger().warn(
                        f"{side} pause_servo service not ready"
                    )
                    self._servo_ready[side] = True
            else:
                self.get_logger().warn(
                    f"Failed to set {side} command type, retrying..."
                )
        except Exception as e:
            self.get_logger().error(f"{side} command type call failed: {e}")

    def _unpause_done(self, future, side: str) -> None:
        try:
            future.result()
            self._servo_ready[side] = True
            self.get_logger().info(f"{side} servo unpaused and ready")
        except Exception as e:
            self.get_logger().error(f"{side} unpause failed: {e}")
            self._servo_ready[side] = True

    # ------------------------------------------------------------------ #
    # 3D deprojection
    # ------------------------------------------------------------------ #

    def _deproject_pixel(
        self, px: float, py: float, depth_image: np.ndarray
    ) -> Optional[np.ndarray]:
        """Deproject a pixel + depth to a 3D point in camera optical frame."""
        h, w = depth_image.shape[:2]
        ix, iy = int(round(px)), int(round(py))
        if ix < 0 or ix >= w or iy < 0 or iy >= h:
            return None

        # Sample a small patch around the landmark for robustness
        patch_size = 3
        y0 = max(0, iy - patch_size)
        y1 = min(h, iy + patch_size + 1)
        x0 = max(0, ix - patch_size)
        x1 = min(w, ix + patch_size + 1)
        patch = depth_image[y0:y1, x0:x1].astype(np.float64)
        valid = patch[patch > 0]
        if len(valid) == 0:
            return None

        depth_mm = float(np.median(valid))
        depth_m = depth_mm / 1000.0
        if depth_m < 0.1 or depth_m > 3.0:
            return None

        ray = self._cam_model.projectPixelTo3dRay((px, py))
        point_3d = np.array([ray[0] * depth_m, ray[1] * depth_m, ray[2] * depth_m])
        return point_3d

    # ------------------------------------------------------------------ #
    # Palm orientation from landmarks
    # ------------------------------------------------------------------ #

    @staticmethod
    def _compute_palm_orientation(
        wrist: np.ndarray, index_mcp: np.ndarray, pinky_mcp: np.ndarray
    ) -> np.ndarray:
        """Compute palm orientation quaternion from 3 landmark positions.

        Returns quaternion [x, y, z, w].
        """
        # Palm coordinate frame:
        #   Z = palm normal (cross product of two palm edge vectors)
        #   Y = wrist → middle of (index_mcp + pinky_mcp) / 2 (along fingers)
        #   X = Y cross Z (across palm)
        palm_center = (index_mcp + pinky_mcp) / 2.0
        y_axis = palm_center - wrist
        y_norm = np.linalg.norm(y_axis)
        if y_norm < 1e-6:
            return np.array([0.0, 0.0, 0.0, 1.0])
        y_axis /= y_norm

        edge = index_mcp - pinky_mcp
        z_axis = np.cross(edge, y_axis)
        z_norm = np.linalg.norm(z_axis)
        if z_norm < 1e-6:
            return np.array([0.0, 0.0, 0.0, 1.0])
        z_axis /= z_norm

        x_axis = np.cross(y_axis, z_axis)
        x_axis /= np.linalg.norm(x_axis)

        # Rotation matrix → quaternion
        rot = np.eye(3)
        rot[:, 0] = x_axis
        rot[:, 1] = y_axis
        rot[:, 2] = z_axis
        return _rotation_matrix_to_quaternion(rot)

    # ------------------------------------------------------------------ #
    # Finger curl computation
    # ------------------------------------------------------------------ #

    @staticmethod
    def _compute_finger_curl(
        landmarks, mcp_idx: int, pip_idx: int, tip_idx: int
    ) -> float:
        """Compute finger curl angle (0=extended, pi=fully curled)."""
        mcp = np.array([landmarks[mcp_idx].x, landmarks[mcp_idx].y, landmarks[mcp_idx].z])
        pip = np.array([landmarks[pip_idx].x, landmarks[pip_idx].y, landmarks[pip_idx].z])
        tip = np.array([landmarks[tip_idx].x, landmarks[tip_idx].y, landmarks[tip_idx].z])

        v1 = pip - mcp
        v2 = tip - pip
        n1 = np.linalg.norm(v1)
        n2 = np.linalg.norm(v2)
        if n1 < 1e-6 or n2 < 1e-6:
            return 0.0

        cos_angle = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
        angle = math.acos(cos_angle)
        # angle is large when finger is curled (vectors diverge), small when extended
        return max(0.0, angle)

    @staticmethod
    def _compute_thumb_abduction(landmarks) -> float:
        """Compute thumb abduction angle."""
        cmc = np.array([landmarks[1].x, landmarks[1].y, landmarks[1].z])
        mcp = np.array([landmarks[2].x, landmarks[2].y, landmarks[2].z])
        index_mcp = np.array([landmarks[5].x, landmarks[5].y, landmarks[5].z])

        v_thumb = mcp - cmc
        v_index = index_mcp - cmc
        n1 = np.linalg.norm(v_thumb)
        n2 = np.linalg.norm(v_index)
        if n1 < 1e-6 or n2 < 1e-6:
            return 0.0

        cos_angle = np.clip(np.dot(v_thumb, v_index) / (n1 * n2), -1.0, 1.0)
        return math.acos(cos_angle)

    def _get_finger_positions(self, landmarks) -> list:
        """Compute 6 robot hand joint positions from MediaPipe landmarks.

        Returns [thumb_yaw, thumb_pitch, index, middle, ring, pinky].
        """
        # Thumb abduction → thumb_proximal_yaw (inverted: wide spread = small joint value)
        thumb_abd = self._compute_thumb_abduction(landmarks)
        thumb_yaw = _map_range(thumb_abd, 0.0, math.pi / 2, self.FINGER_JOINT_MAX["thumb_proximal_yaw"], 0.0)

        # Thumb curl → thumb_proximal_pitch
        thumb_curl = self._compute_finger_curl(landmarks, *FINGER_LANDMARKS["thumb"])
        thumb_pitch = _map_range(thumb_curl, 0.0, math.pi, 0.0, self.FINGER_JOINT_MAX["thumb_proximal_pitch"])

        # Other fingers
        index_curl = self._compute_finger_curl(landmarks, *FINGER_LANDMARKS["index"])
        index_pos = _map_range(index_curl, 0.0, math.pi, 0.0, self.FINGER_JOINT_MAX["index_proximal"])

        middle_curl = self._compute_finger_curl(landmarks, *FINGER_LANDMARKS["middle"])
        middle_pos = _map_range(middle_curl, 0.0, math.pi, 0.0, self.FINGER_JOINT_MAX["middle_proximal"])

        ring_curl = self._compute_finger_curl(landmarks, *FINGER_LANDMARKS["ring"])
        ring_pos = _map_range(ring_curl, 0.0, math.pi, 0.0, self.FINGER_JOINT_MAX["ring_proximal"])

        pinky_curl = self._compute_finger_curl(landmarks, *FINGER_LANDMARKS["pinky"])
        pinky_pos = _map_range(pinky_curl, 0.0, math.pi, 0.0, self.FINGER_JOINT_MAX["pinky_proximal"])

        return [thumb_yaw, thumb_pitch, index_pos, middle_pos, ring_pos, pinky_pos]

    # ------------------------------------------------------------------ #
    # EMA smoothing
    # ------------------------------------------------------------------ #

    def _smooth_position(
        self, hand: HandState, new_pos: np.ndarray
    ) -> np.ndarray:
        if hand.position is None:
            hand.position = new_pos.copy()
            return hand.position
        alpha = self._smoothing_alpha
        hand.position = alpha * new_pos + (1.0 - alpha) * hand.position
        return hand.position

    def _smooth_orientation(
        self, hand: HandState, new_quat: np.ndarray
    ) -> np.ndarray:
        if hand.orientation is None:
            hand.orientation = new_quat.copy()
            return hand.orientation
        alpha = self._smoothing_alpha
        # Simple LERP + normalize (good enough for small deltas at 30Hz)
        # Ensure quaternions are in same hemisphere
        if np.dot(hand.orientation, new_quat) < 0:
            new_quat = -new_quat
        hand.orientation = alpha * new_quat + (1.0 - alpha) * hand.orientation
        hand.orientation /= np.linalg.norm(hand.orientation)
        return hand.orientation

    # ------------------------------------------------------------------ #
    # Main loop - runs in main thread for OpenCV GUI compatibility
    # ------------------------------------------------------------------ #

    def run_main_loop(self) -> None:
        """Run the main tracking + display loop in the main thread.

        This MUST be called from the main thread because OpenCV's
        imshow/waitKey require it on most platforms.
        """
        import time

        period = 1.0 / self._control_rate
        while rclpy.ok():
            t0 = time.monotonic()
            self._process_frame()
            elapsed = time.monotonic() - t0
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _process_frame(self) -> None:
        """Process one frame: detect hands, publish poses, show GUI."""
        with self._lock:
            color = self._latest_color
            depth = self._latest_depth

        if color is None or depth is None or not self._cam_model_ready:
            # Still show a blank/waiting window so it doesn't freeze
            blank = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(blank, "Waiting for camera...", (100, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            cv2.imshow("Hand Tracking Teleop", blank)
            key = cv2.waitKey(30) & 0xFF
            if key == ord("q") or key == 27:
                rclpy.shutdown()
            return

        if not self._tracking_enabled:
            self._show_debug(color, None)
            self._handle_key(cv2.waitKey(1) & 0xFF)
            return

        # Run MediaPipe
        rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        h, w = color.shape[:2]

        # --- Face detection (head reference) ---
        face_results = self._mp_face.process(rgb)
        if face_results.detections:
            det = face_results.detections[0]  # first face
            # Nose tip = keypoint 2 in face detection
            nose = det.location_data.relative_keypoints[2]
            nose_px = nose.x * w
            nose_py = nose.y * h
            head_3d = self._deproject_pixel(nose_px, nose_py, depth)
            if head_3d is not None:
                alpha = self._smoothing_alpha
                if self._head_position is None:
                    self._head_position = head_3d.copy()
                else:
                    self._head_position = (
                        alpha * head_3d + (1.0 - alpha) * self._head_position
                    )
                self._publish_head_marker(self._head_position)

        # --- Hand detection ---
        results = self._mp_hands.process(rgb)

        self._left_hand.detected = False
        self._right_hand.detected = False

        if results.multi_hand_landmarks and results.multi_handedness:
            for hand_landmarks, handedness_info in zip(
                results.multi_hand_landmarks, results.multi_handedness
            ):
                label = handedness_info.classification[0].label
                # MediaPipe labels from camera's perspective (mirrored),
                # so "Left" in MediaPipe = user's right hand and vice versa.
                is_left = label == "Right"

                hand_state = self._left_hand if is_left else self._right_hand

                # Get wrist pixel coords
                wrist_lm = hand_landmarks.landmark[WRIST]
                px = wrist_lm.x * w
                py = wrist_lm.y * h

                # Deproject to 3D
                point_3d = self._deproject_pixel(px, py, depth)
                if point_3d is None:
                    if hand_state.last_valid_position is not None:
                        point_3d = hand_state.last_valid_position
                    else:
                        continue

                hand_state.last_valid_position = point_3d.copy()
                smoothed_pos = self._smooth_position(hand_state, point_3d)

                # Palm orientation from 3 landmarks in 3D
                idx_mcp_lm = hand_landmarks.landmark[INDEX_MCP]
                pinky_mcp_lm = hand_landmarks.landmark[PINKY_MCP]

                pts_3d = []
                for lm in [wrist_lm, idx_mcp_lm, pinky_mcp_lm]:
                    lm_px = lm.x * w
                    lm_py = lm.y * h
                    p = self._deproject_pixel(lm_px, lm_py, depth)
                    pts_3d.append(p)

                if all(p is not None for p in pts_3d):
                    quat = self._compute_palm_orientation(*pts_3d)
                    smoothed_quat = self._smooth_orientation(hand_state, quat)
                else:
                    smoothed_quat = hand_state.orientation
                    if smoothed_quat is None:
                        smoothed_quat = np.array([0.0, 0.0, 0.0, 1.0])

                hand_state.detected = True

                # Compute finger curls
                hand_state.finger_curls = self._get_finger_positions(
                    hand_landmarks.landmark
                )

                # Publish pose command to servo (head-relative)
                self._publish_pose(
                    smoothed_pos,
                    smoothed_quat,
                    is_left,
                )

        # Debug visualization + keyboard (must be in main thread)
        self._show_debug(color, results)
        self._handle_key(cv2.waitKey(1) & 0xFF)

    def _handle_key(self, key: int) -> None:
        if key == ord("q") or key == 27:
            self.get_logger().info("Quit requested")
            rclpy.shutdown()
        elif key == ord("e"):
            self._tracking_enabled = not self._tracking_enabled
            self.get_logger().info(
                f"Tracking {'enabled' if self._tracking_enabled else 'disabled'}"
            )
        elif key == ord("d"):
            self._debug_mode = not self._debug_mode
            self.get_logger().info(f"Debug mode: {self._debug_mode}")

    # ------------------------------------------------------------------ #
    # Finger publishing loop (10 Hz)
    # ------------------------------------------------------------------ #

    def _finger_loop(self) -> None:
        for hand_state, joints, pub in [
            (self._left_hand, self.LEFT_HAND_JOINTS, self._left_hand_pub),
            (self._right_hand, self.RIGHT_HAND_JOINTS, self._right_hand_pub),
        ]:
            if not hand_state.detected or not hand_state.finger_curls:
                continue

            positions = hand_state.finger_curls
            if not isinstance(positions, list) or len(positions) != 6:
                continue

            msg = JointTrajectory()
            msg.joint_names = list(joints)
            point = JointTrajectoryPoint()
            point.positions = [float(p) for p in positions]
            point.time_from_start = Duration(sec=0, nanosec=200_000_000)
            msg.points = [point]
            pub.publish(msg)

    # ------------------------------------------------------------------ #
    # Pose publishing
    # ------------------------------------------------------------------ #

    def _publish_head_marker(self, head_cam: np.ndarray) -> None:
        """Transform head position to urdf_base and publish RViz marker."""
        try:
            tf_cam_to_base = self._tf_buffer.lookup_transform(
                self._target_frame,
                CAMERA_OPTICAL_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except Exception:
            return

        cam_pose = PoseStamped()
        cam_pose.header.stamp = self.get_clock().now().to_msg()
        cam_pose.header.frame_id = CAMERA_OPTICAL_FRAME
        cam_pose.pose.position.x = float(head_cam[0])
        cam_pose.pose.position.y = float(head_cam[1])
        cam_pose.pose.position.z = float(head_cam[2])
        cam_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        base_pose = do_transform_pose_stamped(cam_pose, tf_cam_to_base)
        self._head_position_base = np.array([
            base_pose.pose.position.x,
            base_pose.pose.position.y,
            base_pose.pose.position.z,
        ])

        # Blue sphere for head reference on robot
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self._target_frame
        marker.ns = "head_ref"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(ROBOT_HEAD_REF[0])
        marker.pose.position.y = float(ROBOT_HEAD_REF[1])
        marker.pose.position.z = float(ROBOT_HEAD_REF[2])
        marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.a = 0.6
        marker.color.r, marker.color.g, marker.color.b = 0.0, 0.5, 1.0
        self._head_marker_pub.publish(marker)

    def _publish_pose(
        self,
        position: np.ndarray,
        orientation: np.ndarray,
        is_left: bool,
    ) -> None:
        """Compute hand position relative to head, apply to robot reference.

        hand_offset = hand_3d - head_3d (in camera frame)
        robot_target = ROBOT_HEAD_REF + hand_offset (transformed to robot frame)
        """
        side = "left" if is_left else "right"

        if not self._servo_ready.get(side, False):
            self.get_logger().warn(
                f"{side} servo not ready, skipping pose publish",
                throttle_duration_sec=2.0,
            )
            return

        if self._head_position is None:
            self.get_logger().warn(
                "Head not detected yet, skipping pose publish",
                throttle_duration_sec=2.0,
            )
            return

        # 1. Compute offset from head in camera frame
        offset_cam = position - self._head_position

        # 2. Transform the offset to urdf_base orientation
        #    Build a PoseStamped for the offset point (relative to camera origin)
        try:
            tf_cam_to_base = self._tf_buffer.lookup_transform(
                self._target_frame,
                CAMERA_OPTICAL_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except Exception as e:
            self.get_logger().warn(
                f"TF camera→base lookup failed: {e}",
                throttle_duration_sec=2.0,
            )
            return

        # Transform offset as a direction vector (use two points: origin + offset)
        origin_pose = PoseStamped()
        origin_pose.header.stamp = self.get_clock().now().to_msg()
        origin_pose.header.frame_id = CAMERA_OPTICAL_FRAME
        origin_pose.pose.position.x = 0.0
        origin_pose.pose.position.y = 0.0
        origin_pose.pose.position.z = 0.0
        origin_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        offset_pose = PoseStamped()
        offset_pose.header = origin_pose.header
        offset_pose.pose.position.x = float(offset_cam[0])
        offset_pose.pose.position.y = float(offset_cam[1])
        offset_pose.pose.position.z = float(offset_cam[2])
        offset_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        origin_base = do_transform_pose_stamped(origin_pose, tf_cam_to_base)
        offset_base = do_transform_pose_stamped(offset_pose, tf_cam_to_base)

        # Offset in robot frame = transformed_offset - transformed_origin
        dx = offset_base.pose.position.x - origin_base.pose.position.x
        dy = offset_base.pose.position.y - origin_base.pose.position.y
        dz = offset_base.pose.position.z - origin_base.pose.position.z

        # 3. Robot target = robot head ref + offset
        target_x = ROBOT_HEAD_REF[0] + dx
        target_y = ROBOT_HEAD_REF[1] + dy
        target_z = ROBOT_HEAD_REF[2] + dz

        # 4. Get current EE orientation (keep stable)
        ee_frame = EE_FRAMES[side]
        try:
            tf_ee = self._tf_buffer.lookup_transform(
                self._target_frame,
                ee_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except Exception as e:
            self.get_logger().warn(
                f"TF EE lookup failed ({ee_frame}): {e}",
                throttle_duration_sec=2.0,
            )
            return

        # 5. Publish PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self._target_frame
        pose_msg.pose.position.x = float(target_x)
        pose_msg.pose.position.y = float(target_y)
        pose_msg.pose.position.z = float(target_z)
        pose_msg.pose.orientation = Quaternion(
            x=tf_ee.transform.rotation.x,
            y=tf_ee.transform.rotation.y,
            z=tf_ee.transform.rotation.z,
            w=tf_ee.transform.rotation.w,
        )

        pub = self._left_pose_pub if is_left else self._right_pose_pub
        pub.publish(pose_msg)

        # Publish visualization marker for RViz
        marker = Marker()
        marker.header = pose_msg.header
        marker.ns = f"{side}_target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose_msg.pose
        marker.scale.x = 0.06
        marker.scale.y = 0.06
        marker.scale.z = 0.06
        marker.color.a = 0.8
        if is_left:
            marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0
        else:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
        marker_pub = self._left_marker_pub if is_left else self._right_marker_pub
        marker_pub.publish(marker)

        if self._debug_mode:
            self.get_logger().info(
                f"{side} offset=({dx:.3f},{dy:.3f},{dz:.3f}) "
                f"target=({target_x:.3f},{target_y:.3f},{target_z:.3f})",
                throttle_duration_sec=0.5,
            )

    # ------------------------------------------------------------------ #
    # Debug visualization
    # ------------------------------------------------------------------ #

    def _show_debug(self, frame: np.ndarray, results) -> None:
        display = frame.copy()

        if results and results.multi_hand_landmarks:
            for hand_lms in results.multi_hand_landmarks:
                self._mp_drawing.draw_landmarks(
                    display,
                    hand_lms,
                    mp.solutions.hands.HAND_CONNECTIONS,
                )

        # Status overlay
        status = "TRACKING" if self._tracking_enabled else "PAUSED"
        color = (0, 255, 0) if self._tracking_enabled else (0, 0, 255)
        cv2.putText(
            display, status, (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2,
        )

        l_status = "L:OK" if self._left_hand.detected else "L:--"
        r_status = "R:OK" if self._right_hand.detected else "R:--"
        cv2.putText(
            display, f"{l_status}  {r_status}", (10, 70),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2,
        )

        servo_l = "Servo L:OK" if self._servo_ready["left"] else "Servo L:WAIT"
        servo_r = "Servo R:OK" if self._servo_ready["right"] else "Servo R:WAIT"
        cv2.putText(
            display, f"{servo_l}  {servo_r}", (10, 110),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 0), 1,
        )

        cv2.imshow("Hand Tracking Teleop", display)

    def destroy_node(self) -> None:
        self._mp_hands.close()
        self._mp_face.close()
        cv2.destroyAllWindows()
        super().destroy_node()


# ---------------------------------------------------------------------- #
# Utility functions
# ---------------------------------------------------------------------- #


def _rotation_matrix_to_quaternion(rot: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to quaternion [x, y, z, w]."""
    trace = rot[0, 0] + rot[1, 1] + rot[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (rot[2, 1] - rot[1, 2]) * s
        y = (rot[0, 2] - rot[2, 0]) * s
        z = (rot[1, 0] - rot[0, 1]) * s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = 2.0 * math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2])
        w = (rot[2, 1] - rot[1, 2]) / s
        x = 0.25 * s
        y = (rot[0, 1] + rot[1, 0]) / s
        z = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = 2.0 * math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2])
        w = (rot[0, 2] - rot[2, 0]) / s
        x = (rot[0, 1] + rot[1, 0]) / s
        y = 0.25 * s
        z = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1])
        w = (rot[1, 0] - rot[0, 1]) / s
        x = (rot[0, 2] + rot[2, 0]) / s
        y = (rot[1, 2] + rot[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def _map_range(
    value: float,
    in_min: float,
    in_max: float,
    out_min: float,
    out_max: float,
) -> float:
    """Map value from input range to output range, clamped."""
    if in_max - in_min < 1e-9:
        return out_min
    t = (value - in_min) / (in_max - in_min)
    t = max(0.0, min(1.0, t))
    return out_min + t * (out_max - out_min)


def main() -> None:
    rclpy.init()
    node = ServoHandTrackingTeleop()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Spin ROS executor in a background thread so callbacks work
    import threading
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        # Run OpenCV loop in main thread (required for GUI)
        node.run_main_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
