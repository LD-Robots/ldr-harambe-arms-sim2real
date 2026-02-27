#!/usr/bin/env python3
"""
RealSense skeleton tracking teleop for the LDR humanoid arm.

Captures RGB frames from an Intel RealSense camera, runs MediaPipe Pose
to extract 3D body landmarks, computes joint angles for both arms,
and publishes sensor_msgs/msg/JointState.

The user faces the camera. The robot faces the user (face-to-face).
Left arm -> robot left arm, right arm -> robot right arm (body-relative).

Controls (OpenCV window):
    e : toggle tracking on/off
    c : start calibration routine
    0 : all joints mode (default)
    1-6 : isolate joint (1=shoulder_pitch .. 6=wrist_roll)
    t : toggle TEST POSE mode (robot leads, you mimic)
        w/s : increase/decrease test angle by 15 deg
        1-6 : select joint in test mode
    q : quit
"""

from __future__ import annotations

import math
import os
import sys
import threading

# Add site-packages paths for mediapipe and pyrealsense2
_EXTRA_PATHS = [
    os.path.expanduser("~/.local/lib/python3.12/site-packages"),
    os.path.expanduser("~/miniforge3/lib/python3.12/site-packages"),
    os.path.expanduser("~/.platformio/penv/lib/python3.12/site-packages"),
]
for p in _EXTRA_PATHS:
    if p not in sys.path and os.path.isdir(p):
        sys.path.insert(0, p)

import cv2
import mediapipe as mp
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from skeleton_calibration import (
    CalibrationManager,
    CalibrationProfile,
    apply_neutral_offset,
    apply_rom_mapping,
)

# MediaPipe Tasks API (0.10+)
BaseOptions = mp.tasks.BaseOptions
PoseLandmarker = mp.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
PoseLandmarksConnections = mp.tasks.vision.PoseLandmarksConnections
VisionRunningMode = mp.tasks.vision.RunningMode

# Pose landmark connections for drawing
_POSE_CONNECTIONS = [
    (c.start, c.end) for c in PoseLandmarksConnections.POSE_LANDMARKS
]


def _find_model_path() -> str:
    """Locate the pose_landmarker model file."""
    candidates = [
        os.path.join(os.path.dirname(__file__), "..", "models", "pose_landmarker_full.task"),
        os.path.join(os.path.dirname(__file__), "pose_landmarker_full.task"),
    ]
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("arm_teleop")
        candidates.append(os.path.join(share, "models", "pose_landmarker_full.task"))
    except Exception:
        pass

    for c in candidates:
        real = os.path.realpath(c)
        if os.path.isfile(real):
            return real

    raise FileNotFoundError(
        "Could not find pose_landmarker_full.task. "
        "Download it with:\n"
        "  curl -L -o src/teleop/arm_teleop/models/pose_landmarker_full.task "
        '"https://storage.googleapis.com/mediapipe-models/pose_landmarker/'
        'pose_landmarker_full/float16/latest/pose_landmarker_full.task"'
    )


class RealsenseSkeletonTeleop(Node):
    """Track user's arm skeletons and publish joint angles."""

    # Left arm joints
    LEFT_JOINT_NAMES = [
        "left_shoulder_pitch_joint_X6",
        "left_shoulder_roll_joint_X6",
        "left_shoulder_yaw_joint_X4",
        "left_elbow_pitch_joint_X6",
        "left_wrist_yaw_joint_X4",
        "left_wrist_roll_joint_X4",
    ]
    # Right arm joints
    RIGHT_JOINT_NAMES = [
        "right_shoulder_pitch_joint_X6",
        "right_shoulder_roll_joint_X6",
        "right_shoulder_yaw_joint_X4",
        "right_elbow_pitch_joint_X6",
        "right_wrist_yaw_joint_X4",
        "right_wrist_roll_joint_X4",
    ]
    ALL_JOINT_NAMES = LEFT_JOINT_NAMES + RIGHT_JOINT_NAMES

    JOINT_LABELS = [
        "L Shoulder pitch", "L Shoulder roll", "L Shoulder yaw",
        "L Elbow pitch", "L Wrist yaw", "L Wrist roll",
        "R Shoulder pitch", "R Shoulder roll", "R Shoulder yaw",
        "R Elbow pitch", "R Wrist yaw", "R Wrist roll",
    ]

    # URDF joint limits (rad)
    JOINT_LIMITS = {
        # Left arm
        "left_shoulder_pitch_joint_X6": (-3.02, 1.69),
        "left_shoulder_roll_joint_X6": (-0.21, 2.93),
        "left_shoulder_yaw_joint_X4": (-1.75, 2.97),
        "left_elbow_pitch_joint_X6": (-1.31, 1.83),
        "left_wrist_yaw_joint_X4": (-2.36, 2.36),
        "left_wrist_roll_joint_X4": (-1.57, 1.57),
        # Right arm (note: roll/yaw limits are inverted vs left)
        "right_shoulder_pitch_joint_X6": (-3.02, 1.69),
        "right_shoulder_roll_joint_X6": (-2.93, 0.21),
        "right_shoulder_yaw_joint_X4": (-2.97, 1.75),
        "right_elbow_pitch_joint_X6": (-1.31, 1.83),
        "right_wrist_yaw_joint_X4": (-2.36, 2.36),
        "right_wrist_roll_joint_X4": (-1.57, 1.57),
    }


    # MediaPipe landmark indices
    LM_LEFT_SHOULDER = 11
    LM_RIGHT_SHOULDER = 12
    LM_LEFT_ELBOW = 13
    LM_RIGHT_ELBOW = 14
    LM_LEFT_WRIST = 15
    LM_RIGHT_WRIST = 16
    LM_LEFT_PINKY = 17
    LM_RIGHT_PINKY = 18
    LM_LEFT_INDEX = 19
    LM_RIGHT_INDEX = 20
    LM_LEFT_HIP = 23
    LM_RIGHT_HIP = 24

    def __init__(self) -> None:
        super().__init__("realsense_skeleton_teleop")

        # Parameters
        self.declare_parameter("smoothing_alpha", 0.3)
        self.declare_parameter("gain", 1.0)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("enabled", True)
        self.declare_parameter("mirror_display", True)
        self.declare_parameter("show_debug_window", True)
        self.declare_parameter("min_detection_confidence", 0.7)
        self.declare_parameter("min_tracking_confidence", 0.5)
        self.declare_parameter("publish_topic", "/skeleton_joint_states")
        self.declare_parameter("calibration_profile", "")
        self.declare_parameter("auto_calibrate", False)

        self._alpha = self.get_parameter("smoothing_alpha").value
        self._gain = self.get_parameter("gain").value
        rate = self.get_parameter("publish_rate").value
        self._enabled = self.get_parameter("enabled").value
        self._mirror_display = self.get_parameter("mirror_display").value
        self._show_debug = self.get_parameter("show_debug_window").value
        det_conf = self.get_parameter("min_detection_confidence").value
        trk_conf = self.get_parameter("min_tracking_confidence").value
        pub_topic = self.get_parameter("publish_topic").value

        # State — 12 joints total (6 left + 6 right)
        n_joints = len(self.ALL_JOINT_NAMES)
        self._lock = threading.Lock()
        self._smoothed = [0.0] * n_joints
        self._last_angles = [0.0] * n_joints
        self._frame_count = 0

        # Joint isolation mode: -1 = all joints, 0..5 = isolate that per-arm joint index
        self._isolated_joint = -1

        # Test pose mode: robot leads, human mimics
        self._test_mode = False
        self._test_joint = 3       # default to elbow_pitch (index 3)
        self._test_angle = 0.0     # target angle in rad (right arm)
        self._test_measured = [0.0] * 6  # raw measured right arm angles

        # Calibration
        self._calibration: CalibrationProfile | None = None
        self._calib_manager = CalibrationManager(
            self.ALL_JOINT_NAMES, self.JOINT_LIMITS, self.get_logger()
        )
        self._calibrating = False

        # Load calibration profile if specified
        calib_param = self.get_parameter("calibration_profile").value
        if calib_param:
            profile = CalibrationManager.load_profile(calib_param)
            if profile is None:
                # Try as a name in the default directory
                path = os.path.join(
                    CalibrationManager.get_profile_dir(), f"{calib_param}.yaml"
                )
                profile = CalibrationManager.load_profile(path)
            if profile:
                self._calibration = profile
                self.get_logger().info(
                    f"Loaded calibration profile: {profile.profile_name}"
                )
            else:
                self.get_logger().warn(
                    f"Could not load calibration profile: {calib_param}"
                )

        if self.get_parameter("auto_calibrate").value:
            self._calibrating = True

        # Publisher
        self._pub = self.create_publisher(JointState, pub_topic, 10)

        # Recalibrate service
        self._recalib_srv = self.create_service(
            Trigger, "~/recalibrate", self._recalibrate_callback
        )

        # RealSense pipeline
        self._rs_pipeline = rs.pipeline()
        rs_config = rs.config()
        rs_config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
        try:
            self._rs_pipeline.start(rs_config)
        except RuntimeError as e:
            self.get_logger().fatal(f"Failed to start RealSense: {e}")
            raise

        # MediaPipe PoseLandmarker (Tasks API)
        model_path = _find_model_path()
        self.get_logger().info(f"Loading model: {model_path}")
        options = PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=VisionRunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=det_conf,
            min_pose_presence_confidence=det_conf,
            min_tracking_confidence=trk_conf,
        )
        self._landmarker = PoseLandmarker.create_from_options(options)

        # Timer
        self._timer = self.create_timer(1.0 / rate, self._timer_callback)

        calib_status = "auto-calibrate on start" if self._calibrating else (
            f"loaded '{self._calibration.profile_name}'" if self._calibration
            else "none (press 'c' to calibrate)"
        )
        self.get_logger().info(
            f"Skeleton teleop started (both arms)\n"
            f"  Topic: {pub_topic}\n"
            f"  Rate: {rate} Hz\n"
            f"  Smoothing: {self._alpha}\n"
            f"  Mirror display: {self._mirror_display}\n"
            f"  Calibration: {calib_status}\n"
            f"  Press 'e' to toggle, 'c' to calibrate, 'q' to quit"
        )

    # ------------------------------------------------------------------ #
    # Timer callback
    # ------------------------------------------------------------------ #
    def _timer_callback(self) -> None:
        frames = self._rs_pipeline.poll_for_frames()
        if not frames:
            return
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        image = np.asanyarray(color_frame.get_data())

        # IMPORTANT: Do NOT mirror for MediaPipe — it must see the real
        # image so LEFT landmarks = user's actual LEFT arm.
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

        self._frame_count += 1
        timestamp_ms = int(self._frame_count * (1000.0 / 30.0))
        result = self._landmarker.detect_for_video(mp_image, timestamp_ms)

        has_pose = result.pose_world_landmarks and len(result.pose_world_landmarks) > 0

        if has_pose:
            wl = result.pose_world_landmarks[0]
            lm = self._extract_all_landmarks(wl)

            # Compute angles for each arm
            left_angles = self._compute_arm_angles(lm, is_right=False)
            right_angles = self._compute_arm_angles(lm, is_right=True)
            raw_angles = left_angles + right_angles

            # Calibration mode: feed frames to the state machine
            if self._calibrating:
                if not self._calib_manager.is_active():
                    self._calib_manager.start(
                        mirror_display=self._mirror_display,
                    )
                self._calib_manager.process_frame(lm, raw_angles)
                if self._calib_manager.is_complete():
                    self._calibration = self._calib_manager.get_result()
                    self._calibrating = False
                    # Save to user profile directory
                    save_path = os.path.join(
                        CalibrationManager.get_profile_dir(),
                        f"{self._calibration.profile_name}.yaml",
                    )
                    CalibrationManager.save_profile(self._calibration, save_path)
                    self.get_logger().info(f"Calibration saved to {save_path}")
                elif self._calib_manager.is_failed():
                    self._calibrating = False
                    self.get_logger().warn("Calibration failed — press 'c' to retry")
            elif self._test_mode:
                # TEST POSE MODE: robot goes to fixed pose, we measure human
                self._test_measured = list(right_angles)

                # Build command: only the selected joint on the right arm
                cmd = [0.0] * len(self.ALL_JOINT_NAMES)
                right_idx = self._test_joint + 6  # right arm offset
                cmd[right_idx] = self._test_angle

                # No smoothing in test mode — direct command
                clamped = self._clamp_to_limits(cmd)
                with self._lock:
                    self._last_angles = list(clamped)
                if self._enabled:
                    self._publish(clamped)
            else:
                # Apply calibration if available
                if self._calibration:
                    offset = apply_neutral_offset(
                        raw_angles, self.ALL_JOINT_NAMES, self._calibration
                    )
                    mapped = apply_rom_mapping(
                        offset, self.ALL_JOINT_NAMES, self._calibration
                    )
                else:
                    mapped = raw_angles

                # Joint isolation: zero out all joints except the isolated one
                if self._isolated_joint >= 0:
                    isolated = [0.0] * len(mapped)
                    j = self._isolated_joint       # left arm index
                    jr = self._isolated_joint + 6  # right arm index
                    if j < len(mapped):
                        isolated[j] = mapped[j]
                    if jr < len(mapped):
                        isolated[jr] = mapped[jr]
                    mapped = isolated

                smoothed = self._apply_smoothing(mapped)
                scaled = [a * self._gain for a in smoothed]
                clamped = self._clamp_to_limits(scaled)

                with self._lock:
                    self._last_angles = list(clamped)

                if self._enabled:
                    self._publish(clamped)

        # Debug visualization — mirror the display for natural feel
        if self._show_debug:
            display_img = image.copy()
            if self._mirror_display:
                display_img = cv2.flip(display_img, 1)
            self._draw_debug(display_img, result)
            if self._calibrating or self._calib_manager.is_complete():
                self._calib_manager.draw_overlay(display_img)
            cv2.imshow("Skeleton Teleop", display_img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("e"):
                self._enabled = not self._enabled
                self.get_logger().info(
                    f"Tracking {'enabled' if self._enabled else 'paused'}"
                )
            elif key == ord("t"):
                self._test_mode = not self._test_mode
                if self._test_mode:
                    self._test_angle = 0.0
                    self._smoothed = [0.0] * len(self.ALL_JOINT_NAMES)
                    label = self.JOINT_LABELS[self._test_joint + 6]
                    self.get_logger().info(
                        f"TEST MODE ON — right arm, joint: {label}\n"
                        f"  w/s = +/- 15 deg, 1-6 = select joint, t = exit"
                    )
                else:
                    self._smoothed = [0.0] * len(self.ALL_JOINT_NAMES)
                    self.get_logger().info("TEST MODE OFF — back to teleop")
            elif key == ord("c") and not self._test_mode:
                self._calibrating = True
                self.get_logger().info("Starting calibration...")
            elif key == ord("w") and self._test_mode:
                self._test_angle += math.radians(15)
                name = self.RIGHT_JOINT_NAMES[self._test_joint]
                lo, hi = self.JOINT_LIMITS[name]
                self._test_angle = min(self._test_angle, hi)
                self.get_logger().info(
                    f"Test angle: {math.degrees(self._test_angle):+.0f} deg"
                )
            elif key == ord("s") and self._test_mode:
                self._test_angle -= math.radians(15)
                name = self.RIGHT_JOINT_NAMES[self._test_joint]
                lo, hi = self.JOINT_LIMITS[name]
                self._test_angle = max(self._test_angle, lo)
                self.get_logger().info(
                    f"Test angle: {math.degrees(self._test_angle):+.0f} deg"
                )
            elif key == ord("0") and not self._test_mode:
                self._isolated_joint = -1
                self._smoothed = [0.0] * len(self.ALL_JOINT_NAMES)
                self.get_logger().info("All joints mode")
            elif key in (ord("1"), ord("2"), ord("3"), ord("4"), ord("5"), ord("6")):
                idx = key - ord("1")  # 0..5
                if self._test_mode:
                    self._test_joint = idx
                    self._test_angle = 0.0
                    label = self.JOINT_LABELS[idx + 6]
                    self.get_logger().info(f"Test joint: {label}")
                else:
                    self._isolated_joint = idx
                    self._smoothed = [0.0] * len(self.ALL_JOINT_NAMES)
                    label = self.JOINT_LABELS[idx]
                    self.get_logger().info(
                        f"Isolated joint: {label} (+ right counterpart)"
                    )
            elif key == ord("q"):
                self.get_logger().info("Quit requested")
                rclpy.shutdown()

    # ------------------------------------------------------------------ #
    # Landmark extraction
    # ------------------------------------------------------------------ #
    def _extract_all_landmarks(self, world_landmarks) -> dict:
        """Extract all needed landmarks as {index: np.array([x,y,z])}."""
        needed = [
            self.LM_LEFT_SHOULDER, self.LM_RIGHT_SHOULDER,
            self.LM_LEFT_ELBOW, self.LM_RIGHT_ELBOW,
            self.LM_LEFT_WRIST, self.LM_RIGHT_WRIST,
            self.LM_LEFT_PINKY, self.LM_RIGHT_PINKY,
            self.LM_LEFT_INDEX, self.LM_RIGHT_INDEX,
            self.LM_LEFT_HIP, self.LM_RIGHT_HIP,
        ]
        out = {}
        for idx in needed:
            lm = world_landmarks[idx]
            out[idx] = np.array([lm.x, lm.y, lm.z])
        return out

    # ------------------------------------------------------------------ #
    # Joint angle math
    # ------------------------------------------------------------------ #
    @staticmethod
    def _angle_between(v1: np.ndarray, v2: np.ndarray) -> float:
        """Unsigned angle (rad) between two 3D vectors."""
        n1 = np.linalg.norm(v1)
        n2 = np.linalg.norm(v2)
        if n1 < 1e-6 or n2 < 1e-6:
            return 0.0
        cos_a = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
        return float(np.arccos(cos_a))

    @staticmethod
    def _signed_angle(v1: np.ndarray, v2: np.ndarray, axis: np.ndarray) -> float:
        """Signed angle from v1 to v2 about axis (right-hand rule)."""
        v1p = v1 - np.dot(v1, axis) * axis
        v2p = v2 - np.dot(v2, axis) * axis
        n1 = np.linalg.norm(v1p)
        n2 = np.linalg.norm(v2p)
        if n1 < 1e-6 or n2 < 1e-6:
            return 0.0
        cos_a = np.clip(np.dot(v1p, v2p) / (n1 * n2), -1.0, 1.0)
        angle = float(np.arccos(cos_a))
        if np.dot(np.cross(v1p, v2p), axis) < 0:
            angle = -angle
        return angle

    def _compute_arm_angles(self, lm: dict, is_right: bool) -> list:
        """
        Compute 6 joint angles for one arm from MediaPipe world landmarks.

        MediaPipe world coordinate system (person facing camera):
            X = person's left, Y = downward, Z = toward camera

        The same math works for both arms — the vectors naturally point in
        opposite directions for left vs right, producing opposite-sign
        angles that match the URDF's symmetric limit convention.
        """
        if is_right:
            shoulder = lm[self.LM_RIGHT_SHOULDER]
            other_shoulder = lm[self.LM_LEFT_SHOULDER]
            elbow = lm[self.LM_RIGHT_ELBOW]
            wrist = lm[self.LM_RIGHT_WRIST]
            pinky = lm[self.LM_RIGHT_PINKY]
            index = lm[self.LM_RIGHT_INDEX]
        else:
            shoulder = lm[self.LM_LEFT_SHOULDER]
            other_shoulder = lm[self.LM_RIGHT_SHOULDER]
            elbow = lm[self.LM_LEFT_ELBOW]
            wrist = lm[self.LM_LEFT_WRIST]
            pinky = lm[self.LM_LEFT_PINKY]
            index = lm[self.LM_LEFT_INDEX]

        upper_arm = elbow - shoulder
        forearm = wrist - elbow
        # Points from other shoulder toward this shoulder
        shoulder_across = shoulder - other_shoulder

        down = np.array([0.0, 1.0, 0.0])  # MediaPipe Y is down

        # --- shoulder_pitch (Y axis): sagittal plane rotation ----------
        # Project upper arm onto YZ plane, measure angle from downward.
        # Positive = forward swing. Same for both arms.
        ua_yz = np.array([0.0, upper_arm[1], upper_arm[2]])
        shoulder_pitch = self._signed_angle(
            down, ua_yz, np.array([1.0, 0.0, 0.0])
        )

        # --- shoulder_roll (X axis): lateral abduction -----------------
        # Project upper arm onto XY plane, measure from downward.
        # URDF: left abduction = positive (limits up to +2.93),
        #        right abduction = negative (limits down to -2.93).
        ua_xy = np.array([upper_arm[0], upper_arm[1], 0.0])
        shoulder_roll = -self._signed_angle(
            down, ua_xy, np.array([0.0, 0.0, 1.0])
        )

        # --- shoulder_yaw (Z axis): axial rotation ---------------------
        # Observed via forearm direction when elbow is bent.
        # shoulder_across points outward from body for this arm.
        ua_len = np.linalg.norm(upper_arm)
        if ua_len > 1e-6:
            ua_unit = upper_arm / ua_len
            ref = shoulder_across - np.dot(shoulder_across, ua_unit) * ua_unit
            ref_n = np.linalg.norm(ref)
            fa_proj = forearm - np.dot(forearm, ua_unit) * ua_unit
            if ref_n > 1e-6 and np.linalg.norm(fa_proj) > 1e-6:
                shoulder_yaw = -self._signed_angle(
                    ref / ref_n, fa_proj, ua_unit
                )
            else:
                shoulder_yaw = 0.0
        else:
            shoulder_yaw = 0.0

        # --- elbow_pitch (Y axis): flexion/extension -------------------
        # Angle between upper_arm and forearm: 0 = straight, increases with bend.
        # Negated because URDF flexion = negative.
        elbow_pitch = -self._angle_between(upper_arm, forearm)

        # --- wrist_yaw (Z axis): forearm pronation/supination ----------
        fa_len = np.linalg.norm(forearm)
        if fa_len > 1e-6:
            fa_unit = forearm / fa_len
            hand_across = index - pinky
            hand_proj = hand_across - np.dot(hand_across, fa_unit) * fa_unit
            ua_fa_proj = (-upper_arm) - np.dot(-upper_arm, fa_unit) * fa_unit
            hp_n = np.linalg.norm(hand_proj)
            ufp_n = np.linalg.norm(ua_fa_proj)
            if hp_n > 1e-6 and ufp_n > 1e-6:
                wrist_yaw = self._signed_angle(
                    ua_fa_proj / ufp_n, hand_proj / hp_n, fa_unit
                )
            else:
                wrist_yaw = 0.0
        else:
            wrist_yaw = 0.0

        # --- wrist_roll (X axis): wrist flexion/extension --------------
        hand_center = (pinky + index) / 2.0
        hand_dir = hand_center - wrist
        wrist_roll = self._angle_between(forearm, hand_dir)
        wrist_roll = float(np.clip(wrist_roll, -0.5, 0.5))

        return [
            shoulder_pitch,
            shoulder_roll,
            shoulder_yaw,
            elbow_pitch,
            wrist_yaw,
            wrist_roll,
        ]

    # ------------------------------------------------------------------ #
    # Smoothing / clamping / publishing
    # ------------------------------------------------------------------ #
    def _apply_smoothing(self, raw: list) -> list:
        a = self._alpha
        for i in range(len(raw)):
            self._smoothed[i] = a * raw[i] + (1.0 - a) * self._smoothed[i]
        return list(self._smoothed)

    def _clamp_to_limits(self, angles: list) -> list:
        clamped = []
        for name, val in zip(self.ALL_JOINT_NAMES, angles):
            lo, hi = self.JOINT_LIMITS[name]
            clamped.append(max(lo, min(hi, val)))
        return clamped

    def _publish(self, angles: list) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.ALL_JOINT_NAMES)
        msg.position = list(angles)
        self._pub.publish(msg)

    # ------------------------------------------------------------------ #
    # Debug visualization
    # ------------------------------------------------------------------ #
    def _draw_debug(self, image: np.ndarray, result) -> None:
        h, w = image.shape[:2]
        has_pose = result.pose_landmarks and len(result.pose_landmarks) > 0

        if has_pose:
            landmarks = result.pose_landmarks[0]
            for start_idx, end_idx in _POSE_CONNECTIONS:
                pt1 = landmarks[start_idx]
                pt2 = landmarks[end_idx]
                x1 = int(pt1.x * w)
                y1 = int(pt1.y * h)
                x2 = int(pt2.x * w)
                y2 = int(pt2.y * h)
                if self._mirror_display:
                    x1 = w - 1 - x1
                    x2 = w - 1 - x2
                cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            for lm in landmarks:
                cx = int(lm.x * w)
                cy = int(lm.y * h)
                if self._mirror_display:
                    cx = w - 1 - cx
                cv2.circle(image, (cx, cy), 4, (0, 0, 255), -1)

        # Joint angle readout — left column for left arm, right for right
        with self._lock:
            angles = list(self._last_angles)

        for i in range(6):
            is_active = (self._isolated_joint == -1 or self._isolated_joint == i)
            # Left arm on left side
            l_color = (0, 255, 0) if is_active else (80, 80, 80)
            marker = ">" if (self._isolated_joint == i) else " "
            text = f"{marker}{self.JOINT_LABELS[i]}: {math.degrees(angles[i]):+6.1f}"
            cv2.putText(
                image, text, (10, 25 + i * 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, l_color, 1,
            )
            # Right arm on right side
            j = i + 6
            r_color = (255, 200, 0) if is_active else (80, 80, 80)
            text = f"{marker}{self.JOINT_LABELS[j]}: {math.degrees(angles[j]):+6.1f}"
            cv2.putText(
                image, text, (w - 280, 25 + i * 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, r_color, 1,
            )

        # Status
        status = "TRACKING" if self._enabled else "PAUSED"
        color = (0, 255, 0) if self._enabled else (0, 0, 255)
        cv2.putText(
            image, status, (10, h - 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2,
        )

        # Joint isolation indicator
        if self._isolated_joint >= 0 and not self._test_mode:
            iso_label = self.JOINT_LABELS[self._isolated_joint].split(" ", 1)[1]
            iso_text = f"ISOLATED: {iso_label} [press 0 for all]"
            cv2.putText(
                image, iso_text, (10, h - 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2,
            )

        # Test pose mode overlay
        if self._test_mode:
            joint_label = self.JOINT_LABELS[self._test_joint + 6]
            target_deg = math.degrees(self._test_angle)
            measured_deg = math.degrees(self._test_measured[self._test_joint])

            # Banner
            overlay = image.copy()
            cv2.rectangle(overlay, (0, h - 160), (w, h), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)

            cv2.putText(
                image, "TEST POSE MODE (right arm)", (10, h - 135),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2,
            )
            cv2.putText(
                image, f"Joint: {joint_label}  [1-6 to change, w/s +/-15deg]",
                (10, h - 110),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1,
            )

            # Target vs measured comparison
            cv2.putText(
                image, f"ROBOT TARGET:  {target_deg:+6.1f} deg", (10, h - 75),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2,
            )
            cv2.putText(
                image, f"YOUR MEASURED: {measured_deg:+6.1f} deg", (10, h - 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
            )

            # Sign match indicator
            if abs(target_deg) > 5:
                same_sign = (target_deg > 0) == (measured_deg > 0)
                if same_sign and abs(measured_deg) > 5:
                    match_text = "SIGN OK"
                    match_color = (0, 255, 0)
                elif abs(measured_deg) < 5:
                    match_text = "MOVE TO MATCH"
                    match_color = (200, 200, 200)
                else:
                    match_text = "SIGN INVERTED!"
                    match_color = (0, 0, 255)
                cv2.putText(
                    image, match_text, (w - 280, h - 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, match_color, 2,
                )

        if not has_pose:
            cv2.putText(
                image, "No skeleton detected", (10, h // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2,
            )

    # ------------------------------------------------------------------ #
    # Service callback
    # ------------------------------------------------------------------ #
    def _recalibrate_callback(self, request, response):
        self._calibrating = True
        self.get_logger().info("Recalibration triggered via service")
        response.success = True
        response.message = "Calibration started"
        return response

    # ------------------------------------------------------------------ #
    # Cleanup
    # ------------------------------------------------------------------ #
    def destroy_node(self) -> None:
        self._rs_pipeline.stop()
        self._landmarker.close()
        cv2.destroyAllWindows()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = RealsenseSkeletonTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
