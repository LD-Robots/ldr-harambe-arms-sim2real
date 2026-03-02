#!/usr/bin/env python3
"""
Robot-pose-copy calibration for skeleton teleop.

The robot in RViz moves through a sequence of predefined poses.
The user copies each pose while facing the camera. The script captures
the human's MediaPipe skeleton angles at each pose and builds a
calibration profile that maps human motion to robot motion.

The resulting YAML profile is compatible with the existing skeleton
teleop pipeline (apply_neutral_offset + apply_rom_mapping).

Usage:
    ros2 launch arm_teleop robot_pose_calibration.launch.py
    # or standalone (requires robot_state_publisher + joint_state_publisher):
    ros2 run arm_teleop robot_pose_calibration.py

Controls (OpenCV window):
    SPACE : skip capture early (use what's collected)
    r     : redo current pose
    q     : quit
"""

from __future__ import annotations

import math
import os
import sys
import time
from datetime import datetime

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

from skeleton_calibration import (
    CalibrationManager,
    CalibrationProfile,
    JointCalibration,
)

# MediaPipe Tasks API
BaseOptions = mp.tasks.BaseOptions
PoseLandmarker = mp.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
PoseLandmarksConnections = mp.tasks.vision.PoseLandmarksConnections
VisionRunningMode = mp.tasks.vision.RunningMode

_POSE_CONNECTIONS = [
    (c.start, c.end) for c in PoseLandmarksConnections.POSE_LANDMARKS
]

# ------------------------------------------------------------------ #
# Calibration poses
# ------------------------------------------------------------------ #
# Joint order per arm: [pitch, roll, yaw, elbow_pitch, wrist_yaw, wrist_roll]
# 12 total: left[0:6] + right[6:12]
#
# URDF conventions (from testing):
#   shoulder_pitch: positive = forward swing
#   shoulder_roll:  left positive = abduction,  right negative = abduction
#   shoulder_yaw:   left negative = outward rotation, right positive = outward
#   elbow_pitch:    negative = flexion (bend)
#   wrist_yaw/roll: symmetric limits

CALIBRATION_POSES = [
    {
        "name": "Neutral",
        "desc": "Arms relaxed at your sides",
        "angles": [0.0] * 12,
    },
    {
        "name": "Arms Forward 45\u00b0",
        "desc": "Raise both arms straight forward ~45 degrees",
        #         L_pitch                              R_pitch
        "angles": [0.785, 0, 0, 0, 0, 0, 0.785, 0, 0, 0, 0, 0],
    },
    {
        "name": "Arms Back 30\u00b0",
        "desc": "Swing both arms straight backward ~30 degrees",
        "angles": [-0.524, 0, 0, 0, 0, 0, -0.524, 0, 0, 0, 0, 0],
    },
    {
        "name": "Arms Out 45\u00b0",
        "desc": "Raise both arms out to the sides ~45 degrees",
        #         L_roll=+0.785                      R_roll=-0.785
        "angles": [0, 0.785, 0, 0, 0, 0, 0, -0.785, 0, 0, 0, 0],
    },
    {
        "name": "Elbows Bent 60\u00b0",
        "desc": "Bend both elbows ~60 degrees",
        #         L_elbow=-1.047                     R_elbow=-1.047
        "angles": [0, 0, 0, -1.047, 0, 0, 0, 0, 0, -1.047, 0, 0],
    },
    {
        "name": "Forearms Rotated Out",
        "desc": "Elbows bent, rotate forearms outward",
        #         L_yaw=-0.524 L_elbow=-1.047         R_yaw=+0.524 R_elbow=-1.047
        "angles": [0, 0, -0.524, -1.047, 0, 0, 0, 0, 0.524, -1.047, 0, 0],
    },
    {
        "name": "Forearms Rotated In",
        "desc": "Elbows bent, rotate forearms inward",
        #         L_yaw=+0.524 L_elbow=-1.047         R_yaw=-0.524 R_elbow=-1.047
        "angles": [0, 0, 0.524, -1.047, 0, 0, 0, 0, -0.524, -1.047, 0, 0],
    },
]


def _find_model_path() -> str:
    """Locate the pose_landmarker model file."""
    candidates = [
        os.path.join(
            os.path.dirname(__file__), "..", "models", "pose_landmarker_full.task"
        ),
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


# ------------------------------------------------------------------ #
# Main node
# ------------------------------------------------------------------ #


class RobotPoseCalibration(Node):
    """Calibration node: robot shows poses in RViz, human copies them."""

    LEFT_JOINT_NAMES = [
        "left_shoulder_pitch_joint_X6",
        "left_shoulder_roll_joint_X6",
        "left_shoulder_yaw_joint_X4",
        "left_elbow_pitch_joint_X6",
        "left_wrist_yaw_joint_X4",
        "left_wrist_roll_joint_X4",
    ]
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
        "L Pitch", "L Roll", "L Yaw", "L Elbow", "L WrYaw", "L WrRoll",
        "R Pitch", "R Roll", "R Yaw", "R Elbow", "R WrYaw", "R WrRoll",
    ]

    JOINT_LIMITS = {
        "left_shoulder_pitch_joint_X6": (-3.02, 1.69),
        "left_shoulder_roll_joint_X6": (-0.21, 2.93),
        "left_shoulder_yaw_joint_X4": (-1.75, 2.97),
        "left_elbow_pitch_joint_X6": (-1.31, 1.83),
        "left_wrist_yaw_joint_X4": (-2.36, 2.36),
        "left_wrist_roll_joint_X4": (-1.57, 1.57),
        "right_shoulder_pitch_joint_X6": (-3.02, 1.69),
        "right_shoulder_roll_joint_X6": (-2.93, 0.21),
        "right_shoulder_yaw_joint_X4": (-2.97, 1.75),
        "right_elbow_pitch_joint_X6": (-1.31, 1.83),
        "right_wrist_yaw_joint_X4": (-2.36, 2.36),
        "right_wrist_roll_joint_X4": (-1.57, 1.57),
    }

    # Timings
    TRANSITION_TIME = 3.0  # seconds to show pose before capture
    CAPTURE_TIME = 3.0  # seconds to average human angles
    DETECTION_TIMEOUT = 10.0

    def __init__(self) -> None:
        super().__init__("robot_pose_calibration")

        # Parameters
        self.declare_parameter("profile_name", "pose_calibrated")
        self.declare_parameter("mirror_display", True)
        self.declare_parameter("publish_topic", "/skeleton_joint_states")

        self._profile_name = self.get_parameter("profile_name").value
        self._mirror = self.get_parameter("mirror_display").value
        pub_topic = self.get_parameter("publish_topic").value

        # State machine
        self._state = "detection"  # detection|transition|capture|complete|failed
        self._pose_idx = 0
        self._state_start = time.time()
        self._frame_count = 0
        self._detection_frames = 0

        # Capture buffers
        self._angle_buffer: list[list[float]] = []
        self._pose_results: list[dict] = []  # {"robot": [12], "human": [12]}

        # Publisher
        self._pub = self.create_publisher(JointState, pub_topic, 10)

        # RealSense
        self._rs_pipeline = rs.pipeline()
        rs_config = rs.config()
        rs_config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
        try:
            self._rs_pipeline.start(rs_config)
        except RuntimeError as e:
            self.get_logger().fatal(f"Failed to start RealSense: {e}")
            raise

        # MediaPipe PoseLandmarker
        model_path = _find_model_path()
        self.get_logger().info(f"Loading model: {model_path}")
        options = PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=VisionRunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=0.7,
            min_pose_presence_confidence=0.7,
            min_tracking_confidence=0.5,
        )
        self._landmarker = PoseLandmarker.create_from_options(options)

        # Timer
        self._timer = self.create_timer(1.0 / 30.0, self._timer_callback)

        self.get_logger().info(
            f"Robot Pose Calibration started\n"
            f"  Profile: {self._profile_name}\n"
            f"  {len(CALIBRATION_POSES)} poses to capture\n"
            f"  Stand facing the camera and copy the robot's pose in RViz"
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
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

        self._frame_count += 1
        ts = int(self._frame_count * (1000.0 / 30.0))
        result = self._landmarker.detect_for_video(mp_image, ts)

        has_pose = (
            result.pose_world_landmarks and len(result.pose_world_landmarks) > 0
        )
        elapsed = time.time() - self._state_start

        # Compute human angles if skeleton detected
        human_angles = None
        if has_pose:
            wl = result.pose_world_landmarks[0]
            lm = self._extract_landmarks(wl)
            left = self._compute_arm_angles(lm, is_right=False)
            right = self._compute_arm_angles(lm, is_right=True)
            human_angles = left + right

        # Publish robot target pose to RViz
        if self._state in ("transition", "capture"):
            self._publish_robot_pose(CALIBRATION_POSES[self._pose_idx]["angles"])
        elif self._state == "detection":
            self._publish_robot_pose([0.0] * 12)

        # State machine
        if self._state == "detection":
            if has_pose:
                self._detection_frames += 1
            else:
                self._detection_frames = 0
            if self._detection_frames >= 15:
                self._pose_idx = 0
                self._transition("transition")
                self.get_logger().info("Skeleton detected — starting calibration")
            elif elapsed > self.DETECTION_TIMEOUT:
                self._state = "failed"
                self.get_logger().error("Detection timed out")

        elif self._state == "transition":
            if elapsed >= self.TRANSITION_TIME:
                self._transition("capture")

        elif self._state == "capture":
            if human_angles is not None:
                self._angle_buffer.append(human_angles)

            if elapsed >= self.CAPTURE_TIME:
                self._finalize_pose()

        # Display
        display = image.copy()
        if self._mirror:
            display = cv2.flip(display, 1)
        self._draw_skeleton(display, result)
        self._draw_overlay(display, human_angles)
        cv2.imshow("Robot Pose Calibration", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self.get_logger().info("Quit requested")
            rclpy.shutdown()
        elif key == ord("r") and self._state in ("transition", "capture"):
            self._angle_buffer.clear()
            self._transition("transition")
            self.get_logger().info("Redoing current pose")
        elif key == ord(" ") and self._state == "capture":
            self._finalize_pose()

    # ------------------------------------------------------------------ #
    # State helpers
    # ------------------------------------------------------------------ #

    def _transition(self, new_state: str) -> None:
        self._state = new_state
        self._state_start = time.time()
        self._angle_buffer.clear()

    def _finalize_pose(self) -> None:
        """Average captured angles and advance to next pose."""
        pose = CALIBRATION_POSES[self._pose_idx]

        if not self._angle_buffer:
            self.get_logger().warn(
                f"No skeleton data for '{pose['name']}' — using zeros"
            )
            avg = [0.0] * 12
        else:
            arr = np.array(self._angle_buffer)
            avg = np.median(arr, axis=0).tolist()

        self._pose_results.append({
            "robot": list(pose["angles"]),
            "human": avg,
        })
        self.get_logger().info(
            f"  Pose '{pose['name']}' captured "
            f"({len(self._angle_buffer)} frames)"
        )

        self._pose_idx += 1
        if self._pose_idx >= len(CALIBRATION_POSES):
            self._compute_and_save()
        else:
            self._transition("transition")

    # ------------------------------------------------------------------ #
    # Calibration computation
    # ------------------------------------------------------------------ #

    def _compute_and_save(self) -> None:
        """Build a CalibrationProfile from the collected pose data."""
        if not self._pose_results:
            self._state = "failed"
            return

        neutral_human = self._pose_results[0]["human"]

        profile = CalibrationProfile(
            profile_name=self._profile_name,
            timestamp=datetime.now().isoformat(),
            mirror_display=self._mirror,
            frames_averaged=sum(1 for _ in self._pose_results),
        )

        for j, name in enumerate(self.ALL_JOINT_NAMES):
            lo, hi = self.JOINT_LIMITS[name]
            neutral = neutral_human[j]

            # Collect scale factors from non-neutral poses
            pos_scales: list[float] = []
            neg_scales: list[float] = []

            for res in self._pose_results[1:]:
                robot_target = res["robot"][j]
                human_measured = res["human"][j]
                corrected = human_measured - neutral

                if abs(robot_target) < 0.01:
                    continue  # this pose doesn't test this joint

                if abs(corrected) < 0.03:
                    self.get_logger().warn(
                        f"  {name}: human didn't move for target "
                        f"{math.degrees(robot_target):.0f} deg"
                    )
                    continue

                scale = robot_target / corrected
                if scale < 0:
                    self.get_logger().warn(
                        f"  {name}: sign mismatch! target={robot_target:.2f}, "
                        f"corrected={corrected:.2f}"
                    )
                    continue

                if robot_target > 0:
                    pos_scales.append(scale)
                else:
                    neg_scales.append(scale)

            # Derive human_min/human_max so ROM mapping produces the right scale
            robot_pos_range = abs(hi)
            robot_neg_range = abs(lo)

            if pos_scales:
                avg_scale = float(np.median(pos_scales))
                # Clamp scale to reasonable range [0.2, 5.0]
                avg_scale = max(0.2, min(5.0, avg_scale))
                human_pos_range = robot_pos_range / avg_scale
            else:
                human_pos_range = robot_pos_range  # 1:1

            if neg_scales:
                avg_scale = float(np.median(neg_scales))
                avg_scale = max(0.2, min(5.0, avg_scale))
                human_neg_range = robot_neg_range / avg_scale
            else:
                human_neg_range = robot_neg_range  # 1:1

            profile.joints[name] = JointCalibration(
                neutral_angle=neutral,
                human_min=neutral - human_neg_range,
                human_max=neutral + human_pos_range,
                robot_min=lo,
                robot_max=hi,
            )

            self.get_logger().info(
                f"  {name}: neutral={math.degrees(neutral):+.1f} deg, "
                f"pos_scale={'x'.join(f'{s:.2f}' for s in pos_scales) or '1:1'}, "
                f"neg_scale={'x'.join(f'{s:.2f}' for s in neg_scales) or '1:1'}"
            )

        # Save
        save_dir = CalibrationManager.get_profile_dir()
        save_path = os.path.join(save_dir, f"{self._profile_name}.yaml")
        CalibrationManager.save_profile(profile, save_path)
        self.get_logger().info(f"Calibration saved to {save_path}")
        self.get_logger().info(
            f"Use with teleop:\n"
            f"  ros2 launch arm_teleop realsense_skeleton_teleop.launch.py "
            f"calibration_profile:={self._profile_name}"
        )

        self._state = "complete"

    # ------------------------------------------------------------------ #
    # Joint state publishing
    # ------------------------------------------------------------------ #

    def _publish_robot_pose(self, angles: list[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.ALL_JOINT_NAMES)
        msg.position = list(angles)
        self._pub.publish(msg)

    # ------------------------------------------------------------------ #
    # Angle computation (same as realsense_skeleton_teleop.py)
    # ------------------------------------------------------------------ #

    @staticmethod
    def _angle_between(v1: np.ndarray, v2: np.ndarray) -> float:
        n1 = np.linalg.norm(v1)
        n2 = np.linalg.norm(v2)
        if n1 < 1e-6 or n2 < 1e-6:
            return 0.0
        cos_a = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
        return float(np.arccos(cos_a))

    @staticmethod
    def _signed_angle(v1: np.ndarray, v2: np.ndarray, axis: np.ndarray) -> float:
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

    def _extract_landmarks(self, world_landmarks) -> dict:
        needed = [11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 23, 24]
        out = {}
        for idx in needed:
            lm = world_landmarks[idx]
            out[idx] = np.array([lm.x, lm.y, lm.z])
        return out

    def _compute_arm_angles(self, lm: dict, is_right: bool) -> list:
        """Compute 6 joint angles — identical to realsense_skeleton_teleop.py."""
        if is_right:
            shoulder = lm[12]
            other_shoulder = lm[11]
            elbow = lm[14]
            wrist = lm[16]
            pinky = lm[18]
            index = lm[20]
        else:
            shoulder = lm[11]
            other_shoulder = lm[12]
            elbow = lm[13]
            wrist = lm[15]
            pinky = lm[17]
            index = lm[19]

        upper_arm = elbow - shoulder
        forearm = wrist - elbow
        shoulder_across = shoulder - other_shoulder
        down = np.array([0.0, 1.0, 0.0])

        # shoulder_pitch
        ua_yz = np.array([0.0, upper_arm[1], upper_arm[2]])
        shoulder_pitch = self._signed_angle(
            down, ua_yz, np.array([1.0, 0.0, 0.0])
        )

        # shoulder_roll
        ua_xy = np.array([upper_arm[0], upper_arm[1], 0.0])
        shoulder_roll = -self._signed_angle(
            down, ua_xy, np.array([0.0, 0.0, 1.0])
        )

        # shoulder_yaw
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

        # elbow_pitch
        elbow_pitch = -self._angle_between(upper_arm, forearm)

        # wrist_yaw
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

        # wrist_roll
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
    # Drawing
    # ------------------------------------------------------------------ #

    def _draw_skeleton(self, image: np.ndarray, result) -> None:
        """Draw MediaPipe skeleton on the display image."""
        if not result.pose_landmarks or len(result.pose_landmarks) == 0:
            return

        h, w = image.shape[:2]
        landmarks = result.pose_landmarks[0]

        for start_idx, end_idx in _POSE_CONNECTIONS:
            pt1 = landmarks[start_idx]
            pt2 = landmarks[end_idx]
            x1 = int(pt1.x * w)
            y1 = int(pt1.y * h)
            x2 = int(pt2.x * w)
            y2 = int(pt2.y * h)
            if self._mirror:
                x1 = w - 1 - x1
                x2 = w - 1 - x2
            cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        for lm in landmarks:
            cx = int(lm.x * w)
            cy = int(lm.y * h)
            if self._mirror:
                cx = w - 1 - cx
            cv2.circle(image, (cx, cy), 4, (0, 0, 255), -1)

    def _draw_overlay(
        self, image: np.ndarray, human_angles: list[float] | None
    ) -> None:
        """Render calibration state overlay on the camera feed."""
        h, w = image.shape[:2]
        elapsed = time.time() - self._state_start

        # Top banner background
        overlay = image.copy()
        cv2.rectangle(overlay, (0, 0), (w, 90), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, image, 0.4, 0, image)

        if self._state == "detection":
            self._put_centered(
                image, "ROBOT POSE CALIBRATION", (w // 2, 30),
                (0, 255, 255), 0.8,
            )
            self._put_centered(
                image, "Stand facing the camera - detecting skeleton...",
                (w // 2, 60), (200, 200, 200), 0.5,
            )
            remaining = max(0, self.DETECTION_TIMEOUT - elapsed)
            self._put_centered(
                image, f"Timeout: {remaining:.0f}s",
                (w // 2, h // 2), (200, 200, 200), 0.6,
            )

        elif self._state == "transition":
            pose = CALIBRATION_POSES[self._pose_idx]
            remaining = max(0, self.TRANSITION_TIME - elapsed)

            self._put_centered(
                image,
                f"POSE {self._pose_idx + 1}/{len(CALIBRATION_POSES)}: "
                f"{pose['name']}",
                (w // 2, 30), (0, 255, 255), 0.7,
            )
            self._put_centered(
                image, pose["desc"], (w // 2, 60), (255, 255, 255), 0.5,
            )

            # Big countdown in center
            self._put_centered(
                image, f"GET READY  {remaining:.1f}s",
                (w // 2, h // 2), (0, 200, 255), 1.2,
            )
            self._put_centered(
                image, "Copy the robot's pose in RViz",
                (w // 2, h // 2 + 50), (200, 200, 200), 0.6,
            )

            # Progress bar
            progress = self._pose_idx / len(CALIBRATION_POSES)
            self._draw_progress(image, progress)

        elif self._state == "capture":
            pose = CALIBRATION_POSES[self._pose_idx]
            remaining = max(0, self.CAPTURE_TIME - elapsed)

            self._put_centered(
                image,
                f"CAPTURING {self._pose_idx + 1}/{len(CALIBRATION_POSES)}: "
                f"{pose['name']}",
                (w // 2, 30), (0, 255, 0), 0.7,
            )
            self._put_centered(
                image, f"HOLD STILL  {remaining:.1f}s",
                (w // 2, 60), (0, 255, 0), 0.5,
            )

            # Show live angle comparison for active joints
            if human_angles:
                robot_targets = pose["angles"]
                y = 120
                for j in range(12):
                    if abs(robot_targets[j]) < 0.01:
                        continue
                    target_deg = math.degrees(robot_targets[j])
                    human_deg = math.degrees(human_angles[j])
                    diff = abs(target_deg - human_deg)

                    if diff < 15:
                        color = (0, 255, 0)
                    elif diff < 30:
                        color = (0, 200, 255)
                    else:
                        color = (0, 100, 255)

                    text = (
                        f"{self.JOINT_LABELS[j]}: "
                        f"target {target_deg:+.0f}  |  "
                        f"you {human_deg:+.0f}  |  "
                        f"diff {diff:.0f}"
                    )
                    cv2.putText(
                        image, text, (20, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1,
                    )
                    y += 24

            # Frame count
            cv2.putText(
                image,
                f"Frames: {len(self._angle_buffer)}",
                (20, h - 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1,
            )

            # Progress bar
            progress = (
                self._pose_idx + elapsed / self.CAPTURE_TIME
            ) / len(CALIBRATION_POSES)
            self._draw_progress(image, progress)

        elif self._state == "complete":
            self._put_centered(
                image, "CALIBRATION COMPLETE!", (w // 2, h // 2 - 30),
                (0, 255, 0), 1.2,
            )
            save_path = os.path.join(
                CalibrationManager.get_profile_dir(),
                f"{self._profile_name}.yaml",
            )
            self._put_centered(
                image, f"Saved: {save_path}", (w // 2, h // 2 + 20),
                (200, 200, 200), 0.45,
            )
            self._put_centered(
                image,
                f"Use: calibration_profile:={self._profile_name}",
                (w // 2, h // 2 + 50), (200, 200, 200), 0.5,
            )
            self._put_centered(
                image, "Press 'q' to quit", (w // 2, h // 2 + 90),
                (150, 150, 150), 0.5,
            )

        elif self._state == "failed":
            self._put_centered(
                image, "CALIBRATION FAILED", (w // 2, h // 2 - 10),
                (0, 0, 255), 1.0,
            )
            self._put_centered(
                image, "Could not detect skeleton. Press 'q' to quit.",
                (w // 2, h // 2 + 30), (200, 200, 200), 0.5,
            )

        # Controls hint
        controls = "[r] redo  [SPACE] skip  [q] quit"
        cv2.putText(
            image, controls, (10, h - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1,
        )

    @staticmethod
    def _put_centered(
        image: np.ndarray,
        text: str,
        pos: tuple,
        color: tuple,
        scale: float,
    ) -> None:
        font = cv2.FONT_HERSHEY_SIMPLEX
        thickness = max(1, int(scale * 2))
        (tw, th), _ = cv2.getTextSize(text, font, scale, thickness)
        x = pos[0] - tw // 2
        y = pos[1] + th // 2
        cv2.putText(image, text, (x, y), font, scale, color, thickness)

    @staticmethod
    def _draw_progress(image: np.ndarray, progress: float) -> None:
        h, w = image.shape[:2]
        bar_y = h - 20
        bar_h = 8
        bar_w = w - 40
        x0 = 20
        progress = max(0.0, min(1.0, progress))
        cv2.rectangle(
            image, (x0, bar_y), (x0 + bar_w, bar_y + bar_h), (80, 80, 80), -1
        )
        fill = int(bar_w * progress)
        if fill > 0:
            cv2.rectangle(
                image, (x0, bar_y), (x0 + fill, bar_y + bar_h),
                (0, 255, 0), -1,
            )

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
    node = RobotPoseCalibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
