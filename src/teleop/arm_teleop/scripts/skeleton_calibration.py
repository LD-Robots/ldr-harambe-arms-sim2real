#!/usr/bin/env python3
"""
Human calibration routine for skeleton teleop.

Captures neutral pose, per-joint range of motion, and body proportions.
Calibration data is stored as YAML profiles that persist across sessions.

Calibration phases (~40 seconds total):
  0. Detection check (2s) — verify skeleton is visible
  1. Neutral pose (5s) — arms at sides, capture zero reference
  2. ROM sweep (30s) — 5s per joint, measure human's min/max angles
  3. Verification (3s) — return to neutral, check calibration

Press 'c' during teleop to start calibration.
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass, field, asdict
from datetime import datetime
from enum import Enum
from typing import Optional

import cv2
import numpy as np
import yaml


# ------------------------------------------------------------------ #
# Data structures
# ------------------------------------------------------------------ #

@dataclass
class JointCalibration:
    """Calibration data for a single joint."""
    neutral_angle: float = 0.0
    human_min: float = 0.0
    human_max: float = 0.0
    robot_min: float = 0.0
    robot_max: float = 0.0


@dataclass
class BodyProportions:
    """Measured body segment lengths in meters (MediaPipe world units)."""
    upper_arm_length_left: float = 0.0
    upper_arm_length_right: float = 0.0
    forearm_length_left: float = 0.0
    forearm_length_right: float = 0.0
    hand_span_left: float = 0.0
    hand_span_right: float = 0.0
    shoulder_width: float = 0.0
    torso_length_left: float = 0.0
    torso_length_right: float = 0.0


@dataclass
class CalibrationProfile:
    """Complete calibration for one user."""
    profile_name: str = "default"
    timestamp: str = ""
    mirror_display: bool = True
    frames_averaged: int = 30
    joints: dict = field(default_factory=dict)  # name -> JointCalibration
    proportions: Optional[BodyProportions] = None
    neutral_landmarks: Optional[dict] = None  # idx -> [x, y, z]


# ------------------------------------------------------------------ #
# State machine
# ------------------------------------------------------------------ #

class CalibrationState(Enum):
    IDLE = 0
    DETECTION_CHECK = 1
    NEUTRAL_CAPTURE = 2
    ROM_SWEEP = 3
    VERIFICATION = 4
    COMPLETE = 5
    FAILED = 6


# ROM sweep instructions — both arms perform the same motion
ROM_INSTRUCTIONS = [
    ("Shoulder Pitch", "Swing arms FORWARD and BACKWARD"),
    ("Shoulder Roll", "Raise arms OUT TO SIDES and back down"),
    ("Shoulder Yaw", "Bend elbows, ROTATE forearms left and right"),
    ("Elbow Pitch", "BEND and STRAIGHTEN your elbows fully"),
    ("Wrist Yaw", "Rotate forearms PALM-UP and PALM-DOWN"),
    ("Wrist Roll", "FLEX and EXTEND your wrists up and down"),
]

# Timings (seconds)
DETECTION_TIMEOUT = 5.0
DETECTION_MIN_FRAMES = 15
NEUTRAL_HOLD_TIME = 5.0
ROM_SWEEP_TIME = 5.0
VERIFICATION_TIME = 3.0


class CalibrationManager:
    """Finite state machine that drives the calibration procedure."""

    def __init__(self, joint_names: list, joint_limits: dict, logger=None):
        """
        Args:
            joint_names: All joint names (left + right, 12 total).
            joint_limits: Dict of joint_name -> (lo, hi).
            logger: ROS logger (or None for print fallback).
        """
        self._joint_names = joint_names
        self._joint_limits = joint_limits
        self._logger = logger

        self._state = CalibrationState.IDLE
        self._state_start_time = 0.0
        self._rom_joint_index = 0  # 0..5, indexes into the 6 per-arm joints

        # Buffers
        self._landmark_buffer: list[dict] = []
        self._angle_buffer: list[list[float]] = []

        # Result
        self._profile = CalibrationProfile()

        # Callable: landmarks -> angles (set by teleop node)
        self._compute_angles_fn = None

    def _log(self, msg: str):
        if self._logger:
            self._logger.info(msg)
        else:
            print(f"[Calibration] {msg}")

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #

    def set_angle_fn(self, fn):
        """Set the function that computes joint angles from landmarks."""
        self._compute_angles_fn = fn

    def start(self, profile_name: str = "default", mirror_display: bool = True):
        """Begin calibration."""
        self._profile = CalibrationProfile(
            profile_name=profile_name,
            timestamp=datetime.now().isoformat(),
            mirror_display=mirror_display,
        )
        self._rom_joint_index = 0
        self._transition(CalibrationState.DETECTION_CHECK)
        self._log(f"Calibration started for profile '{profile_name}'")

    def is_active(self) -> bool:
        return self._state not in (CalibrationState.IDLE, CalibrationState.COMPLETE,
                                    CalibrationState.FAILED)

    def is_complete(self) -> bool:
        return self._state == CalibrationState.COMPLETE

    def is_failed(self) -> bool:
        return self._state == CalibrationState.FAILED

    def get_result(self) -> CalibrationProfile:
        return self._profile

    def process_frame(self, landmarks: dict, angles: list[float]) -> None:
        """
        Feed one frame to the calibration state machine.

        Args:
            landmarks: {idx: np.array([x,y,z])} from MediaPipe.
            angles: 12 computed joint angles (left + right).
        """
        if self._state == CalibrationState.IDLE:
            return

        elapsed = time.time() - self._state_start_time

        if self._state == CalibrationState.DETECTION_CHECK:
            self._handle_detection(landmarks, elapsed)
        elif self._state == CalibrationState.NEUTRAL_CAPTURE:
            self._handle_neutral(landmarks, angles, elapsed)
        elif self._state == CalibrationState.ROM_SWEEP:
            self._handle_rom(angles, elapsed)
        elif self._state == CalibrationState.VERIFICATION:
            self._handle_verification(angles, elapsed)

    def draw_overlay(self, image: np.ndarray) -> None:
        """Render calibration UI on the debug frame."""
        if self._state == CalibrationState.IDLE:
            return

        h, w = image.shape[:2]
        elapsed = time.time() - self._state_start_time

        # Semi-transparent banner at top
        overlay = image.copy()
        cv2.rectangle(overlay, (0, 0), (w, 80), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, image, 0.4, 0, image)

        if self._state == CalibrationState.DETECTION_CHECK:
            self._draw_text(image, "CALIBRATION: Stand facing camera",
                           (w // 2, 30), (0, 255, 255), 0.7, center=True)
            self._draw_text(image, "Ensure full upper body is visible",
                           (w // 2, 60), (200, 200, 200), 0.5, center=True)
            progress = len(self._landmark_buffer) / DETECTION_MIN_FRAMES
            self._draw_progress_bar(image, progress, (0, 255, 255))

        elif self._state == CalibrationState.NEUTRAL_CAPTURE:
            remaining = max(0, NEUTRAL_HOLD_TIME - elapsed)
            self._draw_text(image, "NEUTRAL POSE: Arms relaxed at sides",
                           (w // 2, 30), (0, 255, 0), 0.7, center=True)
            self._draw_text(image, f"Hold still... {remaining:.1f}s",
                           (w // 2, 60), (200, 200, 200), 0.5, center=True)
            self._draw_progress_bar(image, elapsed / NEUTRAL_HOLD_TIME, (0, 255, 0))

        elif self._state == CalibrationState.ROM_SWEEP:
            remaining = max(0, ROM_SWEEP_TIME - elapsed)
            joint_label, instruction = ROM_INSTRUCTIONS[self._rom_joint_index]
            self._draw_text(
                image,
                f"ROM [{self._rom_joint_index + 1}/6]: {joint_label}",
                (w // 2, 30), (255, 200, 0), 0.7, center=True)
            self._draw_text(image, instruction,
                           (w // 2, 60), (255, 255, 255), 0.5, center=True)

            # Countdown in center
            self._draw_text(image, f"{remaining:.1f}s",
                           (w // 2, h // 2), (255, 200, 0), 1.5, center=True)
            self._draw_progress_bar(image, elapsed / ROM_SWEEP_TIME, (255, 200, 0))

            # Live ROM bars for current joint
            self._draw_rom_bars(image)

        elif self._state == CalibrationState.VERIFICATION:
            remaining = max(0, VERIFICATION_TIME - elapsed)
            self._draw_text(image, "VERIFY: Return to neutral pose",
                           (w // 2, 30), (0, 255, 0), 0.7, center=True)
            self._draw_text(image, f"{remaining:.1f}s",
                           (w // 2, 60), (200, 200, 200), 0.5, center=True)
            self._draw_progress_bar(image, elapsed / VERIFICATION_TIME, (0, 255, 0))

        elif self._state == CalibrationState.COMPLETE:
            self._draw_text(image, "CALIBRATION COMPLETE!",
                           (w // 2, h // 2), (0, 255, 0), 1.2, center=True)

        elif self._state == CalibrationState.FAILED:
            self._draw_text(image, "CALIBRATION FAILED - press 'c' to retry",
                           (w // 2, h // 2), (0, 0, 255), 0.8, center=True)

    # ------------------------------------------------------------------ #
    # State handlers
    # ------------------------------------------------------------------ #

    def _handle_detection(self, landmarks: dict, elapsed: float):
        """Check that all needed landmarks are visible."""
        # Simple check: we have at least the basic landmarks
        needed = {11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 23, 24}
        if needed.issubset(landmarks.keys()):
            self._landmark_buffer.append(landmarks)
        else:
            self._landmark_buffer.clear()

        if len(self._landmark_buffer) >= DETECTION_MIN_FRAMES:
            self._log("Detection stable, capturing neutral pose")
            self._transition(CalibrationState.NEUTRAL_CAPTURE)
        elif elapsed > DETECTION_TIMEOUT:
            self._log("Detection failed — skeleton not visible")
            self._state = CalibrationState.FAILED

    def _handle_neutral(self, landmarks: dict, angles: list, elapsed: float):
        """Accumulate frames for neutral pose averaging."""
        self._landmark_buffer.append(landmarks)
        self._angle_buffer.append(list(angles))

        if elapsed >= NEUTRAL_HOLD_TIME:
            self._compute_neutral()
            self._log("Neutral pose captured, starting ROM sweep")
            self._rom_joint_index = 0
            self._transition(CalibrationState.ROM_SWEEP)

    def _handle_rom(self, angles: list, elapsed: float):
        """Record angles during ROM sweep for current joint."""
        self._angle_buffer.append(list(angles))

        if elapsed >= ROM_SWEEP_TIME:
            self._compute_rom_for_joint(self._rom_joint_index)
            self._rom_joint_index += 1
            if self._rom_joint_index >= 6:
                self._log("ROM sweep complete, verifying")
                self._transition(CalibrationState.VERIFICATION)
            else:
                label = ROM_INSTRUCTIONS[self._rom_joint_index][0]
                self._log(f"ROM sweep: {label}")
                self._transition(CalibrationState.ROM_SWEEP)

    def _handle_verification(self, angles: list, elapsed: float):
        """Verify calibration by checking near-zero at neutral."""
        self._angle_buffer.append(list(angles))

        if elapsed >= VERIFICATION_TIME:
            self._verify()
            self._state = CalibrationState.COMPLETE
            self._log("Calibration complete!")

    # ------------------------------------------------------------------ #
    # Computation
    # ------------------------------------------------------------------ #

    def _compute_neutral(self):
        """Average landmarks and compute neutral angles + body proportions."""
        n_frames = len(self._landmark_buffer)
        if n_frames == 0:
            return

        # Average landmark positions
        all_indices = set()
        for frame in self._landmark_buffer:
            all_indices.update(frame.keys())

        avg_lm = {}
        for idx in all_indices:
            positions = [f[idx] for f in self._landmark_buffer if idx in f]
            if positions:
                avg_lm[idx] = np.mean(positions, axis=0)

        # Average angles
        n_joints = len(self._joint_names)
        avg_angles = [0.0] * n_joints
        n_angle_frames = len(self._angle_buffer)
        if n_angle_frames > 0:
            for frame in self._angle_buffer:
                for i in range(min(len(frame), n_joints)):
                    avg_angles[i] += frame[i]
            avg_angles = [a / n_angle_frames for a in avg_angles]

        # Store neutral angles for all joints
        for i, name in enumerate(self._joint_names):
            lo, hi = self._joint_limits.get(name, (-3.14, 3.14))
            self._profile.joints[name] = JointCalibration(
                neutral_angle=avg_angles[i],
                human_min=avg_angles[i],
                human_max=avg_angles[i],
                robot_min=lo,
                robot_max=hi,
            )

        # Body proportions
        def _dist(a_idx, b_idx):
            if a_idx in avg_lm and b_idx in avg_lm:
                return float(np.linalg.norm(avg_lm[a_idx] - avg_lm[b_idx]))
            return 0.0

        self._profile.proportions = BodyProportions(
            upper_arm_length_left=_dist(11, 13),
            upper_arm_length_right=_dist(12, 14),
            forearm_length_left=_dist(13, 15),
            forearm_length_right=_dist(14, 16),
            hand_span_left=_dist(19, 17),
            hand_span_right=_dist(20, 18),
            shoulder_width=_dist(11, 12),
            torso_length_left=_dist(11, 23),
            torso_length_right=_dist(12, 24),
        )

        # Store neutral landmarks
        self._profile.neutral_landmarks = {
            idx: pos.tolist() for idx, pos in avg_lm.items()
        }
        self._profile.frames_averaged = n_frames

    def _compute_rom_for_joint(self, per_arm_joint_index: int):
        """
        Extract min/max for the given per-arm joint index (0..5) from angle buffer.
        Applies to both left (index) and right (index + 6) joints.
        """
        if not self._angle_buffer:
            return

        for offset, side in [(0, "left"), (6, "right")]:
            joint_idx = per_arm_joint_index + offset
            if joint_idx >= len(self._joint_names):
                continue
            name = self._joint_names[joint_idx]
            jcal = self._profile.joints.get(name)
            if jcal is None:
                continue

            values = [frame[joint_idx] for frame in self._angle_buffer
                      if joint_idx < len(frame)]
            if len(values) < 5:
                continue

            # 5th/95th percentile to reject noise
            p5 = float(np.percentile(values, 5))
            p95 = float(np.percentile(values, 95))
            jcal.human_min = min(jcal.human_min, p5)
            jcal.human_max = max(jcal.human_max, p95)

    def _verify(self):
        """Log how close the verification angles are to zero after calibration."""
        if not self._angle_buffer:
            return

        # Average the last few frames
        recent = self._angle_buffer[-min(10, len(self._angle_buffer)):]
        n = len(recent)
        n_joints = len(self._joint_names)
        avg = [0.0] * n_joints
        for frame in recent:
            for i in range(min(len(frame), n_joints)):
                avg[i] += frame[i]
        avg = [a / n for a in avg]

        for i, name in enumerate(self._joint_names):
            jcal = self._profile.joints.get(name)
            if jcal:
                offset = abs(avg[i] - jcal.neutral_angle)
                if offset > 0.3:
                    self._log(f"  Warning: {name} offset = {offset:.2f} rad at neutral")

    # ------------------------------------------------------------------ #
    # Internal helpers
    # ------------------------------------------------------------------ #

    def _transition(self, new_state: CalibrationState):
        self._state = new_state
        self._state_start_time = time.time()
        self._landmark_buffer.clear()
        self._angle_buffer.clear()

    # ------------------------------------------------------------------ #
    # Drawing helpers
    # ------------------------------------------------------------------ #

    @staticmethod
    def _draw_text(image, text, pos, color, scale, center=False):
        font = cv2.FONT_HERSHEY_SIMPLEX
        thickness = max(1, int(scale * 2))
        if center:
            (tw, th), _ = cv2.getTextSize(text, font, scale, thickness)
            pos = (pos[0] - tw // 2, pos[1] + th // 2)
        cv2.putText(image, text, pos, font, scale, color, thickness)

    @staticmethod
    def _draw_progress_bar(image, progress, color):
        h, w = image.shape[:2]
        bar_y = h - 10
        bar_h = 6
        bar_w = w - 40
        x0 = 20
        progress = max(0.0, min(1.0, progress))
        cv2.rectangle(image, (x0, bar_y), (x0 + bar_w, bar_y + bar_h),
                      (80, 80, 80), -1)
        fill_w = int(bar_w * progress)
        if fill_w > 0:
            cv2.rectangle(image, (x0, bar_y), (x0 + fill_w, bar_y + bar_h),
                          color, -1)

    def _draw_rom_bars(self, image):
        """Draw live ROM bars for the current ROM sweep joint (both arms)."""
        if not self._angle_buffer:
            return

        h, w = image.shape[:2]
        y_start = h - 80

        for offset, label, color in [(0, "L", (0, 255, 0)),
                                      (6, "R", (255, 200, 0))]:
            joint_idx = self._rom_joint_index + offset
            if joint_idx >= len(self._joint_names):
                continue
            name = self._joint_names[joint_idx]
            jcal = self._profile.joints.get(name)
            if jcal is None:
                continue

            values = [f[joint_idx] for f in self._angle_buffer if joint_idx < len(f)]
            if not values:
                continue

            cur_min = min(values)
            cur_max = max(values)
            rom = cur_max - cur_min

            y = y_start + offset // 6 * 25
            text = f"{label}: ROM {np.degrees(rom):.0f}deg [{np.degrees(cur_min):.0f}, {np.degrees(cur_max):.0f}]"
            cv2.putText(image, text, (20, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

    # ------------------------------------------------------------------ #
    # YAML persistence
    # ------------------------------------------------------------------ #

    @staticmethod
    def save_profile(profile: CalibrationProfile, path: str) -> None:
        """Save calibration profile to YAML file."""
        os.makedirs(os.path.dirname(path), exist_ok=True)

        data = {
            "profile_name": profile.profile_name,
            "timestamp": profile.timestamp,
            "mirror_display": profile.mirror_display,
            "frames_averaged": profile.frames_averaged,
        }

        if profile.proportions:
            data["proportions"] = asdict(profile.proportions)

        if profile.joints:
            joints_data = {}
            for name, jcal in profile.joints.items():
                joints_data[name] = asdict(jcal)
            data["joints"] = joints_data

        if profile.neutral_landmarks:
            data["neutral_landmarks"] = profile.neutral_landmarks

        with open(path, "w") as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)

    @staticmethod
    def load_profile(path: str) -> Optional[CalibrationProfile]:
        """Load calibration profile from YAML file."""
        if not os.path.isfile(path):
            return None

        with open(path, "r") as f:
            data = yaml.safe_load(f)

        if not data:
            return None

        profile = CalibrationProfile(
            profile_name=data.get("profile_name", "unknown"),
            timestamp=data.get("timestamp", ""),
            mirror_display=data.get("mirror_display", True),
            frames_averaged=data.get("frames_averaged", 30),
        )

        if "proportions" in data and data["proportions"]:
            profile.proportions = BodyProportions(**data["proportions"])

        if "joints" in data and data["joints"]:
            for name, jdata in data["joints"].items():
                profile.joints[name] = JointCalibration(**jdata)

        if "neutral_landmarks" in data:
            profile.neutral_landmarks = data["neutral_landmarks"]

        return profile

    @staticmethod
    def get_profile_dir() -> str:
        """Return the default directory for calibration profiles."""
        return os.path.expanduser("~/.config/arm_teleop/calibration_profiles")

    @staticmethod
    def list_profiles(directory: str = "") -> list[str]:
        """List available profile YAML files."""
        if not directory:
            directory = CalibrationManager.get_profile_dir()
        if not os.path.isdir(directory):
            return []
        return [f for f in os.listdir(directory) if f.endswith(".yaml")]


# ------------------------------------------------------------------ #
# Calibration application helpers (used by the teleop node)
# ------------------------------------------------------------------ #

def apply_neutral_offset(
    raw_angles: list[float],
    joint_names: list[str],
    calibration: CalibrationProfile,
) -> list[float]:
    """Subtract neutral pose angles so the human's resting pose maps to zero."""
    result = []
    for name, angle in zip(joint_names, raw_angles):
        jcal = calibration.joints.get(name)
        if jcal:
            result.append(angle - jcal.neutral_angle)
        else:
            result.append(angle)
    return result


def apply_rom_mapping(
    offset_angles: list[float],
    joint_names: list[str],
    calibration: CalibrationProfile,
) -> list[float]:
    """
    Map human ROM to robot ROM per-joint, piecewise linear around zero.

    After neutral offset subtraction, angle=0 means "at neutral".
    Negative angles map through the human's negative range to the robot's
    negative range (robot_min to 0), and positive angles similarly.
    """
    result = []
    for name, angle in zip(joint_names, offset_angles):
        jcal = calibration.joints.get(name)
        if jcal is None:
            result.append(angle)
            continue

        # Human range relative to neutral
        human_neg_range = jcal.neutral_angle - jcal.human_min  # positive value
        human_pos_range = jcal.human_max - jcal.neutral_angle  # positive value

        # Robot range (absolute from limits)
        robot_neg_range = abs(jcal.robot_min)
        robot_pos_range = abs(jcal.robot_max)

        if angle < 0 and human_neg_range > 0.05:
            scale = robot_neg_range / human_neg_range
            result.append(angle * scale)
        elif angle >= 0 and human_pos_range > 0.05:
            scale = robot_pos_range / human_pos_range
            result.append(angle * scale)
        else:
            # Minimal measured ROM — pass through unscaled
            result.append(angle)

    return result
