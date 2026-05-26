#!/usr/bin/env python3
"""
Handshake demo: move the arm to a "hand outstretched" pose, then trigger
the Inspire hand's on-board handshake (action_index=16) when a human grasps
the open palm hard enough to push fingertip force above threshold.

Pre-reqs at runtime:
  - arm_real.launch.py must already be running (controllers active).
  - The inspire ROS 2 workspace must be sourced so `ldr_inspire_msgs` is
    importable; the inspire hand stack must be running (publishing
    /inspire/<side>/state and serving /inspire/<side>/run_action).

Modes:
  - continuous: hold the pose, fire on each detected handshake, cooldown
                between fires. Ctrl+C to stop.
  - oneshot:    hold the pose, fire once, wait out cooldown, return arm to
                URDF zero, exit.
"""

import sys
import threading
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# Side → (controller name, joint names). Mirrors the canonical layout in
# src/bringup/arm_real_bringup/config/controllers.yaml.
ARM_SIDES = {
    "right": (
        "right_arm_trajectory_controller",
        [
            "right_shoulder_pitch_joint_X6",
            "right_shoulder_roll_joint_X6",
            "right_shoulder_yaw_joint_X4",
            "right_elbow_pitch_joint_X6",
            "right_wrist_yaw_joint_X4",
            "right_wrist_roll_joint_X4",
        ],
    ),
    "left": (
        "left_arm_trajectory_controller",
        [
            "left_shoulder_pitch_joint_X6",
            "left_shoulder_roll_joint_X6",
            "left_shoulder_yaw_joint_X4",
            "left_elbow_pitch_joint_X6",
            "left_wrist_yaw_joint_X4",
            "left_wrist_roll_joint_X4",
        ],
    ),
}


class HandshakeDemo(Node):

    def __init__(self):
        super().__init__("handshake_demo")

        # ── Parameters ───────────────────────────────────────────────
        arm_side = (
            self.declare_parameter("arm_side", "right")
            .get_parameter_value().string_value
        )
        if arm_side not in ARM_SIDES:
            raise ValueError(f"arm_side must be 'right' or 'left', got '{arm_side}'")
        default_controller, default_joints = ARM_SIDES[arm_side]

        self._controller_name = (
            self.declare_parameter("controller_name", default_controller)
            .get_parameter_value().string_value
        )
        self._joint_names = list(
            self.declare_parameter("joint_names", default_joints)
            .get_parameter_value().string_array_value
        )

        self._pose_source = (
            self.declare_parameter("pose_source", "predefined")
            .get_parameter_value().string_value
        )
        if self._pose_source not in ("predefined", "current"):
            raise ValueError(
                f"pose_source must be 'predefined' or 'current', got '{self._pose_source}'"
            )

        self._predefined_pose = list(
            self.declare_parameter(
                "predefined_pose", [0.0] * len(self._joint_names)
            ).get_parameter_value().double_array_value
        )

        self._mode = (
            self.declare_parameter("mode", "continuous")
            .get_parameter_value().string_value
        )
        if self._mode not in ("continuous", "oneshot"):
            raise ValueError(f"mode must be 'continuous' or 'oneshot', got '{self._mode}'")

        self._max_velocity = (
            self.declare_parameter("max_velocity", 0.3)
            .get_parameter_value().double_value
        )
        self._min_duration = (
            self.declare_parameter("min_duration", 3.0)
            .get_parameter_value().double_value
        )
        self._tolerance = (
            self.declare_parameter("tolerance", 0.01)
            .get_parameter_value().double_value
        )
        self._joint_state_timeout = (
            self.declare_parameter("joint_state_timeout", 10.0)
            .get_parameter_value().double_value
        )
        self._action_server_timeout = (
            self.declare_parameter("action_server_timeout", 30.0)
            .get_parameter_value().double_value
        )

        self._hand_namespace = (
            self.declare_parameter("hand_namespace", "/inspire/right")
            .get_parameter_value().string_value
        ).rstrip("/")
        self._action_index = (
            self.declare_parameter("action_index", 16)
            .get_parameter_value().integer_value
        )
        self._monitored_finger_indices = list(
            self.declare_parameter("monitored_finger_indices", [0, 1, 2, 3])
            .get_parameter_value().integer_array_value
        )
        self._force_threshold_g = (
            self.declare_parameter("force_threshold_g", 50.0)
            .get_parameter_value().double_value
        )
        self._force_hold_seconds = (
            self.declare_parameter("force_hold_seconds", 0.15)
            .get_parameter_value().double_value
        )
        self._cooldown_seconds = (
            self.declare_parameter("cooldown_seconds", 5.0)
            .get_parameter_value().double_value
        )

        if len(self._predefined_pose) != len(self._joint_names):
            raise ValueError(
                f"predefined_pose has {len(self._predefined_pose)} values, "
                f"expected {len(self._joint_names)} (one per joint)."
            )

        # ── State ────────────────────────────────────────────────────
        self._lock = threading.Lock()
        self._current_positions: Optional[List[float]] = None
        self._joint_index = {n: i for i, n in enumerate(self._joint_names)}

        self._cross_time: Optional[rclpy.time.Time] = None
        self._cooldown_until: Optional[rclpy.time.Time] = None
        self._fire_count = 0
        self._armed = False
        self._baseline_peak = 0.0
        self._baseline_samples = 0
        self._hand_state_seen = False

        # ── ROS interfaces ───────────────────────────────────────────
        self._state_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )

        action_topic = f"/{self._controller_name}/follow_joint_trajectory"
        self._action_client = ActionClient(self, FollowJointTrajectory, action_topic)

        # Inspire service + state subscription are created lazily after
        # the move-to-pose phase, so import errors surface only when needed.
        self._run_action_client = None
        self._hand_state_sub = None

        self.get_logger().info(
            f"HandshakeDemo init: side={arm_side}, controller={self._controller_name}, "
            f"pose_source={self._pose_source}, mode={self._mode}, "
            f"hand_ns={self._hand_namespace}, threshold={self._force_threshold_g} g, "
            f"hold={self._force_hold_seconds}s, cooldown={self._cooldown_seconds}s"
        )

    # ── /joint_states ───────────────────────────────────────────────
    def _joint_state_cb(self, msg: JointState):
        positions = [None] * len(self._joint_names)
        for name, pos in zip(msg.name, msg.position):
            if name in self._joint_index:
                positions[self._joint_index[name]] = pos

        if any(v is None for v in positions):
            return

        with self._lock:
            self._current_positions = [float(p) for p in positions]

    def _wait_for_joint_states(self) -> Optional[List[float]]:
        self.get_logger().info("Waiting for /joint_states...")
        start = self.get_clock().now()
        timeout = rclpy.duration.Duration(seconds=self._joint_state_timeout)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            with self._lock:
                if self._current_positions is not None:
                    return list(self._current_positions)
            if (self.get_clock().now() - start) > timeout:
                self.get_logger().error(
                    f"Timed out waiting for /joint_states ({self._joint_state_timeout}s)"
                )
                return None
        return None

    # ── Move-to-pose ────────────────────────────────────────────────
    def _move_to(self, target: List[float], label: str) -> bool:
        current = self._wait_for_joint_states()
        if current is None:
            return False

        displacements = [abs(c - t) for c, t in zip(current, target)]
        max_disp = max(displacements)
        self.get_logger().info(
            f"{label}: per-joint deltas (deg): "
            f"{[f'{d * 57.2958:.1f}' for d in displacements]}"
        )

        if max_disp < self._tolerance:
            self.get_logger().info(
                f"{label}: already at target (max delta {max_disp * 57.2958:.2f} deg). Skip."
            )
            return True

        duration = max(max_disp / self._max_velocity, self._min_duration)
        self.get_logger().info(
            f"{label}: moving in {duration:.1f}s (max delta {max_disp * 57.2958:.1f} deg)"
        )

        action_topic = f"/{self._controller_name}/follow_joint_trajectory"
        if not self._action_client.wait_for_server(timeout_sec=self._action_server_timeout):
            self.get_logger().error(
                f"Action server not available after {self._action_server_timeout}s: {action_topic}"
            )
            return False

        point = JointTrajectoryPoint()
        point.positions = list(target)
        point.velocities = [0.0] * len(self._joint_names)
        sec = int(duration)
        nsec = int((duration - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nsec)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = list(self._joint_names)
        goal.trajectory.points = [point]
        goal.goal_time_tolerance = Duration(sec=5, nanosec=0)

        send_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle.accepted:
            self.get_logger().error(f"{label}: goal REJECTED by controller.")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"{label}: complete.")
            return True

        err = result.result
        self.get_logger().error(
            f"{label}: ended with status {result.status}, "
            f"error_code={getattr(err, 'error_code', '?')}, "
            f"error_string='{getattr(err, 'error_string', '?')}'"
        )
        return False

    # ── Hand bring-up ────────────────────────────────────────────────
    def _setup_hand_interfaces(self) -> bool:
        """Import the service type, create the client + subscription. Called
        BEFORE the arm starts moving so baseline force is collected during
        the entire motion. The trigger stays disarmed (self._armed=False)
        until the arm reaches the pose."""
        try:
            from ldr_inspire_msgs.srv import RunAction  # noqa: F401
        except ImportError as e:
            self.get_logger().error(
                f"Could not import ldr_inspire_msgs: {e}. "
                "Source the inspire workspace before launching."
            )
            return False
        self._RunAction = RunAction

        service_name = f"{self._hand_namespace}/run_action"
        self._run_action_client = self.create_client(RunAction, service_name)
        if not self._run_action_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                f"Service {service_name} not yet available — will retry on each fire."
            )

        topic = f"{self._hand_namespace}/state"
        self._hand_state_sub = self.create_subscription(
            JointState, topic, self._hand_state_cb, 10
        )
        self.get_logger().info(
            f"Subscribed to {topic}; service: {service_name}. "
            "Sampling idle baseline during arm motion."
        )
        return True

    # ── Force-trigger state machine ─────────────────────────────────
    def _hand_state_cb(self, msg: JointState):
        if not msg.effort:
            return

        try:
            forces = [float(msg.effort[i]) for i in self._monitored_finger_indices]
        except IndexError:
            self.get_logger().warn(
                f"effort has only {len(msg.effort)} elements; "
                f"requested indices {self._monitored_finger_indices}. Skipping sample."
            )
            return
        peak = max(forces)
        now = self.get_clock().now()

        # Always update the rolling baseline. Pre-arm (during arm motion)
        # this is just baseline collection. Post-arm we keep updating so
        # later log lines reflect any drift.
        if not self._hand_state_seen:
            self._hand_state_seen = True
            self.get_logger().info(
                f"First hand-state sample: peak {peak:.1f} g across fingers "
                f"{self._monitored_finger_indices}"
            )
        self._baseline_peak = max(self._baseline_peak, peak)
        self._baseline_samples += 1

        # Trigger logic disabled until the arm has reached the pose.
        if not self._armed:
            return

        # Cooldown: ignore force readings entirely until cooldown elapses.
        if self._cooldown_until is not None and now < self._cooldown_until:
            return

        if peak >= self._force_threshold_g:
            if self._cross_time is None:
                self._cross_time = now
                return
            held = (now - self._cross_time).nanoseconds / 1e9
            if held >= self._force_hold_seconds:
                self._fire(peak)
        else:
            self._cross_time = None

    def _fire(self, peak: float):
        self._fire_count += 1
        self.get_logger().info(
            f"Trigger #{self._fire_count}: peak {peak:.1f} g ≥ {self._force_threshold_g:.1f} g, "
            f"calling {self._hand_namespace}/run_action(action_index={self._action_index})"
        )

        if not self._run_action_client.service_is_ready():
            if not self._run_action_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error(
                    f"Service {self._hand_namespace}/run_action unavailable; skipping fire."
                )
                # Reset cross_time so a fresh sustained press can re-trigger
                # without waiting on cooldown.
                self._cross_time = None
                return

        req = self._RunAction.Request()
        req.action_index = self._action_index
        future = self._run_action_client.call_async(req)
        future.add_done_callback(self._on_run_action_response)

        # Engage cooldown immediately — the action's own gripping force
        # would otherwise re-trigger.
        self._cooldown_until = self.get_clock().now() + rclpy.duration.Duration(
            seconds=self._cooldown_seconds
        )
        self._cross_time = None

    def _on_run_action_response(self, future):
        try:
            result = future.result()
            self.get_logger().info(f"run_action response: {result}")
        except Exception as e:
            self.get_logger().error(f"run_action call failed: {e}")

    # ── Top-level ────────────────────────────────────────────────────
    def execute(self) -> int:
        # Phase 1: bring up hand interfaces FIRST so /inspire/<side>/state
        # can be sampled for baseline force throughout the arm motion.
        # The trigger stays disarmed until phase 3.
        if not self._setup_hand_interfaces():
            return 1

        # Phase 2: move to handshake pose (or snapshot current pose).
        if self._pose_source == "current":
            current = self._wait_for_joint_states()
            if current is None:
                return 1
            target = list(current)
            self.get_logger().info(
                f"pose_source=current → using snapshot: {[f'{p:.3f}' for p in target]}"
            )
        else:
            target = list(self._predefined_pose)
            self.get_logger().info(
                f"pose_source=predefined → target: {[f'{p:.3f}' for p in target]}"
            )

        if not self._move_to(target, "Move to handshake pose"):
            return 1

        # Phase 3: report baseline and arm the trigger.
        if self._hand_state_seen:
            self.get_logger().info(
                f"Idle baseline peak over arm motion: {self._baseline_peak:.1f} g "
                f"across {self._baseline_samples} samples "
                f"(threshold: {self._force_threshold_g:.1f} g)"
            )
            if self._baseline_peak >= self._force_threshold_g:
                self.get_logger().warn(
                    "Baseline already exceeds threshold — first trigger may be a false positive. "
                    "Consider raising force_threshold_g."
                )
        else:
            self.get_logger().warn(
                f"No samples received from {self._hand_namespace}/state during the move. "
                "Is the inspire hand stack running?"
            )

        self._armed = True
        self.get_logger().info(
            f"=== ARMED. Mode={self._mode}. "
            f"Grasp the palm to trigger handshake. ==="
        )

        if self._mode == "oneshot":
            return self._run_oneshot()
        return self._run_continuous()

    def _run_oneshot(self) -> int:
        # Spin until first fire + cooldown elapses, then return home.
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._fire_count >= 1 and self._cooldown_until is not None:
                if self.get_clock().now() >= self._cooldown_until:
                    break

        self.get_logger().info("oneshot: cooldown elapsed; returning to URDF zero.")
        # Refresh current pose before computing return move.
        with self._lock:
            self._current_positions = None
        if not self._move_to([0.0] * len(self._joint_names), "Return to home"):
            return 1
        return 0

    def _run_continuous(self) -> int:
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass
        self.get_logger().info(
            f"continuous: stopped after {self._fire_count} handshake(s). "
            "Arm left in place."
        )
        return 0


def main():
    rclpy.init()
    node = HandshakeDemo()
    try:
        exit_code = node.execute()
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted by user.")
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
