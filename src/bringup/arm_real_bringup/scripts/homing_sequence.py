#!/usr/bin/env python3
"""
One-shot homing sequence for the LDR Harambe arm.

Slowly moves all arm joints to URDF zero position after controllers are active
and before MoveIt is launched, to avoid collisions from large first-move
trajectories.

Exits with code 0 on success, 1 on failure.
"""

import sys
import threading
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

JOINT_NAMES = [
    "left_shoulder_pitch_joint_X6",
    "left_shoulder_roll_joint_X6",
    "left_shoulder_yaw_joint_X4",
    "left_elbow_pitch_joint_X6",
    "left_wrist_yaw_joint_X4",
    "left_wrist_roll_joint_X4",
]


class HomingSequence(Node):

    def __init__(self):
        super().__init__("homing_sequence")

        # Parameters
        self._controller_name = (
            self.declare_parameter("controller_name", "left_arm_group_controller")
            .get_parameter_value()
            .string_value
        )
        self._max_velocity = (
            self.declare_parameter("max_velocity", 0.2)
            .get_parameter_value()
            .double_value
        )
        self._target_position = list(
            self.declare_parameter("target_position", [0.0] * 6)
            .get_parameter_value()
            .double_array_value
        )
        self._min_duration = (
            self.declare_parameter("min_duration", 3.0)
            .get_parameter_value()
            .double_value
        )
        self._tolerance = (
            self.declare_parameter("tolerance", 0.01)
            .get_parameter_value()
            .double_value
        )
        self._joint_state_timeout = (
            self.declare_parameter("joint_state_timeout", 10.0)
            .get_parameter_value()
            .double_value
        )
        self._action_server_timeout = (
            self.declare_parameter("action_server_timeout", 30.0)
            .get_parameter_value()
            .double_value
        )

        # State
        self._lock = threading.Lock()
        self._current_positions: Optional[List[float]] = None
        self._joint_index = {name: i for i, name in enumerate(JOINT_NAMES)}

        # Subscription
        self._state_sub = self.create_subscription(
            JointState, "/joint_states", self._state_callback, 10
        )

        # Action client
        action_topic = f"/{self._controller_name}/follow_joint_trajectory"
        self._action_client = ActionClient(
            self, FollowJointTrajectory, action_topic
        )

        self.get_logger().info(
            f"Homing initialized. Controller: {self._controller_name}, "
            f"max velocity: {self._max_velocity} rad/s"
        )

    def _state_callback(self, msg: JointState):
        if self._current_positions is not None:
            return  # Only need first valid reading

        positions = [None] * len(JOINT_NAMES)
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
                    self.get_logger().info(
                        f"Joint states received: "
                        f"{[f'{p:.3f}' for p in self._current_positions]}"
                    )
                    return list(self._current_positions)
            if (self.get_clock().now() - start) > timeout:
                self.get_logger().error(
                    f"Timed out waiting for /joint_states ({self._joint_state_timeout}s)"
                )
                return None
        return None

    def execute(self) -> int:
        """Run the homing sequence. Returns 0 on success, 1 on failure."""
        # Get current positions
        current = self._wait_for_joint_states()
        if current is None:
            return 1

        target = self._target_position

        # Compute displacement per joint
        displacements = [abs(c - t) for c, t in zip(current, target)]
        max_disp = max(displacements)
        self.get_logger().info(
            f"Per-joint displacements (deg): "
            f"{[f'{d * 57.2958:.1f}' for d in displacements]}"
        )

        # Skip if already at target
        if max_disp < self._tolerance:
            self.get_logger().info(
                f"Already at home position (max displacement "
                f"{max_disp * 57.2958:.1f} deg). Skipping homing."
            )
            return 0

        # Adaptive duration from max displacement and velocity limit
        duration = max(max_disp / self._max_velocity, self._min_duration)
        self.get_logger().info(
            f"Max displacement: {max_disp:.3f} rad ({max_disp * 57.2958:.1f} deg) "
            f"-> duration: {duration:.1f}s"
        )

        # Wait for action server
        action_topic = f"/{self._controller_name}/follow_joint_trajectory"
        self.get_logger().info(f"Waiting for action server: {action_topic}...")
        if not self._action_client.wait_for_server(
            timeout_sec=self._action_server_timeout
        ):
            self.get_logger().error(
                f"Action server not available after {self._action_server_timeout}s"
            )
            return 1

        # Build trajectory goal — single point at target with zero end velocity
        point = JointTrajectoryPoint()
        point.positions = list(target)
        point.velocities = [0.0] * len(JOINT_NAMES)
        duration_sec = int(duration)
        duration_nsec = int((duration - duration_sec) * 1e9)
        point.time_from_start = Duration(sec=duration_sec, nanosec=duration_nsec)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        goal.trajectory.points = [point]
        goal.goal_time_tolerance = Duration(sec=5, nanosec=0)

        # Send goal
        self.get_logger().info(
            f"=== HOMING: Moving to zero position in {duration:.1f}s ==="
        )
        send_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Homing goal REJECTED by controller.")
            return 1

        self.get_logger().info("Homing goal accepted. Waiting for completion...")

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                "=== HOMING COMPLETE. Arm at URDF zero position. ==="
            )
            self.get_logger().info("Safe to launch MoveIt.")
            return 0
        elif status == GoalStatus.STATUS_ABORTED:
            err = result.result
            self.get_logger().error(
                f"Homing ABORTED. error_code={err.error_code}, "
                f"error_string='{err.error_string}'"
            )
            return 1
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Homing CANCELED.")
            return 1
        else:
            self.get_logger().error(f"Homing ended with unexpected status: {status}")
            return 1


def main():
    rclpy.init()
    node = HomingSequence()
    try:
        exit_code = node.execute()
    except KeyboardInterrupt:
        node.get_logger().warn("Homing interrupted by user.")
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
