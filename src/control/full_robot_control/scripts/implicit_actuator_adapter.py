#!/usr/bin/env python3
"""
ImplicitActuatorAdapter — ROS 2 node that replicates Isaac Lab's ImplicitActuator.

Runs a PD control loop at 200 Hz (matching Isaac Lab sim_dt=0.005):
    torque = kp * (target_pos - current_pos) - kd * current_vel
    torque = clip(torque, -effort_limit, effort_limit)

Subscribes to:
    /joint_states          — current joint positions and velocities
    /target_joint_positions — position targets from RL policy (Float64MultiArray, 25 values)

Publishes to:
    /whole_body_controller/commands — effort commands (Float64MultiArray, 25 values)

On startup, target = default positions (from training config).
When a new target is received, PD tracks it immediately.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# Hardware joint order — matches Isaac Lab training exactly
ALL_JOINTS = [
    # Left arm (0-5)
    "left_shoulder_pitch_joint_X6",
    "left_shoulder_roll_joint_X6",
    "left_shoulder_yaw_joint_X4",
    "left_elbow_pitch_joint_X6",
    "left_wrist_yaw_joint_X4",
    "left_wrist_roll_joint_X4",
    # Right arm (6-11)
    "right_shoulder_pitch_joint_X6",
    "right_shoulder_roll_joint_X6",
    "right_shoulder_yaw_joint_X4",
    "right_elbow_pitch_joint_X6",
    "right_wrist_yaw_joint_X4",
    "right_wrist_roll_joint_X4",
    # Waist (12)
    "waist_yaw_joint_X8",
    # Left leg (13-18)
    "left_hip_pitch_joint_X8",
    "left_hip_roll_joint_X8",
    "left_hip_yaw_joint_X8",
    "left_knee_joint_X8",
    "left_ankle_pitch_joint_X4",
    "left_ankle_roll_joint_X4",
    # Right leg (19-24)
    "right_hip_pitch_joint_X8",
    "right_hip_roll_joint_X8",
    "right_hip_yaw_joint_X8",
    "right_knee_joint_X8",
    "right_ankle_pitch_joint_X4",
    "right_ankle_roll_joint_X4",
]
NUM_JOINTS = len(ALL_JOINTS)

# Default positions from training (locomotion_env_cfg.py init_state.joint_pos)
DEFAULT_POSITIONS = np.array([
    # Left arm
    0.2, 0.0, 0.0, 0.35, 0.0, 0.0,
    # Right arm
    0.2, 0.0, 0.0, 0.35, 0.0, 0.0,
    # Waist
    0.0,
    # Left leg
    -0.05, 0.0, 0.0, 0.15, -0.1, 0.0,
    # Right leg
    -0.05, 0.0, 0.0, 0.15, -0.1, 0.0,
], dtype=np.float32)

# PD gains from Isaac Lab locomotion_env_cfg.py ImplicitActuator
PD_KP = np.array([
    # Left arm (0-5): reduced ~50% for DDS latency tolerance
    15.0, 15.0, 15.0, 15.0, 15.0, 15.0,
    # Right arm (6-11)
    15.0, 15.0, 15.0, 15.0, 15.0, 15.0,
    # Waist (12)
    75.0,
    # Left leg (13-18)
    100.0, 75.0, 75.0, 100.0, 30.0, 30.0,
    # Right leg (19-24)
    100.0, 75.0, 75.0, 100.0, 30.0, 30.0,
], dtype=np.float32)

PD_KD = np.array([
    # Left arm (0-5): increased relative to kp for extra damping
    6.0, 6.0, 6.0, 6.0, 6.0, 6.0,
    # Right arm (6-11)
    6.0, 6.0, 6.0, 6.0, 6.0, 6.0,
    # Waist (12)
    8.0,
    # Left leg (13-18)
    12.0, 12.0, 12.0, 12.0, 8.0, 8.0,
    # Right leg (19-24)
    12.0, 12.0, 12.0, 12.0, 8.0, 8.0,
], dtype=np.float32)

# Effort limits from Isaac Lab locomotion_env_cfg.py (effort_limit_sim per motor type)
EFFORT_LIMITS = np.array([
    # Left arm (0-5): X6=23Nm (moderate), X4=12.075Nm (moderate)
    23.0, 23.0, 12.075, 23.0, 12.075, 12.075,
    # Right arm (6-11)
    23.0, 23.0, 12.075, 23.0, 12.075, 12.075,
    # Waist (12): X8=72Nm (moderate)
    72.0,
    # Left leg (13-18): X8=72Nm, ankle_pitch=30Nm, ankle_roll=20Nm
    72.0, 72.0, 72.0, 72.0, 30.0, 20.0,
    # Right leg (19-24)
    72.0, 72.0, 72.0, 72.0, 30.0, 20.0,
], dtype=np.float32)


class ImplicitActuatorAdapter(Node):
    def __init__(self):
        super().__init__("implicit_actuator_adapter")

        # State
        self._joint_indices = None
        self._latest_joint_state = None
        self._target_positions = DEFAULT_POSITIONS.copy()
        self._ready = False

        # Subscribers
        self._joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )
        self._target_sub = self.create_subscription(
            Float64MultiArray, "/target_joint_positions", self._target_cb, 10
        )

        # Publisher — raw effort to JointGroupEffortController
        self._cmd_pub = self.create_publisher(
            Float64MultiArray, "/whole_body_controller/commands", 10
        )

        # PD timer at 200 Hz (sim_dt = 0.005s, matching Isaac Lab)
        self._pd_timer = self.create_timer(0.005, self._pd_tick)

        self.get_logger().info(
            f"ImplicitActuatorAdapter started: PD at 200 Hz, {NUM_JOINTS} joints. "
            f"Waiting for /joint_states..."
        )

    def _joint_state_cb(self, msg: JointState):
        if self._joint_indices is None:
            self._joint_indices = {}
            for name in ALL_JOINTS:
                if name in msg.name:
                    self._joint_indices[name] = msg.name.index(name)
                else:
                    self.get_logger().error(f"Joint '{name}' not in /joint_states!")
                    self._joint_indices = None
                    return
            self._ready = True
            self.get_logger().info(
                f"Joint mapping OK ({len(self._joint_indices)} joints). "
                f"Holding default positions. Publish to /target_joint_positions to command."
            )
        self._latest_joint_state = msg

    def _target_cb(self, msg: Float64MultiArray):
        if len(msg.data) == NUM_JOINTS:
            self._target_positions = np.array(msg.data, dtype=np.float32)
        else:
            self.get_logger().warn(
                f"Target has {len(msg.data)} values, expected {NUM_JOINTS}. Ignoring."
            )

    def _pd_tick(self):
        if not self._ready or self._latest_joint_state is None:
            return

        msg = self._latest_joint_state
        positions = np.zeros(NUM_JOINTS, dtype=np.float32)
        velocities = np.zeros(NUM_JOINTS, dtype=np.float32)

        for i, name in enumerate(ALL_JOINTS):
            idx = self._joint_indices[name]
            positions[i] = msg.position[idx]
            if idx < len(msg.velocity):
                velocities[i] = msg.velocity[idx]

        # PD torque — exact replica of Isaac Lab ImplicitActuator
        torques = PD_KP * (self._target_positions - positions) - PD_KD * velocities
        torques = np.clip(torques, -EFFORT_LIMITS, EFFORT_LIMITS)

        cmd = Float64MultiArray()
        cmd.data = torques.tolist()
        self._cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ImplicitActuatorAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
