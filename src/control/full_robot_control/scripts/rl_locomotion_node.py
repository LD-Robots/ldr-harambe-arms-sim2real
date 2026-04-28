#!/usr/bin/env -S /usr/bin/python3
"""ROS 2 node — ONNX locomotion policy for Gazebo.

Sends position targets to ImplicitActuatorController (effort + internal PD at 200Hz)
via Float64MultiArray on /whole_body_controller/target_joint_positions.
Policy runs at 50Hz. Auto-resets robot pose if fallen after spawn.
"""

import os
import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray

# ── Joint ordering ──────────────────────────────────────────────────────────
POLICY_JOINTS = [
    "waist_yaw_joint_X8",
    "left_shoulder_pitch_joint_X6", "left_shoulder_roll_joint_X6",
    "left_shoulder_yaw_joint_X4", "left_elbow_pitch_joint_X6",
    "left_wrist_yaw_joint_X4", "left_wrist_roll_joint_X4",
    "right_shoulder_pitch_joint_X6", "right_shoulder_roll_joint_X6",
    "right_shoulder_yaw_joint_X4", "right_elbow_pitch_joint_X6",
    "right_wrist_yaw_joint_X4", "right_wrist_roll_joint_X4",
    "left_hip_pitch_joint_X8", "left_hip_roll_joint_X8",
    "left_hip_yaw_joint_X8", "left_knee_joint_X8",
    "left_ankle_pitch_joint_X4", "left_ankle_roll_joint_X4",
    "right_hip_pitch_joint_X8", "right_hip_roll_joint_X8",
    "right_hip_yaw_joint_X8", "right_knee_joint_X8",
    "right_ankle_pitch_joint_X4", "right_ankle_roll_joint_X4",
]

CTRL_JOINTS = [
    "left_shoulder_pitch_joint_X6", "left_shoulder_roll_joint_X6",
    "left_shoulder_yaw_joint_X4", "left_elbow_pitch_joint_X6",
    "left_wrist_yaw_joint_X4", "left_wrist_roll_joint_X4",
    "right_shoulder_pitch_joint_X6", "right_shoulder_roll_joint_X6",
    "right_shoulder_yaw_joint_X4", "right_elbow_pitch_joint_X6",
    "right_wrist_yaw_joint_X4", "right_wrist_roll_joint_X4",
    "waist_yaw_joint_X8",
    "left_hip_pitch_joint_X8", "left_hip_roll_joint_X8",
    "left_hip_yaw_joint_X8", "left_knee_joint_X8",
    "left_ankle_pitch_joint_X4", "left_ankle_roll_joint_X4",
    "right_hip_pitch_joint_X8", "right_hip_roll_joint_X8",
    "right_hip_yaw_joint_X8", "right_knee_joint_X8",
    "right_ankle_pitch_joint_X4", "right_ankle_roll_joint_X4",
]

POLICY_TO_CTRL = [CTRL_JOINTS.index(j) for j in POLICY_JOINTS]

# Isaac Lab default standing pose — V25+ training (hip+knee+ankle = 0, foot level)
# Source: ~/IsaacLab/source/isaaclab_assets/isaaclab_assets/robots/harambe.py init_state
ISAAC_DEFAULT = np.array([
    0.0,                                        # waist_yaw
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,             # left arm
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,             # right arm
    -0.25, 0.0, 0.0, 0.50, -0.25, 0.0,         # left leg
    -0.25, 0.0, 0.0, 0.50, -0.25, 0.0,         # right leg
], dtype=np.float32)

# Standup pose = same as ISAAC_DEFAULT (avoid pose mismatch at policy hand-off)
STANDUP_POS = ISAAC_DEFAULT.copy()

ACTION_SCALE = 0.25  # V29+ training value (was 0.10 for V6/V17 — must match the loaded policy)
POLICY_RATE = 50.0
N = 25

# Per-joint action clamp — tighter for Gazebo PID stability
ACTION_CLIP = np.array([
    1.0,                                         # waist_yaw
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0,             # left arm
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0,             # right arm
    1.5, 1.0, 0.8, 1.5, 1.5, 1.0,             # L: hip_p, hip_r, hip_y, knee, ank_p, ank_r
    1.5, 1.0, 0.8, 1.5, 1.5, 1.0,             # R: same
], dtype=np.float32)

# DART compensation: policy leans forward in PhysX but DART has less ankle resistance
# Positive hip_pitch bias = less forward lean, positive ankle = foot tilts back
DART_BIAS = np.zeros(N, dtype=np.float32)
# Re-enable selectively if Gazebo shows persistent forward lean (DART has lower
# ankle resistance than PhysX). Try +0.03 to +0.04 on hip_pitch/ankle_pitch.

GZ_RESET_CMD = (
    "gz service -s /world/empty/control "
    "--reqtype gz.msgs.WorldControl "
    "--reptype gz.msgs.Boolean "
    "--req 'reset: {all: true}' "
    "--timeout 2000"
)


def quat_rotate_inverse(qx, qy, qz, qw, vx, vy, vz):
    q = np.array([qw, qx, qy, qz])
    v = np.array([vx, vy, vz])
    a = v * (2.0 * q[0]**2 - 1.0)
    b = np.cross(q[1:], v) * q[0] * 2.0
    c = q[1:] * np.dot(q[1:], v) * 2.0
    return tuple(a - b + c)


class RLNode(Node):
    # States
    STATE_WAIT_ODOM = 0
    STATE_FALLEN_RESET = 1
    STATE_SETTLE = 2
    STATE_STANDUP = 3
    STATE_POLICY = 4

    def __init__(self):
        super().__init__("rl_locomotion_node")
        self.declare_parameter("policy_path", "")
        self.declare_parameter("cmd_vel_x", 0.0)
        self.declare_parameter("cmd_vel_y", 0.0)
        self.declare_parameter("cmd_vel_yaw", 0.0)

        pp = self.get_parameter("policy_path").get_parameter_value().string_value
        if not pp:
            self.get_logger().fatal("No policy_path!")
            raise SystemExit(1)
        self.session = ort.InferenceSession(pp, providers=["CPUExecutionProvider"])
        self.inp_name = self.session.get_inputs()[0].name
        inp_shape = self.session.get_inputs()[0].shape
        # V21+ policies (93 inputs) add torso IMU; older V6 (87 inputs) skip.
        self.expects_torso = (inp_shape[-1] == 93)
        self.get_logger().info(
            f"Loaded: {pp}  input_shape={inp_shape}  torso_imu={self.expects_torso}")

        self.pos_p = np.zeros(N, dtype=np.float32)
        self.vel_p = np.zeros(N, dtype=np.float32)
        self.ang_vel = np.zeros(3, dtype=np.float32)
        self.proj_grav = np.array([0., 0., -1.], dtype=np.float32)
        self.lin_vel = np.zeros(3, dtype=np.float32)
        # V21+ torso IMU observations (required for 93-dim policies)
        self.torso_ang_vel = np.zeros(3, dtype=np.float32)
        self.torso_proj_grav = np.array([0., 0., -1.], dtype=np.float32)
        self.cmd = np.array([
            self.get_parameter("cmd_vel_x").get_parameter_value().double_value,
            self.get_parameter("cmd_vel_y").get_parameter_value().double_value,
            self.get_parameter("cmd_vel_yaw").get_parameter_value().double_value,
        ], dtype=np.float32)
        self.last_act = np.zeros(N, dtype=np.float32)      # raw actions for obs
        self.smoothed_act = np.zeros(N, dtype=np.float32)  # EMA smoothed for targets
        self.p2idx = {n: i for i, n in enumerate(POLICY_JOINTS)}
        self.ready = False
        self.odom_received = False

        # State machine (uses IMU for orientation, odom optional for lin_vel)
        self.state = self.STATE_WAIT_ODOM
        self.state_step = 0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(JointState, "/joint_states", self._js, qos)
        self.create_subscription(Imu, "/pelvis_imu/data", self._imu, qos)
        self.create_subscription(Imu, "/torso_imu/data", self._torso_imu, qos)
        self.create_subscription(Odometry, "/model/harambe/odometry", self._odom, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd, 10)

        self.pub = self.create_publisher(
            Float64MultiArray, "/whole_body_controller/target_joint_positions", 10)

        self.create_timer(1.0 / POLICY_RATE, self._tick)
        self.get_logger().info(
            f"RL ImplicitActuator node: {POLICY_RATE}Hz, "
            f"cmd=({self.cmd[0]:.1f},{self.cmd[1]:.1f},{self.cmd[2]:.1f})")

    def _js(self, msg):
        for i, n in enumerate(msg.name):
            pi = self.p2idx.get(n)
            if pi is not None and i < len(msg.position):
                self.pos_p[pi] = msg.position[i]
                if i < len(msg.velocity):
                    self.vel_p[pi] = msg.velocity[i]
        self.ready = True

    def _imu(self, msg):
        q = msg.orientation
        self.ang_vel[:] = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ]
        gx, gy, gz = quat_rotate_inverse(q.x, q.y, q.z, q.w, 0., 0., -1.)
        self.proj_grav[:] = [gx, gy, gz]

        # FIX BUG #1: estimate lin_vel from IMU accelerometer
        # Integrate acceleration in body frame, with leak factor to prevent drift
        ax = msg.linear_acceleration.x - gx * 9.81
        ay = msg.linear_acceleration.y - gy * 9.81
        az = msg.linear_acceleration.z - gz * 9.81
        dt = 1.0 / POLICY_RATE
        LEAK = 0.95  # velocity decays to prevent drift
        self.lin_vel[0] = LEAK * self.lin_vel[0] + ax * dt
        self.lin_vel[1] = LEAK * self.lin_vel[1] + ay * dt
        self.lin_vel[2] = LEAK * self.lin_vel[2] + az * dt

        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info(f"First IMU: proj_grav={self.proj_grav}")

    def _torso_imu(self, msg):
        q = msg.orientation
        self.torso_ang_vel[:] = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ]
        gx, gy, gz = quat_rotate_inverse(q.x, q.y, q.z, q.w, 0., 0., -1.)
        self.torso_proj_grav[:] = [gx, gy, gz]

    def _odom(self, msg):
        # Use odom for accurate lin_vel if available (overrides IMU estimate)
        self.lin_vel[:] = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ]

    def _cmd(self, msg):
        self.cmd[:] = [msg.linear.x, msg.linear.y, msg.angular.z]

    def _is_upright(self):
        return self.proj_grav[2] < -0.8

    def _is_fallen(self):
        return self.proj_grav[2] > -0.3

    def _is_stable(self):
        # Tighter than _is_upright — also require low joint velocities.
        # Used to gate policy hand-off so impact dynamics fully decay.
        upright = self.proj_grav[2] < -0.95
        slow = float(np.max(np.abs(self.vel_p))) < 0.5
        return upright and slow

    def _send_standup(self):
        targets_ctrl = np.zeros(N, dtype=np.float64)
        for i in range(N):
            targets_ctrl[POLICY_TO_CTRL[i]] = float(STANDUP_POS[i])
        self._send_trajectory(targets_ctrl)

    def _tick(self):
        if not self.ready:
            return

        # ── STATE: wait for odom ──
        if self.state == self.STATE_WAIT_ODOM:
            self._send_standup()
            if self.odom_received:
                self.get_logger().info("IMU received, going to standup...")
                self.state = self.STATE_STANDUP
                self.state_step = 0
            return

        # ── STATE: detect fallen → reset ──
        if self.state == self.STATE_FALLEN_RESET:
            self._send_standup()
            self.state_step += 1
            # Wait 50 steps (1s) to check if robot is actually fallen
            if self.state_step < 50:
                return
            if self._is_fallen():
                self.get_logger().info(
                    f"Robot fallen (proj_grav={self.proj_grav}). Teleporting...")
                os.system(GZ_RESET_CMD + " > /dev/null 2>&1")
                self.state_step = 0
                self.state = self.STATE_SETTLE
            elif self._is_upright():
                self.get_logger().info("Robot upright! Skipping to standup.")
                self.state = self.STATE_STANDUP
                self.state_step = 0
            else:
                # Still tilted, wait more
                self.state_step = 25
            return

        # ── STATE: settle after teleport (2s) ──
        if self.state == self.STATE_SETTLE:
            self._send_standup()
            self.state_step += 1
            if self.state_step >= 150:
                self.get_logger().info(
                    f"Settle done. proj_grav={self.proj_grav}")
                if self._is_fallen():
                    # Still fallen after settle — try again
                    self.get_logger().info("Still fallen, retrying reset...")
                    self.state = self.STATE_FALLEN_RESET
                    self.state_step = 0
                else:
                    self.state = self.STATE_STANDUP
                    self.state_step = 0
            return

        # ── STATE: standup — hold ISAAC_DEFAULT pose until robot is stable ──
        if self.state == self.STATE_STANDUP:
            # Send ISAAC_DEFAULT (what policy expects)
            targets_ctrl = np.zeros(N, dtype=np.float64)
            for i in range(N):
                targets_ctrl[POLICY_TO_CTRL[i]] = float(ISAAC_DEFAULT[i])
            self._send_trajectory(targets_ctrl)
            self.state_step += 1
            # Require minimum 100 steps (2s) AND robot upright with low velocities.
            # Policy hand-off needs zero velocity to match training initial conditions.
            if self._is_stable() and self.state_step >= 100:
                self.get_logger().info(
                    f"Standup stable at step {self.state_step}! "
                    f"proj_grav={self.proj_grav} "
                    f"max_vel={float(np.max(np.abs(self.vel_p))):.3f}")
                self.state = self.STATE_POLICY
                self.state_step = 0
                self.last_act[:] = 0
                self.smoothed_act[:] = 0
                return
            if self.state_step >= 150:
                self.get_logger().info(
                    f"Standup complete! proj_grav={self.proj_grav} "
                    f"pos_legs={self.pos_p[13:19]}")
                self.state = self.STATE_POLICY
                self.state_step = 0
                self.last_act[:] = 0
                self.smoothed_act[:] = 0
            return

        # ── STATE: policy active ──
        if self.state == self.STATE_POLICY:
            # Fall detection disabled — reset doesn't work in Gazebo
            # if self._is_fallen():
            #     self.get_logger().info("Fell during policy! Resetting...")
            #     self.state = self.STATE_FALLEN_RESET
            #     self.state_step = 0
            #     return

            self.state_step += 1

            # Policy obs uses ISAAC default (what the policy was trained with).
            # V21+ policies (93 inputs) also consume torso IMU; older (87) skip.
            base_obs = [
                self.lin_vel, self.ang_vel, self.proj_grav, self.cmd,
                self.pos_p - ISAAC_DEFAULT, self.vel_p, self.last_act,
            ]
            if self.expects_torso:
                base_obs += [self.torso_proj_grav, self.torso_ang_vel]
            obs = np.concatenate(base_obs).astype(np.float32).clip(-100, 100)

            act = self.session.run(
                None, {self.inp_name: obs.reshape(1, -1)})[0][0]
            # FIX BUG #2: last_act stores RAW actions (for obs, matching training)
            # smoothed_act stores EMA for targets (deployment smoothing only)
            self.last_act[:] = act  # raw, unclipped — this is what obs sees

            # Clip for safety (only affects targets, not obs)
            act_clipped = np.clip(act, -ACTION_CLIP, ACTION_CLIP)

            # Light EMA smoothing — reduces Gazebo PID oscillation
            EMA_ALPHA = 0.85
            self.smoothed_act[:] = EMA_ALPHA * act_clipped + (1.0 - EMA_ALPHA) * self.smoothed_act

            # Policy target — no blend, direct output (matches Isaac Lab)
            tgt = self.smoothed_act * ACTION_SCALE + ISAAC_DEFAULT + DART_BIAS

            # Log every 2s for first 20s
            if self.state_step <= 500 and self.state_step % 100 == 1:
                leg_tgt = tgt[13:19]
                leg_act = self.pos_p[13:19]
                self.get_logger().info(
                    f"[step {self.state_step}] "
                    f"proj_grav={self.proj_grav}\n"
                    f"  left_leg tgt={np.array2string(leg_tgt, precision=3)} "
                    f"act={np.array2string(leg_act, precision=3)}\n"
                    f"  raw_act min={act.min():.2f} max={act.max():.2f}")

            targets_ctrl = np.zeros(N, dtype=np.float64)
            for i in range(N):
                targets_ctrl[POLICY_TO_CTRL[i]] = float(tgt[i])
            self._send_trajectory(targets_ctrl)

    def _send_trajectory(self, positions):
        msg = Float64MultiArray()
        msg.data = positions.tolist()
        self.pub.publish(msg)


def main():
    rclpy.init()
    n = RLNode()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        n.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()
