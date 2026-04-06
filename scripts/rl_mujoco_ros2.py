#!/usr/bin/python3
"""Standalone ROS2 node: V9 RL policy + MuJoCo physics.

Runs the trained ONNX policy in MuJoCo and publishes to ROS2:
  /joint_states (sensor_msgs/JointState)
  /pelvis_imu/data (sensor_msgs/Imu)
  /robot_description (std_msgs/String) - for RViz RobotModel

Subscribes:
  /cmd_vel (geometry_msgs/Twist) - velocity commands

Usage:
  source /opt/ros/jazzy/setup.bash
  /usr/bin/python3 ~/Git/ldr-harambe-arms-sim2real/scripts/rl_mujoco_ros2.py

  # In another terminal:
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
  # Or for standing:
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}}"
"""

import os
import time
import threading
import numpy as np
import mujoco
import mujoco.viewer
import onnxruntime as ort

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist, Quaternion, Vector3
from std_msgs.msg import String
from builtin_interfaces.msg import Time as RosTime

# Paths
SCENE_PATH = "/tmp/harambe_scene.xml"  # Generated from harambe_full_for_isaac.urdf with meshes
POLICY_PATH = os.path.expanduser(
    "~/IsaacLab/logs/rsl_rl/harambe_flat/2026-04-01_13-58-55/exported/policy.onnx"
)
URDF_PATH = os.path.expanduser(
    "~/Git/ldr-harambe-arms-sim2real/src/robot_description/full_robot_description/urdf/harambe_full_for_isaac.urdf"
)

# Policy joint order (matches Isaac Lab training)
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

ISAAC_DEFAULT = np.array([
    0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.28, 0.0, 0.0, 0.56, -0.28, 0.0,
    -0.28, 0.0, 0.0, 0.56, -0.28, 0.0,
], dtype=np.float32)

KP = np.array([
    416.0,
    435.0, 440.0, 93.0, 118.0, 38.5, 45.3,
    435.0, 440.0, 93.0, 118.0, 38.5, 45.3,
    1553.0, 1548.0, 252.0, 324.0, 524.0, 157.0,
    1553.0, 1548.0, 252.0, 324.0, 524.0, 157.0,
], dtype=np.float32)

KD = np.array([
    24.0,
    29.0, 29.3, 6.2, 7.9, 2.6, 3.0,
    29.0, 29.3, 6.2, 7.9, 2.6, 3.0,
    77.7, 77.4, 12.6, 16.2, 26.2, 7.9,
    77.7, 77.4, 12.6, 16.2, 26.2, 7.9,
], dtype=np.float32)

EFFORT_LIMITS = np.array([
    120.0,
    60.0, 60.0, 34.0, 60.0, 34.0, 34.0,
    60.0, 60.0, 34.0, 60.0, 34.0, 34.0,
    120.0, 120.0, 120.0, 120.0, 272.0, 81.6,
    120.0, 120.0, 120.0, 120.0, 272.0, 81.6,
], dtype=np.float32)

ACTION_SCALE = 0.10
POLICY_DT = 0.02
SIM_DT = 0.001
N_SUBSTEPS = int(POLICY_DT / SIM_DT)


def quat_rotate_inverse(quat, vec):
    q = quat
    a = vec * (2.0 * q[0]**2 - 1.0)
    b = np.cross(q[1:4], vec) * q[0] * 2.0
    c = q[1:4] * np.dot(q[1:4], vec) * 2.0
    return a - b + c


class MujocoROS2Node(Node):
    def __init__(self):
        super().__init__("rl_mujoco_sim")

        # Publishers
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.imu_pub = self.create_publisher(Imu, "/pelvis_imu/data", 10)

        # Latched URDF publisher
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.urdf_pub = self.create_publisher(String, "/robot_description", latched_qos)

        # Publish URDF for RViz
        try:
            with open(URDF_PATH) as f:
                urdf_str = f.read()
            self.urdf_pub.publish(String(data=urdf_str))
            self.get_logger().info(f"Published URDF ({len(urdf_str)} bytes)")
        except Exception as e:
            self.get_logger().warn(f"Could not publish URDF: {e}")

        # Subscriber
        self.cmd_vel = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)

        self.get_logger().info("MuJoCo ROS2 node ready — /joint_states, /pelvis_imu/data, /cmd_vel")

    def _cmd_cb(self, msg):
        self.cmd_vel[0] = msg.linear.x
        self.cmd_vel[1] = msg.linear.y
        self.cmd_vel[2] = msg.angular.z
        self.get_logger().info(f"cmd_vel: vx={msg.linear.x:.2f} vy={msg.linear.y:.2f} yaw={msg.angular.z:.2f}")

    def publish_state(self, sim_time, positions, velocities, quat_wxyz, ang_vel):
        sec = int(sim_time)
        nsec = int((sim_time - sec) * 1e9)
        stamp = RosTime(sec=sec, nanosec=nsec)

        # JointState
        js = JointState()
        js.header.stamp = stamp
        js.header.frame_id = "urdf_base"
        js.name = POLICY_JOINTS
        js.position = [float(p) for p in positions]
        js.velocity = [float(v) for v in velocities]
        self.js_pub.publish(js)

        # IMU
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "pelvis_imu"
        imu.orientation = Quaternion(
            x=float(quat_wxyz[1]), y=float(quat_wxyz[2]),
            z=float(quat_wxyz[3]), w=float(quat_wxyz[0]))
        imu.angular_velocity = Vector3(
            x=float(ang_vel[0]), y=float(ang_vel[1]), z=float(ang_vel[2]))
        imu.linear_acceleration = Vector3(x=0.0, y=0.0, z=9.81)
        self.imu_pub.publish(imu)


def main():
    rclpy.init()
    node = MujocoROS2Node()

    # Spin ROS2 in background
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Load MuJoCo model
    node.get_logger().info(f"Loading MuJoCo: {SCENE_PATH}")
    model = mujoco.MjModel.from_xml_path(SCENE_PATH)
    data = mujoco.MjData(model)
    model.opt.timestep = SIM_DT

    # Map joints
    policy_to_mj = []
    for pj in POLICY_JOINTS:
        found = -1
        for j in range(model.njnt):
            if model.joint(j).name == pj:
                found = j
                break
        policy_to_mj.append(found)

    mapped = sum(1 for j in policy_to_mj if j >= 0)
    node.get_logger().info(f"Mapped {mapped}/25 joints")

    # Find free joint
    root_joint_id = None
    for j in range(model.njnt):
        if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
            root_joint_id = j
            break

    root_qpos = model.jnt_qposadr[root_joint_id] if root_joint_id else 0
    root_dof = model.jnt_dofadr[root_joint_id] if root_joint_id else 0

    # Set initial pose
    mujoco.mj_resetData(model, data)
    if root_joint_id is not None:
        data.qpos[root_qpos:root_qpos+3] = [0, 0, 1.215]
        data.qpos[root_qpos+3:root_qpos+7] = [1, 0, 0, 0]

    for pi, mj_id in enumerate(policy_to_mj):
        if mj_id >= 0:
            data.qpos[model.jnt_qposadr[mj_id]] = ISAAC_DEFAULT[pi]

    mujoco.mj_forward(model, data)

    # Load policy
    node.get_logger().info(f"Loading policy: {POLICY_PATH}")
    session = ort.InferenceSession(POLICY_PATH, providers=["CPUExecutionProvider"])
    inp_name = session.get_inputs()[0].name

    last_act = np.zeros(25, dtype=np.float32)
    step_count = 0
    sim_time = 0.0

    node.get_logger().info("Starting simulation loop — send /cmd_vel to control")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running() and rclpy.ok():
            t0 = time.time()

            # Read state
            pos_p = np.zeros(25, dtype=np.float32)
            vel_p = np.zeros(25, dtype=np.float32)
            for pi, mj_id in enumerate(policy_to_mj):
                if mj_id >= 0:
                    pos_p[pi] = data.qpos[model.jnt_qposadr[mj_id]]
                    vel_p[pi] = data.qvel[model.jnt_dofadr[mj_id]]

            if root_joint_id is not None:
                qw = data.qpos[root_qpos + 3]
                qx = data.qpos[root_qpos + 4]
                qy = data.qpos[root_qpos + 5]
                qz = data.qpos[root_qpos + 6]
                # MuJoCo: ang_vel is body frame (correct), lin_vel is WORLD frame
                ang_vel = data.qvel[root_dof+3:root_dof+6].astype(np.float32)
                lin_vel_world = data.qvel[root_dof:root_dof+3].astype(np.float32)
                # Rotate lin_vel from world to body frame (Isaac Lab expects body frame)
                quat = np.array([qw, qx, qy, qz])
                lin_vel = quat_rotate_inverse(quat, lin_vel_world).astype(np.float32)
            else:
                qw, qx, qy, qz = 1, 0, 0, 0
                ang_vel = np.zeros(3, dtype=np.float32)
                lin_vel = np.zeros(3, dtype=np.float32)

            proj_grav = quat_rotate_inverse(
                np.array([qw, qx, qy, qz]), np.array([0., 0., -1.])
            ).astype(np.float32)

            # Policy
            obs = np.concatenate([
                lin_vel, ang_vel, proj_grav, node.cmd_vel,
                pos_p - ISAAC_DEFAULT, vel_p, last_act,
            ]).astype(np.float32).clip(-100, 100)

            actions = session.run(None, {inp_name: obs.reshape(1, -1)})[0][0]
            last_act = actions.copy()
            targets = actions * ACTION_SCALE + ISAAC_DEFAULT

            # PD torques
            for pi, mj_id in enumerate(policy_to_mj):
                if mj_id >= 0:
                    pos_err = targets[pi] - data.qpos[model.jnt_qposadr[mj_id]]
                    vel_err = -data.qvel[model.jnt_dofadr[mj_id]]
                    torque = np.clip(
                        KP[pi] * pos_err + KD[pi] * vel_err,
                        -EFFORT_LIMITS[pi], EFFORT_LIMITS[pi]
                    )
                    data.qfrc_applied[model.jnt_dofadr[mj_id]] = torque

            # Step physics
            for _ in range(N_SUBSTEPS):
                mujoco.mj_step(model, data)

            sim_time += POLICY_DT
            step_count += 1

            # Publish to ROS2
            node.publish_state(
                sim_time, pos_p, vel_p,
                np.array([qw, qx, qy, qz]), ang_vel
            )

            if step_count % 100 == 0:
                h = data.qpos[root_qpos + 2] if root_joint_id else 0
                node.get_logger().info(
                    f"[{step_count}] h={h:.3f}m grav={proj_grav} cmd={node.cmd_vel}"
                )

            viewer.sync()

            elapsed = time.time() - t0
            if POLICY_DT - elapsed > 0:
                time.sleep(POLICY_DT - elapsed)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
