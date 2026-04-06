#!/usr/bin/python3
"""MuJoCo ↔ ROS2 bidirectional bridge.

Runs MuJoCo physics internally and bridges to ROS2:
  Publishes:  /joint_states, /pelvis_imu/data, /robot_description
  Subscribes: /whole_body_controller/joint_trajectory

Works with rl_locomotion_node.py or any ROS2 controller.

Usage:
  # Terminal 1: MuJoCo bridge
  source /opt/ros/jazzy/setup.bash
  /usr/bin/python3 ~/Git/ldr-harambe-arms-sim2real/scripts/mujoco_ros2_bridge.py

  # Terminal 2: RL policy node
  source /opt/ros/jazzy/setup.bash && cd ~/Git/ldr-harambe-arms-sim2real && source install/setup.bash
  ros2 run full_robot_control rl_locomotion_node.py --ros-args \
    -p policy_path:=$HOME/IsaacLab/logs/rsl_rl/harambe_flat/2026-04-01_13-58-55/exported/policy.onnx \
    -p cmd_vel_x:=0.5
"""

import os
import time
import threading
import numpy as np
import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from builtin_interfaces.msg import Time as RosTime

SCENE_PATH = "/tmp/harambe_scene.xml"

# Same joint order as controller config
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

# PD gains per CTRL_JOINTS order (real motor values)
KP = {
    "waist_yaw_joint_X8": 416.0,
    "left_shoulder_pitch_joint_X6": 435.0, "left_shoulder_roll_joint_X6": 440.0,
    "left_shoulder_yaw_joint_X4": 93.0, "left_elbow_pitch_joint_X6": 118.0,
    "left_wrist_yaw_joint_X4": 38.5, "left_wrist_roll_joint_X4": 45.3,
    "right_shoulder_pitch_joint_X6": 435.0, "right_shoulder_roll_joint_X6": 440.0,
    "right_shoulder_yaw_joint_X4": 93.0, "right_elbow_pitch_joint_X6": 118.0,
    "right_wrist_yaw_joint_X4": 38.5, "right_wrist_roll_joint_X4": 45.3,
    "left_hip_pitch_joint_X8": 1553.0, "left_hip_roll_joint_X8": 1548.0,
    "left_hip_yaw_joint_X8": 252.0, "left_knee_joint_X8": 324.0,
    "left_ankle_pitch_joint_X4": 524.0, "left_ankle_roll_joint_X4": 157.0,
    "right_hip_pitch_joint_X8": 1553.0, "right_hip_roll_joint_X8": 1548.0,
    "right_hip_yaw_joint_X8": 252.0, "right_knee_joint_X8": 324.0,
    "right_ankle_pitch_joint_X4": 524.0, "right_ankle_roll_joint_X4": 157.0,
}
KD = {
    "waist_yaw_joint_X8": 24.0,
    "left_shoulder_pitch_joint_X6": 29.0, "left_shoulder_roll_joint_X6": 29.3,
    "left_shoulder_yaw_joint_X4": 6.2, "left_elbow_pitch_joint_X6": 7.9,
    "left_wrist_yaw_joint_X4": 2.6, "left_wrist_roll_joint_X4": 3.0,
    "right_shoulder_pitch_joint_X6": 29.0, "right_shoulder_roll_joint_X6": 29.3,
    "right_shoulder_yaw_joint_X4": 6.2, "right_elbow_pitch_joint_X6": 7.9,
    "right_wrist_yaw_joint_X4": 2.6, "right_wrist_roll_joint_X4": 3.0,
    "left_hip_pitch_joint_X8": 77.7, "left_hip_roll_joint_X8": 77.4,
    "left_hip_yaw_joint_X8": 12.6, "left_knee_joint_X8": 16.2,
    "left_ankle_pitch_joint_X4": 26.2, "left_ankle_roll_joint_X4": 7.9,
    "right_hip_pitch_joint_X8": 77.7, "right_hip_roll_joint_X8": 77.4,
    "right_hip_yaw_joint_X8": 12.6, "right_knee_joint_X8": 16.2,
    "right_ankle_pitch_joint_X4": 26.2, "right_ankle_roll_joint_X4": 7.9,
}
EFFORT_LIM = {
    "waist_yaw_joint_X8": 120.0,
    "left_shoulder_pitch_joint_X6": 60.0, "left_shoulder_roll_joint_X6": 60.0,
    "left_shoulder_yaw_joint_X4": 34.0, "left_elbow_pitch_joint_X6": 60.0,
    "left_wrist_yaw_joint_X4": 34.0, "left_wrist_roll_joint_X4": 34.0,
    "right_shoulder_pitch_joint_X6": 60.0, "right_shoulder_roll_joint_X6": 60.0,
    "right_shoulder_yaw_joint_X4": 34.0, "right_elbow_pitch_joint_X6": 60.0,
    "right_wrist_yaw_joint_X4": 34.0, "right_wrist_roll_joint_X4": 34.0,
    "left_hip_pitch_joint_X8": 120.0, "left_hip_roll_joint_X8": 120.0,
    "left_hip_yaw_joint_X8": 120.0, "left_knee_joint_X8": 120.0,
    "left_ankle_pitch_joint_X4": 272.0, "left_ankle_roll_joint_X4": 81.6,
    "right_hip_pitch_joint_X8": 120.0, "right_hip_roll_joint_X8": 120.0,
    "right_hip_yaw_joint_X8": 120.0, "right_knee_joint_X8": 120.0,
    "right_ankle_pitch_joint_X4": 272.0, "right_ankle_roll_joint_X4": 81.6,
}

# Default standing pose for initialization
DEFAULT_POS = {
    "left_hip_pitch_joint_X8": -0.28, "left_knee_joint_X8": 0.56,
    "left_ankle_pitch_joint_X4": -0.28,
    "right_hip_pitch_joint_X8": -0.28, "right_knee_joint_X8": 0.56,
    "right_ankle_pitch_joint_X4": -0.28,
}

SIM_DT = 0.001
PUB_RATE = 50.0  # Hz — publish joint_states at policy rate
PUB_DT = 1.0 / PUB_RATE
N_SUBSTEPS = int(PUB_DT / SIM_DT)


class MujocoBridge(Node):
    def __init__(self):
        super().__init__("mujoco_ros2_bridge")

        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.imu_pub = self.create_publisher(Imu, "/pelvis_imu/data", 10)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.urdf_pub = self.create_publisher(String, "/robot_description", latched)

        self.create_subscription(
            JointTrajectory,
            "/whole_body_controller/joint_trajectory",
            self._traj_cb, 10
        )

        # Target positions from controller (updated by trajectory callback)
        self.targets = {}  # joint_name -> target_pos
        self.targets_lock = threading.Lock()

        # Publish URDF
        urdf_path = os.path.expanduser(
            "~/Git/ldr-harambe-arms-sim2real/src/robot_description/"
            "full_robot_description/urdf/harambe_full_for_isaac.urdf"
        )
        try:
            with open(urdf_path) as f:
                self.urdf_pub.publish(String(data=f.read()))
            self.get_logger().info("Published /robot_description")
        except Exception:
            pass

        self.get_logger().info("MuJoCo bridge ready — waiting for /whole_body_controller/joint_trajectory")

    def _traj_cb(self, msg):
        if not msg.points:
            return
        pt = msg.points[0]
        with self.targets_lock:
            for i, name in enumerate(msg.joint_names):
                if i < len(pt.positions):
                    self.targets[name] = pt.positions[i]

    def get_targets(self):
        with self.targets_lock:
            return dict(self.targets)

    def publish_state(self, sim_time, joint_names, positions, velocities, quat_wxyz, ang_vel):
        sec = int(sim_time)
        nsec = int((sim_time - sec) * 1e9)
        stamp = RosTime(sec=sec, nanosec=nsec)

        js = JointState()
        js.header.stamp = stamp
        js.header.frame_id = "urdf_base"
        js.name = list(joint_names)
        js.position = [float(p) for p in positions]
        js.velocity = [float(v) for v in velocities]
        self.js_pub.publish(js)

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
    node = MujocoBridge()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Load MuJoCo
    node.get_logger().info(f"Loading: {SCENE_PATH}")
    model = mujoco.MjModel.from_xml_path(SCENE_PATH)
    data = mujoco.MjData(model)
    model.opt.timestep = SIM_DT

    # Map CTRL_JOINTS to MuJoCo joint ids
    joint_map = {}  # joint_name -> (mj_joint_id, qpos_addr, dof_addr)
    for name in CTRL_JOINTS:
        for j in range(model.njnt):
            if model.joint(j).name == name:
                joint_map[name] = (j, model.jnt_qposadr[j], model.jnt_dofadr[j])
                break

    node.get_logger().info(f"Mapped {len(joint_map)}/25 joints")

    # Free joint
    root_id = None
    for j in range(model.njnt):
        if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
            root_id = j
            break

    root_qpos = model.jnt_qposadr[root_id] if root_id else 0
    root_dof = model.jnt_dofadr[root_id] if root_id else 0

    # Init pose
    mujoco.mj_resetData(model, data)
    if root_id is not None:
        data.qpos[root_qpos:root_qpos+3] = [0, 0, 1.215]
        data.qpos[root_qpos+3:root_qpos+7] = [1, 0, 0, 0]

    for name, pos in DEFAULT_POS.items():
        if name in joint_map:
            data.qpos[joint_map[name][1]] = pos

    mujoco.mj_forward(model, data)

    sim_time = 0.0
    step_count = 0

    node.get_logger().info("Simulation running — press ESC in viewer to quit")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running() and rclpy.ok():
            t0 = time.time()

            # Get targets from ROS2 trajectory commands
            tgts = node.get_targets()

            # Apply PD torques based on targets
            for name, (mj_id, qpos_addr, dof_addr) in joint_map.items():
                if name in tgts:
                    target = tgts[name]
                else:
                    target = DEFAULT_POS.get(name, 0.0)

                pos_err = target - data.qpos[qpos_addr]
                vel = data.qvel[dof_addr]
                torque = KP[name] * pos_err - KD[name] * vel
                torque = np.clip(torque, -EFFORT_LIM[name], EFFORT_LIM[name])
                data.qfrc_applied[dof_addr] = torque

            # Step physics
            for _ in range(N_SUBSTEPS):
                mujoco.mj_step(model, data)

            sim_time += PUB_DT
            step_count += 1

            # Read state and publish
            positions = []
            velocities = []
            for name in CTRL_JOINTS:
                if name in joint_map:
                    _, qa, da = joint_map[name]
                    positions.append(data.qpos[qa])
                    velocities.append(data.qvel[da])
                else:
                    positions.append(0.0)
                    velocities.append(0.0)

            if root_id is not None:
                quat = data.qpos[root_qpos+3:root_qpos+7].copy()  # w,x,y,z
                ang_vel = data.qvel[root_dof+3:root_dof+6].copy()
            else:
                quat = np.array([1, 0, 0, 0])
                ang_vel = np.zeros(3)

            node.publish_state(sim_time, CTRL_JOINTS, positions, velocities, quat, ang_vel)

            if step_count % 100 == 0:
                h = data.qpos[root_qpos + 2] if root_id else 0
                n_tgts = len(tgts)
                node.get_logger().info(f"[{step_count}] h={h:.3f}m targets={n_tgts}/25")

            viewer.sync()

            elapsed = time.time() - t0
            if PUB_DT - elapsed > 0:
                time.sleep(PUB_DT - elapsed)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
