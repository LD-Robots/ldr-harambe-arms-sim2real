#!/usr/bin/env python3
"""
Bidirectional bridge: Isaac Lab (UDP) ↔ ROS 2 topics.

Isaac Lab sends on UDP 9870: joint_states(25pos+25vel) + IMU(4quat+3angvel+3linvel) = 60 floats
Isaac Lab receives on UDP 9871: joint_commands(25 positions) = 25 floats

This node:
  - Publishes /joint_states (sensor_msgs/JointState)
  - Publishes /pelvis_imu/data (sensor_msgs/Imu)
  - Subscribes /whole_body_controller/commands (std_msgs/Float64MultiArray)
  - Forwards commands to Isaac via UDP

Usage:
    python3 isaac_imu_bridge.py
"""

import socket
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
import numpy as np


HARDWARE_JOINT_ORDER = [
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

# Isaac Lab URDF order (first 25 joints as discovered by PhysX)
ISAAC_JOINT_ORDER = [
    "left_hip_pitch_joint_X8", "right_hip_pitch_joint_X8",
    "waist_yaw_joint_X8",
    "left_hip_roll_joint_X8", "right_hip_roll_joint_X8",
    "left_shoulder_pitch_joint_X6", "right_shoulder_pitch_joint_X6",
    "left_hip_yaw_joint_X8", "right_hip_yaw_joint_X8",
    "left_shoulder_roll_joint_X6", "right_shoulder_roll_joint_X6",
    "left_knee_joint_X8", "right_knee_joint_X8",
    "left_shoulder_yaw_joint_X4", "right_shoulder_yaw_joint_X4",
    "left_ankle_pitch_joint_X4", "right_ankle_pitch_joint_X4",
    "left_elbow_pitch_joint_X6", "right_elbow_pitch_joint_X6",
    "left_ankle_roll_joint_X4", "right_ankle_roll_joint_X4",
    "left_wrist_yaw_joint_X4", "right_wrist_yaw_joint_X4",
    "left_wrist_roll_joint_X4", "right_wrist_roll_joint_X4",
]

# Build mapping indices
# isaac_to_hw[i] = index in ISAAC_JOINT_ORDER of HARDWARE_JOINT_ORDER[i]
isaac_to_hw = [ISAAC_JOINT_ORDER.index(name) for name in HARDWARE_JOINT_ORDER]
# hw_to_isaac[i] = index in HARDWARE_JOINT_ORDER of ISAAC_JOINT_ORDER[i]
hw_to_isaac = [HARDWARE_JOINT_ORDER.index(name) for name in ISAAC_JOINT_ORDER]


def quat_rotate_inverse(q_wxyz, v):
    """Rotate vector v by inverse of quaternion q (w,x,y,z)."""
    w, x, y, z = q_wxyz
    q_vec = np.array([x, y, z])
    a = v * (2.0 * w * w - 1.0)
    b = np.cross(q_vec, v) * w * 2.0
    c = q_vec * (np.dot(q_vec, v) * 2.0)
    return a - b + c


class IsaacBridge(Node):
    def __init__(self):
        super().__init__("isaac_bridge")

        # Publishers
        self.js_pub = self.create_publisher(JointState, "/isaac/joint_states_hw", 10)
        self.imu_pub = self.create_publisher(Imu, "/pelvis_imu/data", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        # Subscriber for commands from deploy script
        self.cmd_sub = self.create_subscription(
            Float64MultiArray, "/whole_body_controller/commands",
            self.cmd_cb, 10)

        # UDP sockets
        self.sock_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_in.bind(("127.0.0.1", 9870))
        self.sock_in.setblocking(False)

        self.sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_addr = ("127.0.0.1", 9871)

        # Timer at 200Hz
        self.timer = self.create_timer(0.005, self.timer_cb)

        self.get_logger().info(
            "Isaac bridge: UDP 9870 → /joint_states + /pelvis_imu/data, "
            "/whole_body_controller/commands → UDP 9871")

    def cmd_cb(self, msg):
        """Forward joint commands to Isaac Lab (reorder HW → Isaac URDF order)."""
        if len(msg.data) == 25:
            hw_cmd = list(msg.data)
            # Reorder: HARDWARE_JOINT_ORDER → Isaac URDF order
            isaac_cmd = [hw_cmd[hw_to_isaac[i]] for i in range(25)]
            data = struct.pack('25f', *isaac_cmd)
            try:
                self.sock_out.sendto(data, self.cmd_addr)
            except OSError:
                pass

    def timer_cb(self):
        """Receive state from Isaac Lab and publish on ROS 2."""
        try:
            data, _ = self.sock_in.recvfrom(4096)
        except BlockingIOError:
            return

        if len(data) != 60 * 4:  # 60 floats
            return

        vals = struct.unpack('60f', data)
        isaac_pos = vals[0:25]
        isaac_vel = vals[25:50]
        qw, qx, qy, qz = vals[50:54]
        ang_vel = vals[54:57]
        lin_vel = vals[57:60]

        # Reorder: Isaac URDF order → HARDWARE_JOINT_ORDER
        hw_pos = [isaac_pos[isaac_to_hw[i]] for i in range(25)]
        hw_vel = [isaac_vel[isaac_to_hw[i]] for i in range(25)]

        now = self.get_clock().now().to_msg()

        # Publish JointState in HARDWARE_JOINT_ORDER
        js = JointState()
        js.header.stamp = now
        js.header.frame_id = ""
        js.name = list(HARDWARE_JOINT_ORDER)
        js.position = hw_pos
        js.velocity = hw_vel
        js.effort = []
        self.js_pub.publish(js)

        # Publish IMU — simulate Gazebo BNO055 pelvis IMU format
        # so deploy_full_body_ros2.py with use_gazebo_imu=true works correctly.
        #
        # Gazebo BNO055 IMU mount: rpy="0 pi/2 pi" on urdf_base
        # sensor_to_base transform: v_base = [-v_sensor[2], -v_sensor[1], -v_sensor[0]]
        # Gravity in sensor frame when upright: [-1, 0, 0] (lin_acc.x = -9.81)
        #
        # Isaac gives: quat world frame (w,x,y,z), ang_vel body frame, projected_gravity body frame
        # We need to: transform body frame → sensor frame (inverse of sensor_to_base)
        # base_to_sensor: v_sensor = [-v_base[2], -v_base[1], -v_base[0]]

        q_body = np.array([qw, qx, qy, qz])
        grav_body = quat_rotate_inverse(q_body, np.array([0.0, 0.0, -1.0]))
        # Align gravity convention with Isaac projected_gravity_b used by internal ONNX path.
        # Keep z unchanged (up/down), flip x/y signs.
        grav_body = np.array([-grav_body[0], -grav_body[1], grav_body[2]])
        ang_vel_body = np.array(ang_vel)

        # Transform body → sensor frame (inverse of sensor_to_base)
        # sensor_to_base: [-z, -y, -x], so base_to_sensor: [-z, -y, -x] (same transform)
        grav_sensor = np.array([-grav_body[2], -grav_body[1], -grav_body[0]])
        ang_vel_sensor = np.array([-ang_vel_body[2], -ang_vel_body[1], -ang_vel_body[0]])

        # For orientation: publish identity-ish quaternion (deploy script extracts gravity from quat)
        # The deploy script does: quat_rotate_inverse(q, gravity_ref) then sensor_to_base
        # With gravity_ref = [1, 0, 0] for Gazebo IMU
        # We need q such that quat_rotate_inverse(q, [1,0,0]) = grav_sensor
        # Simplest: publish the orientation that makes deploy script reconstruct correct proj_gravity
        # Since deploy does: proj_gravity = sensor_to_base(quat_rotate_inverse(q, [1,0,0]))
        # We want: proj_gravity = grav_body
        # So: sensor_to_base(quat_rotate_inverse(q, [1,0,0])) = grav_body
        # i.e.: quat_rotate_inverse(q, [1,0,0]) = base_to_sensor(grav_body) = grav_sensor
        # This means q must rotate [1,0,0] to grav_sensor

        # Build quaternion that rotates [1,0,0] → grav_sensor (normalized)
        grav_sensor_n = grav_sensor / (np.linalg.norm(grav_sensor) + 1e-8)
        ref = np.array([1.0, 0.0, 0.0])
        # Rotation axis = cross(ref, grav_sensor_n), angle = acos(dot)
        dot = np.clip(np.dot(ref, grav_sensor_n), -1, 1)
        cross = np.cross(ref, grav_sensor_n)
        cross_norm = np.linalg.norm(cross)
        if cross_norm < 1e-6:
            if dot > 0:
                sim_qw, sim_qx, sim_qy, sim_qz = 1, 0, 0, 0
            else:
                sim_qw, sim_qx, sim_qy, sim_qz = 0, 0, 1, 0
        else:
            axis = cross / cross_norm
            angle = np.arccos(dot)
            half = angle / 2
            sim_qw = np.cos(half)
            sim_qx = axis[0] * np.sin(half)
            sim_qy = axis[1] * np.sin(half)
            sim_qz = axis[2] * np.sin(half)

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = "harambe/urdf_base/pelvis_imu"
        imu.orientation.x = float(sim_qx)
        imu.orientation.y = float(sim_qy)
        imu.orientation.z = float(sim_qz)
        imu.orientation.w = float(sim_qw)
        imu.angular_velocity.x = float(ang_vel_sensor[0])
        imu.angular_velocity.y = float(ang_vel_sensor[1])
        imu.angular_velocity.z = float(ang_vel_sensor[2])
        imu.linear_acceleration.x = float(-grav_sensor[0] * 9.81)
        imu.linear_acceleration.y = float(-grav_sensor[1] * 9.81)
        imu.linear_acceleration.z = float(-grav_sensor[2] * 9.81)
        self.imu_pub.publish(imu)

        # Publish Odometry (base_lin_vel in body frame)
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "urdf_base"
        odom.twist.twist.linear.x = float(lin_vel[0])
        odom.twist.twist.linear.y = float(lin_vel[1])
        odom.twist.twist.linear.z = float(lin_vel[2])
        self.odom_pub.publish(odom)

    def destroy_node(self):
        self.sock_in.close()
        self.sock_out.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = IsaacBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
