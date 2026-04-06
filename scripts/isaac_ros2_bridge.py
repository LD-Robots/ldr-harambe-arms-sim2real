#!/usr/bin/python3
"""ROS2 bridge — receives Isaac Lab state via UDP, publishes to ROS2.

Run in a terminal with ROS2 sourced:
  source /opt/ros/jazzy/setup.bash
  python3 ~/Git/ldr-harambe-arms-sim2real/scripts/isaac_ros2_bridge.py
"""

import socket
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Quaternion, Vector3
from builtin_interfaces.msg import Time as RosTime

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

UDP_PORT = 9870
N_FLOATS = 1 + 25 + 25 + 4 + 3  # timestamp + pos + vel + quat + ang_vel = 58


class BridgeNode(Node):
    def __init__(self):
        super().__init__("isaac_ros2_bridge")
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.imu_pub = self.create_publisher(Imu, "/pelvis_imu/data", 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", UDP_PORT))
        self.sock.settimeout(0.1)

        self.create_timer(0.001, self._recv)  # 1kHz poll
        self.get_logger().info(f"Listening on UDP port {UDP_PORT}")
        self.count = 0

    def _recv(self):
        try:
            data, _ = self.sock.recvfrom(N_FLOATS * 4 + 64)
        except socket.timeout:
            return

        if len(data) < N_FLOATS * 4:
            return

        vals = struct.unpack(f"<{N_FLOATS}f", data[:N_FLOATS * 4])
        idx = 0
        sim_time = vals[idx]; idx += 1
        positions = vals[idx:idx+25]; idx += 25
        velocities = vals[idx:idx+25]; idx += 25
        quat_wxyz = vals[idx:idx+4]; idx += 4
        ang_vel = vals[idx:idx+3]; idx += 3

        sec = int(sim_time)
        nsec = int((sim_time - sec) * 1e9)
        stamp = RosTime(sec=sec, nanosec=nsec)

        # JointState
        js = JointState()
        js.header.stamp = stamp
        js.header.frame_id = "base_link"
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

        self.count += 1
        if self.count % 500 == 0:
            self.get_logger().info(f"Published {self.count} msgs, sim_t={sim_time:.1f}s")


def main():
    rclpy.init()
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.sock.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
