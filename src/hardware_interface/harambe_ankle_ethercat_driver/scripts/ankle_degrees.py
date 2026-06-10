#!/usr/bin/env python3
"""Debug aid: republish the ankle joint positions in DEGREES.

/joint_states carries the four ankle joints (pitch/roll, both legs) already
transformed to joint space, but in radians. This node converts just those four
to degrees and publishes them on /ankle_degrees as a sensor_msgs/JointState, so
they are easy to read in `ros2 topic echo` or plot per-name in PlotJuggler.

It is a passive listener (no command path, safe to run any time, including the
read-only viewer). Run with:  ros2 run harambe_ankle_ethercat_driver ankle_degrees.py
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ANKLE_JOINTS = [
    "left_ankle_pitch_joint_X4",
    "left_ankle_roll_joint_X4",
    "right_ankle_pitch_joint_X4",
    "right_ankle_roll_joint_X4",
]


class AnkleDegrees(Node):
    def __init__(self):
        super().__init__("ankle_degrees")
        self.pub = self.create_publisher(JointState, "ankle_degrees", 10)
        self.create_subscription(JointState, "joint_states", self._on_joint_states, 10)
        self.get_logger().info(
            "Publishing ankle pitch/roll (deg) on /ankle_degrees from /joint_states")

    def _on_joint_states(self, msg: JointState):
        index = {name: i for i, name in enumerate(msg.name)}
        out = JointState()
        out.header = msg.header
        for joint in ANKLE_JOINTS:
            i = index.get(joint)
            if i is None or i >= len(msg.position):
                continue
            out.name.append(joint)
            out.position.append(math.degrees(msg.position[i]))
            if i < len(msg.velocity):
                out.velocity.append(math.degrees(msg.velocity[i]))
            if i < len(msg.effort):
                out.effort.append(msg.effort[i])
        if out.name:
            self.pub.publish(out)


def main():
    rclpy.init()
    node = AnkleDegrees()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
