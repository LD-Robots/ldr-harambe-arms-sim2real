#!/usr/bin/env python3
"""Bridge: subscribes to /policy/debug (Float32Array from gz bridge) and
republishes as /policy/state (harambe_msgs/PolicyDebug) with named fields."""

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Float32Array
from harambe_msgs.msg import PolicyDebug


class PolicyDebugBridge(Node):
    def __init__(self):
        super().__init__('policy_debug_bridge')
        self.sub_ = self.create_subscription(
            Float32Array, '/policy/debug', self.on_msg, 10)
        self.pub_ = self.create_publisher(PolicyDebug, '/policy/state', 10)
        self.get_logger().info('PolicyDebugBridge ready: /policy/debug -> /policy/state')

    def on_msg(self, msg):
        d = msg.data
        if len(d) < 218:
            return

        out = PolicyDebug()
        out.header.stamp = self.get_clock().now().to_msg()
        out.efforts = list(d[0:25])
        out.targets = list(d[25:50])
        out.positions = list(d[50:75])
        out.velocities = list(d[75:100])
        out.actions = list(d[100:125])
        out.observations = list(d[125:218])
        self.pub_.publish(out)


def main():
    rclpy.init()
    rclpy.spin(PolicyDebugBridge())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
