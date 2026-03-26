#!/usr/bin/env python3
"""Joint State Monitor — dynamic GUI for sim (Gazebo) and real (EtherCAT)."""

import sys

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication

from ethercat_tools.demo_joint_state.state_window import JointStateWindow


def main():
    app = QApplication(sys.argv)

    ros_node = None
    ros_timer = None

    try:
        import rclpy
        from sensor_msgs.msg import JointState

        rclpy.init(args=sys.argv)
        ros_node = rclpy.create_node("joint_state_monitor")

        window = JointStateWindow(ros_node=ros_node)

        # Always subscribe to /joint_states (works in both sim and real)
        ros_node.create_subscription(
            JointState, "/joint_states", window.joint_state_callback, 10
        )

        # Mode detection and EtherCAT subscription happen automatically
        # in the window's tick loop via _detect_mode()

        # Spin ROS in the Qt event loop (non-blocking)
        ros_timer = QTimer()
        ros_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
        ros_timer.start(10)  # 100 Hz

        ros_node.get_logger().info("Joint State Monitor started (ROS mode)")

    except Exception:
        window = JointStateWindow(ros_node=None)

    window.show()
    ret = app.exec_()

    if ros_node is not None:
        try:
            ros_node.destroy_node()
            import rclpy
            rclpy.shutdown()
        except Exception:
            pass

    sys.exit(ret)


if __name__ == "__main__":
    main()
