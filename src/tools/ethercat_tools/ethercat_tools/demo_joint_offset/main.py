#!/usr/bin/env python3
"""Joint Zero Calibration Tool — demo GUI with optional ROS 2 integration."""

import sys

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication

from ethercat_tools.demo_joint_offset.offset_window import JointOffsetWindow


def main():
    app = QApplication(sys.argv)

    ros_node = None

    # Try to initialize ROS 2 and subscribe to /joint_states
    try:
        import rclpy
        from sensor_msgs.msg import JointState

        rclpy.init(args=sys.argv)
        ros_node = rclpy.create_node("joint_offset_calibrator")

        window = JointOffsetWindow(ros_node=ros_node)

        ros_node.create_subscription(
            JointState, "/joint_states", window.joint_state_callback, 10
        )

        # Spin ROS in the Qt event loop (non-blocking)
        ros_timer = QTimer()
        ros_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
        ros_timer.start(10)  # 100 Hz

        ros_node.get_logger().info("Joint Offset Calibrator started (ROS mode)")

    except Exception:
        # No ROS available — run in pure demo mode
        window = JointOffsetWindow(ros_node=None)

    window.show()
    ret = app.exec_()

    # Cleanup
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
