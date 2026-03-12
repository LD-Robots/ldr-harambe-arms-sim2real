#!/usr/bin/env python3
"""Joint Calibration Tool — sweep-based offset calibration with controller
listing and 3D robot visualization.  Requires ROS 2."""

import sys

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMessageBox

from ethercat_tools.joint_calibration.calibration_window import CalibrationWindow


def main():
    app = QApplication(sys.argv)

    try:
        import rclpy
        from sensor_msgs.msg import JointState

        rclpy.init(args=sys.argv)
        ros_node = rclpy.create_node("joint_calibration")
    except Exception as e:
        QMessageBox.critical(
            None,
            "ROS 2 Error",
            f"Failed to initialize ROS 2:\n\n{e}\n\n"
            "This tool requires a running ROS 2 environment.\n"
            "Source your workspace and try again.",
        )
        sys.exit(1)

    window = CalibrationWindow(ros_node=ros_node)

    ros_node.create_subscription(
        JointState, "/joint_states", window.joint_state_callback, 10
    )

    # Spin ROS in the Qt event loop (non-blocking)
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    ros_timer.start(10)  # 100 Hz

    ros_node.get_logger().info("Joint Calibration tool started")

    window.show()
    ret = app.exec_()

    try:
        ros_node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass

    sys.exit(ret)


if __name__ == "__main__":
    main()
