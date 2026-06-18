#!/usr/bin/env python3
"""Policy legs panel — one-click commands for ``harambe_policy_legs_controller``.

A small button panel that fires the bring-up commands an operator otherwise
types as ``ros2 topic pub`` one-liners:

  * ``Calibrate IMU``  → ``Bool(true)`` on ``/harambe_policy_legs_controller/calibrate_imu``
  * ``Enable`` / ``Disable`` → ``Bool(true|false)`` on ``/harambe_policy_legs_controller/enable``
  * ``Linear``  Start/Stop → ``Twist(linear.x = speed | 0)`` on ``/cmd_vel``
  * ``Angular`` Start/Stop → ``Twist(angular.z = speed | 0)`` on ``/cmd_vel``

The controller reads ``linear.x``, ``linear.y`` and ``angular.z`` from the Twist
and latches the last value in a realtime buffer, so a single publish per click is
enough — Start holds until Stop sends zero. Linear and angular are independent:
each Start sends a full Twist with only its own axis set, zeroing the other (same
behaviour as the separate ``ros2 topic pub -1`` commands). UI follows the
Catppuccin Mocha look of ``bus_voltage_viewer.py`` / ``pvt_dashboard.py``."""

import sys
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

# ───────────────────────────────────────────────────────────
# Topics — single source of truth. Change CONTROLLER to retarget.
# ───────────────────────────────────────────────────────────

CONTROLLER    = "/harambe_policy_legs_controller"
CALIB_TOPIC   = f"{CONTROLLER}/calibrate_imu"   # std_msgs/Bool
ENABLE_TOPIC  = f"{CONTROLLER}/enable"          # std_msgs/Bool
CMD_VEL_TOPIC = "/cmd_vel"                       # geometry_msgs/Twist

DEFAULT_SPEED = 0.3

# ───────────────────────────────────────────────────────────
# CATPPUCCIN MOCHA PALETTE — matches bus_voltage_viewer.py / pvt_dashboard.py
# ───────────────────────────────────────────────────────────

C_BASE     = "#1e1e2e"
C_SURFACE0 = "#313244"
C_SURFACE1 = "#45475a"
C_TEXT     = "#cdd6f4"
C_SUBTEXT  = "#6c7086"
C_GREEN    = "#a6e3a1"
C_YELLOW   = "#f9e2af"
C_RED      = "#f38ba8"
C_BLUE     = "#89b4fa"
C_PEACH    = "#fab387"
C_MAUVE    = "#cba6f7"
C_TEAL     = "#94e2d5"

GLOBAL_STYLESHEET = f"""
QMainWindow {{ background-color: {C_BASE}; color: {C_TEXT}; }}
QLabel {{ color: {C_TEXT}; }}
QGroupBox {{ background-color: {C_SURFACE0}; border: 1px solid {C_SURFACE1};
             border-radius: 6px; margin-top: 14px; padding: 10px;
             font-weight: bold; }}
QGroupBox::title {{ subcontrol-origin: margin; left: 10px; padding: 0 4px;
                    color: {C_PEACH}; }}
QFrame#StatusBar {{ background-color: {C_SURFACE0}; border-radius: 4px;
                    padding: 4px; }}
QDoubleSpinBox {{ background-color: {C_BASE}; color: {C_TEXT};
                  border: 1px solid {C_SURFACE1}; border-radius: 4px;
                  padding: 3px 6px; }}
QPushButton {{ background-color: {C_SURFACE0}; color: {C_TEXT};
               border: 1px solid {C_SURFACE1}; border-radius: 4px;
               padding: 8px 14px; font-weight: bold; }}
QPushButton:hover {{ background-color: {C_SURFACE1}; }}
QPushButton:pressed {{ background-color: #585b70; }}
QPushButton:disabled {{ color: {C_SUBTEXT}; }}
QPushButton#Enable {{ color: {C_GREEN}; }}
QPushButton#Disable {{ color: {C_RED}; }}
QPushButton#Start {{ color: {C_GREEN}; }}
QPushButton#Stop {{ color: {C_PEACH}; }}
QPushButton#Calib {{ color: {C_BLUE}; }}
"""


# ───────────────────────────────────────────────────────────
# Main window
# ───────────────────────────────────────────────────────────


class PolicyLegsPanel(QMainWindow):
    def __init__(self, node: Node):
        super().__init__()
        self.setWindowTitle("Policy Legs Panel")
        self.setMinimumWidth(420)
        self.setStyleSheet(GLOBAL_STYLESHEET)
        self._node = node

        # Persistent publishers (depth-10 default QoS is wire-compatible with the
        # controller's SystemDefaultsQoS subscriptions).
        self._calib_pub  = node.create_publisher(Bool, CALIB_TOPIC, 10)
        self._enable_pub = node.create_publisher(Bool, ENABLE_TOPIC, 10)
        self._cmd_pub    = node.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        self._build_title(root)
        self._build_policy_group(root)
        self._lin_spin = self._build_velocity_group(
            root, "Linear  (cmd_vel linear.x)", "m/s", self._on_linear_start)
        self._ang_spin = self._build_velocity_group(
            root, "Angular  (cmd_vel angular.z)", "rad/s", self._on_angular_start)
        self._build_status_bar(root)
        root.addStretch(1)

    # ─── UI BUILDERS ────────────────────────────────────────

    def _build_title(self, parent):
        title = QLabel("POLICY LEGS PANEL")
        title.setFont(QFont("monospace", 12, QFont.Bold))
        title.setStyleSheet(f"color: {C_MAUVE};")
        parent.addWidget(title)
        sub = QLabel(CONTROLLER)
        sub.setFont(QFont("monospace", 9))
        sub.setStyleSheet(f"color: {C_SUBTEXT};")
        parent.addWidget(sub)

    def _build_policy_group(self, parent):
        box = QGroupBox("Policy")
        row = QHBoxLayout(box)
        row.setSpacing(8)

        calib = QPushButton("Calibrate IMU")
        calib.setObjectName("Calib")
        calib.clicked.connect(self._on_calibrate)
        row.addWidget(calib)

        enable = QPushButton("Enable (ON)")
        enable.setObjectName("Enable")
        enable.clicked.connect(self._on_enable)
        row.addWidget(enable)

        disable = QPushButton("Disable (OFF)")
        disable.setObjectName("Disable")
        disable.clicked.connect(self._on_disable)
        row.addWidget(disable)

        parent.addWidget(box)

    def _build_velocity_group(self, parent, title, unit, start_slot) -> QDoubleSpinBox:
        box = QGroupBox(title)
        row = QHBoxLayout(box)
        row.setSpacing(8)

        spin = QDoubleSpinBox()
        spin.setRange(-2.0, 2.0)
        spin.setSingleStep(0.1)
        spin.setDecimals(2)
        spin.setValue(DEFAULT_SPEED)
        spin.setSuffix(f" {unit}")
        spin.setFixedWidth(120)
        row.addWidget(spin)
        row.addStretch(1)

        start = QPushButton("Start")
        start.setObjectName("Start")
        start.clicked.connect(start_slot)
        row.addWidget(start)

        stop = QPushButton("Stop")
        stop.setObjectName("Stop")
        stop.clicked.connect(self._on_stop)
        row.addWidget(stop)

        parent.addWidget(box)
        return spin

    def _build_status_bar(self, parent):
        bar = QFrame()
        bar.setObjectName("StatusBar")
        bar.setFixedHeight(34)
        h = QHBoxLayout(bar)
        h.setContentsMargins(10, 4, 10, 4)
        lab = QLabel("LAST:")
        lab.setFont(QFont("monospace", 9))
        lab.setStyleSheet(f"color: {C_SUBTEXT};")
        h.addWidget(lab)
        self._status = QLabel("(idle)")
        self._status.setFont(QFont("monospace", 9, QFont.Bold))
        self._status.setStyleSheet(f"color: {C_SUBTEXT};")
        h.addWidget(self._status)
        h.addStretch(1)
        parent.addWidget(bar)

    # ─── PUBLISH HELPERS ────────────────────────────────────

    def _emit(self, status, color, raw_cmd):
        """Update the GUI status line and echo the equivalent raw `ros2 topic
        pub` command to the launching terminal (copy-pasteable, flushed)."""
        self._status.setText(status)
        self._status.setStyleSheet(f"color: {color};")
        print(raw_cmd, flush=True)

    @staticmethod
    def _bool_cmd(topic, val):
        return (f'ros2 topic pub -1 {topic} std_msgs/msg/Bool '
                f'"{{data: {str(bool(val)).lower()}}}"')

    @staticmethod
    def _twist_cmd(lx, az):
        return (f'ros2 topic pub -1 {CMD_VEL_TOPIC} geometry_msgs/msg/Twist '
                f'"{{linear: {{x: {lx:.2f}}}, angular: {{z: {az:.2f}}}}}"')

    def _publish_twist(self, lx=0.0, az=0.0):
        """Publish a full Twist; unspecified fields stay 0 (axes independent)."""
        msg = Twist()
        msg.linear.x = float(lx)
        msg.angular.z = float(az)
        self._cmd_pub.publish(msg)

    # ─── BUTTON SLOTS ───────────────────────────────────────

    def _on_calibrate(self):
        self._calib_pub.publish(Bool(data=True))
        self._emit("calibrate_imu → true", C_BLUE,
                   self._bool_cmd(CALIB_TOPIC, True))

    def _on_enable(self):
        self._enable_pub.publish(Bool(data=True))
        self._emit("enable → true", C_GREEN,
                   self._bool_cmd(ENABLE_TOPIC, True))

    def _on_disable(self):
        self._enable_pub.publish(Bool(data=False))
        self._emit("enable → false", C_RED,
                   self._bool_cmd(ENABLE_TOPIC, False))

    def _on_linear_start(self):
        v = self._lin_spin.value()
        self._publish_twist(lx=v)
        self._emit(f"cmd_vel linear.x = {v:.2f}", C_GREEN,
                   self._twist_cmd(v, 0.0))

    def _on_angular_start(self):
        v = self._ang_spin.value()
        self._publish_twist(az=v)
        self._emit(f"cmd_vel angular.z = {v:.2f}", C_GREEN,
                   self._twist_cmd(0.0, v))

    def _on_stop(self):
        self._publish_twist()
        self._emit("cmd_vel = 0 (stop)", C_PEACH,
                   self._twist_cmd(0.0, 0.0))


# ───────────────────────────────────────────────────────────
# main
# ───────────────────────────────────────────────────────────


def main():
    rclpy.init(args=sys.argv)
    node = Node("policy_legs_panel_gui")

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    app = QApplication(sys.argv)
    win = PolicyLegsPanel(node)
    win.show()
    exit_code = app.exec_()

    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
