import math
import os
import random
import time

from PyQt5 import uic
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import QMainWindow, QTableWidgetItem

# Actuator definitions matching the left arm
ACTUATORS = [
    {"joint": "left_shoulder_pitch_joint_X6", "motor": "X6", "slave": 0},
    {"joint": "left_shoulder_roll_joint_X6",  "motor": "X6", "slave": 1},
    {"joint": "left_shoulder_yaw_joint_X4",   "motor": "X4", "slave": 2},
    {"joint": "left_elbow_pitch_joint_X6",    "motor": "X6", "slave": 3},
    {"joint": "left_wrist_yaw_joint_X4",      "motor": "X4", "slave": 4},
    {"joint": "left_wrist_roll_joint_X4",     "motor": "X4", "slave": 5},
]

CIA402_STATES = {
    0x0000: ("Not Ready",        "#f38ba8"),  # Red
    0x0040: ("Switch On Disabled", "#fab387"),  # Peach
    0x0021: ("Ready to Switch On", "#f9e2af"),  # Yellow
    0x0023: ("Switched On",       "#f9e2af"),  # Yellow
    0x0027: ("Operation Enabled", "#a6e3a1"),  # Green
    0x0007: ("Quick Stop Active", "#f38ba8"),  # Red
    0x000F: ("Fault Reaction",    "#f38ba8"),  # Red
    0x0008: ("Fault",             "#f38ba8"),  # Red
}


def _find_ui_file():
    """Locate the .ui file, checking both local and installed paths."""
    # Local (development / symlink-install)
    local = os.path.join(os.path.dirname(__file__), "ethercat_status.ui")
    if os.path.exists(local):
        return local

    # Installed via colcon
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(
            get_package_share_directory("ethercat_tools"), "ui", "ethercat_status.ui"
        )
    except Exception:
        return local  # Fallback


class EtherCatStatusWindow(QMainWindow):

    def __init__(self, ros_node=None):
        super().__init__()
        uic.loadUi(_find_ui_file(), self)

        self._ros_node = ros_node
        self._start_time = time.monotonic()
        self._joint_state_data = {}  # Populated by ROS callback
        self._demo_time = 0.0
        self._cycle_count = 0

        self._setup_table()
        self._setup_safety_table()

        # 10 Hz update timer
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(100)

        if ros_node is None:
            self.modeValue.setText("DEMO")
            self.modeValue.setStyleSheet(
                "font-size: 13px; font-weight: bold; color: #f9e2af;"
            )
        else:
            self.modeValue.setText("ROS")
            self.modeValue.setStyleSheet(
                "font-size: 13px; font-weight: bold; color: #a6e3a1;"
            )

    # ── Table setup ──────────────────────────────────────────────

    def _setup_table(self):
        table = self.actuatorsTable
        table.horizontalHeader().setStretchLastSection(True)
        table.verticalHeader().setVisible(False)

        for row, act in enumerate(ACTUATORS):
            # Static columns
            self._set_cell(table, row, 0, act["joint"])
            self._set_cell(table, row, 1, act["motor"])
            self._set_cell(table, row, 2, str(act["slave"]))

        # Resize columns to content initially
        table.resizeColumnsToContents()
        # Give the joint name column more room
        table.setColumnWidth(0, 260)

    def _setup_safety_table(self):
        table = self.safetyTable
        table.horizontalHeader().setStretchLastSection(True)
        table.verticalHeader().setVisible(False)

        for row, act in enumerate(ACTUATORS):
            self._set_cell(table, row, 0, act["joint"].replace("left_", "").replace("_joint_X4", "").replace("_joint_X6", ""))
            self._set_cell(table, row, 1, "OK", "#a6e3a1")
            self._set_cell(table, row, 2, "OK", "#a6e3a1")
            self._set_cell(table, row, 3, "OK", "#a6e3a1")
            self._set_cell(table, row, 4, "NOMINAL", "#a6e3a1")

        table.resizeColumnsToContents()

    # ── Tick / update ────────────────────────────────────────────

    def _tick(self):
        self._demo_time += 0.1
        self._cycle_count += 100  # Simulated 1 kHz

        self._update_master_status()
        self._update_actuators()
        self._update_raw_pdo()

    def _update_master_status(self):
        elapsed = time.monotonic() - self._start_time
        h = int(elapsed // 3600)
        m = int((elapsed % 3600) // 60)
        s = int(elapsed % 60)
        self.uptimeValue.setText(f"{h:02d}:{m:02d}:{s:02d}")
        self.cycleValue.setText(f"1000 Hz")
        self.slavesValue.setText("6 / 6")

    def _update_actuators(self):
        table = self.actuatorsTable

        for row, act in enumerate(ACTUATORS):
            if self._ros_node and act["joint"] in self._joint_state_data:
                data = self._joint_state_data[act["joint"]]
                pos_deg = math.degrees(data.get("position", 0.0))
                vel_deg = math.degrees(data.get("velocity", 0.0))
                torque = data.get("effort", 0.0)
            else:
                # Demo: sinusoidal motion with per-joint phase offset
                phase = row * 0.8
                pos_deg = 30.0 * math.sin(self._demo_time * 0.5 + phase)
                vel_deg = 15.0 * math.cos(self._demo_time * 0.5 + phase)
                torque = 2.0 * math.sin(self._demo_time * 0.3 + phase) + random.gauss(0, 0.1)

            # CiA 402 state — always operational in demo
            status_word = 0x0027
            cia_state, cia_color = CIA402_STATES.get(
                status_word, ("Unknown", "#cdd6f4")
            )
            error_code = 0x0000

            self._set_cell(table, row, 3, cia_state, cia_color)
            self._set_cell(table, row, 4, f"{pos_deg:+8.2f}")
            self._set_cell(table, row, 5, f"{vel_deg:+8.2f}")
            self._set_cell(table, row, 6, f"{torque:+6.3f}")
            self._set_cell(table, row, 7, f"0x{status_word:04X}")
            self._set_cell(table, row, 8, f"0x{error_code:04X}")

            # Health indicator
            if error_code != 0:
                health_text, health_color = "FAULT", "#f38ba8"
            elif abs(torque) > 5.0:
                health_text, health_color = "WARN", "#f9e2af"
            else:
                health_text, health_color = "OK", "#a6e3a1"
            self._set_cell(table, row, 9, health_text, health_color)

    def _update_raw_pdo(self):
        lines = []
        for act in ACTUATORS:
            slave = act["slave"]
            # Simulated TxPDO (16 bytes): status_word, position, velocity, torque, error, mode, pad
            sw = 0x0027
            pos_raw = int(30.0 * math.sin(self._demo_time * 0.5 + slave * 0.8) / 180.0 * 65535)
            vel_raw = int(15.0 * math.cos(self._demo_time * 0.5 + slave * 0.8) / 180.0 * 65535)
            torque_raw = int(2.0 * math.sin(self._demo_time * 0.3 + slave * 0.8) * 100)
            err = 0x0000
            mode = 0x08  # CSP

            tx_bytes = (
                sw.to_bytes(2, "little")
                + pos_raw.to_bytes(4, "little", signed=True)
                + vel_raw.to_bytes(4, "little", signed=True)
                + torque_raw.to_bytes(2, "little", signed=True)
                + err.to_bytes(2, "little")
                + mode.to_bytes(1, "little")
                + b"\x00"
            )
            hex_str = " ".join(f"{b:02X}" for b in tx_bytes)
            lines.append(f"Slave {slave} TxPDO: {hex_str}")

            # Simulated RxPDO
            rx_bytes = (
                (0x000F).to_bytes(2, "little")
                + pos_raw.to_bytes(4, "little", signed=True)
                + vel_raw.to_bytes(4, "little", signed=True)
                + torque_raw.to_bytes(2, "little", signed=True)
                + (1000).to_bytes(2, "little")
                + mode.to_bytes(1, "little")
                + b"\x00"
            )
            hex_str = " ".join(f"{b:02X}" for b in rx_bytes)
            lines.append(f"Slave {slave} RxPDO: {hex_str}")
            lines.append("")

        self.rawPdoText.setPlainText("\n".join(lines))

    # ── ROS callback ─────────────────────────────────────────────

    def joint_state_callback(self, msg):
        """Called from the ROS subscriber with sensor_msgs/JointState."""
        for i, name in enumerate(msg.name):
            self._joint_state_data[name] = {
                "position": msg.position[i] if i < len(msg.position) else 0.0,
                "velocity": msg.velocity[i] if i < len(msg.velocity) else 0.0,
                "effort": msg.effort[i] if i < len(msg.effort) else 0.0,
            }

    # ── Helpers ──────────────────────────────────────────────────

    @staticmethod
    def _set_cell(table, row, col, text, color=None):
        item = table.item(row, col)
        if item is None:
            item = QTableWidgetItem()
            item.setFlags(item.flags() & ~Qt.ItemIsEditable)
            table.setItem(row, col, item)
        item.setText(str(text))
        if color:
            item.setForeground(QColor(color))
        item.setTextAlignment(Qt.AlignCenter)
