import math
import os
import time
from datetime import datetime

from PyQt5 import uic
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import QMainWindow, QTableWidgetItem

# Known left-arm joints — used to enrich the table with motor type and slave #.
# Joints not in this dict still display position/velocity/effort.
KNOWN_JOINTS = {
    "left_shoulder_pitch_joint_X6": {"motor": "X6", "slave": 0},
    "left_shoulder_roll_joint_X6":  {"motor": "X6", "slave": 1},
    "left_shoulder_yaw_joint_X4":   {"motor": "X4", "slave": 2},
    "left_elbow_pitch_joint_X6":    {"motor": "X6", "slave": 3},
    "left_wrist_yaw_joint_X4":      {"motor": "X4", "slave": 4},
    "left_wrist_roll_joint_X4":     {"motor": "X4", "slave": 5},
}

# Demo joints for when no ROS is available
DEMO_JOINTS = [
    "left_shoulder_pitch_joint_X6",
    "left_shoulder_roll_joint_X6",
    "left_shoulder_yaw_joint_X4",
    "left_elbow_pitch_joint_X6",
    "left_wrist_yaw_joint_X4",
    "left_wrist_roll_joint_X4",
]

CIA402_STATES = {
    "NOT_READY_TO_SWITCH_ON":  "#f38ba8",  # Red
    "SWITCH_ON_DISABLED":      "#fab387",  # Peach
    "READY_TO_SWITCH_ON":      "#f9e2af",  # Yellow
    "SWITCHED_ON":             "#f9e2af",  # Yellow
    "OPERATION_ENABLED":       "#a6e3a1",  # Green
    "QUICK_STOP_ACTIVE":       "#f38ba8",  # Red
    "FAULT_REACTION_ACTIVE":   "#f38ba8",  # Red
    "FAULT":                   "#f38ba8",  # Red
}

# Column indices
COL_NAME = 0
COL_MOTOR = 1
COL_SLAVE = 2
COL_POS = 3
COL_VEL = 4
COL_EFF = 5
COL_HEALTH = 6
COL_CIA = 7


def _find_ui_file():
    local = os.path.join(os.path.dirname(__file__), "joint_state.ui")
    if os.path.exists(local):
        return local
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(
            get_package_share_directory("ethercat_tools"), "ui", "joint_state.ui"
        )
    except Exception:
        return local


class JointStateWindow(QMainWindow):
    """Dynamic joint state monitor for both Gazebo (sim) and EtherCAT (real)."""

    def __init__(self, ros_node=None):
        super().__init__()
        uic.loadUi(_find_ui_file(), self)

        self._ros_node = ros_node
        self._start_time = time.monotonic()
        self._demo_time = 0.0

        # Joint data
        self._joint_state_data = {}   # {name: {position, velocity, effort}}
        self._joint_order = []        # Preserves first-seen order
        self._discovered = False

        # Mode: DEMO, SIM, or REAL
        self._mode = "DEMO"
        self._mode_locked = False     # True once mode is confirmed with data
        self._ethercat_subscribed = False

        # EtherCAT-specific data (only populated in REAL mode)
        self._safety_ok = None        # None = unknown, True/False from topic
        self._diag_data = {}          # {joint_name: {state, error_code, ...}}

        # Rate measurement
        self._msg_count = 0
        self._rate_window_start = time.monotonic()
        self._current_rate = 0.0

        # Setup table header
        table = self.jointTable
        table.horizontalHeader().setStretchLastSection(True)
        table.verticalHeader().setVisible(False)

        # 10 Hz UI update timer
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(100)

        if ros_node is None:
            self._mode = "DEMO"
            self._mode_locked = True
            self._setup_demo_joints()
        else:
            # ROS available but no data yet — show waiting state
            self._mode = "SIM"  # Assume SIM until proven otherwise
            self.statusIndicator.setText("WAITING")
            self.statusIndicator.setStyleSheet(
                "color: #f9e2af; font-size: 14px; font-weight: bold;"
            )
            self.jointsValue.setText("Waiting for /joint_states...")

        self._update_mode_display()

    # -- Mode detection ------------------------------------------------------

    def _detect_mode(self):
        """Check topic list to determine SIM vs REAL. Called periodically."""
        if self._ros_node is None or self._mode_locked:
            return

        topic_list = [t[0] for t in self._ros_node.get_topic_names_and_types()]
        ethercat_topics = {"/ethercat/raw_positions", "/safety/status"}

        if ethercat_topics & set(topic_list):
            self._mode = "REAL"
            if not self._ethercat_subscribed:
                self._subscribe_ethercat_topics()
            self._mode_locked = True
            self._update_mode_display()
        elif "/joint_states" in topic_list:
            # /joint_states exists but no EtherCAT → SIM
            self._mode = "SIM"
            # Don't lock yet — EtherCAT topics may appear later
            self._update_mode_display()

    def _subscribe_ethercat_topics(self):
        """Subscribe to EtherCAT-only topics for real hardware mode."""
        if self._ethercat_subscribed:
            return
        self._ethercat_subscribed = True

        try:
            from std_msgs.msg import Bool
            self._ros_node.create_subscription(
                Bool, "/safety/status", self.safety_callback, 10
            )
        except Exception:
            pass

        try:
            from diagnostic_msgs.msg import DiagnosticArray
            self._ros_node.create_subscription(
                DiagnosticArray, "/diagnostics", self.diagnostics_callback, 10
            )
        except Exception:
            pass

    def _update_mode_display(self):
        mode_colors = {
            "DEMO": "#f9e2af",  # Yellow
            "SIM":  "#89b4fa",  # Blue
            "REAL": "#a6e3a1",  # Green
        }
        color = mode_colors.get(self._mode, "#cdd6f4")
        self.modeValue.setText(self._mode)
        self.modeValue.setStyleSheet(
            f"font-size: 13px; font-weight: bold; color: {color};"
        )

    # -- Demo setup ----------------------------------------------------------

    def _setup_demo_joints(self):
        """Populate table with demo joints for offline testing."""
        self._joint_order = list(DEMO_JOINTS)
        self._build_table()
        self._discovered = True

    # -- Dynamic joint discovery ---------------------------------------------

    def _discover_joints(self, names):
        """Build table rows from the first /joint_states message."""
        self._joint_order = list(names)
        self._build_table()
        self._discovered = True

        # Update status indicator now that we have data
        self.statusIndicator.setText("JOINT MONITOR")
        self.statusIndicator.setStyleSheet(
            "color: #a6e3a1; font-size: 14px; font-weight: bold;"
        )

        # Lock SIM mode once we've received data and confirmed no EtherCAT
        if self._mode == "SIM" and not self._mode_locked:
            # Give it one more check
            self._detect_mode()
            if self._mode == "SIM":
                self._mode_locked = True

    def _build_table(self):
        """Create table rows for all discovered joints."""
        table = self.jointTable
        table.setRowCount(len(self._joint_order))

        for row, name in enumerate(self._joint_order):
            info = KNOWN_JOINTS.get(name, {})
            self._set_cell(table, row, COL_NAME, name)
            self._set_cell(table, row, COL_MOTOR,
                           info.get("motor", "---"), "#6c7086")
            slave = info.get("slave")
            self._set_cell(table, row, COL_SLAVE,
                           str(slave) if slave is not None else "---", "#6c7086")
            for col in (COL_POS, COL_VEL, COL_EFF, COL_HEALTH, COL_CIA):
                self._set_cell(table, row, col, "---")

        table.resizeColumnsToContents()
        table.setColumnWidth(COL_NAME, 260)
        self.jointsValue.setText(str(len(self._joint_order)))

    # -- Tick / update -------------------------------------------------------

    def _tick(self):
        self._demo_time += 0.1
        self._update_uptime()
        self._update_rate()

        # Keep trying to detect mode until locked
        if not self._mode_locked:
            self._detect_mode()

        if self._discovered:
            self._update_joints()
            self._update_live_values()
            self._update_ethercat_tab()

    def _update_uptime(self):
        elapsed = time.monotonic() - self._start_time
        h = int(elapsed // 3600)
        m = int((elapsed % 3600) // 60)
        s = int(elapsed % 60)
        self.uptimeValue.setText(f"{h:02d}:{m:02d}:{s:02d}")

    def _update_rate(self):
        now = time.monotonic()
        dt = now - self._rate_window_start
        if dt >= 1.0:
            self._current_rate = self._msg_count / dt
            self._msg_count = 0
            self._rate_window_start = now
            if self._discovered:
                self.rateValue.setText(f"{self._current_rate:.0f} Hz")

    def _update_joints(self):
        table = self.jointTable

        for row, name in enumerate(self._joint_order):
            if self._ros_node and name in self._joint_state_data:
                data = self._joint_state_data[name]
                pos_deg = math.degrees(data.get("position", 0.0))
                vel_deg = math.degrees(data.get("velocity", 0.0))
                effort = data.get("effort", 0.0)
            elif self._mode == "DEMO":
                phase = row * 0.8
                pos_deg = 30.0 * math.sin(self._demo_time * 0.5 + phase)
                vel_deg = 15.0 * math.cos(self._demo_time * 0.5 + phase)
                effort = 2.0 * math.sin(self._demo_time * 0.3 + phase)
            else:
                continue  # No data yet

            self._set_cell(table, row, COL_POS, f"{pos_deg:+8.2f}")
            self._set_cell(table, row, COL_VEL, f"{vel_deg:+8.2f}")
            self._set_cell(table, row, COL_EFF, f"{effort:+6.3f}")

            # Health
            if abs(effort) > 5.0:
                self._set_cell(table, row, COL_HEALTH, "WARN", "#f9e2af")
            else:
                self._set_cell(table, row, COL_HEALTH, "OK", "#a6e3a1")

            # EtherCAT state
            if self._mode == "REAL" and name in self._diag_data:
                state = self._diag_data[name].get("state", "---")
                color = CIA402_STATES.get(state, "#cdd6f4")
                display = state.replace("_", " ").title()
                self._set_cell(table, row, COL_CIA, display, color)
            elif self._mode == "DEMO" and name in KNOWN_JOINTS:
                self._set_cell(table, row, COL_CIA,
                               "Operation Enabled", "#a6e3a1")
            else:
                self._set_cell(table, row, COL_CIA, "---", "#6c7086")

    def _update_live_values(self):
        selected = self.jointTable.currentRow()
        if selected < 0 or selected >= len(self._joint_order):
            return

        name = self._joint_order[selected]

        if self._ros_node and name in self._joint_state_data:
            data = self._joint_state_data[name]
            pos = data.get("position", 0.0)
            vel = data.get("velocity", 0.0)
            eff = data.get("effort", 0.0)
        elif self._mode == "DEMO":
            phase = selected * 0.8
            pos = math.radians(30.0 * math.sin(self._demo_time * 0.5 + phase))
            vel = math.radians(15.0 * math.cos(self._demo_time * 0.5 + phase))
            eff = 2.0 * math.sin(self._demo_time * 0.3 + phase)
        else:
            return

        line = (
            f"[{name}]  "
            f"pos={math.degrees(pos):+8.2f}deg  "
            f"vel={math.degrees(vel):+8.2f}deg/s  "
            f"eff={eff:+6.3f}"
        )
        self.liveValuesText.append(line)

        doc = self.liveValuesText.document()
        if doc.blockCount() > 200:
            cursor = self.liveValuesText.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.movePosition(cursor.Down, cursor.KeepAnchor,
                                doc.blockCount() - 150)
            cursor.removeSelectedText()

    def _update_ethercat_tab(self):
        if self._mode == "SIM":
            self.ethercatStatusText.setPlainText(
                "Not available (simulation mode)\n\n"
                "EtherCAT status is only available when running on real hardware.\n"
                "Launch with: ros2 launch arm_real_bringup arm_real.launch.py"
            )
            return

        if self._mode == "DEMO":
            lines = [
                "EtherCAT Master    OPERATIONAL",
                f"Safety             {'OK' if self._safety_ok is not False else 'FAULT'}",
                "E-Stop             CLEAR",
                "Watchdog           OK",
                "",
                "--- Demo mode: simulated values ---",
            ]
            self.ethercatStatusText.setPlainText("\n".join(lines))
            return

        # REAL mode
        safety_str = "---"
        if self._safety_ok is True:
            safety_str = "OK"
        elif self._safety_ok is False:
            safety_str = "FAULT"

        lines = [
            f"Safety Status      {safety_str}",
            "",
            "Per-joint diagnostics:",
        ]

        for name in self._joint_order:
            if name in self._diag_data:
                d = self._diag_data[name]
                state = d.get("state", "---")
                err = d.get("error_code", "0x0000")
                lines.append(f"  {name:<40s}  {state:<25s}  err={err}")
            elif name in KNOWN_JOINTS:
                lines.append(f"  {name:<40s}  ---")

        self.ethercatStatusText.setPlainText("\n".join(lines))

    # -- ROS callbacks -------------------------------------------------------

    def joint_state_callback(self, msg):
        self._msg_count += 1

        for i, name in enumerate(msg.name):
            self._joint_state_data[name] = {
                "position": msg.position[i] if i < len(msg.position) else 0.0,
                "velocity": msg.velocity[i] if i < len(msg.velocity) else 0.0,
                "effort": msg.effort[i] if i < len(msg.effort) else 0.0,
            }

        if not self._discovered:
            self._discover_joints(msg.name)

    def safety_callback(self, msg):
        self._safety_ok = msg.data

    def diagnostics_callback(self, msg):
        for status in msg.status:
            joint_name = None
            state = None
            error_code = "0x0000"

            for kv in status.values:
                if kv.key == "joint":
                    joint_name = kv.value
                elif kv.key == "drive_state":
                    state = kv.value
                elif kv.key == "error_code":
                    error_code = kv.value

            if joint_name:
                self._diag_data[joint_name] = {
                    "state": state,
                    "error_code": error_code,
                }

    # -- Helpers -------------------------------------------------------------

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
