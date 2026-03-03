#!/usr/bin/env python3
"""Joint State Monitor GUI — dynamically discovers and displays all joint states.

Subscribes to /joint_states and auto-populates a table with every joint found.
Follows the Catppuccin Mocha design guide (docs/GUI_DESIGN_GUIDE.md).
Layout: Status Bar -> Main Data Table -> Diagnostic Tabs.
"""

import math
import os
import re
import sys
import time
from datetime import datetime

from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import QApplication, QTableWidgetItem

# ───────────────────────────────────────────────────────────
# CATPPUCCIN MOCHA PALETTE
# ───────────────────────────────────────────────────────────

C_BASE = '#1e1e2e'
C_SURFACE0 = '#313244'
C_SURFACE1 = '#45475a'
C_TEXT = '#cdd6f4'
C_SUBTEXT = '#6c7086'
C_GREEN = '#a6e3a1'
C_YELLOW = '#f9e2af'
C_RED = '#f38ba8'
C_BLUE = '#89b4fa'
C_PEACH = '#fab387'

STALE_TIMEOUT = 2.0  # seconds

# Column indices
COL_NAME = 0
COL_GROUP = 1
COL_POS = 2
COL_VEL = 3
COL_EFF = 4
COL_HEALTH = 5

# Demo joints for offline testing
DEMO_JOINTS = [
    'left_shoulder_pitch_joint_X6',
    'left_shoulder_roll_joint_X6',
    'left_shoulder_yaw_joint_X4',
    'left_elbow_pitch_joint_X6',
    'left_wrist_yaw_joint_X4',
    'left_wrist_roll_joint_X4',
    'left_thumb_proximal_yaw_joint',
    'left_index_proximal_joint',
    'left_middle_proximal_joint',
    'right_shoulder_pitch_joint_X6',
    'right_shoulder_roll_joint_X6',
    'right_elbow_pitch_joint_X6',
]

# Group classification patterns
_ARM_SUFFIX = re.compile(r'_X[46]$')
_HAND_KEYWORDS = {'thumb', 'index', 'middle', 'ring', 'pinky'}


def _classify_group(joint_name):
    """Classify a joint name into a display group."""
    lower = joint_name.lower()
    is_arm = bool(_ARM_SUFFIX.search(joint_name))

    if lower.startswith('left_'):
        if is_arm:
            return 'L Arm'
        for kw in _HAND_KEYWORDS:
            if kw in lower:
                return 'L Hand'
        return 'L Other'
    elif lower.startswith('right_'):
        if is_arm:
            return 'R Arm'
        for kw in _HAND_KEYWORDS:
            if kw in lower:
                return 'R Hand'
        return 'R Other'
    return 'Other'


GROUP_ORDER = ['L Arm', 'R Arm', 'L Hand', 'R Hand', 'L Other', 'R Other', 'Other']
GROUP_COLORS = {
    'L Arm': C_BLUE, 'R Arm': C_BLUE,
    'L Hand': C_PEACH, 'R Hand': C_PEACH,
    'L Other': C_SUBTEXT, 'R Other': C_SUBTEXT, 'Other': C_SUBTEXT,
}


# ───────────────────────────────────────────────────────────
# TABLE HELPER
# ───────────────────────────────────────────────────────────

def _set_cell(table, row, col, text, color=None):
    """Consistent table cell formatting per design guide."""
    item = table.item(row, col)
    if item is None:
        item = QTableWidgetItem()
        item.setFlags(item.flags() & ~Qt.ItemIsEditable)
        table.setItem(row, col, item)
    item.setText(str(text))
    if color:
        item.setForeground(QColor(color))
    item.setTextAlignment(Qt.AlignCenter)


# ───────────────────────────────────────────────────────────
# ROS 2 NODE
# ───────────────────────────────────────────────────────────

class JointStateMonitorNode:
    """Thin wrapper that creates a ROS 2 subscription on an existing node."""

    def __init__(self, ros_node, gui_callback):
        from sensor_msgs.msg import JointState

        self._node = ros_node
        self._gui_callback = gui_callback
        self._data = {
            'joint_states': {},
            'last_update': None,
            'msg_count': 0,
        }

        ros_node.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10)

    def _joint_state_cb(self, msg):
        joints = {}
        for i, name in enumerate(msg.name):
            joints[name] = {
                'position': msg.position[i] if i < len(msg.position) else 0.0,
                'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                'effort': msg.effort[i] if i < len(msg.effort) else 0.0,
            }
        self._data['joint_states'] = joints
        self._data['last_update'] = time.time()
        self._data['msg_count'] += 1
        self._gui_callback(self._data)


# ───────────────────────────────────────────────────────────
# MAIN WINDOW
# ───────────────────────────────────────────────────────────

class JointStateMonitorWindow(QtWidgets.QMainWindow):

    def __init__(self, ros_node=None):
        super().__init__()
        self._load_ui()

        self._ros_node = ros_node
        self._start_time = time.monotonic()
        self._demo_time = 0.0

        # Joint data
        self._joint_data = {}       # {name: {position, velocity, effort}}
        self._joint_order = []      # Sorted joint names
        self._discovered = False

        # Rate estimation
        self._rate_window_start = time.monotonic()
        self._rate_window_count = 0
        self._current_rate = 0.0
        self._last_msg_count = 0

        # Last update time for stale detection
        self._last_update = None

        # Setup table
        table = self.table_joints
        table.horizontalHeader().setStretchLastSection(True)
        table.verticalHeader().setVisible(False)

        # Demo mode
        if ros_node is None:
            self._setup_demo()

        # 10 Hz UI tick
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(100)

    def _load_ui(self):
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('arm_gui_tools')
            ui_file = os.path.join(pkg_share, 'ui', 'joint_state_monitor.ui')
        except Exception:
            ui_file = os.path.join(
                os.path.dirname(__file__), '../../ui/joint_state_monitor.ui')
        uic.loadUi(ui_file, self)

    def _setup_demo(self):
        """Populate with demo joints for offline testing."""
        for name in DEMO_JOINTS:
            self._joint_data[name] = {
                'position': 0.0, 'velocity': 0.0, 'effort': 0.0}
        self._discover_joints(DEMO_JOINTS)

        self.label_state_indicator.setText('DEMO')
        self.label_state_indicator.setStyleSheet(
            f'color: {C_YELLOW}; font-size: 14px; font-weight: bold;')

    # ── Dynamic joint discovery ─────────────────────────────

    def _discover_joints(self, names):
        """Build table rows sorted by group then name."""
        grouped = {}
        for name in names:
            group = _classify_group(name)
            grouped.setdefault(group, []).append(name)

        ordered = []
        for g in GROUP_ORDER:
            if g in grouped:
                ordered.extend(sorted(grouped[g]))

        self._joint_order = ordered
        self._build_table()
        self._discovered = True

    def _build_table(self):
        table = self.table_joints
        table.setRowCount(len(self._joint_order))

        for row, name in enumerate(self._joint_order):
            group = _classify_group(name)
            _set_cell(table, row, COL_NAME, name, C_TEXT)
            _set_cell(table, row, COL_GROUP, group,
                      GROUP_COLORS.get(group, C_SUBTEXT))
            for col in (COL_POS, COL_VEL, COL_EFF):
                _set_cell(table, row, col, '---', C_SUBTEXT)
            _set_cell(table, row, COL_HEALTH, '---', C_SUBTEXT)

        table.resizeColumnsToContents()
        table.setColumnWidth(COL_NAME, 280)
        self.label_sb_joints_value.setText(str(len(self._joint_order)))

    # ── Data callback from ROS node ─────────────────────────

    def update_data(self, data):
        """Called from ROS callback — store data for next tick."""
        self._joint_data = data.get('joint_states', {})
        self._last_update = data.get('last_update')
        self._last_msg_count = data.get('msg_count', 0)

        if not self._discovered and self._joint_data:
            self._discover_joints(list(self._joint_data.keys()))

    # ── 10 Hz tick ──────────────────────────────────────────

    def _tick(self):
        self._demo_time += 0.1
        self._update_status_bar()
        self._update_uptime()
        self._update_rate()

        if self._discovered:
            self._update_joint_table()
            self._update_live_values()
            self._update_raw_data()

    def _update_status_bar(self):
        if self._ros_node is None:
            return  # Demo mode — already set

        if self._last_update is None:
            self.label_state_indicator.setText('WAITING')
            self.label_state_indicator.setStyleSheet(
                f'color: {C_YELLOW}; font-size: 14px; font-weight: bold;')
        elif (time.time() - self._last_update) < STALE_TIMEOUT:
            self.label_state_indicator.setText('CONNECTED')
            self.label_state_indicator.setStyleSheet(
                f'color: {C_GREEN}; font-size: 14px; font-weight: bold;')
        else:
            self.label_state_indicator.setText('STALE')
            self.label_state_indicator.setStyleSheet(
                f'color: {C_RED}; font-size: 14px; font-weight: bold;')

    def _update_uptime(self):
        elapsed = time.monotonic() - self._start_time
        h = int(elapsed // 3600)
        m = int((elapsed % 3600) // 60)
        s = int(elapsed % 60)
        self.label_sb_uptime_value.setText(f'{h:02d}:{m:02d}:{s:02d}')

    def _update_rate(self):
        now = time.monotonic()
        dt = now - self._rate_window_start
        if dt >= 1.0:
            delta = self._last_msg_count - self._rate_window_count
            self._current_rate = delta / dt
            self._rate_window_count = self._last_msg_count
            self._rate_window_start = now
            self.label_sb_rate_value.setText(f'{self._current_rate:.0f} Hz')

    # ── Joint table ─────────────────────────────────────────

    def _update_joint_table(self):
        table = self.table_joints
        is_demo = self._ros_node is None

        for row, name in enumerate(self._joint_order):
            if is_demo:
                phase = row * 0.8
                pos = math.radians(
                    30.0 * math.sin(self._demo_time * 0.5 + phase))
                vel = math.radians(
                    15.0 * math.cos(self._demo_time * 0.5 + phase))
                effort = 2.0 * math.sin(self._demo_time * 0.3 + phase)
            elif name in self._joint_data:
                jd = self._joint_data[name]
                pos = jd['position']
                vel = jd['velocity']
                effort = jd['effort']
            else:
                continue

            pos_deg = math.degrees(pos)
            vel_deg = math.degrees(vel)

            _set_cell(table, row, COL_POS, f'{pos_deg:+8.2f}', C_TEXT)
            _set_cell(table, row, COL_VEL, f'{vel_deg:+8.2f}', C_TEXT)
            _set_cell(table, row, COL_EFF, f'{effort:+8.3f}', C_TEXT)

            # Health based on effort magnitude
            abs_eff = abs(effort)
            if abs_eff >= 10.0:
                _set_cell(table, row, COL_HEALTH, 'HIGH', C_RED)
            elif abs_eff >= 5.0:
                _set_cell(table, row, COL_HEALTH, 'WARN', C_YELLOW)
            else:
                _set_cell(table, row, COL_HEALTH, 'OK', C_GREEN)

    # ── Live values tab ─────────────────────────────────────

    def _update_live_values(self):
        selected = self.table_joints.currentRow()
        if selected < 0 or selected >= len(self._joint_order):
            return

        name = self._joint_order[selected]
        is_demo = self._ros_node is None

        if is_demo:
            phase = selected * 0.8
            pos = math.radians(
                30.0 * math.sin(self._demo_time * 0.5 + phase))
            vel = math.radians(
                15.0 * math.cos(self._demo_time * 0.5 + phase))
            eff = 2.0 * math.sin(self._demo_time * 0.3 + phase)
        elif name in self._joint_data:
            jd = self._joint_data[name]
            pos = jd['position']
            vel = jd['velocity']
            eff = jd['effort']
        else:
            return

        now_str = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        line = (
            f'[{now_str}] {name}  '
            f'pos={math.degrees(pos):+8.2f}deg  '
            f'vel={math.degrees(vel):+8.2f}deg/s  '
            f'eff={eff:+8.3f}'
        )
        self.text_live_values.append(line)

        # Trim old lines
        doc = self.text_live_values.document()
        if doc.blockCount() > 200:
            cursor = self.text_live_values.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.movePosition(cursor.Down, cursor.KeepAnchor,
                                doc.blockCount() - 150)
            cursor.removeSelectedText()

    # ── Raw data tab ────────────────────────────────────────

    def _update_raw_data(self):
        lines = []
        now_str = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        lines.append(f'--- Raw Joint States @ {now_str} ---')
        lines.append('')
        lines.append(f'{"Joint":<42s}  {"pos (rad)":>12s}  '
                     f'{"vel (rad/s)":>12s}  {"effort":>10s}')
        lines.append('-' * 82)

        is_demo = self._ros_node is None

        for i, name in enumerate(self._joint_order):
            if is_demo:
                phase = i * 0.8
                pos = math.radians(
                    30.0 * math.sin(self._demo_time * 0.5 + phase))
                vel = math.radians(
                    15.0 * math.cos(self._demo_time * 0.5 + phase))
                eff = 2.0 * math.sin(self._demo_time * 0.3 + phase)
            elif name in self._joint_data:
                jd = self._joint_data[name]
                pos = jd['position']
                vel = jd['velocity']
                eff = jd['effort']
            else:
                pos = vel = eff = 0.0

            lines.append(
                f'{name:<42s}  {pos:+12.6f}  {vel:+12.6f}  {eff:+10.4f}')

        self.text_raw_data.setPlainText('\n'.join(lines))


# ───────────────────────────────────────────────────────────
# ENTRY POINT
# ───────────────────────────────────────────────────────────

def main(args=None):
    app = QApplication(sys.argv)
    ros_node = None

    try:
        import rclpy
        rclpy.init(args=args)
        ros_node = rclpy.create_node('joint_state_monitor_gui')
    except Exception:
        pass  # Fall back to demo mode

    window = JointStateMonitorWindow(ros_node=ros_node)

    if ros_node is not None:
        monitor = JointStateMonitorNode(ros_node, window.update_data)  # noqa: F841

        ros_timer = QTimer()
        ros_timer.timeout.connect(
            lambda: rclpy.spin_once(ros_node, timeout_sec=0))
        ros_timer.start(10)  # 100 Hz spin

    window.show()

    try:
        exit_code = app.exec_()
    finally:
        if ros_node is not None:
            ros_node.destroy_node()
            rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
