#!/usr/bin/env python3
"""Actuator Status GUI — lists every EtherCAT actuator with real-time status.

Displays all 24 actuators grouped by body part, shows CiA 402 drive state,
position, and logs state transitions as they happen.

Follows the Catppuccin Mocha design guide (docs/GUI_DESIGN_GUIDE.md).
Layout: Status Bar -> Main Actuator Table -> Diagnostic Tabs.

Supports DEMO mode (no ROS) with simulated state transitions.
"""

import math
import os
import sys
import time
from collections import deque
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

STALE_TIMEOUT = 2.0


# ───────────────────────────────────────────────────────────
# ACTUATOR CONFIGURATION (from ros2_control_real.xacro)
# ───────────────────────────────────────────────────────────

ACTUATOR_CONFIG = [
    # Left Arm (bus 0-5)
    {'name': 'left_shoulder_pitch_joint_X6', 'display': 'L Shoulder Pitch',
     'group': 'L Arm', 'motor': 'X6', 'bus_pos': 0},
    {'name': 'left_shoulder_roll_joint_X6', 'display': 'L Shoulder Roll',
     'group': 'L Arm', 'motor': 'X6', 'bus_pos': 1},
    {'name': 'left_shoulder_yaw_joint_X4', 'display': 'L Shoulder Yaw',
     'group': 'L Arm', 'motor': 'X4', 'bus_pos': 2},
    {'name': 'left_elbow_pitch_joint_X6', 'display': 'L Elbow Pitch',
     'group': 'L Arm', 'motor': 'X6', 'bus_pos': 3},
    {'name': 'left_wrist_yaw_joint_X4', 'display': 'L Wrist Yaw',
     'group': 'L Arm', 'motor': 'X4', 'bus_pos': 4},
    {'name': 'left_wrist_roll_joint_X4', 'display': 'L Wrist Roll',
     'group': 'L Arm', 'motor': 'X4', 'bus_pos': 5},
    # Right Arm (bus 6-11)
    {'name': 'right_shoulder_pitch_joint_X6', 'display': 'R Shoulder Pitch',
     'group': 'R Arm', 'motor': 'X6', 'bus_pos': 6},
    {'name': 'right_shoulder_roll_joint_X6', 'display': 'R Shoulder Roll',
     'group': 'R Arm', 'motor': 'X6', 'bus_pos': 7},
    {'name': 'right_shoulder_yaw_joint_X4', 'display': 'R Shoulder Yaw',
     'group': 'R Arm', 'motor': 'X4', 'bus_pos': 8},
    {'name': 'right_elbow_pitch_joint_X6', 'display': 'R Elbow Pitch',
     'group': 'R Arm', 'motor': 'X6', 'bus_pos': 9},
    {'name': 'right_wrist_yaw_joint_X4', 'display': 'R Wrist Yaw',
     'group': 'R Arm', 'motor': 'X4', 'bus_pos': 10},
    {'name': 'right_wrist_roll_joint_X4', 'display': 'R Wrist Roll',
     'group': 'R Arm', 'motor': 'X4', 'bus_pos': 11},
    # Waist (bus 12)
    {'name': 'waist_yaw_joint', 'display': 'Waist Yaw',
     'group': 'Waist', 'motor': 'X8', 'bus_pos': 12},
    # Left Leg (bus 13-18)
    {'name': 'left_hip_pitch_joint', 'display': 'L Hip Pitch',
     'group': 'L Leg', 'motor': 'X8', 'bus_pos': 13},
    {'name': 'left_hip_roll_joint', 'display': 'L Hip Roll',
     'group': 'L Leg', 'motor': 'X8', 'bus_pos': 14},
    {'name': 'left_hip_yaw_joint', 'display': 'L Hip Yaw',
     'group': 'L Leg', 'motor': 'X8', 'bus_pos': 15},
    {'name': 'left_knee_joint', 'display': 'L Knee',
     'group': 'L Leg', 'motor': 'X8', 'bus_pos': 16},
    {'name': 'left_ankle_pitch_joint', 'display': 'L Ankle Pitch',
     'group': 'L Leg', 'motor': 'X4', 'bus_pos': 17},
    {'name': 'left_ankle_roll_joint', 'display': 'L Ankle Roll',
     'group': 'L Leg', 'motor': 'X4', 'bus_pos': 18},
    # Right Leg (bus 19-23)
    {'name': 'right_hip_pitch_joint', 'display': 'R Hip Pitch',
     'group': 'R Leg', 'motor': 'X8', 'bus_pos': 19},
    {'name': 'right_hip_roll_joint', 'display': 'R Hip Roll',
     'group': 'R Leg', 'motor': 'X8', 'bus_pos': 20},
    {'name': 'right_hip_yaw_joint', 'display': 'R Hip Yaw',
     'group': 'R Leg', 'motor': 'X8', 'bus_pos': 21},
    {'name': 'right_knee_joint', 'display': 'R Knee',
     'group': 'R Leg', 'motor': 'X8', 'bus_pos': 22},
    {'name': 'right_ankle_roll_joint', 'display': 'R Ankle Roll',
     'group': 'R Leg', 'motor': 'X4', 'bus_pos': 23},
]

GROUP_COLORS = {
    'L Arm': C_BLUE, 'R Arm': C_BLUE,
    'Waist': C_PEACH,
    'L Leg': C_GREEN, 'R Leg': C_GREEN,
}

CIA402_STATES = {
    'NOT_READY_TO_SWITCH_ON': ('Not Ready', C_SUBTEXT),
    'SWITCH_ON_DISABLED':     ('Disabled', C_PEACH),
    'READY_TO_SWITCH_ON':     ('Ready', C_YELLOW),
    'SWITCHED_ON':            ('Switched On', C_YELLOW),
    'OPERATION_ENABLED':      ('Enabled', C_GREEN),
    'QUICK_STOP_ACTIVE':      ('Quick Stop', C_PEACH),
    'FAULT_REACTION_ACTIVE':  ('Fault React', C_RED),
    'FAULT':                  ('FAULT', C_RED),
}

CIA402_ORDER = [
    'NOT_READY_TO_SWITCH_ON', 'SWITCH_ON_DISABLED',
    'READY_TO_SWITCH_ON', 'SWITCHED_ON', 'OPERATION_ENABLED',
]

# Demo mode: simulated transition sequence per actuator
_DEMO_SEQUENCE = [
    'NOT_READY_TO_SWITCH_ON', 'SWITCH_ON_DISABLED',
    'READY_TO_SWITCH_ON', 'SWITCHED_ON', 'OPERATION_ENABLED',
]


# ───────────────────────────────────────────────────────────
# HELPERS
# ───────────────────────────────────────────────────────────

def _decode_cia402_state(status_word):
    sw = int(status_word)
    if (sw & 0x4F) == 0x00:
        return 'NOT_READY_TO_SWITCH_ON'
    if (sw & 0x4F) == 0x40:
        return 'SWITCH_ON_DISABLED'
    if (sw & 0x6F) == 0x21:
        return 'READY_TO_SWITCH_ON'
    if (sw & 0x6F) == 0x23:
        return 'SWITCHED_ON'
    if (sw & 0x6F) == 0x27:
        return 'OPERATION_ENABLED'
    if (sw & 0x6F) == 0x07:
        return 'QUICK_STOP_ACTIVE'
    if (sw & 0x4F) == 0x0F:
        return 'FAULT_REACTION_ACTIVE'
    if (sw & 0x4F) == 0x08:
        return 'FAULT'
    return 'N/A'


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


# ───────────────────────────────────────────────────────────
# MAIN WINDOW
# ───────────────────────────────────────────────────────────

class ActuatorStatusWindow(QtWidgets.QMainWindow):

    def __init__(self, ros_node=None):
        super().__init__()
        self._ros_node = ros_node
        self._load_ui()

        self._start_time = time.monotonic()
        self._event_log = deque(maxlen=500)

        # Drive state tracking
        self._prev_drive_states = {}
        self._drive_states = {}

        # ROS data
        self._joint_data = {}
        self._last_update = None
        self._msg_count = 0

        # Rate estimation
        self._rate_window_start = time.monotonic()
        self._rate_window_count = 0
        self._current_rate = 0.0

        # Demo mode state
        self._demo_time = 0.0
        self._demo_actuator_idx = 0
        self._demo_state_idx = 0

        self._setup_table()

        if ros_node is None:
            self._set_state_indicator('DEMO', C_YELLOW)
            self._set_label(self.label_sb_mode_value, 'DEMO', C_YELLOW)
        else:
            self._set_label(self.label_sb_mode_value, 'ROS', C_GREEN)

        # 10 Hz UI tick
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(100)

    def _load_ui(self):
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('arm_gui_tools')
            ui_file = os.path.join(pkg_share, 'ui', 'actuator_status.ui')
        except Exception:
            ui_file = os.path.join(
                os.path.dirname(__file__), '../../ui/actuator_status.ui')
        uic.loadUi(ui_file, self)

    def _set_state_indicator(self, text, color):
        self.label_state_indicator.setText(text)
        self.label_state_indicator.setStyleSheet(
            f'color: {color}; font-size: 14px; font-weight: bold;')

    def _set_label(self, label, text, color):
        label.setText(text)
        label.setStyleSheet(
            f'color: {color}; font-size: 13px; font-weight: bold;')

    # ── Table Setup ──────────────────────────────────────────

    def _setup_table(self):
        table = self.table_actuators
        table.setRowCount(len(ACTUATOR_CONFIG))
        table.horizontalHeader().setStretchLastSection(True)
        table.verticalHeader().setVisible(False)

        for row, acfg in enumerate(ACTUATOR_CONFIG):
            group_color = GROUP_COLORS.get(acfg['group'], C_SUBTEXT)
            _set_cell(table, row, 0, acfg['group'], group_color)
            _set_cell(table, row, 1, acfg['display'], C_TEXT)
            _set_cell(table, row, 2, acfg['motor'], C_BLUE)
            _set_cell(table, row, 3, str(acfg['bus_pos']), C_BLUE)
            _set_cell(table, row, 4, '--', C_SUBTEXT)
            _set_cell(table, row, 5, '--', C_SUBTEXT)
            _set_cell(table, row, 6, '--', C_SUBTEXT)

        table.resizeColumnsToContents()
        table.setColumnWidth(1, 180)

    # ── ROS Callbacks ────────────────────────────────────────

    def joint_state_callback(self, msg):
        joints = {}
        for i, name in enumerate(msg.name):
            joints[name] = {
                'position': msg.position[i] if i < len(msg.position) else 0.0,
                'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                'effort': msg.effort[i] if i < len(msg.effort) else 0.0,
            }
        self._joint_data = joints
        self._last_update = time.time()
        self._msg_count += 1

    def dynamic_joint_state_callback(self, msg):
        drive_states = {}
        for i, jname in enumerate(msg.joint_names):
            if i < len(msg.interface_values):
                iv = msg.interface_values[i]
                for k, name in enumerate(iv.interface_names):
                    if name == 'status_word' and k < len(iv.values):
                        state = _decode_cia402_state(iv.values[k])
                        if state:
                            drive_states[jname] = state
                        break
        if drive_states:
            self._log_transitions(drive_states)
            self._drive_states = drive_states
            self._last_update = time.time()

    # ── State Transition Logging ─────────────────────────────

    def _log_transitions(self, new_states):
        display_map = {a['name']: a['display'] for a in ACTUATOR_CONFIG}
        now_str = datetime.now().strftime('%H:%M:%S.%f')[:-3]

        for jname, new_state in new_states.items():
            old_state = self._prev_drive_states.get(jname)
            if old_state == new_state:
                continue

            display = display_map.get(jname, jname)
            old_label = CIA402_STATES.get(old_state, ('--', C_SUBTEXT))[0]
            new_label = CIA402_STATES.get(new_state, (new_state, C_SUBTEXT))[0]

            if new_state in ('FAULT', 'FAULT_REACTION_ACTIVE'):
                color = C_RED
            elif (new_state in CIA402_ORDER and
                  old_state in CIA402_ORDER and
                  CIA402_ORDER.index(new_state) <
                  CIA402_ORDER.index(old_state)):
                color = C_YELLOW
            else:
                color = C_GREEN

            self._log_event(
                now_str,
                f'{display}: {old_label} \u2192 {new_label}',
                color)

        self._prev_drive_states = dict(new_states)

    def _log_event(self, timestamp, message, color=C_GREEN):
        line = f'[{timestamp}] {message}'
        self._event_log.append(line)
        self.text_state_log.append(
            f'<span style="color: {color};">{line}</span>')

    # ── 10 Hz Tick ───────────────────────────────────────────

    def _tick(self):
        if self._ros_node is None:
            self._tick_demo()
        self._update_status_bar()
        self._update_table()
        self._update_raw_data()

    def _tick_demo(self):
        """Simulate sequential actuator initialization (like real hardware)."""
        self._demo_time += 0.1

        # Every 0.8s, advance the current actuator's state
        step = int(self._demo_time / 0.8)
        total_steps = len(ACTUATOR_CONFIG) * len(_DEMO_SEQUENCE)

        if step < total_steps:
            act_idx = step // len(_DEMO_SEQUENCE)
            state_idx = step % len(_DEMO_SEQUENCE)
            # Set all previous actuators to final state
            for i in range(act_idx):
                name = ACTUATOR_CONFIG[i]['name']
                self._drive_states[name] = 'OPERATION_ENABLED'
            # Set current actuator to current state
            name = ACTUATOR_CONFIG[act_idx]['name']
            new_state = _DEMO_SEQUENCE[state_idx]
            old_state = self._drive_states.get(name)
            if old_state != new_state:
                self._drive_states[name] = new_state
                self._log_transitions({name: new_state})
        else:
            # All done — all operational
            for acfg in ACTUATOR_CONFIG:
                self._drive_states[acfg['name']] = 'OPERATION_ENABLED'

        # Simulate position data
        for i, acfg in enumerate(ACTUATOR_CONFIG):
            phase = i * 0.5
            self._joint_data[acfg['name']] = {
                'position': math.radians(
                    15.0 * math.sin(self._demo_time * 0.3 + phase)),
                'velocity': 0.0,
                'effort': 0.0,
            }

    def _update_status_bar(self):
        now = time.time()
        is_demo = self._ros_node is None

        # State indicator
        if not is_demo:
            if self._last_update is None:
                self._set_state_indicator('WAITING', C_YELLOW)
            elif (now - self._last_update) < STALE_TIMEOUT:
                n_enabled = sum(
                    1 for ds in self._drive_states.values()
                    if ds == 'OPERATION_ENABLED')
                has_fault = any(
                    ds in ('FAULT', 'FAULT_REACTION_ACTIVE')
                    for ds in self._drive_states.values())
                total = len(ACTUATOR_CONFIG)
                if has_fault:
                    self._set_state_indicator('FAULT', C_RED)
                elif n_enabled == total:
                    self._set_state_indicator('OPERATIONAL', C_GREEN)
                elif self._drive_states:
                    self._set_state_indicator(
                        f'INITIALIZING {n_enabled}/{total}', C_PEACH)
                else:
                    self._set_state_indicator('CONNECTED', C_BLUE)
            else:
                self._set_state_indicator('STALE', C_RED)

        # Actuators reporting
        n_reporting = sum(
            1 for a in ACTUATOR_CONFIG if a['name'] in self._joint_data)
        total = len(ACTUATOR_CONFIG)
        self.label_sb_actuators_value.setText(f'{n_reporting} / {total}')

        # Operational count
        n_enabled = sum(
            1 for ds in self._drive_states.values()
            if ds == 'OPERATION_ENABLED')
        color = C_GREEN if n_enabled == total else (
            C_YELLOW if n_enabled > 0 else C_SUBTEXT)
        self._set_label(self.label_sb_operational_value, str(n_enabled), color)

        # Rate
        now_m = time.monotonic()
        dt = now_m - self._rate_window_start
        if dt >= 2.0:
            delta = self._msg_count - self._rate_window_count
            self._current_rate = delta / dt
            self._rate_window_count = self._msg_count
            self._rate_window_start = now_m
        self.label_sb_rate_value.setText(
            f'{self._current_rate:.0f} Hz' if self._current_rate > 0
            else '-- Hz')

        # Uptime
        elapsed = now_m - self._start_time
        h = int(elapsed // 3600)
        m = int((elapsed % 3600) // 60)
        s = int(elapsed % 60)
        self.label_sb_uptime_value.setText(f'{h:02d}:{m:02d}:{s:02d}')

    def _update_table(self):
        table = self.table_actuators

        for row, acfg in enumerate(ACTUATOR_CONFIG):
            jname = acfg['name']

            # CiA 402 state
            ds = self._drive_states.get(jname)
            if ds:
                ds_text, ds_color = CIA402_STATES.get(ds, (ds, C_SUBTEXT))
                _set_cell(table, row, 4, ds_text, ds_color)
            else:
                _set_cell(table, row, 4, '--', C_SUBTEXT)

            # Position
            jd = self._joint_data.get(jname)
            if jd:
                _set_cell(table, row, 5, f'{jd["position"]:+8.3f}', C_TEXT)
            else:
                _set_cell(table, row, 5, '--', C_SUBTEXT)

            # Overall status
            if ds in ('FAULT', 'FAULT_REACTION_ACTIVE'):
                _set_cell(table, row, 6, 'FAULT', C_RED)
            elif ds == 'OPERATION_ENABLED':
                _set_cell(table, row, 6, 'OK', C_GREEN)
            elif ds in ('READY_TO_SWITCH_ON', 'SWITCHED_ON'):
                _set_cell(table, row, 6, 'INIT', C_YELLOW)
            elif ds == 'SWITCH_ON_DISABLED':
                _set_cell(table, row, 6, 'OFF', C_PEACH)
            elif jd:
                _set_cell(table, row, 6, 'DATA', C_BLUE)
            else:
                _set_cell(table, row, 6, '--', C_SUBTEXT)

    def _update_raw_data(self):
        lines = []
        now_str = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        lines.append(f'--- Actuator Status @ {now_str} ---')
        lines.append('')
        lines.append(
            f'{"Joint":<36s}  {"CiA 402":>16s}  '
            f'{"pos (rad)":>12s}  {"vel (rad/s)":>12s}  {"effort":>10s}')
        lines.append('-' * 92)

        for acfg in ACTUATOR_CONFIG:
            jname = acfg['name']
            ds = self._drive_states.get(jname, '--')
            ds_label = CIA402_STATES.get(ds, (ds, C_SUBTEXT))[0]
            jd = self._joint_data.get(jname)

            if jd:
                lines.append(
                    f'{acfg["display"]:<36s}  {ds_label:>16s}  '
                    f'{jd["position"]:+12.6f}  '
                    f'{jd["velocity"]:+12.6f}  '
                    f'{jd["effort"]:+10.4f}')
            else:
                lines.append(
                    f'{acfg["display"]:<36s}  {ds_label:>16s}  '
                    f'{"--":>12s}  {"--":>12s}  {"--":>10s}')

        self.text_raw_data.setPlainText('\n'.join(lines))


# ───────────────────────────────────────────────────────────
# ENTRY POINT
# ───────────────────────────────────────────────────────────

def main(args=None):
    app = QApplication(sys.argv)
    ros_node = None

    try:
        import rclpy
        from sensor_msgs.msg import JointState
        from control_msgs.msg import DynamicJointState

        rclpy.init(args=args)
        ros_node = rclpy.create_node('actuator_status_gui')
    except Exception:
        pass  # Demo mode

    window = ActuatorStatusWindow(ros_node=ros_node)

    if ros_node is not None:
        from sensor_msgs.msg import JointState
        from control_msgs.msg import DynamicJointState

        ros_node.create_subscription(
            JointState, '/joint_states',
            window.joint_state_callback, 10)
        ros_node.create_subscription(
            DynamicJointState, '/dynamic_joint_states',
            window.dynamic_joint_state_callback, 10)

        ros_timer = QTimer()
        ros_timer.timeout.connect(
            lambda: rclpy.spin_once(ros_node, timeout_sec=0))
        ros_timer.start(10)  # 100 Hz spin

        ros_node.get_logger().info('Actuator Status GUI started')

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
