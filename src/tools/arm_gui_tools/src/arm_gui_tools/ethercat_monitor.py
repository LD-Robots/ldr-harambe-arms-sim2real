#!/usr/bin/env python3
"""EtherCAT monitoring GUI for MyActuator RMD X V4 dual-arm system.

Follows the Catppuccin Mocha design guide (docs/GUI_DESIGN_GUIDE.md).
Layout: Status Bar -> Main Data Table -> Diagnostic Tabs.

Requires ROS 2 to be running. Use with position_reader_node for read-only
hardware monitoring, or with the full arm_real bringup for live control.

Zoom: Ctrl+Plus / Ctrl+Minus to scale UI, Ctrl+0 to reset.
"""

import sys
import os
import time
from collections import deque
from datetime import datetime

from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QTableWidgetItem, QApplication
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32MultiArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_srvs.srv import Trigger


# ===========================================================
# CATPPUCCIN MOCHA PALETTE
# ===========================================================

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


# ===========================================================
# JOINT CONFIGURATION
# ===========================================================

JOINT_CONFIG = [
    {'name': 'left_shoulder_pitch_joint_X6', 'display': 'L Shoulder Pitch',
     'motor': 'X6', 'bus_pos': 0,
     'pos_min': -3.14, 'pos_max': 3.14, 'vel_max': 2.0,
     'torque_max_pct': 80, 'pos_margin': 0.1},
    {'name': 'left_shoulder_roll_joint_X6', 'display': 'L Shoulder Roll',
     'motor': 'X6', 'bus_pos': 1,
     'pos_min': -3.14, 'pos_max': 3.14, 'vel_max': 2.0,
     'torque_max_pct': 80, 'pos_margin': 0.1},
    {'name': 'left_shoulder_yaw_joint_X4', 'display': 'L Shoulder Yaw',
     'motor': 'X4', 'bus_pos': 2,
     'pos_min': -3.14, 'pos_max': 3.14, 'vel_max': 2.5,
     'torque_max_pct': 80, 'pos_margin': 0.1},
    {'name': 'left_elbow_pitch_joint_X6', 'display': 'L Elbow Pitch',
     'motor': 'X6', 'bus_pos': 3,
     'pos_min': -3.14, 'pos_max': 3.14, 'vel_max': 2.0,
     'torque_max_pct': 80, 'pos_margin': 0.1},
    {'name': 'left_wrist_yaw_joint_X4', 'display': 'L Wrist Yaw',
     'motor': 'X4', 'bus_pos': 4,
     'pos_min': -3.14, 'pos_max': 3.14, 'vel_max': 3.0,
     'torque_max_pct': 80, 'pos_margin': 0.1},
    {'name': 'left_wrist_roll_joint_X4', 'display': 'L Wrist Roll',
     'motor': 'X4', 'bus_pos': 5,
     'pos_min': -3.14, 'pos_max': 3.14, 'vel_max': 3.0,
     'torque_max_pct': 80, 'pos_margin': 0.1},
]

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

STALE_TIMEOUT = 2.0


# ===========================================================
# HELPERS
# ===========================================================

def _decode_cia402_state(status_word):
    """Decode CiA 402 drive state from status word (0x6041).

    Mirrors the C++ implementation in cia402_state_machine.cpp.
    """
    sw = status_word & 0xFFFF
    if (sw & 0x004F) == 0x0000:
        return 'NOT_READY_TO_SWITCH_ON'
    if (sw & 0x006F) == 0x0040:
        return 'SWITCH_ON_DISABLED'
    if (sw & 0x006F) == 0x0021:
        return 'READY_TO_SWITCH_ON'
    if (sw & 0x006F) == 0x0023:
        return 'SWITCHED_ON'
    if (sw & 0x006F) == 0x0027:
        return 'OPERATION_ENABLED'
    if (sw & 0x006F) == 0x0007:
        return 'QUICK_STOP_ACTIVE'
    if (sw & 0x004F) == 0x000F:
        return 'FAULT_REACTION_ACTIVE'
    if (sw & 0x004F) == 0x0008:
        return 'FAULT'
    return 'NOT_READY_TO_SWITCH_ON'


def _is_alive(data, topic):
    last = data.get('last_update', {}).get(topic)
    if last is None:
        return None
    return (time.time() - last) < STALE_TIMEOUT


def _status_color(ratio):
    if ratio >= 0.9:
        return C_RED
    elif ratio >= 0.7:
        return C_YELLOW
    return C_GREEN


def _status_text(ratio):
    if ratio >= 0.9:
        return 'LIMIT'
    elif ratio >= 0.7:
        return 'WARN'
    return 'OK'


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


# ===========================================================
# MAIN WINDOW
# ===========================================================

class EthercatMonitorWindow(QtWidgets.QMainWindow):

    def __init__(self, ros_node):
        super().__init__()
        self._ros_node = ros_node
        self._zoom_level = 0  # px offset from default font sizes
        self._load_ui()

        # Rate estimation
        self._rate_window_start = {}
        self._rate_window_count = {}
        self._last_rates = {}

        # Fault log
        self._event_log = deque(maxlen=500)
        self._known_faults = set()
        self._last_diag_id = None

        # ROS data accumulator
        self._ros_data = {
            'joint_states': {},
            'raw_positions': [],
            'status_words': [],
            'safety_ok': None,
            'diagnostics': None,
            'last_update': {},
            'msg_counts': {},
        }

        self._setup_joint_table()
        self._setup_safety_table()

        # Mode indicator
        self._set_label(self.label_sb_mode_value, 'ROS', C_GREEN)

        # Service clients
        self._cli_estop = ros_node.create_client(Trigger, '/safety/estop')
        self._cli_reset = ros_node.create_client(Trigger, '/safety/reset')

        self.button_estop.clicked.connect(self._on_estop)
        self.button_reset.clicked.connect(self._on_reset)
        self.button_clear_log.clicked.connect(self._on_clear_log)

    # ----------------------------------------------------------
    # Zoom
    # ----------------------------------------------------------

    def _sz(self, base):
        """Return font size scaled by current zoom level."""
        return max(base + self._zoom_level, 6)

    def _dim(self, base):
        """Return a dimension (padding/margin/spacing) scaled by zoom."""
        scale = 1.0 + self._zoom_level * 0.08
        return max(int(base * scale), 1)

    def keyPressEvent(self, event):
        if event.modifiers() & Qt.ControlModifier:
            if event.key() in (Qt.Key_Plus, Qt.Key_Equal):
                self._zoom_level += 1
                self._apply_zoom()
                return
            elif event.key() == Qt.Key_Minus:
                self._zoom_level = max(self._zoom_level - 1, -4)
                self._apply_zoom()
                return
            elif event.key() == Qt.Key_0:
                self._zoom_level = 0
                self._apply_zoom()
                return
        super().keyPressEvent(event)

    def _apply_zoom(self):
        """Rebuild global stylesheet with scaled sizes and refresh."""
        # Scaled dimensions
        pad_sm = self._dim(4)
        pad_md = self._dim(6)
        pad_lg = self._dim(8)
        pad_xl = self._dim(16)
        margin = self._dim(12)
        spacing = self._dim(8)
        sb_height = self._dim(65)
        sb_max = self._dim(70)
        btn_h = self._dim(36)
        btn_w_estop = self._dim(100)
        btn_w_reset = self._dim(70)
        radius = self._dim(8)
        radius_sm = self._dim(4)
        radius_grp = self._dim(6)
        grp_margin_top = self._dim(8)
        grp_pad_top = self._dim(16)
        tab_pad_h = self._dim(16)
        tab_pad_v = self._dim(8)

        self.setStyleSheet(f"""
QMainWindow {{ background-color: {C_BASE}; color: {C_TEXT}; }}
QLabel {{ color: {C_TEXT}; }}
QFrame {{ background-color: {C_SURFACE0}; border-radius: {radius}px; }}
QTableWidget {{ background-color: {C_BASE}; color: {C_TEXT};
    gridline-color: {C_SURFACE1}; border: none;
    font-family: monospace; font-size: {self._sz(12)}px; }}
QTableWidget::item {{ padding: {pad_sm}px {pad_lg}px; }}
QTableWidget QHeaderView::section {{ background-color: {C_SURFACE0};
    color: {C_TEXT}; border: 1px solid {C_SURFACE1}; padding: {pad_md}px;
    font-weight: bold; font-size: {self._sz(12)}px; }}
QTabWidget::pane {{ border: 1px solid {C_SURFACE1};
    background-color: {C_BASE}; border-radius: {radius_sm}px; }}
QTabBar::tab {{ background-color: {C_SURFACE0}; color: {C_TEXT};
    padding: {tab_pad_v}px {tab_pad_h}px; margin-right: 2px;
    border-top-left-radius: {radius_sm}px;
    border-top-right-radius: {radius_sm}px;
    font-size: {self._sz(12)}px;
    min-height: {self._dim(20)}px; min-width: {self._dim(60)}px; }}
QTabBar::tab:selected {{ background-color: {C_SURFACE1}; color: {C_TEXT}; }}
QTextEdit {{ background-color: {C_BASE}; color: {C_GREEN};
    border: none; font-family: monospace; font-size: {self._sz(11)}px; }}
QGroupBox {{ color: {C_TEXT}; border: 1px solid {C_SURFACE1};
    border-radius: {radius_grp}px; margin-top: {grp_margin_top}px;
    padding-top: {grp_pad_top}px;
    font-weight: bold; font-size: {self._sz(12)}px; }}
QGroupBox::title {{ subcontrol-origin: margin; left: {margin}px;
    padding: 0 {radius_sm}px; }}
QPushButton {{ background-color: {C_SURFACE0}; color: {C_TEXT};
    border: 1px solid {C_SURFACE1}; border-radius: {radius_sm}px;
    padding: {pad_md}px {pad_xl}px;
    font-weight: bold; font-size: {self._sz(12)}px; }}
QPushButton:hover {{ background-color: {C_SURFACE1}; }}
QPushButton:disabled {{ color: {C_SUBTEXT}; }}
QSplitter::handle {{ background-color: {C_SURFACE1};
    border-radius: 2px; }}
QSplitter::handle:hover {{ background-color: {C_BLUE}; }}
        """)

        # Splitter handle width
        self.splitter_main.setHandleWidth(self._dim(6))

        # Status bar dimensions
        self.frame_status_bar.setMinimumHeight(sb_height)
        self.frame_status_bar.setMaximumHeight(sb_max)
        self.layout_status_bar.setContentsMargins(pad_xl, 0, pad_xl, 0)
        self.layout_status_bar.setSpacing(self._dim(24))

        # Main layout margins and spacing
        self.main_layout.setContentsMargins(margin, margin, margin, margin)
        self.main_layout.setSpacing(spacing)

        # Table row heights
        row_h = self._dim(28)
        for table in [self.table_joints, self.table_safety_limits]:
            for r in range(table.rowCount()):
                table.setRowHeight(r, row_h)
            table.resizeColumnsToContents()

        # Tab content margins
        for tab in [self.layout_fault_log, self.layout_safety, self.layout_raw_data]:
            tab.setContentsMargins(pad_lg, pad_lg, pad_lg, pad_lg)
            tab.setSpacing(spacing)

        # Button sizes
        self.button_estop.setMinimumSize(btn_w_estop, btn_h)
        self.button_reset.setMinimumSize(btn_w_reset, btn_h)

        # Re-apply button-specific styles (E-Stop red, Reset blue)
        self.button_estop.setStyleSheet(
            f'QPushButton {{ background-color: {C_RED}; color: {C_BASE};'
            f' border-radius: {radius_sm}px; font-weight: bold;'
            f' font-size: {self._sz(13)}px; }}'
            f' QPushButton:hover {{ background-color: #eba0ac; }}'
            f' QPushButton:disabled {{ background-color: {C_SURFACE1};'
            f' color: {C_SUBTEXT}; }}')
        self.button_reset.setStyleSheet(
            f'QPushButton {{ background-color: {C_BLUE}; color: {C_BASE};'
            f' border-radius: {radius_sm}px; font-weight: bold;'
            f' font-size: {self._sz(12)}px; }}'
            f' QPushButton:hover {{ background-color: #b4d0fb; }}'
            f' QPushButton:disabled {{ background-color: {C_SURFACE1};'
            f' color: {C_SUBTEXT}; }}')

        # Caption label font sizes
        for cap in [self.label_sb_mode_caption, self.label_sb_iface_caption,
                    self.label_sb_slaves_caption, self.label_sb_cycle_caption,
                    self.label_sb_safety_caption, self.label_sb_estop_caption]:
            cap.setStyleSheet(
                f'color: {C_SUBTEXT}; font-size: {self._sz(10)}px;')

        # Safety tab captions
        for cap in [self.label_safety_overall_caption,
                    self.label_watchdog_caption, self.label_wd_action_caption,
                    self.label_violations_caption]:
            cap.setStyleSheet(
                f'color: {C_SUBTEXT}; font-size: {self._sz(10)}px;')

        # Status bar value labels not updated by dynamic callbacks
        for val in [self.label_sb_iface_value, self.label_sb_slaves_value,
                    self.label_sb_cycle_value]:
            val.setStyleSheet(
                f'font-size: {self._sz(13)}px; font-weight: bold;')

        # Safety tab value labels not updated by dynamic callbacks
        for val in [self.label_wd_action, self.label_violations_count]:
            val.setStyleSheet(
                f'font-size: {self._sz(13)}px; font-weight: bold;')

        # Raw data text color override
        self.text_raw_data.setStyleSheet(
            f'color: {C_BLUE}; font-family: monospace;'
            f' font-size: {self._sz(11)}px;')

        # Refresh all dynamic labels (sets font-size via _set_label)
        self._set_label(self.label_sb_mode_value, 'ROS', C_GREEN)
        self._refresh()

    # ----------------------------------------------------------
    # Style helpers
    # ----------------------------------------------------------

    def _set_state_indicator(self, text, color):
        self.label_state_indicator.setText(text)
        self.label_state_indicator.setStyleSheet(
            f'color: {color}; font-size: {self._sz(14)}px; font-weight: bold;')

    def _set_label(self, label, text, color):
        label.setText(text)
        label.setStyleSheet(
            f'color: {color}; font-size: {self._sz(13)}px; font-weight: bold;')

    # ----------------------------------------------------------
    # UI loading
    # ----------------------------------------------------------

    def _load_ui(self):
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('arm_gui_tools')
            ui_file = os.path.join(pkg_share, 'ui', 'ethercat_monitor.ui')
        except Exception:
            ui_file = os.path.join(
                os.path.dirname(__file__), '../../ui/ethercat_monitor.ui')
        uic.loadUi(ui_file, self)

    def _setup_joint_table(self):
        table = self.table_joints
        table.setRowCount(len(JOINT_CONFIG))
        table.horizontalHeader().setStretchLastSection(True)
        table.verticalHeader().setVisible(False)

        for row, jcfg in enumerate(JOINT_CONFIG):
            _set_cell(table, row, 0, jcfg['display'], C_TEXT)
            _set_cell(table, row, 1, jcfg['motor'], C_BLUE)
            _set_cell(table, row, 2, str(jcfg['bus_pos']), C_BLUE)
            _set_cell(table, row, 3, '--', C_SUBTEXT)
            _set_cell(table, row, 4, '--', C_SUBTEXT)
            _set_cell(table, row, 5, '--', C_SUBTEXT)
            _set_cell(table, row, 6, 'N/A', C_SUBTEXT)
            _set_cell(table, row, 7, '--', C_BLUE)
            _set_cell(table, row, 8, '--', C_SUBTEXT)

        table.resizeColumnsToContents()

    def _setup_safety_table(self):
        table = self.table_safety_limits
        table.setRowCount(len(JOINT_CONFIG))
        table.horizontalHeader().setStretchLastSection(True)
        table.verticalHeader().setVisible(False)

        for row, jcfg in enumerate(JOINT_CONFIG):
            _set_cell(table, row, 0, jcfg['display'], C_TEXT)
            _set_cell(table, row, 1, f"{jcfg['pos_min']:+.2f}", C_TEXT)
            _set_cell(table, row, 2, f"{jcfg['pos_max']:+.2f}", C_TEXT)
            _set_cell(table, row, 3, f"{jcfg['vel_max']:.1f}", C_TEXT)
            _set_cell(table, row, 4, f"{jcfg['torque_max_pct']}", C_TEXT)
            _set_cell(table, row, 5, '--', C_SUBTEXT)

        table.resizeColumnsToContents()

    # ----------------------------------------------------------
    # ROS callbacks
    # ----------------------------------------------------------

    def joint_state_callback(self, msg):
        joints = {}
        for i, name in enumerate(msg.name):
            joints[name] = {
                'position': msg.position[i] if i < len(msg.position) else 0.0,
                'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                'effort': msg.effort[i] if i < len(msg.effort) else 0.0,
            }
        self._ros_data['joint_states'] = joints
        now = time.time()
        self._ros_data['last_update']['/joint_states'] = now
        self._ros_data['msg_counts'].setdefault('/joint_states', 0)
        self._ros_data['msg_counts']['/joint_states'] += 1
        self._refresh()

    def raw_positions_callback(self, msg):
        self._ros_data['raw_positions'] = list(msg.data)
        self._ros_data['last_update']['/ethercat/raw_positions'] = time.time()
        self._refresh()

    def status_words_callback(self, msg):
        self._ros_data['status_words'] = list(msg.data)
        self._ros_data['last_update']['/ethercat/status_words'] = time.time()
        self._refresh()

    def safety_status_callback(self, msg):
        self._ros_data['safety_ok'] = msg.data
        self._ros_data['last_update']['/safety/status'] = time.time()
        self._refresh()

    def diagnostics_callback(self, msg):
        self._ros_data['diagnostics'] = msg
        self._ros_data['last_update']['/diagnostics'] = time.time()
        self._refresh()

    def _get_drive_states(self, data):
        """Extract per-joint CiA 402 drive states.

        Primary source: /ethercat/status_words (decoded from status word).
        Fallback: /diagnostics key-value pairs with 'drive_state'.
        """
        drive_states = {}

        # Primary: decode from status words
        sw_list = data.get('status_words', [])
        if sw_list and _is_alive(data, '/ethercat/status_words'):
            for i, sw in enumerate(sw_list):
                if i < len(JOINT_CONFIG):
                    jname = JOINT_CONFIG[i]['name']
                    drive_states[jname] = _decode_cia402_state(sw)

        # Fallback: diagnostics topic
        if not drive_states:
            diag = data.get('diagnostics')
            if diag:
                for status in diag.status:
                    kv = {item.key: item.value for item in status.values}
                    if 'drive_state' in kv:
                        joint = kv.get('joint', status.name)
                        drive_states[joint] = kv['drive_state']

        return drive_states

    def _refresh(self):
        self._update_status_bar(self._ros_data)
        self._update_joint_table(self._ros_data)
        self._update_diagnostics(self._ros_data)
        self._update_buttons()

    # ----------------------------------------------------------
    # Status Bar
    # ----------------------------------------------------------

    def _update_status_bar(self, data):
        now = time.time()
        js_alive = _is_alive(data, '/joint_states')

        if js_alive is None:
            self._set_state_indicator('DISCONNECTED', C_SUBTEXT)
        elif js_alive:
            n_joints = len(data.get('joint_states', {}))
            expected = len(JOINT_CONFIG)
            self.label_sb_slaves_value.setText(
                f'{min(n_joints, expected)} / {expected}')

            if n_joints < expected:
                self._set_state_indicator('DEGRADED', C_PEACH)
            else:
                # Check drive states to determine real status
                drive_states = self._get_drive_states(data)
                has_fault = any(
                    ds in ('FAULT', 'FAULT_REACTION_ACTIVE')
                    for ds in drive_states.values())
                all_enabled = (
                    len(drive_states) >= expected and
                    all(ds == 'OPERATION_ENABLED'
                        for ds in drive_states.values()))
                any_enabled = any(
                    ds == 'OPERATION_ENABLED'
                    for ds in drive_states.values())

                if has_fault:
                    self._set_state_indicator('FAULT', C_RED)
                elif all_enabled:
                    self._set_state_indicator('OPERATIONAL', C_GREEN)
                elif any_enabled:
                    self._set_state_indicator('PARTIAL', C_YELLOW)
                elif drive_states:
                    self._set_state_indicator('DISABLED', C_PEACH)
                else:
                    self._set_state_indicator('CONNECTED', C_BLUE)
        else:
            self._set_state_indicator('STALE', C_RED)

        # Cycle rate
        topic = '/joint_states'
        count = data.get('msg_counts', {}).get(topic, 0)
        if topic not in self._rate_window_start:
            self._rate_window_start[topic] = now
            self._rate_window_count[topic] = count
        elapsed = now - self._rate_window_start[topic]
        if elapsed >= 2.0:
            delta = count - self._rate_window_count[topic]
            self._last_rates[topic] = delta / elapsed
            self._rate_window_start[topic] = now
            self._rate_window_count[topic] = count
        rate = self._last_rates.get(topic)
        self.label_sb_cycle_value.setText(
            f'{rate:.0f} Hz' if rate is not None else '-- Hz')

        # Safety
        safety_ok = data.get('safety_ok')
        if _is_alive(data, '/safety/status') is None or safety_ok is None:
            self._set_label(self.label_sb_safety_value, 'Unknown', C_SUBTEXT)
        elif safety_ok:
            self._set_label(self.label_sb_safety_value, 'OK', C_GREEN)
        else:
            self._set_label(self.label_sb_safety_value, 'FAULT', C_RED)

    # ----------------------------------------------------------
    # Joint Data Table
    # ----------------------------------------------------------

    def _update_joint_table(self, data):
        joint_data = data.get('joint_states', {})
        raw_positions = data.get('raw_positions', [])
        status_words = data.get('status_words', [])
        js_alive = _is_alive(data, '/joint_states')
        drive_states = self._get_drive_states(data)

        table = self.table_joints

        for row, jcfg in enumerate(JOINT_CONFIG):
            jname = jcfg['name']
            jd = joint_data.get(jname)

            if jd is None or not js_alive:
                _set_cell(table, row, 3, '--', C_SUBTEXT)
                _set_cell(table, row, 4, '--', C_SUBTEXT)
                _set_cell(table, row, 5, '--', C_SUBTEXT)
                _set_cell(table, row, 6, 'N/A', C_SUBTEXT)
                _set_cell(table, row, 7, '--', C_SUBTEXT)
                _set_cell(table, row, 8, '--', C_SUBTEXT)
                continue

            pos = jd['position']
            vel = jd['velocity']
            effort = jd['effort']

            # Position proximity
            pos_range = jcfg['pos_max'] - jcfg['pos_min']
            if pos_range > 0:
                pos_from_limit = min(
                    abs(pos - jcfg['pos_min']),
                    abs(pos - jcfg['pos_max']))
                half_range = pos_range / 2.0
                pos_ratio = max(0.0, 1.0 - (pos_from_limit / half_range))
            else:
                pos_ratio = 0.0
            _set_cell(table, row, 3, f'{pos:+8.3f}',
                      _status_color(pos_ratio))

            # Velocity
            abs_vel = abs(vel)
            vel_ratio = abs_vel / jcfg['vel_max'] if jcfg['vel_max'] > 0 else 0
            _set_cell(table, row, 4, f'{vel:+8.3f}',
                      _status_color(min(vel_ratio, 1.0)))

            # Torque
            abs_effort = abs(effort)
            torque_ratio = (abs_effort / jcfg['torque_max_pct']
                            if jcfg['torque_max_pct'] > 0 else 0)
            _set_cell(table, row, 5, f'{effort:+8.1f}',
                      _status_color(min(torque_ratio, 1.0)))

            # Drive state (from status words or diagnostics)
            ds = drive_states.get(jname)
            if ds:
                ds_text, ds_color = CIA402_STATES.get(ds, (ds, C_SUBTEXT))
                # Append status word hex if available
                if row < len(status_words):
                    ds_text = f'{ds_text} (0x{status_words[row] & 0xFFFF:04X})'
                _set_cell(table, row, 6, ds_text, ds_color)
            else:
                _set_cell(table, row, 6, 'N/A', C_SUBTEXT)

            # Raw encoder
            if row < len(raw_positions):
                _set_cell(table, row, 7, str(raw_positions[row]), C_BLUE)
            else:
                _set_cell(table, row, 7, '--', C_SUBTEXT)

            # Overall status
            worst = max(pos_ratio, min(vel_ratio, 1.0), min(torque_ratio, 1.0))
            status_txt = _status_text(worst)
            _set_cell(table, row, 8, status_txt, _status_color(worst))
            _set_cell(self.table_safety_limits, row, 5,
                      status_txt, _status_color(worst))

    # ----------------------------------------------------------
    # Diagnostic Tabs
    # ----------------------------------------------------------

    def _update_diagnostics(self, data):
        diag = data.get('diagnostics')
        if diag is None or diag is self._last_diag_id:
            self._update_raw_data(data)
            return
        self._last_diag_id = diag

        now_str = datetime.now().strftime('%H:%M:%S.%f')[:-3]

        for status in diag.status:
            kv = {item.key: item.value for item in status.values}
            name_lower = status.name.lower()

            if status.level == DiagnosticStatus.WARN:
                self._log_event(
                    now_str, f'[WARN] {status.name}: {status.message}',
                    C_YELLOW)
            elif status.level == DiagnosticStatus.ERROR:
                self._log_event(
                    now_str, f'[ERROR] {status.name}: {status.message}',
                    C_RED)
                joint = kv.get('joint', status.name)
                sw = kv.get('status_word', '--')
                ec = kv.get('error_code', '--')
                fault_key = f'{joint}:{sw}:{ec}'
                if fault_key not in self._known_faults:
                    self._known_faults.add(fault_key)

            if 'watchdog' in name_lower:
                if status.level == DiagnosticStatus.OK:
                    self._set_label(
                        self.label_watchdog_state, 'OK', C_GREEN)
                else:
                    self._set_label(
                        self.label_watchdog_state, 'TIMEOUT', C_RED)

            if 'estop' in name_lower or 'e-stop' in name_lower:
                active = kv.get('active', 'false').lower() == 'true'
                if active:
                    self._set_label(
                        self.label_sb_estop_value, 'ACTIVE', C_RED)
                else:
                    self._set_label(
                        self.label_sb_estop_value, 'Released', C_GREEN)

            if 'safety' in name_lower and 'overall' in name_lower:
                if status.level == DiagnosticStatus.OK:
                    self._set_label(
                        self.label_safety_overall, 'OK', C_GREEN)
                elif status.level == DiagnosticStatus.WARN:
                    self._set_label(
                        self.label_safety_overall, 'WARNING', C_YELLOW)
                else:
                    self._set_label(
                        self.label_safety_overall, 'FAULT', C_RED)

            if 'violation' in name_lower or 'joint_limit' in name_lower:
                count = kv.get('violation_count', '0')
                self.label_violations_count.setText(str(count))

        self._update_raw_data(data)

    def _update_raw_data(self, data):
        lines = []
        now_str = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        lines.append(f'--- Raw EtherCAT Data @ {now_str} ---')
        lines.append('')

        raw = data.get('raw_positions', [])
        status_words = data.get('status_words', [])

        if raw:
            lines.append('Raw Encoder Positions:')
            for i, val in enumerate(raw):
                if i < len(JOINT_CONFIG):
                    name = JOINT_CONFIG[i]['display']
                else:
                    name = f'Slave {i}'
                lines.append(f'  [{i}] {name:24s}  raw={val:+8d}  '
                             f'hex=0x{val & 0xFFFFFFFF:08X}')
        else:
            lines.append('Raw Encoder Positions: (no data)')

        lines.append('')

        if status_words:
            lines.append('CiA 402 Status Words:')
            for i, sw in enumerate(status_words):
                if i < len(JOINT_CONFIG):
                    name = JOINT_CONFIG[i]['display']
                else:
                    name = f'Slave {i}'
                state = _decode_cia402_state(sw)
                ds_text = CIA402_STATES.get(state, (state, ''))[0]
                lines.append(
                    f'  [{i}] {name:24s}  SW=0x{sw & 0xFFFF:04X}  '
                    f'state={ds_text}')
        else:
            lines.append('CiA 402 Status Words: (no data)')

        lines.append('')

        diag = data.get('diagnostics')
        if diag:
            lines.append('Diagnostic Messages:')
            for status in diag.status:
                kv = {item.key: item.value for item in status.values}
                sw = kv.get('status_word')
                ec = kv.get('error_code')
                parts = [f'  {status.name}:']
                if sw:
                    try:
                        parts.append(f'SW=0x{int(sw, 0):04X}')
                    except ValueError:
                        parts.append(f'SW={sw}')
                if ec:
                    try:
                        parts.append(f'EC=0x{int(ec, 0):04X}')
                    except ValueError:
                        parts.append(f'EC={ec}')
                if sw or ec:
                    lines.append(' '.join(parts))

        self.text_raw_data.setPlainText('\n'.join(lines))

    def _log_event(self, timestamp, message, color=C_GREEN):
        line = f'[{timestamp}] {message}'
        self._event_log.append(line)
        self.text_fault_log.append(
            f'<span style="color: {color};">{line}</span>')

    # ----------------------------------------------------------
    # Buttons
    # ----------------------------------------------------------

    def _update_buttons(self):
        estop_ready = self._cli_estop.service_is_ready()
        reset_ready = self._cli_reset.service_is_ready()

        # E-Stop always enabled — it's a critical safety control
        self.button_estop.setEnabled(True)
        self.button_reset.setEnabled(True)

        if not estop_ready:
            self.button_estop.setToolTip(
                'Service /safety/estop not available — will log warning')
        else:
            self.button_estop.setToolTip('Trigger software E-Stop')
        if not reset_ready:
            self.button_reset.setToolTip(
                'Service /safety/reset not available — will log warning')
        else:
            self.button_reset.setToolTip('Reset safety system')

    def _on_estop(self):
        now_str = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        if self._cli_estop.service_is_ready():
            self._cli_estop.call_async(Trigger.Request())
            self._log_event(now_str, '[ACTION] Software E-Stop triggered', C_RED)
        else:
            self._log_event(
                now_str,
                '[WARN] E-Stop service /safety/estop not available — '
                'is the safety node running?', C_YELLOW)

    def _on_reset(self):
        now_str = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        if self._cli_reset.service_is_ready():
            self._cli_reset.call_async(Trigger.Request())
            self._log_event(now_str, '[ACTION] Safety reset requested', C_BLUE)
        else:
            self._log_event(
                now_str,
                '[WARN] Reset service /safety/reset not available — '
                'is the safety node running?', C_YELLOW)

    def _on_clear_log(self):
        self._event_log.clear()
        self.text_fault_log.clear()
        self._known_faults.clear()


# ===========================================================
# ENTRY POINT
# ===========================================================

def main():
    rclpy.init(args=sys.argv)
    app = QApplication(sys.argv)

    ros_node = rclpy.create_node('ethercat_monitor_gui')
    window = EthercatMonitorWindow(ros_node)

    # Subscribe to topics
    ros_node.create_subscription(
        JointState, '/joint_states',
        window.joint_state_callback, 10)
    ros_node.create_subscription(
        Int32MultiArray, '/ethercat/raw_positions',
        window.raw_positions_callback, 10)
    ros_node.create_subscription(
        Int32MultiArray, '/ethercat/status_words',
        window.status_words_callback, 10)
    ros_node.create_subscription(
        Bool, '/safety/status',
        window.safety_status_callback, 10)
    ros_node.create_subscription(
        DiagnosticArray, '/diagnostics',
        window.diagnostics_callback, 10)

    # Spin ROS in Qt event loop
    ros_timer = QTimer()
    ros_timer.timeout.connect(
        lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    ros_timer.start(10)  # 100 Hz

    ros_node.get_logger().info('EtherCAT Monitor started')

    window.show()
    ret = app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()
