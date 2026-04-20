#!/usr/bin/env python3
"""Action Position Sender GUI — send joint positions via FollowJointTrajectory action.

Discovers active JointTrajectoryControllers, provides per-joint sliders with URDF
limits, sends goals via the action interface with feedback/result status, and
supports saving/loading named positions to YAML.

Follows the Catppuccin Mocha design guide (docs/GUI_DESIGN_GUIDE.md).
"""

import math
import os
import re
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET
from datetime import datetime
from functools import partial

import yaml

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QDoubleValidator, QFont
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSlider,
    QSplitter,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

# ───────────────────────────────────────────────────────────
# CATPPUCCIN MOCHA PALETTE
# ───────────────────────────────────────────────────────────

C_BASE = "#1e1e2e"
C_SURFACE0 = "#313244"
C_SURFACE1 = "#45475a"
C_TEXT = "#cdd6f4"
C_SUBTEXT = "#6c7086"
C_GREEN = "#a6e3a1"
C_YELLOW = "#f9e2af"
C_RED = "#f38ba8"
C_BLUE = "#89b4fa"
C_PEACH = "#fab387"

GLOBAL_STYLESHEET = """
QMainWindow { background-color: #1e1e2e; color: #cdd6f4; }
QLabel { color: #cdd6f4; }
QFrame { background-color: #313244; border-radius: 8px; }
QTableWidget { background-color: #1e1e2e; color: #cdd6f4; gridline-color: #45475a;
               border: none; font-family: monospace; font-size: 12px; }
QTableWidget::item { padding: 4px 8px; }
QTableWidget QHeaderView::section { background-color: #313244; color: #cdd6f4;
    border: 1px solid #45475a; padding: 6px; font-weight: bold; font-size: 12px; }
QTabWidget::pane { border: 1px solid #45475a; background-color: #1e1e2e; border-radius: 4px; }
QTabBar::tab { background-color: #313244; color: #cdd6f4; padding: 8px 16px;
    margin-right: 2px; border-top-left-radius: 4px; border-top-right-radius: 4px; }
QTabBar::tab:selected { background-color: #45475a; color: #cdd6f4; }
QTextEdit { background-color: #1e1e2e; color: #a6e3a1; border: none;
            font-family: monospace; font-size: 11px; }
QGroupBox { color: #cdd6f4; border: 1px solid #45475a; border-radius: 6px;
            margin-top: 8px; padding-top: 16px; font-weight: bold; }
QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px; }
QScrollArea { border: none; background-color: #1e1e2e; }
QScrollArea > QWidget > QWidget { background-color: #1e1e2e; }
QSlider::groove:horizontal { background: #313244; height: 6px; border-radius: 3px; }
QSlider::handle:horizontal { background: #89b4fa; width: 14px; margin: -4px 0;
                             border-radius: 7px; }
QSlider::sub-page:horizontal { background: #45475a; border-radius: 3px; }
QLineEdit { background-color: #313244; color: #cdd6f4; border: 1px solid #45475a;
            border-radius: 4px; padding: 4px 6px; font-family: monospace; font-size: 12px; }
QPushButton { background-color: #313244; color: #cdd6f4; border: 1px solid #45475a;
              border-radius: 4px; padding: 6px 12px; font-weight: bold; }
QPushButton:hover { background-color: #45475a; }
QPushButton:pressed { background-color: #585b70; }
QPushButton:disabled { color: #6c7086; }
QComboBox { background-color: #313244; color: #cdd6f4; border: 1px solid #45475a;
            border-radius: 4px; padding: 4px 8px; }
QComboBox::drop-down { border: none; }
QComboBox QAbstractItemView { background-color: #313244; color: #cdd6f4;
                              selection-background-color: #45475a; }
QDoubleSpinBox { background-color: #313244; color: #cdd6f4; border: 1px solid #45475a;
                 border-radius: 4px; padding: 4px 6px; font-family: monospace; }
QListWidget { background-color: #1e1e2e; color: #cdd6f4; border: 1px solid #45475a;
              border-radius: 4px; font-family: monospace; font-size: 12px; }
QListWidget::item:selected { background-color: #45475a; }
QDialog { background-color: #1e1e2e; color: #cdd6f4; }
"""

SAVED_POSITIONS_DIR = os.path.expanduser("~/.config/arm_gui_tools")
SAVED_POSITIONS_FILE = os.path.join(SAVED_POSITIONS_DIR, "saved_positions.yaml")

# Slider precision: 10000 steps per radian
SLIDER_SCALE = 10000

# Controller types we support
SUPPORTED_CONTROLLER_TYPES = {
    "joint_trajectory_controller/JointTrajectoryController": "action",
    "position_controllers/JointGroupPositionController": "topic",
}

# Strip ANSI escape codes
_ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")


# ───────────────────────────────────────────────────────────
# CHECKABLE COMBO BOX
# ───────────────────────────────────────────────────────────


class CheckableComboBox(QComboBox):
    """QComboBox with checkable items for multi-select."""

    selection_changed = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.view().pressed.connect(self._handle_item_pressed)
        self._updating = False

    def add_checkable_item(self, text, checked=False):
        self.addItem(text)
        item = self.model().item(self.count() - 1)
        item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
        item.setCheckState(Qt.Checked if checked else Qt.Unchecked)

    def _handle_item_pressed(self, index):
        item = self.model().itemFromIndex(index)
        if item.checkState() == Qt.Checked:
            item.setCheckState(Qt.Unchecked)
        else:
            item.setCheckState(Qt.Checked)
        self._update_text()
        self.selection_changed.emit()

    def _update_text(self):
        selected = self.checked_items()
        if selected:
            self.setCurrentText(f"{len(selected)} controller(s)")
        else:
            self.setCurrentText("No controllers")

    def checked_items(self):
        items = []
        for i in range(self.count()):
            if self.model().item(i).checkState() == Qt.Checked:
                items.append(self.itemText(i))
        return items

    def set_items(self, names, checked=None):
        self._updating = True
        self.clear()
        for name in names:
            is_checked = checked and name in checked
            self.add_checkable_item(name, is_checked)
        self._updating = False
        self._update_text()

    # Prevent popup from closing on click
    def hidePopup(self):
        if not self._updating:
            super().hidePopup()


# ───────────────────────────────────────────────────────────
# SAVE POSITION DIALOG
# ───────────────────────────────────────────────────────────


class SavePositionDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Save Position")
        self.setStyleSheet(f"background-color: {C_BASE}; color: {C_TEXT};")
        layout = QFormLayout(self)
        self.name_edit = QLineEdit()
        self.name_edit.setPlaceholderText("e.g. home, wave_left")
        self.desc_edit = QLineEdit()
        self.desc_edit.setPlaceholderText("Optional description")
        layout.addRow("Name:", self.name_edit)
        layout.addRow("Description:", self.desc_edit)
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addRow(buttons)


# ───────────────────────────────────────────────────────────
# JOINT CONTROL ROW
# ───────────────────────────────────────────────────────────


class JointControlWidget(QWidget):
    """Single joint row: name | current | [-] slider [+] | textbox."""

    value_changed = pyqtSignal(str, float)  # joint_name, radians

    def __init__(self, joint_name, lower, upper, parent=None):
        super().__init__(parent)
        self.joint_name = joint_name
        self.lower = lower
        self.upper = upper

        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 2, 4, 2)
        layout.setSpacing(6)

        # Joint name
        short_name = joint_name.replace("left_", "L ").replace("right_", "R ")
        self.name_label = QLabel(short_name)
        self.name_label.setFont(QFont("monospace", 11))
        self.name_label.setFixedWidth(200)
        self.name_label.setToolTip(joint_name)
        layout.addWidget(self.name_label)

        # Current position label
        self.cur_label = QLabel("+0.0000")
        self.cur_label.setFont(QFont("monospace", 11))
        self.cur_label.setStyleSheet(f"color: {C_GREEN};")
        self.cur_label.setFixedWidth(72)
        self.cur_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout.addWidget(self.cur_label)

        # Decrement button
        self.dec_btn = QPushButton("-")
        self.dec_btn.setFixedSize(28, 28)
        self.dec_btn.clicked.connect(self._decrement)
        layout.addWidget(self.dec_btn)

        # Slider
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(int(lower * SLIDER_SCALE), int(upper * SLIDER_SCALE))
        self.slider.setValue(0)
        self.slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.slider.valueChanged.connect(self._on_slider)
        layout.addWidget(self.slider)

        # Increment button
        self.inc_btn = QPushButton("+")
        self.inc_btn.setFixedSize(28, 28)
        self.inc_btn.clicked.connect(self._increment)
        layout.addWidget(self.inc_btn)

        # Text box
        self.textbox = QLineEdit("0.0000")
        self.textbox.setFont(QFont("monospace", 11))
        self.textbox.setFixedWidth(80)
        self.textbox.setAlignment(Qt.AlignRight)
        self.textbox.setValidator(QDoubleValidator(lower, upper, 4))
        self.textbox.editingFinished.connect(self._on_textbox)
        layout.addWidget(self.textbox)

        # Limits label
        lim_label = QLabel(f"[{lower:+.2f}, {upper:+.2f}]")
        lim_label.setFont(QFont("monospace", 9))
        lim_label.setStyleSheet(f"color: {C_SUBTEXT};")
        layout.addWidget(lim_label)

        self._step = 0.01  # default step

    def set_step(self, step):
        self._step = step

    def set_value(self, rad):
        rad = max(self.lower, min(self.upper, rad))
        self.slider.blockSignals(True)
        self.slider.setValue(int(rad * SLIDER_SCALE))
        self.slider.blockSignals(False)
        self.textbox.blockSignals(True)
        self.textbox.setText(f"{rad:.4f}")
        self.textbox.blockSignals(False)

    def get_value(self):
        return self.slider.value() / SLIDER_SCALE

    def set_current(self, rad):
        self.cur_label.setText(f"{rad:+.4f}")

    def _on_slider(self, value):
        rad = value / SLIDER_SCALE
        self.textbox.blockSignals(True)
        self.textbox.setText(f"{rad:.4f}")
        self.textbox.blockSignals(False)
        self.value_changed.emit(self.joint_name, rad)

    def _on_textbox(self):
        try:
            rad = float(self.textbox.text())
        except ValueError:
            return
        rad = max(self.lower, min(self.upper, rad))
        self.slider.blockSignals(True)
        self.slider.setValue(int(rad * SLIDER_SCALE))
        self.slider.blockSignals(False)
        self.textbox.setText(f"{rad:.4f}")
        self.value_changed.emit(self.joint_name, rad)

    def _increment(self):
        self.set_value(self.get_value() + self._step)
        self.value_changed.emit(self.joint_name, self.get_value())

    def _decrement(self):
        self.set_value(self.get_value() - self._step)
        self.value_changed.emit(self.joint_name, self.get_value())


# ───────────────────────────────────────────────────────────
# MAIN WINDOW
# ───────────────────────────────────────────────────────────


class ActionPositionSenderWindow(QMainWindow):

    # Signal to update log from ROS callback threads
    _log_signal = pyqtSignal(str, str)  # message, color
    _status_signal = pyqtSignal(str, str)  # controller, status

    def __init__(self, ros_node):
        super().__init__()
        self.setWindowTitle("Action Position Sender")
        self.setMinimumSize(1100, 700)
        self.setStyleSheet(GLOBAL_STYLESHEET)

        self._node = ros_node
        self._start_time = time.time()

        # State
        self._controllers = {}  # name -> list of joint names
        self._controller_types = {}  # name -> "action" or "topic"
        self._joint_limits = {}  # joint_name -> (lower, upper)
        self._current_positions = {}  # joint_name -> float
        self._current_efforts = {}  # joint_name -> float
        self._joint_widgets = {}  # joint_name -> JointControlWidget
        self._action_clients = {}  # controller_name -> ActionClient (for JTC)
        self._topic_publishers = {}  # controller_name -> Publisher (for JointGroupPosition)
        self._goal_handles = {}  # controller_name -> GoalHandle
        self._controller_status = {}  # controller_name -> str
        self._last_joint_state_time = 0.0
        self._is_real = False

        # Signals
        self._log_signal.connect(self._append_log)
        self._status_signal.connect(self._update_controller_status)

        self._build_ui()
        self._setup_ros()

        # Timers
        self._ros_timer = QTimer()
        self._ros_timer.timeout.connect(self._spin_ros)
        self._ros_timer.start(10)

        self._ui_timer = QTimer()
        self._ui_timer.timeout.connect(self._update_ui)
        self._ui_timer.start(100)

        # Initial controller discovery
        self._refresh_controllers()

    # ─── UI CONSTRUCTION ───────────────────────────────────

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(8)

        # Status bar
        self._build_status_bar(main_layout)

        # Toolbar
        self._build_toolbar(main_layout)

        # Splitter: joint controls | tabs
        splitter = QSplitter(Qt.Horizontal)
        splitter.setStyleSheet("QSplitter { background-color: #1e1e2e; }"
                               "QSplitter::handle { background-color: #45475a; width: 2px; }")

        # Left: joint controls scroll area
        self._scroll_area = QScrollArea()
        self._scroll_area.setWidgetResizable(True)
        self._scroll_content = QWidget()
        self._scroll_layout = QVBoxLayout(self._scroll_content)
        self._scroll_layout.setAlignment(Qt.AlignTop)
        self._scroll_area.setWidget(self._scroll_content)
        splitter.addWidget(self._scroll_area)

        # Right: tabs
        self._tabs = QTabWidget()
        self._build_current_state_tab()
        self._build_action_log_tab()
        self._build_saved_positions_tab()
        splitter.addWidget(self._tabs)

        splitter.setSizes([600, 500])
        main_layout.addWidget(splitter, 1)

        # Bottom bar
        self._build_bottom_bar(main_layout)

    def _build_status_bar(self, parent_layout):
        bar = QFrame()
        bar.setFixedHeight(60)
        bar.setStyleSheet(f"background-color: {C_SURFACE0}; border-radius: 8px;")
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(16, 0, 16, 0)
        layout.setSpacing(24)

        # State indicator
        self._state_label = QLabel("CONNECTING")
        self._state_label.setStyleSheet(f"color: {C_YELLOW}; font-size: 14px; font-weight: bold;")
        layout.addWidget(self._state_label)

        # Mode
        self._mode_label = self._make_metric(layout, "Mode", "---")

        # Controllers
        self._ctrl_count_label = self._make_metric(layout, "Controllers", "0")

        # Joints
        self._joint_count_label = self._make_metric(layout, "Joints", "0")

        # Uptime
        self._uptime_label = self._make_metric(layout, "Uptime", "00:00:00")

        layout.addStretch()
        parent_layout.addWidget(bar)

    def _make_metric(self, layout, caption, initial):
        vbox = QVBoxLayout()
        cap = QLabel(caption)
        cap.setStyleSheet(f"color: {C_SUBTEXT}; font-size: 10px;")
        val = QLabel(initial)
        val.setStyleSheet("font-size: 13px; font-weight: bold;")
        vbox.addWidget(cap)
        vbox.addWidget(val)
        layout.addLayout(vbox)
        return val

    def _build_toolbar(self, parent_layout):
        toolbar = QFrame()
        toolbar.setFixedHeight(50)
        toolbar.setStyleSheet(f"background-color: {C_SURFACE0}; border-radius: 8px;")
        layout = QHBoxLayout(toolbar)
        layout.setContentsMargins(12, 4, 12, 4)
        layout.setSpacing(12)

        # Controller selector
        lbl = QLabel("Controllers:")
        lbl.setStyleSheet(f"color: {C_SUBTEXT}; font-size: 10px;")
        layout.addWidget(lbl)

        self._ctrl_combo = CheckableComboBox()
        self._ctrl_combo.setMinimumWidth(220)
        self._ctrl_combo.selection_changed.connect(self._on_controller_selection_changed)
        layout.addWidget(self._ctrl_combo)

        # Refresh button
        refresh_btn = QPushButton("Refresh")
        refresh_btn.setFixedWidth(70)
        refresh_btn.clicked.connect(self._refresh_controllers)
        layout.addWidget(refresh_btn)

        layout.addSpacing(16)

        # Time from start
        lbl2 = QLabel("Duration (s):")
        lbl2.setStyleSheet(f"color: {C_SUBTEXT}; font-size: 10px;")
        layout.addWidget(lbl2)

        self._time_spin = QDoubleSpinBox()
        self._time_spin.setRange(0.5, 30.0)
        self._time_spin.setValue(2.0)
        self._time_spin.setSingleStep(0.5)
        self._time_spin.setDecimals(1)
        self._time_spin.setFixedWidth(70)
        layout.addWidget(self._time_spin)

        layout.addSpacing(16)

        # Step size
        lbl3 = QLabel("Step (rad):")
        lbl3.setStyleSheet(f"color: {C_SUBTEXT}; font-size: 10px;")
        layout.addWidget(lbl3)

        self._step_spin = QDoubleSpinBox()
        self._step_spin.setRange(0.001, 1.0)
        self._step_spin.setValue(0.01)
        self._step_spin.setSingleStep(0.005)
        self._step_spin.setDecimals(3)
        self._step_spin.setFixedWidth(70)
        self._step_spin.valueChanged.connect(self._on_step_changed)
        layout.addWidget(self._step_spin)

        layout.addSpacing(16)

        # Read Current button
        read_btn = QPushButton("Read Current")
        read_btn.clicked.connect(self._read_current_positions)
        layout.addWidget(read_btn)

        layout.addStretch()
        parent_layout.addWidget(toolbar)

    def _build_current_state_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(8, 8, 8, 8)

        self._state_table = QTableWidget()
        self._state_table.setColumnCount(5)
        self._state_table.setHorizontalHeaderLabels(
            ["Joint", "Current", "Target", "Delta", "Effort"]
        )
        self._state_table.horizontalHeader().setStretchLastSection(True)
        self._state_table.verticalHeader().setVisible(False)
        self._state_table.setAlternatingRowColors(True)
        self._state_table.setSelectionBehavior(self._state_table.SelectRows)
        layout.addWidget(self._state_table)

        self._tabs.addTab(widget, "Current State")

    def _build_action_log_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(8, 8, 8, 8)

        self._log_text = QTextEdit()
        self._log_text.setReadOnly(True)
        self._log_text.setFont(QFont("monospace", 11))
        self._log_text.setStyleSheet(
            f"background-color: {C_BASE}; color: {C_GREEN}; border: none;"
        )
        layout.addWidget(self._log_text)

        self._tabs.addTab(widget, "Action Log")

    def _build_saved_positions_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(8, 8, 8, 8)

        self._saved_list = QListWidget()
        layout.addWidget(self._saved_list, 1)

        btn_layout = QHBoxLayout()

        save_btn = QPushButton("Save Current")
        save_btn.clicked.connect(self._save_position)
        btn_layout.addWidget(save_btn)

        load_btn = QPushButton("Load")
        load_btn.clicked.connect(self._load_position)
        btn_layout.addWidget(load_btn)

        exec_btn = QPushButton("Execute")
        exec_btn.setStyleSheet(
            f"background-color: {C_SURFACE0}; color: {C_GREEN}; border: 1px solid {C_GREEN};"
        )
        exec_btn.clicked.connect(self._execute_position)
        btn_layout.addWidget(exec_btn)

        del_btn = QPushButton("Delete")
        del_btn.setStyleSheet(
            f"background-color: {C_SURFACE0}; color: {C_RED}; border: 1px solid {C_RED};"
        )
        del_btn.clicked.connect(self._delete_position)
        btn_layout.addWidget(del_btn)

        layout.addLayout(btn_layout)
        self._tabs.addTab(widget, "Saved Positions")

        self._refresh_saved_list()

    def _build_bottom_bar(self, parent_layout):
        bar = QFrame()
        bar.setFixedHeight(50)
        bar.setStyleSheet(f"background-color: {C_SURFACE0}; border-radius: 8px;")
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(12, 4, 12, 4)
        layout.setSpacing(12)

        self._send_btn = QPushButton("Send All")
        self._send_btn.setFixedWidth(100)
        self._send_btn.setStyleSheet(
            f"background-color: {C_SURFACE0}; color: {C_GREEN}; "
            f"border: 2px solid {C_GREEN}; font-size: 13px; font-weight: bold;"
        )
        self._send_btn.clicked.connect(self._send_goals)
        layout.addWidget(self._send_btn)

        self._cancel_btn = QPushButton("Cancel")
        self._cancel_btn.setFixedWidth(80)
        self._cancel_btn.setStyleSheet(
            f"background-color: {C_SURFACE0}; color: {C_RED}; "
            f"border: 2px solid {C_RED}; font-size: 13px; font-weight: bold;"
        )
        self._cancel_btn.setEnabled(False)
        self._cancel_btn.clicked.connect(self._cancel_goals)
        layout.addWidget(self._cancel_btn)

        layout.addSpacing(24)

        # Per-controller status labels
        self._ctrl_status_layout = QHBoxLayout()
        layout.addLayout(self._ctrl_status_layout)

        layout.addStretch()
        parent_layout.addWidget(bar)

    # ─── ROS SETUP ─────────────────────────────────────────

    def _setup_ros(self):
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, DurabilityPolicy

        self._node.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )
        # robot_description is published with TRANSIENT_LOCAL durability by
        # robot_state_publisher — must match QoS to receive the latched message
        qos_latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._node.create_subscription(
            String, "/robot_description", self._robot_description_cb, qos_latched
        )

        # Detect SIM vs REAL
        self._real_detect_sub = self._node.create_subscription(
            __import__("std_msgs.msg", fromlist=["Int32MultiArray"]).Int32MultiArray,
            "/ethercat/raw_positions",
            self._real_detect_cb,
            1,
        )
        self._real_detected = False

    def _joint_state_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self._current_positions[name] = pos
        if msg.effort:
            for name, eff in zip(msg.name, msg.effort):
                self._current_efforts[name] = eff
        self._last_joint_state_time = time.time()

    def _robot_description_cb(self, msg):
        self._parse_urdf_limits(msg.data)

    def _real_detect_cb(self, msg):
        if not self._real_detected:
            self._real_detected = True
            self._is_real = True

    def _parse_urdf_limits(self, urdf_xml):
        try:
            root = ET.fromstring(urdf_xml)
            for joint_el in root.iter("joint"):
                name = joint_el.get("name")
                if not name:
                    continue
                limit_el = joint_el.find("limit")
                if limit_el is not None:
                    lower = float(limit_el.get("lower", str(-math.pi)))
                    upper = float(limit_el.get("upper", str(math.pi)))
                    self._joint_limits[name] = (lower, upper)
            self._log(f"Parsed URDF: {len(self._joint_limits)} joint limits loaded", C_GREEN)
        except ET.ParseError as e:
            self._log(f"URDF parse error: {e}", C_RED)

    def _spin_ros(self):
        import rclpy
        rclpy.spin_once(self._node, timeout_sec=0)

    # ─── CONTROLLER DISCOVERY ──────────────────────────────

    def _refresh_controllers(self):
        self._log("Discovering controllers...", C_BLUE)
        thread = threading.Thread(target=self._discover_controllers, daemon=True)
        thread.start()

    def _discover_controllers(self):
        try:
            result = subprocess.run(
                ["ros2", "control", "list_controllers", "-v"],
                capture_output=True, text=True, timeout=10,
            )
            if result.returncode != 0:
                self._log_signal.emit(
                    f"Controller discovery failed: {result.stderr.strip()}", C_RED
                )
                return

            controllers = {}  # name -> list of joint names
            controller_types = {}  # name -> "action" or "topic"
            current_controller = None
            in_claimed = False

            for raw_line in result.stdout.splitlines():
                line = _ANSI_RE.sub("", raw_line).strip()
                if not line:
                    continue

                # Controller header line: "name  type  active/inactive"
                # Format: "whole_body_controller   position_controllers/JointGroupPositionController   active"
                ctrl_match = re.match(
                    r"^(\S+)\s+(\S+/\S+)\s+(active|inactive|unconfigured)", line
                )
                if ctrl_match:
                    name = ctrl_match.group(1)
                    ctype = ctrl_match.group(2)
                    state = ctrl_match.group(3)
                    if ctype in SUPPORTED_CONTROLLER_TYPES and state == "active":
                        current_controller = name
                        controllers[name] = []
                        controller_types[name] = SUPPORTED_CONTROLLER_TYPES[ctype]
                    else:
                        current_controller = None
                    in_claimed = False
                    continue

                if not current_controller:
                    continue

                # Detect "claimed interfaces:" section
                if "claimed interface" in line.lower():
                    in_claimed = True
                    continue

                # Any other section header ends claimed
                if line.endswith(":") and "claimed" not in line.lower():
                    in_claimed = False
                    continue

                # Parse claimed interface lines: "joint_name/position"
                if in_claimed:
                    iface_match = re.match(r"^(\S+)/position$", line)
                    if iface_match:
                        joint = iface_match.group(1)
                        if joint not in controllers[current_controller]:
                            controllers[current_controller].append(joint)

            self._controllers = controllers
            self._controller_types = controller_types
            type_summary = ", ".join(
                f"{n} ({controller_types[n]})" for n in controllers
            )
            self._log_signal.emit(
                f"Found {len(controllers)} active controller(s): {type_summary}",
                C_GREEN,
            )

            # Update combo on main thread
            QTimer.singleShot(0, self._populate_controller_combo)

        except subprocess.TimeoutExpired:
            self._log_signal.emit("Controller discovery timed out", C_RED)
        except FileNotFoundError:
            self._log_signal.emit("ros2 command not found", C_RED)

    def _populate_controller_combo(self):
        previously_checked = self._ctrl_combo.checked_items()
        self._ctrl_combo.set_items(
            list(self._controllers.keys()),
            checked=previously_checked if previously_checked else list(self._controllers.keys()),
        )
        self._on_controller_selection_changed()

    # ─── JOINT PANEL MANAGEMENT ────────────────────────────

    def _on_controller_selection_changed(self):
        selected = self._ctrl_combo.checked_items()
        self._rebuild_joint_panel(selected)
        self._rebuild_action_clients(selected)
        self._rebuild_status_labels(selected)
        self._update_state_table_rows()

    def _rebuild_joint_panel(self, selected_controllers):
        # Clear existing widgets
        self._joint_widgets.clear()
        while self._scroll_layout.count():
            child = self._scroll_layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

        step = self._step_spin.value()

        for ctrl_name in selected_controllers:
            joints = self._controllers.get(ctrl_name, [])
            if not joints:
                continue

            # Controller section header
            header = QLabel(f"  {ctrl_name}")
            header.setFont(QFont("monospace", 12, QFont.Bold))
            header.setStyleSheet(
                f"color: {C_BLUE}; background-color: {C_SURFACE0}; "
                f"border-radius: 4px; padding: 4px 8px;"
            )
            header.setFixedHeight(30)
            self._scroll_layout.addWidget(header)

            for joint_name in joints:
                lower, upper = self._joint_limits.get(joint_name, (-math.pi, math.pi))
                widget = JointControlWidget(joint_name, lower, upper)
                widget.set_step(step)
                # Set to current position if available
                if joint_name in self._current_positions:
                    widget.set_value(self._current_positions[joint_name])
                self._joint_widgets[joint_name] = widget
                self._scroll_layout.addWidget(widget)

        self._scroll_layout.addStretch()

        # Update counts
        total_joints = len(self._joint_widgets)
        self._joint_count_label.setText(str(total_joints))
        self._ctrl_count_label.setText(str(len(selected_controllers)))

    def _rebuild_action_clients(self, selected_controllers):
        from rclpy.action import ActionClient
        from control_msgs.action import FollowJointTrajectory
        from std_msgs.msg import Float64MultiArray

        # Remove clients for deselected controllers
        for name in list(self._action_clients.keys()):
            if name not in selected_controllers:
                client = self._action_clients[name]
                if isinstance(client, ActionClient):
                    client.destroy()
                # topic publishers are destroyed with the node
                del self._action_clients[name]
                self._goal_handles.pop(name, None)
                self._controller_status.pop(name, None)

        # Remove topic publishers for deselected controllers
        for name in list(self._topic_publishers.keys()):
            if name not in selected_controllers:
                self._node.destroy_publisher(self._topic_publishers[name])
                del self._topic_publishers[name]

        # Create clients for new controllers
        for name in selected_controllers:
            ctrl_type = self._controller_types.get(name, "action")
            if ctrl_type == "action" and name not in self._action_clients:
                topic = f"/{name}/follow_joint_trajectory"
                client = ActionClient(self._node, FollowJointTrajectory, topic)
                self._action_clients[name] = client
            elif ctrl_type == "topic" and name not in self._topic_publishers:
                topic = f"/{name}/commands"
                pub = self._node.create_publisher(Float64MultiArray, topic, 10)
                self._topic_publishers[name] = pub
            if name not in self._controller_status:
                self._controller_status[name] = "IDLE"

    def _rebuild_status_labels(self, selected_controllers):
        # Clear
        while self._ctrl_status_layout.count():
            child = self._ctrl_status_layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

        self._status_labels = {}
        for name in selected_controllers:
            lbl = QLabel(f"{name}: IDLE")
            lbl.setFont(QFont("monospace", 10))
            lbl.setStyleSheet(f"color: {C_SUBTEXT};")
            self._ctrl_status_layout.addWidget(lbl)
            self._status_labels[name] = lbl

    def _on_step_changed(self, value):
        for widget in self._joint_widgets.values():
            widget.set_step(value)

    def _read_current_positions(self):
        count = 0
        for joint_name, widget in self._joint_widgets.items():
            if joint_name in self._current_positions:
                widget.set_value(self._current_positions[joint_name])
                count += 1
        self._log(f"Read current positions for {count} joints", C_GREEN)

    # ─── SEND / CANCEL GOALS ──────────────────────────────

    def _send_goals(self):
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from builtin_interfaces.msg import Duration
        from std_msgs.msg import Float64MultiArray

        selected = self._ctrl_combo.checked_items()
        if not selected:
            self._log("No controllers selected", C_YELLOW)
            return

        duration = self._time_spin.value()
        dur_sec = int(duration)
        dur_nsec = int((duration - dur_sec) * 1e9)

        has_action = False

        for ctrl_name in selected:
            joints = self._controllers.get(ctrl_name, [])
            if not joints:
                continue

            # Collect positions
            positions = []
            for j in joints:
                widget = self._joint_widgets.get(j)
                if widget:
                    positions.append(widget.get_value())
                else:
                    positions.append(self._current_positions.get(j, 0.0))

            pos_str = ", ".join(f"{p:+.3f}" for p in positions)
            ctrl_type = self._controller_types.get(ctrl_name, "action")

            if ctrl_type == "topic":
                # JointGroupPositionController — send via topic
                pub = self._topic_publishers.get(ctrl_name)
                if not pub:
                    self._log(f"{ctrl_name}: publisher not ready", C_RED)
                    continue
                msg = Float64MultiArray()
                msg.data = positions
                pub.publish(msg)
                self._log(
                    f"{ctrl_name} (topic): Sent [{pos_str}]", C_BLUE
                )
                self._status_signal.emit(ctrl_name, "SENT")

            else:
                # JointTrajectoryController — send via action
                client = self._action_clients.get(ctrl_name)
                if not client:
                    continue

                if not client.wait_for_server(timeout_sec=0.1):
                    self._log(f"{ctrl_name}: action server not available", C_RED)
                    self._status_signal.emit(ctrl_name, "UNAVAILABLE")
                    continue

                point = JointTrajectoryPoint()
                point.positions = positions
                point.velocities = [0.0] * len(joints)
                point.time_from_start = Duration(sec=dur_sec, nanosec=dur_nsec)

                goal = FollowJointTrajectory.Goal()
                goal.trajectory = JointTrajectory()
                goal.trajectory.joint_names = list(joints)
                goal.trajectory.points = [point]
                goal.goal_time_tolerance = Duration(sec=5, nanosec=0)

                self._log(
                    f"{ctrl_name} (action): Sending [{pos_str}] in {duration:.1f}s",
                    C_BLUE,
                )

                future = client.send_goal_async(
                    goal, feedback_callback=partial(self._feedback_cb, ctrl_name)
                )
                future.add_done_callback(partial(self._goal_response_cb, ctrl_name))

                self._controller_status[ctrl_name] = "SENDING"
                self._status_signal.emit(ctrl_name, "SENDING")
                has_action = True

        if has_action:
            self._send_btn.setEnabled(False)
            self._cancel_btn.setEnabled(True)

    def _goal_response_cb(self, ctrl_name, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._log_signal.emit(f"{ctrl_name}: Goal REJECTED", C_RED)
            self._status_signal.emit(ctrl_name, "REJECTED")
            self._check_all_done()
            return

        self._goal_handles[ctrl_name] = goal_handle
        self._log_signal.emit(f"{ctrl_name}: Goal accepted, executing...", C_YELLOW)
        self._status_signal.emit(ctrl_name, "EXECUTING")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(partial(self._result_cb, ctrl_name))

    def _feedback_cb(self, ctrl_name, feedback_msg):
        fb = feedback_msg.feedback
        if fb.actual.positions:
            pos_str = ", ".join(f"{p:+.3f}" for p in fb.actual.positions[:3])
            if len(fb.actual.positions) > 3:
                pos_str += "..."
            self._log_signal.emit(
                f"{ctrl_name} feedback: [{pos_str}]", C_SUBTEXT
            )

    def _result_cb(self, ctrl_name, future):
        from action_msgs.msg import GoalStatus

        result = future.result()
        status = result.status

        self._goal_handles.pop(ctrl_name, None)

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._log_signal.emit(f"{ctrl_name}: SUCCEEDED", C_GREEN)
            self._status_signal.emit(ctrl_name, "SUCCEEDED")
        elif status == GoalStatus.STATUS_ABORTED:
            err = result.result
            self._log_signal.emit(
                f"{ctrl_name}: ABORTED (error_code={err.error_code}, "
                f"'{err.error_string}')",
                C_RED,
            )
            self._status_signal.emit(ctrl_name, "ABORTED")
        elif status == GoalStatus.STATUS_CANCELED:
            self._log_signal.emit(f"{ctrl_name}: CANCELED", C_YELLOW)
            self._status_signal.emit(ctrl_name, "CANCELED")
        else:
            self._log_signal.emit(
                f"{ctrl_name}: Ended with status {status}", C_PEACH
            )
            self._status_signal.emit(ctrl_name, f"STATUS:{status}")

        self._check_all_done()

    def _cancel_goals(self):
        for ctrl_name, handle in list(self._goal_handles.items()):
            self._log(f"{ctrl_name}: Canceling goal...", C_YELLOW)
            handle.cancel_goal_async()
        self._cancel_btn.setEnabled(False)

    def _check_all_done(self):
        if not self._goal_handles:
            QTimer.singleShot(0, lambda: self._send_btn.setEnabled(True))
            QTimer.singleShot(0, lambda: self._cancel_btn.setEnabled(False))

    # ─── STATUS / UI UPDATES ──────────────────────────────

    def _update_controller_status(self, ctrl_name, status):
        lbl = self._status_labels.get(ctrl_name)
        if not lbl:
            return
        color_map = {
            "IDLE": C_SUBTEXT,
            "SENDING": C_YELLOW,
            "EXECUTING": C_YELLOW,
            "SENT": C_GREEN,
            "SUCCEEDED": C_GREEN,
            "ABORTED": C_RED,
            "REJECTED": C_RED,
            "CANCELED": C_YELLOW,
            "UNAVAILABLE": C_RED,
        }
        color = color_map.get(status, C_SUBTEXT)
        lbl.setText(f"{ctrl_name}: {status}")
        lbl.setStyleSheet(f"color: {color};")
        self._controller_status[ctrl_name] = status

    def _update_ui(self):
        now = time.time()

        # Uptime
        elapsed = int(now - self._start_time)
        h, m, s = elapsed // 3600, (elapsed % 3600) // 60, elapsed % 60
        self._uptime_label.setText(f"{h:02d}:{m:02d}:{s:02d}")

        # Connection state
        age = now - self._last_joint_state_time if self._last_joint_state_time else 999
        if age < 2.0:
            self._state_label.setText("CONNECTED")
            self._state_label.setStyleSheet(
                f"color: {C_GREEN}; font-size: 14px; font-weight: bold;"
            )
        else:
            self._state_label.setText("STALE")
            self._state_label.setStyleSheet(
                f"color: {C_RED}; font-size: 14px; font-weight: bold;"
            )

        # Mode
        self._mode_label.setText("REAL" if self._is_real else "SIM")

        # Update current position labels on joint widgets
        for joint_name, widget in self._joint_widgets.items():
            if joint_name in self._current_positions:
                widget.set_current(self._current_positions[joint_name])

        # Update state table
        self._update_state_table()

    def _update_state_table_rows(self):
        joints = list(self._joint_widgets.keys())
        self._state_table.setRowCount(len(joints))
        for row, joint_name in enumerate(joints):
            self._set_cell(self._state_table, row, 0, joint_name, C_TEXT)

    def _update_state_table(self):
        for row in range(self._state_table.rowCount()):
            name_item = self._state_table.item(row, 0)
            if not name_item:
                continue
            joint_name = name_item.text()

            cur = self._current_positions.get(joint_name)
            widget = self._joint_widgets.get(joint_name)
            tgt = widget.get_value() if widget else None
            effort = self._current_efforts.get(joint_name)

            if cur is not None:
                self._set_cell(self._state_table, row, 1, f"{cur:+.4f}", C_TEXT)
            if tgt is not None:
                self._set_cell(self._state_table, row, 2, f"{tgt:+.4f}", C_TEXT)
            if cur is not None and tgt is not None:
                delta = abs(cur - tgt)
                if delta < 0.01:
                    color = C_GREEN
                elif delta < 0.1:
                    color = C_YELLOW
                else:
                    color = C_RED
                self._set_cell(self._state_table, row, 3, f"{delta:.4f}", color)
            if effort is not None:
                self._set_cell(self._state_table, row, 4, f"{effort:+.3f}", C_TEXT)

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

    # ─── LOGGING ───────────────────────────────────────────

    def _log(self, message, color=C_GREEN):
        self._log_signal.emit(message, color)

    def _append_log(self, message, color):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._log_text.append(
            f'<span style="color:{C_SUBTEXT}">[{ts}]</span> '
            f'<span style="color:{color}">{message}</span>'
        )
        # Trim to 500 lines
        doc = self._log_text.document()
        if doc.blockCount() > 500:
            cursor = self._log_text.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.movePosition(cursor.Down, cursor.KeepAnchor, doc.blockCount() - 500)
            cursor.removeSelectedText()

    # ─── SAVED POSITIONS ──────────────────────────────────

    def _load_saved_yaml(self):
        if not os.path.exists(SAVED_POSITIONS_FILE):
            return {}
        try:
            with open(SAVED_POSITIONS_FILE, "r") as f:
                data = yaml.safe_load(f) or {}
            return data.get("saved_positions", {})
        except Exception as e:
            self._log(f"Error loading saved positions: {e}", C_RED)
            return {}

    def _save_yaml(self, positions_dict):
        os.makedirs(SAVED_POSITIONS_DIR, exist_ok=True)
        data = {"saved_positions": positions_dict}
        with open(SAVED_POSITIONS_FILE, "w") as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)

    def _refresh_saved_list(self):
        self._saved_list.clear()
        saved = self._load_saved_yaml()
        for name, info in saved.items():
            desc = info.get("description", "")
            n_joints = len(info.get("positions", {}))
            tfs = info.get("time_from_start", "?")
            display = f"{name}  ({n_joints} joints, {tfs}s)"
            if desc:
                display += f"  -- {desc}"
            item = QListWidgetItem(display)
            item.setData(Qt.UserRole, name)
            self._saved_list.addItem(item)

    def _save_position(self):
        dialog = SavePositionDialog(self)
        if dialog.exec_() != QDialog.Accepted:
            return

        name = dialog.name_edit.text().strip()
        if not name:
            return
        # Sanitize name
        name = re.sub(r"[^a-zA-Z0-9_]", "_", name)

        desc = dialog.desc_edit.text().strip()
        positions = {}
        for joint_name, widget in self._joint_widgets.items():
            positions[joint_name] = round(widget.get_value(), 6)

        saved = self._load_saved_yaml()
        saved[name] = {
            "description": desc,
            "positions": positions,
            "time_from_start": self._time_spin.value(),
        }
        self._save_yaml(saved)
        self._refresh_saved_list()
        self._log(f"Saved position '{name}' ({len(positions)} joints)", C_GREEN)

    def _get_selected_save_name(self):
        item = self._saved_list.currentItem()
        if not item:
            self._log("No saved position selected", C_YELLOW)
            return None
        return item.data(Qt.UserRole)

    def _load_position(self):
        name = self._get_selected_save_name()
        if not name:
            return
        saved = self._load_saved_yaml()
        entry = saved.get(name)
        if not entry:
            return

        positions = entry.get("positions", {})
        tfs = entry.get("time_from_start")
        if tfs is not None:
            self._time_spin.setValue(float(tfs))

        loaded = 0
        skipped = []
        for joint_name, value in positions.items():
            widget = self._joint_widgets.get(joint_name)
            if widget:
                widget.set_value(value)
                loaded += 1
            else:
                skipped.append(joint_name)

        self._log(f"Loaded '{name}': {loaded} joints set", C_GREEN)
        if skipped:
            self._log(
                f"  Skipped (not in active controllers): {', '.join(skipped)}",
                C_YELLOW,
            )

    def _execute_position(self):
        name = self._get_selected_save_name()
        if not name:
            return
        # Load first, then send
        self._load_position()
        self._send_goals()

    def _delete_position(self):
        name = self._get_selected_save_name()
        if not name:
            return

        reply = QMessageBox.question(
            self,
            "Delete Position",
            f"Delete saved position '{name}'?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return

        saved = self._load_saved_yaml()
        saved.pop(name, None)
        self._save_yaml(saved)
        self._refresh_saved_list()
        self._log(f"Deleted position '{name}'", C_PEACH)

    # ─── CLEANUP ───────────────────────────────────────────

    def closeEvent(self, event):
        for client in self._action_clients.values():
            client.destroy()
        self._action_clients.clear()
        for pub in self._topic_publishers.values():
            self._node.destroy_publisher(pub)
        self._topic_publishers.clear()
        event.accept()


# ───────────────────────────────────────────────────────────
# ENTRY POINT
# ───────────────────────────────────────────────────────────


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    ros_node = rclpy.create_node("action_position_sender")

    app = QApplication(sys.argv)
    window = ActionPositionSenderWindow(ros_node)
    window.show()

    exit_code = app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
