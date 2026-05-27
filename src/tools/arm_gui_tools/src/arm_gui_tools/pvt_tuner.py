#!/usr/bin/env python3
"""Robot PVT tuner — per-joint live gain tuning for the 25-joint whole-body
RobotPVTController.

Discovers the active PVT controller (looks for a controller whose type ends in
``PVTController``) and the drive-status broadcaster (ends in
``DriveStatusBroadcaster``) by scanning the controller_manager. Lets the
operator pick one joint out of the controller's roster and edit its slice of
the per-joint gain arrays (Kp / Kd / Kd_damp / mgl / J / Fv / comp_sign /
hold_position / ff_*), toggle whole-body HOLD / FREE / DAMP, drive the
supervisor e-stop & reset, edit that joint's safety override
(``safety.joints.<joint>.<field>``), and live-monitor bus voltage, motor /
drive temperature, CiA 402 error code, and a tuner-computed following error
for the selected joint.

Single-file by design — palette and ros2-CLI fallback are inlined.
"""

import os
import re
import subprocess
import sys
import threading
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int8, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QTabWidget,
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

GLOBAL_STYLESHEET = f"""
QMainWindow {{ background-color: {C_BASE}; color: {C_TEXT}; }}
QLabel {{ color: {C_TEXT}; }}
QGroupBox {{ color: {C_TEXT}; border: 1px solid {C_SURFACE1}; border-radius: 6px;
             margin-top: 10px; padding: 12px 10px 10px 10px; font-weight: bold; }}
QGroupBox::title {{ subcontrol-origin: margin; left: 12px; padding: 0 4px;
                    color: {C_PEACH}; }}
QPushButton {{ background-color: {C_SURFACE0}; color: {C_TEXT};
               border: 1px solid {C_SURFACE1}; border-radius: 4px;
               padding: 6px 12px; font-weight: bold; }}
QPushButton:hover {{ background-color: {C_SURFACE1}; }}
QPushButton:pressed {{ background-color: #585b70; }}
QPushButton:disabled {{ color: {C_SUBTEXT}; }}
QDoubleSpinBox, QSpinBox {{ background-color: {C_SURFACE0}; color: {C_TEXT};
                            border: 1px solid {C_SURFACE1}; border-radius: 4px;
                            padding: 4px 6px; font-family: monospace;
                            min-width: 90px; }}
QComboBox {{ background-color: {C_SURFACE0}; color: {C_TEXT};
             border: 1px solid {C_SURFACE1}; border-radius: 4px;
             padding: 4px 8px; min-width: 180px; }}
QComboBox QAbstractItemView {{ background-color: {C_SURFACE0}; color: {C_TEXT};
                                selection-background-color: {C_SURFACE1}; }}
QCheckBox {{ color: {C_TEXT}; }}
QLineEdit {{ background-color: {C_SURFACE0}; color: {C_TEXT};
             border: 1px solid {C_SURFACE1}; border-radius: 4px;
             padding: 4px 6px; font-family: monospace; }}
"""

# ───────────────────────────────────────────────────────────
# Telemetry thresholds — keep aligned with drive_status_broadcaster YAML
# defaults at src/bringup/robot_bringup/config/controllers_pvt.yaml.
# ───────────────────────────────────────────────────────────

MOTOR_TEMP_WARN = 70.0
MOTOR_TEMP_ERROR = 90.0
DRIVE_TEMP_WARN = 70.0
DRIVE_TEMP_ERROR = 85.0
BUS_V_MIN = 40.0
BUS_V_MAX = 54.0

ESTOP_LABELS = {
    0: ("CLEAR", C_GREEN),
    1: ("E-STOP FREE", C_RED),
    2: ("E-STOP HOLD", C_RED),
}

# Supervisor node + safety topic namespace — both moved when the pendulum
# port became whole-body (was /pendulum_safety_supervisor + /pendulum/safety/*).
SUPERVISOR_NODE = "/robot_safety_supervisor"
SAFETY_ESTOP_TOPIC = "/safety/estop_state"
SAFETY_BREACH_TOPIC = "/safety/breach_reason"
SAFETY_KP_SCALE_TOPIC = "/safety/kp_scale"
JOINT_STATES_TOPIC = "/joint_states"


# ───────────────────────────────────────────────────────────
# Per-joint controller parameters — display order in the gains panel.
# Each entry: (param_name, label, lo, hi, step, decimals, kind)
#   kind ∈ {"double", "bool"}.  "double" → QDoubleSpinBox bound to
#   the j-th element of a double_array param; "bool" → QCheckBox bound
#   to the j-th element of a bool_array param.
# Critical: never push a length-1 array back — the controller's
# normalize() broadcasts a scalar to all 25 joints (see
# robot_pvt_controller.cpp normalize()). Apply always does
# read-modify-write on the full array.
# ───────────────────────────────────────────────────────────

PER_JOINT_FIELDS = (
    ("Kp",            "Kp",            0.0,    1000.0, 0.5,  3, "double"),
    ("Kd",            "Kd",            0.0,    100.0,  0.1,  3, "double"),
    ("Kd_damp",       "Kd_damp",       0.0,    100.0,  0.1,  3, "double"),
    ("mgl",           "mgl",         -100.0,    100.0, 0.01, 4, "double"),
    ("J",             "J",             0.0,    100.0,  0.001,4, "double"),
    ("Fv",            "Fv",            0.0,    100.0,  0.01, 4, "double"),
    ("comp_sign",     "comp_sign",    -1.0,    1.0,    1.0,  1, "double"),
    ("hold_position", "hold_position",-6.2832, 6.2832, 0.01, 4, "double"),
    ("ff_gravity",    "ff_gravity",    0,      0,      0,    0, "bool"),
    ("ff_inertia",    "ff_inertia",    0,      0,      0,    0, "bool"),
    ("ff_viscous",    "ff_viscous",    0,      0,      0,    0, "bool"),
)

# Per-joint safety override fields. The controller and supervisor both load
# safety.global.<field> and safety.joints.<joint>.<field>. Apply writes only
# to the joint override path — global edits affect every joint and are too
# dangerous to expose in a per-joint tuner. With the controller-side hot-
# reload in robot_pvt_controller.cpp post_set_param_cb_, writes here take
# effect at the next update().
SAFETY_LIMIT_FIELDS = (
    # (key, label, lo, hi, step, decimals)
    ("position_min",                "pos_min",        -6.2832,  6.2832, 0.01, 4),
    ("position_max",                "pos_max",        -6.2832,  6.2832, 0.01, 4),
    ("velocity_limit",              "vel_limit",         0.0,    100.0, 0.1,  3),
    ("effort_limit",                "effort_limit",      0.0,   1000.0, 0.1,  3),
    ("sustained_effort_threshold",  "sus_effort",        0.0,   1000.0, 0.1,  3),
    ("slew_rate_limit",             "slew_rate",         0.0,    100.0, 0.1,  3),
    ("acceleration_limit",          "accel_limit",       0.0,   1000.0, 1.0,  2),
)

# Body-group sizes for the scope banner. Stays in sync with body_groups/*.yaml.
BODY_GROUP_SIZES = {
    "arms": 12,
    "arms_waist": 13,
    "legs": 12,
    "full": 25,
}


# ───────────────────────────────────────────────────────────
# Main window
# ───────────────────────────────────────────────────────────


class PvtTunerWindow(QMainWindow):

    # Cross-thread signals (ROS callbacks → GUI thread)
    _voltage_sig = pyqtSignal(float)
    _motor_temp_sig = pyqtSignal(float)
    _drive_temp_sig = pyqtSignal(float)
    _error_code_sig = pyqtSignal(float)
    _following_err_sig = pyqtSignal(float)
    _kp_scale_sig = pyqtSignal(float)
    _estop_state_sig = pyqtSignal(int)
    _breach_sig = pyqtSignal(str)
    _service_result_sig = pyqtSignal(str, bool, str)  # label, ok, message
    _discovery_sig = pyqtSignal(list, object, int, str)  # pvt, broadcaster, count, source
    _gains_loaded_sig = pyqtSignal(str, object)  # ctrl, {param_name: list}
    _gains_applied_sig = pyqtSignal(object)      # list of (name, ok, msg)
    _all_params_sig = pyqtSignal(str, object)    # ctrl, {name: value}
    _param_set_sig = pyqtSignal(str, bool, str)  # name, ok, message
    # Safety panel — loaded per joint: {field: (sup_global, sup_jt, ctrl_global, ctrl_jt)}
    _safety_loaded_sig = pyqtSignal(object)
    # Apply result: [(field, sup_ok, ctrl_ok, msg)]
    _safety_applied_sig = pyqtSignal(object)

    def __init__(self, node: Node):
        super().__init__()
        self.setWindowTitle("Robot PVT Tuner")
        # Two-column layout fits in ~700 px tall — leaves room on a 1080 p
        # display for the title bar, IDE chrome, and the OS taskbar.
        self.setMinimumSize(1200, 680)
        self.setStyleSheet(GLOBAL_STYLESHEET)

        self._node = node
        self._controllers: list[str] = []
        self._current_ctrl: str | None = None
        self._joints: list[str] = []          # full roster (25 names)
        self._joint_idx: int = -1              # index into self._joints
        self._body_group: str = "—"
        self._drive_side_pd: bool = True

        # Cached per-joint arrays — read-modify-write on Apply.
        self._gain_arrays: dict[str, list] = {}

        # Last broadcaster MultiArray payloads. None until first publication.
        self._last_motor_temp: list[float] | None = None
        self._last_drive_temp: list[float] | None = None
        self._last_bus_voltage: list[float] | None = None
        self._last_error_code: list[float] | None = None
        self._last_kp_scale: list[float] | None = None

        # Following-error sources. q_cmd populated from /robot_pvt_controller/
        # setpoint (legacy streaming path). q_meas from /joint_states.
        self._q_cmd: dict[str, float] = {}
        self._q_meas: dict[str, float] = {}

        # Subscriptions that depend on a discovered broadcaster — rebuilt
        # when the broadcaster namespace changes.
        self._broadcaster_subs: list = []
        self._broadcaster_ns: str | None = None
        # Subscriptions that depend on a discovered PVT controller —
        # rebuilt when the controller selection changes.
        self._controller_subs: list = []

        # Build UI — two tabs: focused tuner + full parameter table
        self._tabs = QTabWidget()
        self.setCentralWidget(self._tabs)

        tuner_tab = QWidget()
        outer = QVBoxLayout(tuner_tab)
        outer.setContentsMargins(0, 0, 0, 0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        outer.addWidget(scroll)
        scroll_host = QWidget()
        scroll.setWidget(scroll_host)

        # Two columns side-by-side. Left: controller picker, gain editor,
        # mode triggers — the panels the operator interacts with most.
        # Right: safety status, safety overrides, telemetry — read-heavy.
        cols = QHBoxLayout(scroll_host)
        cols.setSpacing(10)
        cols.setContentsMargins(12, 12, 12, 12)
        left = QVBoxLayout()
        left.setSpacing(10)
        right = QVBoxLayout()
        right.setSpacing(10)
        cols.addLayout(left, 1)
        cols.addLayout(right, 1)

        self._build_controller_box(left)
        self._build_gains_box(left)
        self._build_mode_box(left)
        left.addStretch()

        self._build_safety_box(right)
        self._build_safety_limits_box(right)
        self._build_telem_box(right)
        right.addStretch()

        self._tabs.addTab(tuner_tab, "Tuner")

        params_tab = QWidget()
        self._build_params_tab(params_tab)
        self._tabs.addTab(params_tab, "All parameters")

        # Wire cross-thread signals
        self._voltage_sig.connect(self._on_voltage)
        self._motor_temp_sig.connect(self._on_motor_temp)
        self._drive_temp_sig.connect(self._on_drive_temp)
        self._error_code_sig.connect(self._on_error_code)
        self._following_err_sig.connect(self._on_following_err)
        self._kp_scale_sig.connect(self._on_kp_scale)
        self._estop_state_sig.connect(self._on_estop_state)
        self._breach_sig.connect(self._on_breach)
        self._service_result_sig.connect(self._on_service_result)
        self._discovery_sig.connect(self._on_discovery_result)
        self._gains_loaded_sig.connect(self._on_gains_loaded)
        self._gains_applied_sig.connect(self._on_gains_applied)
        self._all_params_sig.connect(self._on_all_params_loaded)
        self._param_set_sig.connect(self._on_param_set)
        self._safety_loaded_sig.connect(self._on_safety_loaded)
        self._safety_applied_sig.connect(self._on_safety_applied)

        # Subscriptions that don't depend on controller discovery —
        # /safety/* topics (latched, from the supervisor) and /joint_states.
        self._setup_safety_subs()
        self._setup_joint_states_sub()

        # One-shot discovery at startup. Re-runs only on Refresh click —
        # cross-machine `ros2 control list_controllers` can take 10s+, so
        # polling would pile up parallel invocations and never finish.
        self._discovery_in_flight = False
        self._discover_async()

    # ─── UI BUILDERS ────────────────────────────────────────

    def _build_controller_box(self, parent):
        box = QGroupBox("Controller && joint")
        v = QVBoxLayout(box)

        # Row 1: controller dropdown + Refresh
        h1 = QHBoxLayout()
        h1.setSpacing(10)
        h1.addWidget(QLabel("ctrl:"))
        self._ctrl_combo = QComboBox()
        self._ctrl_combo.currentTextChanged.connect(self._on_controller_changed)
        h1.addWidget(self._ctrl_combo)
        h1.addStretch()
        self._refresh_btn = QPushButton("Refresh")
        self._refresh_btn.setFixedWidth(80)
        self._refresh_btn.clicked.connect(self._discover_async)
        h1.addWidget(self._refresh_btn)
        v.addLayout(h1)

        # Row 2: joint dropdown
        h2 = QHBoxLayout()
        h2.setSpacing(10)
        h2.addWidget(QLabel("joint:"))
        self._joint_combo = QComboBox()
        self._joint_combo.setMinimumWidth(360)
        self._joint_combo.currentIndexChanged.connect(self._on_joint_changed)
        h2.addWidget(self._joint_combo)
        h2.addStretch()
        v.addLayout(h2)

        # Row 3: scope banner — body_group + active count
        self._scope_label = QLabel("scope: (unknown)")
        self._scope_label.setFont(QFont("monospace", 10))
        self._scope_label.setStyleSheet(f"color: {C_SUBTEXT};")
        v.addWidget(self._scope_label)

        # Row 4: discovery status
        self._discovery_status = QLabel("(discovering…)")
        self._discovery_status.setFont(QFont("monospace", 9))
        self._discovery_status.setStyleSheet(f"color: {C_SUBTEXT};")
        v.addWidget(self._discovery_status)
        parent.addWidget(box)

    def _build_gains_box(self, parent):
        box = QGroupBox("Gains && feedforward  (per-joint slice of array)")
        grid = QGridLayout(box)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(6)

        # Two fields per row: (label, widget, label, widget) across 6 cols.
        self._gain_widgets: dict[str, QWidget] = {}
        for idx, (name, label, lo, hi, step, decimals, kind) in enumerate(
            PER_JOINT_FIELDS
        ):
            row = idx // 2
            col = (idx % 2) * 3
            if kind == "double":
                w = self._make_spin(lo, hi, step, decimals)
            else:  # bool
                w = QCheckBox()
            self._gain_widgets[name] = w
            grid.addWidget(QLabel(label), row, col)
            grid.addWidget(w, row, col + 1)

        btn_row = (len(PER_JOINT_FIELDS) + 1) // 2 + 1
        self._gains_read_btn = QPushButton("Read")
        self._gains_read_btn.setStyleSheet(
            f"background-color: {C_SURFACE0}; color: {C_BLUE}; "
            f"border: 1px solid {C_BLUE};"
        )
        self._gains_read_btn.clicked.connect(self._on_read_gains)
        grid.addWidget(self._gains_read_btn, btn_row, 4)

        self._gains_apply_btn = QPushButton("Apply")
        self._gains_apply_btn.setStyleSheet(
            f"background-color: {C_SURFACE0}; color: {C_GREEN}; "
            f"border: 1px solid {C_GREEN};"
        )
        self._gains_apply_btn.clicked.connect(self._on_apply_gains)
        grid.addWidget(self._gains_apply_btn, btn_row, 5)

        self._gains_status = QLabel(" ")
        self._gains_status.setFont(QFont("monospace", 9))
        self._gains_status.setStyleSheet(f"color: {C_SUBTEXT};")
        grid.addWidget(self._gains_status, btn_row + 1, 0, 1, 6)

        parent.addWidget(box)

    def _build_mode_box(self, parent):
        box = QGroupBox("Mode")
        v = QVBoxLayout(box)

        # Per [[feedback-real-hardware-safety]]: the operator must see at a
        # glance that these affect every active joint in the body group, not
        # the selected joint.
        self._mode_scope_label = QLabel(
            "mode actions affect ALL active joints in scope"
        )
        self._mode_scope_label.setFont(QFont("monospace", 9, QFont.Bold))
        self._mode_scope_label.setStyleSheet(f"color: {C_PEACH};")
        v.addWidget(self._mode_scope_label)

        h = QHBoxLayout()
        self._hold_btn = QPushButton("HOLD (activate PVT)")
        self._hold_btn.setStyleSheet(
            f"background-color: {C_SURFACE0}; color: {C_GREEN}; "
            f"border: 2px solid {C_GREEN};"
        )
        self._hold_btn.clicked.connect(lambda: self._call_controller_trigger("hold"))
        h.addWidget(self._hold_btn)

        self._free_btn = QPushButton("FREE (zero torque)")
        self._free_btn.setStyleSheet(
            f"background-color: {C_SURFACE0}; color: {C_YELLOW}; "
            f"border: 2px solid {C_YELLOW};"
        )
        self._free_btn.clicked.connect(lambda: self._call_controller_trigger("free"))
        h.addWidget(self._free_btn)

        self._damp_btn = QPushButton("DAMP (brake)")
        self._damp_btn.setStyleSheet(
            f"background-color: {C_SURFACE0}; color: {C_PEACH}; "
            f"border: 2px solid {C_PEACH};"
        )
        self._damp_btn.clicked.connect(lambda: self._call_controller_trigger("damp"))
        h.addWidget(self._damp_btn)
        v.addLayout(h)

        self._mode_label = QLabel("last action: —")
        self._mode_label.setFont(QFont("monospace", 10))
        self._mode_label.setStyleSheet(f"color: {C_SUBTEXT};")
        v.addWidget(self._mode_label)
        parent.addWidget(box)

    def _build_safety_box(self, parent):
        box = QGroupBox("Safety")
        v = QVBoxLayout(box)
        h = QHBoxLayout()
        self._estop_btn = QPushButton("E-STOP")
        self._estop_btn.setStyleSheet(
            f"background-color: {C_RED}; color: {C_BASE}; "
            f"border: 2px solid {C_RED}; font-size: 14px;"
        )
        self._estop_btn.clicked.connect(
            lambda: self._call_supervisor_trigger("estop")
        )
        h.addWidget(self._estop_btn)

        self._reset_btn = QPushButton("RESET")
        self._reset_btn.setStyleSheet(
            f"background-color: {C_SURFACE0}; color: {C_PEACH}; "
            f"border: 2px solid {C_PEACH};"
        )
        self._reset_btn.clicked.connect(
            lambda: self._call_supervisor_trigger("reset")
        )
        h.addWidget(self._reset_btn)
        h.addStretch()
        v.addLayout(h)

        info = QGridLayout()
        info.addWidget(QLabel("state:"), 0, 0)
        self._estop_state_label = QLabel("(waiting)")
        self._estop_state_label.setFont(QFont("monospace", 11, QFont.Bold))
        info.addWidget(self._estop_state_label, 0, 1)
        info.addWidget(QLabel("breach:"), 1, 0)
        self._breach_label = QLabel("—")
        self._breach_label.setFont(QFont("monospace", 10))
        self._breach_label.setStyleSheet(f"color: {C_SUBTEXT};")
        info.addWidget(self._breach_label, 1, 1)
        info.setColumnStretch(1, 1)
        v.addLayout(info)
        parent.addWidget(box)

    def _build_safety_limits_box(self, parent):
        box = QGroupBox(
            "Safety limits  (per-joint override → supervisor && controller)"
        )
        grid = QGridLayout(box)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(6)

        # Two fields per row: (label, spin, source-tag) × 2 across 8 columns.
        # source-tag is "(global)" / "(override)" so the operator sees which
        # path the displayed value came from.
        self._safety_spins: dict[str, QDoubleSpinBox] = {}
        self._safety_origin_labels: dict[str, QLabel] = {}
        for idx, (key, label, lo, hi, step, decimals) in enumerate(
            SAFETY_LIMIT_FIELDS
        ):
            row = idx // 2
            col = (idx % 2) * 4
            spin = self._make_spin(lo, hi, step, decimals)
            self._safety_spins[key] = spin
            origin = QLabel("—")
            origin.setFont(QFont("monospace", 9))
            origin.setStyleSheet(f"color: {C_SUBTEXT};")
            self._safety_origin_labels[key] = origin
            grid.addWidget(QLabel(label), row, col)
            grid.addWidget(spin, row, col + 1)
            grid.addWidget(origin, row, col + 2)

        btn_row = (len(SAFETY_LIMIT_FIELDS) + 1) // 2 + 1
        self._safety_read_btn = QPushButton("Read")
        self._safety_read_btn.setStyleSheet(
            f"background-color: {C_SURFACE0}; color: {C_BLUE}; "
            f"border: 1px solid {C_BLUE};"
        )
        self._safety_read_btn.clicked.connect(self._on_read_safety_limits)
        grid.addWidget(self._safety_read_btn, btn_row, 6)

        self._safety_apply_btn = QPushButton("Apply override")
        self._safety_apply_btn.setStyleSheet(
            f"background-color: {C_SURFACE0}; color: {C_GREEN}; "
            f"border: 1px solid {C_GREEN};"
        )
        self._safety_apply_btn.clicked.connect(self._on_apply_safety_limits)
        grid.addWidget(self._safety_apply_btn, btn_row, 7)

        self._safety_status = QLabel(" ")
        self._safety_status.setFont(QFont("monospace", 9))
        self._safety_status.setStyleSheet(f"color: {C_SUBTEXT};")
        grid.addWidget(self._safety_status, btn_row + 1, 0, 1, 8)

        parent.addWidget(box)

    def _build_telem_box(self, parent):
        box = QGroupBox("Telemetry  (selected joint)")
        grid = QGridLayout(box)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(6)

        def mk_label():
            lab = QLabel("—")
            lab.setFont(QFont("monospace", 12, QFont.Bold))
            lab.setStyleSheet(f"color: {C_SUBTEXT};")
            return lab

        grid.addWidget(QLabel("bus voltage"), 0, 0)
        self._voltage_label = mk_label()
        grid.addWidget(self._voltage_label, 0, 1)

        grid.addWidget(QLabel("motor temp"), 1, 0)
        self._motor_temp_label = mk_label()
        grid.addWidget(self._motor_temp_label, 1, 1)

        grid.addWidget(QLabel("drive temp"), 2, 0)
        self._drive_temp_label = mk_label()
        grid.addWidget(self._drive_temp_label, 2, 1)

        grid.addWidget(QLabel("following error"), 3, 0)
        self._foll_err_label = mk_label()
        self._foll_err_label.setToolTip(
            "Computed in the tuner: q_cmd (last ~/setpoint) − q_meas "
            "(/joint_states). Will be stale during a ~/follow_joint_trajectory "
            "action goal — the controller does not republish its interpolated "
            "target."
        )
        grid.addWidget(self._foll_err_label, 3, 1)

        grid.addWidget(QLabel("error code"), 4, 0)
        self._error_code_label = mk_label()
        self._error_code_label.setToolTip(
            "CiA 402 error register from /robot_drive_status_broadcaster/"
            "error_code. Nonzero → drive fault."
        )
        grid.addWidget(self._error_code_label, 4, 1)

        grid.addWidget(QLabel("kp scale"), 5, 0)
        self._kp_scale_label = mk_label()
        self._kp_scale_label.setToolTip(
            "Per-joint Kp multiplier from /safety/kp_scale. <1.0 → thermal "
            "or sustained-effort derate."
        )
        grid.addWidget(self._kp_scale_label, 5, 1)

        self._broadcaster_label = QLabel("(searching for drive_status_broadcaster…)")
        self._broadcaster_label.setFont(QFont("monospace", 9))
        self._broadcaster_label.setStyleSheet(f"color: {C_SUBTEXT};")
        grid.addWidget(self._broadcaster_label, 6, 0, 1, 2)
        grid.setColumnStretch(1, 1)
        parent.addWidget(box)

    @staticmethod
    def _make_spin(lo, hi, step, decimals):
        s = QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setSingleStep(step)
        s.setDecimals(decimals)
        return s

    # ─── DISCOVERY ──────────────────────────────────────────

    def _discover_async(self):
        if self._discovery_in_flight:
            print("[pvt_tuner] discovery already in flight; ignoring",
                  file=sys.stderr, flush=True)
            return
        self._discovery_in_flight = True
        self._refresh_btn.setEnabled(False)
        self._refresh_btn.setText("…")
        print("[pvt_tuner] discovery cycle starting", file=sys.stderr, flush=True)
        threading.Thread(target=self._discover_worker_safe, daemon=True).start()

    def _discover_worker_safe(self):
        try:
            self._discover_worker()
        except Exception as e:
            print(
                f"[pvt_tuner] discovery thread crashed: {e!r}",
                file=sys.stderr, flush=True,
            )
            QTimer.singleShot(0, self._discovery_done)

    def _discovery_done(self):
        self._discovery_in_flight = False
        self._refresh_btn.setEnabled(True)
        self._refresh_btn.setText("Refresh")

    def _discover_worker(self):
        """Try rclpy /controller_manager/list_controllers first (uses the GUI's
        warm DDS cache → fast), fall back to `ros2 control list_controllers`
        subprocess (slow on cold cross-machine DDS, but always works)."""
        print("[pvt_tuner] trying rclpy ListControllers…",
              file=sys.stderr, flush=True)
        controllers = self._list_controllers_rclpy()
        source = "rclpy"
        if controllers is None:
            print("[pvt_tuner] rclpy path failed; falling back to subprocess "
                  "(may take 10–30s on first call cross-machine)…",
                  file=sys.stderr, flush=True)
            controllers = self._list_controllers_subprocess(timeout=30)
            source = "subprocess"

        pvt: list[str] = []
        broadcaster_ns: str | None = None
        for name, ctype, state in controllers:
            print(f"[pvt_tuner]   - {name!r}  type={ctype!r}  state={state!r}",
                  file=sys.stderr, flush=True)
            if state != "active":
                continue
            if ctype.endswith("PVTController"):
                pvt.append(f"/{name}")
            elif ctype.endswith("DriveStatusBroadcaster"):
                broadcaster_ns = f"/{name}"

        pvt.sort()
        print(f"[pvt_tuner] dispatching to GUI: pvt={pvt} broadcaster={broadcaster_ns}",
              file=sys.stderr, flush=True)
        self._discovery_sig.emit(pvt, broadcaster_ns, len(controllers), source)

    def _list_controllers_rclpy(self):
        """Call /controller_manager/list_controllers from the GUI's rclpy node.

        Returns a list of (name, type, state) on success, or None on failure.
        Verbose stderr logging so we can pin down which step stalls in
        cross-machine setups where DDS discovery is slow."""
        print("[pvt_tuner] rclpy: creating client", file=sys.stderr, flush=True)
        client = self._node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        try:
            print("[pvt_tuner] rclpy: waiting for service (≤30s)",
                  file=sys.stderr, flush=True)
            t0 = time.time()
            ok = client.wait_for_service(timeout_sec=30.0)
            print(f"[pvt_tuner] rclpy: wait_for_service={ok} "
                  f"after {time.time()-t0:.1f}s", file=sys.stderr, flush=True)
            if not ok:
                return None
            print("[pvt_tuner] rclpy: sending request", file=sys.stderr, flush=True)
            future = client.call_async(ListControllers.Request())
            deadline = time.time() + 30.0
            last_log = time.time()
            while not future.done() and time.time() < deadline:
                time.sleep(0.05)
                if time.time() - last_log > 2.0:
                    print(f"[pvt_tuner] rclpy: still waiting on response "
                          f"({time.time()-t0:.1f}s)",
                          file=sys.stderr, flush=True)
                    last_log = time.time()
            if not future.done():
                print("[pvt_tuner] rclpy: response timed out after 30s",
                      file=sys.stderr, flush=True)
                return None
            resp = future.result()
            print(f"[pvt_tuner] rclpy: response received with "
                  f"{len(resp.controller)} controller(s)",
                  file=sys.stderr, flush=True)
            return [(c.name, c.type, c.state) for c in resp.controller]
        finally:
            self._node.destroy_client(client)

    @staticmethod
    def _list_controllers_subprocess(timeout=10):
        """Parse `ros2 control list_controllers` output.

        Each row: ``name  pkg/Plugin  active|inactive|unconfigured``."""
        try:
            r = subprocess.run(
                ["ros2", "control", "list_controllers"],
                capture_output=True, text=True, timeout=timeout,
            )
            if r.returncode != 0:
                print(
                    f"[pvt_tuner] `ros2 control list_controllers` returned "
                    f"{r.returncode}: {r.stderr.strip()}",
                    file=sys.stderr, flush=True,
                )
                return []
        except (subprocess.TimeoutExpired, FileNotFoundError) as e:
            print(f"[pvt_tuner] subprocess failed: {e!r}",
                  file=sys.stderr, flush=True)
            return []
        out = []
        ansi = re.compile(r"\x1b\[[0-9;]*[a-zA-Z]")
        for raw in r.stdout.splitlines():
            line = ansi.sub("", raw).strip()
            m = re.match(r"^(\S+)\s+(\S+/\S+)\s+(active|inactive|unconfigured)", line)
            if m:
                out.append((m.group(1), m.group(2), m.group(3)))
        return out

    def _on_discovery_result(self, pvt, broadcaster_ns, total_controllers, source):
        """pyqtSignal handler — runs on the GUI thread."""
        print("[pvt_tuner] _on_discovery_result fired on GUI thread",
              file=sys.stderr, flush=True)
        self._apply_discovery(pvt, broadcaster_ns, total_controllers, source)
        self._discovery_done()

    def _apply_discovery(
        self,
        pvt: list[str],
        broadcaster_ns: str | None,
        total_controllers: int,
        source: str,
    ):
        domain = os.environ.get("ROS_DOMAIN_ID", "0")
        if pvt:
            msg = (
                f"found {len(pvt)} PVT controller(s) via {source} "
                f"on domain {domain}: {', '.join(pvt)}"
            )
            self._discovery_status.setStyleSheet(f"color: {C_GREEN};")
        else:
            msg = (
                f"no PVT controller found via {source} on domain {domain} "
                f"({total_controllers} controllers listed). Is the "
                f"controller_manager reachable? Click Refresh."
            )
            self._discovery_status.setStyleSheet(f"color: {C_YELLOW};")
        self._discovery_status.setText(msg)
        print(f"[pvt_tuner] discovery: {msg}", file=sys.stderr, flush=True)
        if broadcaster_ns:
            print(
                f"[pvt_tuner] telemetry broadcaster: {broadcaster_ns}",
                file=sys.stderr, flush=True,
            )

        # Update controller dropdown without losing the current selection.
        if pvt != self._controllers:
            self._controllers = pvt
            current = self._ctrl_combo.currentText()
            self._ctrl_combo.blockSignals(True)
            self._ctrl_combo.clear()
            self._ctrl_combo.addItems(pvt)
            if current in pvt:
                self._ctrl_combo.setCurrentText(current)
            elif pvt:
                self._ctrl_combo.setCurrentIndex(0)
            self._ctrl_combo.blockSignals(False)
            self._on_controller_changed(self._ctrl_combo.currentText())

        if broadcaster_ns and broadcaster_ns != self._broadcaster_ns:
            self._broadcaster_ns = broadcaster_ns
            self._setup_telemetry_subs(broadcaster_ns)
            self._broadcaster_label.setText(f"broadcaster: {broadcaster_ns}")

    # ─── CONTROLLER PARAMS ──────────────────────────────────

    def _on_controller_changed(self, name: str):
        name = name.strip()
        if not name:
            self._current_ctrl = None
            return
        self._current_ctrl = name
        # Subscribe to this controller's ~/setpoint so we can compute
        # following error. Tear down any subs from a previous controller.
        self._setup_controller_subs(name)
        # Kick off async load of joints/body_group/gain arrays; the joint
        # picker and gain panel populate from the loaded data.
        threading.Thread(
            target=self._load_controller_data, args=(name,), daemon=True
        ).start()
        # Auto-load the All Parameters tab too so it's never stale.
        self._refresh_all_params_async()

    def _load_controller_data(self, ctrl: str):
        """Load joints list, body_group, and per-joint gain arrays in one
        round-trip. Single get_parameters call to minimise discovery latency
        on cross-machine setups."""
        names = ["joints", "body_group", "drive_side_pd"] + [
            n for (n, *_) in PER_JOINT_FIELDS
        ]
        print(f"[pvt_tuner] get_parameters({ctrl}, {names})",
              file=sys.stderr, flush=True)
        values = self._rclpy_get_params(ctrl, names)
        print(f"[pvt_tuner] joints loaded: "
              f"{len(values.get('joints', []) or [])} entries",
              file=sys.stderr, flush=True)
        self._gains_loaded_sig.emit(ctrl, values)

    def _on_gains_loaded(self, ctrl: str, values: dict):
        """GUI-thread handler — populate the joint picker, scope banner, and
        gain widgets for the currently selected joint."""
        if self._current_ctrl != ctrl:
            return  # stale

        joints = values.get("joints") or []
        if not isinstance(joints, list) or not joints:
            self._scope_label.setText("scope: (no joints returned)")
            self._scope_label.setStyleSheet(f"color: {C_YELLOW};")
            return

        body_group = values.get("body_group") or "—"
        self._drive_side_pd = bool(values.get("drive_side_pd", True))
        self._body_group = str(body_group)
        # Scope banner: body_group + (best-effort) active count.
        size = BODY_GROUP_SIZES.get(self._body_group)
        if size is not None:
            self._scope_label.setText(
                f"scope: body_group='{self._body_group}'  "
                f"({size}/{len(joints)} joints active)"
            )
            self._scope_label.setStyleSheet(f"color: {C_GREEN};")
        else:
            self._scope_label.setText(
                f"scope: body_group='{self._body_group}'"
            )
            self._scope_label.setStyleSheet(f"color: {C_YELLOW};")
        self._mode_scope_label.setText(
            f"mode actions affect ALL active joints in body_group='"
            f"{self._body_group}'"
        )

        # Cache per-joint arrays for read-modify-write on Apply.
        self._gain_arrays = {}
        for name, *_ in PER_JOINT_FIELDS:
            v = values.get(name)
            if isinstance(v, list):
                self._gain_arrays[name] = list(v)
            else:
                self._gain_arrays[name] = []

        # Populate joint picker, preserving selection if possible.
        new_joints = list(joints)
        if new_joints != self._joints:
            current = self._joint_combo.currentText()
            self._joint_combo.blockSignals(True)
            self._joint_combo.clear()
            self._joint_combo.addItems(new_joints)
            if current in new_joints:
                self._joint_combo.setCurrentText(current)
            else:
                self._joint_combo.setCurrentIndex(0)
            self._joint_combo.blockSignals(False)
            self._joints = new_joints

        self._joint_idx = self._joint_combo.currentIndex()
        self._populate_gain_widgets()
        # Safety + telemetry depend on the joint index — refresh both.
        self._on_read_safety_limits()
        self._refresh_telemetry_labels()
        self._refresh_following_error()
        self._gains_read_btn.setEnabled(True)

    def _on_joint_changed(self, idx: int):
        if idx < 0:
            self._joint_idx = -1
            return
        self._joint_idx = idx
        # Re-show cached arrays at new index, no fresh fetch needed.
        self._populate_gain_widgets()
        # Refresh safety panel + telemetry labels from cached payloads.
        self._on_read_safety_limits()
        self._refresh_telemetry_labels()
        self._refresh_following_error()

    def _populate_gain_widgets(self):
        if self._joint_idx < 0:
            return
        j = self._joint_idx
        for name, *_, kind in PER_JOINT_FIELDS:
            arr = self._gain_arrays.get(name) or []
            if j >= len(arr):
                continue
            w = self._gain_widgets[name]
            v = arr[j]
            if kind == "bool":
                w.setChecked(bool(v))
            else:
                try:
                    w.setValue(float(v))
                except (TypeError, ValueError):
                    pass

    def _rclpy_get_params(self, ctrl: str, names: list[str], timeout: float = 5.0):
        """Read parameters from <ctrl> via rcl_interfaces/srv/GetParameters.

        Returns dict mapping name → Python value. Covers all the scalar +
        array types the controller exposes. Unset / unsupported types are
        omitted from the dict so callers can use .get() with sensible
        defaults."""
        client = self._node.create_client(GetParameters, f"{ctrl}/get_parameters")
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                print(f"[pvt_tuner] get_parameters: {ctrl}/get_parameters "
                      f"unavailable after {timeout}s", file=sys.stderr, flush=True)
                return {}
            req = GetParameters.Request()
            req.names = names
            future = client.call_async(req)
            deadline = time.time() + timeout
            while not future.done() and time.time() < deadline:
                time.sleep(0.02)
            if not future.done():
                print("[pvt_tuner] get_parameters: response timed out",
                      file=sys.stderr, flush=True)
                return {}
            resp = future.result()
            out: dict[str, object] = {}
            for name, val in zip(names, resp.values):
                t = val.type
                if t == ParameterType.PARAMETER_BOOL:
                    out[name] = bool(val.bool_value)
                elif t == ParameterType.PARAMETER_INTEGER:
                    out[name] = int(val.integer_value)
                elif t == ParameterType.PARAMETER_DOUBLE:
                    out[name] = float(val.double_value)
                elif t == ParameterType.PARAMETER_STRING:
                    out[name] = str(val.string_value)
                elif t == ParameterType.PARAMETER_DOUBLE_ARRAY:
                    out[name] = [float(x) for x in val.double_array_value]
                elif t == ParameterType.PARAMETER_BOOL_ARRAY:
                    out[name] = [bool(x) for x in val.bool_array_value]
                elif t == ParameterType.PARAMETER_INTEGER_ARRAY:
                    out[name] = [int(x) for x in val.integer_array_value]
                elif t == ParameterType.PARAMETER_STRING_ARRAY:
                    out[name] = [str(x) for x in val.string_array_value]
                # NOT_SET / other types are skipped — caller handles missing keys.
            return out
        finally:
            self._node.destroy_client(client)

    def _rclpy_set_params(
        self,
        ctrl: str,
        pairs: list[tuple[str, object]],
        timeout: float = 5.0,
    ):
        """Set parameters via rcl_interfaces/srv/SetParameters.

        Returns list of (name, ok, message). Handles scalars (bool, int,
        float, str) and arrays (list[float], list[bool], list[str])."""
        client = self._node.create_client(SetParameters, f"{ctrl}/set_parameters")
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                return [(n, False, "service unavailable") for n, _ in pairs]
            req = SetParameters.Request()
            params = []
            for name, value in pairs:
                p = ParameterMsg()
                p.name = name
                pv = ParameterValue()
                # bool is a subclass of int — check bool first.
                if isinstance(value, bool):
                    pv.type = ParameterType.PARAMETER_BOOL
                    pv.bool_value = value
                elif isinstance(value, int):
                    pv.type = ParameterType.PARAMETER_INTEGER
                    pv.integer_value = value
                elif isinstance(value, float):
                    pv.type = ParameterType.PARAMETER_DOUBLE
                    pv.double_value = value
                elif isinstance(value, str):
                    pv.type = ParameterType.PARAMETER_STRING
                    pv.string_value = value
                elif isinstance(value, list):
                    # Array — infer element type from the first element.
                    if not value:
                        return [(n, False, "empty array not supported")
                                for n, _ in pairs]
                    # bool before int (subclass).
                    if all(isinstance(x, bool) for x in value):
                        pv.type = ParameterType.PARAMETER_BOOL_ARRAY
                        pv.bool_array_value = [bool(x) for x in value]
                    elif all(isinstance(x, (int, float)) and
                             not isinstance(x, bool) for x in value):
                        pv.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                        pv.double_array_value = [float(x) for x in value]
                    elif all(isinstance(x, str) for x in value):
                        pv.type = ParameterType.PARAMETER_STRING_ARRAY
                        pv.string_array_value = [str(x) for x in value]
                    else:
                        return [(n, False, f"mixed-type array for {name}")
                                for n, _ in pairs]
                else:
                    return [(n, False, f"unsupported type {type(value)}")
                            for n, _ in pairs]
                p.value = pv
                params.append(p)
            req.parameters = params
            future = client.call_async(req)
            deadline = time.time() + timeout
            while not future.done() and time.time() < deadline:
                time.sleep(0.02)
            if not future.done():
                return [(n, False, "timeout") for n, _ in pairs]
            resp = future.result()
            return [
                (n, bool(r.successful), str(r.reason) if r.reason else "")
                for (n, _), r in zip(pairs, resp.results)
            ]
        finally:
            self._node.destroy_client(client)

    def _on_read_gains(self):
        ctrl = self._current_ctrl
        if not ctrl:
            self._gains_status.setText("no controller selected")
            self._gains_status.setStyleSheet(f"color: {C_YELLOW};")
            return
        self._gains_read_btn.setEnabled(False)
        threading.Thread(
            target=self._load_controller_data, args=(ctrl,), daemon=True
        ).start()

    def _on_apply_gains(self):
        ctrl = self._current_ctrl
        if not ctrl or self._joint_idx < 0:
            self._gains_status.setText("no controller / joint selected")
            self._gains_status.setStyleSheet(f"color: {C_YELLOW};")
            return
        if not self._gain_arrays:
            self._gains_status.setText("no cached arrays — click Read first")
            self._gains_status.setStyleSheet(f"color: {C_YELLOW};")
            return
        self._gains_apply_btn.setEnabled(False)

        # Read-modify-write on the cached arrays. Push the full array back
        # so the controller's normalize() does not broadcast a scalar.
        j = self._joint_idx
        targets: list[tuple[str, list]] = []
        for name, *_, kind in PER_JOINT_FIELDS:
            arr = self._gain_arrays.get(name)
            if not arr or j >= len(arr):
                continue
            w = self._gain_widgets[name]
            if kind == "bool":
                arr[j] = bool(w.isChecked())
            else:
                arr[j] = float(w.value())
            targets.append((name, list(arr)))

        def worker():
            print(f"[pvt_tuner] set_parameters({ctrl}, "
                  f"{[n for n,_ in targets]}) joint_idx={j}",
                  file=sys.stderr, flush=True)
            results = self._rclpy_set_params(ctrl, targets)
            print(f"[pvt_tuner] set results: {results}",
                  file=sys.stderr, flush=True)
            self._gains_applied_sig.emit(results)
            # Re-read so the cache reflects what the controller latched.
            self._load_controller_data(ctrl)

        threading.Thread(target=worker, daemon=True).start()

    def _on_gains_applied(self, results: list):
        self._gains_apply_btn.setEnabled(True)
        all_ok = True
        lines = []
        for name, ok, msg in results:
            if not ok:
                all_ok = False
                lines.append(f"{name} FAIL: {msg}")
            else:
                lines.append(name)
        if self._joint_idx >= 0 and self._joints:
            jn = self._joints[self._joint_idx]
        else:
            jn = "?"
        self._gains_status.setText(
            f"set [{jn}]: " + " | ".join(lines)
        )
        self._gains_status.setStyleSheet(
            f"color: {C_GREEN if all_ok else C_RED};"
        )

    # ─── SAFETY LIMITS PANEL ────────────────────────────────

    def _on_read_safety_limits(self):
        ctrl = self._current_ctrl
        if not ctrl or self._joint_idx < 0 or not self._joints:
            return
        joint = self._joints[self._joint_idx]
        self._safety_read_btn.setEnabled(False)
        # Build name lists: read both global and joint-override paths.
        keys = [k for k, *_ in SAFETY_LIMIT_FIELDS]
        global_names = [f"safety.global.{k}" for k in keys]
        joint_names = [f"safety.joints.{joint}.{k}" for k in keys]

        def worker():
            sup_global = self._rclpy_get_params(SUPERVISOR_NODE, global_names)
            sup_joint  = self._rclpy_get_params(SUPERVISOR_NODE, joint_names)
            ctrl_global = self._rclpy_get_params(ctrl, global_names)
            ctrl_joint  = self._rclpy_get_params(ctrl, joint_names)
            merged = {}
            for k in keys:
                merged[k] = (
                    sup_global.get(f"safety.global.{k}"),
                    sup_joint.get(f"safety.joints.{joint}.{k}"),
                    ctrl_global.get(f"safety.global.{k}"),
                    ctrl_joint.get(f"safety.joints.{joint}.{k}"),
                )
            self._safety_loaded_sig.emit(merged)

        threading.Thread(target=worker, daemon=True).start()

    def _on_safety_loaded(self, merged: dict):
        self._safety_read_btn.setEnabled(True)
        mismatches = []
        any_value = False
        for key, (sup_g, sup_j, ctrl_g, ctrl_j) in merged.items():
            # Resolve the displayed value: prefer joint override; fall back to
            # global. Prefer supervisor's value when both nodes have one.
            sup_resolved = sup_j if isinstance(sup_j, (int, float)) else sup_g
            ctrl_resolved = ctrl_j if isinstance(ctrl_j, (int, float)) else ctrl_g
            v = sup_resolved if isinstance(sup_resolved, (int, float)) else ctrl_resolved
            spin = self._safety_spins[key]
            origin = self._safety_origin_labels[key]
            if isinstance(v, (int, float)):
                any_value = True
                spin.setValue(float(v))
                # Origin tag: joint override beats global.
                if isinstance(sup_j, (int, float)) or isinstance(ctrl_j, (int, float)):
                    origin.setText("(override)")
                    origin.setStyleSheet(f"color: {C_GREEN};")
                else:
                    origin.setText("(global)")
                    origin.setStyleSheet(f"color: {C_SUBTEXT};")
            else:
                origin.setText("(missing)")
                origin.setStyleSheet(f"color: {C_YELLOW};")
            # Cross-check: if both supervisor and controller resolved values
            # disagree, surface as a mismatch (the supervisor would trip on a
            # threshold the controller doesn't enforce, or vice versa).
            if (isinstance(sup_resolved, (int, float)) and
                isinstance(ctrl_resolved, (int, float)) and
                abs(sup_resolved - ctrl_resolved) > 1e-9):
                mismatches.append(
                    f"{key}: sup={sup_resolved:.4g} ctrl={ctrl_resolved:.4g}"
                )
        if self._joint_idx >= 0 and self._joints:
            jn = self._joints[self._joint_idx]
        else:
            jn = "?"
        if not any_value:
            self._safety_status.setText(
                f"[{jn}] no safety.* params returned"
            )
            self._safety_status.setStyleSheet(f"color: {C_YELLOW};")
        elif mismatches:
            self._safety_status.setText(
                f"[{jn}] mismatch: " + " | ".join(mismatches)
            )
            self._safety_status.setStyleSheet(f"color: {C_YELLOW};")
        else:
            self._safety_status.setText(
                f"[{jn}] read OK — supervisor and controller agree"
            )
            self._safety_status.setStyleSheet(f"color: {C_GREEN};")

    def _on_apply_safety_limits(self):
        ctrl = self._current_ctrl
        if not ctrl or self._joint_idx < 0 or not self._joints:
            self._safety_status.setText("no controller / joint selected")
            self._safety_status.setStyleSheet(f"color: {C_YELLOW};")
            return
        joint = self._joints[self._joint_idx]
        self._safety_apply_btn.setEnabled(False)
        pairs = [
            (f"safety.joints.{joint}.{k}", float(self._safety_spins[k].value()))
            for k, *_ in SAFETY_LIMIT_FIELDS
        ]

        def worker():
            print(
                f"[pvt_tuner] safety apply -> supervisor + {ctrl}  joint={joint}: "
                f"{pairs}",
                file=sys.stderr, flush=True,
            )
            sup_res = self._rclpy_set_params(SUPERVISOR_NODE, pairs)
            ctrl_res = self._rclpy_set_params(ctrl, pairs)
            sup_by = {n: (ok, msg) for (n, ok, msg) in sup_res}
            ctrl_by = {n: (ok, msg) for (n, ok, msg) in ctrl_res}
            stitched = []
            for full, _ in pairs:
                # strip "safety.joints.<joint>." to recover the short key
                short = full.split(".")[-1]
                sup_ok, sup_msg = sup_by.get(full, (False, "no response"))
                ctrl_ok, ctrl_msg = ctrl_by.get(full, (False, "no response"))
                msg = ""
                if not sup_ok:
                    msg = f"sup: {sup_msg}"
                if not ctrl_ok:
                    msg = (msg + " | " if msg else "") + f"ctrl: {ctrl_msg}"
                stitched.append((short, sup_ok, ctrl_ok, msg))
            self._safety_applied_sig.emit(stitched)

        threading.Thread(target=worker, daemon=True).start()

    def _on_safety_applied(self, results: list):
        self._safety_apply_btn.setEnabled(True)
        all_ok = True
        fails = []
        for short, sup_ok, ctrl_ok, msg in results:
            if not (sup_ok and ctrl_ok):
                all_ok = False
                fails.append(f"{short} ({msg})")
        if self._joint_idx >= 0 and self._joints:
            jn = self._joints[self._joint_idx]
        else:
            jn = "?"
        if all_ok:
            self._safety_status.setText(
                f"[{jn}] override applied to supervisor + controller "
                f"({len(results)} fields)"
            )
            self._safety_status.setStyleSheet(f"color: {C_GREEN};")
            # Re-read so origin tags flip to "(override)" where appropriate.
            self._on_read_safety_limits()
        else:
            self._safety_status.setText(f"[{jn}] FAIL: " + " | ".join(fails))
            self._safety_status.setStyleSheet(f"color: {C_RED};")

    # ─── SERVICE CALLS ──────────────────────────────────────

    def _call_controller_trigger(self, leaf: str):
        ctrl = self._current_ctrl
        if not ctrl:
            self._mode_label.setText("last action: (no controller selected)")
            return
        srv = f"{ctrl}/{leaf}"
        self._async_trigger(srv, label=f"{leaf}")

    def _call_supervisor_trigger(self, leaf: str):
        srv = f"{SUPERVISOR_NODE}/{leaf}"
        self._async_trigger(srv, label=f"supervisor/{leaf}")

    def _async_trigger(self, service: str, label: str):
        """Fire a Trigger service call from a transient client. Reports the
        result via _service_result_sig so the GUI thread can render it."""

        def worker():
            client = self._node.create_client(Trigger, service)
            try:
                if not client.wait_for_service(timeout_sec=2.0):
                    self._service_result_sig.emit(
                        label, False, f"service {service} unavailable"
                    )
                    return
                future = client.call_async(Trigger.Request())
                deadline = time.time() + 5.0
                while not future.done() and time.time() < deadline:
                    time.sleep(0.02)
                if not future.done():
                    self._service_result_sig.emit(label, False, "timeout")
                    return
                resp = future.result()
                self._service_result_sig.emit(
                    label, bool(resp.success), resp.message or ""
                )
            finally:
                self._node.destroy_client(client)

        threading.Thread(target=worker, daemon=True).start()

    def _on_service_result(self, label: str, ok: bool, message: str):
        text = f"{label}: {'OK' if ok else 'FAIL'}"
        if message:
            text += f" — {message}"
        if label.startswith("supervisor/"):
            # Surface as a breach hint until the latched topic refreshes.
            self._breach_label.setText(text)
            self._breach_label.setStyleSheet(
                f"color: {C_GREEN if ok else C_RED};"
            )
        else:
            self._mode_label.setText(f"last action: {text}")
            self._mode_label.setStyleSheet(
                f"color: {C_GREEN if ok else C_RED};"
            )

    # ─── SAFETY SUBSCRIPTIONS (latched, supervisor-published) ───────

    def _setup_safety_subs(self):
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._node.create_subscription(
            Int8, SAFETY_ESTOP_TOPIC,
            lambda m: self._estop_state_sig.emit(int(m.data)), latched,
        )
        self._node.create_subscription(
            String, SAFETY_BREACH_TOPIC,
            lambda m: self._breach_sig.emit(str(m.data)), latched,
        )
        self._node.create_subscription(
            Float64MultiArray, SAFETY_KP_SCALE_TOPIC,
            self._on_kp_scale_msg, latched,
        )

    def _on_kp_scale_msg(self, msg):
        self._last_kp_scale = [float(x) for x in msg.data]
        if 0 <= self._joint_idx < len(self._last_kp_scale):
            self._kp_scale_sig.emit(self._last_kp_scale[self._joint_idx])

    def _on_estop_state(self, state: int):
        label, color = ESTOP_LABELS.get(state, (f"UNKNOWN ({state})", C_RED))
        self._estop_state_label.setText(label)
        self._estop_state_label.setStyleSheet(f"color: {color};")

    def _on_breach(self, reason: str):
        text = reason if reason else "—"
        self._breach_label.setText(text)
        color = C_SUBTEXT if reason in ("", "NONE", "none") else C_RED
        self._breach_label.setStyleSheet(f"color: {color};")

    # ─── TELEMETRY SUBSCRIPTIONS ────────────────────────────

    def _setup_telemetry_subs(self, ns: str):
        # Tear down any subs from a previous broadcaster.
        for sub in self._broadcaster_subs:
            self._node.destroy_subscription(sub)
        self._broadcaster_subs.clear()

        def sub(topic_leaf, on_msg):
            topic = f"{ns}/{topic_leaf}"
            s = self._node.create_subscription(
                Float64MultiArray, topic, on_msg, 10,
            )
            self._broadcaster_subs.append(s)

        sub("motor_temperature", self._on_motor_temp_msg)
        sub("drive_temperature", self._on_drive_temp_msg)
        sub("bus_voltage", self._on_bus_voltage_msg)
        sub("error_code", self._on_error_code_msg)

    def _on_motor_temp_msg(self, msg):
        self._last_motor_temp = [float(x) for x in msg.data]
        if 0 <= self._joint_idx < len(self._last_motor_temp):
            self._motor_temp_sig.emit(self._last_motor_temp[self._joint_idx])

    def _on_drive_temp_msg(self, msg):
        self._last_drive_temp = [float(x) for x in msg.data]
        if 0 <= self._joint_idx < len(self._last_drive_temp):
            self._drive_temp_sig.emit(self._last_drive_temp[self._joint_idx])

    def _on_bus_voltage_msg(self, msg):
        self._last_bus_voltage = [float(x) for x in msg.data]
        if 0 <= self._joint_idx < len(self._last_bus_voltage):
            self._voltage_sig.emit(self._last_bus_voltage[self._joint_idx])

    def _on_error_code_msg(self, msg):
        self._last_error_code = [float(x) for x in msg.data]
        if 0 <= self._joint_idx < len(self._last_error_code):
            self._error_code_sig.emit(self._last_error_code[self._joint_idx])

    def _refresh_telemetry_labels(self):
        """Re-emit cached MultiArray entries at the current joint index so
        labels update immediately on joint change (without waiting for the
        next 50 Hz broadcast tick)."""
        j = self._joint_idx
        if j < 0:
            return
        if self._last_motor_temp and j < len(self._last_motor_temp):
            self._motor_temp_sig.emit(self._last_motor_temp[j])
        if self._last_drive_temp and j < len(self._last_drive_temp):
            self._drive_temp_sig.emit(self._last_drive_temp[j])
        if self._last_bus_voltage and j < len(self._last_bus_voltage):
            self._voltage_sig.emit(self._last_bus_voltage[j])
        if self._last_error_code and j < len(self._last_error_code):
            self._error_code_sig.emit(self._last_error_code[j])
        if self._last_kp_scale and j < len(self._last_kp_scale):
            self._kp_scale_sig.emit(self._last_kp_scale[j])

    def _on_voltage(self, v: float):
        color = C_GREEN if BUS_V_MIN <= v <= BUS_V_MAX else C_RED
        self._voltage_label.setText(f"{v:7.2f} V")
        self._voltage_label.setStyleSheet(f"color: {color};")

    def _on_motor_temp(self, t: float):
        color = (
            C_RED if t >= MOTOR_TEMP_ERROR
            else C_YELLOW if t >= MOTOR_TEMP_WARN
            else C_GREEN
        )
        self._motor_temp_label.setText(f"{t:7.1f} °C")
        self._motor_temp_label.setStyleSheet(f"color: {color};")

    def _on_drive_temp(self, t: float):
        color = (
            C_RED if t >= DRIVE_TEMP_ERROR
            else C_YELLOW if t >= DRIVE_TEMP_WARN
            else C_GREEN
        )
        self._drive_temp_label.setText(f"{t:7.1f} °C")
        self._drive_temp_label.setStyleSheet(f"color: {color};")

    def _on_error_code(self, code: float):
        ic = int(code)
        if ic == 0:
            self._error_code_label.setText("0")
            self._error_code_label.setStyleSheet(f"color: {C_GREEN};")
        else:
            self._error_code_label.setText(f"0x{ic:04X} ({ic})")
            self._error_code_label.setStyleSheet(f"color: {C_RED};")

    def _on_kp_scale(self, s: float):
        color = C_GREEN if s >= 0.99 else C_YELLOW if s >= 0.5 else C_RED
        self._kp_scale_label.setText(f"{s:5.2f}")
        self._kp_scale_label.setStyleSheet(f"color: {color};")

    def _on_following_err(self, e: float):
        absv = abs(e)
        color = C_GREEN if absv < 0.05 else C_YELLOW if absv < 0.2 else C_RED
        self._foll_err_label.setText(f"{e:+8.4f} rad")
        self._foll_err_label.setStyleSheet(f"color: {color};")

    # ─── FOLLOWING-ERROR SOURCES ────────────────────────────

    def _setup_joint_states_sub(self):
        self._node.create_subscription(
            JointState, JOINT_STATES_TOPIC, self._on_joint_state, 50,
        )

    def _on_joint_state(self, msg):
        # Cache q_meas keyed by joint name; cheaper than rebuilding the lookup
        # on every joint-picker change.
        for name, pos in zip(msg.name, msg.position):
            self._q_meas[name] = float(pos)
        self._refresh_following_error()

    def _setup_controller_subs(self, ctrl: str):
        for sub in self._controller_subs:
            self._node.destroy_subscription(sub)
        self._controller_subs.clear()
        # ~/setpoint is the legacy streaming path. Action-driven goals don't
        # republish their interpolated target, so during a FollowJointTrajectory
        # the cached q_cmd will not reflect what the controller is sending —
        # callers see a stale following-error display (tooltip on the label).
        s = self._node.create_subscription(
            JointTrajectoryPoint, f"{ctrl}/setpoint",
            self._on_setpoint, 50,
        )
        self._controller_subs.append(s)

    def _on_setpoint(self, msg):
        # Setpoint positions are indexed by the controller's joints_ order —
        # same order as self._joints (we got both from the same controller).
        for j, pos in enumerate(msg.positions):
            if j < len(self._joints):
                self._q_cmd[self._joints[j]] = float(pos)
        self._refresh_following_error()

    def _refresh_following_error(self):
        if self._joint_idx < 0 or not self._joints:
            return
        name = self._joints[self._joint_idx]
        q_cmd = self._q_cmd.get(name)
        q_meas = self._q_meas.get(name)
        if q_cmd is None or q_meas is None:
            self._foll_err_label.setText("— (no data)")
            self._foll_err_label.setStyleSheet(f"color: {C_SUBTEXT};")
            return
        self._following_err_sig.emit(q_cmd - q_meas)

    # ─── ALL-PARAMETERS TAB ─────────────────────────────────

    _PARAMS_HIDDEN = frozenset({
        "use_sim_time",
        "qos_overrides",
        "start_type_description_service",
    })

    def _build_params_tab(self, root: QWidget):
        layout = QVBoxLayout(root)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        header = QHBoxLayout()
        self._params_tab_status = QLabel("(select a controller in the Tuner tab)")
        self._params_tab_status.setFont(QFont("monospace", 9))
        self._params_tab_status.setStyleSheet(f"color: {C_SUBTEXT};")
        header.addWidget(self._params_tab_status)
        header.addStretch()
        self._params_refresh_btn = QPushButton("Refresh")
        self._params_refresh_btn.setFixedWidth(90)
        self._params_refresh_btn.clicked.connect(self._refresh_all_params_async)
        header.addWidget(self._params_refresh_btn)
        layout.addLayout(header)

        # Scrollable grid: name | editor | Set
        self._params_grid_host = QWidget()
        self._params_grid = QGridLayout(self._params_grid_host)
        self._params_grid.setColumnStretch(1, 1)
        self._params_grid.setHorizontalSpacing(10)
        self._params_grid.setVerticalSpacing(4)

        scroll = QScrollArea()
        scroll.setWidget(self._params_grid_host)
        scroll.setWidgetResizable(True)
        layout.addWidget(scroll, 1)

        # name -> (editor, get_value_fn, set_btn)
        self._params_editors: dict[str, tuple] = {}

    def _clear_params_grid(self):
        while self._params_grid.count():
            item = self._params_grid.takeAt(0)
            w = item.widget()
            if w is not None:
                w.deleteLater()
        self._params_editors.clear()

    def _refresh_all_params_async(self):
        ctrl = self._current_ctrl
        if not ctrl:
            self._params_tab_status.setText("(select a controller in the Tuner tab)")
            self._params_tab_status.setStyleSheet(f"color: {C_SUBTEXT};")
            return
        self._params_refresh_btn.setEnabled(False)
        self._params_tab_status.setText(f"loading {ctrl} …")
        self._params_tab_status.setStyleSheet(f"color: {C_BLUE};")
        threading.Thread(
            target=self._all_params_worker, args=(ctrl,), daemon=True
        ).start()

    def _all_params_worker(self, ctrl: str):
        names = self._rclpy_list_params(ctrl)
        names = sorted(
            n for n in names
            if n not in self._PARAMS_HIDDEN and not n.startswith("qos_overrides.")
        )
        print(f"[pvt_tuner] list_parameters({ctrl}) -> {len(names)} names",
              file=sys.stderr, flush=True)
        if not names:
            self._all_params_sig.emit(ctrl, {})
            return
        values = self._rclpy_get_params(ctrl, names, timeout=10.0)
        self._all_params_sig.emit(ctrl, values)

    def _on_all_params_loaded(self, ctrl: str, values: dict):
        """GUI-thread handler — rebuild the grid."""
        self._params_refresh_btn.setEnabled(True)
        if self._current_ctrl != ctrl:
            return  # stale response
        self._clear_params_grid()
        if not values:
            self._params_tab_status.setText(
                f"no parameters returned for {ctrl}"
            )
            self._params_tab_status.setStyleSheet(f"color: {C_YELLOW};")
            return

        self._params_tab_status.setText(
            f"{ctrl}  ({len(values)} parameters)"
        )
        self._params_tab_status.setStyleSheet(f"color: {C_GREEN};")

        for row, (name, value) in enumerate(values.items()):
            label = QLabel(name)
            label.setFont(QFont("monospace", 10))
            self._params_grid.addWidget(label, row, 0)

            editor, get_fn = self._make_param_editor(value)
            self._params_grid.addWidget(editor, row, 1)

            set_btn = QPushButton("Set")
            set_btn.setFixedWidth(60)
            set_btn.setStyleSheet(
                f"background-color: {C_SURFACE0}; color: {C_GREEN}; "
                f"border: 1px solid {C_GREEN};"
            )
            set_btn.clicked.connect(
                lambda _checked=False, n=name, g=get_fn: self._set_one_param(n, g())
            )
            self._params_grid.addWidget(set_btn, row, 2)

            self._params_editors[name] = (editor, get_fn, set_btn)

    @staticmethod
    def _make_param_editor(value):
        """Return (widget, get_value_callable) appropriate for ``value`` type."""
        if isinstance(value, bool):
            w = QCheckBox()
            w.setChecked(value)
            return w, w.isChecked
        if isinstance(value, int):
            w = QSpinBox()
            w.setRange(-2_147_483_648, 2_147_483_647)
            w.setValue(value)
            return w, w.value
        if isinstance(value, float):
            w = QDoubleSpinBox()
            w.setDecimals(6)
            w.setRange(-1e9, 1e9)
            w.setSingleStep(0.01)
            w.setValue(value)
            return w, w.value
        if isinstance(value, str):
            w = QLineEdit(value)
            return w, w.text
        # Arrays etc.: read-only repr. The per-joint widget is the editor for
        # tuning arrays — this tab is a visibility surface.
        w = QLineEdit(repr(value))
        w.setReadOnly(True)
        w.setStyleSheet(f"color: {C_SUBTEXT};")
        return w, lambda: None

    def _set_one_param(self, name: str, value):
        ctrl = self._current_ctrl
        if not ctrl or value is None:
            return
        editor_tuple = self._params_editors.get(name)
        if editor_tuple is not None:
            editor_tuple[2].setEnabled(False)
        threading.Thread(
            target=self._set_one_worker, args=(ctrl, name, value), daemon=True
        ).start()

    def _set_one_worker(self, ctrl, name, value):
        print(f"[pvt_tuner] set_one {name}={value!r}",
              file=sys.stderr, flush=True)
        results = self._rclpy_set_params(ctrl, [(name, value)])
        if results:
            n, ok, msg = results[0]
            self._param_set_sig.emit(n, ok, msg)
        else:
            self._param_set_sig.emit(name, False, "no response")

    def _on_param_set(self, name: str, ok: bool, msg: str):
        editor_tuple = self._params_editors.get(name)
        if editor_tuple is None:
            return
        editor, _, set_btn = editor_tuple
        set_btn.setEnabled(True)
        color = C_GREEN if ok else C_RED
        set_btn.setStyleSheet(
            f"background-color: {C_SURFACE0}; color: {color}; "
            f"border: 1px solid {color};"
        )
        self._params_tab_status.setText(
            f"{name}: {'OK' if ok else 'FAIL'}" + (f" — {msg}" if msg else "")
        )
        self._params_tab_status.setStyleSheet(f"color: {color};")

    def _rclpy_list_params(self, ctrl: str, timeout: float = 5.0):
        client = self._node.create_client(ListParameters, f"{ctrl}/list_parameters")
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                return []
            req = ListParameters.Request()
            req.depth = 0  # all
            future = client.call_async(req)
            deadline = time.time() + timeout
            while not future.done() and time.time() < deadline:
                time.sleep(0.02)
            if not future.done():
                return []
            resp = future.result()
            return list(resp.result.names)
        finally:
            self._node.destroy_client(client)


# ───────────────────────────────────────────────────────────
# main
# ───────────────────────────────────────────────────────────


def main():
    rclpy.init(args=sys.argv)
    node = Node("pvt_tuner_gui")

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    app = QApplication(sys.argv)
    win = PvtTunerWindow(node)
    win.show()
    exit_code = app.exec_()

    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
