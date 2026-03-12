"""Joint Calibration Window — sweep-based offset calibration with controller
listing and embedded 3D robot viewer."""

import math
import os
import re
import subprocess
import threading
import time
from datetime import datetime

import yaml

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QFont
from PyQt5.QtWidgets import (
    QComboBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSplitter,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from ethercat_tools.joint_calibration.apply_offsets import apply_offsets, find_ethercat_dir
from ethercat_tools.joint_calibration.reverse_direction import reverse_direction, find_ethercat_dirs

# ---------------------------------------------------------------------------
# Dynamic actuator discovery — parses EtherCAT YAML slave configs and URDF
# to build actuator definitions at runtime for ALL joints, not just left arm.
# ---------------------------------------------------------------------------


def _discover_actuators_from_configs():
    """Scan arm_real_bringup/config/ethercat/ YAML files for actuator metadata.

    Each file has structured comment headers:
      Line 2: # ICube EtherCAT slave config — {joint_name}
      Line 4: # Bus position: {N}
      Line 5: # Direction: {+1|-1}
    And rpdo position channel with factor (sign = direction) and offset.

    Returns list of actuator dicts (without URDF limits — those are added later).
    """
    actuators = []

    # Find ethercat config directory relative to this source file
    this_dir = os.path.dirname(os.path.abspath(__file__))
    # Walk up: joint_calibration/ -> ethercat_tools/ -> ethercat_tools/ -> tools/ -> src/
    ws_src = os.path.abspath(os.path.join(this_dir, "..", "..", "..", "..", ".."))
    ethercat_dir = os.path.join(
        ws_src, "src", "bringup", "arm_real_bringup", "config", "ethercat"
    )

    # Also try installed path via ament_index
    if not os.path.isdir(ethercat_dir):
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_dir = get_package_share_directory("arm_real_bringup")
            ethercat_dir = os.path.join(pkg_dir, "config", "ethercat")
        except Exception:
            pass

    if not os.path.isdir(ethercat_dir):
        return actuators

    for fname in sorted(os.listdir(ethercat_dir)):
        if not fname.endswith(".yaml"):
            continue

        fpath = os.path.join(ethercat_dir, fname)
        joint_name = None
        bus_pos = None
        direction = 1

        # Extract motor type from filename (e.g. "left_shoulder_pitch_X6.yaml" -> "X6")
        stem = fname.replace(".yaml", "")
        motor = "?"
        for suffix in ("X4", "X6", "X8"):
            if stem.endswith(f"_{suffix}"):
                motor = suffix
                break

        # Parse comment headers and YAML data
        try:
            with open(fpath, "r") as f:
                raw_text = f.read()

            # Extract joint name from comment: "# ICube EtherCAT slave config — {name}"
            m = re.search(r"#\s*ICube EtherCAT slave config\s*[—–-]\s*(\S+)", raw_text)
            if m:
                joint_name = m.group(1)

            # Extract bus position from comment: "# Bus position: {N}"
            m = re.search(r"#\s*Bus position:\s*(\d+)", raw_text)
            if m:
                bus_pos = int(m.group(1))

            # Extract direction from comment: "# Direction: {+1|-1}"
            m = re.search(r"#\s*Direction:\s*([+-]?\d+)", raw_text)
            if m:
                direction = int(m.group(1))

            # If no joint name from comment, infer from filename + URDF convention
            # Arm joints have motor suffix: left_shoulder_pitch_X6 -> left_shoulder_pitch_joint_X6
            # Leg/waist joints don't: left_hip_pitch_X8 -> left_hip_pitch_joint
            if joint_name is None:
                base = stem
                for suffix in ("_X4", "_X6", "_X8"):
                    if base.endswith(suffix):
                        base = base[: -len(suffix)]
                        break
                # Check if this is an arm joint (has motor suffix in URDF name)
                if "shoulder" in base or "elbow" in base or "wrist" in base:
                    joint_name = f"{base}_joint_{motor}"
                else:
                    joint_name = f"{base}_joint"

        except Exception:
            continue

        if joint_name is None:
            continue

        actuators.append({
            "joint": joint_name,
            "motor": motor,
            "slave": bus_pos if bus_pos is not None else len(actuators),
            "direction": direction,
            "lower": -math.pi,   # Default — will be overridden by URDF
            "upper": math.pi,    # Default — will be overridden by URDF
            "config": fname,
        })

    # Sort by bus position
    actuators.sort(key=lambda a: a["slave"])
    return actuators


def _apply_urdf_limits(actuators, ros_node):
    """Get URDF from /robot_description and apply joint limits to actuator dicts."""
    urdf_string = _get_urdf_string_for_limits(ros_node)
    if not urdf_string:
        return

    try:
        from urdf_parser_py.urdf import URDF
        urdf = URDF.from_xml_string(urdf_string)
    except Exception:
        return

    # Build joint limits map from URDF
    urdf_limits = {}
    for joint in urdf.joints:
        if joint.type in ("revolute", "prismatic") and joint.limit:
            urdf_limits[joint.name] = {
                "lower": joint.limit.lower if joint.limit.lower is not None else -math.pi,
                "upper": joint.limit.upper if joint.limit.upper is not None else math.pi,
            }

    # Apply to actuators
    for act in actuators:
        if act["joint"] in urdf_limits:
            lim = urdf_limits[act["joint"]]
            act["lower"] = lim["lower"]
            act["upper"] = lim["upper"]


def _get_urdf_string_for_limits(ros_node):
    """Get URDF string from /robot_description ROS parameter."""
    if ros_node is None:
        return None

    try:
        from rcl_interfaces.srv import GetParameters
        from rcl_interfaces.msg import ParameterType
        client = ros_node.create_client(
            GetParameters, "/robot_state_publisher/get_parameters"
        )
        if not client.wait_for_service(timeout_sec=2.0):
            return None

        request = GetParameters.Request()
        request.names = ["robot_description"]
        future = client.call_async(request)

        import rclpy
        rclpy.spin_until_future_complete(ros_node, future, timeout_sec=3.0)

        if future.result() is not None:
            values = future.result().values
            if values and values[0].type == ParameterType.PARAMETER_STRING:
                return values[0].string_value
    except Exception:
        pass

    # Fallback: try /robot_description topic
    try:
        from std_msgs.msg import String
        import rclpy

        urdf_holder = [None]

        def _cb(msg):
            urdf_holder[0] = msg.data

        sub = ros_node.create_subscription(String, "/robot_description", _cb, 1)
        for _ in range(30):
            rclpy.spin_once(ros_node, timeout_sec=0.1)
            if urdf_holder[0]:
                break
        ros_node.destroy_subscription(sub)
        return urdf_holder[0]
    except Exception:
        pass

    return None

# Unit conversion (raw ±65535 = ±180° = ±π rad)
RAW_PER_PI = 65535

# Catppuccin Mocha palette
_BG = "#1e1e2e"
_BG2 = "#313244"
_BG3 = "#45475a"
_FG = "#cdd6f4"
_SUBTEXT = "#6c7086"
_GREEN = "#a6e3a1"
_YELLOW = "#f9e2af"
_BLUE = "#89b4fa"
_RED = "#f38ba8"
_PEACH = "#fab387"

STYLESHEET = f"""
QMainWindow {{ background-color: {_BG}; color: {_FG}; }}
QWidget {{ background-color: {_BG}; color: {_FG}; }}
QLabel {{ color: {_FG}; background: transparent; }}
QFrame {{ background-color: {_BG2}; border-radius: 8px; }}
QTableWidget {{ background-color: {_BG}; color: {_FG}; gridline-color: {_BG3};
               border: none; font-family: monospace; font-size: 12px; }}
QTableWidget::item {{ padding: 4px 8px; }}
QTableWidget QHeaderView::section {{ background-color: {_BG2}; color: {_FG};
    border: 1px solid {_BG3}; padding: 6px; font-weight: bold; font-size: 12px; }}
QTabWidget::pane {{ border: 1px solid {_BG3}; background-color: {_BG};
                   border-radius: 4px; }}
QTabBar::tab {{ background-color: {_BG2}; color: {_FG}; padding: 8px 16px;
              margin-right: 2px; border-top-left-radius: 4px;
              border-top-right-radius: 4px; }}
QTabBar::tab:selected {{ background-color: {_BG3}; color: {_FG}; }}
QTextEdit {{ background-color: {_BG}; color: {_GREEN}; border: none;
           font-family: monospace; font-size: 11px; }}
QPushButton {{ background-color: {_BG2}; color: {_FG}; border: 1px solid {_BG3};
             border-radius: 6px; padding: 8px 16px; font-size: 12px;
             font-weight: bold; }}
QPushButton:hover {{ background-color: {_BG3}; }}
QPushButton:pressed {{ background-color: #585b70; }}
QComboBox {{ background-color: {_BG2}; color: {_FG}; border: 1px solid {_BG3};
           border-radius: 6px; padding: 6px 12px; font-size: 12px; }}
QComboBox::drop-down {{ border: none; }}
QComboBox QAbstractItemView {{ background-color: {_BG2}; color: {_FG};
    selection-background-color: {_BG3}; border: 1px solid {_BG3}; }}
QSplitter::handle {{ background-color: {_BG3}; width: 3px; }}
"""


def rad_to_raw(rad):
    return int(rad * RAW_PER_PI / math.pi)


def raw_to_deg(raw):
    return raw * 180.0 / RAW_PER_PI


def _compute_offset_from_sweep(act, real_limits):
    """Compute offset from observed encoder sweep and URDF limits.

    Returns (offset_raw, offset_tpdo_rad, offset_rpdo_raw, coverage_pct)
    or (None, None, None, 0.0) if no sweep data.
    """
    rl = real_limits[act["joint"]]
    if rl["min"] is None or rl["max"] is None:
        return None, None, None, 0.0

    real_center_raw = (rl["min"] + rl["max"]) / 2.0
    urdf_center_rad = (act["lower"] + act["upper"]) / 2.0
    urdf_center_raw = rad_to_raw(urdf_center_rad)
    offset_raw = int(round(real_center_raw - urdf_center_raw))

    offset_rad = offset_raw * math.pi / RAW_PER_PI
    offset_tpdo_rad = -offset_rad
    direction = act.get("direction", 1)
    offset_rpdo_raw = direction * offset_raw

    urdf_range_raw = rad_to_raw(act["upper"] - act["lower"])
    real_range_raw = rl["max"] - rl["min"]
    coverage = (real_range_raw / urdf_range_raw * 100.0) if urdf_range_raw > 0 else 0.0

    return offset_raw, offset_tpdo_rad, offset_rpdo_raw, min(coverage, 100.0)


# ---------------------------------------------------------------------------
# Controller query (subprocess, same pattern as joint_state_tui.py)
# ---------------------------------------------------------------------------

def _query_controllers():
    """Parse output of ``ros2 control list_controllers``."""
    controllers = []
    try:
        result = subprocess.run(
            ["ros2", "control", "list_controllers"],
            capture_output=True, text=True, timeout=5,
        )
        if result.returncode == 0:
            for line in result.stdout.strip().split("\n"):
                line = line.strip()
                if not line:
                    continue
                parts = line.split()
                if len(parts) >= 3:
                    name = parts[0]
                    brackets = re.findall(r"\[([^\]]+)\]", line)
                    if len(brackets) >= 2:
                        controllers.append(
                            {"name": name, "type": brackets[0], "state": brackets[1]}
                        )
                    elif len(brackets) == 1:
                        controllers.append(
                            {"name": name, "type": brackets[0], "state": "?"}
                        )
    except Exception:
        pass
    return controllers


def _query_controller_joints(controller_name):
    """Get joints for a specific controller via ``ros2 control list_controllers -v``.

    The verbose output lists interfaces per controller. We parse the
    ``cmd_iface`` lines (e.g. ``left_shoulder_pitch_joint_X6/position [claimed]``)
    to extract joint names.  Falls back to list_hardware_interfaces if needed.
    """
    joints = []
    seen = set()
    try:
        result = subprocess.run(
            ["ros2", "control", "list_controllers", "-v"],
            capture_output=True, text=True, timeout=5,
        )
        if result.returncode == 0:
            in_target = False
            for line in result.stdout.split("\n"):
                stripped = line.strip()
                # Controller header line: "controller_name [type] [state]"
                if stripped and not stripped.startswith("cmd") and not stripped.startswith("state"):
                    parts = stripped.split()
                    if parts and "[" in stripped:
                        in_target = (parts[0] == controller_name)
                        continue
                # Interface lines when inside target controller
                if in_target and ("cmd" in stripped or "state" in stripped):
                    # Lines like: cmd_iface: joint_name/position [claimed]
                    # or: state_iface: joint_name/position [active]
                    for token in stripped.split():
                        if "/" in token and "[" not in token:
                            joint_name = token.split("/")[0]
                            if joint_name not in seen:
                                seen.add(joint_name)
                                joints.append(joint_name)
            if joints:
                return joints
    except Exception:
        pass

    # Fallback: parse hardware interfaces for claimed joints
    try:
        result = subprocess.run(
            ["ros2", "control", "list_hardware_interfaces"],
            capture_output=True, text=True, timeout=5,
        )
        if result.returncode == 0:
            for line in result.stdout.strip().split("\n"):
                line = line.strip()
                if not line or "[claimed]" not in line.lower():
                    continue
                iface_name = line.split("[")[0].strip()
                if "/" in iface_name:
                    joint_name = iface_name.split("/")[0]
                    if joint_name not in seen:
                        seen.add(joint_name)
                        joints.append(joint_name)
    except Exception:
        pass
    return joints


# ═══════════════════════════════════════════════════════════════════════════
# Main Window
# ═══════════════════════════════════════════════════════════════════════════

class CalibrationWindow(QMainWindow):
    """Joint calibration window with controller panel and 3D robot viewer."""

    # Column indices for calibration table
    COL_NAME = 0
    COL_MOTOR = 1
    COL_URDF_LIMITS = 2
    COL_REAL_LIMITS = 3
    COL_CURRENT_DEG = 4
    COL_CURRENT_RAW = 5
    COL_OFFSET = 6
    COL_STATUS = 7
    COL_ACTIONS = 8

    def __init__(self, ros_node):
        super().__init__()
        self._ros_node = ros_node
        self._start_time = time.monotonic()
        self._joint_state_data = {}

        # Discover actuators dynamically from EtherCAT configs + URDF
        self._actuators = _discover_actuators_from_configs()
        _apply_urdf_limits(self._actuators, ros_node)
        self._actuator_map = {act["joint"]: act for act in self._actuators}

        if self._actuators:
            ros_node.get_logger().info(
                f"Discovered {len(self._actuators)} actuators from EtherCAT configs"
            )
        else:
            ros_node.get_logger().warn(
                "No EtherCAT configs found — calibration features limited"
            )

        # Calibration state (for discovered actuators)
        self._current_raw = {act["joint"]: 0 for act in self._actuators}
        self._offsets = {act["joint"]: None for act in self._actuators}
        self._real_limits = {
            act["joint"]: {"min": None, "max": None} for act in self._actuators
        }

        # Mode detection (SIM or REAL)
        self._mode = "SIM"
        self._mode_locked = False
        self._ethercat_subscribed = False
        self._safety_ok = None
        self._diag_data = {}

        # Controller data (populated by background thread)
        self._controllers = []
        self._controller_lock = threading.Lock()
        self._running = True

        # Visible joints — list of joint name strings.
        # When "All Joints" is selected, populated from /joint_states.
        # When a controller is selected, populated from that controller's joints.
        self._visible_joints = []
        self._all_discovered_joints = []  # From /joint_states (preserves order)
        self._joints_discovered = False

        self._build_ui()
        self._setup_table()

        # 10 Hz update timer
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(100)

        # Start background controller refresh thread
        self._ctrl_thread = threading.Thread(
            target=self._controller_refresh_loop, daemon=True
        )
        self._ctrl_thread.start()

        self._update_mode_display()
        self._log("Calibration tool started — sweep-based offset calculation.")
        self._log("Procedure:")
        self._log("  1. Sweep each joint through its full range of motion")
        self._log("  2. Status shows 'Ready' (green) when coverage is sufficient")
        self._log("  3. Click 'Capture Selected' to lock in the computed offset")
        self._log("  4. Export YAML when all joints are captured")

    # -- UI construction (programmatic, no .ui file) -------------------------

    def _build_ui(self):
        self.setWindowTitle("Joint Calibration")
        self.resize(1500, 800)
        self.setStyleSheet(STYLESHEET)

        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)
        root_layout.setSpacing(8)
        root_layout.setContentsMargins(12, 12, 12, 12)

        # ── Status bar ──
        self._status_frame = QFrame()
        self._status_frame.setMinimumHeight(60)
        self._status_frame.setMaximumHeight(70)
        status_layout = QHBoxLayout(self._status_frame)
        status_layout.setSpacing(24)
        status_layout.setContentsMargins(16, 0, 16, 0)

        self._status_indicator = QLabel("JOINT CALIBRATION")
        self._status_indicator.setFont(QFont("", 14, QFont.Bold))
        self._status_indicator.setStyleSheet(f"color: {_GREEN}; font-size: 14px; font-weight: bold;")
        status_layout.addWidget(self._status_indicator)

        # Captured counter
        cap_vbox = QVBoxLayout()
        cap_vbox.addWidget(self._make_label("Captured", _SUBTEXT, 10))
        self._captured_value = QLabel("0 / 6")
        self._captured_value.setStyleSheet("font-size: 13px; font-weight: bold; font-family: monospace;")
        cap_vbox.addWidget(self._captured_value)
        status_layout.addLayout(cap_vbox)

        # Duration
        dur_vbox = QVBoxLayout()
        dur_vbox.addWidget(self._make_label("Duration", _SUBTEXT, 10))
        self._duration_value = QLabel("00:00:00")
        self._duration_value.setStyleSheet("font-size: 13px; font-weight: bold; font-family: monospace;")
        dur_vbox.addWidget(self._duration_value)
        status_layout.addLayout(dur_vbox)

        # Mode
        mode_vbox = QVBoxLayout()
        mode_vbox.addWidget(self._make_label("Mode", _SUBTEXT, 10))
        self._mode_value = QLabel("SIM")
        self._mode_value.setStyleSheet(f"font-size: 13px; font-weight: bold; color: {_BLUE};")
        mode_vbox.addWidget(self._mode_value)
        status_layout.addLayout(mode_vbox)

        status_layout.addStretch()
        root_layout.addWidget(self._status_frame)

        # ── Main splitter (left panel | right 3D viewer) ──
        self._splitter = QSplitter(Qt.Horizontal)

        # LEFT PANEL
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setSpacing(8)
        left_layout.setContentsMargins(0, 0, 0, 0)

        # Controller dropdown
        ctrl_frame = QFrame()
        ctrl_layout = QVBoxLayout(ctrl_frame)
        ctrl_layout.setContentsMargins(12, 8, 12, 8)
        ctrl_layout.setSpacing(4)

        ctrl_header = QLabel("Controller")
        ctrl_header.setStyleSheet(f"color: {_SUBTEXT}; font-size: 10px; font-weight: bold;")
        ctrl_layout.addWidget(ctrl_header)

        self._ctrl_combo = QComboBox()
        self._ctrl_combo.addItem("All Joints")
        self._ctrl_combo.currentIndexChanged.connect(self._on_controller_changed)
        ctrl_layout.addWidget(self._ctrl_combo)

        ctrl_info_row = QHBoxLayout()
        self._ctrl_type_label = QLabel("Type: ---")
        self._ctrl_type_label.setStyleSheet(f"color: {_SUBTEXT}; font-size: 11px;")
        self._ctrl_state_label = QLabel("State: ---")
        self._ctrl_state_label.setStyleSheet(f"color: {_SUBTEXT}; font-size: 11px;")
        ctrl_info_row.addWidget(self._ctrl_type_label)
        ctrl_info_row.addWidget(self._ctrl_state_label)
        ctrl_info_row.addStretch()
        ctrl_layout.addLayout(ctrl_info_row)

        left_layout.addWidget(ctrl_frame)

        # Calibration table
        self._cal_table = QTableWidget()
        self._cal_table.setColumnCount(9)
        self._cal_table.setHorizontalHeaderLabels([
            "Joint Name", "Motor", "URDF Limits (deg)", "Real Limits (deg)",
            "Current (deg)", "Current (raw)", "Offset", "Status", "Actions",
        ])
        self._cal_table.setAlternatingRowColors(True)
        self._cal_table.setSelectionMode(QTableWidget.SingleSelection)
        self._cal_table.setSelectionBehavior(QTableWidget.SelectRows)
        self._cal_table.verticalHeader().setVisible(False)
        self._cal_table.horizontalHeader().setStretchLastSection(True)
        self._cal_table.setMinimumHeight(220)
        left_layout.addWidget(self._cal_table)

        # Control buttons
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(8)

        self._capture_sel_btn = QPushButton("Capture Selected")
        self._capture_sel_btn.setStyleSheet(f"color: {_BLUE}; border: 1px solid {_BLUE};")
        self._capture_sel_btn.clicked.connect(self._capture_selected)
        btn_layout.addWidget(self._capture_sel_btn)

        self._capture_all_btn = QPushButton("Capture All")
        self._capture_all_btn.clicked.connect(self._capture_all)
        btn_layout.addWidget(self._capture_all_btn)

        self._clear_btn = QPushButton("Clear Offsets")
        self._clear_btn.clicked.connect(self._clear_all)
        btn_layout.addWidget(self._clear_btn)

        self._reset_btn = QPushButton("Reset Limits")
        self._reset_btn.clicked.connect(self._reset_limits)
        btn_layout.addWidget(self._reset_btn)

        btn_layout.addStretch()

        self._export_btn = QPushButton("Export YAML")
        self._export_btn.setStyleSheet(f"color: {_GREEN}; border: 1px solid {_GREEN};")
        self._export_btn.clicked.connect(self._export_yaml)
        btn_layout.addWidget(self._export_btn)

        left_layout.addLayout(btn_layout)

        # Diagnostics tabs
        self._diag_tabs = QTabWidget()

        cal_log_tab = QWidget()
        cal_log_layout = QVBoxLayout(cal_log_tab)
        self._cal_log = QTextEdit()
        self._cal_log.setReadOnly(True)
        self._cal_log.setPlaceholderText("Calibration events will appear here...")
        cal_log_layout.addWidget(self._cal_log)
        self._diag_tabs.addTab(cal_log_tab, "Calibration Log")

        live_tab = QWidget()
        live_layout = QVBoxLayout(live_tab)
        self._live_text = QTextEdit()
        self._live_text.setReadOnly(True)
        self._live_text.setStyleSheet(f"font-family: monospace; font-size: 11px; color: {_BLUE};")
        self._live_text.setPlaceholderText("Select a joint row to see live raw values...")
        live_layout.addWidget(self._live_text)
        self._diag_tabs.addTab(live_tab, "Live Values")

        left_layout.addWidget(self._diag_tabs)

        self._splitter.addWidget(left_widget)

        # RIGHT PANEL — 3D robot viewer
        try:
            from ethercat_tools.joint_calibration.robot_viewer import RobotViewer
            self._robot_viewer = RobotViewer(self._ros_node)
            self._splitter.addWidget(self._robot_viewer)
        except Exception as e:
            fallback = QLabel(
                f"3D viewer unavailable:\n{e}\n\n"
                "Install: pip install pyvista pyvistaqt vtk"
            )
            fallback.setAlignment(Qt.AlignCenter)
            fallback.setStyleSheet(f"color: {_SUBTEXT}; font-size: 12px;")
            fallback.setWordWrap(True)
            self._robot_viewer = None
            self._splitter.addWidget(fallback)

        # Splitter proportions: 60% left, 40% right
        self._splitter.setSizes([900, 600])
        root_layout.addWidget(self._splitter)

    @staticmethod
    def _make_label(text, color, size):
        lbl = QLabel(text)
        lbl.setStyleSheet(f"color: {color}; font-size: {size}px;")
        return lbl

    # -- Table setup ---------------------------------------------------------

    def _setup_table(self):
        self._rebuild_table()

    def _rebuild_table(self):
        """Rebuild calibration table for currently visible joints.

        Known actuators (in ACTUATOR_MAP) get full calibration columns.
        Unknown joints show position/velocity/effort only.
        """
        table = self._cal_table
        table.setRowCount(len(self._visible_joints))

        for row, joint_name in enumerate(self._visible_joints):
            act = self._actuator_map.get(joint_name)
            self._set_cell(table, row, self.COL_NAME, joint_name)

            if act:
                self._set_cell(table, row, self.COL_MOTOR, act["motor"])
                lo_deg = math.degrees(act["lower"])
                hi_deg = math.degrees(act["upper"])
                self._set_cell(table, row, self.COL_URDF_LIMITS,
                               f"{lo_deg:+.0f} .. {hi_deg:+.0f}", _SUBTEXT)

                # Per-row action buttons
                btn_widget = QWidget()
                btn_lay = QHBoxLayout(btn_widget)
                btn_lay.setContentsMargins(2, 2, 2, 2)
                btn_lay.setSpacing(4)

                write_btn = QPushButton("Write")
                write_btn.setFixedSize(55, 24)
                write_btn.setStyleSheet(
                    f"font-size: 10px; padding: 2px 4px; color: {_GREEN}; border: 1px solid {_GREEN};"
                )
                write_btn.clicked.connect(
                    lambda _, j=joint_name: self._write_offset_for_joint(j)
                )

                rev_btn = QPushButton("Rev")
                rev_btn.setFixedSize(45, 24)
                rev_btn.setStyleSheet(
                    f"font-size: 10px; padding: 2px 4px; color: {_PEACH}; border: 1px solid {_PEACH};"
                )
                rev_btn.clicked.connect(
                    lambda _, j=joint_name: self._reverse_direction_for_joint(j)
                )

                btn_lay.addWidget(write_btn)
                btn_lay.addWidget(rev_btn)
                table.setCellWidget(row, self.COL_ACTIONS, btn_widget)
            else:
                self._set_cell(table, row, self.COL_MOTOR, "---", _SUBTEXT)
                self._set_cell(table, row, self.COL_URDF_LIMITS, "---", _SUBTEXT)

            for col in (self.COL_REAL_LIMITS, self.COL_CURRENT_DEG,
                        self.COL_CURRENT_RAW, self.COL_OFFSET, self.COL_STATUS):
                self._set_cell(table, row, col, "---")

        table.resizeColumnsToContents()
        table.setColumnWidth(self.COL_NAME, 260)
        table.setColumnWidth(self.COL_ACTIONS, 130)

    # -- Mode detection ------------------------------------------------------

    def _detect_mode(self):
        if self._mode_locked:
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
            self._mode = "SIM"
            self._update_mode_display()

    def _subscribe_ethercat_topics(self):
        if self._ethercat_subscribed:
            return
        self._ethercat_subscribed = True
        try:
            from std_msgs.msg import Bool
            self._ros_node.create_subscription(
                Bool, "/safety/status", self._safety_callback, 10
            )
        except Exception:
            pass
        try:
            from diagnostic_msgs.msg import DiagnosticArray
            self._ros_node.create_subscription(
                DiagnosticArray, "/diagnostics", self._diagnostics_callback, 10
            )
        except Exception:
            pass

    def _update_mode_display(self):
        colors = {"SIM": _BLUE, "REAL": _GREEN}
        color = colors.get(self._mode, _FG)
        self._mode_value.setText(self._mode)
        self._mode_value.setStyleSheet(f"font-size: 13px; font-weight: bold; color: {color};")

    # -- Controller management -----------------------------------------------

    def _controller_refresh_loop(self):
        """Background thread: polls controllers every 5 seconds."""
        while self._running:
            ctrls = _query_controllers()
            with self._controller_lock:
                self._controllers = ctrls
            for _ in range(50):
                if not self._running:
                    return
                time.sleep(0.1)

    def _update_controller_combo(self):
        """Sync combo box with latest controller data (called from UI thread)."""
        with self._controller_lock:
            ctrls = list(self._controllers)

        current_text = self._ctrl_combo.currentText()
        names = [c["name"] for c in ctrls]

        # Only update if the list changed
        existing = [self._ctrl_combo.itemText(i) for i in range(self._ctrl_combo.count())]
        expected = ["All Joints"] + names
        if existing != expected:
            self._ctrl_combo.blockSignals(True)
            self._ctrl_combo.clear()
            self._ctrl_combo.addItem("All Joints")
            for c in ctrls:
                state_color = _GREEN if c["state"] == "active" else _SUBTEXT
                self._ctrl_combo.addItem(c["name"])
            self._ctrl_combo.blockSignals(False)

            # Restore selection
            idx = self._ctrl_combo.findText(current_text)
            if idx >= 0:
                self._ctrl_combo.setCurrentIndex(idx)

    def _on_controller_changed(self, index):
        """Filter calibration table by selected controller's joints."""
        if index <= 0:
            # "All Joints" — show all discovered joints from /joint_states
            self._visible_joints = list(self._all_discovered_joints)
            self._ctrl_type_label.setText("Type: ---")
            self._ctrl_state_label.setText("State: ---")
            self._ctrl_state_label.setStyleSheet(f"color: {_SUBTEXT}; font-size: 11px;")
        else:
            ctrl_name = self._ctrl_combo.currentText()
            with self._controller_lock:
                ctrl_info = next(
                    (c for c in self._controllers if c["name"] == ctrl_name), None
                )
            if ctrl_info:
                short_type = ctrl_info["type"].split("/")[-1] if "/" in ctrl_info["type"] else ctrl_info["type"]
                self._ctrl_type_label.setText(f"Type: {short_type}")
                state_color = _GREEN if ctrl_info["state"] == "active" else _YELLOW
                self._ctrl_state_label.setText(f"State: {ctrl_info['state']}")
                self._ctrl_state_label.setStyleSheet(f"color: {state_color}; font-size: 11px;")

            # Get joints for this controller
            ctrl_joints = _query_controller_joints(ctrl_name)
            if ctrl_joints:
                self._visible_joints = list(ctrl_joints)
            else:
                # Fallback to all discovered joints
                self._visible_joints = list(self._all_discovered_joints)

        self._rebuild_table()

    # -- Tick / update -------------------------------------------------------

    def _tick(self):
        self._update_status_bar()

        if not self._mode_locked:
            self._detect_mode()

        self._update_controller_combo()
        self._update_calibration()
        self._update_live_values()

        # Update 3D viewer
        if self._robot_viewer is not None and self._joint_state_data:
            self._robot_viewer.update_joints(self._joint_state_data)

            # Highlight selected joint
            selected = self._cal_table.currentRow()
            if 0 <= selected < len(self._visible_joints):
                self._robot_viewer.highlight_joint(self._visible_joints[selected])

    def _update_status_bar(self):
        elapsed = time.monotonic() - self._start_time
        h = int(elapsed // 3600)
        m = int((elapsed % 3600) // 60)
        s = int(elapsed % 60)
        self._duration_value.setText(f"{h:02d}:{m:02d}:{s:02d}")

        captured = sum(1 for v in self._offsets.values() if v is not None)
        total = len(self._actuators)
        self._captured_value.setText(f"{captured} / {total}")
        if captured == total:
            self._captured_value.setStyleSheet(
                f"font-size: 13px; font-weight: bold; font-family: monospace; color: {_GREEN};"
            )
        else:
            self._captured_value.setStyleSheet(
                "font-size: 13px; font-weight: bold; font-family: monospace;"
            )

    def _update_calibration(self):
        table = self._cal_table

        for row, joint in enumerate(self._visible_joints):
            if joint not in self._joint_state_data:
                continue  # No data yet

            data = self._joint_state_data[joint]
            pos_rad = data.get("position", 0.0)
            current_deg = math.degrees(pos_rad)
            current_raw = rad_to_raw(pos_rad)

            act = self._actuator_map.get(joint)

            # Live position (shown for all joints)
            self._set_cell(table, row, self.COL_CURRENT_DEG, f"{current_deg:+8.2f}")

            if act:
                # Known actuator — full calibration support
                self._current_raw[joint] = current_raw
                self._set_cell(table, row, self.COL_CURRENT_RAW, f"{current_raw:+8d}")

                # Track real (observed) limits
                rl = self._real_limits[joint]
                if rl["min"] is None or current_raw < rl["min"]:
                    rl["min"] = current_raw
                if rl["max"] is None or current_raw > rl["max"]:
                    rl["max"] = current_raw

                # Real limits display
                if rl["min"] is not None:
                    rl_lo = raw_to_deg(rl["min"])
                    rl_hi = raw_to_deg(rl["max"])
                    urdf_range = math.degrees(act["upper"] - act["lower"])
                    real_range = rl_hi - rl_lo
                    if real_range > 5.0 and abs(real_range - urdf_range) > 10.0:
                        rl_color = _YELLOW
                    else:
                        rl_color = _FG
                    self._set_cell(table, row, self.COL_REAL_LIMITS,
                                   f"{rl_lo:+.0f} .. {rl_hi:+.0f}", rl_color)

                # Compute live offset preview from sweep
                preview_raw, _, _, coverage = _compute_offset_from_sweep(
                    act, self._real_limits
                )

                # Offset column
                locked = self._offsets.get(joint)
                if locked is not None:
                    tpdo = locked["tpdo"]
                    self._set_cell(table, row, self.COL_OFFSET,
                                   f"{tpdo:+.4f} rad", _YELLOW)
                elif preview_raw is not None:
                    preview_rad = -preview_raw * math.pi / RAW_PER_PI
                    self._set_cell(table, row, self.COL_OFFSET,
                                   f"({preview_rad:+.4f})", _SUBTEXT)

                # Status column
                if locked is not None:
                    self._set_cell(table, row, self.COL_STATUS,
                                   f"Captured ({coverage:.0f}%)", _GREEN)
                elif coverage >= 90.0:
                    self._set_cell(table, row, self.COL_STATUS,
                                   f"Ready {coverage:.0f}%", _GREEN)
                elif coverage > 0.0:
                    self._set_cell(table, row, self.COL_STATUS,
                                   f"Sweep {coverage:.0f}%", _YELLOW)
                else:
                    self._set_cell(table, row, self.COL_STATUS, "Pending", _SUBTEXT)
            else:
                # Unknown joint — show raw position as raw column, no calibration
                self._set_cell(table, row, self.COL_CURRENT_RAW, f"{current_raw:+8d}")

    def _update_live_values(self):
        selected = self._cal_table.currentRow()
        if selected < 0 or selected >= len(self._visible_joints):
            return

        joint = self._visible_joints[selected]
        if joint not in self._joint_state_data:
            return

        data = self._joint_state_data[joint]
        pos_rad = data.get("position", 0.0)
        vel = data.get("velocity", 0.0)
        eff = data.get("effort", 0.0)
        current_deg = math.degrees(pos_rad)

        act = self._actuator_map.get(joint)
        if act:
            current_raw = self._current_raw.get(joint, 0)
            locked = self._offsets.get(joint)
            preview_raw, _, _, coverage = _compute_offset_from_sweep(
                act, self._real_limits
            )
            if locked is not None:
                offset_str = f"{locked['tpdo']:+.4f} rad"
            elif preview_raw is not None:
                preview_rad = -preview_raw * math.pi / RAW_PER_PI
                offset_str = f"({preview_rad:+.4f})"
            else:
                offset_str = "    ---"
            rl = self._real_limits[joint]
            rl_min = f"{rl['min']:+8d}" if rl["min"] is not None else "    ---"
            rl_max = f"{rl['max']:+8d}" if rl["max"] is not None else "    ---"
            line = (
                f"[{joint}]  "
                f"raw={current_raw:+8d}  "
                f"deg={current_deg:+8.2f}  "
                f"min={rl_min}  max={rl_max}  "
                f"offset={offset_str}  cov={coverage:.0f}%"
            )
        else:
            line = (
                f"[{joint}]  "
                f"pos={current_deg:+8.2f}deg  "
                f"vel={math.degrees(vel):+8.2f}deg/s  "
                f"eff={eff:+6.3f}"
            )

        self._live_text.append(line)

        doc = self._live_text.document()
        if doc.blockCount() > 200:
            cursor = self._live_text.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.movePosition(cursor.Down, cursor.KeepAnchor,
                                doc.blockCount() - 150)
            cursor.removeSelectedText()

    # -- Button handlers -----------------------------------------------------

    def _capture_selected(self):
        selected = self._cal_table.currentRow()
        if selected < 0 or selected >= len(self._visible_joints):
            self._log("No joint selected. Click a row first.")
            return
        joint = self._visible_joints[selected]
        act = self._actuator_map.get(joint)
        if not act:
            self._log(f"{joint}: not a known actuator — calibration not supported.")
            return
        offset_raw, tpdo, rpdo, coverage = _compute_offset_from_sweep(
            act, self._real_limits
        )
        if offset_raw is None:
            self._log(f"{joint}: no sweep data — move the joint first.")
            return
        rl = self._real_limits[joint]
        self._offsets[joint] = {"tpdo": tpdo, "rpdo": rpdo, "raw": offset_raw}
        self._log(
            f"Captured {joint}:  "
            f"tpdo={tpdo:+.5f} rad  rpdo={rpdo:+d}  "
            f"(sweep {raw_to_deg(rl['min']):+.0f}..{raw_to_deg(rl['max']):+.0f} deg, "
            f"coverage {coverage:.0f}%)"
        )

    def _capture_all(self):
        captured = 0
        for act in self._actuators:
            joint = act["joint"]
            offset_raw, tpdo, rpdo, coverage = _compute_offset_from_sweep(
                act, self._real_limits
            )
            if offset_raw is None:
                self._log(f"  {joint}: SKIPPED — no sweep data")
                continue
            self._offsets[joint] = {"tpdo": tpdo, "rpdo": rpdo, "raw": offset_raw}
            captured += 1
        self._log(f"Captured {captured}/{len(self._actuators)} joints from sweep data.")
        for act in self._actuators:
            joint = act["joint"]
            off = self._offsets[joint]
            if off is not None:
                self._log(
                    f"  {joint}: tpdo={off['tpdo']:+.5f} rad  rpdo={off['rpdo']:+d}"
                )

    def _clear_all(self):
        for joint in self._offsets:
            self._offsets[joint] = None
        self._start_time = time.monotonic()
        self._log("Cleared all captured offsets.")

    def _reset_limits(self):
        for joint in self._real_limits:
            self._real_limits[joint] = {"min": None, "max": None}
        self._log("Reset real limits tracking — sweep joints again to re-measure.")

    def _export_yaml(self):
        captured = sum(1 for v in self._offsets.values() if v is not None)
        if captured == 0:
            self._log("Nothing to export — no offsets captured yet.")
            return

        self._log("")
        self._log("# ── ICube ethercat_driver_ros2 offset values ──")
        self._log("# Per-joint YAML files in arm_real_bringup/config/ethercat/")
        self._log("#")
        self._log("# TxPDO offset (radians): added to position state_interface")
        self._log("# RxPDO offset (raw):     added to position command_interface")
        self._log("")

        for act in self._actuators:
            joint = act["joint"]
            off = self._offsets[joint]
            rl = self._real_limits[joint]
            urdf_lo = math.degrees(act["lower"])
            urdf_hi = math.degrees(act["upper"])
            config = act.get("config", "???")

            self._log(f"# {config}")
            if off is not None:
                self._log(f"  tpdo position:  offset: {off['tpdo']:.6f}    # radians")
                self._log(f"  rpdo position:  offset: {off['rpdo']}    # raw")
            else:
                self._log("  # NOT CAPTURED")

            if rl["min"] is not None:
                rl_lo = raw_to_deg(rl["min"])
                rl_hi = raw_to_deg(rl["max"])
                self._log(
                    f"  # URDF: {urdf_lo:+.0f}..{urdf_hi:+.0f} deg  "
                    f"Real: {rl_lo:+.0f}..{rl_hi:+.0f} deg"
                )
            self._log("")

        self._log("# ── End of offset export ──")
        self._diag_tabs.setCurrentIndex(0)

    # -- Per-joint config actions ---------------------------------------------

    def _write_offset_for_joint(self, joint_name):
        """Write captured offset to EtherCAT YAML config files for one joint."""
        act = self._actuator_map.get(joint_name)
        if not act:
            self._log(f"{joint_name}: not a known actuator.")
            return

        off = self._offsets.get(joint_name)
        if off is None:
            self._log(f"{joint_name}: no captured offset — capture first.")
            return

        config = act["config"]
        rpdo = off["rpdo"]
        tpdo = off["tpdo"]

        reply = QMessageBox.question(
            self, "Write Offset",
            f"Write offset to {config}?\n\n"
            f"  rpdo (raw): {rpdo:+d}\n"
            f"  tpdo (rad): {tpdo:+.6f}\n\n"
            "This modifies files in ethercat/ and ethercat_readonly/.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return

        offsets_dict = {config: {"rpdo": rpdo, "tpdo": tpdo}}

        # Write to both ethercat/ and ethercat_readonly/
        ethercat_dir = find_ethercat_dir()
        if ethercat_dir is None:
            self._log(f"ERROR: could not locate ethercat config directory.")
            return

        results = apply_offsets(offsets_dict, ethercat_dir=ethercat_dir)

        # Also write to ethercat_readonly/ (sibling directory)
        readonly_dir = ethercat_dir.replace("/ethercat", "/ethercat_readonly")
        if os.path.isdir(readonly_dir):
            results += apply_offsets(offsets_dict, ethercat_dir=readonly_dir)

        for fname, success, msg in results:
            status = "OK" if success else "FAIL"
            self._log(f"  {status}: {fname} — {msg}")

    def _reverse_direction_for_joint(self, joint_name):
        """Reverse motor direction for one joint's EtherCAT config files."""
        act = self._actuator_map.get(joint_name)
        if not act:
            self._log(f"{joint_name}: not a known actuator.")
            return

        config = act["config"]

        reply = QMessageBox.question(
            self, "Reverse Direction",
            f"Reverse motor direction for {joint_name}?\n\n"
            f"Config: {config}\n"
            f"Current direction: {act['direction']:+d}\n\n"
            "This negates all factor values in both\n"
            "ethercat/ and ethercat_readonly/.\n\n"
            "Offsets must be recalibrated after reversing.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return

        results = reverse_direction([config])

        for fname, dirname, success, msg in results:
            status = "OK" if success else "FAIL"
            self._log(f"  {status}: {dirname}/{fname} — {msg}")

        # Update in-memory state
        act["direction"] = -act["direction"]
        self._offsets[joint_name] = None
        self._real_limits[joint_name] = {"min": None, "max": None}
        self._log(
            f"Direction reversed for {joint_name}. "
            f"New direction: {act['direction']:+d}. "
            "Offset cleared — recalibrate this joint."
        )

    # -- ROS callbacks -------------------------------------------------------

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            self._joint_state_data[name] = {
                "position": msg.position[i] if i < len(msg.position) else 0.0,
                "velocity": msg.velocity[i] if i < len(msg.velocity) else 0.0,
                "effort": msg.effort[i] if i < len(msg.effort) else 0.0,
            }

        # Discover joints from first message
        if not self._joints_discovered:
            self._all_discovered_joints = list(msg.name)
            # Only populate table if "All Joints" is selected
            if self._ctrl_combo.currentIndex() <= 0:
                self._visible_joints = list(self._all_discovered_joints)
                self._rebuild_table()
            self._joints_discovered = True
            self._status_indicator.setText("JOINT CALIBRATION")
            self._status_indicator.setStyleSheet(
                f"color: {_GREEN}; font-size: 14px; font-weight: bold;"
            )

    def _safety_callback(self, msg):
        self._safety_ok = msg.data

    def _diagnostics_callback(self, msg):
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
                    "state": state, "error_code": error_code
                }

    # -- Helpers -------------------------------------------------------------

    def _log(self, text):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self._cal_log.append(f"[{timestamp}] {text}")

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

    def closeEvent(self, event):
        self._running = False
        if self._robot_viewer is not None:
            try:
                self._robot_viewer.close()
            except Exception:
                pass
        super().closeEvent(event)
