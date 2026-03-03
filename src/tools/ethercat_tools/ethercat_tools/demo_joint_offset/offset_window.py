import math
import os
import time
from datetime import datetime

from PyQt5 import uic
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import QMainWindow, QTableWidgetItem

# Actuator definitions matching the left arm.
# URDF limits from left_arm_joints.xacro (radians).
# URDF zero (0.0 rad) is the kinematic reference — the pose where all
# links align per the kinematic chain definition.
# Direction: +1 or -1, matching the sign of TxPDO/RxPDO factors in
# the per-joint ICube ethercat_driver_ros2 config files.
ACTUATORS = [
    {"joint": "left_shoulder_pitch_joint_X6", "motor": "X6", "slave": 0,
     "direction": 1,  "lower": -3.02, "upper": 1.69,
     "config": "left_shoulder_pitch_X6.yaml"},
    {"joint": "left_shoulder_roll_joint_X6",  "motor": "X6", "slave": 1,
     "direction": 1,  "lower": -0.21, "upper": 2.93,
     "config": "left_shoulder_roll_X6.yaml"},
    {"joint": "left_shoulder_yaw_joint_X4",   "motor": "X4", "slave": 2,
     "direction": -1, "lower": -1.75, "upper": 2.97,
     "config": "left_shoulder_yaw_X4.yaml"},
    {"joint": "left_elbow_pitch_joint_X6",    "motor": "X6", "slave": 3,
     "direction": 1,  "lower": -1.31, "upper": 1.83,
     "config": "left_elbow_pitch_X6.yaml"},
    {"joint": "left_wrist_yaw_joint_X4",      "motor": "X4", "slave": 4,
     "direction": -1, "lower": -2.36, "upper": 2.36,
     "config": "left_wrist_yaw_X4.yaml"},
    {"joint": "left_wrist_roll_joint_X4",     "motor": "X4", "slave": 5,
     "direction": -1, "lower": -1.57, "upper": 1.57,
     "config": "left_wrist_roll_X4.yaml"},
]

# ICube ethercat_driver_ros2 config directory (relative to arm_real_bringup share)
ETHERCAT_CONFIG_DIR = "config/ethercat"

# Unit conversion constants (raw ±65535 = ±180° = ±π rad)
RAW_PER_PI = 65535


def rad_to_raw(rad):
    return int(rad * RAW_PER_PI / math.pi)


def raw_to_deg(raw):
    return raw * 180.0 / RAW_PER_PI


def _compute_offset_from_sweep(act, real_limits):
    """Compute offset from observed encoder sweep and URDF limits.

    The tool tracks positions from /joint_states (already factor-converted
    by the ICube driver, but with offset=0).  current_raw ≈ encoder_raw
    for direction +1, ≈ -encoder_raw for direction -1.

    Returns (offset_raw, offset_tpdo_rad, offset_rpdo_raw, coverage_pct)
    or (None, None, None, 0.0) if no sweep data.

    offset_raw       — tool-domain offset (direction-corrected)
    offset_tpdo_rad  — radians, for the TxPDO position channel
    offset_rpdo_raw  — raw encoder units, for the RxPDO position channel
    """
    rl = real_limits[act["joint"]]
    if rl["min"] is None or rl["max"] is None:
        return None, None, None, 0.0

    real_center_raw = (rl["min"] + rl["max"]) / 2.0
    urdf_center_rad = (act["lower"] + act["upper"]) / 2.0
    urdf_center_raw = rad_to_raw(urdf_center_rad)
    offset_raw = int(round(real_center_raw - urdf_center_raw))

    # Convert to ICube driver units:
    #   TxPDO: ros = factor * enc + offset_tpdo  (radians)
    #   RxPDO: enc = factor * ros + offset_rpdo  (raw)
    offset_rad = offset_raw * math.pi / RAW_PER_PI
    offset_tpdo_rad = -offset_rad
    direction = act.get("direction", 1)
    offset_rpdo_raw = direction * offset_raw

    urdf_range_raw = rad_to_raw(act["upper"] - act["lower"])
    real_range_raw = rl["max"] - rl["min"]
    coverage = (real_range_raw / urdf_range_raw * 100.0) if urdf_range_raw > 0 else 0.0

    return offset_raw, offset_tpdo_rad, offset_rpdo_raw, min(coverage, 100.0)


def _find_ui_file():
    """Locate the .ui file, checking both local and installed paths."""
    local = os.path.join(os.path.dirname(__file__), "joint_offset.ui")
    if os.path.exists(local):
        return local

    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(
            get_package_share_directory("ethercat_tools"), "ui", "joint_offset.ui"
        )
    except Exception:
        return local


class JointOffsetWindow(QMainWindow):
    """Joint zero calibration window — sweep-based offset calculation.

    Calibration procedure:
      1. Sweep each joint through its full mechanical range of motion
      2. Watch the Status column — "Ready" (green) means enough range captured
      3. Click "Capture Selected" to lock in the computed offset
      4. Repeat for each joint, then "Export YAML"

    The offset is derived from the center of the observed encoder range
    vs. the expected URDF center, so it does not depend on the joint's
    position at capture time.
    """

    def __init__(self, ros_node=None):
        super().__init__()
        uic.loadUi(_find_ui_file(), self)

        self._ros_node = ros_node
        self._start_time = time.monotonic()
        self._joint_state_data = {}  # Populated by ROS callback
        self._demo_time = 0.0

        # Per-joint current raw (updated every tick)
        self._current_raw = {act["joint"]: 0 for act in ACTUATORS}
        # Per-joint captured offset (None = not yet captured)
        self._offsets = {act["joint"]: None for act in ACTUATORS}
        # Per-joint observed min/max (real limits from encoder readings)
        self._real_limits = {
            act["joint"]: {"min": None, "max": None} for act in ACTUATORS
        }

        self._setup_table()

        # Connect buttons
        self.captureSelectedBtn.clicked.connect(self._capture_selected)
        self.captureAllBtn.clicked.connect(self._capture_all)
        self.clearAllBtn.clicked.connect(self._clear_all)
        self.resetLimitsBtn.clicked.connect(self._reset_limits)
        self.exportBtn.clicked.connect(self._export_yaml)

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

        self._log("Calibration tool started — sweep-based offset calculation.")
        self._log("Procedure:")
        self._log("  1. Sweep each joint through its full range of motion")
        self._log("  2. Status shows 'Ready' (green) when coverage is sufficient")
        self._log("  3. Click 'Capture Selected' to lock in the computed offset")
        self._log("  4. Export YAML when all joints are captured")

    # -- Table setup ---------------------------------------------------------

    # Column indices
    COL_NAME = 0
    COL_MOTOR = 1
    COL_URDF_LIMITS = 2
    COL_REAL_LIMITS = 3
    COL_CURRENT_DEG = 4
    COL_CURRENT_RAW = 5
    COL_OFFSET = 6
    COL_STATUS = 7

    def _setup_table(self):
        table = self.calibrationTable
        table.horizontalHeader().setStretchLastSection(True)
        table.verticalHeader().setVisible(False)

        for row, act in enumerate(ACTUATORS):
            self._set_cell(table, row, self.COL_NAME, act["joint"])
            self._set_cell(table, row, self.COL_MOTOR, act["motor"])
            # URDF limits in degrees
            lo_deg = math.degrees(act["lower"])
            hi_deg = math.degrees(act["upper"])
            self._set_cell(table, row, self.COL_URDF_LIMITS,
                           f"{lo_deg:+.0f} .. {hi_deg:+.0f}", "#6c7086")
            # Initialize live columns with dashes
            for col in (self.COL_REAL_LIMITS, self.COL_CURRENT_DEG,
                        self.COL_CURRENT_RAW, self.COL_OFFSET, self.COL_STATUS):
                self._set_cell(table, row, col, "---")

        table.resizeColumnsToContents()
        table.setColumnWidth(self.COL_NAME, 260)

    # -- Tick / update -------------------------------------------------------

    def _tick(self):
        self._demo_time += 0.1

        self._update_status_bar()
        self._update_calibration()
        self._update_live_values()

    def _update_status_bar(self):
        elapsed = time.monotonic() - self._start_time
        h = int(elapsed // 3600)
        m = int((elapsed % 3600) // 60)
        s = int(elapsed % 60)
        self.durationValue.setText(f"{h:02d}:{m:02d}:{s:02d}")

        captured = sum(1 for v in self._offsets.values() if v is not None)
        total = len(ACTUATORS)
        self.capturedValue.setText(f"{captured} / {total}")
        if captured == total:
            self.capturedValue.setStyleSheet(
                "font-size: 13px; font-weight: bold; font-family: monospace; "
                "color: #a6e3a1;"
            )
        else:
            self.capturedValue.setStyleSheet(
                "font-size: 13px; font-weight: bold; font-family: monospace;"
            )

    def _update_calibration(self):
        table = self.calibrationTable

        for row, act in enumerate(ACTUATORS):
            joint = act["joint"]

            if self._ros_node and joint in self._joint_state_data:
                pos_rad = self._joint_state_data[joint].get("position", 0.0)
                current_raw = rad_to_raw(pos_rad)
            else:
                # Demo: sinusoidal motion with per-joint phase offset
                phase = row * 0.8
                pos_deg = 30.0 * math.sin(self._demo_time * 0.5 + phase)
                current_raw = int(pos_deg / 180.0 * RAW_PER_PI)

            self._current_raw[joint] = current_raw
            current_deg = raw_to_deg(current_raw)

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
                # Color-code: yellow if real range differs significantly from URDF
                urdf_range = math.degrees(act["upper"] - act["lower"])
                real_range = rl_hi - rl_lo
                if real_range > 5.0 and abs(real_range - urdf_range) > 10.0:
                    rl_color = "#f9e2af"  # yellow — mismatch
                else:
                    rl_color = "#cdd6f4"  # normal
                self._set_cell(table, row, self.COL_REAL_LIMITS,
                               f"{rl_lo:+.0f} .. {rl_hi:+.0f}", rl_color)
            else:
                self._set_cell(table, row, self.COL_REAL_LIMITS, "---")

            # Live position
            self._set_cell(table, row, self.COL_CURRENT_DEG,
                           f"{current_deg:+8.2f}")
            self._set_cell(table, row, self.COL_CURRENT_RAW,
                           f"{current_raw:+8d}")

            # Compute live offset preview from sweep
            preview_raw, _, _, coverage = _compute_offset_from_sweep(
                act, self._real_limits
            )

            # Offset column — show TxPDO offset in radians
            locked = self._offsets[joint]
            if locked is not None:
                tpdo = locked["tpdo"]
                self._set_cell(table, row, self.COL_OFFSET,
                               f"{tpdo:+.4f} rad", "#f9e2af")
            elif preview_raw is not None:
                preview_rad = -preview_raw * math.pi / RAW_PER_PI
                self._set_cell(table, row, self.COL_OFFSET,
                               f"({preview_rad:+.4f})", "#6c7086")
            else:
                self._set_cell(table, row, self.COL_OFFSET, "---")

            # Status column — show sweep coverage
            if locked is not None:
                self._set_cell(table, row, self.COL_STATUS,
                               f"Captured ({coverage:.0f}%)", "#a6e3a1")
            elif coverage >= 90.0:
                self._set_cell(table, row, self.COL_STATUS,
                               f"Ready {coverage:.0f}%", "#a6e3a1")
            elif coverage > 0.0:
                self._set_cell(table, row, self.COL_STATUS,
                               f"Sweep {coverage:.0f}%", "#f9e2af")
            else:
                self._set_cell(table, row, self.COL_STATUS,
                               "Pending", "#6c7086")

    def _update_live_values(self):
        """Show raw value stream for the selected joint in the Live Values tab."""
        selected = self.calibrationTable.currentRow()
        if selected < 0 or selected >= len(ACTUATORS):
            return

        act = ACTUATORS[selected]
        joint = act["joint"]
        current_raw = self._current_raw[joint]
        current_deg = raw_to_deg(current_raw)
        locked = self._offsets[joint]
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
        self.liveValuesText.append(line)

        # Keep the log from growing unbounded
        doc = self.liveValuesText.document()
        if doc.blockCount() > 200:
            cursor = self.liveValuesText.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.movePosition(cursor.Down, cursor.KeepAnchor,
                                doc.blockCount() - 150)
            cursor.removeSelectedText()

    # -- Button handlers -----------------------------------------------------

    def _capture_selected(self):
        selected = self.calibrationTable.currentRow()
        if selected < 0 or selected >= len(ACTUATORS):
            self._log("No joint selected. Click a row first.")
            return
        act = ACTUATORS[selected]
        joint = act["joint"]
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
        for row, act in enumerate(ACTUATORS):
            joint = act["joint"]
            offset_raw, tpdo, rpdo, coverage = _compute_offset_from_sweep(
                act, self._real_limits
            )
            if offset_raw is None:
                self._log(f"  {joint}: SKIPPED — no sweep data")
                continue
            self._offsets[joint] = {"tpdo": tpdo, "rpdo": rpdo, "raw": offset_raw}
            captured += 1
        self._log(f"Captured {captured}/{len(ACTUATORS)} joints from sweep data.")
        for act in ACTUATORS:
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

        for act in ACTUATORS:
            joint = act["joint"]
            off = self._offsets[joint]
            rl = self._real_limits[joint]
            urdf_lo = math.degrees(act["lower"])
            urdf_hi = math.degrees(act["upper"])
            config = act.get("config", "???")

            self._log(f"# {config}")
            if off is not None:
                self._log(
                    f"  tpdo position:  offset: {off['tpdo']:.6f}    # radians"
                )
                self._log(
                    f"  rpdo position:  offset: {off['rpdo']}    # raw"
                )
            else:
                self._log(f"  # NOT CAPTURED")

            if rl["min"] is not None:
                rl_lo = raw_to_deg(rl["min"])
                rl_hi = raw_to_deg(rl["max"])
                self._log(
                    f"  # URDF: {urdf_lo:+.0f}..{urdf_hi:+.0f} deg  "
                    f"Real: {rl_lo:+.0f}..{rl_hi:+.0f} deg"
                )
            self._log("")

        self._log("# ── End of offset export ──")

        # Switch to the log tab so user can see the output
        self.diagnosticsTabs.setCurrentIndex(0)

    # -- ROS callback --------------------------------------------------------

    def joint_state_callback(self, msg):
        """Called from the ROS subscriber with sensor_msgs/JointState."""
        for i, name in enumerate(msg.name):
            self._joint_state_data[name] = {
                "position": msg.position[i] if i < len(msg.position) else 0.0,
                "velocity": msg.velocity[i] if i < len(msg.velocity) else 0.0,
                "effort": msg.effort[i] if i < len(msg.effort) else 0.0,
            }

    # -- Helpers -------------------------------------------------------------

    def _log(self, text):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.calibrationLog.append(f"[{timestamp}] {text}")

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
