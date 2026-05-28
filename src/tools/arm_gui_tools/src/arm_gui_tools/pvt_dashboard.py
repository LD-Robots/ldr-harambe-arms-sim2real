#!/usr/bin/env python3
"""Robot PVT dashboard — all 25 joints at a glance, grouped by body part,
with per-joint sparkline plots of position, velocity, and effort.

Subscribes to /joint_states_filtered (the filtered joint-state broadcaster's
output: position + IIR-filtered velocity + IIR-filtered effort at 100 Hz)
plus /safety/{estop_state, breach_reason}. Fetches per-joint safety envelopes
once at startup from /robot_safety_supervisor so each sparkline can color its
trace by proximity to the limit.

UI fits a 1080 p display: 1700 × 950 px, no scrolling. 5 columns (Left arm /
Right arm / Waist / Left leg / Right leg) × 6 rows of joint cells = 30 slots,
25 populated, 5 placeholders in the Waist column.
"""

import sys
import threading
import time
from collections import deque

import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int8, String

from PyQt5.QtCore import Qt, QPointF, QTimer
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QVBoxLayout,
    QWidget,
)

# ───────────────────────────────────────────────────────────
# CATPPUCCIN MOCHA PALETTE — matches pvt_tuner.py
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
QFrame#StatusBar {{ background-color: {C_SURFACE0}; border-radius: 4px;
                    padding: 4px; }}
QFrame#JointCell {{ background-color: {C_SURFACE0}; border-radius: 6px;
                    border: 1px solid {C_SURFACE1}; }}
QFrame#PlaceholderCell {{ background-color: transparent; border: 1px dashed
                          {C_SURFACE1}; border-radius: 6px; }}
QLabel#GroupHeader {{ color: {C_PEACH}; font-weight: bold;
                      padding: 4px 0 4px 0; }}
QLabel#CellTitle {{ color: {C_TEXT}; font-weight: bold; }}
"""

# ───────────────────────────────────────────────────────────
# Joint roster, in groups. Order matches controllers_pvt.yaml's `joints:`
# list (and the right-leg ankle order: roll before pitch).
# ───────────────────────────────────────────────────────────

GROUPS = [
    ("Left arm", [
        "left_shoulder_pitch_joint_X6",
        "left_shoulder_roll_joint_X6",
        "left_shoulder_yaw_joint_X4",
        "left_elbow_pitch_joint_X6",
        "left_wrist_yaw_joint_X4",
        "left_wrist_roll_joint_X4",
    ]),
    ("Right arm", [
        "right_shoulder_pitch_joint_X6",
        "right_shoulder_roll_joint_X6",
        "right_shoulder_yaw_joint_X4",
        "right_elbow_pitch_joint_X6",
        "right_wrist_yaw_joint_X4",
        "right_wrist_roll_joint_X4",
    ]),
    ("Waist", [
        "waist_yaw_joint_X8",
    ]),
    ("Left leg", [
        "left_hip_pitch_joint_X8",
        "left_hip_roll_joint_X8",
        "left_hip_yaw_joint_X8",
        "left_knee_joint_X8",
        "left_ankle_pitch_joint_X4",
        "left_ankle_roll_joint_X4",
    ]),
    ("Right leg", [
        "right_hip_pitch_joint_X8",
        "right_hip_roll_joint_X8",
        "right_hip_yaw_joint_X8",
        "right_knee_joint_X8",
        "right_ankle_roll_joint_X4",
        "right_ankle_pitch_joint_X4",
    ]),
]

# All joints flat — used to seed buffers and resolve indices into the
# /joint_states.name array (which may arrive in any order).
ALL_JOINTS = [j for _, joints in GROUPS for j in joints]
NUM_JOINTS = len(ALL_JOINTS)
GROUP_ROWS = max(len(joints) for _, joints in GROUPS)  # 6

# Conservative envelopes if a parameter read returns nothing. Match
# safety_limits.yaml globals so colours still mean something on first launch.
DEFAULT_LIMITS = {
    "position_min": -3.141592653589793,
    "position_max":  3.141592653589793,
    "velocity_limit": 5.0,
    "effort_limit": 12.0,
}

SAMPLE_BUFFER_LEN = 200      # samples per sparkline
UPDATE_RATE_HZ    = 30       # GUI redraw cadence

ESTOP_LABELS = {
    0: ("CLEAR", C_GREEN),
    1: ("E-STOP FREE", C_RED),
    2: ("E-STOP HOLD", C_RED),
}

JOINT_STATES_TOPIC      = "/joint_states_filtered"
SAFETY_ESTOP_TOPIC      = "/safety/estop_state"
SAFETY_BREACH_TOPIC     = "/safety/breach_reason"
SAFETY_KP_SCALE_TOPIC   = "/safety/kp_scale"
SUPERVISOR_NODE         = "/robot_safety_supervisor"


def short_name(joint: str) -> str:
    """Strip side / motor-type clutter so the cell title fits the column.

    `left_shoulder_pitch_joint_X6` → `shoulder pitch (X6)`."""
    name = joint
    for prefix in ("left_", "right_"):
        if name.startswith(prefix):
            name = name[len(prefix):]
            break
    # extract motor type from suffix `_joint_X?`
    motor = ""
    for suffix in ("_joint_X4", "_joint_X6", "_joint_X8"):
        if name.endswith(suffix):
            motor = suffix[-2:]
            name = name[:-len(suffix)]
            break
    name = name.replace("_", " ")
    return f"{name} ({motor})" if motor else name


# ───────────────────────────────────────────────────────────
# Sparkline — compact custom-painted polyline. 200 samples → ~6 s at
# 30 Hz GUI update; the underlying buffer accepts the 100 Hz JointState
# stream and only the most recent SAMPLE_BUFFER_LEN are kept.
# ───────────────────────────────────────────────────────────


class Sparkline(QWidget):
    def __init__(self, width=180, height=28, parent=None):
        super().__init__(parent)
        self.setFixedSize(width, height)
        self._values: deque = deque(maxlen=SAMPLE_BUFFER_LEN)
        self._low = -1.0
        self._high = 1.0
        self._color = QColor(C_GREEN)
        self._warn_color = QColor(C_YELLOW)
        self._error_color = QColor(C_RED)
        # Coloring thresholds, as |value| / max(|low|, |high|).
        self._warn_frac = 0.75
        self._error_frac = 0.90

    def set_range(self, low: float, high: float):
        if high <= low:
            return
        self._low = low
        self._high = high

    def push(self, v: float):
        self._values.append(v)

    def request_repaint(self):
        # Decoupled from push() so the 100 Hz callback path doesn't trigger
        # 100 Hz repaints — the 30 Hz QTimer drives display.
        self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.fillRect(self.rect(), QColor(C_BASE))

        w = self.width()
        h = self.height()
        span = self._high - self._low
        if span <= 0:
            return

        # Zero line if the range straddles zero.
        if self._low < 0 < self._high:
            zero_y = h - (0 - self._low) * h / span
            p.setPen(QPen(QColor(C_SURFACE1), 1, Qt.DashLine))
            p.drawLine(0, int(zero_y), w, int(zero_y))

        if len(self._values) < 2:
            return

        # Colour by the latest sample's proximity to the envelope edge.
        latest = self._values[-1]
        envelope = max(abs(self._low), abs(self._high), 1e-9)
        frac = abs(latest) / envelope
        if frac >= self._error_frac:
            color = self._error_color
        elif frac >= self._warn_frac:
            color = self._warn_color
        else:
            color = self._color

        n = len(self._values)
        pts = []
        for i, v in enumerate(self._values):
            x = i * (w - 1) / (n - 1)
            y = h - (v - self._low) * h / span
            # Clip to the widget so an out-of-range value can't draw outside.
            y = max(0.0, min(float(h - 1), y))
            pts.append(QPointF(x, y))
        p.setPen(QPen(color, 1.4))
        p.drawPolyline(QPolygonF(pts))


# ───────────────────────────────────────────────────────────
# JointCell — one joint, three rows (q / qd / τ). Each row has a label, a
# sparkline, and a current-value readout. The cell is fixed-size so the 5×6
# grid pins to predictable pixels.
# ───────────────────────────────────────────────────────────


class JointCell(QFrame):
    CELL_WIDTH = 332
    CELL_HEIGHT = 132
    SPARK_WIDTH = 170
    SPARK_HEIGHT = 26

    def __init__(self, joint: str, limits: dict, parent=None):
        super().__init__(parent)
        self.setObjectName("JointCell")
        self.setFixedSize(self.CELL_WIDTH, self.CELL_HEIGHT)
        self.joint = joint
        self.limits = limits

        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 4, 8, 6)
        outer.setSpacing(2)

        self.title = QLabel(short_name(joint))
        self.title.setObjectName("CellTitle")
        self.title.setFont(QFont("monospace", 9, QFont.Bold))
        outer.addWidget(self.title)

        # Build (label, sparkline, value-label) per signal.
        self.pos_spark = Sparkline(self.SPARK_WIDTH, self.SPARK_HEIGHT)
        self.vel_spark = Sparkline(self.SPARK_WIDTH, self.SPARK_HEIGHT)
        self.eff_spark = Sparkline(self.SPARK_WIDTH, self.SPARK_HEIGHT)

        self.pos_spark.set_range(
            limits.get("position_min", DEFAULT_LIMITS["position_min"]),
            limits.get("position_max", DEFAULT_LIMITS["position_max"]),
        )
        vlim = limits.get("velocity_limit", DEFAULT_LIMITS["velocity_limit"])
        self.vel_spark.set_range(-vlim, vlim)
        elim = limits.get("effort_limit", DEFAULT_LIMITS["effort_limit"])
        self.eff_spark.set_range(-elim, elim)

        self.pos_val = self._value_label()
        self.vel_val = self._value_label()
        self.eff_val = self._value_label()

        outer.addLayout(self._row("q",  self.pos_spark, self.pos_val))
        outer.addLayout(self._row("qd", self.vel_spark, self.vel_val))
        outer.addLayout(self._row("τ",  self.eff_spark, self.eff_val))

    @staticmethod
    def _value_label() -> QLabel:
        lab = QLabel("—")
        lab.setFont(QFont("monospace", 9))
        lab.setStyleSheet(f"color: {C_SUBTEXT};")
        lab.setMinimumWidth(70)
        lab.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        return lab

    @staticmethod
    def _row(tag: str, spark: Sparkline, value: QLabel) -> QHBoxLayout:
        h = QHBoxLayout()
        h.setSpacing(4)
        h.setContentsMargins(0, 0, 0, 0)
        t = QLabel(tag)
        t.setFont(QFont("monospace", 9))
        t.setStyleSheet(f"color: {C_SUBTEXT};")
        t.setMinimumWidth(16)
        h.addWidget(t)
        h.addWidget(spark)
        h.addWidget(value)
        return h

    def push_sample(self, q: float, qd: float, tau: float):
        self.pos_spark.push(q)
        self.vel_spark.push(qd)
        self.eff_spark.push(tau)

    def refresh_display(self):
        """Re-draw sparklines and update value labels. Called on the GUI
        thread by the 30 Hz timer."""
        self.pos_spark.request_repaint()
        self.vel_spark.request_repaint()
        self.eff_spark.request_repaint()

        for value, spark, unit in (
            (self.pos_val, self.pos_spark, "rad"),
            (self.vel_val, self.vel_spark, "rad/s"),
            (self.eff_val, self.eff_spark, "Nm"),
        ):
            if not spark._values:
                continue
            latest = spark._values[-1]
            envelope = max(abs(spark._low), abs(spark._high), 1e-9)
            frac = abs(latest) / envelope
            if frac >= spark._error_frac:
                color = C_RED
            elif frac >= spark._warn_frac:
                color = C_YELLOW
            else:
                color = C_TEXT
            value.setText(f"{latest:+7.3f} {unit}")
            value.setStyleSheet(f"color: {color};")


class PlaceholderCell(QFrame):
    """Empty cell used to keep the grid aligned where a column is shorter
    than the row span (e.g. Waist has only one joint in a six-row grid)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("PlaceholderCell")
        self.setFixedSize(JointCell.CELL_WIDTH, JointCell.CELL_HEIGHT)


# ───────────────────────────────────────────────────────────
# Main window
# ───────────────────────────────────────────────────────────


class PvtDashboardWindow(QMainWindow):
    def __init__(self, node: Node):
        super().__init__()
        self.setWindowTitle("Robot PVT Dashboard")
        # Fits 1080 p height (with task bar) and 1920 p width.
        self.setMinimumSize(1700, 940)
        self.setStyleSheet(GLOBAL_STYLESHEET)
        self._node = node

        # Thread-safe pipe ROS → GUI:
        # /joint_states callback pushes raw samples into _sample_lock-
        # protected dicts; the QTimer drains them onto each cell.
        self._sample_lock = threading.Lock()
        self._latest_q   = {name: None for name in ALL_JOINTS}
        self._latest_qd  = {name: None for name in ALL_JOINTS}
        self._latest_tau = {name: None for name in ALL_JOINTS}
        # Pending samples to drain into sparkline buffers (keep all values
        # received since last GUI tick so the sparkline shows real cadence).
        self._pending: dict[str, list[tuple[float, float, float]]] = {
            name: [] for name in ALL_JOINTS
        }

        self._cells: dict[str, JointCell] = {}

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(8)

        self._build_status_bar(root)
        self._build_grid(root)
        root.addStretch(0)

        # Latched supervisor subscriptions (always present)
        self._setup_safety_subs()

        # /joint_states subscription
        self._node.create_subscription(
            JointState, JOINT_STATES_TOPIC, self._on_joint_state, 100,
        )

        # GUI redraw timer — decoupled from ROS rate.
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(int(1000 / UPDATE_RATE_HZ))

        # Limits fetched async; cells get default ranges until then.
        threading.Thread(target=self._load_limits_worker, daemon=True).start()

    # ─── UI BUILDERS ────────────────────────────────────────

    def _build_status_bar(self, parent):
        bar = QFrame()
        bar.setObjectName("StatusBar")
        bar.setFixedHeight(40)
        h = QHBoxLayout(bar)
        h.setContentsMargins(10, 4, 10, 4)
        h.setSpacing(20)

        title = QLabel("ROBOT PVT DASHBOARD")
        title.setFont(QFont("monospace", 11, QFont.Bold))
        title.setStyleSheet(f"color: {C_MAUVE};")
        h.addWidget(title)
        h.addSpacing(20)

        h.addWidget(self._make_status_pair("E-STOP:", "(waiting)",
                                            attr="estop_label"))
        h.addWidget(self._make_status_pair("BREACH:", "—",
                                            attr="breach_label", wide=True))
        h.addWidget(self._make_status_pair("JOINT FEED:", "(waiting)",
                                            attr="feed_label"))
        h.addStretch(1)

        legend = QLabel("q / qd / τ  per joint  ·  colour = % of safety envelope")
        legend.setFont(QFont("monospace", 9))
        legend.setStyleSheet(f"color: {C_SUBTEXT};")
        h.addWidget(legend)

        parent.addWidget(bar)

    def _make_status_pair(self, label, value, attr, wide=False) -> QWidget:
        wrap = QFrame()
        l = QHBoxLayout(wrap)
        l.setContentsMargins(0, 0, 0, 0)
        l.setSpacing(6)
        lab = QLabel(label)
        lab.setFont(QFont("monospace", 9))
        lab.setStyleSheet(f"color: {C_SUBTEXT};")
        l.addWidget(lab)
        val = QLabel(value)
        val.setFont(QFont("monospace", 10, QFont.Bold))
        val.setStyleSheet(f"color: {C_SUBTEXT};")
        if wide:
            val.setMinimumWidth(360)
        l.addWidget(val)
        setattr(self, f"_{attr}", val)
        return wrap

    def _build_grid(self, parent):
        host = QWidget()
        grid = QGridLayout(host)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(6)
        grid.setContentsMargins(0, 0, 0, 0)

        for col, (group_name, joints) in enumerate(GROUPS):
            header = QLabel(group_name.upper())
            header.setObjectName("GroupHeader")
            header.setFont(QFont("monospace", 11, QFont.Bold))
            header.setAlignment(Qt.AlignHCenter)
            grid.addWidget(header, 0, col)
            for row in range(GROUP_ROWS):
                if row < len(joints):
                    joint = joints[row]
                    cell = JointCell(joint, {})  # limits filled later
                    self._cells[joint] = cell
                    grid.addWidget(cell, row + 1, col)
                else:
                    grid.addWidget(PlaceholderCell(), row + 1, col)

        parent.addWidget(host)

    # ─── ROS CALLBACKS ──────────────────────────────────────

    def _on_joint_state(self, msg: JointState):
        # The filtered JSB publishes arrays sized to its `joints` parameter
        # in roster order, but a generic /joint_states aggregator may zip
        # multiple sources and reorder. Match by name to be safe.
        n = len(msg.name)
        has_eff = len(msg.effort) == n
        has_vel = len(msg.velocity) == n
        with self._sample_lock:
            for i, name in enumerate(msg.name):
                if name not in self._latest_q:
                    continue
                q = float(msg.position[i]) if i < len(msg.position) else 0.0
                qd = float(msg.velocity[i]) if has_vel else 0.0
                tau = float(msg.effort[i]) if has_eff else 0.0
                self._latest_q[name] = q
                self._latest_qd[name] = qd
                self._latest_tau[name] = tau
                self._pending[name].append((q, qd, tau))

    def _setup_safety_subs(self):
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._node.create_subscription(
            Int8, SAFETY_ESTOP_TOPIC,
            lambda m: self._post_estop(int(m.data)), latched,
        )
        self._node.create_subscription(
            String, SAFETY_BREACH_TOPIC,
            lambda m: self._post_breach(str(m.data)), latched,
        )

    def _post_estop(self, state: int):
        # Cross-thread call into Qt — QTimer reads these on the GUI thread.
        # QLabel.setText is technically not thread-safe; in practice it works
        # for simple atomic string assigns. To be strict we'd post a custom
        # event; this matches pvt_tuner's pattern which has been fine.
        QTimer.singleShot(0, lambda s=state: self._apply_estop(s))

    def _post_breach(self, reason: str):
        QTimer.singleShot(0, lambda r=reason: self._apply_breach(r))

    def _apply_estop(self, state: int):
        label, color = ESTOP_LABELS.get(state, (f"UNKNOWN ({state})", C_RED))
        self._estop_label.setText(label)
        self._estop_label.setStyleSheet(f"color: {color};")

    def _apply_breach(self, reason: str):
        text = reason if reason else "—"
        self._breach_label.setText(text)
        color = C_SUBTEXT if reason in ("", "NONE", "none") else C_RED
        self._breach_label.setStyleSheet(f"color: {color};")

    # ─── PERIODIC GUI TICK ──────────────────────────────────

    def _tick(self):
        # Drain pending samples into sparkline buffers, then ask each cell
        # to redraw and update its readout. One lock-acquire, swap, release.
        with self._sample_lock:
            pending = self._pending
            self._pending = {name: [] for name in ALL_JOINTS}

        any_data = False
        for name, samples in pending.items():
            if not samples:
                continue
            any_data = True
            cell = self._cells.get(name)
            if cell is None:
                continue
            for q, qd, tau in samples:
                cell.push_sample(q, qd, tau)

        for cell in self._cells.values():
            cell.refresh_display()

        if any_data:
            self._feed_label.setText("LIVE")
            self._feed_label.setStyleSheet(f"color: {C_GREEN};")
        else:
            # No samples this tick. Still mark waiting until the first sample
            # arrives so the operator knows whether the controller is up.
            cur = self._feed_label.text()
            if cur == "(waiting)":
                pass
            elif cur == "LIVE":
                self._feed_label.setText("idle")
                self._feed_label.setStyleSheet(f"color: {C_YELLOW};")

    # ─── ASYNC LIMIT LOADING ────────────────────────────────

    def _load_limits_worker(self):
        """Fetch per-joint safety envelopes from /robot_safety_supervisor
        once at startup. Reads safety.global.<field> + safety.joints.<j>.
        <field> so per-joint overrides take effect."""
        keys = ("position_min", "position_max", "velocity_limit", "effort_limit")
        # 1) Read globals.
        global_names = [f"safety.global.{k}" for k in keys]
        global_vals = self._rclpy_get_params(SUPERVISOR_NODE, global_names)
        globals_resolved = {
            k: global_vals.get(f"safety.global.{k}", DEFAULT_LIMITS.get(k))
            for k in keys
        }
        # 2) Read per-joint overrides in one batched request (each joint x
        # each field). If the supervisor isn't up, we silently fall back to
        # globals + DEFAULT_LIMITS; the dashboard still functions.
        joint_names = [
            f"safety.joints.{j}.{k}" for j in ALL_JOINTS for k in keys
        ]
        if joint_names:
            joint_vals = self._rclpy_get_params(
                SUPERVISOR_NODE, joint_names, timeout=10.0,
            )
        else:
            joint_vals = {}

        # 3) Resolve per joint and post back to GUI thread.
        resolved: dict[str, dict] = {}
        for j in ALL_JOINTS:
            limits = dict(globals_resolved)
            for k in keys:
                full = f"safety.joints.{j}.{k}"
                v = joint_vals.get(full)
                if isinstance(v, (int, float)):
                    limits[k] = float(v)
            resolved[j] = limits

        QTimer.singleShot(0, lambda r=resolved: self._apply_limits(r))

    def _apply_limits(self, resolved: dict):
        for joint, limits in resolved.items():
            cell = self._cells.get(joint)
            if cell is None:
                continue
            cell.limits = limits
            cell.pos_spark.set_range(
                limits.get("position_min", DEFAULT_LIMITS["position_min"]),
                limits.get("position_max", DEFAULT_LIMITS["position_max"]),
            )
            vlim = limits.get("velocity_limit",
                              DEFAULT_LIMITS["velocity_limit"])
            cell.vel_spark.set_range(-vlim, vlim)
            elim = limits.get("effort_limit",
                              DEFAULT_LIMITS["effort_limit"])
            cell.eff_spark.set_range(-elim, elim)

    def _rclpy_get_params(self, node_name: str, names: list[str],
                          timeout: float = 5.0) -> dict[str, object]:
        client = self._node.create_client(
            GetParameters, f"{node_name}/get_parameters"
        )
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                print(
                    f"[pvt_dashboard] {node_name}/get_parameters unavailable "
                    f"after {timeout}s — using defaults",
                    file=sys.stderr, flush=True,
                )
                return {}
            req = GetParameters.Request()
            req.names = names
            future = client.call_async(req)
            deadline = time.time() + timeout
            while not future.done() and time.time() < deadline:
                time.sleep(0.02)
            if not future.done():
                print("[pvt_dashboard] get_parameters: timeout",
                      file=sys.stderr, flush=True)
                return {}
            resp = future.result()
            out: dict[str, object] = {}
            for name, val in zip(names, resp.values):
                t = val.type
                if t == ParameterType.PARAMETER_DOUBLE:
                    out[name] = float(val.double_value)
                elif t == ParameterType.PARAMETER_INTEGER:
                    out[name] = int(val.integer_value)
                elif t == ParameterType.PARAMETER_BOOL:
                    out[name] = bool(val.bool_value)
                # NOT_SET / arrays are skipped — every safety.*<scalar>
                # field is a double in safety_limits.yaml.
            return out
        finally:
            self._node.destroy_client(client)


# ───────────────────────────────────────────────────────────
# main
# ───────────────────────────────────────────────────────────


def main():
    rclpy.init(args=sys.argv)
    node = Node("pvt_dashboard_gui")

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    app = QApplication(sys.argv)
    win = PvtDashboardWindow(node)
    win.show()
    exit_code = app.exec_()

    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
