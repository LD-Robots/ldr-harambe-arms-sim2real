#!/usr/bin/env python3
"""Bus voltage viewer — DC link voltage for all 25 joints at a glance.

Subscribes to ``<broadcaster_ns>/bus_voltage`` (Float64MultiArray, one float
per joint in canonical controller order). The broadcaster namespace is
discovered dynamically by scanning ``/controller_manager`` for a controller
whose plugin type ends with ``DriveStatusBroadcaster``. UI mirrors
``pvt_dashboard.py``: 5×6 grid grouped Left arm / Right arm / Waist / Left
leg / Right leg, Catppuccin Mocha palette. Each cell shows the current
voltage (big, colored green inside ``[BUS_V_MIN, BUS_V_MAX]`` and red
outside) plus session min/max. A ``Reset min/max`` button at the top
clears the extremes so the operator can scope a single motion."""

import os
import re
import subprocess
import sys
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers
from std_msgs.msg import Float64MultiArray

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

# ───────────────────────────────────────────────────────────
# CATPPUCCIN MOCHA PALETTE — matches pvt_tuner.py / pvt_dashboard.py
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
QPushButton {{ background-color: {C_SURFACE0}; color: {C_TEXT};
               border: 1px solid {C_SURFACE1}; border-radius: 4px;
               padding: 4px 10px; font-weight: bold; }}
QPushButton:hover {{ background-color: {C_SURFACE1}; }}
QPushButton:pressed {{ background-color: #585b70; }}
QPushButton:disabled {{ color: {C_SUBTEXT}; }}
"""

# ───────────────────────────────────────────────────────────
# Joint roster — same as pvt_dashboard.py / controllers_pvt.yaml.
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

ALL_JOINTS = [j for _, joints in GROUPS for j in joints]
NUM_JOINTS = len(ALL_JOINTS)
GROUP_ROWS = max(len(joints) for _, joints in GROUPS)  # 6

# Thresholds — keep aligned with pvt_tuner.py BUS_V_MIN/MAX (lines 111-112)
# and the drive_status_broadcaster YAML defaults.
BUS_V_MIN = 40.0
BUS_V_MAX = 54.0

UPDATE_RATE_HZ = 30
STALE_TIMEOUT_S = 1.0           # mark cells stale if no msg this long
NO_TOPIC_TIMEOUT_S = 5.0        # warn "no bus_voltage on <ns>" after this
DISCOVERY_RETRY_S = 5.0         # auto-retry interval when broadcaster is gone


def short_name(joint: str) -> str:
    """Strip side / motor-type clutter so the cell title fits the column.

    ``left_shoulder_pitch_joint_X6`` → ``shoulder pitch (X6)``."""
    name = joint
    for prefix in ("left_", "right_"):
        if name.startswith(prefix):
            name = name[len(prefix):]
            break
    motor = ""
    for suffix in ("_joint_X4", "_joint_X6", "_joint_X8"):
        if name.endswith(suffix):
            motor = suffix[-2:]
            name = name[:-len(suffix)]
            break
    name = name.replace("_", " ")
    return f"{name} ({motor})" if motor else name


# ───────────────────────────────────────────────────────────
# JointCell — one joint, three lines: title, big current V, min/max row.
# ───────────────────────────────────────────────────────────


class JointCell(QFrame):
    CELL_WIDTH = 332
    CELL_HEIGHT = 96

    def __init__(self, joint: str, parent=None):
        super().__init__(parent)
        self.setObjectName("JointCell")
        self.setFixedSize(self.CELL_WIDTH, self.CELL_HEIGHT)
        self.joint = joint

        self._min: float | None = None
        self._max: float | None = None
        self._last_value: float | None = None
        self._last_ts: float | None = None

        outer = QVBoxLayout(self)
        outer.setContentsMargins(10, 6, 10, 8)
        outer.setSpacing(4)

        self.title = QLabel(short_name(joint))
        self.title.setObjectName("CellTitle")
        self.title.setFont(QFont("monospace", 9, QFont.Bold))
        outer.addWidget(self.title)

        self.value_label = QLabel("no data")
        self.value_label.setFont(QFont("monospace", 18, QFont.Bold))
        self.value_label.setAlignment(Qt.AlignCenter)
        self.value_label.setStyleSheet(f"color: {C_SUBTEXT};")
        outer.addWidget(self.value_label)

        row = QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(8)
        self.min_label = QLabel("min:   —.—  V")
        self.max_label = QLabel("max:   —.—  V")
        for lab in (self.min_label, self.max_label):
            lab.setFont(QFont("monospace", 8))
            lab.setStyleSheet(f"color: {C_SUBTEXT};")
        row.addWidget(self.min_label)
        row.addStretch(1)
        row.addWidget(self.max_label)
        outer.addLayout(row)

    def update_voltage(self, v: float, now: float):
        self._last_value = v
        self._last_ts = now
        if self._min is None or v < self._min:
            self._min = v
        if self._max is None or v > self._max:
            self._max = v
        in_band = BUS_V_MIN <= v <= BUS_V_MAX
        color = C_GREEN if in_band else C_RED
        self.value_label.setText(f"{v:6.2f} V")
        self.value_label.setStyleSheet(f"color: {color};")
        self.min_label.setText(f"min: {self._min:6.2f} V")
        self.max_label.setText(f"max: {self._max:6.2f} V")

    def mark_stale(self, now: float):
        if self._last_ts is None:
            return  # already in "no data" state, nothing to do
        if now - self._last_ts <= STALE_TIMEOUT_S:
            return
        # Recolor the current value yellow + (stale) suffix once.
        v = self._last_value if self._last_value is not None else 0.0
        text = f"{v:6.2f} V (stale)"
        if self.value_label.text() != text:
            self.value_label.setText(text)
            self.value_label.setStyleSheet(f"color: {C_YELLOW};")

    def reset_extremes(self):
        self._min = self._last_value
        self._max = self._last_value
        if self._last_value is None:
            self.min_label.setText("min:   —.—  V")
            self.max_label.setText("max:   —.—  V")
        else:
            self.min_label.setText(f"min: {self._last_value:6.2f} V")
            self.max_label.setText(f"max: {self._last_value:6.2f} V")


class PlaceholderCell(QFrame):
    """Empty cell used to pad columns shorter than ``GROUP_ROWS`` (Waist)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("PlaceholderCell")
        self.setFixedSize(JointCell.CELL_WIDTH, JointCell.CELL_HEIGHT)


# ───────────────────────────────────────────────────────────
# Main window
# ───────────────────────────────────────────────────────────


class BusVoltageViewerWindow(QMainWindow):
    # Cross-thread signals (ROS callbacks → GUI thread).
    _voltage_array_sig = pyqtSignal(object)               # list[float]
    _discovery_sig = pyqtSignal(object, int, str)         # ns, total, source

    def __init__(self, node: Node):
        super().__init__()
        self.setWindowTitle("Bus Voltage Viewer")
        self.setMinimumSize(1700, 720)
        self.setStyleSheet(GLOBAL_STYLESHEET)
        self._node = node

        self._broadcaster_ns: str | None = None
        self._broadcaster_sub = None
        self._subscription_started_at: float | None = None
        self._discovery_in_flight = False
        self._cells: dict[str, JointCell] = {}

        # Subscription state — written off the GUI thread.
        self._array_lock = threading.Lock()
        self._last_array_ts: float | None = None
        self._msg_count = 0
        self._msg_count_window = deque(maxlen=UPDATE_RATE_HZ * 2)  # ~2 s history

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(8)

        self._build_status_bar(root)
        self._build_grid(root)
        root.addStretch(0)

        self._voltage_array_sig.connect(self._on_voltage_array)
        self._discovery_sig.connect(self._on_discovery_result)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(int(1000 / UPDATE_RATE_HZ))

        QTimer.singleShot(0, self._start_discovery)

    # ─── UI BUILDERS ────────────────────────────────────────

    def _build_status_bar(self, parent):
        bar = QFrame()
        bar.setObjectName("StatusBar")
        bar.setFixedHeight(40)
        h = QHBoxLayout(bar)
        h.setContentsMargins(10, 4, 10, 4)
        h.setSpacing(20)

        title = QLabel("BUS VOLTAGE VIEWER")
        title.setFont(QFont("monospace", 11, QFont.Bold))
        title.setStyleSheet(f"color: {C_MAUVE};")
        h.addWidget(title)
        h.addSpacing(20)

        h.addWidget(self._make_status_pair("BROADCASTER:", "discovering…",
                                           attr="broadcaster_label",
                                           wide=True, color=C_PEACH))
        h.addWidget(self._make_status_pair("FEED:", "(waiting)",
                                           attr="feed_label"))
        h.addWidget(self._make_status_pair("LEN:", "—",
                                           attr="len_label"))
        h.addStretch(1)

        self._reset_btn = QPushButton("Reset min/max")
        self._reset_btn.clicked.connect(self._reset_all_extremes)
        h.addWidget(self._reset_btn)

        self._refresh_btn = QPushButton("Refresh")
        self._refresh_btn.clicked.connect(self._start_discovery)
        h.addWidget(self._refresh_btn)

        h.addSpacing(10)
        legend = QLabel(
            f"green: {BUS_V_MIN:.0f} ≤ V ≤ {BUS_V_MAX:.0f}  ·  "
            "red: out of range  ·  yellow: stale"
        )
        legend.setFont(QFont("monospace", 9))
        legend.setStyleSheet(f"color: {C_SUBTEXT};")
        h.addWidget(legend)

        parent.addWidget(bar)

    def _make_status_pair(self, label, value, attr,
                          wide=False, color=C_SUBTEXT) -> QWidget:
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
        val.setStyleSheet(f"color: {color};")
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
                    cell = JointCell(joint)
                    self._cells[joint] = cell
                    grid.addWidget(cell, row + 1, col)
                else:
                    grid.addWidget(PlaceholderCell(), row + 1, col)

        parent.addWidget(host)

    # ─── DISCOVERY ──────────────────────────────────────────

    def _start_discovery(self):
        if self._discovery_in_flight:
            print("[bus_voltage_viewer] discovery already in flight; ignoring",
                  file=sys.stderr, flush=True)
            return
        self._discovery_in_flight = True
        self._refresh_btn.setEnabled(False)
        self._refresh_btn.setText("…")
        print("[bus_voltage_viewer] discovery cycle starting",
              file=sys.stderr, flush=True)
        threading.Thread(target=self._discover_worker_safe, daemon=True).start()

    def _discover_worker_safe(self):
        try:
            self._discover_worker()
        except Exception as e:
            print(f"[bus_voltage_viewer] discovery thread crashed: {e!r}",
                  file=sys.stderr, flush=True)
            QTimer.singleShot(0, self._discovery_done)

    def _discovery_done(self):
        self._discovery_in_flight = False
        self._refresh_btn.setEnabled(True)
        self._refresh_btn.setText("Refresh")

    def _discover_worker(self):
        """Try rclpy ListControllers first (fast on warm DDS cache); fall back
        to ``ros2 control list_controllers`` subprocess if that fails."""
        print("[bus_voltage_viewer] trying rclpy ListControllers…",
              file=sys.stderr, flush=True)
        controllers = self._list_controllers_rclpy()
        source = "rclpy"
        if controllers is None:
            print("[bus_voltage_viewer] rclpy path failed; falling back to "
                  "subprocess…", file=sys.stderr, flush=True)
            controllers = self._list_controllers_subprocess(timeout=30)
            source = "subprocess"

        broadcaster_ns: str | None = None
        for name, ctype, state in controllers:
            print(f"[bus_voltage_viewer]   - {name!r}  type={ctype!r}  "
                  f"state={state!r}", file=sys.stderr, flush=True)
            if state != "active":
                continue
            if ctype.endswith("DriveStatusBroadcaster"):
                broadcaster_ns = f"/{name}"

        print(f"[bus_voltage_viewer] dispatching to GUI: "
              f"broadcaster={broadcaster_ns}", file=sys.stderr, flush=True)
        self._discovery_sig.emit(broadcaster_ns, len(controllers), source)

    def _list_controllers_rclpy(self):
        client = self._node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        try:
            t0 = time.time()
            ok = client.wait_for_service(timeout_sec=30.0)
            print(f"[bus_voltage_viewer] rclpy: wait_for_service={ok} "
                  f"after {time.time()-t0:.1f}s",
                  file=sys.stderr, flush=True)
            if not ok:
                return None
            future = client.call_async(ListControllers.Request())
            deadline = time.time() + 30.0
            last_log = time.time()
            while not future.done() and time.time() < deadline:
                time.sleep(0.05)
                if time.time() - last_log > 2.0:
                    print(f"[bus_voltage_viewer] rclpy: still waiting "
                          f"({time.time()-t0:.1f}s)",
                          file=sys.stderr, flush=True)
                    last_log = time.time()
            if not future.done():
                print("[bus_voltage_viewer] rclpy: response timed out after 30s",
                      file=sys.stderr, flush=True)
                return None
            resp = future.result()
            print(f"[bus_voltage_viewer] rclpy: response received with "
                  f"{len(resp.controller)} controller(s)",
                  file=sys.stderr, flush=True)
            return [(c.name, c.type, c.state) for c in resp.controller]
        finally:
            self._node.destroy_client(client)

    @staticmethod
    def _list_controllers_subprocess(timeout=10):
        try:
            r = subprocess.run(
                ["ros2", "control", "list_controllers"],
                capture_output=True, text=True, timeout=timeout,
            )
            if r.returncode != 0:
                print(f"[bus_voltage_viewer] `ros2 control list_controllers` "
                      f"returned {r.returncode}: {r.stderr.strip()}",
                      file=sys.stderr, flush=True)
                return []
        except (subprocess.TimeoutExpired, FileNotFoundError) as e:
            print(f"[bus_voltage_viewer] subprocess failed: {e!r}",
                  file=sys.stderr, flush=True)
            return []
        out = []
        ansi = re.compile(r"\x1b\[[0-9;]*[a-zA-Z]")
        for raw in r.stdout.splitlines():
            line = ansi.sub("", raw).strip()
            m = re.match(r"^(\S+)\s+(\S+/\S+)\s+(active|inactive|unconfigured)",
                         line)
            if m:
                out.append((m.group(1), m.group(2), m.group(3)))
        return out

    def _on_discovery_result(self, broadcaster_ns, total_controllers, source):
        domain = os.environ.get("ROS_DOMAIN_ID", "0")
        if broadcaster_ns is None:
            text = (
                f"not running (via {source}, {total_controllers} "
                f"controller(s) on domain {domain})"
            )
            color = C_YELLOW
            # Tear down any old subscription if the broadcaster vanished.
            if self._broadcaster_sub is not None:
                self._node.destroy_subscription(self._broadcaster_sub)
                self._broadcaster_sub = None
                self._broadcaster_ns = None
                self._subscription_started_at = None
            # Auto-retry so the viewer recovers without operator action.
            QTimer.singleShot(int(DISCOVERY_RETRY_S * 1000), self._start_discovery)
        else:
            text = f"{broadcaster_ns} (via {source})"
            color = C_GREEN
            if broadcaster_ns != self._broadcaster_ns:
                if self._broadcaster_sub is not None:
                    self._node.destroy_subscription(self._broadcaster_sub)
                self._broadcaster_ns = broadcaster_ns
                self._broadcaster_sub = self._node.create_subscription(
                    Float64MultiArray,
                    f"{broadcaster_ns}/bus_voltage",
                    self._on_bus_voltage_msg,
                    10,
                )
                self._subscription_started_at = time.monotonic()
                print(f"[bus_voltage_viewer] subscribed to "
                      f"{broadcaster_ns}/bus_voltage",
                      file=sys.stderr, flush=True)

        self._broadcaster_label.setText(text)
        self._broadcaster_label.setStyleSheet(f"color: {color};")
        self._discovery_done()

    # ─── SUBSCRIPTION CALLBACK (off-GUI thread) ─────────────

    def _on_bus_voltage_msg(self, msg):
        arr = [float(x) for x in msg.data]
        now = time.monotonic()
        with self._array_lock:
            self._last_array_ts = now
            self._msg_count += 1
            self._msg_count_window.append(now)
        self._voltage_array_sig.emit(arr)

    # ─── GUI-THREAD SLOTS ───────────────────────────────────

    def _on_voltage_array(self, arr):
        n = len(arr)
        if n == NUM_JOINTS:
            self._len_label.setText(f"{n}")
            self._len_label.setStyleSheet(f"color: {C_GREEN};")
        else:
            self._len_label.setText(f"len={n} != {NUM_JOINTS}")
            self._len_label.setStyleSheet(f"color: {C_RED};")

        now = time.monotonic()
        # Trust controller joint order — broadcaster's `joints:` parameter is
        # pinned to controllers_pvt.yaml; reorder check would add a failure
        # mode for negligible benefit (same trade-off as pvt_dashboard).
        for i, joint in enumerate(ALL_JOINTS):
            if i >= n:
                break
            self._cells[joint].update_voltage(arr[i], now)

    def _tick(self):
        now = time.monotonic()
        # Stale check per cell.
        for cell in self._cells.values():
            cell.mark_stale(now)

        # FEED status — depends on broadcaster state too so we can distinguish
        # "no broadcaster" from "broadcaster up but no bus_voltage topic".
        with self._array_lock:
            last_ts = self._last_array_ts
            # Drop entries from the rolling window older than 1 s.
            while self._msg_count_window and \
                    now - self._msg_count_window[0] > 1.0:
                self._msg_count_window.popleft()
            hz = len(self._msg_count_window)

        if last_ts is None:
            if self._broadcaster_ns is None or \
                    self._subscription_started_at is None:
                text, color = "(waiting)", C_SUBTEXT
            elif now - self._subscription_started_at > NO_TOPIC_TIMEOUT_S:
                text, color = f"no bus_voltage on {self._broadcaster_ns}", C_RED
            else:
                text, color = "(waiting)", C_SUBTEXT
        elif now - last_ts > STALE_TIMEOUT_S:
            text, color = "idle", C_YELLOW
        else:
            text, color = f"LIVE @ {hz} Hz", C_GREEN

        self._feed_label.setText(text)
        self._feed_label.setStyleSheet(f"color: {color};")

    def _reset_all_extremes(self):
        for cell in self._cells.values():
            cell.reset_extremes()

    # ─── CLEANUP ────────────────────────────────────────────

    def closeEvent(self, event):
        if self._broadcaster_sub is not None:
            self._node.destroy_subscription(self._broadcaster_sub)
            self._broadcaster_sub = None
        super().closeEvent(event)


# ───────────────────────────────────────────────────────────
# main
# ───────────────────────────────────────────────────────────


def main():
    rclpy.init(args=sys.argv)
    node = Node("bus_voltage_viewer_gui")

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    app = QApplication(sys.argv)
    win = BusVoltageViewerWindow(node)
    win.show()
    exit_code = app.exec_()

    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
