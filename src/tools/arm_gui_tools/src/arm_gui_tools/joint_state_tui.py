#!/usr/bin/env python3
"""Robot dashboard TUI — htop for the Harambe dual-arm system.

Multi-tab terminal dashboard for safe sim2real operation.
Tabs: Joints | Safety | Controllers | Event Log
Keyboard: 1-4 switch tabs, e=e-stop, r=reset, q=quit

Usage:
    ros2 run arm_gui_tools joint_state_tui
    ros2 run arm_gui_tools joint_state_tui -- --demo
"""

import io
import math
import os
import re
import select
import signal
import subprocess
import sys
import termios
import threading
import time
import tty
from collections import deque
from datetime import datetime

from rich.console import Console
from rich.layout import Layout
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

# ═══════════════════════════════════════════════════════════
# CATPPUCCIN MOCHA PALETTE
# ═══════════════════════════════════════════════════════════

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
C_MAUVE = '#cba6f7'
C_TEAL = '#94e2d5'
C_LAVENDER = '#b4befe'

# ═══════════════════════════════════════════════════════════
# JOINT CLASSIFICATION
# ═══════════════════════════════════════════════════════════

_ARM_SUFFIX = re.compile(r'_X[46]$')
_HAND_KEYWORDS = {'thumb', 'index', 'middle', 'ring', 'pinky'}
GROUP_ORDER = ['L Arm', 'R Arm', 'L Hand', 'R Hand', 'Other']
GROUP_STYLES = {
    'L Arm': C_BLUE, 'R Arm': C_TEAL,
    'L Hand': C_PEACH, 'R Hand': C_MAUVE,
    'Other': C_SUBTEXT,
}


def _classify(name):
    lower = name.lower()
    is_arm = bool(_ARM_SUFFIX.search(name))
    if lower.startswith('left_'):
        if is_arm:
            return 'L Arm'
        for kw in _HAND_KEYWORDS:
            if kw in lower:
                return 'L Hand'
    elif lower.startswith('right_'):
        if is_arm:
            return 'R Arm'
        for kw in _HAND_KEYWORDS:
            if kw in lower:
                return 'R Hand'
    return 'Other'


# ═══════════════════════════════════════════════════════════
# VISUAL HELPERS
# ═══════════════════════════════════════════════════════════

BRAILLE_BASE = 0x2800
BRAILLE_ROWS = [
    (0x01, 0x08), (0x02, 0x10), (0x04, 0x20), (0x40, 0x80),
]


def sparkline(values, width=20, vmin=None, vmax=None):
    if not values:
        return '\u2800' * width
    data = list(values)
    needed = width * 2
    if len(data) < needed:
        data = [data[0]] * (needed - len(data)) + data
    else:
        data = data[-needed:]
    if vmin is None:
        vmin = min(data)
    if vmax is None:
        vmax = max(data)
    span = vmax - vmin if vmax != vmin else 1.0
    chars = []
    for i in range(0, len(data), 2):
        code = BRAILLE_BASE
        for col_idx in range(2):
            if i + col_idx < len(data):
                v = data[i + col_idx]
                level = int((v - vmin) / span * 3.99)
                level = max(0, min(3, level))
                code |= BRAILLE_ROWS[3 - level][col_idx]
        chars.append(chr(code))
    return ''.join(chars)


FULL_BLOCK = '\u2588'
PARTIAL_BLOCKS = ' \u258f\u258e\u258d\u258c\u258b\u258a\u2589\u2588'


def hbar(value, vmin, vmax, width=15):
    if vmax == vmin:
        ratio = 0.5
    else:
        ratio = (value - vmin) / (vmax - vmin)
    ratio = max(0.0, min(1.0, ratio))
    fill = ratio * width
    full = int(fill)
    frac = fill - full
    partial_idx = int(frac * 8)
    result = FULL_BLOCK * full
    if full < width:
        result += PARTIAL_BLOCKS[partial_idx]
        result += ' ' * (width - full - 1)
    return result


def ratio_bar(ratio, width=8):
    """Bar from 0.0-1.0 ratio."""
    ratio = max(0.0, min(1.0, ratio))
    fill = ratio * width
    full = int(fill)
    frac = fill - full
    partial_idx = int(frac * 8)
    result = FULL_BLOCK * full
    if full < width:
        result += PARTIAL_BLOCKS[partial_idx]
        result += '\u2591' * (width - full - 1)
    return result


def severity_color(ratio):
    if ratio >= 0.9:
        return C_RED
    elif ratio >= 0.7:
        return C_YELLOW
    return C_GREEN


# ═══════════════════════════════════════════════════════════
# CONSTANTS
# ═══════════════════════════════════════════════════════════

STALE_TIMEOUT = 2.0
HISTORY_LEN = 80

JOINT_LIMITS = {
    'left_shoulder_pitch_joint_X6':  (-3.02, 1.69),
    'left_shoulder_roll_joint_X6':   (-0.21, 2.93),
    'left_shoulder_yaw_joint_X4':    (-1.75, 2.97),
    'left_elbow_pitch_joint_X6':     (-1.31, 1.83),
    'left_wrist_yaw_joint_X4':       (-2.36, 2.36),
    'left_wrist_roll_joint_X4':      (-1.57, 1.57),
    'right_shoulder_pitch_joint_X6': (-3.02, 1.69),
    'right_shoulder_roll_joint_X6':  (-2.93, 0.21),
    'right_shoulder_yaw_joint_X4':   (-2.97, 1.75),
    'right_elbow_pitch_joint_X6':    (-1.31, 1.83),
    'right_wrist_yaw_joint_X4':      (-2.36, 2.36),
    'right_wrist_roll_joint_X4':     (-1.57, 1.57),
}

SAFETY_LIMITS = {
    'left_shoulder_pitch_joint_X6':  {'pos_min': -3.02, 'pos_max': 1.72, 'vel_max': 2.0, 'torque_max': 40},
    'left_shoulder_roll_joint_X6':   {'pos_min': -0.21, 'pos_max': 2.90, 'vel_max': 2.0, 'torque_max': 40},
    'left_shoulder_yaw_joint_X4':    {'pos_min': -1.72, 'pos_max': 3.02, 'vel_max': 2.0, 'torque_max': 40},
    'left_elbow_pitch_joint_X6':     {'pos_min': -1.61, 'pos_max': 1.73, 'vel_max': 2.0, 'torque_max': 40},
    'left_wrist_yaw_joint_X4':       {'pos_min': -2.23, 'pos_max': 2.35, 'vel_max': 2.0, 'torque_max': 40},
    'left_wrist_roll_joint_X4':      {'pos_min': -1.59, 'pos_max': 1.57, 'vel_max': 2.0, 'torque_max': 40},
}

CIA402_STATES = {
    'NOT_READY_TO_SWITCH_ON': ('Not Ready', C_SUBTEXT),
    'SWITCH_ON_DISABLED':     ('Disabled', C_PEACH),
    'READY_TO_SWITCH_ON':     ('Ready', C_YELLOW),
    'SWITCHED_ON':            ('Switched On', C_YELLOW),
    'OPERATION_ENABLED':      ('Enabled', C_GREEN),
    'QUICK_STOP_ACTIVE':      ('Quick Stop', C_RED),
    'FAULT_REACTION_ACTIVE':  ('Fault React', C_RED),
    'FAULT':                  ('FAULT', C_RED),
}

DEMO_JOINTS = [
    'left_shoulder_pitch_joint_X6',
    'left_shoulder_roll_joint_X6',
    'left_shoulder_yaw_joint_X4',
    'left_elbow_pitch_joint_X6',
    'left_wrist_yaw_joint_X4',
    'left_wrist_roll_joint_X4',
    'left_thumb_proximal_yaw_joint',
    'left_index_proximal_joint',
    'right_shoulder_pitch_joint_X6',
    'right_shoulder_roll_joint_X6',
    'right_elbow_pitch_joint_X6',
    'right_wrist_roll_joint_X4',
]

TAB_NAMES = ['Joints', 'Safety', 'Controllers', 'Log']


# ═══════════════════════════════════════════════════════════
# ROBOT DATA STORE
# ═══════════════════════════════════════════════════════════

class RobotStore:
    """Thread-safe data store for all robot telemetry."""

    def __init__(self):
        self.lock = threading.Lock()

        # Joint states
        self.joints = {}
        self.joint_order = []
        self.history = {}
        self.joints_discovered = False
        self.js_msg_count = 0
        self.js_last_update = None
        self.js_rate_count_start = 0
        self.js_rate_time_start = time.monotonic()
        self.js_current_rate = 0.0

        # Mode
        self.mode = 'DEMO'
        self.mode_locked = False

        # Safety (from /safety/status and /diagnostics)
        self.safety_ok = None
        self.watchdog_ok = None
        self.estop_active = None
        self.overall_safe = None
        self.violations = []
        self.drive_states = {}      # {joint: state_str}
        self.diag_last_update = None

        # Controllers (refreshed periodically)
        self.controllers = []       # [{name, type, state}]
        self.hw_interfaces = []     # [{name, state}]
        self.ctrl_last_update = None

        # Event log
        self.event_log = deque(maxlen=500)

    def update_joints(self, joint_dict):
        with self.lock:
            self.joints = dict(joint_dict)
            self.js_last_update = time.time()
            self.js_msg_count += 1
            if not self.joints_discovered and joint_dict:
                self._discover_joints(list(joint_dict.keys()))
            for name, vals in joint_dict.items():
                if name not in self.history:
                    self.history[name] = deque(maxlen=HISTORY_LEN)
                self.history[name].append(
                    (vals['position'], vals['velocity'], vals['effort']))

    def _discover_joints(self, names):
        grouped = {}
        for name in names:
            g = _classify(name)
            grouped.setdefault(g, []).append(name)
        ordered = []
        for g in GROUP_ORDER:
            if g in grouped:
                ordered.extend(sorted(grouped[g]))
        self.joint_order = ordered
        self.joints_discovered = True

    def update_safety(self, safe):
        with self.lock:
            old = self.safety_ok
            self.safety_ok = safe
            if old is not None and old != safe:
                level = 'INFO' if safe else 'ERROR'
                self._log(level, f'Safety status: {"OK" if safe else "FAULT"}')

    def update_diagnostics(self, diag_msg):
        with self.lock:
            self.diag_last_update = time.time()
            for status in diag_msg.status:
                kv = {item.key: item.value for item in status.values}
                name_lower = status.name.lower()

                if 'watchdog' in name_lower:
                    was = self.watchdog_ok
                    self.watchdog_ok = (status.level == 0)
                    if was is not None and was and not self.watchdog_ok:
                        self._log('ERROR', f'Watchdog TIMEOUT: {status.message}')

                elif 'estop' in name_lower or 'e-stop' in name_lower:
                    active = kv.get('active', 'false').lower() == 'true'
                    was = self.estop_active
                    self.estop_active = active
                    if was is not None and not was and active:
                        self._log('ERROR', 'E-Stop ACTIVATED')
                    elif was and not active:
                        self._log('INFO', 'E-Stop released')

                elif 'joint_limits' in name_lower or 'joint_limit' in name_lower:
                    if status.level >= 2:  # ERROR
                        violations = []
                        for item in status.values:
                            violations.append(f'{item.key}={item.value}')
                        if violations and violations != self.violations:
                            self._log('WARN', f'Joint limit violations: {", ".join(violations)}')
                        self.violations = violations
                    else:
                        self.violations = []

                elif 'overall' in name_lower:
                    self.overall_safe = (status.level == 0)

                # Extract drive states
                if 'drive_state' in kv:
                    joint = kv.get('joint', status.name)
                    state = kv['drive_state']
                    old_state = self.drive_states.get(joint)
                    self.drive_states[joint] = state
                    if old_state and old_state != state:
                        ds_text, ds_color = CIA402_STATES.get(state, (state, C_SUBTEXT))
                        if state == 'FAULT':
                            self._log('ERROR', f'{joint}: drive FAULT (err={kv.get("error_code", "?")})')
                        elif state == 'OPERATION_ENABLED' and old_state != 'OPERATION_ENABLED':
                            self._log('INFO', f'{joint}: drive enabled')

    def update_controllers(self, controllers, hw_interfaces):
        with self.lock:
            self.controllers = controllers
            self.hw_interfaces = hw_interfaces
            self.ctrl_last_update = time.time()

    def log_event(self, level, message):
        with self.lock:
            self._log(level, message)

    def _log(self, level, message):
        ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        self.event_log.append((ts, level, message))

    def compute_rate(self):
        now = time.monotonic()
        dt = now - self.js_rate_time_start
        if dt >= 1.0:
            delta = self.js_msg_count - self.js_rate_count_start
            self.js_current_rate = delta / dt
            self.js_rate_count_start = self.js_msg_count
            self.js_rate_time_start = now

    def snapshot(self):
        with self.lock:
            return {
                'joints': dict(self.joints),
                'order': list(self.joint_order),
                'history': {k: list(v) for k, v in self.history.items()},
                'discovered': self.joints_discovered,
                'js_last_update': self.js_last_update,
                'rate': self.js_current_rate,
                'mode': self.mode,
                'safety_ok': self.safety_ok,
                'watchdog_ok': self.watchdog_ok,
                'estop_active': self.estop_active,
                'overall_safe': self.overall_safe,
                'violations': list(self.violations),
                'drive_states': dict(self.drive_states),
                'diag_last_update': self.diag_last_update,
                'controllers': list(self.controllers),
                'hw_interfaces': list(self.hw_interfaces),
                'ctrl_last_update': self.ctrl_last_update,
                'event_log': list(self.event_log),
            }


# ═══════════════════════════════════════════════════════════
# KEYBOARD INPUT (non-blocking)
# ═══════════════════════════════════════════════════════════

class KeyReader:
    """Non-blocking single-character keyboard reader using raw terminal mode."""

    def __init__(self):
        self._old_settings = None
        self._active = False

    def start(self):
        if not os.isatty(sys.stdin.fileno()):
            return
        try:
            self._old_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
            self._active = True
        except Exception:
            pass

    def stop(self):
        if self._old_settings is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)
            except Exception:
                pass
            self._old_settings = None
        self._active = False

    def read(self):
        """Return a key character or None if nothing pressed."""
        if not self._active:
            return None
        try:
            if select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                return ch
        except (InterruptedError, OSError):
            pass  # SIGWINCH interrupts select()
        except Exception:
            pass
        return None


# ═══════════════════════════════════════════════════════════
# TAB BUILDERS
# ═══════════════════════════════════════════════════════════

def _limit_ratio(name, pos):
    lims = JOINT_LIMITS.get(name)
    if lims is None:
        return 0.0
    lo, hi = lims
    rng = hi - lo
    if rng <= 0:
        return 0.0
    dist = min(abs(pos - lo), abs(pos - hi))
    return max(0.0, 1.0 - dist / (rng / 2.0))


def build_joints_tab(snap, demo_time=None):
    """Tab 1: Joint telemetry table."""
    is_demo = demo_time is not None
    mode = snap['mode']
    show_drive = mode == 'REAL' or is_demo

    table = Table(
        show_header=True, header_style=f'bold {C_TEXT}',
        border_style=C_SURFACE0, expand=True,
        pad_edge=False, padding=(0, 1),
    )
    table.add_column('Joint', style=C_TEXT, min_width=26, no_wrap=True)
    table.add_column('Grp', width=6, justify='center')
    table.add_column('Position', width=12, justify='right', no_wrap=True)
    table.add_column('Bar', width=17, no_wrap=True)
    table.add_column('Vel\u00b0/s', width=9, justify='right')
    table.add_column('Effort', width=9, justify='right')
    table.add_column('History', width=22, no_wrap=True)
    if show_drive:
        table.add_column('Drive', width=10, justify='center')
    table.add_column('HP', width=4, justify='center')

    for i, name in enumerate(snap['order']):
        group = _classify(name)
        grp_color = GROUP_STYLES.get(group, C_SUBTEXT)

        if is_demo:
            phase = i * 0.8
            pos = math.radians(30.0 * math.sin(demo_time * 0.5 + phase))
            vel = math.radians(15.0 * math.cos(demo_time * 0.5 + phase))
            eff = 2.0 * math.sin(demo_time * 0.3 + phase)
        elif name in snap['joints']:
            jd = snap['joints'][name]
            pos, vel, eff = jd['position'], jd['velocity'], jd['effort']
        else:
            row = [name, Text(group, style=grp_color)] + ['---'] * (6 if show_drive else 5)
            table.add_row(*row)
            continue

        pos_deg = math.degrees(pos)
        vel_deg = math.degrees(vel)

        ratio = _limit_ratio(name, pos)
        pc = severity_color(ratio)
        pos_text = Text(f'{pos_deg:+8.2f}\u00b0', style=f'bold {pc}')

        lims = JOINT_LIMITS.get(name, (-3.14, 3.14))
        bar_str = hbar(pos, lims[0], lims[1], width=15)
        bar_text = Text(bar_str, style=pc)

        vel_text = Text(f'{vel_deg:+8.2f}', style=C_TEXT)

        abs_eff = abs(eff)
        eff_style = C_RED if abs_eff >= 10.0 else (C_YELLOW if abs_eff >= 5.0 else C_TEXT)
        eff_text = Text(f'{eff:+8.3f}', style=eff_style)

        hist = snap['history'].get(name, [])
        pos_hist = [h[0] for h in hist]
        spark = sparkline(pos_hist, width=20, vmin=lims[0], vmax=lims[1])
        spark_text = Text(spark, style=C_BLUE)

        hp = Text('!!', style=f'bold {C_RED}') if abs_eff >= 10.0 \
            else (Text('! ', style=f'bold {C_YELLOW}') if abs_eff >= 5.0
                  else Text('OK', style=f'bold {C_GREEN}'))

        row_items = [
            Text(name, style=C_TEXT), Text(group, style=grp_color),
            pos_text, bar_text, vel_text, eff_text, spark_text,
        ]

        if show_drive:
            ds = snap['drive_states'].get(name)
            if is_demo and name in SAFETY_LIMITS:
                row_items.append(Text('Enabled', style=C_GREEN))
            elif ds:
                ds_text, ds_color = CIA402_STATES.get(ds, (ds, C_SUBTEXT))
                row_items.append(Text(ds_text, style=ds_color))
            else:
                row_items.append(Text('---', style=C_SUBTEXT))

        row_items.append(hp)
        table.add_row(*row_items)

    return table


def build_safety_tab(snap, demo_time=None):
    """Tab 2: Safety status and joint limit utilization."""
    is_demo = demo_time is not None
    mode = snap['mode']

    # Left panel: system status
    status_lines = Text()

    def _status_line(label, value, color):
        status_lines.append(f'  {label:<16s}', style=C_SUBTEXT)
        status_lines.append(f'{value}\n', style=f'bold {color}')

    if mode == 'SIM' and not is_demo:
        status_lines.append('\n  Safety monitoring not available in simulation mode.\n\n',
                            style=C_SUBTEXT)
        status_lines.append('  Launch real hardware to see safety data:\n', style=C_SUBTEXT)
        status_lines.append('  ros2 launch arm_real_bringup arm_real.launch.py\n',
                            style=C_BLUE)
        return Panel(status_lines, title='[bold]Safety[/]', border_style=C_SURFACE0)

    # Overall
    if is_demo:
        overall_safe = True
        watchdog_ok = True
        estop_active = False
        safety_ok = True
        violations = []
    else:
        overall_safe = snap['overall_safe']
        watchdog_ok = snap['watchdog_ok']
        estop_active = snap['estop_active']
        safety_ok = snap['safety_ok']
        violations = snap['violations']

    if overall_safe is None:
        _status_line('Overall', 'Unknown', C_SUBTEXT)
    elif overall_safe:
        _status_line('Overall', 'SYSTEM SAFE', C_GREEN)
    else:
        _status_line('Overall', 'SYSTEM UNSAFE', C_RED)

    if safety_ok is None:
        _status_line('Safety Topic', 'No data', C_SUBTEXT)
    elif safety_ok:
        _status_line('Safety Topic', 'OK', C_GREEN)
    else:
        _status_line('Safety Topic', 'FAULT', C_RED)

    if watchdog_ok is None:
        _status_line('Watchdog', 'Unknown', C_SUBTEXT)
    elif watchdog_ok:
        _status_line('Watchdog', 'OK', C_GREEN)
    else:
        _status_line('Watchdog', 'TIMEOUT', C_RED)

    if estop_active is None:
        _status_line('E-Stop', 'Unknown', C_SUBTEXT)
    elif estop_active:
        _status_line('E-Stop', 'ACTIVE', C_RED)
    else:
        _status_line('E-Stop', 'CLEAR', C_GREEN)

    _status_line('Violations', str(len(violations)), C_RED if violations else C_GREEN)

    status_lines.append('\n')
    _status_line('Cmd Timeout', '50 ms', C_TEXT)
    _status_line('Reaction', 'quickstop', C_TEXT)

    left_panel = Panel(status_lines, title='[bold]System Status[/]',
                       border_style=C_SURFACE0)

    # Right panel: per-joint limit utilization
    limit_table = Table(
        show_header=True, header_style=f'bold {C_TEXT}',
        border_style=C_SURFACE0, expand=True,
        pad_edge=False, padding=(0, 1),
    )
    limit_table.add_column('Joint', min_width=18, no_wrap=True)
    limit_table.add_column('Pos %', width=12, no_wrap=True)
    limit_table.add_column('Vel %', width=12, no_wrap=True)
    limit_table.add_column('Torq %', width=12, no_wrap=True)

    for i, name in enumerate(snap['order']):
        sl = SAFETY_LIMITS.get(name)
        if sl is None:
            continue

        if is_demo:
            phase = i * 0.8
            pos = math.radians(30.0 * math.sin(demo_time * 0.5 + phase))
            vel = math.radians(15.0 * math.cos(demo_time * 0.5 + phase))
            eff = 2.0 * math.sin(demo_time * 0.3 + phase)
        elif name in snap['joints']:
            jd = snap['joints'][name]
            pos, vel, eff = jd['position'], jd['velocity'], jd['effort']
        else:
            continue

        # Position ratio
        pos_range = sl['pos_max'] - sl['pos_min']
        if pos_range > 0:
            dist = min(abs(pos - sl['pos_min']), abs(pos - sl['pos_max']))
            pos_ratio = max(0.0, 1.0 - dist / (pos_range / 2.0))
        else:
            pos_ratio = 0.0

        vel_ratio = min(1.0, abs(vel) / sl['vel_max']) if sl['vel_max'] > 0 else 0.0
        torq_ratio = min(1.0, abs(eff) / sl['torque_max']) if sl['torque_max'] > 0 else 0.0

        short = name.replace('left_', 'L ').replace('right_', 'R ') \
                     .replace('_joint_X6', ' X6').replace('_joint_X4', ' X4') \
                     .replace('_joint', '').replace('_', ' ').title()

        def _bar_cell(r):
            b = ratio_bar(r, width=8)
            c = severity_color(r)
            return Text(f'{b} {r*100:3.0f}%', style=c)

        limit_table.add_row(
            Text(short, style=C_TEXT),
            _bar_cell(pos_ratio),
            _bar_cell(vel_ratio),
            _bar_cell(torq_ratio),
        )

    right_panel = Panel(limit_table, title='[bold]Joint Limits[/]',
                        border_style=C_SURFACE0)

    # Two-column layout
    layout = Layout()
    layout.split_row(
        Layout(left_panel, ratio=2),
        Layout(right_panel, ratio=3),
    )
    return layout


def build_controllers_tab(snap):
    """Tab 3: ros2_control controller and hardware interface status."""
    controllers = snap['controllers']
    hw_interfaces = snap['hw_interfaces']
    ctrl_time = snap['ctrl_last_update']

    # Controllers table
    ctrl_table = Table(
        show_header=True, header_style=f'bold {C_TEXT}',
        border_style=C_SURFACE0, expand=True,
        pad_edge=False, padding=(0, 1),
    )
    ctrl_table.add_column('Controller', min_width=25, no_wrap=True)
    ctrl_table.add_column('Type', min_width=20, no_wrap=True)
    ctrl_table.add_column('State', width=12, justify='center')

    if not controllers:
        if ctrl_time is None:
            ctrl_table.add_row(
                Text('Querying...', style=C_SUBTEXT), '', '')
        else:
            ctrl_table.add_row(
                Text('No controllers found', style=C_SUBTEXT), '', '')
    else:
        for c in controllers:
            state = c.get('state', '?')
            if state == 'active':
                state_text = Text('active', style=f'bold {C_GREEN}')
            elif state == 'inactive':
                state_text = Text('inactive', style=C_YELLOW)
            else:
                state_text = Text(state, style=C_SUBTEXT)
            ctrl_table.add_row(
                Text(c.get('name', '?'), style=C_TEXT),
                Text(c.get('type', '?'), style=C_SUBTEXT),
                state_text,
            )

    left_panel = Panel(ctrl_table, title='[bold]Controllers[/]',
                       border_style=C_SURFACE0)

    # Hardware interfaces
    hw_table = Table(
        show_header=True, header_style=f'bold {C_TEXT}',
        border_style=C_SURFACE0, expand=True,
        pad_edge=False, padding=(0, 1),
    )
    hw_table.add_column('Interface', min_width=30, no_wrap=True)
    hw_table.add_column('State', width=12, justify='center')

    if not hw_interfaces:
        hw_table.add_row(Text('Querying...', style=C_SUBTEXT), '')
    else:
        for hw in hw_interfaces:
            state = hw.get('state', '?')
            if state in ('active', 'available'):
                st = Text(state, style=C_GREEN)
            elif state == 'claimed':
                st = Text(state, style=C_BLUE)
            elif state == 'unclaimed':
                st = Text(state, style=C_YELLOW)
            else:
                st = Text(state, style=C_SUBTEXT)
            hw_table.add_row(Text(hw.get('name', '?'), style=C_TEXT), st)

    right_panel = Panel(hw_table, title='[bold]Hardware Interfaces[/]',
                        border_style=C_SURFACE0)

    age = ''
    if ctrl_time:
        ago = time.time() - ctrl_time
        age = f' (updated {ago:.0f}s ago)'

    layout = Layout()
    layout.split_row(
        Layout(left_panel, ratio=1),
        Layout(right_panel, ratio=1),
    )
    return layout


def build_log_tab(snap):
    """Tab 4: Event log."""
    log = snap['event_log']
    text = Text()

    if not log:
        text.append('  No events yet.\n', style=C_SUBTEXT)
    else:
        # Show last entries that fit
        for ts, level, msg in log:
            if level == 'ERROR':
                color = C_RED
                tag = 'ERR '
            elif level == 'WARN':
                color = C_YELLOW
                tag = 'WARN'
            elif level == 'ACTION':
                color = C_BLUE
                tag = 'ACT '
            else:
                color = C_GREEN
                tag = 'INFO'
            text.append(f' [{ts}] ', style=C_SUBTEXT)
            text.append(f'{tag} ', style=f'bold {color}')
            text.append(f'{msg}\n', style=color)

    return Panel(text, title='[bold]Event Log[/]', border_style=C_SURFACE0)


# ═══════════════════════════════════════════════════════════
# DASHBOARD ASSEMBLY
# ═══════════════════════════════════════════════════════════

def build_dashboard(snap, start_time, active_tab, demo_time=None):
    is_demo = demo_time is not None
    mode = snap['mode']
    elapsed = time.monotonic() - start_time
    h, m, s = int(elapsed // 3600), int((elapsed % 3600) // 60), int(elapsed % 60)

    # ── Header line 1 ───────────────────────────────────────
    header1 = Text()
    header1.append('  HARAMBE ', style=f'bold {C_LAVENDER}')

    if is_demo:
        header1.append(' DEMO ', style=f'bold on {C_YELLOW} #1e1e2e')
    elif snap['js_last_update'] is None:
        header1.append(' WAITING ', style=f'bold on {C_SUBTEXT} #1e1e2e')
    elif (time.time() - snap['js_last_update']) < STALE_TIMEOUT:
        header1.append(' CONNECTED ', style=f'bold on {C_GREEN} #1e1e2e')
    else:
        header1.append(' STALE ', style=f'bold on {C_RED} #1e1e2e')

    mode_colors = {'DEMO': C_YELLOW, 'SIM': C_BLUE, 'REAL': C_GREEN}
    header1.append(f'  Mode: ', style=C_SUBTEXT)
    header1.append(mode, style=f'bold {mode_colors.get(mode, C_TEXT)}')
    header1.append(f'  Joints: ', style=C_SUBTEXT)
    header1.append(f'{len(snap["order"])}', style=f'bold {C_TEXT}')
    header1.append(f'  Rate: ', style=C_SUBTEXT)
    rate = snap['rate']
    header1.append(f'{rate:.0f} Hz' if rate > 0 else '-- Hz', style=f'bold {C_TEXT}')

    # ── Header line 2 ───────────────────────────────────────
    header2 = Text()
    header2.append('  ', style=C_TEXT)

    # Safety indicators
    safety_ok = snap['safety_ok']
    if safety_ok is None:
        header2.append('Safety: ', style=C_SUBTEXT)
        header2.append('N/A  ', style=C_SUBTEXT)
    elif safety_ok:
        header2.append('Safety: ', style=C_SUBTEXT)
        header2.append('OK  ', style=f'bold {C_GREEN}')
    else:
        header2.append('Safety: ', style=C_SUBTEXT)
        header2.append('FAULT  ', style=f'bold {C_RED}')

    estop = snap['estop_active']
    if estop is None:
        header2.append('E-Stop: ', style=C_SUBTEXT)
        header2.append('N/A  ', style=C_SUBTEXT)
    elif estop:
        header2.append('E-Stop: ', style=C_SUBTEXT)
        header2.append('ACTIVE  ', style=f'bold {C_RED}')
    else:
        header2.append('E-Stop: ', style=C_SUBTEXT)
        header2.append('CLEAR  ', style=f'bold {C_GREEN}')

    wd = snap['watchdog_ok']
    if wd is None:
        header2.append('Watchdog: ', style=C_SUBTEXT)
        header2.append('N/A  ', style=C_SUBTEXT)
    elif wd:
        header2.append('Watchdog: ', style=C_SUBTEXT)
        header2.append('OK  ', style=f'bold {C_GREEN}')
    else:
        header2.append('Watchdog: ', style=C_SUBTEXT)
        header2.append('TIMEOUT  ', style=f'bold {C_RED}')

    header2.append(f'  Uptime: ', style=C_SUBTEXT)
    header2.append(f'{h:02d}:{m:02d}:{s:02d}', style=f'bold {C_TEXT}')

    header = Text()
    header.append_text(header1)
    header.append('\n')
    header.append_text(header2)

    # ── Tab bar ─────────────────────────────────────────────
    tab_bar = Text()
    tab_bar.append('  ')
    for i, name in enumerate(TAB_NAMES):
        key = str(i + 1)
        if i == active_tab:
            tab_bar.append(f' {key} {name} ', style=f'bold on {C_SURFACE1} {C_TEXT}')
        else:
            tab_bar.append(f' {key} {name} ', style=f'{C_SUBTEXT}')
        tab_bar.append(' ')

    tab_bar.append('       ', style=C_SUBTEXT)
    tab_bar.append('e', style=f'bold {C_RED}')
    tab_bar.append(':e-stop  ', style=C_SUBTEXT)
    tab_bar.append('r', style=f'bold {C_GREEN}')
    tab_bar.append(':reset  ', style=C_SUBTEXT)
    tab_bar.append('q', style=f'bold {C_TEXT}')
    tab_bar.append(':quit', style=C_SUBTEXT)

    # ── Active tab content ──────────────────────────────────
    if active_tab == 0:
        content = build_joints_tab(snap, demo_time)
    elif active_tab == 1:
        content = build_safety_tab(snap, demo_time)
    elif active_tab == 2:
        content = build_controllers_tab(snap)
    elif active_tab == 3:
        content = build_log_tab(snap)
    else:
        content = Text('Unknown tab', style=C_RED)

    # ── Assemble ────────────────────────────────────────────
    layout = Layout()
    layout.split_column(
        Layout(Panel(header, border_style=C_SURFACE0), size=4),
        Layout(Panel(tab_bar, border_style=C_SURFACE0), size=3),
        Layout(content, ratio=1),
    )
    return layout


# ═══════════════════════════════════════════════════════════
# CONTROLLER QUERY (subprocess)
# ═══════════════════════════════════════════════════════════

def _query_controllers():
    """Parse output of ros2 control list_controllers."""
    controllers = []
    try:
        result = subprocess.run(
            ['ros2', 'control', 'list_controllers'],
            capture_output=True, text=True, timeout=3)
        if result.returncode == 0:
            for line in result.stdout.strip().split('\n'):
                line = line.strip()
                if not line:
                    continue
                # Format: "controller_name [type] [state]"
                parts = line.split()
                if len(parts) >= 3:
                    # Extract bracketed values
                    name = parts[0]
                    # Find type and state in brackets
                    brackets = re.findall(r'\[([^\]]+)\]', line)
                    if len(brackets) >= 2:
                        controllers.append({
                            'name': name,
                            'type': brackets[0],
                            'state': brackets[1],
                        })
                    elif len(brackets) == 1:
                        controllers.append({
                            'name': name,
                            'type': brackets[0],
                            'state': '?',
                        })
    except Exception:
        pass
    return controllers


def _query_hw_interfaces():
    """Parse output of ros2 control list_hardware_interfaces."""
    interfaces = []
    try:
        result = subprocess.run(
            ['ros2', 'control', 'list_hardware_interfaces'],
            capture_output=True, text=True, timeout=3)
        if result.returncode == 0:
            for line in result.stdout.strip().split('\n'):
                line = line.strip()
                if not line:
                    continue
                # Format: "interface_name [state]"
                brackets = re.findall(r'\[([^\]]+)\]', line)
                name = line.split('[')[0].strip() if '[' in line else line
                state = brackets[0] if brackets else '?'
                if name:
                    interfaces.append({'name': name, 'state': state})
    except Exception:
        pass
    return interfaces


def _controller_refresh_loop(store, is_running):
    """Background thread that refreshes controller info every 5 seconds."""
    while is_running():
        try:
            ctrls = _query_controllers()
            hw = _query_hw_interfaces()
            store.update_controllers(ctrls, hw)
        except Exception:
            pass
        # Sleep in small increments so we can exit quickly
        for _ in range(50):
            if not is_running():
                return
            time.sleep(0.1)


# ═══════════════════════════════════════════════════════════
# DEMO DATA GENERATOR
# ═══════════════════════════════════════════════════════════

def generate_demo(store, t):
    joints = {}
    for i, name in enumerate(DEMO_JOINTS):
        phase = i * 0.8
        joints[name] = {
            'position': math.radians(30.0 * math.sin(t * 0.5 + phase)),
            'velocity': math.radians(15.0 * math.cos(t * 0.5 + phase)),
            'effort': 2.0 * math.sin(t * 0.3 + phase),
        }
    store.update_joints(joints)

    # Fake safety data
    with store.lock:
        store.safety_ok = True
        store.watchdog_ok = True
        store.estop_active = False
        store.overall_safe = True
        store.violations = []
        for name in DEMO_JOINTS:
            if name in SAFETY_LIMITS:
                store.drive_states[name] = 'OPERATION_ENABLED'

    # Fake controllers
    if not store.controllers:
        store.update_controllers([
            {'name': 'joint_state_broadcaster', 'type': 'joint_state_broadcaster/JointStateBroadcaster', 'state': 'active'},
            {'name': 'left_arm_controller', 'type': 'joint_trajectory_controller/JointTrajectoryController', 'state': 'active'},
        ], [
            {'name': 'left_shoulder_pitch_joint_X6/position', 'state': 'claimed'},
            {'name': 'left_shoulder_pitch_joint_X6/velocity', 'state': 'available'},
            {'name': 'left_shoulder_pitch_joint_X6/effort', 'state': 'available'},
        ])

    # Random events
    if int(t * 10) % 100 == 0:
        store.log_event('INFO', 'System nominal — all joints within limits')


# ═══════════════════════════════════════════════════════════
# MODE DETECTION
# ═══════════════════════════════════════════════════════════

def _detect_mode(ros_node, store):
    """Check available topics to determine SIM vs REAL."""
    if store.mode_locked:
        return
    try:
        topics = [t[0] for t in ros_node.get_topic_names_and_types()]
        ethercat_topics = {'/ethercat/raw_positions', '/safety/status', '/diagnostics'}
        if ethercat_topics & set(topics):
            store.mode = 'REAL'
            store.mode_locked = True
            store.log_event('INFO', 'Mode: REAL hardware detected')
        elif '/joint_states' in topics:
            store.mode = 'SIM'
    except Exception:
        pass


# ═══════════════════════════════════════════════════════════
# ROS SPIN
# ═══════════════════════════════════════════════════════════

def _ros_spin(node, is_running):
    import rclpy
    while is_running():
        try:
            rclpy.spin_once(node, timeout_sec=0.05)
        except Exception:
            break


# ═══════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════

def main():
    demo_mode = '--demo' in sys.argv
    store = RobotStore()
    start_time = time.monotonic()
    console = Console()
    running = True
    active_tab = 0

    def _sigint(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _sigint)

    # Ignore SIGWINCH — we query terminal size each frame
    if hasattr(signal, 'SIGWINCH'):
        signal.signal(signal.SIGWINCH, lambda s, f: None)

    ros_node = None
    estop_client = None
    reset_client = None

    if not demo_mode:
        try:
            import rclpy
            from sensor_msgs.msg import JointState
            from std_msgs.msg import Bool
            from diagnostic_msgs.msg import DiagnosticArray
            from std_srvs.srv import Trigger

            rclpy.init(args=sys.argv)
            ros_node = rclpy.create_node('robot_dashboard_tui')

            # Joint states (always)
            def _js_cb(msg):
                joints = {}
                for i, name in enumerate(msg.name):
                    joints[name] = {
                        'position': msg.position[i] if i < len(msg.position) else 0.0,
                        'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                        'effort': msg.effort[i] if i < len(msg.effort) else 0.0,
                    }
                store.update_joints(joints)

            ros_node.create_subscription(JointState, '/joint_states', _js_cb, 10)

            # Safety (will work once topic appears)
            def _safety_cb(msg):
                store.update_safety(msg.data)

            ros_node.create_subscription(Bool, '/safety/status', _safety_cb, 10)

            # Diagnostics
            def _diag_cb(msg):
                store.update_diagnostics(msg)

            ros_node.create_subscription(DiagnosticArray, '/diagnostics', _diag_cb, 10)

            # Services
            estop_client = ros_node.create_client(Trigger, '/safety/estop')
            reset_client = ros_node.create_client(Trigger, '/safety/reset')

            store.mode = 'SIM'
            store.log_event('INFO', 'ROS 2 initialized, waiting for topics...')

            # Spin thread
            threading.Thread(
                target=_ros_spin, args=(ros_node, lambda: running),
                daemon=True).start()

            # Mode detection thread
            def _mode_loop():
                while running and not store.mode_locked:
                    _detect_mode(ros_node, store)
                    time.sleep(1.0)

            threading.Thread(target=_mode_loop, daemon=True).start()

            # Controller refresh thread
            threading.Thread(
                target=_controller_refresh_loop,
                args=(store, lambda: running),
                daemon=True).start()

        except Exception as e:
            console.print(f'[{C_YELLOW}]ROS 2 not available ({e}), running in demo mode[/]')
            demo_mode = True
            time.sleep(1)

    if demo_mode:
        store.mode = 'DEMO'
        store.mode_locked = True
        store.log_event('INFO', 'Demo mode — no ROS 2 connection')

    demo_time = 0.0
    key_reader = KeyReader()
    key_reader.start()

    # Enter alternate screen
    sys.stdout.write('\033[?1049h')  # Switch to alt screen
    sys.stdout.write('\033[?25l')    # Hide cursor
    sys.stdout.flush()

    try:
        while running:
            # Handle keyboard
            ch = key_reader.read()
            if ch == 'q' or ch == '\x03':  # q or Ctrl+C
                running = False
                break
            elif ch in ('1', '2', '3', '4'):
                active_tab = int(ch) - 1
            elif ch == 'e':
                if estop_client and estop_client.service_is_ready():
                    estop_client.call_async(Trigger.Request())
                    store.log_event('ACTION', 'Software E-STOP triggered by operator')
                else:
                    store.log_event('WARN', 'E-Stop service not available')
            elif ch == 'r':
                if reset_client and reset_client.service_is_ready():
                    reset_client.call_async(Trigger.Request())
                    store.log_event('ACTION', 'Safety RESET requested by operator')
                else:
                    store.log_event('WARN', 'Reset service not available')

            # Generate demo data
            if demo_mode:
                demo_time += 0.1
                generate_demo(store, demo_time)

            # Compute rate
            store.compute_rate()

            # Build and render
            try:
                # Get current terminal size
                term_w, term_h = os.get_terminal_size()

                snap = store.snapshot()
                dashboard = build_dashboard(
                    snap, start_time, active_tab,
                    demo_time=demo_time if demo_mode else None)

                # Render to buffer using exact terminal width
                buf = io.StringIO()
                buf_console = Console(
                    file=buf, width=term_w, height=term_h,
                    force_terminal=True, color_system='truecolor')
                buf_console.print(dashboard)
                frame = buf.getvalue()

                # Atomic write: cursor home + frame + clear below
                sys.stdout.write(
                    '\033[H'    # cursor to 1,1
                    + frame
                    + '\033[J'  # erase from cursor to end of screen
                )
                sys.stdout.flush()
            except Exception:
                pass  # Transient render error during resize
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # Restore terminal
        sys.stdout.write('\033[?25h')    # Show cursor
        sys.stdout.write('\033[?1049l')  # Exit alt screen
        sys.stdout.flush()
        key_reader.stop()
        if ros_node is not None:
            ros_node.destroy_node()
            import rclpy
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
