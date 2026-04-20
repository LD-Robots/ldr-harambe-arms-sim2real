#!/usr/bin/env python3
"""EtherCAT startup runtime monitor.

Monitors joint positions during the startup window between EtherCAT drive
enable and controller activation. Logs warnings when joints move before
their controller is active — indicating missing/bad PDO default values
or other startup sequencing issues.

Auto-terminates after all position controllers are active or after a
configurable timeout.
"""

import re
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState
from sensor_msgs.msg import JointState

# Position controllers and their joints (must match controllers.yaml)
CONTROLLER_JOINTS = {
    "left_arm_trajectory_controller": [
        "left_shoulder_pitch_joint_X6",
        "left_shoulder_roll_joint_X6",
        "left_shoulder_yaw_joint_X4",
        "left_elbow_pitch_joint_X6",
        "left_wrist_yaw_joint_X4",
        "left_wrist_roll_joint_X4",
    ],
    "right_arm_trajectory_controller": [
        "right_shoulder_pitch_joint_X6",
        "right_shoulder_roll_joint_X6",
        "right_shoulder_yaw_joint_X4",
        "right_elbow_pitch_joint_X6",
        "right_wrist_yaw_joint_X4",
        "right_wrist_roll_joint_X4",
    ],
    "waist_controller": [
        "waist_yaw_joint",
    ],
    "left_leg_trajectory_controller": [
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
    ],
    "right_leg_trajectory_controller": [
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_joint",
        "right_ankle_roll_joint",
    ],
}

# All monitored joints (flat set for quick lookup)
ALL_MONITORED_JOINTS = set()
for joints in CONTROLLER_JOINTS.values():
    ALL_MONITORED_JOINTS.update(joints)

# Reverse map: joint -> controller
JOINT_TO_CONTROLLER = {}
for ctrl, joints in CONTROLLER_JOINTS.items():
    for j in joints:
        JOINT_TO_CONTROLLER[j] = ctrl

# Joint to log ALL diagnostics for (verbose debug)
DIAG_JOINT = "left_shoulder_yaw_joint_X4"

# CiA 402 status_word bit decoding
CIA402_STATES = {
    0x0000: "NOT_READY_TO_SWITCH_ON",
    0x0040: "SWITCH_ON_DISABLED",
    0x0021: "READY_TO_SWITCH_ON",
    0x0023: "SWITCHED_ON",
    0x0027: "OPERATION_ENABLED",
    0x0007: "QUICK_STOP_ACTIVE",
    0x000F: "FAULT_REACTION_ACTIVE",
    0x0008: "FAULT",
}

def decode_cia402_state(status_word):
    """Decode CiA 402 state from status_word bits [0..6]."""
    masked = int(status_word) & 0x006F
    return CIA402_STATES.get(masked, f"UNKNOWN(0x{masked:04X})")


class StartupMonitorNode(Node):
    def __init__(self):
        super().__init__("startup_monitor")

        # Parameters
        self.declare_parameter("jump_threshold", 0.05)  # rad per callback
        self.declare_parameter("drift_threshold", 0.02)  # rad total before ctrl
        self.declare_parameter("monitor_duration", 30.0)  # seconds
        self.declare_parameter("stale_timeout", 2.0)  # seconds

        self._jump_thresh = self.get_parameter("jump_threshold").value
        self._drift_thresh = self.get_parameter("drift_threshold").value
        self._monitor_dur = self.get_parameter("monitor_duration").value
        self._stale_timeout = self.get_parameter("stale_timeout").value

        self._lock = threading.Lock()
        self._start_time = time.monotonic()

        # Joint state tracking
        self._initial_positions = {}   # joint -> first observed position
        self._previous_positions = {}  # joint -> last observed position
        self._current_positions = {}   # joint -> current position
        self._last_js_time = None      # monotonic time of last /joint_states

        # Controller state tracking
        self._active_controllers = set()
        self._controller_activation_times = {}  # controller -> monotonic time
        self._joints_with_active_ctrl = set()

        # Drift reports (only report once per joint)
        self._reported_drift_joints = set()
        self._reported_jump_joints = set()

        # Stale warning state
        self._stale_warned = False

        # --- Verbose diag state for DIAG_JOINT ---
        self._diag_last_cia402 = None       # last decoded CiA 402 state string
        self._diag_last_status_word = None   # raw status_word
        self._diag_last_mode = None          # mode_of_operation display
        self._diag_last_pos_raw = None       # position from /joint_states_raw
        self._diag_last_pos = None           # position from /joint_states
        self._diag_msg_count = 0             # /dynamic_joint_states message count
        self._diag_js_raw_count = 0          # /joint_states_raw message count

        self.get_logger().info(
            f"Startup monitor active — jump_threshold={self._jump_thresh:.3f} rad, "
            f"drift_threshold={self._drift_thresh:.3f} rad, "
            f"timeout={self._monitor_dur:.0f}s"
        )
        self.get_logger().info(
            f"DIAG: Verbose logging enabled for '{DIAG_JOINT}'"
        )

        # Subscribe to /joint_states (merged by joint_state_publisher)
        self._state_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )

        # Subscribe to /joint_states_raw (direct from EtherCAT, before merge)
        self._raw_sub = self.create_subscription(
            JointState, "/joint_states_raw", self._joint_state_raw_cb, 10
        )

        # Subscribe to /dynamic_joint_states for status_word + mode
        self._dyn_sub = self.create_subscription(
            DynamicJointState, "/dynamic_joint_states", self._dynamic_joint_state_cb, 10
        )

        # 1 Hz timer to poll controller states
        self._ctrl_timer = self.create_timer(1.0, self._poll_controllers)

        # 2 Hz timer for stale detection and timeout
        self._watchdog_timer = self.create_timer(0.5, self._watchdog_cb)

    def _joint_state_cb(self, msg: JointState):
        now = time.monotonic()
        elapsed = now - self._start_time

        with self._lock:
            self._last_js_time = now
            self._stale_warned = False

            for name, pos in zip(msg.name, msg.position):
                if name not in ALL_MONITORED_JOINTS:
                    continue

                self._current_positions[name] = pos

                # First time seeing this joint
                if name not in self._initial_positions:
                    self._initial_positions[name] = pos
                    self._previous_positions[name] = pos
                    if name == DIAG_JOINT:
                        self.get_logger().info(
                            f"DIAG JS [{DIAG_JOINT}] T+{elapsed:.2f}s  "
                            f"FIRST pos={pos:.6f}"
                        )
                    continue

                # Check for jumps only if controller is NOT yet active
                if name not in self._joints_with_active_ctrl:
                    prev = self._previous_positions[name]
                    delta = abs(pos - prev)

                    # Verbose logging for diag joint
                    if name == DIAG_JOINT:
                        prev_diag = self._diag_last_pos
                        diag_delta = abs(pos - prev_diag) if prev_diag is not None else 0.0
                        if elapsed < 5.0 or diag_delta > 0.0005:
                            self.get_logger().info(
                                f"DIAG JS [{DIAG_JOINT}] T+{elapsed:.2f}s  "
                                f"pos={pos:.6f}  delta={delta:.6f}  "
                                f"total_drift={abs(pos - self._initial_positions[name]):.6f}  "
                                f"ctrl_active={name in self._joints_with_active_ctrl}"
                            )
                        self._diag_last_pos = pos

                    if delta > self._jump_thresh and name not in self._reported_jump_joints:
                        ctrl = JOINT_TO_CONTROLLER.get(name, "unknown")
                        self.get_logger().warn(
                            f"Joint {name} jumped {delta:.4f} rad "
                            f"(pos={pos:.4f}) at T+{elapsed:.1f}s "
                            f"— controller '{ctrl}' NOT yet active"
                        )
                        self._reported_jump_joints.add(name)

                self._previous_positions[name] = pos

    def _joint_state_raw_cb(self, msg: JointState):
        """Log every /joint_states_raw message for DIAG_JOINT."""
        now = time.monotonic()
        elapsed = now - self._start_time

        for name, pos in zip(msg.name, msg.position):
            if name != DIAG_JOINT:
                continue

            self._diag_js_raw_count += 1
            vel = 0.0
            eff = 0.0
            idx = list(msg.name).index(name)
            if idx < len(msg.velocity):
                vel = msg.velocity[idx]
            if idx < len(msg.effort):
                eff = msg.effort[idx]

            # Log every message for the first 5s, then only on position change
            prev = self._diag_last_pos_raw
            delta = abs(pos - prev) if prev is not None else 0.0
            if elapsed < 5.0 or delta > 0.0005 or self._diag_js_raw_count <= 10:
                self.get_logger().info(
                    f"DIAG RAW [{DIAG_JOINT}] T+{elapsed:.2f}s  "
                    f"pos={pos:.6f} vel={vel:.6f} eff={eff:.4f}  "
                    f"delta={delta:.6f}  msg#{self._diag_js_raw_count}"
                )
            self._diag_last_pos_raw = pos

    def _dynamic_joint_state_cb(self, msg: DynamicJointState):
        """Log CiA 402 state transitions and all interfaces for DIAG_JOINT."""
        now = time.monotonic()
        elapsed = now - self._start_time

        if DIAG_JOINT not in msg.joint_names:
            return

        idx = msg.joint_names.index(DIAG_JOINT)
        iv = msg.interface_values[idx]

        # Build dict of interface_name -> value
        ifaces = dict(zip(iv.interface_names, iv.values))

        status_word = ifaces.get("status_word", None)
        mode_display = None
        position = ifaces.get("position", None)
        velocity = ifaces.get("velocity", None)
        effort = ifaces.get("effort", None)

        # Find mode_of_operation display if present (interface name varies)
        for k, v in ifaces.items():
            if "mode" in k.lower():
                mode_display = v

        self._diag_msg_count += 1

        # Decode CiA 402 state
        cia402 = decode_cia402_state(status_word) if status_word is not None else "N/A"

        # Log on state change, mode change, or first 10 messages, or every message in first 5s
        state_changed = (cia402 != self._diag_last_cia402)
        mode_changed = (mode_display != self._diag_last_mode)
        sw_changed = (status_word != self._diag_last_status_word)

        if state_changed or mode_changed or sw_changed or elapsed < 5.0 or self._diag_msg_count <= 10:
            sw_hex = f"0x{int(status_word):04X}" if status_word is not None else "N/A"
            mode_str = f"{int(mode_display)}" if mode_display is not None else "N/A"

            prefix = ""
            if state_changed and self._diag_last_cia402 is not None:
                prefix = f"*** STATE CHANGE: {self._diag_last_cia402} -> {cia402} *** "
            if mode_changed and self._diag_last_mode is not None:
                old_mode = int(self._diag_last_mode) if self._diag_last_mode is not None else "?"
                prefix += f"*** MODE CHANGE: {old_mode} -> {mode_str} *** "

            self.get_logger().info(
                f"DIAG DYN [{DIAG_JOINT}] T+{elapsed:.2f}s  "
                f"{prefix}"
                f"cia402={cia402}  sw={sw_hex}  mode={mode_str}  "
                f"pos={position:.6f if position is not None else 'N/A'}  "
                f"vel={velocity:.6f if velocity is not None else 'N/A'}  "
                f"eff={effort:.4f if effort is not None else 'N/A'}  "
                f"msg#{self._diag_msg_count}"
            )

            # Log all interfaces on state change for full picture
            if state_changed or mode_changed:
                iface_str = "  ".join(f"{k}={v:.6f}" for k, v in ifaces.items())
                self.get_logger().info(
                    f"DIAG DYN [{DIAG_JOINT}] ALL INTERFACES: {iface_str}"
                )

        self._diag_last_cia402 = cia402
        self._diag_last_status_word = status_word
        self._diag_last_mode = mode_display

    def _poll_controllers(self):
        """Query ros2 control list_controllers and detect activations."""
        try:
            result = subprocess.run(
                ["ros2", "control", "list_controllers"],
                capture_output=True, text=True, timeout=3,
            )
            if result.returncode != 0:
                return
        except Exception:
            return

        now = time.monotonic()
        elapsed = now - self._start_time

        for line in result.stdout.strip().split("\n"):
            if not line.strip():
                continue
            parts = line.split()
            if len(parts) < 1:
                continue
            ctrl_name = parts[0]
            if ctrl_name not in CONTROLLER_JOINTS:
                continue

            brackets = re.findall(r"\[([^\]]+)\]", line)
            if len(brackets) < 2:
                continue
            state = brackets[1]  # e.g. 'active', 'inactive'

            if state == "active" and ctrl_name not in self._active_controllers:
                self._active_controllers.add(ctrl_name)
                self._controller_activation_times[ctrl_name] = elapsed
                self.get_logger().info(
                    f"{ctrl_name} activated at T+{elapsed:.1f}s"
                )

                # Check drift for each joint in this controller
                with self._lock:
                    for joint in CONTROLLER_JOINTS[ctrl_name]:
                        self._joints_with_active_ctrl.add(joint)

                        if joint in self._initial_positions and joint in self._current_positions:
                            drift = abs(
                                self._current_positions[joint]
                                - self._initial_positions[joint]
                            )
                            if drift > self._drift_thresh and joint not in self._reported_drift_joints:
                                self.get_logger().error(
                                    f"{joint} drifted {drift:.4f} rad "
                                    f"before controller activation"
                                )
                                self._reported_drift_joints.add(joint)

        # Check if all controllers are active
        if self._active_controllers >= set(CONTROLLER_JOINTS.keys()):
            self._finish("All position controllers active.")

    def _watchdog_cb(self):
        """Check for stale joint states and monitor timeout."""
        now = time.monotonic()
        elapsed = now - self._start_time

        # Timeout
        if elapsed > self._monitor_dur:
            missing = set(CONTROLLER_JOINTS.keys()) - self._active_controllers
            if missing:
                self.get_logger().warn(
                    f"Monitor timeout — controllers never activated: "
                    f"{', '.join(sorted(missing))}"
                )
            self._finish("Monitor timeout reached.")
            return

        # Stale joint states
        with self._lock:
            last = self._last_js_time

        if last is not None and (now - last) > self._stale_timeout:
            if not self._stale_warned:
                self.get_logger().warn(
                    f"/joint_states stale for {now - last:.1f}s "
                    f"at T+{elapsed:.1f}s"
                )
                self._stale_warned = True

    def _finish(self, reason: str):
        """Log summary and request shutdown."""
        elapsed = time.monotonic() - self._start_time

        # Summary
        lines = [f"Startup monitor complete ({reason})"]
        lines.append(f"  Duration: {elapsed:.1f}s")

        if self._controller_activation_times:
            lines.append("  Controller activation timeline:")
            for ctrl, t in sorted(
                self._controller_activation_times.items(), key=lambda x: x[1]
            ):
                lines.append(f"    {ctrl}: T+{t:.1f}s")

        if self._reported_drift_joints:
            lines.append(
                f"  Joints with pre-controller drift: "
                f"{', '.join(sorted(self._reported_drift_joints))}"
            )
        else:
            lines.append("  No pre-controller drift detected.")

        self.get_logger().info("\n".join(lines))

        # Stop timers and shutdown
        self._ctrl_timer.cancel()
        self._watchdog_timer.cancel()
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = StartupMonitorNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
