# Monitoring & Diagnostic Tools

Real-time monitoring and diagnostic GUI/TUI tools for the dual-arm system. All tools use the **Catppuccin Mocha** dark theme and support three operating modes:

| Mode | Condition | Color |
|------|-----------|-------|
| **DEMO** | No ROS available | Yellow |
| **SIM** | `/joint_states` only (Gazebo) | Blue |
| **REAL** | EtherCAT topics present (`/ethercat/raw_positions`, `/safety/status`) | Green |

Mode is auto-detected. Tools gracefully degrade to DEMO with sinusoidal test data when ROS is unavailable.

---

## `ethercat_tools` Package

Standalone PyQt5 tools. Run directly from the command line (no `ros2 run` needed after sourcing).

### `demo_ethercat_status` — EtherCAT Status Monitor

Real-time view of EtherCAT master and 6 left-arm actuator slaves.

```bash
demo_ethercat_status
```

**Features:**
- Master status bar: uptime, cycle rate (1000 Hz), slave count, mode indicator
- Actuator table (10 columns): joint name, motor type (X4/X6), slave #, CiA 402 state, position (deg), velocity (deg/s), torque (Nm), status word (hex), error code (hex), health
- Safety table: per-joint position/velocity/torque limit checks
- Tabs: Fault Log, Safety Status, Raw PDO (hex dumps)

**Health logic:**
- OK (green) — normal operation
- WARN (yellow) — torque > 5 Nm
- FAULT (red) — CiA 402 fault state or error code != 0x0000

**ROS subscription:** `/joint_states` (sensor_msgs/JointState)

---

### `demo_joint_state` — Joint State Monitor

Universal joint state viewer that dynamically discovers joints and auto-detects SIM vs REAL mode.

```bash
demo_joint_state
```

**Features:**
- Dynamic joint discovery from `/joint_states`
- Status bar: mode, joint count, message rate (Hz), uptime
- Joint table (8 columns): joint name, motor, slave #, position (deg), velocity (deg/s), effort (Nm), health, EtherCAT state
- Tabs: EtherCAT Status (per-joint drive state), Live Values (streaming log of selected joint, max 200 lines)

**Mode detection:** checks for `/ethercat/raw_positions` and `/safety/status` topics every tick. If found, locks to REAL mode and subscribes to `/diagnostics` for CiA 402 drive state.

**ROS subscriptions:**
- `/joint_states` (sensor_msgs/JointState)
- `/safety/status` (std_msgs/Bool) — REAL mode only
- `/diagnostics` (diagnostic_msgs/DiagnosticArray) — REAL mode only

---

### `demo_joint_offset` — Encoder Zero Calibration

Interactive calibration tool for absolute encoder zero offsets on the left arm. Used to align URDF zero (0.0 rad) with physical zero on MyActuator RMD X V4 encoders.

```bash
demo_joint_offset
```

**Calibration procedure:**
1. Physically move a joint to its URDF zero position
2. Select the joint row in the table
3. Click **Capture Selected** to store the raw encoder value
4. Repeat for all 6 joints
5. Click **Export YAML** to generate offset config
6. Copy the output into `myactuator_hardware/config/left_arm_actuators.yaml`

**Features:**
- Calibration table (8 columns): joint name, motor, URDF limits (deg), real limits (deg, observed min/max), current position (deg), current raw value, captured offset (raw), status
- Real vs URDF limit mismatch alert (yellow highlight when >10 deg difference)
- Buttons: Capture Selected, Capture All, Clear Offsets, Reset Limits, Export YAML
- Tabs: Calibration Log (timestamped events + YAML export), Live Values (raw encoder stream)

**Unit conversion:** raw ±65535 = ±180 deg. `offset_raw` is the encoder reading when the joint is at URDF zero.

**ROS subscription:** `/joint_states` (sensor_msgs/JointState)

---

## `arm_gui_tools` Package — New Entry Points

Run via `ros2 run arm_gui_tools <command>`.

### `ethercat_monitor` — Full EtherCAT Diagnostics

Production monitoring GUI for real hardware. Requires active ROS topics from the EtherCAT stack.

```bash
ros2 run arm_gui_tools ethercat_monitor
```

**Features:**
- Status bar: connection state (DISCONNECTED/OPERATIONAL/DEGRADED/STALE), slave count, cycle rate, safety status, e-stop state, watchdog, violation count
- Joint table (9 columns): joint, motor, bus #, position (rad), velocity (rad/s), torque (Nm), drive state, raw encoder, status
- Color-coded proximity warnings: position/velocity/torque cells turn yellow (>70% of limit) or red (>90%)
- E-Stop and Reset buttons (call `/safety/estop` and `/safety/reset` services)
- Tabs: Fault Log (HTML-colored events), Safety Limits (per-joint config), Raw Data (hex protocol dump)

**ROS subscriptions:** `/joint_states`, `/ethercat/raw_positions`, `/safety/status`, `/diagnostics`
**ROS services:** `/safety/estop` (Trigger), `/safety/reset` (Trigger)

---

### `joint_state_monitor` — Lightweight Joint Table

Minimal joint state viewer with dynamic joint grouping. Works offline (DEMO) or with ROS.

```bash
ros2 run arm_gui_tools joint_state_monitor
```

**Features:**
- Auto-groups joints: L Arm (blue), R Arm (teal), L Hand (peach), R Hand (peach), Other
- Status bar: state (WAITING/CONNECTED/STALE/DEMO), joint count, rate (Hz), uptime
- Joint table (6 columns): joint name, group, position (deg), velocity (deg/s), effort (Nm), health
- Tabs: Live Values (streaming per-joint log), Raw Data (tabular dump)
- 12 demo joints with sinusoidal motion in DEMO mode

**ROS subscription:** `/joint_states` (sensor_msgs/JointState)

---

### `joint_state_tui` — Terminal Dashboard

Rich-based terminal dashboard with multi-tab keyboard navigation. No GUI dependencies (runs in any terminal).

```bash
ros2 run arm_gui_tools joint_state_tui
```

**Features:**
- Multi-tab layout: Joints | Safety | Controllers | Event Log
- Keyboard controls: `1`-`4` switch tabs, `e` trigger e-stop, `r` reset, `q` quit
- Heartbeat detection with stale timeout
- Color-coded tables via Rich library

**Dependencies:** `rich` (Python terminal formatting library)
**ROS subscription:** `/joint_states` (sensor_msgs/JointState)

---

## ROS Interface Summary

### Topics consumed

| Topic | Type | Used by |
|-------|------|---------|
| `/joint_states` | sensor_msgs/JointState | All tools |
| `/ethercat/raw_positions` | std_msgs/Int32MultiArray | ethercat_monitor, demo_joint_state (REAL mode detection) |
| `/safety/status` | std_msgs/Bool | ethercat_monitor, demo_joint_state (REAL mode detection) |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | ethercat_monitor, demo_joint_state (drive states) |

### Services called

| Service | Type | Used by |
|---------|------|---------|
| `/safety/estop` | std_srvs/Trigger | ethercat_monitor |
| `/safety/reset` | std_srvs/Trigger | ethercat_monitor |
