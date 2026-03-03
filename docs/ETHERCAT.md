# EtherCAT Implementation

EtherCAT driver stack for the left arm's 6 MyActuator RMD X V4 actuators, using EtherLab IgH master and the `ethercat_driver_ros2` (ICube Robotics) ROS 2 framework.

## Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│  ros2_control  (controller_manager @ 1 kHz)                      │
│    ├── joint_state_broadcaster  → /joint_states (500 Hz)         │
│    └── left_arm_controller      → JointTrajectoryController      │
├──────────────────────────────────────────────────────────────────┤
│  ethercat_driver/EthercatDriver  (ros2_control SystemInterface)  │
│    └── 6 × EcCiA402Drive modules (one per joint)                 │
│         └── Per-joint YAML slave config (PDO mapping, factors)   │
├──────────────────────────────────────────────────────────────────┤
│  EtherLab IgH master  (kernel module: ec_master)                 │
│    └── /dev/EtherCAT0                                            │
├──────────────────────────────────────────────────────────────────┤
│  Physical EtherCAT bus (daisy-chain, 6 slaves)                   │
│    Slave 0: X6 shoulder pitch                                    │
│    Slave 1: X6 shoulder roll                                     │
│    Slave 2: X4 shoulder yaw                                      │
│    Slave 3: X6 elbow pitch                                       │
│    Slave 4: X4 wrist yaw                                         │
│    Slave 5: X4 wrist roll                                        │
└──────────────────────────────────────────────────────────────────┘
```

A separate **safety monitor node** (`arm_ethercat_safety`) runs independently at 100 Hz, subscribing to `/joint_states` and publishing `/safety/status` + `/diagnostics`.

---

## Package Layout

| Package | Location | Role |
|---------|----------|------|
| `ethercat_driver_ros2` | `src/ethercat_driver_ros2/` | Generic EtherCAT master framework (ICube, external) |
| `arm_ethercat_safety` | `src/hardware_interface/arm_ethercat_safety/` | Safety monitor node (watchdog, e-stop, joint limits, faults) |
| `arm_real_bringup` | `src/bringup/arm_real_bringup/` | Launch files, controller config, per-joint EtherCAT YAML configs |

---

## Bus Topology

Left arm, shoulder to wrist:

| Slave | Joint | Motor | Direction |
|-------|-------|-------|-----------|
| 0 | `left_shoulder_pitch_joint_X6` | X6 | +1 |
| 1 | `left_shoulder_roll_joint_X6` | X6 | +1 |
| 2 | `left_shoulder_yaw_joint_X4` | X4 | +1 |
| 3 | `left_elbow_pitch_joint_X6` | X6 | +1 |
| 4 | `left_wrist_yaw_joint_X4` | X4 | -1 |
| 5 | `left_wrist_roll_joint_X4` | X4 | +1 |

Direction -1 means the motor's positive rotation is opposite to the URDF convention. This is encoded in the PDO conversion factors (negated).

---

## Configuration Files

### URDF Hardware Plugin

**File:** `src/robot_description/dual_arm_description/urdf/macros/ros2_control_real.xacro`

Loaded when `dual_arm.urdf.xacro use_sim:=false`. Defines one `<ros2_control>` block per arm with `ethercat_driver/EthercatDriver` plugin and `EcCiA402Drive` modules per joint:

```xml
<ros2_control name="LeftArmSystem" type="system">
    <hardware>
        <plugin>ethercat_driver/EthercatDriver</plugin>
        <param name="master_id">0</param>
        <param name="control_frequency">1000</param>
    </hardware>

    <joint name="left_shoulder_pitch_joint_X6">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
        <ec_module name="LeftShoulderPitch">
            <plugin>ethercat_generic_plugins/EcCiA402Drive</plugin>
            <param name="alias">0</param>
            <param name="position">0</param>
            <param name="slave_config">$(find arm_real_bringup)/config/ethercat/left_shoulder_pitch_X6.yaml</param>
        </ec_module>
    </joint>
    <!-- ... 5 more joints ... -->
</ros2_control>
```

Each `<ec_module>` references a per-joint YAML slave config.

### Per-Joint Slave Config (EtherCAT YAML)

**Location:** `src/bringup/arm_real_bringup/config/ethercat/`

One YAML file per joint. These define the EtherCAT slave's vendor ID, PDO mapping, conversion factors, and SDO initialization.

**Example — normal direction** (`left_shoulder_pitch_X6.yaml`):

```yaml
vendor_id:  0x00202008
product_id: 0x00000000
assign_activate: 0x0300          # DC sync mode
auto_fault_reset: true
auto_state_transitions: true
mode_of_operation: 8             # CSP (Cyclic Synchronous Position)

sdo:
  - {index: 0x6072, sub_index: 0, type: uint16, value: 800}   # max_torque 80%

rpdo:   # Master → Slave (16 bytes)
  - index: 0x1600
    channels:
      - {index: 0x6040, sub_index: 0, type: uint16, command_interface: ~,        default: 0}       # control_word
      - {index: 0x607a, sub_index: 0, type: int32,  command_interface: position, factor: 20861.0}   # target_position
      - {index: 0x60ff, sub_index: 0, type: int32,  command_interface: velocity, factor: 20861.0}   # target_velocity
      - {index: 0x6071, sub_index: 0, type: int16,  command_interface: effort,   factor: 50.0}      # target_torque
      - {index: 0x6072, sub_index: 0, type: int16,  command_interface: ~,        default: 800}      # max_torque
      - {index: 0x6060, sub_index: 0, type: int8,   command_interface: ~,        default: 8}        # mode (CSP)
      - {index: 0x5ff1, sub_index: 0, type: int8,   command_interface: ~,        default: 0}        # padding

tpdo:   # Slave → Master (16 bytes)
  - index: 0x1a00
    channels:
      - {index: 0x6041, sub_index: 0, type: uint16, state_interface: ~}                              # status_word
      - {index: 0x6064, sub_index: 0, type: int32,  state_interface: position, factor: 4.794e-5}     # actual_position
      - {index: 0x606c, sub_index: 0, type: int32,  state_interface: velocity, factor: 4.794e-5}     # actual_velocity
      - {index: 0x6077, sub_index: 0, type: int16,  state_interface: effort,   factor: 0.02}         # actual_torque
      - {index: 0x603f, sub_index: 0, type: uint16, state_interface: ~}                              # error_code
      - {index: 0x6061, sub_index: 0, type: int8,   state_interface: ~}                              # mode_display
      - {index: 0x5ff2, sub_index: 0, type: int8,   state_interface: ~}                              # padding
```

**Reversed direction** (`left_wrist_yaw_X4.yaml`) — negate all factors:

```yaml
rpdo:
  - index: 0x1600
    channels:
      - {index: 0x607a, sub_index: 0, type: int32, command_interface: position, factor: -20861.0}
      # ...
tpdo:
  - index: 0x1a00
    channels:
      - {index: 0x6064, sub_index: 0, type: int32, state_interface: position, factor: -4.794e-5}
      # ...
```

### Gear Ratios

| Motor | Gear Ratio | Used On |
|-------|-----------|---------|
| X6 | 19.612:1 | Shoulder pitch, shoulder roll, elbow pitch |
| X4 | 36.0:1 | Shoulder yaw, wrist yaw, wrist roll |

### Encoder Resolution & Conversion Factors

**Encoder resolution:** raw ±65535 = ±180° → `position_factor = π/65535 ≈ 4.794e-5 rad/count` (20,861 counts/rad)

| Quantity | Raw range | SI | Factor (raw→SI) | Factor (SI→raw) |
|----------|-----------|-----|-----------------|-----------------|
| Position | ±65535 | ±π rad | 4.794e-5 | 20861.0 |
| Velocity | LSB | rad/s | 4.794e-5 | 20861.0 |
| Torque | 0–1000 | 0–100% rated | 0.02 (×50 pct→raw) | 50.0 |

**Per motor type position factors** (X4 is 2x X6 due to higher gear ratio):

| Motor | Position cmd (SI→raw) | Position state (raw→SI) |
|-------|----------------------|------------------------|
| X6 | 20861.0 counts/rad | 4.794e-5 rad/count |
| X4 | 41722.0 counts/rad | 2.397e-5 rad/count |

### Calibrated Zero Offsets

Absolute encoders don't align with URDF zero out of the box. These offsets correct the alignment. Source: `arm_real_bringup/config/ethercat/*.yaml`.

| Joint | Motor | Bus Pos | Dir | Cmd Offset (raw) | State Offset (rad) |
|-------|-------|---------|-----|-------------------|---------------------|
| left_shoulder_pitch_X6 | X6 | 0 | -1 | -3892 | 0.186573 |
| left_shoulder_roll_X6 | X6 | 1 | -1 | 652 | -0.031255 |
| left_shoulder_yaw_X4 | X4 | 2 | -1 | 14022 | 0.672181 |
| left_elbow_pitch_X6 | X6 | 3 | -1 | -3328 | 0.159536 |
| left_wrist_yaw_X4 | X4 | 4 | -1 | 131 | 0.006280 |
| left_wrist_roll_X4 | X4 | 5 | +1 | 2 | 0.000096 |

Offset formula: command `raw = rad × factor + offset_raw`, state `rad = raw × factor + offset_rad`. Recalibrate with `demo_joint_offset`.

### Controller Config

**File:** `src/bringup/arm_real_bringup/config/controllers.yaml`

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz — matches EtherCAT cycle time

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
      publish_rate: 500

    left_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

left_arm_controller:
  ros__parameters:
    joints:
      - left_shoulder_pitch_joint_X6
      - left_shoulder_roll_joint_X6
      - left_shoulder_yaw_joint_X4
      - left_elbow_pitch_joint_X6
      - left_wrist_yaw_joint_X4
      - left_wrist_roll_joint_X4
    command_interfaces: [position]
    state_interfaces: [position, velocity, effort]
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.0
    allow_partial_joints_goal: false
    allow_nonzero_velocity_at_trajectory_end: false
```

### Safety Limits

**File:** `src/hardware_interface/arm_ethercat_safety/config/safety_limits.yaml`

```yaml
joints:
  left_shoulder_pitch_joint_X6:
    position_min: -3.14       # rad
    position_max: 3.14        # rad
    velocity_max: 2.0         # rad/s
    torque_max_pct: 80        # % of rated current
    position_margin: 0.1      # rad — warning zone before hard limit
  # ... per joint (X4 wrists use velocity_max: 3.0)
```

---

## Safety Monitor

**Package:** `arm_ethercat_safety`
**Node:** `safety_monitor`

Independent ROS 2 node (not in the real-time control loop) that monitors system health and provides emergency stop capability.

### Subsystems

| Subsystem | What it monitors | Action on fault |
|-----------|-----------------|-----------------|
| **Watchdog** | Time since last `/joint_states` message (default 50 ms timeout) | Hold position or disable drives |
| **E-stop** | Hardware GPIO pin (active low) or software trigger via `/safety/estop` | Quick stop (CiA 402 0x605A) or immediate disable |
| **Joint Limits** | Per-joint position, velocity, torque against `safety_limits.yaml` | Log violation, set system unsafe |
| **Fault Handler** | CiA 402 status word bit 3 + error code (0x603F) | Log decoded fault, set system unsafe |

### ROS Interface

| Interface | Type | Direction |
|-----------|------|-----------|
| `/joint_states` | sensor_msgs/JointState | Subscribe |
| `/safety/status` | std_msgs/Bool | Publish (true = safe) |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | Publish |
| `/safety/estop` | std_srvs/Trigger | Service |
| `/safety/reset` | std_srvs/Trigger | Service |

### Diagnostics Published

The `/diagnostics` topic publishes 4 status entries per cycle:

- `safety_monitor: watchdog` — OK or "Communication timeout"
- `safety_monitor: estop` — OK, "Hardware e-stop active", or "Software e-stop active"
- `safety_monitor: joint_limits` — OK or "Joint limit violation" (with per-joint key-value details)
- `safety_monitor: overall` — "System safe" or "SYSTEM UNSAFE"

---

## CiA 402 State Machine

All MyActuator drives follow the CiA 402 (IEC 61800-7-204) drive profile.

### Enable Sequence

```
NOT_READY_TO_SWITCH_ON  →  (automatic)
SWITCH_ON_DISABLED      →  control_word = 0x0006 (Shutdown)
READY_TO_SWITCH_ON      →  control_word = 0x0007 (Switch On)
SWITCHED_ON             →  [set target_position = actual_position]
                           control_word = 0x000F (Enable Operation)
OPERATION_ENABLED       ←  status_word bit check confirms
```

**CRITICAL:** Before sending 0x000F, always set `target_position = actual_position` to prevent sudden joint jumps on enable.

### Status Word Decoding (0x6041)

| Bits [6,5,3,2,1,0] | State |
|---------------------|-------|
| xx0x 0000 | Not Ready to Switch On |
| xx1x 0000 | Switch On Disabled |
| xx01 0001 | Ready to Switch On |
| xx01 0011 | Switched On |
| xx01 0111 | Operation Enabled |
| xx00 0111 | Quick Stop Active |
| xx0x 1000 | Fault |

### Common Error Codes (0x603F)

| Code | Meaning |
|------|---------|
| 0x0000 | No error |
| 0x2310 | Over current |
| 0x3210 | DC link over voltage |
| 0x4210 | Over temperature (drive) |
| 0x5441 | Motor blocked / following error |
| 0xFF01 | EtherCAT sync error |
| 0xFF02 | EtherCAT watchdog error |

---

## PDO Layout

### RxPDO 0x1600 — Master to Slave (16 bytes)

| Offset | Object | Type | Description |
|--------|--------|------|-------------|
| 0 | 0x6040 | uint16 | Control word |
| 2 | 0x607A | int32 | Target position (raw ±65535 = ±180°) |
| 6 | 0x60FF | int32 | Target velocity |
| 10 | 0x6071 | int16 | Target torque |
| 12 | 0x6072 | int16 | Max torque |
| 14 | 0x6060 | int8 | Mode of operation (8=CSP) |
| 15 | 0x5FF1 | int8 | Padding |

### TxPDO 0x1A00 — Slave to Master (16 bytes)

| Offset | Object | Type | Description |
|--------|--------|------|-------------|
| 0 | 0x6041 | uint16 | Status word |
| 2 | 0x6064 | int32 | Actual position |
| 6 | 0x606C | int32 | Actual velocity |
| 10 | 0x6077 | int16 | Actual torque |
| 12 | 0x603F | uint16 | Error code |
| 14 | 0x6061 | int8 | Mode display |
| 15 | 0x5FF2 | int8 | Padding |

---

## Launching Real Hardware

### Pre-Flight Check

```bash
bash $(ros2 pkg prefix arm_real_bringup)/share/arm_real_bringup/scripts/check_ethercat.sh
```

Verifies:
1. `ec_master` kernel module loaded
2. `/dev/EtherCAT0` exists
3. Master in Idle or Operation phase
4. 6 slaves detected
5. All slaves in PREOP or higher

### Launch

```bash
ros2 launch arm_real_bringup arm_real.launch.py
```

Startup sequence:
1. `robot_state_publisher` — publishes URDF TF (use_sim:=false variant)
2. `ros2_control_node` — loads `EthercatDriver` plugin, initializes EtherCAT master, activates slaves
3. `joint_state_broadcaster` — spawned after control node ready
4. `left_arm_controller` — spawned after JSB ready
5. `safety_monitor` — launched in parallel via included launch file

### Verifying

```bash
ros2 control list_controllers          # Should show left_arm_controller active
ros2 topic hz /joint_states            # Should show ~500 Hz
ros2 topic echo /safety/status         # data: true (system safe)
ros2 topic echo /diagnostics           # Per-subsystem status
```

---

## Offset Calibration

Absolute encoders on MyActuator drives read raw values (±65535 = ±180°) that don't necessarily align with URDF zero. The `offset_raw` parameter corrects this.

### Procedure

1. Run the calibration tool: `demo_joint_offset`
2. Physically move each joint to its URDF zero position
3. Capture the raw encoder reading for each joint
4. Export and copy offset values into the slave config YAML files (set `offset` field on the TxPDO position channel)

### Where Offsets Are Applied

In each per-joint YAML (`config/ethercat/*.yaml`), the TxPDO position channel has an `offset` field (rad), and the RxPDO position channel also has an `offset` field (raw counts).

---

## Adding a New Joint or Arm

1. **Create a slave config YAML** in `arm_real_bringup/config/ethercat/` following the existing pattern. Set the correct bus `position`, and negate factors if direction is reversed.
2. **Add `<joint>` + `<ec_module>` block** in `ros2_control_real.xacro` pointing to the new YAML.
3. **Add joint to `controllers.yaml`** under the appropriate controller's `joints` list.
4. **Add safety limits** in `arm_ethercat_safety/config/safety_limits.yaml`.
5. **Run offset calibration** with `demo_joint_offset` to determine `offset_raw`.
6. **Update slave count** in `check_ethercat.sh` if the bus now has more than 6 slaves.

---

## Reference Files

| File | Purpose |
|------|---------|
| `docs/myActuator/esi/MT-Device-250702.xml` | ESI (EtherCAT Slave Information) file |
| `docs/myActuator/` | MyActuator protocol documentation |
