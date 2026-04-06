# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Full humanoid robot (Harambe) with ROS 2 Jazzy: dual 6-DOF arms + 6-DOF hands, legs, and waist (25 DOF total). Supports Gazebo Harmonic simulation, MoveIt 2 motion planning for arms, ros2_control, Isaac Lab RL locomotion training, and EtherCAT real hardware. Left arm uses MyActuator RMD X V4 EtherCAT actuators (X6-X6-X4-X6-X4-X4 from shoulder to wrist) via EtherLab IgH master.

## Build & Run

```bash
# Build entire workspace
colcon build

# Build with symlinks (faster iteration — Python changes apply immediately)
colcon build --symlink-install

# Build a single package
colcon build --packages-select arm_control

# Source after every build
source install/setup.bash

# Clean rebuild
rm -rf build/ install/ log/ && colcon build

# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y
```

Shell shortcuts are available via `source setup_shortcuts.bash` (aliases like `cb`, `cbs`, `cbp`, `cs`, `ccs`, `clean`, `rebuild`).

### Launching the System

**GUI launcher (recommended):**
```bash
ros2 run arm_gui_tools full_system_launcher
```
The GUI provides per-component start/stop, world selection, and three modes: Full Local, Robot, Client.

**CLI alternatives:**
```bash
# All-in-one with RViz
ros2 launch arm_system_bringup moveit_gazebo.launch.py

# Headless (no GUI)
ros2 launch arm_system_bringup headless.launch.py

# Modular (two terminals, wait ~20s between)
ros2 launch arm_control sim.launch.py
ros2 launch arm_moveit_config demo.launch.py
```

### Checking System Health

```bash
ros2 control list_controllers   # Verify controllers active
ros2 topic hz /joint_states     # Confirm joint state publishing
ros2 topic list                 # See all active topics
```

## Architecture

### 5-Layer Stack

```
User Interface      — RViz2, PyQt5 GUI launcher, teleop scripts
Application         — MoveIt 2, MoveIt Task Constructor (MTC), perception (planned)
Control             — ros2_control, JointTrajectoryController (100 Hz)
Hardware Abstraction — Gazebo plugin (sim) or hardware_interface plugin (real)
Physical            — Gazebo Harmonic simulation or real robot
```

Sim/real switching is via the `use_sim` xacro arg (`dual_arm.urdf.xacro use_sim:=true/false`).

### Dual Control Paradigm

- **Direct trajectory** (`motion_planner.py`) — sends joint goals directly to `arm_controller`, fast for known motions
- **MoveIt planning** (`moveit_py`) — inverse kinematics, collision avoidance, Cartesian planning

Both share the same underlying `JointTrajectoryController`.

### Package Layout (`src/`)

| Directory | Packages | Purpose |
|-----------|----------|---------|
| `robot_description/` | arm_description, dual_arm_description, full_robot_description, hand_description, camera_description, imu_description | URDF models and meshes |
| `control/` | arm_control, dual_arm_control, full_robot_control, gripper_control, hand_control, implicit_actuator_controller | Motion control nodes; `full_robot_control` hosts the RL locomotion node |
| `planning/` | arm_moveit_config, dual_arm_moveit_config, arm_gripper_moveit_config, arm_hand_moveit_config, arm_mtc | MoveIt 2 configs and MTC tasks |
| `simulation/` | arm_gazebo, dual_arm_gazebo, full_robot_gazebo, dual_arm_isaac | Gazebo worlds; `full_robot_gazebo` is used for RL deploy |
| `hardware_interface/` | arm_hardware, gripper_hardware, hand_hardware, arm_ethercat_safety | Legacy placeholders + safety monitor |
| `ethercat_driver_ros2/` | ethercat_driver, ethercat_generic_plugins, ethercat_interface | ICube EtherCAT framework (external) |
| `bringup/` | arm_system_bringup, arm_real_bringup | Sim bringup and real hardware bringup |
| `teleop/` | arm_teleop | Keyboard, joystick, and Cartesian teleop |
| `tools/` | arm_gui_tools, dualsense_tools, ethercat_tools | GUI launcher, EtherCAT diagnostics, gamepad input |
| `applications/` | *(empty — future demos)* | |

All packages use `ament_cmake`. Python is the primary language for nodes and launch files; C++ is used for MTC nodes (`arm_mtc/src/`), the joint teleop node, and the safety monitor node.

### Key Configuration Files

| File | Location | What it controls |
|------|----------|-----------------|
| `controllers.yaml` | `src/control/arm_control/config/` | Controller types, joint names, interfaces (100 Hz update rate) |
| `kinematics.yaml` | `src/planning/arm_moveit_config/config/` | IK solver settings |
| `joint_limits.yaml` | `src/planning/arm_moveit_config/config/` | Position/velocity/acceleration limits for planning |
| `arm_description.srdf` | `src/planning/arm_moveit_config/config/` | Planning groups, named poses, collision pairs |
| `initial_positions.yaml` | `src/planning/arm_moveit_config/config/` | Named joint configurations (home, ready, etc.) |

### Robot Joint Names

**Arm (6 joints, single arm):** `shoulder_pitch_joint`, `shoulder_roll_joint`, `shoulder_yaw_joint`, `elbow_pitch_joint`, `elbow_yaw_joint`, `wrist_roll_joint`

**Left arm (dual arm, with motor type suffix):** `left_shoulder_pitch_joint_X6`, `left_shoulder_roll_joint_X6`, `left_shoulder_yaw_joint_X4`, `left_elbow_pitch_joint_X6`, `left_wrist_yaw_joint_X4`, `left_wrist_roll_joint_X4`

**Hand (6 joints):** `left_thumb_proximal_yaw_joint`, `left_thumb_proximal_pitch_joint`, `left_index_proximal_joint`, `left_middle_proximal_joint`, `left_ring_proximal_joint`, `left_pinky_proximal_joint`

### Gazebo Worlds

Located in `src/simulation/arm_gazebo/worlds/`: `lab-ldr.sdf` (default), `lab2.sdf`, `obstacle_course.sdf`, `pick_place.sdf`, `manipulation_demo.sdf`.

## Development Notes

- After modifying URDF/SDF/config files, rebuild the relevant package and re-source
- Launch files follow a dependency chain: Gazebo → spawn robot → controller_manager → controllers → MoveIt. Respect startup ordering (~20s between sim and planning layers)
- The GUI launcher handles startup sequencing automatically
- Dual-arm packages mirror single-arm packages with `dual_` prefix and namespace separation
- Comprehensive architecture docs live in `docs/ARCHITECTURE.md` and `docs/ARCHITECTURE_DIAGRAMS.md`

## RL Locomotion & Sim2Real

### Isaac Lab Training

Training runs in `~/IsaacLab/` (outside this repo). The task is `Isaac-Velocity-Flat-Harambe-v0` with RSL-RL, 1024 envs, physics at 200 Hz, policy at 50 Hz.

- **Best model**: `~/IsaacLab/logs/rsl_rl/harambe_flat/2026-03-27_20-30-26/model_38800.pt`
- **ONNX export**: `exported/policy.onnx` — input [batch,87], output [batch,25]
- **Network**: MLP 87→256→128→128→25, ELU activations, action scale 0.10 rad
- **Training config**: `~/IsaacLab/source/isaaclab_tasks/.../harambe/rough_env_cfg.py`
- **Monitor scripts**: `scripts/monitor_training.sh`, `scripts/monitor_v81.sh`

**25-DOF joint ordering (policy order):**
```
0: waist_yaw
1-6:  left arm  (shoulder_pitch/roll/yaw, elbow_pitch, wrist_yaw/roll)
7-12: right arm (same)
13-18: left leg  (hip_pitch/roll/yaw, knee, ankle_pitch/roll)
19-24: right leg (same)
```

### Gazebo Deploy (Sim2Sim)

```bash
ros2 launch full_robot_control rl_walk_effort.launch.py
```

**Architecture:**
```
Policy (ONNX, 50Hz) → position targets
  → implicit_actuator_controller (effort + PD, 200Hz)
  → gz_ros2_control (effort commands)
  → Gazebo DART physics (1000Hz)
```

**Key files:**
- `src/control/full_robot_control/scripts/rl_locomotion_node.py` — ONNX policy runner; state machine: WAIT_ODOM → FALLEN_RESET → SETTLE → STANDUP → POLICY
- `src/simulation/full_robot_gazebo/config/controllers_effort_pid.yaml` — PID gains tuned for DART
- `src/robot_description/full_robot_description/urdf/harambe_mujoco.urdf` — MuJoCo variant for validation

**`implicit_actuator_controller`** (C++ plugin): Mimics Isaac Lab `ImplicitActuator` — per-joint Kp/Kd, integral clamping, effort limits, smooth position ramping.

**Transfer status** (see `docs/sim2real_rl_deploy_notes.md` for full details):
- Standing (cmd_vel=0): Works ~24s with DART_BIAS + EMA + action clips
- Walking (cmd_vel=0.3): Fails — PhysX implicit PD vs. Gazebo JTC effort PID creates incompatible contact dynamics

**Bridge scripts** (in `scripts/`):
- `isaac_ros2_bridge.py` — UDP bridge from Isaac Lab to ROS 2 for live visualization
- `rl_mujoco_ros2.py` — standalone ONNX policy in MuJoCo + ROS 2
- `mujoco_ros2_bridge.py` — MuJoCo → ROS 2 joint state bridge

### URDF Variants

| File | Purpose |
|------|---------|
| `full_robot.urdf.xacro` | Gazebo sim |
| `harambe_full_for_isaac.urdf` | Isaac Lab (auto-generated by `dual_arm_isaac/scripts/generate_urdf_rl.py`) |
| `harambe_mujoco*.urdf` | MuJoCo validation variants (with/without collision meshes) |

## Motion Recording

Record joint trajectories via gravity-comp mode, then play back on sim or real:

```bash
# Record (requires CST/torque mode active)
ros2 run arm_control motion_recorder.py

# Play back
ros2 run arm_control motion_player.py --file motions/wave_hello_4.yaml
```

Motion files are YAML stored in `motions/`. See `docs/motions/README.md` for format and options (speed scaling, looping, dry-run).

## EtherCAT Real Hardware

### Architecture (3 layers)

1. **`ethercat_driver_ros2`** (ICube Robotics) — Generic EtherCAT framework providing `ethercat_driver/EthercatDriver` ros2_control `SystemInterface` plugin and `EcCiA402Drive` per-joint modules. Handles EtherLab IgH master communication, CiA 402 state machine, PDO exchange.
2. **`arm_ethercat_safety`** — Independent safety monitoring node: watchdog, e-stop, joint limits, CiA 402 fault detection. Subscribes to `/joint_states`, publishes `/safety/status` + `/diagnostics`.
3. **`arm_real_bringup`** — Launch files and per-joint EtherCAT YAML slave configs for real hardware.

### URDF Switching

`dual_arm.urdf.xacro use_sim:=false` loads `ros2_control_real.xacro` with the `ethercat_driver/EthercatDriver` plugin. Each joint exports three command interfaces (`position`, `effort`, `mode_of_operation`) and has an `<ec_module>` referencing a per-joint YAML slave config via `EcCiA402Drive`. `use_sim:=true` loads Gazebo (unchanged).

An additional `readonly:=true` xacro arg selects read-only slave configs (`ethercat_readonly/`) that keep drives in SWITCH_ON_DISABLED state.

### Launching Real Hardware

```bash
# Pre-flight check
bash $(ros2 pkg prefix arm_real_bringup)/share/arm_real_bringup/scripts/check_ethercat.sh

# Full control — CSP mode (position), enables drives, 100 Hz
ros2 launch arm_real_bringup arm_real.launch.py

# Gravity-compensated recording — CST mode (torque), enables drives, 100 Hz
ros2 launch arm_real_bringup recording.launch.py

# Read-only position viewer (drives stay disabled, 100 Hz)
ros2 launch arm_real_bringup position_viewer.launch.py
```

### Key EtherCAT Configs

| File | Location | Purpose |
|------|----------|---------|
| `left_*_X[4|6].yaml` (25 files) | `arm_real_bringup/config/ethercat/` | Per-joint slave config (single source of truth): PDO mapping, factors, offsets, mode_of_operation interface |
| `ethercat_readonly/` | *(generated at build time)* | Auto-generated from `ethercat/` with `auto_state_transitions: false` (read-only) |
| `ethercat_gravcomp/` | *(generated at build time)* | Auto-generated from `ethercat/` with `mode_of_operation: 10` (CST torque) |
| `controllers.yaml` | `arm_real_bringup/config/` | Unified controllers: JTC (position), effort, mode_controller (100 Hz) |
| `controllers_viewer.yaml` | `arm_real_bringup/config/` | Read-only viewer (JSB only, 100 Hz) |
| `safety_limits.yaml` | `arm_ethercat_safety/config/` | Per-joint position/velocity/torque safety limits |
| `ros2_control_real.xacro` | `dual_arm_description/urdf/macros/` | URDF hardware plugin definition (position + effort + mode_of_operation) |

### Motor Specs & Calibration

**Encoder resolution:** raw ±65535 = ±180° → `position_factor = π/65535 ≈ 4.794e-5 rad/count` (20,861 counts/rad)

**Gear ratios:** X6 = 19.612:1, X4 = 36:1

**Conversion factors by motor type:**

| Motor | Position cmd factor | Position state factor | Velocity factor | Torque cmd/state |
|-------|--------------------|-----------------------|-----------------|------------------|
| X6 | 20861.0 counts/rad | 4.794e-5 rad/count | 20861.0 / 4.794e-5 | 50.0 / 0.02 |
| X4 | 41722.0 counts/rad | 2.397e-5 rad/count | 20861.0 / 4.794e-5 | 50.0 / 0.02 |

X4 position factor is 2x X6 due to the higher gear ratio (36 vs 19.612).

**Calibrated zero offsets** (from `arm_real_bringup/config/ethercat/`):

| Joint | Motor | Bus Pos | Dir | Cmd Offset (raw) | State Offset (rad) |
|-------|-------|---------|-----|-------------------|---------------------|
| left_shoulder_pitch_X6 | X6 | 0 | -1 | 3892 | 0.186573 |
| left_shoulder_roll_X6 | X6 | 1 | -1 | -652 | -0.031255 |
| left_shoulder_yaw_X4 | X4 | 2 | -1 | 28044 | 0.672181 |
| left_elbow_pitch_X6 | X6 | 3 | -1 | 3328 | 0.159536 |
| left_wrist_yaw_X4 | X4 | 4 | -1 | 262 | 0.006280 |
| left_wrist_roll_X4 | X4 | 5 | +1 | -4 | 0.000096 |

Offsets are encoded in each per-joint YAML slave config as `offset` fields on the RxPDO/TxPDO position channels. Direction reversal is done by negating the `factor` values. Use `demo_joint_offset` tool from `ethercat_tools` to recalibrate.

### Per-Joint Slave Config Format

Each YAML file in `arm_real_bringup/config/ethercat/` defines: vendor/product ID, DC sync, SDO initialization, RxPDO channels (command interfaces with factors and offsets), TxPDO channels (state interfaces with factors and offsets). Direction reversal is achieved by negating all factors. The mode_of_operation PDO channel (0x6060) is exposed as a `command_interface: mode_of_operation` for runtime mode switching. See `docs/ETHERCAT.md` for the full annotated example.

### Dynamic Mode Switching (CSP ↔ CST)

CiA 402 modes are switched at runtime without restarting. Three ros2_control controllers claim **different** interfaces on the same joints (no conflicts):

| Controller | Interface | Purpose | Startup State |
|------------|-----------|---------|---------------|
| `left_arm_group_controller` | position | JointTrajectoryController for CSP | active |
| `left_arm_effort_controller` | effort | JointGroupEffortController for CST | inactive |
| `mode_controller` | mode_of_operation | ForwardCommandController to set CiA 402 mode | active |

**Switch to CST (torque/gravity comp):**
```bash
ros2 control switch_controllers --deactivate left_arm_group_controller
ros2 topic pub --once /mode_controller/commands std_msgs/msg/Float64MultiArray "{data: [10,10,10,10,10,10]}"
ros2 control switch_controllers --activate left_arm_effort_controller
```

**Switch back to CSP (position control):**
```bash
ros2 control switch_controllers --deactivate left_arm_effort_controller
ros2 topic pub --once /mode_controller/commands std_msgs/msg/Float64MultiArray "{data: [8,8,8,8,8,8]}"
ros2 control switch_controllers --activate left_arm_group_controller
```

Mode values: **8** = CSP (Cyclic Synchronous Position), **9** = CSV (Cyclic Synchronous Velocity), **10** = CST (Cyclic Synchronous Torque).

The existing `EcCiA402Drive::processData()` handles mode switching automatically — when the drive reports CST mode via TxPDO (0x6061), position commands are overridden with `last_position_` to prevent jumps.

### MyActuator Protocol Reference

- **PDO layout**: RxPDO 0x1600 (16 bytes: control_word, target_position, target_velocity, target_torque, max_torque, mode, padding), TxPDO 0x1A00 (16 bytes: status_word, position_actual, velocity_actual, torque_actual, error_code, mode_display, padding)
- **Position**: raw ±65535 = ±180° → `rad = raw × (π / 65535)`
- **CiA 402 enable**: Handled automatically by `EcCiA402Drive` when `auto_state_transitions: true`. Sequence: 0x0006 → 0x0007 → 0x000F. Sets target_position = actual_position before enabling to prevent joint jumps.
- Full protocol docs in `docs/myActuator/`
- ESI file: `docs/myActuator/esi/MT-Device-250702.xml`

## Monitoring & Diagnostic Tools

### EtherCAT Tools (`ethercat_tools` package)

| Command | Purpose |
|---------|---------|
| `demo_ethercat_status` | Real-time EtherCAT master + 6 slave status (CiA 402 state, PDO data, safety) |
| `demo_joint_state` | Joint state monitor with auto SIM/REAL detection |
| `demo_joint_offset` | Absolute encoder zero-offset calibration tool |

### GUI Tools (`arm_gui_tools` package)

| Command | Purpose |
|---------|---------|
| `ros2 run arm_gui_tools full_system_launcher` | Full system GUI launcher |
| `ros2 run arm_gui_tools ethercat_monitor` | Full EtherCAT diagnostics (real hardware, subscribes to /diagnostics) |
| `ros2 run arm_gui_tools joint_state_monitor` | Lightweight joint table (ROS or DEMO) |
| `ros2 run arm_gui_tools joint_state_tui` | Terminal dashboard with Rich (multi-tab, keyboard controls) |

All tools support three modes: **DEMO** (offline, simulated data), **SIM** (Gazebo /joint_states only), **REAL** (EtherCAT topics present). Mode is auto-detected. Detailed docs in `docs/TOOLS.md`.
