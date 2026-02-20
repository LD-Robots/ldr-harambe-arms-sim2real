# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 Jazzy dual-arm humanoid manipulation system with Gazebo Harmonic simulation, MoveIt 2 motion planning, ros2_control, and EtherCAT real hardware support. The robot has a 6-DOF arm and a 6-DOF hand per side. The left arm uses MyActuator RMD X V4 EtherCAT actuators (X6-X6-X4-X6-X4-X4 from shoulder to wrist) via EtherLab IgH master.

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
| `robot_description/` | arm_description, dual_arm_description, hand_description | URDF models and meshes |
| `control/` | arm_control, dual_arm_control, gripper_control, hand_control | Motion control nodes and configs |
| `planning/` | arm_moveit_config, dual_arm_moveit_config, arm_gripper_moveit_config, arm_hand_moveit_config, arm_mtc | MoveIt 2 configs and MTC tasks |
| `simulation/` | arm_gazebo, dual_arm_gazebo | Gazebo worlds and spawn launch files |
| `hardware_interface/` | arm_hardware, gripper_hardware, hand_hardware | Legacy placeholders |
| `hardware_interface/` | myactuator_ethercat, myactuator_hardware, arm_ethercat_safety | EtherCAT driver, ros2_control plugin, safety monitor |
| `bringup/` | arm_system_bringup, arm_real_bringup | Sim bringup and real hardware bringup |
| `teleop/` | arm_teleop | Keyboard, joystick, and Cartesian teleop |
| `tools/` | arm_gui_tools, diagnostic_tools, dualsense_tools | GUI launcher, monitoring, gamepad input |
| `applications/` | *(empty — future demos)* | |

All packages use `ament_cmake`. Python is the primary language for nodes and launch files; C++ is used for MTC nodes (`arm_mtc/src/`), the joint teleop node, and the EtherCAT driver stack.

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

## EtherCAT Real Hardware

### Package Architecture (4 layers)

1. **`myactuator_ethercat`** — Pure C++ EtherLab driver library (no ROS dependency). CiA 402 state machine, PDO types matching ESI exactly, unit conversions.
2. **`myactuator_hardware`** — ros2_control `SystemInterface` plugin. Bridges EtherCAT to `controller_manager`. Plugin class: `myactuator_hardware::MyActuatorSystem`.
3. **`arm_ethercat_safety`** — Safety monitoring node: watchdog, e-stop, joint limits, CiA 402 fault detection.
4. **`arm_real_bringup`** — Launch files and configs for real hardware. Uses `ros2_control_node` instead of Gazebo plugin.

### URDF Switching

`dual_arm.urdf.xacro use_sim:=false` loads `dual_arm_real.xacro` with the EtherCAT hardware plugin. `use_sim:=true` loads `dual_arm_gazebo.xacro` with Gazebo (unchanged).

### Launching Real Hardware

```bash
# Pre-flight check
bash $(ros2 pkg prefix arm_real_bringup)/share/arm_real_bringup/scripts/check_ethercat.sh

# Launch
ros2 launch arm_real_bringup arm_real.launch.py
```

### Key EtherCAT Configs

| File | Location | Purpose |
|------|----------|---------|
| `left_arm_actuators.yaml` | `myactuator_hardware/config/` | Per-joint slave position, motor type (X4/X6), direction, offset |
| `myactuator_rmd_x_v4.yaml` | `myactuator_ethercat/config/` | Device constants, PDO indices, unit conversion factors |
| `controllers.yaml` | `arm_real_bringup/config/` | Real hardware controllers (1 kHz update rate) |
| `safety_limits.yaml` | `arm_ethercat_safety/config/` | Per-joint position/velocity/torque safety limits |
| `MT-Device-250702.xml` | `myactuator_ethercat/esi/` | ESI file for MyActuator RMD X V4 |

### MyActuator Protocol Reference

- **PDO layout**: RxPDO 0x1600 (16 bytes: control_word, target_position, target_velocity, target_torque, max_torque, mode, padding), TxPDO 0x1A00 (16 bytes: status_word, position_actual, velocity_actual, torque_actual, error_code, mode_display, padding)
- **Position**: raw ±65535 = ±180° → `rad = raw × (π / 65535)`
- **CiA 402 enable**: 0x0006 → 0x0007 → 0x000F (verify status at each step)
- **CRITICAL**: Set target_position = actual_position BEFORE enabling to prevent joint jumps
- Full protocol docs in `docs/myActuator/`
