# Motion Recording & Playback

## Recording

Launch gravity-compensated mode so the arm floats freely, then record in a second terminal.

```bash
# Terminal 1: Launch gravity comp (arm floats, guide by hand)
ros2 launch arm_real_bringup recording.launch.py

# Terminal 2: Record a motion
ros2 run dual_arm_moveit_config motion_recorder.py -n wave_hello
```

Press **ENTER** to start recording, then **ENTER** again (or **Ctrl+C**) to stop. The motion is saved as a YAML file.

### Recorder options

| Flag | Default | Description |
|------|---------|-------------|
| `-n`, `--name` | *(required)* | Motion name (used as filename) |
| `--rate` | `10` | Sample rate in Hz |
| `-o`, `--output` | `motions/<name>.yaml` | Output file path |
| `--topic` | `/joint_states` | Joint states topic |
| `--min-change` | `0.001` | Dead-band in rad (skip if no joint moved more than this) |
| `-d`, `--description` | `""` | Optional description saved in the YAML |

## Playback

Launch the arm in normal position control mode, then play back in a second terminal.

```bash
# Terminal 1: Launch normal control mode
ros2 launch arm_real_bringup arm_real.launch.py

# Terminal 2: Play back a motion
ros2 run dual_arm_moveit_config motion_player.py motions/wave_hello.yaml
```

### Player options

| Flag | Default | Description |
|------|---------|-------------|
| `--mode` | `direct` | `direct` (fast, via controller) or `moveit` (collision-aware) |
| `--speed` | `1.0` | Speed multiplier (0.5 = half speed, 2.0 = double) |
| `--velocity-scaling` | from YAML | MoveIt velocity scaling factor |
| `--accel-scaling` | from YAML | MoveIt acceleration scaling factor |
| `--controller` | from YAML | Controller action name |
| `--planning-group` | from YAML | MoveIt planning group |
| `--loop` | off | Loop playback continuously |
| `--waypoint-step` | `1` | Use every Nth waypoint (downsample) |
| `--dry-run` | off | Print trajectory without executing |

## Full Workflow

```bash
# 1. Record with gravity comp
ros2 launch arm_real_bringup recording.launch.py
ros2 run dual_arm_moveit_config motion_recorder.py -n my_motion

# 2. Play back with position control
ros2 launch arm_real_bringup arm_real.launch.py
ros2 run dual_arm_moveit_config motion_player.py motions/my_motion.yaml

# 3. Play back at half speed, looped
ros2 run dual_arm_moveit_config motion_player.py motions/my_motion.yaml --speed 0.5 --loop

# 4. Play back with MoveIt (collision checking)
ros2 run dual_arm_moveit_config motion_player.py motions/my_motion.yaml --mode moveit
```
