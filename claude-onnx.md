# ONNX Policy Integration Plan

## Context

You trained an RL policy in Isaac Sim and exported it as ONNX. The goal is to deploy it as a ROS 2 node that reads joint states, runs inference, and sends commands to the existing controllers — working identically in Gazebo sim and on real EtherCAT hardware. **No Jetson required** for inference; ONNX Runtime on CPU handles a 6-DOF policy in <1ms. A Jetson is only needed if you want to run the policy onboard the robot untethered.

## Approach: New `arm_policy_control` Package

Create `src/control/arm_policy_control/` — a lightweight ROS 2 node modeled on the existing `gravity_comp_node.py` pattern (timer-based loop, joint_states subscription, command publishing).

### New Files

```
src/control/arm_policy_control/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── policy_config.yaml          # obs/action mapping, scaling, safety limits
├── launch/
│   └── policy_control.launch.py    # standalone launch (works after any bringup)
├── models/
│   └── .gitkeep                    # put your .onnx files here (gitignored)
└── scripts/
    └── policy_inference_node.py    # the main node
```

### Node Architecture (`policy_inference_node.py`)

Based on [gravity_comp_node.py](src/bringup/arm_real_bringup/scripts/gravity_comp_node.py) pattern:

1. **Init**: load ONNX model via `onnxruntime.InferenceSession`, declare params, create sub/pub/timer
2. **Subscribe** `/joint_states` → store latest positions + velocities (with threading lock)
3. **Timer callback** at `inference_rate` Hz (default 50):
   - Build observation vector (joint pos/vel, optionally target pose + previous action)
   - Apply observation normalization (must match Isaac Sim training normalization)
   - Run ONNX inference → raw action
   - Scale action: `joint_cmd = action * action_scale + action_offset`
   - Clamp to safety limits + rate-limit (max delta per tick)
   - Publish to controller
4. **Command mode**: effort (torque) — publish `Float64MultiArray` to `/left_arm_effort_controller/commands` (same as gravity_comp_node). Position mode also supported via param.
5. **Observation vector** (assembled in order matching Isaac Sim training):
   - Joint positions (6) — from `/joint_states`
   - Joint velocities (6) — from `/joint_states`
   - Target pose (7: xyz + quaternion) — from `/target_pose` topic (PoseStamped)
   - Total: 19-dim input (verify against your ONNX model's input shape)
6. **Enable/disable** via `/policy_control/enable` topic (starts disabled for safety)
6. **Shutdown**: zero torques (effort mode) or stop publishing (position mode)

### Config (`policy_config.yaml`)

```yaml
policy_inference_node:
  ros__parameters:
    model_path: ""
    joint_names: [left_shoulder_pitch_joint_X6, left_shoulder_roll_joint_X6,
                  left_shoulder_yaw_joint_X4, left_elbow_pitch_joint_X6,
                  left_wrist_yaw_joint_X4, left_wrist_roll_joint_X4]
    command_mode: "effort"          # torque output from Isaac Sim
    inference_rate: 100.0           # Hz — matches training frequency
    action_scale: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]   # torque scaling (tune for sim2real)
    action_offset: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    obs_position_scale: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    obs_velocity_scale: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    include_previous_action: false
    include_target_pose: true
    target_pose_topic: "/target_pose"   # geometry_msgs/PoseStamped
    # Safety (from safety_limits.yaml)
    position_limits_min: [-3.0194, -0.2094, -1.7453, -1.3089, -2.3561, -1.5707]
    position_limits_max: [1.69296, 2.93215, 2.96705, 1.8326, 2.35619, 1.57079]
    max_delta_per_tick: [0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
    max_effort: [15.0, 10.0, 8.0, 10.0, 8.0, 8.0]
```

### Launch Integration

**Standalone** — does NOT spawn controllers, just the inference node:

```bash
# In Gazebo (after sim is up)
ros2 launch arm_policy_control policy_control.launch.py \
  model_path:=/path/to/policy.onnx use_sim_time:=true

# On real hardware (after arm_real.launch.py is up)
ros2 launch arm_policy_control policy_control.launch.py \
  model_path:=/path/to/policy.onnx

# Enable inference
ros2 topic pub --once /policy_control/enable std_msgs/msg/Bool "{data: true}"
```

Since the policy outputs torques, on real hardware use `recording.launch.py` (boots directly into CST mode) instead of `arm_real.launch.py` + mode switching. The policy node replaces the gravity_comp_node — don't run both.

**In Gazebo**: The sim `JointGroupEffortController` needs to be added to the sim controllers config (currently only JTC is configured for sim). The plan includes adding an effort controller to `dual_arm_control/config/controllers.yaml`.

### Key Reference Files

| File | Why |
|------|-----|
| [gravity_comp_node.py](src/bringup/arm_real_bringup/scripts/gravity_comp_node.py) | Primary code pattern (timer loop, locking, effort publishing, safety) |
| [recording.launch.py](src/bringup/arm_real_bringup/launch/recording.launch.py) | Launch file pattern |
| [safety_limits.yaml](src/hardware_interface/arm_ethercat_safety/config/safety_limits.yaml) | Joint limit values |
| [controllers.yaml](src/bringup/arm_real_bringup/config/controllers.yaml) | Controller names and interfaces |

### Safety Layers

1. **Starts disabled** — explicit enable required
2. **Position clamping** to safety limits
3. **Rate limiting** — max rad/tick prevents jumps
4. **Effort clamping** per joint
5. **Watchdog** — auto-disables if no joint_states for 10 cycles
6. **Existing `arm_ethercat_safety`** node still runs independently on real HW

### Dependencies

- `onnxruntime` (pip install, ~50MB) — CPU is sufficient for 6-DOF policy
- No PyTorch, no CUDA, no Jetson required for inference

## Sim Effort Support (Required Changes to Existing Files)

The Gazebo URDF currently only exposes `position` command interfaces. To run a torque policy in sim:

1. **Add `effort` command interface** to each arm joint in [ros2_control.xacro](src/robot_description/dual_arm_description/urdf/macros/ros2_control.xacro) — add `<command_interface name="effort"/>` alongside the existing `<command_interface name="position"/>` for all 12 arm joints
2. **Add effort controllers** to [dual_arm_control/config/controllers.yaml](src/control/dual_arm_control/config/controllers.yaml):
   ```yaml
   left_arm_effort_controller:
     type: effort_controllers/JointGroupEffortController
   right_arm_effort_controller:
     type: effort_controllers/JointGroupEffortController
   ```
   Plus their parameter blocks with the same joint lists.
3. **Spawn effort controllers** (inactive) in [control.launch.py](src/control/dual_arm_control/launch/control.launch.py) — add spawners with `--inactive` flag so they don't conflict with position controllers at startup.

## Implementation Steps

1. Modify `ros2_control.xacro` — add effort command interfaces to arm joints
2. Modify `dual_arm_control/config/controllers.yaml` — add effort controllers
3. Modify `dual_arm_control/launch/control.launch.py` — spawn inactive effort controllers
4. Create new package `src/control/arm_policy_control/` skeleton (`CMakeLists.txt`, `package.xml`)
5. Write `config/policy_config.yaml`
6. Write `scripts/policy_inference_node.py` (core node)
7. Write `launch/policy_control.launch.py`
8. Add `models/` dir with `.gitignore` for ONNX files
9. Build and test in Gazebo with a dummy ONNX model
10. Test with actual trained model in Gazebo
11. Deploy to real hardware

## Verification

1. `colcon build --packages-select arm_policy_control && source install/setup.bash`
2. Launch Gazebo sim: `ros2 launch arm_system_bringup headless.launch.py`
3. Launch policy node: `ros2 launch arm_policy_control policy_control.launch.py model_path:=...`
4. Verify node is running: `ros2 node list | grep policy`
5. Check it subscribes to joint_states: `ros2 topic info /joint_states`
6. Enable and observe robot moving: `ros2 topic pub --once /policy_control/enable std_msgs/msg/Bool "{data: true}"`
7. Monitor commands: `ros2 topic echo /left_arm_trajectory_controller/joint_trajectory`
