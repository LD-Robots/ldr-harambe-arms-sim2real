# RL Locomotion Sim2Real Deploy Notes

## Training Configuration (Isaac Lab, 2026-03-27)

### Best Model
- **Run**: `~/IsaacLab/logs/rsl_rl/harambe_flat/2026-03-27_20-30-26/`
- **Checkpoint**: `model_38800.pt` (best reward 34.55, difficulty 37.8%)
- **ONNX export**: `exported/policy.onnx` (input: [batch,87], output: [batch,25])
- **Total iterations**: ~45k (15k + 5k resume + 25k resume with curriculum)

### Training Config (v5 curriculum)
- **Env**: `Isaac-Velocity-Flat-Harambe-v0` (flat terrain, 1024 envs)
- **Network**: MLP 87→256→128→128→25, ELU activations
- **Action scale**: 0.10 rad
- **Policy rate**: 50 Hz (in training, physics at 200Hz)
- **Curriculum**: threshold=25, reward_max=45, auto-ramps domain randomization
- **Domain randomization** (at difficulty 37.8%):
  - Actuator gains: Kp ±50%, Kd ±50%
  - Push velocity: ±0.95 m/s
  - External force: ±25.7 N
  - Mass offset: ±3.5 kg
  - CoM offset: ±6 cm
  - Friction: 0.35-1.65
- **Config file**: `~/IsaacLab/source/isaaclab_tasks/.../harambe/rough_env_cfg.py`

### Isaac Lab Actuator Gains (training)
| Motor | Kp | Kd |
|-------|-----|-----|
| X8 (hip, knee, waist) | 200.0 | 5.0 |
| X6 (shoulder, elbow) | 100.0 | 2.0 |
| X4 ankle pitch | 100.0 | 4.0 |
| X4 ankle roll | 25.0 | 4.0 |
| X4 wrist | 50.0 | 2.0 |

### Joint Ordering (25 DOF, policy order)
```
0:  waist_yaw_joint_X8
1-6:  left arm (shoulder_pitch/roll/yaw, elbow_pitch, wrist_yaw/roll)
7-12: right arm (same)
13-18: left leg (hip_pitch/roll/yaw, knee, ankle_pitch/roll)
19-24: right leg (same)
```

### Default Standing Pose (Isaac Lab)
```
hip_pitch: -0.28, hip_roll: 0, hip_yaw: 0
knee: 0.56, ankle_pitch: -0.28, ankle_roll: 0
arms: all 0, waist: 0
```

## Gazebo DART Deploy Configuration

### Key Files
- **Deploy node**: `src/control/full_robot_control/scripts/rl_locomotion_node.py`
- **Launch**: `src/control/full_robot_control/launch/rl_walk_effort.launch.py`
- **PID config**: `src/simulation/full_robot_gazebo/config/controllers_effort_pid.yaml`
- **World**: `src/simulation/full_robot_gazebo/worlds/empty.sdf`

### Deploy Architecture
```
Policy (ONNX, 50Hz) → position targets
  → JointTrajectoryController (effort + PID, 200Hz)
  → gz_ros2_control (effort commands)
  → Gazebo DART physics (1000Hz, 5ms step)
```

### State Machine (rl_locomotion_node.py)
1. WAIT_ODOM → wait for odometry
2. FALLEN_RESET → detect fall (proj_grav z > -0.3) → gz set_pose teleport
3. SETTLE → 2s hold standup pose
4. STANDUP → 5s hold standup pose (fall detection active)
5. POLICY → RL inference with blend from STANDUP_POS to policy targets

### Gazebo PID Gains (tuned for DART)
| Motor | Kp | Kd |
|-------|-----|-----|
| X8 (hip, knee, waist) | 200.0 | 12.0 |
| X6 (shoulder, elbow) | 100.0 | 2.0 |
| X4 ankle pitch | 200.0 | 10.0 |
| X4 ankle roll | 25.0 | 6.0 |
| X4 wrist | 50.0 | 2.0 |

### Standup Pose (more bent for DART stability)
```
hip_pitch: -0.35, knee: 0.70, ankle_pitch: -0.35
```

### Deploy Tuning Parameters
- **DART_BIAS**: +0.04 hip_pitch, +0.03 ankle_pitch (compensate forward lean)
- **ACTION_CLIP**: hip_p ±0.20, knee ±0.20, hip_r ±0.10, hip_y ±0.05, ankle ±0.20/0.10
- **Blend**: 400 steps (8s) smooth interpolation from STANDUP_POS to policy
- **Ground friction**: mu=2.0, contact kd=100

### Sim2Sim Transfer Results
- **Standing (cmd_vel=0)**: Works ~24s with DART_BIAS + EMA smoothing + tight clips
- **Walking (cmd_vel=0.3)**: Fails — policy slides instead of stepping
- **Root cause**: PhysX implicit PD at 1000Hz vs Gazebo JTC effort PID at 200Hz creates different dynamics. Policy learned PhysX-specific contact behavior.

### Next Steps for Walking Transfer
1. Retrain with much more aggressive randomization:
   - PD gains: Kp 50-400, Kd 2-20 (simulate JTC PID lag)
   - Friction: 0.1-3.0
   - Control delay: 1-5 steps
   - Contact stiffness randomization
2. Or use direct position control interface (Gazebo internal PD) instead of effort PID
3. Or train directly in Gazebo (requires RL training framework for Gazebo)
