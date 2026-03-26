# Audit: Isaac Lab → Gazebo Sim-to-Sim Deployment

**Date**: 2026-03-25 (updated)
**Robot**: Harambe humanoid (37 kg, 25 DOF)
**Training**: Isaac Lab + RSL-RL PPO + ImplicitActuator
**Target**: Gazebo Harmonic + ROS 2 Jazzy

---

## Executive Summary

Policy-ul antrenat în Isaac Lab funcționează în Isaac Lab dar **nu transferă direct** în Gazebo. Am implementat un **Gazebo system plugin C++ (`HarambePolicyPlugin`)** care rulează ONNX inference + PD control + velocity limiting direct în physics loop. Plugin-ul funcționează — warmup-ul ține robotul stabil, velocity limiting previne viteze excesive. Problema rămasă: **policy-ul nu a învățat un gait natural** (ep_len plafonat la ~250, fără stepping behavior). Trebuie ajustat reward shaping.

---

## 1. Probleme identificate și rezolvate

| # | Problema | Impact | Status |
|---|---------|--------|--------|
| 1 | Action clip la [-1,1] — policy outputează [-10,+10] | 90% signal pierdut | **REZOLVAT** — clip la ±5 matching training |
| 2 | Default positions mismatch (training vs Gazebo) | Policy out-of-distribution | **REZOLVAT** — default=0 în training |
| 3 | base_lin_vel = 0 în deploy | Policy fără feedback viteză | **ATENUAT** — noise ±0.5 în training |
| 4 | Effort limits mismatch (rated vs moderate) | Torque diferit | **REZOLVAT** — moderate limits în training |
| 5 | Gazebo nu enforce velocity limits | Joints la viteză maximă instant | **REZOLVAT** — velocity limiting în plugin C++ |
| 6 | PD la 50Hz via DDS (latență) | Oscilații, instabilitate | **REZOLVAT** — PD în physics loop via plugin |
| 7 | Gazebo PD ca forță externă (nu constraint solver) | Comportament diferit de PhysX | **ATENUAT** — velocity limiting compensează |

## 2. Ce funcționează

### Gazebo Plugin (`HarambePolicyPlugin`)
- **Plugin Gazebo C++** care rulează ONNX + PD + velocity limiting in-process
- PD la fiecare physics step (~200 Hz), policy la 50 Hz (decimation=4)
- Velocity limiting: dacă `vel > vel_limit && torque accelerează → torque = -effort_limit` (max brake)
- Warmup: ține la default positions N policy steps înainte de ONNX inference
- Toate parametrele configurabile din SDF/URDF
- **Locație**: `src/simulation/harambe_gz_policy_plugin/`

### Training Pipeline
- Default positions = 0 (matching Gazebo standing)
- Effort limits moderate (matching URDF real)
- clip_actions = 5.0
- action_scale = 0.15
- Domain randomization: mase ±15%, friction, push extern ±20N
- Observație: base_lin_vel cu noise ±0.5

### Deploy Script (`deploy_full_body_ros2.py`)
- Mode `gz_pid`: publică Float64 per joint pe `/joint_cmd/<name>`
- Mode `command_topic`: publică Float64MultiArray pe orice topic
- Acțiuni neclipate (sau clip la ±5)
- Suportă ONNX, TorchScript, SB3

## 3. Abordări încercate

| Abordare | Rezultat | Concluzie |
|----------|----------|-----------|
| ros2_control JointGroupPositionController | Stă, policy explodează | P=0.1 intern prea slab |
| ros2_control JointGroupEffortController + PD Python | Cade la startup | PD la 50Hz prea lent |
| ros2_control JointTrajectoryController effort | Stă, velocity spikes | Spline interpolation ≠ Isaac Lab |
| ImplicitActuatorController C++ (ros2_control) | Stă la 0, cade la tranziție | DDS latency |
| Gazebo JointPositionController nativ (gz_pid) | Stă, policy tremură | Fără velocity limiting |
| **HarambePolicyPlugin Gazebo C++** | **Stă, velocity limited, policy cade** | **Funcționează — policy trebuie îmbunătățit** |

## 4. Problema curentă: Policy Quality

Policy-ul (ep_len ~250, reward ~2.5 la iter 9000) **nu a învățat gait natural**:
- `feet_air_time ≈ 0` — nu ridică picioarele
- Posibil sliding sau jumping
- Acțiuni mari (saturate la ±5) → mișcări violente chiar cu velocity limiting
- În Isaac Lab play: robotul cade lent, fără mișcări violente (PhysX constraint solver)
- În Gazebo: robotul cade cu mișcări mai violente (velocity limiting ajută dar nu suficient)

### Cauze posibile
1. **Reward shaping** nu produce stepping behavior
2. **Default pos = 0** (picioare drepte) nu e optim pentru inițierea mersului
3. **lin_vel_x range (0, 0.5)** poate fi prea conservator
4. **action_scale = 0.15** poate fi prea mic sau prea mare

## 5. Utilizare

### Mode gz_policy (plugin ONNX direct în Gazebo)
```bash
# Necesită plugin instalat la system path:
sudo cp install/harambe_gz_policy_plugin/lib/libharambe_gz_policy_plugin.so /opt/ros/jazzy/opt/gz_sim_vendor/lib/

ros2 launch full_robot_gazebo full_robot_world.launch.py \
  control_mode:=gz_policy \
  onnx_path:=/path/to/policy.onnx
```

### Mode gz_pid (Gazebo JointPositionController + deploy script extern)
```bash
ros2 launch full_robot_gazebo full_robot_world.launch.py control_mode:=gz_pid

python3 deploy_full_body_ros2.py --ros-args \
  -p model_path:=policy.onnx -p action_scale:=0.15 \
  -p gz_pid_mode:=true -p warmup_steps:=0
```

### Training
```bash
~/isaacsim/.../python.sh train.py --num_envs 4096 --max_iterations 10000 --headless
```

## 6. Plan de acțiune

### Prioritate 1: Reward Shaping (în curs)
- Forțează gait alternant (un picior pe sol, altul în aer)
- Penalizează contact ilegal (genunchi, torso)
- Penalizează acțiuni mari
- Start cu viteze mici (0-0.5 m/s)
- Rezultat așteptat: ep_len > 500, feet_air_time > 0.01

### Prioritate 2: Plugin Improvements
- Fix plugin loading (env vars sau instalare automată)
- Adaugă angular velocity în observație (lipsește din plugin)
- Adaugă cmd_vel subscriber funcțional
- Logging configurabil

### Prioritate 3: Real Hardware
- Deploy pe EtherCAT cu PD la 1kHz în hardware_interface
- Safety: rate limiter, effort limits, kill switch
- Motor firmware PD ≈ Isaac Lab ImplicitActuator

## 7. Fișiere cheie

| Fișier | Descriere |
|--------|-----------|
| `src/simulation/harambe_gz_policy_plugin/` | **Plugin Gazebo C++ — ONNX + PD + vel limiting** |
| `src/robot_description/.../gz_joint_position_controllers.xacro` | Gazebo JointPositionController per joint |
| `src/robot_description/.../full_robot_gazebo.xacro` | URDF principal cu toate modurile |
| `src/control/implicit_actuator_controller/` | Custom ros2_control PID plugin |
| `training/.../locomotion_env_cfg.py` | Training config (rewards, actuators, obs) |
| `training/.../rsl_rl_ppo_cfg.py` | PPO config (clip_actions, normalization) |
| `training/.../deploy_full_body_ros2.py` | Deploy script ROS 2 |
| `docs/AUDIT_sim2sim_deployment.md` | Acest document |

## 8. Referință: Velocity Limits din Training

| Joint | kp | kd | effort_limit | velocity_limit |
|-------|----|----|-------------|----------------|
| Arms X6 | 30 | 8 | 23 Nm | 16.02 rad/s |
| Arms X4 | 30 | 8 | 12.075 Nm | 8.69 rad/s |
| Waist X8 | 150 | 8 | 72 Nm | 13.3 rad/s |
| Hip pitch/knee X8 | 200 | 10 | 72 Nm | 13.3 rad/s |
| Hip roll/yaw X8 | 150 | 10 | 72 Nm | 13.3 rad/s |
| Ankle pitch (linkage) | 200/300 | 10/12 | 96.6 Nm | 2.17 rad/s |
| Ankle roll (linkage) | 150 | 8 | 28.98 Nm | 7.24 rad/s |
