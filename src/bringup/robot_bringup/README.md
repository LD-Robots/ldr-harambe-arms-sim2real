# robot_bringup

Whole-body PVT bring-up for the humanoid on real EtherCAT hardware. Brings up all 25 joints in CiA 402 mode 5 (PVT — drive-side PD at 1 kHz), wires together the `robot_pvt_controller` plus three broadcasters and an independent safety supervisor, and ships the per-joint slave configs that live as the source of truth for the PVT stack. Pairs with a read-only viewer launch for pre-flight, calibration, and back-driven inspection.

## Status & scope

- **Hardware only.** Real EtherCAT, drive-side PD. Sim bring-up lives in `arm_system_bringup`; this package does not load Gazebo.
- **Whole body (25 joints).** Per-launch scope via `robot_group:=…` — joints outside the active group are commanded FREE (zero torque, back-drivable) so the bus stays full but only the group you asked for is under control.
- **Operator-gated activation.** `robot_pvt_controller` is spawned `--inactive` on purpose; gains are initialized to 0. See [Why the PVT controller starts inactive](#why-the-pvt-controller-starts-inactive).
- **Companion GUIs** (`pvt_tuner`, `pvt_dashboard`) live in [src/tools/robot_tools/](../../tools/robot_tools/) — cross-referenced below but not documented here.

## Architecture at a glance

```
dual_arm.urdf.xacro                        robot_safety
  (use_sim:=false, pvt_mode:=true)             (independent node;
        │                                       supervisor + e-stop)
        ▼                                            ▲
robot_state_publisher ───────► /robot_description    │
        │                                            │
        ▼                                       /safety/...
ros2_control_node  (chrt -f 49, 1 kHz)
        │
        ├── joint_state_broadcaster ──────────► /joint_states_raw  (1 kHz)
        ├── robot_filtered_joint_state_broadcaster ─► /joint_states_filtered (100 Hz, IIR 30 Hz cutoff)
        ├── robot_drive_status_broadcaster ──► /drive_status (50 Hz, TxPDO 0x1A02)
        └── robot_pvt_controller (INACTIVE) ──► drive-side PD on all 25 joints
                                                (mode 5; τ = Kp·Δp + Kd·Δv + τ_ff)

joint_state_publisher (30 Hz aggregator) ────► /joint_states
```

`ros2_control_node` runs under `chrt -f 49` (SCHED_FIFO, priority 49) — drive-side PD is timing-critical. The standard JSB writes to `/joint_states` and the launch remaps it to `/joint_states_raw` so `joint_state_publisher` stays the sole publisher on `/joint_states`.

---

# Operator runbook

## Pre-flight

- **EtherCAT bus check.** Run [arm_real_bringup/scripts/check_ethercat.sh](../arm_real_bringup/scripts/check_ethercat.sh) before launching anything that enables drives.
- **Real-time permissions.** The launch starts `ros2_control_node` with `chrt -f 49`. Your user needs CAP_SYS_NICE / the rtprio rlimit raised, or the launch will fail to set the policy.
- **Cold start always begins with the viewer.** It brings the bus up without enabling any drive, so you can confirm telemetry is healthy before committing to mode 5.
- **Real hardware is unforgiving.** Gains start at zero across all 25 joints by design — see the PVT controller config comments. The left wrist yaw X4 was physically broken during an effort PD test in April 2026; the bench-rig ramp protocol exists so that does not happen again.

## Launching

### Read-only viewer (always start here)

```bash
ros2 launch robot_bringup robot_pvt_viewer.launch.py robot_group:=arms
```

- Uses the auto-generated `ethercat_pvt_readonly/` slave configs (`auto_state_transitions: false`, `auto_fault_reset: false`) — drives stay in `SWITCH_ON_DISABLED`. No torque output is possible, even if a stale setpoint were published.
- Spawns the three broadcasters only. `robot_pvt_controller` is **not** loaded.
- The safety supervisor still runs (useful for watching breach reasons in isolation).
- Use it for: position calibration, sanity-checking the bus, watching `/drive_status` for temperatures and bus voltage before any drive is enabled.

`robot_group` choices: `arms` (default), `arms_waist`, `legs`, `full`. In viewer mode the choice only affects which joints the controller would be scoped to once you switch to the live launch — the bus enumerates all 25 either way.

### Full PVT bring-up

```bash
ros2 launch robot_bringup robot_pvt.launch.py robot_group:={arms|arms_waist|legs|full}
```

- Default `robot_group` is `arms`.
- Brings drives through `PREOP → SAFEOP → OP` and enables CSP-equivalent mode-5 operation.
- Spawn chain (sequential, gated by `OnProcessExit` so the controller-manager service queue stays single-file):
  1. `ros2_control_node` (1 kHz, `chrt -f 49`).
  2. `joint_state_broadcaster` → `/joint_states_raw`.
  3. `robot_filtered_joint_state_broadcaster` → `/joint_states_filtered`.
  4. `robot_drive_status_broadcaster` → `/drive_status`.
  5. `robot_pvt_controller` — **spawned `--inactive`**.
- In parallel: `robot_safety_supervisor`, `joint_state_publisher`, RViz.

## Why the PVT controller starts inactive

From the launch file header ([launch/robot_pvt.launch.py:11-13](launch/robot_pvt.launch.py#L11-L13)):

> The PVT controller is spawned INACTIVE so the drives stay in mode 5 with Kp=Kd=0 (back-drivable FREE) until the operator ramps gains per the bench-rig protocol.

And from the controller config ([config/controllers_pvt.yaml:159-161](config/controllers_pvt.yaml#L159-L161)):

> FIRST BRINGUP: leave Kp / Kd at 0.0 across the board. Activate the controller, verify ~/setpoint streams reach the drives, then ramp Kd then Kp per the bench-rig protocol.

Two layers of safety: the controller is not auto-activated, and even if it were, every `Kp` and `Kd` entry in the shipped config is `0.0`. With both gains zero, the drive PD law collapses to zero torque — the joint stays back-drivable. The deliberate sequence is **activate first → verify telemetry → ramp gains**, never the other way around.

## Activating the PVT controller

```bash
ros2 control list_controllers                                    # confirm INACTIVE
ros2 control switch_controllers --activate robot_pvt_controller  # safe with Kp=Kd=0
ros2 control list_controllers                                    # confirm ACTIVE
```

Activation with all gains at zero is safe. Activation alone does not produce torque.

## Bench-rig gain ramp protocol

After activation, ramp gains one joint at a time:

1. Pick a joint family (X4, X6, or X8). The X4s have the highest gear ratio and the tightest envelope — start there.
2. Verify telemetry on `/joint_states_filtered` and `/drive_status` is sane.
3. Ramp `Kd` first, in small steps, watching for oscillation.
4. Once damping is stable, ramp `Kp`.
5. Move to the next joint only after the current one is validated.

Gain values live in the controller's parameter namespace; set them via `ros2 param set /controller_manager/robot_pvt_controller …` or the live tuner GUI. **Do not edit the YAML and rebuild during a bench session** — relaunching will drop the bus through SAFEOP and you lose the validated state.

Numeric targets per joint family are in `~/Documents/GitHub/ldr-harambe-docs/docs/tuning/ethercat_control_guide.html` §23. They are not inlined here on purpose — the docs repo is the source of truth for the tuning table.

`Kd_damp` is a separate pure-brake gain used only while `~/damp` is latched (Kp = 0, Kd = `Kd_damp`, q_des = q_meas → τ = −Kd_damp · q̇). Ramp it on the same discipline as `Kd`.

## Operating modes (FREE / DAMP / TRACK)

| Mode | Drive law | Triggered by | Joint behavior |
|------|-----------|--------------|----------------|
| FREE | Kp = Kd = 0, target = measured | Default at activation; inactive joints under any `body_group` | Back-drivable, zero torque |
| DAMP | Kp = 0, Kd = `Kd_damp`, q_des = q_meas | `~/damp` latched by host or supervisor | Velocity-damped, no position hold |
| TRACK | Kp, Kd, τ_ff from setpoint stream | Host publishing `~/setpoint`; gains > 0 | Full PVT — position tracking with feedforward |

The supervisor can derate `Kp` (down to 0) on a safety breach, sliding the joint from TRACK back through DAMP toward FREE. Watch `/safety/breach_reason` to see why.

## Body groups

| Group | Joints active | Inactive joints |
|-------|---------------|-----------------|
| `arms` (default) | 12 (left/right arm) | 13 |
| `arms_waist` | 13 (arms + waist yaw) | 12 |
| `legs` | 12 (left/right leg) | 13 |
| `full` | 25 | 0 |

Inactive joints receive the FREE pattern (`q = measured`, `v = eff = Kp = Kd = 0`) so the drive PD law collapses to zero torque. The bus is always physically full; `body_group` only changes which joints the controller drives. The setting is **not** runtime-reconfigurable — switching groups requires relaunching.

## Topics you will use

| Topic | Rate | Type | Purpose |
|-------|------|------|---------|
| `/joint_states_raw` | 1000 Hz | `sensor_msgs/JointState` | Raw EtherCAT feedback. Primary state feed for host-side interpolation. |
| `/joint_states_filtered` | 100 Hz | `sensor_msgs/JointState` | IIR 30 Hz cutoff on velocity / effort. Position passes through unfiltered. For dashboards. |
| `/joint_states` | 30 Hz | `sensor_msgs/JointState` | Aggregated by `joint_state_publisher`. Standard ROS UI feed. |
| `/drive_status` | 50 Hz | `std_msgs/Float64MultiArray` | TxPDO 0x1A02 — motor_temp, drive_temp, bus_voltage, error_code per joint. |
| `/safety/...` | varies | (see `robot_safety`) | Supervisor state, breach reasons, e-stop. |

PVT controller setpoint / damp / hold topics are exposed by `robot_pvt_control`; discover them at runtime with `ros2 topic list | grep pvt` after activation rather than relying on names here.

## Companion tools

The paired GUIs live in [src/tools/robot_tools/](../../tools/robot_tools/) — installed as console scripts:

```bash
ros2 run robot_tools pvt_dashboard   # whole-body at-a-glance
ros2 run robot_tools pvt_tuner       # per-joint live gain tuning
```

## Troubleshooting

- **"`robot_pvt_controller` is inactive."** By design. Activate per [Activating the PVT controller](#activating-the-pvt-controller).
- **Controllers fail to spawn.** Check the `ros2_control_node` log — drives must reach `OP` (live launch) or `SAFEOP` (viewer launch) before the spawn chain starts. The 4-second `TimerAction` gates this. If a drive is stuck, drop to viewer mode and inspect with `ethercat slaves -v`.
- **Legs cause controller_manager to fail at startup.** The PVT URDF requires `fixed_legs:=false` so the leg joints stay revolute — the controller_manager cannot claim hardware interfaces on `fixed` joints. The launch enforces this.
- **Bus voltage or temperatures out of envelope.** `/drive_status` will flag it; thresholds are set in [config/controllers_pvt.yaml](config/controllers_pvt.yaml) (`bus_voltage_min: 40.0`, `bus_voltage_max: 54.0`, motor warn 70 °C / error 90 °C, drive warn 70 °C / error 85 °C).
- **Joint drifts toward gravity at zero PD gain.** `comp_sign` for that joint is wrong. Flip a single entry, never the whole array — see the [config/controllers_pvt.yaml](config/controllers_pvt.yaml#L218-L224) comment.

---

# Developer reference

## Package layout

```
robot_bringup/
├── package.xml
├── CMakeLists.txt
├── launch/
│   ├── robot_pvt.launch.py          # full bring-up (drives enabled, PVT inactive)
│   └── robot_pvt_viewer.launch.py   # read-only (drives SWITCH_ON_DISABLED)
├── config/
│   ├── controllers_pvt.yaml         # controller_manager + 4 controllers
│   ├── controllers_pvt_viewer.yaml  # controller_manager + 3 broadcasters
│   ├── relaxed_pose.yaml            # reference pose for gravcomp
│   ├── body_groups/
│   │   ├── arms.yaml
│   │   ├── arms_waist.yaml
│   │   ├── legs.yaml
│   │   └── full.yaml
│   └── ethercat_pvt/                # 25 per-joint slave configs (source of truth)
│       ├── left_shoulder_pitch_X6.yaml
│       └── …
└── scripts/
    └── generate_ethercat_pvt.py     # one-shot bootstrap; NOT run at build time
```

At build time, CMake also generates `share/robot_bringup/config/ethercat_pvt_readonly/` by copying `ethercat_pvt/` and flipping `auto_fault_reset` and `auto_state_transitions` to `false`. The viewer launch passes that directory as `pvt_yaml_dir` to the URDF xacro.

## Build & install rules

[CMakeLists.txt](CMakeLists.txt) — `ament_cmake` + `ament_cmake_python`:

- `install(DIRECTORY config/ …)` — ships all hand-tuned YAMLs.
- `add_custom_target(generate_ethercat_pvt_readonly ALL …)` — `cp` + `sed` to derive the readonly variant. This is the **only** auto-generation in the package; the generator script itself is not invoked at build (per the project rule: per-joint config files are source-controlled and hand-tunable, generators bootstrap once).
- `install(DIRECTORY launch/ …)` — launch files.
- `install(PROGRAMS scripts/generate_ethercat_pvt.py …)` — the bootstrap tool, available as `ros2 run robot_bringup generate_ethercat_pvt.py` for rebootstraps.

## Parameter flow

`ros2_control_node` receives a `parameters=[...]` list. Each successive entry overrides keys from the previous one. Load order:

1. `robot_description` — URDF from `dual_arm.urdf.xacro use_sim:=false pvt_mode:=true fixed_legs:=false`. The viewer adds `readonly:=true pvt_yaml_dir:=<readonly dir>`.
2. [config/controllers_pvt.yaml](config/controllers_pvt.yaml) — controllers + PVT defaults (`body_group: "full"`, gains all zero, FF off, gravcomp model populated but disabled).
3. `robot_safety/config/safety_limits.yaml` — supervisor + a mirrored `robot_pvt_controller.ros__parameters` block read by the controller's `loadSafetyLimits()`. Keeping the supervisor and the controller pinned to the same per-joint envelopes prevents the divergence bug surfaced by the PVT audit.
4. [config/body_groups/{robot_group}.yaml](config/body_groups/) — overrides `body_group` per launch.

## Controllers registered

From [config/controllers_pvt.yaml](config/controllers_pvt.yaml#L19-L33):

| Name | Type | Notes |
|------|------|-------|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | `publish_rate: 1000`. Remapped `/joint_states → /joint_states_raw`. |
| `robot_filtered_joint_state_broadcaster` | `robot_filtered_joint_state_broadcaster/FilteredJointStateBroadcaster` | IIR `velocity_cutoff_hz: 30.0`, `torque_cutoff_hz: 30.0`, `publish_rate: 100.0`, topic `/joint_states_filtered`. |
| `robot_drive_status_broadcaster` | `robot_drive_status_broadcaster/DriveStatusBroadcaster` | `publish_rate: 50.0`. Temperature and bus-voltage thresholds inline. |
| `robot_pvt_controller` | `robot_pvt_control/RobotPVTController` | Claims position, velocity, effort, kp, kd on all 25 joints. Spawned INACTIVE. |

`controller_manager.update_rate: 1000` (1 kHz) — matches the PVT slave 1 ms interpolation period.

The viewer config ([config/controllers_pvt_viewer.yaml](config/controllers_pvt_viewer.yaml)) omits `robot_pvt_controller` entirely; everything else is identical.

## PVT controller parameter schema

All array params follow the joint roster (25 entries, in the order declared by `joints:` in [controllers_pvt.yaml:124-148](config/controllers_pvt.yaml#L124-L148)).

| Param | Default | Purpose |
|-------|---------|---------|
| `joints` | 25-joint roster | Roster order — every other array indexes against this. |
| `drive_side_pd` | `true` | Drive executes the PD law (mode 5). Host streams setpoint only. |
| `body_group` | `"full"` (overridden per-launch) | `arms` / `arms_waist` / `legs` / `full`. Inactive joints get the FREE pattern. |
| `Kp[25]` | all `0.0` | Position gain. Ramp per bench protocol. |
| `Kd[25]` | all `0.0` | Velocity gain. Ramp first. |
| `Kd_damp[25]` | all `0.0` | Pure-brake gain, used when `~/damp` is latched. |
| `ff_gravity[25]` | all `false` | Per-joint host gravity feedforward enable. Flip one entry at a time. |
| `ff_inertia` | `[false]` | Inertia FF (global; not per-joint). |
| `ff_viscous` | `[false]` | Viscous-friction FF. |
| `mgl[25]` | populated from URDF via `gravcomp_from_urdf.py` against `relaxed_pose.yaml` | Peak \|τ_g(q_j)\| at the relaxed pose. Legs zeroed (closed-chain). |
| `J` | `[0.0]` | Joint inertia model (placeholder). |
| `Fv` | `[0.0]` | Viscous-friction coefficient (placeholder). |
| `comp_sign[25]` | all `1.0` | Per-joint sign for gravity FF. Flip per-joint after bench observation. |
| `hold_position` | `[0.0]` | Hold-position helper. |
| `lag_free` | `0.04` | Lag governor — free mode. |
| `lag_pause` | `0.14` | Lag governor — paused mode. |
| `alpha_slew` | `2.0` | Slew-rate alpha. |

`mgl` regeneration: re-run `python src/tools/ethercat_tools/scripts/gravcomp_from_urdf.py evaluate --pose src/bringup/robot_bringup/config/relaxed_pose.yaml` whenever the URDF, the relaxed pose, or the joint roster changes.

## Body-group mechanism

`body_group` is read by `robot_pvt_controller` at activation. Inactive joints get the FREE pattern (`q = measured`, `v = eff = Kp = Kd = 0`) so the drive PD law collapses to zero torque. Switching groups requires relaunch — the parameter is not exposed for live reconfigure. Every body-groups override file is just two lines:

```yaml
robot_pvt_controller:
  ros__parameters:
    body_group: "arms"
```

Even `full.yaml` is kept as an explicit override so every launch path goes through the same mechanism.

## Per-joint EtherCAT config (`ethercat_pvt/*.yaml`)

25 files, one per joint. Each carries:

- **Vendor / product ID** — MyActuator MT-Device family.
- **DC sync** — 1 ms cycle.
- **SDO init** — `mode_of_operation: 5` (PVT), `interpolation_time_period.base = 10^-3 s`, per-joint `max_torque` (500 for shoulders / wrists, 800 for legs).
- **RxPDO 0x1601** (22 B, host → drive) — control_word, target_position, target_velocity, torque_ff, PVT_KP, PVT_KD, mode_of_operation (locked to 5).
- **TxPDO 0x1A02** (22 B, drive → host) — status_word, position_actual, velocity_actual, torque_actual, error_code, mode_display, motor_temp, drive_temp, bus_voltage.
- **Conversion factors** per channel — position, velocity, torque, KP/KD. Per-family numbers come from the motor specs table in the project CLAUDE.md and from `recompute_torque_factors.py`. **Never hand-edit factors** — re-run the script.
- **Per-joint offset** — calibrated zero offset.

These files are the **source of truth**. The generator (`scripts/generate_ethercat_pvt.py`) bootstrapped them once from `arm_real_bringup/config/ethercat/` (CSP / mode 8) and is not run again. Hand-edits land here; the generator does not.

The readonly variant (`ethercat_pvt_readonly/`, generated at build) is a literal copy with two booleans flipped — no independent tuning.

## Generator script

[scripts/generate_ethercat_pvt.py](scripts/generate_ethercat_pvt.py) — one-shot bootstrap from the canonical CSP configs in `arm_real_bringup/config/ethercat/`. Documented transformations:

- `mode_of_operation: 8` (CSP) → `5` (PVT).
- Interpolation period `10 ms` → `1 ms` (drive cycle 100 Hz → 1 kHz).
- RxPDO `0x1600` → `0x1601` (adds `0x2000` PVT_KP and `0x2001` PVT_KD channels).
- TxPDO `0x1A00` → `0x1A02` (adds motor_temp, drive_temp, bus_voltage).
- Padding object indices remapped (`0x5FF1/0x5FF2` → `0x2FFD/0x2FFE`).

Not invoked at build time. The hand-tuned YAMLs are authoritative.

## Dependencies (and what each provides)

From [package.xml](package.xml):

| Package | Role |
|---------|------|
| `robot_pvt_control` | The `RobotPVTController` plugin. |
| `robot_filtered_joint_state_broadcaster` | IIR low-pass broadcaster (`/joint_states_filtered`). |
| `robot_drive_status_broadcaster` | TxPDO 0x1A02 telemetry decoder (`/drive_status`). |
| `robot_safety` | Supervisor + `safety_limits.yaml` (shared with the controller). |
| `ethercat_driver` | EtherCAT master ros2_control plugin (`ethercat_driver/EthercatDriver`). |
| `dual_arm_description` | URDF (`dual_arm.urdf.xacro` with `pvt_mode:=true`). |
| `controller_manager`, `robot_state_publisher`, `joint_state_publisher`, `rviz2`, `xacro` | Standard ROS 2 bring-up. |

## Conventions and gotchas

- **Real-time priority.** `ros2_control_node` runs `chrt -f 49`. Required for 1 kHz PVT.
- **`/joint_states` remap.** Standard JSB writes to `/joint_states_raw`; `joint_state_publisher` is the sole publisher on `/joint_states`. Do not undo this — host-side interpolation needs every drive sample.
- **Safety limits dual ownership.** Per-joint envelopes are loaded into both the supervisor and the controller from the same `robot_safety/config/safety_limits.yaml`. Edit the YAML; both nodes pick it up at relaunch.
- **Legs must stay revolute** (`fixed_legs:=false`). The controller_manager cannot claim hardware interfaces on fixed joints.
- **No rounding.** Joint limits, conversion factors, offsets, gravcomp `mgl` values, and the relaxed-pose radians are exact from the URDF source of truth. A rounding error in an EtherCAT factor once took two weeks to find. Copy exact values; never truncate for readability.

## Related reading

- Project [CLAUDE.md](../../../CLAUDE.md) — § "EtherCAT Real Hardware" for the broader stack (CSP / CST live in `arm_real_bringup`; this package is the PVT branch).
- `~/Documents/GitHub/ldr-harambe-docs/docs/tuning/ethercat_control_guide.html` §23 — the bench-rig gain ramp protocol with per-family numeric targets.
- `~/Documents/GitHub/ldr-harambe-docs/docs/strategy/sim_to_real_guide.html` — higher-level pipeline.
- [docs/ETHERCAT.md](../../../docs/ETHERCAT.md) — annotated per-joint slave config walkthrough (CSP variant; PVT layout adds the 0x1601 / 0x1A02 PDOs).
