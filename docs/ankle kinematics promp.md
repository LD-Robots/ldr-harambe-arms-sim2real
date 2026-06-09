# Implementation Prompt — Differential Ankle Kinematics (ROS 2)

> Hand this document to a coding assistant. It specifies a ROS 2 package that implements the
> forward and inverse kinematics of a 2-DOF differential ankle driven by two motor-cranks and
> two pushrods. All formulas below are exact and have been numerically validated; implement them
> verbatim. Where a value is "assumed", expose it as a runtime-editable parameter.

---

## 1. What you are building

A ROS 2 package (`ankle_kinematics`) that converts between:

- **two motor angles** `θA, θB` (crank/cam rotation, measured by encoders), and
- **two foot angles** `pitch` (plantar/dorsiflexion) and `roll` (inversion/eversion).

The mechanism is a **differential**: the two pushrods act on the foot so that moving both rods the
same way produces pitch, and moving them oppositely produces roll. The foot hangs on **two stacked
revolute hinges** (a pitch hinge above a roll hinge, offset vertically), so the chain is serial, not
a single point gimbal. The package must provide exact FK (encoders → pose) and exact IK
(commanded pose → motor angles), plus a documented first-order approximation used for seeding and
as an optional lightweight mode.

---

## 2. Coordinate frames and sign conventions

Right-handed frame, origin **O at the pitch-hinge axis**:

- **x** — anterior, toward the toe (+).
- **y** — lateral, toward the subject's right (+).
- **z** — up (+).

Joint angles (these are the reported/URDF angles):

- `pitch` — rotation that raises/lowers the toe. **+pitch = toe up = dorsiflexion.**
- `roll`  — rotation that tilts the foot side to side. **+roll = right side down.**

Serial chain order (proximal → distal):

```
shank (ankle_base) ── pitch hinge @ O (axis ‖ y) ── link ── roll hinge (axis ‖ x), located 28 mm below O ── foot
```

Internal rotation angles used by the math (note the pitch sign flip vs the reported joint angle):

- `phi_p = -pitch`  (the R_y angle; +phi_p drops the toe)
- `phi_r =  roll`   (the R_x angle)

Rotation matrices:

```
R_x(a) = [[1, 0,      0     ],
          [0, cos(a), -sin(a)],
          [0, sin(a),  cos(a)]]

R_y(a) = [[ cos(a), 0, sin(a)],
          [ 0,      1, 0     ],
          [-sin(a), 0, cos(a)]]
```

---

## 3. Parameters (ROS parameters, units, defaults)

Lengths are entered in **millimetres** for user convenience; convert to metres internally for
TF/poses. Angles are **radians** internally; parameter for the motor limit is in degrees.

| Parameter (ROS name)        | Symbol | Default  | Meaning |
|-----------------------------|--------|----------|---------|
| `crank_throw_mm`            | r      | 25.0     | Crank pin radius (throw) |
| `mount_foreaft_mm`          | Lp     | 63.0     | Rod-mount fore/aft offset from hinge (toward toe, +x) — the pitch lever |
| `mount_separation_mm`       | S      | 60.0     | Lateral spacing between the two rod mounts (mounts at ±S/2 in y) |
| `cam_foreaft_mm`            | a      | 78.0     | Cam (motor) axis fore/aft offset from pitch axis (toward toe, +x) |
| `hinge_spacing_mm`          | h      | 28.0     | Vertical gap between stacked hinges (pitch above roll) |
| `rod_a_length_mm`           | L_A    | 255.0    | Pushrod A length (drives right rod, motor A) |
| `rod_b_length_mm`           | L_B    | 180.0    | Pushrod B length (drives left rod, motor B) |
| `cam_a_height_mm`           | H_A    | derived  | Cam A axis height above pitch axis (+z) |
| `cam_b_height_mm`           | H_B    | derived  | Cam B axis height above pitch axis (+z) |
| `lock_heights_to_rods`      | —      | true     | If true, H_A,H_B are computed from rod lengths each update; else use the values above |
| `mount_drop_mm`             | δ      | 0.0      | Vertical drop from the roll hinge down to the rod-mount plane |
| `motor_limit_deg`           | θ_lim  | 80.0     | Motor travel limit, ± about rest (rest = pins horizontal) |
| `update_rate_hz`            | —      | 200.0    | Control/publish loop rate |
| `base_frame`                | —      | `ankle_base` | TF parent |
| `pitch_link_frame`          | —      | `ankle_pitch_link` | intermediate link frame |
| `foot_frame`                | —      | `ankle_roll_link`  | foot frame |

Provide a `config/ankle.yaml`:

```yaml
ankle_kinematics:
  ros__parameters:
    crank_throw_mm: 25.0
    mount_foreaft_mm: 63.0
    mount_separation_mm: 60.0
    cam_foreaft_mm: 78.0
    hinge_spacing_mm: 28.0
    rod_a_length_mm: 255.0
    rod_b_length_mm: 180.0
    cam_a_height_mm: 226.5     # used only if lock_heights_to_rods=false
    cam_b_height_mm: 151.3
    lock_heights_to_rods: true
    mount_drop_mm: 0.0
    motor_limit_deg: 80.0
    update_rate_hz: 200.0
    base_frame: ankle_base
    pitch_link_frame: ankle_pitch_link
    foot_frame: ankle_roll_link
```

Register an `add_on_set_parameters_callback` (rclcpp) / `add_on_set_parameters_callback` (rclpy)
so the geometry parameters — especially `cam_a_height_mm`, `cam_b_height_mm`, `mount_drop_mm`,
`lock_heights_to_rods` — can be corrected at runtime without restarting.

---

## 4. Derived cam heights (ASSUMPTION — make overridable)

If `lock_heights_to_rods` is true, compute the cam heights from the rod lengths under the
assumption that **at neutral (pitch = roll = 0) the rods hang straight from cam to mount**:

```
base = (a - Lp)^2 + (r - S/2)^2
H_A  = sqrt(L_A^2 - base) - h - δ
H_B  = sqrt(L_B^2 - base) - h - δ
```

With the defaults this gives `H_A ≈ 226.51 mm`, `H_B ≈ 151.30 mm`, and a derived motor spacing
`H_A - H_B ≈ 75.2 mm` (consistent with the physical 75 mm). If measured heights are supplied,
set `lock_heights_to_rods: false` and the rods are no longer assumed vertical.

---

## 5. Geometry primitives

**Rod mount points in the foot frame** (relative to the roll hinge):

```
m_A = (Lp, +S/2, -δ)        # right rod
m_B = (Lp, -S/2, -δ)        # left rod
```

**Map a foot point to the base frame** (this is the serial chain — the entire source of cross-coupling,
because the roll hinge and mounts sit `h` below the pitch axis and are carried around by R_y):

```
P(m, phi_p, phi_r) = R_y(phi_p) · ( R_x(phi_r) · m + (0, 0, -h) )
```

**Crank pin positions in the base frame** (cams are fixed; both cam axes on the centreline y = 0;
rotation is in the y–z plane, axis ‖ x; rest pins horizontal, A toward +y, B toward −y):

```
pin_A(θA) = ( a,  r·cos(θA),  H_A + r·sin(θA) )
pin_B(θB) = ( a, -r·cos(θB),  H_B + r·sin(θB) )
```

**Rod length constraints** (the exact kinematic relationship):

```
|| pin_A(θA) - P(m_A, phi_p, phi_r) || = L_A
|| pin_B(θB) - P(m_B, phi_p, phi_r) || = L_B
```

---

## 6. Inverse kinematics — pose → motor angles (CLOSED FORM)

Given desired `(pitch, roll)`:

1. Set `phi_p = -pitch`, `phi_r = roll`.
2. Compute mounts in base frame: `M_A = P(m_A, phi_p, phi_r) = (mx, my, mz)`, likewise `M_B`.
3. Each rod constraint reduces to `A·cos(θ) + B·sin(θ) = K`. Expanding the squared distance and
   cancelling `r²(cos²+sin²)=r²`:

   **Rod A** (uses `M_A`, `H_A`, `L_A`):
   ```
   A1 = -2·r·my
   B1 =  2·r·(H_A - mz)
   K1 =  L_A^2 - r^2 - (a - mx)^2 - my^2 - (H_A - mz)^2
   ```

   **Rod B** (uses `M_B`, `H_B`, `L_B`; note the +sign on A2 from pin_B's −r·cos term):
   ```
   A2 =  2·r·my
   B2 =  2·r·(H_B - mz)
   K2 =  L_B^2 - r^2 - (a - mx)^2 - my^2 - (H_B - mz)^2
   ```

4. Solve each:
   ```
   R = hypot(A, B)
   if |K / R| > 1  -> pose is UNREACHABLE for this rod (report, clamp; do not command)
   psi = atan2(B, A)
   θ   = psi ± acos(K / R)        # two roots
   ```
   Pick the root within `[-θ_lim, +θ_lim]` that is **closest to the previous commanded angle**
   (branch continuity). Wrap to (−π, π] before the limit check.

Return `θA, θB` and a `reachable` flag.

---

## 7. Forward kinematics — motor angles → pose (NEWTON)

No closed form; solve the two constraints for `(phi_p, phi_r)` with a 2-variable Newton iteration.

```
residual(phi_p, phi_r):
    F1 = || pin_A(θA) - P(m_A, phi_p, phi_r) ||^2 - L_A^2
    F2 = || pin_B(θB) - P(m_B, phi_p, phi_r) ||^2 - L_B^2
    return (F1, F2)

seed (first-order, see §8):
    dA = r·sin(θA);  dB = r·sin(θB)
    phi_p = -asin(clamp((dA+dB)/(2·Lp), -1, 1))    # note minus: phi_p = -pitch
    phi_r =  asin(clamp((dA-dB)/S,      -1, 1))

iterate (max ~20):
    F = residual(phi_p, phi_r)
    J = Jacobian(residual)            # central difference, step 1e-6 rad, is sufficient
    solve J·Δ = -F  (2x2)
    (phi_p, phi_r) += Δ
    stop when |Δ| < 1e-9

report:
    pitch = -phi_p
    roll  =  phi_r
```

Warm-start each call from the previous solution (guarantees the correct branch and 2–3 iteration
convergence in a real-time loop). An analytic Jacobian is optional; central difference is fine at
these rates.

---

## 8. First-order approximation (seed + optional lightweight mode)

Planar/decoupled model, valid near neutral:

```
dA = r·sin(θA)              # rod-A vertical travel
dB = r·sin(θB)
pitch ≈  asin( (dA + dB) / (2·Lp) )
roll  ≈  asin( (dA - dB) / S )

# inverse:
dA = Lp·sin(pitch) + (S/2)·sin(roll)
dB = Lp·sin(pitch) - (S/2)·sin(roll)
θ  = asin( clamp(d / r, -1, 1) )
```

Expose `use_first_order` (bool, default false). When true, skip Newton/closed-form-3D and use these.
**Accuracy with default parameters:** vs the exact model, this deviates by ≤ ~1.2° in pitch and
≤ ~2.3° in roll across the full ±80° motor range, with < 0.5° cross-axis bleed. Acceptable for many
controllers; for precision use the exact model or a one-step Newton refinement of the first-order seed.

---

## 9. Limits, singularities, validity

- Clamp motor commands to `[-θ_lim, +θ_lim]`. If IK requests beyond, clamp to nearest reachable and
  publish a warning + diagnostic; never publish an out-of-range command.
- IK unreachable when `|K/R| > 1` for either rod (outside the workspace). Report `reachable=false`.
- The crank is near top-dead-centre as `θ → ±90°` (sensitivity `d(d)/dθ = r·cos(θ) → 0`): low motion
  per motor degree, high torque. Not a math singularity but flag in docs.
- Branch continuity: always select the IK root nearest the previous command; warm-start FK.
- Validate `base = (a-Lp)^2 + (r-S/2)^2 < min(L_A,L_B)^2` when locking heights to rods (else sqrt of
  negative — surface a clear parameter error).

---

## 10. ROS 2 interface

Target ROS 2 Humble or Jazzy. Prefer **C++ / rclcpp + Eigen** for the loop; rclpy acceptable.

**Node:** `ankle_kinematics_node`

**Subscriptions**
- `~/cmd_pose` (`ankle_interfaces/msg/AnklePose` → `{float64 pitch; float64 roll}`, radians):
  run IK, publish `~/motor_cmd`.
- `~/motor_states` (`sensor_msgs/msg/JointState`, names `["ankle_motor_a","ankle_motor_b"]`,
  position in rad): run FK, publish `~/foot_pose`, the URDF joint states, and TF.

**Publications**
- `~/motor_cmd` (`sensor_msgs/msg/JointState`): positions `[θA, θB]`, names as above.
- `~/foot_pose` (`ankle_interfaces/msg/AnklePoseStamped` or `geometry_msgs/msg/Vector3Stamped`
  with `x=pitch, y=roll`).
- `~/ankle_joint_states` (`sensor_msgs/msg/JointState`): names `["ankle_pitch_joint","ankle_roll_joint"]`,
  positions `[pitch, roll]` — feed this to `robot_state_publisher` for visualization.
- TF: broadcast `base_frame → pitch_link_frame → foot_frame`. Prefer feeding the joint states to
  `robot_state_publisher` (with the URDF in §11) rather than hand-rolling TF; provide a
  `TransformBroadcaster` fallback.

**Services**
- `~/compute_ik` (`ankle_interfaces/srv/ComputeIK`): request `{pitch, roll}` → response
  `{motor_a, motor_b, bool reachable}`.
- `~/compute_fk` (`ankle_interfaces/srv/ComputeFK`): request `{motor_a, motor_b}` → response `{pitch, roll}`.

**Interface package** `ankle_interfaces`:
```
# msg/AnklePose.msg
float64 pitch   # rad, +dorsiflexion
float64 roll    # rad, +right-side-down

# srv/ComputeIK.srv
float64 pitch
float64 roll
---
float64 motor_a
float64 motor_b
bool reachable

# srv/ComputeFK.srv
float64 motor_a
float64 motor_b
---
float64 pitch
float64 roll
```

---

## 11. URDF (for visualization via robot_state_publisher)

Two revolute joints matching the convention. Note the pitch joint axis sign so that positive
`pitch` is toe-up; the roll link origin is `h` below the pitch axis.

```xml
<joint name="ankle_pitch_joint" type="revolute">
  <parent link="ankle_base"/>
  <child  link="ankle_pitch_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis   xyz="0 -1 0"/>   <!-- +pitch = toe up -->
  <limit lower="-0.40" upper="0.40" effort="50" velocity="5"/>  <!-- set from computed range -->
</joint>

<joint name="ankle_roll_joint" type="revolute">
  <parent link="ankle_pitch_link"/>
  <child  link="ankle_roll_link"/>
  <origin xyz="0 0 -0.028" rpy="0 0 0"/>   <!-- hinge_spacing in metres -->
  <axis   xyz="1 0 0"/>
  <limit lower="-0.96" upper="0.96" effort="50" velocity="5"/>
</joint>
```

The pushrods/cranks are not part of this kinematic tree (they're the actuation that produces the two
joint angles); model them as fixed visuals or a separate display if desired.

---

## 12. Units, structure, and quality

- **Internal:** radians for angles, metres for lengths in TF/poses; keep mm parameters and convert once.
- **Package layout:** `ankle_kinematics/` (node, `src/ankle_kinematics.{cpp,hpp}` with a pure
  `AnkleModel` class free of ROS deps for testability), `config/ankle.yaml`, `launch/ankle.launch.py`,
  `urdf/ankle.urdf.xacro`, and `ankle_interfaces/`.
- The `AnkleModel` class exposes `ik(pitch,roll) -> {θA,θB,reachable}`, `fk(θA,θB) -> {pitch,roll}`,
  `derive_heights()`, and `set_params(...)`. Unit-test it without a running ROS graph.

**Acceptance tests** (gtest / pytest on `AnkleModel`):
1. Neutral: `fk(0,0) == (0,0)` within 1e-9.
2. Round trip: for a grid of reachable `(pitch,roll)`, `fk(ik(pitch,roll)) == (pitch,roll)` within 1e-4 rad.
3. Model agreement (defaults): over the ±80° motor grid, `|pitch_exact - pitch_firstorder| ≤ 1.2°`
   and `|roll_exact - roll_firstorder| ≤ 2.3°`.
4. Cross-axis bleed: pure-pitch motor command (`θA=θB`) yields `|roll| < 0.5°`; pure-roll
   (`θA=-θB`) yields `|pitch| < 0.5°`.
5. Reachability: a known out-of-workspace pose returns `reachable=false` and the node does not
   publish a motor command beyond `±θ_lim`.

---

## 13. Assumptions (explicit — verify against hardware)

1. **Both cam (motor) axes lie on the sagittal centreline, y = 0.** Pin A points toward +y at rest,
   pin B toward −y; each crank rotates in the y–z plane (axis ‖ x).
2. **Crank pin vertical = H + r·sin(θ)**, with rest defined at pins horizontal (θ = 0) and travel
   symmetric about rest, `θ ∈ [-θ_lim, +θ_lim]`.
3. **Cam heights are derived from rod lengths assuming the rods are vertical at neutral**
   (unless `lock_heights_to_rods=false` and measured heights are provided).
4. **Rod mounts on the foot:** `Lp` toward the toe, `±S/2` laterally, and `δ` below the roll hinge
   (default δ = 0, i.e. mounts at roll-hinge level).
5. **Stacked serial hinges:** pitch hinge above roll hinge by `h`; pitch ‖ y, roll ‖ x; not a
   coincident gimbal. This offset is what produces the (small) cross-coupling.
6. **Rigid pushrods, ideal pin/rod-end joints, no backlash, no compliance.** No rod-end angular
   limit is modelled — if the spherical rod-ends bind before the motor limit, add that as a separate
   clamp on the workspace.
7. **Sign conventions:** +pitch = toe up (dorsiflexion); +roll = right side down. Internally
   `phi_p = -pitch`, `phi_r = roll`. Adjust axis signs in the URDF if your hardware differs.
8. **First-order decoupling** (`pitch≈(dA+dB)/2Lp`, `roll≈(dA−dB)/S`) is accurate near neutral and
   within ~1–2° at the extremes for the default geometry; the exact model is authoritative.