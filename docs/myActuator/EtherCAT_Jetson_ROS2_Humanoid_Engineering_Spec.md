# EtherCAT + Jetson + ROS2 Humanoid

## Engineering-Grade Safety & Control Specification

### MYACTUATOR Drives · EtherLab (IgH) Master

------------------------------------------------------------------------

## ⚠️ NON-NEGOTIABLE SAFETY STATEMENT

This document assumes **full responsibility is on the integrator**.
Failure to follow these rules can result in: - uncontrolled joint
motion - structural damage - actuator destruction - severe injury

This specification is written to be **audit-grade** and **bring-up
safe**.

------------------------------------------------------------------------

# 1. Deterministic Real-Time Architecture

## 1.1 Thread Model (MANDATORY)

  Thread          Priority   Policy        Notes
  --------------- ---------- ------------- -----------------------
  EtherCAT RT     90--95     SCHED_FIFO    PDO exchange + safety
  ROS2 Executor   \< 50      SCHED_OTHER   Non-blocking
  Logging         \< 20      SCHED_OTHER   Async only

### CPU Isolation

-   1 dedicated core for EtherCAT
-   IRQ affinity pinned away from RT core
-   No dynamic memory allocation in RT thread

------------------------------------------------------------------------

## 1.2 EtherCAT RT Loop Template (C++)

``` cpp
while (rt_running) {
  wait_for_cycle();

  ec_receive_processdata(master, EC_TIMEOUTRET);

  read_status();
  enforce_limits();
  watchdog_check();

  if (drive_enabled) {
    apply_ramped_commands();
  } else {
    write_safe_outputs();
  }

  ec_send_processdata(master);
}
```

**Forbidden inside RT loop:** - malloc / new - ROS logging - DDS calls -
mutex locking

------------------------------------------------------------------------

# 2. Drive Enable State Machine (STRICT)

## 2.1 EtherCAT States

    INIT → PREOP → SAFEOP → OP

## 2.2 Drive States (0x6040 / 0x6041)

    DISABLED (0x0006)
       ↓
    READY     (0x0007)
       ↓
    ENABLED   (0x000F)

**Transition rule:**\
Advance only if verified in StatusWord.

------------------------------------------------------------------------

# 3. Mode Safety Contracts

## 3.1 CSP -- Position Mode Contract

MUST: - Align target = actual before enable - Enforce max Δposition per
cycle - Enforce absolute joint limits

FAILURE MODE: - Immediate joint jump

------------------------------------------------------------------------

## 3.2 CSV -- Velocity Mode Contract

MUST: - target_velocity = 0 before enable - acceleration limit active

FAILURE MODE: - Sudden spin-up

------------------------------------------------------------------------

## 3.3 CST -- Torque Mode Contract

MUST: - target_torque = 0 before enable - torque ramp limiter - software
current ceiling

FAILURE MODE: - Full-force impulse

------------------------------------------------------------------------

# 4. Unit Conversion Reference

## 4.1 Position

Given: - raw = ±65535 → ±180°

    rad = raw * (π / 65535)
    raw = rad * (65535 / π)

------------------------------------------------------------------------

## 4.2 Velocity

Given:

    RPM = (raw × 60) / 131072
    rad/s = raw × (2π / 131072)

------------------------------------------------------------------------

## 4.3 Torque

Given: - raw = thousandths of rated current - default rated current =
10A

    I = (raw / 1000) × I_rated
    τ_motor = Kt × I
    τ_joint = τ_motor × gear_ratio × efficiency

⚠️ Torque constant **must be measured or verified**.

------------------------------------------------------------------------

# 5. Commissioning Worksheet (PER JOINT)

  Item                      Value   Verified
  ------------------------- ------- ----------
  Joint name                        ⬜
  Gear ratio                        ⬜
  Direction (+/−)                   ⬜
  Zero offset                       ⬜
  Max position (rad)                ⬜
  Max velocity (rad/s)              ⬜
  Max torque (Nm)                   ⬜
  Emergency stop verified           ⬜
  Watchdog tested                   ⬜

------------------------------------------------------------------------

# 6. Fault Tree Analysis (FTA)

## 6.1 Immediate Disable Conditions

  Condition                  Action
  -------------------------- ---------
  StatusWord bit 3 (Fault)   Disable
  EtherCAT frame loss        Disable
  Watchdog timeout           Disable
  Joint limit exceeded       Disable
  DC sync lost               Disable

------------------------------------------------------------------------

## 6.2 Controlled Shutdown

Sequence: 1. Zero torque / velocity 2. Hold position 3. Disable drive 4.
Cut power stage (relay)

------------------------------------------------------------------------

# 7. Distributed Clocks Validation

MUST: - Sync0 enabled - Cycle jitter \< 50 µs - No drift accumulation

CHECK: - EtherCAT master DC diagnostics - Oscilloscope trigger on Sync0
if available

------------------------------------------------------------------------

# 8. Humanoid-Specific Hard Rules

-   No CST during initial bring-up
-   No torque \> 30% rated during testing
-   Always start in CSP
-   One joint at a time
-   Physical restraints during first enable

------------------------------------------------------------------------

# 9. Emergency Stop Architecture

RECOMMENDED: - Hardware relay cutting motor supply - Software disable
(0x6040 = 0) - ROS2 E-stop topic (latched)

All three should trigger together.

------------------------------------------------------------------------

# 10. Pre-Power Checklist (FINAL)

-   [ ] Mechanical clearance
-   [ ] Load disconnected
-   [ ] RT kernel verified
-   [ ] CPU isolated
-   [ ] DC sync active
-   [ ] Targets zeroed
-   [ ] Watchdog armed
-   [ ] E-stop tested
-   [ ] Limits active

------------------------------------------------------------------------

# END OF ENGINEERING SPEC
