# EtherCAT + Jetson + ROS2 Humanoid Integration Guide

## For MYACTUATOR Drives (EtherLab / IgH Master)

------------------------------------------------------------------------

# ⚠️ CRITICAL SAFETY WARNING

This actuator can: - Apply full torque instantly - Move to a commanded
position immediately - Hold torque indefinitely - Damage gears,
linkages, or people

There are NO built-in safety guardrails when using EtherLab. You are
responsible for implementing all safety logic.

------------------------------------------------------------------------

# 1. SYSTEM ARCHITECTURE (RECOMMENDED)

## 1.1 Layer Separation (MANDATORY)

### Layer A --- Real-Time EtherCAT Thread (Hard RT)

-   Runs at fixed frequency (e.g., 1 kHz)
-   Exchanges PDO data
-   Implements state machine
-   Enforces limits
-   Handles watchdog
-   Handles faults

### Layer B --- ROS2 / ros2_control (Soft RT)

-   Controllers
-   Trajectory generation
-   Planning
-   Logging
-   Diagnostics

⚠ NEVER block the EtherCAT thread with ROS2 calls.

Use: - Lock-free ring buffer - Atomic struct - Shared memory double
buffer

------------------------------------------------------------------------

# 2. ETHERLAB MASTER SETUP (JETSON)

## 2.1 Kernel

MUST DO: - Use PREEMPT_RT kernel - Isolate CPU core for EtherCAT -
Disable CPU frequency scaling - Disable power saving modes

Jetson recommendation: - Pin EtherCAT thread to isolated core - Set
SCHED_FIFO priority \> 80

------------------------------------------------------------------------

## 2.2 Network Interface

MUST DO: - Dedicated Ethernet port - No switch during commissioning -
Disable NetworkManager on that interface

Check: - No packet loss - Stable cycle timing - DC synchronization
configured

------------------------------------------------------------------------

# 3. DRIVE STATE MACHINE (MANDATORY)

EtherCAT states: INIT → PREOP → SAFEOP → OP

Drive states via 0x6040 (ControlWord).

## Safe Enable Sequence (REQUIRED)

1.  Write Mode (0x6060)
2.  Zero or align target
3.  Write 0x6040 = 0x0006
4.  Verify status
5.  Write 0x6040 = 0x0007
6.  Verify status
7.  Write 0x6040 = 0x000F
8.  Verify Operation Enabled

NEVER jump directly to 0x000F.

------------------------------------------------------------------------

# 4. MODE-SPECIFIC REQUIREMENTS

## 4.1 CSP (Position Mode)

MUST DO: - Read actual position (0x6064) - Set target position (0x607A)
= actual before enabling

RECOMMENDED: - Add position ramp limiter - Add max delta per cycle

------------------------------------------------------------------------

## 4.2 CSV (Velocity Mode)

MUST DO: - Set 0x60FF = 0 before enabling

RECOMMENDED: - Limit max RPM - Add acceleration limiter

Velocity conversion: RPM = (X × 60 / 131072) rad/s = X × (2π / 131072)

------------------------------------------------------------------------

## 4.3 CST (Torque Mode)

MUST DO: - Set 0x6071 = 0 before enabling - Apply torque ramp

Torque unit: - Raw value = thousandths of rated current - Rated current
default = 10A

RECOMMENDED: - Never exceed 30--40% rated current during testing -
Implement torque ceiling in RT layer

------------------------------------------------------------------------

# 5. HUMANOID-SPECIFIC SAFETY RULES

## 5.1 Watchdog (MANDATORY)

If no new command in N cycles: - Freeze position (CSP) - Zero velocity
(CSV) - Zero torque (CST) - Disable after timeout

------------------------------------------------------------------------

## 5.2 Joint Limits (MANDATORY)

Implement: - Soft limits (URDF) - Hard safety limit in RT loop -
Mechanical end-stop detection

Monitor: - StatusWord bit 11 (Internal limit active) - StatusWord bit 3
(Fault)

------------------------------------------------------------------------

## 5.3 Slew Rate Limiting (MANDATORY)

Limit per-cycle change: - Δposition - Δvelocity - Δtorque

Prevents: - Gear shock - Structural damage - Harmonic oscillation

------------------------------------------------------------------------

# 6. DISTRIBUTED CLOCKS (DC)

MUST DO: - Configure DC sync - Validate sync jitter - Match cycle time
exactly

Failure to configure DC: - Oscillation - Following errors - Torque
ripple

------------------------------------------------------------------------

# 7. ROS2 INTEGRATION

## 7.1 ros2_control Hardware Interface

Expose: - position - velocity - effort

DO NOT: - Call EtherCAT directly from ROS2 callbacks - Allocate memory
in real-time loop - Log inside RT loop

------------------------------------------------------------------------

# 8. COMMISSIONING PROCEDURE

Step 1: - Disconnect mechanical load

Step 2: - Enable CSP only - Move small increments - Verify direction

Step 3: - Validate limits - Validate emergency stop

Step 4: - Increase torque gradually

------------------------------------------------------------------------

# 9. NICE TO HAVE (BUT STRONGLY RECOMMENDED)

-   Redundant E-stop chain
-   Hardware torque limiter
-   Thermal monitoring
-   EtherCAT watchdog monitoring
-   Separate power stage enable relay
-   Brownout detection

------------------------------------------------------------------------

# 10. FAILURE RESPONSE PLAN

On fault: - Write ControlWord = 0 - Log fault code - Require manual
reset sequence

Never auto-retry enable without inspection.

------------------------------------------------------------------------

# 11. PERFORMANCE TARGETS (HUMANOID)

Recommended: - 1 kHz cycle - \< 50 µs jitter - DC sync enabled - Torque
ramp ≤ 5% rated per cycle

------------------------------------------------------------------------

# 12. FINAL SAFETY CHECKLIST

Before full-power operation:

-   [ ] PREEMPT_RT kernel active
-   [ ] CPU isolated
-   [ ] Dedicated NIC
-   [ ] DC sync verified
-   [ ] Targets zeroed
-   [ ] Watchdog active
-   [ ] Slew limits active
-   [ ] Joint limits active
-   [ ] E-stop tested
-   [ ] Mechanical clearance confirmed

------------------------------------------------------------------------

# END OF DOCUMENT
