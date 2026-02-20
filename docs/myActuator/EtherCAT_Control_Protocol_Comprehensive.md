# EtherCAT Control Protocol -- Comprehensive Technical Reference

**Version:** V1.0\
**Date:** 2024.12\
**Manufacturer:** Suzhou Micro Actuator Technology Co., Ltd. 

------------------------------------------------------------------------

# ⚠️ CRITICAL SAFETY NOTICE

Incorrect EtherCAT configuration or incorrect Control Word sequencing
**can cause:** - Sudden motor motion - High current draw - Mechanical
damage - Equipment destruction - Personal injury

ALWAYS: - Verify wiring before power-on - Start with zero
velocity/torque targets - Confirm status transitions via 0x6041 before
proceeding - Test with mechanical load disconnected when possible

------------------------------------------------------------------------

# 1. System Overview

EtherCAT uses a master--slave architecture:

-   1 Master (typically TwinCAT)
-   Multiple servo slave drives
-   Communication speed: 100 Mbit/s
-   Cable: CAT5 or higher
-   Max distance between stations: 100 meters

The master sends one Ethernet frame per cycle. Each slave: 1. Extracts
its data 2. Inserts feedback data 3. Passes frame to next slave 4. Frame
returns to master

------------------------------------------------------------------------

# 2. Communication & Hardware

## 2.1 EtherCAT Interface Pins

  Signal   Description
  -------- -------------------
  RX+      Receive Positive
  RX-      Receive Negative
  TX+      Transmit Positive
  TX-      Transmit Negative

Use shielded twisted pair cable.

------------------------------------------------------------------------

# 3. Control & State Management

Drive states are controlled via:

-   0x6040 → Control Word (WRITE / RxPDO)
-   0x6041 → Status Word (READ / TxPDO)

## 3.1 0x6040 -- Control Word (UINT16)

  Bit      Name               Function
  -------- ------------------ ----------------
  0        Switch On          1 = valid
  1        Enable Voltage     1 = valid
  2        Quick Stop         0 = active
  3        Enable Operation   1 = valid
  4--6     Mode specific      Mode dependent
  7--8     Reserved           ---
  9        Mode specific      Mode dependent
  10--15   Reserved           ---

### Safe Enable Sequence

1.  Write 0x6040 = 6\
2.  Write 0x6040 = 7\
3.  Write 0x6040 = 15

Verify state change via 0x6041 after each step.

------------------------------------------------------------------------

## 3.2 0x6041 -- Status Word (UINT16)

  Bit      Name
  -------- -----------------------
  0        Ready to switch on
  1        Switched on
  2        Operation enabled
  3        Fault
  4        Voltage enabled
  5        Quick stop
  6        Switch on disabled
  7        Warning
  9        Remote
  10       Target reached
  11       Internal limit active
  12--13   Mode specific

------------------------------------------------------------------------

# 4. Operating Modes

## 4.1 0x6060 -- Mode of Operation (SINT)

  Value   Mode
  ------- -----------------------------------
  0x08    Cyclic Synchronous Position (CSP)
  0x09    Cyclic Synchronous Velocity (CSV)
  0x0A    Cyclic Synchronous Torque (CST)

Activated immediately after write.

## 4.2 0x6061 -- Mode Display (SINT)

Readback of current operating mode.

------------------------------------------------------------------------

# 5. Cyclic Synchronous Position Mode (CSP)

## Activation

Write:

    0x6060 = 8

## Required Objects

  Index    Name                    Direction
  -------- ----------------------- -----------
  0x607A   Target Position         RxPDO
  0x6064   Position Actual Value   TxPDO
  0x60B1   Velocity Offset         RxPDO
  0x60B2   Torque Offset           RxPDO

## Units

Position: - -65535 → -180° - 65535 → +180°

## Safe Startup Procedure

1.  Set 0x6060 = 8
2.  Read 0x6064 (actual position)
3.  Write same value to 0x607A
4.  Enable drive (6 → 7 → 15)
5.  Begin cyclic target updates

------------------------------------------------------------------------

# 6. Cyclic Synchronous Velocity Mode (CSV)

## Activation

    0x6060 = 9

## Required Objects

  Index    Name
  -------- -----------------------
  0x60FF   Target Velocity
  0x606C   Velocity Actual Value
  0x60B1   Velocity Offset
  0x60B2   Torque Offset

## CRITICAL SAFETY STEP

Before enabling:

    0x60FF = 0

Drive runs immediately at target velocity when enabled.

## Velocity Conversion

RPM = (X × 60) / 131072

------------------------------------------------------------------------

# 7. Cyclic Synchronous Torque Mode (CST)

## Activation

    0x6060 = 10

## Required Objects

  Index    Name
  -------- ---------------------
  0x6071   Target Torque
  0x6077   Torque Actual Value
  0x60B2   Torque Offset

## Units

Torque values are in: - Thousandths (0.1%) of rated current - Default
rated current: 10A

Example: Value 1000 → 100% rated current (10A)

## Safe Startup

1.  Set 0x6071 = 0
2.  Enable drive
3.  Apply torque gradually

------------------------------------------------------------------------

# 8. Object Dictionary Summary

  Index    Name              Type   Direction
  -------- ----------------- ------ -----------
  0x6040   Control Word      UINT   RxPDO
  0x6041   Status Word       UINT   TxPDO
  0x6060   Mode              SINT   RxPDO
  0x6061   Mode Display      SINT   TxPDO
  0x6064   Position Actual   DINT   TxPDO
  0x606C   Velocity Actual   DINT   TxPDO
  0x6071   Target Torque     INT    RxPDO
  0x6077   Torque Actual     INT    TxPDO
  0x607A   Target Position   DINT   RxPDO
  0x60B0   Position Offset   DINT   RxPDO
  0x60B1   Velocity Offset   DINT   RxPDO
  0x60B2   Torque Offset     INT    RxPDO

------------------------------------------------------------------------

# 9. TwinCAT Configuration Procedure

1.  Scan Devices
2.  Accept detected slaves
3.  Configure I/O ports
4.  Set DC mode to "DC-Synchron"
5.  Restart TwinCAT
6.  Set ModeOfOperation (8/9/10)
7.  Write ControlWord = 15 to start
8.  Write ControlWord = 0 to stop

------------------------------------------------------------------------

# 10. Recommended Safety Practices

-   Never enable with non-zero velocity or torque.
-   Monitor 0x6041 bit 3 (Fault).
-   Monitor 0x6041 bit 11 (Internal limit).
-   Confirm Target Reached (bit 10) before motion step change.
-   Test with reduced Max Torque.
-   Use mechanical end stops during initial testing.

------------------------------------------------------------------------

# END OF DOCUMENT
