# MyActuator RMD X V4 Motor Specifications

All motors share: EtherCAT & CAN BUS communication, crossed roller bearings, Y-connected 3-phase, insulation grade F, control accuracy <0.01 degree, dual absolute encoder (17-bit input).

## X4-36

| Parameter | Value | Unit |
|-----------|-------|------|
| Gear Ratio | 36 | – |
| Input Voltage | 24 | V |
| No Load Speed | 111 | RPM |
| No-Load Input Current | 0.9 | A |
| Rated Speed | 83 | RPM |
| Rated Torque | 10.5 | N·m |
| Rated Output Power | 100 | W |
| Rated Phase Current | 6.1 | A(rms) |
| Peak Torque | 34 | N·m |
| Peak Phase Current | 21.5 | A(rms) |
| Efficiency | 63.1 | % |
| Motor Back-EMF Constant | 6 | Vdc/Krpm |
| Module Torque Constant | 1.9 | N·m/A |
| Motor Phase Resistance | 0.35 | Ω |
| Motor Phase Inductance | 0.17 | mH |
| Pole Pairs | 13 | – |
| Back Drive Torque | 1.14 | N·m |
| Backlash | 10 | Arcmin |
| Axial Load (Suffer) | 1.3 | KN |
| Axial Load (Stress) | 1.3 | KN |
| Radial Load | 1.5 | KN |
| Inertia | 0.3 | Kg·cm² |
| Encoder | Dual ABS 17-bit (Input) / 18-bit (Output) | – |
| Weight | 0.36 | Kg |

## X6-60

| Parameter | Value | Unit |
|-----------|-------|------|
| Gear Ratio | 19.612 | – |
| Input Voltage | 48 | V |
| No Load Speed | 176 | RPM |
| No-Load Input Current | 0.9 | A |
| Rated Speed | 153 | RPM |
| Rated Torque | 20 | N·m |
| Rated Output Power | 320 | W |
| Rated Phase Current | 9.5 | A(rms) |
| Peak Torque | 60 | N·m |
| Peak Phase Current | 29.1 | A(rms) |
| Efficiency | 72.7 | % |
| Motor Back-EMF Constant | 16 | Vdc/Krpm |
| Module Torque Constant | 2.1 | N·m/A |
| Motor Phase Resistance | 0.41 | Ω |
| Motor Phase Inductance | 0.51 | mH |
| Pole Pairs | 10 | – |
| Back Drive Torque | 1.6 | N·m |
| Backlash | 10 | Arcmin |
| Axial Load (Suffer) | 1.8 | KN |
| Axial Load (Stress) | 0.8 | KN |
| Radial Load | 2 | KN |
| Inertia | 0.66 | Kg·cm² |
| Encoder | Dual ABS 17-bit (Input) / 17-bit (Output) | – |
| Weight | 0.82 | Kg |

## X8-120

| Parameter | Value | Unit |
|-----------|-------|------|
| Gear Ratio | 19.612 | – |
| Input Voltage | 48 | V |
| No Load Speed | 158 | RPM |
| No-Load Input Current | 1.6 | A |
| Rated Speed | 127 | RPM |
| Rated Torque | 43 | N·m |
| Rated Output Power | 574 | W |
| Rated Phase Current | 17.6 | A(rms) |
| Peak Torque | 120 | N·m |
| Peak Phase Current | 43.8 | A(rms) |
| Efficiency | 79 | % |
| Motor Back-EMF Constant | 19.2 | Vdc/Krpm |
| Module Torque Constant | 2.4 | N·m/A |
| Motor Phase Resistance | 0.18 | Ω |
| Motor Phase Inductance | 0.31 | mH |
| Pole Pairs | 10 | – |
| Back Drive Torque | 3.21 | N·m |
| Backlash | 10 | Arcmin |
| Axial Load (Suffer) | 4 | KN |
| Axial Load (Stress) | 1 | KN |
| Radial Load | 4.5 | KN |
| Inertia | 1.5 | Kg·cm² |
| Encoder | Dual ABS 17-bit (Input) / 17-bit (Output) | – |
| Weight | 1.40 | Kg |

## EtherCAT Conversion Factors

Derived from output encoder resolution:

| Motor | Output Encoder | Position Cmd (counts/rad) | Position State (rad/count) | Velocity | Effort Cmd | Effort State |
|-------|---------------|--------------------------|---------------------------|----------|------------|-------------|
| X4-36 | 18-bit (262144/rev) | 41722.0 | 2.397e-5 | same as position | 50.0 | 0.02 |
| X6-60 | 17-bit (131072/rev) | 20861.0 | 4.794e-5 | same as position | 50.0 | 0.02 |
| X8-120 | 17-bit (131072/rev) | 20861.0 | 4.794e-5 | same as position | 50.0 | 0.02 |

**X8 uses identical conversion factors to X6** (same gear ratio + same output encoder resolution).

Direction reversal: negate ALL factor signs (position, velocity, effort).
