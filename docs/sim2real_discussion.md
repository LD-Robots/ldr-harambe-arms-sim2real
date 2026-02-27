# Sim2Real Discussion

## Current State

### Simulation Side (working)
- Gazebo Harmonic with `use_sim:=true` xacro arg
- `JointTrajectoryController` at **100 Hz**
- Gazebo plugin as `hardware_interface`
- Controlled via MoveIt 2 or direct trajectory commands

### Real Hardware Side (in progress)
- `use_sim:=false` switches URDF to load EtherCAT hardware plugin
- **4-layer stack:**
  1. `myactuator_ethercat` — Pure C++ EtherLab driver (headers + sources written)
  2. `myactuator_hardware` — ros2_control `SystemInterface` plugin (**compilation commented out**)
  3. `arm_ethercat_safety` — Safety monitor node (**compilation commented out**)
  4. `arm_real_bringup` — Launch files and real-hardware configs

### What exists but isn't compiling yet
- `myactuator_hardware` — plugin class is written but CMakeLists has build commented out
- `arm_ethercat_safety` — all 4 subsystems (watchdog, joint limits, fault handler, e-stop) are written but CMakeLists has build commented out

### Key differences sim vs real
| Aspect | Sim | Real |
|--------|-----|------|
| Update rate | 100 Hz | 1000 Hz |
| Hardware plugin | Gazebo | MyActuatorSystem (EtherCAT) |
| Controller config | `arm_control/config/` | `arm_real_bringup/config/` |
| sim_time | true | false |
| Safety monitor | N/A | arm_ethercat_safety |

---

## Agreed Decisions

*(To be filled during discussion)*

---

## Open Questions

*(To be filled during discussion)*
