# Sim2Real Discussion

## Current State

### Simulation Side (working)
- Gazebo Harmonic with `use_sim:=true` xacro arg
- `JointTrajectoryController` at **100 Hz**
- Gazebo plugin as `hardware_interface`
- Controlled via MoveIt 2 or direct trajectory commands

### Real Hardware Side (operational)
- `use_sim:=false` switches URDF to load EtherCAT hardware plugin
- **EtherCAT stack:** Uses `ethercat_driver_ros2` (ICube Robotics) with `EcCiA402Drive` per-joint modules. Per-joint YAML slave configs define PDO mapping, conversion factors, and offsets.
  1. `ethercat_driver_ros2` — Generic EtherCAT framework (ICube, external). Provides `EthercatDriver` ros2_control plugin and `EcCiA402Drive` per-joint modules.
  2. `arm_ethercat_safety` — Safety monitor node (watchdog, e-stop, joint limits, faults)
  3. `arm_real_bringup` — Launch files and per-joint EtherCAT YAML configs

### Key differences sim vs real
| Aspect | Sim | Real |
|--------|-----|------|
| Update rate | 100 Hz | 1000 Hz |
| Hardware plugin | Gazebo | ethercat_driver/EthercatDriver |
| Controller config | `arm_control/config/` | `arm_real_bringup/config/` |
| sim_time | true | false |
| Safety monitor | N/A | arm_ethercat_safety |

---

## Agreed Decisions

*(To be filled during discussion)*

---

## Open Questions

*(To be filled during discussion)*
