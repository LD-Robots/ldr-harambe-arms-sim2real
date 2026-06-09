#ifndef HARAMBE_ANKLE_ETHERCAT_DRIVER__ANKLE_LINKAGE_DRIVER_HPP_
#define HARAMBE_ANKLE_ETHERCAT_DRIVER__ANKLE_LINKAGE_DRIVER_HPP_

#include "ethercat_driver/ethercat_driver.hpp"
#include "harambe_ankle_ethercat_driver/ankle_model.hpp"

#include <cstddef>
#include <vector>

namespace harambe_ankle_ethercat_driver
{

/**
 * AnkleLinkageDriver — shared base for the EtherCAT drivers that apply the
 * EXACT differential-ankle serial-chain transform.
 *
 * Holds the AnkleModel geometry, the joint-pair parsing, and the read() path
 * (motor → joint), which are identical regardless of the command-interface
 * layout. Subclasses implement write() for their specific layout:
 *   - HarambeEthercatDriver  → CSP/CST (position, effort, mode_of_operation)
 *   - HarambePvtDriver       → PVT     (position, velocity, effort, kp, kd)
 *
 * State indices 0/1/2 are position/velocity/effort in both layouts, so read()
 * is shared; higher state indices (status_word, temps, ...) pass through.
 *
 * Slot mapping (UNCHANGED from the legacy harambe_ethercat_driver so the
 * controller stack keeps working): the ankle_linkage_pairs string lists
 * "nameA,nameB" per pair, parsed into {pitch_idx <- nameA, roll_idx <- nameB}.
 * pitch_idx is physically motor A's slot, roll_idx motor B's. The exact FK
 * pitch output is stored in the pitch_idx slot, roll in the roll_idx slot;
 * IK theta_A goes to the pitch_idx slot, theta_B to the roll_idx slot.
 */
class AnkleLinkageDriver : public ethercat_driver::EthercatDriver
{
public:
  AnkleLinkageDriver() = default;

  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  struct AnkleLinkage
  {
    size_t pitch_idx;   // joint index: motor A slot; holds the pitch output
    size_t roll_idx;    // joint index: motor B slot; holds the roll output
    // Branch-continuity (IK) and Newton warm-start (FK) state, per pair.
    double last_cmd_thetaA = 0.0;
    double last_cmd_thetaB = 0.0;
    double last_fk_pitch = 0.0;
    double last_fk_roll = 0.0;
  };

  std::vector<AnkleLinkage> ankle_linkages_;
  AnkleModel model_;   // one shared model — both ankles assumed identical geometry

  void parseAnkleParams();      // read mm/deg URDF params -> AnkleParams -> model_
  void parseAnkleLinkages();    // parse "nameA,nameB;..." into ankle_linkages_
};

}  // namespace harambe_ankle_ethercat_driver

#endif  // HARAMBE_ANKLE_ETHERCAT_DRIVER__ANKLE_LINKAGE_DRIVER_HPP_
