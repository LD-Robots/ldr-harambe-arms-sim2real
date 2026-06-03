#ifndef HARAMBE_ETHERCAT_DRIVER__ANKLE_LINKAGE_DRIVER_HPP_
#define HARAMBE_ETHERCAT_DRIVER__ANKLE_LINKAGE_DRIVER_HPP_

#include "ethercat_driver/ethercat_driver.hpp"
#include "harambe_ethercat_driver/ankle_linkage.hpp"
#include <cstddef>
#include <vector>

namespace harambe_ethercat_driver
{

/**
 * AnkleLinkageDriver — shared base for the EtherCAT drivers that apply the
 * eccentric-pushrod ankle transform.
 *
 * Holds the linkage geometry, the joint-pair parsing, and the read() path
 * (motor → joint), which are identical regardless of the command-interface
 * layout. Subclasses implement write() for their specific layout:
 *   - HarambeEthercatDriver  → CSP/CST (position, effort, mode_of_operation)
 *   - HarambePvtDriver       → PVT     (position, velocity, effort, kp, kd)
 *
 * State indices 0/1/2 are position/velocity/effort in both layouts, so read()
 * is shared; higher state indices (status_word, temps, ...) pass through.
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
  struct AnkleLinkage {
    size_t pitch_idx;  // joint index: pitch output stored here, motor A read from here
    size_t roll_idx;   // joint index: roll output stored here, motor B read from here
  };

  std::vector<AnkleLinkage> ankle_linkages_;

  // Linkage geometry (meters) — overridden from URDF hardware params in on_init
  double r_exc_ = 0.025;
  double d_pitch_ = 0.03;
  double d_roll_ = 0.056;

  // Output sign correction (±1.0): applied to pitch/roll after kinematics
  double pitch_sign_ = 1.0;
  double roll_sign_ = 1.0;

  AnkleGeometry geometry() const { return {r_exc_, d_pitch_, d_roll_, pitch_sign_, roll_sign_}; }

  void parseAnkleLinkages();
};

}  // namespace harambe_ethercat_driver

#endif  // HARAMBE_ETHERCAT_DRIVER__ANKLE_LINKAGE_DRIVER_HPP_
