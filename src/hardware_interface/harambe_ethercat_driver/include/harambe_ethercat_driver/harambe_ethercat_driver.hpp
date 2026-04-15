#ifndef HARAMBE_ETHERCAT_DRIVER__HARAMBE_ETHERCAT_DRIVER_HPP_
#define HARAMBE_ETHERCAT_DRIVER__HARAMBE_ETHERCAT_DRIVER_HPP_

#include "ethercat_driver/ethercat_driver.hpp"
#include <string>
#include <vector>

namespace harambe_ethercat_driver
{

/**
 * HarambeEthercatDriver — EtherCAT driver with nonlinear ankle linkage transform.
 *
 * Inherits EthercatDriver and overrides read()/write() to apply the eccentric-pushrod
 * kinematics for differential ankle joints. The two URDF ankle joints keep their
 * ec_modules (each mapped to one physical motor). The driver transforms the
 * hw_joint_states_/hw_joint_commands_ buffers in-place after read / before write.
 *
 * In the URDF <ros2_control> block, the first joint in each pair is motor A,
 * the second is motor B. After the transform, they become pitch and roll respectively.
 *
 * Eccentric-pushrod kinematics:
 *
 *   Read (motor → joint):
 *     pitch = arcsin(r_exc * (sin(θA) + sin(θB)) / (2 * d_pitch))
 *     roll  = arcsin(r_exc * (sin(θA) - sin(θB)) / (2 * d_roll))
 *
 *   Write (joint → motor):
 *     d_A = d_pitch * sin(pitch) + d_roll * sin(roll)
 *     d_B = d_pitch * sin(pitch) - d_roll * sin(roll)
 *     θA = arcsin(d_A / r_exc)
 *     θB = arcsin(d_B / r_exc)
 *
 * Configured via URDF hardware parameters:
 *   - ankle_linkage_pairs: "motorAJoint,motorBJoint;..." (semicolon-separated)
 *   - r_exc, d_pitch, d_roll: linkage geometry in meters
 */
class HarambeEthercatDriver : public ethercat_driver::EthercatDriver
{
public:
  HarambeEthercatDriver() = default;

  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct AnkleLinkage {
    size_t pitch_idx;  // joint index: pitch output stored here, motor A read from here
    size_t roll_idx;   // joint index: roll output stored here, motor B read from here
  };

  std::vector<AnkleLinkage> ankle_linkages_;

  // Linkage geometry (meters)
  double r_exc_ = 0.025;
  double d_pitch_ = 0.1;
  double d_roll_ = 0.03;

  // Output sign correction (±1.0): applied to pitch/roll after kinematics
  double pitch_sign_ = 1.0;
  double roll_sign_ = 1.0;

  void parseAnkleLinkages();
  double safe_asin(double x, const char * context) const;
};

}  // namespace harambe_ethercat_driver

#endif  // HARAMBE_ETHERCAT_DRIVER__HARAMBE_ETHERCAT_DRIVER_HPP_
