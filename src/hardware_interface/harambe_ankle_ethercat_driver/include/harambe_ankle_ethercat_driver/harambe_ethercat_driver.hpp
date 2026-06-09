#ifndef HARAMBE_ANKLE_ETHERCAT_DRIVER__HARAMBE_ETHERCAT_DRIVER_HPP_
#define HARAMBE_ANKLE_ETHERCAT_DRIVER__HARAMBE_ETHERCAT_DRIVER_HPP_

#include "harambe_ankle_ethercat_driver/ankle_linkage_driver.hpp"

namespace harambe_ankle_ethercat_driver
{

/**
 * HarambeEthercatDriver — CSP/CST EtherCAT driver with the EXACT ankle
 * serial-chain transform.
 *
 * Used by ros2_control_real.xacro (CSP/CST stack). Command layout per joint:
 *   [0]=position, [1]=effort, [2]=mode_of_operation
 *
 * on_init() and read() (motor → joint) are inherited from AnkleLinkageDriver.
 * Only write() (joint → motor) is layout-specific and lives here.
 *
 * For the PVT stack (position, velocity, effort, kp, kd) see HarambePvtDriver.
 */
class HarambeEthercatDriver : public AnkleLinkageDriver
{
public:
  HarambeEthercatDriver() = default;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
};

}  // namespace harambe_ankle_ethercat_driver

#endif  // HARAMBE_ANKLE_ETHERCAT_DRIVER__HARAMBE_ETHERCAT_DRIVER_HPP_
