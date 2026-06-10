#ifndef HARAMBE_ANKLE_ETHERCAT_DRIVER_V2__HARAMBE_ETHERCAT_DRIVER_HPP_
#define HARAMBE_ANKLE_ETHERCAT_DRIVER_V2__HARAMBE_ETHERCAT_DRIVER_HPP_

#include "harambe_ankle_ethercat_driver_v2/ankle_linkage_driver.hpp"

namespace harambe_ankle_ethercat_driver_v2
{

/**
 * HarambeEthercatDriver (v2) — CSP/CST EtherCAT driver with the cubic ankle
 * transform (primary). Command layout per joint:
 *   [0]=position, [1]=effort, [2]=mode_of_operation
 * on_init()/read() inherited from AnkleLinkageDriver; only write() is here.
 */
class HarambeEthercatDriver : public AnkleLinkageDriver
{
public:
  HarambeEthercatDriver() = default;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
};

}  // namespace harambe_ankle_ethercat_driver_v2

#endif  // HARAMBE_ANKLE_ETHERCAT_DRIVER_V2__HARAMBE_ETHERCAT_DRIVER_HPP_
