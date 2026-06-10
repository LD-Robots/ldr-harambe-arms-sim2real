#ifndef HARAMBE_ANKLE_ETHERCAT_DRIVER_V2__HARAMBE_PVT_DRIVER_HPP_
#define HARAMBE_ANKLE_ETHERCAT_DRIVER_V2__HARAMBE_PVT_DRIVER_HPP_

#include "harambe_ankle_ethercat_driver_v2/ankle_linkage_driver.hpp"

namespace harambe_ankle_ethercat_driver_v2
{

/**
 * HarambePvtDriver (v2) — PVT (CiA 402 mode 5) EtherCAT driver with the cubic
 * ankle transform (primary). Command layout per joint:
 *   [0]=position, [1]=velocity, [2]=effort, [3]=kp, [4]=kd
 * kp/kd pass through per-motor (drive-side PD runs in motor space).
 * on_init()/read() inherited from AnkleLinkageDriver; only write() is here.
 */
class HarambePvtDriver : public AnkleLinkageDriver
{
public:
  HarambePvtDriver() = default;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
};

}  // namespace harambe_ankle_ethercat_driver_v2

#endif  // HARAMBE_ANKLE_ETHERCAT_DRIVER_V2__HARAMBE_PVT_DRIVER_HPP_
