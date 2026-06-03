#ifndef HARAMBE_ETHERCAT_DRIVER__HARAMBE_PVT_DRIVER_HPP_
#define HARAMBE_ETHERCAT_DRIVER__HARAMBE_PVT_DRIVER_HPP_

#include "harambe_ethercat_driver/ankle_linkage_driver.hpp"

namespace harambe_ethercat_driver
{

/**
 * HarambePvtDriver — PVT (CiA 402 mode 5) EtherCAT driver with the ankle
 * linkage transform.
 *
 * Used by ros2_control_real_pvt.xacro (PVT stack). Command layout per joint:
 *   [0]=position, [1]=velocity, [2]=effort, [3]=kp, [4]=kd
 *
 * on_init() and read() (motor → joint) are inherited from AnkleLinkageDriver.
 * write() transforms position/velocity/effort joint → motor for the ankle
 * pairs. kp [3] and kd [4] pass through per-motor: the drive-side PD therefore
 * runs in MOTOR space, so an ankle's commanded kp/kd is a per-motor gain, not a
 * joint-space stiffness. (Each pair's two motors get the pitch joint's and the
 * roll joint's gains respectively.)
 */
class HarambePvtDriver : public AnkleLinkageDriver
{
public:
  HarambePvtDriver() = default;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
};

}  // namespace harambe_ethercat_driver

#endif  // HARAMBE_ETHERCAT_DRIVER__HARAMBE_PVT_DRIVER_HPP_
