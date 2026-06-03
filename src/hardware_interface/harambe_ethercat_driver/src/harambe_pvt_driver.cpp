#include "harambe_ethercat_driver/harambe_pvt_driver.hpp"
#include "harambe_ethercat_driver/ankle_linkage.hpp"

namespace harambe_ethercat_driver
{

hardware_interface::return_type HarambePvtDriver::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Transform in-place: pitch/roll commands → motor A/B.
  // PVT command layout: [0]=position, [1]=velocity, [2]=effort, [3]=kp, [4]=kd.
  // position/velocity/effort are transformed joint → motor.
  // kp [3] / kd [4] pass through per-motor (drive-side PD runs in motor space).
  const AnkleGeometry g = geometry();
  for (const auto & lk : ankle_linkages_) {
    auto & ca = hw_joint_commands_[lk.pitch_idx];  // pitch commands (from controller)
    auto & cb = hw_joint_commands_[lk.roll_idx];  // roll commands (from controller)

    JointPair jp;
    jp.pos_pitch = ca[0];  jp.vel_pitch = ca[1];  jp.eff_pitch = ca[2];
    jp.pos_roll = cb[0];   jp.vel_roll = cb[1];   jp.eff_roll = cb[2];
    const MotorPair m = joint_to_motor(jp, g);

    ca[0] = m.pos_a;  cb[0] = m.pos_b;
    ca[1] = m.vel_a;  cb[1] = m.vel_b;
    ca[2] = m.eff_a;  cb[2] = m.eff_b;
    // kp [3] / kd [4]: per-motor passthrough (no mixing)
  }

  return EthercatDriver::write(time, period);
}

}  // namespace harambe_ethercat_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  harambe_ethercat_driver::HarambePvtDriver,
  hardware_interface::SystemInterface)
