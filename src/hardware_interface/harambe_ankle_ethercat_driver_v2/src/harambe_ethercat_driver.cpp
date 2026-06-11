#include "harambe_ankle_ethercat_driver_v2/harambe_ethercat_driver.hpp"

#include <Eigen/Dense>
#include <cmath>

namespace harambe_ankle_ethercat_driver_v2
{

hardware_interface::return_type HarambeEthercatDriver::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // CSP/CST command layout: [0]=position, [1]=effort, [2]=mode_of_operation.
  // Transform pitch/roll -> motor A/B via the PRIMARY (cubic) IK, then RESTORE
  // the joint-space command interfaces (so an uncommanded cycle does not read
  // back our motor-space output and re-transform it to the limit fixed point).
  for (auto & lk : ankle_linkages_) {
    auto & ca = hw_joint_commands_[lk.pitch_idx];
    auto & cb = hw_joint_commands_[lk.roll_idx];

    lk.saved_cmd_pitch[0] = ca[0];  lk.saved_cmd_pitch[1] = ca[1];
    lk.saved_cmd_roll[0] = cb[0];   lk.saved_cmd_roll[1] = cb[1];

    const double pitch = ca[0];
    const double roll = cb[0];

    if (!std::isfinite(pitch) || !std::isfinite(roll)) {
      // Uncommanded: HOLD the measured motor position (zero effort). NEVER force
      // motor 0 — on an enabled drive that slews the ankle to neutral (e.g. on
      // e-stop reset before the hold controller writes). mode_of_op [2] passes through.
      ca[0] = lk.meas_thetaA;  ca[1] = 0.0;
      cb[0] = lk.meas_thetaB;  cb[1] = 0.0;
      continue;
    }

    const IkResult ik = primary_->ik(pitch, roll, lk.last_cmd_thetaA, lk.last_cmd_thetaB);
    lk.last_cmd_thetaA = ik.thetaA;
    lk.last_cmd_thetaB = ik.thetaB;
    if (!ik.reachable) {
      static rclcpp::Clock throttle_clock(RCL_STEADY_TIME);
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("HarambeEthercatDriverV2"),
        throttle_clock, 1000,
        "Ankle pose (pitch=%.3f roll=%.3f) unreachable — motor command clamped", pitch, roll);
    }

    const Eigen::Matrix2d J = primary_->jacobian_at(ik.thetaA, ik.thetaB, pitch, roll);
    const Eigen::Vector2d tm = J.transpose() * Eigen::Vector2d(ca[1], cb[1]);

    ca[0] = ik.thetaA;  cb[0] = ik.thetaB;
    ca[1] = tm(0);      cb[1] = tm(1);
    // ca[2]/cb[2] mode_of_operation: per-motor passthrough
  }

  const auto ret = EthercatDriver::write(time, period);

  for (auto & lk : ankle_linkages_) {
    auto & ca = hw_joint_commands_[lk.pitch_idx];
    auto & cb = hw_joint_commands_[lk.roll_idx];
    ca[0] = lk.saved_cmd_pitch[0];  ca[1] = lk.saved_cmd_pitch[1];
    cb[0] = lk.saved_cmd_roll[0];   cb[1] = lk.saved_cmd_roll[1];
  }

  return ret;
}

}  // namespace harambe_ankle_ethercat_driver_v2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  harambe_ankle_ethercat_driver_v2::HarambeEthercatDriver,
  hardware_interface::SystemInterface)
