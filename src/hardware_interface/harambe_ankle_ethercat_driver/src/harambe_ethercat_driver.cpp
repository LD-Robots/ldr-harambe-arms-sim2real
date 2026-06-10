#include "harambe_ankle_ethercat_driver/harambe_ethercat_driver.hpp"

#include <Eigen/Dense>
#include <cmath>

namespace harambe_ankle_ethercat_driver
{

hardware_interface::return_type HarambeEthercatDriver::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Transform pitch/roll commands → motor A/B for the bus, then RESTORE the
  // joint-space command interfaces afterwards (see HarambePvtDriver::write for
  // the full rationale — the command interface is an INPUT and must not retain
  // our motor-space OUTPUT, or an uncommanded cycle reads it back and
  // re-transforms it to the ±motor_limit fixed point).
  //
  // CSP/CST command layout: [0]=position, [1]=effort, [2]=mode_of_operation.
  // position via exact IK; effort via the Jacobian transpose; mode_of_operation
  // [2] is per-motor passthrough. (No velocity command in this stack.)
  for (auto & lk : ankle_linkages_) {
    auto & ca = hw_joint_commands_[lk.pitch_idx];  // pitch commands (from controller)
    auto & cb = hw_joint_commands_[lk.roll_idx];   // roll commands (from controller)

    lk.saved_cmd_pitch[0] = ca[0];  lk.saved_cmd_pitch[1] = ca[1];
    lk.saved_cmd_roll[0] = cb[0];   lk.saved_cmd_roll[1] = cb[1];

    const double pitch = ca[0];
    const double roll = cb[0];

    if (!std::isfinite(pitch) || !std::isfinite(roll)) {
      // Uncommanded interface: leave untransformed/un-warned, push a safe zero.
      ca[0] = 0.0;  ca[1] = 0.0;
      cb[0] = 0.0;  cb[1] = 0.0;
      continue;
    }

    const IkResult ik = model_.ik(pitch, roll, lk.last_cmd_thetaA, lk.last_cmd_thetaB);
    lk.last_cmd_thetaA = ik.thetaA;
    lk.last_cmd_thetaB = ik.thetaB;
    if (!ik.reachable) {
      static rclcpp::Clock throttle_clock(RCL_STEADY_TIME);
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("HarambeEthercatDriver"),
        throttle_clock, 1000,
        "Ankle pose (pitch=%.3f roll=%.3f) unreachable — motor command clamped", pitch, roll);
    }

    // Effort: tau_motor = J^T · tau_joint (analytic J at the commanded pose).
    const Eigen::Matrix2d J = model_.jacobian_at(ik.thetaA, ik.thetaB, pitch, roll);
    const Eigen::Vector2d tm = J.transpose() * Eigen::Vector2d(ca[1], cb[1]);

    ca[0] = ik.thetaA;  cb[0] = ik.thetaB;
    ca[1] = tm(0);      cb[1] = tm(1);
    // ca[2]/cb[2] mode_of_operation: per-motor passthrough
  }

  const auto ret = EthercatDriver::write(time, period);

  // Restore joint-space command interfaces (breaks the self-feedback loop).
  for (auto & lk : ankle_linkages_) {
    auto & ca = hw_joint_commands_[lk.pitch_idx];
    auto & cb = hw_joint_commands_[lk.roll_idx];
    ca[0] = lk.saved_cmd_pitch[0];  ca[1] = lk.saved_cmd_pitch[1];
    cb[0] = lk.saved_cmd_roll[0];   cb[1] = lk.saved_cmd_roll[1];
  }

  return ret;
}

}  // namespace harambe_ankle_ethercat_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  harambe_ankle_ethercat_driver::HarambeEthercatDriver,
  hardware_interface::SystemInterface)
