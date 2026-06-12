#include "harambe_ankle_ethercat_driver_v2/harambe_pvt_driver.hpp"

#include <Eigen/Dense>
#include <cmath>

namespace harambe_ankle_ethercat_driver_v2
{

hardware_interface::return_type HarambePvtDriver::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // PVT command layout: [0]=position, [1]=velocity, [2]=effort, [3]=kp, [4]=kd.
  // position/velocity/effort transformed joint -> motor via the PRIMARY (cubic)
  // IK + Jacobian; kp/kd pass through per-motor. Save/restore the joint-space
  // command interfaces around the bus write (self-feedback guard).
  const bool tmg = timing_enabled_;   // timing flag (avoid clashing with torque vec 'tm')
  const uint64_t t_write_start = tmg ? nowNs() : 0;

  for (auto & lk : ankle_linkages_) {
    auto & ca = hw_joint_commands_[lk.pitch_idx];
    auto & cb = hw_joint_commands_[lk.roll_idx];

    lk.saved_cmd_pitch[0] = ca[0];  lk.saved_cmd_pitch[1] = ca[1];  lk.saved_cmd_pitch[2] = ca[2];
    lk.saved_cmd_roll[0] = cb[0];   lk.saved_cmd_roll[1] = cb[1];   lk.saved_cmd_roll[2] = cb[2];

    const double pitch = ca[0];
    const double roll = cb[0];

    if (!std::isfinite(pitch) || !std::isfinite(roll)) {
      // Uncommanded (no active controller): HOLD the measured motor position with
      // zero velocity/effort. NEVER force motor 0 — on an enabled drive that
      // would slew the ankle to neutral (e.g. on e-stop reset before the hold
      // controller writes). kp/kd [3]/[4] are left as-is (per-motor passthrough).
      ca[0] = lk.meas_thetaA;  ca[1] = 0.0;  ca[2] = 0.0;
      cb[0] = lk.meas_thetaB;  cb[1] = 0.0;  cb[2] = 0.0;
      continue;
    }

    const IkResult ik = primary_->ik(pitch, roll, lk.last_cmd_thetaA, lk.last_cmd_thetaB);
    lk.last_cmd_thetaA = ik.thetaA;
    lk.last_cmd_thetaB = ik.thetaB;
    if (!ik.reachable) {
      static rclcpp::Clock throttle_clock(RCL_STEADY_TIME);
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("HarambePvtDriverV2"),
        throttle_clock, 1000,
        "Ankle pose (pitch=%.3f roll=%.3f) unreachable — motor command clamped", pitch, roll);
    }

    const Eigen::Matrix2d J = primary_->jacobian_at(ik.thetaA, ik.thetaB, pitch, roll);
    const double det = J.determinant();

    Eigen::Vector2d vm, tm;
    if (std::abs(det) > 1e-9) {
      vm = J.fullPivLu().solve(Eigen::Vector2d(ca[1], cb[1]));
      tm = J.transpose() * Eigen::Vector2d(ca[2], cb[2]);
    } else {
      vm = Eigen::Vector2d::Zero();
      tm = J.transpose() * Eigen::Vector2d(ca[2], cb[2]);
    }

    ca[0] = ik.thetaA;  cb[0] = ik.thetaB;
    ca[1] = vm(0);      cb[1] = vm(1);
    ca[2] = tm(0);      cb[2] = tm(1);
    // ca[3]/ca[4] kp/kd: per-motor passthrough
  }
  if (tmg) timing_[PH_WRITE_ANKLE_IK].add((nowNs() - t_write_start) * 1e-3);

  const uint64_t t_bus = tmg ? nowNs() : 0;
  const auto ret = EthercatDriver::write(time, period);
  if (tmg) timing_[PH_WRITE_BUS].add((nowNs() - t_bus) * 1e-3);

  for (auto & lk : ankle_linkages_) {
    auto & ca = hw_joint_commands_[lk.pitch_idx];
    auto & cb = hw_joint_commands_[lk.roll_idx];
    ca[0] = lk.saved_cmd_pitch[0];  ca[1] = lk.saved_cmd_pitch[1];  ca[2] = lk.saved_cmd_pitch[2];
    cb[0] = lk.saved_cmd_roll[0];   cb[1] = lk.saved_cmd_roll[1];   cb[2] = lk.saved_cmd_roll[2];
  }

  if (tmg) timing_[PH_WRITE_TOTAL].add((nowNs() - t_write_start) * 1e-3);
  return ret;
}

}  // namespace harambe_ankle_ethercat_driver_v2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  harambe_ankle_ethercat_driver_v2::HarambePvtDriver,
  hardware_interface::SystemInterface)
