#include "harambe_ankle_ethercat_driver/harambe_pvt_driver.hpp"

#include <Eigen/Dense>
#include <cmath>

namespace harambe_ankle_ethercat_driver
{

hardware_interface::return_type HarambePvtDriver::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Transform in-place: pitch/roll commands → motor A/B.
  // PVT command layout: [0]=position, [1]=velocity, [2]=effort, [3]=kp, [4]=kd.
  // position/velocity/effort are transformed joint → motor (exact IK + Jacobian).
  // kp [3] / kd [4] pass through per-motor (drive-side PD runs in motor space).
  for (auto & lk : ankle_linkages_) {
    auto & ca = hw_joint_commands_[lk.pitch_idx];  // pitch commands (from controller)
    auto & cb = hw_joint_commands_[lk.roll_idx];   // roll commands (from controller)

    const double pitch = ca[0];
    const double roll = cb[0];

    // Position: closed-form IK with branch continuity.
    const IkResult ik = model_.ik(pitch, roll, lk.last_cmd_thetaA, lk.last_cmd_thetaB);
    lk.last_cmd_thetaA = ik.thetaA;
    lk.last_cmd_thetaB = ik.thetaB;
    if (!ik.reachable) {
      static rclcpp::Clock throttle_clock(RCL_STEADY_TIME);
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("HarambePvtDriver"),
        throttle_clock, 1000,
        "Ankle pose (pitch=%.3f roll=%.3f) unreachable — motor command clamped", pitch, roll);
    }

    // J = d(pitch,roll)/d(thetaA,thetaB), analytic at the commanded pose
    // (internal angles are known from the command — no FK solve here).
    const Eigen::Matrix2d J = model_.jacobian_at(ik.thetaA, ik.thetaB, pitch, roll);
    const double det = J.determinant();

    // Velocity: solve J · [vA;vB] = [vp;vr] → [vA;vB] = J^{-1} [vp;vr].
    // Effort: tau_motor = J^T · tau_joint.
    Eigen::Vector2d vm, tm;
    if (std::abs(det) > 1e-9) {
      vm = J.fullPivLu().solve(Eigen::Vector2d(ca[1], cb[1]));
      tm = J.transpose() * Eigen::Vector2d(ca[2], cb[2]);
    } else {
      vm = Eigen::Vector2d::Zero();                    // near singularity: no velocity FF
      tm = J.transpose() * Eigen::Vector2d(ca[2], cb[2]);
    }

    ca[0] = ik.thetaA;  cb[0] = ik.thetaB;
    ca[1] = vm(0);      cb[1] = vm(1);
    ca[2] = tm(0);      cb[2] = tm(1);
    // ca[3]/ca[4] kp/kd: per-motor passthrough (no mixing)
  }

  return EthercatDriver::write(time, period);
}

}  // namespace harambe_ankle_ethercat_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  harambe_ankle_ethercat_driver::HarambePvtDriver,
  hardware_interface::SystemInterface)
