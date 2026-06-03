#include "harambe_ethercat_driver/ankle_linkage.hpp"

#include <cmath>

#include "rclcpp/rclcpp.hpp"

namespace harambe_ethercat_driver
{

double safe_asin(double x, const char * context)
{
  if (x > 1.0) {
    RCLCPP_WARN(rclcpp::get_logger("harambe_ankle_linkage"),
      "arcsin clamp: %s = %.6f > 1.0", context, x);
    return M_PI / 2.0;
  }
  if (x < -1.0) {
    RCLCPP_WARN(rclcpp::get_logger("harambe_ankle_linkage"),
      "arcsin clamp: %s = %.6f < -1.0", context, x);
    return -M_PI / 2.0;
  }
  return std::asin(x);
}

JointPair motor_to_joint(const MotorPair & m, const AnkleGeometry & g)
{
  JointPair out;

  // ── Position: linkage kinematics ──
  const double theta_a = m.pos_a;
  const double theta_b = m.pos_b;
  const double sin_a = std::sin(theta_a);
  const double sin_b = std::sin(theta_b);

  const double pitch_arg = g.r_exc * (sin_a + sin_b) / (2.0 * g.d_pitch);
  const double roll_arg = g.r_exc * (sin_a - sin_b) / (2.0 * g.d_roll);

  const double pitch_lk = safe_asin(pitch_arg, "read pitch");
  const double roll_lk = safe_asin(roll_arg, "read roll");

  // ── Velocity: Jacobian ──
  const double omega_a = m.vel_a;
  const double omega_b = m.vel_b;
  const double cos_a = std::cos(theta_a);
  const double cos_b = std::cos(theta_b);
  const double cos_pitch_lk = std::cos(pitch_lk);
  const double cos_roll_lk = std::cos(roll_lk);

  double pitch_vel_lk = 0.0, roll_vel_lk = 0.0;
  if (std::abs(cos_pitch_lk) > 1e-6) {
    pitch_vel_lk = g.r_exc * (cos_a * omega_a + cos_b * omega_b) / (2.0 * g.d_pitch * cos_pitch_lk);
  }
  if (std::abs(cos_roll_lk) > 1e-6) {
    roll_vel_lk = g.r_exc * (cos_a * omega_a - cos_b * omega_b) / (2.0 * g.d_roll * cos_roll_lk);
  }

  // ── Effort: (J^T)^{-1} maps motor torques → joint torques ──
  const double tau_a = m.eff_a;
  const double tau_b = m.eff_b;
  double pitch_eff_lk, roll_eff_lk;
  if (std::abs(cos_a) > 1e-6 && std::abs(cos_b) > 1e-6) {
    pitch_eff_lk = g.d_pitch * cos_pitch_lk / g.r_exc * (tau_a / cos_a + tau_b / cos_b);
    roll_eff_lk = g.d_roll * cos_roll_lk / g.r_exc * (tau_a / cos_a - tau_b / cos_b);
  } else {
    pitch_eff_lk = (tau_a + tau_b) / 2.0;
    roll_eff_lk = (tau_a - tau_b) / 2.0;
  }

  // ── Apply sign correction (linkage space → URDF convention) ──
  out.pos_pitch = g.pitch_sign * pitch_lk;
  out.pos_roll = g.roll_sign * roll_lk;
  out.vel_pitch = g.pitch_sign * pitch_vel_lk;
  out.vel_roll = g.roll_sign * roll_vel_lk;
  out.eff_pitch = g.pitch_sign * pitch_eff_lk;
  out.eff_roll = g.roll_sign * roll_eff_lk;
  return out;
}

MotorPair joint_to_motor(const JointPair & j, const AnkleGeometry & g)
{
  MotorPair out;

  // ── Convert URDF convention → linkage space ──
  const double pitch_lk = g.pitch_sign * j.pos_pitch;
  const double roll_lk = g.roll_sign * j.pos_roll;

  // ── Position: forward kinematics ──
  const double d_A = g.d_pitch * std::sin(pitch_lk) + g.d_roll * std::sin(roll_lk);
  const double d_B = g.d_pitch * std::sin(pitch_lk) - g.d_roll * std::sin(roll_lk);

  const double theta_a = safe_asin(d_A / g.r_exc, "write motor_a");
  const double theta_b = safe_asin(d_B / g.r_exc, "write motor_b");

  const double cos_a = std::cos(theta_a);
  const double cos_b = std::cos(theta_b);
  const double cos_pitch_lk = std::cos(pitch_lk);
  const double cos_roll_lk = std::cos(roll_lk);

  // ── Effort: J^T maps joint torques → motor torques ──
  const double tau_pitch_lk = g.pitch_sign * j.eff_pitch;
  const double tau_roll_lk = g.roll_sign * j.eff_roll;
  double tau_a, tau_b;
  if (std::abs(cos_pitch_lk) > 1e-6 && std::abs(cos_roll_lk) > 1e-6) {
    const double jt_a_pitch = g.r_exc * cos_a / (2.0 * g.d_pitch * cos_pitch_lk);
    const double jt_a_roll = g.r_exc * cos_a / (2.0 * g.d_roll * cos_roll_lk);
    const double jt_b_pitch = g.r_exc * cos_b / (2.0 * g.d_pitch * cos_pitch_lk);
    const double jt_b_roll = -g.r_exc * cos_b / (2.0 * g.d_roll * cos_roll_lk);

    tau_a = jt_a_pitch * tau_pitch_lk + jt_a_roll * tau_roll_lk;
    tau_b = jt_b_pitch * tau_pitch_lk + jt_b_roll * tau_roll_lk;
  } else {
    tau_a = (tau_pitch_lk + tau_roll_lk) / 2.0;
    tau_b = (tau_pitch_lk - tau_roll_lk) / 2.0;
  }

  // ── Velocity: inverse of the read-path Jacobian ──
  // Read maps motor→joint; here we invert: omega_a/b from pitch/roll vel.
  const double vel_pitch_lk = g.pitch_sign * j.vel_pitch;
  const double vel_roll_lk = g.roll_sign * j.vel_roll;
  double omega_a = 0.0, omega_b = 0.0;
  if (std::abs(cos_a) > 1e-6 && std::abs(cos_b) > 1e-6) {
    const double P = vel_pitch_lk * 2.0 * g.d_pitch * cos_pitch_lk / g.r_exc;
    const double R = vel_roll_lk * 2.0 * g.d_roll * cos_roll_lk / g.r_exc;
    omega_a = ((P + R) / 2.0) / cos_a;
    omega_b = ((P - R) / 2.0) / cos_b;
  }

  out.pos_a = theta_a;  out.pos_b = theta_b;
  out.vel_a = omega_a;  out.vel_b = omega_b;
  out.eff_a = tau_a;    out.eff_b = tau_b;
  return out;
}

}  // namespace harambe_ethercat_driver
