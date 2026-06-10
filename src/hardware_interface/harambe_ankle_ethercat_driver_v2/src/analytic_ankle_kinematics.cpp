#include "harambe_ankle_ethercat_driver_v2/analytic_ankle_kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace harambe_ankle_ethercat_driver_v2
{

// Serial-chain ankle, CORRECTED. Motor B's crank is mirror-mounted
// (p_.crank_b_sign = -1): its pin sweeps OPPOSITE in z to motor A for the same
// rotation. So equal motor direction lifts pin A / drops pin B -> differential
// -> ROLL; opposite direction -> common -> PITCH, matching the hardware.
//
// Native internal angles: phi_p = R_y (model pitch), phi_r = R_x (model roll).
// With the mirror crank the native axes already align with the real axes; the
// physical (grid, +pitch = toe up) convention is native negated:
//   phys_pitch = -native_pitch,  phys_roll = -native_roll
// pitch_sign/roll_sign then map physical -> URDF (default +1 = physical/grid).

void AnalyticAnkleKinematics::set_params(const AnkleParams & p)
{
  p_ = p;
  if (p_.lock_heights_to_rods) {
    derive_heights();
  }
}

bool AnalyticAnkleKinematics::derive_heights()
{
  if (!p_.lock_heights_to_rods) return true;
  // Neutral (theta=0): sin(0)=0, so the crank sign does not enter here.
  const double base = (p_.a - p_.Lp) * (p_.a - p_.Lp) +
                      (p_.r - p_.S / 2.0) * (p_.r - p_.S / 2.0);
  const double min_rod2 = std::min(p_.L_A, p_.L_B) * std::min(p_.L_A, p_.L_B);
  if (base >= min_rod2) return false;
  p_.H_A = std::sqrt(p_.L_A * p_.L_A - base) - p_.h - p_.delta;
  p_.H_B = std::sqrt(p_.L_B * p_.L_B - base) - p_.h - p_.delta;
  return true;
}

Eigen::Matrix3d AnalyticAnkleKinematics::rot_x(double a)
{
  const double c = std::cos(a), s = std::sin(a);
  Eigen::Matrix3d R; R << 1, 0, 0, 0, c, -s, 0, s, c; return R;
}
Eigen::Matrix3d AnalyticAnkleKinematics::rot_y(double a)
{
  const double c = std::cos(a), s = std::sin(a);
  Eigen::Matrix3d R; R << c, 0, s, 0, 1, 0, -s, 0, c; return R;
}
Eigen::Vector3d AnalyticAnkleKinematics::pin_a(double tA) const
{
  return Eigen::Vector3d(p_.a, p_.r * std::cos(tA), p_.H_A + p_.r * std::sin(tA));
}
Eigen::Vector3d AnalyticAnkleKinematics::pin_b(double tB) const
{
  // Mirror crank: z term carries crank_b_sign.
  return Eigen::Vector3d(p_.a, -p_.r * std::cos(tB),
                         p_.H_B + p_.crank_b_sign * p_.r * std::sin(tB));
}
Eigen::Vector3d AnalyticAnkleKinematics::mount(
  const Eigen::Vector3d & m_foot, double phi_p, double phi_r) const
{
  const Eigen::Vector3d rolled = rot_x(phi_r) * m_foot + Eigen::Vector3d(0.0, 0.0, -p_.h);
  return rot_y(phi_p) * rolled;
}
Eigen::Vector2d AnalyticAnkleKinematics::residual(
  double tA, double tB, double phi_p, double phi_r) const
{
  const Eigen::Vector3d m_A(p_.Lp, p_.S / 2.0, -p_.delta);
  const Eigen::Vector3d m_B(p_.Lp, -p_.S / 2.0, -p_.delta);
  const Eigen::Vector3d MA = mount(m_A, phi_p, phi_r);
  const Eigen::Vector3d MB = mount(m_B, phi_p, phi_r);
  const double f1 = (pin_a(tA) - MA).squaredNorm() - p_.L_A * p_.L_A;
  const double f2 = (pin_b(tB) - MB).squaredNorm() - p_.L_B * p_.L_B;
  return Eigen::Vector2d(f1, f2);
}
double AnalyticAnkleKinematics::wrap_pi(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

void AnalyticAnkleKinematics::newton(
  double tA, double tB, double seed_phi_p, double seed_phi_r,
  double & phi_p, double & phi_r, bool & ok) const
{
  phi_p = seed_phi_p; phi_r = seed_phi_r;
  const double hs = 1e-6; ok = false;
  for (int i = 0; i < 20; ++i) {
    const Eigen::Vector2d F = residual(tA, tB, phi_p, phi_r);
    const Eigen::Vector2d Fpp = residual(tA, tB, phi_p + hs, phi_r);
    const Eigen::Vector2d Fpm = residual(tA, tB, phi_p - hs, phi_r);
    const Eigen::Vector2d Frp = residual(tA, tB, phi_p, phi_r + hs);
    const Eigen::Vector2d Frm = residual(tA, tB, phi_p, phi_r - hs);
    Eigen::Matrix2d J;
    J.col(0) = (Fpp - Fpm) / (2.0 * hs);
    J.col(1) = (Frp - Frm) / (2.0 * hs);
    if (std::abs(J.determinant()) < 1e-12) break;
    const Eigen::Vector2d d = J.fullPivLu().solve(-F);
    phi_p += d(0); phi_r += d(1);
    if (d.cwiseAbs().sum() < 1e-9) { ok = true; break; }
  }
}

void AnalyticAnkleKinematics::fk_native(
  double tA, double tB, double, double, double & native_pitch, double & native_roll, bool & ok) const
{
  const double dA = p_.r * std::sin(tA);
  const double dB = p_.crank_b_sign * p_.r * std::sin(tB);
  const double seed_phi_p = -std::asin(clamp_unit((dA + dB) / (2.0 * p_.Lp)));
  const double seed_phi_r = std::asin(clamp_unit((dA - dB) / p_.S));
  double phi_p, phi_r;
  newton(tA, tB, seed_phi_p, seed_phi_r, phi_p, phi_r, ok);
  native_pitch = -phi_p;
  native_roll = phi_r;
}

FkResult AnalyticAnkleKinematics::fk(double tA, double tB, double, double) const
{
  double np, nr; bool ok;
  fk_native(tA, tB, 0.0, 0.0, np, nr, ok);
  FkResult out;
  out.pitch = p_.pitch_sign * (-np);    // phys = -native, then URDF sign
  out.roll = p_.roll_sign * (-nr);
  out.converged = ok;
  return out;
}

namespace
{
bool solve_rod(double A, double B, double K, double & r0, double & r1)
{
  const double R = std::hypot(A, B);
  if (R < 1e-12) { r0 = r1 = 0.0; return false; }
  const double ratio = K / R;
  const double psi = std::atan2(B, A);
  if (std::abs(ratio) > 1.0) {
    r0 = r1 = (ratio > 0.0) ? psi : (psi + M_PI);
    return false;
  }
  const double d = std::acos(ratio);
  r0 = psi + d; r1 = psi - d;
  return true;
}
}  // namespace

IkResult AnalyticAnkleKinematics::ik(
  double pitch, double roll, double prev_thetaA, double prev_thetaB) const
{
  // URDF -> physical -> native internal angles.
  const double phys_pitch = p_.pitch_sign * pitch;
  const double phys_roll = p_.roll_sign * roll;
  const double native_pitch = -phys_pitch;
  const double native_roll = -phys_roll;
  const double phi_p = -native_pitch;
  const double phi_r = native_roll;

  const Eigen::Vector3d m_A(p_.Lp, p_.S / 2.0, -p_.delta);
  const Eigen::Vector3d m_B(p_.Lp, -p_.S / 2.0, -p_.delta);
  const Eigen::Vector3d MA = mount(m_A, phi_p, phi_r);
  const Eigen::Vector3d MB = mount(m_B, phi_p, phi_r);

  IkResult out;
  out.reachable = true;

  auto resolve = [&](double A, double B, double K, double prev, double & theta) {
    double r0, r1;
    const bool reach = solve_rod(A, B, K, r0, r1);
    r0 = wrap_pi(r0); r1 = wrap_pi(r1);
    if (!reach) {
      theta = std::max(-p_.motor_limit, std::min(p_.motor_limit, r0));
      out.reachable = false; return;
    }
    const bool in0 = std::abs(r0) <= p_.motor_limit;
    const bool in1 = std::abs(r1) <= p_.motor_limit;
    if (in0 && in1) {
      theta = (std::abs(r0 - prev) <= std::abs(r1 - prev)) ? r0 : r1;
    } else if (in0) { theta = r0; }
    else if (in1) { theta = r1; }
    else {
      const double cand = (std::abs(r0 - prev) <= std::abs(r1 - prev)) ? r0 : r1;
      theta = std::max(-p_.motor_limit, std::min(p_.motor_limit, cand));
      out.reachable = false;
    }
  };

  {
    const double mx = MA.x(), my = MA.y(), mz = MA.z();
    const double A1 = -2.0 * p_.r * my;
    const double B1 = 2.0 * p_.r * (p_.H_A - mz);
    const double K1 = p_.L_A * p_.L_A - p_.r * p_.r - (p_.a - mx) * (p_.a - mx) -
                      my * my - (p_.H_A - mz) * (p_.H_A - mz);
    resolve(A1, B1, K1, prev_thetaA, out.thetaA);
  }
  {
    const double mx = MB.x(), my = MB.y(), mz = MB.z();
    const double A2 = 2.0 * p_.r * my;
    const double B2 = 2.0 * p_.crank_b_sign * p_.r * (p_.H_B - mz);   // mirror crank
    const double K2 = p_.L_B * p_.L_B - p_.r * p_.r - (p_.a - mx) * (p_.a - mx) -
                      my * my - (p_.H_B - mz) * (p_.H_B - mz);
    resolve(A2, B2, K2, prev_thetaB, out.thetaB);
  }
  return out;
}

Eigen::Matrix2d AnalyticAnkleKinematics::jacobian_at(
  double tA, double tB, double pitch, double roll) const
{
  const double phys_pitch = p_.pitch_sign * pitch;
  const double phys_roll = p_.roll_sign * roll;
  const double native_pitch = -phys_pitch;
  const double native_roll = -phys_roll;
  const double phi_p = -native_pitch;
  const double phi_r = native_roll;

  const double hs = 1e-6;
  const Eigen::Vector2d Fpp = residual(tA, tB, phi_p + hs, phi_r);
  const Eigen::Vector2d Fpm = residual(tA, tB, phi_p - hs, phi_r);
  const Eigen::Vector2d Frp = residual(tA, tB, phi_p, phi_r + hs);
  const Eigen::Vector2d Frm = residual(tA, tB, phi_p, phi_r - hs);
  Eigen::Matrix2d dFdphi;
  dFdphi.col(0) = (Fpp - Fpm) / (2.0 * hs);
  dFdphi.col(1) = (Frp - Frm) / (2.0 * hs);

  const Eigen::Vector3d m_A(p_.Lp, p_.S / 2.0, -p_.delta);
  const Eigen::Vector3d m_B(p_.Lp, -p_.S / 2.0, -p_.delta);
  const Eigen::Vector3d MA = mount(m_A, phi_p, phi_r);
  const Eigen::Vector3d MB = mount(m_B, phi_p, phi_r);
  const Eigen::Vector3d dpinA(0.0, -p_.r * std::sin(tA), p_.r * std::cos(tA));
  const Eigen::Vector3d dpinB(0.0, p_.r * std::sin(tB), p_.crank_b_sign * p_.r * std::cos(tB));
  const double dF1dA = 2.0 * (pin_a(tA) - MA).dot(dpinA);
  const double dF2dB = 2.0 * (pin_b(tB) - MB).dot(dpinB);
  Eigen::Matrix2d dFdtheta;
  dFdtheta << dF1dA, 0.0, 0.0, dF2dB;

  Eigen::Matrix2d dphi;   // d(phi_p, phi_r)/d(thetaA, thetaB)
  if (std::abs(dFdphi.determinant()) < 1e-12) {
    dphi.setZero();
  } else {
    dphi = -dFdphi.inverse() * dFdtheta;
  }

  // native_pitch = -phi_p, native_roll = phi_r; phys = -native; then URDF sign.
  // => d phys_pitch/dth =  d phi_p/dth ; d phys_roll/dth = -d phi_r/dth
  Eigen::Matrix2d J;
  J.row(0) = p_.pitch_sign * dphi.row(0);
  J.row(1) = -p_.roll_sign * dphi.row(1);
  return J;
}

}  // namespace harambe_ankle_ethercat_driver_v2
