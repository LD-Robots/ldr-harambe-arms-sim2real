#include "harambe_ankle_ethercat_driver/ankle_model.hpp"

#include <algorithm>

namespace harambe_ankle_ethercat_driver
{

// ───────────────────────── parameters ─────────────────────────

void AnkleModel::set_params(const AnkleParams & p)
{
  p_ = p;
  if (p_.lock_heights_to_rods) {
    derive_heights();  // ignore failure here; on_init validates and reports
  }
}

bool AnkleModel::derive_heights()
{
  if (!p_.lock_heights_to_rods) {
    return true;  // caller-supplied H_A/H_B are authoritative
  }
  const double base = (p_.a - p_.Lp) * (p_.a - p_.Lp) +
                      (p_.r - p_.S / 2.0) * (p_.r - p_.S / 2.0);
  const double min_rod2 = std::min(p_.L_A, p_.L_B) * std::min(p_.L_A, p_.L_B);
  if (base >= min_rod2) {
    return false;  // sqrt of negative — parameter error
  }
  p_.H_A = std::sqrt(p_.L_A * p_.L_A - base) - p_.h - p_.delta;
  p_.H_B = std::sqrt(p_.L_B * p_.L_B - base) - p_.h - p_.delta;
  return true;
}

// ───────────────────────── geometry primitives (§5) ─────────────────────────

Eigen::Matrix3d AnkleModel::rot_x(double ang)
{
  const double c = std::cos(ang), s = std::sin(ang);
  Eigen::Matrix3d R;
  R << 1, 0, 0,
       0, c, -s,
       0, s, c;
  return R;
}

Eigen::Matrix3d AnkleModel::rot_y(double ang)
{
  const double c = std::cos(ang), s = std::sin(ang);
  Eigen::Matrix3d R;
  R <<  c, 0, s,
        0, 1, 0,
       -s, 0, c;
  return R;
}

Eigen::Vector3d AnkleModel::pin_a(double thetaA) const
{
  return Eigen::Vector3d(p_.a, p_.r * std::cos(thetaA), p_.H_A + p_.r * std::sin(thetaA));
}

Eigen::Vector3d AnkleModel::pin_b(double thetaB) const
{
  return Eigen::Vector3d(p_.a, -p_.r * std::cos(thetaB), p_.H_B + p_.r * std::sin(thetaB));
}

Eigen::Vector3d AnkleModel::mount(const Eigen::Vector3d & m_foot, double phi_p, double phi_r) const
{
  const Eigen::Vector3d rolled = rot_x(phi_r) * m_foot + Eigen::Vector3d(0.0, 0.0, -p_.h);
  return rot_y(phi_p) * rolled;
}

Eigen::Vector2d AnkleModel::residual(
  double thetaA, double thetaB, double phi_p, double phi_r) const
{
  const Eigen::Vector3d m_A(p_.Lp, p_.S / 2.0, -p_.delta);
  const Eigen::Vector3d m_B(p_.Lp, -p_.S / 2.0, -p_.delta);
  const Eigen::Vector3d MA = mount(m_A, phi_p, phi_r);
  const Eigen::Vector3d MB = mount(m_B, phi_p, phi_r);
  const double f1 = (pin_a(thetaA) - MA).squaredNorm() - p_.L_A * p_.L_A;
  const double f2 = (pin_b(thetaB) - MB).squaredNorm() - p_.L_B * p_.L_B;
  return Eigen::Vector2d(f1, f2);
}

double AnkleModel::wrap_pi(double ang)
{
  while (ang > M_PI) ang -= 2.0 * M_PI;
  while (ang <= -M_PI) ang += 2.0 * M_PI;
  return ang;
}

// ───────────────────────── first-order model (§8) ─────────────────────────

FkResult AnkleModel::first_order_fk(double thetaA, double thetaB) const
{
  const double dA = p_.r * std::sin(thetaA);
  const double dB = p_.r * std::sin(thetaB);
  const double model_pitch = std::asin(clamp_unit((dA + dB) / (2.0 * p_.Lp)));
  const double model_roll  = std::asin(clamp_unit((dA - dB) / p_.S));
  FkResult out;
  out.pitch = p_.pitch_sign * model_pitch;
  out.roll  = p_.roll_sign * model_roll;
  out.converged = true;
  return out;
}

IkResult AnkleModel::first_order_ik(double pitch, double roll) const
{
  const double model_pitch = p_.pitch_sign * pitch;
  const double model_roll  = p_.roll_sign * roll;
  const double dA = p_.Lp * std::sin(model_pitch) + (p_.S / 2.0) * std::sin(model_roll);
  const double dB = p_.Lp * std::sin(model_pitch) - (p_.S / 2.0) * std::sin(model_roll);
  IkResult out;
  out.thetaA = std::asin(clamp_unit(dA / p_.r));
  out.thetaB = std::asin(clamp_unit(dB / p_.r));
  out.reachable = (std::abs(dA / p_.r) <= 1.0) && (std::abs(dB / p_.r) <= 1.0);
  return out;
}

// ───────────────────────── inverse kinematics (§6) ─────────────────────────

namespace
{
// Solve A·cos(theta) + B·sin(theta) = K. Returns both roots in `roots`;
// `reachable` is false when |K/R| > 1. Roots are wrapped to (-pi, pi].
bool solve_rod(double A, double B, double K, double & root0, double & root1)
{
  const double R = std::hypot(A, B);
  if (R < 1e-12) {
    root0 = root1 = 0.0;
    return false;
  }
  const double ratio = K / R;
  if (std::abs(ratio) > 1.0) {
    // Closest reachable: theta at the extremum (cos(theta - psi) = +/-1).
    const double psi = std::atan2(B, A);
    root0 = root1 = (ratio > 0.0) ? psi : (psi + M_PI);
    return false;
  }
  const double psi = std::atan2(B, A);
  const double d = std::acos(ratio);
  root0 = psi + d;
  root1 = psi - d;
  return true;
}
}  // namespace

IkResult AnkleModel::ik(double pitch, double roll, double prev_thetaA, double prev_thetaB) const
{
  const double model_pitch = p_.pitch_sign * pitch;
  const double model_roll  = p_.roll_sign * roll;
  const double phi_p = -model_pitch;   // R_y angle
  const double phi_r = model_roll;     // R_x angle

  const Eigen::Vector3d m_A(p_.Lp, p_.S / 2.0, -p_.delta);
  const Eigen::Vector3d m_B(p_.Lp, -p_.S / 2.0, -p_.delta);
  const Eigen::Vector3d MA = mount(m_A, phi_p, phi_r);
  const Eigen::Vector3d MB = mount(m_B, phi_p, phi_r);

  IkResult out;
  out.reachable = true;

  // Solve one rod and pick the in-limit root nearest the previous command.
  auto resolve = [&](double A, double B, double K, double prev, double & theta) {
    double r0, r1;
    const bool reach = solve_rod(A, B, K, r0, r1);
    r0 = wrap_pi(r0);
    r1 = wrap_pi(r1);
    if (!reach) {
      // Unreachable: clamp the closest-reach angle to the motor limit.
      theta = std::max(-p_.motor_limit, std::min(p_.motor_limit, r0));
      out.reachable = false;
      return;
    }
    const bool in0 = std::abs(r0) <= p_.motor_limit;
    const bool in1 = std::abs(r1) <= p_.motor_limit;
    if (in0 && in1) {
      theta = (std::abs(r0 - prev) <= std::abs(r1 - prev)) ? r0 : r1;
    } else if (in0) {
      theta = r0;
    } else if (in1) {
      theta = r1;
    } else {
      // Both roots out of range — clamp the nearest to the previous command.
      const double cand = (std::abs(r0 - prev) <= std::abs(r1 - prev)) ? r0 : r1;
      theta = std::max(-p_.motor_limit, std::min(p_.motor_limit, cand));
      out.reachable = false;
    }
  };

  // Rod A: A1=-2r·my, B1=2r(H_A-mz), K1 = L_A^2 - r^2 - (a-mx)^2 - my^2 - (H_A-mz)^2.
  {
    const double mx = MA.x(), my = MA.y(), mz = MA.z();
    const double A1 = -2.0 * p_.r * my;
    const double B1 = 2.0 * p_.r * (p_.H_A - mz);
    const double K1 = p_.L_A * p_.L_A - p_.r * p_.r - (p_.a - mx) * (p_.a - mx) -
                      my * my - (p_.H_A - mz) * (p_.H_A - mz);
    resolve(A1, B1, K1, prev_thetaA, out.thetaA);
  }
  // Rod B: A2=+2r·my (sign flip from pin_B's -r·cos), B2=2r(H_B-mz), K2 analogous.
  {
    const double mx = MB.x(), my = MB.y(), mz = MB.z();
    const double A2 = 2.0 * p_.r * my;
    const double B2 = 2.0 * p_.r * (p_.H_B - mz);
    const double K2 = p_.L_B * p_.L_B - p_.r * p_.r - (p_.a - mx) * (p_.a - mx) -
                      my * my - (p_.H_B - mz) * (p_.H_B - mz);
    resolve(A2, B2, K2, prev_thetaB, out.thetaB);
  }

  return out;
}

// ───────────────────────── forward kinematics (§7) ─────────────────────────

void AnkleModel::newton(
  double thetaA, double thetaB, double seed_phi_p, double seed_phi_r,
  double & phi_p, double & phi_r, bool & ok) const
{
  phi_p = seed_phi_p;
  phi_r = seed_phi_r;
  const double hstep = 1e-6;
  ok = false;
  for (int i = 0; i < 20; ++i) {
    const Eigen::Vector2d F = residual(thetaA, thetaB, phi_p, phi_r);
    // Central-difference 2x2 Jacobian dF/d(phi_p, phi_r).
    const Eigen::Vector2d Fpp = residual(thetaA, thetaB, phi_p + hstep, phi_r);
    const Eigen::Vector2d Fpm = residual(thetaA, thetaB, phi_p - hstep, phi_r);
    const Eigen::Vector2d Frp = residual(thetaA, thetaB, phi_p, phi_r + hstep);
    const Eigen::Vector2d Frm = residual(thetaA, thetaB, phi_p, phi_r - hstep);
    Eigen::Matrix2d J;
    J.col(0) = (Fpp - Fpm) / (2.0 * hstep);
    J.col(1) = (Frp - Frm) / (2.0 * hstep);
    const double det = J.determinant();
    if (std::abs(det) < 1e-12) {
      break;  // singular — leave phi at the latest estimate
    }
    const Eigen::Vector2d delta = J.fullPivLu().solve(-F);
    phi_p += delta(0);
    phi_r += delta(1);
    if (delta.cwiseAbs().sum() < 1e-9) {
      ok = true;
      break;
    }
  }
}

FkResult AnkleModel::fk(double thetaA, double thetaB, double seed_pitch, double seed_roll) const
{
  // Seed in internal (model) frame.
  double seed_phi_p, seed_phi_r;
  if (std::isfinite(seed_pitch) && std::isfinite(seed_roll)) {
    const double mp = p_.pitch_sign * seed_pitch;
    const double mr = p_.roll_sign * seed_roll;
    seed_phi_p = -mp;
    seed_phi_r = mr;
  } else {
    // First-order seed (docs §7: phi_p = -asin(...), phi_r = +asin(...)).
    const double dA = p_.r * std::sin(thetaA);
    const double dB = p_.r * std::sin(thetaB);
    seed_phi_p = -std::asin(clamp_unit((dA + dB) / (2.0 * p_.Lp)));
    seed_phi_r = std::asin(clamp_unit((dA - dB) / p_.S));
  }

  double phi_p, phi_r;
  bool ok;
  newton(thetaA, thetaB, seed_phi_p, seed_phi_r, phi_p, phi_r, ok);

  FkResult out;
  // pitch = -phi_p, roll = phi_r (model), then apply URDF sign.
  out.pitch = p_.pitch_sign * (-phi_p);
  out.roll  = p_.roll_sign * phi_r;
  out.converged = ok;
  return out;
}

// ───────────────────────── numerical Jacobian ─────────────────────────

Eigen::Matrix2d AnkleModel::jacobian(
  double thetaA, double thetaB, double seed_pitch, double seed_roll) const
{
  // Converge the pose once, then use the analytic Jacobian at it.
  const FkResult f = fk(thetaA, thetaB, seed_pitch, seed_roll);
  return jacobian_at(thetaA, thetaB, f.pitch, f.roll);
}

Eigen::Matrix2d AnkleModel::jacobian_at(
  double thetaA, double thetaB, double pitch, double roll) const
{
  // URDF pose → internal angles (phi_p = -pitch, phi_r = roll in model space).
  const double phi_p = -(p_.pitch_sign * pitch);
  const double phi_r = p_.roll_sign * roll;

  // dF/dphi (2x2): central difference of the rod residuals (no Newton).
  const double hs = 1e-6;
  const Eigen::Vector2d Fpp = residual(thetaA, thetaB, phi_p + hs, phi_r);
  const Eigen::Vector2d Fpm = residual(thetaA, thetaB, phi_p - hs, phi_r);
  const Eigen::Vector2d Frp = residual(thetaA, thetaB, phi_p, phi_r + hs);
  const Eigen::Vector2d Frm = residual(thetaA, thetaB, phi_p, phi_r - hs);
  Eigen::Matrix2d dFdphi;
  dFdphi.col(0) = (Fpp - Fpm) / (2.0 * hs);
  dFdphi.col(1) = (Frp - Frm) / (2.0 * hs);

  // dF/dtheta (diagonal): F1 depends only on thetaA, F2 only on thetaB.
  //   dFi/dtheta = 2 (pin - mount) · d(pin)/dtheta
  const Eigen::Vector3d m_A(p_.Lp, p_.S / 2.0, -p_.delta);
  const Eigen::Vector3d m_B(p_.Lp, -p_.S / 2.0, -p_.delta);
  const Eigen::Vector3d MA = mount(m_A, phi_p, phi_r);
  const Eigen::Vector3d MB = mount(m_B, phi_p, phi_r);
  const Eigen::Vector3d dpinA(0.0, -p_.r * std::sin(thetaA), p_.r * std::cos(thetaA));
  const Eigen::Vector3d dpinB(0.0, p_.r * std::sin(thetaB), p_.r * std::cos(thetaB));
  const double dF1dA = 2.0 * (pin_a(thetaA) - MA).dot(dpinA);
  const double dF2dB = 2.0 * (pin_b(thetaB) - MB).dot(dpinB);
  Eigen::Matrix2d dFdtheta;
  dFdtheta << dF1dA, 0.0,
              0.0, dF2dB;

  // dphi/dtheta = -(dF/dphi)^{-1} (dF/dtheta).
  Eigen::Matrix2d dphi;
  const double det = dFdphi.determinant();
  if (std::abs(det) < 1e-12) {
    dphi.setZero();  // singular configuration — driver guards on det too
  } else {
    dphi = -dFdphi.inverse() * dFdtheta;
  }

  // Map internal → URDF: pitch = pitch_sign·(-phi_p), roll = roll_sign·phi_r.
  Eigen::Matrix2d J;
  J.row(0) = -p_.pitch_sign * dphi.row(0);   // d pitch / d theta
  J.row(1) = p_.roll_sign * dphi.row(1);     // d roll  / d theta
  return J;
}

}  // namespace harambe_ankle_ethercat_driver
