#ifndef HARAMBE_ANKLE_ETHERCAT_DRIVER_V2__ANALYTIC_ANKLE_KINEMATICS_HPP_
#define HARAMBE_ANKLE_ETHERCAT_DRIVER_V2__ANALYTIC_ANKLE_KINEMATICS_HPP_

// Serial-chain ("exact" eccentric-pushrod) ankle model — the user's analytic
// method. Ported verbatim from harambe_ankle_ethercat_driver/ankle_model, with
// the physical axis convention applied at the boundary (its native SUM->pitch /
// DIFF->roll is remapped so equal motors -> roll, opposite -> pitch, matching the
// real hardware and the cubic backend).
//
// COMPARISON ONLY. Measured against the real 9x9 grid this model is off by >=15
// deg (structurally inadequate for this mechanism); it MUST NOT drive control.
// It exists so the driver can publish it on a separate topic next to the cubic.

#include "harambe_ankle_ethercat_driver_v2/ankle_kinematics.hpp"

namespace harambe_ankle_ethercat_driver_v2
{

class AnalyticAnkleKinematics : public AnkleKinematics
{
public:
  AnalyticAnkleKinematics() { set_params(AnkleParams{}); }

  void set_params(const AnkleParams & p) override;
  const AnkleParams & params() const override { return p_; }
  const char * name() const override { return "analytic"; }

  FkResult fk(double thetaA, double thetaB, double seed_pitch, double seed_roll) const override;
  IkResult ik(double pitch, double roll, double prev_thetaA, double prev_thetaB) const override;
  Eigen::Matrix2d jacobian_at(
    double thetaA, double thetaB, double pitch, double roll) const override;

  bool derive_heights();   // H_A/H_B from rod lengths; false on parameter error

private:
  AnkleParams p_;

  // --- serial-chain geometry primitives (internal/model frame) ---
  static Eigen::Matrix3d rot_x(double ang);
  static Eigen::Matrix3d rot_y(double ang);
  static Eigen::Matrix3d rot_x_d(double ang);   // d rot_x / d ang
  static Eigen::Matrix3d rot_y_d(double ang);   // d rot_y / d ang
  Eigen::Vector3d pin_a(double thetaA) const;
  Eigen::Vector3d pin_b(double thetaB) const;
  Eigen::Vector3d mount(const Eigen::Vector3d & m_foot, double phi_p, double phi_r) const;
  Eigen::Vector2d residual(double thetaA, double thetaB, double phi_p, double phi_r) const;
  // Fused residual F and its EXACT analytic Jacobian dF/dphi (col0 = d/dphi_p,
  // col1 = d/dphi_r) — same layout as the central-difference block it replaces in
  // newton()/jacobian_at(). Builds the four rotation matrices once.
  void residual_jac(double thetaA, double thetaB, double phi_p, double phi_r,
                    Eigen::Vector2d & F, Eigen::Matrix2d & Jphi) const;
  void newton(double thetaA, double thetaB, double seed_phi_p, double seed_phi_r,
              double & phi_p, double & phi_r, bool & ok) const;

  // Native serial-chain FK/IK (SUM->pitch convention), before the physical swap.
  void fk_native(double thetaA, double thetaB, double seed_pp, double seed_pr,
                 double & native_pitch, double & native_roll, bool & ok) const;

  static double clamp_unit(double x) { return std::max(-1.0, std::min(1.0, x)); }
  static double wrap_pi(double a);
};

}  // namespace harambe_ankle_ethercat_driver_v2

#endif  // HARAMBE_ANKLE_ETHERCAT_DRIVER_V2__ANALYTIC_ANKLE_KINEMATICS_HPP_
