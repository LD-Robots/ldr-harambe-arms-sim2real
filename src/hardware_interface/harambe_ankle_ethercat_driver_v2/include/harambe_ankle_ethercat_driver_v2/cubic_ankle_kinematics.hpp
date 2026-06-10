#ifndef HARAMBE_ANKLE_ETHERCAT_DRIVER_V2__CUBIC_ANKLE_KINEMATICS_HPP_
#define HARAMBE_ANKLE_ETHERCAT_DRIVER_V2__CUBIC_ANKLE_KINEMATICS_HPP_

// PRIMARY ankle kinematics: a bicubic fit of the real measured 9x9 lever grid.
//
// pitch_deg = c_p . m(X,Y),  roll_deg = c_r . m(X,Y)
//   X = inside-lever angle (deg) = thetaA in deg   (1:1, zero at flat)
//   Y = outside-lever angle (deg) = thetaB in deg
//   m = [1, X, Y, X^2, XY, Y^2, X^3, X^2 Y, X Y^2, Y^3]   (10 terms)
// Coefficients are solved by least squares from the embedded grid table at
// set_params() — the grid is the single source of truth, no transcribed numbers.
// Reproduces the grid to ~0.15 deg (pitch) / ~0.24 deg (roll).
//
// IK inverts the two cubics with a 2x2 Newton (analytic Jacobian), seeded by the
// linear part; outside the +/-80 deg measured box the pose is clamped and flagged
// unreachable (the fit must not be trusted beyond its data).

#include "harambe_ankle_ethercat_driver_v2/ankle_kinematics.hpp"

namespace harambe_ankle_ethercat_driver_v2
{

class CubicAnkleKinematics : public AnkleKinematics
{
public:
  CubicAnkleKinematics() { set_params(AnkleParams{}); }

  void set_params(const AnkleParams & p) override;
  const AnkleParams & params() const override { return p_; }
  const char * name() const override { return "cubic"; }

  FkResult fk(double thetaA, double thetaB, double seed_pitch, double seed_roll) const override;
  IkResult ik(double pitch, double roll, double prev_thetaA, double prev_thetaB) const override;
  Eigen::Matrix2d jacobian_at(
    double thetaA, double thetaB, double pitch, double roll) const override;

private:
  AnkleParams p_;
  // Cubic coefficients (degrees in / degrees out), monomial order in monos().
  Eigen::Matrix<double, 10, 1> cp_;   // pitch (physical convention, toe-up +)
  Eigen::Matrix<double, 10, 1> cr_;   // roll  (physical convention, right-down +)

  static Eigen::Matrix<double, 10, 1> monos(double X, double Y);
  // d(monomials)/dX and d/dY for the analytic Jacobian.
  static void dmonos(double X, double Y, Eigen::Matrix<double, 10, 1> & dX,
                     Eigen::Matrix<double, 10, 1> & dY);
  // Raw cubic evaluation in DEGREES (physical convention, before sign mapping).
  void eval_deg(double X, double Y, double & pitch_deg, double & roll_deg) const;
};

}  // namespace harambe_ankle_ethercat_driver_v2

#endif  // HARAMBE_ANKLE_ETHERCAT_DRIVER_V2__CUBIC_ANKLE_KINEMATICS_HPP_
