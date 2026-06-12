#include "harambe_ankle_ethercat_driver_v2/cubic_ankle_kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace harambe_ankle_ethercat_driver_v2
{

namespace
{
constexpr double kRad2Deg = 180.0 / M_PI;
constexpr double kDeg2Rad = M_PI / 180.0;
constexpr int N = 9;

// Lever sweep axis (deg): X = inside (motor A), Y = outside (motor B).
const double kLever[N] = {-80, -60, -40, -20, 0, 20, 40, 60, 80};

// Measured foot angle (deg), index [inside_A][outside_B]. SOURCE OF TRUTH —
// FINAL calibration data (frozen 2026-06-12). Both kinematics backends and the
// grid tests derive from this; re-measuring means re-embedding here + re-fitting
// the analytic geometry. Do not edit casually.
const double kPitch[N][N] = {
  {  -3.67,  -2.01,   0.85,   4.51,   8.50,  12.36,  15.60,  17.78,  18.62},
  {  -4.23,  -2.58,   0.22,   3.82,   7.74,  11.53,  14.70,  16.84,  17.64},
  {  -5.62,  -4.01,  -1.30,   2.15,   5.93,   9.56,  12.59,  14.62,  15.37},
  {  -7.81,  -6.22,  -3.63,  -0.34,   3.24,   6.67,   9.52,  11.41,  12.11},
  { -10.57,  -9.01,  -6.52,  -3.39,  0.0001,   3.22,   5.87,   7.63,   8.27},
  { -13.55, -12.01,  -9.60,  -6.61,  -3.39,  -0.36,   2.10,   3.72,   4.31},
  { -16.35, -14.82, -12.48,  -9.59,  -6.52,  -3.65,  -1.34,   0.15,   0.70},
  { -18.60, -17.08, -14.78, -11.96,  -8.98,  -6.22,  -4.03,  -2.65,  -2.13},
  { -20.02, -18.50, -16.21, -13.41, -10.47,  -7.75,  -5.60,  -4.26,  -3.76},
};
const double kRoll[N][N] = {
  {  54.08,  48.98,  41.06,  32.00,  22.93,  14.53,   7.46,   2.55,   0.63},
  {  50.43,  45.66,  38.18,  29.46,  20.58,  12.26,   5.21,   0.30,  -1.61},
  {  43.41,  39.01,  32.10,  23.86,  15.28,   7.11,   0.11,  -4.80,  -6.70},
  {  34.91,  30.76,  24.27,  16.42,   8.09,   0.02,  -6.98, -11.92, -13.83},
  {  26.13,  22.08,  15.83,   8.21, 0.0006,  -8.07, -15.18, -20.26, -22.24},
  {  17.76,  13.72,   7.56,   0.01,  -8.20, -16.40, -23.76, -29.15, -31.31},
  {  10.32,   6.24,   0.08,  -7.47, -15.79, -24.24, -32.01, -37.90, -40.38},
  {   4.37,   0.25,  -5.92, -13.52, -21.96, -30.68, -38.92, -45.42, -48.34},
  {   0.57,  -3.56,  -9.72, -17.32, -25.81, -34.68, -43.22, -50.16, -53.43},
};
}  // namespace

// Monomial order: [1, X, Y, X^2, XY, Y^2, X^3, X^2 Y, X Y^2, Y^3].
Eigen::Matrix<double, 10, 1> CubicAnkleKinematics::monos(double X, double Y)
{
  Eigen::Matrix<double, 10, 1> m;
  m << 1.0, X, Y, X * X, X * Y, Y * Y,
       X * X * X, X * X * Y, X * Y * Y, Y * Y * Y;
  return m;
}

void CubicAnkleKinematics::dmonos(
  double X, double Y, Eigen::Matrix<double, 10, 1> & dX, Eigen::Matrix<double, 10, 1> & dY)
{
  // d/dX of [1, X, Y, X^2, XY, Y^2, X^3, X^2 Y, X Y^2, Y^3]
  dX << 0.0, 1.0, 0.0, 2.0 * X, Y, 0.0, 3.0 * X * X, 2.0 * X * Y, Y * Y, 0.0;
  // d/dY
  dY << 0.0, 0.0, 1.0, 0.0, X, 2.0 * Y, 0.0, X * X, 2.0 * X * Y, 3.0 * Y * Y;
}

void CubicAnkleKinematics::set_params(const AnkleParams & p)
{
  p_ = p;
  // Least-squares fit the two bicubics to the measured grid, CONSTRAINED through
  // the origin (no constant term) so fk(0,0) = (0,0) exactly — flat-at-neutral is
  // a build guarantee (rods sized in-situ with the ankle blocking jig). The
  // dropped constant is ~0.04 deg, well inside sensor noise. Fit the 9 non-
  // constant monomials (81 x 9 system).
  Eigen::Matrix<double, N * N, 9> A;
  Eigen::Matrix<double, N * N, 1> bp, br;
  int k = 0;
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < N; ++j) {
      A.row(k) = monos(kLever[i], kLever[j]).tail<9>().transpose();
      bp(k) = kPitch[i][j];
      br(k) = kRoll[i][j];
      ++k;
    }
  }
  const auto qr = A.colPivHouseholderQr();
  cp_.setZero();
  cr_.setZero();
  cp_.tail<9>() = qr.solve(bp);
  cr_.tail<9>() = qr.solve(br);
}

void CubicAnkleKinematics::eval_deg(double X, double Y, double & pitch_deg, double & roll_deg) const
{
  const Eigen::Matrix<double, 10, 1> m = monos(X, Y);
  pitch_deg = cp_.dot(m);
  roll_deg = cr_.dot(m);
}

FkResult CubicAnkleKinematics::fk(double thetaA, double thetaB, double, double) const
{
  const double X = thetaA * kRad2Deg;   // inside lever (deg)
  const double Y = thetaB * kRad2Deg;   // outside lever (deg)
  double pd, rd;
  eval_deg(X, Y, pd, rd);
  FkResult out;
  out.pitch = p_.pitch_sign * pd * kDeg2Rad;
  out.roll = p_.roll_sign * rd * kDeg2Rad;
  out.converged = true;
  return out;
}

IkResult CubicAnkleKinematics::ik(
  double pitch, double roll, double /*prev_thetaA*/, double /*prev_thetaB*/) const
{
  // Target in the cubic's physical/degree convention (undo URDF sign).
  const double tp = p_.pitch_sign * pitch * kRad2Deg;   // target pitch (deg)
  const double tr = p_.roll_sign * roll * kRad2Deg;     // target roll (deg)

  // Linear seed: solve [cp1 cp2; cr1 cr2] [X;Y] = [tp-cp0; tr-cr0].
  Eigen::Matrix2d L;
  L << cp_(1), cp_(2), cr_(1), cr_(2);
  Eigen::Vector2d rhs(tp - cp_(0), tr - cr_(0));
  Eigen::Vector2d xy = Eigen::Vector2d::Zero();
  if (std::abs(L.determinant()) > 1e-12) {
    xy = L.fullPivLu().solve(rhs);
  }

  // 2x2 Newton on f(X,Y) = (pitch_deg - tp, roll_deg - tr).
  for (int it = 0; it < 30; ++it) {
    double pd, rd;
    eval_deg(xy(0), xy(1), pd, rd);
    const Eigen::Vector2d F(pd - tp, rd - tr);
    Eigen::Matrix<double, 10, 1> dX, dY;
    dmonos(xy(0), xy(1), dX, dY);
    Eigen::Matrix2d J;
    J << cp_.dot(dX), cp_.dot(dY), cr_.dot(dX), cr_.dot(dY);
    if (std::abs(J.determinant()) < 1e-12) break;
    const Eigen::Vector2d step = J.fullPivLu().solve(-F);
    xy += step;
    if (step.cwiseAbs().maxCoeff() < 1e-9) break;
  }

  const double lim_deg = p_.motor_limit * kRad2Deg;
  IkResult out;
  out.reachable = true;
  for (int c = 0; c < 2; ++c) {
    if (!std::isfinite(xy(c)) || std::abs(xy(c)) > lim_deg + 1e-6) {
      xy(c) = std::max(-lim_deg, std::min(lim_deg, std::isfinite(xy(c)) ? xy(c) : 0.0));
      out.reachable = false;
    }
  }
  out.thetaA = xy(0) * kDeg2Rad;
  out.thetaB = xy(1) * kDeg2Rad;
  return out;
}

Eigen::Matrix2d CubicAnkleKinematics::jacobian_at(
  double thetaA, double thetaB, double /*pitch*/, double /*roll*/) const
{
  // d(pitch,roll)/d(thetaA,thetaB). The deg<->rad factors cancel (pitch is
  // deg*kDeg2Rad, X is theta*kRad2Deg), leaving the dimensionless deg/deg slopes
  // times the sign mapping.
  const double X = thetaA * kRad2Deg;
  const double Y = thetaB * kRad2Deg;
  Eigen::Matrix<double, 10, 1> dX, dY;
  dmonos(X, Y, dX, dY);
  Eigen::Matrix2d J;
  J(0, 0) = p_.pitch_sign * cp_.dot(dX);   // d pitch / d thetaA
  J(0, 1) = p_.pitch_sign * cp_.dot(dY);   // d pitch / d thetaB
  J(1, 0) = p_.roll_sign * cr_.dot(dX);    // d roll  / d thetaA
  J(1, 1) = p_.roll_sign * cr_.dot(dY);    // d roll  / d thetaB
  return J;
}

}  // namespace harambe_ankle_ethercat_driver_v2
