// Unit tests for the v2 ankle kinematics (primary = cubic). Pure math.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "harambe_ankle_ethercat_driver_v2/ankle_kinematics.hpp"

using harambe_ankle_ethercat_driver_v2::AnkleKinematics;
using harambe_ankle_ethercat_driver_v2::AnkleParams;
using harambe_ankle_ethercat_driver_v2::FkResult;
using harambe_ankle_ethercat_driver_v2::IkResult;

namespace
{
constexpr double kDeg = M_PI / 180.0;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

std::unique_ptr<AnkleKinematics> cubic()
{
  auto k = AnkleKinematics::create("cubic");
  k->set_params(AnkleParams{});
  return k;
}
}  // namespace

TEST(AnkleKinematicsV2, Neutral)
{
  auto k = cubic();
  const FkResult f = k->fk(0.0, 0.0, kNaN, kNaN);
  EXPECT_TRUE(f.converged);
  EXPECT_NEAR(f.pitch, 0.0, 1e-7);
  EXPECT_NEAR(f.roll, 0.0, 1e-7);
  const IkResult ik = k->ik(0.0, 0.0, 0.0, 0.0);
  EXPECT_TRUE(ik.reachable);
  EXPECT_NEAR(ik.thetaA, 0.0, 1e-6);
  EXPECT_NEAR(ik.thetaB, 0.0, 1e-6);
}

// Confirmed hardware convention: equal motors -> roll, opposite -> pitch.
TEST(AnkleKinematicsV2, DifferentialConvention)
{
  auto k = cubic();
  const FkResult eq = k->fk(40 * kDeg, 40 * kDeg, kNaN, kNaN);   // same direction
  EXPECT_GT(std::abs(eq.roll), std::abs(eq.pitch));               // roll-dominant
  const FkResult op = k->fk(40 * kDeg, -40 * kDeg, kNaN, kNaN);  // opposite
  EXPECT_GT(std::abs(op.pitch), std::abs(op.roll));              // pitch-dominant
}

TEST(AnkleKinematicsV2, RoundTripInterior)
{
  auto k = cubic();
  int tested = 0;
  for (double A = -70; A <= 70 + 1e-9; A += 10) {
    for (double B = -70; B <= 70 + 1e-9; B += 10) {
      const FkResult f = k->fk(A * kDeg, B * kDeg, kNaN, kNaN);
      const IkResult ik = k->ik(f.pitch, f.roll, 0.0, 0.0);
      ASSERT_TRUE(ik.reachable) << "A=" << A << " B=" << B;
      EXPECT_NEAR(ik.thetaA / kDeg, A, 1e-2) << "A=" << A << " B=" << B;
      EXPECT_NEAR(ik.thetaB / kDeg, B, 1e-2) << "A=" << A << " B=" << B;
      ++tested;
    }
  }
  EXPECT_GT(tested, 100);
}

TEST(AnkleKinematicsV2, JacobianMatchesFiniteDiff)
{
  auto k = cubic();
  const double tA = 0.3, tB = -0.2, h = 1e-6;
  const FkResult f = k->fk(tA, tB, kNaN, kNaN);
  const Eigen::Matrix2d J = k->jacobian_at(tA, tB, f.pitch, f.roll);
  const double dpa = (k->fk(tA + h, tB, kNaN, kNaN).pitch - k->fk(tA - h, tB, kNaN, kNaN).pitch) / (2 * h);
  const double dpb = (k->fk(tA, tB + h, kNaN, kNaN).pitch - k->fk(tA, tB - h, kNaN, kNaN).pitch) / (2 * h);
  const double dra = (k->fk(tA + h, tB, kNaN, kNaN).roll - k->fk(tA - h, tB, kNaN, kNaN).roll) / (2 * h);
  const double drb = (k->fk(tA, tB + h, kNaN, kNaN).roll - k->fk(tA, tB - h, kNaN, kNaN).roll) / (2 * h);
  EXPECT_NEAR(J(0, 0), dpa, 1e-4);
  EXPECT_NEAR(J(0, 1), dpb, 1e-4);
  EXPECT_NEAR(J(1, 0), dra, 1e-4);
  EXPECT_NEAR(J(1, 1), drb, 1e-4);
  EXPECT_GT(std::abs(J.determinant()), 1e-6);
}

TEST(AnkleKinematicsV2, OutOfBoxUnreachable)
{
  auto k = cubic();
  const double lim = k->params().motor_limit;
  // A pose well beyond the +/-80 deg measured box.
  const IkResult ik = k->ik(1.2, 1.2, 0.0, 0.0);
  EXPECT_FALSE(ik.reachable);
  EXPECT_LE(std::abs(ik.thetaA), lim + 1e-6);
  EXPECT_LE(std::abs(ik.thetaB), lim + 1e-6);
}
