// Acceptance tests for the exact differential-ankle serial-chain model.
// Pure math — no live EtherCAT bus or ROS graph required (prompt §12).

#include <gtest/gtest.h>

#include <cmath>

#include "harambe_ankle_ethercat_driver/ankle_model.hpp"

using harambe_ankle_ethercat_driver::AnkleModel;
using harambe_ankle_ethercat_driver::AnkleParams;
using harambe_ankle_ethercat_driver::FkResult;
using harambe_ankle_ethercat_driver::IkResult;

namespace
{
constexpr double kDeg = M_PI / 180.0;

// Default ASSUMED geometry, unit signs (test the math independent of the URDF flip).
AnkleModel default_model()
{
  AnkleModel m;
  AnkleParams p;  // docs defaults
  p.pitch_sign = 1.0;
  p.roll_sign = 1.0;
  m.set_params(p);
  return m;
}
}  // namespace

// §12.1 — derived cam heights and the parameter-error guard.
TEST(AnkleModel, DeriveHeights)
{
  AnkleModel m = default_model();
  const AnkleParams & p = m.params();
  // base = (78-63)^2 + (25-30)^2 mm^2 → in metres: (0.015)^2 + (-0.005)^2.
  // H_A = sqrt(0.255^2 - base) - 0.028 - 0; H_B = sqrt(0.180^2 - base) - 0.028.
  EXPECT_NEAR(p.H_A, 0.22651, 1e-4);
  EXPECT_NEAR(p.H_B, 0.15130, 1e-4);
  EXPECT_NEAR(p.H_A - p.H_B, 0.0752, 1e-3);

  // Negative case: a rod shorter than sqrt(base) → derive fails.
  AnkleParams bad;
  bad.L_A = 0.005;  // far too short
  AnkleModel m2;
  m2.set_params(bad);          // set_params swallows the failure
  EXPECT_FALSE(m2.derive_heights());
}

// §12.1 — neutral pose.
TEST(AnkleModel, Neutral)
{
  AnkleModel m = default_model();
  const FkResult f = m.fk(0.0, 0.0);
  EXPECT_TRUE(f.converged);
  EXPECT_NEAR(f.pitch, 0.0, 1e-9);
  EXPECT_NEAR(f.roll, 0.0, 1e-9);

  const IkResult ik = m.ik(0.0, 0.0, 0.0, 0.0);
  EXPECT_TRUE(ik.reachable);
  EXPECT_NEAR(ik.thetaA, 0.0, 1e-9);
  EXPECT_NEAR(ik.thetaB, 0.0, 1e-9);
}

// §12.2 — round trip fk(ik(pitch,roll)) == (pitch,roll) over a reachable grid.
TEST(AnkleModel, RoundTrip)
{
  AnkleModel m = default_model();
  double prevA = 0.0, prevB = 0.0;
  int tested = 0;
  for (double pitch = -0.30; pitch <= 0.30 + 1e-9; pitch += 0.05) {
    for (double roll = -0.60; roll <= 0.60 + 1e-9; roll += 0.10) {
      const IkResult ik = m.ik(pitch, roll, prevA, prevB);
      if (!ik.reachable) continue;
      prevA = ik.thetaA;
      prevB = ik.thetaB;
      const FkResult f = m.fk(ik.thetaA, ik.thetaB);
      ASSERT_TRUE(f.converged) << "pitch=" << pitch << " roll=" << roll;
      EXPECT_NEAR(f.pitch, pitch, 1e-4) << "pitch=" << pitch << " roll=" << roll;
      EXPECT_NEAR(f.roll, roll, 1e-4) << "pitch=" << pitch << " roll=" << roll;
      ++tested;
    }
  }
  EXPECT_GT(tested, 20);
}

// §12.3 — exact vs first-order agreement over the ±80° motor grid.
TEST(AnkleModel, ExactVsFirstOrderAgreement)
{
  AnkleModel m = default_model();
  double max_dp = 0.0, max_dr = 0.0;
  const double lim = 80.0 * kDeg;
  for (int i = 0; i <= 14; ++i) {
    for (int j = 0; j <= 14; ++j) {
      const double tA = -lim + 2.0 * lim * i / 14.0;
      const double tB = -lim + 2.0 * lim * j / 14.0;
      const FkResult e = m.fk(tA, tB);
      const FkResult l = m.first_order_fk(tA, tB);
      max_dp = std::max(max_dp, std::abs(e.pitch - l.pitch));
      max_dr = std::max(max_dr, std::abs(e.roll - l.roll));
    }
  }
  EXPECT_LE(max_dp, 1.2 * kDeg) << "max pitch deviation " << max_dp / kDeg << " deg";
  EXPECT_LE(max_dr, 2.3 * kDeg) << "max roll deviation " << max_dr / kDeg << " deg";
}

// §12.4 — cross-axis bleed.
//
// Bleed is only near-zero when the two sides are SYMMETRIC (equal rod lengths →
// equal derived cam heights). With symmetric geometry, y→−y reflection symmetry
// forces pure-pitch (θA=θB) to produce exactly zero roll, and pure-roll
// (θA=−θB) to produce < 0.5° pitch. This validates that the exact model adds no
// spurious coupling beyond what the geometry dictates.
//
// NOTE: with the ASYMMETRIC ASSUMED default geometry (rod A 255 ≠ rod B 180),
// equal motor angles genuinely tilt the foot (≈1–2° roll at ±80°). That coupling
// is physical — it is precisely what the exact serial-chain model captures and
// the old decoupled asin() model could not. So this test uses symmetric geometry;
// the asymmetric-bleed magnitude depends on the (to-be-confirmed) real geometry.
TEST(AnkleModel, CrossAxisBleedSymmetric)
{
  AnkleParams p;          // docs defaults, then make the two sides symmetric
  p.L_B = p.L_A;
  AnkleModel m;
  m.set_params(p);
  ASSERT_NEAR(m.params().H_A, m.params().H_B, 1e-12);

  const double lim = 80.0 * kDeg;
  for (int i = 0; i <= 20; ++i) {
    const double t = -lim + 2.0 * lim * i / 20.0;
    // Pure pitch: both motors equal → roll == 0 by symmetry.
    const FkResult fp = m.fk(t, t);
    EXPECT_LT(std::abs(fp.roll), 1e-3 * kDeg) << "theta=" << t / kDeg;
    // Pure roll: opposite motors → pitch ≈ 0 (small nonlinear term).
    const FkResult fr = m.fk(t, -t);
    EXPECT_LT(std::abs(fr.pitch), 0.5 * kDeg) << "theta=" << t / kDeg;
  }
}

// Companion to the above: with the asymmetric ASSUMED defaults the exact model
// MUST report nonzero roll bleed for a pure-pitch command (proving it models the
// cross-coupling the decoupled linkage ignored).
TEST(AnkleModel, AsymmetricGeometryCouples)
{
  AnkleModel m = default_model();   // asymmetric defaults (L_A=255, L_B=180)
  double max_bleed = 0.0;
  const double lim = 80.0 * kDeg;
  for (int i = 0; i <= 20; ++i) {
    const double t = -lim + 2.0 * lim * i / 20.0;
    max_bleed = std::max(max_bleed, std::abs(m.fk(t, t).roll));   // pure-pitch command
  }
  EXPECT_GT(max_bleed, 0.5 * kDeg);   // genuine coupling, not numerical noise
}

// §12.5 — reachability flag + never commands beyond ±motor_limit.
TEST(AnkleModel, ReachabilityAndClamp)
{
  AnkleModel m = default_model();
  const double lim = m.params().motor_limit;
  const IkResult ik = m.ik(1.4, 1.4, 0.0, 0.0);  // far out of workspace
  EXPECT_FALSE(ik.reachable);
  EXPECT_LE(std::abs(ik.thetaA), lim + 1e-9);
  EXPECT_LE(std::abs(ik.thetaB), lim + 1e-9);
}

// Branch continuity: the returned root is the in-range one nearest prev.
TEST(AnkleModel, BranchContinuity)
{
  AnkleModel m = default_model();
  // A reachable mild pose; solving from two different previous commands should
  // both land within limits and stay near the previous command.
  const double pitch = 0.10, roll = 0.05;
  const IkResult a = m.ik(pitch, roll, 0.0, 0.0);
  ASSERT_TRUE(a.reachable);
  const IkResult b = m.ik(pitch, roll, a.thetaA, a.thetaB);
  EXPECT_NEAR(a.thetaA, b.thetaA, 1e-9);
  EXPECT_NEAR(a.thetaB, b.thetaB, 1e-9);
  EXPECT_LE(std::abs(a.thetaA), m.params().motor_limit);
  EXPECT_LE(std::abs(a.thetaB), m.params().motor_limit);
}

// §7/§8 — numerical Jacobian matches finite-difference of fk; velocity and
// effort round-trip through J / J^T.
TEST(AnkleModel, JacobianAndDuality)
{
  AnkleModel m = default_model();
  const double tA = 0.25, tB = -0.15;
  const FkResult f = m.fk(tA, tB);
  const Eigen::Matrix2d J = m.jacobian(tA, tB, f.pitch, f.roll);

  // Finite-difference reference.
  const double h = 1e-5;
  const double dpa = (m.fk(tA + h, tB).pitch - m.fk(tA - h, tB).pitch) / (2 * h);
  const double dra = (m.fk(tA + h, tB).roll - m.fk(tA - h, tB).roll) / (2 * h);
  EXPECT_NEAR(J(0, 0), dpa, 1e-3);
  EXPECT_NEAR(J(1, 0), dra, 1e-3);

  ASSERT_GT(std::abs(J.determinant()), 1e-9);
  // Velocity round-trip: motor → joint → motor.
  const Eigen::Vector2d vm(0.8, -0.3);
  const Eigen::Vector2d vj = J * vm;
  const Eigen::Vector2d vm2 = J.fullPivLu().solve(vj);
  EXPECT_NEAR(vm2(0), vm(0), 1e-7);
  EXPECT_NEAR(vm2(1), vm(1), 1e-7);
  // Effort duality: tau_joint = J^{-T} (J^T tau_joint).
  const Eigen::Vector2d tj(1.5, 0.4);
  const Eigen::Vector2d tm = J.transpose() * tj;
  const Eigen::Vector2d tj2 = J.transpose().fullPivLu().solve(tm);
  EXPECT_NEAR(tj2(0), tj(0), 1e-7);
  EXPECT_NEAR(tj2(1), tj(1), 1e-7);
}

// Sign convention: pitch_sign = -1 flips the reported pitch but not reachability.
TEST(AnkleModel, SignConvention)
{
  AnkleParams p;
  p.pitch_sign = -1.0;
  p.roll_sign = 1.0;
  AnkleModel m;
  m.set_params(p);
  const FkResult f = m.fk(0.3, -0.2);   // arbitrary motor angles
  AnkleParams p2 = p;
  p2.pitch_sign = 1.0;
  AnkleModel m2;
  m2.set_params(p2);
  const FkResult f2 = m2.fk(0.3, -0.2);
  EXPECT_NEAR(f.pitch, -f2.pitch, 1e-9);
  EXPECT_NEAR(f.roll, f2.roll, 1e-9);
}

// Singularity-ish extreme stays finite (guarded solves).
TEST(AnkleModel, ExtremeStaysFinite)
{
  AnkleModel m = default_model();
  const double lim = 80.0 * kDeg;
  const FkResult f = m.fk(lim, lim);
  EXPECT_TRUE(std::isfinite(f.pitch));
  EXPECT_TRUE(std::isfinite(f.roll));
  const Eigen::Matrix2d J = m.jacobian(lim, lim, f.pitch, f.roll);
  EXPECT_TRUE(J.allFinite());
}
