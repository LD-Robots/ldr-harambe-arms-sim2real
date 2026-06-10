// Acceptance gate: the PRIMARY (cubic) kinematics must reproduce the real
// measured 9x9 lever grid. The analytic serial-chain is scored too, only to
// document (not assert) that it is structurally inadequate for this mechanism.
//
// Mapping (confirmed on hardware): thetaA = inside lever (rows), thetaB = outside
// lever (cols); 1:1 (lever deg = crank deg), lever 0 = flat; equal motors -> roll,
// opposite -> pitch. Validated in the physical convention (pitch_sign=roll_sign=1).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#include "harambe_ankle_ethercat_driver_v2/ankle_kinematics.hpp"

using harambe_ankle_ethercat_driver_v2::AnkleKinematics;
using harambe_ankle_ethercat_driver_v2::AnkleParams;
using harambe_ankle_ethercat_driver_v2::FkResult;

namespace
{
constexpr double kDeg = M_PI / 180.0;
constexpr int N = 9;
const double kLever[N] = {-80, -60, -40, -20, 0, 20, 40, 60, 80};

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

struct Stats { double rms_p, max_p, rms_r, max_r; };

Stats score(const AnkleKinematics & k, bool verbose, const char * tag)
{
  double sp = 0, sr = 0, mp = 0, mr = 0;
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < N; ++j) {
      const FkResult f = k.fk(kLever[i] * kDeg, kLever[j] * kDeg,
                              std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::quiet_NaN());
      const double ep = f.pitch / kDeg - kPitch[i][j];
      const double er = f.roll / kDeg - kRoll[i][j];
      sp += ep * ep; sr += er * er;
      mp = std::max(mp, std::abs(ep)); mr = std::max(mr, std::abs(er));
    }
  }
  Stats s{std::sqrt(sp / (N * N)), mp, std::sqrt(sr / (N * N)), mr};
  if (verbose) {
    std::printf("[%s] pitch RMS=%.4f max=%.4f | roll RMS=%.4f max=%.4f (deg)\n",
                tag, s.rms_p, s.max_p, s.rms_r, s.max_r);
  }
  return s;
}
}  // namespace

TEST(AnkleGridCalibrationV2, CubicReproducesGrid)
{
  auto k = AnkleKinematics::create("cubic");
  k->set_params(AnkleParams{});
  const Stats s = score(*k, true, "cubic PRIMARY");
  EXPECT_LT(s.rms_p, 0.30);
  EXPECT_LT(s.rms_r, 0.40);
  EXPECT_LT(s.max_p, 1.00);
  EXPECT_LT(s.max_r, 1.00);
}

TEST(AnkleGridCalibrationV2, CubicNeutralIsFlat)
{
  auto k = AnkleKinematics::create("cubic");
  k->set_params(AnkleParams{});
  const FkResult f = k->fk(0.0, 0.0, std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::quiet_NaN());
  EXPECT_NEAR(f.pitch / kDeg, 0.0, 1e-6);
  EXPECT_NEAR(f.roll / kDeg, 0.0, 1e-6);
}

// Documents (does not gate) that the serial-chain is inadequate here.
TEST(AnkleGridCalibrationV2, AnalyticIsInadequate)
{
  auto k = AnkleKinematics::create("analytic");
  k->set_params(AnkleParams{});
  const Stats s = score(*k, true, "analytic COMPARE");
  // It should be far worse than the cubic — that is the whole point of keeping
  // it on a separate topic rather than in the control path.
  EXPECT_GT(s.rms_p + s.rms_r, 5.0);
  SUCCEED();
}
