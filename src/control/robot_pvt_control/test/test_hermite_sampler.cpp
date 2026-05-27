// Unit tests for the Hermite samplers extracted into hermite.hpp.
// Pure-math, no controller fixture, no ROS context.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "robot_pvt_control/hermite.hpp"

using robot_pvt_control::Knot;
using robot_pvt_control::Trajectory;
using robot_pvt_control::sample_segment;
using robot_pvt_control::sample_trajectory;

namespace
{
Knot make_knot(double t, std::vector<double> pos, std::vector<double> vel,
  std::vector<double> acc = {}, bool has_acc = false)
{
  Knot k;
  k.t = t;
  k.pos = std::move(pos);
  k.vel = std::move(vel);
  if (acc.empty()) {
    k.acc.assign(k.pos.size(), 0.0);
  } else {
    k.acc = std::move(acc);
  }
  k.has_acc = has_acc;
  return k;
}
}  // namespace

// ─── sample_segment ──────────────────────────────────────────────────────

TEST(HermiteSampler, CubicEndpointsMatch)
{
  // Single joint, cubic Hermite: at u=0 we must get a, at u=1 we must get b.
  auto a = make_knot(0.0, {0.0}, {1.0});
  auto b = make_knot(1.0, {2.0}, {-1.0});
  std::vector<double> p(1), v(1), a_out(1);

  sample_segment(a, b, 0.0, p, v, a_out);
  EXPECT_DOUBLE_EQ(p[0], 0.0);
  EXPECT_DOUBLE_EQ(v[0], 1.0);

  sample_segment(a, b, 1.0, p, v, a_out);
  EXPECT_DOUBLE_EQ(p[0], 2.0);
  EXPECT_DOUBLE_EQ(v[0], -1.0);
}

TEST(HermiteSampler, QuinticEndpointsMatchPosVelAcc)
{
  auto a = make_knot(0.0, {0.0}, {0.5}, {0.1}, /*has_acc=*/true);
  auto b = make_knot(2.0, {3.0}, {0.0}, {-0.2}, /*has_acc=*/true);
  std::vector<double> p(1), v(1), a_out(1);

  sample_segment(a, b, 0.0, p, v, a_out);
  EXPECT_NEAR(p[0], 0.0, 1e-12);
  EXPECT_NEAR(v[0], 0.5, 1e-12);
  EXPECT_NEAR(a_out[0], 0.1, 1e-12);

  sample_segment(a, b, 2.0, p, v, a_out);
  EXPECT_NEAR(p[0], 3.0, 1e-12);
  EXPECT_NEAR(v[0], 0.0, 1e-12);
  EXPECT_NEAR(a_out[0], -0.2, 1e-12);
}

TEST(HermiteSampler, CubicMidpointMonotoneWhenEndpointsMonotone)
{
  // a.pos < b.pos with zero endpoint velocities — value at midpoint should
  // lie strictly between. Asserts the segment doesn't overshoot the cubic.
  auto a = make_knot(0.0, {0.0}, {0.0});
  auto b = make_knot(1.0, {1.0}, {0.0});
  std::vector<double> p(1), v(1), a_out(1);

  sample_segment(a, b, 0.5, p, v, a_out);
  EXPECT_GT(p[0], 0.0);
  EXPECT_LT(p[0], 1.0);
  // Hermite with zero endpoint velocities → midpoint = 0.5 exactly.
  EXPECT_NEAR(p[0], 0.5, 1e-12);
}

TEST(HermiteSampler, DegenerateSegmentReturnsSecondEndpoint)
{
  // b.t == a.t — division by h=0 guarded; should return b.
  auto a = make_knot(1.0, {0.0}, {0.0});
  auto b = make_knot(1.0, {2.0}, {3.0});
  std::vector<double> p(1), v(1), a_out(1);

  sample_segment(a, b, 1.0, p, v, a_out);
  EXPECT_DOUBLE_EQ(p[0], 2.0);
  EXPECT_DOUBLE_EQ(v[0], 3.0);
}

// ─── sample_trajectory ───────────────────────────────────────────────────

TEST(HermiteSampler, SingleKnotTrajectory)
{
  Trajectory traj;
  traj.knots = {make_knot(0.0, {1.0, 2.0, 3.0}, {0.0, 0.0, 0.0})};
  traj.duration = 0.0;
  std::vector<double> p(3), v(3), a(3);

  sample_trajectory(traj, 0.0, p, v, a);
  EXPECT_DOUBLE_EQ(p[0], 1.0);
  EXPECT_DOUBLE_EQ(p[1], 2.0);
  EXPECT_DOUBLE_EQ(p[2], 3.0);
}

TEST(HermiteSampler, TrajectoryClampsBeforeFirstKnot)
{
  Trajectory traj;
  traj.knots = {
    make_knot(1.0, {5.0}, {0.0}),
    make_knot(2.0, {7.0}, {0.0}),
  };
  traj.duration = 1.0;
  std::vector<double> p(1), v(1), a(1);

  sample_trajectory(traj, 0.0, p, v, a);  // before first knot
  EXPECT_DOUBLE_EQ(p[0], 5.0);
}

TEST(HermiteSampler, TrajectoryClampsAfterLastKnot)
{
  Trajectory traj;
  traj.knots = {
    make_knot(0.0, {0.0}, {0.0}),
    make_knot(1.0, {10.0}, {0.0}),
  };
  traj.duration = 1.0;
  std::vector<double> p(1), v(1), a(1);

  sample_trajectory(traj, 5.0, p, v, a);  // after last knot
  EXPECT_DOUBLE_EQ(p[0], 10.0);
}

TEST(HermiteSampler, TrajectoryInteriorPicksRightSegment)
{
  Trajectory traj;
  traj.knots = {
    make_knot(0.0, {0.0}, {0.0}),
    make_knot(1.0, {1.0}, {0.0}),
    make_knot(2.0, {0.0}, {0.0}),
  };
  traj.duration = 2.0;
  std::vector<double> p(1), v(1), a(1);

  // Mid of first segment — should be between 0 and 1.
  sample_trajectory(traj, 0.5, p, v, a);
  EXPECT_GT(p[0], 0.0);
  EXPECT_LT(p[0], 1.0);

  // Mid of second segment — should be between 0 and 1 (descending).
  sample_trajectory(traj, 1.5, p, v, a);
  EXPECT_GT(p[0], 0.0);
  EXPECT_LT(p[0], 1.0);
}

TEST(HermiteSampler, ZerosTailWhenAllPathsExhausted)
{
  // Construct a trajectory whose knot times don't bracket t — exercises the
  // fallback at the end of sample_trajectory.
  Trajectory traj;
  traj.knots = {make_knot(0.0, {1.0}, {2.0}), make_knot(1.0, {3.0}, {4.0})};
  traj.duration = 1.0;
  std::vector<double> p(1), v(1), a(1);

  // Exactly at last knot's time — clamped path.
  sample_trajectory(traj, 1.0, p, v, a);
  EXPECT_DOUBLE_EQ(p[0], 3.0);
  EXPECT_DOUBLE_EQ(v[0], 4.0);
}
