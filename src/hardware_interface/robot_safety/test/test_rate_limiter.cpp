// Unit tests for robot_safety::RateLimiter — the deceleration-aware
// slew + acceleration limiter every controller leans on.

#include <cmath>

#include <gtest/gtest.h>

#include "robot_safety/rate_limiter.hpp"

using robot_safety::RateLimiter;

TEST(RateLimiter, FirstCallSeedsAndPassesThrough)
{
  RateLimiter rl;
  EXPECT_FALSE(rl.seeded());
  const double out = rl.limit(1.0, 0.01, 10.0, 100.0);
  EXPECT_DOUBLE_EQ(out, 1.0);
  EXPECT_TRUE(rl.seeded());
}

TEST(RateLimiter, ExplicitSeedMakesSeededTrue)
{
  RateLimiter rl;
  rl.seed(0.5);
  EXPECT_TRUE(rl.seeded());
}

TEST(RateLimiter, ResetUnseeds)
{
  RateLimiter rl;
  rl.seed(0.5);
  EXPECT_TRUE(rl.seeded());
  rl.reset();
  EXPECT_FALSE(rl.seeded());
}

TEST(RateLimiter, DtZeroHoldsPreviousCommand)
{
  RateLimiter rl;
  rl.seed(0.5);
  const double out = rl.limit(2.0, 0.0, 10.0, 100.0);
  EXPECT_DOUBLE_EQ(out, 0.5);
}

TEST(RateLimiter, SlewRateClampsLargeStep)
{
  RateLimiter rl;
  rl.seed(0.0);
  // Disable accel layer (accel_limit<=0): pure slew clamp.
  // Step from 0 → 10 with slew=2 rad/s and dt=0.1 → max delta per tick = 0.2.
  const double out = rl.limit(10.0, 0.1, 2.0, 0.0);
  EXPECT_NEAR(out, 0.2, 1e-12);
}

TEST(RateLimiter, AccelLimitBrakesBeforeOvershoot)
{
  // Deceleration-aware: after enough ticks the limiter must settle on the
  // target without ringing — final state == target, velocity == 0.
  RateLimiter rl;
  rl.seed(0.0);
  const double dt = 0.01;
  double cmd = 0.0;
  for (int i = 0; i < 1000; ++i) {
    cmd = rl.limit(1.0, dt, /*slew=*/10.0, /*accel=*/20.0);
  }
  EXPECT_NEAR(cmd, 1.0, 1e-9);

  // One more tick — same input — must be a no-op (settled).
  EXPECT_NEAR(rl.limit(1.0, dt, 10.0, 20.0), 1.0, 1e-12);
}

TEST(RateLimiter, SettlesExactlyOnTargetWhenStepWouldCross)
{
  // The limiter explicitly snaps to the target when one tick would otherwise
  // overshoot — assertion in rate_limiter.cpp lines 47-50.
  RateLimiter rl;
  rl.seed(0.95);
  const double out = rl.limit(1.0, 1.0, 100.0, 100.0);  // huge dt vs small gap
  EXPECT_DOUBLE_EQ(out, 1.0);
}

TEST(RateLimiter, SymmetricForNegativeDirection)
{
  RateLimiter rl;
  rl.seed(0.0);
  const double up = rl.limit(1.0, 0.1, 2.0, 0.0);
  RateLimiter rl2;
  rl2.seed(0.0);
  const double dn = rl2.limit(-1.0, 0.1, 2.0, 0.0);
  EXPECT_NEAR(up, -dn, 1e-12);
}

TEST(RateLimiter, ZeroSlewAndZeroAccelDisablesBothLayers)
{
  RateLimiter rl;
  rl.seed(0.0);
  // Both layers off — limiter degenerates to pass-through after the first
  // call. Setting them to 0 disables each layer per the doc comment.
  const double out = rl.limit(5.0, 0.01, 0.0, 0.0);
  EXPECT_NEAR(out, 5.0, 1e-12);  // step delivered in one tick
}

TEST(RateLimiter, SeededVelocityInfluencesFirstStep)
{
  // When seeded with a non-zero velocity, the next step's accel-clamped
  // velocity is bounded by [prev_vel - a*dt, prev_vel + a*dt] — so an
  // already-moving limiter doesn't pretend to be at rest.
  RateLimiter rl_moving;
  rl_moving.seed(0.0, /*velocity=*/5.0);  // moving forward
  RateLimiter rl_rest;
  rl_rest.seed(0.0, /*velocity=*/0.0);
  // Both targeting the same point, same constraints — moving one must reach
  // closer to target on the first tick.
  const double out_moving = rl_moving.limit(10.0, 0.01, 100.0, 10.0);
  const double out_rest   = rl_rest.limit(10.0, 0.01, 100.0, 10.0);
  EXPECT_GT(out_moving, out_rest);
}
