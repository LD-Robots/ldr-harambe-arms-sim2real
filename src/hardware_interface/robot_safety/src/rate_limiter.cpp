#include "robot_safety/rate_limiter.hpp"

#include <algorithm>
#include <cmath>

namespace robot_safety
{

double RateLimiter::limit(double desired, double dt, double slew_rate,
  double accel_limit)
{
  if (!seeded_) {
    // First call after construction / reset() — seed to avoid a startup jump.
    prev_cmd_ = desired;
    prev_vel_ = 0.0;
    seeded_ = true;
    return desired;
  }
  if (dt <= 0.0) {
    return prev_cmd_;
  }

  const double error = desired - prev_cmd_;

  double cmd_vel;
  if (accel_limit > 0.0) {
    // Deceleration-aware target velocity: sqrt(2*a*|error|) is the fastest the
    // command may travel and still brake to rest exactly at `desired`. A purely
    // reactive limiter brakes too late and rings around a step input.
    double target_vel = std::copysign(
      std::sqrt(2.0 * accel_limit * std::abs(error)), error);
    if (slew_rate > 0.0) {
      target_vel = std::clamp(target_vel, -slew_rate, slew_rate);
    }
    const double dv = accel_limit * dt;
    cmd_vel = std::clamp(target_vel, prev_vel_ - dv, prev_vel_ + dv);
  } else {
    cmd_vel = error / dt;
    if (slew_rate > 0.0) {
      cmd_vel = std::clamp(cmd_vel, -slew_rate, slew_rate);
    }
  }

  const double cmd = prev_cmd_ + cmd_vel * dt;

  // Settle exactly on the target once a step would reach or cross it.
  if ((error >= 0.0 && cmd >= desired) || (error <= 0.0 && cmd <= desired)) {
    prev_cmd_ = desired;
    prev_vel_ = 0.0;
    return desired;
  }

  prev_cmd_ = cmd;
  prev_vel_ = cmd_vel;
  return cmd;
}

void RateLimiter::reset()
{
  seeded_ = false;
  prev_cmd_ = 0.0;
  prev_vel_ = 0.0;
}

void RateLimiter::seed(double position, double velocity)
{
  prev_cmd_ = position;
  prev_vel_ = velocity;
  seeded_ = true;
}

}  // namespace robot_safety
