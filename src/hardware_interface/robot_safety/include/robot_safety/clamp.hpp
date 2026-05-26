#ifndef ROBOT_SAFETY__CLAMP_HPP_
#define ROBOT_SAFETY__CLAMP_HPP_

#include "robot_safety/safety_limits.hpp"

namespace robot_safety
{

/// Symmetric clamp to [-limit, +limit]. A non-positive limit disables the
/// clamp (the value passes through). Realtime-safe: no allocation, no locking,
/// no logging.
inline double clampSymmetric(double value, double limit)
{
  if (limit <= 0.0) {
    return value;
  }
  if (value < -limit) {
    return -limit;
  }
  if (value > limit) {
    return limit;
  }
  return value;
}

inline double clampEffort(double effort, const SafetyLimits & limits)
{
  return clampSymmetric(effort, limits.effort_limit);
}

inline double clampVelocity(double velocity, const SafetyLimits & limits)
{
  return clampSymmetric(velocity, limits.velocity_limit);
}

inline double clampPosition(double position, const SafetyLimits & limits)
{
  if (!limits.position_limits_enable) {
    return position;
  }
  if (position < limits.position_min) {
    return limits.position_min;
  }
  if (position > limits.position_max) {
    return limits.position_max;
  }
  return position;
}

}  // namespace robot_safety

#endif  // ROBOT_SAFETY__CLAMP_HPP_
