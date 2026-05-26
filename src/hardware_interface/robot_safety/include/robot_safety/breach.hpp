#ifndef ROBOT_SAFETY__BREACH_HPP_
#define ROBOT_SAFETY__BREACH_HPP_

#include "robot_safety/safety_limits.hpp"

namespace robot_safety
{

/// Breach predicates used by the supervisor. Each returns the matching
/// BreachReason, or BreachReason::NONE when within limits. A NaN input yields
/// NONE — missing telemetry must never by itself latch an e-stop.

BreachReason checkPosition(double position, const SafetyLimits & limits);
BreachReason checkVelocity(double velocity, const SafetyLimits & limits);
BreachReason checkMotorTemp(double temp, const SafetyLimits & limits);
BreachReason checkDriveTemp(double temp, const SafetyLimits & limits);
BreachReason checkBusVoltage(double voltage, const SafetyLimits & limits);

}  // namespace robot_safety

#endif  // ROBOT_SAFETY__BREACH_HPP_
