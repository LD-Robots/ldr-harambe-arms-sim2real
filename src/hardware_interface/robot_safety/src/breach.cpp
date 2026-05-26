#include "robot_safety/breach.hpp"

#include <cmath>

namespace robot_safety
{

BreachReason checkPosition(double position, const SafetyLimits & limits)
{
  if (!std::isfinite(position) || !limits.position_limits_enable) {
    return BreachReason::NONE;
  }
  if (position < limits.position_min) {
    return BreachReason::POSITION_LOW;
  }
  if (position > limits.position_max) {
    return BreachReason::POSITION_HIGH;
  }
  return BreachReason::NONE;
}

BreachReason checkVelocity(double velocity, const SafetyLimits & limits)
{
  if (!std::isfinite(velocity) || limits.velocity_limit <= 0.0) {
    return BreachReason::NONE;
  }
  return std::abs(velocity) > limits.velocity_limit
         ? BreachReason::OVERSPEED
         : BreachReason::NONE;
}

BreachReason checkMotorTemp(double temp, const SafetyLimits & limits)
{
  if (!std::isfinite(temp)) {
    return BreachReason::NONE;
  }
  return temp >= limits.motor_temp_error
         ? BreachReason::MOTOR_OVERTEMP
         : BreachReason::NONE;
}

BreachReason checkDriveTemp(double temp, const SafetyLimits & limits)
{
  if (!std::isfinite(temp)) {
    return BreachReason::NONE;
  }
  return temp >= limits.drive_temp_error
         ? BreachReason::DRIVE_OVERTEMP
         : BreachReason::NONE;
}

BreachReason checkBusVoltage(double voltage, const SafetyLimits & limits)
{
  if (!std::isfinite(voltage)) {
    return BreachReason::NONE;
  }
  if (voltage < limits.bus_voltage_min) {
    return BreachReason::BUS_VOLTAGE_LOW;
  }
  if (voltage > limits.bus_voltage_max) {
    return BreachReason::BUS_VOLTAGE_HIGH;
  }
  return BreachReason::NONE;
}

}  // namespace robot_safety
