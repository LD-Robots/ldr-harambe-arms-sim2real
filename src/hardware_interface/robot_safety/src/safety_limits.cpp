#include "robot_safety/safety_limits.hpp"

namespace robot_safety
{

const char * to_string(BreachReason reason)
{
  switch (reason) {
    case BreachReason::NONE:              return "NONE";
    case BreachReason::POSITION_LOW:      return "POSITION_LOW";
    case BreachReason::POSITION_HIGH:     return "POSITION_HIGH";
    case BreachReason::OVERSPEED:         return "OVERSPEED";
    case BreachReason::MOTOR_OVERTEMP:    return "MOTOR_OVERTEMP";
    case BreachReason::DRIVE_OVERTEMP:    return "DRIVE_OVERTEMP";
    case BreachReason::BUS_VOLTAGE_LOW:   return "BUS_VOLTAGE_LOW";
    case BreachReason::BUS_VOLTAGE_HIGH:  return "BUS_VOLTAGE_HIGH";
    case BreachReason::SUSTAINED_EFFORT:  return "SUSTAINED_EFFORT";
    case BreachReason::STALE_JOINT_STATE: return "STALE_JOINT_STATE";
    case BreachReason::MANUAL:            return "MANUAL";
    case BreachReason::STARTUP:           return "STARTUP";
  }
  return "UNKNOWN";
}

const char * to_string(EstopAction action)
{
  switch (action) {
    case EstopAction::FREE: return "FREE";
    case EstopAction::HOLD: return "HOLD";
    case EstopAction::DAMP: return "DAMP";
  }
  return "UNKNOWN";
}

}  // namespace robot_safety
