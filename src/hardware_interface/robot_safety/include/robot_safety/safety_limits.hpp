#ifndef ROBOT_SAFETY__SAFETY_LIMITS_HPP_
#define ROBOT_SAFETY__SAFETY_LIMITS_HPP_

#include <cstdint>

namespace robot_safety
{

/// Why the supervisor latched an e-stop. NONE means no active breach.
enum class BreachReason : uint8_t
{
  NONE = 0,
  POSITION_LOW,
  POSITION_HIGH,
  OVERSPEED,
  MOTOR_OVERTEMP,
  DRIVE_OVERTEMP,
  BUS_VOLTAGE_LOW,
  BUS_VOLTAGE_HIGH,
  SUSTAINED_EFFORT,
  STALE_JOINT_STATE,
  MANUAL,
  STARTUP,    ///< boot-time latch: drives limp until the operator calls ~/reset
};

/// What a control path should do while an e-stop is latched.
enum class EstopAction : uint8_t
{
  FREE = 0,   ///< zero all torque — every joint coasts
  HOLD = 1,   ///< hold the position latched when the e-stop fired
};

/// Per-joint plain-data limit set. POD: trivially copyable into a realtime
/// buffer, no heap. Every field has a default; a partially-specified YAML still
/// yields a valid struct. The supervisor stores one of these per joint
/// (positions, velocities, etc. differ wildly between e.g. an ankle and a
/// wrist).
struct SafetyLimits
{
  // 3.1 position
  bool   position_limits_enable = true;
  double position_min = -3.141592653589793;   ///< rad — symmetric default; overridden per joint
  double position_max =  3.141592653589793;   ///< rad

  // 3.2 effort
  double effort_limit = 20.0;                  ///< Nm, symmetric
  double sustained_effort_threshold = 16.0;    ///< Nm — "high effort" level
  double sustained_effort_window_sec = 1.0;    ///< s above threshold => breach

  // 3.3 velocity
  double velocity_limit = 16.02;               ///< rad/s
  double slew_rate_limit = 5.5;                ///< rad/s — position-command slew
  double acceleration_limit = 60.0;            ///< rad/s^2 — command accel cap

  // 3.4 temperature / bus voltage (whole-drive properties; here per joint
  //     because each joint has its own motor + drive thermal envelope)
  double motor_temp_warn  = 70.0;              ///< degC — start derating Kp
  double motor_temp_error = 90.0;              ///< degC — e-stop
  double drive_temp_warn  = 70.0;              ///< degC
  double drive_temp_error = 85.0;              ///< degC
  double bus_voltage_min  = 40.0;              ///< V
  double bus_voltage_max  = 54.0;              ///< V
  double kp_scale_floor   = 0.3;               ///< Kp multiplier at/above error

  // watchdog (global; one timeout applies to /joint_states for the whole bus)
  double joint_state_timeout_sec = 0.25;       ///< s — /joint_states freshness
};

/// Human-readable name for a breach reason (string literal, no allocation).
const char * to_string(BreachReason reason);

/// Human-readable name for an e-stop action (string literal, no allocation).
const char * to_string(EstopAction action);

}  // namespace robot_safety

#endif  // ROBOT_SAFETY__SAFETY_LIMITS_HPP_
