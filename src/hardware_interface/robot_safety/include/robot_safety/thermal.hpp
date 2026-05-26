#ifndef ROBOT_SAFETY__THERMAL_HPP_
#define ROBOT_SAFETY__THERMAL_HPP_

#include "robot_safety/safety_limits.hpp"

namespace robot_safety
{

/// Thermal Kp derating curve. Returns a multiplier in [floor, 1.0]:
///   - 1.0 for temp <= warn
///   - linear ramp from 1.0 down to `floor` between warn and error
///   - `floor` for temp >= error
/// A NaN temperature (missing telemetry) yields 1.0 (no derating).
double kpScaleForTemp(double temp, double warn, double error, double floor);

/// Combined Kp scale for the motor and drive temperatures: the minimum of the
/// two individual scales, so the hotter device dominates. NaN temperatures are
/// ignored (treated as no derating).
double kpScaleCombined(double motor_temp, double drive_temp,
  const SafetyLimits & limits);

}  // namespace robot_safety

#endif  // ROBOT_SAFETY__THERMAL_HPP_
