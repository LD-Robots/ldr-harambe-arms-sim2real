#include "robot_safety/thermal.hpp"

#include <algorithm>
#include <cmath>

namespace robot_safety
{

double kpScaleForTemp(double temp, double warn, double error, double floor)
{
  if (!std::isfinite(temp)) {
    return 1.0;  // missing telemetry — no derating
  }
  floor = std::clamp(floor, 0.0, 1.0);

  if (error <= warn) {
    // Degenerate band — no ramp; derate fully at/above the error threshold.
    return temp >= error ? floor : 1.0;
  }
  if (temp <= warn) {
    return 1.0;
  }
  if (temp >= error) {
    return floor;
  }
  const double frac = (temp - warn) / (error - warn);  // 0..1 across the band
  return 1.0 - frac * (1.0 - floor);
}

double kpScaleCombined(double motor_temp, double drive_temp,
  const SafetyLimits & limits)
{
  const double motor_scale = kpScaleForTemp(
    motor_temp, limits.motor_temp_warn, limits.motor_temp_error,
    limits.kp_scale_floor);
  const double drive_scale = kpScaleForTemp(
    drive_temp, limits.drive_temp_warn, limits.drive_temp_error,
    limits.kp_scale_floor);
  return std::min(motor_scale, drive_scale);
}

}  // namespace robot_safety
