#ifndef MYACTUATOR_ETHERCAT__UNIT_CONVERSION_HPP_
#define MYACTUATOR_ETHERCAT__UNIT_CONVERSION_HPP_

#include <cmath>
#include <cstdint>

namespace myactuator_ethercat
{

/// Unit conversions between raw actuator values and SI units.
///
/// Position: raw ±65535 = ±180° (±π rad)
///   rad = raw × (π / 65535)
///   raw = rad × (65535 / π)
///
/// Velocity: raw unit defined by drive firmware
///   RPM = (raw × 60) / 131072
///   rad/s = raw × (2π / 131072)
///
/// Torque: raw = thousandths of rated current (default 10A)
///   Amps = (raw / 1000) × I_rated
///   Nm = Kt × I × gear_ratio × efficiency
class UnitConversion
{
public:
  // Position conversions
  static double raw_to_rad(int32_t raw);
  static int32_t rad_to_raw(double rad);

  // Velocity conversions
  static double raw_to_rad_per_sec(int32_t raw);
  static int32_t rad_per_sec_to_raw(double rad_per_sec);

  // Torque conversions (returns current in thousandths of rated)
  static double raw_to_torque_pct(int16_t raw);
  static int16_t torque_pct_to_raw(double pct);

private:
  static constexpr double POSITION_FACTOR = M_PI / 65535.0;
  static constexpr double VELOCITY_FACTOR = (2.0 * M_PI) / 131072.0;
  static constexpr double TORQUE_SCALE    = 1000.0;
};

}  // namespace myactuator_ethercat

#endif  // MYACTUATOR_ETHERCAT__UNIT_CONVERSION_HPP_
