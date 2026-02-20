#include "myactuator_ethercat/unit_conversion.hpp"

namespace myactuator_ethercat
{

double UnitConversion::raw_to_rad(int32_t raw)
{
  return static_cast<double>(raw) * POSITION_FACTOR;
}

int32_t UnitConversion::rad_to_raw(double rad)
{
  return static_cast<int32_t>(rad / POSITION_FACTOR);
}

double UnitConversion::raw_to_rad_per_sec(int32_t raw)
{
  return static_cast<double>(raw) * VELOCITY_FACTOR;
}

int32_t UnitConversion::rad_per_sec_to_raw(double rad_per_sec)
{
  return static_cast<int32_t>(rad_per_sec / VELOCITY_FACTOR);
}

double UnitConversion::raw_to_torque_pct(int16_t raw)
{
  return static_cast<double>(raw) / TORQUE_SCALE * 100.0;
}

int16_t UnitConversion::torque_pct_to_raw(double pct)
{
  return static_cast<int16_t>(pct / 100.0 * TORQUE_SCALE);
}

}  // namespace myactuator_ethercat
