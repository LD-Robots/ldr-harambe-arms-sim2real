#include "harambe_ankle_ethercat_driver_v2/ankle_kinematics.hpp"

#include "harambe_ankle_ethercat_driver_v2/analytic_ankle_kinematics.hpp"
#include "harambe_ankle_ethercat_driver_v2/cubic_ankle_kinematics.hpp"

namespace harambe_ankle_ethercat_driver_v2
{

std::unique_ptr<AnkleKinematics> AnkleKinematics::create(const std::string & backend)
{
  if (backend == "analytic") {
    return std::make_unique<AnalyticAnkleKinematics>();
  }
  // Default / "cubic": the validated primary backend.
  return std::make_unique<CubicAnkleKinematics>();
}

}  // namespace harambe_ankle_ethercat_driver_v2
