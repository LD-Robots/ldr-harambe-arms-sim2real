#include "robot_pvt_control/body_group.hpp"

namespace robot_pvt_control
{

bool is_valid_body_group(const std::string & body_group)
{
  return body_group == "arms" || body_group == "arms_waist" ||
    body_group == "legs" || body_group == "full";
}

bool joint_in_group(
  const std::string & joint, const std::string & body_group)
{
  auto has = [&](const char * s) { return joint.find(s) != std::string::npos; };
  const bool is_arm   = has("shoulder") || has("elbow") || has("wrist");
  const bool is_waist = has("waist");
  const bool is_leg   = has("hip") || has("knee") || has("ankle");
  if (body_group == "arms")       return is_arm;
  if (body_group == "arms_waist") return is_arm || is_waist;
  if (body_group == "legs")       return is_leg;
  return is_arm || is_waist || is_leg;  // "full" or anything else (validated upstream)
}

}  // namespace robot_pvt_control
