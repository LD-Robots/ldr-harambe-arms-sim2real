#ifndef ROBOT_PVT_CONTROL__BODY_GROUP_HPP_
#define ROBOT_PVT_CONTROL__BODY_GROUP_HPP_

#include <string>

namespace robot_pvt_control
{

/// True iff `body_group` is one of the four recognised values:
/// "arms", "arms_waist", "legs", "full".
bool is_valid_body_group(const std::string & body_group);

/// True iff `joint` belongs in the active set for `body_group`. Substring
/// match on the joint name — naming convention is
/// `<chain>_<dof>_joint_X<n>` where chain ∈ {left, right} and
/// dof ∈ {shoulder*, elbow*, wrist*} (arms),
/// {hip*, knee, ankle*} (legs), or {waist_yaw} (waist).
///
/// Caller must validate `body_group` first with is_valid_body_group();
/// passing an unknown group is equivalent to "full" (everything active).
bool joint_in_group(
  const std::string & joint, const std::string & body_group);

}  // namespace robot_pvt_control

#endif  // ROBOT_PVT_CONTROL__BODY_GROUP_HPP_
