#ifndef ROBOT_SAFETY__PARAM_LOADER_HPP_
#define ROBOT_SAFETY__PARAM_LOADER_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "robot_safety/safety_limits.hpp"

namespace rclcpp { class Node; }
namespace rclcpp_lifecycle { class LifecycleNode; }

namespace robot_safety
{

/// Per-joint limits, addressable by joint name. The `global` field carries the
/// defaults and applies to any joint absent from `per_joint`.
struct SafetyLimitsTable
{
  SafetyLimits global;
  std::unordered_map<std::string, SafetyLimits> per_joint;

  /// Resolve the limits for a specific joint: per-joint override if present,
  /// otherwise the global defaults. Never throws.
  const SafetyLimits & limitsFor(const std::string & joint) const
  {
    const auto it = per_joint.find(joint);
    return it == per_joint.end() ? global : it->second;
  }
};

/// Declare and read the safety limits under `prefix`.
///
/// Reads `<prefix>global.<field>` for shared defaults, then for each joint
/// `<prefix>joints.<joint>.<field>` as an override (defaulting to the global
/// value when absent). Two overloads cover plain rclcpp::Node (supervisor) and
/// rclcpp_lifecycle::LifecycleNode (controller's get_node()).
SafetyLimitsTable loadSafetyLimits(
  rclcpp::Node & node,
  const std::vector<std::string> & joint_names,
  const std::string & prefix = "safety.");
SafetyLimitsTable loadSafetyLimits(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::vector<std::string> & joint_names,
  const std::string & prefix = "safety.");

}  // namespace robot_safety

#endif  // ROBOT_SAFETY__PARAM_LOADER_HPP_
