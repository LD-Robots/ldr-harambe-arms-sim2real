#include "robot_safety/param_loader.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace robot_safety
{
namespace
{

/// Declare-if-absent then read a double parameter. `has_parameter` guards
/// re-declaration so the loader is safe to call after the node has declared
/// parameters of its own.
template<typename NodeT>
double getDouble(NodeT & node, const std::string & name, double def)
{
  if (!node.has_parameter(name)) {
    node.declare_parameter(name, def);
  }
  return node.get_parameter(name).as_double();
}

template<typename NodeT>
bool getBool(NodeT & node, const std::string & name, bool def)
{
  if (!node.has_parameter(name)) {
    node.declare_parameter(name, def);
  }
  return node.get_parameter(name).as_bool();
}

template<typename NodeT>
SafetyLimits readLimits(NodeT & node, const std::string & p,
  const SafetyLimits & defaults)
{
  SafetyLimits L = defaults;
  L.position_limits_enable = getBool(node, p + "position_limits_enable",
    defaults.position_limits_enable);
  L.position_min = getDouble(node, p + "position_min", defaults.position_min);
  L.position_max = getDouble(node, p + "position_max", defaults.position_max);

  L.effort_limit = getDouble(node, p + "effort_limit", defaults.effort_limit);
  L.sustained_effort_threshold = getDouble(node,
    p + "sustained_effort_threshold", defaults.sustained_effort_threshold);
  L.sustained_effort_window_sec = getDouble(node,
    p + "sustained_effort_window_sec", defaults.sustained_effort_window_sec);

  L.velocity_limit = getDouble(node, p + "velocity_limit", defaults.velocity_limit);
  L.slew_rate_limit = getDouble(node, p + "slew_rate_limit", defaults.slew_rate_limit);
  L.acceleration_limit = getDouble(node, p + "acceleration_limit",
    defaults.acceleration_limit);

  L.motor_temp_warn = getDouble(node, p + "motor_temp_warn", defaults.motor_temp_warn);
  L.motor_temp_error = getDouble(node, p + "motor_temp_error", defaults.motor_temp_error);
  L.drive_temp_warn = getDouble(node, p + "drive_temp_warn", defaults.drive_temp_warn);
  L.drive_temp_error = getDouble(node, p + "drive_temp_error", defaults.drive_temp_error);
  L.bus_voltage_min = getDouble(node, p + "bus_voltage_min", defaults.bus_voltage_min);
  L.bus_voltage_max = getDouble(node, p + "bus_voltage_max", defaults.bus_voltage_max);
  L.kp_scale_floor = getDouble(node, p + "kp_scale_floor", defaults.kp_scale_floor);

  L.joint_state_timeout_sec = getDouble(node, p + "joint_state_timeout_sec",
    defaults.joint_state_timeout_sec);
  return L;
}

template<typename NodeT>
SafetyLimitsTable loadImpl(NodeT & node,
  const std::vector<std::string> & joint_names,
  const std::string & prefix)
{
  SafetyLimitsTable table;
  // 1. Global defaults — built-in SafetyLimits defaults provide the fallback
  //    when the YAML omits a field entirely.
  table.global = readLimits(node, prefix + "global.", SafetyLimits{});
  // 2. Per-joint overrides — each joint reads from its own block, defaulting
  //    to the global value if a particular field is not set in the YAML.
  for (const auto & joint : joint_names) {
    table.per_joint[joint] = readLimits(
      node, prefix + "joints." + joint + ".", table.global);
  }
  return table;
}

}  // namespace

SafetyLimitsTable loadSafetyLimits(rclcpp::Node & node,
  const std::vector<std::string> & joint_names, const std::string & prefix)
{
  return loadImpl(node, joint_names, prefix);
}

SafetyLimitsTable loadSafetyLimits(rclcpp_lifecycle::LifecycleNode & node,
  const std::vector<std::string> & joint_names, const std::string & prefix)
{
  return loadImpl(node, joint_names, prefix);
}

}  // namespace robot_safety
