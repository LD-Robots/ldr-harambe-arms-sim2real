#ifndef ARM_ETHERCAT_SAFETY__SAFETY_MONITOR_NODE_HPP_
#define ARM_ETHERCAT_SAFETY__SAFETY_MONITOR_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include "arm_ethercat_safety/watchdog.hpp"
#include "arm_ethercat_safety/estop_handler.hpp"
#include "arm_ethercat_safety/joint_limit_monitor.hpp"
#include "arm_ethercat_safety/fault_handler.hpp"

namespace arm_ethercat_safety
{

/// Main safety monitoring node for the EtherCAT arm system.
///
/// Subscribes to:
///   - /joint_states (sensor_msgs/JointState) — position, velocity, effort
///   - /ethercat_status (custom) — drive status words, error codes
///
/// Publishes:
///   - /safety/status (std_msgs/Bool) — true = safe, false = fault
///   - /diagnostics (diagnostic_msgs/DiagnosticArray) — detailed status
///
/// Services:
///   - /safety/estop (std_srvs/Trigger) — software emergency stop
///   - /safety/reset (std_srvs/Trigger) — reset from fault state
///
/// Aggregates results from watchdog, e-stop handler, joint limit monitor,
/// and fault handler. If ANY subsystem reports unsafe, triggers protective action.
class SafetyMonitorNode : public rclcpp::Node
{
public:
  explicit SafetyMonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void safety_check_timer_callback();

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

  // Timer for periodic safety checks
  rclcpp::TimerBase::SharedPtr safety_timer_;

  // Safety subsystems
  std::unique_ptr<Watchdog> watchdog_;
  std::unique_ptr<EstopHandler> estop_handler_;
  std::unique_ptr<JointLimitMonitor> joint_limit_monitor_;
  std::unique_ptr<FaultHandler> fault_handler_;

  bool system_safe_{true};
};

}  // namespace arm_ethercat_safety

#endif  // ARM_ETHERCAT_SAFETY__SAFETY_MONITOR_NODE_HPP_
