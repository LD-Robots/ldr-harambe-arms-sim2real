#ifndef ROBOT_SAFETY__SUPERVISOR_NODE_HPP_
#define ROBOT_SAFETY__SUPERVISOR_NODE_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "robot_safety/param_loader.hpp"
#include "robot_safety/safety_limits.hpp"
#include "robot_safety/sustained_effort_monitor.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace robot_safety
{

/// Whole-body safety supervisor. Monitors per-joint position / velocity /
/// effort plus per-joint motor + drive temperature and bus voltage, detects
/// limit breaches and drives a single whole-robot, configurable,
/// manually-reset e-stop.
///
/// Published surface:
///   /safety/estop_state   std_msgs/Int8                latched (0/1/2)
///   /safety/kp_scale      std_msgs/Float64MultiArray   latched, N entries
///   /safety/breach_reason std_msgs/String              latched ("JOINT: REASON")
///   /diagnostics          diagnostic_msgs/DiagnosticArray
/// Services:
///   ~/estop  std_srvs/Trigger  manual e-stop (latches MANUAL on no joint)
///   ~/reset  std_srvs/Trigger  manual reset — refused while a breach is active
///
/// All callbacks and the watchdog timer run on one executor thread, so the
/// telemetry and latch state need no locking.
class SafetySupervisor : public rclcpp::Node
{
public:
  explicit SafetySupervisor(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onJointState(sensor_msgs::msg::JointState::SharedPtr msg);
  void onMotorTemp(std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void onDriveTemp(std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void onBusVoltage(std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void onWatchdog();
  void onEstopRequest(
    std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void onResetRequest(
    std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  /// Run every instantaneous breach predicate over all joints. Returns the
  /// first breach found and the offending joint index. NONE => no breach.
  struct BreachHit { BreachReason reason; int joint_idx; };
  BreachHit detectBreach();
  EstopAction actionFor(BreachReason reason) const;
  void latch(BreachReason reason, int joint_idx);
  void publishEstopState();
  void publishKpScale(const std::vector<double> & kp_scale);
  void publishDiagnostics(const std::vector<double> & kp_scale);

  /// Copy a Float64MultiArray into a per-joint state vector, defensively
  /// padding / truncating to num_joints_.
  void copyTelemetry(const std_msgs::msg::Float64MultiArray & msg,
    std::vector<double> & dst);

  // --- configuration ---
  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, std::size_t> joint_index_;
  std::size_t num_joints_ = 0;
  SafetyLimitsTable limits_;
  bool monitor_temperature_{true};
  double watchdog_rate_hz_{200.0};
  int report_period_cycles_{20};
  bool start_latched_{true};
  EstopAction action_temperature_{EstopAction::FREE};
  EstopAction action_overspeed_{EstopAction::HOLD};
  EstopAction action_position_{EstopAction::HOLD};
  EstopAction action_manual_{EstopAction::FREE};
  EstopAction action_stale_{EstopAction::HOLD};
  EstopAction action_sustained_{EstopAction::FREE};

  // --- latest telemetry (NaN until first received), one entry per joint ---
  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> effort_;
  std::vector<double> motor_temp_;
  std::vector<double> drive_temp_;
  std::vector<double> bus_voltage_;
  rclcpp::Time last_joint_state_;
  bool joint_state_received_{false};

  // --- e-stop latch ---
  bool estop_latched_{false};
  BreachReason latched_reason_{BreachReason::NONE};
  EstopAction latched_action_{EstopAction::FREE};
  int latched_joint_idx_{-1};
  std::vector<SustainedEffortMonitor> effort_monitors_;
  int cycle_count_{0};

  // --- startup arming ---
  double startup_grace_sec_{2.0};
  bool detection_armed_{false};
  bool warmup_started_{false};
  rclcpp::Time warmup_start_;

  // --- ROS interfaces ---
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr motor_temp_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr drive_temp_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr bus_voltage_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr estop_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr kp_scale_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr breach_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
};

}  // namespace robot_safety

#endif  // ROBOT_SAFETY__SUPERVISOR_NODE_HPP_
