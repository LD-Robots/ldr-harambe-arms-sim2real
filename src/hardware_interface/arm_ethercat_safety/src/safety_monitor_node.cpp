#include "arm_ethercat_safety/safety_monitor_node.hpp"

#include <chrono>
#include <functional>
#include <set>
#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace arm_ethercat_safety
{

SafetyMonitorNode::SafetyMonitorNode(const rclcpp::NodeOptions & options)
: Node("safety_monitor",
    rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
{
  RCLCPP_INFO(get_logger(), "Initializing safety monitor node...");

  // --- Initialize Watchdog ---
  watchdog_ = std::make_unique<Watchdog>();
  Watchdog::Config wd_config;
  wd_config.command_timeout_ms = static_cast<int>(
    get_parameter_or("watchdog.command_timeout_ms", rclcpp::ParameterValue(50))
    .get<int64_t>());
  wd_config.ethercat_frame_loss_limit = static_cast<int>(
    get_parameter_or("watchdog.ethercat_frame_loss_limit", rclcpp::ParameterValue(3))
    .get<int64_t>());
  wd_config.action_on_timeout =
    get_parameter_or("watchdog.action_on_timeout",
      rclcpp::ParameterValue(std::string("hold_position"))).get<std::string>();
  watchdog_->configure(wd_config);

  int check_rate_hz = static_cast<int>(
    get_parameter_or("watchdog.check_rate_hz", rclcpp::ParameterValue(100))
    .get<int64_t>());

  // --- Initialize E-stop Handler ---
  estop_handler_ = std::make_unique<EstopHandler>();
  EstopHandler::Config estop_config;
  estop_config.gpio_pin = static_cast<int>(
    get_parameter_or("estop.gpio_pin", rclcpp::ParameterValue(-1))
    .get<int64_t>());
  estop_config.debounce_ms = static_cast<int>(
    get_parameter_or("estop.debounce_ms", rclcpp::ParameterValue(10))
    .get<int64_t>());
  estop_config.reaction =
    get_parameter_or("estop.reaction",
      rclcpp::ParameterValue(std::string("quickstop"))).get<std::string>();
  estop_handler_->configure(estop_config);

  // --- Initialize Joint Limit Monitor ---
  joint_limit_monitor_ = std::make_unique<JointLimitMonitor>();

  // Discover joint names from parameters loaded via safety_limits.yaml.
  // Parameters are like: joints.<joint_name>.position_min
  std::vector<JointLimitMonitor::JointLimits> joint_limits;
  auto all_params = list_parameters({"joints"}, 2);

  std::set<std::string> discovered_joints;
  for (const auto & prefix : all_params.prefixes) {
    // prefix format: "joints.left_shoulder_pitch_joint_X6"
    auto dot_pos = prefix.find('.');
    if (dot_pos != std::string::npos) {
      discovered_joints.insert(prefix.substr(dot_pos + 1));
    }
  }

  for (const auto & jname : discovered_joints) {
    JointLimitMonitor::JointLimits lim;
    lim.joint_name = jname;
    std::string p = "joints." + jname;

    lim.position_min = get_parameter(p + ".position_min").as_double();
    lim.position_max = get_parameter(p + ".position_max").as_double();
    lim.velocity_max = get_parameter(p + ".velocity_max").as_double();
    lim.torque_max_pct = get_parameter(p + ".torque_max_pct").as_double();
    lim.position_margin = get_parameter(p + ".position_margin").as_double();

    joint_limits.push_back(lim);
    RCLCPP_INFO(get_logger(), "Loaded safety limits for joint: %s "
      "[pos: %.2f..%.2f, vel: %.1f, torque: %.0f%%]",
      jname.c_str(), lim.position_min, lim.position_max,
      lim.velocity_max, lim.torque_max_pct);
  }

  joint_limit_monitor_->configure(joint_limits);

  // --- Initialize Fault Handler (not actively wired to drive status) ---
  fault_handler_ = std::make_unique<FaultHandler>();

  // --- Controller Management ---
  controller_name_ = get_parameter_or("safety_monitor.controller_name",
    rclcpp::ParameterValue(std::string("left_arm_controller"))).get<std::string>();

  switch_controller_client_ =
    create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");

  RCLCPP_INFO(get_logger(), "Controller to deactivate on violation: %s",
    controller_name_.c_str());

  // --- Create ROS Interfaces ---

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(),
    std::bind(&SafetyMonitorNode::joint_state_callback, this, _1));

  safety_status_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/safety/status", rclcpp::QoS(10));
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::QoS(10));

  estop_service_ = create_service<std_srvs::srv::Trigger>(
    "/safety/estop",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      estop_handler_->trigger_software_estop();
      system_safe_ = false;
      response->success = true;
      response->message = "Software e-stop triggered";
      RCLCPP_WARN(get_logger(), "SOFTWARE E-STOP TRIGGERED");
    });

  reset_service_ = create_service<std_srvs::srv::Trigger>(
    "/safety/reset",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      if (estop_handler_->reset()) {
        watchdog_->reset();
        fault_handler_->clear();
        system_safe_ = true;
        if (controller_deactivated_) {
          activate_controller();
        }
        response->success = true;
        response->message = "Safety system reset successful";
        RCLCPP_INFO(get_logger(), "Safety system reset");
      } else {
        response->success = false;
        response->message = "Cannot reset: hardware e-stop still active";
        RCLCPP_WARN(get_logger(), "Reset failed: hardware e-stop active");
      }
    });

  auto check_period = std::chrono::milliseconds(1000 / check_rate_hz);
  safety_timer_ = create_wall_timer(
    check_period,
    std::bind(&SafetyMonitorNode::safety_check_timer_callback, this));

  RCLCPP_INFO(get_logger(),
    "Safety monitor initialized: %zu joints, check rate=%d Hz",
    joint_limits.size(), check_rate_hz);
}

void SafetyMonitorNode::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Feed the watchdog -- valid data received
  first_joint_state_received_ = true;
  watchdog_->feed();

  // Check joint limits
  auto violations = joint_limit_monitor_->check(
    msg->position, msg->velocity, msg->effort);

  if (joint_limit_monitor_->has_error()) {
    system_safe_ = false;
    for (const auto & v : violations) {
      if (v.severity == JointLimitMonitor::Severity::ERROR) {
        RCLCPP_ERROR(get_logger(),
          "SAFETY VIOLATION: joint '%s' %s limit exceeded: actual=%.4f, limit=%.4f",
          v.joint_name.c_str(), v.limit_type.c_str(),
          v.actual_value, v.limit_value);
      }
    }
  }
}

void SafetyMonitorNode::safety_check_timer_callback()
{
  auto now = std::chrono::steady_clock::now();

  // Update subsystems
  watchdog_->update(now);
  estop_handler_->update();

  // Aggregate safety state
  bool was_safe = system_safe_;

  // Don't arm watchdog until first /joint_states message arrives
  // (joint_state_broadcaster may take 10+ seconds to start)
  if (first_joint_state_received_ && watchdog_->is_timed_out()) {
    system_safe_ = false;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Watchdog timeout! Action: %s", watchdog_->get_timeout_action().c_str());
  }

  if (estop_handler_->is_active()) {
    system_safe_ = false;
  }

  if (was_safe && !system_safe_) {
    RCLCPP_ERROR(get_logger(), "SYSTEM UNSAFE -- protective action required");
  }

  // Deactivate controller on any unsafe state (not just transition)
  if (!system_safe_ && !controller_deactivated_) {
    deactivate_controller();
  }

  // Publish safety status
  auto status_msg = std_msgs::msg::Bool();
  status_msg.data = system_safe_;
  safety_status_pub_->publish(status_msg);

  // Publish diagnostics
  auto diag_msg = diagnostic_msgs::msg::DiagnosticArray();
  diag_msg.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus wd_status;
  wd_status.name = "safety_monitor: watchdog";
  wd_status.hardware_id = "ethercat_arm";
  if (watchdog_->is_timed_out()) {
    wd_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    wd_status.message = "Communication timeout";
  } else {
    wd_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    wd_status.message = "OK";
  }
  diag_msg.status.push_back(wd_status);

  diagnostic_msgs::msg::DiagnosticStatus estop_status;
  estop_status.name = "safety_monitor: estop";
  estop_status.hardware_id = "ethercat_arm";
  if (estop_handler_->is_active()) {
    estop_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    estop_status.message = estop_handler_->is_hardware_estop() ?
      "Hardware e-stop active" : "Software e-stop active";
  } else {
    estop_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    estop_status.message = "OK";
  }
  diag_msg.status.push_back(estop_status);

  diagnostic_msgs::msg::DiagnosticStatus jl_status;
  jl_status.name = "safety_monitor: joint_limits";
  jl_status.hardware_id = "ethercat_arm";
  if (joint_limit_monitor_->has_error()) {
    jl_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    jl_status.message = "Joint limit violation";
    for (const auto & v : joint_limit_monitor_->get_violations()) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = v.joint_name + "/" + v.limit_type;
      kv.value = std::to_string(v.actual_value);
      jl_status.values.push_back(kv);
    }
  } else {
    jl_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    jl_status.message = "OK";
  }
  diag_msg.status.push_back(jl_status);

  diagnostic_msgs::msg::DiagnosticStatus overall;
  overall.name = "safety_monitor: overall";
  overall.hardware_id = "ethercat_arm";
  overall.level = system_safe_ ?
    diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  overall.message = system_safe_ ? "System safe" : "SYSTEM UNSAFE";
  diag_msg.status.push_back(overall);

  diagnostics_pub_->publish(diag_msg);
}

void SafetyMonitorNode::deactivate_controller()
{
  if (!switch_controller_client_->wait_for_service(0s)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "controller_manager not available — cannot deactivate %s",
      controller_name_.c_str());
    return;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->deactivate_controllers = {controller_name_};
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;

  controller_deactivated_ = true;

  switch_controller_client_->async_send_request(request,
    [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
      auto result = future.get();
      if (result->ok) {
        RCLCPP_WARN(get_logger(), "Controller '%s' DEACTIVATED by safety monitor",
          controller_name_.c_str());
      } else {
        // Keep controller_deactivated_ = true to avoid retry spam.
        // In viewer mode the controller doesn't exist — nothing to deactivate.
        // Only /safety/reset can clear this flag.
        RCLCPP_WARN(get_logger(),
          "Controller '%s' not found or already inactive — treating as deactivated",
          controller_name_.c_str());
      }
    });
}

void SafetyMonitorNode::activate_controller()
{
  if (!switch_controller_client_->wait_for_service(0s)) {
    RCLCPP_WARN(get_logger(),
      "controller_manager not available — cannot reactivate %s",
      controller_name_.c_str());
    return;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->activate_controllers = {controller_name_};
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;

  switch_controller_client_->async_send_request(request,
    [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
      auto result = future.get();
      if (result->ok) {
        controller_deactivated_ = false;
        RCLCPP_INFO(get_logger(), "Controller '%s' REACTIVATED after safety reset",
          controller_name_.c_str());
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to reactivate controller '%s'",
          controller_name_.c_str());
      }
    });
}

}  // namespace arm_ethercat_safety

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arm_ethercat_safety::SafetyMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
