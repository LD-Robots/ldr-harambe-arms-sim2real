#include "robot_safety/supervisor_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <sstream>
#include <string>

#include "robot_safety/breach.hpp"
#include "robot_safety/thermal.hpp"

namespace robot_safety
{
namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

EstopAction parseAction(const std::string & text, EstopAction fallback)
{
  if (text == "hold" || text == "HOLD") {
    return EstopAction::HOLD;
  }
  if (text == "free" || text == "FREE") {
    return EstopAction::FREE;
  }
  if (text == "damp" || text == "DAMP") {
    return EstopAction::DAMP;
  }
  return fallback;
}

}  // namespace

SafetySupervisor::SafetySupervisor(const rclcpp::NodeOptions & options)
: rclcpp::Node("robot_safety_supervisor", options),
  last_joint_state_(0, 0, RCL_ROS_TIME),
  warmup_start_(0, 0, RCL_ROS_TIME)
{
  // --- joint roster (required parameter, no default — refusing to start
  //     without it is safer than silently watching nothing) ---
  joint_names_ = declare_parameter<std::vector<std::string>>(
    "joint_names", std::vector<std::string>{});
  if (joint_names_.empty()) {
    RCLCPP_FATAL(get_logger(),
      "robot_safety_supervisor: 'joint_names' parameter is empty — refusing "
      "to start. Set it to the full body joint roster in safety_limits.yaml.");
    throw std::runtime_error("robot_safety_supervisor: missing joint_names");
  }
  num_joints_ = joint_names_.size();
  for (std::size_t i = 0; i < num_joints_; ++i) {
    joint_index_[joint_names_[i]] = i;
  }

  // --- topics + watchdog config ---
  const std::string joint_state_topic =
    declare_parameter<std::string>("joint_state_topic", "/joint_states");
  monitor_temperature_ = declare_parameter<bool>("monitor_temperature", true);
  const std::string motor_topic = declare_parameter<std::string>(
    "temperature_topic_motor", "/drive_status_broadcaster/motor_temperature");
  const std::string drive_topic = declare_parameter<std::string>(
    "temperature_topic_drive", "/drive_status_broadcaster/drive_temperature");
  const std::string voltage_topic = declare_parameter<std::string>(
    "bus_voltage_topic", "/drive_status_broadcaster/bus_voltage");
  watchdog_rate_hz_ = declare_parameter<double>("watchdog_rate_hz", 200.0);
  if (watchdog_rate_hz_ <= 0.0) {
    watchdog_rate_hz_ = 200.0;
  }
  startup_grace_sec_ = declare_parameter<double>("startup_grace_sec", 2.0);
  start_latched_ = declare_parameter<bool>("start_latched", true);
  report_period_cycles_ =
    std::max(1, static_cast<int>(std::lround(watchdog_rate_hz_ / 10.0)));

  action_temperature_ = parseAction(
    declare_parameter<std::string>("action.temperature", "free"),
    EstopAction::FREE);
  action_overspeed_ = parseAction(
    declare_parameter<std::string>("action.overspeed", "hold"),
    EstopAction::HOLD);
  action_position_ = parseAction(
    declare_parameter<std::string>("action.position", "hold"),
    EstopAction::HOLD);
  action_manual_ = parseAction(
    declare_parameter<std::string>("action.manual", "free"),
    EstopAction::FREE);
  action_stale_ = parseAction(
    declare_parameter<std::string>("action.stale_joint_state", "hold"),
    EstopAction::HOLD);
  action_sustained_ = parseAction(
    declare_parameter<std::string>("action.sustained_effort", "free"),
    EstopAction::FREE);

  limits_ = loadSafetyLimits(*this, joint_names_, "safety.");

  // --- preallocate per-joint state ---
  position_.assign(num_joints_, kNaN);
  velocity_.assign(num_joints_, kNaN);
  effort_.assign(num_joints_, kNaN);
  motor_temp_.assign(num_joints_, kNaN);
  drive_temp_.assign(num_joints_, kNaN);
  bus_voltage_.assign(num_joints_, kNaN);
  effort_monitors_.resize(num_joints_);

  // --- publishers (estop / kp_scale / breach_reason are latched) ---
  const auto latched = rclcpp::QoS(1).transient_local();
  estop_pub_ = create_publisher<std_msgs::msg::Int8>(
    "/safety/estop_state", latched);
  kp_scale_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/safety/kp_scale", latched);
  breach_pub_ = create_publisher<std_msgs::msg::String>(
    "/safety/breach_reason", latched);
  diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::QoS(10));

  if (start_latched_) {
    estop_latched_ = true;
    latched_reason_ = BreachReason::STARTUP;
    latched_action_ = EstopAction::FREE;
    latched_joint_idx_ = -1;
    RCLCPP_WARN(
      get_logger(),
      "boot latched (STARTUP, action FREE) — call ~/reset after the joint "
      "feed warms up to take the system out of e-stop");
  }
  // Seed latched topics so consumers spawned later get an initial value.
  publishEstopState();
  publishKpScale(std::vector<double>(num_joints_, 1.0));

  // --- subscriptions ---
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    joint_state_topic, rclcpp::SensorDataQoS(),
    std::bind(&SafetySupervisor::onJointState, this, std::placeholders::_1));
  if (monitor_temperature_) {
    motor_temp_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      motor_topic, rclcpp::SensorDataQoS(),
      std::bind(&SafetySupervisor::onMotorTemp, this, std::placeholders::_1));
    drive_temp_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      drive_topic, rclcpp::SensorDataQoS(),
      std::bind(&SafetySupervisor::onDriveTemp, this, std::placeholders::_1));
    bus_voltage_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      voltage_topic, rclcpp::SensorDataQoS(),
      std::bind(&SafetySupervisor::onBusVoltage, this, std::placeholders::_1));
  }

  // --- services ---
  estop_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/estop",
    std::bind(&SafetySupervisor::onEstopRequest, this,
      std::placeholders::_1, std::placeholders::_2));
  reset_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/reset",
    std::bind(&SafetySupervisor::onResetRequest, this,
      std::placeholders::_1, std::placeholders::_2));

  // --- watchdog timer ---
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / watchdog_rate_hz_));
  watchdog_timer_ = create_wall_timer(
    period, std::bind(&SafetySupervisor::onWatchdog, this));

  // Hot-reload limits_ when the PVT tuner (or any client) writes a safety.*
  // parameter. The controller mirrors this in robot_pvt_controller.cpp so
  // the supervisor and controller agree on the live envelope. Runs on the
  // same single executor thread as onWatchdog, so no locking is needed.
  post_set_param_cb_ = add_post_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      const bool safety_changed = std::any_of(
        params.begin(), params.end(),
        [](const rclcpp::Parameter & p) {
          return p.get_name().rfind("safety.", 0) == 0;
        });
      if (!safety_changed) {
        return;
      }
      limits_ = loadSafetyLimits(*this, joint_names_, "safety.");
      RCLCPP_INFO(get_logger(),
        "safety limits reloaded (post_set_parameters)");
    });

  RCLCPP_INFO(
    get_logger(),
    "robot_safety_supervisor active — %zu joints, watchdog %.0f Hz, "
    "temperature monitoring %s",
    num_joints_, watchdog_rate_hz_, monitor_temperature_ ? "on" : "off");
}

void SafetySupervisor::onJointState(sensor_msgs::msg::JointState::SharedPtr msg)
{
  const std::size_t n = msg->name.size();
  for (std::size_t i = 0; i < n; ++i) {
    const auto it = joint_index_.find(msg->name[i]);
    if (it == joint_index_.end()) {
      continue;  // joint not in our roster — silently skip
    }
    const std::size_t idx = it->second;
    if (i < msg->position.size()) {
      position_[idx] = msg->position[i];
    }
    if (i < msg->velocity.size()) {
      velocity_[idx] = msg->velocity[i];
    }
    if (i < msg->effort.size()) {
      effort_[idx] = msg->effort[i];
    }
  }
  last_joint_state_ = now();
  joint_state_received_ = true;
}

void SafetySupervisor::copyTelemetry(
  const std_msgs::msg::Float64MultiArray & msg, std::vector<double> & dst)
{
  const std::size_t n = std::min(num_joints_, msg.data.size());
  for (std::size_t i = 0; i < n; ++i) {
    dst[i] = msg.data[i];
  }
}

void SafetySupervisor::onMotorTemp(std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  copyTelemetry(*msg, motor_temp_);
}
void SafetySupervisor::onDriveTemp(std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  copyTelemetry(*msg, drive_temp_);
}
void SafetySupervisor::onBusVoltage(std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  copyTelemetry(*msg, bus_voltage_);
}

SafetySupervisor::BreachHit SafetySupervisor::detectBreach()
{
  // Stale joint-state is a whole-robot concern.
  if (joint_state_received_) {
    const double age = (now() - last_joint_state_).seconds();
    if (age > limits_.global.joint_state_timeout_sec) {
      return {BreachReason::STALE_JOINT_STATE, -1};
    }
  }

  for (std::size_t i = 0; i < num_joints_; ++i) {
    const SafetyLimits & L = limits_.limitsFor(joint_names_[i]);
    BreachReason reason = checkPosition(position_[i], L);
    if (reason != BreachReason::NONE) {
      return {reason, static_cast<int>(i)};
    }
    reason = checkVelocity(velocity_[i], L);
    if (reason != BreachReason::NONE) {
      return {reason, static_cast<int>(i)};
    }
    if (monitor_temperature_) {
      reason = checkMotorTemp(motor_temp_[i], L);
      if (reason != BreachReason::NONE) {
        return {reason, static_cast<int>(i)};
      }
      reason = checkDriveTemp(drive_temp_[i], L);
      if (reason != BreachReason::NONE) {
        return {reason, static_cast<int>(i)};
      }
      reason = checkBusVoltage(bus_voltage_[i], L);
      if (reason != BreachReason::NONE) {
        return {reason, static_cast<int>(i)};
      }
    }
  }
  return {BreachReason::NONE, -1};
}

EstopAction SafetySupervisor::actionFor(BreachReason reason) const
{
  switch (reason) {
    case BreachReason::MOTOR_OVERTEMP:
    case BreachReason::DRIVE_OVERTEMP:
    case BreachReason::BUS_VOLTAGE_LOW:
    case BreachReason::BUS_VOLTAGE_HIGH:
      return action_temperature_;
    case BreachReason::OVERSPEED:
      return action_overspeed_;
    case BreachReason::POSITION_LOW:
    case BreachReason::POSITION_HIGH:
      return action_position_;
    case BreachReason::SUSTAINED_EFFORT:
      return action_sustained_;
    case BreachReason::STALE_JOINT_STATE:
      return action_stale_;
    case BreachReason::MANUAL:
      return action_manual_;
    case BreachReason::STARTUP:
      return EstopAction::FREE;
    case BreachReason::NONE:
      return EstopAction::FREE;
  }
  return EstopAction::FREE;
}

void SafetySupervisor::latch(BreachReason reason, int joint_idx)
{
  estop_latched_ = true;
  latched_reason_ = reason;
  latched_action_ = actionFor(reason);
  latched_joint_idx_ = joint_idx;
  const std::string joint = (joint_idx >= 0 &&
    static_cast<std::size_t>(joint_idx) < num_joints_)
    ? joint_names_[joint_idx] : std::string("<global>");
  RCLCPP_ERROR(
    get_logger(), "E-STOP latched: %s on %s (action: %s)",
    to_string(reason), joint.c_str(), to_string(latched_action_));
  publishEstopState();
}

void SafetySupervisor::publishEstopState()
{
  std_msgs::msg::Int8 state;
  int8_t code = 0;
  if (estop_latched_) {
    switch (latched_action_) {
      case EstopAction::HOLD: code = 2; break;
      case EstopAction::DAMP: code = 3; break;
      case EstopAction::FREE: code = 1; break;
    }
  }
  state.data = code;
  estop_pub_->publish(state);

  std_msgs::msg::String breach;
  if (estop_latched_) {
    std::ostringstream os;
    if (latched_joint_idx_ >= 0 &&
      static_cast<std::size_t>(latched_joint_idx_) < num_joints_)
    {
      os << joint_names_[latched_joint_idx_] << ": ";
    }
    os << to_string(latched_reason_);
    breach.data = os.str();
  } else {
    breach.data = to_string(BreachReason::NONE);
  }
  breach_pub_->publish(breach);
}

void SafetySupervisor::publishKpScale(const std::vector<double> & kp_scale)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data = kp_scale;
  kp_scale_pub_->publish(msg);
}

void SafetySupervisor::onWatchdog()
{
  const double dt = 1.0 / watchdog_rate_hz_;

  if (!detection_armed_) {
    const bool fresh = joint_state_received_ &&
      (now() - last_joint_state_).seconds() <= limits_.global.joint_state_timeout_sec;
    if (fresh) {
      if (!warmup_started_) {
        warmup_start_ = now();
        warmup_started_ = true;
      } else if ((now() - warmup_start_).seconds() >= startup_grace_sec_) {
        detection_armed_ = true;
        RCLCPP_INFO(get_logger(), "safety detection armed — joint feed healthy");
      }
    } else {
      warmup_started_ = false;
    }
    for (auto & m : effort_monitors_) {
      m.reset();
    }
  }

  if (detection_armed_ && !estop_latched_) {
    // Per-joint sustained-effort monitor — first joint to trip wins.
    int sustained_idx = -1;
    for (std::size_t i = 0; i < num_joints_; ++i) {
      const SafetyLimits & L = limits_.limitsFor(joint_names_[i]);
      const bool tripped = effort_monitors_[i].update(
        std::abs(effort_[i]), dt, L.sustained_effort_threshold,
        L.sustained_effort_window_sec);
      if (tripped && sustained_idx < 0) {
        sustained_idx = static_cast<int>(i);
      }
    }
    BreachHit hit = detectBreach();
    if (hit.reason == BreachReason::NONE && sustained_idx >= 0) {
      hit = {BreachReason::SUSTAINED_EFFORT, sustained_idx};
    }
    if (hit.reason != BreachReason::NONE) {
      latch(hit.reason, hit.joint_idx);
    }
  }

  // Per-joint thermal Kp derating runs continuously, independent of the latch.
  std::vector<double> kp_scale(num_joints_, 1.0);
  if (monitor_temperature_) {
    for (std::size_t i = 0; i < num_joints_; ++i) {
      const SafetyLimits & L = limits_.limitsFor(joint_names_[i]);
      kp_scale[i] = kpScaleCombined(motor_temp_[i], drive_temp_[i], L);
    }
  }

  // Decimate kp_scale + diagnostics to ~10 Hz; breach detection runs every cycle.
  if (++cycle_count_ >= report_period_cycles_) {
    cycle_count_ = 0;
    publishKpScale(kp_scale);
    publishDiagnostics(kp_scale);
  }
}

void SafetySupervisor::onEstopRequest(
  std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;
  if (!estop_latched_) {
    latch(BreachReason::MANUAL, -1);
  }
  response->success = true;
  response->message = "e-stop latched (manual)";
}

void SafetySupervisor::onResetRequest(
  std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;
  if (!detection_armed_) {
    response->success = false;
    response->message =
      "reset refused — safety detection not yet armed (waiting for a "
      "healthy joint feed)";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }
  if (!estop_latched_) {
    response->success = true;
    response->message = "e-stop already clear";
    return;
  }
  const BreachHit active = detectBreach();
  if (active.reason != BreachReason::NONE) {
    const std::string joint = (active.joint_idx >= 0 &&
      static_cast<std::size_t>(active.joint_idx) < num_joints_)
      ? joint_names_[active.joint_idx] : std::string("<global>");
    response->success = false;
    response->message =
      std::string("reset refused — active breach: ") +
      to_string(active.reason) + " on " + joint;
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }
  estop_latched_ = false;
  latched_reason_ = BreachReason::NONE;
  latched_action_ = EstopAction::FREE;
  latched_joint_idx_ = -1;
  for (auto & m : effort_monitors_) {
    m.reset();
  }
  publishEstopState();
  RCLCPP_INFO(get_logger(), "e-stop reset — supervisor clear");
  response->success = true;
  response->message = "e-stop cleared";
}

void SafetySupervisor::publishDiagnostics(const std::vector<double> & kp_scale)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();

  // Top-level summary status.
  diagnostic_msgs::msg::DiagnosticStatus summary;
  summary.name = "robot_safety: supervisor";
  summary.hardware_id = std::to_string(num_joints_) + " joints";
  if (estop_latched_) {
    summary.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    const std::string joint = (latched_joint_idx_ >= 0 &&
      static_cast<std::size_t>(latched_joint_idx_) < num_joints_)
      ? joint_names_[latched_joint_idx_] : std::string("<global>");
    summary.message = std::string("E-STOP latched: ") +
      to_string(latched_reason_) + " on " + joint +
      " (action " + to_string(latched_action_) + ")";
  } else {
    summary.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    summary.message = detection_armed_
      ? "OK"
      : "warming up — breach detection not yet armed";
  }
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "detection_armed";
    kv.value = detection_armed_ ? "true" : "false";
    summary.values.push_back(kv);
  }
  array.status.push_back(summary);

  // Per-joint detail.
  for (std::size_t i = 0; i < num_joints_; ++i) {
    diagnostic_msgs::msg::DiagnosticStatus js;
    js.name = "robot_safety: " + joint_names_[i];
    js.hardware_id = joint_names_[i];
    js.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    js.message = "ok";

    auto kv = [&js](const std::string & k, const std::string & v) {
        diagnostic_msgs::msg::KeyValue pair;
        pair.key = k;
        pair.value = v;
        js.values.push_back(pair);
      };
    kv("position", std::to_string(position_[i]));
    kv("velocity", std::to_string(velocity_[i]));
    kv("effort", std::to_string(effort_[i]));
    if (monitor_temperature_) {
      kv("motor_temp", std::to_string(motor_temp_[i]));
      kv("drive_temp", std::to_string(drive_temp_[i]));
      kv("bus_voltage", std::to_string(bus_voltage_[i]));
    }
    kv("kp_scale", std::to_string(kp_scale[i]));
    kv("effort_accum_sec",
      std::to_string(effort_monitors_[i].timeAboveThreshold()));
    array.status.push_back(js);
  }

  diag_pub_->publish(array);
}

}  // namespace robot_safety
