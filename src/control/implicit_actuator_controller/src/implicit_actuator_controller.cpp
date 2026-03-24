#include "implicit_actuator_controller/implicit_actuator_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <algorithm>
#include <cmath>

namespace implicit_actuator_controller
{

controller_interface::CallbackReturn ImplicitActuatorController::on_init()
{
  try {
    joint_names_ = auto_declare<std::vector<std::string>>("joints", {});
    if (joint_names_.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "No joints specified!");
      return controller_interface::CallbackReturn::ERROR;
    }
    num_joints_ = joint_names_.size();

    kp_ = auto_declare<std::vector<double>>("kp", std::vector<double>(num_joints_, 100.0));
    ki_ = auto_declare<std::vector<double>>("ki", std::vector<double>(num_joints_, 0.0));
    kd_ = auto_declare<std::vector<double>>("kd", std::vector<double>(num_joints_, 10.0));
    i_clamp_ = auto_declare<std::vector<double>>("i_clamp", std::vector<double>(num_joints_, 30.0));
    effort_limits_ = auto_declare<std::vector<double>>(
      "effort_limits", std::vector<double>(num_joints_, 50.0));
    default_positions_ = auto_declare<std::vector<double>>(
      "default_positions", std::vector<double>(num_joints_, 0.0));

    max_pos_rate_ = auto_declare<double>("max_position_rate", 2.0);

    target_positions_.resize(num_joints_, 0.0);
    commanded_positions_.resize(num_joints_, 0.0);
    integral_error_.resize(num_joints_, 0.0);

    return controller_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init exception: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
}

controller_interface::InterfaceConfiguration
ImplicitActuatorController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & name : joint_names_) {
    config.names.push_back(name + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration
ImplicitActuatorController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & name : joint_names_) {
    config.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
  }
  for (const auto & name : joint_names_) {
    config.names.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

controller_interface::CallbackReturn ImplicitActuatorController::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (kp_.size() != num_joints_ || kd_.size() != num_joints_ ||
      ki_.size() != num_joints_ || i_clamp_.size() != num_joints_ ||
      effort_limits_.size() != num_joints_ || default_positions_.size() != num_joints_)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Parameter size mismatch: joints=%zu, kp=%zu, ki=%zu, kd=%zu",
      num_joints_, kp_.size(), ki_.size(), kd_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  target_positions_ = default_positions_;
  rt_target_buffer_.writeFromNonRT(default_positions_);

  target_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/target_joint_positions", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      if (msg->data.size() == num_joints_) {
        std::vector<double> targets(msg->data.begin(), msg->data.end());
        rt_target_buffer_.writeFromNonRT(targets);
      } else {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "Target size %zu != expected %zu, ignoring", msg->data.size(), num_joints_);
      }
    });

  RCLCPP_INFO(get_node()->get_logger(),
    "ImplicitActuatorController configured: %zu joints, subscribing to ~/target_joint_positions",
    num_joints_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImplicitActuatorController::on_activate(
  const rclcpp_lifecycle::State &)
{
  pos_iface_idx_.resize(num_joints_);
  vel_iface_idx_.resize(num_joints_);
  cmd_iface_idx_.resize(num_joints_);

  for (size_t j = 0; j < num_joints_; ++j) {
    const auto & jname = joint_names_[j];
    bool found_pos = false, found_vel = false, found_cmd = false;

    for (size_t k = 0; k < state_interfaces_.size(); ++k) {
      const auto & si = state_interfaces_[k];
      if (si.get_prefix_name() == jname) {
        if (si.get_interface_name() == "position") { pos_iface_idx_[j] = k; found_pos = true; }
        if (si.get_interface_name() == "velocity") { vel_iface_idx_[j] = k; found_vel = true; }
      }
    }
    for (size_t k = 0; k < command_interfaces_.size(); ++k) {
      if (command_interfaces_[k].get_prefix_name() == jname) {
        cmd_iface_idx_[j] = k; found_cmd = true;
      }
    }

    if (!found_pos || !found_vel || !found_cmd) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "Failed to find interfaces for joint '%s' (pos=%d, vel=%d, cmd=%d)",
        jname.c_str(), found_pos, found_vel, found_cmd);
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  std::fill(integral_error_.begin(), integral_error_.end(), 0.0);

  std::vector<double> current_positions(num_joints_);
  for (size_t j = 0; j < num_joints_; ++j) {
    auto opt = state_interfaces_[pos_iface_idx_[j]].get_optional();
    current_positions[j] = opt.has_value() ? opt.value() : 0.0;
  }
  target_positions_ = current_positions;
  commanded_positions_ = current_positions;
  rt_target_buffer_.writeFromNonRT(current_positions);

  RCLCPP_INFO(get_node()->get_logger(),
    "ImplicitActuatorController activated — %zu joints, PID at controller rate. "
    "Holding current positions. Publish to ~/target_joint_positions to command.", num_joints_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImplicitActuatorController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  for (size_t i = 0; i < num_joints_; ++i) {
    (void)command_interfaces_[cmd_iface_idx_[i]].set_value(0.0);
  }
  RCLCPP_INFO(get_node()->get_logger(), "ImplicitActuatorController deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ImplicitActuatorController::update(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  const auto & cmd = *rt_target_buffer_.readFromRT();
  for (size_t i = 0; i < num_joints_; ++i) {
    commanded_positions_[i] = cmd[i];
  }

  const double dt = period.seconds();

  // Smooth target: move toward commanded at max_pos_rate_ rad/s
  const double max_step = max_pos_rate_ * dt;
  for (size_t i = 0; i < num_joints_; ++i) {
    double diff = commanded_positions_[i] - target_positions_[i];
    // Reset integrator when target is still far from commanded (prevents windup carryover)
    if (std::abs(diff) > 0.01) {
      integral_error_[i] = 0.0;
    }
    diff = std::clamp(diff, -max_step, max_step);
    target_positions_[i] += diff;
  }

  // PID loop
  for (size_t i = 0; i < num_joints_; ++i) {
    auto pos_opt = state_interfaces_[pos_iface_idx_[i]].get_optional();
    auto vel_opt = state_interfaces_[vel_iface_idx_[i]].get_optional();
    double pos = pos_opt.has_value() ? pos_opt.value() : 0.0;
    double vel = vel_opt.has_value() ? vel_opt.value() : 0.0;

    double error = target_positions_[i] - pos;

    if (ki_[i] > 0.0) {
      integral_error_[i] += error * dt;
      integral_error_[i] = std::clamp(integral_error_[i], -i_clamp_[i], i_clamp_[i]);
    }

    double torque = kp_[i] * error + ki_[i] * integral_error_[i] - kd_[i] * vel;
    torque = std::clamp(torque, -effort_limits_[i], effort_limits_[i]);

    (void)command_interfaces_[cmd_iface_idx_[i]].set_value(torque);
  }

  // Debug log every 1 second
  static size_t tick_count = 0;
  if (tick_count % 200 == 0) {
    size_t j_ankle = 17, j_hip = 13;
    auto a_pos = state_interfaces_[pos_iface_idx_[j_ankle]].get_optional().value_or(0.0);
    auto a_vel = state_interfaces_[vel_iface_idx_[j_ankle]].get_optional().value_or(0.0);
    double a_err = target_positions_[j_ankle] - a_pos;
    double a_trq = kp_[j_ankle] * a_err + ki_[j_ankle] * integral_error_[j_ankle] - kd_[j_ankle] * a_vel;

    auto h_pos = state_interfaces_[pos_iface_idx_[j_hip]].get_optional().value_or(0.0);
    auto h_vel = state_interfaces_[vel_iface_idx_[j_hip]].get_optional().value_or(0.0);
    double h_trq = kp_[j_hip] * (target_positions_[j_hip] - h_pos) - kd_[j_hip] * h_vel;

    RCLCPP_INFO(get_node()->get_logger(),
      "[tick %zu] ankle: pos=%.3f tgt=%.3f err=%.3f i=%.3f trq=%.1f | "
      "hip: pos=%.3f tgt=%.3f trq=%.1f",
      tick_count, a_pos, target_positions_[j_ankle],
      a_err, integral_error_[j_ankle], a_trq,
      h_pos, target_positions_[j_hip], h_trq);
  }
  tick_count++;

  return controller_interface::return_type::OK;
}

}  // namespace implicit_actuator_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  implicit_actuator_controller::ImplicitActuatorController,
  controller_interface::ControllerInterface)
