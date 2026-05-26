#include "robot_filtered_joint_state_broadcaster/filtered_joint_state_broadcaster.hpp"

#include <algorithm>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"

namespace robot_filtered_joint_state_broadcaster
{

namespace
{
constexpr double kTwoPi = 6.283185307179586;

// First-order IIR low-pass smoothing factor for a given cutoff and dt.
// cutoff_hz <= 0 disables filtering (alpha = 1 → passthrough).
double lpf_alpha(double cutoff_hz, double dt)
{
  if (cutoff_hz <= 0.0 || dt <= 0.0) {
    return 1.0;
  }
  const double alpha = 1.0 - std::exp(-kTwoPi * cutoff_hz * dt);
  return std::clamp(alpha, 0.0, 1.0);
}
}  // namespace

controller_interface::CallbackReturn FilteredJointStateBroadcaster::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
    auto_declare<double>("velocity_cutoff_hz", 30.0);
    auto_declare<double>("torque_cutoff_hz", 30.0);
    auto_declare<double>("publish_rate", 100.0);
    auto_declare<std::string>("topic", "/joint_states");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FilteredJointStateBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  joints_ = node->get_parameter("joints").as_string_array();
  topic_ = node->get_parameter("topic").as_string();
  velocity_cutoff_hz_ = node->get_parameter("velocity_cutoff_hz").as_double();
  torque_cutoff_hz_ = node->get_parameter("torque_cutoff_hz").as_double();
  publish_rate_ = node->get_parameter("publish_rate").as_double();

  if (joints_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'joints' is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (topic_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'topic' is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (publish_rate_ <= 0.0) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'publish_rate' must be > 0.");
    return controller_interface::CallbackReturn::ERROR;
  }
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

  vel_filt_.assign(joints_.size(), 0.0);
  tau_filt_.assign(joints_.size(), 0.0);

  joint_state_pub_ = node->create_publisher<sensor_msgs::msg::JointState>(
    topic_, rclcpp::SystemDefaultsQoS());
  rt_joint_state_pub_ = std::make_unique<JointStatePublisher>(joint_state_pub_);

  // Pre-size the message so update() never allocates on the RT path.
  auto & msg = rt_joint_state_pub_->msg_;
  msg.name = joints_;
  msg.position.assign(joints_.size(), 0.0);
  msg.velocity.assign(joints_.size(), 0.0);
  msg.effort.assign(joints_.size(), 0.0);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
FilteredJointStateBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration
FilteredJointStateBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.reserve(joints_.size() * 3);
  // Order: joint0/position, joint0/velocity, joint0/effort,
  //        joint1/position, ... — update() indexes by 3*j + {0,1,2}.
  for (const auto & j : joints_) {
    cfg.names.push_back(j + "/position");
    cfg.names.push_back(j + "/velocity");
    cfg.names.push_back(j + "/effort");
  }
  return cfg;
}

controller_interface::CallbackReturn FilteredJointStateBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  filter_initialized_ = false;
  last_publish_time_ = get_node()->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FilteredJointStateBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type FilteredJointStateBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Live-tunable cutoffs — refreshed each cycle.
  velocity_cutoff_hz_ = get_node()->get_parameter("velocity_cutoff_hz").as_double();
  torque_cutoff_hz_ = get_node()->get_parameter("torque_cutoff_hz").as_double();

  const std::size_t n = joints_.size();
  const double dt = period.seconds();
  const double a_v = lpf_alpha(velocity_cutoff_hz_, dt);
  const double a_t = lpf_alpha(torque_cutoff_hz_, dt);

  // First finite-sample seeding is whole-body: wait until every joint has
  // finite velocity + effort. Until then, no publish.
  bool all_finite = true;
  std::vector<double> pos(n), vel(n), eff(n);
  for (std::size_t j = 0; j < n; ++j) {
    const auto p = state_interfaces_[3 * j + 0].get_optional();
    const auto v = state_interfaces_[3 * j + 1].get_optional();
    const auto e = state_interfaces_[3 * j + 2].get_optional();
    if (!p || !v || !e) {
      return controller_interface::return_type::OK;
    }
    pos[j] = p.value();
    vel[j] = v.value();
    eff[j] = e.value();
    if (!std::isfinite(vel[j]) || !std::isfinite(eff[j])) {
      all_finite = false;
    }
  }
  if (!all_finite) {
    return controller_interface::return_type::OK;
  }

  if (!filter_initialized_) {
    for (std::size_t j = 0; j < n; ++j) {
      vel_filt_[j] = vel[j];
      tau_filt_[j] = eff[j];
    }
    filter_initialized_ = true;
  } else {
    for (std::size_t j = 0; j < n; ++j) {
      vel_filt_[j] += a_v * (vel[j] - vel_filt_[j]);
      tau_filt_[j] += a_t * (eff[j] - tau_filt_[j]);
    }
  }

  // Throttle the publish rate; the filter still updates every cycle.
  if ((time - last_publish_time_) < publish_period_) {
    return controller_interface::return_type::OK;
  }
  last_publish_time_ = time;

  if (rt_joint_state_pub_->trylock()) {
    auto & msg = rt_joint_state_pub_->msg_;
    msg.header.stamp = time;
    for (std::size_t j = 0; j < n; ++j) {
      msg.position[j] = pos[j];
      msg.velocity[j] = vel_filt_[j];
      msg.effort[j]   = tau_filt_[j];
    }
    rt_joint_state_pub_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace robot_filtered_joint_state_broadcaster

PLUGINLIB_EXPORT_CLASS(
  robot_filtered_joint_state_broadcaster::FilteredJointStateBroadcaster,
  controller_interface::ControllerInterface)
