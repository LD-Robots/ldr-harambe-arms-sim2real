#include "robot_drive_status_broadcaster/drive_status_broadcaster.hpp"

#include <algorithm>
#include <cstdio>
#include <limits>

#include "pluginlib/class_list_macros.hpp"

namespace robot_drive_status_broadcaster
{

const char * const DriveStatusBroadcaster::kSignalNames[
  DriveStatusBroadcaster::kNumSignals] = {
  "motor_temperature",
  "drive_temperature",
  "bus_voltage",
  "error_code",
};

controller_interface::CallbackReturn DriveStatusBroadcaster::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
    auto_declare<double>("publish_rate", 50.0);
    auto_declare<double>("motor_temp_warn", 70.0);
    auto_declare<double>("motor_temp_error", 90.0);
    auto_declare<double>("drive_temp_warn", 70.0);
    auto_declare<double>("drive_temp_error", 85.0);
    auto_declare<double>("bus_voltage_min", 40.0);
    auto_declare<double>("bus_voltage_max", 54.0);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DriveStatusBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  joints_ = node->get_parameter("joints").as_string_array();
  if (joints_.empty()) {
    RCLCPP_ERROR(node->get_logger(),
      "Parameter 'joints' is empty — nothing to broadcast.");
    return controller_interface::CallbackReturn::ERROR;
  }
  publish_rate_ = node->get_parameter("publish_rate").as_double();
  if (publish_rate_ <= 0.0) {
    RCLCPP_ERROR(node->get_logger(),
      "Parameter 'publish_rate' must be > 0.");
    return controller_interface::CallbackReturn::ERROR;
  }
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

  thr_.motor_temp_warn  = node->get_parameter("motor_temp_warn").as_double();
  thr_.motor_temp_error = node->get_parameter("motor_temp_error").as_double();
  thr_.drive_temp_warn  = node->get_parameter("drive_temp_warn").as_double();
  thr_.drive_temp_error = node->get_parameter("drive_temp_error").as_double();
  thr_.bus_voltage_min  = node->get_parameter("bus_voltage_min").as_double();
  thr_.bus_voltage_max  = node->get_parameter("bus_voltage_max").as_double();

  // Build N-entry publishers, one per signal. The supervisor expects the
  // joint order to match its own joint_names; this is enforced by sharing
  // a single yaml-driven roster across the launch file.
  signal_pubs_.clear();
  signal_rt_pubs_.clear();
  signal_pubs_.reserve(kNumSignals);
  signal_rt_pubs_.reserve(kNumSignals);
  for (std::size_t s = 0; s < kNumSignals; ++s) {
    auto pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
      std::string("~/") + kSignalNames[s], rclcpp::SystemDefaultsQoS());
    auto rt = std::make_unique<ArrayPublisher>(pub);
    rt->msg_.data.assign(joints_.size(),
      std::numeric_limits<double>::quiet_NaN());
    signal_rt_pubs_.push_back(std::move(rt));
    signal_pubs_.push_back(pub);
  }

  diag_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::SystemDefaultsQoS());
  diag_rt_pub_ = std::make_unique<DiagPublisher>(diag_pub_);
  {
    auto & m = diag_rt_pub_->msg_;
    m.status.resize(1);
    auto & st = m.status[0];
    st.name = "robot_drive_status_broadcaster";
    st.hardware_id = std::to_string(joints_.size()) + " joints";
    st.message.reserve(80);
    st.values.resize(joints_.size() * kNumSignals);
    for (std::size_t j = 0; j < joints_.size(); ++j) {
      for (std::size_t s = 0; s < kNumSignals; ++s) {
        auto & v = st.values[j * kNumSignals + s];
        v.key = joints_[j] + "/" + kSignalNames[s];
        v.value.reserve(24);
      }
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
DriveStatusBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration
DriveStatusBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.reserve(joints_.size() * kNumSignals);
  // Order: joint0/sig0, joint0/sig1, ..., joint1/sig0, ...
  // This matches the index arithmetic in update().
  for (const auto & j : joints_) {
    for (std::size_t s = 0; s < kNumSignals; ++s) {
      cfg.names.push_back(j + "/" + kSignalNames[s]);
    }
  }
  return cfg;
}

controller_interface::CallbackReturn DriveStatusBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  last_publish_time_ = get_node()->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DriveStatusBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type DriveStatusBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if ((time - last_publish_time_) < publish_period_) {
    return controller_interface::return_type::OK;
  }
  last_publish_time_ = time;

  const std::size_t n_joints = joints_.size();

  // Pull every state interface up front into local buffers to keep the
  // signal-publisher fan-out tight.
  std::array<std::vector<double>, kNumSignals> buf;
  for (auto & b : buf) {
    b.assign(n_joints, std::numeric_limits<double>::quiet_NaN());
  }
  for (std::size_t j = 0; j < n_joints; ++j) {
    for (std::size_t s = 0; s < kNumSignals; ++s) {
      const auto v = state_interfaces_[j * kNumSignals + s].get_optional();
      buf[s][j] = v ? v.value() : std::numeric_limits<double>::quiet_NaN();
    }
  }

  // Publish each signal's N-entry array.
  for (std::size_t s = 0; s < kNumSignals; ++s) {
    if (signal_rt_pubs_[s]->trylock()) {
      signal_rt_pubs_[s]->msg_.data = buf[s];
      signal_rt_pubs_[s]->unlockAndPublish();
    }
  }

  // Diagnostics: per-joint signal values + worst-level summary across all
  // joints (any single joint over-temp escalates the whole broadcaster).
  if (!diag_rt_pub_->trylock()) {
    return controller_interface::return_type::OK;
  }
  auto & st = diag_rt_pub_->msg_.status[0];
  uint8_t worst_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  const char * worst_msg = "OK";
  auto escalate = [&worst_level, &worst_msg](uint8_t lvl, const char * msg) {
      if (lvl > worst_level) {
        worst_level = lvl;
        worst_msg = msg;
      }
    };

  char fbuf[32];
  for (std::size_t j = 0; j < n_joints; ++j) {
    for (std::size_t s = 0; s < kNumSignals; ++s) {
      const double v = buf[s][j];
      const int n = std::snprintf(fbuf, sizeof(fbuf), "%g", v);
      auto & kv = st.values[j * kNumSignals + s];
      kv.value.assign(fbuf, n > 0 ? static_cast<std::size_t>(n) : 0);
    }
    const double motor_t = buf[0][j];
    const double drive_t = buf[1][j];
    const double bus_v   = buf[2][j];
    if (motor_t >= thr_.motor_temp_error) {
      escalate(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "Motor over-temperature");
    } else if (motor_t >= thr_.motor_temp_warn) {
      escalate(diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "Motor temperature high");
    }
    if (drive_t >= thr_.drive_temp_error) {
      escalate(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "Drive over-temperature");
    } else if (drive_t >= thr_.drive_temp_warn) {
      escalate(diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "Drive temperature high");
    }
    if (bus_v < thr_.bus_voltage_min || bus_v > thr_.bus_voltage_max) {
      escalate(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "Bus voltage out of range");
    }
  }
  st.level = worst_level;
  st.message.assign(worst_msg);
  diag_rt_pub_->msg_.header.stamp = time;
  diag_rt_pub_->unlockAndPublish();

  return controller_interface::return_type::OK;
}

}  // namespace robot_drive_status_broadcaster

PLUGINLIB_EXPORT_CLASS(
  robot_drive_status_broadcaster::DriveStatusBroadcaster,
  controller_interface::ControllerInterface)
