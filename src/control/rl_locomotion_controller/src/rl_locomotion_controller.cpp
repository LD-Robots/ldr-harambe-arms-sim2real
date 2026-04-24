#include "rl_locomotion_controller/rl_locomotion_controller.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace rl_locomotion_controller
{

// No quaternion rotation needed: BNO055 driver publishes the gravity vector
// directly in the sensor body frame on /<name>/grav. We just normalize it to
// match the Isaac convention of a unit vector.

namespace
{
// Resolve a `package://<pkg>/<path>` URI to the installed file path.
// Returns the original string unchanged if it is not a package:// URI.
std::string resolve_package_uri(const std::string & uri)
{
  const std::string prefix = "package://";
  if (uri.rfind(prefix, 0) != 0) {
    return uri;
  }
  const std::string rest = uri.substr(prefix.size());
  const auto slash = rest.find('/');
  if (slash == std::string::npos) {
    throw std::runtime_error("Malformed package URI: " + uri);
  }
  const std::string pkg = rest.substr(0, slash);
  const std::string rel = rest.substr(slash + 1);
  return ament_index_cpp::get_package_share_directory(pkg) + "/" + rel;
}
}  // namespace

// ============================================================================
// on_init
// ============================================================================

controller_interface::CallbackReturn RLLocomotionController::on_init()
{
  try {
    ctrl_joints_ = auto_declare<std::vector<std::string>>("joints", {});
    policy_joints_ = auto_declare<std::vector<std::string>>("policy_joint_order", {});
    policy_path_ = auto_declare<std::string>("policy_path", "");

    action_scale_ = auto_declare<double>("action_scale", 0.25);
    decimation_ = auto_declare<int>("decimation", 2);
    ramp_duration_s_ = auto_declare<double>("ramp_duration_s", 2.0);
    tilt_abort_cos_ = auto_declare<double>("tilt_abort_cos", -0.3);
    max_target_rate_ = auto_declare<double>("max_target_rate", 0.10);

    auto isaac_def = auto_declare<std::vector<double>>("isaac_default", {});

    auto_declare<double>("cmd_vel_x", 0.0);
    auto_declare<double>("cmd_vel_y", 0.0);
    auto_declare<double>("cmd_vel_yaw", 0.0);
    auto_declare<std::string>("pelvis_imu_topic", "/pelvis/imu");
    auto_declare<std::string>("torso_imu_topic", "/torso/imu");
    auto_declare<std::string>("pelvis_grav_topic", "/pelvis/grav");
    auto_declare<std::string>("torso_grav_topic", "/torso/grav");
    auto_declare<std::string>("cmd_vel_topic", "/cmd_vel");
    auto_declare<std::string>("estop_topic", "/emergency_stop");

    if (ctrl_joints_.size() != kNumJoints) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "joints must have %zu entries, got %zu", kNumJoints, ctrl_joints_.size());
      return controller_interface::CallbackReturn::ERROR;
    }
    if (policy_joints_.size() != kNumJoints) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "policy_joint_order must have %zu entries, got %zu",
        kNumJoints, policy_joints_.size());
      return controller_interface::CallbackReturn::ERROR;
    }
    if (isaac_def.size() != kNumJoints) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "isaac_default must have %zu entries, got %zu",
        kNumJoints, isaac_def.size());
      return controller_interface::CallbackReturn::ERROR;
    }
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      isaac_default_[i] = isaac_def[i];
    }

    // Build policy -> ctrl index map
    for (std::size_t p = 0; p < kNumJoints; ++p) {
      auto it = std::find(ctrl_joints_.begin(), ctrl_joints_.end(), policy_joints_[p]);
      if (it == ctrl_joints_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(),
          "policy joint '%s' not found in controller joints",
          policy_joints_[p].c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      policy_to_ctrl_[p] = static_cast<std::size_t>(std::distance(ctrl_joints_.begin(), it));
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init exception: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// on_configure
// ============================================================================

controller_interface::CallbackReturn RLLocomotionController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Load ONNX
  policy_path_ = get_node()->get_parameter("policy_path").as_string();
  if (policy_path_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "policy_path parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  try {
    policy_path_ = resolve_package_uri(policy_path_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to resolve policy_path: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  try {
    ort_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "rl_loco_ctrl");
    Ort::SessionOptions opts;
    opts.SetIntraOpNumThreads(1);
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    ort_session_ = std::make_unique<Ort::Session>(*ort_env_, policy_path_.c_str(), opts);
    ort_mem_info_ = std::make_unique<Ort::MemoryInfo>(
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));

    Ort::AllocatorWithDefaultOptions alloc;
    onnx_input_name_ = ort_session_->GetInputNameAllocated(0, alloc).get();
    onnx_output_name_ = ort_session_->GetOutputNameAllocated(0, alloc).get();
    auto inp_info = ort_session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo();
    auto shape = inp_info.GetShape();
    if (shape.size() < 2) {
      RCLCPP_ERROR(get_node()->get_logger(), "ONNX input has <2 dims");
      return controller_interface::CallbackReturn::ERROR;
    }
    int64_t model_obs = shape.back();
    if (model_obs == static_cast<int64_t>(kObsDim93)) {
      obs_dim_ = kObsDim93;
    } else if (model_obs == static_cast<int64_t>(kObsDim87)) {
      obs_dim_ = kObsDim87;
    } else {
      RCLCPP_ERROR(get_node()->get_logger(),
        "Unsupported ONNX input dim %ld (expected 87 or 93)", model_obs);
      return controller_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_node()->get_logger(),
      "Loaded ONNX: %s (obs_dim=%zu, torso_imu=%s)",
      policy_path_.c_str(), obs_dim_, obs_dim_ == kObsDim93 ? "yes" : "no");
  } catch (const Ort::Exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load ONNX: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // ROS subscriptions (best-effort QoS for IMU to avoid blocking RT loop)
  const auto pelvis_imu_topic  = get_node()->get_parameter("pelvis_imu_topic").as_string();
  const auto torso_imu_topic   = get_node()->get_parameter("torso_imu_topic").as_string();
  const auto pelvis_grav_topic = get_node()->get_parameter("pelvis_grav_topic").as_string();
  const auto torso_grav_topic  = get_node()->get_parameter("torso_grav_topic").as_string();
  const auto cmd_topic         = get_node()->get_parameter("cmd_vel_topic").as_string();
  const auto estop_topic       = get_node()->get_parameter("estop_topic").as_string();

  auto imu_qos = rclcpp::SensorDataQoS();

  // Pelvis: IMU (angular velocity) + gravity (body-frame gravity vector)
  pelvis_imu_sub_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
    pelvis_imu_topic, imu_qos,
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
      auto cur = *pelvis_imu_buf_.readFromNonRT();
      cur.ang_vel = ang_vel_from_imu(*msg);
      cur.valid = true;
      pelvis_imu_buf_.writeFromNonRT(cur);
    });
  pelvis_grav_sub_ = get_node()->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    pelvis_grav_topic, imu_qos,
    [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
      auto cur = *pelvis_imu_buf_.readFromNonRT();
      cur.proj_grav = proj_grav_from_vec(*msg);
      cur.valid = true;
      pelvis_imu_buf_.writeFromNonRT(cur);
    });

  if (obs_dim_ == kObsDim93) {
    torso_imu_sub_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
      torso_imu_topic, imu_qos,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        auto cur = *torso_imu_buf_.readFromNonRT();
        cur.ang_vel = ang_vel_from_imu(*msg);
        cur.valid = true;
        torso_imu_buf_.writeFromNonRT(cur);
      });
    torso_grav_sub_ = get_node()->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      torso_grav_topic, imu_qos,
      [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        auto cur = *torso_imu_buf_.readFromNonRT();
        cur.proj_grav = proj_grav_from_vec(*msg);
        cur.valid = true;
        torso_imu_buf_.writeFromNonRT(cur);
      });
  }

  CmdVel default_cmd{
    get_node()->get_parameter("cmd_vel_x").as_double(),
    get_node()->get_parameter("cmd_vel_y").as_double(),
    get_node()->get_parameter("cmd_vel_yaw").as_double(),
  };
  cmd_vel_buf_.writeFromNonRT(default_cmd);

  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    cmd_topic, 10,
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      CmdVel c{msg->linear.x, msg->linear.y, msg->angular.z};
      cmd_vel_buf_.writeFromNonRT(c);
    });

  estop_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
    estop_topic, 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      emergency_stop_.store(msg->data);
    });

  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// on_activate / on_deactivate
// ============================================================================

controller_interface::CallbackReturn RLLocomotionController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!read_current_pose_ctrl(init_pose_)) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "on_activate: state interfaces not readable — aborting");
    return controller_interface::CallbackReturn::ERROR;
  }
  last_target_ctrl_ = init_pose_;
  frozen_target_ctrl_ = init_pose_;
  last_action_pol_.fill(0.0);

  state_ = State::INIT;
  ramp_t_ = 0.0;
  update_counter_ = 0;
  decim_counter_ = 0;
  emergency_stop_.store(false);

  RCLCPP_INFO(get_node()->get_logger(),
    "RLLocomotionController activated. ramp=%.1fs, action_scale=%.3f, decim=%d",
    ramp_duration_s_, action_scale_, decimation_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RLLocomotionController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  state_ = State::INIT;
  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// Interface configuration
// ============================================================================

controller_interface::InterfaceConfiguration
RLLocomotionController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.reserve(kNumJoints);
  for (const auto & j : ctrl_joints_) {
    cfg.names.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
RLLocomotionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.reserve(kNumJoints * 2);
  for (const auto & j : ctrl_joints_) {
    cfg.names.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
    cfg.names.push_back(j + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return cfg;
}

// ============================================================================
// update (called at controller_manager rate, e.g. 100 Hz)
// ============================================================================

controller_interface::return_type RLLocomotionController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  ++update_counter_;

  // Current measured pose (controller order)
  std::array<double, kNumJoints> cur_pose_ctrl;
  if (!read_current_pose_ctrl(cur_pose_ctrl)) {
    return controller_interface::return_type::ERROR;
  }

  // Safety: emergency stop or tilt abort
  const auto pelvis = *pelvis_imu_buf_.readFromRT();
  const bool tilt_bad = pelvis.valid && pelvis.proj_grav[2] > tilt_abort_cos_;
  if (emergency_stop_.load() || tilt_bad) {
    if (state_ != State::FROZEN) {
      RCLCPP_WARN(get_node()->get_logger(),
        "Freezing targets — estop=%d tilt_bad=%d (gz=%.2f)",
        static_cast<int>(emergency_stop_.load()), static_cast<int>(tilt_bad),
        pelvis.proj_grav[2]);
      frozen_target_ctrl_ = last_target_ctrl_;
      state_ = State::FROZEN;
    }
    write_targets_ctrl(frozen_target_ctrl_);
    return controller_interface::return_type::OK;
  }

  // State machine
  switch (state_) {
    case State::INIT: {
      // Wait for first pelvis IMU reading before starting ramp
      if (pelvis.valid) {
        state_ = State::RAMP;
        ramp_t_ = 0.0;
        RCLCPP_INFO(get_node()->get_logger(), "IMU ready — entering RAMP");
      }
      // Hold the pose we captured at activate
      write_targets_ctrl(init_pose_);
      break;
    }

    case State::RAMP: {
      ramp_t_ += dt;
      const double alpha = std::min(1.0, ramp_t_ / std::max(1e-3, ramp_duration_s_));
      std::array<double, kNumJoints> tgt_ctrl;
      for (std::size_t p = 0; p < kNumJoints; ++p) {
        const std::size_t c = policy_to_ctrl_[p];
        tgt_ctrl[c] = init_pose_[c] * (1.0 - alpha) + isaac_default_[p] * alpha;
      }
      last_target_ctrl_ = tgt_ctrl;
      write_targets_ctrl(tgt_ctrl);
      if (alpha >= 1.0) {
        state_ = State::POLICY;
        decim_counter_ = 0;
        RCLCPP_INFO(get_node()->get_logger(), "Ramp complete — running policy");
      }
      break;
    }

    case State::POLICY: {
      // Run ONNX every `decimation_` updates
      if (decim_counter_ == 0) {
        step_policy();
      }
      decim_counter_ = (decim_counter_ + 1) % static_cast<std::size_t>(std::max(1, decimation_));
      write_targets_ctrl(last_target_ctrl_);
      break;
    }

    case State::FROZEN: {
      write_targets_ctrl(frozen_target_ctrl_);
      break;
    }
  }
  return controller_interface::return_type::OK;
}

// ============================================================================
// Helpers
// ============================================================================

std::array<double, 3>
RLLocomotionController::ang_vel_from_imu(const sensor_msgs::msg::Imu & msg)
{
  return {{msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z}};
}

std::array<double, 3>
RLLocomotionController::proj_grav_from_vec(
  const geometry_msgs::msg::Vector3Stamped & msg)
{
  // Normalize so ‖g‖ = 1 (Isaac convention). BNO055 typically publishes ~9.81 m/s².
  const double x = msg.vector.x, y = msg.vector.y, z = msg.vector.z;
  const double norm = std::sqrt(x * x + y * y + z * z);
  if (norm < 1e-6) {
    return {{0.0, 0.0, -1.0}};  // fallback on degenerate input
  }
  return {{x / norm, y / norm, z / norm}};
}

bool RLLocomotionController::read_current_pose_ctrl(
  std::array<double, kNumJoints> & out) const
{
  // state_interfaces_ layout: [j0/pos, j0/vel, j1/pos, j1/vel, ...] in ctrl_joints_ order
  if (state_interfaces_.size() != kNumJoints * 2) {
    return false;
  }
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    const double v = state_interfaces_[i * 2].get_value();
    if (std::isnan(v)) {
      return false;
    }
    out[i] = v;
  }
  return true;
}

void RLLocomotionController::write_targets_ctrl(
  const std::array<double, kNumJoints> & targets_ctrl)
{
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    command_interfaces_[i].set_value(targets_ctrl[i]);
  }
}

void RLLocomotionController::build_obs(std::vector<float> & out) const
{
  out.assign(obs_dim_, 0.0f);
  const auto pelvis = *pelvis_imu_buf_.readFromRT();
  const auto cmd    = *cmd_vel_buf_.readFromRT();

  // lin_vel (3): unknown on real hardware without odometry — zeros (matches training,
  // which already uses noisy/low-variance base_lin_vel)
  // indices 0..2 already 0

  // ang_vel (3)
  out[3] = static_cast<float>(pelvis.ang_vel[0]);
  out[4] = static_cast<float>(pelvis.ang_vel[1]);
  out[5] = static_cast<float>(pelvis.ang_vel[2]);

  // proj_grav (3)
  out[6] = static_cast<float>(pelvis.proj_grav[0]);
  out[7] = static_cast<float>(pelvis.proj_grav[1]);
  out[8] = static_cast<float>(pelvis.proj_grav[2]);

  // cmd (3)
  out[9]  = static_cast<float>(cmd.x);
  out[10] = static_cast<float>(cmd.y);
  out[11] = static_cast<float>(cmd.yaw);

  // joint_pos_rel (25) in policy order: current - isaac_default
  // and joint_vel (25) in policy order
  std::size_t idx_pos = 12;
  std::size_t idx_vel = 12 + kNumJoints;
  for (std::size_t p = 0; p < kNumJoints; ++p) {
    const std::size_t c = policy_to_ctrl_[p];
    const double pos_raw = state_interfaces_[c * 2].get_value();      // pos
    const double vel_raw = state_interfaces_[c * 2 + 1].get_value();  // vel
    const double pos = std::isnan(pos_raw) ? 0.0 : pos_raw;
    const double vel = std::isnan(vel_raw) ? 0.0 : vel_raw;
    out[idx_pos + p] = static_cast<float>(pos - isaac_default_[p]);
    out[idx_vel + p] = static_cast<float>(vel);
  }

  // last_action (25) in policy order
  std::size_t idx_act = 12 + kNumJoints * 2;
  for (std::size_t p = 0; p < kNumJoints; ++p) {
    out[idx_act + p] = static_cast<float>(last_action_pol_[p]);
  }

  // V21+ extras: torso proj_grav (3) + torso ang_vel (3)
  if (obs_dim_ == kObsDim93) {
    const auto torso = *torso_imu_buf_.readFromRT();
    std::size_t idx_torso_g = 12 + kNumJoints * 3;
    out[idx_torso_g + 0] = static_cast<float>(torso.proj_grav[0]);
    out[idx_torso_g + 1] = static_cast<float>(torso.proj_grav[1]);
    out[idx_torso_g + 2] = static_cast<float>(torso.proj_grav[2]);
    out[idx_torso_g + 3] = static_cast<float>(torso.ang_vel[0]);
    out[idx_torso_g + 4] = static_cast<float>(torso.ang_vel[1]);
    out[idx_torso_g + 5] = static_cast<float>(torso.ang_vel[2]);
  }

  // Clip to [-100, 100] like the Python deploy
  for (auto & v : out) {
    if (v > 100.0f) v = 100.0f;
    if (v < -100.0f) v = -100.0f;
  }
}

void RLLocomotionController::run_policy(std::array<double, kNumJoints> & action_out)
{
  thread_local std::vector<float> obs;
  build_obs(obs);

  const std::array<int64_t, 2> shape{{1, static_cast<int64_t>(obs_dim_)}};
  Ort::Value in_tensor = Ort::Value::CreateTensor<float>(
    *ort_mem_info_, obs.data(), obs.size(), shape.data(), shape.size());

  const char * input_names[] = {onnx_input_name_.c_str()};
  const char * output_names[] = {onnx_output_name_.c_str()};

  auto outputs = ort_session_->Run(
    Ort::RunOptions{nullptr},
    input_names, &in_tensor, 1,
    output_names, 1);

  if (outputs.empty()) {
    action_out.fill(0.0);
    return;
  }
  const float * data = outputs.front().GetTensorData<float>();
  for (std::size_t p = 0; p < kNumJoints; ++p) {
    action_out[p] = static_cast<double>(data[p]);
  }
}

void RLLocomotionController::step_policy()
{
  std::array<double, kNumJoints> action_pol;
  run_policy(action_pol);
  last_action_pol_ = action_pol;

  // Compute target in controller order: isaac_default + action * scale
  std::array<double, kNumJoints> tgt_ctrl = last_target_ctrl_;
  for (std::size_t p = 0; p < kNumJoints; ++p) {
    const std::size_t c = policy_to_ctrl_[p];
    const double desired = isaac_default_[p] + action_pol[p] * action_scale_;
    // Rate-limit vs previous target (per policy step)
    const double prev = last_target_ctrl_[c];
    double delta = desired - prev;
    if (delta > max_target_rate_) delta = max_target_rate_;
    if (delta < -max_target_rate_) delta = -max_target_rate_;
    tgt_ctrl[c] = prev + delta;
  }
  last_target_ctrl_ = tgt_ctrl;
}

}  // namespace rl_locomotion_controller

PLUGINLIB_EXPORT_CLASS(
  rl_locomotion_controller::RLLocomotionController,
  controller_interface::ControllerInterface)
