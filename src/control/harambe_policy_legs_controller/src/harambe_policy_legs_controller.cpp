#include "harambe_policy_legs_controller/harambe_policy_legs_controller.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

#include <algorithm>
#include <cmath>

namespace harambe_policy_legs_controller
{

namespace
{
constexpr size_t kNumLegJoints = 12;
constexpr size_t kObsDim = 52;
constexpr size_t kActionDim = 12;

// Observation layout offsets (see header / deploy/01_observations.md).
constexpr size_t kObsBaseLinVel = 0;     // 3
constexpr size_t kObsBaseAngVel = 3;     // 3  (gyro, body frame)
constexpr size_t kObsProjGravity = 6;    // 3
constexpr size_t kObsCommands = 9;       // 3
constexpr size_t kObsJointPosRel = 12;   // 12
constexpr size_t kObsJointVel = 24;      // 12
constexpr size_t kObsLastAction = 36;    // 12
constexpr size_t kObsGaitClock = 48;     // 4

constexpr double kTwoPi = 2.0 * M_PI;
constexpr double kPolicyRate = 50.0;     // Hz — policy decimated rate
constexpr double kPolicyDt = 1.0 / kPolicyRate;
}  // namespace

// ============================================================================
// on_init — declare parameters
// ============================================================================
controller_interface::CallbackReturn HarambePolicyLegsController::on_init()
{
  try {
    // Default leg-joint roster (LEG_JOINTS order: left leg, then right).
    const std::vector<std::string> default_joints = {
      "left_hip_pitch_joint_X8",   "left_hip_roll_joint_X8",
      "left_hip_yaw_joint_X8",     "left_knee_joint_X8",
      "left_ankle_pitch_joint_X4", "left_ankle_roll_joint_X4",
      "right_hip_pitch_joint_X8",  "right_hip_roll_joint_X8",
      "right_hip_yaw_joint_X8",    "right_knee_joint_X8",
      "right_ankle_pitch_joint_X4", "right_ankle_roll_joint_X4",
    };
    joint_names_ = auto_declare<std::vector<std::string>>("joints", default_joints);
    if (joint_names_.size() != kNumLegJoints) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "Expected %zu leg joints (LEG_JOINTS), got %zu",
        kNumLegJoints, joint_names_.size());
      return controller_interface::CallbackReturn::ERROR;
    }
    num_joints_ = joint_names_.size();

    // Per-joint defaults / kp / kd / effort, defaulting to the deploy contract.
    const std::vector<double> default_kp = {
      75.0, 75.0, 75.0, 75.0, 20.0, 20.0,
      75.0, 75.0, 75.0, 75.0, 20.0, 20.0};
    const std::vector<double> default_kd = {
      5.625, 6.162, 3.750, 5.511, 1.414, 1.000,
      5.625, 6.162, 3.750, 5.511, 1.414, 1.000};
    const std::vector<double> default_eff = {
      120.0, 120.0, 120.0, 120.0, 80.0, 80.0,
      120.0, 120.0, 120.0, 120.0, 80.0, 80.0};
    const std::vector<double> default_pos = {
      -0.15, 0.0, 0.0, 0.30, -0.15, 0.0,
      -0.15, 0.0, 0.0, 0.30, -0.15, 0.0};

    auto kp_vec = auto_declare<std::vector<double>>("kp", default_kp);
    auto kd_vec = auto_declare<std::vector<double>>("kd", default_kd);
    auto effort_vec = auto_declare<std::vector<double>>("effort_limits", default_eff);
    auto default_vec = auto_declare<std::vector<double>>("default_positions", default_pos);

    for (size_t s : {kp_vec.size(), kd_vec.size(), effort_vec.size(), default_vec.size()}) {
      if (s != num_joints_) {
        RCLCPP_ERROR(get_node()->get_logger(),
          "Per-joint parameter size mismatch (expected %zu)", num_joints_);
        return controller_interface::CallbackReturn::ERROR;
      }
    }

    joints_.resize(num_joints_);
    for (size_t i = 0; i < num_joints_; ++i) {
      joints_[i].name = joint_names_[i];
      joints_[i].kp = kp_vec[i];
      joints_[i].kd = kd_vec[i];
      joints_[i].effort_limit = effort_vec[i];
      joints_[i].default_pos = default_vec[i];
    }

    // Policy / loop parameters. decimation defaults are computed for the
    // configured controller_manager update_rate (50 Hz policy rate).
    int update_rate = auto_declare<int>("update_rate", 1000);
    if (update_rate <= 0) {
      update_rate = 1000;
    }
    const int default_decim = std::max(
      1, static_cast<int>(std::lround(static_cast<double>(update_rate) / kPolicyRate)));
    decimation_ = auto_declare<int>("decimation", default_decim);
    if (decimation_ <= 0) {
      decimation_ = default_decim;
    }
    action_scale_ = auto_declare<double>("action_scale", 0.5);
    clip_actions_ = auto_declare<double>("clip_actions", 5.0);
    gait_freq_ = auto_declare<double>("gait_freq", 1.5);
    warmup_steps_ = auto_declare<int>("warmup_steps", 40);
    fall_threshold_ = auto_declare<double>("fall_threshold", 0.5);
    onnx_path_ = auto_declare<std::string>("onnx_path", "");

    // Topic names.
    pelvis_imu_topic_ = auto_declare<std::string>("pelvis_imu_topic", "/pelvis/imu");
    pelvis_grav_topic_ = auto_declare<std::string>("pelvis_grav_topic", "/pelvis/grav");
    odom_topic_ = auto_declare<std::string>("odom_topic", "/odometry/filtered");
    cmd_vel_topic_ = auto_declare<std::string>("cmd_vel_topic", "/cmd_vel");
    enable_topic_ = auto_declare<std::string>("enable_topic", "~/enable");
    debug_topic_ = auto_declare<std::string>("debug_topic", "~/debug");
    publish_debug_ = auto_declare<bool>("publish_debug", false);
    use_grav_topic_ = auto_declare<bool>("use_grav_topic", false);

    if (onnx_path_.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'onnx_path' is required");
      return controller_interface::CallbackReturn::ERROR;
    }

    // Allocate buffers.
    obs_.assign(kObsDim, 0.0f);
    target_pos_.assign(num_joints_, 0.0f);
    for (size_t i = 0; i < num_joints_; ++i) {
      target_pos_[i] = static_cast<float>(joints_[i].default_pos);
    }
    for (auto & h : action_hist_) {
      h.assign(num_joints_, 0.0f);
    }

    // Seed realtime buffers with safe defaults (upright, zero velocity).
    ImuState neutral;
    neutral.proj_gravity = {0.0, 0.0, -1.0};
    neutral.valid = false;
    pelvis_imu_buf_.writeFromNonRT(neutral);
    base_lin_vel_buf_.writeFromNonRT({0.0f, 0.0f, 0.0f});
    cmd_buf_.writeFromNonRT({0.0f, 0.0f, 0.0f});
    // Wait for an explicit ~/enable=true before the policy drives the legs.
    // Until then the controller HOLDS THE DEFAULT POSE. Safer on hardware: you
    // confirm IMU / odometry / obs are sane first. Set start_enabled:=true to
    // auto-run (e.g. in sim).
    const bool start_enabled = auto_declare<bool>("start_enabled", false);
    enable_buf_.writeFromNonRT(start_enabled);

    return controller_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init exception: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
}

// ============================================================================
// on_configure — load ONNX, subscribe to topics
// ============================================================================
controller_interface::CallbackReturn HarambePolicyLegsController::on_configure(
  const rclcpp_lifecycle::State &)
{
  // Load ONNX model.
  try {
    ort_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "HarambePolicyLegsCtrl");
    Ort::SessionOptions opts;
    opts.SetIntraOpNumThreads(1);
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    ort_session_ = std::make_unique<Ort::Session>(*ort_env_, onnx_path_.c_str(), opts);
    RCLCPP_INFO(get_node()->get_logger(), "ONNX model loaded: %s", onnx_path_.c_str());
  } catch (const Ort::Exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "ONNX load failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  auto node = get_node();

  // Pelvis IMU — gyro (always) + projected_gravity (when not using grav topic).
  pelvis_imu_sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
    pelvis_imu_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
      ImuState s = *pelvis_imu_buf_.readFromNonRT();  // preserve proj_gravity if grav topic owns it
      imuToState(*msg, s);
      pelvis_imu_buf_.writeFromNonRT(s);
    });

  // Optional: BNO055 /grav topic (Vector3) for projected_gravity. The BNO
  // reports the "up vector" (gravity direction is opposite). We remap
  // sensor→body and normalize so upright reads [0,0,-1].
  if (use_grav_topic_) {
    pelvis_grav_sub_ = node->create_subscription<geometry_msgs::msg::Vector3>(
      pelvis_grav_topic_, rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
        const double mag = std::sqrt(
          msg->x * msg->x + msg->y * msg->y + msg->z * msg->z);
        if (mag < 1.0) return;  // obviously bad
        // Gravity pull direction in sensor frame (unit vector).
        std::array<double, 3> g_s = {-msg->x / mag, -msg->y / mag, -msg->z / mag};
        const auto g_b = sensorToBody(g_s);
        ImuState s = *pelvis_imu_buf_.readFromNonRT();
        s.proj_gravity = g_b;
        s.valid = true;
        pelvis_imu_buf_.writeFromNonRT(s);
      });
  }

  // Base linear velocity from the state estimator. twist.twist.linear is
  // already in the body frame (child_frame_id = base_link) → NO mount remap.
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      std::array<float, 3> v = {
        static_cast<float>(msg->twist.twist.linear.x),
        static_cast<float>(msg->twist.twist.linear.y),
        static_cast<float>(msg->twist.twist.linear.z),
      };
      base_lin_vel_buf_.writeFromNonRT(v);
    });

  cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      std::array<float, 3> c = {
        static_cast<float>(msg->linear.x),
        static_cast<float>(msg->linear.y),
        static_cast<float>(msg->angular.z),
      };
      cmd_buf_.writeFromNonRT(c);
    });

  enable_sub_ = node->create_subscription<std_msgs::msg::Bool>(
    enable_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      enable_buf_.writeFromNonRT(msg->data);
    });

  if (publish_debug_) {
    debug_pub_ = node->create_publisher<std_msgs::msg::Float32MultiArray>(
      debug_topic_, rclcpp::SystemDefaultsQoS());
  }

  RCLCPP_INFO(get_node()->get_logger(),
    "HarambePolicyLegsController configured: %zu joints, decimation=%d "
    "(policy ~%.0f Hz), action_scale=%.3f, gait_freq=%.2f, warmup=%d, "
    "pelvis_imu='%s', odom='%s', cmd_vel='%s'",
    num_joints_, decimation_, kPolicyRate, action_scale_, gait_freq_,
    warmup_steps_, pelvis_imu_topic_.c_str(), odom_topic_.c_str(),
    cmd_vel_topic_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// Interface configuration
//   command: {joint}/position, {joint}/kp, {joint}/kd  ("mode 5" — drive PD)
//   state:   {joint}/position, {joint}/velocity
// ============================================================================
controller_interface::InterfaceConfiguration
HarambePolicyLegsController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.reserve(num_joints_ * 3);
  for (const auto & n : joint_names_) {
    cfg.names.push_back(n + "/position");
    cfg.names.push_back(n + "/kp");
    cfg.names.push_back(n + "/kd");
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
HarambePolicyLegsController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.reserve(num_joints_ * 2);
  for (const auto & n : joint_names_) {
    cfg.names.push_back(n + "/position");
    cfg.names.push_back(n + "/velocity");
  }
  return cfg;
}

// ============================================================================
// on_activate — resolve interface indices, reset state
// ============================================================================
controller_interface::CallbackReturn HarambePolicyLegsController::on_activate(
  const rclcpp_lifecycle::State &)
{
  for (size_t j = 0; j < num_joints_; ++j) {
    const auto & jname = joints_[j].name;
    bool fp = false, fv = false, fcp = false, fckp = false, fckd = false;
    for (size_t k = 0; k < state_interfaces_.size(); ++k) {
      const auto & si = state_interfaces_[k];
      if (si.get_prefix_name() == jname) {
        if (si.get_interface_name() == "position") {
          joints_[j].pos_state_idx = k; fp = true;
        } else if (si.get_interface_name() == "velocity") {
          joints_[j].vel_state_idx = k; fv = true;
        }
      }
    }
    for (size_t k = 0; k < command_interfaces_.size(); ++k) {
      const auto & ci = command_interfaces_[k];
      if (ci.get_prefix_name() == jname) {
        if (ci.get_interface_name() == "position") {
          joints_[j].pos_cmd_idx = k; fcp = true;
        } else if (ci.get_interface_name() == "kp") {
          joints_[j].kp_cmd_idx = k; fckp = true;
        } else if (ci.get_interface_name() == "kd") {
          joints_[j].kd_cmd_idx = k; fckd = true;
        }
      }
    }
    if (!fp || !fv || !fcp || !fckp || !fckd) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "Missing interfaces for joint '%s' (pos=%d vel=%d cmd_pos=%d kp=%d kd=%d)",
        jname.c_str(), fp, fv, fcp, fckp, fckd);
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  // Reset state machine.
  step_counter_ = 0;
  policy_step_ = 0;
  policy_active_ = false;
  fallen_ = false;
  gait_phase_ = 0.0;
  for (auto & h : action_hist_) {
    std::fill(h.begin(), h.end(), 0.0f);
  }

  // Seed the warmup ramp from the CURRENT measured pose so the legs EASE into
  // the default pose over the warmup window instead of snapping (writing the
  // full default target with kp=75 from an arbitrary start would jerk the legs).
  ramp_start_.assign(num_joints_, 0.0f);
  for (size_t i = 0; i < num_joints_; ++i) {
    const double q =
      state_interfaces_[joints_[i].pos_state_idx].get_optional().value_or(joints_[i].default_pos);
    ramp_start_[i] = static_cast<float>(q);
    target_pos_[i] = static_cast<float>(q);
  }

  RCLCPP_INFO(get_node()->get_logger(),
    "HarambePolicyLegsController activated — holding default pose for %d "
    "warmup policy steps before enabling the policy.", warmup_steps_);

  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// on_deactivate — hold the default pose with the configured gains.
// ============================================================================
controller_interface::CallbackReturn HarambePolicyLegsController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  for (size_t i = 0; i < num_joints_; ++i) {
    (void)command_interfaces_[joints_[i].pos_cmd_idx].set_value(joints_[i].default_pos);
    (void)command_interfaces_[joints_[i].kp_cmd_idx].set_value(joints_[i].kp);
    (void)command_interfaces_[joints_[i].kd_cmd_idx].set_value(joints_[i].kd);
  }
  RCLCPP_INFO(get_node()->get_logger(), "HarambePolicyLegsController deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// update — runs at controller_manager rate; policy inference is decimated.
// ============================================================================
controller_interface::return_type HarambePolicyLegsController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  enabled_ = *enable_buf_.readFromRT();

  // Policy inference at the decimated rate.
  if (step_counter_ % static_cast<uint64_t>(decimation_) == 0) {
    buildObservation();

    // Fall detection on projected_gravity.z (pelvis up ≈ -1 when upright).
    const float grav_z = obs_[kObsProjGravity + 2];
    if (policy_active_ && !fallen_ && std::abs(grav_z) < fall_threshold_) {
      fallen_ = true;
      RCLCPP_WARN(get_node()->get_logger(),
        "FALL DETECTED at policy step %lu (grav_z=%.3f < %.3f). Holding default pose.",
        policy_step_, grav_z, fall_threshold_);
    }

    const bool warmup_done = static_cast<int>(policy_step_) >= warmup_steps_;
    if (warmup_done && enabled_ && !fallen_) {
      runPolicyInference();   // updates target_pos_, pushes the action ring
      policy_active_ = true;
    } else {
      if (!warmup_done) {
        // Warmup: EASE from the activation pose to the default pose (no snap).
        const int wsteps = warmup_steps_ > 0 ? warmup_steps_ : 1;
        float alpha = static_cast<float>(policy_step_ + 1) / static_cast<float>(wsteps);
        if (alpha > 1.0f) { alpha = 1.0f; }
        for (size_t i = 0; i < num_joints_; ++i) {
          const float d = static_cast<float>(joints_[i].default_pos);
          target_pos_[i] = ramp_start_[i] + alpha * (d - ramp_start_[i]);
        }
      } else {
        // Disabled / fallen → hold the default pose.
        for (size_t i = 0; i < num_joints_; ++i) {
          target_pos_[i] = static_cast<float>(joints_[i].default_pos);
        }
      }
      for (auto & h : action_hist_) {
        std::fill(h.begin(), h.end(), 0.0f);
      }
      policy_active_ = false;
    }

    // Advance the gait clock once per policy step (only while the policy runs).
    if (policy_active_) {
      gait_phase_ += kTwoPi * gait_freq_ * kPolicyDt;
      if (gait_phase_ >= kTwoPi) {
        gait_phase_ -= kTwoPi;
      }
    } else {
      gait_phase_ = 0.0;
    }

    if (publish_debug_) {
      publishDebugIfEnabled();
    }

    ++policy_step_;
  }

  // Every controller tick: hold the last target_pos with constant kp/kd. The
  // drive keeps tracking it between policy steps.
  writeCommands();

  ++step_counter_;
  return controller_interface::return_type::OK;
}

// ============================================================================
// Build the 52-dim observation (deploy/01_observations.md).
// ============================================================================
void HarambePolicyLegsController::buildObservation()
{
  // [0:3] base_lin_vel — from /odometry/filtered, already body frame (no remap).
  const auto blv = *base_lin_vel_buf_.readFromRT();
  obs_[kObsBaseLinVel + 0] = blv[0];
  obs_[kObsBaseLinVel + 1] = blv[1];
  obs_[kObsBaseLinVel + 2] = blv[2];

  // [3:6] base_ang_vel (gyro, body frame) and [6:9] projected_gravity (body).
  const auto pelvis = *pelvis_imu_buf_.readFromRT();
  obs_[kObsBaseAngVel + 0] = static_cast<float>(pelvis.ang_vel[0]);
  obs_[kObsBaseAngVel + 1] = static_cast<float>(pelvis.ang_vel[1]);
  obs_[kObsBaseAngVel + 2] = static_cast<float>(pelvis.ang_vel[2]);
  obs_[kObsProjGravity + 0] = static_cast<float>(pelvis.proj_gravity[0]);
  obs_[kObsProjGravity + 1] = static_cast<float>(pelvis.proj_gravity[1]);
  obs_[kObsProjGravity + 2] = static_cast<float>(pelvis.proj_gravity[2]);

  // [9:12] cmd = [vx, vy, yaw_rate].
  const auto cmd = *cmd_buf_.readFromRT();
  obs_[kObsCommands + 0] = cmd[0];
  obs_[kObsCommands + 1] = cmd[1];
  obs_[kObsCommands + 2] = cmd[2];

  // [12:24] joint_pos - default,  [24:36] joint_vel  (LEG_JOINTS order).
  for (size_t i = 0; i < num_joints_; ++i) {
    const auto pos_opt = state_interfaces_[joints_[i].pos_state_idx].get_optional();
    const auto vel_opt = state_interfaces_[joints_[i].vel_state_idx].get_optional();
    const double pos = pos_opt.value_or(joints_[i].default_pos);
    const double vel = vel_opt.value_or(0.0);
    obs_[kObsJointPosRel + i] = static_cast<float>(pos - joints_[i].default_pos);
    obs_[kObsJointVel + i] = static_cast<float>(vel);
  }

  // [36:48] last_action — the RAW action (post-clip, pre-scale) from TWO policy
  // steps ago: action_hist_[1] = action[k-2].
  for (size_t i = 0; i < num_joints_; ++i) {
    obs_[kObsLastAction + i] = action_hist_[1][i];
  }

  // [48:52] gait clock = [cos phL, cos phR, sin phL, sin phR], phR = phL + pi.
  const double phL = gait_phase_;
  const double phR = gait_phase_ + M_PI;
  obs_[kObsGaitClock + 0] = static_cast<float>(std::cos(phL));
  obs_[kObsGaitClock + 1] = static_cast<float>(std::cos(phR));
  obs_[kObsGaitClock + 2] = static_cast<float>(std::sin(phL));
  obs_[kObsGaitClock + 3] = static_cast<float>(std::sin(phR));
}

// ============================================================================
// ONNX inference → target_pos_ = default + action_scale * clip(action).
// ============================================================================
void HarambePolicyLegsController::runPolicyInference()
{
  if (!ort_session_) return;

  std::array<int64_t, 2> input_shape = {1, static_cast<int64_t>(obs_.size())};
  auto input_tensor = Ort::Value::CreateTensor<float>(
    ort_mem_info_, obs_.data(), obs_.size(),
    input_shape.data(), input_shape.size());

  const char * input_names[] = {"obs"};
  const char * output_names[] = {"actions"};

  Ort::RunOptions run_opts{nullptr};
  auto output_tensors = ort_session_->Run(
    run_opts, input_names, &input_tensor, 1, output_names, 1);

  float * actions = output_tensors[0].GetTensorMutableData<float>();

  // Compute this step's RAW action (post-clip) and the position target.
  std::vector<float> a_clipped(num_joints_, 0.0f);
  for (size_t i = 0; i < num_joints_ && i < kActionDim; ++i) {
    const float a = std::clamp(actions[i],
      static_cast<float>(-clip_actions_),
      static_cast<float>(clip_actions_));
    a_clipped[i] = a;
    target_pos_[i] = static_cast<float>(joints_[i].default_pos + action_scale_ * a);
  }

  // Advance the 2-step ring: action_hist_[1] <- [0] <- this action.
  action_hist_[1] = action_hist_[0];
  action_hist_[0] = std::move(a_clipped);
}

// ============================================================================
// Write position target + constant kp/kd to the drives ("mode 5" — drive PD).
// NO torque is computed here; the EtherCAT drive closes the PD loop at 1 kHz.
// ============================================================================
void HarambePolicyLegsController::writeCommands()
{
  for (size_t i = 0; i < num_joints_; ++i) {
    (void)command_interfaces_[joints_[i].pos_cmd_idx].set_value(
      static_cast<double>(target_pos_[i]));
    (void)command_interfaces_[joints_[i].kp_cmd_idx].set_value(joints_[i].kp);
    (void)command_interfaces_[joints_[i].kd_cmd_idx].set_value(joints_[i].kd);
  }
}

// ============================================================================
// Pelvis BNO055 sensor→body remap:
//   sensor +X = up (body +Z), sensor +Y = right (body -Y), sensor +Z = fwd (body +X)
//   => v_body = (v_sz, -v_sy, v_sx)   (self-inverse)
// ============================================================================
std::array<double, 3> HarambePolicyLegsController::sensorToBody(
  const std::array<double, 3> & v_s) const
{
  return {v_s[2], -v_s[1], v_s[0]};
}

// ============================================================================
// IMU ROS msg → body-frame gyro + (optionally) projected gravity.
// angular_velocity is in the SENSOR frame → remap to body. projected_gravity
// is derived from orientation only when the /grav topic is not in use.
// ============================================================================
void HarambePolicyLegsController::imuToState(
  const sensor_msgs::msg::Imu & msg, ImuState & out) const
{
  // Gyro: sensor → body frame.
  const std::array<double, 3> w_s = {
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z};
  out.ang_vel = sensorToBody(w_s);

  // Projected gravity from orientation: proj_grav = quat_rot_inv(q, [0,0,-1]).
  // tf2 expresses the world-gravity vector in the sensor frame; remap to body.
  if (!use_grav_topic_) {
    tf2::Quaternion q(
      msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    const double qmag = std::sqrt(
      q.x() * q.x() + q.y() * q.y() + q.z() * q.z() + q.w() * q.w());
    if (qmag > 0.5) {   // BNO055 returns a zero quat when uncalibrated.
      tf2::Matrix3x3 R_inv(q.inverse());
      tf2::Vector3 g_world(0.0, 0.0, -1.0);
      tf2::Vector3 g_sensor = R_inv * g_world;
      std::array<double, 3> g_s = {g_sensor.x(), g_sensor.y(), g_sensor.z()};
      out.proj_gravity = sensorToBody(g_s);
    }
  }
  out.valid = true;
}

// ============================================================================
// Debug publish — [targets(12) | positions(12) | velocities(12)
//                  | last_action(12) | obs(52)]  = 100 floats
// ============================================================================
void HarambePolicyLegsController::publishDebugIfEnabled()
{
  if (!debug_pub_) return;
  std_msgs::msg::Float32MultiArray m;
  m.data.reserve(num_joints_ * 4 + kObsDim);
  for (size_t i = 0; i < num_joints_; ++i) {
    m.data.push_back(target_pos_[i]);
  }
  for (size_t i = 0; i < num_joints_; ++i) {
    m.data.push_back(static_cast<float>(
      state_interfaces_[joints_[i].pos_state_idx].get_optional().value_or(0.0)));
  }
  for (size_t i = 0; i < num_joints_; ++i) {
    m.data.push_back(static_cast<float>(
      state_interfaces_[joints_[i].vel_state_idx].get_optional().value_or(0.0)));
  }
  for (size_t i = 0; i < num_joints_; ++i) {
    m.data.push_back(action_hist_[0][i]);
  }
  for (float v : obs_) m.data.push_back(v);
  debug_pub_->publish(m);
}

}  // namespace harambe_policy_legs_controller

PLUGINLIB_EXPORT_CLASS(
  harambe_policy_legs_controller::HarambePolicyLegsController,
  controller_interface::ControllerInterface)
