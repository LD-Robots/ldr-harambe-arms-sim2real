#include "harambe_policy_controller/harambe_policy_controller.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

#include <kdl_parser/kdl_parser.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace harambe_policy_controller
{

namespace
{
constexpr size_t kNumJoints = 25;
constexpr size_t kObsDim = 93;
// Observation layout offsets
constexpr size_t kObsBaseLinVel = 0;     // 3
constexpr size_t kObsPelvisAngVel = 3;   // 3
constexpr size_t kObsPelvisGravity = 6;  // 3
constexpr size_t kObsTorsoAngVel = 9;    // 3
constexpr size_t kObsTorsoGravity = 12;  // 3
constexpr size_t kObsCommands = 15;      // 3
constexpr size_t kObsJointPosRel = 18;   // 25
constexpr size_t kObsJointVel = 43;      // 25
constexpr size_t kObsPrevAction = 68;    // 25
}  // namespace

// ============================================================================
// on_init — declare parameters
// ============================================================================
controller_interface::CallbackReturn HarambePolicyController::on_init()
{
  try {
    // Joints
    joint_names_ = auto_declare<std::vector<std::string>>("joints", {});
    if (joint_names_.size() != kNumJoints) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "Expected %zu joints (HARDWARE_JOINT_ORDER), got %zu", kNumJoints, joint_names_.size());
      return controller_interface::CallbackReturn::ERROR;
    }
    num_joints_ = joint_names_.size();

    // Per-joint parameters
    auto kp_vec = auto_declare<std::vector<double>>("kp", std::vector<double>(num_joints_, 100.0));
    auto kd_vec = auto_declare<std::vector<double>>("kd", std::vector<double>(num_joints_, 2.0));
    auto effort_vec = auto_declare<std::vector<double>>(
      "effort_limits", std::vector<double>(num_joints_, 50.0));
    auto default_vec = auto_declare<std::vector<double>>(
      "default_positions", std::vector<double>(num_joints_, 0.0));
    auto friction_vec = auto_declare<std::vector<double>>(
      "friction", std::vector<double>(num_joints_, 0.0));

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
      joints_[i].friction = (i < friction_vec.size()) ? friction_vec[i] : 0.0;
    }

    // Policy / loop parameters
    decimation_ = auto_declare<int>("decimation", 4);
    action_scale_ = auto_declare<double>("action_scale", 0.25);
    clip_actions_ = auto_declare<double>("clip_actions", 5.0);
    warmup_steps_ = auto_declare<int>("warmup_steps", 200);
    fall_threshold_ = auto_declare<double>("fall_threshold", 0.5);
    onnx_path_ = auto_declare<std::string>("onnx_path", "");
    urdf_path_ = auto_declare<std::string>("urdf_path", "");

    // Topic names (defaults align with BNO055 ROS2 driver conventions)
    pelvis_imu_topic_ = auto_declare<std::string>("pelvis_imu_topic", "/pelvis/imu");
    torso_imu_topic_ = auto_declare<std::string>("torso_imu_topic", "/torso/imu");
    pelvis_grav_topic_ = auto_declare<std::string>("pelvis_grav_topic", "/pelvis/grav");
    torso_grav_topic_ = auto_declare<std::string>("torso_grav_topic", "/torso/grav");
    cmd_vel_topic_ = auto_declare<std::string>("cmd_vel_topic", "/cmd_vel");
    enable_topic_ = auto_declare<std::string>("enable_topic", "~/enable");
    debug_topic_ = auto_declare<std::string>("debug_topic", "~/debug");
    publish_debug_ = auto_declare<bool>("publish_debug", false);
    use_grav_topic_ = auto_declare<bool>("use_grav_topic", true);

    if (onnx_path_.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'onnx_path' is required");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (urdf_path_.empty()) {
      RCLCPP_WARN(get_node()->get_logger(),
        "Parameter 'urdf_path' empty — gravity compensation will be DISABLED");
    }

    // Allocate buffers
    obs_.assign(kObsDim, 0.0f);
    prev_action_.assign(num_joints_, 0.0f);
    target_pos_.assign(num_joints_, 0.0f);
    for (size_t i = 0; i < num_joints_; ++i) {
      target_pos_[i] = static_cast<float>(joints_[i].default_pos);
    }
    gravity_torques_.assign(num_joints_, 0.0);

    // Initialize realtime buffers with safe defaults
    ImuState neutral;
    neutral.proj_gravity = {0.0, 0.0, -1.0};
    pelvis_imu_buf_.writeFromNonRT(neutral);
    torso_imu_buf_.writeFromNonRT(neutral);
    cmd_buf_.writeFromNonRT({0.0f, 0.0f, 0.0f});
    enable_buf_.writeFromNonRT(true);

    return controller_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init exception: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
}

// ============================================================================
// on_configure — load ONNX, init KDL, subscribe to topics
// ============================================================================
controller_interface::CallbackReturn HarambePolicyController::on_configure(
  const rclcpp_lifecycle::State &)
{
  // Load ONNX model
  try {
    ort_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "HarambePolicyCtrl");
    Ort::SessionOptions opts;
    opts.SetIntraOpNumThreads(1);
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    ort_session_ = std::make_unique<Ort::Session>(*ort_env_, onnx_path_.c_str(), opts);
    RCLCPP_INFO(get_node()->get_logger(), "ONNX model loaded: %s", onnx_path_.c_str());
  } catch (const Ort::Exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "ONNX load failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Load URDF + init KDL for gravity compensation
  if (!urdf_path_.empty()) {
    std::ifstream urdf_file(urdf_path_);
    if (urdf_file.is_open()) {
      std::string urdf_str((std::istreambuf_iterator<char>(urdf_file)),
                           std::istreambuf_iterator<char>());
      if (initKDL(urdf_str)) {
        RCLCPP_INFO(get_node()->get_logger(), "KDL gravity compensation ready (%zu chains)",
          kdl_chains_.size());
      } else {
        RCLCPP_WARN(get_node()->get_logger(),
          "KDL init failed — gravity compensation DISABLED");
      }
    } else {
      RCLCPP_WARN(get_node()->get_logger(),
        "Could not open URDF file '%s' — gravity compensation DISABLED", urdf_path_.c_str());
    }
  }

  // Subscribe to IMU, cmd_vel, enable
  auto node = get_node();

  pelvis_imu_sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
    pelvis_imu_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
      ImuState s = *pelvis_imu_buf_.readFromNonRT();  // preserve proj_gravity if grav topic owns it
      imuToState(*msg, ImuMount::PELVIS, s);
      pelvis_imu_buf_.writeFromNonRT(s);
    });

  torso_imu_sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
    torso_imu_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
      ImuState s = *torso_imu_buf_.readFromNonRT();
      imuToState(*msg, ImuMount::TORSO, s);
      torso_imu_buf_.writeFromNonRT(s);
    });

  // Optional: use BNO055 /grav topic (Vector3) for projected_gravity.
  // More accurate than deriving from orientation (no Kalman drift, no mag dep).
  if (use_grav_topic_) {
    pelvis_grav_sub_ = node->create_subscription<geometry_msgs::msg::Vector3>(
      pelvis_grav_topic_, rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
        // BNO055 reports the "up vector" (gravity direction is opposite, magnitude ~9.81).
        const double mag = std::sqrt(
          msg->x * msg->x + msg->y * msg->y + msg->z * msg->z);
        if (mag < 1.0) return;  // obviously bad
        // Gravity pull direction in sensor frame (unit vector)
        std::array<double, 3> g_s = {-msg->x / mag, -msg->y / mag, -msg->z / mag};
        // Transform to base frame via pelvis mount
        const auto g_b = sensorToBase(g_s, ImuMount::PELVIS);
        ImuState s = *pelvis_imu_buf_.readFromNonRT();
        s.proj_gravity = g_b;
        s.valid = true;
        pelvis_imu_buf_.writeFromNonRT(s);
      });

    torso_grav_sub_ = node->create_subscription<geometry_msgs::msg::Vector3>(
      torso_grav_topic_, rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
        const double mag = std::sqrt(
          msg->x * msg->x + msg->y * msg->y + msg->z * msg->z);
        if (mag < 1.0) return;
        std::array<double, 3> g_s = {-msg->x / mag, -msg->y / mag, -msg->z / mag};
        const auto g_b = sensorToBase(g_s, ImuMount::TORSO);
        ImuState s = *torso_imu_buf_.readFromNonRT();
        s.proj_gravity = g_b;
        s.valid = true;
        torso_imu_buf_.writeFromNonRT(s);
      });
  }

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
    "HarambePolicyController configured: %zu joints, decimation=%d, action_scale=%.3f, "
    "warmup=%d, pelvis_imu='%s', torso_imu='%s', cmd_vel='%s'",
    num_joints_, decimation_, action_scale_, warmup_steps_,
    pelvis_imu_topic_.c_str(), torso_imu_topic_.c_str(), cmd_vel_topic_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// Interface configuration — claim position+velocity state, effort command
// ============================================================================
controller_interface::InterfaceConfiguration
HarambePolicyController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & n : joint_names_) {
    cfg.names.push_back(n + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
HarambePolicyController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & n : joint_names_) {
    cfg.names.push_back(n + "/" + hardware_interface::HW_IF_POSITION);
  }
  for (const auto & n : joint_names_) {
    cfg.names.push_back(n + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return cfg;
}

// ============================================================================
// on_activate — resolve interface indices
// ============================================================================
controller_interface::CallbackReturn HarambePolicyController::on_activate(
  const rclcpp_lifecycle::State &)
{
  for (size_t j = 0; j < num_joints_; ++j) {
    const auto & jname = joints_[j].name;
    bool fp = false, fv = false, fc = false;
    for (size_t k = 0; k < state_interfaces_.size(); ++k) {
      const auto & si = state_interfaces_[k];
      if (si.get_prefix_name() == jname) {
        if (si.get_interface_name() == "position") {
          joints_[j].pos_iface_idx = k; fp = true;
        } else if (si.get_interface_name() == "velocity") {
          joints_[j].vel_iface_idx = k; fv = true;
        }
      }
    }
    for (size_t k = 0; k < command_interfaces_.size(); ++k) {
      if (command_interfaces_[k].get_prefix_name() == jname) {
        joints_[j].cmd_iface_idx = k; fc = true;
      }
    }
    if (!fp || !fv || !fc) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "Missing interfaces for joint '%s' (pos=%d vel=%d cmd=%d)",
        jname.c_str(), fp, fv, fc);
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  // Reset state
  step_counter_ = 0;
  policy_step_ = 0;
  policy_active_ = false;
  fallen_ = false;
  std::fill(prev_action_.begin(), prev_action_.end(), 0.0f);

  // Seed target_pos_ with current joint positions (so initial warmup ramps from here)
  for (size_t i = 0; i < num_joints_; ++i) {
    target_pos_[i] = static_cast<float>(
      state_interfaces_[joints_[i].pos_iface_idx].get_value());
  }

  RCLCPP_INFO(get_node()->get_logger(),
    "HarambePolicyController activated — policy will start after %d warmup steps "
    "(gain-boosted ramp to default pose).", warmup_steps_);

  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// on_deactivate — zero torques
// ============================================================================
controller_interface::CallbackReturn HarambePolicyController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  for (size_t i = 0; i < num_joints_; ++i) {
    command_interfaces_[joints_[i].cmd_iface_idx].set_value(0.0);
  }
  RCLCPP_INFO(get_node()->get_logger(), "HarambePolicyController deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// update — runs at controller_manager rate (should be 200 Hz)
// ============================================================================
controller_interface::return_type HarambePolicyController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  enabled_ = *enable_buf_.readFromRT();

  // --- Warmup ramp (first 200 steps): target = α·default_pos ---
  const int ramp_steps = warmup_steps_;
  if (static_cast<int>(step_counter_) < ramp_steps) {
    const double alpha =
      static_cast<double>(step_counter_ + 1) / static_cast<double>(ramp_steps);
    for (size_t i = 0; i < num_joints_; ++i) {
      target_pos_[i] = static_cast<float>(alpha * joints_[i].default_pos);
    }
  }

  // --- Policy inference at decimated rate ---
  if (step_counter_ % static_cast<uint64_t>(decimation_) == 0) {
    buildObservation();

    // Fall detection: pelvis gravity z component
    const float grav_z = obs_[kObsPelvisGravity + 2];
    if (policy_active_ && !fallen_ && std::abs(grav_z) < fall_threshold_) {
      fallen_ = true;
      RCLCPP_WARN(get_node()->get_logger(),
        "FALL DETECTED at step %lu (grav_z=%.3f < %.3f). Actions disabled.",
        step_counter_, grav_z, fall_threshold_);
    }

    // Policy only runs after ramp complete
    if (static_cast<int>(step_counter_) >= ramp_steps &&
        static_cast<int>(policy_step_) >= warmup_steps_ &&
        enabled_ && !fallen_) {
      runPolicyInference();
      policy_active_ = true;
    } else if (static_cast<int>(policy_step_) >= warmup_steps_ && !enabled_) {
      // Hold last target when disabled (don't overwrite prev_action)
    }

    ++policy_step_;
  }

  // --- PD + gravity comp + effort write (every tick) ---
  computeGravityCompensation();
  applyPDControl();

  // --- Debug publishing (at policy rate) ---
  if (publish_debug_ && step_counter_ % static_cast<uint64_t>(decimation_) == 0) {
    publishDebugIfEnabled();
  }

  ++step_counter_;
  return controller_interface::return_type::OK;
}

// ============================================================================
// Build 93-dim observation — mirror of HarambePolicyPlugin::BuildObservation
// ============================================================================
void HarambePolicyController::buildObservation()
{
  // [0:3] base_lin_vel — real robot rarely has direct base velocity sensing.
  // The Gazebo plugin derives this from world pose. Here we use a zero baseline
  // (trained DR on lin_vel ±0.5 m/s should cover this). If you have an EKF,
  // publish base velocity via /imu/pelvis linear_acceleration integration or
  // a separate /base_lin_vel topic (TODO: subscriber hook).
  obs_[kObsBaseLinVel + 0] = 0.0f;
  obs_[kObsBaseLinVel + 1] = 0.0f;
  obs_[kObsBaseLinVel + 2] = 0.0f;

  // [3:9] pelvis IMU (ang_vel, projected_gravity)
  const auto pelvis = *pelvis_imu_buf_.readFromRT();
  obs_[kObsPelvisAngVel + 0] = static_cast<float>(pelvis.ang_vel[0]);
  obs_[kObsPelvisAngVel + 1] = static_cast<float>(pelvis.ang_vel[1]);
  obs_[kObsPelvisAngVel + 2] = static_cast<float>(pelvis.ang_vel[2]);
  obs_[kObsPelvisGravity + 0] = static_cast<float>(pelvis.proj_gravity[0]);
  obs_[kObsPelvisGravity + 1] = static_cast<float>(pelvis.proj_gravity[1]);
  obs_[kObsPelvisGravity + 2] = static_cast<float>(pelvis.proj_gravity[2]);

  // [9:15] torso IMU (fallback to pelvis if torso not seen)
  const auto torso_msg = *torso_imu_buf_.readFromRT();
  const ImuState & torso = torso_msg.valid ? torso_msg : pelvis;
  obs_[kObsTorsoAngVel + 0] = static_cast<float>(torso.ang_vel[0]);
  obs_[kObsTorsoAngVel + 1] = static_cast<float>(torso.ang_vel[1]);
  obs_[kObsTorsoAngVel + 2] = static_cast<float>(torso.ang_vel[2]);
  obs_[kObsTorsoGravity + 0] = static_cast<float>(torso.proj_gravity[0]);
  obs_[kObsTorsoGravity + 1] = static_cast<float>(torso.proj_gravity[1]);
  obs_[kObsTorsoGravity + 2] = static_cast<float>(torso.proj_gravity[2]);

  // [15:18] velocity commands
  const auto cmd = *cmd_buf_.readFromRT();
  obs_[kObsCommands + 0] = cmd[0];
  obs_[kObsCommands + 1] = cmd[1];
  obs_[kObsCommands + 2] = cmd[2];

  // [18:43] joint_pos_rel and [43:68] joint_vel (HARDWARE_JOINT_ORDER)
  for (size_t i = 0; i < num_joints_; ++i) {
    const double pos = state_interfaces_[joints_[i].pos_iface_idx].get_value();
    const double vel = state_interfaces_[joints_[i].vel_iface_idx].get_value();
    obs_[kObsJointPosRel + i] = static_cast<float>(pos - joints_[i].default_pos);
    obs_[kObsJointVel + i] = static_cast<float>(vel);
  }

  // [68:93] prev_action
  for (size_t i = 0; i < num_joints_; ++i) {
    obs_[kObsPrevAction + i] = prev_action_[i];
  }
}

// ============================================================================
// ONNX inference
// ============================================================================
void HarambePolicyController::runPolicyInference()
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

  const double scale = fallen_ ? 0.0 : action_scale_;
  for (size_t i = 0; i < num_joints_; ++i) {
    float a = std::clamp(actions[i],
      static_cast<float>(-clip_actions_),
      static_cast<float>(clip_actions_));
    prev_action_[i] = a;
    target_pos_[i] = static_cast<float>(a * scale + joints_[i].default_pos);
  }
}

// ============================================================================
// PD + gravity compensation → effort write
// τ = kp·(target - pos) - kd·vel + g(q),  clamped to ±effort_limit
// ============================================================================
void HarambePolicyController::applyPDControl()
{
  for (size_t i = 0; i < num_joints_; ++i) {
    const double pos = state_interfaces_[joints_[i].pos_iface_idx].get_value();
    const double vel = state_interfaces_[joints_[i].vel_iface_idx].get_value();
    const double target = target_pos_[i];

    double kp = joints_[i].kp;
    double kd = joints_[i].kd;

    // Warmup gain boost (matches plugin): 2x Kp, 3x Kd while policy_active_=false
    if (!policy_active_) {
      kp *= 2.0;
      kd *= 3.0;
    }

    const double grav = (kdl_ready_ && i < gravity_torques_.size())
                        ? gravity_torques_[i] : 0.0;

    double tau = kp * (target - pos) - kd * vel + grav;
    tau = std::clamp(tau, -joints_[i].effort_limit, joints_[i].effort_limit);

    // Emergency: fully disable effort when fallen OR user disabled
    if (fallen_ || !enabled_) {
      // Keep gravity comp only — prevents crumpling while preserving safety behavior
      tau = std::clamp(grav, -joints_[i].effort_limit, joints_[i].effort_limit);
    }

    command_interfaces_[joints_[i].cmd_iface_idx].set_value(tau);
  }
}

// ============================================================================
// KDL initialization — build chains from urdf_base to each limb tip
// ============================================================================
bool HarambePolicyController::initKDL(const std::string & urdf_string)
{
  // Reset state — controller may be re-configured without re-instantiation.
  kdl_chains_.clear();
  kdl_ready_ = false;

  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf_string, tree)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse URDF into KDL tree");
    return false;
  }

  struct ChainDef
  {
    std::string root;
    std::string tip;
    std::vector<std::string> joint_names;
  };
  const std::vector<ChainDef> chain_defs = {
    {"urdf_base", "urdf_foot_assembly",
     {"left_hip_pitch_joint_X8", "left_hip_roll_joint_X8", "left_hip_yaw_joint_X8",
      "left_knee_joint_X8", "left_ankle_pitch_joint_X4", "left_ankle_roll_joint_X4"}},
    {"urdf_base", "urdf_foot_assembly_2",
     {"right_hip_pitch_joint_X8", "right_hip_roll_joint_X8", "right_hip_yaw_joint_X8",
      "right_knee_joint_X8", "right_ankle_pitch_joint_X4", "right_ankle_roll_joint_X4"}},
    {"urdf_base", "urdf_l_wrist_assembly",
     {"left_shoulder_pitch_joint_X6", "left_shoulder_roll_joint_X6",
      "left_shoulder_yaw_joint_X4", "left_elbow_pitch_joint_X6",
      "left_wrist_yaw_joint_X4", "left_wrist_roll_joint_X4"}},
    {"urdf_base", "urdf_r_wrist_assembly",
     {"right_shoulder_pitch_joint_X6", "right_shoulder_roll_joint_X6",
      "right_shoulder_yaw_joint_X4", "right_elbow_pitch_joint_X6",
      "right_wrist_yaw_joint_X4", "right_wrist_roll_joint_X4"}},
  };

  for (const auto & def : chain_defs) {
    KDLChainData cd;
    if (!tree.getChain(def.root, def.tip, cd.chain)) {
      RCLCPP_WARN(get_node()->get_logger(),
        "KDL chain %s -> %s not found", def.root.c_str(), def.tip.c_str());
      continue;
    }
    for (unsigned s = 0; s < cd.chain.getNrOfSegments(); ++s) {
      const auto & seg = cd.chain.getSegment(s);
      if (seg.getJoint().getType() == KDL::Joint::None) continue;
      int plugin_idx = -1;
      for (size_t j = 0; j < num_joints_; ++j) {
        if (joints_[j].name == seg.getJoint().getName()) { plugin_idx = static_cast<int>(j); break; }
      }
      cd.joint_map.push_back(plugin_idx);
    }
    KDL::Vector grav(0.0, 0.0, -9.81);
    cd.dynamics = std::make_unique<KDL::ChainDynParam>(cd.chain, grav);
    kdl_chains_.push_back(std::move(cd));
  }

  kdl_ready_ = !kdl_chains_.empty();
  return kdl_ready_;
}

// ============================================================================
// Compute gravity compensation torques per joint via KDL
// ============================================================================
void HarambePolicyController::computeGravityCompensation()
{
  if (!kdl_ready_) return;
  std::fill(gravity_torques_.begin(), gravity_torques_.end(), 0.0);

  for (auto & cd : kdl_chains_) {
    const unsigned n = cd.chain.getNrOfJoints();
    KDL::JntArray q(n), grav_tau(n);
    unsigned kdl_idx = 0;
    for (unsigned s = 0; s < cd.chain.getNrOfSegments(); ++s) {
      const auto & seg = cd.chain.getSegment(s);
      if (seg.getJoint().getType() == KDL::Joint::None) continue;
      if (kdl_idx < cd.joint_map.size() && cd.joint_map[kdl_idx] >= 0) {
        const int pi = cd.joint_map[kdl_idx];
        q(kdl_idx) = state_interfaces_[joints_[pi].pos_iface_idx].get_value();
      }
      ++kdl_idx;
    }
    cd.dynamics->JntToGravity(q, grav_tau);
    for (unsigned k = 0; k < cd.joint_map.size() && k < n; ++k) {
      if (cd.joint_map[k] >= 0) {
        gravity_torques_[cd.joint_map[k]] = grav_tau(k);
      }
    }
  }
}

// ============================================================================
// Sensor→base mount transform (per xacro IMU mount rpy).
// Training (observations.py) defines these mount axes:
//   Pelvis rpy="π,-π/2,0":  sensor_X+=up (base_Z+), sensor_Y+=right (-base_Y),
//                           sensor_Z+=forward (base_X+)
//     → v_base = (v_sensor.z, -v_sensor.y, v_sensor.x)
//   Torso  rpy="0,π/2,0":   sensor_X+=up (torso_Z+), sensor_Y+=right (-torso_Y),
//                           sensor_Z+=backward (-torso_X)
//     → v_torso = (-v_sensor.z, -v_sensor.y, v_sensor.x)
// These are self-inverse, so training feeds the body-frame values unchanged to
// the policy — we therefore need to deliver BODY-frame values here.
// ============================================================================
std::array<double, 3> HarambePolicyController::sensorToBase(
  const std::array<double, 3> & v_s, ImuMount mount) const
{
  if (mount == ImuMount::PELVIS) {
    return {v_s[2], -v_s[1], v_s[0]};
  }
  // Torso
  return {-v_s[2], -v_s[1], v_s[0]};
}

// ============================================================================
// IMU ROS msg → body-frame ang_vel + projected gravity
// sensor_msgs/Imu reports angular_velocity and linear_acceleration in SENSOR
// frame. We transform to BODY frame using the xacro mount (self-inverse per
// training). If the /grav topic subscriber has already populated proj_gravity,
// we leave it intact; otherwise we derive it from orientation.
// ============================================================================
void HarambePolicyController::imuToState(
  const sensor_msgs::msg::Imu & msg, ImuMount mount, ImuState & out) const
{
  // Angular velocity: sensor → body frame
  const std::array<double, 3> w_s = {
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z};
  out.ang_vel = sensorToBase(w_s, mount);

  // Linear acceleration (sensor frame, raw — kept for reference)
  out.lin_accel[0] = msg.linear_acceleration.x;
  out.lin_accel[1] = msg.linear_acceleration.y;
  out.lin_accel[2] = msg.linear_acceleration.z;

  // Projected gravity: only derive from orientation if grav topic is disabled
  // (otherwise the /pelvis/grav subscriber handles it more accurately).
  if (!use_grav_topic_) {
    tf2::Quaternion q(
      msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    // Validate quaternion magnitude — BNO055 returns zero quat when uncalibrated.
    const double qmag = std::sqrt(
      q.x() * q.x() + q.y() * q.y() + q.z() * q.z() + q.w() * q.w());
    if (qmag > 0.5) {
      tf2::Matrix3x3 R_inv(q.inverse());
      tf2::Vector3 g_world(0.0, 0.0, -1.0);
      tf2::Vector3 g_sensor = R_inv * g_world;
      std::array<double, 3> g_s = {g_sensor.x(), g_sensor.y(), g_sensor.z()};
      out.proj_gravity = sensorToBase(g_s, mount);
    }
  }
  out.valid = true;
}

// ============================================================================
// Debug publish — [efforts(25) | targets(25) | positions(25) | velocities(25)
//                  | actions(25) | obs(93)]  = 218 floats
// ============================================================================
void HarambePolicyController::publishDebugIfEnabled()
{
  if (!debug_pub_) return;
  std_msgs::msg::Float32MultiArray m;
  m.data.reserve(num_joints_ * 5 + kObsDim);
  for (size_t i = 0; i < num_joints_; ++i) {
    m.data.push_back(static_cast<float>(command_interfaces_[joints_[i].cmd_iface_idx].get_value()));
  }
  for (size_t i = 0; i < num_joints_; ++i) {
    m.data.push_back(target_pos_[i]);
  }
  for (size_t i = 0; i < num_joints_; ++i) {
    m.data.push_back(static_cast<float>(state_interfaces_[joints_[i].pos_iface_idx].get_value()));
  }
  for (size_t i = 0; i < num_joints_; ++i) {
    m.data.push_back(static_cast<float>(state_interfaces_[joints_[i].vel_iface_idx].get_value()));
  }
  for (size_t i = 0; i < num_joints_; ++i) {
    m.data.push_back(prev_action_[i]);
  }
  for (float v : obs_) m.data.push_back(v);
  debug_pub_->publish(m);
}

}  // namespace harambe_policy_controller

PLUGINLIB_EXPORT_CLASS(
  harambe_policy_controller::HarambePolicyController,
  controller_interface::ControllerInterface)
