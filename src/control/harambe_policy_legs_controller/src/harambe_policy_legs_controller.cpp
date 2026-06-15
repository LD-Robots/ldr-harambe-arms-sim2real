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

    // Global PD-gain multipliers — effective gain = per-joint kp/kd × scale.
    // Lets you ramp stiffness up SAFELY on hardware (start <1.0) without editing
    // every per-joint value. 1.0 = use the config gains as-is.
    const double kp_scale = auto_declare<double>("kp_scale", 1.0);
    const double kd_scale = auto_declare<double>("kd_scale", 1.0);

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
      joints_[i].kp = kp_vec[i] * kp_scale;
      joints_[i].kd = kd_vec[i] * kd_scale;
      joints_[i].effort_limit = effort_vec[i];
      joints_[i].default_pos = default_vec[i];
    }
    if (kp_scale != 1.0 || kd_scale != 1.0) {
      RCLCPP_WARN(get_node()->get_logger(),
        "PD gains SCALED at deploy: kp_scale=%.3f kd_scale=%.3f "
        "(effective gain = config kp/kd × scale).", kp_scale, kd_scale);
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
    upright_threshold_ = auto_declare<double>("upright_threshold", 0.8);
    obs_timeout_ = auto_declare<double>("obs_timeout", 0.2);
    require_odom_ = auto_declare<bool>("require_odom", true);
    // dry_run: run the policy + log, but HOLD the default pose (never send the
    // policy target to the motors). Safe shadow mode for validating obs/actions.
    dry_run_ = auto_declare<bool>("dry_run", false);
    // fake_imu (diagnostic): feed the policy a sim-like IDEAL IMU —
    // projected_gravity=[0,0,-1], gyro=0 (clean, no noise/latency, as sim shows
    // standing). Isolates whether the real IMU channel drives the thrashing.
    fake_imu_ = auto_declare<bool>("fake_imu", false);
    // IMU calibration filter (normalizes obs). Bake a previously captured
    // calibration here, or leave identity/0 and capture at runtime via
    // ~/calibrate_imu while the robot stands level & still.
    {
      auto R = auto_declare<std::vector<double>>(
        "imu_mount_R", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
      if (R.size() == 9) std::copy(R.begin(), R.end(), imu_mount_R_.begin());
      auto gb = auto_declare<std::vector<double>>("gyro_bias", {0.0, 0.0, 0.0});
      if (gb.size() == 3) std::copy(gb.begin(), gb.end(), gyro_bias_.begin());
    }
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
      last_imu_ns_.store(get_node()->now().nanoseconds());   // freshness gate
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
      last_odom_ns_.store(get_node()->now().nanoseconds());   // freshness gate
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

  // ~/calibrate_imu=true → capture the IMU calibration filter on the next policy
  // step (robot must be standing LEVEL & still). Normalizes gravity → [0,0,-1].
  calibrate_sub_ = node->create_subscription<std_msgs::msg::Bool>(
    "~/calibrate_imu", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      if (msg->data) calibrate_pending_.store(true);
    });

  if (publish_debug_) {
    // SensorDataQoS = BEST_EFFORT: drop a debug sample rather than apply
    // backpressure. Combined with RealtimePublisher (write happens on a non-RT
    // thread), the 1 kHz update() never blocks on the subscriber or the network.
    debug_pub_ = node->create_publisher<std_msgs::msg::Float32MultiArray>(
      debug_topic_, rclcpp::SensorDataQoS());
    rt_debug_pub_ = std::make_unique<
      realtime_tools::RealtimePublisher<std_msgs::msg::Float32MultiArray>>(debug_pub_);
    // Pre-size the message buffer once (off the RT path) so the first publish
    // does not allocate inside update().
    rt_debug_pub_->msg_.data.reserve(num_joints_ * 4 + kObsDim);
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
  cfg.names.reserve(num_joints_ * 5);
  for (const auto & n : joint_names_) {
    cfg.names.push_back(n + "/position");
    // Claim velocity + effort too, and write them to ZERO every tick. The drive
    // (and especially the ankle-linkage driver, which transforms velocity/effort
    // motor↔joint via the Jacobian) READS these PDOs; if left unclaimed they stay
    // NaN and the drive gets a garbage velocity/torque feed-forward → the joint
    // slews to its limit at full speed (this broke an ankle eccentric once).
    cfg.names.push_back(n + "/velocity");
    cfg.names.push_back(n + "/effort");
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
    bool fp = false, fv = false, fcp = false, fcv = false, fce = false,
         fckp = false, fckd = false;
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
        } else if (ci.get_interface_name() == "velocity") {
          joints_[j].vel_cmd_idx = k; fcv = true;
        } else if (ci.get_interface_name() == "effort") {
          joints_[j].eff_cmd_idx = k; fce = true;
        } else if (ci.get_interface_name() == "kp") {
          joints_[j].kp_cmd_idx = k; fckp = true;
        } else if (ci.get_interface_name() == "kd") {
          joints_[j].kd_cmd_idx = k; fckd = true;
        }
      }
    }
    if (!fp || !fv || !fcp || !fcv || !fce || !fckp || !fckd) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "Missing interfaces for joint '%s' (pos=%d vel=%d cmd_pos=%d cmd_vel=%d "
        "cmd_eff=%d kp=%d kd=%d)",
        jname.c_str(), fp, fv, fcp, fcv, fce, fckp, fckd);
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

  // Pre-grow the ONNX Runtime CPU arena OFF the RT path. ORT lazily allocates
  // (and may re-allocate) its memory arena on the FIRST Run() — tens of ms —
  // which on the 1 kHz update() loop is a guaranteed cycle overrun. on_activate
  // is NOT the RT thread, so run one throwaway inference here to make the arena
  // hot before update() ticks. The output is discarded and target_pos_ /
  // action_hist_ are left untouched (we do NOT call runPolicyInference()).
  if (ort_session_) {
    try {
      std::vector<float> warm_obs(kObsDim, 0.0f);
      std::array<int64_t, 2> warm_shape = {1, static_cast<int64_t>(warm_obs.size())};
      auto warm_in = Ort::Value::CreateTensor<float>(
        ort_mem_info_, warm_obs.data(), warm_obs.size(),
        warm_shape.data(), warm_shape.size());
      const char * in_names[] = {"obs"};
      const char * out_names[] = {"actions"};
      Ort::RunOptions warm_opts{nullptr};
      ort_session_->Run(warm_opts, in_names, &warm_in, 1, out_names, 1);
      RCLCPP_INFO(get_node()->get_logger(),
        "ONNX warmup inference done — arena pre-grown before the RT loop.");
    } catch (const Ort::Exception & e) {
      RCLCPP_WARN(get_node()->get_logger(),
        "ONNX warmup inference failed (arena will grow on first RT Run): %s", e.what());
    }
  }

  RCLCPP_INFO(get_node()->get_logger(),
    "HarambePolicyLegsController activated — holding default pose for %d "
    "warmup policy steps before enabling the policy.", warmup_steps_);
  if (dry_run_) {
    RCLCPP_WARN(get_node()->get_logger(),
      "DRY-RUN MODE: the policy will run and log (~/debug) but the motors will "
      "HOLD the default pose — the policy target is NOT sent to the drives.");
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// on_deactivate — hold the default pose with the configured gains.
// ============================================================================
controller_interface::CallbackReturn HarambePolicyLegsController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // Leave the drives LIMP (kp=kd=0) on deactivation — this is the SAFE state.
  // Rationale (learned the hard way): if we left kp>0 here, then in the shutdown
  // window the position command can be corrupted (the interface is released and
  // 0x607A has no PDO `default`, unlike vel/eff/kp/kd) → the still-stiff drive
  // SNAPS the ankle to a garbage position / its limit. Zeroing kp/kd guarantees
  // NO active torque survives deactivation, so a corrupted position can't drive
  // anything; the legs just go gently back-drivable. (vel=0, eff=0 too, so the
  // ankle-linkage driver never transforms a NaN feed-forward.) Position is set to
  // the measured pose for hygiene, but with kp=kd=0 it has no effect.
  // NOTE: limp means a STANDING robot will sag — support it before deactivating.
  for (size_t i = 0; i < num_joints_; ++i) {
    const double q = state_interfaces_[joints_[i].pos_state_idx]
                       .get_optional().value_or(joints_[i].default_pos);
    (void)command_interfaces_[joints_[i].pos_cmd_idx].set_value(q);
    (void)command_interfaces_[joints_[i].vel_cmd_idx].set_value(0.0);
    (void)command_interfaces_[joints_[i].eff_cmd_idx].set_value(0.0);
    (void)command_interfaces_[joints_[i].kp_cmd_idx].set_value(0.0);
    (void)command_interfaces_[joints_[i].kd_cmd_idx].set_value(0.0);
  }
  RCLCPP_INFO(get_node()->get_logger(), "HarambePolicyLegsController deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// update — runs at controller_manager rate; policy inference is decimated.
// ============================================================================
controller_interface::return_type HarambePolicyLegsController::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  enabled_ = *enable_buf_.readFromRT();

  // Policy inference at the decimated rate.
  if (step_counter_ % static_cast<uint64_t>(decimation_) == 0) {
    // IMU calibration capture (one-shot, requested via ~/calibrate_imu). Uses the
    // RAW buffered gravity/gyro so re-calibration is idempotent.
    if (calibrate_pending_.exchange(false)) {
      const auto pelvis = *pelvis_imu_buf_.readFromRT();
      captureImuCalib(pelvis.proj_gravity, pelvis.ang_vel);
    }
    buildObservation();

    // Observation SANITY gate — run BEFORE any other policy logic. A glitched
    // sensor must never reach the network: NaN/Inf in the obs would produce NaN
    // actions and garbage joint targets. Also sanity-check the gravity unit
    // vector magnitude (must be ≈ 1). If invalid → hold default, skip the policy.
    bool obs_valid = true;
    for (float v : obs_) {
      if (!std::isfinite(v)) { obs_valid = false; break; }
    }
    if (obs_valid) {
      const float gx = obs_[kObsProjGravity], gy = obs_[kObsProjGravity + 1],
                  gz0 = obs_[kObsProjGravity + 2];
      const float gmag = std::sqrt(gx * gx + gy * gy + gz0 * gz0);
      if (gmag < 0.5f || gmag > 1.5f) { obs_valid = false; }
    }
    if (enabled_ && !obs_valid) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "OBS INVALID (NaN/Inf or bad gravity magnitude) — holding default pose, "
        "policy will NOT run.");
    }

    // Observation freshness gate: never run the policy on stale/missing sensor
    // data (otherwise the obs falls back to fake "upright, still" seeds and the
    // policy would walk blind). Require a recent IMU msg, and a recent odometry
    // msg when require_odom_.
    const int64_t now_ns = time.nanoseconds();
    const int64_t imu_ns = last_imu_ns_.load();
    const int64_t odom_ns = last_odom_ns_.load();
    const double imu_age = (imu_ns == 0) ? 1e9 : (now_ns - imu_ns) * 1e-9;
    const double odom_age = (odom_ns == 0) ? 1e9 : (now_ns - odom_ns) * 1e-9;
    const bool obs_fresh =
      (imu_age < obs_timeout_) && (!require_odom_ || odom_age < obs_timeout_);
    if (enabled_ && !obs_fresh) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "OBS STALE — holding default pose (IMU age=%.2fs, odom age=%.2fs, "
        "timeout=%.2fs). Policy will NOT run until sensors are fresh.",
        imu_age, odom_age, obs_timeout_);
      obs_stale_warned_ = true;
    } else if (obs_stale_warned_ && obs_fresh) {
      RCLCPP_INFO(get_node()->get_logger(), "OBS fresh again — policy may run.");
      obs_stale_warned_ = false;
    }

    // Fall detection on projected_gravity.z (pelvis up ≈ -1 when upright).
    const float grav_z = obs_[kObsProjGravity + 2];
    if (policy_active_ && !fallen_ && obs_valid && std::abs(grav_z) < fall_threshold_) {
      fallen_ = true;
      RCLCPP_WARN(get_node()->get_logger(),
        "FALL DETECTED at policy step %lu (grav_z=%.3f < %.3f). Holding default pose.",
        policy_step_, grav_z, fall_threshold_);
    }

    // Upright START precondition: the policy ENGAGES only when the pelvis is
    // clearly upright (|proj_grav.z| >= upright_threshold, ≈ within 37° of
    // vertical at 0.8). Once running, the (looser) fall_threshold governs — so a
    // brief tilt while walking does not stop it, but it never STARTS lying down.
    const bool upright = std::abs(grav_z) >= upright_threshold_;
    if (enabled_ && obs_fresh && !fallen_ && !policy_active_ && !upright) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "NOT UPRIGHT — policy will NOT start (|grav_z|=%.2f < %.2f). Holding default.",
        std::abs(grav_z), upright_threshold_);
    }

    const bool warmup_done = static_cast<int>(policy_step_) >= warmup_steps_;
    if (warmup_done && enabled_ && !fallen_ && obs_valid && obs_fresh &&
        (policy_active_ || upright)) {
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
  // Apply the IMU calibration FILTER: rotate by imu_mount_R_ (removes the mounting
  // tilt so level → gravity [0,0,-1]) and subtract gyro_bias_ from the gyro. With
  // the default identity/0 this is a no-op (raw obs).
  const auto pelvis = *pelvis_imu_buf_.readFromRT();
  const auto & R = imu_mount_R_;
  auto rot = [&R](const std::array<double, 3> & v) {
    return std::array<double, 3>{
      R[0] * v[0] + R[1] * v[1] + R[2] * v[2],
      R[3] * v[0] + R[4] * v[1] + R[5] * v[2],
      R[6] * v[0] + R[7] * v[1] + R[8] * v[2]};
  };
  const std::array<double, 3> gyro_unbias = {
    pelvis.ang_vel[0] - gyro_bias_[0],
    pelvis.ang_vel[1] - gyro_bias_[1],
    pelvis.ang_vel[2] - gyro_bias_[2]};
  const auto gyro_c = rot(gyro_unbias);
  const auto grav_c = rot(pelvis.proj_gravity);
  obs_[kObsBaseAngVel + 0] = static_cast<float>(gyro_c[0]);
  obs_[kObsBaseAngVel + 1] = static_cast<float>(gyro_c[1]);
  obs_[kObsBaseAngVel + 2] = static_cast<float>(gyro_c[2]);
  obs_[kObsProjGravity + 0] = static_cast<float>(grav_c[0]);
  obs_[kObsProjGravity + 1] = static_cast<float>(grav_c[1]);
  obs_[kObsProjGravity + 2] = static_cast<float>(grav_c[2]);
  if (fake_imu_) {
    // Diagnostic: replace the real IMU with the sim-like IDEAL (standing, clean):
    // projected_gravity=[0,0,-1], gyro=0. No tilt feedback → cannot balance →
    // run supported / in dry_run.
    obs_[kObsBaseAngVel + 0] = 0.0f;
    obs_[kObsBaseAngVel + 1] = 0.0f;
    obs_[kObsBaseAngVel + 2] = 0.0f;
    obs_[kObsProjGravity + 0] = 0.0f;
    obs_[kObsProjGravity + 1] = 0.0f;
    obs_[kObsProjGravity + 2] = -1.0f;
  }

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
    // dry_run shadow: when the policy is ACTIVE, hold the default pose instead of
    // following the policy target (the policy still ran and is on ~/debug for the
    // CSV; the legs just don't follow it). The WARMUP ramp still runs normally —
    // target_pos_ holds the smooth homing interpolation while policy_active_ is
    // false — so the legs ease into the default pose, they do NOT snap.
    const double pos_cmd = (dry_run_ && policy_active_)
        ? joints_[i].default_pos
        : static_cast<double>(target_pos_[i]);
    (void)command_interfaces_[joints_[i].pos_cmd_idx].set_value(pos_cmd);
    // Zero velocity + effort feed-forward EVERY tick. Pure position-PD policy:
    // the drive must close the loop on position only. Leaving these unwritten
    // (NaN) makes the ankle-linkage driver push NaN motor velocity/torque → the
    // joint slews to its mechanical limit at full speed (broke an eccentric).
    (void)command_interfaces_[joints_[i].vel_cmd_idx].set_value(0.0);
    (void)command_interfaces_[joints_[i].eff_cmd_idx].set_value(0.0);
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
// Capture the IMU calibration filter from a level/still reading.
//   imu_mount_R_ = shortest-arc rotation mapping the measured "down" → [0,0,-1]
//                  (R = c*I + [v]x + (v v^T)/(1+c),  v = a×b,  c = a·b).
//   gyro_bias_   = the current gyro reading (static rate offset).
// ============================================================================
void HarambePolicyLegsController::captureImuCalib(
  const std::array<double, 3> & grav_raw, const std::array<double, 3> & gyro_raw)
{
  const double n = std::sqrt(grav_raw[0] * grav_raw[0] +
                             grav_raw[1] * grav_raw[1] +
                             grav_raw[2] * grav_raw[2]);
  if (n < 1e-6) {
    RCLCPP_WARN(get_node()->get_logger(),
      "calibrate_imu ignored: gravity magnitude ~0 (is the IMU publishing?).");
    return;
  }
  const double a[3] = {grav_raw[0] / n, grav_raw[1] / n, grav_raw[2] / n};
  const double c = -a[2];                       // a · [0,0,-1]
  const double vx = -a[1], vy = a[0], vz = 0.0; // a × [0,0,-1]
  std::array<double, 9> R{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
  if (c < -0.999999) {                          // pointing "up" — won't happen for gravity
    R = {{1, 0, 0, 0, -1, 0, 0, 0, -1}};
  } else if (c <= 0.999999) {                   // general case
    const double k = 1.0 / (1.0 + c);
    R[0] = c + k * vx * vx; R[1] = -vz + k * vx * vy; R[2] = vy + k * vx * vz;
    R[3] = vz + k * vx * vy; R[4] = c + k * vy * vy; R[5] = -vx + k * vy * vz;
    R[6] = -vy + k * vx * vz; R[7] = vx + k * vy * vz; R[8] = c + k * vz * vz;
  }
  imu_mount_R_ = R;
  gyro_bias_ = gyro_raw;
  const double tilt = std::acos(std::max(-1.0, std::min(1.0, -a[2]))) * 180.0 / M_PI;
  RCLCPP_WARN(get_node()->get_logger(),
    "IMU CALIBRATED: leveled %.1f deg tilt (gravity was [%.3f %.3f %.3f]). "
    "To PERSIST, set in config:  imu_mount_R: [%.5f, %.5f, %.5f, %.5f, %.5f, "
    "%.5f, %.5f, %.5f, %.5f]   gyro_bias: [%.5f, %.5f, %.5f]",
    tilt, grav_raw[0], grav_raw[1], grav_raw[2],
    R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8],
    gyro_raw[0], gyro_raw[1], gyro_raw[2]);
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
  if (!rt_debug_pub_) return;
  // trylock(): if the non-RT publish thread is still draining the previous
  // sample, SKIP this one rather than block the 1 kHz loop. Debug telemetry is
  // best-effort — a dropped sample beats a control-cycle overrun.
  if (!rt_debug_pub_->trylock()) return;
  auto & m = rt_debug_pub_->msg_;
  m.data.clear();   // capacity is retained (reserved in on_configure) — no RT alloc
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
  rt_debug_pub_->unlockAndPublish();
}

}  // namespace harambe_policy_legs_controller

PLUGINLIB_EXPORT_CLASS(
  harambe_policy_legs_controller::HarambePolicyLegsController,
  controller_interface::ControllerInterface)
