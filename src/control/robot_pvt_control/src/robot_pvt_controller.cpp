#include "robot_pvt_control/robot_pvt_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <sstream>
#include <utility>

#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "robot_pvt_control/body_group.hpp"
#include "robot_safety/clamp.hpp"

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>

namespace robot_pvt_control
{

namespace
{
// Per-joint command-interface index offsets within the 5-wide drive_side_pd
// stride. update() addresses command_interfaces_[5 * j + kOffset*].
constexpr std::size_t kOffPosition = 0;
constexpr std::size_t kOffVelocity = 1;
constexpr std::size_t kOffEffort   = 2;
constexpr std::size_t kOffKp       = 3;
constexpr std::size_t kOffKd       = 4;
// Sim path: just one effort interface per joint.
constexpr std::size_t kSimOffEffort = 0;

constexpr std::size_t kSimStride = 1;
constexpr std::size_t kDriveStride = 5;

constexpr std::size_t kStatePerJoint = 2;   // position + velocity

/// Resize a vector-of-doubles parameter to N, padding from the last element
/// (or 0 if empty) — lets a yaml supply a scalar that applies to every joint.
void normalize(std::vector<double> & v, std::size_t n, double fill = 0.0)
{
  if (v.empty()) {
    v.assign(n, fill);
    return;
  }
  if (v.size() == 1) {
    v.assign(n, v.front());
    return;
  }
  if (v.size() < n) {
    v.resize(n, v.back());
  } else if (v.size() > n) {
    v.resize(n);
  }
}
void normalize(std::vector<bool> & v, std::size_t n, bool fill = true)
{
  if (v.empty()) {
    v.assign(n, fill);
    return;
  }
  if (v.size() == 1) {
    v.assign(n, v.front());
    return;
  }
  if (v.size() < n) {
    v.resize(n, v.back());
  } else if (v.size() > n) {
    v.resize(n);
  }
}

}  // namespace

controller_interface::CallbackReturn RobotPVTController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
    auto_declare<std::vector<double>>("Kp", std::vector<double>{});
    auto_declare<std::vector<double>>("Kd", std::vector<double>{});
    auto_declare<std::vector<double>>("Kd_damp", std::vector<double>{});
    auto_declare<std::vector<double>>("mgl", std::vector<double>{});
    auto_declare<std::vector<double>>("q_eq", std::vector<double>{});
    auto_declare<std::vector<double>>("J", std::vector<double>{});
    auto_declare<std::vector<double>>("Fv", std::vector<double>{});
    auto_declare<std::vector<double>>("comp_sign", std::vector<double>{});
    auto_declare<std::vector<bool>>("ff_gravity", std::vector<bool>{});
    auto_declare<std::vector<bool>>("ff_inertia", std::vector<bool>{});
    auto_declare<std::vector<bool>>("ff_viscous", std::vector<bool>{});
    auto_declare<std::vector<double>>("hold_position", std::vector<double>{});
    auto_declare<bool>("drive_side_pd", true);
    auto_declare<std::string>("body_group", std::string{"full"});
    auto_declare<double>("lag_free", 0.04);
    auto_declare<double>("lag_pause", 0.14);
    auto_declare<double>("alpha_slew", 2.0);
    // robot_description is normally injected by the controller_manager from
    // its own `robot_description` parameter, but declaring it here makes the
    // dependency explicit and gives us a useful fallback to empty string.
    auto_declare<std::string>("robot_description", "");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

void RobotPVTController::load_params()
{
  auto node = get_node();
  // Joint roster
  joints_ = node->get_parameter("joints").as_string_array();
  num_joints_ = joints_.size();

  params_.Kp            = node->get_parameter("Kp").as_double_array();
  params_.Kd            = node->get_parameter("Kd").as_double_array();
  params_.Kd_damp       = node->get_parameter("Kd_damp").as_double_array();
  params_.mgl           = node->get_parameter("mgl").as_double_array();
  params_.q_eq          = node->get_parameter("q_eq").as_double_array();
  params_.J             = node->get_parameter("J").as_double_array();
  params_.Fv            = node->get_parameter("Fv").as_double_array();
  params_.comp_sign     = node->get_parameter("comp_sign").as_double_array();
  params_.ff_gravity    = node->get_parameter("ff_gravity").as_bool_array();
  params_.ff_inertia    = node->get_parameter("ff_inertia").as_bool_array();
  params_.ff_viscous    = node->get_parameter("ff_viscous").as_bool_array();
  params_.hold_position = node->get_parameter("hold_position").as_double_array();
  params_.drive_side_pd = node->get_parameter("drive_side_pd").as_bool();
  params_.lag_free      = node->get_parameter("lag_free").as_double();
  params_.lag_pause     = node->get_parameter("lag_pause").as_double();
  params_.alpha_slew    = node->get_parameter("alpha_slew").as_double();

  // Pad / truncate per-joint arrays so every accessor is bounded.
  normalize(params_.Kp, num_joints_, 0.0);
  normalize(params_.Kd, num_joints_, 0.0);
  normalize(params_.Kd_damp, num_joints_, 0.0);
  normalize(params_.mgl, num_joints_, 0.0);
  normalize(params_.q_eq, num_joints_, 0.0);
  normalize(params_.J, num_joints_, 0.0);
  normalize(params_.Fv, num_joints_, 0.0);
  normalize(params_.comp_sign, num_joints_, 1.0);
  normalize(params_.ff_gravity, num_joints_, false);
  normalize(params_.ff_inertia, num_joints_, false);
  normalize(params_.ff_viscous, num_joints_, false);
  normalize(params_.hold_position, num_joints_, 0.0);
}

controller_interface::CallbackReturn RobotPVTController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  load_params();
  params_buf_.writeFromNonRT(params_);

  if (joints_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "robot_pvt_controller: parameter 'joints' is empty — refusing to start.");
    return controller_interface::CallbackReturn::ERROR;
  }
  joint_index_.clear();
  for (std::size_t i = 0; i < num_joints_; ++i) {
    joint_index_[joints_[i]] = i;
  }

  drive_side_pd_ = params_.drive_side_pd;
  cmd_per_joint_ = drive_side_pd_ ? kDriveStride : kSimStride;

  // body_group is latched here for the lifetime of this configure session.
  // Runtime changes are surfaced as warnings in load_params() and only take
  // effect after a reconfigure (operator deactivates → configures again).
  const std::string requested_group =
    get_node()->get_parameter("body_group").as_string();
  if (!is_valid_body_group(requested_group)) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "robot_pvt_controller: body_group='%s' is not one of "
      "{arms, arms_waist, legs, full} — refusing to start.",
      requested_group.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  body_group_ = requested_group;
  active_mask_.assign(num_joints_, false);
  std::size_t active_count = 0;
  std::ostringstream active_names;
  for (std::size_t j = 0; j < num_joints_; ++j) {
    active_mask_[j] = joint_in_group(joints_[j], body_group_);
    if (active_mask_[j]) {
      ++active_count;
      if (active_count > 1) active_names << ", ";
      active_names << joints_[j];
    }
  }
  RCLCPP_INFO(get_node()->get_logger(),
    "robot_pvt_controller: body_group=%s, active=%zu/%zu joints: [%s]",
    body_group_.c_str(), active_count, num_joints_,
    active_names.str().c_str());

  // Centralised safety: load shared limits and subscribe to the supervisor.
  limits_buf_.writeFromNonRT(
    robot_safety::loadSafetyLimits(*get_node(), joints_, "safety."));
  safety_.subscribe(get_node(), num_joints_);

  // Preallocate per-joint working state.
  rate_limiters_.clear();
  rate_limiters_.resize(num_joints_);
  estop_hold_pos_.assign(num_joints_, 0.0);
  resume_hold_pos_.assign(num_joints_, 0.0);
  dq_sign_.assign(num_joints_, 1.0);
  tmp_p_.assign(num_joints_, 0.0);
  tmp_v_.assign(num_joints_, 0.0);
  tmp_a_.assign(num_joints_, 0.0);

  Setpoint init;
  init.position = params_.hold_position;
  init.velocity.assign(num_joints_, 0.0);
  init.acceleration.assign(num_joints_, 0.0);
  setpoint_buf_.writeFromNonRT(init);

  auto node = get_node();
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.ignore_local_publications = true;
  setpoint_sub_ =
    node->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
      "~/setpoint", rclcpp::SystemDefaultsQoS(),
      std::bind(&RobotPVTController::setpoint_callback, this,
        std::placeholders::_1),
      sub_opts);

  hold_srv_ = node->create_service<std_srvs::srv::Trigger>(
    "~/hold",
    std::bind(&RobotPVTController::hold_service, this,
      std::placeholders::_1, std::placeholders::_2));
  free_srv_ = node->create_service<std_srvs::srv::Trigger>(
    "~/free",
    std::bind(&RobotPVTController::free_service, this,
      std::placeholders::_1, std::placeholders::_2));
  damp_srv_ = node->create_service<std_srvs::srv::Trigger>(
    "~/damp",
    std::bind(&RobotPVTController::damp_service, this,
      std::placeholders::_1, std::placeholders::_2));

  // Action server.
  traj_buf_.writeFromNonRT(nullptr);
  action_server_ = rclcpp_action::create_server<FJT>(
    node->get_node_base_interface(),
    node->get_node_clock_interface(),
    node->get_node_logging_interface(),
    node->get_node_waitables_interface(),
    "~/follow_joint_trajectory",
    [this](const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const FJT::Goal> goal) {
      return this->handle_goal(uuid, goal);
    },
    [this](const std::shared_ptr<GoalHandleFJT> gh) {
      return this->handle_cancel(gh);
    },
    [this](const std::shared_ptr<GoalHandleFJT> gh) {
      this->handle_accepted(gh);
    });

  feedback_timer_ = node->create_wall_timer(
    std::chrono::milliseconds(100),
    [this]() { this->on_feedback_tick(); });

  // ── Pinocchio gravity model ────────────────────────────────────────────
  // Build once from the URDF. Each update() will call computeGeneralizedGravity
  // with the live q vector and use the result as the per-joint FF term. If the
  // URDF parameter is empty or fails to parse, fall back to the static
  // mgl·sin(q−q_eq) path so a misconfigured launch can still run on the bench.
  const std::string urdf_xml =
    node->get_parameter("robot_description").as_string();
  pin_gravity_available_ = false;
  if (urdf_xml.empty()) {
    RCLCPP_WARN(node->get_logger(),
      "robot_pvt_controller: robot_description is empty — runtime gravity FF "
      "disabled; falling back to mgl·sin(q−q_eq).");
  } else {
    try {
      pin_model_ = std::make_unique<pinocchio::Model>();
      pinocchio::urdf::buildModelFromXML(urdf_xml, *pin_model_);
      pin_data_ = std::make_unique<pinocchio::Data>(*pin_model_);
      q_pin_ = Eigen::VectorXd::Zero(pin_model_->nq);

      pin_q_idx_of_j_.assign(num_joints_, -1);
      pin_v_idx_of_j_.assign(num_joints_, -1);
      std::vector<std::string> missing;
      for (std::size_t j = 0; j < num_joints_; ++j) {
        if (!pin_model_->existJointName(joints_[j])) {
          missing.push_back(joints_[j]);
          continue;
        }
        const auto jid = pin_model_->getJointId(joints_[j]);
        pin_q_idx_of_j_[j] = pin_model_->joints[jid].idx_q();
        pin_v_idx_of_j_[j] = pin_model_->joints[jid].idx_v();
      }
      if (!missing.empty()) {
        std::ostringstream oss;
        for (std::size_t i = 0; i < missing.size(); ++i) {
          if (i) oss << ", ";
          oss << missing[i];
        }
        RCLCPP_ERROR(node->get_logger(),
          "robot_pvt_controller: %zu joint(s) from `joints` not present in URDF: "
          "[%s] — refusing to start.", missing.size(), oss.str().c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      pin_gravity_available_ = true;
      RCLCPP_INFO(node->get_logger(),
        "robot_pvt_controller: pinocchio model loaded (nq=%d, nv=%d); runtime "
        "gravity FF active over %zu controlled joints.",
        pin_model_->nq, pin_model_->nv, num_joints_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node->get_logger(),
        "robot_pvt_controller: pinocchio failed to parse robot_description "
        "(%s) — falling back to mgl·sin(q−q_eq).", e.what());
      pin_model_.reset();
      pin_data_.reset();
      pin_gravity_available_ = false;
    }
  }

  post_set_param_cb_ = node->add_post_set_parameters_callback(
    [this, node](const std::vector<rclcpp::Parameter> & /*params*/) {
      load_params();
      params_buf_.writeFromNonRT(params_);
      // Re-load safety limits too so a live `ros2 param set
      // safety.joints.<j>.<field> ...` from the GUI takes effect at the
      // next update() without restarting the controller. Matches the
      // pendulum_pvt_control behaviour the tuner was written against.
      // Goes through the realtime buffer so update() reads a complete
      // snapshot rather than a half-rebuilt unordered_map.
      limits_buf_.writeFromNonRT(
        robot_safety::loadSafetyLimits(*node, joints_, "safety."));
    });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
RobotPVTController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  if (drive_side_pd_) {
    cfg.names.reserve(num_joints_ * kDriveStride);
    for (const auto & j : joints_) {
      cfg.names.push_back(j + "/position");
      cfg.names.push_back(j + "/velocity");
      cfg.names.push_back(j + "/effort");
      cfg.names.push_back(j + "/kp");
      cfg.names.push_back(j + "/kd");
    }
  } else {
    cfg.names.reserve(num_joints_ * kSimStride);
    for (const auto & j : joints_) {
      cfg.names.push_back(j + "/effort");
    }
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
RobotPVTController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.reserve(num_joints_ * kStatePerJoint);
  for (const auto & j : joints_) {
    cfg.names.push_back(j + "/position");
    cfg.names.push_back(j + "/velocity");
  }
  return cfg;
}

controller_interface::CallbackReturn RobotPVTController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  load_params();
  params_buf_.writeFromNonRT(params_);
  // Default to FREE on activation so a stale setpoint cannot kick a joint.
  mode_.store(Mode::FREE);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotPVTController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  preempt_goal("controller deactivating");
  write_free_outputs();
  return controller_interface::CallbackReturn::SUCCESS;
}

void RobotPVTController::write_free_outputs_for(std::size_t j)
{
  if (command_interfaces_.empty()) {
    return;
  }
  if (drive_side_pd_) {
    double q = 0.0;
    const auto pos_opt = state_interfaces_[kStatePerJoint * j].get_optional();
    if (pos_opt) {
      q = pos_opt.value();
    }
    (void)command_interfaces_[kDriveStride * j + kOffPosition].set_value(q);
    (void)command_interfaces_[kDriveStride * j + kOffVelocity].set_value(0.0);
    (void)command_interfaces_[kDriveStride * j + kOffEffort].set_value(0.0);
    (void)command_interfaces_[kDriveStride * j + kOffKp].set_value(0.0);
    (void)command_interfaces_[kDriveStride * j + kOffKd].set_value(0.0);
  } else {
    (void)command_interfaces_[kSimStride * j + kSimOffEffort].set_value(0.0);
  }
}

void RobotPVTController::write_damping_outputs_for(
  std::size_t j, double qd, const robot_safety::SafetyLimits & L)
{
  if (command_interfaces_.empty()) {
    return;
  }
  if (drive_side_pd_) {
    // Drive's MIT/PVT law collapses to tau = 0*(q_des-q) + Kd_damp*(0 - qd)
    // = -Kd_damp * qd. q_des is pinned to measured to keep the position
    // error term zero even if Kp drifts above 0 due to ordering effects.
    double q = 0.0;
    const auto pos_opt = state_interfaces_[kStatePerJoint * j].get_optional();
    if (pos_opt) {
      q = pos_opt.value();
    }
    (void)command_interfaces_[kDriveStride * j + kOffPosition].set_value(q);
    (void)command_interfaces_[kDriveStride * j + kOffVelocity].set_value(0.0);
    (void)command_interfaces_[kDriveStride * j + kOffEffort].set_value(0.0);
    (void)command_interfaces_[kDriveStride * j + kOffKp].set_value(0.0);
    (void)command_interfaces_[kDriveStride * j + kOffKd].set_value(params_.Kd_damp[j]);
  } else {
    // Sim path has no drive-side PD — compute the brake torque host-side
    // and clamp to the joint's effort envelope so the GUI's safety panel
    // still bounds output.
    const double tau = robot_safety::clampEffort(-params_.Kd_damp[j] * qd, L);
    (void)command_interfaces_[kSimStride * j + kSimOffEffort].set_value(tau);
  }
}

void RobotPVTController::write_free_outputs()
{
  if (command_interfaces_.empty()) {
    return;
  }
  if (drive_side_pd_) {
    // For every joint write {q, 0, 0, 0, 0} so the drive's law collapses to
    // zero torque while pinning the held position to wherever the joint is.
    for (std::size_t j = 0; j < num_joints_; ++j) {
      double q = 0.0;
      const auto pos_opt = state_interfaces_[kStatePerJoint * j].get_optional();
      if (pos_opt) {
        q = pos_opt.value();
      }
      (void)command_interfaces_[kDriveStride * j + kOffPosition].set_value(q);
      (void)command_interfaces_[kDriveStride * j + kOffVelocity].set_value(0.0);
      (void)command_interfaces_[kDriveStride * j + kOffEffort].set_value(0.0);
      (void)command_interfaces_[kDriveStride * j + kOffKp].set_value(0.0);
      (void)command_interfaces_[kDriveStride * j + kOffKd].set_value(0.0);
    }
  } else {
    for (std::size_t j = 0; j < num_joints_; ++j) {
      (void)command_interfaces_[kSimStride * j + kSimOffEffort].set_value(0.0);
    }
  }
}

controller_interface::return_type RobotPVTController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  params_ = *params_buf_.readFromRT();
  const auto & limits = *limits_buf_.readFromRT();

  const robot_safety::SafetySignal safety = safety_.get();
  const auto mode = mode_.load();
  const Setpoint sp = *setpoint_buf_.readFromRT();

  const double dt = period.seconds();

  // Snapshot per-joint q / qd.
  std::vector<double> q(num_joints_, 0.0);
  std::vector<double> qd(num_joints_, 0.0);
  bool state_ok = true;
  for (std::size_t j = 0; j < num_joints_; ++j) {
    const auto pos_opt = state_interfaces_[kStatePerJoint * j].get_optional();
    const auto vel_opt = state_interfaces_[kStatePerJoint * j + 1].get_optional();
    if (!pos_opt || !vel_opt) {
      state_ok = false;
      break;
    }
    q[j] = pos_opt.value();
    qd[j] = vel_opt.value();
  }
  if (!state_ok) {
    return controller_interface::return_type::OK;
  }

  // First cycle of an e-stop: pick the per-joint HOLD targets.
  if (safety.estop_active && !estop_was_active_) {
    if (safety.action == robot_safety::EstopAction::HOLD) {
      for (std::size_t j = 0; j < num_joints_; ++j) {
        const auto & L = limits.limitsFor(joints_[j]);
        const double accel = L.acceleration_limit > 0.0 ? L.acceleration_limit : 60.0;
        double v0 = qd[j];
        if (L.slew_rate_limit > 0.0) {
          v0 = std::clamp(qd[j], -L.slew_rate_limit, L.slew_rate_limit);
        }
        const double brake = (v0 * v0) / (2.0 * accel);
        estop_hold_pos_[j] = robot_safety::clampPosition(
          q[j] + std::copysign(brake, v0), L);
        rate_limiters_[j].seed(q[j], v0);
      }
    } else {
      for (std::size_t j = 0; j < num_joints_; ++j) {
        estop_hold_pos_[j] = robot_safety::clampPosition(
          q[j], limits.limitsFor(joints_[j]));
      }
    }
  }
  // Falling edge — e-stop just cleared.
  if (!safety.estop_active && estop_was_active_) {
    for (std::size_t j = 0; j < num_joints_; ++j) {
      resume_hold_pos_[j] = robot_safety::clampPosition(
        q[j], limits.limitsFor(joints_[j]));
      rate_limiters_[j].seed(q[j]);
    }
    waiting_for_setpoint_.store(true);
    traj_buf_.writeFromNonRT(nullptr);
    traj_done_.store(true);
  }
  estop_was_active_ = safety.estop_active;

  const bool estop_free =
    safety.estop_active && safety.action == robot_safety::EstopAction::FREE;
  if (estop_free || (mode == Mode::FREE && !safety.estop_active)) {
    for (auto & r : rate_limiters_) {
      r.reset();
    }
    write_free_outputs();
    return controller_interface::return_type::OK;
  }

  // DAMPING — controller-internal mode, no e-stop equivalent (the supervisor's
  // EstopAction enum only covers FREE/HOLD). Each active joint runs the brake
  // pattern; inactive joints in the body group stay back-drivable (FREE).
  if (mode == Mode::DAMPING && !safety.estop_active) {
    for (auto & r : rate_limiters_) {
      r.reset();
    }
    for (std::size_t j = 0; j < num_joints_; ++j) {
      if (active_mask_[j]) {
        write_damping_outputs_for(j, qd[j], limits.limitsFor(joints_[j]));
      } else {
        write_free_outputs_for(j);
      }
    }
    return controller_interface::return_type::OK;
  }

  // Resolve the per-joint reference {ref_pos[j], ref_vel[j], ref_acc[j]}.
  std::vector<double> ref_pos(num_joints_, 0.0);
  std::vector<double> ref_vel(num_joints_, 0.0);
  std::vector<double> ref_acc(num_joints_, 0.0);

  if (safety.estop_active) {
    for (std::size_t j = 0; j < num_joints_; ++j) {
      ref_pos[j] = estop_hold_pos_[j];
    }
  } else if (waiting_for_setpoint_.load()) {
    for (std::size_t j = 0; j < num_joints_; ++j) {
      ref_pos[j] = resume_hold_pos_[j];
    }
  } else if (auto traj_ptr = *traj_buf_.readFromRT(); traj_ptr) {
    if (!traj_done_.load()) {
      // Sample at the current tv_ first; the lag governor reads the lead
      // before we advance.
      robot_pvt_control::sample_trajectory(
        *traj_ptr, tv_, tmp_p_, tmp_v_, tmp_a_);

      // Whole-body lag: worst active joint's lead in its travel direction.
      // Skip joints outside the body_group — they're back-drivable, will
      // always have a huge "lead", and would falsely trip the pause.
      double max_lead = -std::numeric_limits<double>::infinity();
      for (std::size_t j = 0; j < num_joints_; ++j) {
        if (!active_mask_[j]) continue;
        const double lead = (tmp_p_[j] - q[j]) * dq_sign_[j];
        if (lead > max_lead) {
          max_lead = lead;
        }
      }
      // No active joints? alpha_target=1 (no lag pressure).
      if (!std::isfinite(max_lead)) max_lead = 0.0;
      const double denom = std::max(params_.lag_pause - params_.lag_free, 1e-6);
      double alpha_target = (params_.lag_pause - max_lead) / denom;
      alpha_target = std::clamp(alpha_target, 0.0, 1.0);
      const double max_step = params_.alpha_slew * dt;
      alpha_ += std::clamp(alpha_target - alpha_, -max_step, +max_step);
      alpha_ = std::clamp(alpha_, 0.0, 1.0);

      tv_ = std::min(tv_ + alpha_ * dt, traj_ptr->duration);
      for (std::size_t j = 0; j < num_joints_; ++j) {
        ref_pos[j] = tmp_p_[j];
        ref_vel[j] = tmp_v_[j] * alpha_;
        ref_acc[j] = tmp_a_[j] * alpha_ * alpha_;
      }
      if (tv_ >= traj_ptr->duration) {
        traj_completed_clean_.store(true);
        traj_done_.store(true);
      }
    } else {
      // Trajectory finished but feedback timer has not finalised yet — hold
      // the endpoint so a stale legacy setpoint doesn't snap the joints back.
      for (std::size_t j = 0; j < num_joints_; ++j) {
        ref_pos[j] = traj_ptr->knots.back().pos[j];
      }
    }
  } else {
    // Legacy streaming setpoint
    for (std::size_t j = 0; j < num_joints_; ++j) {
      ref_pos[j] = (j < sp.position.size()) ? sp.position[j] :
        params_.hold_position[j];
      ref_vel[j] = (j < sp.velocity.size()) ? sp.velocity[j] : 0.0;
      ref_acc[j] = (j < sp.acceleration.size()) ? sp.acceleration[j] : 0.0;
    }
  }

  // Runtime gravity feedforward: one O(n) RNEA pass for the whole chain.
  // pin_data_->g[v_idx] is the actuator torque required to hold the joint
  // static against gravity at the current q — replaces mgl·sin(q−q_eq).
  // Joints not in the controller roster (e.g. the inspire hand fingers) stay
  // at q=0 in the Pinocchio config, which matches the hand mounts' frozen
  // reference pose used by the URDF.
  if (pin_gravity_available_) {
    for (std::size_t j = 0; j < num_joints_; ++j) {
      q_pin_[pin_q_idx_of_j_[j]] = q[j];
    }
    pinocchio::computeGeneralizedGravity(*pin_model_, *pin_data_, q_pin_);
  }

  // Per-joint slew/accel limiting + clamps + drive write.
  for (std::size_t j = 0; j < num_joints_; ++j) {
    // Inactive joints (outside the body_group scope) stay back-drivable:
    // write the FREE pattern so the drive's PD law evaluates to zero.
    if (!active_mask_[j]) {
      write_free_outputs_for(j);
      continue;
    }
    const auto & L = limits.limitsFor(joints_[j]);
    if (!rate_limiters_[j].seeded()) {
      rate_limiters_[j].seed(q[j]);
    }
    double p_cmd = rate_limiters_[j].limit(
      ref_pos[j], dt, L.slew_rate_limit, L.acceleration_limit);
    p_cmd = robot_safety::clampPosition(p_cmd, L);
    const double v_cmd = robot_safety::clampVelocity(ref_vel[j], L);

    // Gravity FF: prefer runtime Pinocchio RNEA (true chain physics).
    // Fall back to the static mgl·sin(q−q_eq) pendulum approximation only
    // when the URDF wasn't loaded (empty robot_description, parse failure).
    const double tau_g_raw = pin_gravity_available_
      ? pin_data_->g[pin_v_idx_of_j_[j]]
      : params_.mgl[j] * std::sin(q[j] - params_.q_eq[j]);
    const double tau_g = params_.ff_gravity[j] ? tau_g_raw : 0.0;
    const double tau_J = params_.ff_inertia[j] ?
      params_.J[j] * ref_acc[j] : 0.0;
    const double tau_v = params_.ff_viscous[j] ?
      params_.Fv[j] * qd[j] : 0.0;
    const double tau_ff = params_.comp_sign[j] * (tau_g + tau_J + tau_v);

    const double kp_eff = params_.Kp[j] * safety_.kpScale(j);

    if (drive_side_pd_) {
      (void)command_interfaces_[kDriveStride * j + kOffPosition].set_value(p_cmd);
      (void)command_interfaces_[kDriveStride * j + kOffVelocity].set_value(v_cmd);
      (void)command_interfaces_[kDriveStride * j + kOffEffort].set_value(
        robot_safety::clampEffort(tau_ff, L));
      (void)command_interfaces_[kDriveStride * j + kOffKp].set_value(kp_eff);
      (void)command_interfaces_[kDriveStride * j + kOffKd].set_value(params_.Kd[j]);
    } else {
      const double tau_pd = kp_eff * (p_cmd - q[j]) +
        params_.Kd[j] * (v_cmd - qd[j]);
      const double tau = robot_safety::clampEffort(tau_ff + tau_pd, L);
      (void)command_interfaces_[kSimStride * j + kSimOffEffort].set_value(tau);
    }
  }
  return controller_interface::return_type::OK;
}

void RobotPVTController::setpoint_callback(
  const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg)
{
  preempt_goal("preempted by direct ~/setpoint");
  Setpoint sp;
  sp.position = params_.hold_position;
  sp.velocity.assign(num_joints_, 0.0);
  sp.acceleration.assign(num_joints_, 0.0);
  const std::size_t np = std::min(num_joints_, msg->positions.size());
  for (std::size_t i = 0; i < np; ++i) {
    sp.position[i] = msg->positions[i];
  }
  const std::size_t nv = std::min(num_joints_, msg->velocities.size());
  for (std::size_t i = 0; i < nv; ++i) {
    sp.velocity[i] = msg->velocities[i];
  }
  const std::size_t na = std::min(num_joints_, msg->accelerations.size());
  for (std::size_t i = 0; i < na; ++i) {
    sp.acceleration[i] = msg->accelerations[i];
  }
  setpoint_buf_.writeFromNonRT(std::move(sp));
  mode_.store(Mode::PVT);
  waiting_for_setpoint_.store(false);
}

void RobotPVTController::hold_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  preempt_goal("preempted by ~/hold");
  Setpoint sp;
  sp.position.assign(num_joints_, 0.0);
  sp.velocity.assign(num_joints_, 0.0);
  sp.acceleration.assign(num_joints_, 0.0);
  for (std::size_t j = 0; j < num_joints_; ++j) {
    const auto pos_opt = state_interfaces_[kStatePerJoint * j].get_optional();
    sp.position[j] = pos_opt ? pos_opt.value() : params_.hold_position[j];
  }
  setpoint_buf_.writeFromNonRT(std::move(sp));
  mode_.store(Mode::PVT);
  waiting_for_setpoint_.store(false);
  response->success = true;
  response->message = "Holding at measured positions across all joints";
}

void RobotPVTController::free_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  preempt_goal("preempted by ~/free");
  mode_.store(Mode::FREE);
  response->success = true;
  response->message = "FREE mode — zero drive torque";
}

void RobotPVTController::damp_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  // DAMPING has no reference position — the brake settles wherever the joint
  // is when the operator releases it. Cancel any in-flight action so it
  // cannot resume PVT tracking under the brake.
  preempt_goal("preempted by ~/damp");
  mode_.store(Mode::DAMPING);
  std::size_t active_count = 0;
  for (bool a : active_mask_) {
    if (a) ++active_count;
  }
  response->success = true;
  response->message = "DAMPING — body_group='" + body_group_ + "' (" +
    std::to_string(active_count) + "/" + std::to_string(num_joints_) +
    " joints), tau = -Kd_damp[j] * qd";
}

void RobotPVTController::preempt_goal(const std::string & why)
{
  traj_buf_.writeFromNonRT(nullptr);
  traj_done_.store(false);
  traj_completed_clean_.store(false);
  if (active_goal_) {
    auto result = std::make_shared<FJT::Result>();
    result->error_code = FJT::Result::INVALID_GOAL;
    result->error_string = why;
    if (active_goal_->is_active()) {
      active_goal_->abort(result);
    }
    active_goal_.reset();
  }
}

rclcpp_action::GoalResponse RobotPVTController::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const FJT::Goal> goal)
{
  if (get_lifecycle_state().id() !=
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_WARN(get_node()->get_logger(),
      "FJT goal rejected: controller is not ACTIVE");
    return rclcpp_action::GoalResponse::REJECT;
  }
  const auto & names = goal->trajectory.joint_names;
  // Every requested joint name must be in our roster (subset OK — unspecified
  // joints will hold their current position via the implicit start knot below).
  for (const auto & n : names) {
    if (joint_index_.find(n) == joint_index_.end()) {
      RCLCPP_WARN(get_node()->get_logger(),
        "FJT goal rejected: joint '%s' is not in the controller's roster",
        n.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  const auto & points = goal->trajectory.points;
  if (points.empty()) {
    RCLCPP_WARN(get_node()->get_logger(),
      "FJT goal rejected: empty trajectory");
    return rclcpp_action::GoalResponse::REJECT;
  }
  double prev_t = -1.0;
  for (const auto & pt : points) {
    const double t = rclcpp::Duration(pt.time_from_start).seconds();
    if (t <= prev_t) {
      RCLCPP_WARN(get_node()->get_logger(),
        "FJT goal rejected: non-monotonic time_from_start");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (pt.positions.size() != names.size()) {
      RCLCPP_WARN(get_node()->get_logger(),
        "FJT goal rejected: positions.size() != joint_names.size()");
      return rclcpp_action::GoalResponse::REJECT;
    }
    prev_t = t;
  }
  if (prev_t <= 0.0) {
    RCLCPP_WARN(get_node()->get_logger(),
      "FJT goal rejected: total duration must be > 0");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotPVTController::handle_cancel(
  const std::shared_ptr<GoalHandleFJT> /*goal_handle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotPVTController::handle_accepted(
  const std::shared_ptr<GoalHandleFJT> goal_handle)
{
  if (active_goal_) {
    preempt_goal("preempted by new goal");
  }

  // Snapshot the current joint positions for the implicit start knot and for
  // any joints the trajectory doesn't reference (those will be held at q_start).
  std::vector<double> q_start = params_.hold_position;
  for (std::size_t j = 0; j < num_joints_; ++j) {
    const auto pos_opt = state_interfaces_[kStatePerJoint * j].get_optional();
    if (pos_opt) {
      q_start[j] = pos_opt.value();
    }
  }

  const auto & goal = goal_handle->get_goal();
  const auto & names = goal->trajectory.joint_names;
  const auto & points = goal->trajectory.points;

  // Build the goal's joint index → our internal joint index mapping.
  std::vector<int> goal_to_internal(names.size(), -1);
  for (std::size_t k = 0; k < names.size(); ++k) {
    const auto it = joint_index_.find(names[k]);
    goal_to_internal[k] = (it != joint_index_.end()) ?
      static_cast<int>(it->second) : -1;
  }

  auto traj = std::make_shared<Trajectory>();
  const double first_t = rclcpp::Duration(points.front().time_from_start).seconds();

  // Implicit start knot — every joint at its current position, vel = 0.
  if (first_t > 0.0) {
    Knot k0;
    k0.t = 0.0;
    k0.pos = q_start;
    k0.vel.assign(num_joints_, 0.0);
    k0.acc.assign(num_joints_, 0.0);
    k0.has_acc = false;
    traj->knots.push_back(std::move(k0));
  }

  for (const auto & pt : points) {
    Knot k;
    k.t = rclcpp::Duration(pt.time_from_start).seconds();
    // Default every joint to its q_start; the trajectory then only modifies
    // joints it explicitly names — others hold.
    k.pos = q_start;
    k.vel.assign(num_joints_, 0.0);
    k.acc.assign(num_joints_, 0.0);
    k.has_acc = !pt.accelerations.empty();
    for (std::size_t gi = 0; gi < goal_to_internal.size(); ++gi) {
      const int j = goal_to_internal[gi];
      if (j < 0) {
        continue;
      }
      const std::size_t ji = static_cast<std::size_t>(j);
      k.pos[ji] = pt.positions[gi];
      if (gi < pt.velocities.size()) {
        k.vel[ji] = pt.velocities[gi];
      }
      if (gi < pt.accelerations.size()) {
        k.acc[ji] = pt.accelerations[gi];
      }
    }
    traj->knots.push_back(std::move(k));
  }
  traj->duration = traj->knots.back().t;

  // Reset governor BEFORE publishing traj — RT must never see a fresh traj
  // with stale tv_/alpha_/dq_sign_.
  tv_ = 0.0;
  alpha_ = 1.0;
  for (std::size_t j = 0; j < num_joints_; ++j) {
    dq_sign_[j] =
      (traj->knots.back().pos[j] >= traj->knots.front().pos[j]) ? 1.0 : -1.0;
  }
  traj_done_.store(false);
  traj_completed_clean_.store(false);

  traj_buf_.writeFromNonRT(std::shared_ptr<const Trajectory>(std::move(traj)));
  mode_.store(Mode::PVT);
  waiting_for_setpoint_.store(false);
  active_goal_ = goal_handle;
}

void RobotPVTController::on_feedback_tick()
{
  if (!active_goal_) {
    return;
  }

  if (active_goal_->is_canceling()) {
    std::vector<double> measured(num_joints_, 0.0);
    for (std::size_t j = 0; j < num_joints_; ++j) {
      const auto pos_opt = state_interfaces_[kStatePerJoint * j].get_optional();
      measured[j] = pos_opt ? pos_opt.value() : params_.hold_position[j];
    }
    promote_to_setpoint(measured);
    traj_buf_.writeFromNonRT(nullptr);
    traj_done_.store(false);
    traj_completed_clean_.store(false);
    auto result = std::make_shared<FJT::Result>();
    result->error_code = FJT::Result::SUCCESSFUL;
    result->error_string = "canceled";
    active_goal_->canceled(result);
    active_goal_.reset();
    return;
  }

  // Light feedback: just publish actuals + reference snapshot collated from
  // measured state. Per-cycle interpolated sample isn't broadcast here; the RT
  // path writes it directly to the drives.
  auto feedback = std::make_shared<FJT::Feedback>();
  feedback->joint_names = joints_;
  feedback->actual.positions.resize(num_joints_);
  for (std::size_t j = 0; j < num_joints_; ++j) {
    const auto pos_opt = state_interfaces_[kStatePerJoint * j].get_optional();
    feedback->actual.positions[j] = pos_opt ? pos_opt.value() : 0.0;
  }
  active_goal_->publish_feedback(feedback);

  if (traj_done_.load()) {
    if (traj_completed_clean_.load()) {
      // Success: hold the commanded endpoint so FJT semantics are preserved
      // and the rate limiter can close any residual cleanly.
      auto traj_ptr = *traj_buf_.readFromNonRT();
      if (traj_ptr && !traj_ptr->knots.empty()) {
        promote_to_setpoint(traj_ptr->knots.back().pos);
      }
    } else {
      // Abort (e-stop/external preempt): hold-in-place at measured position.
      std::vector<double> measured(num_joints_, 0.0);
      for (std::size_t j = 0; j < num_joints_; ++j) {
        const auto pos_opt = state_interfaces_[kStatePerJoint * j].get_optional();
        measured[j] = pos_opt ? pos_opt.value() : params_.hold_position[j];
      }
      promote_to_setpoint(measured);
    }
    traj_buf_.writeFromNonRT(nullptr);
    auto result = std::make_shared<FJT::Result>();
    if (traj_completed_clean_.load()) {
      result->error_code = FJT::Result::SUCCESSFUL;
      result->error_string = "trajectory completed";
      active_goal_->succeed(result);
    } else {
      result->error_code = FJT::Result::INVALID_GOAL;
      result->error_string = "preempted (e-stop or external)";
      active_goal_->abort(result);
    }
    traj_done_.store(false);
    traj_completed_clean_.store(false);
    active_goal_.reset();
  }
}

void RobotPVTController::promote_to_setpoint(
  const std::vector<double> & positions)
{
  Setpoint sp;
  sp.position = positions;
  sp.position.resize(num_joints_, 0.0);
  sp.velocity.assign(num_joints_, 0.0);
  sp.acceleration.assign(num_joints_, 0.0);
  setpoint_buf_.writeFromNonRT(std::move(sp));
}

}  // namespace robot_pvt_control

PLUGINLIB_EXPORT_CLASS(
  robot_pvt_control::RobotPVTController,
  controller_interface::ControllerInterface)
