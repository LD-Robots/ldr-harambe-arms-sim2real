#include "robot_pvt_control/robot_pvt_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "robot_safety/clamp.hpp"

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
    auto_declare<std::vector<double>>("mgl", std::vector<double>{});
    auto_declare<std::vector<double>>("J", std::vector<double>{});
    auto_declare<std::vector<double>>("Fv", std::vector<double>{});
    auto_declare<std::vector<double>>("comp_sign", std::vector<double>{});
    auto_declare<std::vector<bool>>("ff_gravity", std::vector<bool>{});
    auto_declare<std::vector<bool>>("ff_inertia", std::vector<bool>{});
    auto_declare<std::vector<bool>>("ff_viscous", std::vector<bool>{});
    auto_declare<std::vector<double>>("hold_position", std::vector<double>{});
    auto_declare<bool>("drive_side_pd", true);
    auto_declare<double>("lag_free", 0.04);
    auto_declare<double>("lag_pause", 0.14);
    auto_declare<double>("alpha_slew", 2.0);
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
  params_.mgl           = node->get_parameter("mgl").as_double_array();
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
  normalize(params_.mgl, num_joints_, 0.0);
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

  // Centralised safety: load shared limits and subscribe to the supervisor.
  limits_ = robot_safety::loadSafetyLimits(*get_node(), joints_, "safety.");
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

  post_set_param_cb_ = node->add_post_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & /*params*/) {
      load_params();
      params_buf_.writeFromNonRT(params_);
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
        const auto & L = limits_.limitsFor(joints_[j]);
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
          q[j], limits_.limitsFor(joints_[j]));
      }
    }
  }
  // Falling edge — e-stop just cleared.
  if (!safety.estop_active && estop_was_active_) {
    for (std::size_t j = 0; j < num_joints_; ++j) {
      resume_hold_pos_[j] = robot_safety::clampPosition(
        q[j], limits_.limitsFor(joints_[j]));
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
      sample_trajectory(*traj_ptr, tv_, tmp_p_, tmp_v_, tmp_a_);

      // Whole-body lag: worst joint's lead in its travel direction.
      double max_lead = -std::numeric_limits<double>::infinity();
      for (std::size_t j = 0; j < num_joints_; ++j) {
        const double lead = (tmp_p_[j] - q[j]) * dq_sign_[j];
        if (lead > max_lead) {
          max_lead = lead;
        }
      }
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

  // Per-joint slew/accel limiting + clamps + drive write.
  for (std::size_t j = 0; j < num_joints_; ++j) {
    const auto & L = limits_.limitsFor(joints_[j]);
    if (!rate_limiters_[j].seeded()) {
      rate_limiters_[j].seed(q[j]);
    }
    double p_cmd = rate_limiters_[j].limit(
      ref_pos[j], dt, L.slew_rate_limit, L.acceleration_limit);
    p_cmd = robot_safety::clampPosition(p_cmd, L);
    const double v_cmd = robot_safety::clampVelocity(ref_vel[j], L);

    const double tau_g = params_.ff_gravity[j] ?
      params_.mgl[j] * std::sin(q[j]) : 0.0;
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

void RobotPVTController::sample_trajectory(
  const Trajectory & traj, double t,
  std::vector<double> & p, std::vector<double> & v,
  std::vector<double> & a) const
{
  if (traj.knots.size() == 1) {
    p = traj.knots[0].pos;
    v = traj.knots[0].vel;
    a = traj.knots[0].acc;
    return;
  }
  if (t <= traj.knots.front().t) {
    p = traj.knots.front().pos;
    v = traj.knots.front().vel;
    a = traj.knots.front().acc;
    return;
  }
  if (t >= traj.knots.back().t) {
    p = traj.knots.back().pos;
    v = traj.knots.back().vel;
    a = traj.knots.back().acc;
    return;
  }
  for (std::size_t i = 1; i < traj.knots.size(); ++i) {
    if (t <= traj.knots[i].t) {
      sample_segment(traj.knots[i - 1], traj.knots[i], t, p, v, a);
      return;
    }
  }
  p = traj.knots.back().pos;
  std::fill(v.begin(), v.end(), 0.0);
  std::fill(a.begin(), a.end(), 0.0);
}

void RobotPVTController::sample_segment(
  const Knot & a_k, const Knot & b_k, double t,
  std::vector<double> & p, std::vector<double> & v,
  std::vector<double> & a_out) const
{
  const double h = b_k.t - a_k.t;
  if (h <= 0.0) {
    p = b_k.pos;
    v = b_k.vel;
    a_out = b_k.acc;
    return;
  }
  const double u = std::clamp((t - a_k.t) / h, 0.0, 1.0);
  const std::size_t n = num_joints_;

  if (a_k.has_acc && b_k.has_acc) {
    // Quintic Hermite — matches (p, v, a) at both endpoints.
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u3 * u;
    const double u5 = u4 * u;
    const double h0 = 1.0 - 10.0 * u3 + 15.0 * u4 - 6.0 * u5;
    const double h1 = u - 6.0 * u3 + 8.0 * u4 - 3.0 * u5;
    const double h2 = 0.5 * u2 - 1.5 * u3 + 1.5 * u4 - 0.5 * u5;
    const double h3 = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
    const double h4 = -4.0 * u3 + 7.0 * u4 - 3.0 * u5;
    const double h5 = 0.5 * u3 - u4 + 0.5 * u5;

    const double h0p = -30.0 * u2 + 60.0 * u3 - 30.0 * u4;
    const double h1p = 1.0 - 18.0 * u2 + 32.0 * u3 - 15.0 * u4;
    const double h2p = u - 4.5 * u2 + 6.0 * u3 - 2.5 * u4;
    const double h3p = 30.0 * u2 - 60.0 * u3 + 30.0 * u4;
    const double h4p = -12.0 * u2 + 28.0 * u3 - 15.0 * u4;
    const double h5p = 1.5 * u2 - 4.0 * u3 + 2.5 * u4;

    const double h0pp = -60.0 * u + 180.0 * u2 - 120.0 * u3;
    const double h1pp = -36.0 * u + 96.0 * u2 - 60.0 * u3;
    const double h2pp = 1.0 - 9.0 * u + 18.0 * u2 - 10.0 * u3;
    const double h3pp = 60.0 * u - 180.0 * u2 + 120.0 * u3;
    const double h4pp = -24.0 * u + 84.0 * u2 - 60.0 * u3;
    const double h5pp = 3.0 * u - 12.0 * u2 + 10.0 * u3;

    for (std::size_t j = 0; j < n; ++j) {
      p[j] = h0 * a_k.pos[j] + h1 * (a_k.vel[j] * h) + h2 * (a_k.acc[j] * h * h)
        + h3 * b_k.pos[j] + h4 * (b_k.vel[j] * h) + h5 * (b_k.acc[j] * h * h);
      v[j] = (h0p * a_k.pos[j] + h3p * b_k.pos[j]) / h
        + (h1p * a_k.vel[j] + h4p * b_k.vel[j])
        + (h2p * a_k.acc[j] + h5p * b_k.acc[j]) * h;
      a_out[j] = (h0pp * a_k.pos[j] + h3pp * b_k.pos[j]) / (h * h)
        + (h1pp * a_k.vel[j] + h4pp * b_k.vel[j]) / h
        + (h2pp * a_k.acc[j] + h5pp * b_k.acc[j]);
    }
  } else {
    // Cubic Hermite — matches (p, v) at both endpoints.
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
    const double h10 = u3 - 2.0 * u2 + u;
    const double h01 = -2.0 * u3 + 3.0 * u2;
    const double h11 = u3 - u2;

    const double h00p = 6.0 * u2 - 6.0 * u;
    const double h10p = 3.0 * u2 - 4.0 * u + 1.0;
    const double h01p = -6.0 * u2 + 6.0 * u;
    const double h11p = 3.0 * u2 - 2.0 * u;

    const double h00pp = 12.0 * u - 6.0;
    const double h10pp = 6.0 * u - 4.0;
    const double h01pp = -12.0 * u + 6.0;
    const double h11pp = 6.0 * u - 2.0;

    for (std::size_t j = 0; j < n; ++j) {
      p[j] = h00 * a_k.pos[j] + h10 * (a_k.vel[j] * h)
        + h01 * b_k.pos[j] + h11 * (b_k.vel[j] * h);
      v[j] = (h00p * a_k.pos[j] + h01p * b_k.pos[j]) / h
        + h10p * a_k.vel[j] + h11p * b_k.vel[j];
      a_out[j] = (h00pp * a_k.pos[j] + h01pp * b_k.pos[j]) / (h * h)
        + (h10pp * a_k.vel[j] + h11pp * b_k.vel[j]) / h;
    }
  }
}

}  // namespace robot_pvt_control

PLUGINLIB_EXPORT_CLASS(
  robot_pvt_control::RobotPVTController,
  controller_interface::ControllerInterface)
