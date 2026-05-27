#ifndef ROBOT_PVT_CONTROL__ROBOT_PVT_CONTROLLER_HPP_
#define ROBOT_PVT_CONTROL__ROBOT_PVT_CONTROLLER_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_pvt_control/hermite.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "robot_safety/estop_subscriber.hpp"
#include "robot_safety/param_loader.hpp"
#include "robot_safety/rate_limiter.hpp"
#include "robot_safety/safety_limits.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace robot_pvt_control
{

enum class Mode : uint8_t
{
  FREE    = 0,   // neutral output — every drive produces zero torque
  PVT     = 1,   // active — track the streaming setpoint
  DAMPING = 2,   // brake — drive runs Kp=0, Kd=Kd_damp[j], q_des=q_meas
};

/// Whole-body PVT controller. Ported from pendulum_pvt_control, generalised to
/// N joints. Claims, per joint, command interfaces {position, velocity, effort,
/// kp, kd} (drive_side_pd=true) or just {effort} (sim, drive_side_pd=false) in
/// the order the `joints` parameter lists them. State interfaces are
/// {position, velocity} per joint.
///
/// Streaming setpoint: ~/setpoint (trajectory_msgs/JointTrajectoryPoint) — the
/// positions/velocities/accelerations arrays index the joints in the same
/// order as the `joints` parameter. Action server: ~/follow_joint_trajectory
/// with cubic/quintic-Hermite interpolation at the controller-manager rate.
/// Services: ~/hold, ~/free, ~/damp (std_srvs/Trigger). ~/damp drives Kp=0,
/// Kd=Kd_damp[j] across the active body group — a pure-brake mode used
/// during gain ramp-up (see docs/tuning/pvt_tuning_guide.html).
class RobotPVTController : public controller_interface::ControllerInterface
{
public:
  RobotPVTController() = default;

  using FJT = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FJT>;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  /// One streaming setpoint payload (all joints in `joints_` order).
  struct Setpoint
  {
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> acceleration;
  };

  // Knot / Trajectory live in hermite.hpp now — pure-math types reused by
  // the unit tests. The action-server callback (handle_accepted) reorders
  // the inbound action joint_names into our internal order before
  // populating Knot::pos/vel/acc; that remapping logic still lives here.

  /// Hot-reloadable parameter snapshot. Per-joint arrays are sized to
  /// joints_.size() in load_params(); update() reads the snapshot via
  /// RealtimeBuffer with try_lock.
  struct Params
  {
    // Per-joint
    std::vector<double> Kp;            ///< N·m/rad (drive-side or software PD)
    std::vector<double> Kd;            ///< N·m·s/rad
    std::vector<double> Kd_damp;       ///< N·m·s/rad, used only in Mode::DAMPING
    std::vector<double> mgl;           ///< gravity feedforward magnitude
    std::vector<double> J;             ///< inertia feedforward
    std::vector<double> Fv;            ///< viscous feedforward
    std::vector<double> comp_sign;     ///< host-FF sign (drive-side PD ignores)
    std::vector<bool> ff_gravity;
    std::vector<bool> ff_inertia;
    std::vector<bool> ff_viscous;
    std::vector<double> hold_position;
    // Global
    bool drive_side_pd{true};
    double lag_free{0.04};
    double lag_pause{0.14};
    double alpha_slew{2.0};
  };

  void load_params();
  void write_free_outputs();
  /// Write the FREE pattern for a single joint slot (q=measured,
  /// v/eff/kp/kd = 0). Used by update() to leave inactive joints
  /// back-drivable while still tracking the active set.
  void write_free_outputs_for(std::size_t j);
  /// Write the DAMPING pattern for a single joint slot: q_des=measured,
  /// v_des=0, eff=0, kp=0, kd=Kd_damp[j]. The drive's PD law collapses to
  /// tau = -Kd_damp[j] * qd, producing pure mechanical brake without
  /// reference tracking. Sim path: writes tau = -Kd_damp[j] * qd directly.
  void write_damping_outputs_for(std::size_t j, double qd);
  void setpoint_callback(
    const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg);
  void hold_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void free_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void damp_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FJT::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFJT> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle);
  void on_feedback_tick();
  void preempt_goal(const std::string & why);

  // Sampling helpers — see robot_pvt_control::sample_segment /
  // sample_trajectory in hermite.hpp.

  // --- configuration ---
  std::vector<std::string> joints_;
  std::unordered_map<std::string, std::size_t> joint_index_;
  std::size_t num_joints_ = 0;
  std::size_t cmd_per_joint_ = 5;   // 5 for drive_side_pd, 1 for sim

  Params params_;
  realtime_tools::RealtimeBuffer<Params> params_buf_;
  rclcpp_lifecycle::LifecycleNode::PostSetParametersCallbackHandle::SharedPtr
    post_set_param_cb_;
  bool drive_side_pd_{true};

  // body_group scope. Latched at on_configure — see load_params() for the
  // string→mask resolution and the post_set_param_cb_ reject. Inactive
  // slots get write_free_outputs_for(j) every update().
  std::string body_group_{"full"};
  std::vector<bool> active_mask_;

  // Safety
  robot_safety::SafetyLimitsTable limits_;
  robot_safety::EstopSubscriber safety_;
  std::vector<robot_safety::RateLimiter> rate_limiters_;
  std::vector<double> estop_hold_pos_;
  bool estop_was_active_{false};
  std::vector<double> resume_hold_pos_;
  std::atomic<bool> waiting_for_setpoint_{false};

  // Streaming setpoint
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr
    setpoint_sub_;
  realtime_tools::RealtimeBuffer<Setpoint> setpoint_buf_;
  std::atomic<Mode> mode_{Mode::FREE};

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hold_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr free_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr damp_srv_;

  // Action server
  rclcpp_action::Server<FJT>::SharedPtr action_server_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<const Trajectory>> traj_buf_;
  std::atomic<bool> traj_done_{false};
  std::atomic<bool> traj_completed_clean_{false};

  // Lag-governor virtual time (single per-trajectory)
  double tv_{0.0};
  double alpha_{1.0};
  std::vector<double> dq_sign_;       // per joint: +1 / -1 net travel direction

  // Cached temporaries for sampling (preallocated to avoid RT-path allocation)
  mutable std::vector<double> tmp_p_;
  mutable std::vector<double> tmp_v_;
  mutable std::vector<double> tmp_a_;

  std::shared_ptr<GoalHandleFJT> active_goal_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
};

}  // namespace robot_pvt_control

#endif  // ROBOT_PVT_CONTROL__ROBOT_PVT_CONTROLLER_HPP_
