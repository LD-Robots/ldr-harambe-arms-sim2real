#ifndef RL_LOCOMOTION_CONTROLLER_HPP_
#define RL_LOCOMOTION_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include <onnxruntime_cxx_api.h>

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

namespace rl_locomotion_controller
{

constexpr std::size_t kNumJoints = 25;
constexpr std::size_t kObsDim93 = 93;  // V21+ policies (pelvis+torso IMU)
constexpr std::size_t kObsDim87 = 87;  // legacy policies (pelvis only)

/**
 * RLLocomotionController — ros2_control plugin running an ONNX locomotion
 * policy on the 25-DOF Harambe humanoid.
 *
 * Pipeline (runs at controller_manager rate, policy inferred every decimation):
 *
 *   state interfaces (pos+vel, 25 joints)   IMU (pelvis+torso)   /cmd_vel
 *              \                               |                  /
 *               +---------------+--------------+-----------------+
 *                               |
 *                      Build obs [87 or 93]
 *                               |
 *                         ONNX inference
 *                               |
 *                  targets = default + action * action_scale
 *                               |
 *                  Rate-limit + safety gates
 *                               |
 *                  Write position command interfaces
 *
 * State machine:
 *   INIT     — hold current joint positions, wait for IMU
 *   RAMP     — smooth interpolation from current -> ISAAC_DEFAULT (~2s)
 *   POLICY   — run ONNX, output action-shifted targets
 *   FROZEN   — safety tripped (IMU tilt > tilt_abort), hold last safe targets
 *
 * Parameters (see config/rl_locomotion_params.yaml for full list):
 *   joints:           25 joint names in CONTROLLER order (URDF order)
 *   policy_joint_order: 25 joint names in POLICY order (Isaac training order)
 *   policy_path:      path to exported policy.onnx
 *   action_scale:     per-action multiplier (0.25 for V29)
 *   isaac_default:    25-vector of default joint positions (policy order)
 *   decimation:       controller updates per policy step (100 Hz / 50 Hz -> 2)
 *   ramp_duration_s:  seconds to ramp from init pose to Isaac default
 *   tilt_abort_cos:   cos(max tilt); freeze when proj_grav.z > this. Default -0.3
 *                     (about 72 deg). Set to 0 to disable.
 *   max_target_rate:  max |target_t - target_{t-1}| per policy step (rad)
 *   cmd_vel_x/y/yaw:  default velocity command; overridden by /cmd_vel topic
 *   pelvis_imu_topic: default /pelvis_imu/data
 *   torso_imu_topic:  default /torso_imu/data  (ignored if obs_dim == 87)
 *   cmd_vel_topic:    default /cmd_vel
 */
class RLLocomotionController : public controller_interface::ControllerInterface
{
public:
  RLLocomotionController() = default;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  enum class State { INIT, RAMP, POLICY, FROZEN };

  // ---- Configuration ----
  std::vector<std::string> ctrl_joints_;        // controller / URDF order, 25
  std::vector<std::string> policy_joints_;      // policy / Isaac order, 25
  std::array<std::size_t, kNumJoints> policy_to_ctrl_{};  // idx map
  std::array<double, kNumJoints> isaac_default_{};        // policy order

  std::string policy_path_;
  double action_scale_ = 0.25;
  int decimation_ = 2;          // controller_updates / policy_step
  double ramp_duration_s_ = 2.0;
  double tilt_abort_cos_ = -0.3;
  double max_target_rate_ = 0.10;  // rad per policy step

  // ---- ONNX ----
  std::unique_ptr<Ort::Env> ort_env_;
  std::unique_ptr<Ort::Session> ort_session_;
  std::unique_ptr<Ort::MemoryInfo> ort_mem_info_;
  std::string onnx_input_name_;
  std::string onnx_output_name_;
  std::size_t obs_dim_ = kObsDim93;  // detected from model

  // ---- State machine ----
  State state_ = State::INIT;
  std::size_t update_counter_ = 0;
  std::size_t decim_counter_ = 0;
  double ramp_t_ = 0.0;
  std::array<double, kNumJoints> init_pose_{};       // controller order (captured at activate)
  std::array<double, kNumJoints> last_target_ctrl_{};// controller order
  std::array<double, kNumJoints> last_action_pol_{}; // policy order (used by obs)
  std::array<double, kNumJoints> frozen_target_ctrl_{};

  // ---- Live data (realtime-safe) ----
  struct ImuSnapshot
  {
    std::array<double, 3> ang_vel{{0, 0, 0}};
    std::array<double, 3> proj_grav{{0, 0, -1}};
    bool valid = false;
  };
  // mutable: readFromRT() is non-const but is RT-safe; build_obs is const.
  mutable realtime_tools::RealtimeBuffer<ImuSnapshot> pelvis_imu_buf_;
  mutable realtime_tools::RealtimeBuffer<ImuSnapshot> torso_imu_buf_;

  struct CmdVel
  {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
  };
  mutable realtime_tools::RealtimeBuffer<CmdVel> cmd_vel_buf_;

  std::atomic<bool> emergency_stop_{false};

  // ---- ROS subs ----
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr pelvis_imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr torso_imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr pelvis_grav_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr torso_grav_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;

  // ---- Helpers ----
  static std::array<double, 3> ang_vel_from_imu(const sensor_msgs::msg::Imu & msg);
  static std::array<double, 3> proj_grav_from_vec(
    const geometry_msgs::msg::Vector3Stamped & msg);
  void build_obs(std::vector<float> & out) const;
  void run_policy(std::array<double, kNumJoints> & action_out);
  void step_policy();
  void write_targets_ctrl(const std::array<double, kNumJoints> & targets_ctrl);
  bool read_current_pose_ctrl(std::array<double, kNumJoints> & out) const;
};

}  // namespace rl_locomotion_controller

#endif  // RL_LOCOMOTION_CONTROLLER_HPP_
