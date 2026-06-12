#ifndef HARAMBE_POLICY_LEGS_CONTROLLER_HPP_
#define HARAMBE_POLICY_LEGS_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "realtime_tools/realtime_buffer.hpp"

#include <onnxruntime_cxx_api.h>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace harambe_policy_legs_controller
{

/**
 * HarambePolicyLegsController — ROS2 ros2_control controller running an RL
 * walking policy that drives ONLY the 12 leg joints with POSITION targets.
 *
 * Output mode: "mode 5" (PVT / CiA-402 mode 5). The EtherCAT drive closes a
 * pure PD loop at 1 kHz:  tau = kp*(target - q) - kd*qd  (NO gravity / inertia
 * feed-forward — the policy was trained to compensate gravity itself). The
 * controller therefore claims, per leg joint, the command interfaces
 *   {joint}/position, {joint}/kp, {joint}/kd
 * and writes a position target + a constant kp/kd. It does NOT compute torque
 * and contains NO KDL / gravity-compensation code.
 *
 * Policy I/O:
 *   ONNX input  "obs"     shape [1, 52]   (normalizer baked in → feed raw)
 *   ONNX output "actions" shape [1, 12]
 *
 * Observation layout (52, body frame = pelvis, REP-103 +X fwd +Y left +Z up):
 *   [0:3]   base_lin_vel       — /odometry/filtered twist.twist.linear (no remap)
 *   [3:6]   base_ang_vel(gyro) — /pelvis/imu angular_velocity, remap (sz,-sy,sx)
 *   [6:9]   projected_gravity  — /pelvis/imu orientation: quat_rot_inv(q,[0,0,-1])
 *                                (or /pelvis/grav remapped+normalized)
 *   [9:12]  cmd                — /cmd_vel (linear.x, linear.y, angular.z)
 *   [12:24] joint_pos - default (12 legs)
 *   [24:36] joint_vel          (12 legs)
 *   [36:48] last_action        (12) — RAW action (post-clip, pre-scale) 2 steps ago
 *   [48:52] gait clock         — [cos phL, cos phR, sin phL, sin phR], phR = phL+pi
 *
 * Action → target:  a = clip(action[i], -5, +5);  target[i] = default[i] + 0.5*a
 */
class HarambePolicyLegsController : public controller_interface::ControllerInterface
{
public:
  HarambePolicyLegsController() = default;
  ~HarambePolicyLegsController() override = default;

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
  // Per-joint configuration (one entry per leg joint, in LEG_JOINTS order).
  struct JointConfig
  {
    std::string name;
    double kp{0.0};
    double kd{0.0};
    double effort_limit{0.0};
    double default_pos{0.0};
    size_t pos_state_idx{0};    // index into state_interfaces_  (position)
    size_t vel_state_idx{0};    // index into state_interfaces_  (velocity)
    size_t pos_cmd_idx{0};      // index into command_interfaces_ (position)
    size_t kp_cmd_idx{0};       // index into command_interfaces_ (kp)
    size_t kd_cmd_idx{0};       // index into command_interfaces_ (kd)
  };

  // Parameters
  std::vector<std::string> joint_names_;
  std::vector<JointConfig> joints_;
  size_t num_joints_{0};

  int decimation_{20};            // controller steps per policy inference (50 Hz)
  double action_scale_{0.5};
  double clip_actions_{5.0};
  double gait_freq_{1.5};         // Hz — gait-clock frequency at deploy
  int warmup_steps_{40};          // policy steps holding default pose before policy
  double fall_threshold_{0.5};    // |proj_grav_z| < this => fallen => hold default
  std::string onnx_path_;

  // Topic names
  std::string pelvis_imu_topic_;
  std::string pelvis_grav_topic_;   // optional — BNO055 "gravity vector" output
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string enable_topic_;
  std::string debug_topic_;
  bool publish_debug_{false};
  bool use_grav_topic_{false};      // true => /pelvis/grav, else derive from orientation

  // Observation & policy state
  std::vector<float> obs_;              // 52 dims
  std::vector<float> target_pos_;       // current joint target (updated at policy rate)
  std::vector<float> ramp_start_;       // measured pose at activation; warmup ramps this -> default

  // last_action ring buffer (2-step lag, in POLICY steps).
  // action_hist_[0] = action[k-1], action_hist_[1] = action[k-2].
  std::array<std::vector<float>, 2> action_hist_;

  // gait clock phase (left leg), advanced once per policy step.
  double gait_phase_{0.0};

  // IMU data cached from subscribers (realtime-safe).
  struct ImuState
  {
    std::array<double, 3> ang_vel{};      // body frame, rad/s
    std::array<double, 3> proj_gravity{}; // body frame, unit gravity direction
    bool valid{false};
  };
  realtime_tools::RealtimeBuffer<ImuState> pelvis_imu_buf_;
  realtime_tools::RealtimeBuffer<std::array<float, 3>> base_lin_vel_buf_;
  realtime_tools::RealtimeBuffer<std::array<float, 3>> cmd_buf_;
  realtime_tools::RealtimeBuffer<bool> enable_buf_;

  // ONNX runtime
  std::unique_ptr<Ort::Env> ort_env_;
  std::unique_ptr<Ort::Session> ort_session_;
  Ort::MemoryInfo ort_mem_info_{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};

  // Timing / state machine
  uint64_t step_counter_{0};      // controller_manager ticks since activate
  uint64_t policy_step_{0};       // policy (50 Hz) steps since activate
  bool policy_active_{false};
  bool fallen_{false};
  bool enabled_{true};

  // Subscribers (non-RT side, write to realtime buffers)
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr pelvis_imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr pelvis_grav_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr debug_pub_;

  // Helpers
  void buildObservation();
  void runPolicyInference();
  void writeCommands();         // write target_pos + constant kp/kd to drives
  void imuToState(const sensor_msgs::msg::Imu & msg, ImuState & out) const;
  // Pelvis BNO055 sensor→body remap:  v_body = (v_sz, -v_sy, v_sx).
  std::array<double, 3> sensorToBody(const std::array<double, 3> & v_s) const;
  void publishDebugIfEnabled();
};

}  // namespace harambe_policy_legs_controller

#endif  // HARAMBE_POLICY_LEGS_CONTROLLER_HPP_
