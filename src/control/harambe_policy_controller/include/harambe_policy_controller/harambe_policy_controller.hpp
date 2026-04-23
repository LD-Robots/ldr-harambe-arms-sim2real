#ifndef HARAMBE_POLICY_CONTROLLER_HPP_
#define HARAMBE_POLICY_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "realtime_tools/realtime_buffer.hpp"

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>

#include <onnxruntime_cxx_api.h>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace harambe_policy_controller
{

/**
 * HarambePolicyController — ROS2 port of HarambePolicyPlugin (Gazebo).
 *
 * Runs an ONNX RL policy at 50 Hz in the controller_manager loop.
 * Builds a 93-dim observation from joint_states + dual IMU (pelvis + torso),
 * converts policy actions into joint target positions, and applies
 *   τ = kp·(target - q) - kd·q̇ + g(q)
 * where g(q) is KDL gravity compensation. Writes τ to effort command interfaces.
 *
 * Observation layout (must match training, 93 dims):
 *   [0:3]   base_lin_vel   (pelvis body frame)   — from IMU linear_acceleration (integrated) or 0
 *   [3:6]   pelvis_ang_vel (body frame)          — from /imu/pelvis angular_velocity
 *   [6:9]   pelvis_projected_gravity             — from /imu/pelvis orientation → gravity in body
 *   [9:12]  torso_ang_vel  (body frame)          — from /imu/torso angular_velocity
 *   [12:15] torso_projected_gravity              — from /imu/torso orientation → gravity
 *   [15:18] commands [vx, vy, wz]                — from /cmd_vel
 *   [18:43] joint_pos_rel (25)                   — joint_pos - default_pos (HARDWARE_JOINT_ORDER)
 *   [43:68] joint_vel (25)
 *   [68:93] prev_action (25)
 *
 * Command interfaces claimed: effort (25 joints)
 * State interfaces claimed:   position + velocity (25 joints)
 */
class HarambePolicyController : public controller_interface::ControllerInterface
{
public:
  HarambePolicyController() = default;
  ~HarambePolicyController() override = default;

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
  // Per-joint configuration (mirrors Gazebo plugin JointConfig)
  struct JointConfig
  {
    std::string name;
    double kp{0.0};
    double kd{0.0};
    double effort_limit{0.0};
    double default_pos{0.0};
    double friction{0.0};
    size_t pos_iface_idx{0};
    size_t vel_iface_idx{0};
    size_t cmd_iface_idx{0};
  };

  // KDL chain for gravity compensation (one per limb)
  struct KDLChainData
  {
    KDL::Chain chain;
    std::unique_ptr<KDL::ChainDynParam> dynamics;
    std::vector<int> joint_map;   // chain-joint-index -> joints_ index (-1 if not tracked)
  };

  // Parameters
  std::vector<std::string> joint_names_;
  std::vector<JointConfig> joints_;
  size_t num_joints_{0};

  int decimation_{4};             // physics_steps per policy inference
  double action_scale_{0.25};
  double clip_actions_{5.0};
  int warmup_steps_{200};         // policy steps of gain-boosted warmup
  double fall_threshold_{0.5};    // |grav_z| < this => robot fallen, actions zeroed
  std::string onnx_path_;
  std::string urdf_path_;

  // Topic names
  std::string pelvis_imu_topic_;
  std::string torso_imu_topic_;
  std::string pelvis_grav_topic_;   // optional — BNO055 "gravity vector" output
  std::string torso_grav_topic_;
  std::string cmd_vel_topic_;
  std::string enable_topic_;
  std::string debug_topic_;
  bool publish_debug_{false};
  // If true, use /<imu>/grav topic directly for projected_gravity; otherwise
  // derive from orientation quaternion.
  bool use_grav_topic_{true};

  // Observation & state
  std::vector<float> obs_;              // 93 dims
  std::vector<float> prev_action_;      // 25
  std::vector<float> target_pos_;       // current joint target (updated at policy rate)
  std::array<float, 3> commands_{};     // [vx, vy, wz]

  // IMU data cached from subscribers (realtime-safe)
  struct ImuState
  {
    std::array<double, 3> ang_vel{};      // body frame
    std::array<double, 3> proj_gravity{}; // body frame, gravity vector projected
    std::array<double, 3> lin_accel{};    // raw accel (used as fallback for base_lin_vel)
    bool valid{false};
  };
  realtime_tools::RealtimeBuffer<ImuState> pelvis_imu_buf_;
  realtime_tools::RealtimeBuffer<ImuState> torso_imu_buf_;
  realtime_tools::RealtimeBuffer<std::array<float, 3>> cmd_buf_;
  realtime_tools::RealtimeBuffer<bool> enable_buf_;

  // ONNX runtime
  std::unique_ptr<Ort::Env> ort_env_;
  std::unique_ptr<Ort::Session> ort_session_;
  Ort::MemoryInfo ort_mem_info_{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};

  // KDL gravity comp
  std::vector<KDLChainData> kdl_chains_;
  std::vector<double> gravity_torques_;   // size num_joints_
  bool kdl_ready_{false};

  // Timing / state machine
  uint64_t step_counter_{0};
  uint64_t policy_step_{0};
  bool policy_active_{false};
  bool fallen_{false};
  bool enabled_{true};

  // Subscribers (non-RT side, write to realtime buffers)
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr pelvis_imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr torso_imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr pelvis_grav_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr torso_grav_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr debug_pub_;

  // IMU mount type (determines sensor→base transform)
  enum class ImuMount { PELVIS, TORSO };

  // Helpers
  void buildObservation();
  void runPolicyInference();
  void computeGravityCompensation();
  void applyPDControl();
  bool initKDL(const std::string & urdf_string);
  void imuToState(
    const sensor_msgs::msg::Imu & msg, ImuMount mount, ImuState & out) const;
  // Apply sensor→base mount transform for pelvis (rpy=π,-π/2,0) or torso (rpy=0,π/2,0).
  std::array<double, 3> sensorToBase(
    const std::array<double, 3> & v_s, ImuMount mount) const;
  void publishDebugIfEnabled();
};

}  // namespace harambe_policy_controller

#endif
