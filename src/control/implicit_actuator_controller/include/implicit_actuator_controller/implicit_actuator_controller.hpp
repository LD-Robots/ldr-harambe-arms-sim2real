#ifndef IMPLICIT_ACTUATOR_CONTROLLER_HPP_
#define IMPLICIT_ACTUATOR_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include <vector>
#include <string>

namespace implicit_actuator_controller
{

/**
 * ImplicitActuatorController — ros2_control plugin that replicates
 * Isaac Lab's ImplicitActuator behavior.
 *
 * Runs PD control at the controller_manager update rate (in-process, zero latency):
 *   torque = kp * (target_pos - current_pos) - kd * current_vel
 *   torque = clamp(torque, -effort_limit, effort_limit)
 *
 * Subscribes to /target_joint_positions for position targets from RL policy.
 * On startup, holds joints at configurable default positions.
 */
class ImplicitActuatorController : public controller_interface::ControllerInterface
{
public:
  ImplicitActuatorController() = default;

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
  std::vector<std::string> joint_names_;
  size_t num_joints_ = 0;

  // PID gains and effort limits per joint
  std::vector<double> kp_;
  std::vector<double> ki_;
  std::vector<double> kd_;
  std::vector<double> i_clamp_;
  std::vector<double> effort_limits_;
  std::vector<double> default_positions_;
  std::vector<double> target_positions_;      // smoothed (actual PID target)
  std::vector<double> commanded_positions_;   // raw from topic
  std::vector<double> integral_error_;
  double max_pos_rate_;  // max rad/s for target smoothing

  // Interface index maps (joint_index -> state_interfaces_ index)
  std::vector<size_t> pos_iface_idx_;
  std::vector<size_t> vel_iface_idx_;
  std::vector<size_t> cmd_iface_idx_;

  // Realtime-safe target from subscriber
  realtime_tools::RealtimeBuffer<std::vector<double>> rt_target_buffer_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_sub_;
};

}  // namespace implicit_actuator_controller

#endif
