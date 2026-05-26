#ifndef ROBOT_FILTERED_JOINT_STATE_BROADCASTER__FILTERED_JOINT_STATE_BROADCASTER_HPP_
#define ROBOT_FILTERED_JOINT_STATE_BROADCASTER__FILTERED_JOINT_STATE_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/duration.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace robot_filtered_joint_state_broadcaster
{

/// Whole-body filtered joint state broadcaster.
///
/// Claims position / velocity / effort state interfaces for every joint in the
/// `joints` parameter. Each cycle (full controller-manager rate, e.g. 1 kHz)
/// runs a first-order IIR low-pass on velocity and effort per joint, and
/// publishes a sensor_msgs/JointState at `publish_rate` (default 100 Hz) on
/// the configured topic (default `/joint_states`).
///
/// Position is published unfiltered — the MyActuator encoder is already crisp.
/// The cutoff frequencies are live-tunable parameters.
class FilteredJointStateBroadcaster
  : public controller_interface::ControllerInterface
{
public:
  FilteredJointStateBroadcaster() = default;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  using JointStatePublisher = realtime_tools::RealtimePublisher<
    sensor_msgs::msg::JointState>;

  std::vector<std::string> joints_;
  std::string topic_ = "/joint_states";
  double velocity_cutoff_hz_ = 30.0;
  double torque_cutoff_hz_ = 30.0;
  double publish_rate_ = 100.0;
  rclcpp::Duration publish_period_{0, 0};
  rclcpp::Time last_publish_time_;

  // Per-joint filter state. Both vectors are sized to joints_.size() in
  // on_configure; filter_initialized_ guards a one-shot seed on the first
  // finite sample so the filter doesn't ramp from zero.
  std::vector<double> vel_filt_;
  std::vector<double> tau_filt_;
  bool filter_initialized_ = false;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>>
    joint_state_pub_;
  std::unique_ptr<JointStatePublisher> rt_joint_state_pub_;
};

}  // namespace robot_filtered_joint_state_broadcaster

#endif  // ROBOT_FILTERED_JOINT_STATE_BROADCASTER__FILTERED_JOINT_STATE_BROADCASTER_HPP_
