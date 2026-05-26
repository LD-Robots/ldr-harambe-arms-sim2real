#ifndef ROBOT_DRIVE_STATUS_BROADCASTER__DRIVE_STATUS_BROADCASTER_HPP_
#define ROBOT_DRIVE_STATUS_BROADCASTER__DRIVE_STATUS_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/duration.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace robot_drive_status_broadcaster
{

/// Whole-body MyActuator drive telemetry broadcaster.
///
/// Claims a fixed set of state interfaces (motor_temperature, drive_temperature,
/// bus_voltage, error_code) for every joint in the `joints` parameter, in
/// joints[i]/signal index order. Publishes one std_msgs/Float64MultiArray per
/// signal:
///   /<this>/motor_temperature   degC, N entries
///   /<this>/drive_temperature   degC, N entries
///   /<this>/bus_voltage         V,    N entries
///   /<this>/error_code          CiA 402 error register, N entries
///
/// Topic name defaults to drive_status_broadcaster/* but can be re-mapped at
/// launch. Realtime-safe: all message vectors are pre-sized in on_configure
/// and only overwritten in update().
class DriveStatusBroadcaster : public controller_interface::ControllerInterface
{
public:
  DriveStatusBroadcaster() = default;

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
  using ArrayPublisher = realtime_tools::RealtimePublisher<
    std_msgs::msg::Float64MultiArray>;
  using DiagPublisher = realtime_tools::RealtimePublisher<
    diagnostic_msgs::msg::DiagnosticArray>;

  // The four signals this broadcaster always exposes — matches TxPDO 0x1A02
  // in the PVT slave configs. Order matters: index 0..3 below + the publisher
  // vector in the same order.
  static constexpr std::size_t kNumSignals = 4;
  static const char * const kSignalNames[kNumSignals];

  std::vector<std::string> joints_;
  double publish_rate_ = 50.0;
  rclcpp::Duration publish_period_{0, 0};
  rclcpp::Time last_publish_time_;

  struct Thresholds
  {
    double motor_temp_warn  = 70.0;
    double motor_temp_error = 90.0;
    double drive_temp_warn  = 70.0;
    double drive_temp_error = 85.0;
    double bus_voltage_min  = 40.0;
    double bus_voltage_max  = 54.0;
  } thr_;

  // One publisher per signal (4 total). Each carries N=joints_.size() doubles.
  std::vector<std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>>>
    signal_pubs_;
  std::vector<std::unique_ptr<ArrayPublisher>> signal_rt_pubs_;

  std::shared_ptr<rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>>
    diag_pub_;
  std::unique_ptr<DiagPublisher> diag_rt_pub_;
};

}  // namespace robot_drive_status_broadcaster

#endif  // ROBOT_DRIVE_STATUS_BROADCASTER__DRIVE_STATUS_BROADCASTER_HPP_
