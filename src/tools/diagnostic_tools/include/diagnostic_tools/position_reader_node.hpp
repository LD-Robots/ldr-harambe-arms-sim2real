#ifndef DIAGNOSTIC_TOOLS__POSITION_READER_NODE_HPP_
#define DIAGNOSTIC_TOOLS__POSITION_READER_NODE_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include "myactuator_ethercat/ethercat_master.hpp"
#include "myactuator_ethercat/myactuator_slave.hpp"

namespace diagnostic_tools
{

/// Safe, read-only EtherCAT position reader for 6-DOF arm.
///
/// Reads encoder positions from MyActuator RMD X V4 drives without
/// enabling the CiA 402 power stage. control_word is always 0x0000
/// (DISABLE_VOLTAGE), drives stay in SWITCH_ON_DISABLED.
///
/// Publishes:
///   /joint_states          — sensor_msgs/JointState (rad, rad/s, torque %)
///   /ethercat/raw_positions — std_msgs/Int32MultiArray (raw encoder values)
class PositionReaderNode : public rclcpp::Node
{
public:
  explicit PositionReaderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PositionReaderNode() override;

private:
  bool initialize_ethercat();
  void ethercat_cycle_callback();
  void publish_callback();
  void status_log_callback();

  // EtherCAT
  myactuator_ethercat::EthercatMaster master_;
  std::vector<myactuator_ethercat::MyActuatorSlave> slaves_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr raw_position_pub_;

  // Timers
  rclcpp::TimerBase::SharedPtr cycle_timer_;    // 1 kHz EtherCAT exchange
  rclcpp::TimerBase::SharedPtr publish_timer_;  // 100 Hz publish
  rclcpp::TimerBase::SharedPtr status_timer_;   // 1 Hz status logging

  // Configuration
  std::vector<std::string> joint_names_;
  int master_index_{0};
  int cycle_time_us_{1000};
  double publish_rate_hz_{100.0};

  // State
  bool ethercat_active_{false};
  uint64_t cycle_count_{0};
  uint64_t error_count_{0};
};

}  // namespace diagnostic_tools

#endif  // DIAGNOSTIC_TOOLS__POSITION_READER_NODE_HPP_
