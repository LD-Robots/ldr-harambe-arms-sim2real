#include "diagnostic_tools/position_reader_node.hpp"

#include <chrono>
#include <functional>

#include "myactuator_ethercat/unit_conversion.hpp"
#include "myactuator_ethercat/cia402_state_machine.hpp"
#include "myactuator_ethercat/types.hpp"

using namespace std::chrono_literals;
using myactuator_ethercat::UnitConversion;
using myactuator_ethercat::Cia402StateMachine;
using myactuator_ethercat::MotorType;
using myactuator_ethercat::DriveState;

namespace diagnostic_tools
{

PositionReaderNode::PositionReaderNode(const rclcpp::NodeOptions & options)
: Node("position_reader_node", options)
{
  // Declare parameters
  master_index_ = declare_parameter<int>("master_index", 0);
  cycle_time_us_ = declare_parameter<int>("cycle_time_us", 1000);
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 100.0);

  joint_names_ = declare_parameter<std::vector<std::string>>("joint_names", {
    "left_shoulder_pitch_joint_X6",
    "left_shoulder_roll_joint_X6",
    "left_shoulder_yaw_joint_X4",
    "left_elbow_pitch_joint_X6",
    "left_wrist_yaw_joint_X4",
    "left_wrist_roll_joint_X4"
  });

  auto slave_positions = declare_parameter<std::vector<int64_t>>(
    "slave_positions", {0, 1, 2, 3, 4, 5});
  auto motor_type_strs = declare_parameter<std::vector<std::string>>(
    "motor_types", {"X6", "X6", "X4", "X6", "X4", "X4"});
  auto directions = declare_parameter<std::vector<int64_t>>(
    "directions", {1, 1, 1, 1, 1, 1});
  auto offsets = declare_parameter<std::vector<int64_t>>(
    "offsets_raw", {0, 0, 0, 0, 0, 0});

  // Validate parameter array sizes
  const size_t n = joint_names_.size();
  if (slave_positions.size() != n || motor_type_strs.size() != n ||
      directions.size() != n || offsets.size() != n)
  {
    RCLCPP_FATAL(get_logger(),
      "All joint parameter arrays must have the same size (%zu)", n);
    rclcpp::shutdown();
    return;
  }

  // Configure slaves
  slaves_.resize(n);
  for (size_t i = 0; i < n; ++i) {
    MotorType mt = (motor_type_strs[i] == "X6") ? MotorType::X6 : MotorType::X4;
    slaves_[i].configure(
      static_cast<uint16_t>(slave_positions[i]),
      mt,
      static_cast<int>(directions[i]),
      static_cast<int32_t>(offsets[i])
    );
    RCLCPP_INFO(get_logger(), "Slave %zu: %s at pos %ld, type %s, dir %ld, offset %ld",
                i, joint_names_[i].c_str(), slave_positions[i],
                motor_type_strs[i].c_str(), directions[i], offsets[i]);
  }

  // Create publishers
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  raw_position_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
    "/ethercat/raw_positions", 10);

  // Initialize EtherCAT
  if (!initialize_ethercat()) {
    RCLCPP_FATAL(get_logger(), "Failed to initialize EtherCAT master. Shutting down.");
    rclcpp::shutdown();
    return;
  }

  // Create timers
  auto cycle_period = std::chrono::microseconds(cycle_time_us_);
  cycle_timer_ = create_wall_timer(cycle_period,
    std::bind(&PositionReaderNode::ethercat_cycle_callback, this));

  auto publish_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_hz_));
  publish_timer_ = create_wall_timer(publish_period,
    std::bind(&PositionReaderNode::publish_callback, this));

  status_timer_ = create_wall_timer(1s,
    std::bind(&PositionReaderNode::status_log_callback, this));

  RCLCPP_INFO(get_logger(),
    "Position reader started: %zu joints, %d us cycle, %.0f Hz publish",
    n, cycle_time_us_, publish_rate_hz_);
  RCLCPP_WARN(get_logger(),
    "SAFETY MODE: control_word = 0x0000 always. Drives will NOT be enabled.");
}

PositionReaderNode::~PositionReaderNode()
{
  RCLCPP_INFO(get_logger(), "Shutting down EtherCAT master...");
  if (ethercat_active_) {
    master_.stop();
    ethercat_active_ = false;
  }
  RCLCPP_INFO(get_logger(), "EtherCAT master stopped. Clean shutdown.");
}

bool PositionReaderNode::initialize_ethercat()
{
  RCLCPP_INFO(get_logger(), "Initializing EtherCAT master (index %d)...", master_index_);

  if (!master_.init(master_index_, slaves_)) {
    RCLCPP_ERROR(get_logger(), "EtherCAT master init failed (check EtherLab and slaves)");
    return false;
  }

  RCLCPP_INFO(get_logger(), "Master initialized, configuring DC and activating...");

  uint64_t cycle_time_ns = static_cast<uint64_t>(cycle_time_us_) * 1000ULL;
  if (!master_.start(cycle_time_ns)) {
    RCLCPP_ERROR(get_logger(), "EtherCAT master activation failed");
    master_.stop();
    return false;
  }

  ethercat_active_ = true;
  RCLCPP_INFO(get_logger(), "EtherCAT master activated. Cycle time = %d us", cycle_time_us_);
  return true;
}

void PositionReaderNode::ethercat_cycle_callback()
{
  if (!ethercat_active_) return;

  // Perform EtherCAT exchange (receive + read + write 0x0000 + send)
  master_.exchange();
  cycle_count_++;

  // Check for drive faults
  for (size_t i = 0; i < slaves_.size(); ++i) {
    if (slaves_[i].has_fault()) {
      uint16_t err = slaves_[i].get_error_code();
      if (err != 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Slave %zu FAULT: error=0x%04X status=0x%04X",
          i, err, slaves_[i].get_status_word());
        error_count_++;
      }
    }
  }
}

void PositionReaderNode::publish_callback()
{
  if (!ethercat_active_) return;

  auto now = this->now();

  // JointState message (converted to SI units)
  auto js_msg = sensor_msgs::msg::JointState();
  js_msg.header.stamp = now;
  js_msg.name = joint_names_;
  js_msg.position.resize(slaves_.size());
  js_msg.velocity.resize(slaves_.size());
  js_msg.effort.resize(slaves_.size());

  // Raw positions for offset calibration
  auto raw_msg = std_msgs::msg::Int32MultiArray();
  raw_msg.data.resize(slaves_.size());

  for (size_t i = 0; i < slaves_.size(); ++i) {
    int32_t raw_pos = slaves_[i].get_actual_position_raw();
    raw_msg.data[i] = raw_pos;

    js_msg.position[i] = UnitConversion::raw_to_rad(raw_pos);
    js_msg.velocity[i] = UnitConversion::raw_to_rad_per_sec(
      slaves_[i].get_actual_velocity_raw());
    js_msg.effort[i] = UnitConversion::raw_to_torque_pct(
      slaves_[i].get_actual_torque_raw());
  }

  joint_state_pub_->publish(js_msg);
  raw_position_pub_->publish(raw_msg);
}

void PositionReaderNode::status_log_callback()
{
  if (!ethercat_active_) return;

  bool all_op = master_.all_slaves_operational();
  uint32_t slave_count = master_.get_slave_count();

  RCLCPP_INFO(get_logger(),
    "--- Status: cycles=%lu errors=%lu slaves=%u all_op=%s ---",
    cycle_count_, error_count_, slave_count, all_op ? "YES" : "NO");

  for (size_t i = 0; i < slaves_.size(); ++i) {
    DriveState state = slaves_[i].get_drive_state();
    int32_t raw_pos = slaves_[i].get_actual_position_raw();
    double rad = UnitConversion::raw_to_rad(raw_pos);

    const char * state_str = "UNKNOWN";
    switch (state) {
      case DriveState::NOT_READY_TO_SWITCH_ON: state_str = "NOT_READY"; break;
      case DriveState::SWITCH_ON_DISABLED:     state_str = "DISABLED"; break;
      case DriveState::READY_TO_SWITCH_ON:     state_str = "READY"; break;
      case DriveState::SWITCHED_ON:            state_str = "SWITCHED_ON"; break;
      case DriveState::OPERATION_ENABLED:      state_str = "OP_ENABLED"; break;
      case DriveState::QUICK_STOP_ACTIVE:      state_str = "QUICK_STOP"; break;
      case DriveState::FAULT_REACTION_ACTIVE:  state_str = "FAULT_REACT"; break;
      case DriveState::FAULT:                  state_str = "FAULT"; break;
    }

    RCLCPP_INFO(get_logger(),
      "  [%zu] %s: state=%-12s raw=%-8d rad=%+7.4f status=0x%04X err=0x%04X",
      i, joint_names_[i].c_str(), state_str, raw_pos, rad,
      slaves_[i].get_status_word(), slaves_[i].get_error_code());
  }
}

}  // namespace diagnostic_tools

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<diagnostic_tools::PositionReaderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
