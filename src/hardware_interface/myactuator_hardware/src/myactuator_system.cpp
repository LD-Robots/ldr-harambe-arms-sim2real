#include "myactuator_hardware/myactuator_system.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// #include "myactuator_ethercat/ethercat_master.hpp"
// #include "myactuator_ethercat/myactuator_slave.hpp"
// #include "myactuator_ethercat/unit_conversion.hpp"
// #include "myactuator_ethercat/types.hpp"

namespace myactuator_hardware
{

hardware_interface::CallbackReturn MyActuatorSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // TODO: Read hardware-level parameters from URDF <ros2_control><hardware><param>
  //   master_index_ = stoi(info_.hardware_parameters.at("ethercat_master_index"));
  //   network_interface_ = info_.hardware_parameters.at("network_interface");
  //   cycle_time_us_ = stoi(info_.hardware_parameters.at("cycle_time_us"));
  //   dc_sync_ = info_.hardware_parameters.at("dc_sync") == "true";

  // Resize state/command vectors
  const size_t num_joints = info_.joints.size();
  hw_positions_.resize(num_joints, 0.0);
  hw_velocities_.resize(num_joints, 0.0);
  hw_efforts_.resize(num_joints, 0.0);
  hw_commands_positions_.resize(num_joints, 0.0);

  // TODO: For each joint, read per-joint parameters:
  //   slave_position, motor_type, direction, offset_raw
  // Configure MyActuatorSlave instances accordingly

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyActuatorSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: Implementation
  // 1. Create EthercatMaster instance
  // 2. master_->init(master_index_, slaves_)
  // 3. Verify expected number of slaves detected
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MyActuatorSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MyActuatorSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]);
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn MyActuatorSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: Implementation
  // 1. master_->start(cycle_time_us_ * 1000)  // Convert µs to ns
  // 2. For each slave: run CiA 402 enable sequence
  //    CRITICAL: Set target_position = actual_position BEFORE enabling
  //    to prevent joint jumps
  // 3. Verify all slaves reach OPERATION_ENABLED within timeout
  // 4. Initialize command positions from actual positions

  // Initialize commands to current positions (placeholder zeros for now)
  for (size_t i = 0; i < hw_commands_positions_.size(); ++i) {
    hw_commands_positions_[i] = hw_positions_[i];
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyActuatorSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: Implementation
  // 1. For each slave: disable (CW_SHUTDOWN)
  // 2. master_->stop()
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MyActuatorSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO: Implementation
  // 1. master_->exchange() (receive half)
  // 2. For each slave/joint:
  //    hw_positions_[i]  = UnitConversion::raw_to_rad(slave.get_actual_position_raw());
  //    hw_velocities_[i] = UnitConversion::raw_to_rad_per_sec(slave.get_actual_velocity_raw());
  //    hw_efforts_[i]    = UnitConversion::raw_to_torque_pct(slave.get_actual_torque_raw());
  // 3. Check for faults → return ERROR if any slave faulted

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyActuatorSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO: Implementation
  // 1. For each slave/joint:
  //    slave.set_target_position_raw(UnitConversion::rad_to_raw(hw_commands_positions_[i]));
  //    slave.set_control_mode(ControlMode::CSP);
  //    slave.set_control_word(CW_ENABLE_OP);
  // 2. master_->exchange() (send half)
  // 3. Apply slew rate limiting before writing to prevent sudden jumps

  return hardware_interface::return_type::OK;
}

}  // namespace myactuator_hardware

// Register plugin with pluginlib (uncomment when implementing)
// #include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(
//   myactuator_hardware::MyActuatorSystem,
//   hardware_interface::SystemInterface)
