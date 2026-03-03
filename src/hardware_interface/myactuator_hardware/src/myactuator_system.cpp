#include "myactuator_hardware/myactuator_system.hpp"

#include <chrono>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "myactuator_ethercat/ethercat_master.hpp"
#include "myactuator_ethercat/myactuator_slave.hpp"
#include "myactuator_ethercat/unit_conversion.hpp"
#include "myactuator_ethercat/types.hpp"

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

  // Read hardware-level parameters from URDF <ros2_control><hardware><param>
  master_index_ = std::stoi(info_.hardware_parameters.at("ethercat_master_index"));
  network_interface_ = info_.hardware_parameters.at("network_interface");
  cycle_time_us_ = std::stoi(info_.hardware_parameters.at("cycle_time_us"));
  dc_sync_ = (info_.hardware_parameters.at("dc_sync") == "true");

  const size_t num_joints = info_.joints.size();

  // Resize state/command vectors
  hw_positions_.resize(num_joints, 0.0);
  hw_velocities_.resize(num_joints, 0.0);
  hw_efforts_.resize(num_joints, 0.0);
  hw_commands_positions_.resize(num_joints, 0.0);

  // Configure slave instances from per-joint URDF parameters
  slaves_.resize(num_joints);
  for (size_t i = 0; i < num_joints; ++i) {
    const auto & joint = info_.joints[i];

    uint16_t slave_position = static_cast<uint16_t>(
      std::stoi(joint.parameters.at("slave_position")));

    const std::string & motor_type_str = joint.parameters.at("motor_type");
    myactuator_ethercat::MotorType motor_type =
      (motor_type_str == "X6") ? myactuator_ethercat::MotorType::X6
                               : myactuator_ethercat::MotorType::X4;

    int direction = std::stoi(joint.parameters.at("direction"));
    int32_t offset_raw = static_cast<int32_t>(
      std::stol(joint.parameters.at("offset_raw")));

    slaves_[i].configure(slave_position, motor_type, direction, offset_raw);

    RCLCPP_INFO(rclcpp::get_logger("MyActuatorSystem"),
      "Joint '%s': slave_pos=%u, motor=%s, dir=%d, offset=%d",
      joint.name.c_str(), slave_position, motor_type_str.c_str(),
      direction, offset_raw);
  }

  RCLCPP_INFO(rclcpp::get_logger("MyActuatorSystem"),
    "Initialized %zu joints, master_index=%d, cycle_time=%d us",
    num_joints, master_index_, cycle_time_us_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyActuatorSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MyActuatorSystem"),
    "Configuring EtherCAT master %d...", master_index_);

  master_ = std::make_unique<myactuator_ethercat::EthercatMaster>();

  if (!master_->init(master_index_, slaves_)) {
    RCLCPP_FATAL(rclcpp::get_logger("MyActuatorSystem"),
      "Failed to initialize EtherCAT master %d", master_index_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("MyActuatorSystem"),
    "EtherCAT master configured successfully");

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
  RCLCPP_INFO(rclcpp::get_logger("MyActuatorSystem"),
    "Activating EtherCAT drives...");

  // Activate master (transitions slaves through PREOP -> SAFEOP -> OP)
  uint64_t cycle_time_ns = static_cast<uint64_t>(cycle_time_us_) * 1000ULL;
  if (!master_->start(cycle_time_ns)) {
    RCLCPP_FATAL(rclcpp::get_logger("MyActuatorSystem"),
      "Failed to activate EtherCAT master");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Wait for all slaves to reach OP state
  auto op_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (!master_->all_slaves_operational()) {
    if (std::chrono::steady_clock::now() > op_deadline) {
      RCLCPP_FATAL(rclcpp::get_logger("MyActuatorSystem"),
        "Timeout waiting for slaves to reach OP state");
      master_->stop();
      return hardware_interface::CallbackReturn::ERROR;
    }
    master_->exchange();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  RCLCPP_INFO(rclcpp::get_logger("MyActuatorSystem"),
    "All slaves in OP state. Starting CiA 402 enable sequence...");

  // Run CiA 402 enable sequence for each slave.
  // Each cycle: receive status -> step state machine -> send command.
  auto enable_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  bool all_enabled = false;

  while (!all_enabled) {
    if (std::chrono::steady_clock::now() > enable_deadline) {
      RCLCPP_FATAL(rclcpp::get_logger("MyActuatorSystem"),
        "Timeout during CiA 402 enable sequence");
      for (auto & slave : slaves_) {
        slave.disable();
      }
      master_->exchange();
      master_->stop();
      return hardware_interface::CallbackReturn::ERROR;
    }

    master_->receive();

    all_enabled = true;
    for (size_t i = 0; i < slaves_.size(); ++i) {
      if (!slaves_[i].step_enable_sequence()) {
        all_enabled = false;
      }
    }

    master_->send();

    if (!all_enabled) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("MyActuatorSystem"),
    "All drives OPERATION_ENABLED. Reading initial positions...");

  // Final receive to get accurate positions
  master_->receive();

  // Initialize state and command vectors from actual positions
  for (size_t i = 0; i < slaves_.size(); ++i) {
    hw_positions_[i] = myactuator_ethercat::UnitConversion::raw_to_rad(
      slaves_[i].get_actual_position_raw());
    hw_velocities_[i] = myactuator_ethercat::UnitConversion::raw_to_rad_per_sec(
      slaves_[i].get_actual_velocity_raw());
    hw_efforts_[i] = myactuator_ethercat::UnitConversion::raw_to_torque_pct(
      slaves_[i].get_actual_torque_raw());
    hw_commands_positions_[i] = hw_positions_[i];
  }

  RCLCPP_INFO(rclcpp::get_logger("MyActuatorSystem"),
    "EtherCAT hardware interface activated successfully");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyActuatorSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MyActuatorSystem"),
    "Deactivating EtherCAT drives...");

  // Disable all drives
  for (auto & slave : slaves_) {
    slave.disable();
  }

  // Exchange a few cycles to ensure disable command is processed
  for (int i = 0; i < 10; ++i) {
    master_->exchange();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  master_->stop();
  master_.reset();

  RCLCPP_INFO(rclcpp::get_logger("MyActuatorSystem"),
    "EtherCAT hardware interface deactivated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MyActuatorSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Receive EtherCAT frames and read TxPDOs from all slaves
  master_->receive();

  // Convert raw feedback to SI units
  for (size_t i = 0; i < slaves_.size(); ++i) {
    hw_positions_[i] = myactuator_ethercat::UnitConversion::raw_to_rad(
      slaves_[i].get_actual_position_raw());
    hw_velocities_[i] = myactuator_ethercat::UnitConversion::raw_to_rad_per_sec(
      slaves_[i].get_actual_velocity_raw());
    hw_efforts_[i] = myactuator_ethercat::UnitConversion::raw_to_torque_pct(
      slaves_[i].get_actual_torque_raw());

    // Check for drive faults
    if (slaves_[i].has_fault()) {
      RCLCPP_ERROR(rclcpp::get_logger("MyActuatorSystem"),
        "Drive fault on joint '%s' (slave %u): status=0x%04X, error=0x%04X",
        info_.joints[i].name.c_str(), slaves_[i].get_slave_position(),
        slaves_[i].get_status_word(), slaves_[i].get_error_code());
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyActuatorSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Convert SI commands to raw and set on each slave
  for (size_t i = 0; i < slaves_.size(); ++i) {
    slaves_[i].set_target_position_raw(
      myactuator_ethercat::UnitConversion::rad_to_raw(hw_commands_positions_[i]));
    slaves_[i].set_control_mode(myactuator_ethercat::ControlMode::CSP);
  }

  // Write RxPDOs and send EtherCAT frames
  master_->send();

  return hardware_interface::return_type::OK;
}

}  // namespace myactuator_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  myactuator_hardware::MyActuatorSystem,
  hardware_interface::SystemInterface)
