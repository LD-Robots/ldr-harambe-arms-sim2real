#ifndef MYACTUATOR_HARDWARE__MYACTUATOR_SYSTEM_HPP_
#define MYACTUATOR_HARDWARE__MYACTUATOR_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

// #include "myactuator_ethercat/ethercat_master.hpp"
// #include "myactuator_ethercat/myactuator_slave.hpp"

namespace myactuator_hardware
{

/// ros2_control SystemInterface for MyActuator RMD X V4 EtherCAT drives.
///
/// This plugin bridges the EtherCAT driver (myactuator_ethercat) to the
/// ros2_control framework. It:
///   - Reads hardware parameters from URDF <ros2_control> tags
///   - Initializes the EtherLab master and configures slave drives
///   - Provides position/velocity/effort state interfaces
///   - Accepts position command interface
///   - Runs the CiA 402 enable/disable sequence on activate/deactivate
///   - Performs cyclic PDO exchange in read()/write()
///
/// Hardware parameters (set via URDF <param> tags):
///   - ethercat_master_index: EtherLab master index (default 0)
///   - network_interface: NIC name (e.g., "eth0")
///   - cycle_time_us: Cycle time in microseconds (default 1000)
///   - dc_sync: Enable distributed clocks (default true)
///
/// Per-joint parameters (set via URDF <param> tags on each joint):
///   - slave_position: EtherCAT slave position on the bus
///   - motor_type: "X4" or "X6"
///   - direction: 1 or -1
///   - offset_raw: Raw encoder offset
class MyActuatorSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MyActuatorSystem)

  /// Read hardware parameters from URDF
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  /// Initialize EtherLab master and configure slaves
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /// Export state interfaces (position, velocity, effort per joint)
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /// Export command interfaces (position per joint)
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /// CiA 402 enable sequence: bring all drives to OPERATION_ENABLED
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /// CiA 402 disable sequence: bring all drives to SWITCHED_ON
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /// Read TxPDO data from drives → update state interfaces
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Write command interfaces → RxPDO data to drives
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Per-joint state storage
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  // Per-joint command storage
  std::vector<double> hw_commands_positions_;

  // EtherCAT (uncomment when implementing)
  // std::unique_ptr<myactuator_ethercat::EthercatMaster> master_;
  // std::vector<myactuator_ethercat::MyActuatorSlave> slaves_;

  // Configuration
  int master_index_{0};
  std::string network_interface_{"eth0"};
  int cycle_time_us_{1000};
  bool dc_sync_{true};
};

}  // namespace myactuator_hardware

#endif  // MYACTUATOR_HARDWARE__MYACTUATOR_SYSTEM_HPP_
