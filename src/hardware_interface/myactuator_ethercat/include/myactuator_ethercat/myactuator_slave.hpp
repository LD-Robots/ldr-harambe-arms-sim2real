#ifndef MYACTUATOR_ETHERCAT__MYACTUATOR_SLAVE_HPP_
#define MYACTUATOR_ETHERCAT__MYACTUATOR_SLAVE_HPP_

#include <cstdint>
#include <string>
#include "myactuator_ethercat/types.hpp"
#include "myactuator_ethercat/pdo_types.hpp"
#include "myactuator_ethercat/cia402_state_machine.hpp"

namespace myactuator_ethercat
{

/// Represents a single MyActuator RMD X V4 EtherCAT slave drive.
///
/// Manages PDO data exchange, CiA 402 state machine, and provides
/// safe read/write access to process data for one actuator.
class MyActuatorSlave
{
public:
  /// Configure this slave with its bus position and motor type
  void configure(uint16_t slave_position, MotorType motor_type,
                 int direction = 1, int32_t offset_raw = 0);

  /// Register PDO entries with the EtherCAT domain (called during master config)
  /// @param master  Opaque pointer to ec_master_t
  /// @param domain  Opaque pointer to ec_domain_t
  /// @return true on success
  bool register_pdos(void * master, void * domain);

  /// Configure distributed clocks for this slave (call after register_pdos, before activate)
  /// @param cycle_time_ns  SYNC0 cycle time in nanoseconds
  void configure_dc(uint32_t cycle_time_ns);

  /// Read TxPDO data from process data image (called each cycle)
  void read_pdos(const uint8_t * domain_pd);

  /// Write RxPDO data to process data image (called each cycle)
  void write_pdos(uint8_t * domain_pd);

  /// Execute one step of the CiA 402 enable sequence
  /// Returns true when OPERATION_ENABLED is reached
  bool step_enable_sequence();

  /// Execute disable: send shutdown command
  void disable();

  /// Execute quick stop
  void quick_stop();

  /// Attempt fault reset (rising edge on control word bit 7)
  void fault_reset();

  // --- Setters (command side) ---
  void set_target_position_raw(int32_t raw);
  void set_target_velocity_raw(int32_t raw);
  void set_target_torque_raw(int16_t raw);
  void set_max_torque_raw(uint16_t raw);
  void set_control_mode(ControlMode mode);

  // --- Getters (feedback side) ---
  DriveState get_drive_state() const;
  int32_t get_actual_position_raw() const;
  int32_t get_actual_velocity_raw() const;
  int16_t get_actual_torque_raw() const;
  uint16_t get_status_word() const;
  uint16_t get_error_code() const;
  ControlMode get_current_mode() const;
  bool has_fault() const;

  // --- Configuration ---
  uint16_t get_slave_position() const;
  MotorType get_motor_type() const;

private:
  uint16_t slave_position_{0};
  MotorType motor_type_{MotorType::X4};
  int direction_{1};
  int32_t offset_raw_{0};

  RxPdo rx_pdo_{};
  TxPdo tx_pdo_{};

  void * slave_config_{nullptr};  // ec_slave_config_t*

  // PDO byte offsets within the domain process data image
  // [0..6] = RxPDO (control_word, target_pos, target_vel, target_torque, max_torque, mode, pad)
  // [7..13] = TxPDO (status_word, pos_actual, vel_actual, torque_actual, error_code, mode_disp, pad)
  unsigned int pdo_offsets_[14]{};
};

}  // namespace myactuator_ethercat

#endif  // MYACTUATOR_ETHERCAT__MYACTUATOR_SLAVE_HPP_
