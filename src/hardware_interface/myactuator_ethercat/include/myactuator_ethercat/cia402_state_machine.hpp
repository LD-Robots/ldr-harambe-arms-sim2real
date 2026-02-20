#ifndef MYACTUATOR_ETHERCAT__CIA402_STATE_MACHINE_HPP_
#define MYACTUATOR_ETHERCAT__CIA402_STATE_MACHINE_HPP_

#include <cstdint>
#include "myactuator_ethercat/types.hpp"

namespace myactuator_ethercat
{

/// CiA 402 drive profile state machine.
///
/// Manages transitions of the control word (0x6040) based on
/// the current status word (0x6041) to safely enable/disable drives.
///
/// Enable sequence:  0x0006 → 0x0007 → 0x000F
/// Disable sequence: 0x0006
/// Quick stop:       clear bit 2 of control word
/// Fault reset:      rising edge on bit 7 of control word
class Cia402StateMachine
{
public:
  /// Decode the drive state from the status word (0x6041)
  static DriveState decode_state(uint16_t status_word);

  /// Get the control word needed to transition toward the target state
  static uint16_t get_transition_command(DriveState current, DriveState target);

  /// Check if a fault is active (status word bit 3)
  static bool has_fault(uint16_t status_word);

  /// Check if target is reached (status word bit 10)
  static bool target_reached(uint16_t status_word);

  /// Check if internal limit is active (status word bit 11)
  static bool internal_limit_active(uint16_t status_word);

  /// Get the fault reset control word (rising edge on bit 7)
  static uint16_t fault_reset_command();

  /// Control word constants
  static constexpr uint16_t CW_SHUTDOWN        = 0x0006;  // Ready to Switch On
  static constexpr uint16_t CW_SWITCH_ON       = 0x0007;  // Switched On
  static constexpr uint16_t CW_ENABLE_OP       = 0x000F;  // Operation Enabled
  static constexpr uint16_t CW_DISABLE_VOLTAGE = 0x0000;  // Switch On Disabled
  static constexpr uint16_t CW_QUICK_STOP      = 0x0002;  // Quick Stop
  static constexpr uint16_t CW_FAULT_RESET     = 0x0080;  // Fault Reset (bit 7)

  /// Status word bit masks
  static constexpr uint16_t SW_READY_TO_SWITCH_ON  = 0x0001;
  static constexpr uint16_t SW_SWITCHED_ON          = 0x0002;
  static constexpr uint16_t SW_OPERATION_ENABLED    = 0x0004;
  static constexpr uint16_t SW_FAULT                = 0x0008;
  static constexpr uint16_t SW_VOLTAGE_ENABLED      = 0x0010;
  static constexpr uint16_t SW_QUICK_STOP           = 0x0020;
  static constexpr uint16_t SW_SWITCH_ON_DISABLED   = 0x0040;
  static constexpr uint16_t SW_WARNING              = 0x0080;
  static constexpr uint16_t SW_TARGET_REACHED       = 0x0400;
  static constexpr uint16_t SW_INTERNAL_LIMIT       = 0x0800;
};

}  // namespace myactuator_ethercat

#endif  // MYACTUATOR_ETHERCAT__CIA402_STATE_MACHINE_HPP_
