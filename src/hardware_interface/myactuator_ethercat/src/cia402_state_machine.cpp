#include "myactuator_ethercat/cia402_state_machine.hpp"

namespace myactuator_ethercat
{

DriveState Cia402StateMachine::decode_state(uint16_t status_word)
{
  // CiA 402 state decoding from status word bits [6,5,3,2,1,0]
  // Bit masks:
  //   NOT_READY_TO_SWITCH_ON: xxxx xxxx x0xx 0000
  //   SWITCH_ON_DISABLED:     xxxx xxxx x1xx 0000
  //   READY_TO_SWITCH_ON:     xxxx xxxx x01x 0001
  //   SWITCHED_ON:            xxxx xxxx x01x 0011
  //   OPERATION_ENABLED:      xxxx xxxx x01x 0111
  //   QUICK_STOP_ACTIVE:      xxxx xxxx x00x 0111
  //   FAULT_REACTION_ACTIVE:  xxxx xxxx x0xx 1111
  //   FAULT:                  xxxx xxxx x0xx 1000

  if ((status_word & 0x004F) == 0x0000) {
    return DriveState::NOT_READY_TO_SWITCH_ON;
  }
  if ((status_word & 0x006F) == 0x0040) {
    return DriveState::SWITCH_ON_DISABLED;
  }
  if ((status_word & 0x006F) == 0x0021) {
    return DriveState::READY_TO_SWITCH_ON;
  }
  if ((status_word & 0x006F) == 0x0023) {
    return DriveState::SWITCHED_ON;
  }
  if ((status_word & 0x006F) == 0x0027) {
    return DriveState::OPERATION_ENABLED;
  }
  if ((status_word & 0x006F) == 0x0007) {
    return DriveState::QUICK_STOP_ACTIVE;
  }
  if ((status_word & 0x004F) == 0x000F) {
    return DriveState::FAULT_REACTION_ACTIVE;
  }
  if ((status_word & 0x004F) == 0x0008) {
    return DriveState::FAULT;
  }

  return DriveState::NOT_READY_TO_SWITCH_ON;
}

uint16_t Cia402StateMachine::get_transition_command(
  DriveState current, DriveState target)
{
  // If in fault, must reset first
  if (current == DriveState::FAULT) {
    return CW_FAULT_RESET;
  }

  // Enabling sequence toward OPERATION_ENABLED
  if (target == DriveState::OPERATION_ENABLED) {
    switch (current) {
      case DriveState::SWITCH_ON_DISABLED:
        return CW_SHUTDOWN;        // → READY_TO_SWITCH_ON
      case DriveState::READY_TO_SWITCH_ON:
        return CW_SWITCH_ON;       // → SWITCHED_ON
      case DriveState::SWITCHED_ON:
        return CW_ENABLE_OP;       // → OPERATION_ENABLED
      case DriveState::OPERATION_ENABLED:
        return CW_ENABLE_OP;       // Stay enabled
      default:
        return CW_DISABLE_VOLTAGE; // Safety fallback
    }
  }

  // Disabling sequence
  return CW_SHUTDOWN;
}

bool Cia402StateMachine::has_fault(uint16_t status_word)
{
  return (status_word & SW_FAULT) != 0;
}

bool Cia402StateMachine::target_reached(uint16_t status_word)
{
  return (status_word & SW_TARGET_REACHED) != 0;
}

bool Cia402StateMachine::internal_limit_active(uint16_t status_word)
{
  return (status_word & SW_INTERNAL_LIMIT) != 0;
}

uint16_t Cia402StateMachine::fault_reset_command()
{
  return CW_FAULT_RESET;
}

}  // namespace myactuator_ethercat
