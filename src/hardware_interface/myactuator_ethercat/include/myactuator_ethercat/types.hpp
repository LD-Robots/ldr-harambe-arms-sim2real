#ifndef MYACTUATOR_ETHERCAT__TYPES_HPP_
#define MYACTUATOR_ETHERCAT__TYPES_HPP_

#include <cstdint>

namespace myactuator_ethercat
{

/// MyActuator RMD X V4 motor variants
enum class MotorType : uint8_t
{
  X4 = 4,  // Smaller actuator (wrist, shoulder yaw)
  X6 = 6,  // Larger actuator (shoulder pitch/roll, elbow)
};

/// CiA 402 control modes (0x6060 modes of operation)
enum class ControlMode : int8_t
{
  CSP = 8,   // Cyclic Synchronous Position
  CSV = 9,   // Cyclic Synchronous Velocity
  CST = 10,  // Cyclic Synchronous Torque
};

/// CiA 402 drive state (decoded from status word 0x6041)
enum class DriveState : uint8_t
{
  NOT_READY_TO_SWITCH_ON,
  SWITCH_ON_DISABLED,
  READY_TO_SWITCH_ON,
  SWITCHED_ON,
  OPERATION_ENABLED,
  QUICK_STOP_ACTIVE,
  FAULT_REACTION_ACTIVE,
  FAULT,
};

/// EtherCAT slave operational state
enum class SlaveState : uint8_t
{
  INIT   = 0x01,
  PREOP  = 0x02,
  SAFEOP = 0x04,
  OP     = 0x08,
};

/// MyActuator vendor constants (from ESI file)
struct DeviceConstants
{
  static constexpr uint32_t VENDOR_ID     = 0x00202008;
  static constexpr uint32_t PRODUCT_CODE  = 0x00000000;
  static constexpr uint32_t REVISION      = 0x00010000;

  // PDO indices
  static constexpr uint16_t RXPDO_INDEX   = 0x1600;
  static constexpr uint16_t TXPDO_INDEX   = 0x1A00;

  // DC sync
  static constexpr uint16_t DC_ASSIGN_ACTIVATE = 0x0300;
  static constexpr uint32_t MIN_CYCLE_TIME_NS  = 250000;  // 250 µs (from ESI: 0x03D090)
};

}  // namespace myactuator_ethercat

#endif  // MYACTUATOR_ETHERCAT__TYPES_HPP_
