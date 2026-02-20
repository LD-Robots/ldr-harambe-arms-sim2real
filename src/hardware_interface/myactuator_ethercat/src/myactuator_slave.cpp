#include "myactuator_ethercat/myactuator_slave.hpp"
#include <ecrt.h>
#include <cstdio>

namespace myactuator_ethercat
{

void MyActuatorSlave::configure(
  uint16_t slave_position, MotorType motor_type,
  int direction, int32_t offset_raw)
{
  slave_position_ = slave_position;
  motor_type_ = motor_type;
  direction_ = direction;
  offset_raw_ = offset_raw;

  // Zero out PDO buffers — control_word = 0x0000 (DISABLE_VOLTAGE)
  rx_pdo_ = RxPdo{};
  tx_pdo_ = TxPdo{};
}

bool MyActuatorSlave::register_pdos(void * master_handle, void * domain_handle)
{
  auto * master = static_cast<ec_master_t *>(master_handle);
  auto * domain = static_cast<ec_domain_t *>(domain_handle);

  // Get slave configuration
  ec_slave_config_t * sc = ecrt_master_slave_config(
    master,
    0,                             // alias
    slave_position_,               // bus position
    DeviceConstants::VENDOR_ID,    // 0x00202008
    DeviceConstants::PRODUCT_CODE  // 0x00000000
  );
  if (!sc) {
    fprintf(stderr, "ecrt_master_slave_config() failed for slave %u\n", slave_position_);
    return false;
  }
  slave_config_ = sc;

  // Define PDO entry mappings matching ESI MT-Device-250702.xml exactly
  ec_pdo_entry_info_t rx_pdo_entries[] = {
    {0x6040, 0x00, 16},  // control_word
    {0x607A, 0x00, 32},  // target_position
    {0x60FF, 0x00, 32},  // target_velocity
    {0x6071, 0x00, 16},  // target_torque
    {0x6072, 0x00, 16},  // max_torque
    {0x6060, 0x00,  8},  // mode_of_operation
    {0x5FF1, 0x00,  8},  // padding
  };

  ec_pdo_entry_info_t tx_pdo_entries[] = {
    {0x6041, 0x00, 16},  // status_word
    {0x6064, 0x00, 32},  // position_actual
    {0x606C, 0x00, 32},  // velocity_actual
    {0x6077, 0x00, 16},  // torque_actual
    {0x603F, 0x00, 16},  // error_code
    {0x6061, 0x00,  8},  // mode_display
    {0x5FF2, 0x00,  8},  // padding
  };

  ec_pdo_info_t pdos[] = {
    {0x1600, 7, rx_pdo_entries},  // RxPDO mapping 0
    {0x1A00, 7, tx_pdo_entries},  // TxPDO mapping 0
  };

  // Sync manager configuration
  // SM0/SM1 = mailbox (default), SM2 = outputs (RxPDO), SM3 = inputs (TxPDO)
  ec_sync_info_t syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL,     EC_WD_DISABLE},
    {1, EC_DIR_INPUT,  0, NULL,     EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, &pdos[0], EC_WD_ENABLE},
    {3, EC_DIR_INPUT,  1, &pdos[1], EC_WD_DISABLE},
    {0xFF, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE}
  };

  if (ecrt_slave_config_pdos(sc, EC_END, syncs)) {
    fprintf(stderr, "ecrt_slave_config_pdos() failed for slave %u\n", slave_position_);
    return false;
  }

  // Register PDO entries for process data access
  // pdo_offsets_[0..6] = RxPDO, pdo_offsets_[7..13] = TxPDO
  ec_pdo_entry_reg_t pdo_regs[] = {
    // RxPDO entries
    {0, slave_position_, DeviceConstants::VENDOR_ID, DeviceConstants::PRODUCT_CODE,
     0x6040, 0x00, &pdo_offsets_[0], NULL},
    {0, slave_position_, DeviceConstants::VENDOR_ID, DeviceConstants::PRODUCT_CODE,
     0x607A, 0x00, &pdo_offsets_[1], NULL},
    {0, slave_position_, DeviceConstants::VENDOR_ID, DeviceConstants::PRODUCT_CODE,
     0x60FF, 0x00, &pdo_offsets_[2], NULL},
    {0, slave_position_, DeviceConstants::VENDOR_ID, DeviceConstants::PRODUCT_CODE,
     0x6071, 0x00, &pdo_offsets_[3], NULL},
    {0, slave_position_, DeviceConstants::VENDOR_ID, DeviceConstants::PRODUCT_CODE,
     0x6072, 0x00, &pdo_offsets_[4], NULL},
    {0, slave_position_, DeviceConstants::VENDOR_ID, DeviceConstants::PRODUCT_CODE,
     0x6060, 0x00, &pdo_offsets_[5], NULL},
    {0, slave_position_, DeviceConstants::VENDOR_ID, DeviceConstants::PRODUCT_CODE,
     0x5FF1, 0x00, &pdo_offsets_[6], NULL},
    // TxPDO entries
    {0, slave_position_, DeviceConstants::VENDOR_ID, DeviceConstants::PRODUCT_CODE,
     0x6041, 0x00, &pdo_offsets_[7], NULL},
    {0, slave_position_, DeviceConstants::VENDOR_ID, DeviceConstants::PRODUCT_CODE,
     0x6064, 0x00, &pdo_offsets_[8], NULL},
    {0, slave_position_, DeviceConstants::VENDOR_ID, DeviceConstants::PRODUCT_CODE,
     0x606C, 0x00, &pdo_offsets_[9], NULL},
    {0, slave_position_, DeviceConstants::VENDOR_ID, DeviceConstants::PRODUCT_CODE,
     0x6077, 0x00, &pdo_offsets_[10], NULL},
    {0, slave_position_, DeviceConstants::VENDOR_ID, DeviceConstants::PRODUCT_CODE,
     0x603F, 0x00, &pdo_offsets_[11], NULL},
    {0, slave_position_, DeviceConstants::VENDOR_ID, DeviceConstants::PRODUCT_CODE,
     0x6061, 0x00, &pdo_offsets_[12], NULL},
    {0, slave_position_, DeviceConstants::VENDOR_ID, DeviceConstants::PRODUCT_CODE,
     0x5FF2, 0x00, &pdo_offsets_[13], NULL},
    // Terminator
    {0, 0, 0, 0, 0, 0, NULL, NULL}
  };

  if (ecrt_domain_reg_pdo_entry_list(domain, pdo_regs)) {
    fprintf(stderr, "ecrt_domain_reg_pdo_entry_list() failed for slave %u\n", slave_position_);
    return false;
  }

  return true;
}

void MyActuatorSlave::configure_dc(uint32_t cycle_time_ns)
{
  if (!slave_config_) return;
  auto * sc = static_cast<ec_slave_config_t *>(slave_config_);
  ecrt_slave_config_dc(sc, DeviceConstants::DC_ASSIGN_ACTIVATE,
                       cycle_time_ns, 0, 0, 0);
}

void MyActuatorSlave::read_pdos(const uint8_t * domain_pd)
{
  tx_pdo_.status_word     = EC_READ_U16(domain_pd + pdo_offsets_[7]);
  tx_pdo_.position_actual = EC_READ_S32(domain_pd + pdo_offsets_[8]);
  tx_pdo_.velocity_actual = EC_READ_S32(domain_pd + pdo_offsets_[9]);
  tx_pdo_.torque_actual   = static_cast<int16_t>(EC_READ_U16(domain_pd + pdo_offsets_[10]));
  tx_pdo_.error_code      = EC_READ_U16(domain_pd + pdo_offsets_[11]);
  tx_pdo_.mode_display    = static_cast<int8_t>(EC_READ_U8(domain_pd + pdo_offsets_[12]));
}

void MyActuatorSlave::write_pdos(uint8_t * domain_pd)
{
  EC_WRITE_U16(domain_pd + pdo_offsets_[0], rx_pdo_.control_word);
  EC_WRITE_S32(domain_pd + pdo_offsets_[1], rx_pdo_.target_position);
  EC_WRITE_S32(domain_pd + pdo_offsets_[2], rx_pdo_.target_velocity);
  EC_WRITE_S16(domain_pd + pdo_offsets_[3], rx_pdo_.target_torque);
  EC_WRITE_U16(domain_pd + pdo_offsets_[4], rx_pdo_.max_torque);
  EC_WRITE_S8(domain_pd + pdo_offsets_[5],  rx_pdo_.mode_of_operation);
}

bool MyActuatorSlave::step_enable_sequence()
{
  // TODO: Implementation for when we're ready to enable drives
  // 1. Decode current state from tx_pdo_.status_word
  // 2. If FAULT → send fault reset
  // 3. If SWITCH_ON_DISABLED → send CW_SHUTDOWN (0x0006)
  // 4. If READY_TO_SWITCH_ON → send CW_SWITCH_ON (0x0007)
  // 5. If SWITCHED_ON → send CW_ENABLE_OP (0x000F)
  //    CRITICAL: Set target_position = position_actual BEFORE enabling
  // 6. Return true when OPERATION_ENABLED
  return false;
}

void MyActuatorSlave::disable()
{
  rx_pdo_.control_word = Cia402StateMachine::CW_SHUTDOWN;
}

void MyActuatorSlave::quick_stop()
{
  rx_pdo_.control_word = Cia402StateMachine::CW_QUICK_STOP;
}

void MyActuatorSlave::fault_reset()
{
  rx_pdo_.control_word = Cia402StateMachine::CW_FAULT_RESET;
}

void MyActuatorSlave::set_target_position_raw(int32_t raw)
{
  rx_pdo_.target_position = raw * direction_ + offset_raw_;
}

void MyActuatorSlave::set_target_velocity_raw(int32_t raw)
{
  rx_pdo_.target_velocity = raw * direction_;
}

void MyActuatorSlave::set_target_torque_raw(int16_t raw)
{
  rx_pdo_.target_torque = raw * static_cast<int16_t>(direction_);
}

void MyActuatorSlave::set_max_torque_raw(uint16_t raw)
{
  rx_pdo_.max_torque = raw;
}

void MyActuatorSlave::set_control_mode(ControlMode mode)
{
  rx_pdo_.mode_of_operation = static_cast<int8_t>(mode);
}

DriveState MyActuatorSlave::get_drive_state() const
{
  return Cia402StateMachine::decode_state(tx_pdo_.status_word);
}

int32_t MyActuatorSlave::get_actual_position_raw() const
{
  return (tx_pdo_.position_actual - offset_raw_) * direction_;
}

int32_t MyActuatorSlave::get_actual_velocity_raw() const
{
  return tx_pdo_.velocity_actual * direction_;
}

int16_t MyActuatorSlave::get_actual_torque_raw() const
{
  return tx_pdo_.torque_actual * static_cast<int16_t>(direction_);
}

uint16_t MyActuatorSlave::get_status_word() const
{
  return tx_pdo_.status_word;
}

uint16_t MyActuatorSlave::get_error_code() const
{
  return tx_pdo_.error_code;
}

ControlMode MyActuatorSlave::get_current_mode() const
{
  return static_cast<ControlMode>(tx_pdo_.mode_display);
}

bool MyActuatorSlave::has_fault() const
{
  return Cia402StateMachine::has_fault(tx_pdo_.status_word);
}

uint16_t MyActuatorSlave::get_slave_position() const
{
  return slave_position_;
}

MotorType MyActuatorSlave::get_motor_type() const
{
  return motor_type_;
}

}  // namespace myactuator_ethercat
