#ifndef MYACTUATOR_ETHERCAT__PDO_TYPES_HPP_
#define MYACTUATOR_ETHERCAT__PDO_TYPES_HPP_

#include <cstdint>

namespace myactuator_ethercat
{

/// RxPDO 0x1600 — Master → Slave (16 bytes)
/// Matches ESI MT-Device-250702.xml RCV PDO Mapping0 exactly.
struct __attribute__((packed)) RxPdo
{
  uint16_t control_word;       // 0x6040 — CiA 402 control word
  int32_t  target_position;    // 0x607A — CSP target (raw: ±65535 = ±180°)
  int32_t  target_velocity;    // 0x60FF — CSV target (raw: see unit_conversion)
  int16_t  target_torque;      // 0x6071 — CST target (0–1000 = 0–100% of rated current)
  uint16_t max_torque;         // 0x6072 — Torque limit (0–1000)
  int8_t   mode_of_operation;  // 0x6060 — 8=CSP, 9=CSV, 10=CST
  uint8_t  padding;            // 0x5FF1 — Alignment byte
};
static_assert(sizeof(RxPdo) == 16, "RxPdo must be 16 bytes to match PDO mapping");

/// TxPDO 0x1A00 — Slave → Master (16 bytes)
/// Matches ESI MT-Device-250702.xml SND PDO Mapping0 exactly.
struct __attribute__((packed)) TxPdo
{
  uint16_t status_word;        // 0x6041 — CiA 402 status word
  int32_t  position_actual;    // 0x6064 — Actual position (raw)
  int32_t  velocity_actual;    // 0x606C — Actual velocity (raw)
  int16_t  torque_actual;      // 0x6077 — Actual torque (thousandths of rated current)
  uint16_t error_code;         // 0x603F — Drive error code
  int8_t   mode_display;       // 0x6061 — Current operating mode
  uint8_t  padding;            // 0x5FF2 — Alignment byte
};
static_assert(sizeof(TxPdo) == 16, "TxPdo must be 16 bytes to match PDO mapping");

/// Alternative RxPDO 0x1601 with PID gains (for tuning)
struct __attribute__((packed)) RxPdoWithGains
{
  uint16_t control_word;       // 0x6040
  int32_t  target_position;    // 0x607A
  int32_t  target_velocity;    // 0x60FF
  int16_t  target_torque;      // 0x6071
  int32_t  pvt_kp;             // 0x2000 — Position/velocity P gain
  int32_t  pvt_kd;             // 0x2001 — Position/velocity D gain
  int8_t   mode_of_operation;  // 0x6060
  uint8_t  padding;            // 0x5FF1
};
static_assert(sizeof(RxPdoWithGains) == 22, "RxPdoWithGains must be 22 bytes");

}  // namespace myactuator_ethercat

#endif  // MYACTUATOR_ETHERCAT__PDO_TYPES_HPP_
