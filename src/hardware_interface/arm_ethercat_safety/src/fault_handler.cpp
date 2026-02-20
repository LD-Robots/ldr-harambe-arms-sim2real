#include "arm_ethercat_safety/fault_handler.hpp"

namespace arm_ethercat_safety
{

void FaultHandler::update(
  const std::vector<std::string> & joint_names,
  const std::vector<uint16_t> & status_words,
  const std::vector<uint16_t> & error_codes)
{
  active_faults_.clear();

  for (size_t i = 0; i < joint_names.size(); ++i) {
    // Check status word bit 3 (Fault)
    if (i < status_words.size() && (status_words[i] & 0x0008) != 0) {
      uint16_t ec = (i < error_codes.size()) ? error_codes[i] : 0;
      active_faults_.push_back({
        joint_names[i],
        status_words[i],
        ec,
        decode_error_code(ec)
      });
    }
  }
}

bool FaultHandler::has_fault() const
{
  return !active_faults_.empty();
}

const std::vector<FaultHandler::DriveFault> & FaultHandler::get_faults() const
{
  return active_faults_;
}

std::string FaultHandler::decode_error_code(uint16_t error_code)
{
  // CiA 402 standard error codes
  switch (error_code) {
    case 0x0000: return "No error";
    case 0x1000: return "Generic error";
    case 0x2310: return "Over current";
    case 0x3210: return "DC link over voltage";
    case 0x3220: return "DC link under voltage";
    case 0x4210: return "Over temperature drive";
    case 0x4310: return "Over temperature motor";
    case 0x5441: return "Motor blocked / following error";
    case 0x7121: return "Motor commutation error";
    case 0x7300: return "Sensor error";
    case 0x7500: return "Communication error";
    case 0x8611: return "Position limit exceeded";
    case 0xFF01: return "EtherCAT sync error";
    case 0xFF02: return "EtherCAT watchdog error";
    default:     return "Unknown error (0x" + std::to_string(error_code) + ")";
  }
}

void FaultHandler::clear()
{
  active_faults_.clear();
}

}  // namespace arm_ethercat_safety
