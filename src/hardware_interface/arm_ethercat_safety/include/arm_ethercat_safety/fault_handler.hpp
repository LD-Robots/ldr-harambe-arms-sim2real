#ifndef ARM_ETHERCAT_SAFETY__FAULT_HANDLER_HPP_
#define ARM_ETHERCAT_SAFETY__FAULT_HANDLER_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace arm_ethercat_safety
{

/// CiA 402 drive fault handler.
///
/// Monitors status words from all drives for fault conditions:
///   - Status word bit 3 (Fault) → immediate disable
///   - Status word bit 11 (Internal Limit Active) → warning
///   - Error code (0x603F) → decode and log
///
/// On fault detection:
///   1. Log the fault with joint name, status word, and error code
///   2. Signal the safety monitor to trigger protective action
///   3. Require explicit reset before re-enabling
class FaultHandler
{
public:
  struct DriveFault
  {
    std::string joint_name;
    uint16_t status_word;
    uint16_t error_code;
    std::string description;
  };

  /// Update with latest status words from all drives
  void update(
    const std::vector<std::string> & joint_names,
    const std::vector<uint16_t> & status_words,
    const std::vector<uint16_t> & error_codes);

  /// Check if any drive has an active fault
  bool has_fault() const;

  /// Get active faults
  const std::vector<DriveFault> & get_faults() const;

  /// Decode a CiA 402 error code to human-readable string
  static std::string decode_error_code(uint16_t error_code);

  /// Clear fault records (after successful reset)
  void clear();

private:
  std::vector<DriveFault> active_faults_;
};

}  // namespace arm_ethercat_safety

#endif  // ARM_ETHERCAT_SAFETY__FAULT_HANDLER_HPP_
