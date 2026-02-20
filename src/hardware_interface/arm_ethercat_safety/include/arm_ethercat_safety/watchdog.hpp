#ifndef ARM_ETHERCAT_SAFETY__WATCHDOG_HPP_
#define ARM_ETHERCAT_SAFETY__WATCHDOG_HPP_

#include <chrono>
#include <string>

namespace arm_ethercat_safety
{

/// Communication watchdog.
///
/// Monitors:
///   - Time since last valid joint_state message received
///   - EtherCAT frame loss counter (from hardware interface status)
///
/// Triggers protective action if:
///   - No joint state update within command_timeout_ms
///   - Frame loss count exceeds ethercat_frame_loss_limit
///
/// Actions:
///   - "hold_position" — CSP: hold last known position
///   - "disable" — immediately disable all drives
class Watchdog
{
public:
  struct Config
  {
    int command_timeout_ms{50};
    int ethercat_frame_loss_limit{3};
    std::string action_on_timeout{"hold_position"};
  };

  void configure(const Config & config);

  /// Call each cycle with current timestamp
  void update(std::chrono::steady_clock::time_point now);

  /// Mark that a valid command/state was received
  void feed();

  /// Check if watchdog has timed out
  bool is_timed_out() const;

  /// Get the configured action on timeout
  const std::string & get_timeout_action() const;

  /// Reset watchdog state
  void reset();

private:
  Config config_;
  std::chrono::steady_clock::time_point last_feed_time_;
  bool timed_out_{false};
};

}  // namespace arm_ethercat_safety

#endif  // ARM_ETHERCAT_SAFETY__WATCHDOG_HPP_
