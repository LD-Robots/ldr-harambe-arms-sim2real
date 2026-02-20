#ifndef ARM_ETHERCAT_SAFETY__ESTOP_HANDLER_HPP_
#define ARM_ETHERCAT_SAFETY__ESTOP_HANDLER_HPP_

#include <string>

namespace arm_ethercat_safety
{

/// Emergency stop handler.
///
/// Supports both hardware (GPIO) and software e-stop:
///   - Hardware: reads GPIO pin state (active low with debounce)
///   - Software: triggered via ROS service /safety/estop
///
/// Reactions:
///   - "quickstop" — CiA 402 quick stop (controlled deceleration)
///   - "immediate_disable" — immediate drive disable (coast to stop)
class EstopHandler
{
public:
  struct Config
  {
    int gpio_pin{-1};               // -1 = no hardware e-stop
    int debounce_ms{10};
    std::string reaction{"quickstop"};
  };

  void configure(const Config & config);

  /// Check hardware GPIO e-stop state (if configured)
  void update();

  /// Trigger software e-stop
  void trigger_software_estop();

  /// Reset e-stop (only if hardware e-stop is released)
  bool reset();

  /// Check if e-stop is currently active
  bool is_active() const;

  /// Check if this is a hardware or software e-stop
  bool is_hardware_estop() const;

  /// Get the configured reaction type
  const std::string & get_reaction() const;

private:
  Config config_;
  bool software_estop_{false};
  bool hardware_estop_{false};
};

}  // namespace arm_ethercat_safety

#endif  // ARM_ETHERCAT_SAFETY__ESTOP_HANDLER_HPP_
