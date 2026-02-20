#include "arm_ethercat_safety/estop_handler.hpp"

namespace arm_ethercat_safety
{

void EstopHandler::configure(const Config & config)
{
  config_ = config;
  software_estop_ = false;
  hardware_estop_ = false;
}

void EstopHandler::update()
{
  // TODO: If gpio_pin >= 0, read GPIO state
  // hardware_estop_ = !read_gpio(config_.gpio_pin);  // Active low
  // Apply debounce filter
}

void EstopHandler::trigger_software_estop()
{
  software_estop_ = true;
}

bool EstopHandler::reset()
{
  if (hardware_estop_) {
    return false;  // Cannot reset while hardware e-stop is active
  }
  software_estop_ = false;
  return true;
}

bool EstopHandler::is_active() const
{
  return software_estop_ || hardware_estop_;
}

bool EstopHandler::is_hardware_estop() const
{
  return hardware_estop_;
}

const std::string & EstopHandler::get_reaction() const
{
  return config_.reaction;
}

}  // namespace arm_ethercat_safety
