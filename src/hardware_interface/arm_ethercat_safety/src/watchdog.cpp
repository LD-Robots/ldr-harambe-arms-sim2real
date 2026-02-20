#include "arm_ethercat_safety/watchdog.hpp"

namespace arm_ethercat_safety
{

void Watchdog::configure(const Config & config)
{
  config_ = config;
  reset();
}

void Watchdog::update(std::chrono::steady_clock::time_point now)
{
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - last_feed_time_);

  timed_out_ = (elapsed.count() > config_.command_timeout_ms);
}

void Watchdog::feed()
{
  last_feed_time_ = std::chrono::steady_clock::now();
  timed_out_ = false;
}

bool Watchdog::is_timed_out() const
{
  return timed_out_;
}

const std::string & Watchdog::get_timeout_action() const
{
  return config_.action_on_timeout;
}

void Watchdog::reset()
{
  last_feed_time_ = std::chrono::steady_clock::now();
  timed_out_ = false;
}

}  // namespace arm_ethercat_safety
