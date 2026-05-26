#include "robot_safety/sustained_effort_monitor.hpp"

#include <algorithm>
#include <cmath>

namespace robot_safety
{

bool SustainedEffortMonitor::update(double abs_effort, double dt,
  double threshold, double window_sec)
{
  if (tripped_) {
    return true;  // latched until reset()
  }
  if (window_sec <= 0.0 || dt <= 0.0) {
    return false;  // monitoring disabled / no time elapsed
  }

  if (std::isfinite(abs_effort) && abs_effort >= threshold) {
    accum_sec_ += dt;                                  // fill the bucket
  } else {
    accum_sec_ = std::max(0.0, accum_sec_ - dt);       // drain on a dip
  }

  if (accum_sec_ >= window_sec) {
    tripped_ = true;
  }
  return tripped_;
}

void SustainedEffortMonitor::reset()
{
  accum_sec_ = 0.0;
  tripped_ = false;
}

}  // namespace robot_safety
