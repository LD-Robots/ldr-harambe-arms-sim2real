#ifndef ROBOT_SAFETY__SUSTAINED_EFFORT_MONITOR_HPP_
#define ROBOT_SAFETY__SUSTAINED_EFFORT_MONITOR_HPP_

namespace robot_safety
{

/// Leaky-bucket detector for sustained high effort. Accumulates time spent
/// with |effort| above a threshold and trips once that exceeds a window;
/// brief dips below the threshold drain the accumulator rather than resetting
/// it. Once tripped it latches until reset(). Realtime-safe.
class SustainedEffortMonitor
{
public:
  SustainedEffortMonitor() = default;

  /// Advance the monitor by one cycle.
  /// \param abs_effort  |effort| this cycle
  /// \param dt          seconds since the previous call
  /// \param threshold   effort magnitude considered "high"
  /// \param window_sec  accumulated above-threshold time that trips the monitor
  /// \return true once tripped (latched until reset())
  bool update(double abs_effort, double dt, double threshold, double window_sec);

  /// Clear the accumulator and the tripped latch.
  void reset();

  /// Accumulated above-threshold time, in seconds.
  double timeAboveThreshold() const { return accum_sec_; }

  /// True once the monitor has tripped.
  bool tripped() const { return tripped_; }

private:
  double accum_sec_ = 0.0;
  bool   tripped_ = false;
};

}  // namespace robot_safety

#endif  // ROBOT_SAFETY__SUSTAINED_EFFORT_MONITOR_HPP_
