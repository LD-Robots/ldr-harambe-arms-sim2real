#ifndef ROBOT_SAFETY__RATE_LIMITER_HPP_
#define ROBOT_SAFETY__RATE_LIMITER_HPP_

namespace robot_safety
{

/// Stateful slew-rate and acceleration limiter for a single position-command
/// stream. Realtime-safe: a handful of scalars, no allocation, no locking.
///
/// One instance per command stream — controllers instantiate 25 of these for
/// the whole-body PVT path. The first limit() call after construction or
/// reset() seeds the internal state to the requested value, so there is no
/// startup jump.
class RateLimiter
{
public:
  RateLimiter() = default;

  /// Limit `desired` so the emitted command respects `slew_rate` (rad/s, the
  /// first derivative) and `accel_limit` (rad/s^2, the second derivative).
  /// A non-positive `slew_rate` or `accel_limit` disables that layer.
  /// The profile is deceleration-aware: it brakes early enough to settle on
  /// the target without overshoot or ringing, even for a large step input.
  double limit(double desired, double dt, double slew_rate, double accel_limit);

  /// Forget all state; the next limit() call re-seeds (use on (re)activation).
  void reset();

  /// Explicitly seed the internal state to a known position and (optional)
  /// command velocity. Seeding the velocity matters when entering a HOLD from
  /// motion: the limiter then continues from the joint's actual speed and
  /// decelerates smoothly, instead of starting from rest while the joint
  /// overruns the command.
  void seed(double position, double velocity = 0.0);

  bool seeded() const { return seeded_; }

private:
  bool   seeded_ = false;
  double prev_cmd_ = 0.0;
  double prev_vel_ = 0.0;
};

}  // namespace robot_safety

#endif  // ROBOT_SAFETY__RATE_LIMITER_HPP_
