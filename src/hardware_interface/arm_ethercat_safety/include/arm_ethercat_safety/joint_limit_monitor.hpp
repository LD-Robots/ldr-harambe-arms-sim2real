#ifndef ARM_ETHERCAT_SAFETY__JOINT_LIMIT_MONITOR_HPP_
#define ARM_ETHERCAT_SAFETY__JOINT_LIMIT_MONITOR_HPP_

#include <string>
#include <vector>

namespace arm_ethercat_safety
{

/// Joint limit monitor.
///
/// Checks position, velocity, and torque against configured limits
/// for each joint every cycle. Reports violations with severity levels:
///   - WARNING: approaching limit (within margin)
///   - ERROR: at or beyond limit → triggers protective action
///
/// Limits are loaded from safety_limits.yaml config file.
class JointLimitMonitor
{
public:
  struct JointLimits
  {
    std::string joint_name;
    double position_min{-3.14};
    double position_max{3.14};
    double velocity_max{2.0};
    double torque_max_pct{80.0};
    double position_margin{0.1};  // Warning margin in rad
  };

  enum class Severity
  {
    OK,
    WARNING,
    ERROR,
  };

  struct Violation
  {
    std::string joint_name;
    std::string limit_type;  // "position", "velocity", "torque"
    Severity severity;
    double actual_value;
    double limit_value;
  };

  void configure(const std::vector<JointLimits> & limits);

  /// Check all joints against limits. Returns violations (empty = all OK).
  std::vector<Violation> check(
    const std::vector<double> & positions,
    const std::vector<double> & velocities,
    const std::vector<double> & efforts);

  /// Check if any ERROR-level violation exists
  bool has_error() const;

  /// Get the most recent violations
  const std::vector<Violation> & get_violations() const;

private:
  std::vector<JointLimits> limits_;
  std::vector<Violation> violations_;
};

}  // namespace arm_ethercat_safety

#endif  // ARM_ETHERCAT_SAFETY__JOINT_LIMIT_MONITOR_HPP_
