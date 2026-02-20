#include "arm_ethercat_safety/joint_limit_monitor.hpp"

#include <cmath>

namespace arm_ethercat_safety
{

void JointLimitMonitor::configure(const std::vector<JointLimits> & limits)
{
  limits_ = limits;
}

std::vector<JointLimitMonitor::Violation> JointLimitMonitor::check(
  const std::vector<double> & positions,
  const std::vector<double> & velocities,
  const std::vector<double> & efforts)
{
  violations_.clear();

  for (size_t i = 0; i < limits_.size() && i < positions.size(); ++i) {
    const auto & lim = limits_[i];

    // Position check
    if (positions[i] < lim.position_min || positions[i] > lim.position_max) {
      violations_.push_back({
        lim.joint_name, "position", Severity::ERROR,
        positions[i],
        positions[i] < lim.position_min ? lim.position_min : lim.position_max
      });
    } else if (positions[i] < lim.position_min + lim.position_margin ||
      positions[i] > lim.position_max - lim.position_margin)
    {
      violations_.push_back({
        lim.joint_name, "position", Severity::WARNING,
        positions[i],
        positions[i] < lim.position_min + lim.position_margin ?
        lim.position_min : lim.position_max
      });
    }

    // Velocity check
    if (i < velocities.size() && std::abs(velocities[i]) > lim.velocity_max) {
      violations_.push_back({
        lim.joint_name, "velocity", Severity::ERROR,
        velocities[i], lim.velocity_max
      });
    }

    // Torque check
    if (i < efforts.size() && std::abs(efforts[i]) > lim.torque_max_pct) {
      violations_.push_back({
        lim.joint_name, "torque", Severity::ERROR,
        efforts[i], lim.torque_max_pct
      });
    }
  }

  return violations_;
}

bool JointLimitMonitor::has_error() const
{
  for (const auto & v : violations_) {
    if (v.severity == Severity::ERROR) {
      return true;
    }
  }
  return false;
}

const std::vector<JointLimitMonitor::Violation> &
JointLimitMonitor::get_violations() const
{
  return violations_;
}

}  // namespace arm_ethercat_safety
