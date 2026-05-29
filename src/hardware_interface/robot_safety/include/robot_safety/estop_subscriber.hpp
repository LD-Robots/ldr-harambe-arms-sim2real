#ifndef ROBOT_SAFETY__ESTOP_SUBSCRIBER_HPP_
#define ROBOT_SAFETY__ESTOP_SUBSCRIBER_HPP_

#include <atomic>
#include <cstdint>
#include <vector>

#include "realtime_tools/realtime_buffer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int8.hpp"

#include "robot_safety/safety_limits.hpp"

namespace robot_safety
{

/// Snapshot of the supervisor's whole-robot e-stop signal.
struct SafetySignal
{
  bool        estop_active = false;
  EstopAction action       = EstopAction::FREE;
};

/// Consumer-side helper shared by every control path that consumes the
/// supervisor's safety topics. Owns two latched subscriptions:
///   /safety/estop_state   std_msgs/Int8                — 0 clear, 1 FREE, 2 HOLD, 3 DAMP
///   /safety/kp_scale      std_msgs/Float64MultiArray   — N-joint Kp multipliers
///
/// The e-stop state is whole-robot (a single Int8 stored in an atomic). The
/// per-joint thermal Kp scale is published as a Float64MultiArray sized to
/// `num_joints` and held in a realtime-safe single-producer/single-consumer
/// buffer; controllers index into it by joint position in their configured
/// joint_names order, matching the supervisor's own ordering.
///
/// When no supervisor is running the topics have no publisher; the state
/// atomic keeps its safe default (no e-stop) and kpScale() returns 1.0 for
/// every joint, so a control path behaves exactly as it did before the
/// supervisor was wired in.
class EstopSubscriber
{
public:
  EstopSubscriber() = default;

  /// Create the subscriptions on `node`. Allocates one vector of `num_joints`
  /// doubles inside the realtime buffer; do not call from a realtime context.
  /// NodeT is any handle exposing create_subscription — a controller's
  /// get_node() lifecycle handle or a plain rclcpp::Node.
  template<typename NodeT>
  void subscribe(NodeT node, std::size_t num_joints)
  {
    num_joints_ = num_joints;
    // Seed the buffer with kp_scale = 1.0 for every joint so an unsubscribed
    // controller never observes a zero multiplier.
    kp_scale_buf_.writeFromNonRT(std::vector<double>(num_joints, 1.0));

    const auto latched = rclcpp::QoS(1).transient_local();

    estop_sub_ = node->template create_subscription<std_msgs::msg::Int8>(
      "/safety/estop_state", latched,
      [this](std_msgs::msg::Int8::SharedPtr msg) {
        estop_state_.store(msg->data);
      });

    kp_scale_sub_ = node->template create_subscription<std_msgs::msg::Float64MultiArray>(
      "/safety/kp_scale", latched,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        // Defensive: pad / truncate to num_joints so a malformed message can't
        // produce out-of-range reads in the realtime path.
        std::vector<double> v(num_joints_, 1.0);
        const std::size_t n = std::min(num_joints_, msg->data.size());
        for (std::size_t i = 0; i < n; ++i) {
          v[i] = msg->data[i];
        }
        kp_scale_buf_.writeFromNonRT(std::move(v));
      });
  }

  /// Whole-robot e-stop snapshot — realtime-safe.
  SafetySignal get() const
  {
    SafetySignal signal;
    const int8_t state = estop_state_.load();
    signal.estop_active = (state != 0);
    switch (state) {
      case 2:  signal.action = EstopAction::HOLD; break;
      case 3:  signal.action = EstopAction::DAMP; break;
      default: signal.action = EstopAction::FREE; break;
    }
    return signal;
  }

  /// Per-joint Kp multiplier — realtime-safe. Out-of-range indices yield 1.0.
  double kpScale(std::size_t joint_idx) const
  {
    const auto * v = kp_scale_buf_.readFromRT();
    if (v == nullptr || joint_idx >= v->size()) {
      return 1.0;
    }
    return (*v)[joint_idx];
  }

  std::size_t numJoints() const { return num_joints_; }

private:
  std::size_t num_joints_ = 0;
  std::atomic<int8_t> estop_state_{0};
  mutable realtime_tools::RealtimeBuffer<std::vector<double>> kp_scale_buf_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr kp_scale_sub_;
};

}  // namespace robot_safety

#endif  // ROBOT_SAFETY__ESTOP_SUBSCRIBER_HPP_
