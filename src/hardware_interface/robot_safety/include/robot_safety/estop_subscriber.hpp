#ifndef ROBOT_SAFETY__ESTOP_SUBSCRIBER_HPP_
#define ROBOT_SAFETY__ESTOP_SUBSCRIBER_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "realtime_tools/realtime_buffer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
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
/// per-joint thermal Kp scale is published as a Float64MultiArray sized to the
/// supervisor's full joint roster and held in a realtime-safe
/// single-producer/single-consumer buffer.
///
/// There are two subscribe modes:
///   - subscribe(node, num_joints): legacy POSITIONAL mode. kp_scale[i] maps to
///     this consumer's joint i. Correct only when the consumer's roster is the
///     same prefix/order as the supervisor's published vector.
///   - subscribe(node, joint_names): NAME-AWARE mode. The supervisor labels each
///     kp_scale element with its joint name (layout.dim[i].label); this consumer
///     remaps by name so a controller owning an arbitrary SUBSET of the
///     supervisor's joints (e.g. only the legs+waist, in any order) still reads
///     the correct per-joint multiplier. Falls back to positional if the message
///     carries no labels (older supervisor).
///
/// When no supervisor is running the topics have no publisher; the state
/// atomic keeps its safe default (no e-stop) and kpScale() returns 1.0 for
/// every joint, so a control path behaves exactly as it did before the
/// supervisor was wired in.
class EstopSubscriber
{
public:
  EstopSubscriber() = default;

  /// Legacy positional subscribe. Allocates one vector of `num_joints` doubles
  /// inside the realtime buffer; do not call from a realtime context. NodeT is
  /// any handle exposing create_subscription — a controller's get_node()
  /// lifecycle handle or a plain rclcpp::Node.
  template<typename NodeT>
  void subscribe(NodeT node, std::size_t num_joints)
  {
    num_joints_ = num_joints;
    // Seed the buffer with kp_scale = 1.0 for every joint so an unsubscribed
    // controller never observes a zero multiplier.
    kp_scale_buf_.writeFromNonRT(std::vector<double>(num_joints, 1.0));

    const auto latched = rclcpp::QoS(1).transient_local();

    subscribeEstop(node, latched);

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

  /// Name-aware subscribe. `joint_names` is this consumer's roster (any subset
  /// of the supervisor's joints, in any order). kp_scale is remapped by name
  /// from the supervisor's labelled message; kpScale(j) then corresponds to
  /// joint_names[j]. Same realtime constraints as the positional overload.
  template<typename NodeT>
  void subscribe(NodeT node, const std::vector<std::string> & joint_names)
  {
    joint_names_ = joint_names;
    num_joints_ = joint_names_.size();
    kp_scale_buf_.writeFromNonRT(std::vector<double>(num_joints_, 1.0));

    const auto latched = rclcpp::QoS(1).transient_local();

    subscribeEstop(node, latched);

    kp_scale_sub_ = node->template create_subscription<std_msgs::msg::Float64MultiArray>(
      "/safety/kp_scale", latched,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        std::vector<double> v(num_joints_, 1.0);
        const auto & dims = msg->layout.dim;
        if (!dims.empty() && dims.size() == msg->data.size()) {
          // Labelled message: map each local joint to its element by name.
          rebuildIndexIfLabelsChanged(dims);
          for (std::size_t j = 0; j < num_joints_; ++j) {
            const std::size_t src = local_to_src_[j];
            if (src != kInvalidIdx && src < msg->data.size()) {
              v[j] = msg->data[src];
            }
          }
        } else {
          // Unlabelled (older supervisor): fall back to positional truncation.
          const std::size_t n = std::min(num_joints_, msg->data.size());
          for (std::size_t i = 0; i < n; ++i) {
            v[i] = msg->data[i];
          }
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
  static constexpr std::size_t kInvalidIdx = static_cast<std::size_t>(-1);

  /// Shared by both subscribe overloads — the e-stop Int8 is whole-robot.
  template<typename NodeT>
  void subscribeEstop(NodeT node, const rclcpp::QoS & latched)
  {
    estop_sub_ = node->template create_subscription<std_msgs::msg::Int8>(
      "/safety/estop_state", latched,
      [this](std_msgs::msg::Int8::SharedPtr msg) {
        estop_state_.store(msg->data);
      });
  }

  /// Recompute local-joint → supervisor-element index map, but only when the
  /// supervisor's published labels change (they don't, in steady state). Runs
  /// on the executor (subscription) thread — never the realtime path — so the
  /// transient unordered_map allocation here is safe.
  void rebuildIndexIfLabelsChanged(
    const std::vector<std_msgs::msg::MultiArrayDimension> & dims)
  {
    bool same = (dims.size() == cached_labels_.size()) &&
      (local_to_src_.size() == num_joints_);
    for (std::size_t i = 0; same && i < dims.size(); ++i) {
      if (dims[i].label != cached_labels_[i]) {
        same = false;
      }
    }
    if (same) {
      return;
    }
    std::unordered_map<std::string, std::size_t> idx_of;
    idx_of.reserve(dims.size());
    for (std::size_t i = 0; i < dims.size(); ++i) {
      idx_of[dims[i].label] = i;
    }
    local_to_src_.assign(num_joints_, kInvalidIdx);
    for (std::size_t j = 0; j < num_joints_; ++j) {
      const auto it = idx_of.find(joint_names_[j]);
      if (it != idx_of.end()) {
        local_to_src_[j] = it->second;
      }
    }
    cached_labels_.resize(dims.size());
    for (std::size_t i = 0; i < dims.size(); ++i) {
      cached_labels_[i] = dims[i].label;
    }
  }

  std::size_t num_joints_ = 0;
  std::vector<std::string> joint_names_;       // consumer roster (name-aware)
  std::vector<std::string> cached_labels_;     // last supervisor labels seen
  std::vector<std::size_t> local_to_src_;      // local idx → supervisor idx
  std::atomic<int8_t> estop_state_{0};
  mutable realtime_tools::RealtimeBuffer<std::vector<double>> kp_scale_buf_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr kp_scale_sub_;
};

}  // namespace robot_safety

#endif  // ROBOT_SAFETY__ESTOP_SUBSCRIBER_HPP_
