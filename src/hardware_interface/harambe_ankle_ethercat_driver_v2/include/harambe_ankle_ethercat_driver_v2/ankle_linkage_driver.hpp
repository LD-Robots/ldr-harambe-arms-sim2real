#ifndef HARAMBE_ANKLE_ETHERCAT_DRIVER_V2__ANKLE_LINKAGE_DRIVER_HPP_
#define HARAMBE_ANKLE_ETHERCAT_DRIVER_V2__ANKLE_LINKAGE_DRIVER_HPP_

#include "ethercat_driver/ethercat_driver.hpp"
#include "harambe_ankle_ethercat_driver_v2/ankle_kinematics.hpp"

#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <memory>
#include <string>
#include <vector>

namespace harambe_ankle_ethercat_driver_v2
{

/**
 * AnkleLinkageDriver (v2) — shared base for the EtherCAT drivers using the
 * differential-ankle transform.
 *
 * The PRIMARY kinematics (cubic, fit to the real grid) drives read()/write()
 * (motor <-> joint). A SECONDARY method (analytic serial-chain) is evaluated on
 * the same raw motor angles every cycle and published — alongside the primary —
 * on a separate diagnostic topic (default /ankle_method_compare, degrees) via a
 * realtime publisher, so the two methods can be compared live on hardware. The
 * secondary NEVER touches the control path.
 *
 * Slot mapping (unchanged from v1): ankle_linkage_pairs lists "pitchJoint,
 * rollJoint" per pair -> {pitch_idx, roll_idx}. thetaA = raw from pitch_idx's
 * motor (inside lever), thetaB = raw from roll_idx's motor (outside lever). The
 * pitch output goes to pitch_idx, roll to roll_idx; IK thetaA->pitch_idx,
 * thetaB->roll_idx.
 */
class AnkleLinkageDriver : public ethercat_driver::EthercatDriver
{
public:
  AnkleLinkageDriver() = default;

  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  struct AnkleLinkage
  {
    size_t pitch_idx;
    size_t roll_idx;
    std::string name_pitch;
    std::string name_roll;
    double last_cmd_thetaA = 0.0;
    double last_cmd_thetaB = 0.0;
    double last_fk_pitch = 0.0;
    double last_fk_roll = 0.0;
    double saved_cmd_pitch[3] = {0.0, 0.0, 0.0};
    double saved_cmd_roll[3] = {0.0, 0.0, 0.0};
    // Last RAW motor angles seen in read() (rad). Used to HOLD MEASURED when the
    // joint command is uncommanded (non-finite) — never force the ankle to
    // neutral on an enabled drive (e-stop reset must hold where the foot is).
    double meas_thetaA = 0.0;
    double meas_thetaB = 0.0;
    // Last computed pose from each method (rad), for the comparison topic.
    double cmp_primary_pitch = 0.0, cmp_primary_roll = 0.0;
    double cmp_secondary_pitch = 0.0, cmp_secondary_roll = 0.0;
  };

  std::vector<AnkleLinkage> ankle_linkages_;
  std::unique_ptr<AnkleKinematics> primary_;     // cubic — drives control
  std::unique_ptr<AnkleKinematics> secondary_;   // analytic — comparison only

  void parseAnkleParams();      // mm/deg URDF params -> AnkleParams -> both methods
  void parseAnkleLinkages();    // "pitchJoint,rollJoint;..." -> ankle_linkages_

  // ---- RT cycle-timing instrumentation (measurement only; no control effect) ----
  // Splits read()/write() into phases so we can see whether the per-cycle cost is
  // the vendored bus (EthercatDriver::read/write, timed as one black box) or the
  // ankle math (analytic primary FK vs the rest). Protected so HarambePvtDriver's
  // write() reuses the same accumulators. Enabled by default; disable with the
  // 'enable_timing' hardware param. See /ankle_cycle_timing.
  enum Phase : size_t
  {
    PH_READ_TOTAL = 0,
    PH_READ_BUS,
    PH_READ_PRIMARY_FK,
    PH_READ_JAC_SOLVE,
    PH_READ_SECONDARY_FK,
    PH_READ_PUBLISH,
    PH_WRITE_TOTAL,
    PH_WRITE_ANKLE_IK,
    PH_WRITE_BUS,
    PH_COUNT
  };

  struct PhaseAccum
  {
    uint64_t count = 0;
    double sum_us = 0.0;
    double min_us = 0.0;
    double max_us = 0.0;
    void add(double us)
    {
      if (count == 0 || us < min_us) min_us = us;
      if (count == 0 || us > max_us) max_us = us;
      sum_us += us;
      ++count;
    }
    void reset() { count = 0; sum_us = 0.0; min_us = 0.0; max_us = 0.0; }
  };

  static uint64_t nowNs()
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
  }

  std::array<PhaseAccum, PH_COUNT> timing_{};
  bool timing_enabled_ = true;
  int timing_decimation_ = 1000;   // 1 kHz / 1000 = 1 Hz report
  int timing_counter_ = 0;

private:
  void setupComparePublisher();
  void publishCompare(const rclcpp::Time & time);

  std::shared_ptr<rclcpp::Node> cmp_node_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> cmp_pub_;
  int cmp_decimation_ = 5;   // publish every Nth read (100 Hz / 5 = 20 Hz)
  int cmp_counter_ = 0;

  void setupTimingPublisher();
  void timingTick(const rclcpp::Time & time);   // 1 Hz snapshot+publish+reset

  std::shared_ptr<rclcpp::Node> timing_node_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> timing_pub_;
};

}  // namespace harambe_ankle_ethercat_driver_v2

#endif  // HARAMBE_ANKLE_ETHERCAT_DRIVER_V2__ANKLE_LINKAGE_DRIVER_HPP_
