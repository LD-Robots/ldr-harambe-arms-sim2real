#include "harambe_ankle_ethercat_driver_v2/ankle_linkage_driver.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <sstream>
#include <string>
#include <unordered_map>

namespace harambe_ankle_ethercat_driver_v2
{

namespace
{
constexpr double kRad2Deg = 180.0 / M_PI;
}

CallbackReturn AnkleLinkageDriver::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (EthercatDriver::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // PRIMARY drives control; SECONDARY is published for comparison only. The
  // analytic serial-chain (mirror crank, calibrated) now matches the grid with
  // exact closed-form IK and a smooth physical Jacobian, so it is the default
  // primary; the cubic is the cross-check. Override via 'kinematics_primary'.
  std::string prim = "analytic";
  auto kp = info_.hardware_parameters.find("kinematics_primary");
  if (kp != info_.hardware_parameters.end() && !kp->second.empty()) prim = kp->second;
  const std::string sec = (prim == "cubic") ? "analytic" : "cubic";
  primary_ = AnkleKinematics::create(prim);
  secondary_ = AnkleKinematics::create(sec);

  parseAnkleParams();
  parseAnkleLinkages();
  setupComparePublisher();
  setupTimingPublisher();

  return CallbackReturn::SUCCESS;
}

void AnkleLinkageDriver::parseAnkleParams()
{
  auto & hp = info_.hardware_parameters;
  AnkleParams p;

  auto mm = [&](const char * key, double & dst) {
    if (hp.count(key)) dst = std::stod(hp.at(key)) / 1000.0;
  };
  mm("crank_throw_mm", p.r);
  mm("mount_foreaft_mm", p.Lp);
  mm("mount_separation_mm", p.S);
  mm("cam_foreaft_mm", p.a);
  mm("hinge_spacing_mm", p.h);
  mm("rod_a_length_mm", p.L_A);
  mm("rod_b_length_mm", p.L_B);
  mm("cam_a_height_mm", p.H_A);
  mm("cam_b_height_mm", p.H_B);
  mm("mount_drop_mm", p.delta);

  if (hp.count("motor_limit_deg")) {
    p.motor_limit = std::stod(hp.at("motor_limit_deg")) * M_PI / 180.0;
  }
  if (hp.count("lock_heights_to_rods")) {
    const std::string v = hp.at("lock_heights_to_rods");
    p.lock_heights_to_rods = (v == "true" || v == "1" || v == "True");
  }
  if (hp.count("pitch_sign")) p.pitch_sign = std::stod(hp.at("pitch_sign"));
  if (hp.count("roll_sign"))  p.roll_sign  = std::stod(hp.at("roll_sign"));

  if (hp.count("compare_decimation")) {
    cmp_decimation_ = std::max(1, std::stoi(hp.at("compare_decimation")));
  }

  if (hp.count("enable_timing")) {
    const std::string v = hp.at("enable_timing");
    timing_enabled_ = (v == "true" || v == "1" || v == "True");
  }
  if (hp.count("timing_decimation")) {
    timing_decimation_ = std::max(1, std::stoi(hp.at("timing_decimation")));
  }

  primary_->set_params(p);
  secondary_->set_params(p);

  RCLCPP_INFO(rclcpp::get_logger("AnkleLinkageDriverV2"),
    "Ankle v2: primary=%s (control), secondary=%s (compare). "
    "geom(m): r=%.5f Lp=%.5f S=%.5f a=%.5f h=%.5f L_A=%.5f L_B=%.5f delta=%.5f | "
    "motor_limit=%.4f rad | pitch_sign=%.0f roll_sign=%.0f",
    primary_->name(), secondary_->name(),
    p.r, p.Lp, p.S, p.a, p.h, p.L_A, p.L_B, p.delta, p.motor_limit,
    p.pitch_sign, p.roll_sign);
}

void AnkleLinkageDriver::parseAnkleLinkages()
{
  ankle_linkages_.clear();

  auto it = info_.hardware_parameters.find("ankle_linkage_pairs");
  if (it == info_.hardware_parameters.end()) {
    RCLCPP_WARN(rclcpp::get_logger("AnkleLinkageDriverV2"),
      "No 'ankle_linkage_pairs' — ankle transform disabled");
    return;
  }

  std::unordered_map<std::string, size_t> joint_index;
  for (size_t j = 0; j < info_.joints.size(); ++j) {
    joint_index[info_.joints[j].name] = j;
  }

  std::stringstream ss(it->second);
  std::string pair_str;
  while (std::getline(ss, pair_str, ';')) {
    auto comma = pair_str.find(',');
    if (comma == std::string::npos) {
      RCLCPP_ERROR(rclcpp::get_logger("AnkleLinkageDriverV2"),
        "Invalid ankle_linkage_pairs: '%s'", pair_str.c_str());
      continue;
    }
    std::string name_a = pair_str.substr(0, comma);
    std::string name_b = pair_str.substr(comma + 1);

    auto it_a = joint_index.find(name_a);
    auto it_b = joint_index.find(name_b);
    if (it_a == joint_index.end() || it_b == joint_index.end()) {
      RCLCPP_ERROR(rclcpp::get_logger("AnkleLinkageDriverV2"),
        "Joint not found: '%s' or '%s'", name_a.c_str(), name_b.c_str());
      continue;
    }

    AnkleLinkage lk;
    lk.pitch_idx = it_a->second;
    lk.roll_idx = it_b->second;
    lk.name_pitch = name_a;
    lk.name_roll = name_b;
    ankle_linkages_.push_back(lk);

    RCLCPP_INFO(rclcpp::get_logger("AnkleLinkageDriverV2"),
      "Ankle linkage: pitch=[%s idx=%zu] roll=[%s idx=%zu]",
      name_a.c_str(), lk.pitch_idx, name_b.c_str(), lk.roll_idx);
  }
}

void AnkleLinkageDriver::setupComparePublisher()
{
  if (ankle_linkages_.empty()) return;
  try {
    cmp_node_ = std::make_shared<rclcpp::Node>("harambe_ankle_method_compare");
    cmp_pub_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
      cmp_node_->create_publisher<sensor_msgs::msg::JointState>(
        "ankle_method_compare", rclcpp::SystemDefaultsQoS()));
  } catch (const std::exception & e) {
    RCLCPP_WARN(rclcpp::get_logger("AnkleLinkageDriverV2"),
      "Comparison publisher unavailable (%s); continuing without it", e.what());
    cmp_pub_.reset();
    return;
  }

  // Names are constant — set once. Order per linkage: pitch/roll x primary/secondary.
  const std::string ps = std::string("_") + primary_->name();
  const std::string ss = std::string("_") + secondary_->name();
  auto & names = cmp_pub_->msg_.name;
  names.clear();
  for (const auto & lk : ankle_linkages_) {
    names.push_back(lk.name_pitch + ps);
    names.push_back(lk.name_roll + ps);
    names.push_back(lk.name_pitch + ss);
    names.push_back(lk.name_roll + ss);
  }
  cmp_pub_->msg_.position.assign(names.size(), 0.0);
  RCLCPP_INFO(rclcpp::get_logger("AnkleLinkageDriverV2"),
    "Publishing method comparison (deg) on /ankle_method_compare @ ~%d:1 decimation",
    cmp_decimation_);
}

void AnkleLinkageDriver::publishCompare(const rclcpp::Time & time)
{
  if (!cmp_pub_) return;
  if (++cmp_counter_ < cmp_decimation_) return;
  cmp_counter_ = 0;
  if (!cmp_pub_->trylock()) return;
  auto & m = cmp_pub_->msg_;
  m.header.stamp = time;
  size_t k = 0;
  for (const auto & lk : ankle_linkages_) {
    m.position[k++] = lk.cmp_primary_pitch * kRad2Deg;
    m.position[k++] = lk.cmp_primary_roll * kRad2Deg;
    m.position[k++] = lk.cmp_secondary_pitch * kRad2Deg;
    m.position[k++] = lk.cmp_secondary_roll * kRad2Deg;
  }
  cmp_pub_->unlockAndPublish();
}

void AnkleLinkageDriver::setupTimingPublisher()
{
  if (!timing_enabled_) return;
  try {
    timing_node_ = std::make_shared<rclcpp::Node>("harambe_cycle_timing");
    timing_pub_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
      timing_node_->create_publisher<sensor_msgs::msg::JointState>(
        "ankle_cycle_timing", rclcpp::SystemDefaultsQoS()));
  } catch (const std::exception & e) {
    RCLCPP_WARN(rclcpp::get_logger("AnkleLinkageDriverV2"),
      "Cycle-timing publisher unavailable (%s); continuing without it", e.what());
    timing_pub_.reset();
    timing_enabled_ = false;
    return;
  }

  // Phase labels are constant — set once. Order must match the Phase enum.
  static const char * kPhaseNames[PH_COUNT] = {
    "read_total", "read_bus", "read_primary_fk", "read_jac_solve",
    "read_secondary_fk", "read_publish", "write_total", "write_ankle_ik", "write_bus"};
  auto & m = timing_pub_->msg_;
  m.name.assign(kPhaseNames, kPhaseNames + PH_COUNT);
  m.position.assign(PH_COUNT, 0.0);   // mean us
  m.velocity.assign(PH_COUNT, 0.0);   // max us
  m.effort.assign(PH_COUNT, 0.0);     // min us
  RCLCPP_INFO(rclcpp::get_logger("AnkleLinkageDriverV2"),
    "Publishing per-phase cycle timing on /ankle_cycle_timing @ ~%d:1 decimation "
    "(us; position=mean velocity=max effort=min)", timing_decimation_);
}

void AnkleLinkageDriver::timingTick(const rclcpp::Time & time)
{
  if (!timing_enabled_ || !timing_pub_) return;
  if (++timing_counter_ < timing_decimation_) return;
  timing_counter_ = 0;

  if (timing_pub_->trylock()) {
    auto & m = timing_pub_->msg_;
    m.header.stamp = time;
    for (size_t k = 0; k < PH_COUNT; ++k) {
      const auto & a = timing_[k];
      m.position[k] = (a.count > 0) ? (a.sum_us / static_cast<double>(a.count)) : 0.0;
      m.velocity[k] = a.max_us;
      m.effort[k] = a.min_us;
    }
    timing_pub_->unlockAndPublish();
  }
  // Reset whether or not trylock() succeeded — a dropped window just widens to ~2 s.
  for (auto & a : timing_) a.reset();
}

hardware_interface::return_type AnkleLinkageDriver::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const bool tm = timing_enabled_;
  const uint64_t t_read_start = tm ? nowNs() : 0;

  auto ret = EthercatDriver::read(time, period);
  if (tm) timing_[PH_READ_BUS].add((nowNs() - t_read_start) * 1e-3);

  double us_primary = 0.0, us_jac = 0.0, us_secondary = 0.0;

  for (auto & lk : ankle_linkages_) {
    auto & sa = hw_joint_states_[lk.pitch_idx];   // raw motor A (inside)
    auto & sb = hw_joint_states_[lk.roll_idx];    // raw motor B (outside)

    const double theta_a = sa[0];
    const double theta_b = sb[0];
    lk.meas_thetaA = theta_a;   // cache raw motors to hold-measured when uncommanded
    lk.meas_thetaB = theta_b;

    // PRIMARY FK drives the joint state.
    const uint64_t t0 = tm ? nowNs() : 0;
    const FkResult f = primary_->fk(theta_a, theta_b, lk.last_fk_pitch, lk.last_fk_roll);
    if (tm) us_primary += (nowNs() - t0) * 1e-3;
    lk.last_fk_pitch = f.pitch;
    lk.last_fk_roll = f.roll;

    const uint64_t t1 = tm ? nowNs() : 0;
    const Eigen::Matrix2d J = primary_->jacobian_at(theta_a, theta_b, f.pitch, f.roll);
    const Eigen::Vector2d vj = J * Eigen::Vector2d(sa[1], sb[1]);

    Eigen::Vector2d tj;
    const double det = J.determinant();
    if (std::abs(det) > 1e-9) {
      tj = J.transpose().fullPivLu().solve(Eigen::Vector2d(sa[2], sb[2]));
    } else {
      tj = Eigen::Vector2d(sa[2], sb[2]);
      static rclcpp::Clock throttle_clock(RCL_STEADY_TIME);
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("AnkleLinkageDriverV2"),
        throttle_clock, 2000, "Ankle Jacobian near-singular in read(); effort passthrough");
    }
    if (tm) us_jac += (nowNs() - t1) * 1e-3;

    // SECONDARY (analytic) FK — comparison only, never written to state.
    const uint64_t t2 = tm ? nowNs() : 0;
    const FkResult fa = secondary_->fk(theta_a, theta_b, lk.last_fk_pitch, lk.last_fk_roll);
    if (tm) us_secondary += (nowNs() - t2) * 1e-3;
    lk.cmp_primary_pitch = f.pitch;   lk.cmp_primary_roll = f.roll;
    lk.cmp_secondary_pitch = fa.pitch; lk.cmp_secondary_roll = fa.roll;

    sa[0] = f.pitch;  sb[0] = f.roll;
    sa[1] = vj(0);    sb[1] = vj(1);
    sa[2] = tj(0);    sb[2] = tj(1);
  }

  const uint64_t t_pub = tm ? nowNs() : 0;
  publishCompare(time);
  if (tm) {
    timing_[PH_READ_PUBLISH].add((nowNs() - t_pub) * 1e-3);
    timing_[PH_READ_PRIMARY_FK].add(us_primary);
    timing_[PH_READ_JAC_SOLVE].add(us_jac);
    timing_[PH_READ_SECONDARY_FK].add(us_secondary);
    timing_[PH_READ_TOTAL].add((nowNs() - t_read_start) * 1e-3);
  }

  timingTick(time);
  return ret;
}

}  // namespace harambe_ankle_ethercat_driver_v2
