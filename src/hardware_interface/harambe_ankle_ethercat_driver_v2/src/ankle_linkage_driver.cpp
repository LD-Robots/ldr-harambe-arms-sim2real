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

  // PRIMARY = cubic (drives control); SECONDARY = analytic (comparison only).
  primary_ = AnkleKinematics::create("cubic");
  secondary_ = AnkleKinematics::create("analytic");

  parseAnkleParams();
  parseAnkleLinkages();
  setupComparePublisher();

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

hardware_interface::return_type AnkleLinkageDriver::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto ret = EthercatDriver::read(time, period);

  for (auto & lk : ankle_linkages_) {
    auto & sa = hw_joint_states_[lk.pitch_idx];   // raw motor A (inside)
    auto & sb = hw_joint_states_[lk.roll_idx];    // raw motor B (outside)

    const double theta_a = sa[0];
    const double theta_b = sb[0];

    // PRIMARY (cubic) FK drives the joint state.
    const FkResult f = primary_->fk(theta_a, theta_b, lk.last_fk_pitch, lk.last_fk_roll);
    lk.last_fk_pitch = f.pitch;
    lk.last_fk_roll = f.roll;

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

    // SECONDARY (analytic) FK — comparison only, never written to state.
    const FkResult fa = secondary_->fk(theta_a, theta_b, lk.last_fk_pitch, lk.last_fk_roll);
    lk.cmp_primary_pitch = f.pitch;   lk.cmp_primary_roll = f.roll;
    lk.cmp_secondary_pitch = fa.pitch; lk.cmp_secondary_roll = fa.roll;

    sa[0] = f.pitch;  sb[0] = f.roll;
    sa[1] = vj(0);    sb[1] = vj(1);
    sa[2] = tj(0);    sb[2] = tj(1);
  }

  publishCompare(time);
  return ret;
}

}  // namespace harambe_ankle_ethercat_driver_v2
