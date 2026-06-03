#include "harambe_ethercat_driver/ankle_linkage_driver.hpp"

#include <sstream>
#include <string>
#include <unordered_map>

namespace harambe_ethercat_driver
{

CallbackReturn AnkleLinkageDriver::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (EthercatDriver::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  auto & hp = info_.hardware_parameters;

  if (hp.count("r_exc"))       r_exc_       = std::stod(hp.at("r_exc"));
  if (hp.count("d_pitch"))     d_pitch_     = std::stod(hp.at("d_pitch"));
  if (hp.count("d_roll"))      d_roll_      = std::stod(hp.at("d_roll"));
  if (hp.count("pitch_sign"))  pitch_sign_  = std::stod(hp.at("pitch_sign"));
  if (hp.count("roll_sign"))   roll_sign_   = std::stod(hp.at("roll_sign"));

  RCLCPP_INFO(rclcpp::get_logger("AnkleLinkageDriver"),
    "Ankle linkage: r_exc=%.4f m, d_pitch=%.4f m, d_roll=%.4f m, "
    "pitch_sign=%.0f, roll_sign=%.0f",
    r_exc_, d_pitch_, d_roll_, pitch_sign_, roll_sign_);

  parseAnkleLinkages();

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type AnkleLinkageDriver::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto ret = EthercatDriver::read(time, period);

  // Transform in-place: motor A/B raw states → pitch/roll joint states.
  // State indices 0/1/2 are pos/vel/effort in both CSP/CST and PVT layouts.
  // Higher state indices (status_word, temps, ...) are per-motor passthrough.
  const AnkleGeometry g = geometry();
  for (const auto & lk : ankle_linkages_) {
    auto & sa = hw_joint_states_[lk.pitch_idx];  // raw motor A states
    auto & sb = hw_joint_states_[lk.roll_idx];  // raw motor B states

    const MotorPair m{sa[0], sa[1], sa[2], sb[0], sb[1], sb[2]};
    const JointPair jp = motor_to_joint(m, g);

    sa[0] = jp.pos_pitch;  sb[0] = jp.pos_roll;
    sa[1] = jp.vel_pitch;  sb[1] = jp.vel_roll;
    sa[2] = jp.eff_pitch;  sb[2] = jp.eff_roll;
  }

  return ret;
}

void AnkleLinkageDriver::parseAnkleLinkages()
{
  // Format: "pitchJoint,rollJoint;pitchJoint,rollJoint;..."
  ankle_linkages_.clear();

  auto it = info_.hardware_parameters.find("ankle_linkage_pairs");
  if (it == info_.hardware_parameters.end()) {
    RCLCPP_WARN(rclcpp::get_logger("AnkleLinkageDriver"),
      "No 'ankle_linkage_pairs' — ankle linkage transform disabled");
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
      RCLCPP_ERROR(rclcpp::get_logger("AnkleLinkageDriver"),
        "Invalid ankle_linkage_pairs: '%s'", pair_str.c_str());
      continue;
    }

    std::string name_a = pair_str.substr(0, comma);
    std::string name_b = pair_str.substr(comma + 1);

    auto it_a = joint_index.find(name_a);
    auto it_b = joint_index.find(name_b);
    if (it_a == joint_index.end() || it_b == joint_index.end()) {
      RCLCPP_ERROR(rclcpp::get_logger("AnkleLinkageDriver"),
        "Joint not found: '%s' or '%s'", name_a.c_str(), name_b.c_str());
      continue;
    }

    AnkleLinkage lk;
    lk.pitch_idx = it_a->second;
    lk.roll_idx = it_b->second;
    ankle_linkages_.push_back(lk);

    RCLCPP_INFO(rclcpp::get_logger("AnkleLinkageDriver"),
      "Ankle linkage: pitch=[%s idx=%zu] roll=[%s idx=%zu]",
      name_a.c_str(), lk.pitch_idx, name_b.c_str(), lk.roll_idx);
  }
}

}  // namespace harambe_ethercat_driver
