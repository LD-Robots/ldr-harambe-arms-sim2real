#include "harambe_ankle_ethercat_driver/ankle_linkage_driver.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <sstream>
#include <string>
#include <unordered_map>

namespace harambe_ankle_ethercat_driver
{

CallbackReturn AnkleLinkageDriver::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (EthercatDriver::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  parseAnkleParams();
  if (!model_.derive_heights()) {
    RCLCPP_ERROR(rclcpp::get_logger("AnkleLinkageDriver"),
      "Ankle geometry invalid: base = (a-Lp)^2 + (r-S/2)^2 >= min(L_A,L_B)^2 "
      "(rod too short to reach the mount). Check crank_throw/mount/cam/rod params.");
    return CallbackReturn::ERROR;
  }

  parseAnkleLinkages();

  return CallbackReturn::SUCCESS;
}

void AnkleLinkageDriver::parseAnkleParams()
{
  auto & hp = info_.hardware_parameters;
  AnkleParams p;  // ASSUMED docs defaults — overridden below where supplied

  // Lengths are entered in MILLIMETRES for convenience; convert to metres.
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

  model_.set_params(p);
  const AnkleParams & e = model_.params();

  RCLCPP_INFO(rclcpp::get_logger("AnkleLinkageDriver"),
    "Exact ankle model (m): r=%.5f Lp=%.5f S=%.5f a=%.5f h=%.5f "
    "L_A=%.5f L_B=%.5f delta=%.5f motor_limit=%.4f rad | derived H_A=%.5f H_B=%.5f "
    "(spacing %.5f) | pitch_sign=%.0f roll_sign=%.0f",
    e.r, e.Lp, e.S, e.a, e.h, e.L_A, e.L_B, e.delta, e.motor_limit,
    e.H_A, e.H_B, e.H_A - e.H_B, e.pitch_sign, e.roll_sign);
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

hardware_interface::return_type AnkleLinkageDriver::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto ret = EthercatDriver::read(time, period);

  // Transform in-place: raw motor A/B states → pitch/roll joint states.
  // State indices 0/1/2 = pos/vel/effort in both CSP/CST and PVT layouts;
  // higher indices (status_word, temps, ...) are per-motor passthrough.
  for (auto & lk : ankle_linkages_) {
    auto & sa = hw_joint_states_[lk.pitch_idx];  // raw motor A states
    auto & sb = hw_joint_states_[lk.roll_idx];   // raw motor B states

    const double theta_a = sa[0];
    const double theta_b = sb[0];

    // Position: exact Newton FK, warm-started from the previous solution.
    const FkResult f = model_.fk(theta_a, theta_b, lk.last_fk_pitch, lk.last_fk_roll);
    lk.last_fk_pitch = f.pitch;
    lk.last_fk_roll = f.roll;

    // J = d(pitch,roll)/d(thetaA,thetaB), analytic at the just-solved pose
    // (reuses the FK result — no extra Newton iteration).
    const Eigen::Matrix2d J = model_.jacobian_at(theta_a, theta_b, f.pitch, f.roll);

    // Velocity: [vp;vr] = J · [vA;vB].
    const Eigen::Vector2d vj = J * Eigen::Vector2d(sa[1], sb[1]);

    // Effort: tau_joint = J^{-T} · tau_motor (guarded for singular J).
    Eigen::Vector2d tj;
    const double det = J.determinant();
    if (std::abs(det) > 1e-9) {
      tj = J.transpose().fullPivLu().solve(Eigen::Vector2d(sa[2], sb[2]));
    } else {
      tj = Eigen::Vector2d(sa[2], sb[2]);  // near singularity: pass through
      static rclcpp::Clock throttle_clock(RCL_STEADY_TIME);
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("AnkleLinkageDriver"),
        throttle_clock, 2000, "Ankle Jacobian near-singular in read(); effort passthrough");
    }

    sa[0] = f.pitch;  sb[0] = f.roll;
    sa[1] = vj(0);    sb[1] = vj(1);
    sa[2] = tj(0);    sb[2] = tj(1);
  }

  return ret;
}

}  // namespace harambe_ankle_ethercat_driver
