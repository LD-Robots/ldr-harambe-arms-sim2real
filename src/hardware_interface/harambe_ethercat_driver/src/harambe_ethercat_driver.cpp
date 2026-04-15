#include "harambe_ethercat_driver/harambe_ethercat_driver.hpp"
#include <cmath>
#include <sstream>
#include <unordered_map>

namespace harambe_ethercat_driver
{

CallbackReturn HarambeEthercatDriver::on_init(
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

  RCLCPP_INFO(rclcpp::get_logger("HarambeEthercatDriver"),
    "Ankle linkage: r_exc=%.4f m, d_pitch=%.4f m, d_roll=%.4f m, "
    "pitch_sign=%.0f, roll_sign=%.0f",
    r_exc_, d_pitch_, d_roll_, pitch_sign_, roll_sign_);

  parseAnkleLinkages();

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type HarambeEthercatDriver::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto ret = EthercatDriver::read(time, period);

  // Transform in-place: motor A/B raw positions → pitch/roll
  // State layout: [0]=position, [1]=velocity, [2]=effort, [3]=status_word
  // pitch_sign_ / roll_sign_ applied AFTER pure kinematics to match URDF convention
  for (const auto & lk : ankle_linkages_) {
    auto & sa = hw_joint_states_[lk.pitch_idx];  // raw motor A states
    auto & sb = hw_joint_states_[lk.roll_idx];  // raw motor B states

    // ── Position: linkage kinematics ──
    double theta_a = sa[0];
    double theta_b = sb[0];
    double sin_a = std::sin(theta_a);
    double sin_b = std::sin(theta_b);

    double pitch_arg = r_exc_ * (sin_a + sin_b) / (2.0 * d_pitch_);
    double roll_arg  = r_exc_ * (sin_a - sin_b) / (2.0 * d_roll_);

    double pitch_lk = safe_asin(pitch_arg, "read pitch");
    double roll_lk  = safe_asin(roll_arg, "read roll");

    // ── Velocity: Jacobian ──
    double omega_a = sa[1];
    double omega_b = sb[1];
    double cos_a = std::cos(theta_a);
    double cos_b = std::cos(theta_b);
    double cos_pitch_lk = std::cos(pitch_lk);
    double cos_roll_lk  = std::cos(roll_lk);

    double pitch_vel_lk = 0.0, roll_vel_lk = 0.0;
    if (std::abs(cos_pitch_lk) > 1e-6) {
      pitch_vel_lk = r_exc_ * (cos_a * omega_a + cos_b * omega_b) / (2.0 * d_pitch_ * cos_pitch_lk);
    }
    if (std::abs(cos_roll_lk) > 1e-6) {
      roll_vel_lk = r_exc_ * (cos_a * omega_a - cos_b * omega_b) / (2.0 * d_roll_ * cos_roll_lk);
    }

    // ── Effort: (J^T)^{-1} maps motor torques → joint torques ──
    double tau_a = sa[2];
    double tau_b = sb[2];
    double pitch_eff_lk, roll_eff_lk;
    if (std::abs(cos_a) > 1e-6 && std::abs(cos_b) > 1e-6) {
      pitch_eff_lk = d_pitch_ * cos_pitch_lk / r_exc_ * (tau_a / cos_a + tau_b / cos_b);
      roll_eff_lk  = d_roll_  * cos_roll_lk  / r_exc_ * (tau_a / cos_a - tau_b / cos_b);
    } else {
      pitch_eff_lk = (tau_a + tau_b) / 2.0;
      roll_eff_lk  = (tau_a - tau_b) / 2.0;
    }

    // ── Apply sign correction and write back ──
    sa[0] = pitch_sign_ * pitch_lk;      sb[0] = roll_sign_ * roll_lk;
    sa[1] = pitch_sign_ * pitch_vel_lk;  sb[1] = roll_sign_ * roll_vel_lk;
    sa[2] = pitch_sign_ * pitch_eff_lk;  sb[2] = roll_sign_ * roll_eff_lk;
    // status_word [3]: leave as-is (per-motor passthrough)
  }

  return ret;
}

hardware_interface::return_type HarambeEthercatDriver::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Transform in-place: pitch/roll commands → motor A/B
  // Command layout: [0]=position, [1]=effort, [2]=mode_of_operation
  // pitch_sign_ / roll_sign_ undo URDF convention → linkage space before kinematics
  for (const auto & lk : ankle_linkages_) {
    auto & ca = hw_joint_commands_[lk.pitch_idx];  // pitch commands (from controller)
    auto & cb = hw_joint_commands_[lk.roll_idx];  // roll commands (from controller)

    // ── Convert URDF convention → linkage space ──
    double pitch_lk = pitch_sign_ * ca[0];
    double roll_lk  = roll_sign_  * cb[0];

    // ── Position: forward kinematics ──
    double d_A = d_pitch_ * std::sin(pitch_lk) + d_roll_ * std::sin(roll_lk);
    double d_B = d_pitch_ * std::sin(pitch_lk) - d_roll_ * std::sin(roll_lk);

    double theta_a = safe_asin(d_A / r_exc_, "write motor_a");
    double theta_b = safe_asin(d_B / r_exc_, "write motor_b");

    // ── Effort: J^T maps joint torques → motor torques ──
    double tau_pitch_lk = pitch_sign_ * ca[1];
    double tau_roll_lk  = roll_sign_  * cb[1];
    double tau_a, tau_b;

    double cos_a = std::cos(theta_a);
    double cos_b = std::cos(theta_b);
    double cos_pitch_lk = std::cos(pitch_lk);
    double cos_roll_lk  = std::cos(roll_lk);

    if (std::abs(cos_pitch_lk) > 1e-6 && std::abs(cos_roll_lk) > 1e-6) {
      double jt_a_pitch =  r_exc_ * cos_a / (2.0 * d_pitch_ * cos_pitch_lk);
      double jt_a_roll  =  r_exc_ * cos_a / (2.0 * d_roll_  * cos_roll_lk);
      double jt_b_pitch =  r_exc_ * cos_b / (2.0 * d_pitch_ * cos_pitch_lk);
      double jt_b_roll  = -r_exc_ * cos_b / (2.0 * d_roll_  * cos_roll_lk);

      tau_a = jt_a_pitch * tau_pitch_lk + jt_a_roll * tau_roll_lk;
      tau_b = jt_b_pitch * tau_pitch_lk + jt_b_roll * tau_roll_lk;
    } else {
      tau_a = (tau_pitch_lk + tau_roll_lk) / 2.0;
      tau_b = (tau_pitch_lk - tau_roll_lk) / 2.0;
    }

    // Write motor commands
    ca[0] = theta_a; cb[0] = theta_b;
    ca[1] = tau_a;   cb[1] = tau_b;
    // mode_of_operation [2]: pass through directly (per-motor, no mixing)
  }

  return EthercatDriver::write(time, period);
}

void HarambeEthercatDriver::parseAnkleLinkages()
{
  // Format: "pitchJoint,rollJoint;pitchJoint,rollJoint;..."
  ankle_linkages_.clear();

  auto it = info_.hardware_parameters.find("ankle_linkage_pairs");
  if (it == info_.hardware_parameters.end()) {
    RCLCPP_WARN(rclcpp::get_logger("HarambeEthercatDriver"),
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
      RCLCPP_ERROR(rclcpp::get_logger("HarambeEthercatDriver"),
        "Invalid ankle_linkage_pairs: '%s'", pair_str.c_str());
      continue;
    }

    std::string name_a = pair_str.substr(0, comma);
    std::string name_b = pair_str.substr(comma + 1);

    auto it_a = joint_index.find(name_a);
    auto it_b = joint_index.find(name_b);
    if (it_a == joint_index.end() || it_b == joint_index.end()) {
      RCLCPP_ERROR(rclcpp::get_logger("HarambeEthercatDriver"),
        "Joint not found: '%s' or '%s'", name_a.c_str(), name_b.c_str());
      continue;
    }

    AnkleLinkage lk;
    lk.pitch_idx = it_a->second;
    lk.roll_idx = it_b->second;
    ankle_linkages_.push_back(lk);

    RCLCPP_INFO(rclcpp::get_logger("HarambeEthercatDriver"),
      "Ankle linkage: pitch=[%s idx=%zu] roll=[%s idx=%zu]",
      name_a.c_str(), lk.pitch_idx, name_b.c_str(), lk.roll_idx);
  }
}

double HarambeEthercatDriver::safe_asin(double x, const char * context) const
{
  if (x > 1.0) {
    RCLCPP_WARN(rclcpp::get_logger("HarambeEthercatDriver"),
      "arcsin clamp: %s = %.6f > 1.0", context, x);
    return M_PI / 2.0;
  }
  if (x < -1.0) {
    RCLCPP_WARN(rclcpp::get_logger("HarambeEthercatDriver"),
      "arcsin clamp: %s = %.6f < -1.0", context, x);
    return -M_PI / 2.0;
  }
  return std::asin(x);
}

}  // namespace harambe_ethercat_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  harambe_ethercat_driver::HarambeEthercatDriver,
  hardware_interface::SystemInterface)
