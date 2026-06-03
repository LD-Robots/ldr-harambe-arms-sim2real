#ifndef HARAMBE_ETHERCAT_DRIVER__ANKLE_LINKAGE_HPP_
#define HARAMBE_ETHERCAT_DRIVER__ANKLE_LINKAGE_HPP_

// Pure (bus-free) eccentric-pushrod ankle linkage transform.
//
// Two X4 motors per foot jointly drive ankle pitch AND roll through an
// eccentric-pushrod mechanism. This module holds the motor<->joint coordinate
// transform as free functions on plain doubles so that:
//   - both driver classes (CSP/CST HarambeEthercatDriver and PVT HarambePvtDriver)
//     share one implementation, and
//   - the kinematics are unit-testable without a live EtherCAT bus.
//
// Kinematics (linkage space, before the URDF sign correction):
//
//   motor -> joint (read):
//     pitch = asin(r_exc * (sin(thetaA) + sin(thetaB)) / (2 * d_pitch))
//     roll  = asin(r_exc * (sin(thetaA) - sin(thetaB)) / (2 * d_roll))
//
//   joint -> motor (write):
//     d_A   = d_pitch * sin(pitch) + d_roll * sin(roll)
//     d_B   = d_pitch * sin(pitch) - d_roll * sin(roll)
//     thetaA = asin(d_A / r_exc),  thetaB = asin(d_B / r_exc)
//
// Velocity uses the linkage Jacobian; effort uses its transpose. pitch_sign /
// roll_sign (+/-1) are applied between linkage space and the URDF convention.

namespace harambe_ethercat_driver
{

// Linkage geometry (meters) + per-output sign correction (+/-1).
struct AnkleGeometry
{
  double r_exc = 0.025;
  double d_pitch = 0.03;
  double d_roll = 0.056;
  double pitch_sign = 1.0;
  double roll_sign = 1.0;
};

// Raw per-motor triplet (motor A and motor B): position, velocity, effort.
struct MotorPair
{
  double pos_a = 0.0, vel_a = 0.0, eff_a = 0.0;
  double pos_b = 0.0, vel_b = 0.0, eff_b = 0.0;
};

// Joint-space triplet (pitch and roll): position, velocity, effort.
struct JointPair
{
  double pos_pitch = 0.0, vel_pitch = 0.0, eff_pitch = 0.0;
  double pos_roll = 0.0, vel_roll = 0.0, eff_roll = 0.0;
};

// arcsin with clamping to [-pi/2, pi/2] for out-of-range arguments (mechanical
// singularity or calibration error). Logs a warning on clamp. context labels
// the call site in the warning.
double safe_asin(double x, const char * context);

// Read path: raw motor A/B states -> pitch/roll joint states (pos, vel, effort).
JointPair motor_to_joint(const MotorPair & m, const AnkleGeometry & g);

// Write path: pitch/roll joint commands -> raw motor A/B commands (pos, vel, effort).
MotorPair joint_to_motor(const JointPair & j, const AnkleGeometry & g);

}  // namespace harambe_ethercat_driver

#endif  // HARAMBE_ETHERCAT_DRIVER__ANKLE_LINKAGE_HPP_
