#ifndef HARAMBE_ANKLE_ETHERCAT_DRIVER_V2__ANKLE_KINEMATICS_HPP_
#define HARAMBE_ANKLE_ETHERCAT_DRIVER_V2__ANKLE_KINEMATICS_HPP_

// Backend-agnostic differential-ankle kinematics interface (bus-free, ROS-free).
//
// Two motor cranks (A = inside lever, B = outside lever) drive the foot in
// pitch + roll. Confirmed on the real hardware via a 9x9 measured grid:
//   motors SAME direction  -> ROLL      (roll  ~ -(A+B))
//   motors OPPOSITE         -> PITCH     (pitch ~  (B-A))
//   1:1 (lever deg = crank deg), lever 0 = foot flat (neutral), +pitch = toe up.
//
// PRIMARY backend is CubicAnkleKinematics — a bicubic least-squares fit of the
// measured grid (reproduces it to ~0.15 deg). The serial-chain AnalyticAnkle-
// Kinematics is kept ONLY for live comparison: it is structurally inadequate for
// this mechanism (>=15 deg vs the grid) and must NOT drive control.
//
// fk : (thetaA, thetaB) [rad] -> (pitch, roll) [rad, URDF convention]
// ik : (pitch, roll)    [rad] -> (thetaA, thetaB) [rad], branch-continuous
// jacobian_at : d(pitch,roll)/d(thetaA,thetaB) at a known pose [rad/rad]

#include <Eigen/Dense>
#include <limits>
#include <memory>
#include <string>

namespace harambe_ankle_ethercat_driver_v2
{

// Shared params. Geometry (mm->m) is used only by the analytic backend; the
// cubic backend uses the embedded measured grid. motor_limit / signs are shared.
// NEVER round: mm defaults divide by an exact 1000.
//
// Geometry defaults are CALIBRATED to the measured 9x9 grid (least-squares fit of
// the serial-chain with the mirror crank), reproducing it to pitch 0.014 deg /
// roll 0.16 deg — close to the hand-measured dims but refined by the fit. r and h
// were held at their measured values.
struct AnkleParams
{
  // --- analytic serial-chain geometry (ignored by the cubic backend) ---
  double r      = 25.0       / 1000.0;   // crank_throw (fixed, measured)
  double Lp     = 73.2322    / 1000.0;   // mount_foreaft (calibrated)
  double S      = 59.7839    / 1000.0;   // mount_separation (calibrated)
  double a      = 79.9263    / 1000.0;   // cam_foreaft (calibrated)
  double h      = 28.0       / 1000.0;   // hinge_spacing (fixed, measured)
  double L_A    = 229.0121   / 1000.0;   // rod_a_length (calibrated)
  double L_B    = 166.6107   / 1000.0;   // rod_b_length (calibrated)
  double H_A    = 0.0;                   // cam A height (derived from rods)
  double H_B    = 0.0;                   // cam B height (derived from rods)
  double delta  = -10.2996   / 1000.0;   // foot-mount drop vs roll hinge (neg = above)
  bool   lock_heights_to_rods = true;
  // Motor B's crank is mirror-mounted: it sweeps OPPOSITE in z to motor A for the
  // same rotation sense (so equal motor direction -> roll, opposite -> pitch).
  // This single sign is what makes the serial-chain match the hardware.
  double crank_b_sign = -1.0;

  // --- shared ---
  double motor_limit = 80.0 * M_PI / 180.0;  // +/- crank travel (rad); grid box
  double pitch_sign = 1.0;   // URDF <-> physical sign correction (+/-1)
  double roll_sign  = 1.0;
};

struct FkResult
{
  double pitch = 0.0;        // URDF convention (after pitch_sign)
  double roll  = 0.0;
  bool converged = true;
};

struct IkResult
{
  double thetaA = 0.0;
  double thetaB = 0.0;
  bool reachable = true;     // false if the pose is outside the valid box / clamped
};

class AnkleKinematics
{
public:
  virtual ~AnkleKinematics() = default;

  virtual void set_params(const AnkleParams & p) = 0;
  virtual const AnkleParams & params() const = 0;

  // Short human-readable backend id ("cubic" / "analytic") for logs/topics.
  virtual const char * name() const = 0;

  virtual FkResult fk(
    double thetaA, double thetaB,
    double seed_pitch = std::numeric_limits<double>::quiet_NaN(),
    double seed_roll  = std::numeric_limits<double>::quiet_NaN()) const = 0;

  virtual IkResult ik(
    double pitch, double roll, double prev_thetaA, double prev_thetaB) const = 0;

  virtual Eigen::Matrix2d jacobian_at(
    double thetaA, double thetaB, double pitch, double roll) const = 0;

  // Factory: "cubic" (primary) or "analytic" (comparison only).
  static std::unique_ptr<AnkleKinematics> create(const std::string & backend);
};

}  // namespace harambe_ankle_ethercat_driver_v2

#endif  // HARAMBE_ANKLE_ETHERCAT_DRIVER_V2__ANKLE_KINEMATICS_HPP_
