#ifndef HARAMBE_ANKLE_ETHERCAT_DRIVER__ANKLE_MODEL_HPP_
#define HARAMBE_ANKLE_ETHERCAT_DRIVER__ANKLE_MODEL_HPP_

// Exact differential-ankle serial-chain kinematics (bus-free, ROS-free).
//
// Two motor cranks drive two pushrods onto a foot that hangs from TWO stacked
// revolute hinges (a pitch hinge a height `h` above a roll hinge). Because the
// roll hinge and rod mounts are carried around by the pitch rotation, the two
// axes cross-couple — this is the exact model the decoupled asin() linkage
// (old harambe_ethercat_driver) only approximated.
//
// Source of the math: docs/ankle kinematics promp.md (§4-§8) and the JS
// reference docs/ankle_exact_vs_linear_solver.html. Implemented verbatim.
//
//   IK (pose -> motor angles): closed form. Each rod constraint reduces to
//     A·cos(theta) + B·sin(theta) = K, solved via atan2/acos, root nearest the
//     previous command (branch continuity), clamped to +/- motor_limit.
//   FK (motor angles -> pose): 2-var Newton on the two rod-length constraints,
//     seeded by the first-order model, warm-started for real-time convergence.
//   Velocity/effort: the docs give only POSITION FK/IK; velocity and effort use
//     the numerical Jacobian J = d(pitch,roll)/d(thetaA,thetaB) of fk() — see
//     AnkleLinkageDriver / the driver write paths.
//
// Coordinates (docs §2): origin at the pitch-hinge axis; +pitch = toe up
// (dorsiflexion), +roll = right side down. Internally phi_p = -pitch (the R_y
// angle), phi_r = roll (the R_x angle). pitch_sign / roll_sign (+/-1) map between
// this model's pose and the URDF joint convention and are applied consistently
// across ik / fk / jacobian so position, velocity, and effort signs all agree.

#include <Eigen/Dense>
#include <cmath>
#include <limits>

namespace harambe_ankle_ethercat_driver
{

// Geometry + conventions. Lengths in METRES (convert mm->m once at the URDF
// boundary). Defaults are the docs' ASSUMED placeholders — the true measured
// geometry is supplied via URDF hardware params and must be confirmed on the
// real robot. NEVER round: mm defaults are divided by an exact 1000.
struct AnkleParams
{
  double r      = 25.0  / 1000.0;   // crank_throw       (pin radius / throw)   ASSUMED
  double Lp     = 63.0  / 1000.0;   // mount_foreaft     (pitch lever, +x)      ASSUMED
  double S      = 60.0  / 1000.0;   // mount_separation  (lateral, mounts +-S/2) ASSUMED
  double a      = 78.0  / 1000.0;   // cam_foreaft       (cam axis +x offset)   ASSUMED
  double h      = 28.0  / 1000.0;   // hinge_spacing     (pitch above roll)     ASSUMED
  double L_A    = 255.0 / 1000.0;   // rod_a_length      (drives motor A)       ASSUMED
  double L_B    = 180.0 / 1000.0;   // rod_b_length      (drives motor B)       ASSUMED
  double H_A    = 0.0;              // cam A height (+z); derived unless !lock_heights_to_rods
  double H_B    = 0.0;              // cam B height (+z); derived unless !lock_heights_to_rods
  double delta  = 0.0  / 1000.0;    // mount_drop        (below roll hinge)      ASSUMED
  double motor_limit = 80.0 * M_PI / 180.0;  // +/- motor travel (rad)          CONFIRMED real actuator limit
  bool   lock_heights_to_rods = true;        // derive H_A/H_B from rod lengths

  // URDF <-> model sign correction (+/-1). urdf = sign * model, and (sign self-
  // inverse) model = sign * urdf.
  double pitch_sign = 1.0;
  double roll_sign  = 1.0;
};

struct IkResult
{
  double thetaA = 0.0;
  double thetaB = 0.0;
  bool reachable = true;   // false if either rod's |K/R| > 1, or a root had to be clamped to a limit
};

struct FkResult
{
  double pitch = 0.0;      // URDF convention (after pitch_sign)
  double roll  = 0.0;      // URDF convention (after roll_sign)
  bool converged = true;
};

class AnkleModel
{
public:
  AnkleModel() { set_params(AnkleParams{}); }

  // Store params; derive H_A/H_B from rod lengths when lock_heights_to_rods.
  void set_params(const AnkleParams & p);
  const AnkleParams & params() const { return p_; }

  // §4: base = (a-Lp)^2 + (r-S/2)^2; H = sqrt(L^2 - base) - h - delta, assuming
  // the rods hang straight from cam to mount at neutral. Returns false (and
  // leaves H_A/H_B untouched) when base >= min(L_A,L_B)^2 (sqrt of negative —
  // a clear parameter error). No-op success when !lock_heights_to_rods.
  bool derive_heights();

  // §6 closed-form IK. pitch/roll are URDF-convention radians. prev_thetaA/B
  // drive branch continuity (pick the in-range root nearest the previous
  // command). Out-of-workspace or limit-clamped -> reachable=false.
  IkResult ik(double pitch, double roll, double prev_thetaA, double prev_thetaB) const;

  // §7 Newton FK. Returns URDF-convention pose. Seeded by first_order_fk();
  // warm-start from a previous solution by passing finite seed_pitch/seed_roll
  // (URDF convention) for 2-3 iteration convergence in a real-time loop.
  FkResult fk(
    double thetaA, double thetaB,
    double seed_pitch = std::numeric_limits<double>::quiet_NaN(),
    double seed_roll  = std::numeric_limits<double>::quiet_NaN()) const;

  // §8 first-order decoupled model (Newton seed + optional lightweight mode).
  FkResult first_order_fk(double thetaA, double thetaB) const;
  IkResult first_order_ik(double pitch, double roll) const;

  // J = d(pitch,roll)/d(thetaA,thetaB) (URDF convention). Runs fk() once to
  // converge the internal angles, then the analytic jacobian_at(). Used by
  // tests / callers that do not already know the pose.
  Eigen::Matrix2d jacobian(
    double thetaA, double thetaB, double seed_pitch, double seed_roll) const;

  // Analytic J given the pose (URDF pitch/roll) that (thetaA,thetaB) maps to —
  // no Newton iteration. Via the implicit function theorem on the rod
  // constraints F(theta, phi)=0:  dphi/dtheta = -(dF/dphi)^{-1} (dF/dtheta),
  // then mapped through the pitch/roll sign convention. Real-time cheap (~4
  // residual evals); the driver calls this with the pose it already has (FK
  // result on read, commanded pose on write), so no extra FK solve is needed.
  Eigen::Matrix2d jacobian_at(
    double thetaA, double thetaB, double pitch, double roll) const;

private:
  AnkleParams p_;

  // §5 geometry primitives (model/internal frame).
  static Eigen::Matrix3d rot_x(double ang);
  static Eigen::Matrix3d rot_y(double ang);
  Eigen::Vector3d pin_a(double thetaA) const;
  Eigen::Vector3d pin_b(double thetaB) const;
  // mount(m, phi_p, phi_r) = R_y(phi_p) * (R_x(phi_r) * m + (0,0,-h))
  Eigen::Vector3d mount(const Eigen::Vector3d & m_foot, double phi_p, double phi_r) const;
  // (F1, F2) = squared rod-length residuals at internal angles (phi_p, phi_r).
  Eigen::Vector2d residual(double thetaA, double thetaB, double phi_p, double phi_r) const;

  // Internal-frame (model, not URDF) Newton solving for (phi_p, phi_r).
  // seed_phi_p/seed_phi_r are the warm-start; converged flag returned via *ok.
  void newton(double thetaA, double thetaB, double seed_phi_p, double seed_phi_r,
              double & phi_p, double & phi_r, bool & ok) const;

  static double clamp_unit(double x) { return std::max(-1.0, std::min(1.0, x)); }
  static double wrap_pi(double a);   // wrap to (-pi, pi]
};

}  // namespace harambe_ankle_ethercat_driver

#endif  // HARAMBE_ANKLE_ETHERCAT_DRIVER__ANKLE_MODEL_HPP_
