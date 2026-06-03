// Unit tests for the eccentric-pushrod ankle linkage transform.
// Pure math — no live EtherCAT bus required.

#include <gtest/gtest.h>

#include <cmath>

#include "harambe_ethercat_driver/ankle_linkage.hpp"

using harambe_ethercat_driver::AnkleGeometry;
using harambe_ethercat_driver::JointPair;
using harambe_ethercat_driver::MotorPair;
using harambe_ethercat_driver::joint_to_motor;
using harambe_ethercat_driver::motor_to_joint;
using harambe_ethercat_driver::safe_asin;

namespace
{
// Real robot geometry (ros2_control_real_pvt.xacro).
AnkleGeometry real_geometry()
{
  return AnkleGeometry{0.025, 0.03, 0.056, -1.0, 1.0};
}

// Geometry with unit signs to test the kinematics independent of the URDF
// convention flip.
AnkleGeometry unsigned_geometry()
{
  return AnkleGeometry{0.025, 0.03, 0.056, 1.0, 1.0};
}

// The coupled mechanism cannot reach every (pitch, roll) inside the per-joint
// ±45° box: a motor's pushrod offset d_pitch*sin(pitch) ± d_roll*sin(roll) must
// stay within the eccentric radius r_exc. |sin| is sign-independent, so the
// pitch_sign/roll_sign flip does not change reachability. Use a small margin so
// poses right at the boundary (where cos(theta)→0) are excluded from round-trip
// asserts.
bool reachable(double pitch, double roll, const AnkleGeometry & g)
{
  const double reach = g.d_pitch * std::abs(std::sin(pitch)) +
                       g.d_roll * std::abs(std::sin(roll));
  return reach <= 0.95 * g.r_exc;
}
}  // namespace

TEST(SafeAsin, ClampsOutOfRange)
{
  EXPECT_DOUBLE_EQ(safe_asin(2.0, "test"), M_PI / 2.0);
  EXPECT_DOUBLE_EQ(safe_asin(-2.0, "test"), -M_PI / 2.0);
  EXPECT_NEAR(safe_asin(0.0, "test"), 0.0, 1e-12);
  EXPECT_NEAR(safe_asin(1.0, "test"), M_PI / 2.0, 1e-12);
}

// joint → motor → joint recovers the input across the working range.
TEST(AnkleLinkage, PositionRoundTrip)
{
  const AnkleGeometry g = real_geometry();
  const double lim = 0.785398;  // ±45° URDF ankle limit

  for (double pitch = -lim; pitch <= lim; pitch += 0.05) {
    for (double roll = -lim; roll <= lim; roll += 0.05) {
      if (!reachable(pitch, roll, g)) continue;  // outside coupled workspace
      JointPair jcmd;
      jcmd.pos_pitch = pitch;
      jcmd.pos_roll = roll;

      const MotorPair m = joint_to_motor(jcmd, g);
      // Feed motor positions back through the read path.
      MotorPair mstate;
      mstate.pos_a = m.pos_a;
      mstate.pos_b = m.pos_b;
      const JointPair back = motor_to_joint(mstate, g);

      EXPECT_NEAR(back.pos_pitch, pitch, 1e-9)
        << "pitch=" << pitch << " roll=" << roll;
      EXPECT_NEAR(back.pos_roll, roll, 1e-9)
        << "pitch=" << pitch << " roll=" << roll;
    }
  }
}

// Zero joint pose → zero motor angles (calibration reference).
TEST(AnkleLinkage, ZeroMapsToZero)
{
  const AnkleGeometry g = real_geometry();
  JointPair j;  // all zero
  const MotorPair m = joint_to_motor(j, g);
  EXPECT_NEAR(m.pos_a, 0.0, 1e-12);
  EXPECT_NEAR(m.pos_b, 0.0, 1e-12);

  MotorPair mz;  // all zero
  const JointPair back = motor_to_joint(mz, g);
  EXPECT_NEAR(back.pos_pitch, 0.0, 1e-12);
  EXPECT_NEAR(back.pos_roll, 0.0, 1e-12);
}

// Velocity: joint → motor → joint round-trips at a non-singular pose.
TEST(AnkleLinkage, VelocityRoundTrip)
{
  const AnkleGeometry g = real_geometry();

  for (double pitch = -0.4; pitch <= 0.4; pitch += 0.2) {
    for (double roll = -0.4; roll <= 0.4; roll += 0.2) {
      if (!reachable(pitch, roll, g)) continue;  // outside coupled workspace
      JointPair jcmd;
      jcmd.pos_pitch = pitch;
      jcmd.pos_roll = roll;
      jcmd.vel_pitch = 0.3;
      jcmd.vel_roll = -0.2;

      const MotorPair m = joint_to_motor(jcmd, g);

      // Read path needs the motor positions to rebuild the Jacobian.
      MotorPair mstate;
      mstate.pos_a = m.pos_a;
      mstate.pos_b = m.pos_b;
      mstate.vel_a = m.vel_a;
      mstate.vel_b = m.vel_b;
      const JointPair back = motor_to_joint(mstate, g);

      EXPECT_NEAR(back.vel_pitch, jcmd.vel_pitch, 1e-7)
        << "pitch=" << pitch << " roll=" << roll;
      EXPECT_NEAR(back.vel_roll, jcmd.vel_roll, 1e-7)
        << "pitch=" << pitch << " roll=" << roll;
    }
  }
}

// Effort: a pure pitch joint torque maps to symmetric motor torques; a pure
// roll torque maps to antisymmetric motor torques. Checked at the zero pose
// with unit signs (so the URDF flip does not obscure the symmetry).
TEST(AnkleLinkage, EffortSymmetryAtZero)
{
  const AnkleGeometry g = unsigned_geometry();

  // Pure pitch torque.
  JointPair jp;
  jp.eff_pitch = 1.0;
  jp.eff_roll = 0.0;
  const MotorPair mp = joint_to_motor(jp, g);
  EXPECT_NEAR(mp.eff_a, mp.eff_b, 1e-9);   // symmetric
  EXPECT_GT(std::abs(mp.eff_a), 1e-6);

  // Pure roll torque.
  JointPair jr;
  jr.eff_pitch = 0.0;
  jr.eff_roll = 1.0;
  const MotorPair mr = joint_to_motor(jr, g);
  EXPECT_NEAR(mr.eff_a, -mr.eff_b, 1e-9);  // antisymmetric
  EXPECT_GT(std::abs(mr.eff_a), 1e-6);
}

// Effort round-trips joint → motor → joint at a non-singular pose.
TEST(AnkleLinkage, EffortRoundTrip)
{
  const AnkleGeometry g = real_geometry();

  JointPair jcmd;
  jcmd.pos_pitch = 0.3;
  jcmd.pos_roll = -0.2;
  jcmd.eff_pitch = 5.0;
  jcmd.eff_roll = -3.0;

  const MotorPair m = joint_to_motor(jcmd, g);
  MotorPair mstate;
  mstate.pos_a = m.pos_a;
  mstate.pos_b = m.pos_b;
  mstate.eff_a = m.eff_a;
  mstate.eff_b = m.eff_b;
  const JointPair back = motor_to_joint(mstate, g);

  EXPECT_NEAR(back.eff_pitch, jcmd.eff_pitch, 1e-6);
  EXPECT_NEAR(back.eff_roll, jcmd.eff_roll, 1e-6);
}

// All outputs stay finite even when the commanded joint pose pushes the
// forward kinematics into the arcsin clamp (mechanical singularity).
TEST(AnkleLinkage, SingularityStaysFinite)
{
  const AnkleGeometry g = real_geometry();

  JointPair jcmd;
  jcmd.pos_pitch = 1.5;  // far past the ±45° limit → clamps
  jcmd.pos_roll = 1.5;
  jcmd.vel_pitch = 1.0;
  jcmd.vel_roll = 1.0;
  jcmd.eff_pitch = 1.0;
  jcmd.eff_roll = 1.0;

  const MotorPair m = joint_to_motor(jcmd, g);
  EXPECT_TRUE(std::isfinite(m.pos_a));
  EXPECT_TRUE(std::isfinite(m.pos_b));
  EXPECT_TRUE(std::isfinite(m.vel_a));
  EXPECT_TRUE(std::isfinite(m.vel_b));
  EXPECT_TRUE(std::isfinite(m.eff_a));
  EXPECT_TRUE(std::isfinite(m.eff_b));
}
