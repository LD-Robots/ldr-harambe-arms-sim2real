// Unit tests for the body_group classifier — the runtime scope decision the
// PVT controller uses to skip joints outside the active group.

#include <gtest/gtest.h>

#include "robot_pvt_control/body_group.hpp"

using robot_pvt_control::is_valid_body_group;
using robot_pvt_control::joint_in_group;

TEST(BodyGroup, ValidGroupsAccepted)
{
  EXPECT_TRUE(is_valid_body_group("arms"));
  EXPECT_TRUE(is_valid_body_group("arms_waist"));
  EXPECT_TRUE(is_valid_body_group("legs"));
  EXPECT_TRUE(is_valid_body_group("full"));
}

TEST(BodyGroup, InvalidGroupsRejected)
{
  EXPECT_FALSE(is_valid_body_group(""));
  EXPECT_FALSE(is_valid_body_group("ARMS"));      // case-sensitive
  EXPECT_FALSE(is_valid_body_group("hands"));
  EXPECT_FALSE(is_valid_body_group("full_body"));
  EXPECT_FALSE(is_valid_body_group("arms,waist"));
}

TEST(BodyGroup, ArmsClassifiesAllArmJointsAndNothingElse)
{
  // 12 arm joints (one of each variant)
  EXPECT_TRUE(joint_in_group("left_shoulder_pitch_joint_X6", "arms"));
  EXPECT_TRUE(joint_in_group("left_shoulder_roll_joint_X6", "arms"));
  EXPECT_TRUE(joint_in_group("left_shoulder_yaw_joint_X4", "arms"));
  EXPECT_TRUE(joint_in_group("left_elbow_pitch_joint_X6", "arms"));
  EXPECT_TRUE(joint_in_group("left_wrist_yaw_joint_X4", "arms"));
  EXPECT_TRUE(joint_in_group("left_wrist_roll_joint_X4", "arms"));
  EXPECT_TRUE(joint_in_group("right_shoulder_pitch_joint_X6", "arms"));
  EXPECT_TRUE(joint_in_group("right_wrist_roll_joint_X4", "arms"));

  // Not arms
  EXPECT_FALSE(joint_in_group("waist_yaw_joint_X8", "arms"));
  EXPECT_FALSE(joint_in_group("left_hip_pitch_joint_X8", "arms"));
  EXPECT_FALSE(joint_in_group("left_knee_joint_X8", "arms"));
  EXPECT_FALSE(joint_in_group("right_ankle_pitch_joint_X4", "arms"));
  EXPECT_FALSE(joint_in_group("right_ankle_roll_joint_X4", "arms"));
}

TEST(BodyGroup, ArmsWaistAddsOnlyWaist)
{
  EXPECT_TRUE(joint_in_group("left_shoulder_pitch_joint_X6", "arms_waist"));
  EXPECT_TRUE(joint_in_group("waist_yaw_joint_X8", "arms_waist"));
  EXPECT_FALSE(joint_in_group("left_hip_pitch_joint_X8", "arms_waist"));
  EXPECT_FALSE(joint_in_group("right_knee_joint_X8", "arms_waist"));
  EXPECT_FALSE(joint_in_group("left_ankle_pitch_joint_X4", "arms_waist"));
}

TEST(BodyGroup, LegsClassifiesAllLegJointsAndNothingElse)
{
  EXPECT_TRUE(joint_in_group("left_hip_pitch_joint_X8", "legs"));
  EXPECT_TRUE(joint_in_group("left_hip_roll_joint_X8", "legs"));
  EXPECT_TRUE(joint_in_group("left_hip_yaw_joint_X8", "legs"));
  EXPECT_TRUE(joint_in_group("left_knee_joint_X8", "legs"));
  EXPECT_TRUE(joint_in_group("left_ankle_pitch_joint_X4", "legs"));
  EXPECT_TRUE(joint_in_group("left_ankle_roll_joint_X4", "legs"));
  EXPECT_TRUE(joint_in_group("right_hip_pitch_joint_X8", "legs"));
  EXPECT_TRUE(joint_in_group("right_ankle_roll_joint_X4", "legs"));

  EXPECT_FALSE(joint_in_group("left_shoulder_pitch_joint_X6", "legs"));
  EXPECT_FALSE(joint_in_group("waist_yaw_joint_X8", "legs"));
  EXPECT_FALSE(joint_in_group("right_wrist_roll_joint_X4", "legs"));
}

TEST(BodyGroup, FullIncludesEverything)
{
  EXPECT_TRUE(joint_in_group("left_shoulder_pitch_joint_X6", "full"));
  EXPECT_TRUE(joint_in_group("waist_yaw_joint_X8", "full"));
  EXPECT_TRUE(joint_in_group("right_ankle_roll_joint_X4", "full"));
}

TEST(BodyGroup, FullActiveCountIs25ForCanonicalRoster)
{
  // The 25 EtherCAT joints in the canonical bringup order.
  const std::vector<std::string> roster = {
    "left_shoulder_pitch_joint_X6", "left_shoulder_roll_joint_X6",
    "left_shoulder_yaw_joint_X4",   "left_elbow_pitch_joint_X6",
    "left_wrist_yaw_joint_X4",      "left_wrist_roll_joint_X4",
    "right_shoulder_pitch_joint_X6", "right_shoulder_roll_joint_X6",
    "right_shoulder_yaw_joint_X4",   "right_elbow_pitch_joint_X6",
    "right_wrist_yaw_joint_X4",      "right_wrist_roll_joint_X4",
    "waist_yaw_joint_X8",
    "left_hip_pitch_joint_X8", "left_hip_roll_joint_X8",
    "left_hip_yaw_joint_X8",   "left_knee_joint_X8",
    "left_ankle_pitch_joint_X4", "left_ankle_roll_joint_X4",
    "right_hip_pitch_joint_X8", "right_hip_roll_joint_X8",
    "right_hip_yaw_joint_X8",   "right_knee_joint_X8",
    "right_ankle_roll_joint_X4", "right_ankle_pitch_joint_X4",
  };

  auto count = [&](const std::string & group) {
    int n = 0;
    for (const auto & j : roster) {
      if (joint_in_group(j, group)) ++n;
    }
    return n;
  };
  EXPECT_EQ(count("arms"), 12);
  EXPECT_EQ(count("arms_waist"), 13);
  EXPECT_EQ(count("legs"), 12);
  EXPECT_EQ(count("full"), 25);
}
