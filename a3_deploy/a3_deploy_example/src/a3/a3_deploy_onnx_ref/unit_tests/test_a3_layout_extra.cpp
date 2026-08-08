// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// Unit tests for the 31-DOF A3 layout extension (notes/a3_dof_orderings.md).
//
// Post-April-2026 realignment: MakeA3Layout31() now matches MuJoCo's real
// DOF order (waist[0..2], neck[3..4], arms[5..18], legs[19..30]) rather
// than A3's flat layout (legs+waist+arms+neck). The old "first 29 names
// match A3" assertion is intentionally gone — A3 and A3 layouts differ.

#include <gtest/gtest.h>

#include "robot_io/a3_layout_extra.hpp"
#include "robot_io/layouts.hpp"

TEST(A3LayoutExtra, DofIs31) {
  const auto& layout = robot_io::MakeA3Layout31();
  EXPECT_EQ(layout.dof(), 31);
  EXPECT_EQ(layout.dof(), robot_io::kA3Dof);
  EXPECT_EQ(layout.names.size(), 31u);
}

TEST(A3LayoutExtra, WaistAtSlots0To2) {
  const auto& a3 = robot_io::MakeA3Layout31();
  EXPECT_EQ(a3.names[0], "waist_yaw_joint");
  EXPECT_EQ(a3.names[1], "waist_roll_joint");
  EXPECT_EQ(a3.names[2], "waist_pitch_joint");
}

TEST(A3LayoutExtra, NeckAtSlots3And4) {
  const auto& a3 = robot_io::MakeA3Layout31();
  ASSERT_EQ(robot_io::kA3NeckStart, 3);
  ASSERT_EQ(robot_io::kA3NeckCount, 2);
  EXPECT_EQ(a3.names[robot_io::kA3NeckStart + 0], "head_yaw_joint");
  EXPECT_EQ(a3.names[robot_io::kA3NeckStart + 1], "head_pitch_joint");
}

TEST(A3LayoutExtra, ArmsAtSlots5To18) {
  const auto& a3 = robot_io::MakeA3Layout31();
  // Left arm [5..11]
  EXPECT_EQ(a3.names[5],  "left_shoulder_pitch_joint");
  EXPECT_EQ(a3.names[6],  "left_shoulder_roll_joint");
  EXPECT_EQ(a3.names[7],  "left_shoulder_yaw_joint");
  EXPECT_EQ(a3.names[8],  "left_elbow_joint");
  EXPECT_EQ(a3.names[9],  "left_wrist_roll_joint");
  EXPECT_EQ(a3.names[10], "left_wrist_pitch_joint");
  EXPECT_EQ(a3.names[11], "left_wrist_yaw_joint");
  // Right arm [12..18]
  EXPECT_EQ(a3.names[12], "right_shoulder_pitch_joint");
  EXPECT_EQ(a3.names[13], "right_shoulder_roll_joint");
  EXPECT_EQ(a3.names[14], "right_shoulder_yaw_joint");
  EXPECT_EQ(a3.names[15], "right_elbow_joint");
  EXPECT_EQ(a3.names[16], "right_wrist_roll_joint");
  EXPECT_EQ(a3.names[17], "right_wrist_pitch_joint");
  EXPECT_EQ(a3.names[18], "right_wrist_yaw_joint");
}

TEST(A3LayoutExtra, LegsAtSlots19To30) {
  const auto& a3 = robot_io::MakeA3Layout31();
  // Left leg [19..24]
  EXPECT_EQ(a3.names[19], "left_hip_pitch_joint");
  EXPECT_EQ(a3.names[20], "left_hip_roll_joint");
  EXPECT_EQ(a3.names[21], "left_hip_yaw_joint");
  EXPECT_EQ(a3.names[22], "left_knee_joint");
  EXPECT_EQ(a3.names[23], "left_ankle_pitch_joint");
  EXPECT_EQ(a3.names[24], "left_ankle_roll_joint");
  // Right leg [25..30]
  EXPECT_EQ(a3.names[25], "right_hip_pitch_joint");
  EXPECT_EQ(a3.names[26], "right_hip_roll_joint");
  EXPECT_EQ(a3.names[27], "right_hip_yaw_joint");
  EXPECT_EQ(a3.names[28], "right_knee_joint");
  EXPECT_EQ(a3.names[29], "right_ankle_pitch_joint");
  EXPECT_EQ(a3.names[30], "right_ankle_roll_joint");
}

TEST(A3LayoutExtra, NeckMappingIsIdentity) {
  EXPECT_EQ(robot_io::kFlatToA3NeckIdx[0], 0);
  EXPECT_EQ(robot_io::kFlatToA3NeckIdx[1], 1);
  EXPECT_EQ(robot_io::kA3NeckToFlatIdx[0], 0);
  EXPECT_EQ(robot_io::kA3NeckToFlatIdx[1], 1);
}

TEST(A3LayoutExtra, NeckJointNamesConstant) {
  EXPECT_EQ(robot_io::kA3NeckJointNames[0], "head_yaw_joint");
  EXPECT_EQ(robot_io::kA3NeckJointNames[1], "head_pitch_joint");
}

// Sanity: the 31 names are unique.
TEST(A3LayoutExtra, AllNamesUnique) {
  const auto& a3 = robot_io::MakeA3Layout31();
  for (int i = 0; i < 31; ++i) {
    for (int j = i + 1; j < 31; ++j) {
      EXPECT_NE(a3.names[i], a3.names[j])
          << "duplicate joint name at i=" << i << " j=" << j;
    }
  }
}
