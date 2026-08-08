// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// Regression guard for A3's 31-DOF MuJoCo DOF order (notes/a3_dof_orderings.md).
//
// The "single source of truth" for A3's SDK-layer layout is mujoco.MjModel
// iteration of a3_t2d5.xml (cross-checked by
// training_scripts/generate_a3_order_mapping.py on the training side).
// This test pins the exact 31 joint names, in order, as a hardcoded list —
// so any accidental reshuffle of MakeA3Layout31() (or an A3 revision that
// re-orders its MJCF) lights up loudly in CI instead of drifting silently
// through the policy path.

#include <gtest/gtest.h>

#include "robot_io/a3_layout_extra.hpp"

#include <array>
#include <string>
#include <vector>

TEST(A3MujocoOrderSanity, MakeA3Layout31MatchesMujocoRealOrder) {
  // Authoritative 31-DOF joint list — matches
  // mujoco.MjModel.njnt iteration of a3_t2d5.xml (verified 2026-04-28).
  //
  // Ordering rationale spelled out in notes/a3_dof_orderings.md:
  //   [0..2]   waist  (yaw, roll, pitch)
  //   [3..4]   neck   (head_yaw, head_pitch) — excluded from 29-policy
  //   [5..11]  L_arm  (shoulder p/r/y, elbow, wrist r/p/y)
  //   [12..18] R_arm  (shoulder p/r/y, elbow, wrist r/p/y)
  //   [19..24] L_leg  (hip p/r/y, knee, ankle p/r)
  //   [25..30] R_leg  (hip p/r/y, knee, ankle p/r)
  const std::array<const char*, 31> expected = {
      "waist_yaw_joint",
      "waist_roll_joint",
      "waist_pitch_joint",
      "head_yaw_joint",
      "head_pitch_joint",
      "left_shoulder_pitch_joint",
      "left_shoulder_roll_joint",
      "left_shoulder_yaw_joint",
      "left_elbow_joint",
      "left_wrist_roll_joint",
      "left_wrist_pitch_joint",
      "left_wrist_yaw_joint",
      "right_shoulder_pitch_joint",
      "right_shoulder_roll_joint",
      "right_shoulder_yaw_joint",
      "right_elbow_joint",
      "right_wrist_roll_joint",
      "right_wrist_pitch_joint",
      "right_wrist_yaw_joint",
      "left_hip_pitch_joint",
      "left_hip_roll_joint",
      "left_hip_yaw_joint",
      "left_knee_joint",
      "left_ankle_pitch_joint",
      "left_ankle_roll_joint",
      "right_hip_pitch_joint",
      "right_hip_roll_joint",
      "right_hip_yaw_joint",
      "right_knee_joint",
      "right_ankle_pitch_joint",
      "right_ankle_roll_joint",
  };

  const auto& layout = robot_io::MakeA3Layout31();
  ASSERT_EQ(layout.names.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(layout.names[i], expected[i])
        << "mismatch at SDK slot " << i << " — if you've changed the A3 "
        << "layout, update both this test AND notes/a3_dof_orderings.md "
        << "to match the new MJCF.";
  }
}

TEST(A3MujocoOrderSanity, PolicyViewIsMujocoMinusNeck) {
  // The 29-DOF policy view should be the 31-DOF layout with the 2 neck
  // slots excised, preserving relative order. Verify by composing the
  // gather table against the 31-DOF layout.
  const auto& layout = robot_io::MakeA3Layout31();
  ASSERT_EQ(layout.names.size(), 31u);
  ASSERT_EQ(robot_io::kA3PolicyToSdkIdx.size(), 29u);

  // Collect the joint names each policy-view index corresponds to.
  std::vector<std::string> policy_view;
  policy_view.reserve(29);
  for (int i = 0; i < robot_io::kA3PolicyDof; ++i) {
    const int sdk = robot_io::kA3PolicyToSdkIdx[i];
    ASSERT_GE(sdk, 0);
    ASSERT_LT(sdk, 31);
    policy_view.push_back(layout.names[sdk]);
  }

  // Build the expected 29-element list by excising neck slots [3..4] from
  // the 31-DOF layout.
  std::vector<std::string> expected;
  expected.reserve(29);
  for (int i = 0; i < 31; ++i) {
    if (i >= robot_io::kA3NeckStart &&
        i <  robot_io::kA3NeckStart + robot_io::kA3NeckCount) {
      continue;  // skip neck
    }
    expected.push_back(layout.names[i]);
  }

  ASSERT_EQ(policy_view.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(policy_view[i], expected[i])
        << "policy-view slot " << i << " disagrees with "
        << "MuJoCo-minus-neck expectation";
  }
}

TEST(A3MujocoOrderSanity, PolicyToSdkIdxCoversAllNonNeckSlots) {
  // Every non-neck slot of the 31-DOF layout should appear exactly once
  // in kA3PolicyToSdkIdx. (Bijection check — guards against the inverse
  // bug where we might gather the same SDK slot twice.)
  std::array<int, 31> seen{};
  for (int i = 0; i < robot_io::kA3PolicyDof; ++i) {
    const int sdk = robot_io::kA3PolicyToSdkIdx[i];
    ASSERT_GE(sdk, 0);
    ASSERT_LT(sdk, 31);
    seen[sdk]++;
  }
  for (int sdk = 0; sdk < 31; ++sdk) {
    const bool is_neck =
        (sdk >= robot_io::kA3NeckStart &&
         sdk <  robot_io::kA3NeckStart + robot_io::kA3NeckCount);
    if (is_neck) {
      EXPECT_EQ(seen[sdk], 0) << "neck slot " << sdk
                              << " should NOT appear in the policy view";
    } else {
      EXPECT_EQ(seen[sdk], 1) << "non-neck SDK slot " << sdk
                              << " should appear exactly once";
    }
  }
}
