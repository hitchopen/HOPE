// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// test_a3_policy_parameters.cpp
//
// GoogleTest unit tests for a3_policy_parameters.hpp (PR 2/10).
// See notes/a3_backend_plan.md §5.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>

#include "a3_policy_parameters.hpp"

TEST(A3PolicyParameters, ArraySizesAre29) {
  EXPECT_EQ(a3_default_angles.size(), 29u);
  EXPECT_EQ(a3_kps.size(), 29u);
  EXPECT_EQ(a3_kds.size(), 29u);
  EXPECT_EQ(a3_pd_stand_kps.size(), 29u);
  EXPECT_EQ(a3_pd_stand_kds.size(), 29u);
  EXPECT_EQ(a3_action_scale.size(), 29u);
  EXPECT_EQ(a3_isaaclab_to_mujoco.size(), 29u);
  EXPECT_EQ(a3_mujoco_to_isaaclab.size(), 29u);
  EXPECT_EQ(a3_vr_3point_index.size(), 3u);
  EXPECT_EQ(a3_vr_3point_body_offset.size(), 3u);
}

TEST(A3PolicyParameters, MappingsAreInverses) {
  // All values in valid range [0..28].
  for (int i = 0; i < 29; ++i) {
    EXPECT_GE(a3_isaaclab_to_mujoco[i], 0);
    EXPECT_LT(a3_isaaclab_to_mujoco[i], 29);
    EXPECT_GE(a3_mujoco_to_isaaclab[i], 0);
    EXPECT_LT(a3_mujoco_to_isaaclab[i], 29);
  }
  // Permutation check: both arrays cover [0..28] exactly once.
  std::array<int, 29> seen_itm{};
  std::array<int, 29> seen_mti{};
  for (int i = 0; i < 29; ++i) {
    seen_itm[a3_isaaclab_to_mujoco[i]]++;
    seen_mti[a3_mujoco_to_isaaclab[i]]++;
  }
  for (int v = 0; v < 29; ++v) {
    EXPECT_EQ(seen_itm[v], 1) << "isaaclab_to_mujoco missing/dup value " << v;
    EXPECT_EQ(seen_mti[v], 1) << "mujoco_to_isaaclab missing/dup value " << v;
  }
  // Round-trip: mti[itm[i]] == i and itm[mti[i]] == i.
  for (int i = 0; i < 29; ++i) {
    EXPECT_EQ(a3_mujoco_to_isaaclab[a3_isaaclab_to_mujoco[i]], i)
        << "mti o itm failed at i=" << i;
    EXPECT_EQ(a3_isaaclab_to_mujoco[a3_mujoco_to_isaaclab[i]], i)
        << "itm o mti failed at i=" << i;
  }
}

TEST(A3PolicyParameters, ActionScaleMatchesEffortOverStiffness) {
  constexpr std::array<double, 29> expected = {
      0.25 * 220.0 / 85.0,   // waist_yaw
      0.25 * 46.0 / 50.0,    // waist_roll
      0.25 * 115.0 / 50.0,   // waist_pitch
      0.25 * 60.0 / 40.0,    // left_shoulder_pitch
      0.25 * 60.0 / 40.0,    // left_shoulder_roll
      0.25 * 24.0 / 30.0,    // left_shoulder_yaw
      0.25 * 24.0 / 30.0,    // left_elbow
      0.25 * 24.0 / 30.0,    // left_wrist_roll
      0.25 * 6.0 / 20.0,     // left_wrist_pitch
      0.25 * 6.0 / 20.0,     // left_wrist_yaw
      0.25 * 60.0 / 40.0,    // right_shoulder_pitch
      0.25 * 60.0 / 40.0,    // right_shoulder_roll
      0.25 * 24.0 / 30.0,    // right_shoulder_yaw
      0.25 * 24.0 / 30.0,    // right_elbow
      0.25 * 24.0 / 30.0,    // right_wrist_roll
      0.25 * 6.0 / 20.0,     // right_wrist_pitch
      0.25 * 6.0 / 20.0,     // right_wrist_yaw
      0.25 * 220.0 / 80.0,   // left_hip_pitch
      0.25 * 220.0 / 120.0,  // left_hip_roll
      0.25 * 220.0 / 80.0,   // left_hip_yaw
      0.25 * 320.0 / 250.0,  // left_knee
      0.25 * 118.2 / 50.0,   // left_ankle_pitch
      0.25 * 54.75 / 50.0,   // left_ankle_roll
      0.25 * 220.0 / 80.0,   // right_hip_pitch
      0.25 * 220.0 / 120.0,  // right_hip_roll
      0.25 * 220.0 / 80.0,   // right_hip_yaw
      0.25 * 320.0 / 250.0,  // right_knee
      0.25 * 118.2 / 50.0,   // right_ankle_pitch
      0.25 * 54.75 / 50.0,   // right_ankle_roll
  };
  for (int i = 0; i < 29; ++i) {
    EXPECT_DOUBLE_EQ(a3_action_scale[i], expected[i]) << "i=" << i;
  }
}

TEST(A3PolicyParameters, KeyValuesSpotCheck) {
  // Post 2026-04 MuJoCo realignment (notes/a3_dof_orderings.md): constants
  // are in the 29-DOF MuJoCo policy view (waist, L_arm, R_arm, L_leg, R_leg).
  //
  // Default angles (a3.py lines 109, 115-118) at their new indices.
  EXPECT_DOUBLE_EQ(a3_default_angles[ 0], 0.0);     // waist_yaw
  EXPECT_DOUBLE_EQ(a3_default_angles[ 3], 0.3);     // L_shoulder_pitch
  EXPECT_DOUBLE_EQ(a3_default_angles[ 6], 0.8);     // L_elbow
  EXPECT_DOUBLE_EQ(a3_default_angles[17], -0.1311); // L_hip_pitch
  EXPECT_DOUBLE_EQ(a3_default_angles[20],  0.2468); // L_knee

  // PD gains.
  EXPECT_DOUBLE_EQ(a3_kps[ 0], 85.0);   // waist_yaw
  EXPECT_DOUBLE_EQ(a3_kps[ 6], 30.0);   // L_elbow
  EXPECT_DOUBLE_EQ(a3_kps[ 8], 20.0);   // L_wrist_pitch
  EXPECT_DOUBLE_EQ(a3_kps[20], 250.0);  // L_knee
  EXPECT_DOUBLE_EQ(a3_kps[21], 50.0);   // L_ankle_pitch

  EXPECT_DOUBLE_EQ(a3_kds[ 0], 3.0);    // waist_yaw
  EXPECT_DOUBLE_EQ(a3_kds[20], 8.0);    // L_knee

  // Manual real-robot PD_STAND gains from motion_control_a3.
  EXPECT_DOUBLE_EQ(a3_pd_stand_kps[ 0], 400.0);   // waist_yaw
  EXPECT_DOUBLE_EQ(a3_pd_stand_kps[ 3], 200.0);   // L_shoulder_pitch
  EXPECT_DOUBLE_EQ(a3_pd_stand_kps[17], 1500.0);  // L_hip_pitch
  EXPECT_DOUBLE_EQ(a3_pd_stand_kps[20], 2000.0);  // L_knee

  EXPECT_DOUBLE_EQ(a3_pd_stand_kds[ 0], 4.0);     // waist_yaw
  EXPECT_DOUBLE_EQ(a3_pd_stand_kds[17], 8.0);     // L_hip_pitch
  EXPECT_DOUBLE_EQ(a3_pd_stand_kds[20], 8.0);     // L_knee
}

TEST(A3PolicyParameters, Vr3PointValid) {
  ASSERT_EQ(a3_vr_3point_index.size(), 3u);
  // Body-link indices must be valid IsaacLab body-link indices in
  // A3_ISAACLAB_JOINTS (30 entries, indices [0..29]).
  for (int i = 0; i < 3; ++i) {
    EXPECT_GE(a3_vr_3point_index[i], 0);
    EXPECT_LE(a3_vr_3point_index[i], 29);
  }
  // Spot check exact values.
  EXPECT_EQ(a3_vr_3point_index[0], 28);  // left_wrist_yaw_Link
  EXPECT_EQ(a3_vr_3point_index[1], 29);  // right_wrist_yaw_Link
  EXPECT_EQ(a3_vr_3point_index[2],  9);  // torso_Link

  // Body offsets from sonic_a3_simple.yaml.
  EXPECT_DOUBLE_EQ(a3_vr_3point_body_offset[0][0],  0.11006561);
  EXPECT_DOUBLE_EQ(a3_vr_3point_body_offset[0][1], -0.00410937);
  EXPECT_DOUBLE_EQ(a3_vr_3point_body_offset[0][2],  0.0048369);
  EXPECT_DOUBLE_EQ(a3_vr_3point_body_offset[1][0],  0.11008445);
  EXPECT_DOUBLE_EQ(a3_vr_3point_body_offset[1][1],  0.00411033);
  EXPECT_DOUBLE_EQ(a3_vr_3point_body_offset[1][2],  0.00484137);
  EXPECT_DOUBLE_EQ(a3_vr_3point_body_offset[2][0],  0.0507937);
  EXPECT_DOUBLE_EQ(a3_vr_3point_body_offset[2][1],  0.00056856);
  EXPECT_DOUBLE_EQ(a3_vr_3point_body_offset[2][2],  0.55502408);
}
