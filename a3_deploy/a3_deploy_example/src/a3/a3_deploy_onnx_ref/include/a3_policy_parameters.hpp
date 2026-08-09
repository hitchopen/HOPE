// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// a3_policy_parameters.hpp
//
// A3-specific policy constants: PD gains, default standing angles, per-joint
// action scales, IsaacLab<->MuJoCo DoF mappings, and VR 3-point indices.
//
// All 29-element arrays are in the **29-DOF MuJoCo policy view** order
// (defined by kA3PolicyToSdkIdx in robot_io/a3_layout_extra.hpp — i.e. the
// full 31-DOF MuJoCo layout with the two neck slots [3..4] dropped):
//   waist   [ 0.. 2]  waist_{yaw, roll, pitch}
//   L_arm   [ 3.. 9]  left_{shoulder_p/r/y, elbow, wrist_r/p/y}
//   R_arm   [10..16]  right_{shoulder_p/r/y, elbow, wrist_r/p/y}
//   L_leg   [17..22]  left_{hip_p/r/y, knee, ankle_p/r}
//   R_leg   [23..28]  right_{hip_p/r/y, knee, ankle_p/r}
//
// This matches generate_a3_order_mapping.py's `mujoco_dofs_for_mapping` (the
// 29-DOF filtered subset of mujoco.MjModel.njnt). It also matches the
// 29-DOF axis that a3_isaaclab_to_mujoco / a3_mujoco_to_isaaclab permute
// against — so the perms + constants are all in the same frame.
//
// Historical note: pre 2026-04 these arrays were stored in A3's flat layout
// order (legs + waist + arms), causing a silent mis-mapping in
// A3ObsBuilder. The 2026-04 MuJoCo realignment reordered them to match the
// MuJoCo policy view — values unchanged, indices permuted. See
// notes/a3_dof_orderings.md for the full discussion.
//
// Sources (authoritative):
//   training_envs/manager_env/robots/a3.py
//     - joint_pos initializer (lines 108-141) -> a3_default_angles
//     - actuators.{legs,feet,waist,arms}.{stiffness,damping} (lines 146-345)
//       -> a3_kps / a3_kds
//     - A3_ACTION_SCALE construction: 0.25 * effort_limit_sim / stiffness
//     - A3_ISAACLAB_TO_MUJOCO_DOF (lines 386-391)
//     - A3_ISAACLAB_JOINTS (lines 349-380, 30 body-link names) for VR indices
//   training_config/exp/manager/universal_token/all_modes/sonic_a3_simple.yaml
//     - vr_3point_body / vr_3point_body_offset (lines 83-84)

#pragma once

#include <array>

// -----------------------------------------------------------------------------
// Default standing-pose joint angles (radians), in the 29-DOF MuJoCo policy
// view (see header note above).
// Values from A3_CYLINDER_CFG.init_state.joint_pos (a3.py lines 108-141).
// The 4 "Fixed wrist joints" (left/right wrist_pitch/wrist_yaw, a3.py lines
// 136-140) are 0.0 in the dict; kept explicitly so every joint has a value.
// -----------------------------------------------------------------------------
constexpr std::array<double, 29> a3_default_angles = {
    // waist [0..2]
     0.0,      // [ 0] waist_yaw_joint
     0.0,      // [ 1] waist_roll_joint
     0.0,      // [ 2] waist_pitch_joint
    // left arm [3..9]
     0.3,      // [ 3] left_shoulder_pitch_joint
     0.12,     // [ 4] left_shoulder_roll_joint
     0.0,      // [ 5] left_shoulder_yaw_joint
     0.8,      // [ 6] left_elbow_joint
     0.0,      // [ 7] left_wrist_roll_joint
     0.0,      // [ 8] left_wrist_pitch_joint
     0.0,      // [ 9] left_wrist_yaw_joint
    // right arm [10..16]
     0.3,      // [10] right_shoulder_pitch_joint
    -0.12,     // [11] right_shoulder_roll_joint
     0.0,      // [12] right_shoulder_yaw_joint
     0.8,      // [13] right_elbow_joint
     0.0,      // [14] right_wrist_roll_joint
     0.0,      // [15] right_wrist_pitch_joint
     0.0,      // [16] right_wrist_yaw_joint
    // left leg [17..22]
    -0.1311,   // [17] left_hip_pitch_joint
     0.0056,   // [18] left_hip_roll_joint
    -0.0348,   // [19] left_hip_yaw_joint
     0.2468,   // [20] left_knee_joint
    -0.1204,   // [21] left_ankle_pitch_joint
    -0.0078,   // [22] left_ankle_roll_joint
    // right leg [23..28]
    -0.1311,   // [23] right_hip_pitch_joint
    -0.0056,   // [24] right_hip_roll_joint
     0.0348,   // [25] right_hip_yaw_joint
     0.2468,   // [26] right_knee_joint
    -0.1204,   // [27] right_ankle_pitch_joint
     0.0078,   // [28] right_ankle_roll_joint
};

// -----------------------------------------------------------------------------
// PD stiffness (Kp) per joint (N·m/rad), in the 29-DOF MuJoCo policy view.
// Direct transcription of actuator.stiffness dicts in a3.py:
//   legs  (a3.py 165-170): hip_roll=120, hip_yaw=80, hip_pitch=80, knee=250
//   feet  (a3.py 209-212): ankle_pitch=50, ankle_roll=50
//   waist (a3.py 246-250): waist_yaw=85, waist_roll=50, waist_pitch=50
//   arms  (a3.py 300-308): shoulder_pitch/roll=40, shoulder_yaw/elbow/
//                           wrist_roll=30, wrist_pitch/yaw=20
// -----------------------------------------------------------------------------
constexpr std::array<double, 29> a3_kps = {
    // waist [0..2]
     85.0,   // [ 0] waist_yaw_joint
     50.0,   // [ 1] waist_roll_joint
     50.0,   // [ 2] waist_pitch_joint
    // left arm [3..9]
     40.0,   // [ 3] left_shoulder_pitch_joint
     40.0,   // [ 4] left_shoulder_roll_joint
     30.0,   // [ 5] left_shoulder_yaw_joint
     30.0,   // [ 6] left_elbow_joint
     30.0,   // [ 7] left_wrist_roll_joint
     20.0,   // [ 8] left_wrist_pitch_joint
     20.0,   // [ 9] left_wrist_yaw_joint
    // right arm [10..16]
     40.0,   // [10] right_shoulder_pitch_joint
     40.0,   // [11] right_shoulder_roll_joint
     30.0,   // [12] right_shoulder_yaw_joint
     30.0,   // [13] right_elbow_joint
     30.0,   // [14] right_wrist_roll_joint
     20.0,   // [15] right_wrist_pitch_joint
     20.0,   // [16] right_wrist_yaw_joint
    // left leg [17..22]
     80.0,   // [17] left_hip_pitch_joint
    120.0,   // [18] left_hip_roll_joint
     80.0,   // [19] left_hip_yaw_joint
    250.0,   // [20] left_knee_joint
     50.0,   // [21] left_ankle_pitch_joint
     50.0,   // [22] left_ankle_roll_joint
    // right leg [23..28]
     80.0,   // [23] right_hip_pitch_joint
    120.0,   // [24] right_hip_roll_joint
     80.0,   // [25] right_hip_yaw_joint
    250.0,   // [26] right_knee_joint
     50.0,   // [27] right_ankle_pitch_joint
     50.0,   // [28] right_ankle_roll_joint
};

// -----------------------------------------------------------------------------
// PD damping (Kd) per joint (N·m·s/rad), in the 29-DOF MuJoCo policy view.
// Direct transcription of actuator.damping dicts in a3.py:
//   legs  (a3.py 171-176): hip_roll=4, hip_yaw=3, hip_pitch=3, knee=8
//   feet  (a3.py 213-216): ankle_pitch=2, ankle_roll=2
//   waist (a3.py 251-255): waist_yaw=3, waist_roll=2, waist_pitch=2
//   arms  (a3.py 309-317): shoulder_pitch/roll=3, others=2
// -----------------------------------------------------------------------------
constexpr std::array<double, 29> a3_kds = {
    // waist [0..2]
    3.0,   // [ 0] waist_yaw_joint
    2.0,   // [ 1] waist_roll_joint
    2.0,   // [ 2] waist_pitch_joint
    // left arm [3..9]
    3.0,   // [ 3] left_shoulder_pitch_joint
    3.0,   // [ 4] left_shoulder_roll_joint
    2.0,   // [ 5] left_shoulder_yaw_joint
    2.0,   // [ 6] left_elbow_joint
    2.0,   // [ 7] left_wrist_roll_joint
    2.0,   // [ 8] left_wrist_pitch_joint
    2.0,   // [ 9] left_wrist_yaw_joint
    // right arm [10..16]
    3.0,   // [10] right_shoulder_pitch_joint
    3.0,   // [11] right_shoulder_roll_joint
    2.0,   // [12] right_shoulder_yaw_joint
    2.0,   // [13] right_elbow_joint
    2.0,   // [14] right_wrist_roll_joint
    2.0,   // [15] right_wrist_pitch_joint
    2.0,   // [16] right_wrist_yaw_joint
    // left leg [17..22]
    3.0,   // [17] left_hip_pitch_joint
    4.0,   // [18] left_hip_roll_joint
    3.0,   // [19] left_hip_yaw_joint
    8.0,   // [20] left_knee_joint
    2.0,   // [21] left_ankle_pitch_joint
    2.0,   // [22] left_ankle_roll_joint
    // right leg [23..28]
    3.0,   // [23] right_hip_pitch_joint
    4.0,   // [24] right_hip_roll_joint
    3.0,   // [25] right_hip_yaw_joint
    8.0,   // [26] right_knee_joint
    2.0,   // [27] right_ankle_pitch_joint
    2.0,   // [28] right_ankle_roll_joint
};

// -----------------------------------------------------------------------------
// Real-robot PD_STAND gains, in the 29-DOF MuJoCo policy view.
//
// These are intentionally separate from a3_kps/a3_kds above. The policy path
// continues to use training-side gains from a3.py, while the manual PD_STAND
// bring-up state uses the production stand gains from:
//   ~/aimde/aimrt_motion_control_a3/config/motion_control/configuration/
//     robot/a3_t2d5/pd_stand/default.yaml
//
// Source order in that YAML is:
//   L_leg, R_leg, waist, L_arm, R_arm, head
// and is transcribed here into the 29-DOF policy view:
//   waist, L_arm, R_arm, L_leg, R_leg
// Head/neck is not in the policy view; PD_STAND currently leaves neck passive
// via ExpandToBackend's zero neck slots.
// -----------------------------------------------------------------------------
constexpr std::array<double, 29> a3_pd_stand_kps = {
    // waist [0..2]
    400.0, 500.0, 500.0,
    // left arm [3..9]
    200.0, 200.0, 100.0, 200.0, 100.0, 50.0, 50.0,
    // right arm [10..16]
    200.0, 200.0, 100.0, 200.0, 100.0, 50.0, 50.0,
    // left leg [17..22]
    1500.0, 400.0, 300.0, 2000.0, 500.0, 500.0,
    // right leg [23..28]
    1500.0, 400.0, 300.0, 2000.0, 500.0, 500.0,
};

constexpr std::array<double, 29> a3_pd_stand_kds = {
    // waist [0..2]
    4.0, 4.0, 4.0,
    // left arm [3..9]
    2.0, 2.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    // right arm [10..16]
    2.0, 2.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    // left leg [17..22]
    8.0, 7.0, 7.0, 8.0, 5.0, 5.0,
    // right leg [23..28]
    8.0, 7.0, 7.0, 8.0, 5.0, 5.0,
};

// -----------------------------------------------------------------------------
// Per-joint action scale. Current a3.py builds A3_ACTION_SCALE from actuator
// effort and stiffness:
//   action_scale[joint] = 0.25 * effort_limit_sim[joint] / stiffness[joint]
// The policy target is target = action * action_scale + default_angle.
// -----------------------------------------------------------------------------
constexpr std::array<double, 29> a3_action_scale = {
    // waist [0..2]
    0.6470588235294118,  // [ 0] waist_yaw_joint
    0.23,                // [ 1] waist_roll_joint
    0.575,               // [ 2] waist_pitch_joint
    // left arm [3..9]
    0.375,               // [ 3] left_shoulder_pitch_joint
    0.375,               // [ 4] left_shoulder_roll_joint
    0.2,                 // [ 5] left_shoulder_yaw_joint
    0.2,                 // [ 6] left_elbow_joint
    0.2,                 // [ 7] left_wrist_roll_joint
    0.075,               // [ 8] left_wrist_pitch_joint
    0.075,               // [ 9] left_wrist_yaw_joint
    // right arm [10..16]
    0.375,               // [10] right_shoulder_pitch_joint
    0.375,               // [11] right_shoulder_roll_joint
    0.2,                 // [12] right_shoulder_yaw_joint
    0.2,                 // [13] right_elbow_joint
    0.2,                 // [14] right_wrist_roll_joint
    0.075,               // [15] right_wrist_pitch_joint
    0.075,               // [16] right_wrist_yaw_joint
    // left leg [17..22]
    0.6875,              // [17] left_hip_pitch_joint
    0.4583333333333333,  // [18] left_hip_roll_joint
    0.6875,              // [19] left_hip_yaw_joint
    0.32,                // [20] left_knee_joint
    0.591,               // [21] left_ankle_pitch_joint
    0.27375,             // [22] left_ankle_roll_joint
    // right leg [23..28]
    0.6875,              // [23] right_hip_pitch_joint
    0.4583333333333333,  // [24] right_hip_roll_joint
    0.6875,              // [25] right_hip_yaw_joint
    0.32,                // [26] right_knee_joint
    0.591,               // [27] right_ankle_pitch_joint
    0.27375,             // [28] right_ankle_roll_joint
};

// -----------------------------------------------------------------------------
// IsaacLab -> MuJoCo DoF remapping (gather-index convention):
//   data_in_isaaclab[..., a3_isaaclab_to_mujoco[i]]  produces the value at
//   MuJoCo (policy-view) DoF index i. Verbatim from a3.py A3_ISAACLAB_TO_MUJOCO_DOF
//   (lines 386-391). 29→29 permutation between the two DOF conventions.
// -----------------------------------------------------------------------------
constexpr std::array<int, 29> a3_isaaclab_to_mujoco = {
     2,  5,  8, 11, 15, 19, 21, 23, 25, 27,  // [ 0..9 ]
    12, 16, 20, 22, 24, 26, 28,              // [10..16]
     0,  3,  6,  9, 13, 17,                  // [17..22]
     1,  4,  7, 10, 14, 18,                  // [23..28]
};

// -----------------------------------------------------------------------------
// MuJoCo -> IsaacLab DoF remapping (argsort of a3_isaaclab_to_mujoco).
//
// ⚠️ BUG NOTICE: a3.py's A3_MUJOCO_TO_ISAACLAB_DOF (lines 393-397) has
// out-of-range values (contains 29 and 30, should only be [0..28]) and is
// NOT the true inverse of A3_ISAACLAB_TO_MUJOCO_DOF. It appears to have been
// copy-edited from a 31-DOF body mapping.
//
// The values below are the CORRECT 29-element argsort of
// a3_isaaclab_to_mujoco, i.e. a3_mujoco_to_isaaclab[a3_isaaclab_to_mujoco[i]]
// == i for all i in [0..28]. The Python verify script
// (scripts/verify_a3_constants.py) cross-checks this by computing
// numpy.argsort(A3_ISAACLAB_TO_MUJOCO_DOF) instead of trusting a3.py's
// buggy dict.
// -----------------------------------------------------------------------------
constexpr std::array<int, 29> a3_mujoco_to_isaaclab = {
    17, 23,  0, 18, 24,  1, 19, 25,  2, 20,  // [ 0..9 ]
    26,  3, 10, 21, 27,  4, 11, 22, 28,  5,  // [10..19]
    12,  6, 13,  7, 14,  8, 15,  9, 16,       // [20..28]
};

// -----------------------------------------------------------------------------
// VR 3-point tracking indices (IsaacLab body-link indices, NOT DoF indices).
// From sonic_a3_simple.yaml:
//   vr_3point_body: ["left_wrist_yaw_Link", "right_wrist_yaw_Link",
//                    "torso_Link"]
// Looked up in A3_ISAACLAB_JOINTS (a3.py lines 349-380, 30 body links,
// pelvis_link at index 0):
//   left_wrist_yaw_Link  -> 28
//   right_wrist_yaw_Link -> 29
//   torso_Link           ->  9
// -----------------------------------------------------------------------------
constexpr std::array<int, 3> a3_vr_3point_index = {28, 29, 9};

// -----------------------------------------------------------------------------
// Per-body XYZ offsets (metres) applied to the VR 3-point targets.
// From sonic_a3_simple.yaml:
//   vr_3point_body_offset:
//     [[0.11006561, -0.00410937, 0.0048369],
//      [0.11008445,  0.00411033, 0.00484137],
//      [0.0507937,   0.00056856, 0.55502408]]
// -----------------------------------------------------------------------------
constexpr std::array<std::array<double, 3>, 3> a3_vr_3point_body_offset = {{
    {0.11006561, -0.00410937, 0.0048369},  // left_wrist_yaw_Link -> left_hand_Link inertial
    {0.11008445,  0.00411033, 0.00484137}, // right_wrist_yaw_Link -> right_hand_Link inertial
    {0.0507937,   0.00056856, 0.55502408}, // torso_Link -> head_pitch_Link inertial
}};
