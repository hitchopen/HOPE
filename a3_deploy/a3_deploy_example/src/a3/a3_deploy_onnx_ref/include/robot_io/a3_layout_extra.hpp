// Copyright (c) 2026, AgiBot Inc. All rights reserved.
#pragma once

// A3 31-DOF layout extension — matches the authoritative MuJoCo DOF order
// (mujoco.MjModel.njnt iteration for a3_t2d5.xml). A3 is actuated on all
// 31 DOFs; the "29-DOF policy view" is obtained by skipping the two neck
// slots. See notes/a3_dof_orderings.md for the full discussion.
//
// This header is a pure additive extension over robot_io/layouts.hpp: it
// does not modify any A3 data structures.

#include "robot_io/robot_io_backend.hpp"

#include <array>
#include <string>

namespace robot_io {

// Total DOF of the A3 31-DOF layout.
constexpr int kA3Dof       = 31;

// 31-DOF A3 SDK layout start/count constants.
// Matches MuJoCo's real order (mujoco.MjModel.njnt iteration for a3_t2d5.xml):
//   [0..2]   waist     (yaw, roll, pitch)
//   [3..4]   neck      (head_yaw, head_pitch)
//   [5..18]  arms      (L then R, 7 DOF each)
//   [19..30] legs      (L then R, 6 DOF each)
//
// These MUST be used by A3 publishers/subscribers when slicing a 31-DOF
// RobotCommand/RobotState. Do NOT confuse with the A3 29-DOF layout
// constants kLegStart/kWaistStart/kArmStart in robot_io/layouts.hpp —
// those follow the Unitree SDK order and are incompatible with A3.
//
// Historical note: the initial PR 1 draft placed neck at the tail [29..30]
// with body joints in A3-compatible order [0..28]. We realigned to MuJoCo's
// real order in April 2026 (see notes/a3_dof_orderings.md).
constexpr int kA3WaistStart = 0;
constexpr int kA3WaistCount = 3;
constexpr int kA3NeckStart  = 3;
constexpr int kA3NeckCount  = 2;
constexpr int kA3ArmStart   = 5;
constexpr int kA3ArmCount   = 14;  // 7 L + 7 R
constexpr int kA3LegStart   = 19;
constexpr int kA3LegCount   = 12;  // 6 L + 6 R

// 31-DOF A3 layout accessor.
//
// Order (matches mujoco.MjModel.njnt iteration for a3_t2d5.xml):
//   [0..2]   waist     (yaw, roll, pitch)
//   [3..4]   neck      (head_yaw, head_pitch)
//   [5..11]  left arm  (shoulder_p/r/y, elbow, wrist_r/p/y)
//   [12..18] right arm (shoulder_p/r/y, elbow, wrist_r/p/y)
//   [19..24] left leg  (hip_p/r/y, knee, ankle_p/r)
//   [25..30] right leg (hip_p/r/y, knee, ankle_p/r)
//
// ⚠️ Not the same order as A3's 29-DOF flat layout — A3 uses
//   legs(12) + waist(3) + arms(14) which coincides with Unitree SDK
//   motor_state order and A3 MuJoCo. A3 has no such coincidence.
const JointLayout& MakeA3Layout31();

// Neck-group mappings between the flat-layout neck sub-indices and the
// A3 neck topic's internal order. Currently identity; declared explicitly so
// a future A3 revision that re-orders the neck topic only needs edits here.
//   kFlatToA3NeckIdx[i] == j  ⇒  flat layout neck sub-index `i` (i.e. the
//                              31-DOF slot kA3NeckStart + i) lands at
//                              position `j` inside the A3 neck topic.
extern const std::array<int, kA3NeckCount> kFlatToA3NeckIdx;
extern const std::array<int, kA3NeckCount> kA3NeckToFlatIdx;

// A3 neck joint names in A3-native topic order (same as MuJoCo slots [3..4]).
extern const std::array<std::string, kA3NeckCount> kA3NeckJointNames;

// ---------------------------------------------------------------------------
// 29-DOF policy view ↔ 31-DOF SDK view bridge.
//
// The policy operates on 29 DOF (waist + arms + legs; no neck). The SDK
// layer exposes 31 DOF (the full MuJoCo order — see MakeA3Layout31 above).
// kA3PolicyToSdkIdx[i] holds the 31-DOF slot that the i-th policy-view DOF
// lives in:
//
//   policy_q[i]   = sdk_q[kA3PolicyToSdkIdx[i]]                  (gather)
//   sdk_q[kA3PolicyToSdkIdx[i]] = policy_q[i]                    (scatter)
//
// This is the permutation "MuJoCo [0..30] minus slots [3..4] (neck)".
// Neck slots stay at SDK [3..4] and are filled separately (see
// ExpandToBackend / safe_halt). Normal policy commands hold head/neck at
// zero with fixed PD gains; safe halt still zeroes gains.
//
// Ordering chosen so that policy-view indices preserve MuJoCo ordering for
// the non-neck joints (waist first, then arms L/R, then legs L/R), which
// in turn matches a3_mujoco_to_isaaclab[]'s expectation in
// a3_policy_parameters.hpp.
// ---------------------------------------------------------------------------
constexpr int kA3PolicyDof = 29;
constexpr std::array<int, kA3PolicyDof> kA3PolicyToSdkIdx = {
    // waist  [0..2]     ← SDK [0..2]
    0, 1, 2,
    // L_arm  [3..9]     ← SDK [5..11]   (SDK [3..4] = neck, skipped)
    5, 6, 7, 8, 9, 10, 11,
    // R_arm  [10..16]   ← SDK [12..18]
    12, 13, 14, 15, 16, 17, 18,
    // L_leg  [17..22]   ← SDK [19..24]
    19, 20, 21, 22, 23, 24,
    // R_leg  [23..28]   ← SDK [25..30]
    25, 26, 27, 28, 29, 30,
};

// Gather the 29-DOF policy view out of a 31-DOF SDK-view Eigen vector
// (typically RobotState::q / ::dq). Writes into `out_29`. Inline + noexcept;
// safe to call from the RT policy callback.
//
// If `sdk_31.size() != kA3Dof`, missing slots yield 0.0 (defensive — the
// sync-loop always produces size==31, but we don't throw on the RT path).
inline void ExtractPolicyView(const Eigen::VectorXd& sdk_31,
                              std::array<double, kA3PolicyDof>& out_29) noexcept {
  const int n = static_cast<int>(sdk_31.size());
  for (int i = 0; i < kA3PolicyDof; ++i) {
    const int sdk = kA3PolicyToSdkIdx[i];
    out_29[i] = (sdk < n) ? sdk_31[sdk] : 0.0;
  }
}

}  // namespace robot_io
