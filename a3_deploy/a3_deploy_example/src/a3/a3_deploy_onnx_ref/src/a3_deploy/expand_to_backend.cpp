// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_pr9_spec.md §PR 8 Task 8.2 + 2026-04 MuJoCo realignment
// (notes/a3_dof_orderings.md).
#include "a3_deploy/expand_to_backend.hpp"

#include "robot_io/a3_layout_extra.hpp"

namespace a3_deploy {

// Scatter a 29-DOF policy output (in MuJoCo real 29-DOF policy view —
// waist, L_arm, R_arm, L_leg, R_leg) into a 31-DOF RobotCommand (in MuJoCo
// real 31-DOF SDK view — waist, neck, L_arm, R_arm, L_leg, R_leg).
//
// Non-neck 31-DOF slots fill from the corresponding policy-view slot via
// `kA3PolicyToSdkIdx[i]`. Head/neck slots [3..4] are held at q=0 with
// fixed deployment gains; they are not part of the policy's actuated set.
void ExpandToBackend(const std::array<double, 29>& q_des_29,
                     const std::array<double, 29>& kps,
                     const std::array<double, 29>& kds,
                     robot_io::RobotCommand& out) {
  constexpr int N = robot_io::kA3Dof;  // 31
  if (out.q_des.size()  != N) out.q_des.resize(N);
  if (out.dq_des.size() != N) out.dq_des.resize(N);
  if (out.tau_ff.size() != N) out.tau_ff.resize(N);
  if (out.kp.size()     != N) out.kp.resize(N);
  if (out.kd.size()     != N) out.kd.resize(N);

  // Start with everything zeroed. Non-neck slots get overwritten by the
  // scatter below; head/neck gains are filled after scatter.
  out.q_des.setZero();
  out.dq_des.setZero();
  out.tau_ff.setZero();
  out.kp.setZero();
  out.kd.setZero();

  // Scatter 29-policy → 29-non-neck-SDK-slots via kA3PolicyToSdkIdx.
  for (int i = 0; i < robot_io::kA3PolicyDof; ++i) {
    const int sdk = robot_io::kA3PolicyToSdkIdx[i];
    out.q_des[sdk] = q_des_29[i];
    out.kp[sdk]    = kps[i];
    out.kd[sdk]    = kds[i];
    // dq_des / tau_ff remain zero by default.
  }

  for (int i = 0; i < robot_io::kA3NeckCount; ++i) {
    const int sdk = robot_io::kA3NeckStart + i;
    out.q_des[sdk] = kA3HeadTargetPositionRad;
    out.kp[sdk] = kA3HeadKp;
    out.kd[sdk] = kA3HeadKd;
  }
}

}  // namespace a3_deploy
