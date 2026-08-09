// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_pr9_spec.md §2 (a3_action_decoder)
//
// DecodeAction: pure function mapping a raw ONNX policy output (29 floats,
// IsaacLab order) to a 29-element joint-position command in the 29-DOF
// MuJoCo policy view (waist + arms + legs; see notes/a3_dof_orderings.md).
// The resulting q_des_29 is scattered into the 31-DOF SDK view by
// ExpandToBackend (via kA3PolicyToSdkIdx).
//
// Mathematical formula (IsaacLab MDP `ProcessedAction` convention):
//     q_des_isaac[i] = raw[i] * scale[i] + default[i]
//     q_des_mujoco[i] = q_des_isaac[a3_isaaclab_to_mujoco[i]]
//
// Storage note: `a3_action_scale` and `a3_default_angles` in
// `a3_policy_parameters.hpp` are stored in the 29-DOF MuJoCo policy view
// (the same frame the output uses) — so the equivalent MuJoCo-indexed
// formula used here is
//     q_des_mujoco[i] =
//         raw_isaac[a3_isaaclab_to_mujoco[i]] * a3_action_scale[i]
//       + a3_default_angles[i]
// This matches A3's decoder (a3_deploy_onnx_ref.cpp:3089-3091) modulo the
// different MuJoCo axis convention between A3 (Unitree flat) and A3
// (MuJoCo real).
//
// See notes/a3_onnx_io_spec.md §"A3 DOF order" for the IsaacLab DOF layout
// and a3_policy_parameters.hpp for the authoritative constants.
#pragma once

#include <array>

namespace a3_deploy {

// Decode 29-float IsaacLab-ordered raw policy output into a 29-double
// MuJoCo-ordered desired-position array suitable for ExpandToBackend.
// noexcept — no heap allocations, no logging.
void DecodeAction(const std::array<float, 29>& raw_action_isaaclab,
                  std::array<double, 29>&     q_des_mujoco) noexcept;

}  // namespace a3_deploy
