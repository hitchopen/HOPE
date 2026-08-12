// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_pr9_spec.md §2 (NEW C++ headers) and notes/a3_onnx_io_spec.md
//
// A3ObsBuilder: assembles the 1570-float `obs_dict` input for the A3
// policy. The layout (see notes/a3_onnx_io_spec.md §"1570-dim obs_dict layout"
// and notes/a3_e2e_obs_selfcheck.md §"Root Cause 1" + §"Root Cause 2") is:
//   [   0 ..  579]  command_multi_future_nonflat     (from tokenizer replay)
//   [ 580 ..  639]  motion_anchor_ori_b_mf_nonflat   (from tokenizer replay)
//   [ 640 ..  669]  base_ang_vel  flat over 10 history steps  (3  × 10 =  30)
//   [ 670 ..  959]  joint_pos     flat over 10 history steps  (29 × 10 = 290)
//   [ 960 .. 1249]  joint_vel     flat over 10 history steps  (29 × 10 = 290)
//   [1250 .. 1539]  actions       flat over 10 history steps  (29 × 10 = 290)
//   [1540 .. 1569]  gravity_dir   flat over 10 history steps  (3  × 10 =  30)
//
// Proprio is TERM-FIRST (not step-first) — matches the IsaacLab
// ObservationManager with `concatenate_terms=True` + per-term
// `flatten_history_dim=True`. Within each term's flat block, history is
// step-first C-order (oldest → newest): [step0_d0, …, step0_dN-1,
// step1_d0, …]. Internally the ring buffer stores per-step slices
// (matching the per-tick PushProprioception call shape); BuildObsDict
// transposes the view at emit time.
//
// ⚠ Term output order (base_ang_vel → joint_pos → joint_vel → actions →
// gravity_dir) is DICTATED BY the PolicyCfg class's field declaration
// order in training_envs/manager_env/mdp/observations.py:70+ (not by
// the `defaults:` list in local_dir_hist.yaml). The yaml only decides
// WHICH terms are active; the class decides their ORDER. Empirically
// verified against the training env's compute_group("policy") output.
//
// Joint data arrives in the 29-DOF MuJoCo policy view (see
// notes/a3_dof_orderings.md — waist, L_arm, R_arm, L_leg, R_leg — as
// extracted from RobotState via robot_io::ExtractPolicyView); IsaacLab
// training order is used inside the policy, so PushProprioception reorders
// joint_pos/joint_vel/actions via a3_mujoco_to_isaaclab[]. That perm is a
// 29→29 permutation whose 29-DOF MuJoCo axis is exactly this policy view.
//
// The `actions` slot holds the RAW policy output (network output BEFORE
// action_scale + default_angles are applied), matching IsaacLab's
// ActionsHistoryBuffer convention (Zach confirmed 2026-04-27).
//
// RT contract: PushProprioception, BuildObsDict, RememberAction, LastAction
// are all alloc-free + exception-free; all state lives in pre-allocated
// fixed-size arrays.
#pragma once

#include <array>
#include <cstddef>

namespace a3_deploy {

// Proprioception dimensions — match the training ObservationManager config.
// history_length=10, per-step floats = 93, total = 930.
inline constexpr std::size_t kA3ProprioHistoryLength = 10;
inline constexpr std::size_t kA3ProprioPerStepFloats = 93;
inline constexpr std::size_t kA3ProprioTotalFloats   =
    kA3ProprioHistoryLength * kA3ProprioPerStepFloats;  // 930

// Tokenizer prefix dims (order matches tokenizer_obs_names = ONNX input order).
inline constexpr std::size_t kA3TokenizerTotalFloats     = 640;
inline constexpr std::size_t kA3CommandMultiFutureFloats = 580;  // [0..579]
inline constexpr std::size_t kA3MotionAnchorOriFloats    = 60;   // [580..639]

// Total obs_dict size consumed by monolithic A3 A3-mode ONNX exports.
inline constexpr std::size_t kA3ObsDictTotalFloats =
    kA3TokenizerTotalFloats + kA3ProprioTotalFloats;  // 1570

// Total obs_dict size consumed by monolithic A3 SMPL-mode ONNX exports:
// [smpl_tokenizer_840 | proprio_history_930].
inline constexpr std::size_t kA3SmplTokenizerTotalFloats = 840;
inline constexpr std::size_t kA3SmplObsDictTotalFloats =
    kA3SmplTokenizerTotalFloats + kA3ProprioTotalFloats;  // 1770

class A3ObsBuilder {
 public:
  A3ObsBuilder() noexcept;

  // Push one new proprioception frame into the internal ring buffer.
  // Inputs are in the 29-DOF MuJoCo policy view (waist + arms + legs; see
  // notes/a3_dof_orderings.md). The builder applies the a3_mujoco_to_isaaclab[]
  // permutation internally for the joint vectors.
  //
  // Semantic notes (match training's IsaacLab obs terms):
  //   joint_pos_mujoco: RAW absolute angles. The builder subtracts
  //       a3_default_angles internally before emitting, matching training's
  //       `joint_pos_rel` obs term. See notes/a3_e2e_obs_selfcheck.md §R.C. 3.
  //   joint_vel_mujoco: RAW. Training's `joint_vel_rel` subtracts
  //       `default_joint_vel` which is all-zero for A3 (a3.py:
  //       init_state.joint_vel = {".*": 0.0}), so no subtract is needed.
  //   actions_mujoco: RAW policy output (pre-scale, pre-default-angle),
  //       as stored via RememberAction().
  //
  // Until 10 frames have been accumulated the ring is seeded with copies
  // of the latest frame, so BuildObsDict never emits zero-padded history.
  void PushProprioception(
      const std::array<double, 3>& gravity_dir,
      const std::array<double, 3>& base_ang_vel,
      const std::array<double, 29>& joint_pos_mujoco,
      const std::array<double, 29>& joint_vel_mujoco,
      const std::array<double, 29>& actions_mujoco) noexcept;

  // Build the 1570-float obs_dict for ONNX inference. `tokenizer_640` must
  // point to a 640-float slice (e.g. from A3TokenizerReplay::At(tick)). The
  // output is written to `out` oldest-first over history_length=10.
  //
  // If PushProprioception has never been called the history section is
  // filled with zeros. Normally the caller pushes the first frame before
  // the first BuildObsDict call.
  void BuildObsDict(
      const float* tokenizer_640,
      std::array<float, kA3ObsDictTotalFloats>& out) const noexcept;

  // Build the 1770-float obs_dict for monolithic SMPL ONNX inference:
  // [smpl_tokenizer_840 | proprio_history_930].
  void BuildSmplObsDict(
      const float* smpl_tokenizer_840,
      std::array<float, kA3SmplObsDictTotalFloats>& out) const noexcept;

  // Emit only the 930-float policy proprioception history block, using the
  // same TERM-FIRST layout as BuildObsDict()[640..1569].
  void BuildProprioHistory(
      std::array<float, kA3ProprioTotalFloats>& out) const noexcept;

  // Remember the raw policy output so the next tick's proprioception can
  // use it in the `actions` slot (MuJoCo-ordered copy is stored — the
  // caller passes the raw 29-float network output in IsaacLab order and we
  // convert back).
  void RememberAction(const std::array<float, 29>& raw_action_isaaclab) noexcept;

  // Returns the last remembered action in MuJoCo order, already converted.
  // Returns all zeros until the first RememberAction() call.
  std::array<double, 29> LastAction() const noexcept;

  // Number of distinct proprioception frames pushed so far (clamped to
  // history_length). For diagnostics / warmup assertions.
  std::size_t TicksBuffered() const noexcept { return ticks_buffered_; }

  // Reset internal state (ring buffer + remembered action). Useful for
  // test setup and for the policy-driver re-entry path.
  void Reset() noexcept;

 private:
  // Ring buffer of per-step slices in IsaacLab order. slot_head_ points at
  // the oldest entry; slot_head_ - 1 (mod N) holds the most recent entry.
  // History is emitted oldest-first by iterating forward from slot_head_.
  std::array<std::array<float, kA3ProprioPerStepFloats>,
             kA3ProprioHistoryLength>
      ring_{};

  // Index of the oldest slot (where the next push will overwrite once the
  // ring is full). Always in [0, kA3ProprioHistoryLength).
  std::size_t slot_head_ = 0;
  std::size_t ticks_buffered_ = 0;

  // Last raw policy output, stored in MuJoCo order (converted at
  // RememberAction-time for convenience in the RT loop).
  std::array<double, 29> last_action_mujoco_{};
};

}  // namespace a3_deploy
