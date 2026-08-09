// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_pr9_spec.md §2 (NEW C++ headers)
#include "a3_deploy/a3_obs_builder.hpp"

#include "a3_policy_parameters.hpp"

#include <cstring>

namespace a3_deploy {

namespace {

// Per-step field offsets inside the 93-float IsaacLab-order slice:
//   [0..2]   gravity_dir
//   [3..5]   base_ang_vel
//   [6..34]  joint_pos   (29, IsaacLab order)
//   [35..63] joint_vel   (29, IsaacLab order)
//   [64..92] actions     (29, IsaacLab order)
constexpr std::size_t kOfsGravityDir = 0;
constexpr std::size_t kOfsBaseAngVel = 3;
constexpr std::size_t kOfsJointPos   = 6;
constexpr std::size_t kOfsJointVel   = 35;
constexpr std::size_t kOfsActions    = 64;

// MuJoCo→IsaacLab reorder for a 29-float vector. Dest format is float32 (what
// the ONNX model consumes); source is double (sync-loop's native precision).
// Gather-index convention: out[i] = src[a3_mujoco_to_isaaclab[i]].
inline void MujocoToIsaacF32(const std::array<double, 29>& src_mujoco,
                             float* dst_isaaclab) noexcept {
  for (std::size_t i = 0; i < 29; ++i) {
    dst_isaaclab[i] = static_cast<float>(src_mujoco[a3_mujoco_to_isaaclab[i]]);
  }
}

// IsaacLab→MuJoCo reorder for a 29-float vector (float → double). Used to
// convert the raw policy output back into the MuJoCo layout we stash in
// last_action_mujoco_. Gather-index convention:
//   out_mujoco[i] = src_isaaclab[a3_isaaclab_to_mujoco[i]].
inline void IsaacToMujocoD(const std::array<float, 29>& src_isaaclab,
                           std::array<double, 29>& dst_mujoco) noexcept {
  for (std::size_t i = 0; i < 29; ++i) {
    dst_mujoco[i] = static_cast<double>(src_isaaclab[a3_isaaclab_to_mujoco[i]]);
  }
}

}  // namespace

// ---------------------------------------------------------------------------
A3ObsBuilder::A3ObsBuilder() noexcept { Reset(); }

void A3ObsBuilder::Reset() noexcept {
  for (auto& slice : ring_) slice.fill(0.0f);
  slot_head_ = 0;
  ticks_buffered_ = 0;
  last_action_mujoco_.fill(0.0);
}

// ---------------------------------------------------------------------------
void A3ObsBuilder::PushProprioception(
    const std::array<double, 3>& gravity_dir,
    const std::array<double, 3>& base_ang_vel,
    const std::array<double, 29>& joint_pos_mujoco,
    const std::array<double, 29>& joint_vel_mujoco,
    const std::array<double, 29>& actions_mujoco) noexcept {
  // Assemble the IsaacLab-order slice into a local scratch buffer first, then
  // either copy into all slots (warmup) or into slot_head_ (normal path).
  // std::array on the stack — no heap traffic.
  std::array<float, kA3ProprioPerStepFloats> slice{};

  for (std::size_t i = 0; i < 3; ++i) {
    slice[kOfsGravityDir + i] = static_cast<float>(gravity_dir[i]);
    slice[kOfsBaseAngVel + i] = static_cast<float>(base_ang_vel[i]);
  }

  // Training's obs uses `joint_pos_rel` — joint position RELATIVE to the
  // default standing pose (a3.py init_state.joint_pos). Training computes
  // this as `f32(asset.data.joint_pos) - f32(asset.data.default_joint_pos)`
  // directly in IsaacLab order (both operands are float32). To bit-match
  // that rounding we must do the subtract in f32 *in IsaacLab order*, not
  // in double in MuJoCo order (which leaves up to 1 ULP of accumulation-
  // order drift on some values). See notes/a3_e2e_obs_selfcheck.md §R.C. 5.
  //
  // joint_vel: training uses `joint_vel_rel` too, but A3's default_joint_vel
  // is all-zero (a3.py init_state.joint_vel = {".*": 0.0}), so the subtract
  // is a no-op and we feed joint_vel verbatim.
  for (std::size_t i_il = 0; i_il < 29; ++i_il) {
    const std::size_t i_mj = a3_mujoco_to_isaaclab[i_il];
    const float jp_il_f32  = static_cast<float>(joint_pos_mujoco[i_mj]);
    const float def_il_f32 = static_cast<float>(a3_default_angles[i_mj]);
    slice[kOfsJointPos + i_il] = jp_il_f32 - def_il_f32;
  }

  MujocoToIsaacF32(joint_vel_mujoco, slice.data() + kOfsJointVel);
  MujocoToIsaacF32(actions_mujoco,   slice.data() + kOfsActions);

  if (ticks_buffered_ == 0) {
    // Warmup: seed all slots with this first frame so history is never
    // zero-padded. slot_head_ stays at 0 — the next push will overwrite
    // slot 0 (which then becomes the newest) and advance the head.
    for (auto& r : ring_) r = slice;
    slot_head_ = 0;
    ticks_buffered_ = 1;
    return;
  }

  // Normal ring write: overwrite the oldest slot, advance the head.
  ring_[slot_head_] = slice;
  slot_head_ = (slot_head_ + 1) % kA3ProprioHistoryLength;
  if (ticks_buffered_ < kA3ProprioHistoryLength) ++ticks_buffered_;
}

// ---------------------------------------------------------------------------
void A3ObsBuilder::BuildObsDict(
    const float* tokenizer_640,
    std::array<float, kA3ObsDictTotalFloats>& out) const noexcept {
  // [0..639] tokenizer slice, copied verbatim (bit-exact with training
  // export — see notes/a3_onnx_io_spec.md §"tokenizer_part").
  if (tokenizer_640 != nullptr) {
    std::memcpy(out.data(), tokenizer_640,
                kA3TokenizerTotalFloats * sizeof(float));
  } else {
    std::memset(out.data(), 0, kA3TokenizerTotalFloats * sizeof(float));
  }

  std::array<float, kA3ProprioTotalFloats> proprio{};
  BuildProprioHistory(proprio);
  std::memcpy(out.data() + kA3TokenizerTotalFloats, proprio.data(),
              proprio.size() * sizeof(float));
}

void A3ObsBuilder::BuildSmplObsDict(
    const float* smpl_tokenizer_840,
    std::array<float, kA3SmplObsDictTotalFloats>& out) const noexcept {
  if (smpl_tokenizer_840 != nullptr) {
    std::memcpy(out.data(), smpl_tokenizer_840,
                kA3SmplTokenizerTotalFloats * sizeof(float));
  } else {
    std::memset(out.data(), 0, kA3SmplTokenizerTotalFloats * sizeof(float));
  }

  std::array<float, kA3ProprioTotalFloats> proprio{};
  BuildProprioHistory(proprio);
  std::memcpy(out.data() + kA3SmplTokenizerTotalFloats, proprio.data(),
              proprio.size() * sizeof(float));
}

// ---------------------------------------------------------------------------
void A3ObsBuilder::BuildProprioHistory(
    std::array<float, kA3ProprioTotalFloats>& out) const noexcept {
  // Proprioception history, TERM-FIRST layout — matches the
  // IsaacLab ObservationManager with `concatenate_terms=True` + per-term
  // `flatten_history_dim=True`. That combo emits:
  //   [ base_ang_vel_flat(30) | joint_pos_flat(290) | joint_vel_flat(290)
  //   | actions_flat(290)     | gravity_dir_flat(30) ]
  //   (total 930)
  // where each `_flat(history_length × dim)` is step-first C-order (oldest→
  // newest).
  //
  // ⚠ Term order is DICTATED BY THE PolicyCfg CLASS's FIELD DECLARATIONS in
  // training_envs/manager_env/mdp/observations.py:70+, NOT by the
  // local_dir_hist.yaml `defaults:` list. The class places
  //   base_ang_vel (line 107) < joint_pos < joint_vel < actions (line 110)
  //   < gravity_dir (line 128)
  // and IsaacLab's ObservationManager iterates `group_cfg.__dict__.items()`
  // in declaration order. The yaml only decides WHICH terms are active,
  // not their output ordering. See notes/a3_e2e_obs_selfcheck.md §Root
  // Cause 2 for the investigation that surfaced this.
  //
  // Internally our ring stores per-step slices; this function transposes
  // the view at emit time.
  float* hist = out.data();
  struct TermLayout {
    std::size_t slice_offset;  // within the 93-float per-step slice
    std::size_t dim;
  };
  constexpr std::array<TermLayout, 5> kTerms = {{
      {kOfsBaseAngVel, 3},    // [0..29]
      {kOfsJointPos,   29},   // [30..319]
      {kOfsJointVel,   29},   // [320..609]
      {kOfsActions,    29},   // [610..899]
      {kOfsGravityDir, 3},    // [900..929]
  }};

  std::size_t out_cursor = 0;
  for (const auto& term : kTerms) {
    for (std::size_t step = 0; step < kA3ProprioHistoryLength; ++step) {
      const std::size_t slot =
          (slot_head_ + step) % kA3ProprioHistoryLength;
      const float* src = ring_[slot].data() + term.slice_offset;
      std::memcpy(hist + out_cursor, src, term.dim * sizeof(float));
      out_cursor += term.dim;
    }
  }
  // Sanity: out_cursor should exactly equal kA3ProprioTotalFloats (930).
}

// ---------------------------------------------------------------------------
void A3ObsBuilder::RememberAction(
    const std::array<float, 29>& raw_action_isaaclab) noexcept {
  IsaacToMujocoD(raw_action_isaaclab, last_action_mujoco_);
}

std::array<double, 29> A3ObsBuilder::LastAction() const noexcept {
  return last_action_mujoco_;
}

}  // namespace a3_deploy
