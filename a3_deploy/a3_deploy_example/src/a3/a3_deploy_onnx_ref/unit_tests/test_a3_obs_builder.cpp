// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_pr9_spec.md §5 (Tests — test_a3_obs_builder.cpp)
//
// Verifies A3ObsBuilder's obs_dict layout (tokenizer prefix + proprioception
// history), the MuJoCo→IsaacLab reorder, warmup duplication, and the
// RememberAction / LastAction round-trip.
//
// Post 2026-04-28 fix (R.C. 1 of Level-2): the 930-float proprio half is
// TERM-FIRST (gravity×10 | ang_vel×10 | jp×10 | jv×10 | act×10) to match
// the IsaacLab ObservationManager's `concatenate_terms=True` +
// `flatten_history_dim=True` layout. Within each term's block, history is
// step-first C-order (oldest→newest).
#include <gtest/gtest.h>

#include "a3_deploy/a3_obs_builder.hpp"
#include "a3_policy_parameters.hpp"

#include <array>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

using a3_deploy::A3ObsBuilder;
using a3_deploy::kA3CommandMultiFutureFloats;
using a3_deploy::kA3MotionAnchorOriFloats;
using a3_deploy::kA3ObsDictTotalFloats;
using a3_deploy::kA3ProprioHistoryLength;
using a3_deploy::kA3ProprioPerStepFloats;
using a3_deploy::kA3ProprioTotalFloats;
using a3_deploy::kA3SmplObsDictTotalFloats;
using a3_deploy::kA3SmplTokenizerTotalFloats;
using a3_deploy::kA3TokenizerTotalFloats;

namespace {

// Term-first layout offsets inside the 930-float proprio half.
// Matches a3_obs_builder.cpp's BuildObsDict emit order:
//   base_ang_vel(30) | joint_pos(290) | joint_vel(290) | actions(290) | gravity_dir(30)
// Order dictated by PolicyCfg class's field declaration order in
// training_envs/manager_env/mdp/observations.py — see the header
// comment on a3_obs_builder.hpp.
constexpr std::size_t kGravHistDim   = 3  * kA3ProprioHistoryLength;   //  30
constexpr std::size_t kAngVelHistDim = 3  * kA3ProprioHistoryLength;   //  30
constexpr std::size_t kJointHistDim  = 29 * kA3ProprioHistoryLength;   // 290
constexpr std::size_t kActHistDim    = 29 * kA3ProprioHistoryLength;   // 290

constexpr std::size_t kTermOfsAngVel   = 0;
constexpr std::size_t kTermOfsJointPos = kTermOfsAngVel   + kAngVelHistDim;  //  30
constexpr std::size_t kTermOfsJointVel = kTermOfsJointPos + kJointHistDim;   // 320
constexpr std::size_t kTermOfsActions  = kTermOfsJointVel + kJointHistDim;   // 610
constexpr std::size_t kTermOfsGravity  = kTermOfsActions  + kActHistDim;     // 900

std::array<float, kA3TokenizerTotalFloats> MakeTokenizerSlice(float base) {
  std::array<float, kA3TokenizerTotalFloats> t{};
  for (std::size_t i = 0; i < kA3TokenizerTotalFloats; ++i) {
    t[i] = base + static_cast<float>(i);
  }
  return t;
}

std::array<float, kA3SmplTokenizerTotalFloats> MakeSmplTokenizerSlice(
    float base) {
  std::array<float, kA3SmplTokenizerTotalFloats> t{};
  for (std::size_t i = 0; i < kA3SmplTokenizerTotalFloats; ++i) {
    t[i] = base + static_cast<float>(i);
  }
  return t;
}

struct ProprioFrame {
  std::array<double, 3>  gravity;
  std::array<double, 3>  base_ang_vel;
  std::array<double, 29> jp_mujoco;
  std::array<double, 29> jv_mujoco;
  std::array<double, 29> act_mujoco;
};

ProprioFrame MakeFrame(double tag) {
  ProprioFrame f{};
  f.gravity      = {tag + 0.1, tag + 0.2, tag + 0.3};
  f.base_ang_vel = {tag + 0.4, tag + 0.5, tag + 0.6};
  for (int i = 0; i < 29; ++i) {
    f.jp_mujoco[i]  = tag * 1000.0 + 1.0  * i;
    f.jv_mujoco[i]  = tag * 1000.0 + 100. * i;
    f.act_mujoco[i] = tag * 1000.0 + 200. * i;
  }
  return f;
}

void PushFrame(A3ObsBuilder& b, const ProprioFrame& f) {
  b.PushProprioception(f.gravity, f.base_ang_vel,
                       f.jp_mujoco, f.jv_mujoco, f.act_mujoco);
}

// Assert the 1570-float `out` matches the expected term-first layout for
// the given 10-frame history (oldest-first). Accounts for the
// joint_pos_rel subtract applied internally by PushProprioception.
void ExpectObsMatchesHistory(const std::array<float, kA3ObsDictTotalFloats>& out,
                             const std::array<ProprioFrame,
                                              kA3ProprioHistoryLength>& hist) {
  const float* proprio = out.data() + kA3TokenizerTotalFloats;
  // Gravity: [step_0.g(3) | step_1.g(3) | ... | step_9.g(3)]
  for (std::size_t s = 0; s < kA3ProprioHistoryLength; ++s) {
    for (std::size_t d = 0; d < 3; ++d) {
      EXPECT_FLOAT_EQ(proprio[kTermOfsGravity + s * 3 + d],
                      static_cast<float>(hist[s].gravity[d]))
          << "gravity step " << s << " d " << d;
      EXPECT_FLOAT_EQ(proprio[kTermOfsAngVel + s * 3 + d],
                      static_cast<float>(hist[s].base_ang_vel[d]))
          << "ang_vel step " << s << " d " << d;
    }
    // Joint_pos / joint_vel / actions: 29 floats per step, IsaacLab-ordered
    // via a3_mujoco_to_isaaclab[]. joint_pos has `- a3_default_angles`
    // applied inside PushProprioception (R.C. 3 fix).
    for (int i = 0; i < 29; ++i) {
      const int src = a3_mujoco_to_isaaclab[i];
      EXPECT_FLOAT_EQ(proprio[kTermOfsJointPos + s * 29 + i],
                      static_cast<float>(hist[s].jp_mujoco[src] -
                                         a3_default_angles[src]))
          << "joint_pos step " << s << " isaac i " << i;
      EXPECT_FLOAT_EQ(proprio[kTermOfsJointVel + s * 29 + i],
                      static_cast<float>(hist[s].jv_mujoco[src]))
          << "joint_vel step " << s << " isaac i " << i;
      EXPECT_FLOAT_EQ(proprio[kTermOfsActions + s * 29 + i],
                      static_cast<float>(hist[s].act_mujoco[src]))
          << "actions step " << s << " isaac i " << i;
    }
  }
}

}  // namespace

// =============================================================================
// Obs dict layout
// =============================================================================

TEST(A3ObsBuilder, ObsDictTotalFloatsIs1570) {
  EXPECT_EQ(kA3ObsDictTotalFloats, 1570u);
  EXPECT_EQ(kA3SmplTokenizerTotalFloats, 840u);
  EXPECT_EQ(kA3SmplObsDictTotalFloats, 1770u);
  EXPECT_EQ(kA3TokenizerTotalFloats,
            kA3MotionAnchorOriFloats + kA3CommandMultiFutureFloats);
  EXPECT_EQ(kA3ProprioPerStepFloats, 93u);
  EXPECT_EQ(kA3ProprioTotalFloats, 930u);
  // Term-first offsets sum to 930 (gravity_dir is the LAST term).
  EXPECT_EQ(kTermOfsGravity + kGravHistDim, kA3ProprioTotalFloats);
}

TEST(A3ObsBuilder, TokenizerPrefixIsCopiedVerbatim) {
  A3ObsBuilder b;
  PushFrame(b, MakeFrame(1.0));

  auto tok = MakeTokenizerSlice(10000.0f);
  std::array<float, kA3ObsDictTotalFloats> out{};
  b.BuildObsDict(tok.data(), out);

  for (std::size_t i = 0; i < kA3MotionAnchorOriFloats; ++i) {
    EXPECT_FLOAT_EQ(out[i], tok[i]) << "anchor i=" << i;
  }
  for (std::size_t i = kA3MotionAnchorOriFloats;
       i < kA3TokenizerTotalFloats; ++i) {
    EXPECT_FLOAT_EQ(out[i], tok[i]) << "cmd i=" << i;
  }
}

TEST(A3ObsBuilder, NullTokenizerZerosPrefix) {
  A3ObsBuilder b;
  PushFrame(b, MakeFrame(1.0));
  std::array<float, kA3ObsDictTotalFloats> out{};
  out.fill(42.0f);
  b.BuildObsDict(nullptr, out);
  for (std::size_t i = 0; i < kA3TokenizerTotalFloats; ++i) {
    EXPECT_FLOAT_EQ(out[i], 0.0f);
  }
}

TEST(A3ObsBuilder, SmplObsDictPacksSmplTokenizerAndProprio) {
  A3ObsBuilder b;
  PushFrame(b, MakeFrame(1.0));

  auto smpl = MakeSmplTokenizerSlice(20000.0f);
  std::array<float, kA3SmplObsDictTotalFloats> out{};
  b.BuildSmplObsDict(smpl.data(), out);

  for (std::size_t i = 0; i < kA3SmplTokenizerTotalFloats; ++i) {
    EXPECT_FLOAT_EQ(out[i], smpl[i]) << "smpl token i=" << i;
  }

  std::array<float, kA3ObsDictTotalFloats> a3_out{};
  b.BuildObsDict(nullptr, a3_out);
  for (std::size_t i = 0; i < kA3ProprioTotalFloats; ++i) {
    EXPECT_FLOAT_EQ(out[kA3SmplTokenizerTotalFloats + i],
                    a3_out[kA3TokenizerTotalFloats + i])
        << "proprio i=" << i;
  }
}

// =============================================================================
// Term-first layout — regression guard against the step-first bug fixed
// 2026-04-28 (R.C. 1 of Level-2; see notes/a3_e2e_obs_selfcheck.md).
// =============================================================================

TEST(A3ObsBuilder, ProprioLayoutIsTermFirstGravityContiguous) {
  A3ObsBuilder b;
  // Push 10 frames with gravity components that encode (tag, dim). Then
  // the gravity term block should be a strict sequence of [step, dim]
  // pairs, and NO joint data should appear inside the gravity block.
  std::array<ProprioFrame, kA3ProprioHistoryLength> hist{};
  for (int t = 0; t < static_cast<int>(kA3ProprioHistoryLength); ++t) {
    hist[t] = MakeFrame(static_cast<double>(t + 1));
    // Flag joint_pos with negative values — if the gravity block
    // accidentally pulls from joint_pos (step-first bug), the test fails.
    for (int i = 0; i < 29; ++i) hist[t].jp_mujoco[i] = -1000.0 - 10.0 * t - i;
    PushFrame(b, hist[t]);
  }

  auto tok = MakeTokenizerSlice(0.0f);
  std::array<float, kA3ObsDictTotalFloats> out{};
  b.BuildObsDict(tok.data(), out);

  const float* proprio = out.data() + kA3TokenizerTotalFloats;
  // Gravity block: [0..29] of proprio.
  for (std::size_t s = 0; s < kA3ProprioHistoryLength; ++s) {
    for (std::size_t d = 0; d < 3; ++d) {
      const float val = proprio[kTermOfsGravity + s * 3 + d];
      EXPECT_GT(val, -500.0f)
          << "gravity slot contains negative joint_pos flag — "
          << "step-first layout bug regressed at step " << s << " d " << d;
      EXPECT_FLOAT_EQ(val, static_cast<float>(hist[s].gravity[d]));
    }
  }
}

// =============================================================================
// Warmup duplication
// =============================================================================

TEST(A3ObsBuilder, WarmupDuplicatesFirstFrameAcrossHistory) {
  A3ObsBuilder b;
  auto f1 = MakeFrame(1.0);
  PushFrame(b, f1);

  EXPECT_EQ(b.TicksBuffered(), 1u);

  auto tok = MakeTokenizerSlice(0.0f);
  std::array<float, kA3ObsDictTotalFloats> out{};
  b.BuildObsDict(tok.data(), out);

  // All 10 history slots should equal f1.
  std::array<ProprioFrame, kA3ProprioHistoryLength> hist{};
  for (auto& h : hist) h = f1;
  ExpectObsMatchesHistory(out, hist);
}

TEST(A3ObsBuilder, SecondFramePushesOneEntryKeepsFirstAsOldest) {
  A3ObsBuilder b;
  auto f1 = MakeFrame(1.0);
  auto f2 = MakeFrame(2.0);
  PushFrame(b, f1);
  PushFrame(b, f2);

  EXPECT_EQ(b.TicksBuffered(), 2u);

  auto tok = MakeTokenizerSlice(0.0f);
  std::array<float, kA3ObsDictTotalFloats> out{};
  b.BuildObsDict(tok.data(), out);

  // Oldest→newest: 9 × f1, 1 × f2 (warmup seeded f1 everywhere, then f2
  // push overwrote the newest slot).
  std::array<ProprioFrame, kA3ProprioHistoryLength> hist{};
  for (std::size_t i = 0; i < kA3ProprioHistoryLength - 1; ++i) hist[i] = f1;
  hist[kA3ProprioHistoryLength - 1] = f2;
  ExpectObsMatchesHistory(out, hist);
}

TEST(A3ObsBuilder, ElevenFramesYieldsMostRecentTenOldestFirst) {
  A3ObsBuilder b;
  std::vector<ProprioFrame> frames;
  for (int t = 1; t <= 11; ++t) {
    auto f = MakeFrame(static_cast<double>(t));
    frames.push_back(f);
    PushFrame(b, f);
  }
  EXPECT_EQ(b.TicksBuffered(), kA3ProprioHistoryLength);

  auto tok = MakeTokenizerSlice(0.0f);
  std::array<float, kA3ObsDictTotalFloats> out{};
  b.BuildObsDict(tok.data(), out);

  // After 11 pushes, oldest is f2 and newest is f11.
  std::array<ProprioFrame, kA3ProprioHistoryLength> hist{};
  for (std::size_t i = 0; i < kA3ProprioHistoryLength; ++i) {
    hist[i] = frames[1 + i];  // frames[1] == f2 ... frames[10] == f11
  }
  ExpectObsMatchesHistory(out, hist);
}

// =============================================================================
// Reorder distinctiveness — permutation-aware assertion
// =============================================================================

TEST(A3ObsBuilder, MujocoToIsaacLabReorderAppliedToJointFields) {
  A3ObsBuilder b;

  // jp_mujoco[i] == i → IsaacLab slot i should hold
  //   a3_mujoco_to_isaaclab[i] - a3_default_angles[a3_mujoco_to_isaaclab[i]]
  // (the `- default_angles` is the R.C. 3 fix's joint_pos_rel subtract).
  ProprioFrame f{};
  f.gravity = {0, 0, -1};
  f.base_ang_vel = {0, 0, 0};
  for (int i = 0; i < 29; ++i) {
    f.jp_mujoco[i]  = static_cast<double>(i);
    f.jv_mujoco[i]  = static_cast<double>(i) + 0.5;
    f.act_mujoco[i] = static_cast<double>(i) + 0.25;
  }
  PushFrame(b, f);

  auto tok = MakeTokenizerSlice(0.0f);
  std::array<float, kA3ObsDictTotalFloats> out{};
  b.BuildObsDict(tok.data(), out);

  const float* proprio = out.data() + kA3TokenizerTotalFloats;
  // Newest step = kA3ProprioHistoryLength - 1 = 9.
  constexpr std::size_t newest = kA3ProprioHistoryLength - 1;

  for (int i = 0; i < 29; ++i) {
    const int expected_src = a3_mujoco_to_isaaclab[i];
    EXPECT_FLOAT_EQ(proprio[kTermOfsJointPos + newest * 29 + i],
                    static_cast<float>(expected_src -
                                       a3_default_angles[expected_src]));
    EXPECT_FLOAT_EQ(proprio[kTermOfsJointVel + newest * 29 + i],
                    static_cast<float>(expected_src) + 0.5f);
    EXPECT_FLOAT_EQ(proprio[kTermOfsActions + newest * 29 + i],
                    static_cast<float>(expected_src) + 0.25f);
  }
}

// =============================================================================
// RememberAction / LastAction
// =============================================================================

TEST(A3ObsBuilder, LastActionIsZeroBeforeRemember) {
  A3ObsBuilder b;
  auto last = b.LastAction();
  for (int i = 0; i < 29; ++i) EXPECT_DOUBLE_EQ(last[i], 0.0);
}

TEST(A3ObsBuilder, RememberRoundTripsThroughIsaacLabPermutation) {
  A3ObsBuilder b;
  std::array<float, 29> raw_isaaclab{};
  for (int i = 0; i < 29; ++i) raw_isaaclab[i] = static_cast<float>(i) * 0.1f;

  b.RememberAction(raw_isaaclab);
  auto last_mujoco = b.LastAction();

  for (int i = 0; i < 29; ++i) {
    const int src = a3_isaaclab_to_mujoco[i];
    EXPECT_DOUBLE_EQ(last_mujoco[i], static_cast<double>(raw_isaaclab[src]))
        << "mujoco idx " << i << " should pull from isaaclab idx " << src;
  }

  // Feeding last_mujoco back into PushProprioception round-trips to raw.
  ProprioFrame f{};
  f.jp_mujoco  = {};
  f.jv_mujoco  = {};
  f.act_mujoco = last_mujoco;
  PushFrame(b, f);

  auto tok = MakeTokenizerSlice(0.0f);
  std::array<float, kA3ObsDictTotalFloats> out{};
  b.BuildObsDict(tok.data(), out);

  // Newest action slot should equal raw_isaaclab in IsaacLab order.
  const float* proprio = out.data() + kA3TokenizerTotalFloats;
  constexpr std::size_t newest = kA3ProprioHistoryLength - 1;
  for (int i = 0; i < 29; ++i) {
    EXPECT_FLOAT_EQ(proprio[kTermOfsActions + newest * 29 + i], raw_isaaclab[i])
        << "actions i=" << i;
  }
}

// =============================================================================
// Reset
// =============================================================================

TEST(A3ObsBuilder, ResetClearsHistoryAndLastAction) {
  A3ObsBuilder b;
  PushFrame(b, MakeFrame(1.0));
  std::array<float, 29> raw{};
  raw.fill(0.5f);
  b.RememberAction(raw);

  b.Reset();
  EXPECT_EQ(b.TicksBuffered(), 0u);

  for (int i = 0; i < 29; ++i) EXPECT_DOUBLE_EQ(b.LastAction()[i], 0.0);

  auto tok = MakeTokenizerSlice(0.0f);
  std::array<float, kA3ObsDictTotalFloats> out{};
  out.fill(7.0f);
  b.BuildObsDict(tok.data(), out);
  // History section should be zero after reset.
  for (std::size_t i = kA3TokenizerTotalFloats;
       i < kA3ObsDictTotalFloats; ++i) {
    EXPECT_FLOAT_EQ(out[i], 0.0f) << "i=" << i;
  }
}
