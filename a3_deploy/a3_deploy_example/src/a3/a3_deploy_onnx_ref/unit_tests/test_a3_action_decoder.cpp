// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_pr9_spec.md §5 (Tests — test_a3_action_decoder.cpp)
//
// Verifies DecodeAction against the reference formula element-wise and
// separately checks that the IsaacLab→MuJoCo permutation is applied. Uses
// permutation-distinctive inputs so the test distinguishes a permutation
// bug from a scale/default bug.
#include <gtest/gtest.h>

#include "a3_deploy/a3_action_decoder.hpp"
#include "a3_policy_parameters.hpp"

#include <algorithm>
#include <array>
#include <cmath>

using a3_deploy::DecodeAction;

// The canonical reference formula, spelled out verbatim from
// notes/a3_pr9_spec.md §2. Tests assert DecodeAction matches this exactly.
static std::array<double, 29> Reference(
    const std::array<float, 29>& raw_isaaclab) {
  // Step 1: scale + default in IsaacLab math, but using MuJoCo-indexed
  // constants (matching storage convention — see a3_action_decoder.cpp).
  std::array<double, 29> out{};
  for (std::size_t i = 0; i < 29; ++i) {
    const int src_isaac = a3_isaaclab_to_mujoco[i];
    const double clipped = std::clamp(
        static_cast<double>(raw_isaaclab[src_isaac]), -20.0, 20.0);
    out[i] = clipped * a3_action_scale[i] +
             a3_default_angles[i];
  }
  return out;
}

TEST(A3ActionDecoder, ZeroActionReturnsDefaultAngles) {
  std::array<float, 29> raw{};
  raw.fill(0.0f);
  std::array<double, 29> q_des{};
  DecodeAction(raw, q_des);
  for (std::size_t i = 0; i < 29; ++i) {
    EXPECT_DOUBLE_EQ(q_des[i], a3_default_angles[i])
        << "i=" << i << " (MuJoCo order)";
  }
}

TEST(A3ActionDecoder, UnitActionScaled) {
  // raw = 1 in every IsaacLab slot.
  std::array<float, 29> raw{};
  raw.fill(1.0f);
  std::array<double, 29> q_des{};
  DecodeAction(raw, q_des);
  // Expected: q_des[i] = 1.0 * a3_action_scale[i] + a3_default_angles[i].
  for (std::size_t i = 0; i < 29; ++i) {
    EXPECT_DOUBLE_EQ(q_des[i], a3_action_scale[i] + a3_default_angles[i])
        << "i=" << i;
  }
}

TEST(A3ActionDecoder, NegativeAndPositiveActionsDistinguish) {
  // raw_isaac[0] = +1, raw_isaac[28] = -1, rest = 0. This exercises the
  // permutation so that a wrong `isaaclab_to_mujoco` lookup would surface.
  std::array<float, 29> raw{};
  raw.fill(0.0f);
  raw[0]  =  1.0f;
  raw[28] = -1.0f;

  std::array<double, 29> q_des{};
  DecodeAction(raw, q_des);

  // Expected non-zero contributions only at the MuJoCo indices whose
  // a3_isaaclab_to_mujoco value is 0 or 28.
  for (std::size_t i = 0; i < 29; ++i) {
    const int src = a3_isaaclab_to_mujoco[i];
    double expected = a3_default_angles[i];
    if (src == 0)  expected +=  1.0 * a3_action_scale[i];
    if (src == 28) expected += -1.0 * a3_action_scale[i];
    EXPECT_DOUBLE_EQ(q_des[i], expected)
        << "MuJoCo i=" << i << " src_isaac=" << src;
  }
}

TEST(A3ActionDecoder, MatchesReferenceForDenseInput) {
  // Dense, linear raw actions in IsaacLab order. Checks end-to-end formula.
  std::array<float, 29> raw{};
  for (int i = 0; i < 29; ++i) raw[i] = -1.0f + 0.1f * static_cast<float>(i);

  std::array<double, 29> q_des{};
  DecodeAction(raw, q_des);

  const auto expected = Reference(raw);
  for (std::size_t i = 0; i < 29; ++i) {
    EXPECT_DOUBLE_EQ(q_des[i], expected[i]) << "i=" << i;
  }
}

TEST(A3ActionDecoder, MaxActionBoundary) {
  // raw = {1, -1, 1, -1, ...}. Spot checks the pattern matches the reference.
  std::array<float, 29> raw{};
  for (int i = 0; i < 29; ++i) raw[i] = (i % 2 == 0) ? 1.0f : -1.0f;

  std::array<double, 29> q_des{};
  DecodeAction(raw, q_des);
  const auto expected = Reference(raw);
  for (std::size_t i = 0; i < 29; ++i) {
    EXPECT_DOUBLE_EQ(q_des[i], expected[i]) << "i=" << i;
  }
}

TEST(A3ActionDecoder, ClipsRawActionAtTwenty) {
  std::array<float, 29> raw{};
  raw.fill(0.0f);
  raw[0] = 25.0f;
  raw[28] = -25.0f;

  std::array<double, 29> q_des{};
  DecodeAction(raw, q_des);

  for (std::size_t i = 0; i < 29; ++i) {
    const int src = a3_isaaclab_to_mujoco[i];
    double expected = a3_default_angles[i];
    if (src == 0) expected += 20.0 * a3_action_scale[i];
    if (src == 28) expected -= 20.0 * a3_action_scale[i];
    EXPECT_DOUBLE_EQ(q_des[i], expected)
        << "MuJoCo i=" << i << " src_isaac=" << src;
  }
}

TEST(A3ActionDecoder, PermutationAppliedNotIdentity) {
  // For MuJoCo indices i where a3_isaaclab_to_mujoco[i] != i, a decoder
  // that omits the permutation would produce different output than the
  // reference. The raw input here is specifically constructed to make
  // "identity" and "permute" disagree at every permuted slot.
  std::array<float, 29> raw{};
  for (int i = 0; i < 29; ++i) {
    raw[i] = static_cast<float>(-0.95 + 0.06 * i);
  }

  std::array<double, 29> q_des{};
  DecodeAction(raw, q_des);

  // Identity-permutation reference (WRONG). Any MuJoCo index whose
  // a3_isaaclab_to_mujoco entry != i should differ from q_des.
  for (std::size_t i = 0; i < 29; ++i) {
    const int src_isaac = a3_isaaclab_to_mujoco[i];
    if (static_cast<int>(i) == src_isaac) continue;  // fixed points carry no info

    const double wrong =
        static_cast<double>(raw[i]) * a3_action_scale[i] +
        a3_default_angles[i];
    EXPECT_NE(q_des[i], wrong)
        << "MuJoCo i=" << i << " src_isaac=" << src_isaac
        << " — decoder appears to skip the permutation";
  }
}

// =============================================================================
// PR 9 open item #1 — Formulation-equivalence proof
//
// Spec `notes/a3_pr9_spec.md §2` writes the decoder as:
//     q_des_isaac[i] = raw[i] * scale_isaac[i] + default_isaac[i]
//     then permute with a3_isaaclab_to_mujoco[] → q_des_mujoco
// assuming scale/default are in IsaacLab order.
//
// Our implementation stores scale/default in MuJoCo order (per
// a3_policy_parameters.hpp) and folds the permutation into the raw-action
// gather:
//     q_des_mujoco[i] = raw[a3_isaaclab_to_mujoco[i]] * scale_mujoco[i]
//                     + default_mujoco[i]
//
// This test proves the two formulations produce bit-identical results.
// If they ever diverge (e.g. someone edits the perm or the constant arrays
// drift between conventions), this test will catch it.
// =============================================================================

TEST(A3ActionDecoder, FormulationA_IsaaclabOrderMatchesImplementation) {
  // Step 1: deterministic input.
  std::array<float, 29> raw_isaaclab{};
  for (int i = 0; i < 29; ++i) {
    raw_isaaclab[i] = static_cast<float>(i) * 0.01f - 0.14f;
  }

  // Step 2a: derive IsaacLab-ordered scale / default from the MuJoCo-ordered
  // authoritative constants by applying a3_mujoco_to_isaaclab[]:
  //   x_isaaclab[i] = x_mujoco[a3_mujoco_to_isaaclab[i]]
  // (This is the same gather convention the obs_builder uses for joint_pos /
  // joint_vel — see test_a3_obs_builder.cpp.)
  std::array<double, 29> scale_isaac{};
  std::array<double, 29> default_isaac{};
  for (int i = 0; i < 29; ++i) {
    const int src_mj = a3_mujoco_to_isaaclab[i];
    scale_isaac[i]   = a3_action_scale[src_mj];
    default_isaac[i] = a3_default_angles[src_mj];
  }

  // Step 2b: compute q_des in IsaacLab order per the spec formula …
  std::array<double, 29> tmp_isaaclab{};
  for (int i = 0; i < 29; ++i) {
    tmp_isaaclab[i] = static_cast<double>(raw_isaaclab[i]) * scale_isaac[i] +
                      default_isaac[i];
  }
  // … then reorder to MuJoCo via a3_isaaclab_to_mujoco[]:
  //   q_mujoco[i] = q_isaaclab[a3_isaaclab_to_mujoco[i]]
  std::array<double, 29> q_des_mujoco_A{};
  for (int i = 0; i < 29; ++i) {
    q_des_mujoco_A[i] = tmp_isaaclab[a3_isaaclab_to_mujoco[i]];
  }

  // Step 3: the production implementation.
  std::array<double, 29> q_des_mujoco_B{};
  DecodeAction(raw_isaaclab, q_des_mujoco_B);

  // Step 4: bit-identical (floating-point math is identical up to operand
  // order — since A multiplies raw_isaaclab[i] by scale_isaac[i] where
  // scale_isaac[i] == scale_mujoco[a3_mujoco_to_isaaclab[i]], and B
  // multiplies raw_isaaclab[a3_isaaclab_to_mujoco[k]] by scale_mujoco[k],
  // renaming k := a3_isaaclab_to_mujoco[i] makes the two products identical
  // at the bit level). Use EXPECT_DOUBLE_EQ for the implicit
  // within-4-ulps check and a tight 1e-12 abs tolerance as a secondary
  // assertion for readability).
  for (int i = 0; i < 29; ++i) {
    EXPECT_DOUBLE_EQ(q_des_mujoco_A[i], q_des_mujoco_B[i]) << "i=" << i;
    EXPECT_NEAR(q_des_mujoco_A[i], q_des_mujoco_B[i], 1e-12) << "i=" << i;
  }
}
