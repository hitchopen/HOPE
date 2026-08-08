// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_pr9_spec.md §2 (a3_action_decoder)
#include "a3_deploy/a3_action_decoder.hpp"

#include <algorithm>  // std::clamp

#include "a3_policy_parameters.hpp"

namespace a3_deploy {
namespace {

constexpr double kA3RawActionClip = 20.0;

}  // namespace

void DecodeAction(const std::array<float, 29>& raw_action_isaaclab,
                  std::array<double, 29>&     q_des_mujoco) noexcept {
  // Math (matching current A3 sim2sim deployment):
  //   clipped = clip(raw, -20, 20)
  //   q_des_isaac[i] = clipped[i] * scale[i] + default[i]
  //   permute to MuJoCo order.
  //
  // The clip prevents catastrophic divergence in deployment: if the policy
  // enters an out-of-distribution state, raw actions can explode (>50) and
  // lock joints at hardware limits. Training groundtruth shows raw actions
  // around single digits; current sim2sim keeps a wider ±20 deployment bound.
  for (std::size_t i = 0; i < 29; ++i) {
    const int src_isaac = a3_isaaclab_to_mujoco[i];
    const double clipped =
        std::clamp(static_cast<double>(raw_action_isaaclab[src_isaac]),
                   -kA3RawActionClip, kA3RawActionClip);
    q_des_mujoco[i] = clipped * a3_action_scale[i] + a3_default_angles[i];
  }
}

}  // namespace a3_deploy
