#pragma once

#include <string>
#include <string_view>

namespace a3_pingpong {

// Jeremy Spin001 handoff is bound to the exact model_3440 export selected for this
// hardware workflow.  The policy SHA binds the complete ONNX byte stream,
// including its embedded checkpoint/recipe/runtime metadata; deploy.yaml is
// separately required by PpOnnxPolicy and must name the same policy SHA.
inline constexpr std::string_view kServeModel3440CheckpointSha256 =
    "abf2250d1192be409254c8da2e7e7795d0a06d8de5627399e8615c6ad4791354";
inline constexpr std::string_view kServeModel3440PolicySha256 =
    "fe34787942956eb822f28ead7682f7933a588383aea86bdc173333286118ed4d";
inline constexpr std::string_view kServeModel3440DeploySha256 =
    "f6d6c9f966bff4154cc48fa161308bdae2dc3ed024332ad618ae98df2a28a9c1";

struct ServePolicyIdentity {
  bool uses_deploy_config{false};
  int observation_dim{0};
  std::string_view runtime_contract;
  std::string_view training_recipe;
  std::string_view training_recipe_version;
  std::string_view policy_sha256;
};

// Empty means the exact model_3440 identity passed.  A stable short reason is
// returned for logs/tests; callers must fail closed on every non-empty value.
inline std::string ValidateServeModel3440Policy(
    const ServePolicyIdentity& identity) {
  if (!identity.uses_deploy_config) return "deploy_config_required";
  if (identity.observation_dim != 110) return "observation_dim_must_be_110";
  if (identity.runtime_contract != "rally_final_v2") {
    return "runtime_contract_must_be_rally_final_v2";
  }
  if (identity.training_recipe != "hitter_pingpong_build2_rapid_preempt_v1") {
    return "training_recipe_must_be_build2_rapid_preempt_v1";
  }
  if (identity.training_recipe_version != "1") {
    return "training_recipe_version_must_be_1";
  }
  if (identity.policy_sha256 != kServeModel3440PolicySha256) {
    return "policy_sha256_mismatch";
  }
  return {};
}

}  // namespace a3_pingpong
