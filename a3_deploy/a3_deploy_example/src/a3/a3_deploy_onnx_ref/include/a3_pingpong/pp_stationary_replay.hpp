#pragma once

#include <cmath>
#include <string>
#include <string_view>

namespace a3_pingpong {

// Production loading remains strict.  This alternate profile exists only for
// the x86-only MuJoCo replay executable; it must never be selected by the
// production ping-pong binary.
enum class PpOnnxLoadProfile {
  kProductionStrict,
  kV17R1StationaryMujocoReplay,
  // Read-only host probe for a V17-r6 P0 contract candidate.  The production
  // ping-pong executable never selects this profile.
  kV17R6P0ContractAudit,
  // x86 MuJoCo/Gate3 execution of the currently-trained fixed-station R10
  // contract.  hardware_authorized=false remains fatal in production and on
  // the aarch64 field binary.
  kV17R10P0Gate3,
};

inline bool IsLowercaseSha256(std::string_view value) {
  if (value.size() != 64) return false;
  for (const char ch : value) {
    if (!((ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f')))
      return false;
  }
  return true;
}

struct V17R3QualificationMetadata {
  std::string_view training_recipe;
  std::string_view recipe_version;
  std::string_view deployment_status;
  std::string_view validator_profile;
  std::string_view qualification_status;
  std::string_view validator_receipt_sha256;
  std::string_view checkpoint_sha256;
  std::string_view resolved_task_sha256;
};

inline std::string ValidateV17R3QualificationMetadata(
    const V17R3QualificationMetadata& metadata) {
  auto require = [](std::string_view actual, std::string_view expected,
                    std::string_view label) -> std::string {
    if (actual == expected) return {};
    return "V17-r3 qualification metadata mismatch: " + std::string(label);
  };
  if (auto error = require(
          metadata.training_recipe, "rally_v17", "training_recipe");
      !error.empty())
    return error;
  if (auto error = require(metadata.recipe_version, "3", "recipe_version");
      !error.empty())
    return error;
  if (auto error = require(
          metadata.deployment_status, "gate3_candidate", "deployment_status");
      !error.empty())
    return error;
  if (auto error = require(
          metadata.validator_profile, "v17_r3_strong_stability_v1",
          "validator_profile");
      !error.empty())
    return error;
  if (auto error = require(
          metadata.qualification_status, "strong_stability_candidate",
          "qualification_status");
      !error.empty())
    return error;
  if (!IsLowercaseSha256(metadata.validator_receipt_sha256))
    return "V17-r3 qualification metadata mismatch: validator_receipt_sha256";
  if (!IsLowercaseSha256(metadata.checkpoint_sha256))
    return "V17-r3 qualification metadata mismatch: checkpoint_sha256";
  if (auto error = require(
          metadata.resolved_task_sha256,
          "db0268076fc48e9aa870d1345601e45e15921ebf12622402bfde90a67060cd24",
          "resolved_task_sha256");
      !error.empty())
    return error;
  return {};
}

struct V17R5QualificationMetadata {
  std::string_view training_recipe;
  std::string_view recipe_version;
  std::string_view deployment_status;
  std::string_view validator_profile;
  std::string_view qualification_status;
  std::string_view validator_receipt_sha256;
  std::string_view checkpoint_sha256;
  std::string_view resolved_task_sha256;
};

inline std::string ValidateV17R5QualificationMetadata(
    const V17R5QualificationMetadata& metadata) {
  auto require = [](std::string_view actual, std::string_view expected,
                    std::string_view label) -> std::string {
    if (actual == expected) return {};
    return "V17-r5 qualification metadata mismatch: " + std::string(label);
  };
  if (auto error = require(
          metadata.training_recipe, "rally_v17", "training_recipe");
      !error.empty())
    return error;
  if (auto error = require(metadata.recipe_version, "5", "recipe_version");
      !error.empty())
    return error;
  if (auto error = require(
          metadata.deployment_status, "gate3_candidate", "deployment_status");
      !error.empty())
    return error;
  if (auto error = require(
          metadata.validator_profile, "v17_r5_strong_stability_v1",
          "validator_profile");
      !error.empty())
    return error;
  if (auto error = require(
          metadata.qualification_status, "strong_stability_candidate",
          "qualification_status");
      !error.empty())
    return error;
  if (!IsLowercaseSha256(metadata.validator_receipt_sha256))
    return "V17-r5 qualification metadata mismatch: validator_receipt_sha256";
  if (!IsLowercaseSha256(metadata.checkpoint_sha256))
    return "V17-r5 qualification metadata mismatch: checkpoint_sha256";
  if (auto error = require(
          metadata.resolved_task_sha256,
          "fbfdad2488f236acb3e038bf28b8cbfad166044d4c2d756b3bd78606f0d9ee7a",
          "resolved_task_sha256");
      !error.empty())
    return error;
  return {};
}

struct V17R1StationaryReplayMetadata {
  std::string_view training_recipe;
  std::string_view recipe_version;
  std::string_view runtime_contract;
  std::string_view actor_obs_contract;
  std::string_view qdes_action_contract;
  std::string_view hitter_action_contract;
  std::string_view checkpoint_sha256;
  std::string_view env_cfg_sha256;
  std::string_view task_recipe_sha256;
  std::string_view motion_forehand_sha256;
  std::string_view motion_backhand_sha256;
  std::string_view recovery_recipe;
  std::string_view curriculum_gates;
  std::string_view termination_contract;
};

inline std::string ValidateV17R1StationaryReplayMetadata(
    const V17R1StationaryReplayMetadata& metadata) {
  constexpr std::string_view kCheckpointSha =
      "88ba620d2c2af88c212f5af9b24bdec07c08225576eafa67e2da60c4eb5e5480";
  constexpr std::string_view kEnvSha =
      "37874e1434943a5164d64ddada2cbaa85ed2b2ad7b183a823d68afeaee5f4f5e";
  constexpr std::string_view kTaskSha =
      "8b0b213f0ab1e2c9a8d56e066aff0da3d9be34216294eb866ad12ee0c67143ec";
  constexpr std::string_view kMotionForehandSha =
      "a6c68513720b12b168379cd6fa13f8b77607b4fa0bf7e828c4e1d81eda6f2094";
  constexpr std::string_view kMotionBackhandSha =
      "67d04e13deeed068bdb003e379e18330dcd29210d280188fab7af26c0764eaac";
  constexpr std::string_view kRecoveryRecipe =
      "markov_stratified_v2,0.1000,8192,256,4,16,100,100,50,"
      "0.0800|0.3000|0.8000,0.4500,24000,0.5000,1,0.0019,0.0030,"
      "0.7170,0.0050,0.0200,-0.3500,0.1000,0.2000,0.1000,1.5500";
  constexpr std::string_view kCurriculumGates =
      "50.0000,100.0000;"
      "0.5500,0.1200,0.0800,0.2500,0.0100,0.2000,250;"
      "0.4500,0.0800,0.0400,0.1800,0.0050,0.3000,100;"
      "0.7000,0.1800,0.4000,0.4000,0.2500,0.1000,500;"
      "0.6000,0.1400,0.3000,0.3000,0.1500,0.1500,150;"
      "8000,4000,12000,6000";

  auto require = [](std::string_view actual, std::string_view expected,
                    std::string_view label) -> std::string {
    if (actual == expected) return {};
    return "legacy V17-r1 stationary replay metadata mismatch: " +
           std::string(label);
  };

  if (auto error = require(
          metadata.training_recipe, "rally_v17", "training_recipe");
      !error.empty())
    return error;
  if (auto error = require(metadata.recipe_version, "1", "recipe_version");
      !error.empty())
    return error;
  if (auto error = require(
          metadata.runtime_contract, "rally_final_v2", "runtime_contract");
      !error.empty())
    return error;
  if (auto error = require(
          metadata.actor_obs_contract, "hitter_pure", "actor_obs_contract");
      !error.empty())
    return error;
  if (auto error = require(
          metadata.qdes_action_contract, "v11_affine_safe_qdes_v1",
          "qdes_action_contract");
      !error.empty())
    return error;
  if (auto error = require(
          metadata.hitter_action_contract, "v11_affine_safe_qdes_v1",
          "hitter_action_contract");
      !error.empty())
    return error;
  if (auto error = require(
          metadata.checkpoint_sha256, kCheckpointSha, "checkpoint_sha256");
      !error.empty())
    return error;
  if (auto error = require(metadata.env_cfg_sha256, kEnvSha, "env_cfg_sha256");
      !error.empty())
    return error;
  if (auto error = require(
          metadata.task_recipe_sha256, kTaskSha, "task_recipe_sha256");
      !error.empty())
    return error;
  if (auto error = require(
          metadata.motion_forehand_sha256, kMotionForehandSha,
          "motion_forehand_sha256");
      !error.empty())
    return error;
  if (auto error = require(
          metadata.motion_backhand_sha256, kMotionBackhandSha,
          "motion_backhand_sha256");
      !error.empty())
    return error;
  if (auto error = require(
          metadata.recovery_recipe, kRecoveryRecipe, "recovery_recipe");
      !error.empty())
    return error;
  if (auto error = require(
          metadata.curriculum_gates, kCurriculumGates, "curriculum_gates");
      !error.empty())
    return error;
  if (auto error = require(
          metadata.termination_contract, "rally_v8_reference_feet_no_wrist",
          "termination_contract");
      !error.empty())
    return error;
  return {};
}

struct FixedStationReplayDecision {
  bool accept = false;
  double command_x = 0.0;
  double command_y = 0.0;
  double requested_delta_m = 0.0;
  std::string reason;
};

// A reachable target is allowed to use the immutable session anchor.  A target
// that asks for any meaningful station transition is rejected; the racket
// target itself is never shifted to manufacture reachability.
inline FixedStationReplayDecision DecideFixedStationReplay(
    bool anchor_valid, double anchor_x, double anchor_y,
    double derived_station_x, double derived_station_y,
    double tolerance_m) {
  FixedStationReplayDecision decision;
  decision.command_x = anchor_x;
  decision.command_y = anchor_y;
  if (!anchor_valid) {
    decision.reason = "no_fresh_session_anchor";
    return decision;
  }
  if (!std::isfinite(anchor_x) || !std::isfinite(anchor_y) ||
      !std::isfinite(derived_station_x) ||
      !std::isfinite(derived_station_y) ||
      !std::isfinite(tolerance_m) || tolerance_m <= 0.0) {
    decision.reason = "invalid_fixed_station_input";
    return decision;
  }
  decision.requested_delta_m = std::hypot(
      derived_station_x - anchor_x, derived_station_y - anchor_y);
  if (decision.requested_delta_m > tolerance_m) {
    decision.reason = "station_transition_required";
    return decision;
  }
  decision.accept = true;
  decision.reason = "fixed_station_reachable";
  return decision;
}

}  // namespace a3_pingpong
