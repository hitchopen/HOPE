#pragma once

#include <array>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "a3_pingpong/pp_sha256.hpp"

namespace a3_pingpong {

inline constexpr std::string_view kA3DeployManifestSchema =
    "hope_a3_deploy_manifest_v1";
inline constexpr std::string_view kA3DeployManifestP0Status =
    "p0_contract_only_not_hardware_authorized";

inline constexpr std::array<std::string_view, 31> kA3BackendJointOrder = {
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "head_yaw_joint",
    "head_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
};

inline std::string ValidateA3PolicyJointBijection(
    const std::vector<std::string>& joint_names) {
  if (joint_names.size() != kA3BackendJointOrder.size())
    return "A3 policy joint order does not contain 31 joints";
  for (const std::string_view expected : kA3BackendJointOrder) {
    std::size_t count = 0;
    for (const std::string& actual : joint_names)
      if (actual == expected) ++count;
    if (count != 1)
      return "A3 policy joint order is not a backend-joint bijection: " +
             std::string(expected);
  }
  return {};
}

// Keep byte-identical order with a3_deploy_manifest.py.  The fingerprint
// covers the exact strings read from ONNX metadata, including every numeric
// joint array.  It therefore binds the human-readable JSON receipt to the
// values actually consumed by this runner.
inline constexpr std::array<std::string_view, 42>
    kV17R6P0FingerprintKeys = {
        "joint_names",
        "default_joint_pos",
        "action_scale",
        "joint_stiffness",
        "joint_damping",
        "a3_training_joint_damping",
        "a3_passive_joint_damping",
        "a3_joint_effort_limit",
        "qdes_action_contract",
        "qdes_policy_feedback_contract",
        "qdes_joint_names",
        "qdes_safe_lower_rad",
        "qdes_safe_upper_rad",
        "qdes_hard_lower_rad",
        "qdes_hard_upper_rad",
        "qdes_actual_q_hard_tolerance_rad",
        "actor_obs_contract",
        "actor_obs_total_dim",
        "actor_obs_term_dims",
        "actor_obs_term_sources_json",
        "hitter_pure_training_recipe",
        "hitter_pure_training_recipe_version",
        "hitter_pure_runtime_contract",
        "hitter_pure_action_contract",
        "hitter_pure_v17_recipe_revision",
        "hitter_pure_v17_sensor_contract",
        "base_localization_contract",
        "base_pose_source",
        "base_pose_schema",
        "orientation_contract",
        "angular_velocity_contract",
        "yaw_align_contract",
        "world_frame_contract",
        "calibration_contract",
        "base_mocap_max_age_s",
        "base_mocap_max_propagation_s",
        "a3_control_physics_dt_s",
        "a3_control_decimation",
        "a3_control_policy_dt_s",
        "v17_ground_plant_contract_json",
        "a3_qdes_parity_csv_sha256",
        "hitter_pure_checkpoint_sha256",
};

inline constexpr std::array<std::string_view, 48>
    kV17R10P0FingerprintKeys = {
        "joint_names",
        "default_joint_pos",
        "action_scale",
        "joint_stiffness",
        "joint_damping",
        "a3_training_joint_damping",
        "a3_passive_joint_damping",
        "a3_joint_effort_limit",
        "qdes_action_contract",
        "qdes_policy_feedback_contract",
        "qdes_joint_names",
        "qdes_safe_lower_rad",
        "qdes_safe_upper_rad",
        "qdes_hard_lower_rad",
        "qdes_hard_upper_rad",
        "qdes_actual_q_hard_tolerance_rad",
        "actor_obs_contract",
        "actor_obs_total_dim",
        "actor_obs_term_dims",
        "actor_obs_term_sources_json",
        "hitter_pure_training_recipe",
        "hitter_pure_training_recipe_version",
        "hitter_pure_runtime_contract",
        "hitter_pure_action_contract",
        "hitter_pure_v17_recipe_revision",
        "hitter_pure_v17_sensor_contract",
        "base_localization_contract",
        "base_pose_source",
        "base_pose_schema",
        "orientation_contract",
        "angular_velocity_contract",
        "yaw_align_contract",
        "world_frame_contract",
        "calibration_contract",
        "base_mocap_max_age_s",
        "base_mocap_max_propagation_s",
        "a3_control_physics_dt_s",
        "a3_control_decimation",
        "a3_control_policy_dt_s",
        "v17_ground_plant_contract_json",
        "a3_qdes_parity_csv_sha256",
        "hitter_pure_checkpoint_sha256",
        "hitter_pure_v17_fixed_station_contract",
        "hitter_pure_v17_release_contract",
        "hitter_pure_v17_target_stream_contract",
        "hitter_pure_planner_schema",
        "hitter_pure_planner_stability_contract",
        "hitter_pure_fixed_hit_plane_relative_x_m",
};

using DeployFingerprintValues =
    std::vector<std::pair<std::string, std::string>>;

inline bool IsDeployLowercaseSha256(std::string_view value) {
  if (value.size() != 64) return false;
  for (const char character : value) {
    if (!((character >= '0' && character <= '9') ||
          (character >= 'a' && character <= 'f')))
      return false;
  }
  return true;
}

inline std::string BuildV17R6P0FingerprintPayload(
    const DeployFingerprintValues& values) {
  if (values.size() != kV17R6P0FingerprintKeys.size()) {
    throw std::invalid_argument(
        "V17-r6 P0 fingerprint value count does not match its key contract");
  }
  std::string payload;
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (values[index].first != kV17R6P0FingerprintKeys[index]) {
      throw std::invalid_argument(
          "V17-r6 P0 fingerprint key order mismatch at index " +
          std::to_string(index));
    }
    payload.append(values[index].first);
    payload.push_back('=');
    payload.append(values[index].second);
    payload.push_back('\n');
  }
  return payload;
}

inline std::string ComputeV17R6P0Fingerprint(
    const DeployFingerprintValues& values) {
  return PpSha256::String(BuildV17R6P0FingerprintPayload(values));
}

inline std::string BuildV17R10P0FingerprintPayload(
    const DeployFingerprintValues& values) {
  if (values.size() != kV17R10P0FingerprintKeys.size()) {
    throw std::invalid_argument(
        "V17-r10 P0 fingerprint value count does not match its key contract");
  }
  std::string payload;
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (values[index].first != kV17R10P0FingerprintKeys[index]) {
      throw std::invalid_argument(
          "V17-r10 P0 fingerprint key order mismatch at index " +
          std::to_string(index));
    }
    payload.append(values[index].first);
    payload.push_back('=');
    payload.append(values[index].second);
    payload.push_back('\n');
  }
  return payload;
}

inline std::string ComputeV17R10P0Fingerprint(
    const DeployFingerprintValues& values) {
  return PpSha256::String(BuildV17R10P0FingerprintPayload(values));
}

struct V17R6P0ContractMetadata {
  std::string_view training_recipe;
  std::string_view recipe_version;
  std::string_view deployment_status;
  std::string_view qualification_status;
  std::string_view manifest_schema;
  std::string_view manifest_status;
  std::string_view hardware_authorized;
  std::string_view contract_fingerprint_sha256;
  std::string_view manifest_json;
  std::string_view manifest_sha256;
  DeployFingerprintValues fingerprint_values;
};

inline std::string ValidateV17R6P0ContractMetadata(
    const V17R6P0ContractMetadata& metadata) {
  auto require = [](std::string_view actual, std::string_view expected,
                    std::string_view label) -> std::string {
    if (actual == expected) return {};
    return "V17-r6 P0 deploy contract mismatch: " + std::string(label);
  };
  if (auto error =
          require(metadata.training_recipe, "rally_v17", "training_recipe");
      !error.empty())
    return error;
  if (auto error = require(metadata.recipe_version, "6", "recipe_version");
      !error.empty())
    return error;
  if (auto error = require(
          metadata.deployment_status, "p0_contract_candidate",
          "deployment_status");
      !error.empty())
    return error;
  if (auto error = require(
          metadata.qualification_status, "not_qualified",
          "qualification_status");
      !error.empty())
    return error;
  if (auto error = require(
          metadata.manifest_schema, kA3DeployManifestSchema,
          "manifest_schema");
      !error.empty())
    return error;
  if (auto error = require(
          metadata.manifest_status, kA3DeployManifestP0Status,
          "manifest_status");
      !error.empty())
    return error;
  if (auto error = require(
          metadata.hardware_authorized, "false", "hardware_authorized");
      !error.empty())
    return error;
  if (!IsDeployLowercaseSha256(metadata.contract_fingerprint_sha256))
    return "V17-r6 P0 deploy contract mismatch: contract_fingerprint_sha256";
  if (!IsDeployLowercaseSha256(metadata.manifest_sha256))
    return "V17-r6 P0 deploy contract mismatch: manifest_sha256";

  std::string computed_fingerprint;
  try {
    computed_fingerprint =
        ComputeV17R6P0Fingerprint(metadata.fingerprint_values);
  } catch (const std::exception& error) {
    return std::string("V17-r6 P0 deploy contract mismatch: ") + error.what();
  }
  if (computed_fingerprint != metadata.contract_fingerprint_sha256)
    return "V17-r6 P0 deploy contract mismatch: metadata fingerprint";
  if (PpSha256::String(std::string(metadata.manifest_json)) !=
      metadata.manifest_sha256)
    return "V17-r6 P0 deploy contract mismatch: manifest content SHA256";

  // The JSON is canonical (no whitespace) on export.  These checks establish
  // the fail-closed authorization state and bind it to the same fingerprint;
  // all numerical values remain independently parsed and validated by
  // PpOnnxPolicy.
  const std::string json(metadata.manifest_json);
  const std::array<std::string, 6> required_tokens = {
      "\"schema\":\"hope_a3_deploy_manifest_v1\"",
      "\"status\":\"p0_contract_only_not_hardware_authorized\"",
      "\"hardware_authorized\":false",
      "\"qualification_status\":\"not_qualified\"",
      "\"recipe_revision\":6",
      "\"contract_fingerprint_sha256\":\"" +
          computed_fingerprint + "\"",
  };
  for (const auto& token : required_tokens) {
    if (json.find(token) == std::string::npos)
      return "V17-r6 P0 deploy contract mismatch: manifest JSON token";
  }
  return {};
}

struct V17R10P0ContractMetadata {
  std::string_view training_recipe;
  std::string_view recipe_version;
  std::string_view runtime_contract;
  std::string_view deployment_status;
  std::string_view qualification_status;
  std::string_view manifest_schema;
  std::string_view manifest_status;
  std::string_view hardware_authorized;
  std::string_view fixed_station_contract;
  std::string_view release_contract;
  std::string_view target_stream_contract;
  std::string_view planner_schema;
  std::string_view planner_stability_contract;
  std::string_view fixed_hit_plane_relative_x_m;
  std::string_view contract_fingerprint_sha256;
  std::string_view manifest_json;
  std::string_view manifest_sha256;
  DeployFingerprintValues fingerprint_values;
};

inline std::string ValidateV17R10P0ContractMetadata(
    const V17R10P0ContractMetadata& metadata) {
  auto require = [](std::string_view actual, std::string_view expected,
                    std::string_view label) -> std::string {
    if (actual == expected) return {};
    return "V17-r10 P0 deploy contract mismatch: " + std::string(label);
  };
  const std::array<std::pair<std::string_view, std::string_view>, 14>
      exact = {{{metadata.training_recipe, "rally_v17"},
                {metadata.recipe_version, "10"},
                {metadata.runtime_contract,
                 "rally_v17_fixed_station_ball_clock_v1"},
                {metadata.deployment_status, "p0_contract_candidate"},
                {metadata.qualification_status, "not_qualified"},
                {metadata.manifest_schema, kA3DeployManifestSchema},
                {metadata.manifest_status, kA3DeployManifestP0Status},
                {metadata.hardware_authorized, "false"},
                {metadata.fixed_station_contract,
                 "session_anchor_xy_with_10cm_recovery_v1"},
                {metadata.release_contract,
                 "telemetry_only_ball_clock_v1"},
                {metadata.target_stream_contract, "freeze_at_engage_v1"},
                {metadata.planner_schema, "2"},
                {metadata.planner_stability_contract,
                 "three_revisions_v1,0.0300,0.2500,0.0300"},
                {metadata.fixed_hit_plane_relative_x_m, "0.5800"}}};
  const std::array<std::string_view, 14> labels = {
      "training_recipe", "recipe_version", "runtime_contract",
      "deployment_status", "qualification_status", "manifest_schema",
      "manifest_status", "hardware_authorized", "fixed_station_contract",
      "release_contract", "target_stream_contract", "planner_schema",
      "planner_stability_contract", "fixed_hit_plane_relative_x_m"};
  for (std::size_t index = 0; index < exact.size(); ++index) {
    if (auto error = require(exact[index].first, exact[index].second,
                             labels[index]);
        !error.empty())
      return error;
  }
  if (!IsDeployLowercaseSha256(metadata.contract_fingerprint_sha256))
    return "V17-r10 P0 deploy contract mismatch: contract_fingerprint_sha256";
  if (!IsDeployLowercaseSha256(metadata.manifest_sha256))
    return "V17-r10 P0 deploy contract mismatch: manifest_sha256";

  std::string computed_fingerprint;
  try {
    computed_fingerprint =
        ComputeV17R10P0Fingerprint(metadata.fingerprint_values);
  } catch (const std::exception& error) {
    return std::string("V17-r10 P0 deploy contract mismatch: ") + error.what();
  }
  if (computed_fingerprint != metadata.contract_fingerprint_sha256)
    return "V17-r10 P0 deploy contract mismatch: metadata fingerprint";
  if (PpSha256::String(std::string(metadata.manifest_json)) !=
      metadata.manifest_sha256)
    return "V17-r10 P0 deploy contract mismatch: manifest content SHA256";

  const std::string json(metadata.manifest_json);
  const std::array<std::string, 11> required_tokens = {
      "\"schema\":\"hope_a3_deploy_manifest_v1\"",
      "\"status\":\"p0_contract_only_not_hardware_authorized\"",
      "\"hardware_authorized\":false",
      "\"qualification_status\":\"not_qualified\"",
      "\"recipe_revision\":10",
      "\"runtime_contract\":\"rally_v17_fixed_station_ball_clock_v1\"",
      "\"station\":\"session_anchor_xy_with_10cm_recovery_v1\"",
      "\"release\":\"telemetry_only_ball_clock_v1\"",
      "\"target_stream\":\"freeze_at_engage_v1\"",
      "\"planner_schema\":2",
      "\"contract_fingerprint_sha256\":\"" + computed_fingerprint + "\"",
  };
  for (const auto& token : required_tokens) {
    if (json.find(token) == std::string::npos)
      return "V17-r10 P0 deploy contract mismatch: manifest JSON token";
  }
  return {};
}

}  // namespace a3_pingpong
