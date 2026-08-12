#include <gtest/gtest.h>

#include <string>
#include <utility>

#include "a3_pingpong/pp_deploy_contract.hpp"
#include "a3_pingpong/pp_qdes_contract.hpp"

namespace a3_pingpong {
namespace {

DeployFingerprintValues ExactFingerprintValues() {
  DeployFingerprintValues values;
  values.reserve(kV17R6P0FingerprintKeys.size());
  for (const std::string_view key : kV17R6P0FingerprintKeys) {
    values.emplace_back(std::string(key), "value-for-" + std::string(key));
  }
  return values;
}

V17R6P0ContractMetadata ExactP0Metadata() {
  auto values = ExactFingerprintValues();
  return {
      .training_recipe = "rally_v17",
      .recipe_version = "6",
      .deployment_status = "p0_contract_candidate",
      .qualification_status = "not_qualified",
      .manifest_schema = kA3DeployManifestSchema,
      .manifest_status = kA3DeployManifestP0Status,
      .hardware_authorized = "false",
      .contract_fingerprint_sha256 = {},
      .manifest_json = {},
      .manifest_sha256 = {},
      .fingerprint_values = std::move(values),
  };
}

DeployFingerprintValues ExactR10FingerprintValues() {
  DeployFingerprintValues values;
  values.reserve(kV17R10P0FingerprintKeys.size());
  for (const std::string_view key : kV17R10P0FingerprintKeys) {
    values.emplace_back(std::string(key), "value-for-" + std::string(key));
  }
  return values;
}

V17R10P0ContractMetadata ExactR10P0Metadata() {
  return {
      .training_recipe = "rally_v17",
      .recipe_version = "10",
      .runtime_contract = "rally_v17_fixed_station_ball_clock_v1",
      .deployment_status = "p0_contract_candidate",
      .qualification_status = "not_qualified",
      .manifest_schema = kA3DeployManifestSchema,
      .manifest_status = kA3DeployManifestP0Status,
      .hardware_authorized = "false",
      .fixed_station_contract = "session_anchor_xy_with_10cm_recovery_v1",
      .release_contract = "telemetry_only_ball_clock_v1",
      .target_stream_contract = "freeze_at_engage_v1",
      .planner_schema = "2",
      .planner_stability_contract =
          "three_revisions_v1,0.0300,0.2500,0.0300",
      .fixed_hit_plane_relative_x_m = "0.5800",
      .contract_fingerprint_sha256 = {},
      .manifest_json = {},
      .manifest_sha256 = {},
      .fingerprint_values = ExactR10FingerprintValues(),
  };
}

TEST(PpDeployContract, V17R6P0ReceiptPassesAndRemainsNonAuthorizing) {
  auto metadata = ExactP0Metadata();
  const std::string fingerprint =
      ComputeV17R6P0Fingerprint(metadata.fingerprint_values);
  const std::string manifest =
      "{\"contract_fingerprint_sha256\":\"" + fingerprint +
      "\",\"hardware_authorized\":false,"
      "\"qualification_status\":\"not_qualified\","
      "\"recipe_revision\":6,"
      "\"schema\":\"hope_a3_deploy_manifest_v1\","
      "\"status\":\"p0_contract_only_not_hardware_authorized\"}";
  const std::string manifest_sha = PpSha256::String(manifest);
  metadata.contract_fingerprint_sha256 = fingerprint;
  metadata.manifest_json = manifest;
  metadata.manifest_sha256 = manifest_sha;
  EXPECT_TRUE(ValidateV17R6P0ContractMetadata(metadata).empty());
}

TEST(PpDeployContract, FingerprintAndHardwareAuthorizationFailClosed) {
  auto metadata = ExactP0Metadata();
  const std::string fingerprint =
      ComputeV17R6P0Fingerprint(metadata.fingerprint_values);
  const std::string manifest =
      "{\"contract_fingerprint_sha256\":\"" + fingerprint +
      "\",\"hardware_authorized\":false,"
      "\"qualification_status\":\"not_qualified\","
      "\"recipe_revision\":6,"
      "\"schema\":\"hope_a3_deploy_manifest_v1\","
      "\"status\":\"p0_contract_only_not_hardware_authorized\"}";
  const std::string manifest_sha = PpSha256::String(manifest);
  metadata.contract_fingerprint_sha256 = fingerprint;
  metadata.manifest_json = manifest;
  metadata.manifest_sha256 = manifest_sha;

  metadata.fingerprint_values[2].second = "tampered-action-scale";
  EXPECT_NE(
      ValidateV17R6P0ContractMetadata(metadata).find("fingerprint"),
      std::string::npos);

  metadata = ExactP0Metadata();
  metadata.contract_fingerprint_sha256 =
      ComputeV17R6P0Fingerprint(metadata.fingerprint_values);
  metadata.hardware_authorized = "true";
  EXPECT_NE(
      ValidateV17R6P0ContractMetadata(metadata)
          .find("hardware_authorized"),
      std::string::npos);
}

TEST(PpDeployContract, V17R10P0ReceiptBindsFixedStationExecution) {
  auto metadata = ExactR10P0Metadata();
  const std::string fingerprint =
      ComputeV17R10P0Fingerprint(metadata.fingerprint_values);
  const std::string manifest =
      "{\"contract_fingerprint_sha256\":\"" + fingerprint +
      "\",\"hardware_authorized\":false,"
      "\"planner_schema\":2,\"qualification_status\":\"not_qualified\","
      "\"recipe_revision\":10,"
      "\"release\":\"telemetry_only_ball_clock_v1\","
      "\"runtime_contract\":\"rally_v17_fixed_station_ball_clock_v1\","
      "\"schema\":\"hope_a3_deploy_manifest_v1\","
      "\"station\":\"session_anchor_xy_with_10cm_recovery_v1\","
      "\"status\":\"p0_contract_only_not_hardware_authorized\","
      "\"target_stream\":\"freeze_at_engage_v1\"}";
  metadata.contract_fingerprint_sha256 = fingerprint;
  metadata.manifest_json = manifest;
  const std::string manifest_sha = PpSha256::String(manifest);
  metadata.manifest_sha256 = manifest_sha;
  EXPECT_TRUE(ValidateV17R10P0ContractMetadata(metadata).empty());

  metadata.release_contract = "ready_gated";
  EXPECT_NE(ValidateV17R10P0ContractMetadata(metadata).find("release_contract"),
            std::string::npos);
}

TEST(PpDeployContract, V11AffineSafeQdesMatchesPythonReferenceCases) {
  EXPECT_DOUBLE_EQ(
      ComputeV11AffineSafeQdes(-10.0, 0.1, 0.2, -0.5, 0.5), -0.5);
  EXPECT_DOUBLE_EQ(
      ComputeV11AffineSafeQdes(0.25, -0.2, 0.4, -0.5, 0.5), -0.1);
  EXPECT_DOUBLE_EQ(
      ComputeV11AffineSafeQdes(10.0, 0.3, 0.5, -0.5, 0.5), 0.5);
}

TEST(PpDeployContract, A3JointMappingMustBeAnExactBijection) {
  std::vector<std::string> names;
  for (const std::string_view name : kA3BackendJointOrder)
    names.emplace_back(name);
  EXPECT_TRUE(ValidateA3PolicyJointBijection(names).empty());
  names[7] = names[6];
  EXPECT_FALSE(ValidateA3PolicyJointBijection(names).empty());
}

}  // namespace
}  // namespace a3_pingpong
