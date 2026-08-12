#include <gtest/gtest.h>

#include "a3_pingpong/pp_stationary_replay.hpp"

namespace {

a3_pingpong::V17R3QualificationMetadata ExactR3QualificationMetadata() {
  return {
      .training_recipe = "rally_v17",
      .recipe_version = "3",
      .deployment_status = "gate3_candidate",
      .validator_profile = "v17_r3_strong_stability_v1",
      .qualification_status = "strong_stability_candidate",
      .validator_receipt_sha256 =
          "1234567890abcdef1234567890abcdef1234567890abcdef1234567890abcdef",
      .checkpoint_sha256 =
          "abcdef1234567890abcdef1234567890abcdef1234567890abcdef1234567890",
      .resolved_task_sha256 =
          "db0268076fc48e9aa870d1345601e45e15921ebf12622402bfde90a67060cd24",
  };
}

a3_pingpong::V17R5QualificationMetadata ExactR5QualificationMetadata() {
  return {
      .training_recipe = "rally_v17",
      .recipe_version = "5",
      .deployment_status = "gate3_candidate",
      .validator_profile = "v17_r5_strong_stability_v1",
      .qualification_status = "strong_stability_candidate",
      .validator_receipt_sha256 =
          "1234567890abcdef1234567890abcdef1234567890abcdef1234567890abcdef",
      .checkpoint_sha256 =
          "abcdef1234567890abcdef1234567890abcdef1234567890abcdef1234567890",
      .resolved_task_sha256 =
          "fbfdad2488f236acb3e038bf28b8cbfad166044d4c2d756b3bd78606f0d9ee7a",
  };
}

a3_pingpong::V17R1StationaryReplayMetadata ExactMetadata() {
  return {
      .training_recipe = "rally_v17",
      .recipe_version = "1",
      .runtime_contract = "rally_final_v2",
      .actor_obs_contract = "hitter_pure",
      .qdes_action_contract = "v11_affine_safe_qdes_v1",
      .hitter_action_contract = "v11_affine_safe_qdes_v1",
      .checkpoint_sha256 =
          "88ba620d2c2af88c212f5af9b24bdec07c08225576eafa67e2da60c4eb5e5480",
      .env_cfg_sha256 =
          "37874e1434943a5164d64ddada2cbaa85ed2b2ad7b183a823d68afeaee5f4f5e",
      .task_recipe_sha256 =
          "8b0b213f0ab1e2c9a8d56e066aff0da3d9be34216294eb866ad12ee0c67143ec",
      .motion_forehand_sha256 =
          "a6c68513720b12b168379cd6fa13f8b77607b4fa0bf7e828c4e1d81eda6f2094",
      .motion_backhand_sha256 =
          "67d04e13deeed068bdb003e379e18330dcd29210d280188fab7af26c0764eaac",
      .recovery_recipe =
          "markov_stratified_v2,0.1000,8192,256,4,16,100,100,50,"
          "0.0800|0.3000|0.8000,0.4500,24000,0.5000,1,0.0019,0.0030,"
          "0.7170,0.0050,0.0200,-0.3500,0.1000,0.2000,0.1000,1.5500",
      .curriculum_gates =
          "50.0000,100.0000;"
          "0.5500,0.1200,0.0800,0.2500,0.0100,0.2000,250;"
          "0.4500,0.0800,0.0400,0.1800,0.0050,0.3000,100;"
          "0.7000,0.1800,0.4000,0.4000,0.2500,0.1000,500;"
          "0.6000,0.1400,0.3000,0.3000,0.1500,0.1500,150;"
          "8000,4000,12000,6000",
      .termination_contract = "rally_v8_reference_feet_no_wrist",
  };
}

TEST(PpStationaryReplay, V17R3QualificationMetadataPassesExactly) {
  EXPECT_TRUE(a3_pingpong::ValidateV17R3QualificationMetadata(
                  ExactR3QualificationMetadata())
                  .empty());
}

TEST(PpStationaryReplay, V17R3QualificationFailsClosed) {
  auto metadata = ExactR3QualificationMetadata();
  metadata.validator_profile = "";
  EXPECT_NE(
      a3_pingpong::ValidateV17R3QualificationMetadata(metadata)
          .find("validator_profile"),
      std::string::npos);

  metadata = ExactR3QualificationMetadata();
  metadata.qualification_status = "NOT_PROVEN";
  EXPECT_NE(
      a3_pingpong::ValidateV17R3QualificationMetadata(metadata)
          .find("qualification_status"),
      std::string::npos);

  metadata = ExactR3QualificationMetadata();
  metadata.validator_receipt_sha256 = "not-a-sha256";
  EXPECT_NE(
      a3_pingpong::ValidateV17R3QualificationMetadata(metadata)
          .find("validator_receipt_sha256"),
      std::string::npos);

  metadata = ExactR3QualificationMetadata();
  metadata.resolved_task_sha256 = "not-the-canonical-task";
  EXPECT_NE(
      a3_pingpong::ValidateV17R3QualificationMetadata(metadata)
          .find("resolved_task_sha256"),
      std::string::npos);
}

TEST(PpStationaryReplay, V17R5QualificationMetadataPassesExactly) {
  EXPECT_TRUE(a3_pingpong::ValidateV17R5QualificationMetadata(
                  ExactR5QualificationMetadata())
                  .empty());
}

TEST(PpStationaryReplay, V17R5QualificationFailsOnR3Profile) {
  auto metadata = ExactR5QualificationMetadata();
  metadata.validator_profile = "v17_r3_strong_stability_v1";
  EXPECT_NE(
      a3_pingpong::ValidateV17R5QualificationMetadata(metadata)
          .find("validator_profile"),
      std::string::npos);
}

TEST(PpStationaryReplay, ExactLegacyArtifactMetadataPasses) {
  EXPECT_TRUE(
      a3_pingpong::ValidateV17R1StationaryReplayMetadata(ExactMetadata())
          .empty());
}

TEST(PpStationaryReplay, RecipeV2AndWrongCheckpointFailClosed) {
  auto metadata = ExactMetadata();
  metadata.recipe_version = "2";
  EXPECT_NE(
      a3_pingpong::ValidateV17R1StationaryReplayMetadata(metadata)
          .find("recipe_version"),
      std::string::npos);

  metadata = ExactMetadata();
  const std::string wrong_checkpoint(64, '0');
  metadata.checkpoint_sha256 = wrong_checkpoint;
  EXPECT_NE(
      a3_pingpong::ValidateV17R1StationaryReplayMetadata(metadata)
          .find("checkpoint_sha256"),
      std::string::npos);
}

TEST(PpStationaryReplay, ReachableRequestSnapsToImmutableAnchor) {
  const auto decision = a3_pingpong::DecideFixedStationReplay(
      true, -0.500, -0.7625, -0.497, -0.755, 0.020);
  ASSERT_TRUE(decision.accept);
  EXPECT_DOUBLE_EQ(decision.command_x, -0.500);
  EXPECT_DOUBLE_EQ(decision.command_y, -0.7625);
  EXPECT_LT(decision.requested_delta_m, 0.020);
}

TEST(PpStationaryReplay, StepRequestAndMissingAnchorFailClosed) {
  auto decision = a3_pingpong::DecideFixedStationReplay(
      true, -0.500, -0.7625, -0.500, -0.7424, 0.020);
  EXPECT_FALSE(decision.accept);
  EXPECT_EQ(decision.reason, "station_transition_required");

  decision = a3_pingpong::DecideFixedStationReplay(
      false, -0.500, -0.7625, -0.500, -0.7625, 0.020);
  EXPECT_FALSE(decision.accept);
  EXPECT_EQ(decision.reason, "no_fresh_session_anchor");
}

}  // namespace
