#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "a3_pingpong/pp_joint_limits.hpp"
#include "a3_pingpong/pp_obs_builder.hpp"
#include "a3_pingpong/pp_planner_input.hpp"
#include "a3_pingpong/pp_planner_lifecycle.hpp"
#include "a3_pingpong/pp_runtime_contract.hpp"
#include "a3_pingpong/pp_stationary_replay.hpp"

namespace a3_pingpong {
namespace {

std::vector<double> Schema3RacketForRuntimeContractTest(
    std::uint64_t command_seq, std::uint64_t revision,
    double reach_level, double swing_foot_sign, double valid = 1.0,
    double producer_age_s = 0.0, std::uint64_t flight_id = 77) {
  const double producer = PpNowWallSec() - producer_age_s;
  const double sec = std::floor(producer);
  const double nsec = std::floor((producer - sec) * 1.0e9);
  const double producer_wire = sec + nsec * 1.0e-9;
  const double tts = valid == 1.0 ? 1.0 : 0.0;
  const double strike = valid == 1.0 ? producer_wire + tts : 0.0;
  return {
      3.0, valid, 1.0, 0.58, -0.44, 1.0, 2.0, 0.4, 0.8,
      tts, strike, 0.0, sec, nsec, static_cast<double>(command_seq),
      static_cast<double>(flight_id), static_cast<double>(revision), 8.0, 0.12,
      reach_level, swing_foot_sign};
}

TEST(PpRuntimeContract, AcceptsLegacyAndRuntimeV1Models) {
  EXPECT_EQ(validate_hitter_pure_runtime_contract("", ""),
            HitterPureRuntimeContract::kLegacy);
  EXPECT_EQ(validate_hitter_pure_runtime_contract("rally_final_v1", "rally_v9"),
            HitterPureRuntimeContract::kRallyFinalV1);
  EXPECT_EQ(validate_hitter_pure_runtime_contract("rally_final_v1", "rally_final_v3"),
            HitterPureRuntimeContract::kRallyFinalV1);
}

TEST(PpRuntimeContract, Schema29AcceptsOnlyTrainable112DLineage) {
  const std::string receipt =
      "model_21800.pt,sha256="
      "daca62616178177647cf9949128582785529dbe6b7ba080fc96d86da23916561,"
      "source_input_dim=110,destination_input_dim=112,"
      "zeroed_source_cols=76|81,appended_zero_cols=110|111,trainable_full31=true";
  EXPECT_TRUE(hitter_schema29_initialization_tuple(
      "schema29_from_build1_model21800_functional_vxy_zero_append_reach_trainable_v1",
      receipt));
  EXPECT_FALSE(hitter_schema29_initialization_tuple(
      "schema28_from_build1_model21800_functional_vxy_zero_reach_frozen_core_shadow_v1",
      receipt));
  EXPECT_FALSE(hitter_schema29_initialization_tuple(
      "schema29_from_build1_model21800_functional_vxy_zero_append_reach_trainable_v1",
      receipt + ",unexpected=true"));
  EXPECT_NO_THROW(validate_hitter_pingpong_initialization_contract(
      "hitter_pingpong_build2_trainable_home_optional_reach_v1",
      "schema29_from_build1_model21800_functional_vxy_zero_append_reach_trainable_v1",
      receipt));
  EXPECT_THROW(validate_hitter_pingpong_schema27_nonprivileged_metadata(
                   "hitter_pingpong_build2_trainable_home_optional_reach_v1",
                   "hitter_pure_112_headslots_vxy_reach_v1",
                   "trainable_full31_core_optional_reach_curriculum_v1",
                   "fresh_schema3_flight_shared_session_home_support_intent_v2",
                   "110,111", "planner_target_tuple,planner_target_tuple",
                   "planner_tuple_no_plant_inference_v3",
                   "staged_planner_support_intent_no_plant_inference_v4", "true",
                   "staged_support_intent_training_v3",
                   "immutable_home_world_torso_trainable_support_v5",
                   "full_swing_torso_visible_settle_trainable_v5"),
               std::runtime_error);
  EXPECT_EQ(validate_hitter_pingpong_schema27_nonprivileged_metadata(
                "hitter_pingpong_build2_trainable_home_optional_reach_v1",
                "hitter_pure_112_headslots_vxy_reach_v1",
                "trainable_full31_core_optional_reach_curriculum_v1",
                "fresh_schema3_flight_shared_session_home_support_intent_v2",
                "110,111", "planner_target_tuple,planner_target_tuple",
                "planner_tuple_no_plant_inference_v3",
                "staged_planner_support_intent_no_plant_inference_v4", "false",
                "staged_support_intent_candidate_v4",
                "immutable_home_world_torso_trainable_support_v5",
                "full_swing_torso_visible_settle_trainable_v5"),
            HitterOptionalReachRuntimeMode::kCandidateSchema3Level0Only);
  EXPECT_THROW(validate_hitter_pingpong_schema27_nonprivileged_metadata(
                   "hitter_pingpong_build2_trainable_home_optional_reach_v1",
                   "hitter_pure_112_headslots_vxy_reach_v1",
                   "trainable_full31_core_optional_reach_curriculum_v1",
                   "fresh_schema3_flight_shared_session_home_support_intent_v2",
                   "110,111", "planner_target_tuple,planner_target_tuple",
                   "planner_tuple_no_plant_inference_v3",
                   "staged_planner_support_intent_no_plant_inference_v4", "true",
                   "staged_support_intent_training_v3",
                   "immutable_home_world_torso_unified_support_residual_v4",
                   "full_swing_torso_then_visible_settle_v4"),
               std::runtime_error);

  EXPECT_NO_THROW(validate_hitter_pingpong_schema29_single_output_metadata(
      "hitter_pingpong_build2_trainable_home_optional_reach_v1",
      "policy_owned_112d_markov_trainable_no_qdes_override_v1",
      "schema29_from_build1_model21800_functional_vxy_zero_append_reach_trainable_v1",
      "hitter_pure_112_headslots_vxy_reach_v1",
      "trainable_full31_core_optional_reach_curriculum_v1",
      "actions31_v1", true, false));
  EXPECT_THROW(validate_hitter_pingpong_schema29_single_output_metadata(
                   "hitter_pingpong_build2_trainable_home_optional_reach_v1",
                   "policy_owned_112d_markov_trainable_no_qdes_override_v1",
                   "schema29_from_build1_model21800_functional_vxy_zero_append_reach_trainable_v1",
                   "hitter_pure_112_headslots_vxy_reach_v1",
                   "trainable_full31_core_optional_reach_curriculum_v1",
                   "actions31_core_action_shadow_owned15_v1", true, true),
               std::runtime_error);

  const auto validate_candidate_qualification = [](
      const std::string& trajectory = "NOT_PROVEN",
      const std::string& scope = "SIM_TRAINING_ONLY",
      const std::string& artifact =
          "fixed_home_support_candidate_bank_v4",
      const std::string& bank_sha = kSchema29QuestionBankSha256,
      const std::string& receipt_sha =
          kSchema29QuestionBankReceiptSha256) {
    validate_hitter_pingpong_schema29_candidate_qualification_metadata(
        "hitter_pingpong_build2_trainable_home_optional_reach_v1",
        "fixed_home_support_candidate_bank_v4_axis_envelope_"
        "prefilter_not_trajectory_certification_v1",
        "NOT_PROVEN_axis_prefilter_only",
        artifact,
        bank_sha,
        receipt_sha,
        "fixed_home_coherent_question_bank_v1,device_discrete_row_gather_v1,"
        "TRAINING_SCREENED,SIM_TRAINING_ONLY,deployment_NOT_PROVEN",
        scope, "NOT_PROVEN", "NOT_PROVEN", "NOT_PROVEN", trajectory,
        "NOT_PROVEN");
  };
  EXPECT_NO_THROW(validate_candidate_qualification());
  EXPECT_THROW(validate_candidate_qualification(
                   "TRAINING_TRAJECTORY_CERTIFIED"),
               std::runtime_error);
  EXPECT_THROW(validate_candidate_qualification(
                   "NOT_PROVEN", "TRAINING_TRAJECTORY_CERTIFIED"),
               std::runtime_error);
  EXPECT_THROW(validate_candidate_qualification(
                   "NOT_PROVEN", "SIM_TRAINING_ONLY",
                   "fixed_home_trajectory_certified_bank_v4"),
               std::runtime_error);
  EXPECT_THROW(validate_candidate_qualification(
                   "NOT_PROVEN", "SIM_TRAINING_ONLY",
                   "fixed_home_support_candidate_bank_v4", "bad-bank-sha"),
               std::runtime_error);
  EXPECT_THROW(validate_candidate_qualification(
                   "NOT_PROVEN", "SIM_TRAINING_ONLY",
                   "fixed_home_support_candidate_bank_v4",
                   kSchema29QuestionBankSha256, "bad-receipt-sha"),
               std::runtime_error);

  EXPECT_NO_THROW(validate_hitter_pingpong_schema29_retention_metadata(
      "hitter_pingpong_build2_trainable_home_optional_reach_v1",
      kSchema29SideRehearsalContract, kSchema29TeacherRetentionContract));
  EXPECT_THROW(validate_hitter_pingpong_schema29_retention_metadata(
                   "hitter_pingpong_build2_trainable_home_optional_reach_v1",
                   "build2_side_prior,fixed_home,no_build1_donor",
                   kSchema29TeacherRetentionContract),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pingpong_schema29_retention_metadata(
                   "hitter_pingpong_build2_trainable_home_optional_reach_v1",
                   kSchema29SideRehearsalContract,
                   "disabled_standard_ppo"),
               std::runtime_error);

  EXPECT_NO_THROW(validate_hitter_pingpong_schema29_mocap_stale_metadata(
      "hitter_pingpong_build2_trainable_home_optional_reach_v1",
      "localization_fresh_outage_empirical_v1",
      kSchema29MocapStaleReceiptSha256, "0.015090824406148",
      "3,1,4,12,9,15,11,12,8,7,16,27,19,15,3",
      kSchema29MocapStaleCurriculumContract));
  EXPECT_THROW(validate_hitter_pingpong_schema29_mocap_stale_metadata(
                   "hitter_pingpong_build2_trainable_home_optional_reach_v1",
                   "localization_fresh_outage_empirical_v1",
                   kSchema29MocapStaleReceiptSha256, "0",
                   "3,1,4,12,9,15,11,12,8,7,16,27,19,15,3",
                   kSchema29MocapStaleCurriculumContract),
               std::runtime_error);
}

TEST(PpRuntimeContract, Schema31CandidateIsExactAndDefaultsToLevelZero) {
  const std::string receipt =
      "model_21800.pt,sha256="
      "daca62616178177647cf9949128582785529dbe6b7ba080fc96d86da23916561,"
      "source_input_dim=110,destination_input_dim=112,"
      "zeroed_source_cols=76|81,appended_zero_cols=110|111,trainable_full31=true";
  EXPECT_TRUE(hitter_schema31_initialization_tuple(
      kSchema31InitializationContract, receipt));
  EXPECT_NO_THROW(validate_hitter_pingpong_initialization_contract(
      kSchema31TrainingRecipe, kSchema31InitializationContract, receipt));
  EXPECT_THROW(validate_hitter_pingpong_initialization_contract(
                   kSchema31TrainingRecipe,
                   "schema29_from_build1_model21800_functional_vxy_zero_append_"
                   "reach_trainable_v1",
                   receipt),
               std::runtime_error);

  const auto mode = validate_hitter_pingpong_schema27_nonprivileged_metadata(
      kSchema31TrainingRecipe, "hitter_pure_112_headslots_vxy_reach_v1",
      kSchema31ActorArchitectureContract,
      "fresh_schema3_flight_shared_session_home_support_intent_v2", "110,111",
      "planner_target_tuple,planner_target_tuple",
      "planner_tuple_no_plant_inference_v3", kSchema31OptionalReachContract,
      "true", kSchema31OptionalReachContract, kSchema31RewardContract,
      kSchema31RecoveryPhaseContract);
  EXPECT_EQ(mode, HitterOptionalReachRuntimeMode::kCandidateSchema3Level0Only);
  EXPECT_TRUE(hitter_optional_reach_wire_is_compatible(
      true, mode, 3, 0.0, 0.0));
  EXPECT_FALSE(hitter_optional_reach_wire_is_compatible(
      true, mode, 3, 1.0, -1.0));

  EXPECT_NO_THROW(validate_hitter_pingpong_schema31_single_output_metadata(
      kSchema31TrainingRecipe,
      "policy_owned_112d_markov_trainable_no_qdes_override_v1",
      kSchema31InitializationContract,
      "hitter_pure_112_headslots_vxy_reach_v1",
      kSchema31ActorArchitectureContract, "actions31_v1", true, false));
  EXPECT_NO_THROW(
      validate_hitter_pingpong_schema31_candidate_qualification_metadata(
          kSchema31TrainingRecipe, "true", kSchema31OptionalReachContract,
          kSchema31OptionalReachTrajectoryQualification,
          kSchema31OptionalReachDeploymentQualification));
  EXPECT_NO_THROW(validate_hitter_pingpong_schema31_question_bank_metadata(
      kSchema31TrainingRecipe,
      "fixed_home_support_training_candidate_bank_v5_axis_envelope_"
      "prefilter_not_trajectory_certification_v1",
      "NOT_PROVEN_axis_prefilter_only", kSchema31QuestionBankArtifactContract,
      kSchema31QuestionBankSha256, kSchema31QuestionBankReceiptSha256,
      "fixed_home_coherent_question_bank_v1,device_discrete_row_gather_v1,"
      "SIM_TRAINING_CANDIDATE,"
      "SIM_TRAINING_CANDIDATE_NOT_DEPLOYMENT_QUALIFIED",
      "SIM_TRAINING_CANDIDATE",
      kSchema31OptionalReachDeploymentQualification, "NOT_PROVEN",
      "NOT_PROVEN"));
  EXPECT_THROW(validate_hitter_pingpong_schema31_question_bank_metadata(
                   kSchema31TrainingRecipe,
                   "fixed_home_support_training_candidate_bank_v5_axis_envelope_"
                   "prefilter_not_trajectory_certification_v1",
                   "NOT_PROVEN_axis_prefilter_only",
                   kSchema31QuestionBankArtifactContract, "wrong-bank-hash",
                   kSchema31QuestionBankReceiptSha256,
                   "fixed_home_coherent_question_bank_v1,"
                   "device_discrete_row_gather_v1,SIM_TRAINING_CANDIDATE,"
                   "SIM_TRAINING_CANDIDATE_NOT_DEPLOYMENT_QUALIFIED",
                   "SIM_TRAINING_CANDIDATE",
                   kSchema31OptionalReachDeploymentQualification,
                   "NOT_PROVEN", "NOT_PROVEN"),
               std::runtime_error);
  EXPECT_EQ(hitter_pingpong_ready_hold_steps_contract(kSchema31TrainingRecipe),
            (std::array<double, 2>{150.0, 250.0}));
  EXPECT_NO_THROW(validate_hitter_pingpong_command_recipe_contract(
      kSchema31TrainingRecipe, "fixed_home_fresh_flight_preempt_v1"));
  EXPECT_THROW(validate_hitter_pingpong_command_recipe_contract(
                   kSchema31TrainingRecipe,
                   "postcontact_fresh_flight_preempt_v1"),
               std::runtime_error);
  EXPECT_EQ(validate_hitter_pure_runtime_contract(
                "rally_final_v2", kSchema31TrainingRecipe),
            HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_THROW(validate_hitter_pure_runtime_contract(
                   "rally_final_v1", kSchema31TrainingRecipe),
               std::runtime_error);
}

TEST(PpRuntimeContract, Schema32CandidateHasDistinctRecoveryAndActionIdentity) {
  const std::string receipt =
      "model_21800.pt,sha256="
      "daca62616178177647cf9949128582785529dbe6b7ba080fc96d86da23916561,"
      "source_input_dim=110,destination_input_dim=112,"
      "zeroed_source_cols=76|81,appended_zero_cols=110|111,trainable_full31=true";
  EXPECT_TRUE(hitter_schema32_initialization_tuple(
      kSchema32InitializationContract, receipt));
  EXPECT_FALSE(hitter_schema31_initialization_tuple(
      kSchema32InitializationContract, receipt));
  EXPECT_NO_THROW(validate_hitter_pingpong_initialization_contract(
      kSchema32TrainingRecipe, kSchema32InitializationContract, receipt));
  EXPECT_THROW(validate_hitter_pingpong_initialization_contract(
                   kSchema32TrainingRecipe, kSchema31InitializationContract,
                   receipt),
               std::runtime_error);

  const auto mode = validate_hitter_pingpong_schema27_nonprivileged_metadata(
      kSchema32TrainingRecipe, "hitter_pure_112_headslots_vxy_reach_v1",
      kSchema31ActorArchitectureContract,
      "fresh_schema3_flight_shared_session_home_support_intent_v2", "110,111",
      "planner_target_tuple,planner_target_tuple",
      "planner_tuple_no_plant_inference_v3", kSchema31OptionalReachContract,
      "true", kSchema31OptionalReachContract, kSchema31RewardContract,
      kSchema31RecoveryPhaseContract);
  EXPECT_EQ(mode, HitterOptionalReachRuntimeMode::kCandidateSchema3Level0Only);

  EXPECT_NO_THROW(validate_hitter_pingpong_schema32_single_output_metadata(
      kSchema32TrainingRecipe, kSchema32TransitionContract,
      kSchema32InitializationContract,
      "hitter_pure_112_headslots_vxy_reach_v1",
      kSchema31ActorArchitectureContract, "actions31_v1", true, false));
  EXPECT_THROW(validate_hitter_pingpong_schema32_single_output_metadata(
                   kSchema32TrainingRecipe,
                   "policy_owned_112d_markov_trainable_no_qdes_override_v1",
                   kSchema32InitializationContract,
                   "hitter_pure_112_headslots_vxy_reach_v1",
                   kSchema31ActorArchitectureContract, "actions31_v1", true,
                   false),
               std::runtime_error);
  EXPECT_NO_THROW(
      validate_hitter_pingpong_schema31_candidate_qualification_metadata(
          kSchema32TrainingRecipe, "true", kSchema31OptionalReachContract,
          kSchema31OptionalReachTrajectoryQualification,
          kSchema31OptionalReachDeploymentQualification));
  EXPECT_NO_THROW(validate_hitter_pingpong_schema31_question_bank_metadata(
      kSchema32TrainingRecipe,
      "fixed_home_support_training_candidate_bank_v5_axis_envelope_"
      "prefilter_not_trajectory_certification_v1",
      "NOT_PROVEN_axis_prefilter_only", kSchema31QuestionBankArtifactContract,
      kSchema31QuestionBankSha256, kSchema31QuestionBankReceiptSha256,
      "fixed_home_coherent_question_bank_v1,device_discrete_row_gather_v1,"
      "SIM_TRAINING_CANDIDATE,"
      "SIM_TRAINING_CANDIDATE_NOT_DEPLOYMENT_QUALIFIED",
      "SIM_TRAINING_CANDIDATE",
      kSchema31OptionalReachDeploymentQualification, "NOT_PROVEN",
      "NOT_PROVEN"));
  EXPECT_EQ(hitter_pingpong_ready_hold_steps_contract(kSchema32TrainingRecipe),
            (std::array<double, 2>{150.0, 250.0}));
  EXPECT_NO_THROW(validate_hitter_pingpong_command_recipe_contract(
      kSchema32TrainingRecipe, "fixed_home_fresh_flight_preempt_v1"));
  EXPECT_EQ(validate_hitter_pure_runtime_contract(
                "rally_final_v2", kSchema32TrainingRecipe),
            HitterPureRuntimeContract::kRallyFinalV2);
}

TEST(PpRuntimeContract, Schema33CandidateKeepsTheV12AbiWithDistinctIdentity) {
  const std::string receipt =
      "model_5580.pt,sha256="
      "79423210ecbe3eb7e863ae4bf196eb22fb445d1f6c942796131d4a79e0e311e0,"
      "source_input_dim=112,destination_input_dim=112,"
      "direct_actor_state=true,trainable_full31=true";
  EXPECT_TRUE(hitter_schema33_initialization_tuple(
      kSchema33InitializationContract, receipt));
  EXPECT_FALSE(hitter_schema32_initialization_tuple(
      kSchema33InitializationContract, receipt));
  EXPECT_TRUE(hitter_schema31_candidate_family_recipe(kSchema33TrainingRecipe));
  EXPECT_TRUE(hitter_schema32_slew_family_recipe(kSchema33TrainingRecipe));
  EXPECT_FALSE(hitter_schema32_slew_family_recipe(kSchema31TrainingRecipe));
  EXPECT_NO_THROW(validate_hitter_pingpong_initialization_contract(
      kSchema33TrainingRecipe, kSchema33InitializationContract, receipt));
  // A Schema33 recipe must not accept the Schema32 functional-migration provenance.
  EXPECT_THROW(validate_hitter_pingpong_initialization_contract(
                   kSchema33TrainingRecipe, kSchema32InitializationContract,
                   "model_21800.pt,sha256="
                   "daca62616178177647cf9949128582785529dbe6b7ba080fc96d86da23916561,"
                   "source_input_dim=110,destination_input_dim=112,"
                   "zeroed_source_cols=76|81,appended_zero_cols=110|111,"
                   "trainable_full31=true"),
               std::runtime_error);

  // Reward/recovery identity moved to the licensed-replant v7 pair; the Schema31 pair must
  // now be rejected for a Schema33 recipe and vice versa.
  const auto mode = validate_hitter_pingpong_schema27_nonprivileged_metadata(
      kSchema33TrainingRecipe, "hitter_pure_112_headslots_vxy_reach_v1",
      kSchema31ActorArchitectureContract,
      "fresh_schema3_flight_shared_session_home_support_intent_v2", "110,111",
      "planner_target_tuple,planner_target_tuple",
      "planner_tuple_no_plant_inference_v3", kSchema31OptionalReachContract,
      "true", kSchema31OptionalReachContract, kSchema33RewardContract,
      kSchema33RecoveryPhaseContract);
  EXPECT_EQ(mode, HitterOptionalReachRuntimeMode::kCandidateSchema3Level0Only);
  EXPECT_THROW(validate_hitter_pingpong_schema27_nonprivileged_metadata(
                   kSchema33TrainingRecipe, "hitter_pure_112_headslots_vxy_reach_v1",
                   kSchema31ActorArchitectureContract,
                   "fresh_schema3_flight_shared_session_home_support_intent_v2",
                   "110,111", "planner_target_tuple,planner_target_tuple",
                   "planner_tuple_no_plant_inference_v3",
                   kSchema31OptionalReachContract, "true",
                   kSchema31OptionalReachContract, kSchema31RewardContract,
                   kSchema31RecoveryPhaseContract),
               std::runtime_error);

  EXPECT_NO_THROW(validate_hitter_pingpong_schema32_single_output_metadata(
      kSchema33TrainingRecipe, kSchema32TransitionContract,
      kSchema33InitializationContract,
      "hitter_pure_112_headslots_vxy_reach_v1",
      kSchema31ActorArchitectureContract, "actions31_v1", true, false));
  EXPECT_THROW(validate_hitter_pingpong_schema32_single_output_metadata(
                   kSchema33TrainingRecipe, kSchema32TransitionContract,
                   kSchema32InitializationContract,
                   "hitter_pure_112_headslots_vxy_reach_v1",
                   kSchema31ActorArchitectureContract, "actions31_v1", true,
                   false),
               std::runtime_error);
  EXPECT_NO_THROW(
      validate_hitter_pingpong_schema31_candidate_qualification_metadata(
          kSchema33TrainingRecipe, "true", kSchema31OptionalReachContract,
          kSchema31OptionalReachTrajectoryQualification,
          kSchema31OptionalReachDeploymentQualification));
  EXPECT_EQ(hitter_pingpong_ready_hold_steps_contract(kSchema33TrainingRecipe),
            (std::array<double, 2>{150.0, 250.0}));
  EXPECT_NO_THROW(validate_hitter_pingpong_command_recipe_contract(
      kSchema33TrainingRecipe, "fixed_home_fresh_flight_preempt_v1"));
  EXPECT_EQ(validate_hitter_pure_runtime_contract(
                "rally_final_v2", kSchema33TrainingRecipe),
            HitterPureRuntimeContract::kRallyFinalV2);
}

TEST(PpRuntimeContract, Schema34RequiresSignedSupportSemanticsAndZeroedSourceColumns) {
  const std::string receipt =
      "model_2130.pt,sha256="
      "0df28d798b4bec2af47fbea1e62fcb1371efdc292539f3d08777b63967ac18e3,"
      "source_input_dim=112,destination_input_dim=112,"
      "zeroed_semantic_cols=110|111,"
      "input_transform=zero_semantically_reassigned_support_columns_110_111_v1,"
      "trainable_full31=true";
  EXPECT_TRUE(hitter_schema34_initialization_tuple(
      kSchema34InitializationContract, receipt));
  EXPECT_TRUE(hitter_schema31_candidate_family_recipe(kSchema34TrainingRecipe));
  EXPECT_TRUE(hitter_schema32_slew_family_recipe(kSchema34TrainingRecipe));
  EXPECT_NO_THROW(validate_hitter_pingpong_initialization_contract(
      kSchema34TrainingRecipe, kSchema34InitializationContract, receipt));
  EXPECT_THROW(validate_hitter_pingpong_initialization_contract(
                   kSchema34TrainingRecipe, kSchema33InitializationContract,
                   receipt),
               std::runtime_error);

  const auto mode = validate_hitter_pingpong_schema27_nonprivileged_metadata(
      kSchema34TrainingRecipe, kSchema34ActorObsContract,
      kSchema31ActorArchitectureContract,
      "fresh_schema3_flight_shared_session_home_support_intent_v2", "110,111",
      kSchema34ActorReachSources, kSchema34NonprivilegedContract,
      kSchema34ReachPermissionContract, "true", kSchema31OptionalReachContract,
      kSchema34RewardContract, kSchema34RecoveryPhaseContract);
  EXPECT_EQ(mode, HitterOptionalReachRuntimeMode::kCandidateSchema3Level0Only);
  EXPECT_THROW(validate_hitter_pingpong_schema27_nonprivileged_metadata(
                   kSchema34TrainingRecipe,
                   "hitter_pure_112_headslots_vxy_reach_v1",
                   kSchema31ActorArchitectureContract,
                   "fresh_schema3_flight_shared_session_home_support_intent_v2",
                   "110,111", "planner_target_tuple,planner_target_tuple",
                   "planner_tuple_no_plant_inference_v3",
                   kSchema31OptionalReachContract, "true",
                   kSchema31OptionalReachContract, kSchema33RewardContract,
                   kSchema33RecoveryPhaseContract),
               std::runtime_error);

  EXPECT_NO_THROW(validate_hitter_pingpong_schema32_single_output_metadata(
      kSchema34TrainingRecipe, kSchema34TransitionContract,
      kSchema34InitializationContract, kSchema34ActorObsContract,
      kSchema31ActorArchitectureContract, "actions31_v1", true, false));
  EXPECT_THROW(validate_hitter_pingpong_schema32_single_output_metadata(
                   kSchema34TrainingRecipe, kSchema32TransitionContract,
                   kSchema34InitializationContract, kSchema34ActorObsContract,
                   kSchema31ActorArchitectureContract, "actions31_v1", true,
                   false),
               std::runtime_error);
  EXPECT_NO_THROW(validate_hitter_pingpong_command_recipe_contract(
      kSchema34TrainingRecipe, "fixed_home_fresh_flight_preempt_v1"));
  EXPECT_EQ(validate_hitter_pure_runtime_contract(
                "rally_final_v2", kSchema34TrainingRecipe),
            HitterPureRuntimeContract::kRallyFinalV2);
}

TEST(PpRuntimeContract, Schema34BalanceRevisionBindsRequestedSourceAndMatchingOwners) {
  const std::string receipt =
      "model_5580.pt,sha256="
      "79423210ecbe3eb7e863ae4bf196eb22fb445d1f6c942796131d4a79e0e311e0,"
      "source_input_dim=112,destination_input_dim=112,"
      "zeroed_semantic_cols=110|111,"
      "input_transform=zero_semantically_reassigned_support_columns_110_111_v1,"
      "trainable_full31=true";
  EXPECT_TRUE(hitter_schema34_initialization_tuple(kSchema34BalanceInitializationContract, receipt));
  EXPECT_FALSE(hitter_schema34_initialization_tuple(kSchema34InitializationContract, receipt));
  auto check = [](const char* reward, const char* recovery) {
    return validate_hitter_pingpong_schema27_nonprivileged_metadata(
        kSchema34TrainingRecipe, kSchema34ActorObsContract, kSchema31ActorArchitectureContract,
        "fresh_schema3_flight_shared_session_home_support_intent_v2", "110,111",
        kSchema34ActorReachSources, kSchema34NonprivilegedContract,
        kSchema34ReachPermissionContract, "true", kSchema31OptionalReachContract, reward, recovery);
  };
  EXPECT_NO_THROW(check(kSchema34BalanceRewardContract, kSchema34BalanceRecoveryContract));
  EXPECT_THROW(check(kSchema34BalanceRewardContract, kSchema34RecoveryPhaseContract), std::runtime_error);
  EXPECT_THROW(check(kSchema34RewardContract, kSchema34BalanceRecoveryContract), std::runtime_error);
  EXPECT_NO_THROW(validate_hitter_pingpong_schema32_single_output_metadata(
      kSchema34TrainingRecipe, kSchema34TransitionContract,
      kSchema34BalanceInitializationContract, kSchema34ActorObsContract,
      kSchema31ActorArchitectureContract, "actions31_v1", true, false));
}

TEST(PpRuntimeContract, Schema34StepTaskRevisionBindsSourceAndRejectsOldObservation) {
  const std::string receipt = "model_4710.pt,sha256=c7216a9358452dee1f38a5232a1c08e2beb6cdc2bd4ab785f003b6e3b3207398,source_input_dim=112,destination_input_dim=112,zeroed_semantic_cols=14|19|45|50|111,input_transform=zero_semantically_reassigned_task_columns_14_19_45_50_111_v1,trainable_full31=true";
  EXPECT_TRUE(hitter_schema34_initialization_tuple(kSchema34StepInitializationContract, receipt));
  EXPECT_FALSE(hitter_schema34_initialization_tuple(kSchema34BalanceInitializationContract, receipt));
  auto check = [](const char* obs, const char* reward) {
    return validate_hitter_pingpong_schema27_nonprivileged_metadata(
        kSchema34TrainingRecipe, obs, kSchema31ActorArchitectureContract,
        "fresh_schema3_flight_shared_session_home_support_intent_v2", "110,111",
        kSchema34StepActorReachSources, kSchema34StepNonprivilegedContract,
        kSchema34StepReachPermissionContract, "true", kSchema31OptionalReachContract,
        reward, kSchema34StepRecoveryContract);
  };
  EXPECT_NO_THROW(check(kSchema34StepActorObsContract, kSchema34StepRewardContract));
  EXPECT_THROW(check(kSchema34ActorObsContract, kSchema34StepRewardContract), std::runtime_error);
  EXPECT_THROW(check(kSchema34StepActorObsContract, kSchema34BalanceRewardContract), std::runtime_error);
  EXPECT_NO_THROW(validate_hitter_pingpong_schema32_single_output_metadata(
      kSchema34TrainingRecipe, kSchema34TransitionContract, kSchema34StepInitializationContract,
      kSchema34StepActorObsContract, kSchema31ActorArchitectureContract, "actions31_v1", true, false));
  EXPECT_THROW(validate_hitter_pingpong_schema32_single_output_metadata(
      kSchema34TrainingRecipe, kSchema34TransitionContract, kSchema34StepInitializationContract,
      kSchema34ActorObsContract, kSchema31ActorArchitectureContract, "actions31_v1", true, false), std::runtime_error);
}

TEST(PpRuntimeContract, AcceptsOnlyPairedRuntimeV2AndComponentRecipes) {
  EXPECT_EQ(validate_hitter_pure_runtime_contract("rally_final_v2", "rally_v10"),
            HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_EQ(validate_hitter_pure_runtime_contract("rally_final_v2", "rally_v11"),
            HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_EQ(validate_hitter_pure_runtime_contract("rally_final_v2", "rally_v12"),
            HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_EQ(validate_hitter_pure_runtime_contract("rally_final_v2", "rally_v13"),
            HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_EQ(validate_hitter_pure_runtime_contract("rally_final_v2", "rally_v14"),
            HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_EQ(
      validate_hitter_pure_runtime_contract(
          "rally_final_v2", "hitter_pingpong_recovery_tts_v3"),
      HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_EQ(
      validate_hitter_pure_runtime_contract(
          "rally_final_v2", "hitter_pingpong_recovery_tts_v2"),
      HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_EQ(
      validate_hitter_pure_runtime_contract(
          "rally_final_v2", "hitter_pingpong_continuous_rally_v1"),
      HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_EQ(
      validate_hitter_pure_runtime_contract(
          "rally_final_v2", "hitter_pingpong_continuous_rally_v2"),
      HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_EQ(
      validate_hitter_pure_runtime_contract(
          "rally_final_v2", "hitter_pingpong_continuous_rally_v3"),
      HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_EQ(
      validate_hitter_pure_runtime_contract(
          "rally_final_v2", "hitter_pingpong_continuous_rally_v4"),
      HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_EQ(
      validate_hitter_pure_runtime_contract(
          "rally_final_v2", "hitter_pingpong_build2_minimal_rootfix_v1"),
      HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_EQ(
      validate_hitter_pure_runtime_contract(
          "rally_final_v2", "hitter_pingpong_build2_rapid_preempt_v1"),
      HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_EQ(
      validate_hitter_pure_runtime_contract(
          "rally_final_v2", "hitter_pingpong_build2_fixed_home_recovery_v1"),
      HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_EQ(
      validate_hitter_pure_runtime_contract(
          "rally_final_v2", "hitter_pingpong_build2_fixed_home_feasible_v1"),
      HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_EQ(
      validate_hitter_pure_runtime_contract(
          "rally_final_v2",
          "hitter_pingpong_build2_fixed_home_phase_recovery_v1"),
      HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_EQ(
      validate_hitter_pure_runtime_contract(
          "rally_final_v2", "hitter_pingpong_build2_home_support_frame_v1"),
      HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_EQ(
      validate_hitter_pure_runtime_contract(
          "rally_final_v2",
          "hitter_pingpong_build2_frozen_core_optional_reach_v1"),
      HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_EQ(validate_hitter_pure_runtime_contract("rally_final_v2", "rally_v17"),
            HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_NO_THROW(validate_hitter_pure_runtime_contract("rally_final_v2", "rally_v9"));
  EXPECT_NO_THROW(validate_hitter_pure_runtime_contract("rally_final_v2", "future_training_name"));
  EXPECT_THROW(validate_hitter_pure_runtime_contract("unknown_wire", "future_training_name"),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pure_runtime_contract("rally_final_v1", "rally_v10"),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pure_runtime_contract("rally_final_v1", "rally_v11"),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pure_runtime_contract("rally_final_v1", "rally_v12"),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pure_runtime_contract("rally_final_v1", "rally_v13"),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pure_runtime_contract("rally_final_v1", "rally_v14"),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pure_runtime_contract("rally_final_v1", "rally_v17"),
               std::runtime_error);
  EXPECT_THROW(
      validate_hitter_pure_runtime_contract(
          "rally_final_v1", "hitter_pingpong_recovery_tts_v3"),
      std::runtime_error);
  EXPECT_THROW(validate_hitter_pure_runtime_contract("", "rally_v10"),
               std::runtime_error);
}

TEST(PpRuntimeContract, RepairRecipeAndLateRevealV2AreBidirectionallyPaired) {
  EXPECT_EQ(
      std::string(hitter_pingpong_entry_tts_buckets_contract(
          "strike_followthrough_home_external_commit_v2")),
      "nominal;0.45,0.55;0.40,0.45;0.25,0.35;0.15,0.25;0.45,0.82");
  EXPECT_EQ(
      std::string(hitter_pingpong_entry_tts_buckets_contract(
          "strike_followthrough_home_external_commit_v1")),
      "nominal;0.45,0.55;0.40,0.45;0.25,0.35;0.15,0.25");
  EXPECT_EQ(
      std::string(hitter_pingpong_entry_tts_buckets_contract(
          "strike_followthrough_home_external_commit_v3")),
      "nominal;0.45,0.55;0.40,0.45;0.25,0.35;0.15,0.25;0.45,0.82");
  EXPECT_EQ(
      std::string(hitter_pingpong_entry_tts_buckets_contract(
          "strike_followthrough_home_external_commit_v4")),
      "nominal;0.45,0.55;0.40,0.45;0.25,0.35;0.15,0.25;0.45,0.82");
  EXPECT_EQ(
      std::string(hitter_pingpong_entry_tts_buckets_contract(
          "native_clip_end_immediate_resample_v1")),
      "nominal;0.45,0.55;0.40,0.45;0.25,0.35;0.15,0.25");
  EXPECT_EQ(
      std::string(hitter_pingpong_entry_tts_buckets_contract(
          "postcontact_fresh_flight_preempt_v1")),
      "nominal;0.45,0.55;0.35,0.45;0.25,0.35;0.15,0.25;0.55,0.82");
  EXPECT_EQ(
      hitter_pingpong_ready_hold_steps_contract(
          "hitter_pingpong_continuous_rally_v1"),
      (std::array<double, 2>{45.0, 60.0}));
  EXPECT_EQ(
      hitter_pingpong_ready_hold_steps_contract(
          "hitter_pingpong_continuous_rally_v2"),
      (std::array<double, 2>{45.0, 60.0}));
  EXPECT_EQ(
      hitter_pingpong_ready_hold_steps_contract(
          "hitter_pingpong_continuous_rally_v3"),
      (std::array<double, 2>{0.0, 0.0}));
  EXPECT_EQ(
      hitter_pingpong_ready_hold_steps_contract(
          "hitter_pingpong_continuous_rally_v4"),
      (std::array<double, 2>{0.0, 0.0}));
  EXPECT_EQ(
      hitter_pingpong_ready_hold_steps_contract(
          "hitter_pingpong_build2_minimal_rootfix_v1"),
      (std::array<double, 2>{45.0, 60.0}));
  EXPECT_EQ(
      hitter_pingpong_ready_hold_steps_contract(
          "hitter_pingpong_build2_rapid_preempt_v1"),
      (std::array<double, 2>{45.0, 60.0}));
  EXPECT_THROW(
      hitter_pingpong_ready_hold_steps_contract("hitter_pingpong_unknown"),
      std::runtime_error);
  EXPECT_NO_THROW(validate_hitter_pingpong_command_recipe_contract(
      "hitter_pingpong_recovery_tts_v3", "late_reveal_raw_tts_v2"));
  EXPECT_THROW(validate_hitter_pingpong_command_recipe_contract(
                   "hitter_pingpong_recovery_tts_v3", "late_reveal_raw_tts_v1"),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pingpong_command_recipe_contract(
                   "hitter_pingpong_recovery_tts_v3", ""),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pingpong_command_recipe_contract(
                   "hitter_pingpong_recovery_tts_v2", "late_reveal_raw_tts_v2"),
               std::runtime_error);
  EXPECT_NO_THROW(validate_hitter_pingpong_command_recipe_contract(
      "hitter_pingpong_recovery_tts_v2", "home_preempt_raw_tts_v3"));
  EXPECT_NO_THROW(validate_hitter_pingpong_command_recipe_contract(
      "hitter_pingpong_continuous_rally_v1",
      "strike_followthrough_home_external_commit_v1"));
  EXPECT_NO_THROW(validate_hitter_pingpong_command_recipe_contract(
      "hitter_pingpong_continuous_rally_v2",
      "strike_followthrough_home_external_commit_v2"));
  EXPECT_NO_THROW(validate_hitter_pingpong_command_recipe_contract(
      "hitter_pingpong_continuous_rally_v3",
      "strike_followthrough_home_external_commit_v3"));
  EXPECT_NO_THROW(validate_hitter_pingpong_command_recipe_contract(
      "hitter_pingpong_continuous_rally_v4",
      "strike_followthrough_home_external_commit_v4"));
  EXPECT_NO_THROW(validate_hitter_pingpong_command_recipe_contract(
      "hitter_pingpong_build2_minimal_rootfix_v1",
      "native_clip_end_immediate_resample_v1"));
  EXPECT_NO_THROW(validate_hitter_pingpong_command_recipe_contract(
      "hitter_pingpong_build2_rapid_preempt_v1",
      "postcontact_fresh_flight_preempt_v1"));
  EXPECT_THROW(validate_hitter_pingpong_command_recipe_contract(
                   "hitter_pingpong_build2_minimal_rootfix_v1",
                   "postcontact_fresh_flight_preempt_v1"),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pingpong_command_recipe_contract(
                   "hitter_pingpong_build2_rapid_preempt_v1",
                   "native_clip_end_immediate_resample_v1"),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pingpong_command_recipe_contract(
                   "hitter_pingpong_build2_minimal_rootfix_v1",
                   "strike_followthrough_home_external_commit_v2"),
               std::runtime_error);
  const std::array<std::string, 4> continuous_recipes = {
      "hitter_pingpong_continuous_rally_v1",
      "hitter_pingpong_continuous_rally_v2",
      "hitter_pingpong_continuous_rally_v3",
      "hitter_pingpong_continuous_rally_v4"};
  const std::array<std::string, 4> continuous_commands = {
      "strike_followthrough_home_external_commit_v1",
      "strike_followthrough_home_external_commit_v2",
      "strike_followthrough_home_external_commit_v3",
      "strike_followthrough_home_external_commit_v4"};
  for (std::size_t recipe = 0; recipe < continuous_recipes.size(); ++recipe) {
    for (std::size_t command = 0; command < continuous_commands.size(); ++command) {
      if (recipe == command) continue;
      EXPECT_THROW(validate_hitter_pingpong_command_recipe_contract(
                       continuous_recipes[recipe], continuous_commands[command]),
                   std::runtime_error)
          << "recipe index " << recipe << " unexpectedly accepted command index "
          << command;
    }
  }
  EXPECT_THROW(validate_hitter_pingpong_command_recipe_contract(
                   "hitter_pingpong_continuous_rally_v1",
                   "home_preempt_raw_tts_v3"),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pingpong_command_recipe_contract(
                   "hitter_pingpong_recovery_tts_v3", "home_preempt_raw_tts_v3"),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pingpong_command_recipe_contract(
                   "hitter_pingpong_recovery_tts_v3",
                   "strike_followthrough_home_external_commit_v2"),
               std::runtime_error);
}

TEST(PpRuntimeContract,
     Schema22EntryTtsBucketsAcceptCanonicalAndModel3440LegacyOnly) {
  constexpr const char* kSchema22 =
      "postcontact_fresh_flight_preempt_v1";
  constexpr const char* kCanonical =
      "nominal;0.45,0.55;0.35,0.45;0.25,0.35;0.15,0.25;0.55,0.82";
  constexpr const char* kModel3440Legacy =
      "nominal;0.45,0.55;0.40,0.45;0.25,0.35;0.15,0.25";

  EXPECT_TRUE(hitter_pingpong_entry_tts_buckets_metadata_compatible(
      kSchema22, kCanonical));
  EXPECT_TRUE(hitter_pingpong_entry_tts_buckets_metadata_compatible(
      kSchema22, kModel3440Legacy));
  EXPECT_FALSE(hitter_pingpong_entry_tts_buckets_metadata_compatible(
      kSchema22,
      "nominal;0.45,0.55;0.35,0.45;0.25,0.35;0.15,0.25"));
  EXPECT_FALSE(hitter_pingpong_entry_tts_buckets_metadata_compatible(
      "native_clip_end_immediate_resample_v1", kCanonical));
  EXPECT_TRUE(hitter_pingpong_entry_tts_buckets_metadata_compatible(
      "native_clip_end_immediate_resample_v1", kModel3440Legacy));
}

TEST(PpRuntimeContract, DeployRecipeVersionMustExactlyMatchOnnx) {
  EXPECT_NO_THROW(validate_deploy_training_recipe_pair(
      "hitter_pingpong_continuous_rally_v4", "4",
      "hitter_pingpong_continuous_rally_v4", "4"));
  EXPECT_THROW(validate_deploy_training_recipe_pair(
                   "hitter_pingpong_continuous_rally_v4", "4",
                   "hitter_pingpong_continuous_rally_v4", "3"),
               std::runtime_error);
  EXPECT_THROW(validate_deploy_training_recipe_pair(
                   "hitter_pingpong_continuous_rally_v4", "4",
                   "hitter_pingpong_continuous_rally_v3", "4"),
               std::runtime_error);

  EXPECT_NO_THROW(validate_deploy_training_recipe_pair(
      "hitter_pingpong_continuous_rally_v3", "3",
      "hitter_pingpong_continuous_rally_v3", "3"));
  EXPECT_THROW(validate_deploy_training_recipe_pair(
                   "hitter_pingpong_continuous_rally_v3", "3",
                   "hitter_pingpong_continuous_rally_v3", "2"),
               std::runtime_error);
  EXPECT_THROW(validate_deploy_training_recipe_pair(
                   "hitter_pingpong_continuous_rally_v3", "3",
                   "hitter_pingpong_continuous_rally_v3", ""),
               std::runtime_error);

  EXPECT_NO_THROW(validate_deploy_training_recipe_pair(
      "hitter_pingpong_continuous_rally_v2", "2",
      "hitter_pingpong_continuous_rally_v2", "2"));
  EXPECT_THROW(validate_deploy_training_recipe_pair(
                   "hitter_pingpong_continuous_rally_v2", "2",
                   "hitter_pingpong_continuous_rally_v2", "1"),
               std::runtime_error);
  EXPECT_THROW(validate_deploy_training_recipe_pair(
                   "hitter_pingpong_continuous_rally_v2", "2",
                   "hitter_pingpong_continuous_rally_v2", ""),
               std::runtime_error);

  EXPECT_NO_THROW(validate_deploy_training_recipe_pair(
      "hitter_pingpong_continuous_rally_v1", "1",
      "hitter_pingpong_continuous_rally_v1", "1"));
  EXPECT_THROW(validate_deploy_training_recipe_pair(
                   "hitter_pingpong_continuous_rally_v1", "1",
                   "hitter_pingpong_continuous_rally_v1", "2"),
               std::runtime_error);
}

TEST(PpRuntimeContract, SmallStationMatchedPlantV4Keeps112RuntimeAndRecipePair) {
  const std::string recipe = "hitter_small_station_matched_plant_recovery_v4";
  EXPECT_EQ(validate_hitter_pure_runtime_contract("rally_final_v2", recipe),
            HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_THROW(validate_hitter_pure_runtime_contract("rally_final_v3", recipe),
               std::runtime_error);
  EXPECT_NO_THROW(validate_hitter_pingpong_command_recipe_contract(
      recipe, "small_station_external_schedule_precommit_settle_v1"));
  EXPECT_THROW(validate_hitter_pingpong_command_recipe_contract(recipe, "unknown_command"),
               std::runtime_error);
  EXPECT_NO_THROW(validate_deploy_training_recipe_pair(recipe, "4", recipe, "4"));
  EXPECT_THROW(validate_deploy_training_recipe_pair(recipe, "4", recipe, "3"),
               std::runtime_error);
}

TEST(PpRuntimeContract, SmallStationCommandDoesNotDependOnTrainingProvenance) {
  EXPECT_NO_THROW(validate_hitter_pingpong_command_recipe_contract(
      "future_reward_only_recipe", "small_station_external_schedule_precommit_settle_v1"));
  EXPECT_THROW(validate_hitter_pingpong_command_recipe_contract(
      "hitter_small_station_support_heading_recovery_v3", "unknown_command_v2"),
      std::runtime_error);
}

TEST(PpRuntimeContract, SmallStationHoldResumeV5Keeps112RuntimeAndRecipePair) {
  const std::string recipe = "hitter_small_station_hold_resume_recovery_v5";
  EXPECT_EQ(validate_hitter_pure_runtime_contract("rally_final_v2", recipe),
            HitterPureRuntimeContract::kRallyFinalV2);
  EXPECT_THROW(validate_hitter_pure_runtime_contract("rally_final_v3", recipe),
               std::runtime_error);
  EXPECT_NO_THROW(validate_hitter_pingpong_command_recipe_contract(
      recipe, "small_station_external_schedule_precommit_settle_v1"));
  EXPECT_THROW(validate_hitter_pingpong_command_recipe_contract(recipe, "unknown_command"),
               std::runtime_error);
  EXPECT_NO_THROW(validate_deploy_training_recipe_pair(recipe, "5", recipe, "5"));
  EXPECT_THROW(validate_deploy_training_recipe_pair(recipe, "5", recipe, "4"),
               std::runtime_error);
}

TEST(PpRuntimeContract, ContinuousInitializationProvenanceIsVersionPaired) {
  const std::string v1_contract =
      "build2_model7800_actor_std_raw_obs_fresh_critic_optimizer_v1";
  const std::string v1_receipt =
      "model_7800.pt,"
      "sha256=1e659a56b94955215bd851621f22e4f612f68ab5f25c7b61fce6398f8110a87d,"
      "exported_onnx_sha256=aa24c36b025ce208df70f797103de67ebbff7d374dbfafcc5cffb2a1cffbaefe";
  const std::string v2_contract =
      "build1_model21800_actor_fresh_yaml_std0p19_raw_obs_"
      "fresh_critic_optimizer_v1";
  const std::string v2_receipt =
      "model_21800.pt,"
      "sha256=daca62616178177647cf9949128582785529dbe6b7ba080fc96d86da23916561,"
      "exported_onnx_sha256=6bf1a2418f8538e23577a0153f2fe6a1e78dee91f41650a232259432a84a4dc8";
  const std::string v4_contract =
      "build1_model21800_functional_zero_vxy_cols76_81_v1";
  const std::string v27_contract =
      "schema26_model4470_append_zero_reach_cols110_111_v1";
  const std::string v27_receipt =
      "model_4470.pt,sha256="
      "912a5a3da26dab818e25ad369d78e5f620b589870e1074d48cdaa8cfcb536c28,"
      "source_input_dim=110,destination_input_dim=112,"
      "appended_zero_input_columns=110|111";
  const std::string v27_enabled_contract =
      "schema27_stage0_model4470_frozen_core_zero_residual_v4";
  const std::string v27_enabled_receipt =
      "model_4470.pt,sha256="
      "49feef14e8f895fa8e408abab6aafce09519dce6122cc81ba94632362874eebd,"
      "source_input_dim=112,destination_input_dim=112,"
      "frozen_core=true,zero_residuals=true";

  EXPECT_NO_THROW(validate_hitter_pingpong_initialization_contract(
      "hitter_pingpong_continuous_rally_v1", v1_contract, v1_receipt));
  EXPECT_THROW(validate_hitter_pingpong_initialization_contract(
                   "hitter_pingpong_continuous_rally_v1", v2_contract,
                   v2_receipt),
               std::runtime_error);
  EXPECT_NO_THROW(validate_hitter_pingpong_initialization_contract(
      "hitter_pingpong_continuous_rally_v2", v2_contract, v2_receipt));
  EXPECT_THROW(validate_hitter_pingpong_initialization_contract(
                   "hitter_pingpong_continuous_rally_v2", v1_contract,
                   v1_receipt),
               std::runtime_error);
  EXPECT_NO_THROW(validate_hitter_pingpong_initialization_contract(
      "hitter_pingpong_continuous_rally_v3", v2_contract, v2_receipt));
  EXPECT_THROW(validate_hitter_pingpong_initialization_contract(
                   "hitter_pingpong_continuous_rally_v3", v1_contract,
                   v1_receipt),
               std::runtime_error);
  EXPECT_NO_THROW(validate_hitter_pingpong_initialization_contract(
      "hitter_pingpong_continuous_rally_v4", v4_contract, v2_receipt));
  EXPECT_NO_THROW(validate_hitter_pingpong_initialization_contract(
      "hitter_pingpong_build2_minimal_rootfix_v1", v2_contract, v2_receipt));
  EXPECT_NO_THROW(validate_hitter_pingpong_initialization_contract(
      "hitter_pingpong_build2_rapid_preempt_v1", v2_contract, v2_receipt));
  EXPECT_NO_THROW(validate_hitter_pingpong_initialization_contract(
      "hitter_pingpong_build2_home_support_frame_v1", v4_contract, v2_receipt));
  EXPECT_THROW(validate_hitter_pingpong_initialization_contract(
                   "hitter_pingpong_build2_home_support_frame_v1", v2_contract,
                   v2_receipt),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pingpong_initialization_contract(
                   "hitter_pingpong_build2_minimal_rootfix_v1", v4_contract,
                   v2_receipt),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pingpong_initialization_contract(
                   "hitter_pingpong_build2_rapid_preempt_v1", v4_contract,
                   v2_receipt),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pingpong_initialization_contract(
                   "hitter_pingpong_continuous_rally_v4", v2_contract,
                   v2_receipt),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pingpong_initialization_contract(
                   "hitter_pingpong_continuous_rally_v3", v4_contract,
                   v2_receipt),
               std::runtime_error);
  EXPECT_NO_THROW(validate_hitter_pingpong_initialization_contract(
      "hitter_pingpong_build2_torso_optional_reach_v2", v27_contract,
      v27_receipt));
  EXPECT_NO_THROW(validate_hitter_pingpong_initialization_contract(
      "hitter_pingpong_build2_torso_optional_reach_v2",
      v27_enabled_contract, v27_enabled_receipt));
  EXPECT_THROW(validate_hitter_pingpong_initialization_contract(
                   "hitter_pingpong_build2_torso_optional_reach_v2",
                   v27_enabled_contract, v27_receipt),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pingpong_initialization_contract(
                   "hitter_pingpong_build2_torso_optional_reach_v2",
                   v4_contract, v2_receipt),
               std::runtime_error);
  EXPECT_NO_THROW(validate_hitter_schema27_initialization_runtime_pair(
      v27_contract, v27_receipt,
      HitterOptionalReachRuntimeMode::kDisabledSchema2Zero));
  EXPECT_NO_THROW(validate_hitter_schema27_initialization_runtime_pair(
      v27_enabled_contract, v27_enabled_receipt,
      HitterOptionalReachRuntimeMode::kEnabledSchema3));
  EXPECT_THROW(validate_hitter_schema27_initialization_runtime_pair(
                   v27_contract, v27_receipt,
                   HitterOptionalReachRuntimeMode::kEnabledSchema3),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_schema27_initialization_runtime_pair(
                   v27_enabled_contract, v27_enabled_receipt,
                   HitterOptionalReachRuntimeMode::kDisabledSchema2Zero),
               std::runtime_error);
}

TEST(PpRuntimeContract, Build2MinimalRootfixMetadataIsOneExactSchema21Tuple) {
  struct Contract {
    std::string recipe = "hitter_pingpong_build2_minimal_rootfix_v1";
    std::string version = "1";
    std::string build = "build2_minimal_rootfix_schema21";
    std::string command = "native_clip_end_immediate_resample_v1";
    std::string planner = "native_clip_end_immediate_resample_v1";
    std::string actor_obs = "hitter_pure";
    std::string trajectory_teacher = "contact_linear_actor_visible_v1";
    std::string motion_phase = "signed_tts_contact_frame_v1";
    std::string racket_velocity = "sim_world_body_v1";
    std::string wait = "cold_hold_neutral_hidden_until_release_v1";
    std::array<double, 3> pending_target = {0.58, -0.265, 1.075};
    std::array<double, 3> pending_velocity = {0.0, 0.0, 0.0};
    std::array<double, 3> pending_normal = {1.0, 0.0, 0.0};
    std::string hide_command = "true";
    std::string contact_early_wrap = "false";
    std::string joint_aggregation = "rational_soft_union_all_joints_v1";
    std::string native_wrap_hold = "0,0";
    std::string post_contact_home =
        "contact_plus_0p12_latch_recovery_and_publish_immutable_home_base_target;"
        "preserve_racket_target_negative_tts_and_native_motion_tail;"
        "no_wrap_until_native_seg_end_v1";
    std::string home_return_reward =
        "yaml_owned_bounded_y_only_home_return_active_v1";
    std::string lifecycle =
        "cold_hold_only;native_clip_end_immediate_resample;no_contact_early_wrap;"
        "no_post_hit_arrival_hazard";
    std::string replay = "build2_progressive_0p05_to_0p20_v1";
  } exact;
  auto validate = [](const Contract& contract) {
    validate_hitter_pingpong_build2_minimal_rootfix_metadata(
        contract.recipe, contract.version, contract.build, contract.command,
        contract.planner, contract.actor_obs, contract.trajectory_teacher,
        contract.motion_phase, contract.racket_velocity, contract.wait, contract.pending_target,
        contract.pending_velocity,
        contract.pending_normal,
        contract.hide_command, contract.contact_early_wrap,
        contract.joint_aggregation, contract.native_wrap_hold,
        contract.post_contact_home, contract.home_return_reward,
        contract.lifecycle, contract.replay);
  };
  EXPECT_NO_THROW(validate(exact));

  auto invalid = exact;
  invalid.pending_velocity[1] = 0.01;
  EXPECT_THROW(validate(invalid), std::runtime_error);
  invalid = exact;
  invalid.pending_target[1] = -0.44;
  EXPECT_THROW(validate(invalid), std::runtime_error);
  invalid = exact;
  invalid.command = "strike_followthrough_home_external_commit_v2";
  EXPECT_THROW(validate(invalid), std::runtime_error);
  invalid = exact;
  invalid.actor_obs = "hitter_pure_110_headslots_vxy_v1";
  EXPECT_THROW(validate(invalid), std::runtime_error);
  invalid = exact;
  invalid.trajectory_teacher = "disabled_or_hidden_teacher";
  EXPECT_THROW(validate(invalid), std::runtime_error);
  invalid = exact;
  invalid.motion_phase = "legacy_shifted_reference_tts";
  EXPECT_THROW(validate(invalid), std::runtime_error);
  invalid = exact;
  invalid.racket_velocity = "mocap_vxy_world_racket_velocity_v1";
  EXPECT_THROW(validate(invalid), std::runtime_error);
  invalid = exact;
  invalid.pending_normal = {0.0, 1.0, 0.0};
  EXPECT_THROW(validate(invalid), std::runtime_error);
  invalid = exact;
  invalid.hide_command = "false";
  EXPECT_THROW(validate(invalid), std::runtime_error);
  invalid = exact;
  invalid.contact_early_wrap = "true";
  EXPECT_THROW(validate(invalid), std::runtime_error);
  invalid = exact;
  invalid.joint_aggregation = "legacy_topk_v1";
  EXPECT_THROW(validate(invalid), std::runtime_error);
  invalid = exact;
  invalid.native_wrap_hold = "45,60";
  EXPECT_THROW(validate(invalid), std::runtime_error);
  invalid = exact;
  invalid.post_contact_home =
      "contact_plus_0p12_early_wrap_and_replace_racket_target";
  EXPECT_THROW(validate(invalid), std::runtime_error);
  invalid = exact;
  invalid.home_return_reward = "positive_settle_income_or_xy_distance";
  EXPECT_THROW(validate(invalid), std::runtime_error);
  invalid = exact;
  invalid.lifecycle =
      "contact_plus_0p12_early_wrap;post_hit_memoryless_arrival";
  EXPECT_THROW(validate(invalid), std::runtime_error);
}

TEST(PpRuntimeContract, Schema34HomeReturnOwnerMustMatchRecipeRevision) {
  const auto check = [](const std::string& revision, const std::string& home_owner) {
    validate_hitter_pingpong_build2_rapid_preempt_metadata(
        kSchema34TrainingRecipe, revision, kSchema34BuildContract,
        "fixed_home_fresh_flight_preempt_v1",
        "fresh_schema3_flight_shared_session_home_support_intent_v2",
        kSchema34TransitionContract, 0.12, {0.60, 0.75}, {0.50, 0.65},
        "contact_phase_aligned_control_tick_closed_support_v1",
        {0.40, 0.82, 0.40, 0.55}, "disabled_fixed_session_home", "",
        kSchema34ReplayContract, "true", "false",
        "rational_soft_union_all_joints_v1", "0,0",
        "contact_plus_0p12_latch_recovery_only;station_remains_session_home;"
        "preserve_old_racket_target_negative_tts_and_motion_tail_until_fresh_flight_commit_v1",
        home_owner,
        "cold_hold_neutral;contact_plus_0p12_recovery_latch;fresh_external_flight_preempt_0p50_0p65;"
        "immutable_session_home;policy_owned_transition;no_ready_gate",
        "fixed_home_fresh_flight_preempt_v1", "fixed_session_home_v1",
        "immutable_home_support_frame_all_phases_v2", "immutable_home_minus_mocap_base_v1");
  };
  EXPECT_NO_THROW(check("3", kSchema34StepRecoveryContract));
  EXPECT_NO_THROW(check("2", kSchema34BalanceRecoveryContract));
  EXPECT_NO_THROW(check("1", "continuous_immutable_home_support_frame_v1"));
  EXPECT_THROW(check("2", "continuous_immutable_home_support_frame_v1"), std::runtime_error);
  EXPECT_THROW(check("1", kSchema34BalanceRecoveryContract), std::runtime_error);
  EXPECT_THROW(check("3", kSchema34BalanceRecoveryContract), std::runtime_error);
}

TEST(PpRuntimeContract, Build2RapidPreemptMetadataIsOneExactSchema22Tuple) {
  struct Contract {
    std::string recipe = "hitter_pingpong_build2_rapid_preempt_v1";
    std::string version = "1";
    std::string build = "build2_rapid_preempt_schema22";
    std::string command = "postcontact_fresh_flight_preempt_v1";
    std::string planner =
        "fresh_schema2_flight_after_protected_followthrough_v1";
    std::string transition = "policy_owned_110d_markov_no_qdes_override_v1";
    double no_preempt = 0.12;
    std::array<double, 2> entry_tts = {0.60, 0.75};
    std::array<double, 2> commit_delay = {0.40, 0.55};
    std::string commit_delay_clock =
        "contact_phase_aligned_control_tick_closed_support_v1";
    std::array<double, 4> actor_entry_tts = {0.40, 0.82, 0.40, 0.55};
    std::string preposition = "disabled_until_atomic_commit";
    std::string next_arrival;
    std::string replay = "build2_progressive_0p05_to_0p20_v1";
    std::string entry_hide = "true";
    std::string early_wrap = "false";
    std::string joint_cost = "rational_soft_union_all_joints_v1";
    std::string native_hold = "0,0";
    std::string post_home =
        "contact_plus_0p12_latch_recovery_and_publish_immutable_home_base_target;"
        "preserve_old_racket_target_negative_tts_and_motion_tail_until_fresh_"
        "flight_commit_v1";
    std::string home_reward = "yaml_owned_bounded_y_only_home_return_active_v1";
    std::string lifecycle =
        "cold_hold_neutral;contact_plus_0p12_home;fresh_external_flight_preempt_"
        "0p40_0p55;policy_owned_transition;no_ready_gate";
    std::string post_hit_arrival = "postcontact_fresh_flight_preempt_v1";
    std::string station_mode;
    std::string station_contract;
    std::string base_target_semantics;
  };
  auto validate = [](const Contract& c) {
    validate_hitter_pingpong_build2_rapid_preempt_metadata(
        c.recipe, c.version, c.build, c.command, c.planner, c.transition,
        c.no_preempt, c.entry_tts, c.commit_delay, c.commit_delay_clock,
        c.actor_entry_tts,
        c.preposition,
        c.next_arrival, c.replay, c.entry_hide, c.early_wrap, c.joint_cost,
        c.native_hold, c.post_home, c.home_reward, c.lifecycle,
        c.post_hit_arrival, c.station_mode, c.station_contract,
        c.base_target_semantics);
  };

  const Contract exact;
  EXPECT_NO_THROW(validate(exact));
  {
    auto invalid = exact;
    invalid.recipe = "hitter_pingpong_build2_minimal_rootfix_v1";
    EXPECT_THROW(validate(invalid), std::runtime_error);
  }
  {
    auto invalid = exact;
    invalid.transition = "qdes_blend_or_override";
    EXPECT_THROW(validate(invalid), std::runtime_error);
  }
  {
    auto invalid = exact;
    invalid.no_preempt = 0.119;
    EXPECT_THROW(validate(invalid), std::runtime_error);
  }
  {
    auto invalid = exact;
    invalid.entry_tts[0] = 0.59;
    EXPECT_THROW(validate(invalid), std::runtime_error);
  }
  {
    auto invalid = exact;
    invalid.commit_delay[1] = 0.551;
    EXPECT_THROW(validate(invalid), std::runtime_error);
  }
  {
    auto invalid = exact;
    invalid.commit_delay_clock = "continuous_uniform_support";
    EXPECT_THROW(validate(invalid), std::runtime_error);
  }
  {
    auto invalid = exact;
    invalid.actor_entry_tts = {0.60, 0.75, 0.60, 0.75};
    EXPECT_THROW(validate(invalid), std::runtime_error);
  }
  {
    auto invalid = exact;
    invalid.post_home =
        "contact_plus_0p12_latch_recovery_then_qdes_blend";
    EXPECT_THROW(validate(invalid), std::runtime_error);
  }
  {
    auto invalid = exact;
    invalid.post_hit_arrival = "disabled";
    EXPECT_THROW(validate(invalid), std::runtime_error);
  }
}

TEST(PpRuntimeContract, Build2FixedHomeMetadataAcceptsExactSchema23Through29Tuple) {
  auto validate = [](const std::string& recipe, const std::string& build,
                     const std::string& station_contract,
                     const std::string& replay =
                         "markov_fixed_home_side_phase_severity_v4",
                     const std::string& transition =
                         "policy_owned_110d_markov_no_qdes_override_v1",
                     const std::string& home_reward =
                         "yaml_owned_bounded_xy_post_strike_home_drift_v1",
                     bool force_schema27_version_one = false) {
    const bool schema27 =
        recipe == "hitter_pingpong_build2_torso_optional_reach_v2";
    const bool schema28 =
        recipe == "hitter_pingpong_build2_frozen_core_optional_reach_v1";
    const bool schema29 =
        recipe == "hitter_pingpong_build2_trainable_home_optional_reach_v1";
    const bool optional = schema27 || schema28 || schema29;
    validate_hitter_pingpong_build2_rapid_preempt_metadata(
        recipe,
        schema27 && !force_schema27_version_one ? "2" : "1", build,
        "fixed_home_fresh_flight_preempt_v1",
        optional
            ? "fresh_schema3_flight_shared_session_home_support_intent_v2"
            : "fresh_schema2_flight_fixed_session_home_v1",
        transition, 0.12,
        {0.60, 0.75}, optional ? std::array<double, 2>{0.50, 0.65}
                              : std::array<double, 2>{0.40, 0.55},
        "contact_phase_aligned_control_tick_closed_support_v1",
        {0.40, 0.82, 0.40, 0.55}, "disabled_fixed_session_home", "",
        replay, "true", "false",
        "rational_soft_union_all_joints_v1", "0,0",
        "contact_plus_0p12_latch_recovery_only;station_remains_session_home;"
        "preserve_old_racket_target_negative_tts_and_motion_tail_until_fresh_"
        "flight_commit_v1",
        home_reward,
        optional
            ? "cold_hold_neutral;contact_plus_0p12_recovery_latch;fresh_"
              "external_flight_preempt_0p50_0p65;immutable_session_home;"
              "policy_owned_transition;no_ready_gate"
            : "cold_hold_neutral;contact_plus_0p12_recovery_latch;fresh_"
              "external_flight_preempt_0p40_0p55;immutable_session_home;"
              "policy_owned_transition;no_ready_gate",
        "fixed_home_fresh_flight_preempt_v1", "fixed_session_home_v1",
        station_contract, "immutable_home_minus_mocap_base_v1");
  };
  EXPECT_NO_THROW(validate(
      "hitter_pingpong_build2_fixed_home_recovery_v1",
      "build2_fixed_home_recovery_schema23",
      "immutable_session_home_all_phases_v1"));
  EXPECT_NO_THROW(validate(
      "hitter_pingpong_build2_fixed_home_feasible_v1",
      "build2_fixed_home_feasible_schema24",
      "immutable_session_home_all_phases_v1"));
  EXPECT_NO_THROW(validate(
      "hitter_pingpong_build2_fixed_home_phase_recovery_v1",
      "build2_fixed_home_phase_recovery_schema25",
      "immutable_session_home_all_phases_v1",
      "markov_fixed_home_side_phase_severity_v5"));
  EXPECT_NO_THROW(validate(
      "hitter_pingpong_build2_home_support_frame_v1",
      "build2_home_support_frame_schema26",
      "immutable_home_support_frame_all_phases_v2",
      "markov_fixed_home_support_side_phase_severity_v6",
      "policy_owned_110d_markov_headslots_vxy_no_qdes_override_v2",
      "continuous_immutable_home_support_frame_v1"));
  EXPECT_NO_THROW(validate(
      "hitter_pingpong_build2_torso_optional_reach_v2",
      "build2_torso_optional_reach_schema27_v2",
      "immutable_home_support_frame_all_phases_v2",
      "markov_fixed_home_support_reach_side_phase_severity_v7",
      "policy_owned_112d_markov_frozen_core_residual_no_qdes_override_v3",
      "continuous_immutable_home_support_frame_v1"));
  EXPECT_NO_THROW(validate(
      "hitter_pingpong_build2_frozen_core_optional_reach_v1",
      "build2_frozen_core_shadow_schema28",
      "immutable_home_support_frame_all_phases_v2",
      "markov_fixed_home_support_reach_side_phase_severity_core_shadow_v8",
      "policy_owned_127d_markov_frozen_core_shadow_residual_no_qdes_override_v1",
      "continuous_immutable_home_support_frame_v1"));
  EXPECT_NO_THROW(validate(
      "hitter_pingpong_build2_trainable_home_optional_reach_v1",
      "build2_trainable_home_optional_reach_schema29",
      "immutable_home_support_frame_all_phases_v2",
      "markov_side_phase_severity_v3",
      "policy_owned_112d_markov_trainable_no_qdes_override_v1",
      "continuous_immutable_home_support_frame_v1"));
  EXPECT_THROW(validate(
      "hitter_pingpong_build2_trainable_home_optional_reach_v1",
      "build2_trainable_home_optional_reach_schema29",
      "immutable_home_support_frame_all_phases_v2",
      "markov_fixed_home_support_reach_side_phase_severity_v7",
      "policy_owned_112d_markov_trainable_no_qdes_override_v1",
      "continuous_immutable_home_support_frame_v1"),
      std::runtime_error);
  EXPECT_THROW(validate(
      "hitter_pingpong_build2_trainable_home_optional_reach_v1",
      "build2_trainable_home_optional_reach_schema29",
      "immutable_home_support_frame_all_phases_v2",
      "markov_fixed_home_support_reach_side_phase_severity_v9",
      "policy_owned_112d_markov_trainable_no_qdes_override_v1",
      "continuous_immutable_home_support_frame_v1"),
      std::runtime_error);
  EXPECT_THROW(validate(
      "hitter_pingpong_build2_trainable_home_optional_reach_v1",
      "build2_trainable_home_optional_reach_schema29",
      "immutable_home_support_frame_all_phases_v2",
      "markov_side_phase_severity_v3",
      "policy_owned_127d_markov_frozen_core_shadow_residual_no_qdes_override_v1",
      "continuous_immutable_home_support_frame_v1"),
      std::runtime_error);
  EXPECT_THROW(validate(
      "hitter_pingpong_build2_torso_optional_reach_v2",
      "build2_torso_optional_reach_schema27_v2",
      "immutable_home_support_frame_all_phases_v2",
      "markov_fixed_home_support_reach_side_phase_severity_v7",
      "policy_owned_112d_markov_frozen_core_residual_no_qdes_override_v3",
      "continuous_immutable_home_support_frame_v1", true),
      std::runtime_error);
  EXPECT_THROW(validate(
      "hitter_pingpong_build2_fixed_home_phase_recovery_v1",
      "build2_fixed_home_phase_recovery_schema25",
      "immutable_session_home_all_phases_v1"), std::runtime_error);
  EXPECT_THROW(validate(
      "hitter_pingpong_build2_fixed_home_feasible_v1",
      "build2_fixed_home_recovery_schema23",
      "immutable_session_home_all_phases_v1"), std::runtime_error);
  EXPECT_THROW(validate(
      "hitter_pingpong_build2_fixed_home_feasible_v1",
      "build2_fixed_home_feasible_schema24", "ball_dependent_station"),
      std::runtime_error);
}

TEST(PpRuntimeContract, Schema27ReachInputsSupportDisabledAndEnabledNonPrivilegedTuples) {
  EXPECT_EQ(validate_hitter_pingpong_schema27_nonprivileged_metadata(
      "hitter_pingpong_build2_torso_optional_reach_v2",
      "hitter_pure_112_headslots_vxy_reach_v1",
      "frozen_stage0_core_waist_leg_residual_v2",
      "fresh_schema2_flight_fixed_session_home_v1", "110,111",
      "planner_certified_tuple,planner_certified_tuple",
      "planner_tuple_no_contact_force_or_plant_inference_v1",
      "certified_planner_tuple_labels_no_plant_inference_v1", "false",
      "disabled_zero_v1",
      "immutable_home_torso_settle_optional_reach_stage0_v1",
      "signed_tts_visible_torso_handoff_v1"),
      HitterOptionalReachRuntimeMode::kDisabledSchema2Zero);
  EXPECT_EQ(validate_hitter_pingpong_schema27_nonprivileged_metadata(
      "hitter_pingpong_build2_torso_optional_reach_v2",
      "hitter_pure_112_headslots_vxy_reach_v1",
      "frozen_stage0_core_waist_leg_residual_v2",
      "fresh_schema3_flight_shared_session_home_support_intent_v2", "110,111",
      "planner_target_tuple,planner_target_tuple",
      "planner_tuple_no_contact_force_or_plant_inference_v1",
      "staged_planner_support_intent_no_plant_inference_v4", "true",
      "staged_support_intent_training_v3",
      "immutable_home_world_torso_unified_support_residual_v4",
      "full_swing_torso_then_visible_settle_v4"),
      HitterOptionalReachRuntimeMode::kEnabledSchema3);
  EXPECT_THROW(validate_hitter_pingpong_schema27_nonprivileged_metadata(
                   "hitter_pingpong_build2_torso_optional_reach_v2",
                   "hitter_pure_112_headslots_vxy_reach_v1",
                   "frozen_stage0_core_waist_leg_residual_v2",
                   "fresh_schema3_flight_shared_session_home_support_intent_v2",
                   "110,111",
                   "sim_contact_force,sim_foot_height",
                   "planner_tuple_no_contact_force_or_plant_inference_v1",
                   "staged_planner_support_intent_no_plant_inference_v4",
                   "true", "staged_support_intent_training_v3",
                   "immutable_home_world_torso_unified_support_residual_v4",
                   "full_swing_torso_then_visible_settle_v4"),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pingpong_schema27_nonprivileged_metadata(
                   "hitter_pingpong_build2_torso_optional_reach_v2",
                   "hitter_pure_112_headslots_vxy_reach_v1",
                   "shared_unfrozen_actor_mlp",
                   "fresh_schema3_flight_shared_session_home_support_intent_v2",
                   "110,111", "planner_target_tuple,planner_target_tuple",
                   "planner_tuple_no_contact_force_or_plant_inference_v1",
                   "staged_planner_support_intent_no_plant_inference_v4",
                   "true", "staged_support_intent_training_v3",
                   "immutable_home_world_torso_unified_support_residual_v4",
                   "full_swing_torso_then_visible_settle_v4"),
               std::runtime_error);
}

TEST(PpRuntimeContract, OptionalReachWireRevisionIsArtifactBound) {
  const auto disabled =
      HitterOptionalReachRuntimeMode::kDisabledSchema2Zero;
  const auto candidate =
      HitterOptionalReachRuntimeMode::kCandidateSchema3Level0Only;
  const auto enabled = HitterOptionalReachRuntimeMode::kEnabledSchema3;
  EXPECT_FALSE(hitter_optional_reach_mode_uses_schema3(disabled));
  EXPECT_TRUE(hitter_optional_reach_mode_uses_schema3(candidate));
  EXPECT_TRUE(hitter_optional_reach_mode_uses_schema3(enabled));
  EXPECT_TRUE(hitter_optional_reach_wire_is_compatible(
      true, disabled, 2, 0.0, 0.0));
  EXPECT_FALSE(hitter_optional_reach_wire_is_compatible(
      true, disabled, 3, 0.0, 0.0));
  EXPECT_FALSE(hitter_optional_reach_wire_is_compatible(
      true, candidate, 2, 0.0, 0.0));
  EXPECT_TRUE(hitter_optional_reach_wire_is_compatible(
      true, candidate, 3, 0.0, 0.0));
  EXPECT_FALSE(hitter_optional_reach_wire_is_compatible(
      true, candidate, 3, 1.0, -1.0));
  EXPECT_FALSE(hitter_optional_reach_wire_is_compatible(
      true, enabled, 2, 0.0, 0.0));
  EXPECT_TRUE(hitter_optional_reach_wire_is_compatible(
      true, enabled, 3, 0.0, 0.0));
  EXPECT_TRUE(hitter_optional_reach_wire_is_compatible(
      true, enabled, 3, 1.0, -1.0));
  EXPECT_TRUE(hitter_optional_reach_wire_is_compatible(
      true, enabled, 3, 2.0, 1.0));
  EXPECT_FALSE(hitter_optional_reach_wire_is_compatible(
      true, enabled, 3, 2.0, 0.0));
  EXPECT_FALSE(hitter_optional_reach_wire_is_compatible(
      true, enabled, 3, 0.0, 1.0));
  EXPECT_FALSE(hitter_optional_reach_wire_is_compatible(
      false, disabled, 3, 0.0, 0.0));
}

TEST(PpRuntimeContract, OptionalReachIsVisibleOnlyForAnActiveLevelOneFlight) {
  EXPECT_FALSE(hitter_optional_reach_active_flight(false, 0));
  EXPECT_FALSE(hitter_optional_reach_active_flight(false, 1));
  EXPECT_FALSE(hitter_optional_reach_active_flight(true, 0));
  EXPECT_TRUE(hitter_optional_reach_active_flight(true, 1));
}

TEST(PpRuntimeContract, Schema27RevisionCannotMutateFlightPermission) {
  EXPECT_TRUE(hitter_optional_reach_revision_preserves_flight_permission(
      false, 2.0, -1.0, 1.0, 1.0));
  EXPECT_TRUE(hitter_optional_reach_revision_preserves_flight_permission(
      true, 2.0, -1.0, 2.0, -1.0));
  EXPECT_FALSE(hitter_optional_reach_revision_preserves_flight_permission(
      true, 2.0, -1.0, 1.0, -1.0));
  EXPECT_FALSE(hitter_optional_reach_revision_preserves_flight_permission(
      true, 2.0, -1.0, 2.0, 1.0));

  // A different newer mailbox flight may be busy-dropped between two
  // revisions of the pending flight. Identity still classifies the returning
  // pending revision as an update, so the independent label latch must reject
  // its mutation before pp_policy overwrites the pending command.
  EXPECT_EQ(
      planner_rapid_candidate_identity_decision(
          {1, 11}, 2, 13, {1, 10}, {1, 10}, {}, true, {1, 11}, 1, 11),
      PlannerRapidCandidateIdentityDecision::kUpdatePendingRevision);
  EXPECT_FALSE(hitter_optional_reach_revision_preserves_flight_permission(
      true, 2.0, -1.0, 1.0, -1.0));
}

TEST(PpRuntimeContract, Schema27NonzeroPermissionIsBoundToRunnerHomeTuple) {
  EXPECT_TRUE(hitter_schema27_reach_permission_matches_home_target_tuple(
      0.70, 1.0, 1.0, 2.0, -1.0));
  EXPECT_TRUE(hitter_schema27_reach_permission_matches_home_target_tuple(
      1.095, 1.0, -1.0, 1.0, -1.0));
  // Level 1 unloads the target-opposite foot for either swing family.
  EXPECT_TRUE(hitter_schema27_reach_permission_matches_home_target_tuple(
      0.72, 1.0, 1.0, 1.0, 1.0));
  EXPECT_TRUE(hitter_schema27_reach_permission_matches_home_target_tuple(
      0.90, 1.0, -1.0, 1.0, 1.0));
  EXPECT_TRUE(hitter_schema27_reach_permission_matches_home_target_tuple(
      1.10, 1.0, -1.0, 1.0, -1.0));
  // Level 2 moves the target-side foot for either swing family.
  EXPECT_TRUE(hitter_schema27_reach_permission_matches_home_target_tuple(
      0.87, 1.0, -1.0, 2.0, -1.0));
  EXPECT_TRUE(hitter_schema27_reach_permission_matches_home_target_tuple(
      1.15, 1.0, -1.0, 2.0, 1.0));
  EXPECT_FALSE(hitter_schema27_reach_permission_matches_home_target_tuple(
      0.70, 1.0, 1.0, 1.0, -1.0));
  EXPECT_FALSE(hitter_schema27_reach_permission_matches_home_target_tuple(
      0.70, 1.0, 1.0, 2.0, 1.0));
  EXPECT_FALSE(hitter_schema27_reach_permission_matches_home_target_tuple(
      0.72, 1.0, 1.0, 2.0, -1.0));
  // Zero is ordinary fixed support only inside the same signed production
  // support. It cannot launder an OOD target.
  EXPECT_TRUE(hitter_schema27_reach_permission_matches_home_target_tuple(
      0.74, 1.0, 1.0, 0.0, 0.0));
  EXPECT_FALSE(hitter_schema27_reach_permission_matches_home_target_tuple(
      0.60, 1.0, 1.0, 0.0, 0.0));
  EXPECT_FALSE(hitter_schema27_reach_permission_matches_home_target_tuple(
      1.28, 1.0, 1.0, 1.0, -1.0));
}

TEST(PpRuntimeContract, Schema28ShadowMetadataAndProvenanceAreExact) {
  EXPECT_TRUE(hitter_schema28_initialization_tuple(
      "schema28_from_build1_model21800_functional_vxy_zero_reach_frozen_core_shadow_v1",
      "model_21800.pt,sha256="
      "daca62616178177647cf9949128582785529dbe6b7ba080fc96d86da23916561,"
      "source_input_dim=110,frozen_core_input_dim=112,"
      "destination_actor_observation_dim=127,"
      "zeroed_source_cols=76|81,appended_zero_cols=110|111,frozen_core=true,"
      "core_shadow_feedback=true,zero_residuals=true"));
  EXPECT_NO_THROW(validate_hitter_pingpong_schema28_shadow_metadata(
      "hitter_pingpong_build2_frozen_core_optional_reach_v1",
      "policy_owned_127d_markov_frozen_core_shadow_residual_no_qdes_override_v1",
      "schema28_from_build1_model21800_functional_vxy_zero_reach_frozen_core_shadow_v1",
      "frozen_stage0_core_shadow_feedback_waist_leg_residual_v1",
      "core_owned15_actions_0_1_2_19_30_append112_126_v1",
      "0,1,2,19,20,21,22,23,24,25,26,27,28,29,30",
      "112,113,114,115,116,117,118,119,120,121,122,123,124,125,126",
      "actions31_core_action_shadow_owned15_v1", true, true));
  EXPECT_THROW(validate_hitter_pingpong_schema28_shadow_metadata(
      "hitter_pingpong_build2_frozen_core_optional_reach_v1",
      "policy_owned_127d_markov_frozen_core_shadow_residual_no_qdes_override_v1",
      "schema28_from_build1_model21800_functional_vxy_zero_reach_frozen_core_shadow_v1",
      "frozen_stage0_core_shadow_feedback_waist_leg_residual_v1",
      "core_owned15_actions_0_1_2_19_30_append112_126_v1",
      "0,1,2,19,20,21,22,23,24,25,26,27,28,29,30",
      "112,113,114,115,116,117,118,119,120,121,122,123,124,125,126",
      "actions31_core_action_shadow_owned15_v1", true, false),
      std::runtime_error);
  EXPECT_EQ(validate_hitter_pingpong_schema27_nonprivileged_metadata(
      "hitter_pingpong_build2_frozen_core_optional_reach_v1",
      "hitter_pure_127_headslots_vxy_reach_core_shadow_v1",
      "frozen_stage0_core_shadow_feedback_waist_leg_residual_v1",
      "fresh_schema3_flight_shared_session_home_support_intent_v2", "110,111",
      "planner_target_tuple,planner_target_tuple",
      "planner_tuple_plus_policy_shadow_no_plant_inference_v2",
      "staged_planner_support_intent_no_plant_inference_v4", "true",
      "staged_support_intent_training_v3",
      "immutable_home_world_torso_unified_support_residual_v4",
      "full_swing_torso_then_visible_settle_v4"),
      HitterOptionalReachRuntimeMode::kEnabledSchema3);
}

TEST(PpRuntimeContract, Schema3ParserPreservesPermissionAndAbsoluteDeadline) {
  PpRacketTargetInput input;
  input.SetFromFlat(Schema3RacketForRuntimeContractTest(
      1, 1, 2.0, -1.0));
  const auto accepted = input.Latest();
  ASSERT_TRUE(accepted.has_valid);
  EXPECT_EQ(accepted.cmd.schema, 3);
  EXPECT_DOUBLE_EQ(accepted.cmd.reach_level, 2.0);
  EXPECT_DOUBLE_EQ(accepted.cmd.swing_foot_sign, -1.0);
  EXPECT_NEAR(accepted.control_time_to_strike_s, 1.0, 0.05);

  input.SetFromFlat(Schema3RacketForRuntimeContractTest(
      2, 2, 2.0, 0.0));
  const auto rejected = input.Latest();
  ASSERT_TRUE(rejected.has_valid);
  EXPECT_TRUE(rejected.invalid_after);
  EXPECT_EQ(rejected.cmd.command_seq, 1U);
  EXPECT_DOUBLE_EQ(rejected.cmd.reach_level, 2.0);
  EXPECT_DOUBLE_EQ(rejected.cmd.swing_foot_sign, -1.0);
}

TEST(PpRuntimeContract, Schema3AbsoluteDeadlineIncludesPreReceiptTransport) {
  PpRacketTargetInput input;
  input.SetFromFlat(Schema3RacketForRuntimeContractTest(
      1, 1, 1.0, -1.0, 1.0, 0.20));
  const auto snap = input.Latest();
  ASSERT_TRUE(snap.has_valid);
  EXPECT_NEAR(snap.cmd.time_to_strike, 1.0, 1.0e-12);
  EXPECT_NEAR(snap.control_time_to_strike_s, 0.80, 0.03);
  EXPECT_NEAR(snap.producer_age_s, 0.20, 0.03);
}

TEST(PpRuntimeContract, Schema3PermissionIsImmutableWithinOneFlight) {
  PpRacketTargetInput input;
  input.SetFromFlat(Schema3RacketForRuntimeContractTest(
      1, 1, 2.0, -1.0));
  const auto first = input.Latest();
  ASSERT_TRUE(first.has_valid);
  ASSERT_EQ(first.cmd.command_seq, 1U);

  input.SetFromFlat(Schema3RacketForRuntimeContractTest(
      2, 2, 1.0, -1.0));
  const auto mutation = input.Latest();
  ASSERT_TRUE(mutation.has_valid);
  EXPECT_FALSE(mutation.invalid_after);
  EXPECT_EQ(mutation.cmd.command_seq, 1U);
  EXPECT_EQ(mutation.cmd.revision_id, 1U);
  EXPECT_DOUBLE_EQ(mutation.cmd.reach_level, 2.0);
  EXPECT_DOUBLE_EQ(mutation.cmd.swing_foot_sign, -1.0);

  input.SetFromFlat(Schema3RacketForRuntimeContractTest(
      3, 3, 2.0, -1.0));
  const auto restored = input.Latest();
  ASSERT_TRUE(restored.has_valid);
  EXPECT_EQ(restored.cmd.command_seq, 3U);
  EXPECT_EQ(restored.cmd.revision_id, 3U);
  EXPECT_DOUBLE_EQ(restored.cmd.reach_level, 2.0);
  EXPECT_DOUBLE_EQ(restored.cmd.swing_foot_sign, -1.0);
}

TEST(PpRuntimeContract, Schema3PermissionLatchSurvivesInvalidDropout) {
  PpRacketTargetInput input;
  input.SetFromFlat(Schema3RacketForRuntimeContractTest(
      1, 1, 2.0, -1.0));
  ASSERT_EQ(input.Latest().cmd.command_seq, 1U);

  input.SetFromFlat(Schema3RacketForRuntimeContractTest(
      2, 2, 0.0, 0.0, 0.0));
  const auto dropout = input.Latest();
  ASSERT_TRUE(dropout.has_valid);
  EXPECT_TRUE(dropout.invalid_after);
  EXPECT_EQ(dropout.cmd.command_seq, 1U);

  input.SetFromFlat(Schema3RacketForRuntimeContractTest(
      3, 3, 1.0, -1.0));
  const auto mutation = input.Latest();
  ASSERT_TRUE(mutation.has_valid);
  EXPECT_TRUE(mutation.invalid_after);
  EXPECT_EQ(mutation.cmd.command_seq, 1U);
  EXPECT_EQ(mutation.cmd.revision_id, 1U);
  EXPECT_DOUBLE_EQ(mutation.cmd.reach_level, 2.0);
  EXPECT_DOUBLE_EQ(mutation.cmd.swing_foot_sign, -1.0);

  input.SetFromFlat(Schema3RacketForRuntimeContractTest(
      4, 4, 2.0, -1.0));
  const auto restored = input.Latest();
  ASSERT_TRUE(restored.has_valid);
  EXPECT_FALSE(restored.invalid_after);
  EXPECT_EQ(restored.cmd.command_seq, 4U);
  EXPECT_DOUBLE_EQ(restored.cmd.reach_level, 2.0);
}

TEST(PpRuntimeContract, Schema3PermissionLatchSurvivesBusyFlightInterleave) {
  PpRacketTargetInput input;
  input.SetFromFlat(Schema3RacketForRuntimeContractTest(
      1, 1, 2.0, -1.0, 1.0, 0.0, 77));
  input.SetFromFlat(Schema3RacketForRuntimeContractTest(
      2, 1, 1.0, 1.0, 1.0, 0.0, 78));
  ASSERT_EQ(input.Latest().cmd.flight_id, 78U);

  input.SetFromFlat(Schema3RacketForRuntimeContractTest(
      3, 2, 1.0, -1.0, 1.0, 0.0, 77));
  const auto mutation = input.Latest();
  ASSERT_TRUE(mutation.has_valid);
  EXPECT_EQ(mutation.cmd.command_seq, 2U);
  EXPECT_EQ(mutation.cmd.flight_id, 78U);

  input.SetFromFlat(Schema3RacketForRuntimeContractTest(
      4, 3, 2.0, -1.0, 1.0, 0.0, 77));
  const auto restored = input.Latest();
  ASSERT_TRUE(restored.has_valid);
  EXPECT_EQ(restored.cmd.command_seq, 4U);
  EXPECT_EQ(restored.cmd.flight_id, 77U);
  EXPECT_DOUBLE_EQ(restored.cmd.reach_level, 2.0);
}

TEST(PpRuntimeContract, ExpectedWireSchemaIgnoresMixedPublisherState) {
  auto schema2_high = Schema3RacketForRuntimeContractTest(
      100, 1, 0.0, 0.0);
  schema2_high[0] = 2.0;
  schema2_high.resize(19);

  PpRacketTargetInput enabled_schema3(3);
  enabled_schema3.SetFromFlat(schema2_high);
  EXPECT_FALSE(enabled_schema3.Latest().has_valid);
  enabled_schema3.SetFromFlat(Schema3RacketForRuntimeContractTest(
      1, 1, 1.0, -1.0));
  ASSERT_TRUE(enabled_schema3.Latest().has_valid);
  EXPECT_EQ(enabled_schema3.Latest().cmd.command_seq, 1U);

  PpRacketTargetInput disabled_schema2(2);
  disabled_schema2.SetFromFlat(Schema3RacketForRuntimeContractTest(
      100, 1, 1.0, -1.0));
  EXPECT_FALSE(disabled_schema2.Latest().has_valid);
  auto schema2_low = Schema3RacketForRuntimeContractTest(
      1, 1, 0.0, 0.0);
  schema2_low[0] = 2.0;
  schema2_low.resize(19);
  disabled_schema2.SetFromFlat(schema2_low);
  ASSERT_TRUE(disabled_schema2.Latest().has_valid);
  EXPECT_EQ(disabled_schema2.Latest().cmd.command_seq, 1U);
}

TEST(PpRuntimeContract, Schema24TrainingScreenProfileIsFailClosed) {
  Schema24TrainingScreenGate3Metadata exact{
      .training_recipe = "hitter_pingpong_build2_fixed_home_feasible_v1",
      .recipe_version = "1",
      .runtime_contract = "rally_final_v2",
      .deployment_status = "gate3_screen_only",
      .validator_profile = "schema24_training_screen_gate3_only_v1",
      .qualification_status = "training_screened_not_deployable",
      .hardware_authorized = "false",
      .gate3_screen_contract = "schema24_training_screen_gate3_only_v1",
      .reward_contract = "fixed_home_post_strike_drift_table_clearance_v1",
      .question_bank_contract = "fixed_home_coherent_question_bank_v1",
      .question_bank_certification_contract =
          "fixed_home_training_admissibility_atlas_v1",
      .table_clearance_contract = "swept_racket_obb_table_aabb_v1",
      .training_admissibility = "TRAINING_SCREENED",
      .deployment_qualification = "NOT_PROVEN",
      .source_tree_matches_training = "true",
      .checkpoint_sha256 =
          "d50132d270ffdd80bba0d1997b1d2d38573bcb949a3769ab6b9ce10f7271d9a4",
      .training_source_tree_sha256 =
          "d8d97791ddeb067ef851832bb70001220ca4a0f70030419f5c7ebb39ce0692c6",
      .export_source_tree_sha256 =
          "d8d97791ddeb067ef851832bb70001220ca4a0f70030419f5c7ebb39ce0692c6",
      .frozen_env_sha256 =
          "f5b05ffc360566df0079d62985b9bb6a589b325d2668cf889815f29e4f75a97a",
      .motion_forehand_sha256 =
          "a6c68513720b12b168379cd6fa13f8b77607b4fa0bf7e828c4e1d81eda6f2094",
      .motion_backhand_sha256 =
          "67d04e13deeed068bdb003e379e18330dcd29210d280188fab7af26c0764eaac",
      .question_bank_sha256 =
          "3c039e9ec709eed7ef0986618862287345c89d5b6b72730119c9c4dd8a48164e",
      .question_bank_receipt_sha256 =
          "3ed0a13867a10604d5c792714107c4f3394704527956bb58b55d83d9389b3e4f",
  };
  EXPECT_TRUE(ValidateSchema24TrainingScreenGate3Metadata(exact).empty());
  auto hardware = exact;
  hardware.hardware_authorized = "true";
  EXPECT_FALSE(ValidateSchema24TrainingScreenGate3Metadata(hardware).empty());
  auto qualified = exact;
  qualified.deployment_qualification = "PROVEN";
  EXPECT_FALSE(ValidateSchema24TrainingScreenGate3Metadata(qualified).empty());
  auto source_drift = exact;
  source_drift.source_tree_matches_training = "false";
  EXPECT_FALSE(ValidateSchema24TrainingScreenGate3Metadata(source_drift).empty());
}

TEST(PpRuntimeContract, ContinuousV3MetadataIsOneExactMemorylessTuple) {
  struct Contract {
    std::string recipe = "hitter_pingpong_continuous_rally_v3";
    std::string version = "3";
    std::string build = "continuous_rally_schema19";
    std::string command = "strike_followthrough_home_external_commit_v3";
    std::string wait_clock = "constant_pending_wait_v1";
    double wait_tts = 0.85;
    std::array<double, 3> pending_target = {0.58, -0.44, 1.075};
    std::array<double, 3> pending_velocity = {1.92, 0.19, 1.03};
    std::string arrival = "memoryless_geometric_v1";
    std::string legacy_next_arrival;
    double hazard = 0.08;
    double dt = 0.02;
  } exact;
  auto validate = [](const Contract& contract) {
    validate_hitter_pingpong_continuous_v3_metadata(
        contract.recipe, contract.version, contract.build, contract.command,
        contract.wait_clock, contract.wait_tts, contract.pending_target,
        contract.pending_velocity, contract.arrival,
        contract.legacy_next_arrival, contract.hazard, contract.dt);
  };
  EXPECT_NO_THROW(validate(exact));

  auto expect_rejected = [&validate](Contract invalid) {
    EXPECT_THROW(validate(invalid), std::runtime_error);
  };
  {
    auto invalid = exact;
    invalid.recipe = "hitter_pingpong_continuous_rally_v2";
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.version = "2";
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.build = "continuous_rally_schema18";
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.command = "strike_followthrough_home_external_commit_v2";
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.wait_clock = "visible_elapsed_v1";
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.wait_tts = 1.0;
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.pending_target[1] = -0.265;
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.pending_velocity = {0.0, 0.0, 0.0};
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.arrival = "contact_relative_external_v2";
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.legacy_next_arrival =
        "contact_relative_external_0p12_1p20_all_station_classes_v2";
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.hazard = 0.09;
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.dt = 0.01;
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.pending_velocity[0] =
        std::numeric_limits<double>::quiet_NaN();
    expect_rejected(invalid);
  }
}

TEST(PpRuntimeContract, ContinuousV4MetadataIsOneExactSchema20Tuple) {
  struct Contract {
    std::string recipe = "hitter_pingpong_continuous_rally_v4";
    std::string version = "4";
    std::string build = "continuous_rally_schema20";
    std::string command = "strike_followthrough_home_external_commit_v4";
    std::string planner_commit =
        "fresh_shot_external_event_policy_independent_v2";
    std::string actor_obs = "hitter_pure_110_headslots_vxy_v1";
    std::string trajectory = "contact_linear_actor_visible_v1";
    std::string motion = "signed_tts_contact_frame_v1";
    std::string velocity = "mocap_vxy_world_racket_velocity_v1";
    std::string wait_clock = "constant_pending_wait_v1";
    double wait_tts = 0.85;
    std::array<double, 3> pending_target = {0.58, -0.44, 1.075};
    std::array<double, 3> pending_velocity = {1.92, 0.19, 1.03};
    std::string arrival = "memoryless_geometric_v1";
    std::string legacy_next_arrival;
    double hazard = 0.08;
    double dt = 0.02;
  } exact;
  auto validate = [](const Contract& contract) {
    validate_hitter_pingpong_continuous_v4_metadata(
        contract.recipe, contract.version, contract.build, contract.command,
        contract.planner_commit, contract.actor_obs, contract.trajectory,
        contract.motion, contract.velocity, contract.wait_clock,
        contract.wait_tts, contract.pending_target, contract.pending_velocity,
        contract.arrival, contract.legacy_next_arrival, contract.hazard,
        contract.dt);
  };
  EXPECT_NO_THROW(validate(exact));

  auto expect_rejected = [&validate](Contract invalid) {
    EXPECT_THROW(validate(invalid), std::runtime_error);
  };
  {
    auto invalid = exact;
    invalid.recipe = "hitter_pingpong_continuous_rally_v3";
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.version = "3";
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.build = "continuous_rally_schema19";
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.command = "strike_followthrough_home_external_commit_v3";
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.planner_commit = "fixed_contact_plus_0p40_policy_independent_v1";
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.actor_obs = "hitter_pure";
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.trajectory = "release_hermite_v1";
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.motion = "release_fraction_v1";
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.velocity = "sim_world_body_v1";
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.wait_clock = "visible_elapsed_v1";
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.wait_tts = 1.0;
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.pending_target[0] = 0.57;
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.pending_velocity[1] = 0.0;
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.arrival = "contact_relative_external_v2";
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.legacy_next_arrival = "memoryless_geometric_v1";
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.hazard = 0.09;
    expect_rejected(invalid);
  }
  {
    auto invalid = exact;
    invalid.dt = 0.01;
    expect_rejected(invalid);
  }
}

TEST(PpRuntimeContract, ContinuousV2AllowsOnlyProductionExternalCommitMode) {
  HitterPingPongContinuousV2RuntimeMode mode;
  mode.planner_mode = true;
  mode.policy_native = true;
  mode.single_swing = true;
  mode.initial_level = 0;
  EXPECT_NO_THROW(validate_hitter_pingpong_continuous_v2_runtime_mode(
      true, mode));
  {
    auto schema21_legacy_inversion = mode;
    schema21_legacy_inversion.stay_if_reachable = false;
    EXPECT_NO_THROW(validate_hitter_pingpong_continuous_v2_runtime_mode(
        true, schema21_legacy_inversion));
  }

  // The v1 compatibility path does not inherit v2's runtime-mode restrictions.
  EXPECT_NO_THROW(validate_hitter_pingpong_continuous_v2_runtime_mode(
      false, HitterPingPongContinuousV2RuntimeMode{}));

  auto expect_rejected = [](HitterPingPongContinuousV2RuntimeMode invalid) {
    EXPECT_THROW(validate_hitter_pingpong_continuous_v2_runtime_mode(
                     true, invalid),
                 std::runtime_error);
  };
  {
    auto invalid = mode;
    invalid.planner_mode = false;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.policy_native = false;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.single_swing = false;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.initial_level = 1;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.stream_target = true;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.station_only = true;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.replay_mode = true;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.target_override = true;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.policy_output_override = true;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.clock_override = true;
    expect_rejected(invalid);
  }
  // The CLI separately restricts this to x86 MuJoCo Gate3. It preserves raw finite q_des so the
  // qualification observes unsafe commands instead of hiding them behind a runtime clamp.
  {
    auto gate3 = mode;
    gate3.qdes_audit_only = true;
    EXPECT_NO_THROW(validate_hitter_pingpong_continuous_v2_runtime_mode(
        true, gate3));
  }
}

TEST(PpRuntimeContract, Build2RapidPreemptAllowsOnlyPolicyOwnedSchema22Mode) {
  HitterPingPongContinuousV2RuntimeMode mode;
  mode.planner_mode = true;
  mode.policy_native = true;
  mode.single_swing = true;
  mode.initial_level = 0;
  mode.stay_if_reachable = true;
  EXPECT_NO_THROW(validate_hitter_pingpong_build2_rapid_preempt_runtime_mode(
      true, mode));
  EXPECT_NO_THROW(validate_hitter_pingpong_build2_rapid_preempt_runtime_mode(
      false, HitterPingPongContinuousV2RuntimeMode{}));

  auto expect_rejected = [](HitterPingPongContinuousV2RuntimeMode invalid) {
    EXPECT_THROW(validate_hitter_pingpong_build2_rapid_preempt_runtime_mode(
                     true, invalid),
                 std::runtime_error);
  };
  {
    auto invalid = mode;
    invalid.planner_mode = false;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.policy_native = false;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.single_swing = false;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.initial_level = 1;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.stream_target = true;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.station_only = true;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.replay_mode = true;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.target_override = true;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.policy_output_override = true;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.clock_override = true;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.target_support_gate = false;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.stay_if_reachable = false;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.command_timeout_s = 0.0;
    expect_rejected(invalid);
  }
  {
    auto audit_only = mode;
    audit_only.qdes_audit_only = true;
    EXPECT_NO_THROW(validate_hitter_pingpong_build2_rapid_preempt_runtime_mode(
        true, audit_only));
  }
}

TEST(PpRuntimeContract, Build2FixedHomeHasNoReachabilityAdmissionFlags) {
  HitterPingPongContinuousV2RuntimeMode mode;
  mode.planner_mode = true;
  mode.policy_native = true;
  mode.single_swing = true;
  mode.initial_level = 0;
  mode.target_support_gate = false;
  mode.stay_if_reachable = false;
  EXPECT_NO_THROW(validate_hitter_pingpong_build2_fixed_home_runtime_mode(
      true, mode));
  EXPECT_THROW(validate_hitter_pingpong_build2_rapid_preempt_runtime_mode(
                   true, mode),
               std::runtime_error);

  mode.stream_target = true;
  EXPECT_THROW(validate_hitter_pingpong_build2_fixed_home_runtime_mode(
                   true, mode),
               std::runtime_error);
}

TEST(PpRuntimeContract, ContinuousV3AllowsOnlyProductionExternalCommitMode) {
  HitterPingPongContinuousV2RuntimeMode mode;
  mode.planner_mode = true;
  mode.policy_native = true;
  mode.single_swing = true;
  mode.initial_level = 0;
  EXPECT_NO_THROW(validate_hitter_pingpong_continuous_v3_runtime_mode(
      true, mode));

  auto expect_rejected = [](HitterPingPongContinuousV2RuntimeMode invalid) {
    EXPECT_THROW(validate_hitter_pingpong_continuous_v3_runtime_mode(
                     true, invalid),
                 std::runtime_error);
  };
  {
    auto invalid = mode;
    invalid.planner_mode = false;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.policy_native = false;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.single_swing = false;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.initial_level = 1;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.stream_target = true;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.station_only = true;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.replay_mode = true;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.target_override = true;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.policy_output_override = true;
    expect_rejected(invalid);
  }
  {
    auto invalid = mode;
    invalid.clock_override = true;
    expect_rejected(invalid);
  }
  {
    auto gate3 = mode;
    gate3.qdes_audit_only = true;
    EXPECT_NO_THROW(validate_hitter_pingpong_continuous_v3_runtime_mode(
        true, gate3));
  }
}

TEST(PpRuntimeContract, ContinuousV4AllowsOnlyProductionExternalCommitMode) {
  HitterPingPongContinuousV2RuntimeMode mode;
  mode.planner_mode = true;
  mode.policy_native = true;
  mode.single_swing = true;
  mode.initial_level = 0;
  EXPECT_NO_THROW(validate_hitter_pingpong_continuous_v4_runtime_mode(
      true, mode));

  auto invalid = mode;
  invalid.stream_target = true;
  EXPECT_THROW(validate_hitter_pingpong_continuous_v4_runtime_mode(
                   true, invalid),
               std::runtime_error);
  invalid = mode;
  invalid.policy_native = false;
  EXPECT_THROW(validate_hitter_pingpong_continuous_v4_runtime_mode(
                   true, invalid),
               std::runtime_error);
  EXPECT_NO_THROW(validate_hitter_pingpong_continuous_v4_runtime_mode(
      false, HitterPingPongContinuousV2RuntimeMode{}));
}

TEST(PpRuntimeContract, ContinuousV4MocapVelocityIsFailClosed) {
  EXPECT_NO_THROW(validate_hitter_pingpong_continuous_v4_mocap_observation(
      true, true, true, {0.25, -0.18}));
  EXPECT_THROW(validate_hitter_pingpong_continuous_v4_mocap_observation(
                   true, false, true, {0.25, -0.18}),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pingpong_continuous_v4_mocap_observation(
                   true, true, false, {0.0, 0.0}),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pingpong_continuous_v4_mocap_observation(
                   true, true, true,
                   {std::numeric_limits<double>::quiet_NaN(), 0.0}),
               std::runtime_error);
  // Historical contracts keep their established stale/outage behavior.
  EXPECT_NO_THROW(validate_hitter_pingpong_continuous_v4_mocap_observation(
      false, false, false,
      {std::numeric_limits<double>::quiet_NaN(), 0.0}));
}

TEST(PpRuntimeContract, ContinuousV1V2V3V4KeepHitterPure110By31Interface) {
  EXPECT_EQ(kObsDim110, 110);
  EXPECT_EQ(kNumJoints, 31);
}

TEST(PpRuntimeContract, RejectsUnknownRuntimeVersion) {
  EXPECT_THROW(validate_hitter_pure_runtime_contract("rally_final_v3", "rally_v11"),
               std::runtime_error);
}

TEST(PpRuntimeContract, AcceptsOnlyPairedV17FixedStationRuntime) {
  EXPECT_EQ(
      validate_hitter_pure_runtime_contract(
          "rally_v17_fixed_station_ball_clock_v1", "rally_v17"),
      HitterPureRuntimeContract::kRallyV17FixedStationBallClockV1);
  EXPECT_THROW(
      validate_hitter_pure_runtime_contract(
          "rally_v17_fixed_station_ball_clock_v1", "rally_v14"),
      std::runtime_error);
  EXPECT_THROW(
      validate_hitter_pure_runtime_contract(
          "rally_v17_fixed_station_ball_clock_v1", "rally_v15"),
      std::runtime_error);
}

TEST(PpRuntimeContract, PhysicalHardLimitUsesStrictExportedTolerance) {
  constexpr double lo = -0.5;
  constexpr double hi = 0.4;
  constexpr double tolerance = 0.002;
  EXPECT_FALSE(exceeds_joint_hard_limit(lo - tolerance, lo, hi, tolerance));
  EXPECT_FALSE(exceeds_joint_hard_limit(hi + tolerance, lo, hi, tolerance));
  EXPECT_TRUE(
      exceeds_joint_hard_limit(lo - tolerance - 1e-9, lo, hi, tolerance));
  EXPECT_TRUE(
      exceeds_joint_hard_limit(hi + tolerance + 1e-9, lo, hi, tolerance));
}

TEST(PpRuntimeContract, ExportedTelemetryModeDoesNotTurnMeasuredQIntoFault) {
  constexpr double lo = -0.5;
  constexpr double hi = 0.4;
  constexpr double tolerance = 0.002;
  EXPECT_EQ(classify_actual_q_hard_limit(0.0, lo, hi, tolerance, true),
            ActualQHardLimitDisposition::kInside);
  EXPECT_EQ(
      classify_actual_q_hard_limit(hi + tolerance + 1e-6, lo, hi,
                                   tolerance, true),
      ActualQHardLimitDisposition::kTelemetry);
  EXPECT_EQ(
      classify_actual_q_hard_limit(hi + tolerance + 1e-6, lo, hi,
                                   tolerance, false),
      ActualQHardLimitDisposition::kFault);
}

TEST(PpRuntimeContract, AcceptsOnlyPairedV15RuntimeAndRecipe) {
  EXPECT_EQ(validate_hitter_pure_runtime_contract("rally_v15", "rally_v15"),
            HitterPureRuntimeContract::kRallyV15);
  EXPECT_THROW(validate_hitter_pure_runtime_contract("rally_v15", "rally_v14"),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pure_runtime_contract("rally_final_v2", "rally_v15"),
               std::runtime_error);
  EXPECT_THROW(validate_hitter_pure_runtime_contract("", "rally_v15"),
               std::runtime_error);
}

TEST(PpObsBuilder, V15KeepsExact110PrefixAndAppendsDeployableLocalization) {
  PpRobotState state;
  state.base_pos_w = Vec3(0.25, -0.10, 0.82);
  state.base_quat_w = Vec4(1.0, 0.0, 0.0, 0.0);
  state.base_ang_vel_b = Vec3(0.1, -0.2, 0.3);
  state.q = Eigen::VectorXd::LinSpaced(kNumJoints, -0.3, 0.3);
  state.qd = Eigen::VectorXd::LinSpaced(kNumJoints, 0.4, -0.4);
  state.base_velocity_xy_w = Vec2(0.37, -0.21);
  state.localization_age = 0.6;

  PpRacketTarget target;
  target.pos_w = Vec3(0.72, -0.42, 1.03);
  target.vel_w = Vec3(2.1, 0.2, 0.7);
  target.base_target_xy = Vec2(0.03, -0.24);
  target.time_to_strike = 0.38;

  const Eigen::VectorXd last_action =
      Eigen::VectorXd::LinSpaced(kNumJoints, -0.8, 0.8);
  const Eigen::VectorXd default_q =
      Eigen::VectorXd::LinSpaced(kNumJoints, -0.1, 0.1);

  const Eigen::VectorXd obs110 =
      build_obs_110(state, target, last_action, default_q);
  const Eigen::VectorXd obs113 =
      build_obs_113(state, target, last_action, default_q);

  ASSERT_EQ(obs110.size(), kObsDim110);
  ASSERT_EQ(obs113.size(), kObsDim113);
  EXPECT_TRUE(obs113.head(kObsDim110).isApprox(obs110, 0.0));
  EXPECT_DOUBLE_EQ(obs113[kObsDim110], state.base_velocity_xy_w[0]);
  EXPECT_DOUBLE_EQ(obs113[kObsDim110 + 1], state.base_velocity_xy_w[1]);
  EXPECT_DOUBLE_EQ(obs113[kObsDim110 + 2], state.localization_age);
}

TEST(PpObsBuilder, ContinuousV4OnlyOverwritesHeadFeedbackWithWorldVxy) {
  PpRobotState state;
  state.base_pos_w = Vec3(0.25, -0.10, 0.82);
  state.base_quat_w = Vec4(1.0, 0.0, 0.0, 0.0);
  state.base_ang_vel_b = Vec3(0.1, -0.2, 0.3);
  state.q = Eigen::VectorXd::LinSpaced(kNumJoints, -0.3, 0.3);
  state.qd = Eigen::VectorXd::LinSpaced(kNumJoints, 0.4, -0.4);
  state.base_velocity_xy_w = Vec2(0.37, -0.21);

  PpRacketTarget target;
  target.pos_w = Vec3(0.72, -0.42, 1.03);
  target.vel_w = Vec3(2.1, 0.2, 0.7);
  target.base_target_xy = Vec2(0.03, -0.24);
  target.time_to_strike = 0.38;

  const Eigen::VectorXd last_action =
      Eigen::VectorXd::LinSpaced(kNumJoints, -0.8, 0.8);
  const Eigen::VectorXd default_q =
      Eigen::VectorXd::LinSpaced(kNumJoints, -0.1, 0.1);
  const Eigen::VectorXd legacy =
      build_obs_110(state, target, last_action, default_q);
  const Eigen::VectorXd v4 =
      build_obs_110_headslots_vxy(state, target, last_action, default_q);

  ASSERT_EQ(v4.size(), kObsDim110);
  EXPECT_EQ(kHitterPureHeadYawActionIndex, 11);
  EXPECT_EQ(kHitterPureHeadPitchActionIndex, 16);
  EXPECT_EQ(kHitterPureBaseVelocityXObsIndex, 76);
  EXPECT_EQ(kHitterPureBaseVelocityYObsIndex, 81);
  EXPECT_DOUBLE_EQ(v4[76], 0.37);
  EXPECT_DOUBLE_EQ(v4[81], -0.21);
  for (int index = 0; index < kObsDim110; ++index) {
    if (index == 76 || index == 81) continue;
    EXPECT_DOUBLE_EQ(v4[index], legacy[index]) << "obs index " << index;
  }
  // The legacy builder is unchanged and still carries the previous action values.
  EXPECT_DOUBLE_EQ(legacy[76], last_action[11]);
  EXPECT_DOUBLE_EQ(legacy[81], last_action[16]);
}

TEST(PpObsBuilder, Schema27KeepsExactSchema26PrefixAndAppendsPlannerPermissions) {
  PpRobotState state;
  state.base_pos_w = Vec3(0.25, -0.10, 0.82);
  state.base_quat_w = Vec4(1.0, 0.0, 0.0, 0.0);
  state.base_ang_vel_b = Vec3(0.1, -0.2, 0.3);
  state.q = Eigen::VectorXd::LinSpaced(kNumJoints, -0.3, 0.3);
  state.qd = Eigen::VectorXd::LinSpaced(kNumJoints, 0.4, -0.4);
  state.base_velocity_xy_w = Vec2(0.37, -0.21);

  PpRacketTarget target;
  target.pos_w = Vec3(0.72, -0.42, 1.03);
  target.vel_w = Vec3(2.1, 0.2, 0.7);
  target.base_target_xy = Vec2(0.03, -0.24);
  target.time_to_strike = 0.38;
  target.reach_level = 2.0;
  target.swing_foot_sign = -1.0;

  const Eigen::VectorXd last_action =
      Eigen::VectorXd::LinSpaced(kNumJoints, -0.8, 0.8);
  const Eigen::VectorXd default_q =
      Eigen::VectorXd::LinSpaced(kNumJoints, -0.1, 0.1);
  const Eigen::VectorXd schema26 =
      build_obs_110_headslots_vxy(state, target, last_action, default_q);
  const Eigen::VectorXd schema27 =
      build_obs_112_headslots_vxy_reach(state, target, last_action, default_q);

  ASSERT_EQ(schema27.size(), kObsDim112);
  EXPECT_TRUE(schema27.head(kObsDim110).isApprox(schema26, 0.0));
  EXPECT_DOUBLE_EQ(schema27[kObsDim110], 2.0);
  EXPECT_DOUBLE_EQ(schema27[kObsDim110 + 1], -1.0);

  const PpRacketTarget stage0_target{};
  const Eigen::VectorXd stage0 = build_obs_112_headslots_vxy_reach(
      state, stage0_target, last_action, default_q);
  EXPECT_DOUBLE_EQ(stage0[kObsDim110], 0.0);
  EXPECT_DOUBLE_EQ(stage0[kObsDim110 + 1], 0.0);
}

TEST(PpObsBuilder, Schema34KeepsPrefixAndUsesIndependentSignedSupportColumns) {
  PpRobotState state;
  state.base_pos_w = Vec3(0.25, -0.10, 0.82);
  state.base_quat_w = Vec4(1.0, 0.0, 0.0, 0.0);
  state.base_ang_vel_b = Vec3(0.1, -0.2, 0.3);
  state.q = Eigen::VectorXd::LinSpaced(kNumJoints, -0.3, 0.3);
  state.qd = Eigen::VectorXd::LinSpaced(kNumJoints, 0.4, -0.4);
  state.base_velocity_xy_w = Vec2(0.37, -0.21);
  PpRacketTarget target;
  target.pos_w = Vec3(0.72, -0.42, 1.03);
  target.vel_w = Vec3(2.1, 0.2, 0.7);
  target.base_target_xy = Vec2(0.03, -0.24);
  target.time_to_strike = 0.38;
  target.reach_level = 2.0;
  target.swing_foot_sign = -1.0;
  target.signed_external_reach = -2.0;
  target.signed_replant_need = 0.64;
  const Eigen::VectorXd last_action =
      Eigen::VectorXd::LinSpaced(kNumJoints, -0.8, 0.8);
  const Eigen::VectorXd default_q =
      Eigen::VectorXd::LinSpaced(kNumJoints, -0.1, 0.1);

  const Eigen::VectorXd prefix =
      build_obs_110_headslots_vxy(state, target, last_action, default_q);
  const Eigen::VectorXd schema34 =
      build_obs_112_headslots_vxy_signed_support(
          state, target, last_action, default_q);
  ASSERT_EQ(schema34.size(), kObsDim112);
  EXPECT_TRUE(schema34.head(kObsDim110).isApprox(prefix, 0.0));
  EXPECT_DOUBLE_EQ(schema34[kObsDim110], -2.0);
  EXPECT_DOUBLE_EQ(schema34[kObsDim110 + 1], 0.64);
}

TEST(PpObsBuilder, Schema28KeepsExactSchema27PrefixAndAppendsCoreOwnedShadow) {
  PpRobotState state;
  state.base_pos_w = Vec3(0.25, -0.10, 0.82);
  state.base_quat_w = Vec4(1.0, 0.0, 0.0, 0.0);
  state.base_ang_vel_b = Vec3(0.1, -0.2, 0.3);
  state.q = Eigen::VectorXd::LinSpaced(kNumJoints, -0.3, 0.3);
  state.qd = Eigen::VectorXd::LinSpaced(kNumJoints, 0.4, -0.4);
  state.base_velocity_xy_w = Vec2(0.37, -0.21);
  PpRacketTarget target;
  target.pos_w = Vec3(0.72, -0.42, 1.03);
  target.vel_w = Vec3(2.1, 0.2, 0.7);
  target.base_target_xy = Vec2(0.03, -0.24);
  target.time_to_strike = 0.38;
  target.reach_level = 1.0;
  target.swing_foot_sign = 1.0;
  const Eigen::VectorXd last_action =
      Eigen::VectorXd::LinSpaced(kNumJoints, -0.8, 0.8);
  const Eigen::VectorXd default_q =
      Eigen::VectorXd::LinSpaced(kNumJoints, -0.1, 0.1);
  const Eigen::VectorXd shadow =
      Eigen::VectorXd::LinSpaced(kCoreActionShadowDim, -0.7, 0.7);
  const Eigen::VectorXd schema27 = build_obs_112_headslots_vxy_reach(
      state, target, last_action, default_q);
  const Eigen::VectorXd schema28 =
      build_obs_127_headslots_vxy_reach_core_shadow(
          state, target, last_action, default_q, shadow);
  ASSERT_EQ(schema28.size(), kObsDim127);
  EXPECT_TRUE(schema28.head(kObsDim112).isApprox(schema27, 0.0));
  EXPECT_TRUE(schema28.tail(kCoreActionShadowDim).isApprox(shadow, 0.0));
  EXPECT_THROW(
      build_obs_127_headslots_vxy_reach_core_shadow(
          state, target, last_action, default_q,
          Eigen::VectorXd::Zero(kCoreActionShadowDim - 1)),
      std::invalid_argument);
}

TEST(PpObsBuilder, ContinuousV4VelocitySlotsMatchResolvedIsaacActionOrder) {
  const std::vector<std::string> resolved_isaac_action_order = {
      "left_hip_pitch_joint", "right_hip_pitch_joint", "waist_yaw_joint",
      "left_hip_roll_joint", "right_hip_roll_joint", "waist_roll_joint",
      "left_hip_yaw_joint", "right_hip_yaw_joint", "waist_pitch_joint",
      "left_knee_joint", "right_knee_joint", "head_yaw_joint",
      "left_shoulder_pitch_joint", "right_shoulder_pitch_joint",
      "left_ankle_pitch_joint", "right_ankle_pitch_joint", "head_pitch_joint",
      "left_shoulder_roll_joint", "right_shoulder_roll_joint",
      "left_ankle_roll_joint", "right_ankle_roll_joint",
      "left_shoulder_yaw_joint", "right_shoulder_yaw_joint",
      "left_elbow_joint", "right_elbow_joint", "left_wrist_roll_joint",
      "right_wrist_roll_joint", "left_wrist_pitch_joint",
      "right_wrist_pitch_joint", "left_wrist_yaw_joint",
      "right_wrist_yaw_joint"};
  ASSERT_EQ(resolved_isaac_action_order.size(), kNumJoints);
  EXPECT_TRUE(hitter_pure_v4_velocity_slot_joint_order_matches(
      resolved_isaac_action_order));

  // Backend/SDK order has the same joint names but the head at 3/4; accepting it would recreate
  // the exact global-68/69 ABI bug that this contract is intended to reject.
  auto backend_order = resolved_isaac_action_order;
  std::swap(backend_order[3], backend_order[11]);
  std::swap(backend_order[4], backend_order[16]);
  EXPECT_FALSE(hitter_pure_v4_velocity_slot_joint_order_matches(backend_order));

  auto wrong_pitch_slot = resolved_isaac_action_order;
  std::swap(wrong_pitch_slot[16], wrong_pitch_slot[17]);
  EXPECT_FALSE(hitter_pure_v4_velocity_slot_joint_order_matches(wrong_pitch_slot));
}

TEST(PpObsBuilder, RepairFreshOutageAndReacquirePreserveVerifiedStationInExact110Layout) {
  PpRobotState state;
  state.base_pos_w = Vec3(-0.50, -1.00, 0.90);
  state.base_quat_w = Vec4(1.0, 0.0, 0.0, 0.0);
  state.base_ang_vel_b.setZero();
  state.q = Eigen::VectorXd::Zero(kNumJoints);
  state.qd = Eigen::VectorXd::Zero(kNumJoints);

  PpRacketTarget target;
  target.pos_w = Vec3(0.58, -0.20, 1.05);
  target.vel_w = Vec3(2.0, 0.1, 0.7);
  target.time_to_strike = 1.0;
  target.base_target_xy = Vec2(
      planner_stale_station_coordinate(true, -0.35, state.base_pos_w[0]),
      planner_stale_station_coordinate(true, -0.80, state.base_pos_w[1]));

  const Eigen::VectorXd zeros = Eigen::VectorXd::Zero(kNumJoints);
  const Eigen::VectorXd fresh = build_obs_110(state, target, zeros, zeros);
  ASSERT_EQ(fresh.size(), kObsDim110);
  ASSERT_EQ(zeros.size(), kNumJoints);
  EXPECT_DOUBLE_EQ(fresh[101], 0.15);
  EXPECT_DOUBLE_EQ(fresh[102], 0.20);

  // A localization-fresh outage holds the complete base pose and verified station, so every
  // actor-visible component is byte-for-byte unchanged across the stale run.
  const Eigen::VectorXd outage = build_obs_110(state, target, zeros, zeros);
  EXPECT_TRUE(outage.isApprox(fresh, 0.0));

  // First fresh reacquisition updates the base pose while retaining the station. The delta may
  // jump because the plant moved during the outage, but must not be replaced by an artificial
  // zero. Velocity validity is internal and does not add a 110-D channel.
  state.base_pos_w = Vec3(-0.44, -0.94, 0.90);
  const Eigen::VectorXd reacquired = build_obs_110(state, target, zeros, zeros);
  ASSERT_EQ(reacquired.size(), kObsDim110);
  EXPECT_DOUBLE_EQ(reacquired[101], 0.09);
  EXPECT_DOUBLE_EQ(reacquired[102], 0.14);

  // Historical policies retain their old outage fallback: station equals the current/held base.
  target.base_target_xy = Vec2(
      planner_stale_station_coordinate(false, -0.35, state.base_pos_w[0]),
      planner_stale_station_coordinate(false, -0.80, state.base_pos_w[1]));
  const Eigen::VectorXd cold = build_obs_110(state, target, zeros, zeros);
  EXPECT_DOUBLE_EQ(cold[101], 0.0);
  EXPECT_DOUBLE_EQ(cold[102], 0.0);
  EXPECT_TRUE(cold.head(101).isApprox(reacquired.head(101), 0.0));
  EXPECT_TRUE(cold.tail(kObsDim110 - 103).isApprox(
      reacquired.tail(kObsDim110 - 103), 0.0));
}

TEST(PpObsBuilder, V15ClampsLocalizationAgeOnly) {
  PpRobotState state;
  state.base_pos_w.setZero();
  state.base_quat_w = Vec4(1.0, 0.0, 0.0, 0.0);
  state.base_ang_vel_b.setZero();
  state.q = Eigen::VectorXd::Zero(kNumJoints);
  state.qd = Eigen::VectorXd::Zero(kNumJoints);
  state.base_velocity_xy_w = Vec2(-1.2, 2.4);
  state.localization_age = 4.0;

  PpRacketTarget target;
  target.pos_w.setZero();
  target.vel_w.setZero();
  target.base_target_xy.setZero();
  target.time_to_strike = 0.0;

  const Eigen::VectorXd zeros = Eigen::VectorXd::Zero(kNumJoints);
  const Eigen::VectorXd obs = build_obs_113(state, target, zeros, zeros);
  EXPECT_DOUBLE_EQ(obs[kObsDim110], -1.2);
  EXPECT_DOUBLE_EQ(obs[kObsDim110 + 1], 2.4);
  EXPECT_DOUBLE_EQ(obs[kObsDim110 + 2], 1.0);
}

TEST(PpObsBuilder, RewrittenV15KeepsExact113PrefixAndAppendsFiniteGaitCommand) {
  PpRobotState state;
  state.base_pos_w = Vec3(0.0, 0.1, 1.0);
  state.base_quat_w = Vec4(1.0, 0.0, 0.0, 0.0);
  state.base_ang_vel_b.setZero();
  state.q = Eigen::VectorXd::Zero(kNumJoints);
  state.qd = Eigen::VectorXd::Zero(kNumJoints);
  state.base_velocity_xy_w = Vec2(0.0, -0.25);
  state.localization_age = 0.2;

  PpRacketTarget target;
  target.pos_w = Vec3(0.58, -0.3, 1.0);
  target.vel_w = Vec3(2.0, 0.2, 0.8);
  target.base_target_xy = Vec2(0.0, -0.2);
  target.time_to_strike = 0.8;
  target.desired_lateral_velocity = -0.3;
  target.gait_clock = Vec2(0.25, -0.75);
  target.locomotion_mode = 1.0;
  target.upper_intervention = 0.0;

  const Eigen::VectorXd zeros = Eigen::VectorXd::Zero(kNumJoints);
  const Eigen::VectorXd obs113 = build_obs_113(state, target, zeros, zeros);
  const Eigen::VectorXd obs118 = build_obs_118(state, target, zeros, zeros);
  ASSERT_EQ(obs118.size(), kObsDim118);
  EXPECT_TRUE(obs118.head(kObsDim113).isApprox(obs113, 0.0));
  EXPECT_DOUBLE_EQ(obs118[113], -0.3);
  EXPECT_DOUBLE_EQ(obs118[114], 0.25);
  EXPECT_DOUBLE_EQ(obs118[115], -0.75);
  EXPECT_DOUBLE_EQ(obs118[116], 1.0);
  EXPECT_DOUBLE_EQ(obs118[117], 0.0);
}

}  // namespace
}  // namespace a3_pingpong
