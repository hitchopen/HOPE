#pragma once

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace a3_pingpong {

inline constexpr char kSchema29SideRehearsalContract[] =
    "build2_side_prior,fixed_home,nominal_fh_bh_arm_teacher";
inline constexpr char kSchema29TeacherRetentionContract[] =
    "safe_projected_build1_actor_mean,nominal_fh_bh_arms14,"
    "followthrough=0.12,inset=0.08,huber=0.10,"
    "coefficient=0.25->0.05,gradient_ratio_max=0.03";
inline constexpr char kSchema29QuestionBankSha256[] =
    "894bea744fc5b119312b619c0368afe9ae048d1b076f67e80f25f7c1111907ee";
inline constexpr char kSchema29QuestionBankReceiptSha256[] =
    "809a39b7105229ca89d627fdf508a4d4763a709824e0d69b766bb96d5ec84eec";
inline constexpr char kSchema29MocapStaleReceiptSha256[] =
    "ed805160905888fddc12a388f0ed27aaa11951283301c10496b0925a1401c82c";
inline constexpr char kSchema29MocapStaleCurriculumContract[] =
    "same_run_linear_steps_384000_256000;actor_visible_stale_hold;"
    "first_fresh_velocity_zero_v1";
inline constexpr char kSchema31TrainingRecipe[] =
    "hitter_pingpong_build2_stable_home_step_candidate_v1";
inline constexpr char kSchema31BuildContract[] =
    "build2_stable_home_step_candidate_schema31";
inline constexpr char kSchema31InitializationContract[] =
    "schema31_from_build1_model21800_functional_vxy_zero_append_reach_trainable_v1";
inline constexpr char kSchema31ActorArchitectureContract[] =
    "trainable_full31_stable_home_step_candidate_v1";
inline constexpr char kSchema31OptionalReachContract[] =
    "staged_support_intent_sim_training_candidate_v5";
inline constexpr char kSchema31OptionalReachTrajectoryQualification[] =
    "SIM_TRAINING_CANDIDATE";
inline constexpr char kSchema31OptionalReachDeploymentQualification[] =
    "SIM_TRAINING_CANDIDATE_NOT_DEPLOYMENT_QUALIFIED";
inline constexpr char kSchema31RewardContract[] =
    "immutable_home_terminal_tracking_outer_safety_v6";
inline constexpr char kSchema31RecoveryPhaseContract[] =
    "continuous_l0_terminal_selected_foot_release_v6";
inline constexpr char kSchema31SideRehearsalContract[] =
    "bounded_arm_only_contact_linear_v1";
inline constexpr char kSchema31TeacherContract[] =
    "contact_linear_actor_visible_v1";
inline constexpr char kSchema31QuestionBankArtifactContract[] =
    "fixed_home_support_training_candidate_bank_v5";
inline constexpr char kSchema31QuestionBankSha256[] =
    "9a4ff9e89f360e5be25e6ee6df571b30aee93a9f7f88a70ae5a308606e294f7e";
inline constexpr char kSchema31QuestionBankReceiptSha256[] =
    "347d16d386ba372feb3c8bc4c00d23542f69de62f14ccd4ffcb8a881f5b4b493";
inline constexpr char kSchema32TrainingRecipe[] =
    "hitter_pingpong_build2_recoverability_envelope_v1";
inline constexpr char kSchema32BuildContract[] =
    "build2_recoverability_envelope_schema32";
inline constexpr char kSchema32InitializationContract[] =
    "schema32_from_build1_model21800_functional_vxy_zero_append_reach_trainable_v1";
inline constexpr char kSchema32TransitionContract[] =
    "policy_owned_112d_markov_slew_safe_executed_feedback_v2";
inline constexpr char kSchema32ReplayContract[] =
    "markov_side_phase_severity_v3_recovery_debt_perturbed_v1";
// Schema33: same 112-D/31-D V12 slew ABI as Schema32; new identity for the licensed
// single-foot replant recovery, support-geometry replay strata and the direct 112-D actor
// warm start from the Gate3-exercised Schema32 model_5580.
inline constexpr char kSchema33TrainingRecipe[] =
    "hitter_pingpong_build2_active_replant_recovery_v1";
inline constexpr char kSchema33BuildContract[] =
    "build2_active_replant_recovery_schema33";
inline constexpr char kSchema33InitializationContract[] =
    "schema33_from_schema32_model5580_direct_112d_actor_trainable_v1";
inline constexpr char kSchema33ReplayContract[] =
    "markov_side_phase_severity_v3_recovery_debt_support_geometry_perturbed_v2";
inline constexpr char kSchema33RewardContract[] =
    "immutable_home_terminal_tracking_outer_safety_licensed_replant_v7";
inline constexpr char kSchema33RecoveryPhaseContract[] =
    "continuous_l0_terminal_licensed_replant_release_v7";
// Schema34 keeps the V12 112-D/31-D action ABI but reassigns actor columns 110/111 to
// signed external reach and signed current-FK-to-HOME replant need.  Shape compatibility
// with Schema31--33 is therefore insufficient; every identity string remains fail closed.
inline constexpr char kSchema34TrainingRecipe[] =
    "hitter_pingpong_build2_visible_foot_recovery_v1";
inline constexpr char kSchema34BuildContract[] =
    "build2_visible_foot_recovery_schema34";
inline constexpr char kSchema34InitializationContract[] =
    "schema34_from_schema33_model2130_zero_support_columns_trainable_v1";
inline constexpr char kSchema34BalanceInitializationContract[] =
    "schema34_from_schema32_model5580_zero_support_columns_trainable_v2";
inline constexpr char kSchema34BalanceRewardContract[] = "support_consistent_com_footwork_v9";
inline constexpr char kSchema34BalanceRecoveryContract[] = "current_fk_goal_error_across_commit_v9";
inline constexpr char kSchema34StepInitializationContract[] = "schema34_from_schema34v2_model4710_zero_task_columns_trainable_v3";
inline constexpr char kSchema34StepRewardContract[] = "observable_step_task_com_footwork_v10";
inline constexpr char kSchema34StepRecoveryContract[] = "persistent_observable_lift_transport_land_v10";
inline constexpr char kSchema34StepActorObsContract[] = "hitter_pure_112_headslots_step_task_v3";
inline constexpr char kSchema34StepReachPermissionContract[] = "signed_external_reach_observable_footstep_task_v7";
inline constexpr char kSchema34StepActorReachSources[] = "planner_signed_reach_level_foot,mocap_imu_fk_visible_footstep_task";
inline constexpr char kSchema34StepNonprivilegedContract[] = "planner_plus_mocap_imu_fk_observable_task_v5";
inline constexpr char kSchema34TransitionContract[] =
    "policy_owned_112d_signed_support_slew_safe_executed_feedback_v3";
inline constexpr char kSchema34ReplayContract[] =
    "markov_side_phase_severity_v3_recovery_debt_support_geometry_perturbed_v2";
inline constexpr char kSchema34RewardContract[] =
    "visible_sequential_foot_trajectory_wait_arm_v8";
inline constexpr char kSchema34RecoveryPhaseContract[] =
    "signed_replant_dense_trajectory_no_latch_income_v8";
inline constexpr char kSchema34ActorObsContract[] =
    "hitter_pure_112_headslots_vxy_signed_support_v2";
inline constexpr char kSchema34ReachPermissionContract[] =
    "signed_external_reach_and_fk_replant_need_v6";
inline constexpr char kSchema34ActorReachSources[] =
    "planner_signed_reach_level_foot,fk_home_anchor_signed_need";
inline constexpr char kSchema34NonprivilegedContract[] =
    "planner_plus_fk_home_anchor_no_contact_latch_v4";

inline bool hitter_schema31_candidate_family_recipe(
    const std::string& training_recipe) noexcept {
  return training_recipe == kSchema31TrainingRecipe ||
         training_recipe == kSchema32TrainingRecipe ||
         training_recipe == kSchema33TrainingRecipe ||
         training_recipe == kSchema34TrainingRecipe;
}

// Schema32-family (V12 slew, executed feedback, planner-native rapid COMMIT) = Schema32 + 33.
inline bool hitter_schema32_slew_family_recipe(
    const std::string& training_recipe) noexcept {
  return training_recipe == kSchema32TrainingRecipe ||
         training_recipe == kSchema33TrainingRecipe ||
         training_recipe == kSchema34TrainingRecipe;
}

enum class HitterPureRuntimeContract {
  kLegacy,
  kRallyFinalV1,
  kRallyFinalV2,
  kRallyV15,
  kRallyV17FixedStationBallClockV1,
};

inline const char* hitter_pingpong_entry_tts_buckets_contract(
    const std::string& command_contract) {
  if (command_contract == "strike_followthrough_home_external_commit_v2" ||
      command_contract == "strike_followthrough_home_external_commit_v3" ||
      command_contract == "strike_followthrough_home_external_commit_v4") {
    return "nominal;0.45,0.55;0.40,0.45;0.25,0.35;0.15,0.25;0.45,0.82";
  }
  if (command_contract == "postcontact_fresh_flight_preempt_v1" ||
      command_contract == "fixed_home_fresh_flight_preempt_v1") {
    return "nominal;0.45,0.55;0.35,0.45;0.25,0.35;0.15,0.25;0.55,0.82";
  }
  if (command_contract == "strike_followthrough_home_external_commit_v1" ||
      command_contract == "native_clip_end_immediate_resample_v1" ||
      command_contract == "home_preempt_raw_tts_v3" ||
      command_contract == "late_reveal_raw_tts_v2") {
    return "nominal;0.45,0.55;0.40,0.45;0.25,0.35;0.15,0.25";
  }
  throw std::runtime_error(
      "unsupported HitterPingPong command contract for entry-TTS buckets");
}

// Schema22 added an empirical cold-entry bucket after model3440 was exported.
// Keep that immutable artifact loadable while requiring exact canonical metadata
// for every other command contract.  The actor-entry core support remains a
// separate four-number contract and is intentionally not widened here.
inline bool hitter_pingpong_entry_tts_buckets_metadata_compatible(
    const std::string& command_contract,
    const std::string& entry_tts_buckets) {
  if (entry_tts_buckets ==
      hitter_pingpong_entry_tts_buckets_contract(command_contract)) {
    return true;
  }
  return (command_contract == "postcontact_fresh_flight_preempt_v1" ||
          command_contract == "fixed_home_fresh_flight_preempt_v1") &&
         entry_tts_buckets ==
             "nominal;0.45,0.55;0.40,0.45;0.25,0.35;0.15,0.25";
}

inline std::array<double, 2> hitter_pingpong_ready_hold_steps_contract(
    const std::string& training_recipe) {
  if (hitter_schema31_candidate_family_recipe(training_recipe)) {
    return {150.0, 250.0};
  }
  if (training_recipe == "hitter_pingpong_continuous_rally_v3" ||
      training_recipe == "hitter_pingpong_continuous_rally_v4") {
    return {0.0, 0.0};
  }
  if (training_recipe == "hitter_pingpong_continuous_rally_v1" ||
      training_recipe == "hitter_pingpong_continuous_rally_v2" ||
      training_recipe == "hitter_pingpong_build2_minimal_rootfix_v1" ||
      training_recipe == "hitter_pingpong_build2_rapid_preempt_v1" ||
      training_recipe == "hitter_pingpong_build2_fixed_home_recovery_v1" ||
      training_recipe == "hitter_pingpong_build2_fixed_home_feasible_v1" ||
      training_recipe ==
          "hitter_pingpong_build2_fixed_home_phase_recovery_v1" ||
      training_recipe == "hitter_pingpong_build2_home_support_frame_v1" ||
      training_recipe == "hitter_pingpong_build2_torso_optional_reach_v2" ||
      training_recipe == "hitter_pingpong_build2_frozen_core_optional_reach_v1" ||
      training_recipe == "hitter_pingpong_build2_trainable_home_optional_reach_v1") {
    return {45.0, 60.0};
  }
  throw std::runtime_error(
      "ready-hold contract only supports continuous-rally-v1/v2/v3/v4 and "
      "build2-minimal-rootfix-v1/build2-rapid-preempt-v1/"
      "build2-fixed-home-recovery-v1/build2-fixed-home-feasible-v1/"
      "build2-fixed-home-phase-recovery-v1/build2-home-support-frame-v1/"
      "build2-torso-optional-reach-v2/build2-frozen-core-optional-reach-v1/"
      "build2-stable-home-step-candidate-v1");
}

inline void validate_deploy_training_recipe_pair(
    const std::string& onnx_recipe,
    const std::string& onnx_recipe_version,
    const std::string& deploy_recipe,
    const std::string& deploy_recipe_version) {
  if (deploy_recipe != onnx_recipe ||
      deploy_recipe_version != onnx_recipe_version) {
    throw std::runtime_error(
        "deploy.yaml training recipe name/version do not exactly match ONNX metadata");
  }
}

// Continuous-rally v1 intentionally retains its old model7800 initialization provenance. V2 and
// v3 both start from the corrected model21800 actor with fresh YAML std/critic/optimizer. V4 keeps
// that actor mean but explicitly zero-initializes the two newly active input columns. Schema27
// stage 0 starts from Schema26 model4470 and appends two exact-zero Planner permission
// columns. The reach-enabled continuation starts from the resulting 112-D Schema27 model4470
// without another input transform. Keep both exact pairings here so neither lineage can borrow the
// other's receipt or silently change the meaning of observation columns.
inline bool hitter_schema27_stage0_initialization_tuple(
    const std::string& initialization_contract,
    const std::string& actor_warm_start_receipt) noexcept {
  return initialization_contract ==
             "schema26_model4470_append_zero_reach_cols110_111_v1" &&
         actor_warm_start_receipt ==
             "model_4470.pt,sha256="
             "912a5a3da26dab818e25ad369d78e5f620b589870e1074d48cdaa8cfcb536c28,"
             "source_input_dim=110,destination_input_dim=112,"
             "appended_zero_input_columns=110|111";
}

inline bool hitter_schema27_enabled_initialization_tuple(
    const std::string& initialization_contract,
    const std::string& actor_warm_start_receipt) noexcept {
  return initialization_contract ==
             "schema27_stage0_model4470_frozen_core_zero_residual_v4" &&
         actor_warm_start_receipt ==
             "model_4470.pt,sha256="
             "49feef14e8f895fa8e408abab6aafce09519dce6122cc81ba94632362874eebd,"
             "source_input_dim=112,destination_input_dim=112,"
             "frozen_core=true,zero_residuals=true";
}

inline bool hitter_schema28_initialization_tuple(
    const std::string& initialization_contract,
    const std::string& actor_warm_start_receipt) noexcept {
  return initialization_contract ==
             "schema28_from_build1_model21800_functional_vxy_zero_reach_frozen_core_shadow_v1" &&
         actor_warm_start_receipt ==
             "model_21800.pt,sha256="
             "daca62616178177647cf9949128582785529dbe6b7ba080fc96d86da23916561,"
             "source_input_dim=110,frozen_core_input_dim=112,"
             "destination_actor_observation_dim=127,"
             "zeroed_source_cols=76|81,appended_zero_cols=110|111,frozen_core=true,"
             "core_shadow_feedback=true,zero_residuals=true";
}

inline bool hitter_schema29_initialization_tuple(
    const std::string& initialization_contract,
    const std::string& actor_warm_start_receipt) noexcept {
  return initialization_contract ==
             "schema29_from_build1_model21800_functional_vxy_zero_append_reach_trainable_v1" &&
         actor_warm_start_receipt ==
             "model_21800.pt,sha256="
             "daca62616178177647cf9949128582785529dbe6b7ba080fc96d86da23916561,"
             "source_input_dim=110,destination_input_dim=112,"
             "zeroed_source_cols=76|81,appended_zero_cols=110|111,"
             "trainable_full31=true";
}

inline bool hitter_schema31_initialization_tuple(
    const std::string& initialization_contract,
    const std::string& actor_warm_start_receipt) noexcept {
  return initialization_contract == kSchema31InitializationContract &&
         actor_warm_start_receipt ==
             "model_21800.pt,sha256="
             "daca62616178177647cf9949128582785529dbe6b7ba080fc96d86da23916561,"
             "source_input_dim=110,destination_input_dim=112,"
             "zeroed_source_cols=76|81,appended_zero_cols=110|111,"
             "trainable_full31=true";
}

inline bool hitter_schema32_initialization_tuple(
    const std::string& initialization_contract,
    const std::string& actor_warm_start_receipt) noexcept {
  return initialization_contract == kSchema32InitializationContract &&
         actor_warm_start_receipt ==
             "model_21800.pt,sha256="
             "daca62616178177647cf9949128582785529dbe6b7ba080fc96d86da23916561,"
             "source_input_dim=110,destination_input_dim=112,"
             "zeroed_source_cols=76|81,appended_zero_cols=110|111,"
             "trainable_full31=true";
}

inline bool hitter_schema33_initialization_tuple(
    const std::string& initialization_contract,
    const std::string& actor_warm_start_receipt) noexcept {
  return initialization_contract == kSchema33InitializationContract &&
         actor_warm_start_receipt ==
             "model_5580.pt,sha256="
             "79423210ecbe3eb7e863ae4bf196eb22fb445d1f6c942796131d4a79e0e311e0,"
             "source_input_dim=112,destination_input_dim=112,"
             "direct_actor_state=true,trainable_full31=true";
}

inline bool hitter_schema34_initialization_tuple(
    const std::string& initialization_contract,
    const std::string& actor_warm_start_receipt) noexcept {
  if (initialization_contract == kSchema34StepInitializationContract)
    return actor_warm_start_receipt == "model_4710.pt,sha256=c7216a9358452dee1f38a5232a1c08e2beb6cdc2bd4ab785f003b6e3b3207398,source_input_dim=112,destination_input_dim=112,zeroed_semantic_cols=14|19|45|50|111,input_transform=zero_semantically_reassigned_task_columns_14_19_45_50_111_v1,trainable_full31=true";
  if (initialization_contract == kSchema34BalanceInitializationContract)
    return actor_warm_start_receipt ==
        "model_5580.pt,sha256=79423210ecbe3eb7e863ae4bf196eb22fb445d1f6c942796131d4a79e0e311e0,"
        "source_input_dim=112,destination_input_dim=112,zeroed_semantic_cols=110|111,"
        "input_transform=zero_semantically_reassigned_support_columns_110_111_v1,"
        "trainable_full31=true";
  return initialization_contract == kSchema34InitializationContract &&
         actor_warm_start_receipt ==
             "model_2130.pt,sha256="
             "0df28d798b4bec2af47fbea1e62fcb1371efdc292539f3d08777b63967ac18e3,"
             "source_input_dim=112,destination_input_dim=112,"
             "zeroed_semantic_cols=110|111,"
             "input_transform=zero_semantically_reassigned_support_columns_110_111_v1,"
             "trainable_full31=true";
}

inline void validate_hitter_pingpong_initialization_contract(
    const std::string& training_recipe,
    const std::string& initialization_contract,
    const std::string& actor_warm_start_receipt) {
  const bool continuous_v1 =
      training_recipe == "hitter_pingpong_continuous_rally_v1";
  const bool continuous_v2 =
      training_recipe == "hitter_pingpong_continuous_rally_v2";
  const bool continuous_v3 =
      training_recipe == "hitter_pingpong_continuous_rally_v3";
  const bool continuous_v4 =
      training_recipe == "hitter_pingpong_continuous_rally_v4";
  const bool build2_minimal_rootfix =
      training_recipe == "hitter_pingpong_build2_minimal_rootfix_v1";
  const bool build2_rapid_preempt =
      training_recipe == "hitter_pingpong_build2_rapid_preempt_v1";
  const bool build2_fixed_home =
      training_recipe == "hitter_pingpong_build2_fixed_home_recovery_v1" ||
      training_recipe == "hitter_pingpong_build2_fixed_home_feasible_v1" ||
      training_recipe ==
          "hitter_pingpong_build2_fixed_home_phase_recovery_v1" ||
      training_recipe == "hitter_pingpong_build2_home_support_frame_v1" ||
      training_recipe == "hitter_pingpong_build2_torso_optional_reach_v2" ||
      training_recipe == "hitter_pingpong_build2_frozen_core_optional_reach_v1" ||
      training_recipe == "hitter_pingpong_build2_trainable_home_optional_reach_v1" ||
      hitter_schema31_candidate_family_recipe(training_recipe);
  const bool home_support_frame =
      training_recipe == "hitter_pingpong_build2_home_support_frame_v1";
  const bool torso_optional_reach =
      training_recipe == "hitter_pingpong_build2_torso_optional_reach_v2";
  const bool frozen_core_shadow =
      training_recipe == "hitter_pingpong_build2_frozen_core_optional_reach_v1";
  const bool trainable_schema29 =
      training_recipe == "hitter_pingpong_build2_trainable_home_optional_reach_v1";
  const bool trainable_schema31 = training_recipe == kSchema31TrainingRecipe;
  const bool trainable_schema32 = training_recipe == kSchema32TrainingRecipe;
  const bool trainable_schema33 = training_recipe == kSchema33TrainingRecipe;
  const bool trainable_schema34 = training_recipe == kSchema34TrainingRecipe;
  const bool fixed_home_family = build2_fixed_home || torso_optional_reach;
  if (!continuous_v1 && !continuous_v2 && !continuous_v3 && !continuous_v4 &&
      !build2_minimal_rootfix && !build2_rapid_preempt && !fixed_home_family) {
    throw std::runtime_error(
        "HitterPingPong initialization pairing only supports continuous-rally-v1/v2/v3/v4 "
        "and build2-minimal-rootfix-v1/build2-rapid-preempt-v1/"
        "build2-fixed-home-recovery-v1/build2-fixed-home-feasible-v1/"
        "build2-fixed-home-phase-recovery-v1/build2-home-support-frame-v1/"
        "build2-torso-optional-reach-v2/build2-frozen-core-optional-reach-v1");
  }
  if (trainable_schema34) {
    if (!hitter_schema34_initialization_tuple(
            initialization_contract, actor_warm_start_receipt)) {
      throw std::runtime_error(
          "Schema34 initialization must declare the exact Schema33 model_2130 "
          "112-D semantic-column-zeroing provenance tuple");
    }
    return;
  }
  if (trainable_schema33) {
    if (!hitter_schema33_initialization_tuple(
            initialization_contract, actor_warm_start_receipt)) {
      throw std::runtime_error(
          "Schema33 initialization must declare the exact direct 112-D Schema32 "
          "model_5580 actor provenance tuple");
    }
    return;
  }
  if (trainable_schema32) {
    if (!hitter_schema32_initialization_tuple(
            initialization_contract, actor_warm_start_receipt)) {
      throw std::runtime_error(
          "Schema32 initialization must declare the exact Build1 110-to-112 "
          "functional vxy-zero/reach-append trainable-full31 provenance tuple");
    }
    return;
  }
  if (trainable_schema31) {
    if (!hitter_schema31_initialization_tuple(
            initialization_contract, actor_warm_start_receipt)) {
      throw std::runtime_error(
          "Schema31 initialization must declare the exact Build1 110-to-112 "
          "functional vxy-zero/reach-append trainable-full31 provenance tuple");
    }
    return;
  }
  if (trainable_schema29) {
    if (!hitter_schema29_initialization_tuple(
            initialization_contract, actor_warm_start_receipt)) {
      throw std::runtime_error(
          "Schema29 initialization must declare the exact Build1 110-to-112 "
          "functional vxy-zero/reach-append trainable-full31 provenance tuple");
    }
    return;
  }
  if (frozen_core_shadow) {
    if (!hitter_schema28_initialization_tuple(
            initialization_contract, actor_warm_start_receipt)) {
      throw std::runtime_error(
          "Schema28 initialization must declare the exact Build1 110-to-112-to-127 "
          "functional frozen-core shadow-feedback model21800 provenance tuple");
    }
    return;
  }
  if (torso_optional_reach) {
    const bool stage0_append = hitter_schema27_stage0_initialization_tuple(
        initialization_contract, actor_warm_start_receipt);
    const bool enabled_direct = hitter_schema27_enabled_initialization_tuple(
        initialization_contract, actor_warm_start_receipt);
    if (stage0_append == enabled_direct) {
      throw std::runtime_error(
          "Schema27 initialization must declare exactly one coherent append-zero "
          "or direct-112D model4470 provenance tuple");
    }
    return;
  }
  const std::string expected_initialization =
      (continuous_v4 || home_support_frame)
          ? "build1_model21800_functional_zero_vxy_cols76_81_v1"
          : (continuous_v2 || continuous_v3 || build2_minimal_rootfix ||
             build2_rapid_preempt || fixed_home_family)
          ? "build1_model21800_actor_fresh_yaml_std0p19_raw_obs_"
            "fresh_critic_optimizer_v1"
          : "build2_model7800_actor_std_raw_obs_fresh_critic_optimizer_v1";
  const std::string expected_actor_receipt =
      (continuous_v2 || continuous_v3 || continuous_v4 ||
       build2_minimal_rootfix || build2_rapid_preempt || fixed_home_family)
          ? "model_21800.pt,"
            "sha256=daca62616178177647cf9949128582785529dbe6b7ba080fc96d86da23916561,"
            "exported_onnx_sha256=6bf1a2418f8538e23577a0153f2fe6a1e78dee91f41650a232259432a84a4dc8"
          : "model_7800.pt,"
            "sha256=1e659a56b94955215bd851621f22e4f612f68ab5f25c7b61fce6398f8110a87d,"
            "exported_onnx_sha256=aa24c36b025ce208df70f797103de67ebbff7d374dbfafcc5cffb2a1cffbaefe";
  if (initialization_contract != expected_initialization ||
      actor_warm_start_receipt != expected_actor_receipt) {
    throw std::runtime_error(
        "continuous-rally recipe and actor initialization provenance do not exactly match");
  }
}

// Each standalone Hitter recipe has one exact command wire. Keep this
// check outside the ONNX parser so production and unit tests exercise the same fail-closed rule.
inline void validate_hitter_pingpong_command_recipe_contract(
    const std::string& training_recipe, const std::string& command_contract) {
  if (training_recipe == "small_station_compact_execution_candidate_v1" ||
      training_recipe == "hitter_small_station_independent_recovery_v2" ||
      training_recipe == "hitter_small_station_support_heading_recovery_v3" ||
      training_recipe == "hitter_small_station_matched_plant_recovery_v4" ||
      training_recipe == "hitter_small_station_hold_resume_recovery_v5" ||
      command_contract == "small_station_external_schedule_precommit_settle_v1") {
    if (command_contract != "small_station_external_schedule_precommit_settle_v1")
      throw std::runtime_error("SmallStation command contract mismatch");
    return;
  }
  const bool build3_recipe =
      training_recipe == "hitter_pingpong_recovery_tts_v3";
  const bool build4_recipe =
      training_recipe == "hitter_pingpong_recovery_tts_v2";
  const bool continuous_recipe_v1 =
      training_recipe == "hitter_pingpong_continuous_rally_v1";
  const bool continuous_recipe_v2 =
      training_recipe == "hitter_pingpong_continuous_rally_v2";
  const bool continuous_recipe_v3 =
      training_recipe == "hitter_pingpong_continuous_rally_v3";
  const bool continuous_recipe_v4 =
      training_recipe == "hitter_pingpong_continuous_rally_v4";
  const bool build2_minimal_rootfix_recipe =
      training_recipe == "hitter_pingpong_build2_minimal_rootfix_v1";
  const bool build2_rapid_preempt_recipe =
      training_recipe == "hitter_pingpong_build2_rapid_preempt_v1";
  const bool build2_fixed_home_recipe =
      training_recipe == "hitter_pingpong_build2_fixed_home_recovery_v1" ||
      training_recipe == "hitter_pingpong_build2_fixed_home_feasible_v1" ||
      training_recipe ==
          "hitter_pingpong_build2_fixed_home_phase_recovery_v1" ||
      training_recipe == "hitter_pingpong_build2_home_support_frame_v1" ||
      training_recipe ==
          "hitter_pingpong_build2_torso_optional_reach_v2" ||
      training_recipe ==
          "hitter_pingpong_build2_frozen_core_optional_reach_v1" ||
      training_recipe ==
          "hitter_pingpong_build2_trainable_home_optional_reach_v1" ||
      hitter_schema31_candidate_family_recipe(training_recipe);
  const bool build3_command = command_contract == "late_reveal_raw_tts_v2";
  const bool build4_command = command_contract == "home_preempt_raw_tts_v3";
  const bool continuous_command_v1 =
      command_contract == "strike_followthrough_home_external_commit_v1";
  const bool continuous_command_v2 =
      command_contract == "strike_followthrough_home_external_commit_v2";
  const bool continuous_command_v3 =
      command_contract == "strike_followthrough_home_external_commit_v3";
  const bool continuous_command_v4 =
      command_contract == "strike_followthrough_home_external_commit_v4";
  const bool build2_minimal_rootfix_command =
      command_contract == "native_clip_end_immediate_resample_v1";
  const bool build2_rapid_preempt_command =
      command_contract == "postcontact_fresh_flight_preempt_v1";
  const bool build2_fixed_home_command =
      command_contract == "fixed_home_fresh_flight_preempt_v1";
  const bool repair_recipe =
      build3_recipe || build4_recipe || continuous_recipe_v1 ||
      continuous_recipe_v2 || continuous_recipe_v3 || continuous_recipe_v4 ||
      build2_minimal_rootfix_recipe || build2_rapid_preempt_recipe ||
      build2_fixed_home_recipe;
  const bool repair_command =
      build3_command || build4_command || continuous_command_v1 ||
      continuous_command_v2 || continuous_command_v3 || continuous_command_v4 ||
      build2_minimal_rootfix_command || build2_rapid_preempt_command ||
      build2_fixed_home_command;
  const bool exact_pair =
      (build3_recipe && build3_command) ||
      (build4_recipe && build4_command) ||
      (continuous_recipe_v1 && continuous_command_v1) ||
      (continuous_recipe_v2 && continuous_command_v2) ||
      (continuous_recipe_v3 && continuous_command_v3) ||
      (continuous_recipe_v4 && continuous_command_v4) ||
      (build2_minimal_rootfix_recipe && build2_minimal_rootfix_command) ||
      (build2_rapid_preempt_recipe && build2_rapid_preempt_command) ||
      (build2_fixed_home_recipe && build2_fixed_home_command);
  if ((repair_recipe || repair_command) && !exact_pair) {
    throw std::runtime_error(
        "ONNX HitterPingPong repair recipe requires its exact Build3 "
        "late_reveal-v2, Build4 HOME-preempt-v3, continuous-rally-v1, or "
        "continuous-rally-v2/v3/v4, build2-minimal-rootfix-v1, or "
        "build2-rapid-preempt-v1, or build2-fixed-home-recovery-v1/"
        "build2-fixed-home-feasible-v1/build2-fixed-home-phase-recovery-v1/"
        "build2-home-support-frame-v1/build2-torso-optional-reach-v2/"
        "build2-frozen-core-optional-reach-v1/"
        "build2-trainable-home-optional-reach-v1/"
        "build2-stable-home-step-candidate-v1 "
        "command contract");
  }
}

// The minimal rootfix deliberately returns to the original HITTER wire and actor tensor
// semantics.  Its distinguishing contract is temporal: a reset-only neutral HOLD may delay
// reveal, while a completed native clip immediately samples the next swing.  There is no
// contact-relative early wrap and no post-hit arrival clock.  Keep this pure validator shared by
// the loader and tests so a shape-compatible V4 artifact cannot be mislabeled as this recipe.
inline void validate_hitter_pingpong_build2_minimal_rootfix_metadata(
    const std::string& training_recipe,
    const std::string& recipe_version,
    const std::string& build_contract,
    const std::string& command_contract,
    const std::string& planner_commit_contract,
    const std::string& actor_obs_contract,
    const std::string& trajectory_teacher_contract,
    const std::string& motion_phase_contract,
    const std::string& racket_velocity_contract,
    const std::string& wait_clock_contract,
    const std::array<double, 3>& pending_target,
    const std::array<double, 3>& pending_velocity,
    const std::array<double, 3>& pending_normal,
    const std::string& entry_hide_command_during_hold,
    const std::string& contact_early_wrap_enabled,
    const std::string& joint_cost_aggregation_contract,
    const std::string& native_wrap_hold_steps_range,
    const std::string& post_contact_home_contract,
    const std::string& home_return_reward_contract,
    const std::string& lifecycle_contract,
    const std::string& replay_contract) {
  auto exact_vector = [](const std::array<double, 3>& actual,
                         const std::array<double, 3>& expected) {
    for (std::size_t i = 0; i < actual.size(); ++i) {
      if (!std::isfinite(actual[i]) ||
          std::fabs(actual[i] - expected[i]) > 1.0e-12) {
        return false;
      }
    }
    return true;
  };
  if (training_recipe != "hitter_pingpong_build2_minimal_rootfix_v1" ||
      recipe_version != "1" ||
      build_contract != "build2_minimal_rootfix_schema21" ||
      command_contract != "native_clip_end_immediate_resample_v1" ||
      planner_commit_contract != "native_clip_end_immediate_resample_v1" ||
      actor_obs_contract != "hitter_pure" ||
      trajectory_teacher_contract != "contact_linear_actor_visible_v1" ||
      motion_phase_contract != "signed_tts_contact_frame_v1" ||
      racket_velocity_contract != "sim_world_body_v1" ||
      wait_clock_contract != "cold_hold_neutral_hidden_until_release_v1" ||
      !exact_vector(pending_target, {0.58, -0.265, 1.075}) ||
      !exact_vector(pending_velocity, {0.0, 0.0, 0.0}) ||
      !exact_vector(pending_normal, {1.0, 0.0, 0.0}) ||
      entry_hide_command_during_hold != "true" ||
      contact_early_wrap_enabled != "false" ||
      joint_cost_aggregation_contract !=
          "rational_soft_union_all_joints_v1" ||
      native_wrap_hold_steps_range != "0,0" ||
      post_contact_home_contract !=
          "contact_plus_0p12_latch_recovery_and_publish_immutable_home_base_target;"
          "preserve_racket_target_negative_tts_and_native_motion_tail;"
          "no_wrap_until_native_seg_end_v1" ||
      home_return_reward_contract !=
          "yaml_owned_bounded_y_only_home_return_active_v1" ||
      lifecycle_contract !=
          "cold_hold_only;native_clip_end_immediate_resample;no_contact_early_wrap;"
          "no_post_hit_arrival_hazard" ||
      replay_contract != "build2_progressive_0p05_to_0p20_v1") {
    throw std::runtime_error(
        "build2-minimal-rootfix metadata must exactly preserve the original hitter_pure "
        "actor, actor-visible contact-linear target teacher, signed-TTS contact motion, "
        "and sim-world racket velocity, and declare cold-only neutral HOLD, "
        "native-tail resampling, a neutral hidden command, immutable HOME-base recovery "
        "after contact+0.12 without changing racket/clock/motion, a YAML-owned bounded "
        "y-only HOME reward, all-joint aggregation, and Build2 progressive replay");
  }
}

// Schema 22 changes only the Build2 task lifecycle. This exact tuple is the fail-closed
// discriminator between the native-tail schema21 model and the policy-owned rapid-preempt MDP;
// tensor shape and a partial copy of the surrounding Build2 metadata cannot infer it safely.
inline void validate_hitter_pingpong_build2_rapid_preempt_metadata(
    const std::string& training_recipe,
    const std::string& recipe_version,
    const std::string& build_contract,
    const std::string& command_contract,
    const std::string& planner_commit_contract,
    const std::string& transition_contract,
    double no_preempt_s,
    const std::array<double, 2>& preempt_entry_tts_support_s,
    const std::array<double, 2>& preempt_commit_delay_support_s,
    const std::string& preempt_commit_delay_clock_contract,
    const std::array<double, 4>& actor_entry_tts_support_per_clip_s,
    const std::string& preposition_contract,
    const std::string& next_command_arrival_contract,
    const std::string& replay_contract,
    const std::string& entry_hide_command_during_hold,
    const std::string& contact_early_wrap_enabled,
    const std::string& joint_cost_aggregation_contract,
    const std::string& native_wrap_hold_steps_range,
    const std::string& post_contact_home_contract,
    const std::string& home_return_reward_contract,
    const std::string& lifecycle_contract,
    const std::string& post_hit_arrival_contract,
    const std::string& station_mode,
    const std::string& station_contract,
    const std::string& base_target_semantics) {
  const bool fixed_home_recovery =
      training_recipe == "hitter_pingpong_build2_fixed_home_recovery_v1";
  const bool fixed_home_feasible =
      training_recipe == "hitter_pingpong_build2_fixed_home_feasible_v1";
  const bool fixed_home_phase_recovery =
      training_recipe ==
      "hitter_pingpong_build2_fixed_home_phase_recovery_v1";
  const bool fixed_home_support_frame =
      training_recipe == "hitter_pingpong_build2_home_support_frame_v1";
  const bool fixed_home_torso_optional_reach =
      training_recipe == "hitter_pingpong_build2_torso_optional_reach_v2";
  const bool fixed_home_frozen_core_shadow =
      training_recipe == "hitter_pingpong_build2_frozen_core_optional_reach_v1";
  const bool fixed_home_trainable_schema29 =
      training_recipe == "hitter_pingpong_build2_trainable_home_optional_reach_v1";
  const bool fixed_home_trainable_schema31 =
      training_recipe == kSchema31TrainingRecipe;
  const bool fixed_home_trainable_schema33 =
      training_recipe == kSchema33TrainingRecipe;
  const bool fixed_home_trainable_schema34 =
      training_recipe == kSchema34TrainingRecipe;
  // The V12/planner-native ABI branches below are shared by Schema32--34; Schema34 has a
  // distinct observation-semantic transition and all three retain exact identity strings.
  const bool fixed_home_trainable_schema32 =
      training_recipe == kSchema32TrainingRecipe || fixed_home_trainable_schema33 ||
      fixed_home_trainable_schema34;
  const bool optional_reach_family =
      fixed_home_torso_optional_reach || fixed_home_frozen_core_shadow ||
      fixed_home_trainable_schema29 || fixed_home_trainable_schema31 ||
      fixed_home_trainable_schema32;
  const bool fixed_home =
      fixed_home_recovery || fixed_home_feasible || fixed_home_phase_recovery ||
      fixed_home_support_frame || optional_reach_family;
  const bool rapid_preempt =
      training_recipe == "hitter_pingpong_build2_rapid_preempt_v1";
  const bool schema27_planner_commit_contract =
      planner_commit_contract ==
          "fresh_schema2_flight_fixed_session_home_v1" ||
      planner_commit_contract ==
          "fresh_schema3_flight_shared_session_home_support_intent_v2";
  const double expected_commit_delay_min_s =
      optional_reach_family ? 0.50 : 0.40;
  const double expected_commit_delay_max_s =
      optional_reach_family ? 0.65 : 0.55;
  if ((!rapid_preempt && !fixed_home) ||
      (recipe_version != (fixed_home_torso_optional_reach ? "2" : "1") &&
       !(fixed_home_trainable_schema34 && (recipe_version == "2" || recipe_version == "3"))) ||
      build_contract !=
          (fixed_home_trainable_schema34
               ? kSchema34BuildContract
           : fixed_home_trainable_schema33
               ? kSchema33BuildContract
           : fixed_home_trainable_schema32
               ? kSchema32BuildContract
           : fixed_home_trainable_schema31
               ? kSchema31BuildContract
           : fixed_home_trainable_schema29
               ? "build2_trainable_home_optional_reach_schema29"
           : fixed_home_frozen_core_shadow
               ? "build2_frozen_core_shadow_schema28"
           : fixed_home_torso_optional_reach
               ? "build2_torso_optional_reach_schema27_v2"
               : fixed_home_support_frame
               ? "build2_home_support_frame_schema26"
               : fixed_home_phase_recovery
               ? "build2_fixed_home_phase_recovery_schema25"
               : fixed_home_feasible
               ? "build2_fixed_home_feasible_schema24"
               : (fixed_home_recovery
                      ? "build2_fixed_home_recovery_schema23"
                      : "build2_rapid_preempt_schema22")) ||
      command_contract != (fixed_home
          ? "fixed_home_fresh_flight_preempt_v1"
          : "postcontact_fresh_flight_preempt_v1") ||
      ((fixed_home_frozen_core_shadow || fixed_home_trainable_schema29 ||
        fixed_home_trainable_schema31 || fixed_home_trainable_schema32)
           ? planner_commit_contract !=
                 "fresh_schema3_flight_shared_session_home_support_intent_v2"
       : fixed_home_torso_optional_reach
           ? !schema27_planner_commit_contract
           : planner_commit_contract !=
          (fixed_home
               ? "fresh_schema2_flight_fixed_session_home_v1"
               : "fresh_schema2_flight_after_protected_followthrough_v1")) ||
      transition_contract !=
          (fixed_home_trainable_schema34
               ? kSchema34TransitionContract
           : fixed_home_trainable_schema32
               ? kSchema32TransitionContract
           : (fixed_home_trainable_schema29 || fixed_home_trainable_schema31)
               ? "policy_owned_112d_markov_trainable_no_qdes_override_v1"
           : fixed_home_frozen_core_shadow
               ? "policy_owned_127d_markov_frozen_core_shadow_residual_no_qdes_override_v1"
           : fixed_home_torso_optional_reach
               ? "policy_owned_112d_markov_frozen_core_residual_no_qdes_override_v3"
               : fixed_home_support_frame
               ? "policy_owned_110d_markov_headslots_vxy_no_qdes_override_v2"
               : "policy_owned_110d_markov_no_qdes_override_v1") ||
      !std::isfinite(no_preempt_s) ||
      std::fabs(no_preempt_s - 0.12) > 1.0e-12 ||
      !std::isfinite(preempt_entry_tts_support_s[0]) ||
      !std::isfinite(preempt_entry_tts_support_s[1]) ||
      std::fabs(preempt_entry_tts_support_s[0] - 0.60) > 1.0e-12 ||
      std::fabs(preempt_entry_tts_support_s[1] - 0.75) > 1.0e-12 ||
      !std::isfinite(preempt_commit_delay_support_s[0]) ||
      !std::isfinite(preempt_commit_delay_support_s[1]) ||
      std::fabs(preempt_commit_delay_support_s[0] -
                expected_commit_delay_min_s) > 1.0e-12 ||
      std::fabs(preempt_commit_delay_support_s[1] -
                expected_commit_delay_max_s) > 1.0e-12 ||
      preempt_commit_delay_clock_contract !=
          "contact_phase_aligned_control_tick_closed_support_v1" ||
      !std::isfinite(actor_entry_tts_support_per_clip_s[0]) ||
      !std::isfinite(actor_entry_tts_support_per_clip_s[1]) ||
      !std::isfinite(actor_entry_tts_support_per_clip_s[2]) ||
      !std::isfinite(actor_entry_tts_support_per_clip_s[3]) ||
      std::fabs(actor_entry_tts_support_per_clip_s[0] - 0.40) > 1.0e-12 ||
      std::fabs(actor_entry_tts_support_per_clip_s[1] - 0.82) > 1.0e-12 ||
      std::fabs(actor_entry_tts_support_per_clip_s[2] - 0.40) > 1.0e-12 ||
      std::fabs(actor_entry_tts_support_per_clip_s[3] - 0.55) > 1.0e-12 ||
      preposition_contract != (fixed_home
          ? "disabled_fixed_session_home"
          : "disabled_until_atomic_commit") ||
      !next_command_arrival_contract.empty() ||
      replay_contract != (fixed_home
          ? (optional_reach_family
                 ? (fixed_home_trainable_schema34
                        ? kSchema34ReplayContract
                    : fixed_home_trainable_schema33
                        ? kSchema33ReplayContract
                    : fixed_home_trainable_schema32
                        ? kSchema32ReplayContract
                    : (fixed_home_trainable_schema29 ||
                       fixed_home_trainable_schema31)
                        ? "markov_side_phase_severity_v3"
                    : fixed_home_frozen_core_shadow
                        ? "markov_fixed_home_support_reach_side_phase_severity_core_shadow_v8"
                        : "markov_fixed_home_support_reach_side_phase_severity_v7")
             : fixed_home_support_frame
                 ? "markov_fixed_home_support_side_phase_severity_v6"
                 : fixed_home_phase_recovery
                   ? "markov_fixed_home_side_phase_severity_v5"
                   : "markov_fixed_home_side_phase_severity_v4")
          : "build2_progressive_0p05_to_0p20_v1") ||
      entry_hide_command_during_hold != "true" ||
      contact_early_wrap_enabled != "false" ||
      joint_cost_aggregation_contract !=
          "rational_soft_union_all_joints_v1" ||
      native_wrap_hold_steps_range != "0,0" ||
      post_contact_home_contract !=
          (fixed_home
               ? "contact_plus_0p12_latch_recovery_only;station_remains_session_home;"
                 "preserve_old_racket_target_negative_tts_and_motion_tail_until_fresh_"
                 "flight_commit_v1"
               : "contact_plus_0p12_latch_recovery_and_publish_immutable_home_base_target;"
                 "preserve_old_racket_target_negative_tts_and_motion_tail_until_fresh_"
                 "flight_commit_v1") ||
      home_return_reward_contract !=
          (fixed_home
               ? (fixed_home_trainable_schema34 && recipe_version == "3" ? kSchema34StepRecoveryContract : fixed_home_trainable_schema34 && recipe_version == "2"
                      ? kSchema34BalanceRecoveryContract
                      : fixed_home_support_frame
                      || optional_reach_family
                      ? "continuous_immutable_home_support_frame_v1"
                      : "yaml_owned_bounded_xy_post_strike_home_drift_v1")
               : "yaml_owned_bounded_y_only_home_return_active_v1") ||
      lifecycle_contract !=
          (fixed_home
               ? (optional_reach_family
                      ? "cold_hold_neutral;contact_plus_0p12_recovery_latch;"
                        "fresh_external_flight_preempt_0p50_0p65;immutable_"
                        "session_home;policy_owned_transition;no_ready_gate"
                      : "cold_hold_neutral;contact_plus_0p12_recovery_latch;"
                        "fresh_external_flight_preempt_0p40_0p55;immutable_"
                        "session_home;policy_owned_transition;no_ready_gate")
               : "cold_hold_neutral;contact_plus_0p12_home;fresh_external_flight_"
                 "preempt_0p40_0p55;policy_owned_transition;no_ready_gate") ||
      post_hit_arrival_contract !=
          (fixed_home
               ? "fixed_home_fresh_flight_preempt_v1"
               : "postcontact_fresh_flight_preempt_v1") ||
      (fixed_home &&
       (station_mode != "fixed_session_home_v1" ||
        station_contract !=
            ((fixed_home_support_frame || optional_reach_family)
                 ? "immutable_home_support_frame_all_phases_v2"
                 : "immutable_session_home_all_phases_v1") ||
        base_target_semantics != "immutable_home_minus_mocap_base_v1"))) {
    throw std::runtime_error(
        "build2 external-preempt metadata must exactly declare schema22 through schema31, a fresh "
        "schema-2/schema-3 flight after the 0.12 s protected follow-through, and a "
        "policy-owned 110-D/112-D/127-D Markov transition without q_des override, entry "
        "TTS [0.60,0.75], control-tick-quantized commit-delay provenance "
        "[0.40,0.55] for schema22--26 or [0.50,0.65] for schema27--31, the unchanged "
        "schema21 actor-entry support and exact station/reward/replay semantics");
  }
}

enum class HitterOptionalReachRuntimeMode {
  kDisabledSchema2Zero,
  kCandidateSchema3Level0Only,
  kEnabledSchema3,
};

inline bool hitter_optional_reach_mode_uses_schema3(
    HitterOptionalReachRuntimeMode mode) noexcept {
  return mode == HitterOptionalReachRuntimeMode::kCandidateSchema3Level0Only ||
         mode == HitterOptionalReachRuntimeMode::kEnabledSchema3;
}

inline void validate_hitter_schema27_initialization_runtime_pair(
    const std::string& initialization_contract,
    const std::string& actor_warm_start_receipt,
    HitterOptionalReachRuntimeMode runtime_mode) {
  const bool stage0 = hitter_schema27_stage0_initialization_tuple(
      initialization_contract, actor_warm_start_receipt);
  const bool enabled = hitter_schema27_enabled_initialization_tuple(
      initialization_contract, actor_warm_start_receipt);
  const bool runtime_enabled =
      runtime_mode == HitterOptionalReachRuntimeMode::kEnabledSchema3;
  if (stage0 == enabled || runtime_enabled != enabled) {
    throw std::runtime_error(
        "Schema27 initialization lineage must match its disabled-schema2 or "
        "enabled-schema3 runtime metadata mode");
  }
}

inline bool hitter_optional_reach_tuple_is_legal(
    double reach_level, double swing_foot_sign) noexcept {
  return (reach_level == 0.0 && swing_foot_sign == 0.0) ||
         ((reach_level == 1.0 || reach_level == 2.0) &&
          (swing_foot_sign == -1.0 || swing_foot_sign == 1.0));
}

inline bool hitter_optional_reach_active_flight(
    bool planner_engaged, int swing_level) noexcept {
  return planner_engaged && swing_level == 1;
}

inline bool hitter_optional_reach_revision_preserves_flight_permission(
    bool schema27_actor, double latched_reach_level,
    double latched_swing_foot_sign, double candidate_reach_level,
    double candidate_swing_foot_sign) noexcept {
  return !schema27_actor ||
         (latched_reach_level == candidate_reach_level &&
          latched_swing_foot_sign == candidate_swing_foot_sign);
}

// Runner-side integrity check for a Planner permission. It uses only the same
// target/side tuple and immutable session HOME already visible to the policy.
// Zero is ordinary fixed support only inside the production support envelope.
inline bool hitter_schema27_reach_permission_matches_home_target_tuple(
    double target_world_y, double immutable_home_y, double swing_sign,
    double reach_level, double swing_foot_sign) noexcept {
  if (!std::isfinite(target_world_y) || !std::isfinite(immutable_home_y) ||
      (swing_sign != -1.0 && swing_sign != 1.0)) {
    return false;
  }
  const double relative_y = target_world_y - immutable_home_y;
  const bool target_admitted = swing_sign > 0.0
      ? relative_y >= -0.350 && relative_y < -0.250
      : relative_y >= -0.140 && relative_y <= 0.180;
  if (!target_admitted) return false;
  // A zero permission is ordinary fixed support only *inside* the same
  // receipt-bound production envelope. OOD targets may not be laundered by
  // replacing a nonzero permission with (0,0).
  if (reach_level == 0.0 && swing_foot_sign == 0.0) return true;
  const double lateral = std::fabs(relative_y);
  double expected_level = 0.0;
  if (swing_sign > 0.0) {
    expected_level = lateral >= 0.300 ? 2.0 : lateral >= 0.275 ? 1.0 : 0.0;
  } else {
    expected_level = lateral >= 0.120 ? 2.0 : lateral >= 0.090 ? 1.0 : 0.0;
  }
  // The sign names the actual moving foot. Level 1 unloads the foot opposite
  // the lateral target; Level 2 permits the target-side foot to microstep.
  // FH/BH selects thresholds only.
  const double target_sign = relative_y < 0.0 ? -1.0 : 1.0;
  const double expected_foot = expected_level == 1.0
      ? -target_sign
      : expected_level == 2.0 ? target_sign : 0.0;
  return reach_level == expected_level && swing_foot_sign == expected_foot;
}

// Bind the wire revision to the artifact contract.  A reach-enabled artifact may not silently
// consume a legacy schema-2 packet (which would erase the two actor inputs), and an old/disabled
// artifact may not consume schema-3 even when its permission tuple happens to be zero.
// Schema29 keeps the schema-3 ABI while its candidate bank is unqualified, so that mode
// admits only the explicit level-0 tuple until a future certified contract enables reach.
inline bool hitter_optional_reach_wire_is_compatible(
    bool schema27_actor, HitterOptionalReachRuntimeMode mode, int wire_schema,
    double reach_level, double swing_foot_sign) noexcept {
  if (!std::isfinite(reach_level) || !std::isfinite(swing_foot_sign))
    return false;
  if (!schema27_actor) {
    return wire_schema == 2 && reach_level == 0.0 &&
           swing_foot_sign == 0.0;
  }
  if (mode == HitterOptionalReachRuntimeMode::kEnabledSchema3) {
    return wire_schema == 3 &&
           hitter_optional_reach_tuple_is_legal(
               reach_level, swing_foot_sign);
  }
  if (mode == HitterOptionalReachRuntimeMode::kCandidateSchema3Level0Only) {
    return wire_schema == 3 && reach_level == 0.0 &&
           swing_foot_sign == 0.0;
  }
  return wire_schema == 2 && reach_level == 0.0 &&
         swing_foot_sign == 0.0;
}

// Schema27's appended actor inputs are deploy Planner permissions. This tuple prevents an ONNX
// artifact from relabeling contact force, foot height, achieved motion, or any other plant signal
// as those two inputs. Disabled artifacts remain schema-2/zero-only; enabled artifacts bind the
// training-screened target-tuple labels to schema 3. Schema29's candidate mode keeps schema 3
// but accepts level 0 only. Full trajectory/deployment qualification is deliberately not inferred
// from this transport contract.
inline HitterOptionalReachRuntimeMode
validate_hitter_pingpong_schema27_nonprivileged_metadata(
    const std::string& training_recipe,
    const std::string& actor_obs_contract,
    const std::string& actor_architecture_contract,
    const std::string& planner_commit_contract,
    const std::string& reach_indices,
    const std::string& reach_sources,
    const std::string& nonprivileged_contract,
    const std::string& reach_permission_contract,
    const std::string& optional_reach_training_enabled,
    const std::string& optional_reach_contract,
    const std::string& reward_contract,
    const std::string& recovery_phase_contract) {
  const bool schema27_common =
      training_recipe == "hitter_pingpong_build2_torso_optional_reach_v2" &&
      actor_obs_contract == "hitter_pure_112_headslots_vxy_reach_v1" &&
      actor_architecture_contract ==
          "frozen_stage0_core_waist_leg_residual_v2";
  const bool schema28_common =
      training_recipe == "hitter_pingpong_build2_frozen_core_optional_reach_v1" &&
      actor_obs_contract ==
          "hitter_pure_127_headslots_vxy_reach_core_shadow_v1" &&
      actor_architecture_contract ==
          "frozen_stage0_core_shadow_feedback_waist_leg_residual_v1";
  const bool schema29_common =
      training_recipe == "hitter_pingpong_build2_trainable_home_optional_reach_v1" &&
      actor_obs_contract == "hitter_pure_112_headslots_vxy_reach_v1" &&
      actor_architecture_contract ==
          "trainable_full31_core_optional_reach_curriculum_v1";
  const bool schema31_common =
      training_recipe == kSchema31TrainingRecipe &&
      actor_obs_contract == "hitter_pure_112_headslots_vxy_reach_v1" &&
      actor_architecture_contract == kSchema31ActorArchitectureContract;
  const bool step_v3 = training_recipe == kSchema34TrainingRecipe && actor_obs_contract == kSchema34StepActorObsContract;
  const bool schema32_common =
      hitter_schema32_slew_family_recipe(training_recipe) &&
      actor_obs_contract ==
          (training_recipe == kSchema34TrainingRecipe
               ? (step_v3 ? kSchema34StepActorObsContract : kSchema34ActorObsContract)
               : "hitter_pure_112_headslots_vxy_reach_v1") &&
      actor_architecture_contract == kSchema31ActorArchitectureContract;
  const bool schema31_candidate_family = schema31_common || schema32_common;
  const bool common_ok =
      (schema27_common || schema28_common || schema29_common ||
       schema31_candidate_family) &&
      reach_indices == "110,111" &&
      nonprivileged_contract ==
          (training_recipe == kSchema34TrainingRecipe
               ? (step_v3 ? kSchema34StepNonprivilegedContract : kSchema34NonprivilegedContract)
           :
          (schema28_common
               ? "planner_tuple_plus_policy_shadow_no_plant_inference_v2"
           : (schema29_common || schema31_candidate_family)
               ? "planner_tuple_no_plant_inference_v3"
               : "planner_tuple_no_contact_force_or_plant_inference_v1"));
  if (!common_ok) {
    throw std::runtime_error(
        "Schema27/28/29/31/32 must use its exact actor-lineage Planner-owned, "
        "non-privileged optional-reach input-source contract");
  }
  const bool disabled = schema27_common &&
      planner_commit_contract ==
          "fresh_schema2_flight_fixed_session_home_v1" &&
      reach_sources ==
          "planner_certified_tuple,planner_certified_tuple" &&
      reach_permission_contract ==
          "certified_planner_tuple_labels_no_plant_inference_v1" &&
      optional_reach_training_enabled == "false" &&
      optional_reach_contract == "disabled_zero_v1" &&
      reward_contract ==
          "immutable_home_torso_settle_optional_reach_stage0_v1" &&
      recovery_phase_contract == "signed_tts_visible_torso_handoff_v1";
  const bool schema3_transport_common =
      planner_commit_contract ==
          "fresh_schema3_flight_shared_session_home_support_intent_v2" &&
      reach_sources ==
          (training_recipe == kSchema34TrainingRecipe
               ? (step_v3 ? kSchema34StepActorReachSources : kSchema34ActorReachSources)
               : "planner_target_tuple,planner_target_tuple");
  const bool schema3_common =
      schema3_transport_common && !schema31_candidate_family &&
      reach_permission_contract ==
          "staged_planner_support_intent_no_plant_inference_v4";
  const bool schema31_schema3_common =
      schema3_transport_common && schema31_candidate_family &&
      reach_permission_contract ==
          (training_recipe == kSchema34TrainingRecipe
               ? (step_v3 ? kSchema34StepReachPermissionContract : kSchema34ReachPermissionContract)
               : kSchema31OptionalReachContract);
  const bool enabled_common =
      schema3_common &&
      optional_reach_training_enabled == "true" &&
      optional_reach_contract ==
          "staged_support_intent_training_v3";
  const bool enabled =
      enabled_common && !schema29_common && !schema31_candidate_family &&
      reward_contract ==
          "immutable_home_world_torso_unified_support_residual_v4" &&
      recovery_phase_contract ==
          "full_swing_torso_then_visible_settle_v4";
  const bool candidate =
      schema29_common && schema3_common &&
      optional_reach_training_enabled == "false" &&
      optional_reach_contract == "staged_support_intent_candidate_v4" &&
      reward_contract == "immutable_home_world_torso_trainable_support_v5" &&
      recovery_phase_contract ==
          "full_swing_torso_visible_settle_trainable_v5";
  // Schema31 trains only the planner-visible L1/L2 support skill.  The ONNX
  // metadata therefore truthfully says training_enabled=true, while the
  // ordinary runtime remains a schema-3/L0-only candidate until an explicit
  // x86 simulator lane opts in.  Reward/recovery semantics are validated by
  // the dedicated Schema31 metadata validator rather than overloaded here.
  const bool training_candidate =
      schema31_candidate_family && schema31_schema3_common &&
      optional_reach_training_enabled == "true" &&
      optional_reach_contract == kSchema31OptionalReachContract &&
      (training_recipe == kSchema34TrainingRecipe
           ? (step_v3 ? (reward_contract == kSchema34StepRewardContract && recovery_phase_contract == kSchema34StepRecoveryContract) : ((reward_contract == kSchema34RewardContract &&
               recovery_phase_contract == kSchema34RecoveryPhaseContract) ||
              (reward_contract == kSchema34BalanceRewardContract &&
               recovery_phase_contract == kSchema34BalanceRecoveryContract)))
       : training_recipe == kSchema33TrainingRecipe
           ? (reward_contract == kSchema33RewardContract &&
              recovery_phase_contract == kSchema33RecoveryPhaseContract)
           : (reward_contract == kSchema31RewardContract &&
              recovery_phase_contract == kSchema31RecoveryPhaseContract));
  const int matching_modes = static_cast<int>(disabled) +
      static_cast<int>(candidate) + static_cast<int>(training_candidate) +
      static_cast<int>(enabled);
  if (matching_modes != 1) {
    throw std::runtime_error(
        "Schema27/28/29/31/32 optional reach must declare exactly one coherent "
        "disabled-schema2, level0-only candidate-schema3, simulator-training "
        "candidate-schema3, or enabled-schema3 metadata tuple");
  }
  return enabled
      ? HitterOptionalReachRuntimeMode::kEnabledSchema3
      : (candidate || training_candidate)
          ? HitterOptionalReachRuntimeMode::kCandidateSchema3Level0Only
          : HitterOptionalReachRuntimeMode::kDisabledSchema2Zero;
}

inline void validate_hitter_pingpong_schema28_shadow_metadata(
    const std::string& training_recipe,
    const std::string& transition_contract,
    const std::string& actor_initialization_contract,
    const std::string& actor_architecture_contract,
    const std::string& shadow_feedback_contract,
    const std::string& shadow_action_indices,
    const std::string& shadow_observation_indices,
    const std::string& onnx_output_contract,
    bool actions_output_shape_valid,
    bool shadow_output_shape_valid) {
  if (
      training_recipe != "hitter_pingpong_build2_frozen_core_optional_reach_v1" ||
      transition_contract !=
          "policy_owned_127d_markov_frozen_core_shadow_residual_no_qdes_override_v1" ||
      actor_initialization_contract !=
          "schema28_from_build1_model21800_functional_vxy_zero_reach_frozen_core_shadow_v1" ||
      actor_architecture_contract !=
          "frozen_stage0_core_shadow_feedback_waist_leg_residual_v1" ||
      shadow_feedback_contract !=
          "core_owned15_actions_0_1_2_19_30_append112_126_v1" ||
      shadow_action_indices !=
          "0,1,2,19,20,21,22,23,24,25,26,27,28,29,30" ||
      shadow_observation_indices !=
          "112,113,114,115,116,117,118,119,120,121,122,123,124,125,126" ||
      onnx_output_contract !=
          "actions31_core_action_shadow_owned15_v1" ||
      !actions_output_shape_valid || !shadow_output_shape_valid) {
    throw std::runtime_error(
        "Schema28 requires exact 127-D frozen-core shadow feedback metadata "
        "and named ONNX outputs actions[1,31]/core_action_shadow_owned[1,15]");
  }
}

// Schema29 deliberately returns to a single trainable 31-D actor output. Keep
// this separate from Schema28's 127-D dual-output validator so a future 112-D
// recipe cannot inherit the core-shadow ABI merely by joining the optional-
// reach family.
inline void validate_hitter_pingpong_schema29_single_output_metadata(
    const std::string& training_recipe,
    const std::string& transition_contract,
    const std::string& actor_initialization_contract,
    const std::string& actor_obs_contract,
    const std::string& actor_architecture_contract,
    const std::string& onnx_output_contract,
    bool actions_output_shape_valid,
    bool shadow_output_present) {
  if (
      training_recipe !=
          "hitter_pingpong_build2_trainable_home_optional_reach_v1" ||
      transition_contract !=
          "policy_owned_112d_markov_trainable_no_qdes_override_v1" ||
      actor_initialization_contract !=
          "schema29_from_build1_model21800_functional_vxy_zero_append_reach_trainable_v1" ||
      actor_obs_contract != "hitter_pure_112_headslots_vxy_reach_v1" ||
      actor_architecture_contract !=
          "trainable_full31_core_optional_reach_curriculum_v1" ||
      onnx_output_contract != "actions31_v1" ||
      !actions_output_shape_valid || shadow_output_present) {
    throw std::runtime_error(
        "Schema29 requires the exact trainable 112-D single-output metadata "
        "and named ONNX output actions[1,31] without Schema28 core shadow");
  }
}

inline void validate_hitter_pingpong_schema31_single_output_metadata(
    const std::string& training_recipe,
    const std::string& transition_contract,
    const std::string& actor_initialization_contract,
    const std::string& actor_obs_contract,
    const std::string& actor_architecture_contract,
    const std::string& onnx_output_contract,
    bool actions_output_shape_valid,
    bool shadow_output_present) {
  if (training_recipe != kSchema31TrainingRecipe ||
      transition_contract !=
          "policy_owned_112d_markov_trainable_no_qdes_override_v1" ||
      actor_initialization_contract != kSchema31InitializationContract ||
      actor_obs_contract != "hitter_pure_112_headslots_vxy_reach_v1" ||
      actor_architecture_contract != kSchema31ActorArchitectureContract ||
      onnx_output_contract != "actions31_v1" ||
      !actions_output_shape_valid || shadow_output_present) {
    throw std::runtime_error(
        "Schema31 requires the exact trainable 112-D single-output metadata "
        "and named ONNX output actions[1,31] without Schema28 core shadow");
  }
}

inline void validate_hitter_pingpong_schema32_single_output_metadata(
    const std::string& training_recipe,
    const std::string& transition_contract,
    const std::string& actor_initialization_contract,
    const std::string& actor_obs_contract,
    const std::string& actor_architecture_contract,
    const std::string& onnx_output_contract,
    bool actions_output_shape_valid,
    bool shadow_output_present) {
  const bool schema33 = training_recipe == kSchema33TrainingRecipe;
  const bool schema34 = training_recipe == kSchema34TrainingRecipe;
  const bool step_v3 = schema34 && actor_initialization_contract == kSchema34StepInitializationContract;
  if (!hitter_schema32_slew_family_recipe(training_recipe) ||
      transition_contract !=
          (schema34 ? kSchema34TransitionContract : kSchema32TransitionContract) ||
      (actor_initialization_contract !=
          (schema34 ? kSchema34InitializationContract
           : schema33 ? kSchema33InitializationContract
                    : kSchema32InitializationContract) &&
       !(schema34 && actor_initialization_contract == kSchema34BalanceInitializationContract) && !step_v3) ||
      actor_obs_contract !=
          (step_v3 ? kSchema34StepActorObsContract : schema34 ? kSchema34ActorObsContract
                    : "hitter_pure_112_headslots_vxy_reach_v1") ||
      actor_architecture_contract != kSchema31ActorArchitectureContract ||
      onnx_output_contract != "actions31_v1" ||
      !actions_output_shape_valid || shadow_output_present) {
    throw std::runtime_error(
        "Schema32 requires the exact trainable 112-D single-output, v12 slew-safe "
        "executed-feedback metadata and named ONNX output actions[1,31]");
  }
}

inline void validate_hitter_pingpong_schema31_candidate_qualification_metadata(
    const std::string& training_recipe,
    const std::string& optional_reach_training_enabled,
    const std::string& optional_reach_contract,
    const std::string& optional_reach_trajectory_qualification,
    const std::string& optional_reach_deployment_qualification) {
  if (!hitter_schema31_candidate_family_recipe(training_recipe) ||
      optional_reach_training_enabled != "true" ||
      optional_reach_contract != kSchema31OptionalReachContract ||
      optional_reach_trajectory_qualification !=
          kSchema31OptionalReachTrajectoryQualification ||
      optional_reach_deployment_qualification !=
          kSchema31OptionalReachDeploymentQualification) {
    throw std::runtime_error(
        "Schema31 L1/L2 support skill is a simulator-training candidate only; "
        "ordinary production must remain schema-3 level-0-only");
  }
}

inline void validate_hitter_pingpong_schema31_question_bank_metadata(
    const std::string& training_recipe,
    const std::string& target_tuple_prefilter_contract,
    const std::string& target_tuple_correlated_qualification,
    const std::string& question_bank_artifact_contract,
    const std::string& question_bank_sha256,
    const std::string& question_bank_receipt_sha256,
    const std::string& tuple_contract,
    const std::string& question_bank_qualification_scope,
    const std::string& deployment_qualification,
    const std::string& entry_tts_execution_qualification,
    const std::string& rapid_transition_qualification) {
  if (!hitter_schema31_candidate_family_recipe(training_recipe) ||
      target_tuple_prefilter_contract !=
          "fixed_home_support_training_candidate_bank_v5_axis_envelope_"
          "prefilter_not_trajectory_certification_v1" ||
      target_tuple_correlated_qualification !=
          "NOT_PROVEN_axis_prefilter_only" ||
      question_bank_artifact_contract !=
          kSchema31QuestionBankArtifactContract ||
      question_bank_sha256 != kSchema31QuestionBankSha256 ||
      question_bank_receipt_sha256 !=
          kSchema31QuestionBankReceiptSha256 ||
      tuple_contract !=
          "fixed_home_coherent_question_bank_v1,device_discrete_row_gather_v1,"
          "SIM_TRAINING_CANDIDATE,"
          "SIM_TRAINING_CANDIDATE_NOT_DEPLOYMENT_QUALIFIED" ||
      question_bank_qualification_scope != "SIM_TRAINING_CANDIDATE" ||
      deployment_qualification !=
          kSchema31OptionalReachDeploymentQualification ||
      entry_tts_execution_qualification != "NOT_PROVEN" ||
      rapid_transition_qualification != "NOT_PROVEN") {
    throw std::runtime_error(
        "Schema31 question bank must be the exact receipt-bound v5 L1/L2 "
        "simulator-training candidate; it does not certify trajectory, macro "
        "corner stepping, or deployment use");
  }
}

// The Schema29 question rows and optional-support labels are training-screened
// candidates only. These fields deliberately do not certify a closed-loop
// HOME-to-step-to-strike-to-return trajectory or deployment qualification.
inline void validate_hitter_pingpong_schema29_candidate_qualification_metadata(
    const std::string& training_recipe,
    const std::string& target_tuple_prefilter_contract,
    const std::string& target_tuple_correlated_qualification,
    const std::string& question_bank_artifact_contract,
    const std::string& question_bank_sha256,
    const std::string& question_bank_receipt_sha256,
    const std::string& tuple_contract,
    const std::string& question_bank_qualification_scope,
    const std::string& deployment_qualification,
    const std::string& entry_tts_execution_qualification,
    const std::string& rapid_transition_qualification,
    const std::string& optional_reach_trajectory_qualification,
    const std::string& optional_reach_deployment_qualification) {
  if (
      training_recipe !=
          "hitter_pingpong_build2_trainable_home_optional_reach_v1" ||
      target_tuple_prefilter_contract !=
          "fixed_home_support_candidate_bank_v4_axis_envelope_"
          "prefilter_not_trajectory_certification_v1" ||
      target_tuple_correlated_qualification !=
          "NOT_PROVEN_axis_prefilter_only" ||
      question_bank_artifact_contract !=
          "fixed_home_support_candidate_bank_v4" ||
      question_bank_sha256 != kSchema29QuestionBankSha256 ||
      question_bank_receipt_sha256 !=
          kSchema29QuestionBankReceiptSha256 ||
      tuple_contract !=
          "fixed_home_coherent_question_bank_v1,device_discrete_row_gather_v1,"
          "TRAINING_SCREENED,SIM_TRAINING_ONLY,deployment_NOT_PROVEN" ||
      question_bank_qualification_scope != "SIM_TRAINING_ONLY" ||
      deployment_qualification != "NOT_PROVEN" ||
      entry_tts_execution_qualification != "NOT_PROVEN" ||
      rapid_transition_qualification != "NOT_PROVEN" ||
      optional_reach_trajectory_qualification != "NOT_PROVEN" ||
      optional_reach_deployment_qualification != "NOT_PROVEN") {
    throw std::runtime_error(
        "Schema29 question-bank and optional-reach rows are candidate-only: "
        "trajectory, correlated sampling, entry/rapid execution, and deployment "
        "qualification must remain explicitly NOT_PROVEN");
  }
}

inline void validate_hitter_pingpong_schema29_mocap_stale_metadata(
    const std::string& training_recipe,
    const std::string& stale_contract,
    const std::string& stale_receipt_sha256,
    const std::string& stale_start_probability,
    const std::string& stale_length_weights,
    const std::string& stale_curriculum_contract) {
  if (
      (training_recipe !=
           "hitter_pingpong_build2_trainable_home_optional_reach_v1" &&
       !hitter_schema31_candidate_family_recipe(training_recipe)) ||
      stale_contract != "localization_fresh_outage_empirical_v1" ||
      stale_receipt_sha256 != kSchema29MocapStaleReceiptSha256 ||
      stale_start_probability != "0.015090824406148" ||
      stale_length_weights !=
          "3,1,4,12,9,15,11,12,8,7,16,27,19,15,3" ||
      stale_curriculum_contract != kSchema29MocapStaleCurriculumContract) {
    throw std::runtime_error(
        "Schema29/31/32 requires the checkpoint-bound actor-visible mocap stale/"
        "reacquire distribution and exact empirical receipt");
  }
}

inline void validate_hitter_pingpong_schema29_retention_metadata(
    const std::string& training_recipe,
    const std::string& side_rehearsal_contract,
    const std::string& teacher_contract) {
  if (training_recipe !=
      "hitter_pingpong_build2_trainable_home_optional_reach_v1") {
    throw std::runtime_error(
        "Schema29 retention metadata validator received another recipe");
  }
  if (side_rehearsal_contract != kSchema29SideRehearsalContract ||
      teacher_contract != kSchema29TeacherRetentionContract) {
    throw std::runtime_error(
        "Schema29 requires exact bilateral arm-retention provenance");
  }
}

// V3 is the first memoryless WAIT contract. Validate its complete new metadata tuple in one pure
// helper shared by the ONNX parser and focused unit tests. Exact values are intentional: accepting
// a v2 elapsed clock, a zero/legacy pending velocity, or a different arrival hazard changes the MDP
// while leaving the 110-D tensor shape unchanged.
inline void validate_hitter_pingpong_continuous_v3_metadata(
    const std::string& training_recipe,
    const std::string& recipe_version,
    const std::string& build_contract,
    const std::string& command_contract,
    const std::string& wait_clock_contract,
    double wait_tts_s,
    const std::array<double, 3>& pending_target,
    const std::array<double, 3>& pending_velocity,
    const std::string& arrival_contract,
    const std::string& legacy_next_command_arrival_contract,
    double arrival_hazard_per_tick,
    double policy_dt_s) {
  constexpr std::array<double, 3> kPendingTarget = {0.58, -0.44, 1.075};
  constexpr std::array<double, 3> kPendingVelocity = {1.92, 0.19, 1.03};
  auto exact_vector = [](const std::array<double, 3>& actual,
                         const std::array<double, 3>& expected) {
    for (std::size_t i = 0; i < actual.size(); ++i) {
      if (!std::isfinite(actual[i]) ||
          std::fabs(actual[i] - expected[i]) > 1.0e-12) {
        return false;
      }
    }
    return true;
  };
  if (training_recipe != "hitter_pingpong_continuous_rally_v3" ||
      recipe_version != "3" ||
      build_contract != "continuous_rally_schema19" ||
      command_contract != "strike_followthrough_home_external_commit_v3" ||
      wait_clock_contract != "constant_pending_wait_v1" ||
      !std::isfinite(wait_tts_s) || std::fabs(wait_tts_s - 0.85) > 1.0e-12 ||
      !exact_vector(pending_target, kPendingTarget) ||
      !exact_vector(pending_velocity, kPendingVelocity) ||
      arrival_contract != "memoryless_geometric_v1" ||
      !legacy_next_command_arrival_contract.empty() ||
      !std::isfinite(arrival_hazard_per_tick) ||
      std::fabs(arrival_hazard_per_tick - 0.08) > 1.0e-12 ||
      !std::isfinite(policy_dt_s) || std::fabs(policy_dt_s - 0.02) > 1.0e-12) {
    throw std::runtime_error(
        "continuous-rally-v3 metadata must exactly match its recipe, constant WAIT, "
        "pending target/velocity, 0.08-per-20ms arrival, and command lifecycle");
  }
}

// V4 keeps V3's memoryless WAIT/external-COMMIT lifecycle, but changes three semantics that cannot
// be inferred from tensor dimensions: the two passive-head feedback columns now carry filtered
// mocap v_xy, ACTIVE uses an actor-visible contact line and a signed-TTS motion frame, and racket
// velocity rewards reconstruct world velocity from that same mocap signal. Reject every partial or
// cross-version tuple because each such mismatch is a different MDP behind the same 110-D ABI.
inline void validate_hitter_pingpong_continuous_v4_metadata(
    const std::string& training_recipe,
    const std::string& recipe_version,
    const std::string& build_contract,
    const std::string& command_contract,
    const std::string& planner_commit_contract,
    const std::string& actor_obs_contract,
    const std::string& trajectory_teacher_contract,
    const std::string& motion_phase_contract,
    const std::string& racket_velocity_contract,
    const std::string& wait_clock_contract,
    double wait_tts_s,
    const std::array<double, 3>& pending_target,
    const std::array<double, 3>& pending_velocity,
    const std::string& arrival_contract,
    const std::string& legacy_next_command_arrival_contract,
    double arrival_hazard_per_tick,
    double policy_dt_s) {
  constexpr std::array<double, 3> kPendingTarget = {0.58, -0.44, 1.075};
  constexpr std::array<double, 3> kPendingVelocity = {1.92, 0.19, 1.03};
  auto exact_vector = [](const std::array<double, 3>& actual,
                         const std::array<double, 3>& expected) {
    for (std::size_t i = 0; i < actual.size(); ++i) {
      if (!std::isfinite(actual[i]) ||
          std::fabs(actual[i] - expected[i]) > 1.0e-12) {
        return false;
      }
    }
    return true;
  };
  if (training_recipe != "hitter_pingpong_continuous_rally_v4" ||
      recipe_version != "4" ||
      build_contract != "continuous_rally_schema20" ||
      command_contract != "strike_followthrough_home_external_commit_v4" ||
      planner_commit_contract !=
          "fresh_shot_external_event_policy_independent_v2" ||
      actor_obs_contract != "hitter_pure_110_headslots_vxy_v1" ||
      trajectory_teacher_contract != "contact_linear_actor_visible_v1" ||
      motion_phase_contract != "signed_tts_contact_frame_v1" ||
      racket_velocity_contract != "mocap_vxy_world_racket_velocity_v1" ||
      wait_clock_contract != "constant_pending_wait_v1" ||
      !std::isfinite(wait_tts_s) || std::fabs(wait_tts_s - 0.85) > 1.0e-12 ||
      !exact_vector(pending_target, kPendingTarget) ||
      !exact_vector(pending_velocity, kPendingVelocity) ||
      arrival_contract != "memoryless_geometric_v1" ||
      !legacy_next_command_arrival_contract.empty() ||
      !std::isfinite(arrival_hazard_per_tick) ||
      std::fabs(arrival_hazard_per_tick - 0.08) > 1.0e-12 ||
      !std::isfinite(policy_dt_s) || std::fabs(policy_dt_s - 0.02) > 1.0e-12) {
    throw std::runtime_error(
        "continuous-rally-v4 metadata must exactly match schema20, its visible-vxy "
        "110-D actor, contact-linear/signed-TTS contracts, constant WAIT, and external COMMIT");
  }
}

// V4 makes mocap v_xy actor-visible, so a held/stale pose or invalid differentiated velocity can
// no longer be treated as advisory or replaced with zero. Historical recipes intentionally keep
// their existing outage behavior.
inline void validate_hitter_pingpong_continuous_v4_mocap_observation(
    bool continuous_v4,
    bool mocap_fresh,
    bool velocity_valid,
    const std::array<double, 2>& velocity_xy_w) {
  if (!continuous_v4) return;
  if (!mocap_fresh || !velocity_valid ||
      !std::isfinite(velocity_xy_w[0]) ||
      !std::isfinite(velocity_xy_w[1])) {
    throw std::runtime_error(
        "continuous-rally-v4 requires a fresh authoritative mocap pose and finite filtered v_xy");
  }
}

// Continuous-rally v2 is a policy-native lifecycle. Keep the validation independent of
// PpPolicyConfig so the runner and unit tests share one fail-closed decision without pulling the
// ONNX/runtime implementation into the contract test. qdes_audit_only is deliberately not an
// override here: the main binary restricts it to x86 MuJoCo Gate3, where publishing the finite raw
// command is required to expose rather than hide unsafe policy output.
struct HitterPingPongContinuousV2RuntimeMode {
  bool planner_mode = false;
  bool policy_native = false;
  bool single_swing = false;
  int initial_level = 0;
  bool stream_target = false;
  bool station_only = false;
  bool replay_mode = false;
  bool target_override = false;
  bool policy_output_override = false;
  bool clock_override = false;
  bool qdes_audit_only = false;
  bool target_support_gate = true;
  bool stay_if_reachable = true;
  double command_timeout_s = 0.5;
};

inline void validate_hitter_pingpong_continuous_v2_runtime_mode(
    bool continuous_v2,
    const HitterPingPongContinuousV2RuntimeMode& mode) {
  if (!continuous_v2) return;
  if (!mode.planner_mode || !mode.policy_native || !mode.single_swing ||
      mode.initial_level != 0) {
    throw std::runtime_error(
        "continuous-rally-v2 requires planner_mode + policy_native, an "
        "external-COMMIT idle start, and single-swing follow-through");
  }
  if (mode.stream_target || mode.station_only || mode.replay_mode ||
      mode.target_override || mode.policy_output_override ||
      mode.clock_override) {
    throw std::runtime_error(
        "continuous-rally-v2 rejects stream-target, station-only, replay, "
        "and target/output/clock overrides");
  }
}

inline void validate_hitter_pingpong_continuous_v3_runtime_mode(
    bool continuous_v3,
    const HitterPingPongContinuousV2RuntimeMode& mode) {
  if (!continuous_v3) return;
  if (!mode.planner_mode || !mode.policy_native || !mode.single_swing ||
      mode.initial_level != 0) {
    throw std::runtime_error(
        "continuous-rally-v3 requires planner_mode + policy_native, an "
        "external-COMMIT WAIT start, and single-swing follow-through");
  }
  if (mode.stream_target || mode.station_only || mode.replay_mode ||
      mode.target_override || mode.policy_output_override ||
      mode.clock_override) {
    throw std::runtime_error(
        "continuous-rally-v3 rejects stream-target, station-only, replay, "
        "and target/output/clock overrides");
  }
}

inline void validate_hitter_pingpong_continuous_v4_runtime_mode(
    bool continuous_v4,
    const HitterPingPongContinuousV2RuntimeMode& mode) {
  if (!continuous_v4) return;
  if (!mode.planner_mode || !mode.policy_native || !mode.single_swing ||
      mode.initial_level != 0) {
    throw std::runtime_error(
        "continuous-rally-v4 requires planner_mode + policy_native, an "
        "external-COMMIT WAIT start, and single-swing follow-through");
  }
  if (mode.stream_target || mode.station_only || mode.replay_mode ||
      mode.target_override || mode.policy_output_override ||
      mode.clock_override) {
    throw std::runtime_error(
        "continuous-rally-v4 rejects stream-target, station-only, replay, "
        "and target/output/clock overrides");
  }
}

inline void validate_hitter_pingpong_build2_rapid_preempt_runtime_mode(
    bool build2_rapid_preempt,
    const HitterPingPongContinuousV2RuntimeMode& mode) {
  if (!build2_rapid_preempt) return;
  if (!mode.planner_mode || !mode.policy_native || !mode.single_swing ||
      mode.initial_level != 0 || !mode.target_support_gate ||
      !mode.stay_if_reachable ||
      !std::isfinite(mode.command_timeout_s) ||
      mode.command_timeout_s <= 0.0) {
    throw std::runtime_error(
        "build2-rapid-preempt requires planner_mode + policy_native, an "
        "external-COMMIT idle start, single-swing lifecycle, a finite "
        "freshness timeout, the metadata target-support gate, and exact "
        "stay-if-reachable station inversion");
  }
  if (mode.stream_target || mode.station_only || mode.replay_mode ||
      mode.target_override || mode.policy_output_override ||
      mode.clock_override) {
    throw std::runtime_error(
        "build2-rapid-preempt rejects stream-target, station-only, replay, "
        "and target/output/clock overrides");
  }
}

inline void validate_hitter_pingpong_build2_fixed_home_runtime_mode(
    bool build2_fixed_home,
    const HitterPingPongContinuousV2RuntimeMode& mode) {
  if (!build2_fixed_home) return;
  if (!mode.planner_mode || !mode.policy_native || !mode.single_swing ||
      mode.initial_level != 0 ||
      !std::isfinite(mode.command_timeout_s) ||
      mode.command_timeout_s <= 0.0) {
    throw std::runtime_error(
        "build2-fixed-home requires planner_mode + policy_native, an "
        "external-COMMIT idle start, single-swing lifecycle, and a finite "
        "freshness timeout");
  }
  if (mode.stream_target || mode.station_only || mode.replay_mode ||
      mode.target_override || mode.policy_output_override ||
      mode.clock_override) {
    throw std::runtime_error(
        "build2-fixed-home rejects stream-target, station-only, replay, "
        "and target/output/clock overrides");
  }
  // target_support_gate and stay_if_reachable are historical CLI fields. Schema23 never uses
  // either as an admission condition: the requested station/support box is telemetry only and
  // the actor command is always immutable session HOME.
}

// Parse the capability contract and enforce the bidirectional component-velocity recipe pairing
// before any recipe-specific metadata is consumed. RallyV10--V14, the standalone Hitter repair,
// and legacy RallyV17 share the v2 wire; their recipe-specific fields are validated by PpOnnxPolicy.
inline HitterPureRuntimeContract validate_hitter_pure_runtime_contract(
    const std::string& runtime_contract, const std::string& training_recipe) {
  HitterPureRuntimeContract contract = HitterPureRuntimeContract::kLegacy;
  if (runtime_contract.empty()) {
    contract = HitterPureRuntimeContract::kLegacy;
  } else if (runtime_contract == "rally_final_v1") {
    contract = HitterPureRuntimeContract::kRallyFinalV1;
  } else if (runtime_contract == "rally_final_v2") {
    contract = HitterPureRuntimeContract::kRallyFinalV2;
  } else if (runtime_contract == "rally_v15") {
    contract = HitterPureRuntimeContract::kRallyV15;
  } else if (runtime_contract == "rally_v17_fixed_station_ball_clock_v1") {
    contract = HitterPureRuntimeContract::kRallyV17FixedStationBallClockV1;
  } else {
    throw std::runtime_error(
        "ONNX has unsupported hitter_pure_runtime_contract='" + runtime_contract + "'");
  }

  const bool runtime_v2 = contract == HitterPureRuntimeContract::kRallyFinalV2;
  const bool legacy_component_recipe =
      training_recipe == "rally_v10" || training_recipe == "rally_v11" ||
      training_recipe == "rally_v12" || training_recipe == "rally_v13" ||
      training_recipe == "rally_v14";
  const bool hitter_pingpong_recipe =
      training_recipe == "small_station_compact_execution_candidate_v1" ||
      training_recipe == "hitter_small_station_independent_recovery_v2" ||
      training_recipe == "hitter_small_station_support_heading_recovery_v3" ||
      training_recipe == "hitter_small_station_matched_plant_recovery_v4" ||
      training_recipe == "hitter_small_station_hold_resume_recovery_v5" ||
      training_recipe == "hitter_pingpong_recovery_tts_v2" ||
      training_recipe == "hitter_pingpong_recovery_tts_v3" ||
      training_recipe == "hitter_pingpong_continuous_rally_v1" ||
      training_recipe == "hitter_pingpong_continuous_rally_v2" ||
      training_recipe == "hitter_pingpong_continuous_rally_v3" ||
      training_recipe == "hitter_pingpong_continuous_rally_v4" ||
      training_recipe == "hitter_pingpong_build2_minimal_rootfix_v1" ||
      training_recipe == "hitter_pingpong_build2_rapid_preempt_v1" ||
      training_recipe == "hitter_pingpong_build2_fixed_home_recovery_v1" ||
      training_recipe == "hitter_pingpong_build2_fixed_home_feasible_v1" ||
      training_recipe ==
          "hitter_pingpong_build2_fixed_home_phase_recovery_v1" ||
      training_recipe == "hitter_pingpong_build2_home_support_frame_v1" ||
      training_recipe ==
          "hitter_pingpong_build2_torso_optional_reach_v2" ||
      training_recipe ==
          "hitter_pingpong_build2_frozen_core_optional_reach_v1" ||
      training_recipe ==
          "hitter_pingpong_build2_trainable_home_optional_reach_v1" ||
      hitter_schema31_candidate_family_recipe(training_recipe);
  const bool v17_recipe = training_recipe == "rally_v17";
  const bool runtime_v17_fixed =
      contract == HitterPureRuntimeContract::kRallyV17FixedStationBallClockV1;
  if ((runtime_v2 &&
       !(legacy_component_recipe || hitter_pingpong_recipe || v17_recipe)) ||
      (legacy_component_recipe && !runtime_v2) ||
      (hitter_pingpong_recipe && !runtime_v2) ||
      (v17_recipe && !(runtime_v2 || runtime_v17_fixed)) ||
      (runtime_v17_fixed && !v17_recipe)) {
    throw std::runtime_error(
        "ONNX runtime/recipe pairing is invalid: RallyV10--V14 and "
        "HitterPingPong recovery-v2/v3/continuous-v1/v2/v3/v4 and "
        "build2-minimal-rootfix-v1/build2-rapid-preempt-v1/"
        "build2-fixed-home-recovery-v1/build2-fixed-home-feasible-v1/"
        "build2-fixed-home-phase-recovery-v1/build2-home-support-frame-v1/"
        "build2-torso-optional-reach-v2/build2-frozen-core-optional-reach-v1/"
        "build2-trainable-home-optional-reach-v1/"
        "build2-stable-home-step-candidate-v1 require "
        "rally_final_v2; legacy RallyV17 "
        "accepts rally_final_v2; fixed-station RallyV17 requires "
        "rally_v17_fixed_station_ball_clock_v1");
  }
  const bool runtime_v15 = contract == HitterPureRuntimeContract::kRallyV15;
  if (runtime_v15 != (training_recipe == "rally_v15")) {
    throw std::runtime_error(
        "ONNX runtime_contract=rally_v15 and training_recipe=rally_v15 must be declared "
        "together; the V15 executed-q_des/finite-gait contract cannot fall back to V14 wiring");
  }
  return contract;
}

}  // namespace a3_pingpong
