#!/usr/bin/env bash
# Sole formal standalone-Hitter Gate3 entry. The x86 runner consumes the C++ Planner command.
set -u
SCRIPT_DIR="/home/user/Desktop/workspace/nohope/agi/a3_deploy_example/scripts"
GEAR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
HOPE_ROOT="$(cd -- "$GEAR/../.." && pwd)"
cd "$GEAR" || exit 2
export PP_GEAR="$GEAR"
# The shared physical profile clears PP_EXTRA_ARGS while installing its
# defaults. Preserve caller-supplied Runner composition flags so this harness
# can exercise the exact candidate binary rather than silently reverting to
# model_3440 alone.
HITTER_RUNNER_EXTRA_ARGS="${PP_EXTRA_ARGS:-}"

export PP_GATE3_PROFILE=hitter_pingpong_shared_plane_v1
export PP_GATE3_SCENARIO=hitter_pingpong_fixed_home_rapid_1150ms_v1
export PP_GATE3_PHASE=fixed_home
HITTER_GATE3_MODE="${PP_HITTER_GATE3_MODE:-certification}"
case "$HITTER_GATE3_MODE" in
  certification) ;;
  torture)
    export PP_GATE3_SCENARIO=hitter_pingpong_low_corner_torture_v1
    export PP_GATE3_PHASE=torture
    ;;
  *)
    echo "[hitter-pingpong] PP_HITTER_GATE3_MODE must be certification or torture"
    exit 2
    ;;
esac
# Gate3 measures policy capability.  The production L2/L3 commit supervisor is
# intentionally disabled here so it cannot turn an OOD policy failure into a
# cosmetically clean result.  Refuse a contradictory caller override.
if [ -n "${PP_ENVELOPE_MODE:-}" ] && [ "$PP_ENVELOPE_MODE" != "capability" ]; then
  echo "[hitter-pingpong] Gate3 requires PP_ENVELOPE_MODE=capability; got $PP_ENVELOPE_MODE"
  exit 2
fi
export PP_ENVELOPE_MODE=capability
export PP_SERVES="${PP_SERVES:-26}"
export PP_XHIT=0.08
export PP_XHIT_BH_DELTA=0.0
export PP_FIXED_PLANE_X=0.58
export PP_STATION_X=-0.50
export PP_XLOCK_THRESH=0.05
export PP_READY_X_MAX=0.10
export PP_READY_Y_MAX=0.10
export PP_READY_SPEED_MAX=0.20
export PP_SPLIT_Y=-0.25 PP_SPLIT_HYST=0.04
export PP_SWING_SIDE_REFERENCE_MODE=session_home_v1
export PP_LAND_X=2.055 PP_LAND_Y_FH=-0.7625 PP_LAND_Y_BH=-0.7625
export PP_DTF_FH=0.50 PP_DTF_BH=0.50
# shellcheck disable=SC1091
source "$SCRIPT_DIR/pp_gate3_physical_common.sh"
if [ "$HITTER_GATE3_MODE" = "torture" ]; then
  gate3_apply_hitter_low_corner_torture_v1_contract || exit $?
  # Fall-stress runs retain all lifecycle and plant checks, but physical
  # contact is an 80%-coverage diagnostic rather than the reason to suppress a
  # hard low/corner question. Every ball retains its complete physical flight.
  export PP_MIN_GLOBAL_CONTACTS=$(( (4 * PP_SERVES + 4) / 5 ))
  export PP_MIN_PHYSICAL_SAMPLES_PER_SIDE=$(( PP_SERVES / 2 ))
  export PP_MIN_CONTACTS_PER_SIDE=$(( (2 * PP_SERVES + 4) / 5 ))
  export PP_MIN_RAPID_ENGAGED=$(( PP_SERVES - PP_CLEAN_FIXED_HOME_SERVES ))
  export PP_MIN_RAPID_COMPLETED="$PP_MIN_RAPID_ENGAGED"
  export PP_OPTIONAL_REACH_EXPECTED_FLIGHTS="$PP_SERVES"
else
  gate3_apply_hitter_fixed_home_rapid_1150ms_v1_contract || exit $?
  export PP_MIN_GLOBAL_CONTACTS=22
  # Incoming cadence and outgoing-flight lifetime are independent.
  export PP_MIN_PHYSICAL_SAMPLES_PER_SIDE=13
  export PP_MIN_CONTACTS_PER_SIDE=11
  export PP_MIN_RAPID_ENGAGED=20
  export PP_MIN_RAPID_COMPLETED=20
  if [ "$PP_SERVES" -ne 26 ]; then
    export PP_MIN_GLOBAL_CONTACTS=$(( (5 * PP_SERVES + 5) / 6 ))
    export PP_MIN_PHYSICAL_SAMPLES_PER_SIDE=$(( PP_SERVES / 2 - 1 ))
    export PP_MIN_CONTACTS_PER_SIDE=$(( (5 * (PP_SERVES / 2 - 1) + 5) / 6 ))
    export PP_MIN_RAPID_ENGAGED=$(( PP_SERVES - 6 ))
    export PP_MIN_RAPID_COMPLETED="$PP_MIN_RAPID_ENGAGED"
  fi
fi
export PP_MIN_PHYSICAL_CONTACT_RATE=0.8333333333333334
export PP_MIN_LEGAL_LANDING_RATE=0.8333333333333334
export PP_ALLOWED_LANDING_CENSOR_LANES=""
export PP_MIN_GLOBAL_LANDING_OBSERVATIONS="$PP_MIN_GLOBAL_CONTACTS"
export PP_MIN_GLOBAL_LANDINGS="$PP_MIN_GLOBAL_CONTACTS"
export PP_MIN_LANDING_OBSERVATIONS_PER_SIDE="$PP_MIN_CONTACTS_PER_SIDE"
export PP_MIN_LANDINGS_PER_SIDE="$PP_MIN_CONTACTS_PER_SIDE"
export PP_MIN_POST_CONTACT_LANDING_OBSERVATION_S=0.75
export PP_MIN_STATION_TRANSITIONS=0
export PP_MIN_POSITIVE_MAIN_TRANSITIONS=0
export PP_FIXED_HOME_COMMAND_TOLERANCE_M=0.0015
export PP_FIXED_HOME_DRIFT_NOT_OBSERVED_M=0.01
export PP_RAPID_MAX_ENGAGE_GAP_S=1.40
export PP_RAPID_CONTACT_TO_NEXT_COMMIT_MIN_S=0.40
export PP_RAPID_CONTACT_TO_NEXT_COMMIT_MAX_S=0.55
export PP_NATURAL_RECOVERY_PROTECTED_FOLLOWTHROUGH_S=0.12
export PP_NATURAL_WAIT_ACTUAL_Q_RMS_MAX_RAD=0.20
# Qualification is one no-reset phase (26 shots by default): every serve must engage
# and complete.  Keep this aligned with pp_rally_conductor.py instead of
# inheriting the obsolete 0.8 proxy default from the older V14 wrapper.
export PP_MIN_PROXY_RATE=1.0
export PP_MIN_ENGAGED_SERVES="$PP_SERVES"
export PP_MIN_COMPLETED_SERVES="$PP_SERVES"
export PP_ALLOW_RESCUE=0
export PP_MAX_RESCUES=0
export PP_REQUIRE_READY=1
# The hardware Planner reveals and commits the final target in one event. It reports station
# error/speed at release but does not emit the obsolete pre-release READY token.
# This scenario only requires the common late-reveal commit telemetry.  Whether
# a policy can admit the next flight during recovery is measured by the rapid
# lane itself, not selected here through a recipe-specific readiness label.
export PP_READY_CONTRACT=late_reveal_commit_v1
export PP_RECOVERY_TARGET_CONTRACT=session_anchor
export PP_RECOVERY_RADIUS_M=0.05
export PP_RECOVERY_SPEED_MAX_MPS=0.12
export PP_RECOVERY_DWELL_S=0.30
export PP_REQUIRE_PLANT_TRACE=1
export PP_REQUIRE_IDLE_SMOOTHNESS=1
export PP_REQUIRE_HOME_SUPPORT_FRAME=1
export PP_REQUIRE_TORSO_HOLD=1
export PP_REQUIRE_OPTIONAL_REACH="${PP_REQUIRE_OPTIONAL_REACH:-auto}"
export PP_REQUIRE_COLLISION_LEDGER=1
export PP_TORSO_HOLD_MAX_PITCH_MEDIAN_DEG=3.0
export PP_TORSO_HOLD_MAX_PITCH_P95_DEG=5.0
export PP_TORSO_HOLD_MAX_PITCH_PEAK_DEG=8.0
export PP_HOME_MAX_SUPPORT_MIDPOINT_END_M=0.03
export PP_HOME_MAX_FOOT_ANCHOR_END_M=0.03
export PP_HOME_MIN_SIGNED_WIDTH_M=0.26
export PP_HOME_MAX_SIGNED_WIDTH_M=0.31
export PP_RECOVERY_YAW_MAX_DEG="${PP_RECOVERY_YAW_MAX_DEG:-5.0}"
export PP_MAX_END_YAW_DRIFT_DEG="${PP_MAX_END_YAW_DRIFT_DEG:-5.0}"
export PP_MAX_PEAK_YAW_DRIFT_DEG="${PP_MAX_PEAK_YAW_DRIFT_DEG:-12.0}"
export PP_MOTION_IDLE_S="${PP_MOTION_IDLE_S:-20.0}"
export PP_MIN_MOTION_IDLE_S="${PP_MIN_MOTION_IDLE_S:-15.0}"
# Keep the final serve in MOTION long enough to observe the complete native
# post-contact recovery tail selected later by the recipe-aware report.
export PP_REQUIRED_POST_COMPLETION_TAIL_S=3.0
# Measured burst/dropout robustness is staged on a separate axis and must not
# be silently mixed into the fixed-HOME qualification.  Nominal Gate3 remains
# outage-free, while an explicitly selected serve enables the already isolated
# calibrated-base-relay stress lane.
export PP_BASE_OUTAGE_AT="${PP_BASE_OUTAGE_AT:-0}"
export PP_BASE_OUTAGE_TICKS="${PP_BASE_OUTAGE_TICKS:-15}"
# Metric windows are selected from the validated model recipe, independently
# from the immutable fixed-HOME launch scenario above.
export PP_RALLY_REPORT_MODE=auto
export PP_PLANNER_EVIDENCE_JSON=/tmp/pp_planner_cpp_report.json
export PP_PHYSICAL_EVIDENCE_JSON=/tmp/pp_physical_ball_report.json
export PP_GATE3_VERDICT=certification
# Gate3 is an x86 policy-behavior audit.  It adds no supervisor clamp beyond the
# action contract declared by the artifact: V11 remains affine+safe-clamp and
# Schema32/V12 keeps its deterministic slew layer active. This Gate3 capability
# probe also commits every fresh/finite/identity-valid positive-TTS flight: the
# Runner may not hide a fall by dropping a rapid command at a temporal support
# boundary. Audit counters never alter actor q_des.
export PP_EXTRA_ARGS="${HITTER_RUNNER_EXTRA_ARGS:+$HITTER_RUNNER_EXTRA_ARGS }--gate3-qdes-audit-only --gate3-force-every-flight"

# Bind the Planner wire to the artifact before either process starts. The
# shared model21800 profile intentionally remains Schema 2; only an exact
# Schema28/29/31/32/33/34 optional-reach ONNX receives the explicit Schema-3 overlay.
HITTER_DIST="${PP_DIST:-$GEAR/dist/a3_deploy_x86_64}"
HITTER_POLICY_ONNX="${PP_POLICY_ONNX:-$HITTER_DIST/policy/exported/policy.onnx}"
PLANNER_CONFIG_ROOT="${PP_HOPE_WS:-$(cd -- "$GEAR/../.." && pwd)/hope_ws}/src/hope_planner_cpp/config"
SCHEMA3_OPTIONAL_REACH_PLANNER_PROFILE="$PLANNER_CONFIG_ROOT/schema28_optional_reach.yaml"
SCHEMA29_CANDIDATE_PLANNER_PROFILE="$PLANNER_CONFIG_ROOT/schema29_candidate_level0.yaml"
if [ ! -r "$HITTER_POLICY_ONNX" ]; then
  echo "[hitter-pingpong] policy ONNX missing: $HITTER_POLICY_ONNX"
  exit 2
fi
HITTER_PLANNER_WIRE="$(PYTHONPATH="$SCRIPT_DIR${PYTHONPATH:+:$PYTHONPATH}" python3 - "$HITTER_POLICY_ONNX" <<'PY'
import sys
import onnx

model = onnx.load(sys.argv[1], load_external_data=False)
metadata = {entry.key: entry.value for entry in model.metadata_props}
recipe = metadata.get("hitter_pure_training_recipe", "")
enabled = metadata.get("hitter_pingpong_optional_reach_training_enabled", "false")
# This is routing only; Runner validates the complete executable contract.
from pp_policy_artifact import small_station_mode
shape = lambda v: [d.dim_value for d in v.type.tensor_type.shape.dim]
inputs = {v.name: shape(v) for v in model.graph.input}
outputs = {v.name: shape(v) for v in model.graph.output}
if small_station_mode(metadata, inputs, outputs):
    print("schema2")
elif recipe in (
    "hitter_pingpong_build2_stable_home_step_candidate_v1",
    "hitter_pingpong_build2_recoverability_envelope_v1",
    "hitter_pingpong_build2_active_replant_recovery_v1",
    "hitter_pingpong_build2_visible_foot_recovery_v1",
):
    schema34 = recipe == "hitter_pingpong_build2_visible_foot_recovery_v1"
    schema34_v3 = schema34 and metadata.get("hitter_pure_training_recipe_version") == "3"
    schema34_v2 = schema34 and metadata.get("hitter_pure_training_recipe_version") == "2"
    schema33 = recipe == "hitter_pingpong_build2_active_replant_recovery_v1"
    # Schema33/34 retain the V12 action/planner ABI. Revision 3 explicitly versions
    # the four passive head slots and task phase; historical observation tuples remain strict.
    schema32 = (
        recipe == "hitter_pingpong_build2_recoverability_envelope_v1"
        or schema33
        or schema34
    )
    inputs = {
        input_value.name: [
            dim.dim_value for dim in input_value.type.tensor_type.shape.dim
        ]
        for input_value in model.graph.input
    }
    outputs = {
        output.name: [dim.dim_value for dim in output.type.tensor_type.shape.dim]
        for output in model.graph.output
    }
    if (
        enabled == "true"
        and metadata.get("actor_obs_contract")
        == (
            ("hitter_pure_112_headslots_step_task_v3" if schema34_v3 else "hitter_pure_112_headslots_vxy_signed_support_v2")
            if schema34
            else "hitter_pure_112_headslots_vxy_reach_v1"
        )
        and metadata.get("hitter_pure_runtime_contract") == "rally_final_v2"
        and metadata.get("hitter_pure_training_recipe_version") == ("3" if schema34_v3 else "2" if schema34_v2 else "1")
        and metadata.get("hitter_pure_ready_hold_steps_range") == "150,250"
        and metadata.get("hitter_pingpong_build_contract")
        == (
            "build2_visible_foot_recovery_schema34"
            if schema34
            else "build2_active_replant_recovery_schema33"
            if schema33
            else "build2_recoverability_envelope_schema32"
            if schema32
            else "build2_stable_home_step_candidate_schema31"
        )
        and metadata.get("hitter_pingpong_planner_commit_contract")
        == "fresh_schema3_flight_shared_session_home_support_intent_v2"
        and metadata.get("hitter_pingpong_transition_contract")
        == (
            "policy_owned_112d_signed_support_slew_safe_executed_feedback_v3"
            if schema34
            else "policy_owned_112d_markov_slew_safe_executed_feedback_v2"
            if schema32
            else "policy_owned_112d_markov_trainable_no_qdes_override_v1"
        )
        and metadata.get("hitter_pingpong_training_initialization_contract")
        == (
            ("schema34_from_schema34v2_model4710_zero_task_columns_trainable_v3" if schema34_v3 else "schema34_from_schema32_model5580_zero_support_columns_trainable_v2" if schema34_v2 else "schema34_from_schema33_model2130_zero_support_columns_trainable_v1")
            if schema34
            else "schema33_from_schema32_model5580_direct_112d_actor_trainable_v1"
            if schema33
            else "schema32_from_build1_model21800_functional_vxy_zero_append_reach_trainable_v1"
            if schema32
            else "schema31_from_build1_model21800_functional_vxy_zero_append_reach_trainable_v1"
        )
        and metadata.get("hitter_pingpong_actor_architecture_contract")
        == "trainable_full31_stable_home_step_candidate_v1"
        and metadata.get("hitter_pingpong_side_rehearsal_contract")
        == "bounded_arm_only_contact_linear_v1"
        and metadata.get("hitter_pingpong_teacher_contract")
        == "contact_linear_actor_visible_v1"
        and metadata.get("hitter_pingpong_replay_contract")
        == (
            "markov_side_phase_severity_v3_recovery_debt_support_geometry_perturbed_v2"
            if (schema33 or schema34)
            else "markov_side_phase_severity_v3_recovery_debt_perturbed_v1"
            if schema32
            else "markov_side_phase_severity_v3"
        )
        and metadata.get("hitter_pingpong_reward_contract")
        == (
            ("observable_step_task_com_footwork_v10" if schema34_v3 else "support_consistent_com_footwork_v9" if schema34_v2 else "visible_sequential_foot_trajectory_wait_arm_v8")
            if schema34
            else "immutable_home_terminal_tracking_outer_safety_licensed_replant_v7"
            if schema33
            else "immutable_home_terminal_tracking_outer_safety_v6"
        )
        and metadata.get("hitter_pingpong_recovery_phase_contract")
        == (
            ("persistent_observable_lift_transport_land_v10" if schema34_v3 else "current_fk_goal_error_across_commit_v9" if schema34_v2 else "signed_replant_dense_trajectory_no_latch_income_v8")
            if schema34
            else "continuous_l0_terminal_licensed_replant_release_v7"
            if schema33
            else "continuous_l0_terminal_selected_foot_release_v6"
        )
        and metadata.get("hitter_pingpong_reach_permission_contract")
        == (
            ("signed_external_reach_observable_footstep_task_v7" if schema34_v3 else "signed_external_reach_and_fk_replant_need_v6")
            if schema34
            else "staged_support_intent_sim_training_candidate_v5"
        )
        and metadata.get("hitter_pingpong_actor_observation_reach_indices")
        == "110,111"
        and metadata.get("hitter_pingpong_actor_observation_reach_sources")
        == (
            ("planner_signed_reach_level_foot,mocap_imu_fk_visible_footstep_task" if schema34_v3 else "planner_signed_reach_level_foot,fk_home_anchor_signed_need")
            if schema34
            else "planner_target_tuple,planner_target_tuple"
        )
        and metadata.get("hitter_pingpong_actor_observation_nonprivileged_contract")
        == (
            ("planner_plus_mocap_imu_fk_observable_task_v5" if schema34_v3 else "planner_plus_fk_home_anchor_no_contact_latch_v4")
            if schema34
            else "planner_tuple_no_plant_inference_v3"
        )
        and metadata.get("hitter_pingpong_optional_reach_contract")
        == "staged_support_intent_sim_training_candidate_v5"
        and metadata.get("hitter_pingpong_optional_reach_trajectory_qualification")
        == "SIM_TRAINING_CANDIDATE"
        and metadata.get("hitter_pingpong_optional_reach_deployment_qualification")
        == "SIM_TRAINING_CANDIDATE_NOT_DEPLOYMENT_QUALIFIED"
        and metadata.get("hitter_pingpong_onnx_output_contract") == "actions31_v1"
        and inputs.get("obs") == [1, 112]
        and outputs.get("actions") == [1, 31]
        and "core_action_shadow_owned" not in outputs
        and metadata.get("hitter_pingpong_question_bank_artifact_contract")
        == "fixed_home_support_training_candidate_bank_v5"
        and metadata.get("hitter_pingpong_question_bank_sha256")
        == "9a4ff9e89f360e5be25e6ee6df571b30aee93a9f7f88a70ae5a308606e294f7e"
        and metadata.get("hitter_pingpong_question_bank_receipt_sha256")
        == "347d16d386ba372feb3c8bc4c00d23542f69de62f14ccd4ffcb8a881f5b4b493"
        and metadata.get("hitter_pingpong_target_tuple_prefilter_contract")
        == "fixed_home_support_training_candidate_bank_v5_axis_envelope_prefilter_not_trajectory_certification_v1"
        and metadata.get("qdes_action_contract")
        == (
            "v12_affine_safe_slew_qdes_v1"
            if schema32
            else "v11_affine_safe_qdes_v1"
        )
        and (
            not schema32
            or (
                metadata.get("hitter_pure_last_action_feedback")
                == "executed_qdes_raw_v12"
                and metadata.get("qdes_policy_feedback_contract")
                == "executed_qdes_raw_v12"
                and metadata.get("qdes_slew_tilt_full_scale_rad") == "0.0698"
                and metadata.get("qdes_slew_tilt_min_scale_rad") == "0.1745"
                and metadata.get("qdes_slew_speed_full_scale_mps") == "0.25"
                and metadata.get("qdes_slew_speed_min_scale_mps") == "0.5"
                and metadata.get("qdes_slew_min_scale") == "0.35"
                and metadata.get("qdes_slew_state_source")
                == "actor_visible_projected_gravity_tilt_and_filtered_mocap_speed_v1"
            )
        )
    ):
        print(
            "schema34_candidate"
            if schema34
            else "schema33_candidate"
            if schema33
            else "schema32_candidate" if schema32 else "schema31_candidate"
        )
    else:
        raise SystemExit(
            f"Schema{'34' if schema34 else '33' if schema33 else '32' if schema32 else '31'} stable-HOME support-candidate "
            "metadata tuple is incoherent"
        )
elif recipe == "hitter_pingpong_build2_trainable_home_optional_reach_v1":
    inputs = {
        input_value.name: [
            dim.dim_value for dim in input_value.type.tensor_type.shape.dim
        ]
        for input_value in model.graph.input
    }
    outputs = {
        output.name: [dim.dim_value for dim in output.type.tensor_type.shape.dim]
        for output in model.graph.output
    }
    if (
        enabled == "false"
        and metadata.get("actor_obs_contract")
        == "hitter_pure_112_headslots_vxy_reach_v1"
        and metadata.get("hitter_pure_runtime_contract") == "rally_final_v2"
        and metadata.get("hitter_pure_training_recipe_version") == "1"
        and metadata.get("hitter_pingpong_build_contract")
        == "build2_trainable_home_optional_reach_schema29"
        and metadata.get("hitter_pingpong_planner_commit_contract")
        == "fresh_schema3_flight_shared_session_home_support_intent_v2"
        and metadata.get("hitter_pingpong_transition_contract")
        == "policy_owned_112d_markov_trainable_no_qdes_override_v1"
        and metadata.get("hitter_pingpong_preempt_commit_delay_support_s")
        == "0.50,0.65"
        and metadata.get("hitter_pingpong_training_initialization_contract")
        == "schema29_from_build1_model21800_functional_vxy_zero_append_reach_trainable_v1"
        and metadata.get("hitter_pingpong_actor_warm_start_receipt")
        == (
            "model_21800.pt,sha256="
            "daca62616178177647cf9949128582785529dbe6b7ba080fc96d86da23916561,"
            "source_input_dim=110,destination_input_dim=112,"
            "zeroed_source_cols=76|81,appended_zero_cols=110|111,"
            "trainable_full31=true"
        )
        and metadata.get("hitter_pingpong_actor_architecture_contract")
        == "trainable_full31_core_optional_reach_curriculum_v1"
        and metadata.get("hitter_pingpong_side_rehearsal_contract")
        == "build2_side_prior,fixed_home,nominal_fh_bh_arm_teacher"
        and metadata.get("hitter_pingpong_teacher_contract")
        == (
            "safe_projected_build1_actor_mean,nominal_fh_bh_arms14,"
            "followthrough=0.12,inset=0.08,huber=0.10,"
            "coefficient=0.25->0.05,gradient_ratio_max=0.03"
        )
        and metadata.get("base_mocap_stale_burst_contract")
        == "localization_fresh_outage_empirical_v1"
        and metadata.get("base_mocap_stale_burst_receipt_sha256")
        == "ed805160905888fddc12a388f0ed27aaa11951283301c10496b0925a1401c82c"
        and metadata.get("base_mocap_stale_burst_start_prob")
        == "0.015090824406148"
        and metadata.get("base_mocap_stale_burst_length_weights")
        == "3,1,4,12,9,15,11,12,8,7,16,27,19,15,3"
        and metadata.get("base_mocap_stale_burst_curriculum_contract")
        == (
            "same_run_linear_steps_384000_256000;actor_visible_stale_hold;"
            "first_fresh_velocity_zero_v1"
        )
        and metadata.get("hitter_pingpong_replay_contract")
        == "markov_side_phase_severity_v3"
        and metadata.get("hitter_pingpong_reward_contract")
        == "immutable_home_world_torso_trainable_support_v5"
        and metadata.get("hitter_pingpong_recovery_phase_contract")
        == "full_swing_torso_visible_settle_trainable_v5"
        and metadata.get("hitter_pingpong_reach_permission_contract")
        == "staged_planner_support_intent_no_plant_inference_v4"
        and metadata.get("hitter_pingpong_actor_observation_reach_indices")
        == "110,111"
        and metadata.get("hitter_pingpong_actor_observation_reach_sources")
        == "planner_target_tuple,planner_target_tuple"
        and metadata.get(
            "hitter_pingpong_actor_observation_nonprivileged_contract"
        ) == "planner_tuple_no_plant_inference_v3"
        and metadata.get("hitter_pingpong_optional_reach_contract")
        == "staged_support_intent_candidate_v4"
        and metadata.get("hitter_pingpong_optional_reach_trajectory_qualification")
        == "NOT_PROVEN"
        and metadata.get("hitter_pingpong_optional_reach_deployment_qualification")
        == "NOT_PROVEN"
        and metadata.get("hitter_pingpong_onnx_output_contract")
        == "actions31_v1"
        and inputs.get("obs") == [1, 112]
        and outputs.get("actions") == [1, 31]
        and "core_action_shadow_owned" not in outputs
        and not metadata.get("hitter_pingpong_core_shadow_feedback_contract")
        and not metadata.get("hitter_pingpong_core_shadow_action_indices")
        and not metadata.get("hitter_pingpong_core_shadow_observation_indices")
        and not metadata.get("hitter_pingpong_core_shadow_reset_contract")
        and metadata.get("hitter_pingpong_question_bank_sha256")
        == "894bea744fc5b119312b619c0368afe9ae048d1b076f67e80f25f7c1111907ee"
        and metadata.get("hitter_pingpong_question_bank_artifact_contract")
        == "fixed_home_support_candidate_bank_v4"
        and metadata.get("hitter_pingpong_question_bank_receipt_sha256")
        == "809a39b7105229ca89d627fdf508a4d4763a709824e0d69b766bb96d5ec84eec"
        and metadata.get("hitter_pingpong_target_tuple_prefilter_contract")
        == "fixed_home_support_candidate_bank_v4_axis_envelope_prefilter_not_trajectory_certification_v1"
        and metadata.get("hitter_pingpong_target_tuple_frame_contract")
        == "table_world_xy_minus_immutable_session_home_xy_actor_world_z_v1"
        and metadata.get("hitter_pingpong_target_tuple_relative_x_sensor_tolerance_m")
        == "0.0015"
        and metadata.get("hitter_pingpong_target_tuple_continuous_correlated_qualification")
        == "NOT_PROVEN_axis_prefilter_only"
    ):
        print("schema29_candidate")
    else:
        raise SystemExit("Schema29 trainable optional-reach metadata tuple is incoherent")
elif recipe == "hitter_pingpong_build2_frozen_core_optional_reach_v1":
    outputs = {
        output.name: [dim.dim_value for dim in output.type.tensor_type.shape.dim]
        for output in model.graph.output
    }
    if (
        enabled == "true"
        and metadata.get("actor_obs_contract")
        == "hitter_pure_127_headslots_vxy_reach_core_shadow_v1"
        and metadata.get("hitter_pure_runtime_contract") == "rally_final_v2"
        and metadata.get("hitter_pure_training_recipe_version") == "1"
        and metadata.get("hitter_pingpong_build_contract")
        == "build2_frozen_core_shadow_schema28"
        and metadata.get("hitter_pingpong_planner_commit_contract")
        == "fresh_schema3_flight_shared_session_home_support_intent_v2"
        and metadata.get("hitter_pingpong_transition_contract")
        == "policy_owned_127d_markov_frozen_core_shadow_residual_no_qdes_override_v1"
        and metadata.get("hitter_pingpong_preempt_commit_delay_support_s")
        == "0.50,0.65"
        and metadata.get("hitter_pingpong_lifecycle_contract")
        == (
            "cold_hold_neutral;contact_plus_0p12_recovery_latch;"
            "fresh_external_flight_preempt_0p50_0p65;immutable_session_home;"
            "policy_owned_transition;no_ready_gate"
        )
        and metadata.get("hitter_pingpong_training_initialization_contract")
        == (
            "schema28_from_build1_model21800_functional_vxy_zero_reach_frozen_core_shadow_v1"
        )
        and metadata.get("hitter_pingpong_actor_warm_start_receipt")
        == (
            "model_21800.pt,sha256="
            "daca62616178177647cf9949128582785529dbe6b7ba080fc96d86da23916561,"
            "source_input_dim=110,frozen_core_input_dim=112,"
            "destination_actor_observation_dim=127,"
            "zeroed_source_cols=76|81,appended_zero_cols=110|111,frozen_core=true,"
            "core_shadow_feedback=true,zero_residuals=true"
        )
        and metadata.get("hitter_pingpong_optional_reach_contract")
        == "staged_support_intent_training_v3"
        and metadata.get("hitter_pingpong_question_bank_sha256")
        == "f74db60887fe0f701dee9157c8c7ea87038be78e805f615515f9b0f5d62c84ed"
        and metadata.get("hitter_pingpong_question_bank_receipt_sha256")
        == "3008fbb64c439db29e94a599f198cebb803c284ca8a52f1790e23a87896432fe"
        and metadata.get("hitter_pingpong_reach_permission_contract")
        == "staged_planner_support_intent_no_plant_inference_v4"
        and metadata.get("hitter_pingpong_actor_architecture_contract")
        == "frozen_stage0_core_shadow_feedback_waist_leg_residual_v1"
        and metadata.get("hitter_pingpong_reward_contract")
        == "immutable_home_world_torso_unified_support_residual_v4"
        and metadata.get("hitter_pingpong_recovery_phase_contract")
        == "full_swing_torso_then_visible_settle_v4"
        and metadata.get("hitter_pingpong_actor_observation_reach_indices")
        == "110,111"
        and metadata.get("hitter_pingpong_actor_observation_reach_sources")
        == "planner_target_tuple,planner_target_tuple"
        and metadata.get(
            "hitter_pingpong_actor_observation_nonprivileged_contract"
        ) == "planner_tuple_plus_policy_shadow_no_plant_inference_v2"
        and metadata.get("hitter_pingpong_core_shadow_feedback_contract")
        == "core_owned15_actions_0_1_2_19_30_append112_126_v1"
        and metadata.get("hitter_pingpong_core_shadow_action_indices")
        == "0,1,2,19,20,21,22,23,24,25,26,27,28,29,30"
        and metadata.get("hitter_pingpong_core_shadow_observation_indices")
        == "112,113,114,115,116,117,118,119,120,121,122,123,124,125,126"
        and metadata.get("hitter_pingpong_onnx_output_contract")
        == "actions31_core_action_shadow_owned15_v1"
        and outputs.get("actions") == [1, 31]
        and outputs.get("core_action_shadow_owned") == [1, 15]
        and metadata.get("hitter_pingpong_target_tuple_prefilter_contract")
        == "fixed_home_staged_support_intent_bank_v3_axis_envelope_prefilter_not_certification_v1"
        and metadata.get("hitter_pingpong_target_tuple_frame_contract")
        == "table_world_xy_minus_immutable_session_home_xy_actor_world_z_v1"
        and metadata.get("hitter_pingpong_target_tuple_relative_x_sensor_tolerance_m")
        == "0.0015"
        and metadata.get("hitter_pingpong_target_tuple_continuous_correlated_qualification")
        == "NOT_PROVEN_axis_prefilter_only"
        and metadata.get("hitter_pure_pos_range_per_clip")
        == "0.5800,0.5800,-0.3500,-0.2500,0.9800,1.2600;0.5800,0.5800,-0.1400,0.1800,0.9800,1.2600"
        and metadata.get("hitter_pure_vel_range_per_clip")
        == "1.2600,2.2500,-0.2100,0.6500,0.6200,1.6000;1.5000,2.1700,-0.4700,0.3900,0.7000,1.3500"
    ):
        print("schema28")
    else:
        raise SystemExit("Schema28 optional-reach/shadow metadata tuple is incoherent")
elif recipe == "hitter_pingpong_build2_torso_optional_reach_v2" and enabled == "true":
    raise SystemExit(
        "legacy Schema27 enabled artifact is not a Schema28 formal candidate; "
        "use its explicit legacy harness"
    )
elif enabled == "true":
    raise SystemExit("non-Schema28/29/31/32/33/34 artifact may not enable optional reach")
else:
    print("schema2")
PY
)" || {
  echo "[hitter-pingpong] unable to bind Planner schema to ONNX metadata"
  exit 2
}
case "$HITTER_PLANNER_WIRE" in
  schema31_candidate|schema32_candidate|schema33_candidate|schema34_candidate)
    HITTER_V5_CANDIDATE=1
    ;;
  *)
    HITTER_V5_CANDIDATE=0
    ;;
esac
if [ "$HITTER_GATE3_MODE" = "torture" ] && \
   [ "$HITTER_V5_CANDIDATE" != "1" ]; then
  echo "[hitter-pingpong] low/corner torture requires a Schema31/32/33/34 v5 candidate"
  exit 2
fi
if [ "$HITTER_PLANNER_WIRE" = "schema28" ] || \
   [ "$HITTER_PLANNER_WIRE" = "schema29" ] || \
   [ "$HITTER_PLANNER_WIRE" = "schema29_candidate" ] || \
   [ "$HITTER_V5_CANDIDATE" = "1" ]; then
  EXPECTED_SCHEMA3_PLANNER_PROFILE="$SCHEMA3_OPTIONAL_REACH_PLANNER_PROFILE"
  if [ -n "${PP_SCHEMA3_SIM_OPTIONAL_REACH+x}" ]; then
    SCHEMA31_SIM_OPTIONAL_REACH="$PP_SCHEMA3_SIM_OPTIONAL_REACH"
  elif [ -n "${PP_SCHEMA31_SIM_OPTIONAL_REACH+x}" ]; then
    SCHEMA31_SIM_OPTIONAL_REACH="$PP_SCHEMA31_SIM_OPTIONAL_REACH"
  elif [ "$HITTER_V5_CANDIDATE" = "1" ]; then
    # Formal Schema31/32 Gate3 now includes the v5 corner stress cells by
    # default: 10 L0, 8 L1 unload/side-reach, and 8 L2 temporary-step reaches.
    SCHEMA31_SIM_OPTIONAL_REACH=1
  else
    SCHEMA31_SIM_OPTIONAL_REACH=0
  fi
  case "$SCHEMA31_SIM_OPTIONAL_REACH" in
    0|1) ;;
    *)
      echo "[hitter-pingpong] invalid PP_SCHEMA3_SIM_OPTIONAL_REACH=$SCHEMA31_SIM_OPTIONAL_REACH"
      exit 2
      ;;
  esac
  if [ "$HITTER_GATE3_MODE" = "torture" ] && \
     [ "$SCHEMA31_SIM_OPTIONAL_REACH" != "1" ]; then
    echo "[hitter-pingpong] low/corner torture requires Schema3 simulator L1/L2 reach"
    exit 2
  fi
  if [ "$HITTER_PLANNER_WIRE" = "schema29_candidate" ] || \
     { [ "$HITTER_V5_CANDIDATE" = "1" ] &&
       [ "$SCHEMA31_SIM_OPTIONAL_REACH" = "0" ]; }; then
    EXPECTED_SCHEMA3_PLANNER_PROFILE="$SCHEMA29_CANDIDATE_PLANNER_PROFILE"
  fi
  if [ -n "${PP_PLANNER_PROFILE:-}" ] &&
     [ "$PP_PLANNER_PROFILE" != "$EXPECTED_SCHEMA3_PLANNER_PROFILE" ]; then
    echo "[hitter-pingpong] Schema28/29/31/32/33/34 artifact forbids a different Planner profile: $PP_PLANNER_PROFILE"
    exit 2
  fi
  if [ ! -r "$EXPECTED_SCHEMA3_PLANNER_PROFILE" ]; then
    echo "[hitter-pingpong] Schema3 Planner overlay missing: $EXPECTED_SCHEMA3_PLANNER_PROFILE"
    exit 2
  fi
  export PP_PLANNER_PROFILE="$EXPECTED_SCHEMA3_PLANNER_PROFILE"
  if [ "$HITTER_PLANNER_WIRE" = "schema29_candidate" ]; then
    case "${PP_REQUIRE_OPTIONAL_REACH:-auto}" in
      auto|0) export PP_REQUIRE_OPTIONAL_REACH=0 ;;
      1)
        echo "[hitter-pingpong] Schema29 v4 candidates are NOT_PROVEN; optional qualification lane is forbidden"
        exit 2
        ;;
      *)
        echo "[hitter-pingpong] invalid PP_REQUIRE_OPTIONAL_REACH=${PP_REQUIRE_OPTIONAL_REACH}"
        exit 2
        ;;
    esac
  elif [ "$HITTER_V5_CANDIDATE" = "1" ]; then
    if [ "$SCHEMA31_SIM_OPTIONAL_REACH" = "1" ]; then
      case "${PP_REQUIRE_OPTIONAL_REACH:-auto}" in
        auto|1) export PP_REQUIRE_OPTIONAL_REACH=1 ;;
        0)
          echo "[hitter-pingpong] Schema31/32 simulator L1/L2 lane requires optional-reach evidence"
          exit 2
          ;;
        *)
          echo "[hitter-pingpong] invalid PP_REQUIRE_OPTIONAL_REACH=${PP_REQUIRE_OPTIONAL_REACH}"
          exit 2
          ;;
      esac
      export PP_EXTRA_ARGS="${PP_EXTRA_ARGS:+$PP_EXTRA_ARGS }--schema3-sim-optional-reach"
    else
      case "${PP_REQUIRE_OPTIONAL_REACH:-auto}" in
        auto|0) export PP_REQUIRE_OPTIONAL_REACH=0 ;;
        1)
          echo "[hitter-pingpong] Schema31/32 default lane is locked to L0; set PP_SCHEMA3_SIM_OPTIONAL_REACH=1 only for explicit x86 Gate3 L1/L2 audit"
          exit 2
          ;;
        *)
          echo "[hitter-pingpong] invalid PP_REQUIRE_OPTIONAL_REACH=${PP_REQUIRE_OPTIONAL_REACH}"
          exit 2
          ;;
      esac
    fi
  else
    case "${PP_REQUIRE_OPTIONAL_REACH:-auto}" in
      auto|1) export PP_REQUIRE_OPTIONAL_REACH=1 ;;
      0)
        echo "[hitter-pingpong] certified Schema3 artifact forbids disabling its physical evidence lane"
        exit 2
        ;;
      *)
        echo "[hitter-pingpong] invalid PP_REQUIRE_OPTIONAL_REACH=${PP_REQUIRE_OPTIONAL_REACH}"
        exit 2
        ;;
    esac
  fi
  # Match the filtered bank's immutable-HOME side partition exactly. Legacy
  # Schema2 profiles retain their historical 0.04 hysteresis.
  export PP_SPLIT_Y=-0.25 PP_SPLIT_HYST=0.0
  export PP_RAPID_CONTACT_TO_NEXT_COMMIT_MIN_S=0.50
  export PP_RAPID_CONTACT_TO_NEXT_COMMIT_MAX_S=0.65
  # The certification lane uses 10/8/8.  The torture lane repeats a balanced
  # low/nominal-height side-level cycle without resetting the plant. Neither
  # distribution is a claim about the training mixture.
  if [ "$HITTER_PLANNER_WIRE" = "schema29_candidate" ] || \
     { [ "$HITTER_V5_CANDIDATE" = "1" ] &&
       [ "$SCHEMA31_SIM_OPTIONAL_REACH" = "0" ]; }; then
    export PP_OPTIONAL_REACH_DISTRIBUTION_CONTRACT=candidate_only_l0_v1
  elif [ "$HITTER_GATE3_MODE" = "torture" ]; then
    export PP_OPTIONAL_REACH_DISTRIBUTION_CONTRACT=torture_balanced_low_corner_v1
  else
    export PP_OPTIONAL_REACH_DISTRIBUTION_CONTRACT=stress_balanced_10_8_8_v1
  fi
  export PP_OPTIONAL_REACH_TRAJECTORY_QUALIFICATION="$(
    if [ "$HITTER_V5_CANDIDATE" = "1" ]; then
      printf %s SIM_TRAINING_CANDIDATE
    else
      printf %s NOT_PROVEN
    fi
  )"
  # Schema28/29 requirements are metadata-implied and cannot be disabled with
  # environment overrides. q_des smoothness remains audit-only; physical
  # torso/base/foot/qd/plant evidence owns the formal verdict.
  export PP_REQUIRE_HOME_SUPPORT_FRAME=1
  export PP_REQUIRE_TORSO_HOLD=1
  export PP_REQUIRE_TORSO_HOLD_THREE_AXIS=1
  export PP_REQUIRE_IDLE_SMOOTHNESS=1
  export PP_REQUIRE_COLLISION_LEDGER=1
  export PP_REQUIRE_QUESTION_BANK_PROVENANCE=1
  # Establish the playing-session boundary before the first flight. Planner
  # freezes immutable HOME and derives x_hit=HOME.x+0.58 from the same base
  # snapshot; the Schema28/29 1.5 mm prefilter remains unchanged.
  export PP_REQUIRE_XHIT_FREEZE=1
  export PP_XHIT_FREEZE_SESSION_HOME=true
  if [ "$HITTER_V5_CANDIDATE" = "1" ]; then
    export PP_QUESTION_BANK_PATH="$HOPE_ROOT/hope_training/whole_body_tracking/assets/question_bank/fixed_home_support_training_candidate_bank_v5.npz"
    export PP_QUESTION_BANK_RECEIPT_PATH="$HOPE_ROOT/hope_training/whole_body_tracking/assets/question_bank/fixed_home_support_training_candidate_bank_v5.receipt.json"
    if [ "$HITTER_PLANNER_WIRE" = "schema34_candidate" ]; then
      export PP_RALLY_REPORT_MODE=schema34
    elif [ "$HITTER_PLANNER_WIRE" = "schema33_candidate" ]; then
      export PP_RALLY_REPORT_MODE=schema33
    elif [ "$HITTER_PLANNER_WIRE" = "schema32_candidate" ]; then
      export PP_RALLY_REPORT_MODE=schema32
    else
      export PP_RALLY_REPORT_MODE=schema31
    fi
    if [ "$SCHEMA31_SIM_OPTIONAL_REACH" = "0" ]; then
      # v5 is a discrete complete-row training bank.  Build a simulator-only
      # physical launch ledger from exact routed L0 rows, then bind the C++
      # Planner to those same rows.  The Planner independently checks its raw
      # Stage-2 physical prediction before it may emit an exact bank tuple.
      export PP_QUESTION_FIXTURE_CSV="${PP_QUESTION_FIXTURE_CSV:-/tmp/pp_gate3_v5_question_fixture.csv}"
      export PP_QUESTION_FIXTURE_RECEIPT_JSON="${PP_QUESTION_FIXTURE_RECEIPT_JSON:-/tmp/pp_gate3_v5_question_fixture_receipt.json}"
      if ! python3 "$HOPE_ROOT/hope_ws/src/hope_planner_cpp/scripts/pp_gate3_v5_question_fixture.py" \
        --bank "$PP_QUESTION_BANK_PATH" \
        --receipt "$PP_QUESTION_BANK_RECEIPT_PATH" \
        --planner-csv "$PP_QUESTION_FIXTURE_CSV" \
        --json "$PP_QUESTION_FIXTURE_RECEIPT_JSON"; then
        echo "[hitter-pingpong] unable to construct the exact v5 Gate3 fixture"
        exit 2
      fi
      PP_SERVES_LIST="$(python3 - "$PP_QUESTION_FIXTURE_RECEIPT_JSON" <<'PY'
import json
import sys

receipt = json.load(open(sys.argv[1], encoding="utf-8"))
if (
    receipt.get("receipt_contract") != "gate3_schema31_v5_l0_fixture_receipt_v1"
    or receipt.get("fixture_contract") != "schema31_gate3_v5_l0_fixture_v1"
    or receipt.get("simulator_only") is not True
    or receipt.get("expected_flights") != 26
    or receipt.get("flight_ids") != list(range(1, 27))
    or len(receipt.get("serves_flat", ())) != 26 * 6
):
    raise SystemExit("generated v5 fixture receipt is incoherent")
print(json.dumps(receipt["serves_flat"], separators=(",", ":")))
PY
)" || {
        echo "[hitter-pingpong] unable to bind v5 physical serves to fixture receipt"
        exit 2
      }
      export PP_SERVES_LIST
      export PP_QUESTION_FIXTURE_CONTRACT=schema31_gate3_v5_l0_fixture_v1
      export PP_QUESTION_FIXTURE_BANK_SHA256=9a4ff9e89f360e5be25e6ee6df571b30aee93a9f7f88a70ae5a308606e294f7e
      export PP_QUESTION_FIXTURE_RECEIPT_SHA256=347d16d386ba372feb3c8bc4c00d23542f69de62f14ccd4ffcb8a881f5b4b493
      export PP_QUESTION_FIXTURE_EXPECTED_FLIGHTS=26
      export PP_QUESTION_FIXTURE_MAX_POSITION_ERROR_M=0.03
      export PP_QUESTION_FIXTURE_MAX_VELOCITY_ERROR_MPS=0.10
      export PP_QUESTION_FIXTURE_MAX_TTS_ERROR_S=0.03
      echo "[hitter-pingpong] v5 exact-row simulator fixture bound: $PP_QUESTION_FIXTURE_CSV"
    fi
  elif [ "$HITTER_PLANNER_WIRE" = "schema29_candidate" ]; then
    export PP_QUESTION_BANK_PATH="$HOPE_ROOT/hope_training/whole_body_tracking/assets/question_bank/fixed_home_support_candidate_bank_v4.npz"
    export PP_QUESTION_BANK_RECEIPT_PATH="$HOPE_ROOT/hope_training/whole_body_tracking/assets/question_bank/fixed_home_support_candidate_bank_v4.receipt.json"
    export PP_RALLY_REPORT_MODE=schema29
  else
    export PP_QUESTION_BANK_PATH="$HOPE_ROOT/hope_training/whole_body_tracking/assets/question_bank/fixed_home_staged_support_intent_bank_v3.npz"
    export PP_QUESTION_BANK_RECEIPT_PATH="$HOPE_ROOT/hope_training/whole_body_tracking/assets/question_bank/fixed_home_staged_support_intent_bank_v3.receipt.json"
    export PP_RALLY_REPORT_MODE="$HITTER_PLANNER_WIRE"
  fi
  echo "[hitter-pingpong] Planner wire bound to ${HITTER_PLANNER_WIRE} Schema3 optional reach"
elif [ "${PP_PLANNER_PROFILE:-}" = "$SCHEMA3_OPTIONAL_REACH_PLANNER_PROFILE" ] || \
     [ "${PP_PLANNER_PROFILE:-}" = "$SCHEMA29_CANDIDATE_PLANNER_PROFILE" ]; then
  echo "[hitter-pingpong] legacy/disabled artifact may not load the Schema3 optional-reach Planner overlay"
  exit 2
else
  if [ "${PP_REQUIRE_OPTIONAL_REACH:-auto}" = "auto" ]; then
    export PP_REQUIRE_OPTIONAL_REACH=0
  fi
  echo "[hitter-pingpong] Planner wire bound to legacy Schema2"
fi

export A3_PINGPONG_RUNTIME_CFG="${A3_PINGPONG_RUNTIME_CFG:-$HITTER_DIST/config/a3_runtime_config.pingpong.hitter_pingpong.yaml}"
if [ ! -f "$A3_PINGPONG_RUNTIME_CFG" ]; then
  echo "[hitter-pingpong] runtime config missing: $A3_PINGPONG_RUNTIME_CFG"
  echo "[hitter-pingpong] rebuild with --runtime-cfg src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.hitter_pingpong.yaml"
  exit 2
fi
export PP_SIM_INSTALL="${PP_SIM_INSTALL:-$GEAR/../A3_MuJoCo_Sim/aimrt_mujoco_sim/build_v11/install}"
if [ ! -x "$PP_SIM_INSTALL/bin/aimrt_main" ]; then
  echo "[hitter-pingpong] instrumented MuJoCo install missing: $PP_SIM_INSTALL"
  exit 2
fi
# Unified entry resolves the scene independently of model provenance. Generated
# exports are shell-quoted; this hook follows legacy defaults and ABI routing.
if [ -n "${PP_SCENE_EXPORTS:-}" ]; then
  source "$PP_SCENE_EXPORTS" || exit 2
fi
rm -f "$PP_PLANNER_EVIDENCE_JSON" "$PP_PHYSICAL_EVIDENCE_JSON"
# The shared engine writes this receipt from hope_planner_cpp_node's live CSV.

# Keep the supervised PID alive until the engine's flight drain and recorder
# cleanup finish, including SIGINT/SIGTERM. A wrapper shell can otherwise exit
# first and cause the unified runner to archive a partial trajectory.
exec bash "/home/user/Documents/Codex/2026-11-08/model-u-fusion-13800-gate3-x20-2/work/rally-suite.sh"
