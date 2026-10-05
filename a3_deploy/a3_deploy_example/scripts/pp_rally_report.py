#!/usr/bin/env python3
"""Deep-dive analyzer for the Gate-3 rally rehearsal actor-observation CSV.

Reads the runner's --obs-csv capture (and optionally the conductor's report JSON)
and prints a per-swing table + health checks, so a failed/odd rally can be
debugged LOCALLY from numbers instead of re-running with a viewer.

110-D obs layout (pp_obs_builder.hpp build_obs_110 / LogFirstTick blks110):
  [0:3]    base_ang_vel        [3:34]   joint_pos_rel     [34:65]  joint_vel
  [65:96]  last_action         [96:99]  projected_gravity [99:101] base_forward_xy
  [101:103] base_target_delta_xy (world station - base)
  [103:106] racket_target_rel_base (world)   [106:109] racket_target_vel_w
  [109]    time_to_strike

Schema27 keeps that exact 110-D prefix and appends two Planner tuple permissions:
  [110]    reach_level       [111] swing_foot_sign
Stage 0 requires both values to remain exactly zero; they are never inferred from plant state.

Schema28 preserves those 112 columns and appends the policy-owned frozen-core action shadow:
  [112:127] previous frozen-core actions [0,1,2,19..30]
The first 112 columns continue to carry the real previous composed action; only the frozen
112-D core consumes the shadow replacement. The trainable residual consumes all 127 columns.

Schema29/31/32 return to the Schema27 112-D layout and a single 31-action output.  Unlike Schema28,
the complete 31-D actor mean is trainable and there is no core-action shadow channel/output.

V15 keeps the position-receipt localization state and appends the finite-gait
command that is generated identically in training and in the runner:
  [110:112] filtered base_xy_velocity from table-relative mocap positions
  [112]     normalized local receipt age
  [113]     desired lateral velocity
  [114:116] left/right gait clocks
  [116]     locomotion mode (+1 STEP, 0 STAND, -1 strike/recovery)
  [117]     upper-body intervention indicator (always 0 on deployment)

Swing segmentation: at idle the runner pins tts at the selected clip's windup
maximum; during a swing tts DECREASES every tick down to the clip-end clamp.
A swing = a maximal run of strictly-decreasing tts spanning >0.5 s.

Usage: python3 pp_rally_report.py /tmp/pp_obs.csv [/tmp/pp_rally_report.json]
                                   [--mode legacy|...|schema28|schema29|schema31|schema32|auto]
"""
import argparse
import csv
import json
import math
from pathlib import Path
import re
import sys

B = {"ang_vel": (0, 3), "jpos": (3, 34), "jvel": (34, 65), "act": (65, 96),
     "grav": (96, 99), "fwd": (99, 101), "dstation": (101, 103),
     "rkt_rel": (103, 106), "rkt_vel": (106, 109), "tts": 109,
     "base_vel": (110, 112), "loc_age": 112, "gait_vy": 113,
     "gait_clock": (114, 116), "locomotion_mode": 116, "upper_intervention": 117,
     "reach_level": 110, "swing_foot_sign": 111}
CONTROL_DT = 0.02  # runner policy rate = 50 Hz; CSV `ts` is motion frame, not wall seconds
JOINT_NAMES = (
    "left_hip_pitch_joint", "right_hip_pitch_joint", "waist_yaw_joint",
    "left_hip_roll_joint", "right_hip_roll_joint", "waist_roll_joint",
    "left_hip_yaw_joint", "right_hip_yaw_joint", "waist_pitch_joint",
    "left_knee_joint", "right_knee_joint", "head_yaw_joint",
    "left_shoulder_pitch_joint", "right_shoulder_pitch_joint",
    "left_ankle_pitch_joint", "right_ankle_pitch_joint", "head_pitch_joint",
    "left_shoulder_roll_joint", "right_shoulder_roll_joint",
    "left_ankle_roll_joint", "right_ankle_roll_joint",
    "left_shoulder_yaw_joint", "right_shoulder_yaw_joint", "left_elbow_joint",
    "right_elbow_joint", "left_wrist_roll_joint", "right_wrist_roll_joint",
    "left_wrist_pitch_joint", "right_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_wrist_yaw_joint",
)
BUILD1_SYNCHRONIZED_WAIT_ACTUAL_Q = {
    "waist_yaw_joint": -0.0118993,
    "waist_roll_joint": 0.0109062,
    "waist_pitch_joint": -0.0813170,
    "left_shoulder_pitch_joint": -0.2051270,
    "left_shoulder_roll_joint": 0.4168760,
    "left_shoulder_yaw_joint": -0.2557180,
    "left_elbow_joint": 0.8212550,
    "left_wrist_roll_joint": -0.2939120,
    "left_wrist_pitch_joint": -0.1946470,
    "left_wrist_yaw_joint": 0.2416450,
    "right_shoulder_pitch_joint": -0.0864108,
    "right_shoulder_roll_joint": -0.7464710,
    "right_shoulder_yaw_joint": 0.3140070,
    "right_elbow_joint": 0.2987870,
    "right_wrist_roll_joint": 0.9344860,
    "right_wrist_pitch_joint": 0.1260970,
    "right_wrist_yaw_joint": -0.1458180,
}
HITTER_FIXED_HOME_LANE_CONTRACTS = {
    "hitter_pingpong_fixed_home_rapid_1150ms_v1",
    "hitter_pingpong_low_corner_torture_v1",
}

# Keep the historical report windows bit-for-bit as ``legacy``.  FinalV3 deliberately
# lengthens the station-settle and recovery terms; its deploy report must measure the
# same windows as the task instead of silently certifying only the shorter V2 slice.
WINDOWS = {
    "legacy": {
        "pre": (0.12, 0.45),
        "post": (0.20, 1.00),
        "ready_heading": (0.45, 1.40),
        "post_heading": (0.35, 1.80),
    },
    "rally_final_v3": {
        "pre": (0.12, 1.10),
        "post": (0.20, 1.55),
        "ready_heading": (0.45, 1.10),
        "post_heading": (0.20, 1.55),
    },
    # RallyV8 (v13 FACEFIX clips, 2026-07-13): windup fh 0.82 s / bh 0.96 s pins the hold tts
    # (pre t_hi = the MIN windup, so the fh/bh pre-strike means stay comparable; ready_heading
    # t_hi = the MAX windup so the check spans the LATER-arming bh too). The post-swing x-drift
    # terms close at t_hi 1.2 -> mirror them in the windows.
    # A window shorter than the windup would sample the swing itself as "pre-strike" and the
    # base-speed HARD checks would fail a healthy policy.
    "rally_v8": {
        "pre": (0.12, 0.82),
        "post": (0.20, 1.20),
        "ready_heading": (0.45, 0.96),
        "post_heading": (0.20, 1.20),
    },
    "rally_v9": {
        "pre": (0.12, 0.96),
        "post": (0.20, 1.20),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 1.20),
    },
    "rally_v10": {
        "pre": (0.12, 1.10),
        "post": (0.20, 1.20),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 1.20),
    },
    "rally_v11": {
        "pre": (0.12, 1.10),
        "post": (0.20, 1.20),
        "ready_heading": (0.45, 1.00),
        # V11 extends the heading reward and deterministic telemetry through 1.55 s.
        "post_heading": (0.20, 1.55),
    },
    "rally_v12": {
        "pre": (0.12, 1.10),
        "post": (0.20, 1.20),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 1.55),
    },
    "rally_v13": {
        "pre": (0.12, 1.10),
        # V13's explicit post-swing x-lock remains live through 1.55 s.
        "post": (0.10, 1.55),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 1.55),
    },
    "rally_v14": {
        "pre": (0.12, 1.10),
        "post": (0.10, 1.55),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 1.55),
    },
    "hitter_pingpong_recovery_tts_v2": {
        "pre": (0.12, 1.10),
        "post": (0.10, 1.55),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 1.55),
    },
    # Schema21 preserves the complete native FH/BH clips. Their post-contact
    # tails are 1.30/1.20 s, so score the longer native tail without borrowing
    # the unrelated Build4 contact-preempt recovery window.
    "hitter_pingpong_build2_minimal_rootfix_v1": {
        "pre": (0.12, 1.10),
        "post": (0.10, 1.30),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 1.30),
    },
    # Schema22 keeps the same HITTER state/action and safety surfaces but admits the next fresh
    # flight during the old native tail.  Score only the physically available recovery interval;
    # the independent rapid-lane receipt below owns the 20/20 and engage-gap requirements.
    "hitter_pingpong_build2_rapid_preempt_v1": {
        "pre": (0.12, 1.10),
        "post": (0.10, 0.55),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 0.55),
    },
    # Schema24 keeps the fixed-HOME external-preempt clock.  The question-bank
    # and table-clearance changes do not widen the physically observable
    # post-contact interval used by this report.
    "hitter_pingpong_build2_fixed_home_feasible_v1": {
        "pre": (0.12, 1.10),
        "post": (0.10, 0.55),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 0.55),
    },
    "hitter_pingpong_build2_fixed_home_phase_recovery_v1": {
        "pre": (0.12, 1.10),
        "post": (0.10, 0.55),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 0.55),
    },
    "hitter_pingpong_build2_home_support_frame_v1": {
        "pre": (0.12, 1.10),
        "post": (0.10, 0.55),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 0.55),
    },
    "hitter_pingpong_build2_torso_optional_reach_v2": {
        "pre": (0.12, 1.10),
        "post": (0.10, 0.55),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 0.55),
    },
    "schema28": {
        "pre": (0.12, 1.10),
        "post": (0.10, 0.55),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 0.55),
    },
    "schema29": {
        "pre": (0.12, 1.10),
        "post": (0.10, 0.55),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 0.55),
    },
    "schema31": {
        "pre": (0.12, 1.10),
        "post": (0.10, 0.55),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 0.55),
    },
    "schema32": {
        "pre": (0.12, 1.10),
        "post": (0.10, 0.55),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 0.55),
    },
    "schema33": {
        "pre": (0.12, 1.10),
        "post": (0.10, 0.55),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 0.55),
    },
    "schema34": {
        "pre": (0.12, 1.10),
        "post": (0.10, 0.55),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 0.55),
    },
    # Continuous-v2 returns HOME after 0.12 s and may be preempted by a fresh external flight.
    # Score recovery using physical strike age rather than the visible WAIT clock.
    "hitter_pingpong_continuous_rally_v2": {
        "pre": (0.12, 1.10),
        "post": (0.10, 3.00),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 3.00),
    },
    # Repair-v3 trains and replays the hardware-observed late instability tail through 5.4 s.
    # Keep v2 unchanged for historical reports; a v3 candidate must expose the full interval.
    "hitter_pingpong_recovery_tts_v3": {
        "pre": (0.12, 1.10),
        "post": (0.10, 5.40),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 5.40),
    },
    "rally_v15": {
        "pre": (0.12, 1.10),
        "post": (0.10, 1.55),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 1.55),
    },
    # V17 restores the V11 action/deploy contract and extends recovery supervision through
    # tts=-1.55 s. Score the full trained recovery interval rather than V11's shorter
    # generic post-speed slice.
    "rally_v17": {
        "pre": (0.12, 1.10),
        "post": (0.10, 1.55),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 1.55),
    },
    "rally_v17_r10": {
        "pre": (0.12, 1.10),
        "post": (0.10, 1.55),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 1.55),
    },
}


SCHEMA29_METADATA_TUPLE = {
    "hitter_pure_training_recipe":
        "hitter_pingpong_build2_trainable_home_optional_reach_v1",
    "hitter_pure_training_recipe_version": "1",
    "hitter_pure_runtime_contract": "rally_final_v2",
    "actor_obs_contract": "hitter_pure_112_headslots_vxy_reach_v1",
    "hitter_pingpong_build_contract":
        "build2_trainable_home_optional_reach_schema29",
    "hitter_pingpong_replay_contract":
        "markov_side_phase_severity_v3",
    "hitter_pingpong_reward_contract":
        "immutable_home_world_torso_trainable_support_v5",
    "hitter_pingpong_recovery_phase_contract":
        "full_swing_torso_visible_settle_trainable_v5",
    "hitter_pingpong_transition_contract":
        "policy_owned_112d_markov_trainable_no_qdes_override_v1",
    "hitter_pingpong_preempt_commit_delay_support_s": "0.50,0.65",
    "hitter_pingpong_training_initialization_contract":
        "schema29_from_build1_model21800_functional_vxy_zero_append_reach_trainable_v1",
    "hitter_pingpong_actor_warm_start_receipt": (
        "model_21800.pt,sha256="
        "daca62616178177647cf9949128582785529dbe6b7ba080fc96d86da23916561,"
        "source_input_dim=110,destination_input_dim=112,"
        "zeroed_source_cols=76|81,appended_zero_cols=110|111,"
        "trainable_full31=true"
    ),
    "hitter_pingpong_actor_architecture_contract":
        "trainable_full31_core_optional_reach_curriculum_v1",
    "hitter_pingpong_side_rehearsal_contract":
        "build2_side_prior,fixed_home,nominal_fh_bh_arm_teacher",
    "hitter_pingpong_teacher_contract": (
        "safe_projected_build1_actor_mean,nominal_fh_bh_arms14,"
        "followthrough=0.12,inset=0.08,huber=0.10,"
        "coefficient=0.25->0.05,gradient_ratio_max=0.03"
    ),
    "base_mocap_stale_burst_contract":
        "localization_fresh_outage_empirical_v1",
    "base_mocap_stale_burst_receipt_sha256":
        "ed805160905888fddc12a388f0ed27aaa11951283301c10496b0925a1401c82c",
    "base_mocap_stale_burst_start_prob": "0.015090824406148",
    "base_mocap_stale_burst_length_weights":
        "3,1,4,12,9,15,11,12,8,7,16,27,19,15,3",
    "base_mocap_stale_burst_curriculum_contract": (
        "same_run_linear_steps_384000_256000;actor_visible_stale_hold;"
        "first_fresh_velocity_zero_v1"
    ),
    "hitter_pingpong_planner_commit_contract":
        "fresh_schema3_flight_shared_session_home_support_intent_v2",
    "hitter_pingpong_reach_permission_contract":
        "staged_planner_support_intent_no_plant_inference_v4",
    "hitter_pingpong_actor_observation_reach_indices": "110,111",
    "hitter_pingpong_actor_observation_reach_sources":
        "planner_target_tuple,planner_target_tuple",
    "hitter_pingpong_actor_observation_nonprivileged_contract":
        "planner_tuple_no_plant_inference_v3",
    "hitter_pingpong_optional_reach_training_enabled": "false",
    "hitter_pingpong_optional_reach_contract":
        "staged_support_intent_candidate_v4",
    "hitter_pingpong_optional_reach_trajectory_qualification": "NOT_PROVEN",
    "hitter_pingpong_optional_reach_deployment_qualification": "NOT_PROVEN",
    "hitter_pingpong_onnx_output_contract": "actions31_v1",
    "hitter_pingpong_question_bank_sha256":
        "894bea744fc5b119312b619c0368afe9ae048d1b076f67e80f25f7c1111907ee",
    "hitter_pingpong_question_bank_artifact_contract":
        "fixed_home_support_candidate_bank_v4",
    "hitter_pingpong_question_bank_receipt_sha256":
        "809a39b7105229ca89d627fdf508a4d4763a709824e0d69b766bb96d5ec84eec",
    "hitter_pingpong_target_tuple_prefilter_contract":
        "fixed_home_support_candidate_bank_v4_axis_envelope_prefilter_not_trajectory_certification_v1",
    "hitter_pingpong_target_tuple_frame_contract":
        "table_world_xy_minus_immutable_session_home_xy_actor_world_z_v1",
    "hitter_pingpong_target_tuple_relative_x_sensor_tolerance_m": "0.0015",
    "hitter_pingpong_target_tuple_continuous_correlated_qualification":
        "NOT_PROVEN_axis_prefilter_only",
}

SCHEMA31_METADATA_TUPLE = {
    "hitter_pure_training_recipe":
        "hitter_pingpong_build2_stable_home_step_candidate_v1",
    "hitter_pure_training_recipe_version": "1",
    "hitter_pure_runtime_contract": "rally_final_v2",
    "hitter_pure_ready_hold_steps_range": "150,250",
    "actor_obs_contract": "hitter_pure_112_headslots_vxy_reach_v1",
    "hitter_pingpong_build_contract":
        "build2_stable_home_step_candidate_schema31",
    "hitter_pingpong_replay_contract": "markov_side_phase_severity_v3",
    "hitter_pingpong_reward_contract":
        "immutable_home_terminal_tracking_outer_safety_v6",
    "hitter_pingpong_recovery_phase_contract":
        "continuous_l0_terminal_selected_foot_release_v6",
    "hitter_pingpong_transition_contract":
        "policy_owned_112d_markov_trainable_no_qdes_override_v1",
    "hitter_pingpong_training_initialization_contract":
        "schema31_from_build1_model21800_functional_vxy_zero_append_reach_trainable_v1",
    "hitter_pingpong_actor_architecture_contract":
        "trainable_full31_stable_home_step_candidate_v1",
    "hitter_pingpong_side_rehearsal_contract":
        "bounded_arm_only_contact_linear_v1",
    "hitter_pingpong_teacher_contract": "contact_linear_actor_visible_v1",
    "hitter_pingpong_planner_commit_contract":
        "fresh_schema3_flight_shared_session_home_support_intent_v2",
    "hitter_pingpong_reach_permission_contract":
        "staged_support_intent_sim_training_candidate_v5",
    "hitter_pingpong_actor_observation_reach_indices": "110,111",
    "hitter_pingpong_actor_observation_reach_sources":
        "planner_target_tuple,planner_target_tuple",
    "hitter_pingpong_actor_observation_nonprivileged_contract":
        "planner_tuple_no_plant_inference_v3",
    "hitter_pingpong_optional_reach_training_enabled": "true",
    "hitter_pingpong_optional_reach_contract":
        "staged_support_intent_sim_training_candidate_v5",
    "hitter_pingpong_optional_reach_trajectory_qualification":
        "SIM_TRAINING_CANDIDATE",
    "hitter_pingpong_optional_reach_deployment_qualification":
        "SIM_TRAINING_CANDIDATE_NOT_DEPLOYMENT_QUALIFIED",
    "hitter_pingpong_onnx_output_contract": "actions31_v1",
    "hitter_pingpong_question_bank_artifact_contract":
        "fixed_home_support_training_candidate_bank_v5",
    "hitter_pingpong_question_bank_sha256":
        "9a4ff9e89f360e5be25e6ee6df571b30aee93a9f7f88a70ae5a308606e294f7e",
    "hitter_pingpong_question_bank_receipt_sha256":
        "347d16d386ba372feb3c8bc4c00d23542f69de62f14ccd4ffcb8a881f5b4b493",
    "hitter_pingpong_target_tuple_prefilter_contract":
        "fixed_home_support_training_candidate_bank_v5_axis_envelope_"
        "prefilter_not_trajectory_certification_v1",
    "hitter_pingpong_target_tuple_continuous_correlated_qualification":
        "NOT_PROVEN_axis_prefilter_only",
}

SCHEMA32_METADATA_TUPLE = {
    **SCHEMA31_METADATA_TUPLE,
    "hitter_pure_training_recipe":
        "hitter_pingpong_build2_recoverability_envelope_v1",
    "hitter_pingpong_build_contract":
        "build2_recoverability_envelope_schema32",
    "hitter_pingpong_replay_contract":
        "markov_side_phase_severity_v3_recovery_debt_perturbed_v1",
    "hitter_pingpong_transition_contract":
        "policy_owned_112d_markov_slew_safe_executed_feedback_v2",
    "hitter_pingpong_training_initialization_contract":
        "schema32_from_build1_model21800_functional_vxy_zero_append_reach_trainable_v1",
    "qdes_action_contract": "v12_affine_safe_slew_qdes_v1",
    "qdes_policy_feedback_contract": "executed_qdes_raw_v12",
    "hitter_pure_last_action_feedback": "executed_qdes_raw_v12",
    "qdes_slew_tilt_full_scale_rad": "0.0698",
    "qdes_slew_tilt_min_scale_rad": "0.1745",
    "qdes_slew_speed_full_scale_mps": "0.25",
    "qdes_slew_speed_min_scale_mps": "0.5",
    "qdes_slew_min_scale": "0.35",
    "qdes_slew_state_source":
        "actor_visible_projected_gravity_tilt_and_filtered_mocap_speed_v1",
}


def schema29_metadata_tuple_valid(metadata, output_shapes=None, input_shapes=None):
    """Return whether an ONNX carries the exact Schema29 Gate3 ABI tuple.

    The report does not infer Schema29 from a 112-column CSV: Schema27 uses the
    same shape.  Recipe, learning architecture, replay/reward/recovery semantics,
    non-privileged Planner inputs, and the single-output ABI must agree.
    """
    if any(metadata.get(key) != value for key, value in SCHEMA29_METADATA_TUPLE.items()):
        return False
    if any(metadata.get(key) for key in (
        "hitter_pingpong_core_shadow_feedback_contract",
        "hitter_pingpong_core_shadow_action_indices",
        "hitter_pingpong_core_shadow_observation_indices",
        "hitter_pingpong_core_shadow_reset_contract",
    )):
        return False
    if input_shapes is not None and input_shapes.get("obs") != [1, 112]:
        return False
    if output_shapes is not None and (
        output_shapes.get("actions") != [1, 31]
        or "core_action_shadow_owned" in output_shapes
    ):
        return False
    return True


def schema31_metadata_tuple_valid(metadata, output_shapes=None, input_shapes=None):
    """Return whether an ONNX carries the exact Schema31 candidate ABI tuple."""
    if any(metadata.get(key) != value for key, value in SCHEMA31_METADATA_TUPLE.items()):
        return False
    if any(metadata.get(key) for key in (
        "hitter_pingpong_core_shadow_feedback_contract",
        "hitter_pingpong_core_shadow_action_indices",
        "hitter_pingpong_core_shadow_observation_indices",
        "hitter_pingpong_core_shadow_reset_contract",
    )):
        return False
    if input_shapes is not None and input_shapes.get("obs") != [1, 112]:
        return False
    if output_shapes is not None and (
        output_shapes.get("actions") != [1, 31]
        or "core_action_shadow_owned" in output_shapes
    ):
        return False
    return True


SCHEMA33_METADATA_TUPLE = {
    **SCHEMA32_METADATA_TUPLE,
    "hitter_pure_training_recipe":
        "hitter_pingpong_build2_active_replant_recovery_v1",
    "hitter_pingpong_build_contract":
        "build2_active_replant_recovery_schema33",
    "hitter_pingpong_replay_contract":
        "markov_side_phase_severity_v3_recovery_debt_support_geometry_perturbed_v2",
    "hitter_pingpong_reward_contract":
        "immutable_home_terminal_tracking_outer_safety_licensed_replant_v7",
    "hitter_pingpong_recovery_phase_contract":
        "continuous_l0_terminal_licensed_replant_release_v7",
    "hitter_pingpong_training_initialization_contract":
        "schema33_from_schema32_model5580_direct_112d_actor_trainable_v1",
}

SCHEMA34_METADATA_TUPLE = {
    **SCHEMA33_METADATA_TUPLE,
    "hitter_pure_training_recipe":
        "hitter_pingpong_build2_visible_foot_recovery_v1",
    "actor_obs_contract":
        "hitter_pure_112_headslots_vxy_signed_support_v2",
    "hitter_pingpong_build_contract":
        "build2_visible_foot_recovery_schema34",
    "hitter_pingpong_reward_contract":
        "visible_sequential_foot_trajectory_wait_arm_v8",
    "hitter_pingpong_recovery_phase_contract":
        "signed_replant_dense_trajectory_no_latch_income_v8",
    "hitter_pingpong_transition_contract":
        "policy_owned_112d_signed_support_slew_safe_executed_feedback_v3",
    "hitter_pingpong_training_initialization_contract":
        "schema34_from_schema33_model2130_zero_support_columns_trainable_v1",
    "hitter_pingpong_actor_warm_start_receipt": (
        "model_2130.pt,sha256="
        "0df28d798b4bec2af47fbea1e62fcb1371efdc292539f3d08777b63967ac18e3,"
        "source_input_dim=112,destination_input_dim=112,"
        "zeroed_semantic_cols=110|111,input_transform="
        "zero_semantically_reassigned_support_columns_110_111_v1,"
        "trainable_full31=true"
    ),
    "hitter_pingpong_reach_permission_contract":
        "signed_external_reach_and_fk_replant_need_v6",
    "hitter_pingpong_actor_observation_reach_sources":
        "planner_signed_reach_level_foot,fk_home_anchor_signed_need",
    "hitter_pingpong_actor_observation_nonprivileged_contract":
        "planner_plus_fk_home_anchor_no_contact_latch_v4",
}


def schema33_metadata_tuple_valid(metadata, output_shapes=None, input_shapes=None):
    """Return whether an ONNX carries the exact Schema33 licensed-replant V12 ABI tuple."""
    merged = dict(metadata)
    if any(merged.get(key) != value for key, value in SCHEMA33_METADATA_TUPLE.items()):
        return False
    # Everything else (slew tables, shapes) is identical to the Schema32 ABI check.
    for key, value in SCHEMA32_METADATA_TUPLE.items():
        if key not in SCHEMA33_METADATA_TUPLE or SCHEMA33_METADATA_TUPLE[key] == value:
            continue
        merged[key] = value
    return schema32_metadata_tuple_valid(merged, output_shapes, input_shapes)


def schema34_metadata_tuple_valid(metadata, output_shapes=None, input_shapes=None):
    """Return whether an ONNX carries the exact Schema34 signed-support V12 ABI tuple."""
    merged = dict(metadata)
    expected = dict(SCHEMA34_METADATA_TUPLE)
    if str(metadata.get("hitter_pure_training_recipe_version")) == "2":
        expected.update({
            "hitter_pure_training_recipe_version": "2",
            "hitter_pingpong_reward_contract": "support_consistent_com_footwork_v9",
            "hitter_pingpong_recovery_phase_contract": "current_fk_goal_error_across_commit_v9",
            "hitter_pingpong_training_initialization_contract":
                "schema34_from_schema32_model5580_zero_support_columns_trainable_v2",
            "hitter_pingpong_actor_warm_start_receipt": (
                "model_5580.pt,sha256=79423210ecbe3eb7e863ae4bf196eb22fb445d1f6c942796131d4a79e0e311e0,"
                "source_input_dim=112,destination_input_dim=112,zeroed_semantic_cols=110|111,"
                "input_transform=zero_semantically_reassigned_support_columns_110_111_v1,trainable_full31=true"),
        })
    if str(metadata.get("hitter_pure_training_recipe_version")) == "3":
        expected.update({'hitter_pure_training_recipe_version': '3', 'actor_obs_contract': 'hitter_pure_112_headslots_step_task_v3', 'hitter_pingpong_reward_contract': 'observable_step_task_com_footwork_v10', 'hitter_pingpong_recovery_phase_contract': 'persistent_observable_lift_transport_land_v10', 'hitter_pingpong_training_initialization_contract': 'schema34_from_schema34v2_model4710_zero_task_columns_trainable_v3', 'hitter_pingpong_actor_warm_start_receipt': 'model_4710.pt,sha256=c7216a9358452dee1f38a5232a1c08e2beb6cdc2bd4ab785f003b6e3b3207398,source_input_dim=112,destination_input_dim=112,zeroed_semantic_cols=14|19|45|50|111,input_transform=zero_semantically_reassigned_task_columns_14_19_45_50_111_v1,trainable_full31=true', 'hitter_pingpong_reach_permission_contract': 'signed_external_reach_observable_footstep_task_v7', 'hitter_pingpong_actor_observation_reach_sources': 'planner_signed_reach_level_foot,mocap_imu_fk_visible_footstep_task', 'hitter_pingpong_actor_observation_nonprivileged_contract': 'planner_plus_mocap_imu_fk_observable_task_v5'})
    if any(merged.get(key) != value for key, value in expected.items()):
        return False
    # Reuse the shared V12 slew/single-output validation after replacing the Schema34-specific
    # identity and observation fields with their Schema32 structural equivalents.
    for key, value in SCHEMA32_METADATA_TUPLE.items():
        merged[key] = value
    return schema32_metadata_tuple_valid(merged, output_shapes, input_shapes)


def schema32_metadata_tuple_valid(metadata, output_shapes=None, input_shapes=None):
    """Return whether an ONNX carries the exact Schema32 V12 candidate ABI tuple."""
    if any(metadata.get(key) != value for key, value in SCHEMA32_METADATA_TUPLE.items()):
        return False
    try:
        slew = [float(value) for value in metadata["qdes_slew_limit_rad_per_tick"].split(",")]
        leg_mask = [int(value) for value in metadata["qdes_slew_leg_mask"].split(",")]
    except (KeyError, TypeError, ValueError):
        return False
    if (
        len(slew) != 31
        or len(leg_mask) != 31
        or sum(leg_mask) != 12
        or any(value not in (0, 1) for value in leg_mask)
        or any(not math.isfinite(value) or value < 0.0 for value in slew)
    ):
        return False
    expected_slew = [
        0.0 if name.startswith("head_")
        else 0.30 if any(token in name for token in ("_hip_", "_knee_", "_ankle_"))
        else 0.35 if name.startswith("waist_")
        else 0.70
        for name in JOINT_NAMES
    ]
    expected_leg_mask = [
        int(any(token in name for token in ("_hip_", "_knee_", "_ankle_")))
        for name in JOINT_NAMES
    ]
    if any(abs(actual - expected) > 1.0e-9 for actual, expected in zip(slew, expected_slew)):
        return False
    if leg_mask != expected_leg_mask:
        return False
    # The single-output structural checks are intentionally identical to Schema31.
    structural_metadata = dict(metadata)
    for key, value in SCHEMA31_METADATA_TUPLE.items():
        structural_metadata[key] = value
    return schema31_metadata_tuple_valid(
        structural_metadata, output_shapes=output_shapes, input_shapes=input_shapes
    )


def expected_observation_dim(report_mode):
    if report_mode == "rally_v15":
        return 118
    if report_mode == "schema28":
        return 127
    if report_mode in (
        "hitter_pingpong_build2_torso_optional_reach_v2", "schema29", "schema31",
        "schema32",
        "schema33",
        "schema34",
    ):
        return 112
    return 110


def contact_window_indices(tts, start, end, strike_tick, half_width=0.03):
    """Return contact-window ticks, falling back to the closest-to-strike sample.

    Runner CSVs are normally sampled at 50 Hz, but a sparse capture can skip the closed
    ``|tts| <= half_width`` window entirely.  The already-selected minimum-|tts| strike tick is
    the only defensible fallback; returning an empty list would turn the elbow percentile into a
    NaN and falsely fail an otherwise readable swing.
    """
    indices = [tick for tick in range(start, end + 1) if abs(tts[tick]) <= half_width]
    return indices if indices else [strike_tick]


def post_strike_tail_indices(rows, strike_index, next_strike_index, horizon_s):
    """Return a contiguous physical-time tail after one strike.

    The runner resets TTS to its positive WAIT value when the reference clip completes, so TTS
    cannot identify a recovery window longer than the clip's remaining 1.2--1.3 seconds.  Repair
    v3 instead follows the monotonic 50 Hz policy tick from the selected strike sample, stops at
    the next physical strike, and fails closed unless every tick through ``horizon_s`` is present.
    """

    if not 0 <= strike_index < len(rows):
        raise ValueError("strike_index is outside the observation trace")
    if not math.isfinite(horizon_s) or horizon_s <= 0.0:
        raise ValueError("post-strike horizon must be finite and positive")
    stop = len(rows) if next_strike_index is None else int(next_strike_index)
    if stop <= strike_index or stop > len(rows):
        raise ValueError("next_strike_index must follow the current strike")

    strike_tick = int(rows[strike_index]["tick"])
    selected = []
    for index in range(strike_index + 1, stop):
        elapsed_s = (int(rows[index]["tick"]) - strike_tick) * CONTROL_DT
        if elapsed_s > horizon_s + 0.5 * CONTROL_DT:
            break
        selected.append(index)

    expected_steps = int(round(horizon_s / CONTROL_DT))
    expected_ticks = [strike_tick + offset for offset in range(1, expected_steps + 1)]
    observed_ticks = [int(rows[index]["tick"]) for index in selected[:expected_steps]]
    complete = observed_ticks == expected_ticks
    detail = (
        f"observed={len(observed_ticks)}/{expected_steps} contiguous 50-Hz ticks; "
        f"last_age_s={((observed_ticks[-1] - strike_tick) * CONTROL_DT):.2f}"
        if observed_ticks
        else f"observed=0/{expected_steps} contiguous 50-Hz ticks; last_age_s=none"
    )
    return selected, complete, detail


def localization_fresh_outage_trace_stats(path):
    """Measure actor-visible outage bursts from the runner trace itself."""

    if not path:
        return None
    try:
        stream = open(path, newline="", encoding="utf-8")
    except OSError:
        return None
    with stream:
        reader = csv.DictReader(stream)
        required = {
            "tick", "mode", "wall_time_ns", "localization_fresh", "base_speed_valid"
        }
        if not required.issubset(reader.fieldnames or ()):
            return None
        bursts = []
        current = 0
        current_start_ns = None
        current_end_ns = None
        censored = 0
        reacquisitions = 0
        zero_speed_reacquisitions = 0
        motion_ticks = 0
        previous_motion_tick = None
        for row in reader:
            in_motion = str(row.get("mode", "")).strip().upper() in {
                "3", "MOTION"
            }
            if not in_motion:
                if current:
                    censored += 1
                    current = 0
                    current_start_ns = None
                    current_end_ns = None
                previous_motion_tick = None
                continue
            motion_ticks += 1
            try:
                tick = int(row.get("tick", ""))
                wall_time_ns = int(row.get("wall_time_ns", ""))
            except (TypeError, ValueError):
                return None
            # The formal runner writes one row per 50-Hz policy tick.  A sparse CSV must never be
            # allowed to make a short/natural outage impersonate the injected blackout by simply
            # omitting the intervening stale rows.
            if previous_motion_tick is not None and tick != previous_motion_tick + 1:
                return None
            previous_motion_tick = tick
            fresh = str(row.get("localization_fresh", "")).strip().lower()
            if fresh in {"0", "false"}:
                if current == 0:
                    current_start_ns = wall_time_ns
                current += 1
                current_end_ns = wall_time_ns
                continue
            if fresh not in {"1", "true"}:
                return None
            if current:
                speed_valid = str(row.get("base_speed_valid", "")).strip().lower()
                bursts.append({
                    "length": current,
                    "start_wall_time_ns": current_start_ns,
                    "end_wall_time_ns": current_end_ns,
                    "reacquisition_wall_time_ns": wall_time_ns,
                    "first_fresh_zero_speed": speed_valid in {"0", "false"},
                })
                current = 0
                current_start_ns = None
                current_end_ns = None
                reacquisitions += 1
                zero_speed_reacquisitions += int(speed_valid in {"0", "false"})
        if current:
            censored += 1
        lengths = [burst["length"] for burst in bursts]
        return {
            "motion_ticks": motion_ticks,
            "explicit_stale_ticks": sum(lengths),
            "completed_burst_count": len(lengths),
            "completed_burst_lengths": lengths,
            "completed_bursts": bursts,
            "censored_burst_count": censored,
            "reacquisition_count": reacquisitions,
            "first_fresh_zero_speed_count": zero_speed_reacquisitions,
        }


def empirical_outage_evidence(report, runner_trace=None):
    """Validate isolated base-outage stress, never legacy Planner SIGSTOP stress."""

    stats = localization_fresh_outage_trace_stats(runner_trace)
    stress = report.get("base_outage_stress") if isinstance(report, dict) else None
    if stats is None or not isinstance(stress, dict):
        return False, "missing runner-trace or isolated base_outage_stress evidence"
    lengths = stats["completed_burst_lengths"]
    start_ns = stress.get("start_wall_time_ns")
    end_ns = stress.get("end_wall_time_ns")
    stress_ok = (
        stress.get("source") == "calibrated_base_relay_only"
        and isinstance(stress.get("serve"), int) and stress["serve"] > 0
        and isinstance(stress.get("requested_upstream_blackout_ticks"), int)
        and 1 <= stress["requested_upstream_blackout_ticks"] <= 15
        and isinstance(start_ns, int) and isinstance(end_ns, int)
        and start_ns < end_ns
    )
    requested_duration_ns = int(
        stress.get("requested_upstream_blackout_ticks", 0)
    ) * 20_000_000
    duration_ok = bool(
        stress_ok
        and requested_duration_ns - 20_000_000
            <= end_ns - start_ns
            <= requested_duration_ns + 100_000_000
    )
    matching_bursts = []
    if stress_ok:
        matching_bursts = [
            burst for burst in stats["completed_bursts"]
            if start_ns <= burst["start_wall_time_ns"] <= end_ns
            and burst["reacquisition_wall_time_ns"] >= end_ns
        ]
    bound_burst = matching_bursts[0] if len(matching_bursts) == 1 else None
    support_ok = bool(
        bound_burst is not None
        and 1 <= bound_burst["length"] <= 15
    )
    reacquisition_ok = bool(
        bound_burst is not None
        and bound_burst["first_fresh_zero_speed"]
        and end_ns <= bound_burst["reacquisition_wall_time_ns"]
            <= end_ns + 200_000_000
    )
    no_fall = (
        int(report.get("falls", 1)) == 0
        and int(report.get("physical_falls", 1)) == 0
    )
    valid = (
        support_ok and reacquisition_ok and stress_ok and duration_ok and no_fall
    )
    return valid, (
        f"lengths={lengths!r} stale_ticks={stats['explicit_stale_ticks']} "
        f"matching_bursts={matching_bursts!r} duration_ok={duration_ok} "
        f"stress={stress!r} "
        f"falls={report.get('falls')!r}/{report.get('physical_falls')!r}"
    )


def _discover_mode_from_runner(runner_log, runner_cwd):
    """Return the report mode proven by the runner/ONNX, or ``None``.

    A new FinalV3 runner emits an exact recipe marker after it has validated the
    model.  The metadata fallback keeps existing V2 packages auto-detectable: the
    model path comes from the runner itself, never from its filename.
    """
    if not runner_log:
        return None
    try:
        log_text = Path(runner_log).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None
    # Emitted only after the native loader validates the complete policy pair.
    # The ball-clock 110 ABI shares the V14 observation layout and metric
    # windows. This selects report math, not the artifact's training identity.
    abi_modes = {"small_station_112_v1": "small_station112", "small_station_324_v1": "compact324",
                 "ball_clock_110_v1": "rally_v14"}
    abis = set(re.findall(r"^\[pp\] validated_policy_abi=(\S+)$", log_text, re.MULTILINE))
    if abis:
        return abi_modes.get(next(iter(abis))) if len(abis) == 1 else None
    if "[pp] hitter_pure training_recipe=rally_final_v3" in log_text:
        return "rally_final_v3"
    if "[pp] hitter_pure training_recipe=rally_v8" in log_text:
        return "rally_v8"
    if "[pp] hitter_pure training_recipe=rally_v9" in log_text:
        return "rally_v9"
    runtime_v2_proven = "[pp] hitter_pure runtime_contract=rally_final_v2" in log_text
    if "[pp] hitter_pure training_recipe=rally_v10" in log_text:
        return "rally_v10" if runtime_v2_proven else None
    if "[pp] hitter_pure training_recipe=rally_v11" in log_text:
        return "rally_v11" if runtime_v2_proven else None
    if "[pp] hitter_pure training_recipe=rally_v12" in log_text:
        return "rally_v12" if runtime_v2_proven else None
    if "[pp] hitter_pure training_recipe=rally_v13" in log_text:
        return "rally_v13" if runtime_v2_proven else None
    if "[pp] hitter_pure training_recipe=rally_v14" in log_text:
        return "rally_v14" if runtime_v2_proven else None
    if (
        "[pp] hitter_pure training_recipe=hitter_pingpong_recovery_tts_v3"
        in log_text
    ):
        return "hitter_pingpong_recovery_tts_v3" if runtime_v2_proven else None
    if (
        "[pp] hitter_pure training_recipe=hitter_pingpong_recovery_tts_v2"
        in log_text
    ):
        return "hitter_pingpong_recovery_tts_v2" if runtime_v2_proven else None
    if (
        "[pp] hitter_pure training_recipe="
        "hitter_pingpong_build2_minimal_rootfix_v1" in log_text
    ):
        return (
            "hitter_pingpong_build2_minimal_rootfix_v1"
            if runtime_v2_proven else None
        )
    if (
        "[pp] hitter_pure training_recipe="
        "hitter_pingpong_build2_rapid_preempt_v1" in log_text
    ):
        return (
            "hitter_pingpong_build2_rapid_preempt_v1"
            if runtime_v2_proven else None
        )
    if (
        "[schema24-training-screen-gate3] PROFILE ACCEPTED" in log_text
        and "[pp] hitter_pure training_recipe="
        "hitter_pingpong_build2_fixed_home_feasible_v1" in log_text
    ):
        return (
            "hitter_pingpong_build2_fixed_home_feasible_v1"
            if runtime_v2_proven else None
        )
    if (
        "[pp] hitter_pure training_recipe="
        "hitter_pingpong_build2_fixed_home_phase_recovery_v1" in log_text
    ):
        return (
            "hitter_pingpong_build2_fixed_home_phase_recovery_v1"
            if runtime_v2_proven else None
        )
    if (
        "[pp] hitter_pure training_recipe="
        "hitter_pingpong_build2_home_support_frame_v1" in log_text
    ):
        return (
            "hitter_pingpong_build2_home_support_frame_v1"
            if runtime_v2_proven else None
        )
    if (
        "[pp] hitter_pure training_recipe="
        "hitter_pingpong_build2_torso_optional_reach_v2" in log_text
    ):
        return (
            "hitter_pingpong_build2_torso_optional_reach_v2"
            if runtime_v2_proven else None
        )
    if (
        "[pp] hitter_pure training_recipe="
        "hitter_pingpong_build2_frozen_core_optional_reach_v1" in log_text
    ):
        return "schema28" if runtime_v2_proven else None
    if (
        "[pp] hitter_pure training_recipe="
        "hitter_pingpong_build2_trainable_home_optional_reach_v1" in log_text
    ):
        return "schema29" if runtime_v2_proven else None
    if (
        "[pp] hitter_pure training_recipe="
        "hitter_pingpong_build2_stable_home_step_candidate_v1" in log_text
    ):
        return "schema31" if runtime_v2_proven else None
    if (
        "[pp] hitter_pure training_recipe="
        "hitter_pingpong_build2_recoverability_envelope_v1" in log_text
    ):
        return "schema32" if runtime_v2_proven else None
    if (
        "[pp] hitter_pure training_recipe="
        "hitter_pingpong_build2_active_replant_recovery_v1" in log_text
    ):
        return "schema33" if runtime_v2_proven else None
    if (
        "[pp] hitter_pure training_recipe="
        "hitter_pingpong_build2_visible_foot_recovery_v1" in log_text
    ):
        return "schema34" if runtime_v2_proven else None
    if (
        "[pp] hitter_pure training_recipe=hitter_pingpong_continuous_rally_v2"
        in log_text
    ):
        return "hitter_pingpong_continuous_rally_v2" if runtime_v2_proven else None
    if (
        "[pp] hitter_pure training_recipe=hitter_pingpong_continuous_rally_v4"
        in log_text
    ):
        # Build4 keeps the versioned continuous-rally-v2 Gate3/runtime profile;
        # v4 is the training-recipe revision carried by the validated ONNX.
        return "hitter_pingpong_continuous_rally_v2" if runtime_v2_proven else None
    if (
        "[v17-r10-gate3] PROFILE ACCEPTED" in log_text
        and "[pp] hitter_pure training_recipe=rally_v17" in log_text
        and "[pp] hitter_pure runtime_contract="
        "rally_v17_fixed_station_ball_clock_v1" in log_text
    ):
        return "rally_v17_r10"
    if "[pp] hitter_pure training_recipe=rally_v17" in log_text:
        return "rally_v17" if runtime_v2_proven else None
    runtime_v15_proven = "[pp] hitter_pure runtime_contract=rally_v15" in log_text
    if "[pp] hitter_pure training_recipe=rally_v15" in log_text:
        return "rally_v15" if runtime_v15_proven else None

    model_matches = re.findall(
        r"^\[pingpong\] A3AimrtBackend initialised; model=(\S+)(?:\s+.*)?$",
        log_text,
        flags=re.MULTILINE,
    )
    unique_models = list(dict.fromkeys(model_matches))
    if len(unique_models) != 1:
        return None
    model_path = Path(unique_models[0])
    if not model_path.is_absolute():
        if not runner_cwd:
            return None
        model_path = Path(runner_cwd) / model_path
    try:
        import onnx
        model = onnx.load(str(model_path), load_external_data=False)
    # Discovery is advisory for an explicit mode and mandatory for auto.  Any ONNX
    # parser/provider failure therefore becomes ``None`` and is handled fail-closed
    # by ``resolve_mode`` instead of leaking a backend-specific traceback.
    except Exception:
        return None
    metadata = {entry.key: entry.value for entry in model.metadata_props}
    from pp_policy_artifact import small_station_mode
    shape = lambda v: [d.dim_value for d in v.type.tensor_type.shape.dim]
    mode = small_station_mode(metadata, {v.name: shape(v) for v in model.graph.input},
                              {v.name: shape(v) for v in model.graph.output})
    if mode:
        return mode if runtime_v2_proven else None
    recipe = metadata.get("hitter_pure_training_recipe", "").strip()
    version = metadata.get("hitter_pure_training_recipe_version", "").strip()
    known = {
        ("small_station_compact_execution_candidate_v1", "1"): "compact324",
        ("hitter_small_station_independent_recovery_v2", "2"): "small_station112",
        ("hitter_small_station_support_heading_recovery_v3", "3"): "small_station112",
        ("hitter_small_station_matched_plant_recovery_v4", "4"): "small_station112",
        ("hitter_small_station_hold_resume_recovery_v5", "5"): "small_station112",
        ("legacy_station_step", "0"): "legacy",
        ("rally_final_v1", "1"): "legacy",
        ("rally_final_v2", "2"): "legacy",
        ("rally_final_v3", "3"): "rally_final_v3",
        ("rally_v8", "4"): "rally_v8",
        ("rally_v9", "5"): "rally_v9",
        ("rally_v10", "7"): "rally_v10",
        ("rally_v11", "1"): "rally_v11",
        ("rally_v12", "1"): "rally_v12",
        ("rally_v13", "1"): "rally_v13",
        ("rally_v14", "1"): "rally_v14",
        ("hitter_pingpong_recovery_tts_v2", "2"):
            "hitter_pingpong_recovery_tts_v2",
        ("hitter_pingpong_recovery_tts_v3", "3"):
            "hitter_pingpong_recovery_tts_v3",
        ("hitter_pingpong_continuous_rally_v2", "2"):
            "hitter_pingpong_continuous_rally_v2",
        ("hitter_pingpong_build2_minimal_rootfix_v1", "1"):
            "hitter_pingpong_build2_minimal_rootfix_v1",
        ("hitter_pingpong_build2_rapid_preempt_v1", "1"):
            "hitter_pingpong_build2_rapid_preempt_v1",
        ("hitter_pingpong_build2_fixed_home_feasible_v1", "1"):
            "hitter_pingpong_build2_fixed_home_feasible_v1",
        ("hitter_pingpong_build2_fixed_home_phase_recovery_v1", "1"):
            "hitter_pingpong_build2_fixed_home_phase_recovery_v1",
        ("hitter_pingpong_build2_home_support_frame_v1", "1"):
            "hitter_pingpong_build2_home_support_frame_v1",
        ("hitter_pingpong_build2_torso_optional_reach_v2", "2"):
            "hitter_pingpong_build2_torso_optional_reach_v2",
        ("hitter_pingpong_build2_frozen_core_optional_reach_v1", "1"):
            "schema28",
        ("hitter_pingpong_build2_trainable_home_optional_reach_v1", "1"):
            "schema29",
        ("hitter_pingpong_build2_stable_home_step_candidate_v1", "1"):
            "schema31",
        ("hitter_pingpong_build2_recoverability_envelope_v1", "1"):
            "schema32",
        ("hitter_pingpong_build2_active_replant_recovery_v1", "1"):
            "schema33",
        ("hitter_pingpong_build2_visible_foot_recovery_v1", "1"):
            "schema34",
        ("hitter_pingpong_build2_visible_foot_recovery_v1", "2"):
            "schema34",
        ("hitter_pingpong_continuous_rally_v4", "4"):
            "hitter_pingpong_continuous_rally_v2",
        ("rally_v17", "4"): "rally_v17",
        ("rally_v15", "3"): "rally_v15",
        ("rally_v15", "4"): "rally_v15",
        # "5" was the retired v4 full-span decode (no checkpoint survives — NOT accepted);
        # "6" is the shipping v5 default-anchored decode.
        ("rally_v15", "6"): "rally_v15",
    }
    # Do not silently classify a future recipe as legacy: its phase windows may
    # differ just as V3 differs from V2.
    discovered = known.get((recipe, version))
    if discovered == "small_station112":
        shape = lambda value: [d.dim_value for d in value.type.tensor_type.shape.dim]
        inputs = {v.name: shape(v) for v in model.graph.input}
        outputs = {v.name: shape(v) for v in model.graph.output}
        if (not runtime_v2_proven or inputs.get("obs") != [1, 112] or outputs.get("actions") != [1, 31]
                or metadata.get("actor_obs_contract") != "hitter_pure_112_headslots_vxy_reach_v1"
                or metadata.get("hitter_pingpong_command_contract") != "small_station_external_schedule_precommit_settle_v1"
                or metadata.get("hitter_pingpong_optional_reach_training_enabled") != "false"):
            return None
        return discovered
    if discovered == "compact324":
        shape = lambda value: [d.dim_value for d in value.type.tensor_type.shape.dim]
        inputs = {v.name: shape(v) for v in model.graph.input}
        outputs = {v.name: shape(v) for v in model.graph.output}
        if (not runtime_v2_proven or inputs.get("obs") != [1, 324] or outputs.get("actions") != [1, 31]
                or metadata.get("actor_obs_contract") != "hitter_compact_execution_324_v1"
                or metadata.get("compact_history_contract") != "current114_past3x70_policy_tick_repeat_reset_v1"):
            return None
        return discovered
    if discovered in (
        "rally_v10", "rally_v11", "rally_v12", "rally_v13", "rally_v14",
        "hitter_pingpong_recovery_tts_v2", "hitter_pingpong_recovery_tts_v3",
        "hitter_pingpong_continuous_rally_v2",
        "hitter_pingpong_build2_minimal_rootfix_v1",
        "hitter_pingpong_build2_rapid_preempt_v1",
        "hitter_pingpong_build2_fixed_home_recovery_v1",
        "hitter_pingpong_build2_fixed_home_feasible_v1",
        "hitter_pingpong_build2_fixed_home_phase_recovery_v1",
        "hitter_pingpong_build2_home_support_frame_v1",
        "hitter_pingpong_build2_torso_optional_reach_v2",
        "schema28",
        "schema29",
        "schema31",
        "schema32",
        "schema33",
        "schema34",
        "rally_v17",
    ):
        # An ONNX recipe label alone cannot prove that the running binary implements component
        # velocity gating. Only the v2-capable loader emits this marker after validation.
        if metadata.get("hitter_pure_runtime_contract") != "rally_final_v2":
            return None
        if not runtime_v2_proven:
            return None
        if discovered in (
            "rally_v11", "rally_v12", "rally_v13", "rally_v14",
            "hitter_pingpong_recovery_tts_v2", "hitter_pingpong_recovery_tts_v3",
            "hitter_pingpong_continuous_rally_v2",
            "hitter_pingpong_build2_minimal_rootfix_v1",
            "hitter_pingpong_build2_rapid_preempt_v1",
            "hitter_pingpong_build2_fixed_home_phase_recovery_v1",
            "hitter_pingpong_build2_home_support_frame_v1",
            "hitter_pingpong_build2_torso_optional_reach_v2",
            "schema28",
            "schema29",
            "schema31",
            "schema32",
            "schema33",
            "schema34",
            "rally_v17"
        ) and metadata.get("hitter_pure_deployment_status") != \
                "gate3_candidate":
            return None
        if (
            discovered == "hitter_pingpong_build2_fixed_home_feasible_v1"
            and (
                metadata.get("hitter_pure_deployment_status")
                != "gate3_screen_only"
                or metadata.get("hitter_pure_qualification_status")
                != "training_screened_not_deployable"
                or metadata.get("a3_deploy_hardware_authorized") != "false"
            )
        ):
            return None
        if (
            discovered == "hitter_pingpong_build2_fixed_home_phase_recovery_v1"
            and (
                metadata.get("hitter_pingpong_build_contract")
                != "build2_fixed_home_phase_recovery_schema25"
                or metadata.get("hitter_pingpong_replay_contract")
                != "markov_fixed_home_side_phase_severity_v5"
            )
        ):
            return None
        if (
            discovered == "hitter_pingpong_build2_home_support_frame_v1"
            and (
                metadata.get("hitter_pingpong_build_contract")
                != "build2_home_support_frame_schema26"
                or metadata.get("hitter_pingpong_replay_contract")
                != "markov_fixed_home_support_side_phase_severity_v6"
                or metadata.get("actor_obs_contract")
                != "hitter_pure_110_headslots_vxy_v1"
            )
        ):
            return None
        if (
            discovered == "hitter_pingpong_build2_torso_optional_reach_v2"
            and (
                metadata.get("hitter_pingpong_build_contract")
                != "build2_torso_optional_reach_schema27_v2"
                or metadata.get("hitter_pingpong_replay_contract")
                != "markov_fixed_home_support_reach_side_phase_severity_v7"
                or metadata.get("actor_obs_contract")
                != "hitter_pure_112_headslots_vxy_reach_v1"
                or metadata.get("hitter_pingpong_reach_permission_contract")
                != "staged_planner_support_intent_no_plant_inference_v4"
                or metadata.get(
                    "hitter_pingpong_actor_observation_nonprivileged_contract"
                ) != "planner_tuple_no_contact_force_or_plant_inference_v1"
                or metadata.get("hitter_pingpong_optional_reach_training_enabled")
                != "true"
                or metadata.get("hitter_pingpong_optional_reach_contract")
                != "staged_support_intent_training_v3"
            )
        ):
            return None
        if (
            discovered == "schema28"
            and (
                metadata.get("hitter_pingpong_build_contract")
                != "build2_frozen_core_shadow_schema28"
                or metadata.get("hitter_pingpong_replay_contract")
                != "markov_fixed_home_support_reach_side_phase_severity_core_shadow_v8"
                or metadata.get("actor_obs_contract")
                != "hitter_pure_127_headslots_vxy_reach_core_shadow_v1"
                or metadata.get("hitter_pingpong_core_shadow_feedback_contract")
                != "core_owned15_actions_0_1_2_19_30_append112_126_v1"
                or metadata.get("hitter_pingpong_core_shadow_action_indices")
                != "0,1,2,19,20,21,22,23,24,25,26,27,28,29,30"
                or metadata.get("hitter_pingpong_onnx_output_contract")
                != "actions31_core_action_shadow_owned15_v1"
                or metadata.get("hitter_pingpong_reach_permission_contract")
                != "staged_planner_support_intent_no_plant_inference_v4"
                or metadata.get(
                    "hitter_pingpong_actor_observation_nonprivileged_contract"
                ) != "planner_tuple_plus_policy_shadow_no_plant_inference_v2"
                or metadata.get("hitter_pingpong_optional_reach_training_enabled")
                != "true"
                or metadata.get("hitter_pingpong_optional_reach_contract")
                != "staged_support_intent_training_v3"
            )
        ):
            return None
        if discovered == "schema29":
            input_shapes = {
                input_value.name: [
                    dim.dim_value
                    for dim in input_value.type.tensor_type.shape.dim
                ]
                for input_value in model.graph.input
            }
            output_shapes = {
                output.name: [
                    dim.dim_value for dim in output.type.tensor_type.shape.dim
                ]
                for output in model.graph.output
            }
            if not schema29_metadata_tuple_valid(
                metadata, output_shapes, input_shapes
            ):
                return None
        if discovered == "schema31":
            input_shapes = {
                input_value.name: [
                    dim.dim_value
                    for dim in input_value.type.tensor_type.shape.dim
                ]
                for input_value in model.graph.input
            }
            output_shapes = {
                output.name: [
                    dim.dim_value for dim in output.type.tensor_type.shape.dim
                ]
                for output in model.graph.output
            }
            if not schema31_metadata_tuple_valid(
                metadata, output_shapes, input_shapes
            ):
                return None
        if discovered == "schema32":
            input_shapes = {
                input_value.name: [
                    dim.dim_value
                    for dim in input_value.type.tensor_type.shape.dim
                ]
                for input_value in model.graph.input
            }
            output_shapes = {
                output.name: [
                    dim.dim_value for dim in output.type.tensor_type.shape.dim
                ]
                for output in model.graph.output
            }
            if not schema32_metadata_tuple_valid(
                metadata, output_shapes, input_shapes
            ):
                return None
        if discovered == "schema33":
            input_shapes = {
                input_value.name: [
                    dim.dim_value
                    for dim in input_value.type.tensor_type.shape.dim
                ]
                for input_value in model.graph.input
            }
            output_shapes = {
                output.name: [
                    dim.dim_value for dim in output.type.tensor_type.shape.dim
                ]
                for output in model.graph.output
            }
            if not schema33_metadata_tuple_valid(
                metadata, output_shapes, input_shapes
            ):
                return None
        if discovered == "schema34":
            input_shapes = {
                input_value.name: [
                    dim.dim_value
                    for dim in input_value.type.tensor_type.shape.dim
                ]
                for input_value in model.graph.input
            }
            output_shapes = {
                output.name: [
                    dim.dim_value for dim in output.type.tensor_type.shape.dim
                ]
                for output in model.graph.output
            }
            if not schema34_metadata_tuple_valid(
                metadata, output_shapes, input_shapes
            ):
                return None
        if (
            discovered == "hitter_pingpong_build2_minimal_rootfix_v1"
            and metadata.get("hitter_pingpong_build_contract")
            != "build2_minimal_rootfix_schema21"
        ):
            return None
        if (
            discovered == "hitter_pingpong_build2_rapid_preempt_v1"
            and metadata.get("hitter_pingpong_build_contract")
            != "build2_rapid_preempt_schema22"
        ):
            return None
        if (
            discovered == "hitter_pingpong_build2_fixed_home_feasible_v1"
            and metadata.get("hitter_pingpong_build_contract")
            != "build2_fixed_home_feasible_schema24"
        ):
            return None
    if discovered == "rally_v15":
        if metadata.get("hitter_pure_runtime_contract") != "rally_v15":
            return None
        if not runtime_v15_proven:
            return None
        if metadata.get("hitter_pure_deployment_status") != "gate3_candidate":
            return None
    return discovered


def discover_planes_from_runner(runner_log):
    """Per-SIDE trained plane x from the runner's metadata-geometry banner, or ``None``.

    The runner prints the per-clip target centers it resolved from the loaded ONNX
    pos boxes ("[pp] 110 hitter_pure: target centers from ONNX boxes: fh pos=(...)
    bh pos=(...)").  The reach-x expectation must follow the MODEL, not the report
    generation — hardcoded pairs went stale every clip generation (v11 0.64/0.64,
    v12 0.69/0.69, v13 0.65/0.50 PER-SIDE), which is exactly the "report reach
    expectation" deploy TODO from the v13 facefix migration.
    """
    if not runner_log:
        return None
    try:
        log_text = Path(runner_log).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None
    matches = re.findall(
        r"target centers from ONNX boxes:\s*"
        r"fh pos=\(([+-]?[0-9.]+),[^)]*\).*?bh pos=\(([+-]?[0-9.]+),",
        log_text,
    )
    if not matches:
        return None
    fh, bh = matches[-1]
    return {"fh": float(fh), "bh": float(bh)}


def discover_finite_gait_from_runner(runner_log):
    """Return the V15 gait envelope proven by the validated runner banner.

    The runner emits this banner only after the ONNX loader has checked the metadata stamped
    from the task YAML.  Reading it here keeps Gate3 tied to that contract instead of copying
    another velocity limit into the report script.
    """
    if not runner_log:
        return None
    try:
        log_text = Path(runner_log).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None
    matches = re.findall(
        r"\[pp\] V15 finite gait from ONNX/YAML: "
        r"freq=([0-9.]+) Hz duty=([0-9.]+) deadband=([0-9.]+) m "
        r"step=([0-9.]+) m cycles<=([0-9]+) \|vy\|<=([0-9.]+) m/s; "
        r"intervention deploy value=([+-]?[0-9.]+)",
        log_text,
    )
    unique = list(dict.fromkeys(matches))
    if len(unique) != 1:
        return None
    frequency, duty, deadband, step, cycles, velocity_max, intervention = unique[0]
    contract = {
        "frequency_hz": float(frequency),
        "duty_factor": float(duty),
        "move_deadband": float(deadband),
        "step_distance": float(step),
        "max_cycles": int(cycles),
        "velocity_max": float(velocity_max),
        "deploy_intervention": float(intervention),
    }
    if not (
        contract["frequency_hz"] > 0.0
        and 0.0 < contract["duty_factor"] < 1.0
        and contract["move_deadband"] >= 0.0
        and contract["step_distance"] > 0.0
        and contract["max_cycles"] >= 1
        and contract["velocity_max"] > 0.0
        and contract["deploy_intervention"] == 0.0
    ):
        return None
    return contract


def discover_joint_defaults_from_runner(runner_log, runner_cwd=None):
    """Absolute joint defaults proved by the runner banner or its exact loaded ONNX."""
    if not runner_log:
        return None
    try:
        log_text = Path(runner_log).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None
    model_matches = re.findall(
        r"^\[pingpong\] A3AimrtBackend initialised; model=(\S+)(?:\s+.*)?$",
        log_text,
        flags=re.MULTILINE,
    )
    unique_models = list(dict.fromkeys(model_matches))
    if len(unique_models) != 1:
        return None
    model_path = Path(unique_models[0])
    if not model_path.is_absolute():
        if not runner_cwd:
            model_path = None
        else:
            model_path = Path(runner_cwd) / model_path
    try:
        import onnx
        if model_path is None:
            raise FileNotFoundError("relative ONNX path has no runner cwd")
        model = onnx.load(str(model_path), load_external_data=False)
        metadata = {entry.key: entry.value for entry in model.metadata_props}
        names = metadata["joint_names"].split(",")
        defaults = [float(value) for value in metadata["default_joint_pos"].split(",")]
    except Exception:
        pass
    else:
        if len(names) != len(defaults) or set(names) != set(JOINT_NAMES):
            return None
        return dict(zip(names, defaults))

    # The formal Gate3 container intentionally does not depend on Python ONNX.
    # Its runner banner also binds the exact Unitree-style deploy YAML.  That
    # file stores defaults in actor order and joint_ids_map maps each actor
    # entry to joint_sdk_names, so it is an equivalent complete receipt.
    deploy_matches = re.findall(
        r"^\[pingpong\] A3AimrtBackend initialised; .*\bdeploy_cfg=(\S+)(?:\s+.*)?$",
        log_text,
        flags=re.MULTILINE,
    )
    unique_deploys = list(dict.fromkeys(deploy_matches))
    if len(unique_deploys) == 1:
        deploy_path = Path(unique_deploys[0])
        if not deploy_path.is_absolute():
            deploy_path = Path(runner_cwd) / deploy_path if runner_cwd else None
        try:
            import yaml
            if deploy_path is None:
                raise FileNotFoundError("relative deploy path has no runner cwd")
            deploy = yaml.safe_load(deploy_path.read_text(encoding="utf-8"))
            sdk_names = list(deploy["joint_sdk_names"])
            joint_ids = [int(value) for value in deploy["joint_ids_map"]]
            defaults = [float(value) for value in deploy["default_joint_pos"]]
            names = [sdk_names[index] for index in joint_ids]
        except Exception:
            pass
        else:
            if (
                len(names) != len(defaults)
                or len(names) != len(JOINT_NAMES)
                or set(names) != set(JOINT_NAMES)
                or sorted(joint_ids) != list(range(len(JOINT_NAMES)))
            ):
                return None
            return dict(zip(names, defaults))

    # Older models only print the elbow default needed by their report mode.
    # Prefer the complete loaded-ONNX receipt above: Build4 also scores the
    # synchronized 17-DoF WAIT pose and cannot use this one-joint fallback.
    elbow_matches = re.findall(
        r"^\[pp\] hitter_pure joint_default right_elbow_joint=([+-]?[0-9.]+)\s*$",
        log_text,
        flags=re.MULTILINE,
    )
    unique_elbows = list(dict.fromkeys(elbow_matches))
    if len(unique_elbows) == 1:
        return {"right_elbow_joint": float(unique_elbows[0])}
    return None


def runner_clamp_stats(runner_log):
    """Read final q_des safe/hard-limit telemetry instead of raw-action magnitude."""
    if not runner_log:
        return None
    try:
        log_text = Path(runner_log).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None
    samples = [
        int(value) for value in re.findall(r"\[status\].*?\bclamp=(\d+)", log_text)
    ]
    safe_samples = [
        int(value) for value in re.findall(r"\[status\].*?\bsafe=(\d+)", log_text)
    ]
    audit_only_samples = [
        int(value)
        for value in re.findall(r"\[status\].*?\bqdes_audit_only=(\d+)", log_text)
    ]
    warning_count = log_text.count("[pp WARN] q_des clamped to joint limits")
    warned = warning_count > 0
    if not samples and not warned:
        return None
    joint_summary = {}
    audit_lines = re.findall(r"^\[clamp-audit\].*$", log_text, flags=re.MULTILINE)
    if audit_lines:
        for name, hits, ticks, max_viol in re.findall(
            r"\b([A-Za-z0-9_]+_joint)=(\d+)/(\d+)/([0-9.eE+-]+)", audit_lines[-1]
        ):
            joint_summary[name] = {
                "hits": int(hits),
                "ticks": int(ticks),
                "max_viol": float(max_viol),
            }
    return {
        "samples": len(samples),
        "nonzero_samples": sum(value > 0 for value in samples),
        "peak": max(samples, default=0),
        "safe_samples": len(safe_samples),
        "safe_nonzero_samples": sum(value > 0 for value in safe_samples),
        "safe_peak": max(safe_samples, default=0),
        "audit_only": bool(audit_only_samples and all(audit_only_samples)),
        "warned": warned,
        "warning_count": warning_count,
        "joints": joint_summary,
    }


def qdes_projector_trace_stats(trace_path):
    """Read per-policy-tick V15 projector telemetry from the final command trace."""
    if not trace_path:
        return None
    try:
        with open(trace_path, newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            required = {
                "mode", "qdes_projector_active", "qdes_projector_rate",
                "qdes_projector_tracking", "qdes_projector_torque",
                "qdes_projector_infeasible", "qdes_projector_max_norm_debt",
            }
            if reader.fieldnames is None or not required.issubset(reader.fieldnames):
                return {"error": "required qdes projector columns are missing"}
            motion = [row for row in reader if row.get("mode") == "3"]
    except (OSError, csv.Error) as exc:
        return {"error": str(exc)}
    if not motion:
        return {"error": "no MOTION rows in runner trace"}
    try:
        counts = {
            name: [int(row[f"qdes_projector_{name}"]) for row in motion]
            for name in ("active", "rate", "tracking", "torque", "infeasible")
        }
        debts = [float(row["qdes_projector_max_norm_debt"]) for row in motion]
    except (KeyError, TypeError, ValueError) as exc:
        return {"error": f"malformed projector telemetry: {exc}"}
    if any(value < 0 or value > 31 for values in counts.values() for value in values) or any(
            not math.isfinite(value) or value < 0.0 for value in debts):
        return {"error": "out-of-range/non-finite projector telemetry"}
    n = len(motion)
    return {
        "rows": n,
        "joint_fractions": {
            name: sum(values) / (31.0 * n) for name, values in counts.items()
        },
        "affected_tick_fractions": {
            name: sum(value > 0 for value in values) / n for name, values in counts.items()
        },
        "infeasible_peak": max(counts["infeasible"], default=0),
        "infeasible_ticks": sum(value > 0 for value in counts["infeasible"]),
        "max_norm_debt": max(debts, default=0.0),
    }


def resolve_mode(requested, runner_log=None, runner_cwd=None):
    """Resolve/verify the metric recipe; auto mode is intentionally fail-closed."""
    discovered = _discover_mode_from_runner(runner_log, runner_cwd)
    if requested == "auto":
        if discovered is None:
            raise ValueError(
                "cannot prove report recipe from runner log/loaded ONNX; set "
                "--mode legacy, rally_final_v3, rally_v8, rally_v9, rally_v10, rally_v11, "
                "rally_v12, rally_v13, rally_v14, hitter_pingpong_recovery_tts_v2, "
                "hitter_pingpong_recovery_tts_v3, hitter_pingpong_continuous_rally_v2, "
                "hitter_pingpong_build2_minimal_rootfix_v1, "
                "hitter_pingpong_build2_rapid_preempt_v1, "
                "hitter_pingpong_build2_fixed_home_feasible_v1, "
                "hitter_pingpong_build2_fixed_home_phase_recovery_v1, "
                "hitter_pingpong_build2_home_support_frame_v1, "
                "hitter_pingpong_build2_torso_optional_reach_v2, "
                "schema28, schema29, schema31, schema32, schema33, schema34, "
                "rally_v15, rally_v17 or rally_v17_r10 explicitly"
            )
        return discovered
    if requested in (
        "rally_v10", "rally_v11", "rally_v12", "rally_v13", "rally_v14",
        "hitter_pingpong_recovery_tts_v2", "hitter_pingpong_recovery_tts_v3",
        "hitter_pingpong_continuous_rally_v2",
        "hitter_pingpong_build2_minimal_rootfix_v1",
        "hitter_pingpong_build2_rapid_preempt_v1",
        "hitter_pingpong_build2_fixed_home_feasible_v1",
        "hitter_pingpong_build2_fixed_home_phase_recovery_v1",
        "hitter_pingpong_build2_home_support_frame_v1",
        "hitter_pingpong_build2_torso_optional_reach_v2",
        "schema28",
        "schema29",
        "schema31",
        "schema32",
        "schema33",
        "schema34",
        "rally_v17",
    ) and discovered != requested:
        raise ValueError(
            f"{requested} report requires a runner-validated rally_final_v2 capability marker"
        )
    if requested == "rally_v17_r10" and discovered != requested:
        raise ValueError(
            "rally_v17_r10 report requires the isolated runner-validated "
            "fixed-station ball-clock runtime marker"
        )
    if requested == "rally_v15" and discovered != requested:
        raise ValueError(
            "rally_v15 report requires a runner-validated paired rally_v15 runtime marker"
        )
    if discovered is not None and discovered != requested:
        raise ValueError(
            f"requested report mode {requested!r} contradicts loaded model mode "
            f"{discovered!r}"
        )
    return requested


def engaged_sides_from_report(report):
    """Extract runner-selected sides in chronological engage order.

    The actor observation intentionally has no side channel.  FinalV3 velocity
    boxes may cross ``vy=0``, so the target velocity is not a side oracle.  The
    conductor's runner engage event is the authoritative deploy-side decision.
    """
    errors = []
    rows = report.get("rows")
    if not isinstance(rows, list):
        return [], ["conductor report has no rows list"]
    sides = []
    previous_serve = None
    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            errors.append(f"row {index} is not an object")
            continue
        serve = row.get("serve")
        if not isinstance(serve, int):
            errors.append(f"row {index} has invalid serve index {serve!r}")
        elif previous_serve is not None and serve <= previous_serve:
            errors.append("conductor rows are not in strictly increasing serve order")
        if isinstance(serve, int):
            previous_serve = serve
        engages = row.get("engages")
        if engages is None:  # schema v1 compatibility
            engaged = row.get("engaged")
            engages = [] if engaged is None else [engaged]
        if not isinstance(engages, list):
            errors.append(f"serve {serve!r} engages is not a list")
            continue
        for engaged in engages:
            if not isinstance(engaged, dict):
                errors.append(f"serve {serve!r} engaged event is not an object")
                continue
            side = engaged.get("side")
            if side not in ("forehand", "backhand"):
                errors.append(f"serve {serve!r} has invalid engaged side {side!r}")
                continue
            sides.append("fh" if side == "forehand" else "bh")
    expected = report.get("total_engage_events", report.get("engaged"))
    if not isinstance(expected, int) or expected != len(sides):
        errors.append(
            f"summary engage events={expected!r} but rows contain {len(sides)} valid sides"
        )
    return sides, errors


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("obs_csv", nargs="?", default="/tmp/pp_obs.csv")
    parser.add_argument("conductor_json", nargs="?")
    parser.add_argument(
        "--mode", choices=("small_station112", "compact324", "legacy", "rally_final_v3", "rally_v8", "rally_v9", "rally_v10", "rally_v11", "rally_v12", "rally_v13", "rally_v14", "hitter_pingpong_recovery_tts_v2", "hitter_pingpong_recovery_tts_v3", "hitter_pingpong_continuous_rally_v2", "hitter_pingpong_build2_minimal_rootfix_v1", "hitter_pingpong_build2_rapid_preempt_v1", "hitter_pingpong_build2_fixed_home_feasible_v1", "hitter_pingpong_build2_fixed_home_phase_recovery_v1", "hitter_pingpong_build2_home_support_frame_v1", "hitter_pingpong_build2_torso_optional_reach_v2", "schema28", "schema29", "schema31", "schema32", "schema33", "schema34",
 "rally_v15", "rally_v17", "rally_v17_r10", "auto"), default="auto",
        help="metric windows; auto proves the recipe from --runner-log/loaded ONNX. Default is "
             "AUTO and fail-closed (2026-07-12 audit): the old 'legacy' default silently scored a "
             "rally_v8/v11 trace with the legacy reach 0.51 instead of 0.64 on any bare invocation "
             "without --mode. Pass --runner-log so auto can prove the recipe.",
    )
    parser.add_argument("--runner-log", help="runner log used for fail-closed auto detection")
    parser.add_argument(
        "--runner-cwd", help="working directory used to resolve a relative ONNX path from the log"
    )
    parser.add_argument(
        "--runner-trace", default="/tmp/pp_runner_trace.csv",
        help="runner command trace; mandatory projector evidence for rally_v15",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    path = args.obs_csv
    try:
        report_mode = resolve_mode(args.mode, args.runner_log, args.runner_cwd)
    except ValueError as exc:
        print(f"FAIL: {exc}")
        return 2
    if report_mode in ("compact324", "small_station112"):
        from pp_compact_observation_report import report
        return report(args.obs_csv, args.conductor_json, 324 if report_mode == "compact324" else 112)
    optional_reach_report = report_mode in (
        "hitter_pingpong_build2_torso_optional_reach_v2",
        "schema28",
        "schema29",
        "schema31",
        "schema32",
        "schema33",
        "schema34",
    )
    optional_reach_schema_label = {
        "schema28": "Schema28",
        "schema29": "Schema29",
        "schema31": "Schema31",
        "schema32": "Schema32",
        "schema33": "Schema33",
        "schema34": "Schema34",
    }.get(report_mode, "Schema27")
    onnx_planes = discover_planes_from_runner(args.runner_log)
    joint_defaults = discover_joint_defaults_from_runner(args.runner_log, args.runner_cwd)
    finite_gait = discover_finite_gait_from_runner(args.runner_log)
    if onnx_planes is not None:
        print(f"reach-x expectation from the loaded ONNX (runner banner): "
              f"fh {onnx_planes['fh']:.2f} / bh {onnx_planes['bh']:.2f}")
    conductor_report = None
    if args.conductor_json:
        try:
            with open(args.conductor_json) as f:
                conductor_report = json.load(f)
        except (OSError, json.JSONDecodeError) as exc:
            print(f"FAIL: cannot read conductor report {args.conductor_json}: {exc}")
            return 2
    home_preempt_report = bool(
        report_mode == "hitter_pingpong_recovery_tts_v2"
        and isinstance(conductor_report, dict)
        and conductor_report.get("recovery_contract")
        == "session_anchor_kinetic_dwell_v1"
    )
    post_contact_age_mode = bool(
        report_mode in (
            "hitter_pingpong_recovery_tts_v3",
            "hitter_pingpong_continuous_rally_v2",
        ) or home_preempt_report
    )
    windows = dict(WINDOWS[report_mode])
    if home_preempt_report:
        windows["post"] = (0.10, 3.00)
        windows["post_heading"] = (0.20, 3.00)
    rows = []
    expected_n_obs = expected_observation_dim(report_mode)
    with open(path) as f:
        rd = csv.reader(f)
        hdr = next(rd)
        n_obs = sum(1 for c in hdr if c.startswith("obs_"))
        if n_obs != expected_n_obs:
            print(f"FAIL: obs CSV has {n_obs} obs columns, not the {expected_n_obs}-D contract")
            sys.exit(1)
        o0 = hdr.index("obs_0")
        for r in rd:
            if len(r) < o0 + expected_n_obs:
                continue
            rows.append({
                "tick": int(r[0]), "ts": float(r[1]), "mode": r[2],
                "sync_miss": int(r[hdr.index("sync_miss")]),
                "o": [float(v) for v in r[o0:o0 + expected_n_obs]],
            })
    if not rows:
        print("FAIL: empty obs CSV")
        sys.exit(1)

    def seg(o, k):
        lo, hi = B[k]
        return o[lo:hi]

    # ---- global health ----
    bad = ok = 0
    checks = []
    qdes_audit_only_checks = []
    if report_mode == "hitter_pingpong_recovery_tts_v3" or home_preempt_report:
        outage_ok, outage_detail = empirical_outage_evidence(
            conductor_report, args.runner_trace
        )
        checks.append((
            "receipt-support localization-fresh outage exercised with no fall",
            outage_ok,
            outage_detail,
        ))
    nan_ticks = sum(1 for r in rows if any(math.isnan(v) or math.isinf(v) for v in r["o"]))
    checks.append(("no NaN/Inf in obs", nan_ticks == 0, f"{nan_ticks} ticks affected"))
    if report_mode == "schema28":
        shadow_rows_nonzero = sum(
            any(abs(value) > 1.0e-8 for value in row["o"][112:127])
            for row in rows
        )
        checks.append((
            "Schema28 trace carries nonzero frozen-core action shadow feedback",
            shadow_rows_nonzero > 0,
            f"nonzero_shadow_rows={shadow_rows_nonzero}/{len(rows)}; indices=112:127",
        ))
    if optional_reach_report and report_mode == "schema34":
        signed_external = [row["o"][B["reach_level"]] for row in rows]
        signed_need = [row["o"][B["swing_foot_sign"]] for row in rows]
        legal_external = all(
            any(abs(value - allowed) <= 1.0e-6 for allowed in (-2.0, -1.0, 0.0, 1.0, 2.0))
            for value in signed_external
        )
        legal_need = all(
            math.isfinite(value) and -1.0 - 1.0e-6 <= value <= 1.0 + 1.0e-6
            for value in signed_need
        )
        observed_external = sorted({int(round(value)) for value in signed_external})
        nonzero_external = sum(abs(value) > 1.0e-6 for value in signed_external)
        nonzero_need = sum(abs(value) > 1.0e-6 for value in signed_need)
        checks.append((
            "Schema34 signed external reach values are legal",
            legal_external,
            f"observed={observed_external}; nonzero_rows={nonzero_external}/{len(rows)}",
        ))
        checks.append((
            "Schema34 independent FK-to-HOME replant need is finite and bounded",
            legal_need,
            f"range=[{min(signed_need):.4f},{max(signed_need):.4f}]; "
            f"nonzero_rows={nonzero_need}/{len(rows)}",
        ))
    elif optional_reach_report:
        permission_pairs = [
            (row["o"][B["reach_level"]], row["o"][B["swing_foot_sign"]])
            for row in rows
        ]
        legal_permissions = all(
            (level == 0.0 and foot == 0.0)
            or (level in (1.0, 2.0) and foot in (-1.0, 1.0))
            for level, foot in permission_pairs
        )
        nonzero_permissions = sum(level > 0.0 for level, _foot in permission_pairs)
        observed_levels = sorted({int(round(level)) for level, _foot in permission_pairs})
        checks.append((
            f"{optional_reach_schema_label} Planner reach permission pairs are legal",
            legal_permissions,
            f"observed_pairs={sorted(set(permission_pairs))}",
        ))
        if report_mode == "schema28":
            checks.append((
                f"{optional_reach_schema_label} formal trace retains ordinary L0 and exercises L1/L2",
                observed_levels == [0, 1, 2],
                f"observed_levels={observed_levels}",
            ))
        elif report_mode == "schema29":
            checks.append((
                "Schema29 candidate-only v4 keeps every runtime permission at L0",
                observed_levels == [0] and nonzero_permissions == 0,
                f"observed_levels={observed_levels}; nonzero_rows={nonzero_permissions}/{len(rows)}",
            ))
        elif report_mode in ("schema31", "schema32", "schema33"):
            schema31_valid_levels = (
                observed_levels == [0]
                if nonzero_permissions == 0
                else observed_levels == [0, 1, 2]
            )
            checks.append((
                f"{optional_reach_schema_label} is either default L0 or an exercised x86 L1/L2 lane",
                schema31_valid_levels,
                f"observed_levels={observed_levels}; nonzero_rows={nonzero_permissions}/{len(rows)}",
            ))
        else:
            checks.append((
                f"{optional_reach_schema_label} optional-reach lane is actually exercised",
                nonzero_permissions > 0,
                f"nonzero_rows={nonzero_permissions}/{len(rows)}; labels come from schema3 target tuple",
            ))
    sm = rows[-1]["sync_miss"]
    checks.append(("sync_miss == 0", sm == 0, f"final sync_miss={sm}"))
    if report_mode in (
        "rally_v10", "rally_v11", "rally_v12", "rally_v13", "rally_v14",
        "hitter_pingpong_recovery_tts_v2", "hitter_pingpong_recovery_tts_v3",
        "hitter_pingpong_continuous_rally_v2",
        "hitter_pingpong_build2_minimal_rootfix_v1",
        "hitter_pingpong_build2_rapid_preempt_v1",
        "hitter_pingpong_build2_fixed_home_feasible_v1",
        "hitter_pingpong_build2_fixed_home_phase_recovery_v1",
        "hitter_pingpong_build2_home_support_frame_v1",
        "hitter_pingpong_build2_torso_optional_reach_v2",
        "schema28",
        "schema29",
        "schema31",
        "schema32",
        "schema33",
        "schema34",
        "rally_v15", "rally_v17", "rally_v17_r10",
    ):
        checks.append((
            "loaded policy bundle exposes joint defaults for V10--V17 elbow gate",
            joint_defaults is not None,
            "available" if joint_defaults is not None else "missing/unreadable",
        ))
        fixed_plane_ok = (
            onnx_planes is not None
            and abs(onnx_planes.get("fh", float("nan")) - 0.58) <= 1.0e-6
            and abs(onnx_planes.get("bh", float("nan")) - 0.58) <= 1.0e-6
        )
        checks.append((
            "loaded policy bundle uses one shared x=0.58 m plane",
            fixed_plane_ok,
            "missing runner geometry banner" if onnx_planes is None else
            f"fh={onnx_planes['fh']:.3f} bh={onnx_planes['bh']:.3f}",
        ))
    if report_mode == "rally_v15":
        checks.append((
            "loaded runner proves the V15 finite-gait ONNX/YAML contract",
            finite_gait is not None,
            "missing/ambiguous runner gait banner" if finite_gait is None else
            f"freq={finite_gait['frequency_hz']:.2f}Hz duty={finite_gait['duty_factor']:.2f} "
            f"step={finite_gait['step_distance']:.2f}m cycles<={finite_gait['max_cycles']} "
            f"|vy|<={finite_gait['velocity_max']:.2f}m/s",
        ))
        loc_ages = [row["o"][B["loc_age"]] for row in rows]
        loc_age_ok = all(math.isfinite(value) and 0.0 <= value < 1.0 for value in loc_ages)
        checks.append((
            "position-receipt localization stays fresh",
            loc_age_ok,
            f"max normalized age={max(loc_ages, default=float('nan')):.3f}",
        ))
        gait_vy = [row["o"][B["gait_vy"]] for row in rows]
        gait_clocks = [value for row in rows for value in seg(row["o"], "gait_clock")]
        gait_modes = [row["o"][B["locomotion_mode"]] for row in rows]
        interventions = [row["o"][B["upper_intervention"]] for row in rows]
        velocity_max = finite_gait["velocity_max"] if finite_gait is not None else -1.0
        deploy_intervention = (
            finite_gait["deploy_intervention"] if finite_gait is not None else float("nan")
        )
        checks.append((
            "V15 finite gait command remains inside the YAML envelope",
            finite_gait is not None
            and all(math.isfinite(v) and abs(v) <= velocity_max + 1e-6 for v in gait_vy)
            and all(math.isfinite(v) and abs(v) <= 1.0 + 1e-6 for v in gait_clocks)
            and all(any(abs(v - allowed) <= 1e-6 for allowed in (-1.0, 0.0, 1.0))
                    for v in gait_modes),
            f"max|vy|={max((abs(v) for v in gait_vy), default=float('nan')):.3f} "
            f"max|clock|={max((abs(v) for v in gait_clocks), default=float('nan')):.3f} "
            f"modes={sorted(set(round(v, 3) for v in gait_modes))}",
        ))
        checks.append((
            "training-only upper intervention is disabled on deploy",
            finite_gait is not None
            and all(abs(value - deploy_intervention) <= 1e-9 for value in interventions),
            f"max|indicator|={max((abs(v) for v in interventions), default=float('nan')):.3g}",
        ))
    gz_bad = sum(1 for r in rows if seg(r["o"], "grav")[2] > -0.7)
    checks.append(("upright (grav_z<-0.7) except transients", gz_bad < 0.02 * len(rows),
                   f"{gz_bad}/{len(rows)} ticks tilted"))
    dmax = max(math.hypot(*seg(r["o"], "dstation")) for r in rows)
    checks.append(("|station delta| <= 0.45 (Final step + readiness bound)", dmax <= 0.45,
                   f"max |dstation|={dmax:.3f} m"))
    action_peaks = [
        max(abs(seg(r["o"], "act")[index]) for r in rows) for index in range(31)
    ]
    action_top = sorted(range(31), key=action_peaks.__getitem__, reverse=True)[:5]
    clamp_stats = runner_clamp_stats(args.runner_log)
    if clamp_stats is not None:
        clamp_ok = (
            not clamp_stats["warned"]
            and clamp_stats["nonzero_samples"] == 0
            and clamp_stats["safe_nonzero_samples"] == 0
        )
        clamp_joints = ",".join(
            f"{name}:{stats['hits']}/{stats['ticks']}@{stats['max_viol']:.4f}rad"
            for name, stats in sorted(clamp_stats["joints"].items())
        )
        qdes_audit_only_checks.append((
                       "policy q_des stays inside safe and hard joint limits (audit-only telemetry)", clamp_ok,
                       f"status_nonzero={clamp_stats['nonzero_samples']}/"
                       f"{clamp_stats['samples']} peak={clamp_stats['peak']} "
                       f"safe_nonzero={clamp_stats['safe_nonzero_samples']}/"
                       f"{clamp_stats['safe_samples']} safe_peak={clamp_stats['safe_peak']} "
                       f"audit_only={clamp_stats['audit_only']} "
                       f"async_warning_count={clamp_stats['warning_count']} "
                       f"joints={clamp_joints or 'unavailable'}"))
    projector_stats = qdes_projector_trace_stats(args.runner_trace) if report_mode == "rally_v15" else None
    if report_mode == "rally_v15":
        if projector_stats is None or "error" in projector_stats:
            qdes_audit_only_checks.append((
                "V15 qdes projector evidence availability (audit-only)",
                False,
                "missing" if projector_stats is None else projector_stats["error"],
            ))
        else:
            jf = projector_stats["joint_fractions"]
            tf = projector_stats["affected_tick_fractions"]
            qdes_audit_only_checks.append((
                "qdes projector feasibility telemetry (audit-only)",
                projector_stats["infeasible_ticks"] == 0,
                f"infeasible_ticks={projector_stats['infeasible_ticks']}/"
                f"{projector_stats['rows']} peak_joints={projector_stats['infeasible_peak']}; "
                f"active/rate/tracking/torque joint_frac="
                f"{jf['active']:.4f}/{jf['rate']:.4f}/{jf['tracking']:.4f}/{jf['torque']:.4f}; "
                f"affected_tick_frac={tf['active']:.4f}/{tf['rate']:.4f}/"
                f"{tf['tracking']:.4f}/{tf['torque']:.4f}; "
                f"max_norm_debt={projector_stats['max_norm_debt']:.4f}",
            ))
    jvmax = max(max(abs(v) for v in seg(r["o"], "jvel")) for r in rows)
    checks.append(("|joint_vel| < 25 rad/s", jvmax < 25.0, f"max |joint_vel|={jvmax:.1f}"))

    # ---- swing segmentation from the tts channel ----
    tts = [r["o"][B["tts"]] for r in rows]
    swings = []
    i, n = 1, len(rows)
    while i < n:
        if tts[i] < tts[i - 1] - 1e-6:          # decreasing -> in a swing
            j = i
            while j + 1 < n and tts[j + 1] < tts[j] - 1e-6:
                j += 1
            if (rows[j]["tick"] - rows[i - 1]["tick"]) * CONTROL_DT > 0.5:
                swings.append((i - 1, j))
            i = j + 1
        else:
            i += 1
    checks.append(("at least one complete swing segmented", len(swings) > 0,
                   f"swings={len(swings)}"))
    if optional_reach_report:
        swing_permissions = []
        unstable = []
        for swing_number, (start, end) in enumerate(swings, start=1):
            pairs = {
                (
                    rows[index]["o"][B["reach_level"]],
                    rows[index]["o"][B["swing_foot_sign"]],
                )
                for index in range(start + 1, end + 1)
            }
            if len(pairs) != 1:
                unstable.append((swing_number, sorted(pairs)))
            else:
                swing_permissions.append(next(iter(pairs)))
        checks.append((
            f"{optional_reach_schema_label} reach permission is frozen for each complete flight",
            not unstable,
            f"swing_pairs={swing_permissions}; unstable={unstable}",
        ))
    strike_indices = [
        min(range(a, b + 1), key=lambda index: abs(tts[index]))
        for a, b in swings
    ]

    # V9 removed the z-only wrist termination because valid right-hand racket reaches caused
    # false terminations. V10 replaces that blunt guard with a direct left-wrist joint debt.
    # Keep this deploy-visible check on V9 too so the regression cannot be hidden again.
    if report_mode in (
        "rally_v9", "rally_v10", "rally_v11", "rally_v12", "rally_v13",
        "rally_v14", "hitter_pingpong_recovery_tts_v2",
        "hitter_pingpong_continuous_rally_v2",
        "hitter_pingpong_build2_minimal_rootfix_v1",
        "hitter_pingpong_build2_rapid_preempt_v1",
        "hitter_pingpong_build2_fixed_home_feasible_v1",
        "hitter_pingpong_build2_fixed_home_phase_recovery_v1",
        "hitter_pingpong_build2_home_support_frame_v1",
        "hitter_pingpong_build2_torso_optional_reach_v2",
        "schema28",
        "schema29",
        "schema31",
        "schema32",
        "schema33",
        "schema34",
        "hitter_pingpong_recovery_tts_v3", "rally_v15",
        "rally_v17", "rally_v17_r10",
    ):
        swing_rows = {index for a, b in swings for index in range(a, b + 1)}
        idle_rows = [row for index, row in enumerate(rows) if index not in swing_rows]
        # V15 (2026-07-24): the v13-facefix clips' READY wrist pose sits ~0.32 rad from the zero
        # boot pose (left wrist roll -0.324), and V15's hold supervision anchors the wrists to
        # exactly that clip frame — so the one-time boot->ready transition alone consumes ~0.33
        # of a 0.35 budget and a compliant policy fails the gate by construction.  Budget the
        # anchor offset + noise explicitly for v15; the gate still catches genuine idle creep
        # (v13 failure was 0.364 vs the then-applicable 0.35 with a ~zero anchor).
        idle_wrist_budget = 0.45 if report_mode == "rally_v15" else 0.35
        for joint_name in (
            "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint"
        ):
            joint_index = JOINT_NAMES.index(joint_name)
            values = [seg(row["o"], "jpos")[joint_index] for row in idle_rows]
            joint_range = max(values) - min(values) if values else float("nan")
            checks.append((
                f"idle {joint_name} range <= {idle_wrist_budget:.2f} rad",
                math.isfinite(joint_range) and joint_range <= idle_wrist_budget,
                f"range={joint_range:.3f} rad over {len(values)} ticks",
            ))

        if (
            isinstance(conductor_report, dict)
            and conductor_report.get("lane_contract")
            in HITTER_FIXED_HOME_LANE_CONTRACTS
        ):
            stationary = conductor_report.get("clean_fixed_home") or {}
            threshold = float(
                stationary.get("natural_wait_actual_q_rms_max_rad", 0.20)
            )
            initial_wait_rows = (
                rows[max(0, swings[0][0] - 250) : swings[0][0]]
                if swings
                else []
            )
            donor_names = tuple(BUILD1_SYNCHRONIZED_WAIT_ACTUAL_Q)
            donor_values = tuple(
                BUILD1_SYNCHRONIZED_WAIT_ACTUAL_Q[name]
                for name in donor_names
            )
            donor_indices = tuple(JOINT_NAMES.index(name) for name in donor_names)
            wait_rms = []
            group_rms = {"shoulder": [], "elbow": [], "wrist": []}
            for row in initial_wait_rows:
                relative = seg(row["o"], "jpos")
                errors = [
                    relative[index]
                    + float(joint_defaults[name])
                    - target
                    for name, target, index in zip(
                        donor_names, donor_values, donor_indices
                    )
                ] if joint_defaults is not None else []
                if errors:
                    wait_rms.append(
                        math.sqrt(sum(value * value for value in errors) / len(errors))
                    )
                    for group_name in group_rms:
                        selected = [
                            error
                            for error, name in zip(errors, donor_names)
                            if group_name in name
                        ]
                        group_rms[group_name].append(
                            math.sqrt(
                                sum(value * value for value in selected)
                                / len(selected)
                            )
                        )

            def _quantile(values, fraction):
                if not values:
                    return float("nan")
                ordered = sorted(values)
                return ordered[
                    min(len(ordered) - 1, int(fraction * len(ordered)))
                ]

            wait_median = _quantile(wait_rms, 0.50)
            wait_p95 = _quantile(wait_rms, 0.95)
            group_medians = {
                name: _quantile(values, 0.50)
                for name, values in group_rms.items()
            }
            natural_wait_ok = bool(
                joint_defaults is not None
                and len(wait_rms) >= 100
                and math.isfinite(wait_median)
                and math.isfinite(wait_p95)
                and wait_median <= threshold
                and wait_p95 <= 1.5 * threshold
            )
            checks.append((
                "initial HOME WAIT actual-q matches synchronized Build1 posture",
                natural_wait_ok,
                f"rows={len(wait_rms)} median={wait_median:.3f}rad "
                f"p95={wait_p95:.3f}rad threshold={threshold:.3f}rad "
                f"groups={group_medians}",
            ))

    swing_sides = None
    if conductor_report is not None:
        runner_sides, side_errors = engaged_sides_from_report(conductor_report)
        side_mapping_ok = not side_errors and len(runner_sides) == len(swings)
        side_detail = "; ".join(side_errors) if side_errors else (
            f"segmented={len(swings)} runner_engages={len(runner_sides)}"
        )
        if not side_errors and len(runner_sides) != len(swings):
            side_detail = (
                f"segmented={len(swings)} runner_engages={len(runner_sides)}; "
                "one-to-one chronological mapping is impossible"
            )
        checks.append(("runner engage sides map one-to-one to obs swings",
                       side_mapping_ok, side_detail))
        if side_mapping_ok:
            swing_sides = runner_sides

    print(f"== {path}: {len(rows)} ticks, {len(swings)} swings segmented; "
          f"mode={report_mode} ==")
    print("   raw last_action peaks (diagnostic, not a safety threshold): " + ", ".join(
        f"{JOINT_NAMES[index]}={action_peaks[index]:.2f}" for index in action_top
    ))
    if report_mode == "rally_v8":
        print("   NOTE: the runner holds both head joints physically, but RallyV8 feeds their "
              "raw action slots back to the actor; use q_des clamp telemetry for safety.")
    elif report_mode == "hitter_pingpong_build2_home_support_frame_v1":
        print("   NOTE: both head joints remain physically passive; their two 110-D action-"
              "history slots carry filtered world base v_x/v_y for Schema26 damping.")
    elif optional_reach_report:
        if report_mode == "schema28":
            print("   NOTE: Schema28 preserves the Schema27 112-D prefix and appends the "
                  "policy-owned 15-D frozen-core action shadow at 112:127. Planner target-tuple "
                  "reach permissions remain at 110/111; plant state never produces them. "
                  "Continuous correlated Planner qualification remains NOT_PROVEN.")
        elif report_mode == "schema29":
            print("   NOTE: Schema29 uses the 112-D non-privileged Planner tuple layout at "
                  "110/111 and one 31-action output. The complete actor mean is trainable; "
                  "there is no Schema28 frozen-core action shadow. The v4 bank is candidate-only, "
                  "so runtime reach permission must remain L0; trajectory/deployment qualification "
                  "remains NOT_PROVEN.")
        elif report_mode == "schema34":
            print("   NOTE: Schema34 reassigns 112-D columns 110/111 to signed external "
                  "reach level/foot and an independent ankle-roll-link FK-to-HOME replant "
                  "need. Gate3 keeps the production envelope supervisor off; the result is "
                  "simulator stress evidence, not deployment qualification.")
        elif report_mode in ("schema31", "schema32", "schema33"):
            extra = (
                " Schema32 additionally requires the V12 deterministic slew contract and "
                "executed-q_des feedback; Gate3 keeps the production envelope supervisor off."
                if report_mode in ("schema32", "schema33") else ""
            )
            print(f"   NOTE: {optional_reach_schema_label} keeps the 112-D non-privileged "
                  "Planner tuple and one 31-action output. Default runtime is locked to L0; "
                  "only the explicit x86 Gate3 simulator lane may pass L1/L2. The v5 bank "
                  "remains a simulator-training candidate, not trajectory or deployment "
                  f"qualification.{extra}")
        else:
            print("   NOTE: Schema27 preserves the 110-D Schema26 prefix and appends schema3 "
                  "Planner target-tuple reach permissions at 110/111; plant state never produces "
                  "these labels. Reach trajectory/deployment qualification remains NOT_PROVEN.")
    elif report_mode in (
        "rally_v9", "rally_v10", "rally_v11", "rally_v12", "rally_v13",
        "rally_v14", "hitter_pingpong_recovery_tts_v2",
        "hitter_pingpong_continuous_rally_v2",
        "hitter_pingpong_build2_minimal_rootfix_v1",
        "hitter_pingpong_build2_rapid_preempt_v1",
        "hitter_pingpong_build2_fixed_home_feasible_v1",
        "hitter_pingpong_build2_fixed_home_phase_recovery_v1",
        "hitter_pingpong_recovery_tts_v3", "rally_v17",
        "rally_v17_r10",
    ):
        print(f"   NOTE: {report_mode.replace('_', ' ').title().replace(' ', '')} holds both head joints and feeds the applied zero head actions "
              "back to the actor; raw head outputs are penalized but not executed.")
    elif report_mode == "rally_v15":
        print("   NOTE: RallyV15 holds both head joints and feeds normalized actually executed "
              "q_des (after the stateful projector) back to the actor.")
    if post_contact_age_mode:
        print("   windows: pre=(%.2f,%.2f] TTS; post=(%.2f,%.2f] s physical "
              "strike age; ready_heading=(%.2f,%.2f) TTS; "
              "post_heading=(%.2f,%.2f] s physical strike age" % (
                  windows["pre"][0], windows["pre"][1],
                  windows["post"][0], windows["post"][1],
                  windows["ready_heading"][0], windows["ready_heading"][1],
                  windows["post_heading"][0], windows["post_heading"][1]))
    else:
        print("   windows: pre=(%.2f,%.2f] post=(-%.2f,-%.2f) "
              "ready_heading=(%.2f,%.2f) post_heading=(-%.2f,-%.2f)" % (
                  windows["pre"][0], windows["pre"][1],
                  windows["post"][1], windows["post"][0],
                  windows["ready_heading"][0], windows["ready_heading"][1],
                  windows["post_heading"][1], windows["post_heading"][0]))
    print("swing  t_start  tts0 side  target_rel_base(xyz)      tgt_vel(xyz)"
          "       |dstation xy| heading  pre_v/p90 post_v/p90 peak|angvel|")
    pre_speeds, post_speeds, pre_p90s, post_p90s = [], [], [], []
    station_y_errors, heading_errors = [], []
    ready_heading_means, ready_heading_maxes, ready_yaw_rates = [], [], []
    post_heading_means, post_heading_maxes, post_yaw_rates = [], [], []
    right_elbow_p90s = []

    def heading_deg_at(t):
        fwd = seg(rows[t]["o"], "fwd")
        return abs(math.degrees(math.atan2(fwd[1], fwd[0])))

    def base_speed_from_station_delta(t):
        if report_mode == "rally_v15":
            return math.hypot(*seg(rows[t]["o"], "base_vel"))
        if t <= 0:
            return float("nan")
        dt = (rows[t]["tick"] - rows[t - 1]["tick"]) * CONTROL_DT
        if dt <= 1e-6:
            return float("nan")
        d0, d1 = seg(rows[t - 1]["o"], "dstation"), seg(rows[t]["o"], "dstation")
        return math.hypot(d1[0] - d0[0], d1[1] - d0[1]) / dt

    def percentile(values, q):
        if not values:
            return float("nan")
        values = sorted(values)
        x = (len(values) - 1) * q
        lo, hi = int(math.floor(x)), int(math.ceil(x))
        return values[lo] if lo == hi else values[lo] + (values[hi] - values[lo]) * (x - lo)

    def mean_or_nan(values):
        return sum(values) / len(values) if values else float("nan")

    for k, (a, b) in enumerate(swings, 1):
        tts0 = tts[a]
        # strike tick = tts closest to 0 inside the swing
        st = strike_indices[k - 1]
        o = rows[st]["o"]
        rr = seg(o, "rkt_rel")
        rv = seg(o, "rkt_vel")
        # Do not infer side from target velocity: FinalV3 boxes may cross vy=0.
        # With conductor JSON this is the runner's chronological engage decision;
        # without it the actor trace cannot identify the clip side.
        side = swing_sides[k - 1] if swing_sides is not None else "??"
        dstation = seg(o, "dstation")
        ds = math.hypot(*dstation)
        dx, dy = abs(dstation[0]), abs(dstation[1])
        heading = heading_deg_at(st)
        pre_lo, pre_hi = windows["pre"]
        post_lo, post_hi = windows["post"]
        ready_lo, ready_hi = windows["ready_heading"]
        post_heading_lo, post_heading_hi = windows["post_heading"]
        pre = [base_speed_from_station_delta(t) for t in range(a + 1, b + 1)
               if pre_lo < tts[t] <= pre_hi]
        if post_contact_age_mode:
            next_strike = (
                strike_indices[k] if k < len(strike_indices) else None
            )
            tail_idx, tail_complete, tail_detail = post_strike_tail_indices(
                rows,
                st,
                next_strike,
                max(post_hi, post_heading_hi),
            )
            checks.append((
                f"swing {k}: {max(post_hi, post_heading_hi):.2f} s "
                "post-strike tail is complete and contiguous",
                tail_complete,
                tail_detail,
            ))
            strike_tick = rows[st]["tick"]
            age_s = {
                t: (rows[t]["tick"] - strike_tick) * CONTROL_DT
                for t in tail_idx
            }
            post = [
                base_speed_from_station_delta(t) for t in tail_idx
                if post_lo < age_s[t] <= post_hi
            ]
        else:
            tail_idx = []
            age_s = {}
            post = [base_speed_from_station_delta(t) for t in range(a + 1, b + 1)
                    if -post_hi < tts[t] < -post_lo]
        pre = [v for v in pre if math.isfinite(v)]
        post = [v for v in post if math.isfinite(v)]
        ready_idx = [t for t in range(a, b + 1) if ready_lo < tts[t] < ready_hi]
        if post_contact_age_mode:
            post_heading_idx = [
                t for t in tail_idx
                if post_heading_lo < age_s[t] <= post_heading_hi
            ]
        else:
            post_heading_idx = [
                t for t in range(a, b + 1)
                if -post_heading_hi < tts[t] < -post_heading_lo
            ]
        ready_h = [heading_deg_at(t) for t in ready_idx]
        post_h = [heading_deg_at(t) for t in post_heading_idx]
        ready_wz = [abs(seg(rows[t]["o"], "ang_vel")[2]) for t in ready_idx]
        post_wz = [abs(seg(rows[t]["o"], "ang_vel")[2]) for t in post_heading_idx]
        ready_h_mean = mean_or_nan(ready_h)
        post_h_mean = mean_or_nan(post_h)
        ready_h_max = max(ready_h) if ready_h else float("nan")
        post_h_max = max(post_h) if post_h else float("nan")
        ready_wz_mean = sum(ready_wz) / len(ready_wz) if ready_wz else float("nan")
        post_wz_mean = sum(post_wz) / len(post_wz) if post_wz else float("nan")
        elbow_p90 = float("nan")
        if report_mode in (
            "rally_v10", "rally_v11", "rally_v12", "rally_v13", "rally_v14",
            "hitter_pingpong_recovery_tts_v2", "hitter_pingpong_recovery_tts_v3",
            "hitter_pingpong_continuous_rally_v2",
            "hitter_pingpong_build2_minimal_rootfix_v1",
            "hitter_pingpong_build2_rapid_preempt_v1",
            "hitter_pingpong_build2_fixed_home_feasible_v1",
            "hitter_pingpong_build2_fixed_home_phase_recovery_v1",
            "hitter_pingpong_build2_home_support_frame_v1",
            "hitter_pingpong_build2_torso_optional_reach_v2",
            "schema28",
            "schema29",
            "schema31",
            "schema32",
            "schema33",
            "schema34",
            "rally_v15", "rally_v17", "rally_v17_r10",
        ) and joint_defaults is not None:
            elbow_index = JOINT_NAMES.index("right_elbow_joint")
            contact_idx = contact_window_indices(tts, a, b, st)
            elbow_values = [
                seg(rows[t]["o"], "jpos")[elbow_index]
                + joint_defaults["right_elbow_joint"]
                for t in contact_idx
            ]
            elbow_p90 = percentile(elbow_values, 0.90)
            if math.isfinite(elbow_p90):
                right_elbow_p90s.append(elbow_p90)
        pre_v = sum(pre) / len(pre) if pre else float("nan")
        post_v = sum(post) / len(post) if post else float("nan")
        pre_p90 = percentile(pre, 0.90)
        post_p90 = percentile(post, 0.90)
        if math.isfinite(pre_v): pre_speeds.append(pre_v)
        if math.isfinite(post_v): post_speeds.append(post_v)
        if math.isfinite(pre_p90): pre_p90s.append(pre_p90)
        if math.isfinite(post_p90): post_p90s.append(post_p90)
        station_y_errors.append(dy)
        heading_errors.append(heading)
        ready_heading_means.append(ready_h_mean)
        ready_heading_maxes.append(ready_h_max)
        ready_yaw_rates.append(ready_wz_mean)
        post_heading_means.append(post_h_mean)
        post_heading_maxes.append(post_h_max)
        post_yaw_rates.append(post_wz_mean)
        motion_scope_end = tail_idx[-1] if tail_idx else b
        pav = max(math.sqrt(sum(v * v for v in seg(rows[t]["o"], "ang_vel")))
                  for t in range(a, motion_scope_end + 1))
        print(f"  {k:2d}  {rows[a]['tick'] * CONTROL_DT:7.1f}  {tts0:4.2f} {side}   "
              f"({rr[0]:+.3f},{rr[1]:+.3f},{rr[2]:+.3f})  "
              f"({rv[0]:+.2f},{rv[1]:+.2f},{rv[2]:+.2f})  {dx:5.3f}/{dy:5.3f} "
              f"{heading:6.1f} {pre_v:5.3f}/{pre_p90:5.3f} "
              f"{post_v:5.3f}/{post_p90:5.3f} {pav:6.2f}")
        print(f"      heading ready mean/max={ready_h_mean:.1f}/{ready_h_max:.1f} deg "
              f"post mean/max={post_h_mean:.1f}/{post_h_max:.1f} deg "
              f"ready/post |wz|={ready_wz_mean:.3f}/{post_wz_mean:.3f} rad/s")
        if report_mode in (
            "rally_v10", "rally_v11", "rally_v12", "rally_v13", "rally_v14",
            "hitter_pingpong_recovery_tts_v2", "hitter_pingpong_recovery_tts_v3",
            "hitter_pingpong_continuous_rally_v2",
            "hitter_pingpong_build2_minimal_rootfix_v1",
            "hitter_pingpong_build2_rapid_preempt_v1",
            "hitter_pingpong_build2_fixed_home_feasible_v1",
            "hitter_pingpong_build2_fixed_home_phase_recovery_v1",
            "hitter_pingpong_build2_home_support_frame_v1",
            "hitter_pingpong_build2_torso_optional_reach_v2",
            "schema28",
            "schema29",
            "schema31",
            "schema32",
            "schema33",
            "schema34",
            "rally_v15", "rally_v17", "rally_v17_r10",
        ) and joint_defaults is not None:
            print(f"      contact right-elbow absolute q p90={elbow_p90:.3f} rad")
        # Contract checks: the actor target should stay on the fixed plane and station should be kept.
        # This CSV does not contain actual racket FK; physical tracking is gated in Isaac/full MuJoCo.
        reach_x = rr[0] - dstation[0]
        # Expected reach = the loaded MODEL's per-side plane (runner metadata banner,
        # see discover_planes_from_runner). V10 is a strict shared 0.58 m plane; V8/V9
        # retain their historical per-side geometry. Mode constants are legacy-log fallbacks.
        if onnx_planes is not None and side in onnx_planes:
            expected_reach_x = onnx_planes[side]
        elif onnx_planes:
            # Side unknown (swing_sides missing / "??"): the model's planes are still authoritative.
            # v13's two planes are 0.15 m apart vs the +/-0.01 tolerance, so the NEAREST one is
            # unambiguous — and an off-plane strike still fails. A mode constant here would reject
            # a correct v13 export (both planes differ from the v12-era 0.69).
            expected_reach_x = min(onnx_planes.values(), key=lambda p: abs(reach_x - p))
        elif report_mode in (
            "rally_v8", "rally_v9", "rally_v10", "rally_v11", "rally_v12",
            "rally_v13", "rally_v14", "hitter_pingpong_recovery_tts_v2",
            "hitter_pingpong_continuous_rally_v2",
            "hitter_pingpong_build2_minimal_rootfix_v1",
            "hitter_pingpong_build2_rapid_preempt_v1",
            "hitter_pingpong_build2_fixed_home_feasible_v1",
            "hitter_pingpong_build2_fixed_home_phase_recovery_v1",
            "hitter_pingpong_build2_home_support_frame_v1",
            "hitter_pingpong_build2_torso_optional_reach_v2",
            "schema28",
            "schema29",
            "schema31",
            "schema32",
            "schema33",
            "schema34",
            "hitter_pingpong_recovery_tts_v3",
            "rally_v15", "rally_v17", "rally_v17_r10",
        ):
            expected_reach_x = (
                0.58 if report_mode in (
                    "rally_v10", "rally_v11", "rally_v12", "rally_v13",
                    "rally_v14", "hitter_pingpong_recovery_tts_v2",
                    "hitter_pingpong_continuous_rally_v2",
                    "hitter_pingpong_build2_minimal_rootfix_v1",
                    "hitter_pingpong_build2_rapid_preempt_v1",
                    "hitter_pingpong_build2_fixed_home_feasible_v1",
                    "hitter_pingpong_build2_fixed_home_phase_recovery_v1",
                    "hitter_pingpong_build2_home_support_frame_v1",
                    "hitter_pingpong_build2_torso_optional_reach_v2",
                    "schema28",
                    "schema29",
                    "schema31",
                    "schema32",
                    "schema33",
                    "schema34",
                    "hitter_pingpong_recovery_tts_v3",
                    "rally_v15", "rally_v17", "rally_v17_r10",
                )
                else (0.65 if side == "fh" else 0.50)
            )
        elif report_mode == "rally_final_v3":
            expected_reach_x = 0.70
        else:
            expected_reach_x = 0.51
        reach_x_lo, reach_x_hi = expected_reach_x - 0.01, expected_reach_x + 0.01
        checks.append((
            f"swing {k}: baked target reach-x in [{reach_x_lo:.2f},{reach_x_hi:.2f}]",
            reach_x_lo <= reach_x <= reach_x_hi,
            f"{reach_x:+.3f}",
        ))
        station_x_limit = (
            0.10 if report_mode == "rally_v17_r10" else
            (0.03 if report_mode in (
                "rally_v10", "rally_v11", "rally_v12", "rally_v13",
                "rally_v14", "hitter_pingpong_recovery_tts_v2",
                "hitter_pingpong_continuous_rally_v2",
                "hitter_pingpong_build2_minimal_rootfix_v1",
                "hitter_pingpong_build2_rapid_preempt_v1",
                "hitter_pingpong_build2_fixed_home_feasible_v1",
                "hitter_pingpong_build2_fixed_home_phase_recovery_v1",
                "hitter_pingpong_build2_home_support_frame_v1",
                "hitter_pingpong_build2_torso_optional_reach_v2",
                "schema28",
                "schema29",
                "schema31",
                "schema32",
                "schema33",
                "schema34",
                "hitter_pingpong_recovery_tts_v3", "rally_v15",
                "rally_v17",
            ) else 0.10)
        )
        checks.append((f"swing {k}: |station_x error| at strike <= {station_x_limit:.2f}",
                       dx <= station_x_limit, f"{dx:.3f}"))
        checks.append((f"swing {k}: |station_y error| at strike <= 0.10",
                       dy <= 0.10, f"{dy:.3f}"))
        checks.append((f"swing {k}: heading at strike <= 15 deg",
                       heading <= 15.0, f"{heading:.1f}"))
        checks.append((f"swing {k}: ready heading mean/max <= 10/15 deg",
                       math.isfinite(ready_h_mean) and math.isfinite(ready_h_max)
                       and ready_h_mean <= 10.0 and ready_h_max <= 15.0,
                       f"{ready_h_mean:.1f}/{ready_h_max:.1f}"))
        checks.append((f"swing {k}: post heading mean/max <= 10/15 deg",
                       math.isfinite(post_h_mean) and math.isfinite(post_h_max)
                       and post_h_mean <= 10.0 and post_h_max <= 15.0,
                       f"{post_h_mean:.1f}/{post_h_max:.1f}"))
        checks.append((f"swing {k}: ready/post |yaw-rate| <= 0.20 rad/s",
                       math.isfinite(ready_wz_mean) and math.isfinite(post_wz_mean)
                       and ready_wz_mean <= 0.20 and post_wz_mean <= 0.20,
                       f"{ready_wz_mean:.3f}/{post_wz_mean:.3f}"))
        checks.append((f"swing {k}: pre-strike base speed <= 0.20 m/s",
                       math.isfinite(pre_v) and pre_v <= 0.20, f"{pre_v:.3f}"))
        checks.append((f"swing {k}: post-strike base speed <= 0.25 m/s",
                       math.isfinite(post_v) and post_v <= 0.25, f"{post_v:.3f}"))
        checks.append((f"swing {k}: pre-strike base-speed p90 <= 0.30 m/s",
                       math.isfinite(pre_p90) and pre_p90 <= 0.30, f"{pre_p90:.3f}"))
        checks.append((f"swing {k}: post-strike base-speed p90 <= 0.35 m/s",
                       math.isfinite(post_p90) and post_p90 <= 0.35, f"{post_p90:.3f}"))
        if report_mode in (
            "rally_v10", "rally_v11", "rally_v12", "rally_v13", "rally_v14",
            "hitter_pingpong_recovery_tts_v2", "hitter_pingpong_recovery_tts_v3",
            "hitter_pingpong_continuous_rally_v2",
            "hitter_pingpong_build2_minimal_rootfix_v1",
            "hitter_pingpong_build2_rapid_preempt_v1",
            "hitter_pingpong_build2_fixed_home_feasible_v1",
            "hitter_pingpong_build2_fixed_home_phase_recovery_v1",
            "hitter_pingpong_build2_home_support_frame_v1",
            "hitter_pingpong_build2_torso_optional_reach_v2",
            "schema28",
            "schema29",
            "schema31",
            "schema32",
            "schema33",
            "schema34",
            "rally_v15", "rally_v17", "rally_v17_r10",
        ) and joint_defaults is not None:
            checks.append((
                f"swing {k}: contact right-elbow q p90 <= 1.35 rad",
                math.isfinite(elbow_p90) and elbow_p90 <= 1.35,
                f"{elbow_p90:.3f}",
            ))

    speed_source = ("V15 filtered position-receipt velocity" if report_mode == "rally_v15"
                    else "station-delta finite difference at the 50 Hz control tick")
    print(f"\n== rally phase metrics ({speed_source}) ==")
    print(f"  station_y_error@strike mean={mean_or_nan(station_y_errors):.3f} m")
    print(f"  heading_error@strike mean={mean_or_nan(heading_errors):.1f} deg")
    print(f"  ready_heading mean/max={mean_or_nan(ready_heading_means):.1f}/"
          f"{max(ready_heading_maxes, default=float('nan')):.1f} deg")
    print(f"  post_heading mean/max={mean_or_nan(post_heading_means):.1f}/"
          f"{max(post_heading_maxes, default=float('nan')):.1f} deg")
    print(f"  ready/post |yaw-rate| mean={mean_or_nan(ready_yaw_rates):.3f}/"
          f"{mean_or_nan(post_yaw_rates):.3f} rad/s")
    print(f"  pre_strike_base_speed mean={mean_or_nan(pre_speeds):.3f} m/s")
    print(f"  post_strike_base_speed mean={mean_or_nan(post_speeds):.3f} m/s")
    print(f"  pre/post base_speed p90 mean={mean_or_nan(pre_p90s):.3f}/"
          f"{mean_or_nan(post_p90s):.3f} m/s")
    if report_mode in (
        "rally_v10", "rally_v11", "rally_v12", "rally_v13", "rally_v14",
        "hitter_pingpong_recovery_tts_v2", "hitter_pingpong_recovery_tts_v3",
        "hitter_pingpong_continuous_rally_v2",
        "hitter_pingpong_build2_minimal_rootfix_v1",
        "hitter_pingpong_build2_rapid_preempt_v1",
        "hitter_pingpong_build2_fixed_home_feasible_v1",
        "hitter_pingpong_build2_fixed_home_phase_recovery_v1",
        "hitter_pingpong_build2_home_support_frame_v1",
        "hitter_pingpong_build2_torso_optional_reach_v2",
        "schema28",
        "schema29",
        "schema31",
        "schema32",
        "schema33",
        "schema34",
        "rally_v15", "rally_v17", "rally_v17_r10",
    ) and joint_defaults is not None:
        print(f"  contact right-elbow q p90 mean/max={mean_or_nan(right_elbow_p90s):.3f}/"
              f"{max(right_elbow_p90s, default=float('nan')):.3f} rad")
    print(f"  NOTE: {expected_n_obs}-D obs CSV has no torso pose, foot body velocity, or "
          "left-arm/actual-racket FK. Torso HOLD and support-frame qualification come from "
          "the synchronized MuJoCo plant report; offline full-state metrics remain separate.")

    print("\n== checks ==")
    for name, passed, detail in checks:
        print(f"  {'PASS' if passed else 'FAIL'}  {name}  ({detail})")
        ok, bad = ok + passed, bad + (not passed)
    if qdes_audit_only_checks:
        print("\n== q_des audit-only diagnostics (never affect PASS/FAIL) ==")
        for name, passed, detail in qdes_audit_only_checks:
            print(f"  {'OK' if passed else 'OBSERVED'}  {name}  ({detail})")

    if conductor_report is not None:
        rep = conductor_report
        proxy_completed = int(
            rep.get(
                "completed_lane_proxy",
                rep.get("completed_recovered_proxy", rep.get("returned", 0)),
            )
        )
        proxy_rate = rep.get("proxy_rate", rep.get("return_rate"))
        print(f"\n== conductor report: serves={rep['serves']} proxy_ok={proxy_completed} "
              f"falls={rep['falls']} drift={rep['station_drift_m']}m "
              f"-> {'PASS' if rep['pass'] else 'FAIL'} ==")
        expected_swings = proxy_completed
        trace_coverage = len(swings) >= expected_swings
        checks.append(("obs trace covers every proxy-completed swing", trace_coverage,
                       f"segmented={len(swings)} proxy_ok={expected_swings}"))
        print(f"  {'PASS' if trace_coverage else 'FAIL'}  "
              "obs trace covers every proxy-completed swing "
              f"(segmented={len(swings)} proxy_ok={expected_swings})")
        ok, bad = ok + trace_coverage, bad + (not trace_coverage)
        if rep.get("lane_contract") in HITTER_FIXED_HOME_LANE_CONTRACTS:
            for lane_key, lane_label in (
                ("clean_fixed_home", "clean fixed-HOME hit"),
                ("rapid_fixed_home", "rapid fixed-HOME engage"),
            ):
                lane = rep.get(lane_key) or {}
                passed = bool(lane.get("pass"))
                checks.append((f"Gate3 {lane_label} lane", passed, str(lane)))
                print(
                    f"  {'PASS' if passed else 'FAIL'}  Gate3 {lane_label} lane "
                    f"({lane})"
                )
                ok, bad = ok + passed, bad + (not passed)
        checks.append(("conductor PASS", bool(rep["pass"]),
                       f"proxy_rate={proxy_rate} station_coverage="
                       f"{rep.get('station_transition_coverage_ok')}"))
        # Add the late check immediately; the common loop above has already run.
        passed = bool(rep["pass"])
        print(f"  {'PASS' if passed else 'FAIL'}  conductor PASS")
        ok, bad = ok + passed, bad + (not passed)
    print(f"\n{'PASS' if bad == 0 else 'FAIL'}: {ok} checks passed, {bad} failed")
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
