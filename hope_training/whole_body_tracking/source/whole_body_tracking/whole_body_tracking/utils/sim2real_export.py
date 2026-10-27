"""Deployment metadata for the continuous rally recipe (110-D V2 wire).

Replay and artificial latency are training distributions. They are recorded
here, not reproduced as an extra delay or a reset in the deployment runner.
"""

import hashlib
import json

RECIPE = "continuous_rally_sim2real_v3"
RECIPES = ("continuous_rally_sim2real_v2", RECIPE)


def sim2real_metadata(env, task):
    build = task.get("build_contract", {})
    recipe = build.get("training_recipe_revision")
    action = env.action_manager.get_term("joint_pos")
    motion = env.command_manager.get_term("motion").cfg
    racket = env.command_manager.get_term("racket_target").cfg
    resolved = action.resolved_contract_metadata()
    fixed_home = build.get("station_command_contract") == "fixed_session_anchor_v1"
    checks = {
        "recipe": recipe in RECIPES,
        "actor/action dimensions": (build.get("actor_observation_dim"), build.get("action_dim")) == (110, 31),
        "affine action": resolved["qdes_action_contract"] == "v11_affine_safe_qdes_v1",
        "feedback": resolved["qdes_policy_feedback_contract"] == "legacy_applied_raw_v1",
        "passive head": tuple(action.passive_joint_names) == ("head_yaw_joint", "head_pitch_joint"),
        "mocap": racket.base_mocap_enabled and racket.base_mocap_orientation_enabled,
        "target": racket.target_mode == "hitter_pure" and not racket.locomotion_enabled,
        "component velocity": len(racket.racket_vel_start_range_per_clip) == 2 and len(racket.racket_vel_planner_range_per_clip) == 2,
        "replay": motion.post_swing_replay_contract == "markov_side_phase_severity_v3",
        "rally timing": motion.rally_timing_enabled,
        "no handoff": motion.runtime_handoff_start_prob == 0.0,
        "no recovery scheduler": not racket.recovery_curriculum_enabled,
        "training latency": (resolved["qdes_delay_min_steps"], resolved["qdes_delay_max_steps"], resolved["qdes_delay_nominal_fraction"]) == (0, 1, 0.5),
        "action joint order": list(resolved["qdes_joint_names"]) == list(env.scene["robot"].data.joint_names),
        "control rate": abs(env.step_dt - 0.02) < 1e-9,
        "follow-through coverage": motion.rally_post_strike_s_range[0] <= 0.5 <= motion.rally_post_strike_s_range[1],
    }
    if recipe == RECIPE:
        checks["shared 25-tick follow-through"] = (
            tuple(motion.rally_post_strike_s_range) == (0.5, 0.5)
            and motion.rally_timing_probability == 1.0
            and motion.rally_timing_ramp_steps == 0
            and tuple(motion.hold_steps_range) == (0, 0)
        )
    ready_v7 = build.get("reward_revision") in {
        "actual_ready_recovery_v7", "actual_full_body_hold_v8", "actual_stable_hold_v9",
        "actual_balanced_hold_v10", "actual_hold_reference_v11", "actual_complete_hold_v11",
        "actual_complete_hold_v11_loaded_feet_v1",
        "actual_complete_hold_v11_strike_position_v2",
        "actual_complete_hold_v11_stationary_reach_v1",
        "actual_complete_hold_v11_stationary_reach_v2",
        "actual_complete_hold_v11_stationary_reach_v3",
        "actual_complete_hold_v11_stationary_reach_v4",
        "four_clip_full_body_v12", "fixed_home_four_clip_v13", "joint_clock_four_clip_v14", "discrete_four_stroke_v15",
        "compatible_four_stroke_recovery_v16",
        "four_stroke_motion_quality_v17",
    }
    if ready_v7:
        checks["v7 interruptible recovery without blocking HOLD"] = (
            motion.post_wrap_min_hold == 0 and motion.post_wrap_hold_jitter_steps == 0
            and not motion.wrap_teleport and not motion.hold_long_on_wrap
            and getattr(motion, "interruptible_recovery_enabled", False)
            and tuple(motion.recovery_window_steps_range) == (40, 60)
            and tuple(motion.next_ball_delay_steps_range) == (0, 80)
            and motion.next_ball_immediate_prob == .25
            and not racket.hold_until_settled and not racket.ready_release_enabled
            and getattr(racket, "one_step_contract", "disabled") == "disabled"
            and build.get("inter_ball_hold_contract") == "interruptible_recovery_next_ball_v1"
        )
        checks["v7 recovery target"] = build.get("recovery_target_contract") == (
            "fixed_session_anchor_v1" if fixed_home else "current_command_station_v1")
    # Stationary two-clip rally: the original forehand/backhand clips answer a wide
    # station-relative reach bank from one immutable session anchor. The runner's
    # fixed-anchor target contract value is unchanged; the distribution revision
    # key below records the stationary bank.
    stationary = fixed_home and bool(getattr(racket, "venue_tuple_fixed_station", False))
    if fixed_home:
        checks["immutable station and four-clip contact teacher"] = (
            tuple(racket.base_target_x_range) == (0., 0.)
            and tuple(racket.base_target_y_range) == (0., 0.)
            and racket.station_y_step_range is None
            and (stationary or motion.step_reference_mode in ("target_reach", "discrete_four_stroke")))
    if stationary:
        checks["stationary reach bank"] = (
            getattr(motion, "step_reference_mode", "station_error") == "station_error"
            and racket.venue_tuple_mix_mode == "per_swing_bank_v2"
            and racket.venue_tuple_final_mix_prob == 1.0
            and racket.venue_tuple_robust_sampling
            and not racket.venue_tuple_limit_reset_travel
            and build.get("training_distribution_revision") == "stationary_reach_v1"
            and racket.racket_pos_range_per_clip[0][1][1] <= racket.racket_pos_range_per_clip[1][1][0])
    if getattr(motion, "step_reference_mode", "") == "discrete_four_stroke":
        checks["four complete teachers and causal first-frame HOLD"] = (
            motion.causal_idle and motion.hold_from_backhand_first_frame
            and not motion.hold_reference_sets_action_default
            and len(motion.step_motion_files) == 2)
    natural_tail = bool(getattr(motion, "continue_recovery_reference", False))
    if natural_tail:
        checks["joint question clock and visible recovery tail"] = (
            fixed_home and racket.venue_tuple_joint_clock
            and racket.venue_tuple_bank_curriculum_steps == 0
            and racket.venue_tuple_final_mix_prob == 1.0
            and not racket.venue_tuple_limit_reset_travel
            and build.get("recovery_reference_contract") == "old_ball_tail_until_arrival_v1")
    failed = [name for name, ok in checks.items() if not ok]
    if failed:
        raise RuntimeError(f"Refusing {RECIPE} export: {', '.join(failed)}")
    task_json = json.dumps(task, sort_keys=True, separators=(",", ":"), allow_nan=False)
    csv = lambda values: ",".join(f"{float(v):.4f}" for v in values)
    return {
        "hitter_pure_training_recipe": recipe,
        "hitter_pure_training_recipe_version": "3" if recipe == RECIPE else "2",
        "hitter_pure_runtime_contract": "rally_final_v2",
        "hitter_pure_deployment_status": "gate3_candidate",
        "hitter_pure_action_contract": resolved["qdes_action_contract"],
        "hitter_pure_resolved_task_sha256": hashlib.sha256(task_json.encode()).hexdigest(),
        "hitter_pure_resolved_task_json": task_json,
        "hitter_pingpong_build_contract": recipe,
        "hitter_pure_training_distribution_revision": build.get("training_distribution_revision", "legacy"),
        "hitter_pingpong_rally_end_contract": "ball_clock_followthrough_v1",
        "hitter_pingpong_rally_post_strike_s": "0.5000",
        **({"hitter_pingpong_recovery_reference_contract": "old_ball_tail_until_arrival_v1"} if natural_tail else {}),
        "hitter_pingpong_replay_contract": motion.post_swing_replay_contract,
        "hitter_pingpong_deploy_delay_contract": "direct_no_added_delay_v1",
        "hitter_pure_target_sample_contract": (
            "four_clip_ball_clock_v1" if fixed_home else "v14_near_static_prefix_v1"),
        **({"hitter_pure_station_contract": "fixed_session_anchor_v1"} if fixed_home else {
            "hitter_pure_station_y_step_range": csv(racket.station_y_step_range),
            "hitter_pure_station_y_mixture": csv((racket.station_y_same_prob, racket.station_y_small_step_prob, *racket.station_y_small_step_range, *racket.station_y_step_range)),
        }),
        "hitter_pure_station_side_explicit": str(racket.station_side_explicit).lower(),
        "hitter_pure_ready_hold_steps_range": ",".join(str(v) for v in motion.hold_steps_range),
        # Training-only inter-ball exposure; never add a timer to runtime release.
        "hitter_pure_training_post_wrap_hold_steps": f"{getattr(motion, 'post_wrap_min_hold', 0)},{getattr(motion, 'post_wrap_hold_jitter_steps', 0)}",
        "hitter_pure_training_inter_ball_contract": build.get("inter_ball_hold_contract", "legacy_unspecified"),
        "hitter_pure_recovery_target_contract": build.get("recovery_target_contract", "legacy_unspecified"),
        "hitter_pure_reward_revision": build.get("reward_revision", "legacy_unspecified"),
        "hitter_pure_hold_pose_contract": build.get("hold_pose_contract", "legacy_partial_pose"),
        "hitter_pure_hold_reference_sha256": getattr(motion, "hold_reference_sha256", None) or "legacy",
        "hitter_pure_motion_manifest_sha256": getattr(motion, "reference_manifest_sha256", None) or "legacy",
        **({"hitter_pure_training_stroke_adapter_sha256": motion.stroke_adapter_sha256,
            "hitter_pure_training_recovery_arrival_contract": build.get("training_recovery_arrival_contract", "unspecified")}
           if getattr(motion, "stroke_adapter_path", None) else {}),
        "hitter_pure_hold_long": f"{motion.hold_long_prob:.4f},{motion.hold_long_steps_range[0]},{motion.hold_long_steps_range[1]}",
        "hitter_pure_passive_head": ",".join(action.passive_joint_names),
        "hitter_pure_last_action_head": "zero",
    }
