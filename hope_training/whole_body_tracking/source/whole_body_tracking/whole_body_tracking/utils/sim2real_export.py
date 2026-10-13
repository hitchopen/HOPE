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
        "actual_balanced_hold_v10",
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
        checks["v7 recovery target"] = build.get("recovery_target_contract") == "current_command_station_v1"
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
        "hitter_pingpong_rally_end_contract": "ball_clock_followthrough_v1",
        "hitter_pingpong_rally_post_strike_s": "0.5000",
        "hitter_pingpong_replay_contract": motion.post_swing_replay_contract,
        "hitter_pingpong_deploy_delay_contract": "direct_no_added_delay_v1",
        "hitter_pure_target_sample_contract": "v14_near_static_prefix_v1",
        "hitter_pure_station_y_step_range": csv(racket.station_y_step_range),
        "hitter_pure_station_y_mixture": csv((racket.station_y_same_prob, racket.station_y_small_step_prob, *racket.station_y_small_step_range, *racket.station_y_step_range)),
        "hitter_pure_station_side_explicit": str(racket.station_side_explicit).lower(),
        "hitter_pure_ready_hold_steps_range": ",".join(str(v) for v in motion.hold_steps_range),
        # Training-only inter-ball exposure; never add a timer to runtime release.
        "hitter_pure_training_post_wrap_hold_steps": f"{getattr(motion, 'post_wrap_min_hold', 0)},{getattr(motion, 'post_wrap_hold_jitter_steps', 0)}",
        "hitter_pure_training_inter_ball_contract": build.get("inter_ball_hold_contract", "legacy_unspecified"),
        "hitter_pure_recovery_target_contract": build.get("recovery_target_contract", "legacy_unspecified"),
        "hitter_pure_reward_revision": build.get("reward_revision", "legacy_unspecified"),
        "hitter_pure_hold_pose_contract": build.get("hold_pose_contract", "legacy_partial_pose"),
        "hitter_pure_hold_long": f"{motion.hold_long_prob:.4f},{motion.hold_long_steps_range[0]},{motion.hold_long_steps_range[1]}",
        "hitter_pure_passive_head": ",".join(action.passive_joint_names),
        "hitter_pure_last_action_head": "zero",
    }
