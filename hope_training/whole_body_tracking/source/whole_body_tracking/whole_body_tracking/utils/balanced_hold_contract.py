"""Prevent v10 from silently inheriting the v9 direct temporal actor objective."""


def validate_balanced_hold_algorithm(reward_revision, algorithm):
    if reward_revision not in {"actual_complete_hold_v11_stationary_sweet_spot_v1", "actual_balanced_hold_v10", "actual_complete_hold_v11", "actual_complete_hold_v11_loaded_feet_v1", "actual_complete_hold_v11_strike_position_v2", "actual_complete_hold_v11_stationary_reach_v1", "actual_complete_hold_v11_stationary_reach_v2", "actual_complete_hold_v11_stationary_reach_v3", "actual_complete_hold_v11_stationary_reach_v4", "actual_hold_reference_v11", "four_clip_full_body_v12", "fixed_home_four_clip_v13", "joint_clock_four_clip_v14", "discrete_four_stroke_v15", "compatible_four_stroke_recovery_v16", "four_stroke_motion_quality_v17"}:
        return
    if algorithm.get("settled_aux_enabled", False) or any(
        float(algorithm.get(key, 0.0)) != 0.0 for key in (
            "settled_aux_smooth_coef", "settled_aux_tracking_coef",
            "settled_aux_gradient_ratio_max",
        )
    ):
        raise ValueError("Balanced/CompleteHold requires algo=ppo_balanced_hold: "
                         "direct temporal actor refinement must be disabled with zero coefficients")
