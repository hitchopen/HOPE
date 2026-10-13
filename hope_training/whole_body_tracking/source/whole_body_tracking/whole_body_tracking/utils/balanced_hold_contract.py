"""Prevent v10 from silently inheriting the v9 direct temporal actor objective."""


def validate_balanced_hold_algorithm(reward_revision, algorithm):
    if reward_revision != "actual_balanced_hold_v10":
        return
    if algorithm.get("settled_aux_enabled", False) or any(
        float(algorithm.get(key, 0.0)) != 0.0 for key in (
            "settled_aux_smooth_coef", "settled_aux_tracking_coef",
            "settled_aux_gradient_ratio_max",
        )
    ):
        raise ValueError("HOPEPingPongBalancedHold requires algo=ppo_balanced_hold: "
                         "direct temporal actor refinement must be disabled with zero coefficients")
