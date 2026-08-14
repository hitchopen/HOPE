"""Fail-loud translation of the public task YAML reward recipe.

Isaac Lab registers reward terms on ``EnvCfg.rewards``; Hydra's ``task.rewards``
mapping is a separate recipe layer and does not update those terms by itself.
This module is deliberately free of Isaac imports so its complete key mapping
can be regression-tested on an ordinary host Python.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any, Callable


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "1", "yes", "on"}:
            return True
        if normalized in {"false", "0", "no", "off"}:
            return False
    if isinstance(value, (int, float)) and value in (0, 1):
        return bool(value)
    raise ValueError(f"reward override expects a boolean, got {value!r}")


_WEIGHT_AND_STD_PREFIXES: tuple[tuple[str, str], ...] = (
    ("racket_position", "racket_position"),
    ("racket_velocity", "racket_velocity"),
    ("racket_normal", "racket_normal"),
    ("base_position", "base_position"),
    ("post_strike_brake", "post_strike_brake"),
    ("hold_heading", "hold_heading"),
    ("lower_body_plant_imitation", "lower_body_plant_imitation"),
    ("hold_ready", "hold_ready"),
    ("hold_upper_pose_imitation", "hold_upper_pose_imitation"),
    ("stance_width", "stance_width"),
    ("racket_normal_alignment_debt", "racket_normal_alignment_debt"),
    ("post_strike_x_settle", "post_strike_x_settle"),
    ("post_strike_vx_quiet", "post_strike_vx_quiet"),
    ("pre_strike_station_settle", "pre_strike_station_settle"),
    ("passive_head_raw_action", "passive_head_raw_action"),
    ("left_wrist_reference_debt", "left_wrist_reference_debt"),
    ("right_elbow_extension_debt", "right_elbow_extension_debt"),
    ("ready_stance_width", "ready_stance_width"),
    ("ready_foot_alignment", "ready_foot_alignment"),
    ("ready_leg_settle", "ready_leg_settle"),
    ("strike_x_gate_margin", "strike_x_gate_margin"),
    ("post_swing_xlock", "post_swing_xlock"),
    ("post_swing_base_quiet", "post_swing_base_quiet"),
    ("post_swing_leg_quiet", "post_swing_leg_quiet"),
    ("settle_foot_slip", "settle_foot_slip"),
    ("rally_ready_root_height_debt", "rally_ready_root_height_debt"),
    ("post_swing_tilt_debt", "post_swing_tilt_debt"),
)

WEIGHT_SPECS: dict[str, str] = {
    f"{prefix}_weight": term for prefix, term in _WEIGHT_AND_STD_PREFIXES
}
WEIGHT_SPECS.update(
    {
        "foot_orientation_weight": "foot_orientation",
        "rally_ankle_qdes_saturation_weight": "rally_ankle_qdes_saturation",
        "post_strike_leg_quiet_weight": "post_strike_leg_quiet",
        "joint_torques_weight": "joint_torques",
        "action_rate_weight": "action_rate_l2",
        "joint_acc_weight": "joint_acc",
        "joint_limit_weight": "joint_limit",
        "undesired_contacts_weight": "undesired_contacts",
        "windup_x_recovery_weight": "windup_x_recovery",
        "rally_heading_debt_weight": "rally_heading_debt",
        "ready_deadline_weight": "ready_deadline",
        "idle_left_wrist_debt_weight": "idle_left_wrist_debt",
        "all_joint_qdes_barrier_weight": "rally_joint_qdes_saturation",
    }
)

STD_SPECS: dict[str, str] = {
    f"{prefix}_std": term for prefix, term in _WEIGHT_AND_STD_PREFIXES
}
STD_SPECS["rally_ankle_qdes_std"] = "rally_ankle_qdes_saturation"


FLOAT_PARAM_SPECS: dict[str, tuple[str, str]] = {}
BOOL_PARAM_SPECS: dict[str, tuple[str, str]] = {}
INT_PARAM_SPECS: dict[str, tuple[str, str]] = {}


def _add_float_params(term: str, prefix: str, names: Mapping[str, str]) -> None:
    FLOAT_PARAM_SPECS.update(
        {f"{prefix}_{yaml_suffix}": (term, param) for yaml_suffix, param in names.items()}
    )


def _add_bool_params(term: str, prefix: str, names: Mapping[str, str]) -> None:
    BOOL_PARAM_SPECS.update(
        {f"{prefix}_{yaml_suffix}": (term, param) for yaml_suffix, param in names.items()}
    )


_add_float_params(
    "hold_ready",
    "hold_ready",
    {"reach": "reach", "heading_gate_rad": "heading_gate"},
)
_add_bool_params("hold_ready", "hold_ready", {"include_ang_vel": "include_ang_vel"})
_add_bool_params("foot_orientation", "foot_orientation", {"hold_gate": "hold_gate"})
_add_float_params(
    "rally_ankle_qdes_saturation",
    "rally_ankle_qdes",
    {"safe_abs": "safe_abs", "t_pre": "t_pre", "t_post": "t_post"},
)
_add_float_params("stance_width", "stance_width", {"lo": "lo", "hi": "hi"})
_add_float_params(
    "racket_normal_alignment_debt",
    "racket_normal_alignment_debt",
    {"margin": "margin"},
)
_add_float_params("post_strike_x_settle", "post_strike_x_settle", {"t_hi": "t_hi"})
_add_float_params("post_strike_vx_quiet", "post_strike_vx_quiet", {"t_hi": "t_hi"})
_add_float_params("post_strike_leg_quiet", "post_strike_leg_quiet", {"t_hi": "t_hi"})
_add_float_params(
    "windup_x_recovery",
    "windup_x_recovery",
    {
        "x_margin": "x_margin",
        "x_std": "x_std",
        "vx_margin": "vx_margin",
        "vx_std": "vx_std",
        "position_blend": "position_blend",
        "t_hi": "t_hi",
    },
)
_add_float_params(
    "pre_strike_station_settle",
    "pre_strike_station_settle",
    {
        "v_gain": "v_gain",
        "v_max": "v_max",
        "t_min": "t_min",
        "t_max": "t_max",
        "velocity_margin": "velocity_margin",
    },
)
_add_bool_params(
    "pre_strike_station_settle",
    "pre_strike_station_settle",
    {"debt_huber": "debt_huber"},
)
_add_float_params(
    "rally_heading_debt",
    "rally_heading_debt",
    {
        "yaw_margin": "yaw_margin",
        "yaw_std": "yaw_std",
        "rate_margin": "rate_margin",
        "rate_std": "rate_std",
        "heading_blend": "heading_blend",
        "ready_t_lo": "ready_t_lo",
        "ready_t_hi": "ready_t_hi",
        "post_t_lo": "post_t_lo",
        "post_t_hi": "post_t_hi",
        "yaw_rate_gain": "yaw_rate_gain",
        "yaw_rate_max": "yaw_rate_max",
        "forehand_scale": "forehand_scale",
        "backhand_scale": "backhand_scale",
    },
)
_add_bool_params("rally_heading_debt", "rally_heading_debt", {"huber_tail": "huber_tail"})
_add_float_params(
    "passive_head_raw_action",
    "passive_head_raw_action",
    {"huber_delta": "huber_delta"},
)
_add_float_params(
    "strike_x_drift",
    "strike_x_drift",
    {"margin": "margin", "std": "std", "t_pre": "t_pre", "t_post": "t_post"},
)
_add_bool_params("strike_x_drift", "strike_x_drift", {"huber_tail": "huber_tail"})
_add_float_params(
    "left_wrist_reference_debt",
    "left_wrist_reference_debt",
    {"margin": "margin", "max_blend": "max_blend"},
)
_add_float_params(
    "right_elbow_extension_debt",
    "right_elbow_extension_debt",
    {"extension_start": "extension_start", "t_pre": "t_pre", "t_post": "t_post"},
)
_add_bool_params(
    "right_elbow_extension_debt",
    "right_elbow_extension_debt",
    {"forehand_only": "forehand_only"},
)
_add_float_params(
    "ready_deadline",
    "ready_deadline",
    {
        "x_margin": "x_margin",
        "y_margin": "y_margin",
        "position_std": "position_std",
        "speed_margin": "speed_margin",
        "speed_std": "speed_std",
        "speed_blend": "speed_blend",
        "final_window_s": "final_window_s",
    },
)
INT_PARAM_SPECS["ready_deadline_target_step_class"] = (
    "ready_deadline",
    "target_step_class",
)
_add_float_params("ready_stance_width", "ready_stance_width", {"lo": "lo", "hi": "hi"})
_add_float_params(
    "ready_foot_alignment",
    "ready_foot_alignment",
    {"margin": "margin", "max_blend": "max_blend"},
)
_add_float_params("ready_leg_settle", "ready_leg_settle", {"margin": "margin"})
_add_float_params(
    "strike_x_gate_margin",
    "strike_x_gate_margin",
    {
        "margin": "margin",
        "half_window_s": "half_window_s",
        "forehand_scale": "forehand_scale",
        "backhand_scale": "backhand_scale",
    },
)
_add_float_params(
    "idle_left_wrist_debt",
    "idle_left_wrist_debt",
    {
        "position_margin": "position_margin",
        "position_std": "position_std",
        "velocity_margin": "velocity_margin",
        "velocity_std": "velocity_std",
        "velocity_blend": "velocity_blend",
        "max_blend": "max_blend",
    },
)
_add_float_params(
    "rally_joint_qdes_saturation",
    "all_joint_qdes_barrier",
    {
        "safe_margin_fraction": "safe_margin_fraction",
        "std_fraction": "std_fraction",
        "topk_blend": "topk_blend",
    },
)
INT_PARAM_SPECS["all_joint_qdes_barrier_topk"] = (
    "rally_joint_qdes_saturation",
    "topk",
)
_add_float_params(
    "post_swing_xlock",
    "post_swing_xlock",
    {
        "margin": "margin",
        "t_lo": "t_lo",
        "t_hi": "t_hi",
        "forehand_scale": "forehand_scale",
        "backhand_scale": "backhand_scale",
    },
)
for _term, _prefix in (
    ("post_swing_base_quiet", "post_swing_base_quiet"),
    ("post_swing_leg_quiet", "post_swing_leg_quiet"),
):
    _add_float_params(
        _term,
        _prefix,
        {"margin": "margin", "t_lo": "t_lo", "t_hi": "t_hi"},
    )
    _add_bool_params(_term, _prefix, {"huber_tail": "huber_tail"})
_add_float_params(
    "settle_foot_slip",
    "settle_foot_slip",
    {
        "margin": "margin",
        "station_reach": "station_reach",
        "pre_t_max": "pre_t_max",
        "strike_t_post": "strike_t_post",
        "post_t_lo": "post_t_lo",
        "post_t_hi": "post_t_hi",
    },
)
_add_bool_params("settle_foot_slip", "settle_foot_slip", {"huber_tail": "huber_tail"})
_add_float_params(
    "rally_ready_root_height_debt",
    "rally_ready_root_height_debt",
    {
        "min_height": "min_height",
        "ready_t_lo": "ready_t_lo",
        "ready_t_hi": "ready_t_hi",
        "post_t_lo": "post_t_lo",
        "post_t_hi": "post_t_hi",
    },
)
_add_float_params(
    "post_swing_tilt_debt",
    "post_swing_tilt_debt",
    {"margin": "margin", "t_lo": "t_lo", "t_hi": "t_hi"},
)


MULTI_FLOAT_PARAM_SPECS: dict[str, tuple[tuple[str, str], ...]] = {
    "ready_stance_station_reach": tuple(
        (term, "station_reach")
        for term in ("ready_stance_width", "ready_foot_alignment", "ready_leg_settle")
    ),
    "ready_stance_heading_gate_rad": tuple(
        (term, "heading_gate")
        for term in ("ready_stance_width", "ready_foot_alignment", "ready_leg_settle")
    ),
    "ready_stance_speed_gate": tuple(
        (term, "speed_gate") for term in ("ready_stance_width", "ready_foot_alignment")
    ),
}

STRING_PARAM_SPECS: dict[str, tuple[str, str, frozenset[str] | None]] = {
    "hold_ready_reach_mode": (
        "hold_ready",
        "reach_mode",
        frozenset({"racket", "station"}),
    )
}

TUPLE_INT_PARAM_SPECS: dict[str, tuple[str, str]] = {
    "ready_deadline_target_step_classes": (
        "ready_deadline",
        "target_step_classes",
    )
}

# These predecessor keys are deliberately null in the Build recipe. RallyV13's
# all-joint barrier supersedes both old signatures.
NULLABLE_PREDECESSOR_KEYS = frozenset(
    {
        "rally_joint_qdes_saturation_weight",
        "rally_joint_qdes_saturation_std",
        "rally_joint_qdes_saturation_max_blend",
        "waist_qdes_saturation_weight",
        "waist_qdes_saturation_std",
        "waist_qdes_saturation_max_blend",
    }
)

SUPPORTED_REWARD_KEYS = frozenset(
    set(WEIGHT_SPECS)
    | set(STD_SPECS)
    | set(FLOAT_PARAM_SPECS)
    | set(BOOL_PARAM_SPECS)
    | set(INT_PARAM_SPECS)
    | set(MULTI_FLOAT_PARAM_SPECS)
    | set(STRING_PARAM_SPECS)
    | set(TUPLE_INT_PARAM_SPECS)
    | set(NULLABLE_PREDECESSOR_KEYS)
)


def _term(rewards: Any, name: str) -> Any:
    if not hasattr(rewards, name) or getattr(rewards, name) is None:
        raise AttributeError(
            f"task.rewards targets rewards.{name}, but that reward term is not registered"
        )
    return getattr(rewards, name)


def _set_weight(rewards: Any, term_name: str, value: Any, applied: list[str]) -> None:
    term = _term(rewards, term_name)
    term.weight = float(value)
    applied.append(f"rewards.{term_name}.weight={term.weight}")


def _set_param(
    rewards: Any,
    term_name: str,
    param_name: str,
    value: Any,
    cast: Callable[[Any], Any],
    applied: list[str],
) -> None:
    term = _term(rewards, term_name)
    params = getattr(term, "params", None)
    if params is None or param_name not in params:
        raise AttributeError(
            f"task.rewards targets rewards.{term_name}.params[{param_name!r}], "
            "but that parameter is not registered"
        )
    converted = cast(value)
    params[param_name] = converted
    applied.append(f"rewards.{term_name}.params.{param_name}={converted!r}")


def _apply_nullable_predecessors(
    rewards: Any, values: Mapping[str, Any], applied: list[str]
) -> None:
    # Apply these last, matching the reviewed translator ordering. The shipped
    # recipe sets all six to null, which removes the obsolete max_blend only.
    old_weight = values.get("rally_joint_qdes_saturation_weight")
    if old_weight is not None:
        _set_weight(rewards, "rally_joint_qdes_saturation", old_weight, applied)
    for key, param in (
        ("rally_joint_qdes_saturation_std", "std"),
        ("rally_joint_qdes_saturation_max_blend", "max_blend"),
    ):
        if key not in values:
            continue
        value = values.get(key)
        if value is None:
            # The reference recipe uses explicit null only to retire the inherited
            # max_blend parameter after replacing the scalar term with the
            # all-joint barrier. A null std means "do not override".
            if param == "max_blend":
                term = _term(rewards, "rally_joint_qdes_saturation")
                if param in term.params:
                    term.params.pop(param)
                    applied.append(
                        "rewards.rally_joint_qdes_saturation.params.max_blend=<removed>"
                    )
        else:
            _set_param(
                rewards,
                "rally_joint_qdes_saturation",
                param,
                value,
                float,
                applied,
            )

    waist_values = {
        "weight": values.get("waist_qdes_saturation_weight"),
        "std": values.get("waist_qdes_saturation_std"),
        "max_blend": values.get("waist_qdes_saturation_max_blend"),
    }
    if "waist_qdes_saturation_weight" in values and waist_values["weight"] is not None:
        _set_weight(rewards, "waist_qdes_saturation", waist_values["weight"], applied)
    for param, key in (
        ("std", "waist_qdes_saturation_std"),
        ("max_blend", "waist_qdes_saturation_max_blend"),
    ):
        if key in values and waist_values[param] is not None:
            _set_param(
                rewards,
                "waist_qdes_saturation",
                param,
                waist_values[param],
                float,
                applied,
            )


def apply_reward_overrides(
    rewards: Any, rewards_cfg: Mapping[str, Any] | Any | None, applied: list[str]
) -> None:
    """Apply every configured reward key or fail before environment creation.

    ``rewards_cfg`` may be a plain mapping or an OmegaConf ``DictConfig``. A
    newly-added/misspelled key is an error; silently training with code defaults
    would make the YAML cease to be the recipe source of truth.
    """

    if rewards_cfg is None:
        return
    if not hasattr(rewards_cfg, "items"):
        raise TypeError("task.rewards must be a mapping")
    values = dict(rewards_cfg.items())
    unknown = sorted(set(values) - SUPPORTED_REWARD_KEYS)
    if unknown:
        raise KeyError(f"unsupported task.rewards key(s): {', '.join(unknown)}")

    for key, term_name in WEIGHT_SPECS.items():
        value = values.get(key)
        if value is not None:
            _set_weight(rewards, term_name, value, applied)
    for key, term_name in STD_SPECS.items():
        value = values.get(key)
        if value is not None:
            _set_param(rewards, term_name, "std", value, float, applied)
    for key, (term_name, param_name) in FLOAT_PARAM_SPECS.items():
        value = values.get(key)
        if value is not None:
            _set_param(rewards, term_name, param_name, value, float, applied)
    for key, (term_name, param_name) in BOOL_PARAM_SPECS.items():
        value = values.get(key)
        if value is not None:
            _set_param(rewards, term_name, param_name, value, _as_bool, applied)
    for key, (term_name, param_name) in INT_PARAM_SPECS.items():
        value = values.get(key)
        if value is not None:
            _set_param(rewards, term_name, param_name, value, int, applied)
    for key, targets in MULTI_FLOAT_PARAM_SPECS.items():
        value = values.get(key)
        if value is not None:
            for term_name, param_name in targets:
                _set_param(rewards, term_name, param_name, value, float, applied)
    for key, (term_name, param_name, allowed) in STRING_PARAM_SPECS.items():
        value = values.get(key)
        if value is not None:
            converted = str(value)
            if allowed is not None and converted not in allowed:
                raise ValueError(
                    f"task.rewards.{key} must be one of {sorted(allowed)}, got {converted!r}"
                )
            _set_param(rewards, term_name, param_name, converted, str, applied)
    for key, (term_name, param_name) in TUPLE_INT_PARAM_SPECS.items():
        value = values.get(key)
        if value is not None:
            converted = tuple(int(item) for item in value)
            if not converted:
                raise ValueError(f"task.rewards.{key} must not be empty")
            _set_param(
                rewards,
                term_name,
                param_name,
                converted,
                lambda item: item,
                applied,
            )

    _apply_nullable_predecessors(rewards, values, applied)
