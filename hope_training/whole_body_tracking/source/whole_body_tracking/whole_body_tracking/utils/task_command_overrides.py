"""Apply the public motion/racket/action recipe without silently ignoring knobs.

Kept independent of Isaac so both launchers and host configuration tests use the
same translation. A per-clip mapping is always FH then BH, never YAML key order.
"""

from collections.abc import Mapping


_RACKET_ALIASES = {
    "pos_range_per_clip": "racket_pos_range_per_clip",
    "vel_range_per_clip": "racket_vel_range_per_clip",
    "vel_start_range_per_clip": "racket_vel_start_range_per_clip",
    "vel_planner_range_per_clip": "racket_vel_planner_range_per_clip",
    "vel_planner_mix_prob": "racket_vel_planner_mix_prob",
    "vel_range_ramp_steps": "racket_vel_range_ramp_steps",
}


def _tuple_value(value):
    if isinstance(value, (list, tuple)):
        return tuple(_tuple_value(v) for v in value)
    return value


def _clip_ranges(value):
    if value is None:
        return None
    if not isinstance(value, Mapping) or set(value) != {"forehand", "backhand"}:
        raise ValueError("per-clip ranges require exactly forehand and backhand")
    result = []
    for side in ("forehand", "backhand"):
        if set(value[side]) != {"x", "y", "z"}:
            raise ValueError(f"{side} ranges require exactly x, y, z")
        axes = []
        for axis in ("x", "y", "z"):
            bounds = value[side][axis]
            if len(bounds) != 2 or float(bounds[0]) > float(bounds[1]):
                raise ValueError(f"invalid {side}.{axis} range: {bounds}")
            axes.append(tuple(float(x) for x in bounds))
        result.append(tuple(axes))
    return tuple(result)


def apply_command_overrides(env_cfg, task, applied):
    """Consume plain resolved dictionaries; reject unsupported/misspelled fields."""
    for block, destination, aliases in (
        ("motion", env_cfg.commands.motion, {}),
        ("racket", env_cfg.commands.racket_target, _RACKET_ALIASES),
        ("action", env_cfg.actions.joint_pos, {"contract": "action_contract"}),
    ):
        for key, value in (task.get(block) or {}).items():
            attr = aliases.get(key, key)
            if not hasattr(destination, attr):
                raise ValueError(f"unsupported task.{block}.{key}: no config field {attr}")
            if block == "racket" and key in _RACKET_ALIASES and key.endswith("range_per_clip"):
                value = _clip_ranges(value)
            else:
                value = _tuple_value(value)
            setattr(destination, attr, value)
            applied.append(f"{block}.{attr} = {value}")
