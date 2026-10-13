"""Explicit public sensor/plant knobs, shared by train and play."""

import math


def _range(value, label, *, positive=False):
    if len(value) != 2:
        raise ValueError(f"{label} requires [lo, hi]")
    lo, hi = map(float, value)
    if not all(map(math.isfinite, (lo, hi))) or lo > hi or (positive and lo < 0):
        raise ValueError(f"invalid {label}: {value}")
    return lo, hi


def apply_sim2real_overrides(env_cfg, task, applied):
    # Noise is on the actor only. Never change the privileged critic's observations.
    noise = task.get("observation_noise")
    if noise is not None:
        policy = env_cfg.observations.policy
        supported = {
            "base_ang_vel", "joint_pos", "joint_vel", "projected_gravity",
            "base_forward_xy", "base_target_delta_xy", "racket_target_rel_base",
        }
        unknown = set(noise) - supported - {"enabled"}
        if unknown:
            raise ValueError(f"unsupported observation_noise keys: {sorted(unknown)}")
        if "enabled" in noise:
            if not isinstance(noise["enabled"], bool):
                raise ValueError("observation_noise.enabled must be boolean")
            policy.enable_corruption = noise["enabled"]
        for name in supported & set(noise):
            term = getattr(policy, name)
            cfg = term.noise
            if cfg is None or not hasattr(cfg, "n_min") or not hasattr(cfg, "n_max"):
                raise ValueError(f"{name} is not a uniform-noise observation")
            cfg.n_min, cfg.n_max = _range(noise[name], name)
            applied.append(f"observations.policy.{name}.noise = {noise[name]}")

    dr = task.get("domain_rand") or {}
    if "base_com_range" in dr:
        value = dr["base_com_range"]
        if set(value) != {"x", "y", "z"}:
            raise ValueError("domain_rand.base_com_range requires exactly x/y/z")
        env_cfg.events.base_com.params["com_range"] = {
            axis: _range(bounds, f"COM {axis}") for axis, bounds in value.items()
        }
        applied.append(f"events.base_com.com_range = {value}")
    push = dr.get("push")
    if push is not None:
        event = env_cfg.events.sim2real_push
        names = {"max_delta_v_xy", "cohort_fraction", "ramp_steps", "strike_exclusion_s"}
        if set(push) - names - {"interval_s"}:
            raise ValueError("unsupported domain_rand.push parameter")
        if "interval_s" in push:
            event.interval_range_s = _range(push["interval_s"], "push interval", positive=True)
            if event.interval_range_s[0] <= 0:
                raise ValueError("push interval must be positive")
        for name in names & set(push):
            value = float(push[name])
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"invalid push.{name}")
            if name == "cohort_fraction" and value > 1:
                raise ValueError("push cohort_fraction must be in [0, 1]")
            if name == "ramp_steps" and not value.is_integer():
                raise ValueError("push ramp_steps must be integer")
            event.params[name] = int(value) if name == "ramp_steps" else value
        applied.append(f"events.sim2real_push = {push}")

    friction = task.get("friction_contract")
    if friction is not None:
        # These numeric fields used to be descriptive YAML while the asset kept defaults.
        material = env_cfg.events.physics_material.params
        for name, dest in (
            ("robot_static_friction_range", "static_friction_range"),
            ("robot_dynamic_friction_range", "dynamic_friction_range"),
            ("restitution_range", "restitution_range"),
        ):
            if name in friction:
                material[dest] = _range(friction[name], name, positive=True)
                applied.append(f"events.physics_material.{dest} = {material[dest]}")
        if "make_consistent" in friction:
            if not isinstance(friction["make_consistent"], bool):
                raise ValueError("friction make_consistent must be boolean")
            material["make_consistent"] = friction["make_consistent"]
