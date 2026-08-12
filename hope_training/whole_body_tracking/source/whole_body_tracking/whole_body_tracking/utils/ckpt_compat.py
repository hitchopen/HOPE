"""Shape-tolerant loading for inference/export and explicit cross-layout warm-starts.

2026-07-03: the DeployParity CRITIC obs dropped the vestigial ``base_target_pos_b`` (2 dims,
``HOPECriticDeployParityCfg``), so every pre-change DeployParity/RealSensor checkpoint (including the
deployed p4 lineage) fails rsl_rl's strict ``load_state_dict`` on the critic first layer when loaded
into the current env cfg. play.py (ONNX re-export) and eval_deterministic.py only ever need the ACTOR,
so they fall back to an actor-preserving partial load instead of dying. ``train.py`` uses this helper
only when the operator explicitly selects ``checkpoint_tolerant=true``; strict and exact resume stay
fail-closed.
"""

from __future__ import annotations


def load_actor_std_normalizer_only(runner, path: str) -> dict:
    """Strictly migrate policy actor/std/actor-normalizer while leaving the critic fresh.

    This is deliberately narrower than a tolerant warm start: every actor tensor name and shape
    must match exactly, the exploration-noise tensors must match exactly, and normalizer presence
    must agree on both sides. The optimizer, critic and all runner/environment counters are never
    read from the checkpoint.
    """

    import torch

    checkpoint = torch.load(path, map_location="cpu", weights_only=False)
    source = checkpoint.get("model_state_dict", checkpoint)
    if not isinstance(source, dict):
        raise RuntimeError("checkpoint has no model_state_dict mapping")
    current = runner.alg.policy.state_dict()

    def _actor_key(name: str) -> bool:
        return name.startswith("actor.")

    def _normalizer_key(name: str) -> bool:
        return name.startswith("actor_obs_normalizer.")

    def _noise_key(name: str) -> bool:
        return name in {"std", "log_std"} or name.startswith("distribution.std")

    source_actor = {name for name in source if _actor_key(name)}
    current_actor = {name for name in current if _actor_key(name)}
    if not source_actor or source_actor != current_actor:
        raise RuntimeError(
            "R8->R9 actor tensor-name mismatch: "
            f"missing={sorted(current_actor - source_actor)}, "
            f"extra={sorted(source_actor - current_actor)}"
        )

    source_noise = {name for name in source if _noise_key(name)}
    current_noise = {name for name in current if _noise_key(name)}
    if not source_noise or source_noise != current_noise:
        raise RuntimeError(
            "R8->R9 exploration-noise tensor mismatch: "
            f"source={sorted(source_noise)}, current={sorted(current_noise)}"
        )

    source_normalizer = {name for name in source if _normalizer_key(name)}
    current_normalizer = {name for name in current if _normalizer_key(name)}
    if source_normalizer != current_normalizer:
        raise RuntimeError(
            "R8->R9 actor observation-normalizer mismatch: "
            f"source={sorted(source_normalizer)}, current={sorted(current_normalizer)}"
        )

    selected_names = source_actor | source_noise | source_normalizer
    selected = {}
    for name in sorted(selected_names):
        if tuple(source[name].shape) != tuple(current[name].shape):
            raise RuntimeError(
                f"R8->R9 tensor shape mismatch for {name}: "
                f"source={tuple(source[name].shape)}, current={tuple(current[name].shape)}"
            )
        selected[name] = source[name]
    # rsl_rl's ActorCritic intentionally wraps ``nn.Module.load_state_dict`` and returns a
    # ``bool`` ("is this a resume?") instead of PyTorch's ``_IncompatibleKeys``.  We already
    # prove above that every selected key exists and has the exact shape, so load through the
    # public API and verify the resulting tensor values rather than depending on a version-
    # specific return type.
    load_result = runner.alg.policy.load_state_dict(selected, strict=False)
    if isinstance(load_result, bool) and not load_result:
        raise RuntimeError("rsl_rl rejected the R8->R9 actor migration state")
    if hasattr(load_result, "unexpected_keys") and load_result.unexpected_keys:
        raise RuntimeError(
            "unexpected actor migration tensors: "
            f"{sorted(load_result.unexpected_keys)}"
        )
    loaded_state = runner.alg.policy.state_dict()
    mismatched_after_load = [
        name
        for name in sorted(selected_names)
        if not torch.equal(
            loaded_state[name].detach().cpu(), source[name].detach().cpu()
        )
    ]
    if mismatched_after_load:
        raise RuntimeError(
            "R8->R9 actor migration post-load verification failed for "
            f"{mismatched_after_load}"
        )
    return {
        "loaded_tensor_names": sorted(selected_names),
        "actor_tensor_count": len(source_actor),
        "noise_tensor_names": sorted(source_noise),
        "actor_normalizer_present": bool(source_normalizer),
    }


def load_actor_tolerant(runner, path: str, *, load_optimizer: bool = False) -> None:
    """``runner.load(path)`` with an actor-preserving, shape-tolerant fallback.

    Tries the normal strict model load first. Optimizer restoration is opt-in; evaluation/export and
    cross-recipe warm-starts do not need it.
    On a shape mismatch (pre-2026-07-03 critic layout), re-loads the raw checkpoint, drops every
    tensor whose name/shape does not match the current policy, and loads the rest non-strictly —
    the actor (and its normalizer, if any) survive intact; the critic re-initializes. Loudly warns,
    because a policy loaded this way is a deliberate warm-start (fresh critic + no optimizer
    state), never an exact resume.
    """
    try:
        runner.load(path, load_optimizer=load_optimizer)
        return
    except RuntimeError as e:
        import torch

        ckpt = torch.load(path, map_location="cpu", weights_only=False)
        sd = ckpt.get("model_state_dict", ckpt)
        cur = runner.alg.policy.state_dict()
        keep = {k: v for k, v in sd.items() if k in cur and cur[k].shape == v.shape}
        dropped = sorted(set(sd) - set(keep))
        actor_dropped = [k for k in dropped if not k.startswith("critic")]
        if actor_dropped:
            # The mismatch is NOT confined to the critic — a partial load would silently corrupt the
            # actor. Re-raise the original strict error instead.
            raise RuntimeError(
                f"checkpoint/actor shape mismatch (not just the critic): {actor_dropped}"
            ) from e
        runner.alg.policy.load_state_dict(keep, strict=False)
        print(
            f"[compat] strict checkpoint load failed (pre-2026-07-03 critic layout: base_target_pos_b "
            f"was removed from the DeployParity critic). Loaded {len(keep)}/{len(sd)} tensors, dropped "
            f"{dropped}. ACTOR intact — fresh-critic warm-start/eval only; not exact resume.",
            flush=True,
        )
