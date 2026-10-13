"""Explicit same-ABI actor initialization, never optimizer/environment resume."""
import torch


def initialize_actor(policy, checkpoint, source_task, target_task):
    for key in ("gym_task", "actor_obs_contract", "action", "motion_file", "motion_file_2"):
        if key not in source_task or source_task[key] != target_task.get(key):
            raise ValueError(f"actor warm-start contract mismatch: {key}")
    for key in ("actor_observation_dim", "action_dim"):
        if source_task["build_contract"][key] != target_task["build_contract"][key]:
            raise ValueError(f"actor warm-start dimension mismatch: {key}")
    saved = checkpoint.get("hope_exact_resume_state", {})
    if saved.get("policy_exploration_contract") != policy.exploration_contract:
        raise ValueError("actor warm-start exploration contract mismatch")
    state = checkpoint["model_state_dict"]
    if any("normaliz" in key for key in state):
        raise ValueError("normalized actor migration is not supported")
    actor = {key.removeprefix("actor."): value for key, value in state.items() if key.startswith("actor.")}
    expected = policy.actor.state_dict()
    if set(actor) != set(expected) or any(actor[k].shape != expected[k].shape for k in expected):
        raise ValueError("actor warm-start architecture mismatch")
    std = state.get("std")
    if std is None or std.shape != policy.std.shape or not torch.isfinite(std).all():
        raise ValueError("actor warm-start invalid exploration std")
    if not all(torch.isfinite(v).all() for v in actor.values()):
        raise ValueError("actor warm-start nonfinite weights")
    if (std < policy.min_noise_std).any() or (std > policy.max_noise_std).any():
        raise ValueError("actor warm-start std outside current bounds")
    # Validate everything before mutation. Critic, Adam moments, iteration,
    # environment/replay state, RNG and curriculum are deliberately untouched.
    policy.actor.load_state_dict(actor, strict=True)
    with torch.no_grad():
        policy.std.copy_(std)
