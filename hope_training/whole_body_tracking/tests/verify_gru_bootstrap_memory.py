"""Regression checks for non-consuming recurrent critic bootstrapping.

Runs outside Isaac. Only config types are stubbed by the existing CPU harness;
policy, GRU, rollout storage, GAE and PPO are the real implementations. No
optimizer, simulator or training launcher is run. Optionally inspect trusted
local checkpoint weights without modifying the checkpoint.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path

import torch
from tensordict import TensorDict

from verify_stationary_gru_update import (
    BoundedStdActorCriticRecurrent,
    RecurrentProjectionPPO,
    observation,
)


def algorithm(policy, obs, device):
    alg = RecurrentProjectionPPO(
        policy=policy, device=device, num_learning_epochs=1, num_mini_batches=4,
        projection_aux_enabled=True, projection_aux_lambda=.1,
        projection_aux_lambda_max=.1, projection_aux_gradient_ratio_max=.25,
        projection_aux_gaussian_tails=True, settled_aux_enabled=False,
        settled_aux_smooth_coef=0., settled_aux_gradient_ratio_max=0.,
    )
    alg.init_storage("rl", obs.batch_size[0], 64, obs, [policy.std.numel()])
    return alg


def toy_pair(device):
    torch.manual_seed(7)
    obs = observation().to(device)
    policy = BoundedStdActorCriticRecurrent(
        obs=obs, obs_groups={"policy": ["policy"], "critic": ["critic"]},
        num_actions=4, actor_hidden_dims=[16], critic_hidden_dims=[16],
        rnn_type="gru", rnn_hidden_dim=8, rnn_num_layers=1,
        init_noise_std=.35, min_noise_std=.05, max_noise_std=.6,
        entropy_excluded_action_indices=[3],
    ).to(device)
    return algorithm(policy, obs, device), algorithm(copy.deepcopy(policy), obs, device)


def fill_rollout(algorithms, observation_at, device):
    """Include resets inside and exactly at the 64-step rollout boundary."""
    for step in range(64):
        obs, next_obs = observation_at(step), observation_at(step + 1)
        dones = torch.zeros(obs.batch_size[0], dtype=torch.bool, device=device)
        if step in (1, 17, 39, 63):
            dones[(step // 3) % len(dones)] = True
        for alg in algorithms:
            alg.act(obs)
            alg.process_env_step(next_obs, torch.ones_like(dones, dtype=torch.float), dones, {})
    return observation_at(64)


def assert_same(actual, expected):
    torch.testing.assert_close(actual, expected, rtol=0, atol=0)


def compare_boundary(fixed, legacy, next_obs):
    actor_before = fixed.policy.memory_a.hidden_state.clone()
    critic_before = fixed.policy.memory_c.hidden_state.clone()
    # Independent one-consumption oracle: call the GRU directly, without its
    # stateful Memory wrapper or the method being tested.
    critic_input = fixed.policy.critic_obs_normalizer(fixed.policy.get_critic_obs(next_obs))
    features, expected_hidden = fixed.policy.memory_c.rnn(critic_input.unsqueeze(0), critic_before)
    expected_value = fixed.policy.critic(features.squeeze(0))

    fixed.compute_returns(next_obs)
    delta = float((fixed.policy.memory_c.hidden_state - critic_before).abs().max())
    assert delta == 0., f"bootstrap consumed critic memory: max change={delta}"
    assert_same(fixed.policy.memory_a.hidden_state, actor_before)

    # Deliberately bypass the fix to retain a counterfactual of the original
    # real PPO behavior. GAE must be unchanged by the state-preservation fix.
    super(RecurrentProjectionPPO, legacy).compute_returns(next_obs)
    legacy_delta = float((legacy.policy.memory_c.hidden_state - critic_before).abs().max())
    assert legacy_delta > 1e-6, "fixture did not expose the original memory advance"
    assert_same(fixed.storage.returns, legacy.storage.returns)
    assert_same(fixed.storage.advantages, legacy.storage.advantages)

    returns = fixed.storage.returns.clone()
    fixed.compute_returns(next_obs)
    assert_same(fixed.policy.memory_c.hidden_state, critic_before)
    assert_same(fixed.storage.returns, returns)

    fixed.storage.clear()
    legacy.storage.clear()
    fixed.act(next_obs)
    legacy.act(next_obs)
    assert_same(fixed.transition.hidden_states[1], critic_before)
    assert_same(fixed.transition.values, expected_value)
    assert_same(fixed.policy.memory_c.hidden_state, expected_hidden)
    double_read_error = float((legacy.transition.values - expected_value).abs().max())
    assert double_read_error > 1e-6, "fixture did not distinguish duplicate consumption"
    return dict(
        bootstrap_critic_hidden_change=delta,
        legacy_bootstrap_critic_hidden_change=legacy_delta,
        next_value_error=float((fixed.transition.values - expected_value).abs().max()),
        legacy_next_value_error=double_read_error,
        returns_and_advantages_match_legacy=True,
        repeated_bootstrap_is_idempotent=True,
        next_rollout_starts_with_unconsumed_hidden=True,
    )


def check_empty_and_exception_states(device):
    results = []
    for empty in (False, True):
        alg, _ = toy_pair(device)
        next_obs = fill_rollout([alg], lambda t: observation(t * .01).to(device), device)
        if empty:
            alg.policy.memory_c.reset()
        before = None if empty else alg.policy.memory_c.hidden_state.clone()
        actor_before = alg.policy.memory_a.hidden_state.clone()
        # A failing GAE call happens after the stateful value forward. The
        # original error must propagate, while finally restores the memory.
        original = alg.storage.compute_returns
        marker = RuntimeError("deliberate GAE failure")
        def fail(*args, **kwargs):
            raise marker
        alg.storage.compute_returns = fail
        try:
            try:
                alg.compute_returns(next_obs)
            except RuntimeError as exc:
                assert exc is marker
            else:
                raise AssertionError("GAE failure was swallowed")
        finally:
            alg.storage.compute_returns = original
        if empty:
            assert alg.policy.memory_c.hidden_state is None
        else:
            assert_same(alg.policy.memory_c.hidden_state, before)
        assert_same(alg.policy.memory_a.hidden_state, actor_before)
        alg.compute_returns(next_obs)
        if empty:
            assert alg.policy.memory_c.hidden_state is None
        else:
            assert_same(alg.policy.memory_c.hidden_state, before)
        assert bool(torch.isfinite(alg.storage.returns).all())
        results.append(dict(initial_critic_hidden="None" if empty else "tensor", normal_and_exception_restore=True))
    return results


def checkpoint_pair(path, device):
    saved = torch.load(path, map_location="cpu", weights_only=False)
    resume = saved["hope_exact_resume_state"]
    env = resume["environment_resume_state"]
    fields = env["managers"]["action_manager"]["terms"]["joint_pos"]["fields"]
    obs = TensorDict({key: env["initial_observations"][key][:16].clone().to(device)
                     for key in ("policy", "critic")}, batch_size=[16])
    policy = BoundedStdActorCriticRecurrent(
        obs=obs, obs_groups={"policy": ["policy"], "critic": ["critic"]},
        num_actions=31, actor_hidden_dims=[512, 256, 128], critic_hidden_dims=[512, 256, 128],
        rnn_type="gru", rnn_hidden_dim=110, rnn_num_layers=1,
        init_noise_std=.35, min_noise_std=.05, max_noise_std=.6,
        entropy_excluded_action_indices=fields["_passive_action_cols"].tolist(),
    ).to(device)
    policy.load_state_dict(saved["model_state_dict"], strict=True)
    legacy_policy = copy.deepcopy(policy)
    for item in (policy, legacy_policy):
        for name, hidden in resume["policy_memory_state"].items():
            getattr(item, name).reset(hidden_state=hidden[:, :16].to(device).clone())
    return algorithm(policy, obs, device), algorithm(legacy_policy, obs, device), obs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="cpu", choices=("cpu", "cuda:0"))
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args()
    torch.set_num_threads(4)
    result = dict(device=args.device, seed=7, rollout_steps=64, optimizer_steps=0)
    with torch.inference_mode():
        fixed, legacy = toy_pair(args.device)
        next_obs = fill_rollout([fixed, legacy], lambda t: observation(t * .01).to(args.device), args.device)
        result["toy_boundary"] = compare_boundary(fixed, legacy, next_obs)
        result["empty_and_exception_states"] = check_empty_and_exception_states(args.device)
        if args.checkpoint:
            before_hash = hashlib.sha256(args.checkpoint.read_bytes()).hexdigest()
            fixed, legacy, obs0 = checkpoint_pair(args.checkpoint, args.device)
            weights = {key: value.clone() for key, value in fixed.policy.state_dict().items()}
            def saved_observation(t):
                return TensorDict({key: value + .002 * torch.sin(
                    torch.arange(value.numel(), device=args.device).reshape_as(value) * .03 + t * .1)
                    for key, value in obs0.items()}, batch_size=obs0.batch_size)
            next_obs = fill_rollout([fixed, legacy], saved_observation, args.device)
            result["checkpoint_boundary"] = compare_boundary(fixed, legacy, next_obs)
            for key, value in fixed.policy.state_dict().items():
                assert_same(value, weights[key])
            assert hashlib.sha256(args.checkpoint.read_bytes()).hexdigest() == before_hash
            result.update(checkpoint_sha256=before_hash, checkpoint_unchanged=True, policy_weights_unchanged=True)
    result["result"] = "PASS"
    result["limit"] = "Synthetic observation sequences and existing checkpoint forward checks; no simulator run, optimization or fall-rate improvement claim. Old strict-resume manifests are not migrated."
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print("BOOTSTRAP_MEMORY_RESULT", json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
