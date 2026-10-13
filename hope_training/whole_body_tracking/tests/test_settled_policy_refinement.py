"""Deterministic refinement tests, including a real rsl_rl rollout/update."""
from types import SimpleNamespace as NS
import math

import pytest
import torch

from test_gaussian_action_projection import policy_modules, load_module, UTILS

refinement = load_module("settled_refinement_math_test", UTILS / "settled_policy_refinement.py")


def fake_environment(n=4):
    names = [f"{side}_{j}_joint" for side in ("left", "right")
             for j in ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")]
    names += [f"waist_{j}_joint" for j in ("yaw", "roll", "pitch")]
    names += [f"unused_{i}" for i in range(16)]
    action = NS(_joint_names=names, _action_contract="v11_affine_safe_qdes_v1",
                _offset=torch.zeros(n, 31), _scale=torch.full((n, 31), .3),
                _safe_lo=torch.full((n, 31), -2.), _safe_hi=torch.full((n, 31), 2.),
                _asset=NS(data=NS(joint_pos=torch.zeros(n, 31))))
    action._decoder_parameter_rows = lambda value, ids, name: value[ids]
    action._select_action_joints = lambda value: value
    sensor = NS(data=NS(net_forces_w=torch.zeros(n, 2, 3)))
    sensor.data.net_forces_w[:, :, 2] = 100
    sensor.find_bodies = lambda names, preserve_order: ([0, 1], names)
    motion = NS(in_hold=torch.ones(n, dtype=torch.bool))
    env = NS(common_step_counter=0, action_manager=NS(get_term=lambda _: action),
             command_manager=NS(get_term=lambda _: motion),
             scene=NS(sensors={"contact_forces": sensor}))
    return env, action, sensor, motion


def context():
    env, action, sensor, motion = fake_environment()
    ctx = refinement.SettledPolicyRefinement(
        env, NS(num_transitions_per_env=4, num_envs=4, device="cpu"),
        start_steps=448000, ramp_steps=128000,
    )
    return ctx, env, action, sensor, motion


@pytest.mark.parametrize("steps,expected", [(0, 0), (448000, 0), (512000, .5), (576000, 1)])
def test_schedule_and_physical_support_gate(steps, expected):
    ctx, env, _, sensor, motion = context()
    env.common_step_counter = steps
    motion.in_hold[1] = False
    sensor.data.net_forces_w[2, 0, 2] = 20
    ctx.capture(0)
    torch.testing.assert_close(ctx.gate[0], torch.tensor([expected, 0, 0, expected]).float())


def test_decoder_uses_each_environment_calibration_and_limits():
    ctx, *_ = context()
    ctx.offset[2] = .4
    ctx.upper[2] = .5
    # A minibatch may be larger than num_envs and repeat the same environment.
    ids = torch.tensor([2, 0, 2, 3, 1, 0])
    decoded = ctx.decode(torch.ones(6, 31), ids)
    torch.testing.assert_close(decoded[:, 0], torch.tensor([.5, .3, .5, .3, .3, .3]))


def test_deadband_bounded_loss_conditional_normalization_and_ramp():
    error = torch.tensor([0., .14, .15, .16, 1., 100.], requires_grad=True)
    debt = refinement.bounded_debt(error, .15, .1)
    assert (debt[:3] == 0).all() and (debt >= 0).all() and (debt < 1).all()
    debt.sum().backward()
    assert torch.isfinite(error.grad).all()
    v = torch.tensor([.4, 100., 100., 100.])
    assert refinement.conditional_mean(v, torch.tensor([.5, 0, 0, 0])) == pytest.approx(.2)
    assert refinement.conditional_mean(v, torch.zeros(4)) == 0


def test_only_local_means_receive_gradients():
    current = torch.full((2, 31), .3, requires_grad=True)
    previous = torch.zeros_like(current, requires_grad=True)
    smooth, tracking, _ = refinement.refinement_losses(
        current, previous, torch.zeros(2), torch.ones(2), torch.ones(2), list(range(15)), 14,
    )
    (smooth + tracking).backward()
    assert current.grad[:, :15].abs().sum() > 0
    assert current.grad[:, 15:].abs().sum() == 0
    assert previous.grad[:, 15:].abs().sum() == 0


def build_algorithm(policy_modules, enabled=True):
    from tensordict import TensorDict
    actor_class, ppo_class = policy_modules
    obs = TensorDict({"policy": torch.randn(4, 6)}, batch_size=[4])
    policy = actor_class(obs, {"policy": ["policy"], "critic": ["policy"]}, 31,
                         actor_hidden_dims=[8], critic_hidden_dims=[8],
                         init_noise_std=.35, min_noise_std=.05, max_noise_std=.6)
    alg = ppo_class(policy, device="cpu", num_learning_epochs=2, num_mini_batches=2,
                    projection_aux_enabled=True, projection_aux_lambda=.1,
                    projection_aux_lambda_max=.1, projection_aux_gradient_ratio_max=.25,
                    projection_aux_gaussian_tails=True, settled_aux_enabled=enabled,
                    settled_aux_start_steps=0, settled_aux_ramp_steps=1)
    alg.init_storage("rl", 4, 4, obs, [31])
    alg.bind_executable_action_projection(-torch.ones(4, 31), torch.ones(4, 31),
                                          torch.ones(31, dtype=torch.bool))
    env, action, _, _ = fake_environment()
    env.common_step_counter = 1
    action._asset.data.joint_pos[:, 14] = -1.
    alg.bind_settled_refinement(env)
    return obs, policy, alg


def test_temporal_pairs_exclude_resets_and_rollout_start_and_leave_std_untouched(policy_modules):
    obs, policy, alg = build_algorithm(policy_modules)
    ctx = alg._settled_refinement
    ctx.gate.fill_(1)
    ctx.actual_waist.fill_(-1)
    observations = obs.expand(4, 4).reshape(16)
    # The first row has no previous step. Env 0 resets after step 0.
    dones = torch.zeros(4, 4, 1, dtype=torch.bool)
    dones[0, 0] = True
    indices = torch.tensor([0, 4, 5])
    policy.act(observations[indices])
    smooth, tracking, stats = ctx.losses(policy, observations, policy.action_mean, indices, dones)
    assert stats["pair_fraction"] == pytest.approx(1/3)
    assert torch.autograd.grad(smooth + tracking, policy.std, allow_unused=True)[0] is None
    assert tracking == 0
    assert stats["waist_qdes_offset_abs_rad"] > 0


@pytest.mark.parametrize("ramp_steps,expected_cap", [(1, .05), (2, .025)])
def test_real_ppo_update_caps_new_gradient_and_preserves_finite_policy(policy_modules, ramp_steps, expected_cap):
    obs, policy, alg = build_algorithm(policy_modules)
    alg._settled_refinement.ramp_steps = ramp_steps
    # Force clipping to exercise the ceiling, not merely a small uncapped gradient.
    alg.settled_aux_smooth_coef = 100.
    with torch.no_grad():
        for step in range(4):
            action = alg.act(obs)
            obs = obs.clone()
            obs["policy"] = torch.randn(4, 6)
            alg.process_env_step(obs, -action.square().mean(-1), torch.zeros(4, dtype=torch.bool), {})
        alg.compute_returns(obs)
    stats = alg.update()
    assert all(math.isfinite(x) for x in stats.values())
    # This is a minibatch mean: zero/deadband auxiliary gradients need not
    # saturate the ceiling even with a large coefficient. Check the actual
    # safety invariant and that clipping was exercised, not equality to a cap.
    assert 0 < stats["settled_mean_gradient_ratio"] <= expected_cap + 1e-7
    assert stats["settled_mean_gradient_multiplier"] < 1.
    assert "settled_mean_tracking" not in stats
    assert "settled_mean_waist_qdes_offset_abs_rad" not in stats
    assert stats["settled_mean_smooth"] > 0
    assert stats["settled_mean_pair_fraction"] == pytest.approx(.75)
    assert all(torch.isfinite(p).all() for p in policy.parameters())


def test_inactive_refinement_is_identical_to_original_ppo(policy_modules):
    states = []
    for enabled in (False, True):
        torch.manual_seed(42)
        obs, policy, alg = build_algorithm(policy_modules, enabled=enabled)
        if enabled:
            alg._settled_refinement.env.common_step_counter = 0
            def inactive_loss_must_not_run(*args, **kwargs):
                raise AssertionError("zero curriculum must skip auxiliary forward/backward")
            alg._settled_refinement.losses = inactive_loss_must_not_run
        with torch.no_grad():
            for _ in range(4):
                action = alg.act(obs)
                alg.process_env_step(obs, -action.square().mean(-1), torch.zeros(4, dtype=torch.bool), {})
            alg.compute_returns(obs)
        stats = alg.update()
        if enabled:
            assert stats["settled_mean_gradient_ratio"] == 0
        states.append({k: v.clone() for k, v in policy.state_dict().items()})
    for name in states[0]:
        torch.testing.assert_close(states[0][name], states[1][name], rtol=0, atol=0)
