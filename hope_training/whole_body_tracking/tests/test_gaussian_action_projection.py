"""CPU regression tests for q_des exploration debt and the real rsl_rl PPO update.

Run with the repository's setup_train_env.sh interpreter for the integration
tests; only Isaac configuration decorators are stubbed, not torch or rsl_rl.
"""

import importlib.util
import math
import pathlib
import sys
import types

import pytest

torch = pytest.importorskip("torch")
UTILS = pathlib.Path(__file__).resolve().parents[1] / "source/whole_body_tracking/whole_body_tracking/utils"


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


debt = load_module("gaussian_projection_math_test", UTILS / "gaussian_action_projection.py").gaussian_projection_debt


def test_exact_gaussian_debt_matches_numerical_integration():
    # Deterministic quadrature includes means inside, on, and beyond each rail.
    z = torch.linspace(-9.0, 9.0, 100001, dtype=torch.float64)
    density = torch.exp(-z.square() / 2) / math.sqrt(2 * math.pi)
    for mu, sigma in [(-2.0, 0.3), (-1.0, 0.8), (0.0, 0.5), (0.9, 0.2), (3.0, 1.2)]:
        x = mu + sigma * z
        expected = torch.trapezoid((x - x.clamp(-1.0, 1.0)).square() * density, z)
        actual = debt(torch.tensor(mu), torch.tensor(sigma), torch.tensor(-1.0), torch.tensor(1.0))
        torch.testing.assert_close(actual.double(), expected, rtol=2e-5, atol=1e-8)


def test_legal_mean_still_receives_tail_gradients():
    mean = torch.tensor([0.9, -0.9], requires_grad=True)
    std = torch.tensor([0.3, 0.3], requires_grad=True)
    result = debt(mean, std, torch.full((2,), -1.0), torch.ones(2)).sum()
    result.backward()
    assert result > 0
    assert mean.grad[0] > 0 and mean.grad[1] < 0
    assert bool((std.grad > 0).all())


def test_tiny_std_far_inside_interval_is_finite():
    mean = torch.tensor([0.0, -100.0, 100.0], requires_grad=True)
    std = torch.full((3,), 0.05, requires_grad=True)
    result = debt(mean, std, torch.full((3,), -1.0), torch.ones(3))
    result.sum().backward()
    assert bool(torch.isfinite(result).all())
    assert bool(torch.isfinite(mean.grad).all() & torch.isfinite(std.grad).all())
    assert result[0] == 0


@pytest.fixture
def policy_modules(monkeypatch):
    pytest.importorskip("rsl_rl")
    # Avoid importing Isaac's task registry/Kit for configuration-only bases.
    config_utils = types.ModuleType("isaaclab.utils")
    config_utils.configclass = lambda cls: cls
    configs = types.ModuleType("isaaclab_rl.rsl_rl")
    configs.RslRlPpoActorCriticCfg = object
    configs.RslRlPpoAlgorithmCfg = object
    monkeypatch.setitem(sys.modules, "isaaclab.utils", config_utils)
    monkeypatch.setitem(sys.modules, "isaaclab_rl.rsl_rl", configs)
    package = types.ModuleType("qdes_policy_test")
    package.__path__ = [str(UTILS)]
    monkeypatch.setitem(sys.modules, "qdes_policy_test", package)
    actor = load_module("qdes_policy_test.bounded_actor_critic", UTILS / "bounded_actor_critic.py")
    algorithm = load_module("qdes_policy_test.projection_ppo", UTILS / "projection_ppo.py")
    return actor.BoundedStdActorCritic, algorithm.ProjectionPPO


def make_policy(policy_modules):
    from tensordict import TensorDict
    actor_class, ppo_class = policy_modules
    obs = TensorDict({"policy": torch.zeros(4, 6)}, batch_size=[4])
    policy = actor_class(
        obs, {"policy": ["policy"], "critic": ["policy"]}, 3,
        actor_hidden_dims=[8], critic_hidden_dims=[8],
        init_noise_std=0.35, min_noise_std=0.05, max_noise_std=0.6,
        entropy_excluded_action_indices=[2],
    )
    algorithm = ppo_class(
        policy, device="cpu", num_learning_epochs=2, num_mini_batches=2,
        projection_aux_enabled=True, projection_aux_lambda=0.1,
        projection_aux_lambda_max=0.1, projection_aux_gradient_ratio_max=0.25,
        projection_aux_gaussian_tails=True,
    )
    return obs, policy, algorithm


def test_post_optimizer_step_projects_std_and_load_path_projects_before_sample(policy_modules):
    obs, policy, algorithm = make_policy(policy_modules)
    with torch.no_grad():
        policy.std.fill_(2.0)
    algorithm.optimizer.step()
    assert bool((policy.std <= 0.6).all())
    with torch.no_grad():
        policy.std.fill_(-1.0)
    actions = policy.act(obs)
    assert bool((policy.std >= 0.05).all())
    assert bool((actions[:, 2] == 0.0).all())


def test_real_ppo_rollout_and_update_preserves_bounds_and_caps_auxiliary(policy_modules):
    obs, policy, algorithm = make_policy(policy_modules)
    algorithm.init_storage("rl", 4, 4, obs, [3])
    lower = torch.tensor([[-0.2, -0.3, -1.0]]).expand(4, -1).clone()
    upper = torch.tensor([[0.4, 0.25, 1.0]]).expand(4, -1).clone()
    algorithm.bind_executable_action_projection(lower, upper, torch.tensor([True, True, False]))
    with torch.no_grad():
        for step in range(4):
            action = algorithm.act(obs)
            algorithm.process_env_step(obs, -(action[:, :2].square().sum(-1)), torch.zeros(4, dtype=torch.bool), {})
        algorithm.compute_returns(obs)
    stats = algorithm.update()
    assert all(math.isfinite(value) for value in stats.values())
    assert 0.0 <= stats["projection_gradient_ratio"] <= 0.25 + 1e-6
    assert stats["projection_aux"] > 0.0
    assert bool(((policy.std >= 0.05) & (policy.std <= 0.6)).all())


def test_gaussian_auxiliary_excludes_passive_channels(policy_modules):
    _, _, algorithm = make_policy(policy_modules)
    algorithm._projection_active = torch.tensor([True, True, False])
    means = torch.tensor([[0.1, 0.0, 100.0]], requires_grad=True)
    sigma = torch.full_like(means, 0.4, requires_grad=True)
    loss, _, _ = algorithm._projection_loss(means, -torch.ones_like(means), torch.ones_like(means), sigma)
    loss.backward()
    assert means.grad[0, 2] == 0.0
    assert sigma.grad[0, 2] == 0.0
