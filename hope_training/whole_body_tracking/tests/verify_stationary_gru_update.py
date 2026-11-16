"""CPU check for the StationaryRally GRU recipe and real recurrent PPO update.

Run with Isaac Sim's bundled Python. Isaac Lab config classes are stubbed so
this check needs neither the simulator nor a robot scene.
"""

from __future__ import annotations

import sys
import types
import ast
import os
from pathlib import Path
from types import SimpleNamespace

import torch
from hydra import compose, initialize_config_dir
from omegaconf import OmegaConf
from tensordict import TensorDict


PACKAGE = Path(__file__).resolve().parents[1] / "source/whole_body_tracking/whole_body_tracking"
root = types.ModuleType("whole_body_tracking")
root.__path__ = [str(PACKAGE)]
utils = types.ModuleType("whole_body_tracking.utils")
utils.__path__ = [str(PACKAGE / "utils")]
sys.modules["whole_body_tracking"] = root
sys.modules["whole_body_tracking.utils"] = utils
isaaclab = types.ModuleType("isaaclab")
isaaclab.__path__ = []
isaaclab_utils = types.ModuleType("isaaclab.utils")
isaaclab_utils.configclass = lambda cls: cls
sys.modules["isaaclab"] = isaaclab
sys.modules["isaaclab.utils"] = isaaclab_utils
isaaclab_rl = types.ModuleType("isaaclab_rl")
isaaclab_rl.__path__ = []
isaaclab_rl_rsl = types.ModuleType("isaaclab_rl.rsl_rl")
for name in (
    "RslRlPpoActorCriticCfg", "RslRlPpoActorCriticRecurrentCfg", "RslRlPpoAlgorithmCfg",
):
    setattr(isaaclab_rl_rsl, name, type(name, (), {}))
sys.modules["isaaclab_rl"] = isaaclab_rl
sys.modules["isaaclab_rl.rsl_rl"] = isaaclab_rl_rsl

from whole_body_tracking.utils.bounded_actor_critic import BoundedStdActorCriticRecurrent
from whole_body_tracking.utils.recurrent_projection_ppo import (
    RecurrentProjectionPPO,
)
from whole_body_tracking.utils.robust_ball_distribution import validate_training_contract
from whole_body_tracking.utils.balanced_hold_contract import validate_balanced_hold_algorithm


def check_runner_mapping():
    source = PACKAGE / "utils/ppo_cfg.py"
    function = next(node for node in ast.parse(source.read_text()).body
                    if isinstance(node, ast.FunctionDef) and node.name == "runner_kwargs")
    namespace = {"os": os}
    for name in (
        "RslRlPpoAlgorithmCfg", "RslRlAdamWPpoAlgorithmCfg",
        "RslRlProjectionPpoAlgorithmCfg", "RslRlRecurrentProjectionPpoAlgorithmCfg",
        "RslRlPpoActorCriticCfg", "RslRlBoundedStdActorCriticCfg",
        "RslRlPpoActorCriticRecurrentCfg", "RslRlBoundedStdActorCriticRecurrentCfg",
    ):
        namespace[name] = type(name, (), {"__init__": lambda self, **kw: self.__dict__.update(kw)})
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), "exec"), namespace)
    with initialize_config_dir(config_dir=str(PACKAGE.parents[2] / "cfg"), version_base=None):
        baseline = compose(
            config_name="train",
            overrides=["task=HOPEPingPongStationaryRally", "algo=ppo_balanced_hold"],
        )
        candidate = compose(
            config_name="train",
            overrides=["task=HOPEPingPongStationaryRally", "algo=ppo_stationary_gru"],
        )
    baseline = OmegaConf.to_container(baseline, resolve=True)
    candidate = OmegaConf.to_container(candidate, resolve=True)
    validate_training_contract(candidate["task"], candidate["algo"])
    validate_balanced_hold_algorithm(
        candidate["task"]["build_contract"]["reward_revision"],
        candidate["algo"]["algorithm"],
    )
    mlp, gru = baseline["algo"], candidate["algo"]
    mapped = namespace["runner_kwargs"](gru, "test")
    assert mapped["logger"] == mlp["runner"]["logger"] == "wandb"
    assert mapped["num_steps_per_env"] == mlp["runner"]["num_steps_per_env"] == 64
    assert type(mapped["policy"]).__name__ == "RslRlBoundedStdActorCriticRecurrentCfg"
    assert type(mapped["algorithm"]).__name__ == "RslRlRecurrentProjectionPpoAlgorithmCfg"
    assert mapped["policy"].actor_hidden_dims == mlp["policy"]["actor_hidden_dims"]
    assert mapped["policy"].critic_hidden_dims == mlp["policy"]["critic_hidden_dims"]
    assert mapped["policy"].rnn_hidden_dim == 110
    assert mapped["policy"].rnn_num_layers == 1
    for key in ("min_noise_std", "max_noise_std"):
        assert getattr(mapped["policy"], key) == mlp["policy"][key]
    assert not gru["algorithm"]["settled_aux_enabled"]
    assert baseline["task"] == candidate["task"]
    assert {k: v for k, v in mlp.items() if k not in ("policy", "name")} == {
        k: v for k, v in gru.items() if k not in ("policy", "name")
    }
    for key in ("learning_rate", "projection_aux_enabled", "projection_aux_lambda",
                "projection_aux_lambda_max", "projection_aux_gradient_ratio_max",
                "projection_aux_gaussian_tails", "settled_aux_enabled",
                "settled_aux_start_steps", "settled_aux_ramp_steps", "settled_aux_smooth_coef",
                "settled_aux_tracking_coef", "settled_aux_gradient_ratio_max"):
        assert getattr(mapped["algorithm"], key) == mlp["algorithm"][key]


def observation(value: float = 0.0) -> TensorDict:
    return TensorDict(
        {"policy": torch.full((4, 8), value), "critic": torch.full((4, 9), value)},
        batch_size=[4],
    )


def check_gru_resume_memory(policy):
    """Exercise the runner's actual hidden-state save/restore methods without Isaac."""
    source = PACKAGE / "utils/my_on_policy_runner.py"
    runner_class = next(
        node for node in ast.parse(source.read_text(encoding="utf-8")).body
        if isinstance(node, ast.ClassDef) and node.name == "MotionOnPolicyRunner"
    )
    methods = [
        node for node in runner_class.body
        if isinstance(node, ast.FunctionDef) and node.name in {
            "_capture_policy_memory_state", "_restore_policy_memory_state"
        }
    ]
    harness_class = ast.ClassDef(
        name="MemoryHarness", bases=[], keywords=[], body=methods, decorator_list=[]
    )
    namespace = {"torch": torch}
    exec(compile(ast.fix_missing_locations(ast.Module(
        body=[harness_class], type_ignores=[]
    )), str(source), "exec"), namespace)
    harness = namespace["MemoryHarness"]()
    harness.alg = SimpleNamespace(policy=policy)
    harness.env = SimpleNamespace(num_envs=4)
    harness.device = "cpu"
    harness._clone_resume_tree = lambda value: (
        True, value.detach().cpu().clone() if torch.is_tensor(value) else value
    )
    for offset, name in enumerate(("memory_a", "memory_c")):
        memory = getattr(policy, name)
        memory.reset(hidden_state=torch.full((1, 4, 8), float(offset + 1)))
    saved = harness._capture_policy_memory_state()
    assert set(saved) == {"memory_a", "memory_c"}
    for name in saved:
        getattr(policy, name).reset(hidden_state=torch.zeros_like(saved[name]))
    harness._restore_policy_memory_state(saved)
    for name in saved:
        torch.testing.assert_close(getattr(policy, name).hidden_state, saved[name])
    try:
        harness._restore_policy_memory_state({"memory_a": saved["memory_a"]})
    except RuntimeError:
        pass
    else:
        raise AssertionError("incomplete GRU checkpoint was accepted")


def main():
    check_runner_mapping()
    torch.manual_seed(7)
    policy = BoundedStdActorCriticRecurrent(
        obs=observation(), obs_groups={"policy": ["policy"], "critic": ["critic"]},
        num_actions=4, actor_hidden_dims=[16], critic_hidden_dims=[16],
        rnn_type="gru", rnn_hidden_dim=8, rnn_num_layers=1,
        init_noise_std=0.35, min_noise_std=0.05, max_noise_std=0.6,
        entropy_excluded_action_indices=[3],
    )
    with torch.no_grad():
        policy.std.fill_(2.0)
    first = policy.act(observation())
    assert torch.all(first[:, 3] == 0.0)
    assert torch.all(policy.std <= 0.6)
    policy.reset()
    with torch.no_grad():
        policy.std.zero_()
    policy.act(observation())
    assert torch.all(policy.std >= 0.05)
    policy.reset()

    algorithm = RecurrentProjectionPPO(
        policy=policy, device="cpu", num_learning_epochs=1, num_mini_batches=2,
        learning_rate=0.001, projection_aux_enabled=True,
        projection_aux_lambda=0.1, projection_aux_lambda_max=0.1,
        projection_aux_gradient_ratio_max=0.25,
        projection_aux_gaussian_tails=True, settled_aux_enabled=False,
        settled_aux_smooth_coef=0.0, settled_aux_gradient_ratio_max=0.0,
    )
    algorithm.init_storage("rl", 4, 4, observation(), [4])
    algorithm.bind_executable_action_projection(
        torch.full((4, 4), -0.02), torch.full((4, 4), 0.02),
        torch.tensor([True, True, True, False]),
    )
    with torch.inference_mode():
        for step in range(4):
            obs = observation(step * 0.1)
            actions = algorithm.act(obs)
            assert torch.all(actions[:, 3] == 0.0)
            dones = torch.zeros(4, dtype=torch.bool)
            if step == 1:
                dones[0] = True
            algorithm.process_env_step(obs, torch.ones(4), dones, {})
        algorithm.compute_returns(observation(0.4))
    result = algorithm.update()
    assert result["projection_aux"] > 0.0
    assert result["settled_mean_smooth"] == 0.0
    assert 0.0 <= result["projection_gradient_ratio"] <= 0.25 + 1.0e-6
    assert result["settled_mean_gradient_ratio"] == 0.0
    assert torch.all((policy.std >= 0.05) & (policy.std <= 0.6))
    assert algorithm.storage.step == 0
    check_gru_resume_memory(policy)
    print("StationaryRally GRU bounded-std + balanced-hold projection PPO update: PASS")


if __name__ == "__main__":
    main()
