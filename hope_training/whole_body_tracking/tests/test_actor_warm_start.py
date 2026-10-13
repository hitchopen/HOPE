import copy
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
import torch

path = Path(__file__).resolve().parents[1] / "source/whole_body_tracking/whole_body_tracking/utils/actor_warm_start.py"
spec = importlib.util.spec_from_file_location("actor_warm_start", path)
module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)


def fixture():
    actor = torch.nn.Sequential(torch.nn.Linear(3, 2))
    policy = SimpleNamespace(actor=actor, critic=torch.nn.Linear(3, 1),
                             std=torch.nn.Parameter(torch.tensor([.2, .2])),
                             min_noise_std=.05, max_noise_std=.6, exploration_contract="same")
    state = {"actor."+k: torch.ones_like(v) for k, v in actor.state_dict().items()}
    state["std"] = torch.tensor([.1, .15])
    task = dict(gym_task="task", actor_obs_contract="obs", action={"contract": "affine"},
                motion_file="fh.npz", motion_file_2="bh.npz",
                build_contract={"actor_observation_dim": 3, "action_dim": 2})
    ckpt = {"model_state_dict": state, "hope_exact_resume_state": {"policy_exploration_contract": "same"}}
    return policy, ckpt, task


def test_initializes_actor_std_only_and_leaves_fresh_critic():
    policy, ckpt, task = fixture()
    critic = copy.deepcopy(policy.critic.state_dict())
    module.initialize_actor(policy, ckpt, task, copy.deepcopy(task))
    assert all(v.eq(1).all() for v in policy.actor.state_dict().values())
    torch.testing.assert_close(policy.std, ckpt["model_state_dict"]["std"])
    for k, v in policy.critic.state_dict().items(): torch.testing.assert_close(v, critic[k])


@pytest.mark.parametrize("fault", ["abi", "dimension", "std", "nan", "normalizer", "exploration"])
def test_rejects_incompatible_or_invalid_source_before_mutating_actor(fault):
    policy, ckpt, task = fixture(); target = copy.deepcopy(task)
    if fault == "abi": target["action"]["contract"] = "different"
    elif fault == "dimension": target["build_contract"]["action_dim"] = 4
    elif fault == "std": ckpt["model_state_dict"]["std"][0] = 0.
    elif fault == "nan": ckpt["model_state_dict"]["actor.0.weight"][0, 0] = float("nan")
    elif fault == "normalizer": ckpt["model_state_dict"]["normalizer.mean"] = torch.zeros(3)
    else: ckpt["hope_exact_resume_state"]["policy_exploration_contract"] = "different"
    before = copy.deepcopy(policy.actor.state_dict())
    with pytest.raises(ValueError): module.initialize_actor(policy, ckpt, task, target)
    for k, v in policy.actor.state_dict().items(): torch.testing.assert_close(v, before[k])
