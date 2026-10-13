"""Run the production logging methods without importing IsaacLab or constructing a simulator."""
from __future__ import annotations

import ast
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import torch


_PATH = Path(__file__).resolve().parents[1] / "source/whole_body_tracking/whole_body_tracking/utils/my_on_policy_runner.py"
_TREE = ast.parse(_PATH.read_text())
_CLASS = next(node for node in _TREE.body if isinstance(node, ast.ClassDef) and node.name == "MotionOnPolicyRunner")
_METHODS = {"_log_live_metrics", "_mean_tensor", "_log_scalar", "_log_scalar_mapping"}
_BODY = [node for node in _CLASS.body if isinstance(node, ast.FunctionDef) and node.name in _METHODS]
_MODULE = ast.Module(body=[ast.ClassDef(name="Runner", bases=[], keywords=[], body=_BODY, decorator_list=[])], type_ignores=[])
_NAMESPACE = {"torch": torch, "math": math}
exec(compile(ast.fix_missing_locations(_MODULE), str(_PATH), "exec"), _NAMESPACE)
Runner = _NAMESPACE["Runner"]


def _runner():
    written = {}
    cmd = SimpleNamespace(
        metrics={"value": torch.tensor([1., 3.]), "scalar": 7., "bad": torch.tensor([float("nan")]), "empty": torch.empty(0)},
        command_counter=torch.tensor([2, 4]),
        instrumentation_scalars=lambda: {"matrix": torch.tensor(9.)},
        foundation_instrumentation_scalars=lambda: {"foundation": torch.tensor(10.)},
    )
    action = SimpleNamespace(safety_instrumentation_scalars=lambda: {"safe": torch.tensor(8.)})
    env = SimpleNamespace(
        command_manager=SimpleNamespace(active_terms=["racket_target"], get_term=lambda _: cmd),
        reward_manager=SimpleNamespace(active_terms=["support"], _step_reward=torch.tensor([[2.], [4.]])),
        reward_buf=torch.tensor([3., 5.]),
        termination_manager=SimpleNamespace(active_terms=["fall"], dones=torch.tensor([True, False]), terminated=torch.tensor([True, False]), time_outs=torch.tensor([False, False]), get_term=lambda _: torch.tensor([True, False])),
        action_manager=SimpleNamespace(action=torch.tensor([[1., -3.], [2., 0.]]), prev_action=torch.zeros(2, 2), get_term=lambda _: action),
        episode_length_buf=torch.tensor([10, 20]),
        common_step_counter=123,
    )
    runner = Runner()
    runner.env = SimpleNamespace(unwrapped=env)
    runner.alg = SimpleNamespace(policy=SimpleNamespace(std=torch.tensor([0.2, 0.4]), min_noise_std=0.1, max_noise_std=0.5, distribution=object(), executable_gaussian_entropy=torch.tensor([1., 3.]), all_action_gaussian_entropy=torch.tensor([2., 4.])))
    runner.writer = SimpleNamespace(add_scalar=lambda tag, value, step: written.__setitem__((tag, step), value))
    return runner, written


@pytest.mark.parametrize("step", [1, 10, 50])
def test_all_live_values_and_instrumentation_frequencies_are_preserved(step):
    runner, written = _runner()
    runner._log_live_metrics(step, executed_entropy=torch.tensor([2., 4.]))
    values = {tag: value for (tag, when), value in written.items() if when == step}
    expected = {
        "Live/racket_target/value": 2., "Live/racket_target/scalar": 7., "Live/racket_target/command_counter": 3.,
        "Live/Reward/total": 4., "Live/Reward/support": 3.,
        "Live/Termination/done_rate": .5, "Live/Termination/terminated_rate": .5,
        "Live/Termination/timeout_rate": 0., "Live/Termination/fall": .5,
        "Live/Action/mean_abs": 1.5, "Live/Action/max_abs": 2.5,
        "Live/Action/delta_mean_abs": 1.5, "Live/Action/delta_max_abs": 2.5,
        "Live/Policy/physical_noise_std_parameter_mean": .3, "Live/Policy/physical_noise_std_parameter_max": .4,
        "Live/Policy/effective_noise_std_mean": .3, "Live/Policy/effective_noise_std_max": .4,
        "Live/Policy/noise_std_bound_fraction_mean": .5, "Live/Policy/noise_std_bound_fraction_max": .75,
        "Live/Policy/noise_std_at_lower_bound_fraction": 0., "Live/Policy/noise_std_at_upper_bound_fraction": 0.,
        "Live/Policy/executed_entropy_mean": 3., "Live/Policy/executable_gaussian_entropy_mean": 2.,
        "Live/Policy/all_action_gaussian_entropy_mean": 3., "Live/Policy/passive_gaussian_entropy_mean": 1.,
        "Live/Env/episode_length": 15., "Live/Env/common_step_counter": 123.,
    }
    if step % 10 == 0:
        expected.update({"Instrumentation/racket_target/matrix": 9., "Instrumentation/qdes_safety/safe": 8.})
    if step % 50 == 0:
        expected["Instrumentation/foundation"] = 10.
    assert values == pytest.approx(expected)


def test_live_metrics_share_one_device_transfer(monkeypatch):
    runner, _ = _runner()
    transfers = []
    original_cpu = torch.Tensor.cpu
    def tracked_cpu(value, *args, **kwargs):
        transfers.append(tuple(value.shape))
        return original_cpu(value, *args, **kwargs)
    monkeypatch.setattr(torch.Tensor, "cpu", tracked_cpu)
    runner._log_live_metrics(1, executed_entropy=torch.tensor([2., 4.]))
    assert len(transfers) == 1
    assert transfers[0][0] > 20
