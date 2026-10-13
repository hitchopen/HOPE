"""Exercise the production phase telemetry without booting Isaac Sim."""
import ast
from pathlib import Path
from types import SimpleNamespace

import pytest
import torch

PATH = Path(__file__).resolve().parents[1] / "source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/hope_commands.py"
owner = next(n for n in ast.parse(PATH.read_text()).body if isinstance(n, ast.ClassDef) and n.name == "RacketTargetCommand")
method = next(n for n in owner.body if isinstance(n, ast.FunctionDef) and n.name == "_update_qdes_phase_telemetry")
namespace = {"torch": torch}
exec(compile(ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])), str(PATH), "exec"), namespace)
update = namespace[method.name]


def test_phase_means_empty_phase_history_and_single_transfer(monkeypatch):
    hold = torch.tensor([True, True, False, False, False, False])
    action = SimpleNamespace(executed_qdes_delta=torch.arange(12.).reshape(6, 2) / 10,
                             executed_qdes_second_difference=torch.arange(12.).reshape(6, 2) / 30,
                             _qdes_reversal_fraction=torch.arange(6.) / 6)
    channels = {"qdes_step_rms_rad": action.executed_qdes_delta.square().mean(-1).sqrt(),
                "qdes_second_difference_rms_rad": action.executed_qdes_second_difference.square().mean(-1).sqrt(),
                "qdes_reversal_hz": action._qdes_reversal_fraction / .02}
    cmd = SimpleNamespace(_env=SimpleNamespace(step_dt=.02, action_manager=SimpleNamespace(get_term=lambda _: action)),
                          _motion=lambda: SimpleNamespace(in_hold=hold),
                          time_to_strike=torch.tensor([1., 1., .1, -.2, -.5, .9]),
                          metrics={}, _qdes_phase_telemetry_last={})
    for phase in ("hold", "strike", "recovery"):
        for channel in channels:
            key = f"{channel}_{phase}"
            cmd.metrics[key] = torch.zeros(6)
            cmd._qdes_phase_telemetry_last[key] = 0.
    transfers = []
    original_cpu = torch.Tensor.cpu
    def cpu(value, *args, **kwargs):
        transfers.append(tuple(value.shape))
        return original_cpu(value, *args, **kwargs)
    monkeypatch.setattr(torch.Tensor, "cpu", cpu)
    update(cmd)
    for phase, ids in (("hold", [0, 1]), ("strike", [2, 3]), ("recovery", [4])):
        for channel, values in channels.items():
            key = f"{channel}_{phase}"
            assert cmd._qdes_phase_telemetry_last[key] == pytest.approx(values[ids].mean().item())
            torch.testing.assert_close(cmd.metrics[key], values[ids].mean().expand(6))
    history = dict(cmd._qdes_phase_telemetry_last)
    hold.zero_()
    cmd.time_to_strike.fill_(1.)
    # A reset may clear metric rows, but an empty phase must republish its last sample.
    for value in cmd.metrics.values():
        value.zero_()
    action.executed_qdes_delta.fill_(float("nan"))
    update(cmd)
    assert cmd._qdes_phase_telemetry_last == history
    for key, value in cmd.metrics.items():
        torch.testing.assert_close(value, torch.full((6,), history[key]))
    assert transfers == [(3, 4), (3, 4)]
