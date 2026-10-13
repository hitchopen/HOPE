"""Production reward math without an Isaac process; exercise contact/phase boundaries."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import torch


@pytest.fixture
def functions():
    path = Path(__file__).resolve().parents[1] / "source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/hope_rewards.py"
    names = {"_deadband_huber", "_record_metric_snapshot", "_quiet_command_phase",
             "waist_command_quiet_debt", "loaded_hip_tracking_debt"}
    nodes = [n for n in ast.parse(path.read_text()).body if getattr(n, "name", None) in names]
    scope = {"torch": torch, "math": math}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), "exec"), scope)
    return scope


def env():
    # Non-SDK order: the implementation must resolve names in action-column order.
    names = ["right_hip_roll_joint", "waist_pitch_joint", "left_hip_roll_joint",
             "waist_yaw_joint", "waist_roll_joint"]
    robot = SimpleNamespace(data=SimpleNamespace(joint_pos=torch.zeros(4, 5)))
    action = SimpleNamespace(_joint_names=names, _asset=robot,
                             _select_action_joints=lambda x: x,
                             _qdes_executed=torch.zeros(4, 5),
                             executed_qdes_delta=torch.zeros(4, 5))
    racket = SimpleNamespace(time_to_strike=torch.tensor([0., 0., .5, -.4]), metrics={})
    motion = SimpleNamespace(in_hold=torch.tensor([True, False, False, False]))
    force = torch.zeros(4, 2, 3); force[:, :, 2] = 100.
    sensor = SimpleNamespace(data=SimpleNamespace(net_forces_w=force))
    terms = {"motion": motion, "racket_target": racket}
    e = SimpleNamespace(action_manager=SimpleNamespace(get_term=lambda _: action),
                        command_manager=SimpleNamespace(get_term=terms.__getitem__),
                        scene=SimpleNamespace(sensors={"contact": sensor}))
    return e, action, racket, motion, sensor


def test_waist_uses_radians_and_exogenous_phase_not_actual_speed(functions):
    e, a, cmd, _, _ = env()
    a.executed_qdes_delta[:, 3] = .006
    cost = functions["waist_command_quiet_debt"](e)
    torch.testing.assert_close(cost, torch.tensor([1/6, 0., 1/6, 1/6]))
    a.executed_qdes_delta *= -1
    torch.testing.assert_close(functions["waist_command_quiet_debt"](e), cost)
    snapshot = cmd.metrics["waist_command_step_max_rad"].clone()
    a.executed_qdes_delta.zero_()
    torch.testing.assert_close(cmd.metrics["waist_command_step_max_rad"], snapshot)
    assert functions["waist_command_quiet_debt"](e).eq(0).all()


def test_hip_load_gate_bilateral_error_and_post_physics_measurement(functions):
    e, a, cmd, _, sensor = env()
    cfg = SimpleNamespace(name="contact", body_ids=[0, 1])
    a._qdes_executed[:, 2] = .25
    sensor.data.net_forces_w[2, 0, 2] = 0.  # A recovery step is not pinned.
    expected = torch.tensor([.6, 0., 0., .6])
    torch.testing.assert_close(functions["loaded_hip_tracking_debt"](e, cfg), expected)
    a._qdes_executed[:, 2] = 0.
    a._qdes_executed[:, 0] = -.25
    torch.testing.assert_close(functions["loaded_hip_tracking_debt"](e, cfg), expected)
    # Actual position catches up within the 0.12 rad useful preload band.
    a._asset.data.joint_pos[:, 0] = -.15
    assert functions["loaded_hip_tracking_debt"](e, cfg).eq(0).all()


@pytest.mark.parametrize("std,margin", [(0., .1), (.1, -.1), (float('nan'), .1), (.1, float('inf'))])
def test_invalid_scales_fail(functions, std, margin):
    e, *_ = env()
    with pytest.raises(ValueError):
        functions["waist_command_quiet_debt"](e, std=std, margin=margin)
    with pytest.raises(ValueError):
        functions["loaded_hip_tracking_debt"](e, SimpleNamespace(name="contact", body_ids=[0, 1]), std=std, margin=margin)
