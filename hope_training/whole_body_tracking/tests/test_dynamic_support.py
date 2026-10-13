"""CPU physics-math regressions; no Isaac runtime needed."""
from __future__ import annotations

import importlib.util
import ast
from pathlib import Path
from types import SimpleNamespace

import torch


_PATH = Path(__file__).resolve().parents[1] / "source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/recovery_safe_set.py"
_SPEC = importlib.util.spec_from_file_location("hope_support_math", _PATH)
support = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(support)


def test_stationary_lower_rail_and_escaping_actual_limit_are_not_invisible():
    q = torch.tensor([[-0.95, -1.01, 1.01, 0.0]])
    qd = torch.tensor([[0.0, 0.1, -0.1, 0.0]])
    violation, _, available = support.actual_q_stopping_violation(
        q, qd, torch.full_like(q, -1), torch.ones_like(q), torch.ones_like(q),
    )
    torch.testing.assert_close(violation, torch.tensor([[0.5, 1.0, 1.0, 0.0]]))
    assert available[0, 1] < 0 and available[0, 2] < 0


def test_outward_momentum_predicts_limit_before_position_reaches_rail():
    q = torch.tensor([[0.5, -0.5]])
    qd = torch.tensor([[1.0, -1.0]])
    violation, stopping, available = support.actual_q_stopping_violation(
        q, qd, torch.full_like(q, -1), torch.ones_like(q), torch.ones_like(q),
    )
    assert torch.all(violation > 0.99)
    assert torch.all(stopping > available)


def _state():
    return (
        torch.tensor([[0.0, 0.0, 1.0]]),
        torch.zeros(1, 3),
        torch.tensor([[[0.0, -0.13, 0.0], [0.0, 0.13, 0.0]]]),
        torch.ones(1, 2, dtype=torch.bool),
    )


def test_capture_point_detects_tipping_momentum_while_com_is_centered():
    position, velocity, feet, contact = _state()
    still, _ = support.capture_point_support_excess(position, velocity, feet, contact)
    velocity[:, 0] = 0.7
    moving, _ = support.capture_point_support_excess(position, velocity, feet, contact)
    assert still.item() == 0.0
    assert moving.item() > 0.1


def test_single_support_ignores_the_airborne_foot():
    position, velocity, feet, contact = _state()
    contact[:, 1] = False
    position[:, 1] = -0.13
    stable, flight = support.capture_point_support_excess(position, velocity, feet, contact)
    assert stable.item() == 0.0 and not flight.item()
    position[:, 1] = 0.13
    unstable, _ = support.capture_point_support_excess(position, velocity, feet, contact)
    assert unstable.item() > 0.17
    feet[:, 1, 1] = 10.0
    unchanged, _ = support.capture_point_support_excess(position, velocity, feet, contact)
    torch.testing.assert_close(unstable, unchanged)


def test_step_that_catches_momentum_reduces_debt_without_zeroing_velocity():
    position, velocity, feet, contact = _state()
    velocity[:, 1] = 1.0
    before, _ = support.capture_point_support_excess(position, velocity, feet, contact)
    feet[:, 1, 1] = 0.35
    after, _ = support.capture_point_support_excess(position, velocity, feet, contact)
    assert before.item() > 0.1 and after.item() == 0.0


def test_flight_is_explicit_and_support_math_is_translation_invariant():
    position, velocity, feet, contact = _state()
    velocity[:, 0] = 0.8
    before, _ = support.capture_point_support_excess(position, velocity, feet, contact)
    offset = torch.tensor([10.0, -20.0, 3.0])
    after, _ = support.capture_point_support_excess(position + offset, velocity, feet + offset, contact)
    torch.testing.assert_close(before, after)
    excess, flight = support.capture_point_support_excess(position, velocity, feet, torch.zeros_like(contact))
    assert torch.isfinite(excess).all() and flight.item()
    assert torch.all(excess >= before)


def test_com_cache_keeps_live_world_pose_and_refreshes_after_property_change():
    path = _PATH.with_name("hope_rewards.py")
    node = next(n for n in ast.parse(path.read_text()).body
                if getattr(n, "name", None) == "_dynamic_support_com_pos_w")
    def quat_apply(q, v):
        t = 2 * torch.cross(q[..., 1:], v, dim=-1)
        return v + q[..., :1] * t + torch.cross(q[..., 1:], t, dim=-1)
    scope = {"torch": torch, "quat_apply": quat_apply}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), "exec"), scope)
    compute = scope[node.name]
    coms = torch.tensor([[[.1, .2, .3, 0., 0., 0., 1.]]])
    reads = []
    def get_coms():
        reads.append(1)
        return coms
    data = SimpleNamespace(body_link_pos_w=torch.zeros(1, 1, 3),
                           body_link_quat_w=torch.tensor([[[1., 0., 0., 0.]]]))
    asset = SimpleNamespace(data=data, root_physx_view=SimpleNamespace(get_coms=get_coms))
    torch.testing.assert_close(compute(asset), coms[..., :3])
    data.body_link_pos_w.add_(torch.tensor([3., 4., 5.]))
    data.body_link_quat_w[:] = torch.tensor([0., 0., 0., 1.])  # yaw 180 degrees
    torch.testing.assert_close(compute(asset), torch.tensor([[[2.9, 3.8, 5.3]]]))
    assert len(reads) == 1
    coms[..., :3].add_(.1)
    # Local cache owns its data and is invalidated by COM writes, including exact resume.
    torch.testing.assert_close(compute(asset), torch.tensor([[[2.9, 3.8, 5.3]]]))
    asset._hope_support_com_pos_b = None
    torch.testing.assert_close(compute(asset), torch.tensor([[[2.8, 3.7, 5.4]]]))
    assert len(reads) == 2
