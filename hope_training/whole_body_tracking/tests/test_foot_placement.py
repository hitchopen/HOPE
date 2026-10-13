import importlib.util
import math
from pathlib import Path

import pytest
import torch

path = Path(__file__).resolve().parents[1] / "source/whole_body_tracking/whole_body_tracking/tasks/tracking/foot_placement.py"
spec = importlib.util.spec_from_file_location("foot_placement", path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def state():
    return (torch.tensor([[[0., .14, 0.], [0., -.14, 0.]]]),
            torch.tensor([[[1., 0., 0., 0.], [1., 0., 0., 0.]]]),
            torch.ones((1, 2), dtype=torch.bool))


def test_nominal_forward_stance_is_free_but_narrow_crossed_and_wide_are_not():
    p, q, loaded = state()
    assert module.foot_placement_debt(p, q, loaded)[0].item() == 0
    costs = []
    for width in (.10, -.28, .5):
        p[:, 0, 1], p[:, 1, 1] = width / 2, -width / 2
        costs.append(module.foot_placement_debt(p, q, loaded)[0].item())
    assert all(x > 0 for x in costs)
    assert costs[1] > costs[0]


def test_fore_aft_stagger_cannot_hide_narrow_width():
    p, q, loaded = state()
    p[:, :, 1] *= .2
    before = module.foot_placement_debt(p, q, loaded)[0]
    p[:, 0, 0] = .5
    torch.testing.assert_close(module.foot_placement_debt(p, q, loaded)[0], before)


def test_world_forward_heading_detects_toe_in_and_out_without_chasing_pelvis():
    p, q, loaded = state()
    for yaw in (-.5, .5, math.pi):
        q[:, :, 0], q[:, :, 3] = math.cos(yaw / 2), math.sin(yaw / 2)
        assert module.foot_placement_debt(p, q, loaded)[1].item() > 0
        torch.testing.assert_close(module.foot_placement_debt(p, -q, loaded)[1],
                                   module.foot_placement_debt(p, q, loaded)[1])


def test_single_support_does_not_pin_the_swing_foot_or_demand_double_support():
    p, q, loaded = state()
    loaded[:, 0] = False
    p[:, 0] = torch.tensor([10., -10., 1.])
    q[:, 0] = torch.tensor([0., 0., 0., 1.])
    width, yaw, *_ = module.foot_placement_debt(p, q, loaded)
    assert width.item() == yaw.item() == 0
    q[:, 1] = q[:, 0]
    assert module.foot_placement_debt(p, q, loaded)[1].item() > 0


def test_translation_and_task_frame_rotation_leave_cost_unchanged():
    p, q, loaded = state()
    p[:, :, 1] *= .5
    before = module.foot_placement_debt(p, q, loaded)
    angle = .7
    rotation = torch.tensor([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
    p[:, :, :2] = p[:, :, :2] @ rotation.T + torch.tensor([8., -4.])
    q[:, :, 0], q[:, :, 3] = math.cos(angle / 2), math.sin(angle / 2)
    after = module.foot_placement_debt(p, q, loaded, heading=angle)
    torch.testing.assert_close(before[0], after[0], atol=1e-5, rtol=1e-5)
    torch.testing.assert_close(before[1], after[1], atol=1e-5, rtol=1e-5)


def test_incorrect_contact_mask_and_invalid_band_fail():
    p, q, loaded = state()
    with pytest.raises(ValueError):
        module.foot_placement_debt(p, q, loaded, width_min=.4, width_max=.3)
    with pytest.raises(ValueError):
        module.foot_placement_debt(p, q, loaded.float())
