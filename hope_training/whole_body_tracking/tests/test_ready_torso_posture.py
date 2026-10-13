"""Regression for the observed bent-waist rest pose after Gate3 rallies."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
import torch


@pytest.fixture
def reward():
    p = Path(__file__).resolve().parents[1] / "source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/hope_rewards.py"
    names = {"ready_torso_posture_debt", "_bounded_refinement_debt", "_deadband_huber"}
    nodes = [n for n in ast.parse(p.read_text()).body if getattr(n, "name", None) in names]
    scope = {"torch": torch, "math": math}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(p), "exec"), scope)
    return scope["ready_torso_posture_debt"]


def quat(roll=0., pitch=0., yaw=0.):
    cr, cp, cy = [math.cos(v/2) for v in (roll, pitch, yaw)]
    sr, sp, sy = [math.sin(v/2) for v in (roll, pitch, yaw)]
    return torch.tensor([cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
                         cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy])


def fixture():
    # Pelvis is upright. Only the torso and waist are crooked, as in model 17000.
    body = torch.stack([quat(), quat(.117)]).unsqueeze(0).repeat(7, 1, 1)
    joints = torch.zeros(7, 3); joints[:, 1] = .138
    robot = NS(data=NS(body_quat_w=body, joint_pos=joints),
               find_joints=lambda *a, **kw: ([1], ["waist_roll_joint"]))
    cmd = NS(robot=robot, metrics={}, _resolve_body_index=lambda name: 1,
             time_to_strike=torch.tensor([1.3, .85, .65, .45, 0., -.50, -.80]))
    motion = NS(in_hold=torch.tensor([True, False, False, False, False, False, False]))
    sensor = NS(data=NS(net_forces_w=torch.zeros(7, 2, 3)))
    sensor.data.net_forces_w[:, :, 2] = 100
    terms = {"racket_target": cmd, "motion": motion}
    env = NS(common_step_counter=576000, command_manager=NS(get_term=terms.__getitem__),
             scene=NS(sensors={"contact_forces": sensor}))
    return env, cmd, motion, sensor, NS(name="contact_forces", body_ids=[0, 1])


def test_upright_pelvis_no_longer_hides_bent_torso_and_waist(reward):
    env, cmd, _, _, cfg = fixture()
    assert reward(env, cfg)[0] > 0
    # Straighten torso alone: waist remains charged, preventing counter-lean compensation.
    cmd.robot.data.body_quat_w[:, 1] = quat()
    assert reward(env, cfg)[0] > 0
    cmd.robot.data.joint_pos[:, 1] = 0
    assert reward(env, cfg).eq(0).all()


def test_followthrough_and_windup_are_free_but_next_ready_clock_is_covered(reward):
    env, _, _, _, cfg = fixture()
    v = reward(env, cfg)
    assert v[0] > 0
    torch.testing.assert_close(v[[0, 1, 6]], v[0].expand(3))
    torch.testing.assert_close(v[2], v[0]*.5)
    assert v[3:6].eq(0).all()


def test_pitch_and_yaw_are_not_mistaken_for_lateral_lean(reward):
    env, cmd, _, _, cfg = fixture()
    cmd.robot.data.body_quat_w[:, 1] = quat(0., .30, .8)
    cmd.robot.data.joint_pos[:, 1] = 0
    assert reward(env, cfg).eq(0).all()


def test_actual_single_support_is_free(reward):
    env, _, _, sensor, cfg = fixture()
    sensor.data.net_forces_w[:, 1, 2] = 20
    assert reward(env, cfg).eq(0).all()


@pytest.mark.parametrize("step,scale", [(0, 0), (448000, 0), (512000, .5), (576000, 1)])
def test_late_curriculum(reward, step, scale):
    env, cmd, _, _, cfg = fixture()
    full = reward(env, cfg)
    env.common_step_counter = step
    torch.testing.assert_close(reward(env, cfg), full * scale)
    assert cmd.metrics["ready_posture_scale"].eq(scale).all()


def test_bounded_budget_and_symmetric_roll_deadbands(reward):
    env, cmd, _, _, cfg = fixture()
    cmd.robot.data.body_quat_w[:, 1] = quat(-.117)
    cmd.robot.data.joint_pos[:, 1] = -.138
    negative = reward(env, cfg)
    cmd.robot.data.body_quat_w[:, 1] = quat(.117)
    cmd.robot.data.joint_pos[:, 1] = .138
    torch.testing.assert_close(negative, reward(env, cfg))
    cmd.robot.data.body_quat_w[:, 1] = quat(2.)
    cmd.robot.data.joint_pos[:, 1] = 2.
    assert bool((reward(env, cfg) < 1).all())
    cmd.robot.data.body_quat_w[:, 1] = quat(.04)
    cmd.robot.data.joint_pos[:, 1] = .05
    assert reward(env, cfg).eq(0).all()
