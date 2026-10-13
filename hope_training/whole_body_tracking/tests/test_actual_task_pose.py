"""Physical-reference regressions: correct posture may require a large PD offset."""
import csv
import math
from types import SimpleNamespace as NS

import pytest
import torch

from test_gaussian_action_projection import load_module, policy_modules, UTILS

pose = load_module("actual_task_pose_test", UTILS / "task_pose_reference.py")
refinement = load_module("actual_pose_refinement_test", UTILS / "settled_policy_refinement.py")


def make_env():
    ref = pose.load_reference()
    names = [f"waist_{a}_joint" for a in ("yaw", "roll", "pitch")]
    q = pose.reference_joints(names, torch.zeros(1)).repeat(3, 1)
    rp = torch.tensor(ref["metrics"]["torso_rpy_deg"][:2])*math.pi/180
    r, p = rp/2
    quat = torch.tensor([torch.cos(r)*torch.cos(p), torch.sin(r)*torch.cos(p),
                         torch.cos(r)*torch.sin(p), -torch.sin(r)*torch.sin(p)])
    data = NS(joint_pos=q, joint_vel=torch.zeros_like(q),
              body_quat_w=quat.reshape(1, 1, 4).repeat(3, 1, 1),
              applied_torque=torch.full_like(q, 20.))
    robot = NS(data=data, find_joints=lambda n, **kw: (list(range(3)), n),
               root_physx_view=NS(get_dof_projected_joint_forces=lambda: torch.full_like(q, 17.)))
    cmd = NS(robot=robot, metrics={}, time_to_strike=torch.tensor([1., 0., -.8]),
             _resolve_body_index=lambda _: 0)
    motion = NS(in_hold=torch.tensor([True, False, False]))
    action = NS(_joint_names=names, _qdes_executed=q.clone()+.4)
    terms = {"racket_target": cmd, "motion": motion}
    env = NS(command_manager=NS(get_term=terms.__getitem__), action_manager=NS(get_term=lambda _: action),
             common_step_counter=576000, episode_length_buf=torch.tensor([10, 20, 30]))
    return env, cmd, action


def test_correct_actual_pose_with_point_four_pd_offset_is_not_penalized():
    env, cmd, action = make_env()
    assert pose.evaluate_task_pose(env).eq(0).all()
    torch.testing.assert_close(cmd.metrics["task_pose_waist_pitch_pd_offset_rad"], torch.full((3,), .4))
    # Torque is diagnostic too; no zero-torque target in this posture reward.
    cmd.robot.data.applied_torque.fill_(80.)
    action._qdes_executed += .4
    assert pose.evaluate_task_pose(env).eq(0).all()


def test_wrong_actual_pose_is_penalized_even_when_qdes_equals_q():
    env, cmd, action = make_env()
    cmd.robot.data.joint_pos[:, 2] += .3
    action._qdes_executed.copy_(cmd.robot.data.joint_pos)
    reward = pose.evaluate_task_pose(env)
    assert reward[0] > 0 and reward[2] > 0 and reward[1] == 0


def test_zero_aux_gradient_for_stationary_gravity_offset():
    current = torch.full((2, 31), .4, requires_grad=True)
    previous = current.detach().clone().requires_grad_(True)
    smooth, removed, diagnostics = refinement.refinement_losses(
        current, previous, torch.zeros(2), torch.ones(2), torch.ones(2), list(range(15)), 14)
    (smooth+removed).backward()
    assert current.grad.eq(0).all() and previous.grad.eq(0).all()
    assert not diagnostics["waist_qdes_offset_abs_rad"].requires_grad


def test_torso_wrong_even_when_waist_correct_and_no_compensation_loophole():
    env, cmd, _ = make_env()
    cmd.robot.data.body_quat_w[:] = torch.tensor([math.cos(.15), math.sin(.15), 0, 0])
    assert pose.evaluate_task_pose(env)[0] > 0


@pytest.mark.parametrize("steps,scale", [(448000, 0), (512000, .5), (576000, 1)])
def test_physical_reward_curriculum(steps, scale):
    env, cmd, _ = make_env()
    cmd.robot.data.joint_pos[:, 2] += .3
    full = pose.evaluate_task_pose(env)
    env.common_step_counter = steps
    torch.testing.assert_close(pose.evaluate_task_pose(env), full*scale)


def test_phase_windows_and_new_clock_cover_recovery():
    tts = torch.tensor([1., .85, .65, .45, 0., -.5, -.65, -.8])
    gate, phase = pose.phase_gate(torch.tensor([True]+[False]*7), tts)
    torch.testing.assert_close(gate, torch.tensor([1., 1., .5, 0., 0., 0., .5, 1.]))
    assert phase.tolist() == [0, 2, 2, 1, 1, 1, 2, 2]


def test_metrics_cannot_mutate_actual_state_and_trace_is_synchronized(tmp_path):
    env, cmd, action = make_env()
    env.common_step_counter = 1
    env._task_pose_chain_trace = pose.PoseChainTrace(tmp_path)
    pose.evaluate_task_pose(env)
    original_q = cmd.robot.data.joint_pos.clone()
    original_qdes = action._qdes_executed.clone()
    for value in cmd.metrics.values():
        value.zero_()
    env._task_pose_chain_trace.flush()
    torch.testing.assert_close(cmd.robot.data.joint_pos, original_q)
    torch.testing.assert_close(action._qdes_executed, original_qdes)
    rows = list(csv.DictReader((tmp_path/'task_pose_chain.csv').open()))
    assert len(rows) == 3
    row = rows[0]
    assert float(row['waist_pitch_pd_offset_rad']) == pytest.approx(.4)
    assert float(row['waist_pitch_pd_torque_estimate_Nm']) == 20
    assert float(row['waist_pitch_joint_axis_torque_Nm']) == 17
    assert float(row['waist_pitch_actual_rad']) == pytest.approx(float(row['waist_pitch_reference_rad']))


def test_selected_reference_is_name_mapped_and_torso_heading_does_not_change_tilt():
    forward = pose.reference_joints(['waist_pitch_joint', 'left_knee_joint'], torch.zeros(1))
    reverse = pose.reference_joints(['left_knee_joint', 'waist_pitch_joint'], torch.zeros(1))
    torch.testing.assert_close(forward, reverse.flip(0))
    # Pure yaw has no roll/pitch error; heading remains owned by the existing task.
    torch.testing.assert_close(pose.torso_roll_pitch(torch.tensor([[math.cos(.5), 0, 0, math.sin(.5)]])), torch.zeros(1, 2))


def test_deprecated_aux_coefficient_is_rejected(policy_modules):
    from test_settled_policy_refinement import build_algorithm
    _, policy, _ = build_algorithm(policy_modules)
    _, ppo_class = policy_modules
    with pytest.raises(ValueError, match="q_des-q auxiliary tracking was removed"):
        ppo_class(policy, device='cpu', settled_aux_tracking_coef=.02)


def test_task_pose_diagnostics_are_admitted_to_wandb():
    logging = load_module('task_pose_wandb_test', UTILS/'wandb_logging.py')
    assert logging.wandb_metric_allowed('Episode_Reward/actual_task_pose', 'hitter_pingpong_v1')
    for phase in pose.PHASE_NAMES:
        for field in ('waist_pitch_reference_rad', 'waist_pitch_actual_rad', 'waist_pitch_qdes_rad',
                      'waist_pitch_pd_torque_estimate_Nm', 'waist_pitch_joint_axis_torque_Nm',
                      'waist_pitch_pd_offset_rad', 'waist_pitch_task_error_rad_rms'):
            assert logging.wandb_metric_allowed(f'Live/racket_target/task_pose_{phase}_{field}', 'hitter_pingpong_v1')


def test_batched_trace_preserves_each_tick_before_reused_buffers_change(tmp_path):
    trace = pose.PoseChainTrace(tmp_path)
    values = {'actual': torch.arange(6).float()}
    episodes = torch.arange(6)
    trace.record(1, episodes, values)
    values['actual'].add_(10)
    trace.record(2, episodes, values)
    values['actual'].zero_()
    assert not trace.path.exists()  # no synchronous per-tick disk/host transfer
    trace.flush()
    rows = list(csv.DictReader(trace.path.open()))
    assert [float(r['actual']) for r in rows] == [0, 1, 2, 3, 10, 11, 12, 13]
    assert [int(r['control_step']) for r in rows] == [1]*4+[2]*4
    assert not trace.pending
