"""Regressions for crouching, missing wrap-HOLD, station mismatch and arm chatter."""
import math
import sys
import types
from types import SimpleNamespace as NS

import pytest
import torch
from test_gaussian_action_projection import load_module, UTILS
from test_settled_policy_refinement import fake_environment, refinement

package = types.ModuleType("ready_recovery_test")
package.__path__ = [str(UTILS)]
sys.modules[package.__name__] = package
ready = load_module("ready_recovery_test.ready_recovery", UTILS / "ready_recovery.py")
pose = sys.modules["ready_recovery_test.task_pose_reference"]


def environment():
    n = 4
    names = ["left_knee_joint", "right_knee_joint", "left_ankle_roll_joint", "right_ankle_roll_joint"]
    q = pose.reference_joints(names, torch.zeros(1)).repeat(n, 1)
    data = NS(joint_pos=q, joint_vel=torch.zeros_like(q), root_pos_w=torch.zeros(n, 3),
              root_lin_vel_w=torch.zeros(n, 3))
    data.root_pos_w[:, 2] = pose.load_reference()["metrics"]["pelvis_height_cm"] / 100.
    robot = NS(data=data, find_joints=lambda wanted, **kw: ([names.index(x) for x in wanted], wanted))
    cmd = NS(robot=robot, metrics={}, time_to_strike=torch.zeros(n),
             base_target_pos_w=torch.zeros(n, 2), base_pos_w=data.root_pos_w)
    motion = NS(in_hold=torch.tensor([True, True, False, False]))
    cmd.time_to_strike[:] = torch.tensor([1., 1., 0., -.8])
    sensor = NS(data=NS(net_forces_w=torch.zeros(n, 2, 3)), find_bodies=lambda names, **kw: ([0, 1], names))
    sensor.data.net_forces_w[:, :, 2] = 100.
    terms = {"motion": motion, "racket_target": cmd}
    env = NS(common_step_counter=576000, command_manager=NS(get_term=terms.__getitem__),
             scene=NS(env_origins=torch.zeros(n, 3), sensors={"contact_forces": sensor}))
    return env, cmd, sensor


def test_crouch_is_not_a_valid_hold_but_strike_is_free():
    env, cmd, _ = environment()
    assert ready.actual_ready_legs(env).eq(0).all()
    cmd.robot.data.joint_pos[:, :2] += math.radians(20)
    cmd.robot.data.root_pos_w[:, 2] -= .05  # still above old 0.98 m floor
    value = ready.actual_ready_legs(env)
    assert value[0] > .1 and value[1] > .1 and value[3] > .1
    assert value[2] == 0


def test_no_new_pose_cost_in_strike_or_protected_followthrough():
    tts = torch.tensor([.5, 0., -.49, -.5, -.65, -.8])
    torch.testing.assert_close(ready.recovery_gate(torch.zeros(6, dtype=torch.bool), tts),
                               torch.tensor([0., 0., 0., 0., .5, 1.]))


def test_station_owner_is_current_command_not_reset_anchor_and_approach_beats_freeze():
    env, cmd, _ = environment()
    cmd.base_target_pos_w[:, 1] = .2
    frozen = ready.station_recovery(env)
    cmd.robot.data.root_lin_vel_w[:, 1] = .34
    assert ready.station_recovery(env)[0] < frozen[0]
    cmd.base_pos_w[:, 1] = .2
    cmd.robot.data.root_lin_vel_w.zero_()
    assert ready.station_recovery(env).eq(0).all()


def test_loaded_ankle_velocity_cost_without_qdes_dependency_and_no_step_lock():
    env, cmd, sensor = environment()
    cmd.robot.data.joint_vel[:, 2:] = .18
    assert ready.loaded_ankle_quiet(env)[0] > 0
    assert ready.loaded_ankle_quiet(env)[2] == 0
    sensor.data.net_forces_w[0, 0, 2] = 0
    assert ready.loaded_ankle_quiet(env)[0] == 0
    cmd.base_target_pos_w[1, 1] = .25
    assert ready.loaded_ankle_quiet(env)[1] == 0


@pytest.mark.parametrize("step,scale", [(448000, 0), (512000, .5), (576000, 1)])
def test_curriculum_all_new_physical_costs_and_no_state_alias(step, scale):
    env, cmd, _ = environment()
    cmd.robot.data.joint_pos[:, :2] += .3
    cmd.robot.data.joint_vel[:, 2:] = .3
    cmd.base_target_pos_w[:, 1] = .08
    funcs = [ready.actual_ready_legs, ready.station_recovery, ready.loaded_ankle_quiet]
    full = [f(env) for f in funcs]
    env.common_step_counter = step
    for f, expected in zip(funcs, full):
        torch.testing.assert_close(f(env), expected * scale)
    before = cmd.robot.data.joint_pos.clone()
    for v in cmd.metrics.values(): v.zero_()
    torch.testing.assert_close(cmd.robot.data.joint_pos, before)


def test_torso_signal_strengthens_without_changing_waist_or_stationary_pd_offset():
    waist = torch.zeros(1)
    torso = torch.tensor([[math.radians(4.4), 0.]])
    old = pose.actual_pose_debt(waist, torso, 0., torch.zeros(2))
    new = pose.actual_pose_debt(waist, torso, 0., torch.zeros(2), torso_margin=math.radians(1), torso_std=.06)
    assert new > 20 * old
    assert pose.actual_pose_debt(waist, torso*0, 0., torch.zeros(2), torso_margin=.01745, torso_std=.06) == 0


def test_v7_smoothing_covers_both_arms_and_excludes_passive_head():
    env, action, _, _ = fake_environment()
    arms = [f"{side}_{j}_joint" for side in ("left", "right")
            for j in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")]
    action._joint_names[15:] = arms + ["head_yaw_joint", "head_pitch_joint"]
    ctx = refinement.SettledPolicyRefinement(env, NS(num_transitions_per_env=4, num_envs=4, device="cpu"),
        start_steps=0, ramp_steps=1, include_upper=True, step_margin=.0005, step_scale=.005)
    assert len(ctx.local_columns) == 29
    current = torch.zeros(2, 31, requires_grad=True)
    previous = torch.zeros_like(current)
    previous[:, 15] = .0022
    loss, removed, _ = refinement.refinement_losses(current, previous, torch.zeros(2), torch.ones(2),
        torch.ones(2), ctx.local_columns, ctx.waist_column, step_margin=ctx.step_margin, step_scale=ctx.step_scale)
    (loss + removed).backward()
    assert current.grad[:, 15].abs().sum() > 0
    assert current.grad[:, 29:].eq(0).all()


def test_resolved_v7_preserves_plant_abi_episode_and_schedule():
    from hydra import compose, initialize_config_dir
    from omegaconf import OmegaConf
    cfg_dir = UTILS.parents[3] / "cfg"
    with initialize_config_dir(version_base=None, config_dir=str(cfg_dir)):
        old = compose(config_name="train", overrides=["task=HOPEPingPongTaskReference", "algo=ppo_quiet"])
        new = compose(config_name="train", overrides=["task=HOPEPingPongReadyRecovery", "algo=ppo_ready"])
    for key in ("env", "sim", "domain_rand", "action", "racket", "actor_obs_contract"):
        assert OmegaConf.to_container(old.task, resolve=True)[key] == OmegaConf.to_container(new.task, resolve=True)[key]
    assert new.task.env.episode_length_s == 12
    assert list(new.task.motion.rally_post_strike_s_range) == [.5, .5]
    assert new.task.motion.post_wrap_min_hold == 0 and new.task.motion.post_wrap_hold_jitter_steps == 0
    assert new.task.motion.interruptible_recovery_enabled
    assert list(new.task.motion.recovery_window_steps_range) == [40, 60]
    assert new.task.motion.next_ball_immediate_prob == .25
    assert new.task.rewards.actual_task_pose_start_steps == new.algo.algorithm.settled_aux_start_steps == 448000
    assert new.task.rewards.actual_task_pose_ramp_steps == new.algo.algorithm.settled_aux_ramp_steps == 128000
    assert new.algo.algorithm.settled_aux_tracking_coef == 0
    assert new.algo.algorithm.settled_aux_gradient_ratio_max == .05
