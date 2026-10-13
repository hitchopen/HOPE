import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

SOURCE = Path(__file__).resolve().parents[1] / "source/whole_body_tracking/whole_body_tracking/utils/sim2real_export.py"
spec = importlib.util.spec_from_file_location("sim2real_export", SOURCE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def fixture():
    resolved = dict(qdes_action_contract="v11_affine_safe_qdes_v1",
                    qdes_policy_feedback_contract="legacy_applied_raw_v1",
                    qdes_delay_min_steps=0, qdes_delay_max_steps=1,
                    qdes_delay_nominal_fraction=0.5, qdes_joint_names=["joint"])
    action = NS(passive_joint_names=("head_yaw_joint", "head_pitch_joint"),
                resolved_contract_metadata=lambda: resolved)
    motion = NS(post_swing_replay_contract="markov_side_phase_severity_v3",
                rally_timing_enabled=True, runtime_handoff_start_prob=0.,
                rally_post_strike_s_range=(0.5, 0.5), rally_timing_probability=1.0,
                rally_timing_ramp_steps=0,
                hold_steps_range=(0, 0), hold_long_prob=0.15,
                hold_long_steps_range=(150, 300))
    racket = NS(base_mocap_enabled=True, base_mocap_orientation_enabled=True,
                target_mode="hitter_pure", locomotion_enabled=False,
                racket_vel_start_range_per_clip=[None, None],
                racket_vel_planner_range_per_clip=[None, None],
                recovery_curriculum_enabled=False, station_y_step_range=(0.12, 0.35),
                station_y_same_prob=0.5, station_y_small_step_prob=0.2,
                station_y_small_step_range=(0.03, 0.12), station_side_explicit=True)
    env = NS(action_manager=NS(get_term=lambda _: action),
             command_manager=NS(get_term=lambda name: NS(cfg={"motion": motion, "racket_target": racket}[name])),
             scene={"robot": NS(data=NS(joint_names=["joint"]))}, step_dt=0.02)
    task = {"build_contract": {"training_recipe_revision": module.RECIPE,
                              "actor_observation_dim": 110, "action_dim": 31}}
    return env, task, resolved, motion, racket


def test_training_latency_is_recorded_without_adding_deploy_latency():
    env, task, _, _, _ = fixture()
    md = module.sim2real_metadata(env, task)
    assert md["hitter_pure_training_recipe"] == module.RECIPE
    assert md["hitter_pingpong_deploy_delay_contract"] == "direct_no_added_delay_v1"
    assert json.loads(md["hitter_pure_resolved_task_json"]) == task
    assert "hitter_pure_v14_stability_recipe" not in md
    assert md["hitter_pingpong_rally_end_contract"] == "ball_clock_followthrough_v1"
    assert md["hitter_pingpong_rally_post_strike_s"] == "0.5000"


@pytest.mark.parametrize("revision", ["actual_ready_recovery_v7", "actual_full_body_hold_v8", "actual_stable_hold_v9", "actual_balanced_hold_v10"])
def test_v7_records_inter_ball_training_hold_without_changing_runtime_tail(revision):
    env, task, _, motion, racket = fixture()
    task["build_contract"].update(reward_revision=revision,
                                  recovery_target_contract="current_command_station_v1",
                                  inter_ball_hold_contract="interruptible_recovery_next_ball_v1")
    motion.post_wrap_min_hold = 0
    motion.post_wrap_hold_jitter_steps = 0
    motion.interruptible_recovery_enabled = True
    motion.recovery_window_steps_range = (40, 60)
    motion.next_ball_delay_steps_range = (0, 80)
    motion.next_ball_immediate_prob = .25
    racket.hold_until_settled = racket.ready_release_enabled = False
    motion.wrap_teleport = motion.hold_long_on_wrap = False
    md = module.sim2real_metadata(env, task)
    assert md["hitter_pure_training_post_wrap_hold_steps"] == "0,0"
    assert md["hitter_pure_training_inter_ball_contract"] == "interruptible_recovery_next_ball_v1"
    assert md["hitter_pingpong_rally_post_strike_s"] == "0.5000"
    assert md["hitter_pure_recovery_target_contract"] == "current_command_station_v1"
    motion.post_wrap_min_hold = 40
    with pytest.raises(RuntimeError, match="HOLD"):
        module.sim2real_metadata(env, task)


@pytest.mark.parametrize("mutation", ["tail", "mixture", "ramp", "hold"])
def test_v3_rejects_training_deployment_timing_mismatch(mutation):
    env, task, _, motion, _ = fixture()
    if mutation == "tail": motion.rally_post_strike_s_range = (.35, .65)
    if mutation == "mixture": motion.rally_timing_probability = .75
    if mutation == "ramp": motion.rally_timing_ramp_steps = 96000
    if mutation == "hold": motion.hold_steps_range = (0, 10)
    with pytest.raises(RuntimeError, match="follow-through"):
        module.sim2real_metadata(env, task)


def test_v2_reexport_preserves_identity_and_only_selects_a_covered_tail():
    env, task, _, motion, _ = fixture()
    task["build_contract"]["training_recipe_revision"] = "continuous_rally_sim2real_v2"
    motion.rally_post_strike_s_range = (.35, .65)
    motion.rally_timing_probability = .75
    md = module.sim2real_metadata(env, task)
    assert md["hitter_pure_training_recipe_version"] == "2"
    assert md["hitter_pure_training_recipe"] == "continuous_rally_sim2real_v2"
    motion.rally_post_strike_s_range = (.7, .8)
    with pytest.raises(RuntimeError, match="coverage"):
        module.sim2real_metadata(env, task)


@pytest.mark.parametrize("mutation", ["feedback", "delay", "mocap", "handoff", "rate", "recipe"])
def test_incompatible_runtime_is_rejected(mutation):
    env, task, resolved, motion, racket = fixture()
    if mutation == "feedback": resolved["qdes_policy_feedback_contract"] = "executed_qdes"
    if mutation == "delay": resolved["qdes_delay_max_steps"] = 2
    if mutation == "mocap": racket.base_mocap_orientation_enabled = False
    if mutation == "handoff": motion.runtime_handoff_start_prob = 0.1
    if mutation == "rate": env.step_dt = 0.01
    if mutation == "recipe": task["build_contract"]["training_recipe_revision"] = "unknown"
    with pytest.raises(RuntimeError, match="Refusing"):
        module.sim2real_metadata(env, task)
