"""Exercise production scheduling methods on CPU without starting Isaac Sim."""

import ast
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

torch = pytest.importorskip("torch")
MDP = Path(__file__).resolve().parents[1] / "source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp"


def method(filename, cls, name, stop_before=None):
    tree = ast.parse((MDP / filename).read_text())
    owner = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == cls)
    node = next(node for node in owner.body if isinstance(node, ast.FunctionDef) and node.name == name)
    if stop_before:
        # Stop only before unrelated reference FK; all lifecycle code is production code.
        node.body = node.body[:next(i for i, statement in enumerate(node.body)
                                   if isinstance(statement, ast.Assign)
                                   and any(isinstance(t, ast.Name) and t.id == stop_before for t in statement.targets))]
    namespace = dict(torch=torch, math=math,
                     sample_uniform=lambda lo, hi, shape, device: lo + (hi - lo) * torch.rand(shape, device=device))
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(MDP / filename), "exec"), namespace)
    return namespace[name]


sample = method("commands.py", "MotionCommand", "_sample_rally_timing")
validate = method("commands.py", "MotionCommand", "_validate_rally_timing")
advance = method("commands.py", "MotionCommand", "_update_command", "anchor_pos_w_repeat")
limit_step = method("hope_commands.py", "RacketTargetCommand", "_limit_rally_station_step")
sample_bank = method("hope_commands.py", "RacketTargetCommand", "_apply_precomputed_venue_tuple")


def motion(count=4096, scale=1.0):
    cfg = SimpleNamespace(rally_timing_enabled=True, rally_timing_probability=.75,
                          rally_pre_strike_s_range=(.35, .85), rally_post_strike_s_range=(.35, .65),
                          rally_timing_ramp_steps=100, rally_station_speed_mps=.6,
                          rally_station_brake_s=.2, wrap_teleport=False, clip_switch_prob=0.)
    target = SimpleNamespace(strike_phase_per_clip=(.38679245, .44444444),
                             strike_phase=.38679245, strike_window_s=.12)
    env = SimpleNamespace(common_step_counter=round(scale * 100), step_dt=.02,
                          cfg=SimpleNamespace(commands=SimpleNamespace(racket_target=target)))
    m = SimpleNamespace(cfg=cfg, _env=env, device="cpu", num_envs=count, _multiseg=True,
                        motion=SimpleNamespace(num_segments=2, seg_start=torch.tensor([0, 107]),
                                               seg_len=torch.tensor([107, 109]), time_step_total=216),
                        clip_id=torch.arange(count) % 2, time_steps=torch.zeros(count, dtype=torch.long),
                        rally_wrap_step=torch.zeros(count, dtype=torch.long),
                        rally_shortened=torch.zeros(count, dtype=torch.bool),
                        hold_counter=torch.zeros(count, dtype=torch.long),
                        post_swing_replay_active=torch.zeros(count, dtype=torch.bool),
                        runtime_handoff_active=torch.zeros(count, dtype=torch.bool),
                        inter_ball_recovery=None, metrics={})
    m.post_swing_capture_enabled = lambda: False
    return m


def test_initial_curriculum_retains_full_motion_clock():
    m = motion(scale=0)
    sample(m, torch.arange(m.num_envs))
    assert torch.equal(m.time_steps, m.motion.seg_start[m.clip_id])
    assert torch.equal(m.rally_wrap_step, m.motion.seg_start[m.clip_id] + m.motion.seg_len[m.clip_id])
    assert not m.rally_shortened.any()


def test_v3_uses_exact_shared_25_tick_tail_for_both_sides():
    m = motion()
    m.cfg.rally_timing_probability = 1.0
    m.cfg.rally_timing_ramp_steps = 0
    m.cfg.rally_post_strike_s_range = (.5, .5)
    validate(m)
    sample(m, torch.arange(m.num_envs))
    strike = torch.where(m.clip_id == 0, 41, 155)
    assert torch.all(m.rally_wrap_step - strike == 25)
    assert m.rally_shortened.all()


def test_final_distribution_retains_normal_swings_and_never_skips_contact():
    torch.manual_seed(123)
    m = motion()
    sample(m, torch.arange(m.num_envs))
    short = m.rally_shortened
    assert .70 < short.float().mean() < .80
    strike = torch.where(m.clip_id == 0, 41, 155)
    assert ((strike - m.time_steps)[short] >= 18).all()  # >= 0.35 s, rounded up
    assert ((m.rally_wrap_step - strike)[short] >= 18).all()
    assert ((m.rally_wrap_step - strike)[short] <= 33).all()
    assert torch.equal(m.time_steps[~short], m.motion.seg_start[m.clip_id[~short]])
    assert torch.equal(m.rally_wrap_step[~short],
                       (m.motion.seg_start + m.motion.seg_len)[m.clip_id[~short]])


def test_early_deadline_really_wraps_before_the_clip_end_and_emits_target_resample():
    m = motion(count=2)
    m.time_steps[:] = torch.tensor([60, 174])
    m.rally_wrap_step[:] = torch.tensor([61, 175])
    captured = []
    m._resample_command = lambda ids: captured.extend(ids.tolist())
    advance(m)
    assert captured == [0, 1]
    assert m.just_resampled.tolist() == [True, True]
    assert m.time_steps.tolist() == [61, 175]  # One frame per step, not accelerated.
    assert (m.time_steps < m.motion.seg_start[m.clip_id] + m.motion.seg_len[m.clip_id]).all()


def test_hold_does_not_consume_pre_strike_reference_frames():
    m = motion(count=2)
    m.time_steps[:] = torch.tensor([10, 120])
    m.hold_counter[:] = 3
    m.rally_wrap_step[:] = torch.tensor([70, 180])
    m._resample_command = lambda ids: None
    advance(m)
    assert m.time_steps.tolist() == [10, 120]
    assert m.hold_counter.tolist() == [2, 2]
    assert not m.just_resampled.any()


def test_short_deadline_large_station_step_reserves_braking_and_keeps_normal_coverage():
    m = motion(count=4)
    m.rally_shortened[:] = torch.tensor([True, True, True, False])
    m.hold_counter[2] = 10
    target = SimpleNamespace(_motion=lambda: m, _env=m._env,
                             time_to_strike=torch.tensor([.35, .35, .35, .35]))
    actual = limit_step(target, torch.arange(4), torch.tensor([.35, -.35, .35, .35]))
    torch.testing.assert_close(actual, torch.tensor([.09, -.09, .21, .35]))


def test_disabled_timing_preserves_clock_and_rng():
    m = motion(count=2)
    m.cfg.rally_timing_enabled = False
    m.time_steps[:] = torch.tensor([7, 123])
    before = torch.random.get_rng_state()
    sample(m, torch.arange(2))
    assert m.time_steps.tolist() == [7, 123]
    assert torch.equal(before, torch.random.get_rng_state())


def test_short_ball_bank_sampling_preserves_correlated_tuple_fields():
    m = motion(count=64)
    m.rally_shortened[:] = True
    target = SimpleNamespace(_motion=lambda: m, _env=m._env, device="cpu",
                             cfg=SimpleNamespace(venue_tuple_final_mix_prob=1., venue_tuple_unconditional_outcomes=False),
                             time_to_strike=torch.full((64,), .35), _resample_is_wrap=True,
                             _venue_tuple_selected=torch.ones(64, dtype=torch.bool))
    target._limit_rally_station_step = lambda ids, requested: limit_step(target, ids, requested)
    target._fixed_balanced_tuple_selection = lambda clip, probability, ids: torch.ones_like(clip, dtype=torch.bool)
    target._normal_from_target_velocity = lambda velocity: velocity
    target.metrics = {name: torch.zeros(64) for name in
                      ("venue_tuple_mix_effective", "station_y_step_command", "station_y_step_signed_command",
                       "station_y_step_class", "venue_tuple_selected")}
    fields = {"base_target_pos_w": 2, "racket_target_pos_w": 3, "racket_target_vel_w": 3,
              "racket_target_normal_w": 3, "_venue_planner_contact_normal_w": 3,
              "vb_vel_in_w": 3, "vb_spin_in_w": 3, "_venue_intended_landing_xy": 2,
              "_venue_outgoing_velocity_seed": 3}
    for name, width in fields.items():
        setattr(target, name, torch.zeros(64, width))
    row_code = torch.arange(5).float()[:, None].repeat(1, 3)
    bank = {name: row_code.clone() for name in
            ("target_velocity", "target_position_offset", "target_normal", "incoming_velocity", "incoming_spin",
             "outgoing_velocity")}
    bank["station_xy"] = torch.tensor([[0., -.3], [0., -.08], [0., 0.], [0., .08], [0., .3]])
    bank["intended_landing_xy"] = row_code[:, :2]
    target._venue_tuple_bank_by_clip = {0: bank, 1: bank}
    sample_bank(target, torch.arange(64), torch.zeros(64, 3), m.clip_id, torch.zeros(64))
    assert target.base_target_pos_w[:, 1].abs().max() <= .09
    assert set(target.racket_target_vel_w[:, 0].tolist()) <= {1., 2., 3.}
    torch.testing.assert_close(target.vb_vel_in_w, target.racket_target_vel_w)
    torch.testing.assert_close(target._venue_outgoing_velocity_seed, target.racket_target_vel_w)
    torch.testing.assert_close(target.racket_target_pos_w[:, :2] - target.base_target_pos_w,
                               target.racket_target_vel_w[:, :2])


def test_single_clip_opt_in_has_a_real_pre_strike_question():
    m = motion(count=4)
    m._multiseg = False
    m.motion.num_segments = 1
    m.motion.seg_start = torch.tensor([0])
    m.motion.seg_len = torch.tensor([107])
    m.clip_id[:] = 0
    m._env.cfg.commands.racket_target.strike_phase_per_clip = ()
    m.time_steps[:] = 100  # A late legacy RSI frame must not make a new ball un-hittable.
    sample(m, torch.arange(4))
    assert (m.time_steps < 41).all()
    assert (m.rally_wrap_step > 41).all()


@pytest.mark.parametrize("field,value", [("rally_timing_probability", 1.01),
                                         ("rally_pre_strike_s_range", (float("nan"), .5)),
                                         ("rally_station_speed_mps", 0.),
                                         ("wrap_teleport", True)])
def test_invalid_deadline_distribution_fails_fast(field, value):
    m = motion(count=2)
    setattr(m.cfg, field, value)
    with pytest.raises(ValueError):
        validate(m)
