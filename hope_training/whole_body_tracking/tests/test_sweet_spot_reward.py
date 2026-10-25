"""Reward-phase achieved-state physics, one-shot ownership and rectangle semantics."""
import ast
from collections import defaultdict
import importlib.util
import hashlib
import json
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS, MethodType

import numpy as np
import pytest
import torch

from test_virtual_ball_cuda_graph import vb, PATH

ROOT = PATH.parents[6]
UTILS = PATH.parents[3] / "utils"


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


region = load("hope_sweet_spot_test", UTILS / "sweet_spot.py")


def methods():
    source = PATH.with_name("hope_commands.py")
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "RacketTargetCommand")
    names = {"_sweet_spot_strike_time", "sweet_spot_landing_reward", "_reset_sweet_spot_attempts",
             "_publish_sweet_spot_counts", "_vb_evaluate", "capture_markov_replay_state",
             "restore_markov_replay_state"}
    scope = {"torch": torch, "math": math}
    exec(compile(ast.Module(body=[n for n in cls.body if isinstance(n, ast.FunctionDef)
                                 and n.name in names], type_ignores=[]), str(source), "exec"), scope)
    return {name: scope[name] for name in names}


@pytest.fixture(scope="module")
def solved_racket():
    model = load("sweet_spot_full_model", ROOT / "scripts/full_ball_model.py")
    params = model.params_from_profile(vb._profile.load_profile())
    plan = pytest.importorskip("numba").njit(model.plan)
    result = plan(np.array([.15, -.45, .32]), np.array([-2.2, .18, .18]),
                  np.array([0., -30., 5.]), np.array([2.34, -.5]), .5, params)
    assert result[6] and result[7]
    return torch.tensor(result[0], dtype=torch.float64), torch.tensor(result[1], dtype=torch.float64)


@pytest.fixture
def command(monkeypatch, solved_racket):
    module = ModuleType("whole_body_tracking.tasks.tracking.mdp")
    module.virtual_ball = vb
    monkeypatch.setitem(sys.modules, module.__name__, module)
    monkeypatch.setitem(sys.modules, "whole_body_tracking.utils.sweet_spot", region)
    n = 4
    clip = torch.tensor([0, 1, 0, 1])
    motion = NS(_multiseg=True, clip_id=clip, cfg=NS(causal_idle=False),
                time_steps=torch.tensor([5, 25, 5, 25]),
                motion=NS(num_segments=2, seg_start=torch.tensor([0, 20]), seg_len=torch.tensor([11, 11])))
    origins = torch.tensor([[0., 0, 0], [10., 20, 0], [-20., 10, 0], [0., 0, 3.]], dtype=torch.float64)
    position = torch.tensor([.65, .3125, 1.08], dtype=torch.float64).repeat(n, 1) + origins
    velocity, normal = (value.repeat(n, 1) for value in solved_racket)
    cfg = NS(sweet_spot_reward_enabled=True, sweet_spot_x_range=(2.74, 2.94),
             sweet_spot_y_range=(.1875, .3375), sweet_spot_distance_scale_m=2.5,
             sweet_spot_inside_bonus=1., strike_phase=.5, strike_phase_per_clip=(.5, .5),
             vb_capture_radius=.095, vb_min_approach_speed=.3, vb_table_surface_z=.76,
             vb_rollout_h=.01, vb_rollout_steps=100, vb_min_landing_depth=.3,
             exact_success_decay=.9, exact_success_min_count=1., venue_tuple_unconditional_outcomes=True)
    cmd = NS(cfg=cfg, num_envs=n, device="cpu", _motion=lambda: motion,
             _env=NS(common_step_counter=10, step_dt=.02, reset_buf=torch.zeros(n, dtype=torch.bool),
                     scene=NS(env_origins=origins)),
             _strike_phase_per_clip_t=None, _sweet_spot_pending=torch.ones(n, dtype=torch.bool),
             _sweet_spot_attempt_clip=clip.clone(), _sweet_spot_generation=torch.ones(n, dtype=torch.long),
             _sweet_spot_counts=torch.zeros(2, 4, dtype=torch.float64),
             _sweet_spot_last_reward_step=-1, _sweet_spot_event=None,
             _racket_fk=lambda: (position, None, velocity, normal, normal),
             racket_target_pos_w=position.clone(), _vb_params=vb.load_venue_params(),
             vb_vel_in_w=torch.tensor([-2.2, .18, .18], dtype=torch.float64).repeat(n, 1),
             vb_spin_in_w=torch.tensor([0., -30., 5.], dtype=torch.float64).repeat(n, 1),
             _vb_net_x=1.87, _vb_far_x=3.24, _vb_half_w=.7625, _vb_y_center=0.,
             _vb_net_overhang=.1525, _vb_net_top_z=.9125, _vb_ball_r=.02,
             _vb_target_xy=torch.tensor([2.84, .2625], dtype=torch.float64),
             metrics=defaultdict(lambda: torch.zeros(n, dtype=torch.float64)),
             # Deliberately stale baseline FK must neither drive the reward nor change.
             racket_pos_w=torch.full((n, 3), 99.), racket_lin_vel_w=torch.zeros(n, 3),
             racket_normal_w=torch.zeros(n, 3), vb_fired=torch.zeros(n, dtype=torch.bool),
             _venue_tuple_selected=torch.ones(n, dtype=torch.bool),
             _venue_tuple_outcome_pending=torch.ones(n, dtype=torch.bool),
             _venue_tuple_outcome_clip=clip.clone(),
             _venue_intended_landing_xy=torch.tensor([2.84, .2625]).repeat(n, 1),
             _clip_names={0: "forehand", 1: "backhand"}, _venue_swing_starts_acc_c={0: 0., 1: 0.})
    for name in ("exact", "hit", "net", "land_valid", "inb"):
        setattr(cmd, f"_vb_{name}_acc", 0.)
    for name in ("exact", "hit", "net", "legal", "land_err_sum", "land_err_n"):
        setattr(cmd, f"_vb_{name}_acc_c" if name not in ("land_err_sum", "land_err_n")
                else f"_vb_{name}_c", {0: 0., 1: 0.})
    for name, function in methods().items():
        setattr(cmd, name, MethodType(function, cmd))
    cmd.rollouts = 0

    def predict(*args, **kwargs):
        cmd.rollouts += 1
        return vb.coarse_landing(*args, **kwargs)

    cmd._sweet_spot_landing_predictor = predict
    return cmd


def test_rectangle_inclusive_bounds_distance_and_nonfinite():
    xy = torch.tensor([[2.74, .1875], [2.94, .3375], [2.84, .2625],
                       [3.04, .4375], [float("nan"), 0.], [3., 0.]])
    raw, distance, inside = region.rectangle_reward(xy, torch.tensor([1, 1, 1, 1, 1, 0]).bool(),
                                                   (2.74, 2.94), (.1875, .3375), 2.5, 1.)
    torch.testing.assert_close(raw[:3], torch.full((3,), 2.))
    assert inside[:3].all() and not inside[3:].any()
    assert distance[3] == pytest.approx(math.sqrt(.02), abs=1e-6)
    assert 0 < raw[3] < 1 and raw[4:].eq(0).all()
    assert torch.isfinite(raw).all()


def test_fresh_state_single_rollout_dt_return_and_command_boundary(command):
    cmd = command
    originals = {name: (getattr(cmd, name), getattr(cmd, name).clone()) for name in
                 ("racket_pos_w", "racket_lin_vel_w", "racket_normal_w", "racket_target_pos_w")}
    raw = cmd.sweet_spot_landing_reward(30.)
    torch.testing.assert_close(raw, torch.full_like(raw, 2.))
    torch.testing.assert_close(cmd.metrics["sweet_spot_reward_event_return"], raw * .6)
    assert cmd.rollouts == 1
    assert cmd.sweet_spot_landing_reward(30.).eq(0).all()
    assert cmd.rollouts == 1 and not cmd._sweet_spot_pending.any()
    for name, (identity, values) in originals.items():
        assert getattr(cmd, name) is identity
        torch.testing.assert_close(getattr(cmd, name), values)
    # The motion manager advances before command telemetry. Its exact-strike
    # argument is now false, but the matching reward event is still this swing.
    cmd._motion().time_steps += 1
    cmd._vb_evaluate(torch.zeros(4, dtype=torch.bool), torch.full((4,), 99.))
    assert cmd.rollouts == 1 and cmd.vb_fired.all()
    assert cmd._sweet_spot_event is None
    assert cmd._vb_exact_acc == 4
    cmd._vb_evaluate(torch.ones(4, dtype=torch.bool), torch.zeros(4))
    assert cmd.rollouts == 1 and not cmd.vb_fired.any() and cmd._vb_exact_acc == 4
    for side in ("forehand", "backhand"):
        assert cmd.metrics[f"sweet_spot_attempt_count_{side}"].eq(2).all()
        assert cmd.metrics[f"sweet_spot_region_hit_rate_{side}"].eq(1).all()


def test_off_does_not_touch_fk_rng_latches_or_baseline_metrics(command):
    cmd = command
    cmd.cfg.sweet_spot_reward_enabled = False
    cmd._racket_fk = lambda: pytest.fail("disabled reward read achieved state")
    rng = torch.get_rng_state().clone()
    pending = cmd._sweet_spot_pending.clone()
    assert cmd.sweet_spot_landing_reward(30.).eq(0).all()
    assert cmd.rollouts == 0 and cmd._sweet_spot_last_reward_step == -1
    assert torch.equal(rng, torch.get_rng_state()) and torch.equal(pending, cmd._sweet_spot_pending)
    assert not cmd.metrics


def test_nearby_frames_no_repeated_credit_and_prestrike_fall_count(command):
    cmd = command
    cmd._motion().time_steps -= 1
    assert cmd.sweet_spot_landing_reward(30.).eq(0).all() and cmd.rollouts == 0
    cmd._env.common_step_counter += 1
    cmd._motion().time_steps += 1
    cmd._env.reset_buf[1] = True
    raw = cmd.sweet_spot_landing_reward(30.)
    assert raw[1] == 0 and raw[[0, 2, 3]].eq(2).all()
    cmd._reset_sweet_spot_attempts(torch.tensor([1]))
    assert cmd._sweet_spot_counts[1, 0] == 2 and cmd._sweet_spot_counts[1, 3] == 1
    cmd._env.common_step_counter += 1
    cmd._env.reset_buf.zero_()
    cmd._motion().time_steps += 1
    assert cmd.sweet_spot_landing_reward(30.).eq(0).all()
    assert cmd.rollouts == 1


@pytest.mark.parametrize("mutation", ["reset", "clip", "frame", "step"])
def test_cache_cannot_cross_event_identity(command, mutation):
    cmd = command
    assert cmd.sweet_spot_landing_reward(30.).eq(2).all()
    if mutation == "reset":
        cmd._reset_sweet_spot_attempts(torch.arange(4))
    elif mutation == "clip":
        cmd._motion().clip_id = 1 - cmd._motion().clip_id
    elif mutation == "frame":
        cmd._motion().time_steps += 2
    else:
        cmd._env.common_step_counter += 1
    cmd._vb_evaluate(torch.ones(4, dtype=torch.bool), torch.zeros(4))
    assert cmd.rollouts == 1 and not cmd.vb_fired.any() and cmd._vb_exact_acc == 0


def test_nonfinite_miss_and_illegal_return_earn_zero(command):
    cmd = command
    position, _, velocity, _, normal = cmd._racket_fk()
    velocity[0, 0] = float("nan")
    position[1, 0] += .2  # outside capture radius
    velocity[2].zero_()  # stationary block
    velocity[3] = torch.tensor([1., 0., -1.])  # caught but not a legal return
    raw = cmd.sweet_spot_landing_reward(30.)
    assert raw.eq(0).all() and torch.isfinite(raw).all()
    assert cmd._sweet_spot_counts[:, 0].sum() == 4
    assert cmd._sweet_spot_counts[:, 3].sum() == 0
    for key, value in cmd.metrics.items():
        assert torch.isfinite(value).all(), key


def test_frame_zero_held_clock_and_wrap_each_consume_only_one_attempt(command):
    cmd = command
    cmd.cfg.strike_phase_per_clip = (0., 0.)
    cmd._motion().time_steps = torch.tensor([0, 20, 0, 20])
    assert cmd.sweet_spot_landing_reward(30.).eq(2).all()
    cmd._env.common_step_counter += 1
    assert cmd.sweet_spot_landing_reward(30.).eq(0).all()  # held at zero
    cmd._motion().clip_id = 1 - cmd._motion().clip_id
    cmd._motion().time_steps = torch.tensor([20, 0, 20, 0])
    cmd._reset_sweet_spot_attempts(torch.arange(4))
    cmd._env.common_step_counter += 1
    assert cmd.sweet_spot_landing_reward(30.).eq(2).all()
    assert cmd.rollouts == 2
    torch.testing.assert_close(cmd._sweet_spot_counts[:, 0], torch.tensor([4., 4.], dtype=torch.float64))


def test_replay_preserves_consumed_latch_but_not_old_event_generation(command):
    cmd = command
    cmd._delay_steps = 0
    cmd._markov_replay_tensor_fields = lambda: ("racket_target_pos_w",)
    cmd.cfg.target_mode, cmd.cfg.normal_mode = "legacy", "velocity"
    assert cmd.sweet_spot_landing_reward(30.).eq(2).all()
    ids = torch.arange(4)
    snapshot = cmd.capture_markov_replay_state(ids)
    assert set(snapshot) == {"racket_target_pos_w", "_sweet_spot_pending", "_sweet_spot_attempt_clip"}
    counts = cmd._sweet_spot_counts.clone()
    old_generation = cmd._sweet_spot_generation.clone()
    cmd._reset_sweet_spot_attempts(ids)  # new physical session
    cmd.restore_markov_replay_state(ids, snapshot)  # resume consumed recovery
    assert not cmd._sweet_spot_pending.any()
    assert torch.equal(cmd._sweet_spot_counts, counts)
    assert torch.equal(cmd._sweet_spot_generation, old_generation + 1)
    cmd._vb_evaluate(torch.ones(4, dtype=torch.bool), torch.zeros(4))
    assert not cmd.vb_fired.any() and cmd._vb_exact_acc == 0
    cmd._env.common_step_counter += 1
    assert cmd.sweet_spot_landing_reward(30.).eq(0).all()
    assert cmd.rollouts == 1
    cmd.cfg.sweet_spot_reward_enabled = False
    legacy = cmd.capture_markov_replay_state(ids)
    assert set(legacy) == {"racket_target_pos_w"}
    cmd.restore_markov_replay_state(ids, legacy)
    cmd.cfg.sweet_spot_reward_enabled = True
    with pytest.raises(RuntimeError, match="missing fields"):
        cmd.restore_markov_replay_state(ids, legacy)


def test_failed_flight_never_leaks_nonfinite_reward_or_telemetry(command):
    cmd = command
    def failed_flight(*args, **kwargs):
        return {"land_xy": torch.full((4, 2), float("nan")),
                "land_valid": torch.ones(4, dtype=torch.bool),
                "net_z": torch.full((4,), float("inf")), "net_y": torch.zeros(4),
                "net_valid": torch.ones(4, dtype=torch.bool)}
    cmd._sweet_spot_landing_predictor = failed_flight
    assert cmd.sweet_spot_landing_reward(30.).eq(0).all()
    cmd._vb_evaluate(torch.ones(4, dtype=torch.bool), torch.zeros(4))
    assert not cmd.vb_landing_valid.any() and not cmd.vb_net_crossed.any()
    for key, value in cmd.metrics.items():
        assert torch.isfinite(value).all(), key


@pytest.mark.parametrize("reward_enabled", [False, True])
def test_production_bank_loader_rejects_relabelled_broad_rows(tmp_path, monkeypatch, reward_enabled):
    """Checksums and recipe labels do not replace row-level objective validation."""
    robust = load("sweet_spot_robust_loader_test", UTILS / "robust_ball_distribution.py")
    monkeypatch.setitem(sys.modules, "whole_body_tracking.utils.robust_ball_distribution", robust)
    path, receipt_path = tmp_path / "bank.npz", tmp_path / "bank.receipt.json"
    arrays = dict(schema_version=np.array(1), clip=np.array([0, 1]))
    for key, width in (("station_xy", 2), ("target_position_offset", 3), ("target_velocity", 3),
                       ("target_normal", 3), ("incoming_velocity", 3), ("incoming_spin", 3),
                       ("intended_landing_xy", 2), ("predicted_landing_xy", 2), ("outgoing_velocity", 3)):
        arrays[key] = np.zeros((2, width), dtype=np.float32)
    np.savez(path, **arrays)
    bank_hash = hashlib.sha256(path.read_bytes()).hexdigest()
    receipt_path.write_text(json.dumps(dict(schema_version=1, bank_sha256=bank_hash, contract=dict(
        recipe_revision=23, training_distribution_revision="stationary_reach_ppq004_sweet_spot_v1",
        qualification="full_ppq004_first_bounce_v1"))))
    cfg = NS(normal_mode="velocity", venue_tuple_bank_path=str(path),
             venue_tuple_bank_receipt_path=str(receipt_path), venue_tuple_bank_schema_version=1,
             venue_tuple_bank_recipe_revision=23, venue_tuple_bank_sha256=bank_hash,
             venue_tuple_bank_receipt_sha256=hashlib.sha256(receipt_path.read_bytes()).hexdigest(),
             sweet_spot_reward_enabled=reward_enabled,
             sweet_spot_x_range=(2.74, 2.94), sweet_spot_y_range=(.1875, .3375))
    cls = next(n for n in ast.parse(PATH.with_name("hope_commands.py").read_text()).body
               if isinstance(n, ast.ClassDef) and n.name == "RacketTargetCommand")
    node = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "_load_venue_tuple_bank")
    scope = dict(np=np, json=json, torch=torch)
    exec(compile(ast.Module(body=[node], type_ignores=[]), "production_bank_loader", "exec"), scope)
    obj = NS(cfg=cfg, _resolve_venue_tuple_bank_path=lambda p: Path(p))
    with pytest.raises(RuntimeError, match="canonical region centre"):
        scope["_load_venue_tuple_bank"](obj)
