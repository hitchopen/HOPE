"""CPU tests of production mocap outage, receive, reset and stale handling.

The production initialization block and methods are compiled from AST to avoid
the unrelated Isaac scene constructor. Quaternion operations are Isaac's real
math functions; transport behavior is not duplicated in a test implementation.
"""

import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

torch = pytest.importorskip("torch")
isaac_math = pytest.importorskip("isaaclab.utils.math")
MDP = Path(__file__).resolve().parents[1] / "source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp"
spec = importlib.util.spec_from_file_location("mocap_transport_test_module", MDP / "mocap_transport.py")
transport = importlib.util.module_from_spec(spec)
spec.loader.exec_module(transport)


def production_methods():
    tree = ast.parse((MDP / "hope_commands.py").read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "RacketTargetCommand")
    methods = {n.name: n for n in cls.body if isinstance(n, ast.FunctionDef)}
    statements = methods["__init__"].body
    start = next(i for i, n in enumerate(statements) if isinstance(n, ast.Assign)
                 and any(isinstance(t, ast.Attribute) and t.attr == "_base_mocap_enabled" for t in n.targets))
    stop = next(i for i in range(start, len(statements)) if isinstance(statements[i], ast.For)
                and isinstance(statements[i].target, ast.Name) and statements[i].target.id == "metric_name")
    init = ast.parse("def initialize(self, cfg):\n    pass").body[0]
    init.body = statements[start:stop]
    reset = ast.parse("def reset_session(self, env_ids):\n    pass").body[0]
    reset.body = [n for n in methods["_resample_command"].body if isinstance(n, ast.If)
                  and ast.unparse(n.test) == "self._base_mocap_enabled and (not self._resample_is_wrap)"]
    assert len(reset.body) == 1, "production mocap reset block must be located explicitly"
    term_tree = ast.parse((MDP / "terminations.py").read_text())
    stale = next(n for n in term_tree.body if isinstance(n, ast.FunctionDef) and n.name == "base_mocap_stale")
    namespace = dict(torch=torch, math=__import__("math"), ManagerBasedRLEnv=object,
                     quat_mul=isaac_math.quat_mul, quat_from_euler_xyz=isaac_math.quat_from_euler_xyz,
                     advance_packet_outage=transport.advance_packet_outage,
                     validate_outage_config=transport.validate_outage_config)
    module = ast.fix_missing_locations(ast.Module(
        body=[init, reset, methods["_push_actor_base_mocap"], stale], type_ignores=[]))
    exec(compile(module, str(MDP / "hope_commands.py"), "exec"), namespace)
    return namespace


METHODS = production_methods()


def mocap(count=2, delay=0, scale=1.0, **overrides):
    values = dict(
        base_mocap_enabled=True, base_mocap_orientation_enabled=True,
        base_mocap_delay_steps=delay, base_mocap_delay_min_steps=delay,
        base_mocap_update_interval_steps=1, base_mocap_position_noise_std=(0., 0., 0.),
        base_mocap_orientation_noise_std_rad=(0., 0., 0.),
        base_mocap_extrinsic_residual_rpy_std_rad=(0., 0., 0.),
        base_mocap_position_bias_std=(0., 0., 0.),
        base_mocap_dropout_prob=0., base_mocap_velocity_ema_alpha=1.,
        base_mocap_max_age_s=.05, base_mocap_max_propagation_s=0.,
        base_mocap_burst_start_prob=0., base_mocap_burst_duration_s=(.06, .06),
        base_mocap_burst_long_prob=0., base_mocap_burst_long_duration_s=(.12, .12),
    )
    values.update(overrides)
    quat = torch.zeros(count, 4)
    quat[:, 0] = 1.
    m = SimpleNamespace(cfg=SimpleNamespace(**values), num_envs=count, device="cpu",
                        base_pos_w=torch.zeros(count, 3), base_quat_w=quat,
                        _base_mocap_robustness_scale=scale, _ability_curriculum_enabled=False,
                        _resample_is_wrap=False,
                        metrics={"base_mocap_robustness_scale": torch.zeros(count)},
                        robot=SimpleNamespace(data=SimpleNamespace(
                            root_lin_vel_w=torch.zeros(count, 3), root_ang_vel_b=torch.zeros(count, 3))))
    m._env = SimpleNamespace(step_dt=.02, num_envs=count, device="cpu",
                             command_manager=SimpleNamespace(get_term=lambda name: m))
    METHODS["initialize"](m, m.cfg)
    return m


def push(m):
    METHODS["_push_actor_base_mocap"](m)


def reset(m, rows):
    METHODS["reset_session"](m, torch.tensor(rows))


def test_outage_blocks_exact_duration_and_reset_cancels_it():
    remaining = torch.zeros(2, dtype=torch.long)
    reset_rows = torch.tensor([False, True])
    for tick in range(4):
        remaining, blocked, started = transport.advance_packet_outage(
            remaining, reset_rows, dt=.02, scale=1., start_probability=1. if tick == 0 else 0.,
            short_range_s=(.06, .06), long_probability=0., long_range_s=(.12, .12))
        assert blocked.tolist() == [tick < 3, False]
        assert started.tolist() == [tick == 0, False]
    remaining, blocked, _ = transport.advance_packet_outage(
        torch.tensor([20, 20]), torch.ones(2, dtype=torch.bool), dt=.02, scale=1.,
        start_probability=1., short_range_s=(.06, .06), long_probability=1., long_range_s=(.12, .12))
    assert not bool(blocked.any()) and not bool(remaining.any())


def test_consecutive_loss_freezes_received_pose_and_increments_age_until_stale():
    m = mocap()
    push(m)
    saved_pos = m._actor_base_pos_w.clone()
    saved_quat = m._actor_base_quat_w.clone()
    m.cfg.base_mocap_burst_start_prob = 1.
    m.cfg.base_mocap_burst_long_prob = 1.
    for tick in range(1, 7):
        m.base_pos_w[:, 0] = 10. + tick
        m.base_quat_w[:] = torch.tensor([0., 0., 0., 1.])
        push(m)
        m.cfg.base_mocap_burst_start_prob = 0.
        torch.testing.assert_close(m._actor_base_pos_w, saved_pos, rtol=0, atol=0)
        torch.testing.assert_close(m._actor_base_quat_w, saved_quat, rtol=0, atol=0)
        torch.testing.assert_close(m._actor_base_age_s, torch.full((2,), tick * .02))
        assert not bool(m.metrics["base_mocap_packet_received"].any())
        stale = METHODS["base_mocap_stale"](m._env, "racket_target")
        assert bool(stale.all()) == (tick >= 3)


def test_reset_backfills_delayed_ring_and_breaks_only_reset_environment_burst():
    m = mocap(delay=2)
    push(m)
    m._base_mocap_burst_remaining[:] = 5
    m.base_pos_w[0] = torch.tensor([50., 30., 1.])
    m.base_quat_w[0] = torch.tensor([0., 1., 0., 0.])
    reset(m, [0])
    push(m)
    torch.testing.assert_close(m._actor_base_pos_w[0], m.base_pos_w[0])
    torch.testing.assert_close(m._base_mocap_delay_buf[:, 0], m.base_pos_w[0].expand(3, 3))
    torch.testing.assert_close(m._base_mocap_quat_delay_buf[:, 0], m.base_quat_w[0].expand(3, 4))
    assert m._actor_base_age_s[0] == 0.
    assert bool((m._actor_base_velocity_xy[0] == 0.).all())
    assert m._base_mocap_burst_remaining.tolist() == [0, 4]
    assert m.metrics["base_mocap_packet_received"].tolist() == [1., 0.]
    # A delayed read on the following step must still belong to the new session.
    push(m)
    torch.testing.assert_close(m._actor_base_pos_w[0], m.base_pos_w[0])


@pytest.mark.parametrize("delay", [0, 2])
def test_reacquired_velocity_uses_total_packet_interval(delay):
    m = mocap(delay=delay, base_mocap_max_age_s=.2)
    push(m)
    for tick in range(1, 5):
        m.base_pos_w[:, 0] = tick * .02
        push(m)
    m._base_mocap_burst_remaining[:] = 3
    for tick in range(5, 9):
        m.base_pos_w[:, 0] = tick * .02
        push(m)
    assert bool((m.metrics["base_mocap_packet_received"] == 1.).all())
    torch.testing.assert_close(m._actor_base_velocity_xy[:, 0], torch.ones(2), rtol=1e-5, atol=1e-6)
    assert bool((m._actor_base_age_s == 0.).all())


def test_scale_zero_is_clean_without_consuming_rng_even_between_capture_ticks():
    m = mocap(delay=2, scale=0., base_mocap_delay_min_steps=0,
              base_mocap_update_interval_steps=3,
              base_mocap_position_noise_std=(.1, .1, .1),
              base_mocap_orientation_noise_std_rad=(.1, .1, .1),
              base_mocap_extrinsic_residual_rpy_std_rad=(.1, .1, .1),
              base_mocap_position_bias_std=(.1, .1, .1),
              base_mocap_dropout_prob=.9, base_mocap_burst_start_prob=1.)
    rng = torch.random.get_rng_state().clone()
    for tick in range(4):
        m.base_pos_w[:, 0] = tick * .02
        if tick == 2:
            # Session reset must defer even the per-session delay RNG draw
            # until corruption is enabled, not merely suppress packet noise.
            reset(m, [0])
        push(m)
        torch.testing.assert_close(m._actor_base_pos_w, m.base_pos_w, rtol=0, atol=0)
        assert bool((m._actor_base_age_s == 0.).all())
        assert not bool(m.metrics["base_mocap_burst_active"].any())
    assert torch.equal(torch.random.get_rng_state(), rng)


def test_scale_zero_cancels_existing_outage_without_random_draws():
    rng = torch.random.get_rng_state().clone()
    remaining, blocked, started = transport.advance_packet_outage(
        torch.tensor([8, 3]), torch.zeros(2, dtype=torch.bool), dt=.02, scale=0.,
        start_probability=1., short_range_s=(.04, .08), long_probability=1., long_range_s=(.1, .3))
    assert not bool(remaining.any() | blocked.any() | started.any())
    assert torch.equal(torch.random.get_rng_state(), rng)


def test_quaternion_sign_changes_do_not_create_received_attitude_jumps():
    m = mocap()
    push(m)
    before = m._actor_base_quat_w.clone()
    m.base_quat_w.neg_()
    push(m)
    torch.testing.assert_close(m._actor_base_quat_w, before, rtol=0, atol=0)
    assert bool((m.metrics["base_mocap_orientation_error_rad"] == 0.).all())


def test_position_bias_is_session_constant_and_resampled_only_for_reset_rows():
    torch.manual_seed(812)
    m = mocap(base_mocap_position_bias_std=(.02, .02, .01))
    push(m)
    bias = m._base_mocap_position_bias.clone()
    m.base_pos_w[:, 0] = .1
    push(m)
    torch.testing.assert_close(m._actor_base_pos_w - m.base_pos_w, bias)
    torch.testing.assert_close(m._base_mocap_position_bias, bias, rtol=0, atol=0)
    reset(m, [0])
    push(m)
    assert not torch.equal(m._base_mocap_position_bias[0], bias[0])
    torch.testing.assert_close(m._base_mocap_position_bias[1], bias[1], rtol=0, atol=0)


def test_per_environment_delay_is_preserved_through_receive_ring():
    m = mocap(delay=2)
    m._base_mocap_sampled_delay[:] = torch.tensor([0, 2])
    push(m)
    for tick in range(1, 5):
        m.base_pos_w[:, 0] = float(tick)
        push(m)
        torch.testing.assert_close(m._actor_base_pos_w[:, 0], torch.tensor([float(tick), float(max(0, tick - 2))]))
