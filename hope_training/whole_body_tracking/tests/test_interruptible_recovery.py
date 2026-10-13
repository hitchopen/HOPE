"""A recovery deadline must never gate incoming-command release."""
import torch
from test_gaussian_action_projection import load_module, UTILS

module = load_module("interruptible_recovery_test", UTILS / "interruptible_recovery.py")


def test_immediate_early_and_late_balls_use_arrival_not_settling_budget():
    clock = module.InterruptibleRecovery(3, "cpu")
    clock.begin(torch.arange(3), torch.tensor([40, 60, 40]), torch.tensor([0, 5, 70]))
    released = {}
    for tick in range(1, 71):
        _, due, interrupted = clock.advance()
        for i in torch.where(due)[0].tolist(): released[i] = (tick, bool(interrupted[i]))
        if tick == 45:
            assert clock.waiting[2] and clock.budget_ticks[2] == 0
    assert released == {0: (1, True), 1: (5, True), 2: (70, False)}


def test_earlier_notification_preempts_60_tick_budget_on_next_control_tick():
    clock = module.InterruptibleRecovery(2, "cpu")
    clock.begin(torch.arange(2), torch.tensor([60, 60]), torch.tensor([80, 80]))
    for _ in range(3): clock.advance()
    clock.notify(torch.tensor([0]))
    _, due, interrupted = clock.advance()
    assert due.tolist() == interrupted.tolist() == [True, False]
    assert clock.waiting.tolist() == [False, True]
    # No pose, speed, support, reward or readiness condition participates.
    assert clock.budget_ticks[0] == 0


def test_reset_cannot_leak_old_arrival_into_new_episode():
    clock = module.InterruptibleRecovery(2, "cpu")
    clock.begin(torch.arange(2), torch.tensor([40, 60]), torch.tensor([1, 30]))
    clock.reset(torch.tensor([0]))
    owned, due, _ = clock.advance()
    assert owned.tolist() == [False, True] and not due.any()


def test_sampling_contains_rapid_interruptions_and_uninterrupted_settling():
    with torch.random.fork_rng(devices=[]):
        torch.manual_seed(7)
        budget, arrival = module.sample_arrivals(10000, "cpu", (40, 60), (0, 80), .25)
    assert (arrival == 0).float().mean() > .23
    assert ((arrival > 0) & (arrival < budget)).float().mean() > .35
    assert (arrival >= budget).float().mean() > .20


def test_wandb_admits_interruption_and_waiting_metrics():
    logging = load_module("interruptible_recovery_logging_test", UTILS / "wandb_logging.py")
    assert logging.wandb_metric_allowed("Live/motion/recovery_interrupted_by_ball", "hitter_pingpong_v1")


def test_production_motion_clock_releases_on_arrival_and_exposes_true_time_remaining():
    from test_rally_timing import motion, advance
    m = motion(count=3)
    m._resample_command = lambda ids: None
    m.time_steps[:] = torch.tensor([10, 120, 10])
    start = m.time_steps.clone()
    m.rally_wrap_step[:] = torch.tensor([70, 180, 70])
    state = m.inter_ball_recovery = module.InterruptibleRecovery(3, "cpu")
    state.begin(torch.arange(3), torch.tensor([60, 60, 2]), torch.tensor([0, 80, 80]))
    for tick in range(1, 9):
        if tick == 3:
            state.notify(torch.tensor([1]))
        advance(m)
        assert (m.time_steps - start).tolist() == [tick, max(0, tick-2), 0]
        assert m.hold_counter.tolist() == [0, 80-tick if tick < 3 else 0, 80-tick]
    assert state.waiting[2] and state.budget_ticks[2] == 0


def test_station_step_cannot_spend_interruptible_idle_time_as_guaranteed_travel_time():
    from types import SimpleNamespace
    from test_rally_timing import motion, limit_step
    m = motion(count=2)
    m.inter_ball_recovery = module.InterruptibleRecovery(2, "cpu")
    m.rally_shortened[:] = True
    m.hold_counter[:] = torch.tensor([0, 80])
    target = SimpleNamespace(_motion=lambda: m, _env=m._env,
                             time_to_strike=torch.tensor([.35, .35]))
    result = limit_step(target, torch.arange(2), torch.tensor([.35, .35]))
    torch.testing.assert_close(result, torch.tensor([.09, .09]))
