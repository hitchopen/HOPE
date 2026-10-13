"""Independent incoming-command clock; a settling budget never blocks a ball.

Training samples incoming-command availability, not physical ball contact. The
existing sampled pre-strike clock starts when that command becomes available.
The optional notification supports an earlier event without changing the pose,
station or strike reference that was sampled for this swing.
"""
import torch


class InterruptibleRecovery:
    def __init__(self, count, device):
        self.waiting = torch.zeros(count, dtype=torch.bool, device=device)
        self.arrival_ticks = torch.zeros(count, dtype=torch.long, device=device)
        self.budget_ticks = torch.zeros_like(self.arrival_ticks)
        self.notified = torch.zeros_like(self.waiting)

    def reset(self, ids):
        self.waiting[ids] = False
        self.arrival_ticks[ids] = 0
        self.budget_ticks[ids] = 0
        self.notified[ids] = False

    def begin(self, ids, budget_ticks, arrival_ticks):
        self.reset(ids)
        self.budget_ticks[ids] = budget_ticks
        self.arrival_ticks[ids] = arrival_ticks
        self.waiting[ids] = arrival_ticks > 0
        self.notified[ids] = arrival_ticks == 0

    def notify(self, ids):
        # Consumed before the reference clock advances on the next control tick.
        self.notified[ids] |= self.waiting[ids]

    def advance(self):
        owned = self.waiting | self.notified
        due = owned & ((self.arrival_ticks <= 1) | self.notified)
        interrupted = due & (self.budget_ticks > 0)
        self.waiting &= ~due
        self.arrival_ticks = torch.where(self.waiting, (self.arrival_ticks-1).clamp_min(0), 0)
        self.budget_ticks = torch.where(self.waiting, (self.budget_ticks-1).clamp_min(0), 0)
        self.notified.zero_()
        # Expiry of budget_ticks has deliberately NO effect on release.
        return owned, due, interrupted


def sample_arrivals(count, device, budget_range, arrival_range, immediate_probability):
    budget = torch.randint(budget_range[0], budget_range[1]+1, (count,), device=device)
    arrival = torch.randint(arrival_range[0], arrival_range[1]+1, (count,), device=device)
    immediate = torch.rand(count, device=device) < immediate_probability
    return budget, torch.where(immediate, torch.zeros_like(arrival), arrival)
