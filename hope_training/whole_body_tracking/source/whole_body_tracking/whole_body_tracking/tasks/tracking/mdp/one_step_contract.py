"""Backend-neutral transition function for V15's one-command/one-STEP-bout latch."""

from __future__ import annotations

from typing import NamedTuple


class OneStepBoutResult(NamedTuple):
    gait_active: object
    step_mode_before_completion: object
    step_mode: object
    started: object
    complete: object
    new_bout: object
    blocked_reentry: object
    just_completed: object
    elapsed_steps: object
    dwell_count: object


def advance_one_step_bout(
    xp,
    *,
    in_hold,
    planned,
    elapsed_steps,
    duration_steps,
    started,
    complete,
    dwell_count,
    settled,
    required_dwell_steps: int,
) -> OneStepBoutResult:
    """Advance one control tick while making STEP completion irreversible."""

    if required_dwell_steps < 1:
        raise ValueError("required_dwell_steps must be >= 1")
    unfinished = elapsed_steps < duration_steps
    requested_gait = in_hold & planned & unfinished
    blocked_reentry = requested_gait & complete
    gait_active = requested_gait & (~complete)
    new_bout = gait_active & (~started)
    started_next = started | new_bout
    step_mode_before_completion = (
        in_hold & planned & started_next & (~complete)
    )
    elapsed_next = elapsed_steps + gait_active
    gait_done = (
        planned
        & started_next
        & (~complete)
        & (elapsed_next >= duration_steps)
    )
    # Stable dwell starts only after the last active gait command has ended.
    # Counting that final commanded tick would shorten a 0.30 s zero-command
    # settle by one control period whenever the physical predicates happened
    # to be true before the last gait target was applied.
    settle_phase = gait_done & (~gait_active)
    dwell_next = xp.where(
        settle_phase & settled,
        dwell_count + 1,
        dwell_count * 0,
    )
    just_completed = gait_done & (dwell_next >= required_dwell_steps)
    complete_next = complete | just_completed
    step_mode = step_mode_before_completion & (~just_completed)
    return OneStepBoutResult(
        gait_active=gait_active,
        step_mode_before_completion=step_mode_before_completion,
        step_mode=step_mode,
        started=started_next,
        complete=complete_next,
        new_bout=new_bout,
        blocked_reentry=blocked_reentry,
        just_completed=just_completed,
        elapsed_steps=elapsed_next,
        dwell_count=dwell_next,
    )
