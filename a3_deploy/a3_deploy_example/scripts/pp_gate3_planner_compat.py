"""Planner evidence checks shared across fixed-delay and adaptive commit planners."""
import math

SUPPORTED_STAGE2_MODES = frozenset({
    'post_net_one_shot_effective_bounce_zero_spin',
    'post_net_one_shot_spin_conditioned_bounce',
    'post_net_one_shot_physical_spin_magnus',
})


def planner_version_independent_errors(mode, net_cross_s, commit_s, bounce_gain):
    """Validate evidence without prescribing an algorithm's delay or calibration.

    A fixed net+50ms deadline and gain=0.369 are not universal contracts. The
    adaptive planner owns its deadline and calibrated bounce model. Flight
    identities, validity, freshness and loss checks stay in the caller.
    """
    errors = []
    if mode not in SUPPORTED_STAGE2_MODES:
        errors.append('unsupported C++ Stage-2 mode')
    if not all(math.isfinite(v) for v in (net_cross_s, commit_s, bounce_gain)):
        errors.append('non-finite planner timing or bounce gain')
        return errors
    if commit_s < net_cross_s:
        errors.append('commit timestamp precedes net crossing')
    if not 0.0 <= bounce_gain <= 1.0:
        errors.append('future-bounce tangential gain is outside [0,1]')
    return errors
