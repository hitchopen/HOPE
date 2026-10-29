"""CompleteHold question sampling; never move a target to the measured robot.

Rows remain intact after physical qualification. A locomotion budget uses the
previous commanded station and the real remaining clock. Drift is deliberately
not forgiven by using measured position as the next target's origin.
"""
import torch


def curriculum_scale(step, start, ramp):
    if start < 0 or ramp <= 0:
        raise ValueError("curriculum requires nonnegative start and positive ramp")
    return min(1.0, max(0.0, (float(step) - start) / ramp))


DISTRIBUTIONS = ('complete_hold_robust_v1', 'stationary_reach_v1', 'stationary_reach_ppq004_full_v1',
                 'stationary_reach_ppq004_sweet_spot_v1')


def validate_sweet_spot_bank(arrays, receipt, x_range, y_range):
    """Do not accept broad-return rows merely relabeled as a recipe-23 bank."""
    import numpy as np
    contract = receipt.get('contract', {})
    if (contract.get('recipe_revision') != 23
            or contract.get('training_distribution_revision') != 'stationary_reach_ppq004_sweet_spot_v1'
            or contract.get('qualification') != 'full_ppq004_first_bounce_v1'):
        raise RuntimeError('sweet-spot bank requires recipe-23 full-physics qualification')
    bounds = np.asarray([x_range, y_range], dtype=np.float64)
    if bounds.shape != (2, 2) or not np.isfinite(bounds).all() or not (bounds[:, 0] < bounds[:, 1]).all():
        raise RuntimeError('sweet-spot bank requires finite increasing region bounds')
    target = bounds.mean(axis=1)
    intended = np.asarray(arrays['intended_landing_xy'])
    predicted = np.asarray(arrays['predicted_landing_xy'])
    if (intended.ndim != 2 or intended.shape[1] != 2 or predicted.shape != intended.shape
            or not np.isfinite(intended).all() or not np.isfinite(predicted).all()):
        raise RuntimeError('sweet-spot bank landing arrays must be finite matching [N,2] arrays')
    # Storage is float32; permit rounding only, never the broad qualification box.
    tolerance = 8 * np.finfo(np.float32).eps * np.maximum(1.0, np.abs(bounds).max(axis=1))
    if not (np.abs(intended - target) <= tolerance).all():
        raise RuntimeError('sweet-spot bank intended landings must equal the canonical region centre')
    if not ((predicted >= bounds[:, 0] - tolerance) & (predicted <= bounds[:, 1] + tolerance)).all():
        raise RuntimeError('sweet-spot bank predicted landings must lie inside the inclusive region')


def validate_training_contract(task, algo):
    build = task.get('build_contract', {})
    revision = build.get('training_distribution_revision')
    if revision not in DISTRIBUTIONS:
        return
    if algo['runner']['num_steps_per_env'] != 64:
        raise ValueError('CompleteHold robustness curriculum is calibrated for 64 steps per PPO iteration')
    racket = task['racket']
    if racket['base_mocap_episode_timeout_s'] <= max(racket['base_mocap_stress_long_duration_s']):
        raise ValueError('mocap training timeout must allow the longest stress stall to recover')
    if revision in ('stationary_reach_v1', 'stationary_reach_ppq004_full_v1', 'stationary_reach_ppq004_sweet_spot_v1'):
        # The robot never walks: every question is answered from the session
        # anchor, so the exported runner must also hold that anchor.
        if not (racket.get('venue_tuple_fixed_station') and racket.get('venue_tuple_robust_sampling')
                and list(racket['base_target_x_range']) == [0, 0]
                and list(racket['base_target_y_range']) == [0, 0]
                and racket.get('station_y_step_range') is None
                and not racket.get('venue_tuple_limit_reset_travel', False)
                and build.get('station_command_contract') == 'fixed_session_anchor_v1'
                and build.get('recovery_target_contract') == 'fixed_session_anchor_v1'):
            raise ValueError('stationary_reach_v1 requires a fixed session-anchor station in racket and build_contract')
        fh, bh = racket['pos_range_per_clip']['forehand'], racket['pos_range_per_clip']['backhand']
        if not fh['y'][1] <= bh['y'][0]:
            raise ValueError('stationary reach bands must keep forehand strictly on the negative side of backhand')
    if revision == 'stationary_reach_ppq004_sweet_spot_v1':
        settings, objective = task.get('sweet_spot', {}), task.get('landing_objective', {})
        enabled = settings.get('reward_enabled')
        if (type(enabled) is not bool or racket.get('venue_tuple_bank_recipe_revision') != 23
                or not objective or not objective.get('sha256')
                or build.get('reward_revision') != 'actual_complete_hold_v11_stationary_sweet_spot_v1'
                or racket.get('sweet_spot_reward_enabled') is not enabled
                or task.get('rewards', {}).get('sweet_spot_landing_weight') != (
                    settings.get('reward_weight') if enabled else 0.0)):
            raise ValueError('sweet-spot training requires the validated recipe-23 objective and canonical reward switch')


def sample_rows(bank, previous_xy, remaining_s, *, speed, brake_s, wide_probability,
                attempts=256, unbounded=False, cohort_rows=None):
    """Choose core/wide rows inside a command-to-command travel budget.

The wide cohort is defined by source_kind=1. Fallback is to another *intact*
qualified row, and is reported rather than pretending the requested mixture
was achieved. No physical ball/landing field is clipped or translated here.
``unbounded`` is the fixed-station case: every row commands the session
anchor, so there is no travel to budget and the cohort mixture is exact.
    """
    if not 0 <= wide_probability <= 1 or speed <= 0 or brake_s < 0 or attempts < 1:
        raise ValueError("invalid robust question sampling budget")
    count = len(previous_xy)
    device = previous_xy.device
    budget = speed * (remaining_s - brake_s).clamp_min(0)
    if unbounded:
        budget = torch.full_like(budget, float("inf"))
    choose_wide = torch.rand(count, device=device) < wide_probability
    result = torch.full((count,), -1, dtype=torch.long, device=device)
    fallback = torch.zeros(count, dtype=torch.bool, device=device)
    if unbounded:
        # Every fixed-station candidate is valid. Keep the original random draw
        # shape/order (including discarded candidates) so seeded runs retain
        # exactly the same questions AND subsequent random-number state.
        for cohort in (0, 1):
            ids = torch.where(choose_wide == bool(cohort))[0]
            if not len(ids):
                continue
            pool = (cohort_rows[cohort] if cohort_rows is not None
                    else torch.where(bank['source_kind'] == cohort)[0])
            if not len(pool):
                raise ValueError(f"question bank lacks cohort {cohort}")
            draws = torch.randint(len(pool), (len(ids), attempts), device=device)
            result[ids] = pool[draws[:, 0]]
        return result, choose_wide, fallback
    # Sampling is a curriculum of geometric cases, not an ability filter on
    # the current policy. Falling behind therefore remains visible in rewards.
    for cohort in (0, 1, -1):
        ids = torch.where(result < 0)[0]
        if cohort >= 0:
            ids = ids[choose_wide[ids] == bool(cohort)]
        if not len(ids):
            continue
        pool = torch.where(bank['source_kind'] == cohort)[0] if cohort >= 0 else torch.arange(
            len(bank['station_xy']), device=device)
        if not len(pool):
            raise ValueError(f"question bank lacks cohort {cohort}")
        candidates = pool[torch.randint(len(pool), (len(ids), attempts), device=device)]
        distance = torch.linalg.norm(bank['station_xy'][candidates] - previous_xy[ids, None], dim=-1)
        valid = distance <= budget[ids, None] + 1e-6
        good = valid.any(-1)
        pick = valid.long().argmax(-1)
        result[ids[good]] = candidates[torch.arange(len(ids), device=device)[good], pick[good]]
        if cohort == -1:
            fallback[ids[good]] = True
    if bool((result < 0).any()):
        # Rare narrow budgets use an exhaustive nearest-row query. This is
        # still a fixed external command; never clamp it to actual robot pose.
        ids = torch.where(result < 0)[0]
        for chunk in ids.split(16):
            distances = torch.cdist(previous_xy[chunk], bank['station_xy'])
            distance, nearest = distances.min(-1)
            if bool((distance > budget[chunk] + 1e-6).any()):
                raise RuntimeError("no qualified question within the commanded station travel budget")
            result[chunk] = nearest
            fallback[chunk] = True
    return result, choose_wide, fallback
