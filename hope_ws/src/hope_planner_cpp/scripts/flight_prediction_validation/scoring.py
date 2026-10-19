"""Score explicitly matched independent events without erasing rejected forecasts.

Input records must be matched by session, physical flight, source information and
annotated event before calling this module. Adjacent-frame random splits and
matching a forecast to truth by minimizing its own error are unsupported.
"""
import math
import numpy as np


def score_matched_events(records):
    """Retain all attempted decisions in coverage and report spin groups separately."""
    groups = {}
    for row in records:
        group = groups.setdefault(row['spin_mode'], dict(attempts=0, valid=0,
                                                        position_errors=[], time_errors=[],
                                                        rejection_reasons={}))
        group['attempts'] += 1
        if not row['target_valid']:
            reason = row['reason']
            group['rejection_reasons'][reason] = group['rejection_reasons'].get(reason, 0) + 1
            continue
        if not row.get('independent_truth', False):
            raise ValueError('independent event annotation is required for physical scoring')
        if row['prediction_reference_time_ns'] > row['information_cutoff_ns']:
            raise ValueError('forecast uses information beyond the matched source cutoff')
        group['valid'] += 1
        group['position_errors'].append(float(np.linalg.norm(
            np.asarray(row['position']) - np.asarray(row['truth_position']))))
        group['time_errors'].append(abs(row['arrival_ns'] - row['truth_arrival_ns']) * 1e-9)
    result = {}
    for mode, group in groups.items():
        result[mode] = dict(attempts=group['attempts'], valid=group['valid'],
                            valid_coverage=group['valid'] / group['attempts'],
                            position_rmse_m=math.sqrt(np.mean(np.square(group['position_errors'])))
                            if group['valid'] else None,
                            time_mae_s=float(np.mean(group['time_errors'])) if group['valid'] else None,
                            rejection_reasons=group['rejection_reasons'])
    return result


def interpolate_without_impact_bridge(times, positions, query, contacts):
    """Exclude interpolation intervals that contain an impulse or lack a bracket."""
    right = int(np.searchsorted(times, query, side='right'))
    if right == 0 or right >= len(times):
        return None
    left = right - 1
    if any(times[left] <= contact <= times[right] for contact in contacts):
        return None
    fraction = (query - times[left]) / (times[right] - times[left])
    return np.asarray(positions[left]) + fraction * (np.asarray(positions[right]) - positions[left])
