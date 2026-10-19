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
                                                        position_errors=[], velocity_errors=[], time_errors=[], physical=0,
                                                        position_coverage=[], velocity_coverage=[],
                                                        rejection_reasons={}))
        group['attempts'] += 1
        group['physical'] += int(row.get('physical_available', False))
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
        velocity_residual = None
        if 'velocity' in row and 'truth_velocity' in row:
            velocity_residual = np.asarray(row['velocity']) - row['truth_velocity']
            group['velocity_errors'].append(float(np.linalg.norm(velocity_residual)))
        if row.get('covariance') is not None:
            covariance = np.asarray(row['covariance'], dtype=float)
            if covariance.shape not in ((6, 6), (7, 7)):
                raise ValueError('event covariance must contain p/v or p/v/time')
            position_residual = np.asarray(row['position']) - row['truth_position']
            for residual, marginal, field in (
                    (position_residual, covariance[:3, :3], 'position_coverage'),
                    (velocity_residual, covariance[3:6, 3:6], 'velocity_coverage')):
                if residual is None or not np.all(np.isfinite(marginal)):
                    continue
                if np.linalg.eigvalsh((marginal + marginal.T) / 2).min() <= 0:
                    continue
                # 95% chi-square boundary for three dimensions. Conditional
                # uncertainty remains labelled conditional in the report.
                distance = float(residual @ np.linalg.solve(marginal, residual))
                group[field].append(distance <= 7.814727903251179)

    result = {}
    for mode, group in groups.items():
        result[mode] = dict(attempts=group['attempts'], valid=group['valid'],
                            physical_forecast_coverage=group['physical'] / group['attempts'],
                            velocity_rmse_mps=math.sqrt(np.mean(np.square(group['velocity_errors'])))
                            if group['velocity_errors'] else None,
                            conditional_position_95_coverage=float(np.mean(group['position_coverage']))
                            if group['position_coverage'] else None,
                            conditional_velocity_95_coverage=float(np.mean(group['velocity_coverage']))
                            if group['velocity_coverage'] else None,
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
