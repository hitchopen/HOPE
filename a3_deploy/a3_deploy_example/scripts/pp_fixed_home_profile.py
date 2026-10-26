"""Planner support derived from the production Runner's strict inspection receipt."""
import math


def planner_profile(inspection, home_topic='/hope/runner/session_home_hdu_flat'):
    if (inspection.get('station_contract') != 'fixed_session_anchor_v1'
            or inspection.get('policy_abi') != 'ball_clock_110_v1'):
        raise ValueError('fixed HOME ball-clock inspection required')
    boxes = inspection.get('position_support')
    velocities = inspection.get('velocity_support')
    for group in (boxes, velocities):
        if not isinstance(group, list) or len(group) != 2:
            raise ValueError('two support boxes required')
        for box in group:
            if len(box) != 6 or not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                                       and math.isfinite(v) for v in box):
                raise ValueError('finite six-component support required')
            if any(box[i] > box[i+1] for i in (0, 2, 4)):
                raise ValueError('unordered support')
    fh, bh = boxes
    # Runner uses each box centre as its default plane. Both sides must agree.
    plane = (fh[0] + fh[1]) / 2
    if abs(plane - (bh[0] + bh[1]) / 2) > 1e-12:
        raise ValueError('shared HOME requires a common selected plane')
    if not max(fh[0], bh[0]) <= plane <= min(fh[1], bh[1]):
        raise ValueError('selected plane outside joint support')
    if fh[3] > bh[2]:
        raise ValueError('stationary forehand/backhand partition must be ordered')
    return {'hope_planner': {'ros__parameters': {
        'racket_flat_schema': 4,
        'session_home_input_topic': home_topic,
        'swing_side_reference_mode': 'session_home_v1',
        'swing_side_fh_y_range': fh[2:4],
        'swing_side_bh_y_range': bh[2:4],
        'swing_side_split_y': (fh[3] + bh[2]) / 2,
        'swing_side_hysteresis_y': 0.04,
        'x_hit_follow_robot': False,
        'x_hit_freeze_session_home': False,
        'x_hit_bh_delta': 0.0,
        'plane_reference_mode': 'fixed',
        'reach_permission_enabled': False,
        'reach_permission_contract': 'disabled_zero_v1',
    }}}, plane
