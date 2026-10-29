"""First-bounce placement metrics; misses and interrupted attempts stay in the denominator.

Coordinates are in the table-near-left frame. These diagnostics do not change
success_metric.SuccessRate or the reward, and also run with the reward disabled.
"""
from __future__ import annotations

import numpy as np


class LandingPlacementMetrics:
    def __init__(self, x_range, y_range):
        self.bounds = np.asarray([x_range, y_range], dtype=np.float64)
        if (self.bounds.shape != (2, 2) or not np.isfinite(self.bounds).all()
                or np.any(self.bounds[:, 0] >= self.bounds[:, 1])):
            raise ValueError('Placement bounds must be finite increasing XY pairs')
        self.sides = {name: dict(attempts=0, contacts=0, legal_returns=0, region_hits=0,
                                interrupted_before_strike=0, pre_strike_falls=0,
                                post_strike_falls=0, distances=[], center_errors=[])
                      for name in ('forehand', 'backhand')}

    def _side(self, side):
        if side not in self.sides:
            raise ValueError(f'Unknown strike side: {side}')
        return self.sides[side]

    def add(self, outcome, side):
        values = self._side(side)
        values['attempts'] += 1
        values['contacts'] += int(outcome.contacted)
        values['legal_returns'] += int(outcome.success)
        if not outcome.contacted or outcome.landing_xy is None:
            return
        xy = np.asarray(outcome.landing_xy, dtype=np.float64)
        if xy.shape != (2,) or not np.isfinite(xy).all():
            return
        distance = float(np.linalg.norm(xy - np.clip(xy, self.bounds[:, 0], self.bounds[:, 1])))
        values['distances'].append(distance)
        values['center_errors'].append(float(np.linalg.norm(xy - self.bounds.mean(axis=1))))
        # Do not score a virtual target hit that went through the net or left the table.
        values['region_hits'] += int(outcome.success and distance == 0.)

    def interrupt(self, side, *, fall):
        values = self._side(side)
        values['attempts'] += 1
        values['interrupted_before_strike'] += 1
        values['pre_strike_falls'] += int(fall)

    def post_strike_fall(self, side):
        self._side(side)['post_strike_falls'] += 1

    @staticmethod
    def _summary(values):
        result = {key: value for key, value in values.items()
                  if key not in ('distances', 'center_errors')}
        n = values['attempts']
        result['region_hit_rate'] = values['region_hits'] / n if n else 0.
        result['legal_return_rate'] = values['legal_returns'] / n if n else 0.
        result['contact_rate'] = values['contacts'] / n if n else 0.
        result['finite_contact_landing_count'] = len(values['distances'])
        # Null, not zero, means no finite contact landing was observed.
        for key, output in (('distances', 'region_distance_m'), ('center_errors', 'center_error_m')):
            result[output + '_p50_p95'] = (
                np.percentile(values[key], [50, 95]).tolist() if values[key] else None)
        return result

    def as_dict(self):
        total = {key: [] if isinstance(value, list) else 0
                 for key, value in self.sides['forehand'].items()}
        for values in self.sides.values():
            for key, value in values.items():
                total[key] += value
        return dict(self._summary(total), by_side={
            side: self._summary(values) for side, values in self.sides.items()},
            distance_population='Finite predicted first bounces after captured contact; hit rates include misses and pre-strike interruptions.')


class PlacementAttemptTracker:
    """Track pre-strike interruptions without treating a reset clock as a strike."""
    def __init__(self, metrics, time_to_strike, sides):
        self.metrics = metrics
        self.previous_tts = np.asarray(time_to_strike, dtype=float).copy()
        self.sides = np.asarray(sides).copy()
        self.pending = self.previous_tts >= 0.
        self.struck = np.zeros_like(self.pending)

    def advance(self, time_to_strike, sides, reset, terminated):
        tts = np.asarray(time_to_strike, dtype=float)
        reset = np.asarray(reset, dtype=bool)
        terminated = np.asarray(terminated, dtype=bool)
        current_sides = np.asarray(sides)
        if any(a.shape != self.previous_tts.shape for a in (tts, reset, terminated, current_sides)):
            raise ValueError('Attempt tracker batches must have matching shapes')
        hit_event = self.pending & (self.previous_tts >= 0.) & (tts <= 0.) & ~reset
        for i in np.flatnonzero(reset & self.pending):
            self.metrics.interrupt(str(self.sides[i]), fall=bool(terminated[i]))
        for i in np.flatnonzero(reset & self.struck & ~self.pending & terminated):
            self.metrics.post_strike_fall(str(self.sides[i]))
        self.pending[hit_event | reset] = False
        self.struck[hit_event] = True
        self.struck[reset] = False
        # Zero is a valid first strike frame, but a held zero must not rearm.
        rearm = (tts >= 0.) & ((self.previous_tts < 0.) | reset
                             | ((self.previous_tts == 0.) & (tts > 0.)))
        self.pending[rearm] = True
        self.struck[rearm] = False
        self.previous_tts = tts.copy()
        self.sides = current_sides.copy()
        return hit_event
