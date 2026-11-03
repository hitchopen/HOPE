#!/usr/bin/env python3
"""Compare the installed HOPE solver with an independent DOP853 event reference."""
import argparse
import json
import subprocess
import numpy as np
from scipy.integrate import solve_ivp

KD = 0.12861635416914574
KM = 0.00379260504480258
E = 0.9313775047019153
KT = 0.38744443855065164
MU = 0.16706204822993975
RADIUS = 0.02


def reference(direction, speed, spin_y, duration):
    """Integrate free flights and independently apply the documented contact law."""
    state = np.array([.15 if direction > 0 else 2.59, -.7, .35,
                      direction * speed, .15, -.7], dtype=float)
    omega = np.array([5., spin_y, 8.]) if spin_y else None
    time = 0.
    contacts = []
    while time < duration - 1e-10:
        def rhs(_, value):
            velocity = value[3:]
            acceleration = np.array([0., 0., -9.81]) - KD * np.linalg.norm(velocity) * velocity
            if omega is not None:
                acceleration += KM * np.cross(omega, velocity)
            return np.concatenate([velocity, acceleration])

        def contact(_, value):
            return value[2] - RADIUS
        contact.direction = -1
        contact.terminal = True
        solution = solve_ivp(rhs, (time, duration), state, method='DOP853',
                             rtol=2e-12, atol=2e-14, max_step=.0002, events=contact)
        state = solution.y[:, -1]
        time = solution.t[-1]
        if not len(solution.t_events[0]):
            break
        x, y = state[:2]
        if not (0 <= x <= 2.74 and -1.525 <= y <= 0):
            # Test envelope excludes edges/misses; the geometry suite covers those separately.
            break
        contacts.append((time, state[:3].copy()))
        slip = np.array([state[3], state[4], 0.])
        if omega is not None:
            slip += np.array([-RADIUS * omega[1], RADIUS * omega[0], 0.])
        speed_slip = np.linalg.norm(slip)
        impulse = min(KT * speed_slip, MU * (1 + E) * abs(state[5]))
        delta = -impulse * slip / speed_slip if speed_slip else np.zeros(3)
        state[3:5] += delta[:2]
        state[5] *= -E
        if omega is not None:
            omega += np.array([delta[1], -delta[0], 0.]) / ((2 / 3) * RADIUS)
        # Advance a tiny positive time to avoid re-rooting the contact at t0.
        state += rhs(time, state) * 1e-11
        time += 1e-11
    return contacts


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--executable', required=True)
    parser.add_argument('--output')
    args = parser.parse_args()
    results = []
    for direction in (-1, 1):
        for spin in (0., -30., 30.):
            for speed in (1.5, 3.):
                expected = reference(direction, speed, spin, .65)
                for step in (.001, .0005, .00025):
                    actual = json.loads(subprocess.check_output(
                        [args.executable, '--rollout', str(step), str(direction),
                         str(speed), str(spin), '.65'], text=True))
                    observed = [e for e in actual['events'] if e['kind'] == 'table_bounce']
                    # Only compare before a physical net collision; the reference has no net response.
                    stop = min([e['time'] for e in actual['events'] if e['kind'] == 'net_collision'] + [.65])
                    expected_prefix = [e for e in expected if e[0] <= stop]
                    assert len(observed) == len(expected_prefix), (direction, spin, speed, actual)
                    for event, (time, position) in zip(observed, expected_prefix):
                        error = float(np.linalg.norm(np.array(event['position']) - position))
                        time_error = abs(event['time'] - time)
                        assert error <= .001 and time_error <= .0005, (error, time_error)
                        results.append(dict(direction=direction, spin=spin, speed=speed,
                                            step_s=step, position_error_m=error, time_error_s=time_error))
    assert results
    receipt = dict(status='PASS', reference='independent scipy DOP853 rtol=2e-12 atol=2e-14',
                   events=len(results), max_position_error_m=max(r['position_error_m'] for r in results),
                   max_time_error_s=max(r['time_error_s'] for r in results), cases=results)
    report = json.dumps(receipt, indent=2)
    if args.output:
        with open(args.output, 'w', encoding='utf-8') as output:
            output.write(report + '\n')
    print(json.dumps({k: v for k, v in receipt.items() if k != 'cases'}))


if __name__ == '__main__':
    main()
