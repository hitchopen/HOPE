#!/usr/bin/env python3
"""Identical recorded source cutoffs through library replay and installed ROS.

This typed-input test complements (does not replace) the raw MCT/real decoder
chain. The independent plant is shared only with that test's synthetic source.
"""
import argparse
import json
from pathlib import Path
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from hope_msgs.msg import BallObservationV2, FlightPredictionV2, FlightPacketV2
from integrated_producer_chain import flight


def vector(v):
    return [v.x, v.y, v.z]


def run(args, rate, known, zero=False):
    before, after, bounce = flight(10., 'zero' if zero else 'known' if known else 'unknown')
    process = subprocess.Popen([str(args.node), '--ros-args', '-p', 'synthetic_plane:=true',
        '-p', 'source_clock_domain:=simulation', '-p', 'session_id:=cutoff_parity'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    node = rclpy.create_node('review_cutoff_observer')
    pub = node.create_publisher(BallObservationV2, '/hope/ball/observation_v2', qos_profile_sensor_data)
    received, rows, packets = [], [], []
    node.create_subscription(FlightPredictionV2, '/hope/shadow/flight_v2/prediction',
                             received.append, qos_profile_sensor_data)
    node.create_subscription(FlightPacketV2, '/hope/shadow/flight_v2/packet',
                             packets.append, qos_profile_sensor_data)
    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.0005)
    try:
        until = time.monotonic()+8
        while not pub.get_subscription_count() and time.monotonic() < until:
            spin(.01)
        assert pub.get_subscription_count()
        # Allow the independently discovered prediction subscription to match.
        spin(.1)
        warm = BallObservationV2()
        warm.source_time_ns, warm.source_time_valid = time.time_ns(), True
        warm.position_valid = False
        pub.publish(warm)
        spin(.1)
        start = time.monotonic()
        for i in range(int(.31*rate)):
            deadline = start+i/rate
            spin(max(0., deadline-time.monotonic()))
            t = time.monotonic()-start
            state = before.sol(t) if t < bounce else after.sol(t)
            stamp = time.time_ns()
            omega = (30. if t < bounce else
                     float((after.sol(bounce+.001)[6]-after.sol(bounce)[6])/.001)) if known else 0.
            if zero and t < bounce:
                omega = 0.
            msg = BallObservationV2()
            msg.source_time_ns, msg.source_time_valid = stamp, True
            msg.frame_id, msg.clock_domain, msg.ball_id = 'table', 'simulation', 'fixture'
            msg.clock_mapping_id, msg.clock_mapping_valid = 'recorded', True
            msg.position_valid = True
            msg.position.x, msg.position.y, msg.position.z = map(float, state[:3])
            msg.omega_valid, msg.omega_provenance = known, 'recorded_producer_validity'
            msg.omega_rad_s.y = omega
            pub.publish(msg)
            rows.append([stamp, *map(float, state[:3]), int(known), 0., omega, 0.])
        spin(.03)
        stem = args.output.parent/f'{args.output.stem}-{rate}-{known}'
        if zero:
            stem = stem.with_name(stem.name+'-producer-zero')
        stem.with_name(stem.name+'-published.csv').write_text(
            '\n'.join(','.join(map(str, row)) for row in rows)+'\n')
        # Depth-one ROS input can lose a frame during a source scheduling
        # burst. Compare identical *consumed* prefixes, reconstructed from the
        # actual checkpoint observation ledger, rather than replaying a frame
        # ROS never received. Raw producer tests separately score every sent
        # eligible epoch; no loss is removed from their coverage denominator.
        consumed = {o.source_time_ns: o for p in packets for o in p.observations}
        assert consumed, 'no actual observation ledger received'
        last_ledger_source = max(consumed)
        published_rows = rows
        rows = [row for row in published_rows if row[0] in consumed]
        for row in rows:
            observation = consumed[row[0]]
            assert np.array_equal(row[1:4], vector(observation.position))
            assert bool(row[4]) == observation.omega_valid
        csv = '\n'.join(','.join(map(str, row)) for row in rows)+'\n'
        stem.with_suffix('.csv').write_text(csv)
        replay = subprocess.run([str(args.replay), '--observations'], input=csv,
                                capture_output=True, text=True, check=True)
        stem.with_suffix('.jsonl').write_text(replay.stdout)
        expected = {x['source_ns']: x for x in map(json.loads, replay.stdout.splitlines())}
        matched = before_count = after_count = events = 0
        max_state = max_cov = max_event = 0.
        for wire in received:
            ns = wire.prediction_reference_time_ns
            if not wire.physical_available or not wire.current_state_available or ns not in expected:
                continue
            reference = expected[ns]
            assert reference['physical_available']
            state = reference['current_state']
            error = np.max(np.abs(np.array(vector(wire.current_state.position)+vector(wire.current_state.velocity))-
                                     np.array(state['position']+state['velocity'])))
            cov_error = np.max(np.abs(np.array(wire.current_state.covariance)-state['covariance']))
            max_state, max_cov = max(max_state, float(error)), max(max_cov, float(cov_error))
            assert error < 1e-9 and cov_error < 1e-9, (ns, error, cov_error)
            if state['omega_rad_s'] is not None:
                assert wire.current_state.spin_available
                assert np.allclose(vector(wire.current_state.omega_rad_s), state['omega_rad_s'], atol=1e-9, rtol=0)
                assert wire.current_state.spin_measurement_time_ns <= ns
            else:
                assert not wire.current_state.spin_available
            for event in wire.events:
                if (event.status != 'PREDICTED' or event.last_prediction_reference_time_ns != ns):
                    continue
                candidates = [e for e in reference['future_events'] if e['kind'] == event.kind]
                assert candidates
                ref = min(candidates, key=lambda e: abs(e['arrival_ns']-event.event_time_ns))
                assert abs(ref['arrival_ns']-event.event_time_ns) <= 1
                event_error = np.max(np.abs(np.array(vector(event.position)+vector(event.pre_velocity))-
                                             np.array(ref['position']+ref['velocity'])))
                max_event = max(max_event, float(event_error))
                assert event_error < 1e-8
                if ref['covariance'] is not None:
                    assert np.allclose(event.covariance, np.array(ref['covariance']).reshape(-1), atol=1e-8, rtol=0)
                events += 1
            matched += 1
            before_count += int(ns < rows[0][0]+bounce*1e9)
            after_count += int(ns > rows[0][0]+(bounce+.02)*1e9)
        assert matched >= 20 and before_count >= 5 and after_count >= 5 and events >= 20, (
            matched, before_count, after_count, events)
        return {'rate_hz': rate, 'qualified_spin': known, 'producer_zero_before_contact': zero, 'matched_cutoffs': matched,
                'pre_bounce_cutoffs': before_count, 'post_bounce_cutoffs': after_count,
                'compared_events': events, 'max_state_error': max_state,
                'max_covariance_error': max_cov, 'max_event_state_error': max_event,
                'all_source_rows': len(published_rows), 'consumed_source_rows': len(rows),
                'source_rows_missing_before_last_ledger': [row[0] for row in published_rows
                    if row[0] <= last_ledger_source and row[0] not in consumed],
                'uncovered_tail_rows': [row[0] for row in published_rows if row[0] > last_ledger_source],
                'fixture': str(stem.with_suffix('.csv'))}
    finally:
        process.terminate()
        process.wait(timeout=5)
        node.destroy_node()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--node', type=Path, required=True)
    p.add_argument('--replay', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    try:
        results = [run(args, rate, known, zero) for rate in [150, 300]
                   for known, zero in [(False, False), (True, False), (True, True)]]
        args.output.write_text(json.dumps({'status': 'PASS', 'results': results}, indent=2)+'\n')
        print(json.dumps(results))
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
