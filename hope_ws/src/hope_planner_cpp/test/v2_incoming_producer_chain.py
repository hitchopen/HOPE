#!/usr/bin/env python3
"""Raw camera -> production relays/packetizer -> shared owner -> actual decoder.

Synthetic HOME/registration and independent DOP853 truth; no robot actuation.
Each invocation freezes ten flight opportunities before starting any process.
"""
import argparse
import gc
import hashlib
import json
import math
import multiprocessing as mp
import signal
import subprocess
import time
from collections import Counter
from pathlib import Path

from hope_msgs.msg import BallFlightPacket, FlightPredictionV2, StrikeCandidateV1

from integrated_producer_chain import flight

from motion_capture_tracking_interfaces.msg import LatencyInfo, NamedPose, NamedPoseArrayV2

import numpy as np

from qualification_accounting import SourceCadence, read_stages, resources

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from scipy.optimize import brentq

from std_msgs.msg import Float64MultiArray

PREFIX = '/hope/shadow/v2_incoming/'
BASE_ID, WORLD_ID = int('b' * 13, 16), int('a' * 13, 16)


def source(rate, spin, stop, ready, reference_ready, queue):
    """Offer acquisition-timestamped independent truth at the declared rate."""
    rclpy.init()
    node = rclpy.create_node('incoming_independent_camera')
    camera = node.create_publisher(NamedPoseArrayV2, '/optitrack/poses', qos_profile_sensor_data)
    home = node.create_publisher(Float64MultiArray, '/test/incoming/home', 10)
    before, after, bounce = flight(10.0, spin)
    crossing = brentq(lambda t: after.sol(t)[0] - 0.08, bounce, 0.7)
    deadline = time.monotonic() + 10
    while not camera.get_subscription_count() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)
    if not camera.get_subscription_count():
        raise RuntimeError('camera relay discovery failed')
    rows = []
    home_sequence = 0

    def home_receipt():
        nonlocal home_sequence
        home_sequence += 1
        receipt = Float64MultiArray()
        receipt.data = [
            2.0,
            100.0,
            -0.5,
            -0.7625,
            0.58,
            float(BASE_ID),
            float(WORLD_ID),
            5.0,
            float(home_sequence),
            1.0,
        ]
        home.publish(receipt)

    def body_message(stamp):
        msg = NamedPoseArrayV2()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
        msg.header.frame_id = 'world'
        latency = LatencyInfo()
        latency.source, latency.latency = 'NatNetClockUncertainty', 20
        msg.latencies.append(latency)
        body = NamedPose()
        body.name = 'UCB_P1'
        body.pose.position.x, body.pose.position.y = -0.5, -0.7625
        body.pose.orientation.w = 1.0
        msg.poses.append(body)
        return msg

    # DDS discovery is not a qualified flight opportunity. Offer no ball until
    # the actual planner confirms that it has the registered HOME/reference.
    setup_started = time.time_ns()
    deadline = time.monotonic() + 10
    while not reference_ready.is_set() and time.monotonic() < deadline:
        camera.publish(body_message(time.time_ns()))
        home_receipt()
        rclpy.spin_once(node, timeout_sec=0.01)
    if not reference_ready.is_set():
        raise RuntimeError('planner HOME/reference readiness failed before offering balls')
    origin_wall, origin_mono = time.time_ns(), time.monotonic_ns()
    cadence = SourceCadence(rate, origin_mono)
    gc.disable()
    ready.set()
    index = 0
    next_home = 0
    while not stop.is_set():
        intended = cadence.deadline_ns
        remaining = (intended - time.monotonic_ns()) * 1e-9
        if remaining > 0:
            time.sleep(remaining)
        acquisition_steady_ns = time.monotonic_ns()
        stamp = time.time_ns()
        cadence.acquired(acquisition_steady_ns)
        elapsed = (stamp - origin_wall) * 1e-9
        ordinal = math.floor((elapsed - 0.5) / 0.9)
        phase = elapsed - 0.5 - ordinal * 0.9
        msg = body_message(stamp)
        present = 0 <= ordinal < 10 and phase < crossing + 0.025
        eligible = present and 0.100 < phase < crossing - 0.030
        state = None
        if present:
            state = before.sol(phase) if phase < bounce else after.sol(phase)
            ball = NamedPose()
            ball.name = 'Ball'
            ball.pose.position.x, ball.pose.position.y, ball.pose.position.z = map(
                float, state[:3]
            )
            if spin != 'unknown':
                ball.pose.orientation.y = float(np.sin(state[6] / 2))
                ball.pose.orientation.w = float(np.cos(state[6] / 2))
            msg.poses.append(ball)
        camera.publish(msg)
        if elapsed >= next_home:
            home_receipt()
            next_home = elapsed + 0.02
        rows.append(
            {
                'source_ns': stamp,
                'offered_ns': time.time_ns(),
                'intended_steady_ns': intended,
                'actual_steady_ns': time.monotonic_ns(),
                'acquisition_steady_ns': acquisition_steady_ns,
                'flight': ordinal + 1,
                'phase': phase,
                'present': present,
                'eligible_continuous': eligible,
                'state': state.tolist() if state is not None else None,
            }
        )
        index += 1
    queue.put(
        {
            'rows': rows,
            'origin_ns': origin_wall,
            'bounce_s': bounce,
            'crossing_s': crossing,
            'contact_state': after.sol(crossing).tolist(),
            'rate_hz': rate,
            'cadence': cadence.receipt(),
            'reference_ready_ns': origin_wall,
            'reference_wait_ms': (origin_wall - setup_started) * 1e-6,
        }
    )
    node.destroy_node()
    rclpy.shutdown()


def run(args):
    """Retain every declared opportunity and every process exit, including failures."""
    args.output.mkdir(parents=True, exist_ok=False)
    contract = {
        'version': 3,
        'source_cadence': 'resynchronize without catch-up; actual rate >=95% requested; spacing >=half period',
        'rate_hz': args.rate,
        'mode': args.mode,
        'spin': args.spin,
        'one_shot_opportunities': 10,
        'startup': 'actual planner HOME=100 and registered plane available before first ball',
        'flight_spacing_s': 0.9,
        'continuous_denominator': (
            'every offered qualified ball source epoch; phase>0.100; crossing lead>0.030'
        ),
        'coverage_min': 0.95,
        'publication_completion_age_max_ms': 30.0,
        'decoder_p95_max_ms': 30.0,
        'full_trajectory_rate_max_hz': 50.0,
        'source_expiry_notification_delay_max_ms': 30.0,
        'retry_or_exclusion_on_failure': False,
        'binary_sha256': {
            p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [
                args.build / 'hope_planner_cpp_node',
                args.build / 'hope_ball_flight_packetizer',
                args.build / 'hope_integrated_runner_probe',
            ]
        },
    }
    (args.output / 'contract.json').write_text(json.dumps(contract, indent=2) + '\n')
    processes, logs = [], []
    received = {
        k: []
        for k in [
            'predictions',
            'full',
            'candidates',
            'revocations',
            'commands',
            'decoded',
            'packets',
        ]
    }
    stats, exception, usage = {}, None, {}
    stop, ready, queue = mp.Event(), mp.Event(), mp.Queue()
    reference_ready = mp.Event()
    camera = node = None

    def launch(exe, parameters=(), configs=()):
        command = [str(exe), '--ros-args']
        for config in configs:
            command += ['--params-file', str(config)]
        for key, value in parameters:
            command += ['-p', f'{key}:={value}']
        log = (args.output / (exe.name + '.log')).open('w')
        logs.append(log)
        process = subprocess.Popen(
            command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
        )
        processes.append(process)

    try:
        config = args.repo / 'hope_ws/src/hope_planner_cpp/config'
        launch(
            args.bringup / 'optitrack_mct_relay',
            [
                ('publish_tf', 'false'),
                ('input_history_depth', 8),
                ('ball_calibration_id', 'SYNTHETIC_INCOMING'),
                ('ball_orientation_authoritative', str(args.spin != 'unknown').lower()),
                ('planner_poses_topic', '/poses/planner_full'),
                ('test.stage_trace_path', args.output / 'relay.tsv'),
            ],
        )
        launch(
            args.bringup / 'hope_base_pose_flat_relay',
            [
                ('extrinsic_calibrated', 'true'),
                ('world_frame_calibrated', 'true'),
                ('calibration_sha256', 'b' * 64),
                ('world_frame_sha256', 'a' * 64),
                ('source_stamp_mode', 'input_header'),
                ('policy_z_offset', 0.76),
            ],
        )
        launch(
            args.build / 'hope_planner_cpp_node',
            [
                ('v2_incoming.session_id', 'incoming-chain'),
                ('v2_incoming.frame_id', 'world'),
                ('v2_incoming.calibration_id', 'SYNTHETIC_INCOMING'),
                ('v2_incoming.base_calibration_id', BASE_ID),
                ('v2_incoming.base_world_id', WORLD_ID),
                ('v2_incoming.stage_trace_path', args.output / 'planner.tsv'),
                ('session_home_input_topic', '/test/incoming/home'),
                ('plane_reference_mode', args.mode),
            ],
            [config / 'model21800_hardware.yaml', config / 'v2_incoming.shadow.yaml'],
        )
        launch(
            args.build / 'hope_ball_flight_packetizer',
            [
                ('session_id', 'incoming-chain'),
                ('producer_instance_id', 'incoming-chain-packetizer'),
                ('flight_packet_topic', PREFIX + 'flight_packet'),
                ('adaptive_hit_plane_topic', PREFIX + 'hit_plane'),
                ('debug_csv_path', args.output / 'packetizer.csv'),
            ],
            [config / 'model21800_flight_packetizer.yaml'],
        )
        launch(
            args.build / 'hope_integrated_runner_probe',
            [
                ('test.expected_schema', 4),
                ('test.input_topic', PREFIX + 'command_flat'),
                ('test.stage_trace_path', args.output / 'decoder.tsv'),
            ],
        )
        rclpy.init()
        node = rclpy.create_node('incoming_chain_observer')
        qos = QoSProfile(depth=256, reliability=ReliabilityPolicy.BEST_EFFORT)

        def prediction(m, key='predictions'):
            received[key].append(
                {
                    'source_ns': m.prediction_reference_time_ns,
                    'revision': m.forecast_revision,
                    'generation': m.shot_generation,
                    'physical': m.current_state_available,
                    'target_valid': m.target_valid,
                    'status': m.forecast_status,
                    'reason': m.reason,
                    'receipt_ns': time.time_ns(),
                    'published_ns': m.published_at_ns,
                    'waypoints': len(m.waypoints),
                }
            )

        def candidate(m, key='candidates'):
            if m.home_epoch == 100 and m.plane_available:
                reference_ready.set()
            received[key].append(
                {
                    'source_ns': m.source_time_ns,
                    'revision': m.prediction_revision,
                    'generation': m.physical_shot_generation,
                    'home': m.home_epoch,
                    'revokes': m.revokes_revision,
                    'command_sequence': m.command_sequence,
                    'command_flight': m.command_flight_id,
                    'supported': m.strike_contact_supported,
                    'eligible': m.command_eligible,
                    'status': m.status,
                    'reason': m.reason,
                    'plane_x': m.plane_x,
                    'tentative': m.plane_tentative,
                    'plane_flight': m.plane_flight_id,
                    'contact_ns': m.contact_time_ns,
                    'contact': [m.contact_position.x, m.contact_position.y, m.contact_position.z],
                    'receipt_ns': time.time_ns(),
                    'published_ns': m.published_at_ns,
                }
            )

        node.create_subscription(FlightPredictionV2, PREFIX + 'prediction', prediction, qos)
        node.create_subscription(
            FlightPredictionV2, PREFIX + 'trajectory', lambda m: prediction(m, 'full'), qos
        )
        node.create_subscription(StrikeCandidateV1, PREFIX + 'candidate', candidate, qos)
        node.create_subscription(
            StrikeCandidateV1,
            PREFIX + 'revocation',
            lambda m: candidate(m, 'revocations'),
            QoSProfile(
                depth=128,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        node.create_subscription(
            Float64MultiArray,
            PREFIX + 'command_flat',
            lambda m: received['commands'].append(list(m.data)),
            128,
        )
        node.create_subscription(
            Float64MultiArray,
            '/hope/test/integrated_runner/decoded',
            lambda m: received['decoded'].append(list(m.data)),
            128,
        )
        node.create_subscription(
            BallFlightPacket,
            PREFIX + 'flight_packet',
            lambda m: received['packets'].append(
                {
                    'sequence': m.flight_sequence,
                    'cutoff_ns': m.samples[-1].exposure_unix_stamp_ns,
                    'sample_count': len(m.samples),
                    'sample_sources': [s.exposure_unix_stamp_ns for s in m.samples],
                    'hash': m.payload_hash,
                }
            ),
            qos,
        )
        until = time.monotonic() + 1.0
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.01)
        assert all(p.poll() is None for p in processes), 'child startup failure'
        camera = mp.Process(
            target=source, args=(args.rate, args.spin, stop, ready, reference_ready, queue)
        )
        camera.start()
        deadline = time.monotonic() + 15
        while not ready.is_set() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
        assert ready.is_set(), 'camera startup failure'
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.002)
        stop.set()
        stats = queue.get(timeout=5)
        camera.join(timeout=5)
        assert not camera.is_alive(), 'camera shutdown failure'
        usage = resources(processes)
    except Exception as error:
        exception = repr(error)
    finally:
        stop.set()
        if camera and camera.is_alive():
            camera.terminate()
            camera.join(timeout=5)
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        for log in logs:
            log.close()
        if node:
            node.destroy_node()
            rclpy.shutdown()
        raw = {
            'source': stats,
            'received': received,
            'exception': exception,
            'resources': usage,
            'children': [{'command': p.args, 'exit': p.returncode} for p in processes],
        }
        (args.output / 'raw.json').write_text(json.dumps(raw) + '\n')
    # Exact source-epoch coverage, with a separate committed-flight denominator.
    eligible = {row['source_ns'] for row in stats.get('rows', []) if row['eligible_continuous']}
    covered = {row['source_ns'] for row in received['predictions'] if row['physical']} & eligible
    commands = [row for row in received['commands'] if row[1] == 1.0]
    command_ids = {(row[19], row[15]) for row in commands}
    transport_latencies = [
        (int(row[13]) * 10**9 + int(row[14]) - (int(wire[12]) * 10**9 + int(wire[13]))) * 1e-6
        for row in received['decoded']
        for wire in commands
        if row[0] == wire[14]
    ]
    traces = []
    path = args.output / 'planner.tsv'
    if path.exists():
        import csv

        with path.open() as stream:
            metadata = stream.readline()
            traces = list(csv.DictReader(stream, delimiter='\t'))
    else:
        metadata = 'missing'
    command_sources = {
        int(row['sequence']): int(row['source_ns'])
        for row in traces
        if row['stage'] == 'command_published'
    }
    latencies = [
        (int(row[13]) * 10**9 + int(row[14]) - command_sources[int(row[0])]) * 1e-6
        for row in received['decoded']
        if int(row[0]) in command_sources
    ]
    published = [
        row
        for row in traces
        if row['stage'] in ['forecast_published', 'command_published']
        and int(row['source_ns']) > 0
    ]
    ages = [(int(row['wall_ns']) - int(row['source_ns'])) * 1e-6 for row in published]
    coverage = len(covered) / len(eligible) if eligible else 0
    p95 = float(np.percentile(latencies, 95)) if latencies else None
    full_sources = sorted({row['source_ns'] for row in received['full']})
    cadence_ok = all(b - a >= 20000000 for a, b in zip(full_sources, full_sources[1:]))
    by_source = {}
    for row in traces:
        by_source.setdefault(int(row['source_ns']), []).append(row)
    relay_metadata, relay_rows = read_stages(args.output / 'relay.tsv')
    decoder_metadata, decoder_rows = read_stages(args.output / 'decoder.tsv')
    for row in relay_rows:
        by_source.setdefault(int(row['source_ns']), []).append(row)
    missed = []
    for stamp in sorted(eligible - covered):
        rows = by_source.get(stamp, [])
        stages = {row['stage'] for row in rows}
        loss = (
            'OBSERVER_MISSING'
            if 'forecast_published' in stages
            else (
                'WORKER_RETIRED'
                if 'forecast_retired' in stages
                else (
                    'PENDING_SUPERSEDED'
                    if 'forecast_superseded' in stages
                    else (
                        'FORECAST_UNAVAILABLE'
                        if 'forecast_completed' in stages
                        else (
                            'PLANNER_RECEIVED_NO_FORECAST'
                            if 'planner_received' in stages
                            else (
                                'RELAY_TO_PLANNER'
                                if 'typed_published' in stages
                                else 'SOURCE_TO_RELAY'
                            )
                        )
                    )
                )
            )
        )
        missed.append({'source_ns': stamp, 'first_observed_loss': loss, 'stages': rows})
    source_flights = {
        row['source_ns']: row['flight'] for row in stats.get('rows', []) if row['present']
    }
    generations = {}
    for row in received['predictions']:
        if row['source_ns'] in eligible and row['physical']:
            ordinal = source_flights[row['source_ns']]
            generations.setdefault(ordinal, set()).add(row['generation'])
    command_flights = [source_flights.get(stamp) for stamp in command_sources.values()]
    expiries = [
        row
        for row in received['revocations']
        if row['status'] == 'EXPIRED' and row['reason'] == 'SOURCE_EXPIRED'
    ]
    expiry_ms = [(row['published_ns'] - row['source_ns']) * 1e-6 - 30 for row in expiries]
    contact_errors = []
    for row in received['candidates']:
        if row['source_ns'] not in eligible or not row['supported']:
            continue
        ordinal = source_flights[row['source_ns']]
        truth_ns = stats['origin_ns'] + round(
            (0.5 + (ordinal - 1) * 0.9 + stats['crossing_s']) * 1e9
        )
        contact_errors.append(
            {
                'source_ns': row['source_ns'],
                'revision': row['revision'],
                'arrival_error_ms': (row['contact_ns'] - truth_ns) * 1e-6,
                'position_error_m': (
                    np.array(row['contact']) - np.array(stats['contact_state'][:3])
                ).tolist(),
            }
        )
    gates = {
        'source_cadence': stats.get('cadence', {}).get('valid', False),
        'children': exception is None and all(p.returncode == 0 for p in processes),
        'continuous_coverage': coverage >= 0.95,
        'one_shot_coverage': len(command_ids) >= math.ceil(0.95 * 10),
        'one_shot_immutable': len(commands) == len(command_ids) <= 10,
        'schema_home_plane': bool(commands)
        and all(
            row[0] == (4 if args.mode == 'fixed' else 5)
            and row[19] == 100
            and abs(row[3] - 0.08) < 1e-8
            for row in commands
        ),
        'decoder': len(latencies) == len(commands) and p95 is not None and p95 <= 30,
        'completion_age': bool(ages) and min(ages) >= -1 and max(ages) <= 30,
        'full_cadence': bool(full_sources) and cadence_ok,
        'explicit_revocation': bool(received['revocations']),
        'stable_generations': len(generations) == 10
        and all(len(values) == 1 and next(iter(values)) > 0 for values in generations.values())
        and len(set.union(*generations.values())) == 10,
        'command_flight_identity': sorted(f for f in command_flights if f is not None)
        == list(range(1, 11)),
        'explicit_source_expiry': len(
            {source_flights.get(row['source_ns']) for row in expiries} - {None}
        )
        == 10
        and bool(expiry_ms)
        and min(expiry_ms) >= 0
        and max(expiry_ms) <= 30,
        'complete_stage_trace': 'overwritten=0' in metadata
        and int(relay_metadata.get('overwritten', -1)) == 0
        and int(decoder_metadata.get('overwritten', -1)) == 0
        and len(
            {int(row['sequence']) for row in decoder_rows if row['stage'] == 'decoder_receipt'}
            & command_sources.keys()
        )
        == len(commands),
    }
    counts = Counter(row['stage'] for row in traces)
    ball_rows = [row for row in stats.get('rows', []) if row['present']]
    exposure_seconds = sum(1 for row in ball_rows) / args.rate
    result = {
        'status': 'PASS' if all(gates.values()) else 'FAIL',
        'gates': gates,
        'continuous_eligible': len(eligible),
        'continuous_covered': len(covered),
        'continuous_coverage': coverage,
        'commands': len(commands),
        'packet_flights': len({row['sequence'] for row in received['packets']}),
        'publication_completion_age_max_ms': max(ages) if ages else None,
        'decoder_p95_ms': p95,
        'expiry_notification_delay_ms': expiry_ms,
        'generations_by_flight': {key: sorted(values) for key, values in generations.items()},
        'exception': exception,
        'transport_only_decoder_p95_ms': (
            float(np.percentile(transport_latencies, 95)) if transport_latencies else None
        ),
        'stage_counts': dict(counts),
        'rates_per_active_exposure_second': {
            key: counts[key] / exposure_seconds if exposure_seconds else 0
            for key in [
                'planner_received',
                'tracker_accepted',
                'tracker_rejected',
                'forecast_completed',
                'forecast_published',
                'candidate_published',
                'forecast_superseded',
            ]
        },
        'missing_source_epochs': sorted(eligible - covered),
        'distinct_forecast_sources_completed': len(
            {int(row['source_ns']) for row in traces if row['stage'] == 'forecast_completed'}
        ),
        'distinct_forecast_sources_published': len(
            {int(row['source_ns']) for row in traces if row['stage'] == 'forecast_published'}
        ),
    }
    (args.output / 'accounting.json').write_text(
        json.dumps(
            {'missed': missed, 'stage_counts': dict(counts), 'relay_metadata': relay_metadata},
            indent=2,
        )
        + '\n'
    )
    (args.output / 'synthetic-accuracy.json').write_text(
        json.dumps(
            {
                'truth': 'independent synthetic DOP853; not measured physical accuracy',
                'errors': contact_errors,
            },
            indent=2,
        )
        + '\n'
    )
    (args.output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--repo', type=Path, required=True)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--bringup', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rate', type=int, choices=[150, 300, 360], required=True)
    parser.add_argument('--mode', choices=['fixed', 'per_ball', 'stable_per_ball'], required=True)
    parser.add_argument('--spin', choices=['unknown', 'zero'], required=True)
    mp.set_start_method('spawn')
    raise SystemExit(run(parser.parse_args()))
