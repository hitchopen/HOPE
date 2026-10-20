#!/usr/bin/env python3
"""Exercise installed v2 ROS transport, paced input, revocation and shadow fences."""
import argparse
from collections import Counter
import json
import math
import os
import statistics
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from hope_msgs.msg import BallObservationV2, FlightContextV2, FlightPredictionV2


def percentile(values, fraction):
    return sorted(values)[min(len(values) - 1, math.ceil(fraction * len(values)) - 1)]


def run(executable, rate, full_rate=50., worker_delay_ms=0):
    session = 'acceptance_' + str(rate)
    process = subprocess.Popen([executable, '--ros-args', '-p', 'session_id:=' + session,
                                '-p', 'synthetic_plane:=true', '-p', 'plane_policy:=ON_REFRESH',
                                '-p', 'full_output_rate_hz:=' + str(float(full_rate)),
                                '-p', 'test.worker_delay_ms:=' + str(worker_delay_ms)],
                               stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    node = rclpy.create_node('flight_v2_acceptance_' + str(rate))
    predictions, latency, trajectories = [], [], []
    compact_sizes, full_sizes = [], []
    received_full = []
    resources = {}
    def process_resources():
        try:
            with open(f'/proc/{process.pid}/status', encoding='utf-8') as stream:
                values = {line.split(':')[0]: line.split(':')[1].strip() for line in stream if ':' in line}
            resources.update(rss_kib=int(values['VmRSS'].split()[0]), peak_rss_kib=int(values['VmHWM'].split()[0]))
            with open(f'/proc/{process.pid}/stat', encoding='utf-8') as stream:
                fields = stream.read().rsplit(')', 1)[1].split()
            resources['node_cpu_s'] = (int(fields[11]) + int(fields[12])) / os.sysconf('SC_CLK_TCK')
        except (FileNotFoundError, KeyError):
            pass
    def receive_full(message):
        trajectories.append(message)
        received_full.append(time.time_ns())
        if len(trajectories) % 20 == 1:
            full_sizes.append(len(serialize_message(message)))
            process_resources()
    def receive(message):
        if message.session_id != session:
            return
        predictions.append(message)
        if len(predictions) % 30 == 1:
            compact_sizes.append(len(serialize_message(message)))
        if message.physical_available:
            latency.append((time.time_ns() - message.prediction_reference_time_ns) / 1e6)
    node.create_subscription(FlightPredictionV2, '/hope/shadow/flight_v2/prediction', receive, qos_profile_sensor_data)
    node.create_subscription(FlightPredictionV2, '/hope/shadow/flight_v2/trajectory', receive_full, qos_profile_sensor_data)
    observation = node.create_publisher(BallObservationV2, '/hope/ball/observation_v2', qos_profile_sensor_data)
    context = node.create_publisher(FlightContextV2, '/hope/shadow/flight_v2/context', 32)
    def spin(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.001)
    try:
        spin(1.0)
        assert process.poll() is None, process.stderr.read()
        start = time.time_ns()
        boundary = FlightContextV2()
        boundary.session_id = session
        boundary.authority = 'fixture'
        boundary.operation = 'START'
        boundary.boundary_evidence = 'RACKET_CONTACT'
        boundary.direction = -1
        boundary.history_continuity = 'complete'
        boundary.source_time_ns = start
        context.publish(boundary)
        spin(.015)
        sent = 0
        eligible_sources = set()
        source_stamps = []
        flight_starts = []
        true_plane_times = []
        # Analytic drag-free ballistic input tests source-time transport/classification;
        # dynamics accuracy is independently tested against DOP853.
        # Multiple independent flights prevent a long invalid tail from obscuring coverage.
        for flight in range(4):
            boundary.source_time_ns = time.time_ns()
            context.publish(boundary)
            spin(.008)
            schedule = time.monotonic()
            flight_source_start = time.time_ns()
            flight_starts.append(flight_source_start)
            first_bounce = (.4 + math.sqrt(.4**2 + 2*9.81*(.5-.02)))/9.81
            bounce_vz = .4-9.81*first_bounce
            dv = min(.38744443855065164*4, .16706204822993975*(1+.9313775047019153)*abs(bounce_vz))
            true_plane_times.append(flight_source_start + round((first_bounce+(2-4*first_bounce-.15)/(4-dv))*1e9))
            for i in range(round(.60 * rate)):
                if round(.24 * rate) <= i < round(.24 * rate) + 2:
                    continue  # repeatable source dropout; timestamps preserve the gap
                target = schedule + i / rate + (.00015 if i % 3 == 0 else 0)
                while time.monotonic() < target:
                    rclpy.spin_once(node, timeout_sec=min(.001, target - time.monotonic()))
                message = BallObservationV2()
                message.source_time_ns = time.time_ns()
                # Position and acquisition time must describe the same physical
                # instant even when scheduling or deserialization runs late.
                t = (message.source_time_ns - flight_source_start) * 1e-9
                message.source_time_valid = True
                message.clock_domain = 'ros_system'
                message.clock_mapping_id = 'fixture_exact_wall'
                message.clock_mapping_valid = True
                message.sequence = sent + 1
                message.frame_id = 'table'
                message.ball_id = 'fixture'
                message.position_valid = True
                contact_time = (.4 + math.sqrt(.4 ** 2 + 2 * 9.81 * (.5 - .02))) / 9.81
                contact_vz = .4 - 9.81 * contact_time
                restitution = .9313775047019153
                impulse = min(.38744443855065164 * 4,
                              .16706204822993975 * (1 + restitution) * abs(contact_vz))
                if t <= contact_time:
                    message.position.x = 2.0 - 4.0 * t
                    message.position.z = .5 + .4 * t - 4.905 * t * t
                else:
                    dt = t - contact_time
                    message.position.x = 2.0 - 4.0 * contact_time + (-4 + impulse) * dt
                    message.position.z = .02 - restitution * contact_vz * dt - 4.905 * dt * dt
                message.position.y = -.7
                # Identity values carry an authoritative missing bit through transport.
                message.orientation.w = 1.
                message.orientation_authoritative = True
                message.orientation_valid = False
                if i >= 4:
                    eligible_sources.add(message.source_time_ns)
                source_stamps.append(message.source_time_ns)
                observation.publish(message)
                sent += 1
                rclpy.spin_once(node, timeout_sec=0)
            spin(.015)
        spin(.06)
        forecasts = [p for p in predictions if p.physical_available]
        assert forecasts, 'no physical forecasts'
        for message in forecasts:
            assert message.schema_version == 3
            assert message.shot_id == message.shot_generation
            assert message.current_state_available
            assert message.current_state.prediction_reference_time_ns == message.prediction_reference_time_ns
            assert message.current_state.arrival_dt_ns == 0
            assert not message.current_state.spin_available
            assert message.current_state.position.z < .76, 'physical state received policy height offset'
            assert message.model_config_id.startswith('flight-model-v3:sha256:')
            assert message.physics_profile == 'thin-shell-forward-v3-receipt-20261010'
            for event in message.events:
                assert event.event_time_ns == event.prediction_reference_time_ns + event.arrival_dt_ns
        assert all(p.target_event_id > 0 and p.target_kind for p in forecasts if p.target_valid)
        assert all(p.target_spin_mode in ('POSITION_ONLY', '') for p in forecasts)
        assert all(p.published_at_ns >= p.prediction_reference_time_ns for p in forecasts)
        assert trajectories and any(p.waypoints for p in trajectories), 'full trajectory output missing'
        for trajectory in trajectories:
            for waypoint in trajectory.waypoints:
                assert waypoint.predicted_arrival_time_ns == waypoint.prediction_reference_time_ns + waypoint.arrival_dt_ns
        by_generation = {}
        for p in forecasts:
            key = p.shot_generation
            assert p.forecast_revision > by_generation.get(key, -1), 'nonmonotone revision'
            by_generation[key] = p.forecast_revision
            for waypoint in p.waypoints:
                assert waypoint.predicted_arrival_time_ns == waypoint.prediction_reference_time_ns + waypoint.arrival_dt_ns
        assert any(p.reason == 'forecast_expired' and not p.target_valid for p in predictions), 'expiry revocation missing'
        graph = node.get_publishers_info_by_topic('/hope/shadow/flight_v2/prediction')
        assert graph
        for topic in ('/racket_command', '/racket/command', '/racket_command_flat',
                      '/hope/shadow/flight_v2/decision'):
            assert not node.get_publishers_info_by_topic(topic), topic
        emitted = [name for name, _ in node.get_topic_names_and_types()
                   if node.get_publishers_info_by_topic(name) and
                   any(info.node_name == 'hope_flight_prediction_v2'
                       for info in node.get_publishers_info_by_topic(name))]
        assert set(emitted) <= {'/rosout', '/parameter_events',
            '/hope/shadow/flight_v2/prediction', '/hope/shadow/flight_v2/trajectory',
            '/hope/shadow/flight_v2/packet'}, emitted
        # Count physical outputs against inputs after a four-sample startup allowance per flight.
        denominator = len(eligible_sources)
        eligible_outputs = {p.prediction_reference_time_ns for p in forecasts} & eligible_sources
        coverage = len(eligible_outputs) / denominator
        assert percentile(latency, .95) <= 30, ('receipt P95 exceeds 30 ms', latency)
        assert all((p.published_at_ns - p.prediction_reference_time_ns) <= 30_000_000 for p in forecasts)
        source_span_s = (source_stamps[-1] - source_stamps[0]) * 1e-9
        observed_inputs = max(p.input_count for p in predictions)
        publication_latency = [(p.published_at_ns - p.prediction_reference_time_ns) / 1e6
                               for p in forecasts]
        def distribution(values):
            return dict(minimum=min(values), p50=statistics.median(values), p95=percentile(values,.95),
                        p99=percentile(values,.99), maximum=max(values)) if values else None
        full_physical = [p for p in trajectories if p.physical_available]
        full_sources = {p.prediction_reference_time_ns for p in full_physical} & eligible_sources
        full_publication = [(p.published_at_ns-p.prediction_reference_time_ns)/1e6 for p in full_physical]
        full_receipt = [(receipt-p.prediction_reference_time_ns)/1e6
                        for p,receipt in zip(trajectories,received_full) if p.physical_available]
        generation_latency = [(p.generated_at_ns-p.prediction_reference_time_ns)/1e6 for p in forecasts]
        process_resources()
        first_physical, first_target = [], []
        for start,truth in zip(flight_starts,true_plane_times):
            interval = [p for p in forecasts if start <= p.prediction_reference_time_ns < start+600_000_000]
            if interval: first_physical.append((truth-interval[0].published_at_ns)/1e6)
            usable = [p for p in interval if p.target_valid and not p.target_historical]
            if usable: first_target.append((truth-usable[0].published_at_ns)/1e6)
        assert all(0 <= age <= 30 for age in full_publication)
        report = dict(rate_hz=rate, requested_full_rate_hz=full_rate, worker_delay_ms=worker_delay_ms,
                    source_span_s=source_span_s, published_input_rate_hz=(sent-1)/source_span_s,
                    observed_input_rate_hz=max(0,observed_inputs-1)/source_span_s,
                    observed_input_count=observed_inputs, unaccounted_input_count=sent-observed_inputs,
                    input_count=sent, physical_outputs=len(forecasts), denominator=denominator,
                    physical_publication_coverage=coverage, target_outputs=sum(p.target_valid for p in forecasts),
                    full_physical_outputs=len(full_physical), full_source_coverage=len(full_sources)/denominator,
                    full_received_rate_hz=(len(trajectories)-1)/((received_full[-1]-received_full[0])*1e-9),
                    full_publication_latency_ms=distribution(full_publication), full_receipt_latency_ms=distribution(full_receipt),
                    generation_latency_ms=distribution(generation_latency), publication_latency_ms=distribution(publication_latency),
                    latency_ms=distribution(latency), superseded=max(p.superseded_count for p in forecasts),
                    queue_bound=dict(running=1,pending=1), resources=resources,
                    lifecycle_reasons=dict(Counter(p.reason for p in predictions if not p.physical_available)),
                    generations=sorted({p.shot_generation for p in predictions}),
                    payload=dict(compact_mean_cdr_bytes=statistics.mean(compact_sizes),
                                 full_mean_cdr_bytes=statistics.mean(full_sizes),
                                 full_sampled_cdr_bytes_per_s=statistics.mean(full_sizes)*len(trajectories)/source_span_s,
                                 size_measurement='sampled CDR; excludes DDS/network headers'),
                    first_physical_fixture_lead_ms=first_physical, first_target_fixture_lead_ms=first_target)
        print(json.dumps(report),flush=True)
        assert coverage >= .95, ('physical publication coverage', coverage, len(forecasts), denominator)
        return report

    finally:
        node.destroy_node()
        process.terminate()
        try:
            process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--executable', required=True)
    parser.add_argument('--output')
    parser.add_argument('--full-rate',type=float,default=50.)
    parser.add_argument('--rates',type=int,nargs='+',default=[150,300])
    parser.add_argument('--worker-delay-ms',type=int,default=0)
    args = parser.parse_args()
    rclpy.init()
    try:
        results = [run(args.executable, rate,args.full_rate,args.worker_delay_ms) for rate in args.rates]
        bad = subprocess.run([args.executable, '--ros-args', '-r',
                              '/hope/shadow/flight_v2/prediction:=/racket_command'],
                             capture_output=True, text=True, timeout=10)
        assert bad.returncode != 0 and 'shadow namespace' in bad.stderr, bad.stderr
        print(json.dumps(dict(status='PASS', streams=results)))
        if args.output:
            with open(args.output, 'w', encoding='utf-8') as output:
                json.dump(dict(status='PASS', streams=results), output, indent=2)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
