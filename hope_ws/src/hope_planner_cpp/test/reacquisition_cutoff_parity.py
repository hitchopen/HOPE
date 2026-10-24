#!/usr/bin/env python3
"""Identical cutoffs after actual rejection/rearm and quaternion-only readiness.

This typed-input test complements (does not replace) the raw MCT/real decoder
chain. The independent plant is shared only with that test's synthetic source.
"""

import argparse
import json
from pathlib import Path
import subprocess
import signal
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray
from hope_msgs.msg import BallObservationV2, FlightPredictionV2
from integrated_producer_chain import flight
from qualification_accounting import read_stages
from rosidl_runtime_py.convert import message_to_ordereddict


def vector(v):
    return [v.x, v.y, v.z]


def run(args, rate, known):
    before, after, bounce = flight(10.0, "known" if known else "unknown")
    stem = args.output.parent / f"{args.output.stem}-{rate}-{known}"
    stages = stem.with_suffix('.stages')
    node_log = stem.with_suffix('.node.log').open('w')
    process = subprocess.Popen(
        [
            str(args.node),
            "--ros-args",
            "-p",
            "synthetic_plane:=true",
            "-p",
            "source_clock_domain:=simulation",
            "-p",
            "automatic_rearm_enabled:=true",
            "-p",
            "session_id:=continuation_cutoff",
            "-p",
            "test.stage_trace_path:=" + str(stages),
        ],
        stdout=node_log,
        stderr=subprocess.STDOUT,
    )
    node = rclpy.create_node("review_cutoff_observer")
    pub = node.create_publisher(
        BallObservationV2, "/hope/ball/observation_v2", qos_profile_sensor_data
    )
    received, rows, diagnostics = [], [], {}
    node.create_subscription(
        FlightPredictionV2,
        "/hope/shadow/flight_v2/prediction",
        received.append,
        qos_profile_sensor_data,
    )

    def readiness(message):
        for d in message.status:
            values = {v.key: v.value for v in d.values}
            diagnostics[int(values["source_time_ns"])] = values

    node.create_subscription(
        DiagnosticArray,
        "/hope/shadow/flight_v2/readiness",
        readiness,
        qos_profile_sensor_data,
    )

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.0005)

    try:
        until = time.monotonic() + 8
        while not pub.get_subscription_count() and time.monotonic() < until:
            spin(0.01)
        assert pub.get_subscription_count()
        # Allow the independently discovered prediction subscription to match.
        spin(0.1)
        warm = BallObservationV2()
        warm.source_time_ns = time.time_ns()
        warm.source_time_valid = True
        warm.position_valid = False
        pub.publish(warm)
        spin(0.1)
        start = time.monotonic()
        source_origin = time.time_ns()
        for i in range(int(0.92 * rate)):
            deadline = start + i / rate
            spin(max(0.0, deadline - time.monotonic()))
            stamp = time.time_ns()
            t = (stamp - source_origin) * 1e-9
            local = t if t < 0.08 else t - 0.6
            if 0.08 <= t < 0.10:
                state = np.array([1.0, -0.7, -0.2, 0.0, 0.0, 0.0, 0.0])
            elif 0.10 <= t < 0.6:
                continue
            else:
                state = before.sol(local) if local < bounce else after.sol(local)
            omega = (
                (
                    30.0
                    if t < bounce
                    else float(
                        (after.sol(bounce + 0.001)[6] - after.sol(bounce)[6]) / 0.001
                    )
                )
                if known
                else 0.0
            )
            msg = BallObservationV2()
            msg.source_time_ns, msg.source_time_valid = stamp, True
            msg.frame_id, msg.clock_domain, msg.ball_id = (
                "table",
                "simulation",
                "fixture",
            )
            msg.clock_mapping_id, msg.clock_mapping_valid = "recorded", True
            msg.position_valid = True
            msg.position.x, msg.position.y, msg.position.z = map(float, state[:3])
            msg.omega_valid = False
            msg.orientation_valid = msg.orientation_authoritative = known
            msg.orientation.w = float(np.cos(state[6] / 2))
            msg.orientation.y = float(np.sin(state[6] / 2))
            pub.publish(msg)
            rows.append(
                [
                    stamp,
                    *map(float, state[:3]),
                    0,
                    0.0,
                    0.0,
                    0.0,
                    int(known),
                    int(known),
                    msg.orientation.w,
                    0.0,
                    msg.orientation.y,
                    0.0,
                ]
            )
        spin(0.03)
        # This is a same-consumed-prefix check. The raw producer chain separately
        # measures coverage over every offered eligible source opportunity.
        process.send_signal(signal.SIGINT)
        process.wait(timeout=5)
        assert process.returncode == 0
        metadata, receipts = read_stages(stages)
        assert int(metadata.get('overwritten', -1)) == 0, metadata
        offered_rows = rows
        by_source = {row[0]: row for row in offered_rows}
        consumed = [int(row['source_ns']) for row in receipts if row['stage'] == 'pipeline_observed']
        assert consumed and len(consumed) == len(set(consumed))
        assert set(consumed) <= by_source.keys()
        rows = [by_source[stamp] for stamp in consumed]
        stem.with_name(stem.name + '-published.csv').write_text(
            '\n'.join(','.join(map(str, row)) for row in offered_rows) + '\n')
        csv = "\n".join(",".join(map(str, row)) for row in rows) + "\n"
        stem.with_suffix(".csv").write_text(csv)
        replay = subprocess.run(
            [str(args.replay), "--set", "automatic_rearm_enabled=1", "--observations"],
            input=csv,
            capture_output=True,
            text=True,
            check=True,
        )
        stem.with_suffix(".jsonl").write_text(replay.stdout)
        expected = {
            x["source_ns"]: x for x in map(json.loads, replay.stdout.splitlines())
        }
        readiness_matches = 0
        for ns, d in diagnostics.items():
            ref = expected[ns]
            assert d["spin_readiness"] == ref["spin_readiness"]
            assert (
                abs(float(d["spin_collected_span_s"]) - ref["spin_collected_span_s"])
                < 1e-12
            )
            assert (
                int(d["spin_collected_increments"]) == ref["spin_collected_increments"]
            )
            assert int(d["source_origin_ns"]) == ref["source_origin_ns"]
            assert int(d["physical_generation"]) == ref["generation"]
            assert int(d["spin_measurement_ns"]) == ref["spin_measurement_ns"]
            readiness_matches += 1
        assert readiness_matches >= 10, readiness_matches
        matched = before_count = after_count = events = 0
        max_state = max_cov = max_event = 0.0
        for wire in received:
            ns = wire.prediction_reference_time_ns
            if (
                not wire.physical_available
                or not wire.current_state_available
                or ns not in expected
            ):
                continue
            reference = expected[ns]
            assert reference["physical_available"]
            assert reference["generation"] == wire.shot_generation
            state = reference["current_state"]
            error = np.max(
                np.abs(
                    np.array(
                        vector(wire.current_state.position)
                        + vector(wire.current_state.velocity)
                    )
                    - np.array(state["position"] + state["velocity"])
                )
            )
            cov_error = np.max(
                np.abs(np.array(wire.current_state.covariance) - state["covariance"])
            )
            max_state, max_cov = max(max_state, float(error)), max(
                max_cov, float(cov_error)
            )
            assert error < 1e-9 and cov_error < 1e-9, (ns, error, cov_error)
            if state["omega_rad_s"] is not None:
                assert wire.current_state.spin_available
                assert np.allclose(
                    vector(wire.current_state.omega_rad_s),
                    state["omega_rad_s"],
                    atol=1e-9,
                    rtol=0,
                )
                assert wire.current_state.spin_measurement_time_ns <= ns
            else:
                assert not wire.current_state.spin_available
            for event in wire.events:
                if (
                    event.status != "PREDICTED"
                    or event.last_prediction_reference_time_ns != ns
                ):
                    continue
                candidates = [
                    e for e in reference["future_events"] if e["kind"] == event.kind
                ]
                assert candidates
                ref = min(
                    candidates, key=lambda e: abs(e["arrival_ns"] - event.event_time_ns)
                )
                assert abs(ref["arrival_ns"] - event.event_time_ns) <= 1
                event_error = np.max(
                    np.abs(
                        np.array(vector(event.position) + vector(event.pre_velocity))
                        - np.array(ref["position"] + ref["velocity"])
                    )
                )
                max_event = max(max_event, float(event_error))
                assert event_error < 1e-8
                if ref["covariance"] is not None:
                    assert np.allclose(
                        event.covariance,
                        np.array(ref["covariance"]).reshape(-1),
                        atol=1e-8,
                        rtol=0,
                    )
                events += 1
            matched += 1
            before_count += int(
                rows[0][0] + 0.6e9 < ns < rows[0][0] + (0.6 + bounce) * 1e9
            )
            after_count += int(ns > rows[0][0] + (0.6 + bounce + 0.02) * 1e9)
        assert (
            matched >= 20 and before_count >= 5 and after_count >= 5 and events >= 20
        ), (matched, before_count, after_count, events)
        b_sources = [
            x
            for ns, x in expected.items()
            if ns > rows[0][0] + 0.6e9 and x["physical_available"]
        ]
        assert b_sources and all(x["generation"] > 1 for x in b_sources)
        if known:
            assert any(x["spin_readiness"] == "READY_MEASURED" for x in b_sources)
        else:
            assert all(x["spin_readiness"] == "UNAVAILABLE" for x in b_sources)
        return {
            "rate_hz": rate,
            "quaternion_only": known,
            "matched_cutoffs": matched,
            "readiness_cutoffs": readiness_matches,
            "pre_bounce_cutoffs": before_count,
            "post_bounce_cutoffs": after_count,
            "compared_events": events,
            "max_state_error": max_state,
            "max_covariance_error": max_cov,
            "max_event_state_error": max_event,
            "all_source_rows": len(offered_rows),
            "consumed_source_rows": len(rows),
            "unconsumed_source_rows": sorted(set(by_source) - set(consumed)),
            "actual_consumption_ledger": str(stages),
            "fixture": str(stem.with_suffix(".csv")),
        }
    finally:
        failure = sys.exc_info()[1]
        stem.with_name(stem.name + '-received.json').write_text(json.dumps({
            'exception': None if failure is None else str(failure),
            'command': process.args,
            'offered_rows': locals().get('offered_rows', rows),
            'consumed_rows': rows if 'consumed' in locals() else None,
            'diagnostics': diagnostics,
            'predictions': [message_to_ordereddict(message) for message in received],
        }, default=list, indent=2) + '\n')
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=5)
        node_log.close()
        node.destroy_node()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--node", type=Path, required=True)
    p.add_argument("--replay", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    try:
        results = [
            run(args, rate, known) for rate in [150, 300] for known in [False, True]
        ]
        args.output.write_text(
            json.dumps({"status": "PASS", "results": results}, indent=2) + "\n"
        )
        print(json.dumps(results))
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
