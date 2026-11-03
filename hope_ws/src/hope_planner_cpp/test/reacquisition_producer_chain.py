#!/usr/bin/env python3
"""Repeated raw camera opportunities through installed relays and schema-2 receiver.

All survey/clock metadata and flights are synthetic. Ten A/B rounds are frozen:
A is acquired then observed below the table; B is a fresh supported incoming
flight after a source gap (even rounds) or quiet held-to-release (odd rounds).
Rejected A's continued motion is never a new launch. No context publisher.
"""

import argparse
from collections import Counter
import gc
import json
import multiprocessing as mp
import os
from pathlib import Path
import subprocess
import signal
import sys
from queue import Empty
from qualification_accounting import (
    SourceCadence, stage_parameters, resources as capture_resources,
    validate_source_epoch, write_receipt, read_stages,
)
from lifecycle_observer import verify_lifecycle
import time
import numpy as np
from scipy.optimize import brentq
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray
from motion_capture_tracking_interfaces.msg import (
    NamedPoseArrayV2,
    NamedPose,
    LatencyInfo,
)
from hope_msgs.msg import BallObservationV2
from std_msgs.msg import Float64MultiArray
from integrated_producer_chain import flight


def camera(rate, spin, base_x, base_y, yaw, stop, ready, queue):
    rclpy.init()
    node = rclpy.create_node("continuation_independent_camera")
    pub = node.create_publisher(
        NamedPoseArrayV2, "/optitrack/poses", qos_profile_sensor_data
    )
    before, after, bounce = flight(
        10.0, "unknown" if spin == "unknown" else "zero" if spin == "zero" else "known"
    )
    plane_time = brentq(lambda t: after.sol(t)[0] - (base_x + 0.58), bounce, 0.7)
    visible = min(0.45, plane_time + 0.025)
    until = time.monotonic() + 10
    while not pub.get_subscription_count() and time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=0.01)
    assert pub.get_subscription_count()
    gc.disable()
    cadence = SourceCadence(rate, time.monotonic_ns())
    clock_origin_ns = time.time_ns()
    ready.set()
    index = 0
    rows = []
    eligible = []
    previous_stamp = 0
    while not stop.is_set():
        intended_ns = cadence.deadline_ns
        delay = (intended_ns - time.monotonic_ns()) * 1e-9
        if delay > 0:
            time.sleep(delay)
        # One qualified acquisition epoch drives both plant state and message.
        stamp_mono_ns = time.monotonic_ns()
        stamp = time.time_ns()
        cadence.acquired(stamp_mono_ns)
        elapsed = (stamp - clock_origin_ns) * 1e-9
        state_mono_ns = stamp_mono_ns
        round_id = int((elapsed - 0.8) // 1.8)
        phase = (elapsed - 0.8) % 1.8
        msg = NamedPoseArrayV2()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
        msg.header.frame_id = "world"
        latency = LatencyInfo()
        latency.source = "NatNetClockUncertainty"
        latency.latency = 20
        msg.latencies.append(latency)
        base = NamedPose()
        base.name = "UCB_P1"
        base.pose.position.x = base_x
        base.pose.position.y = base_y
        base.pose.orientation.w = float(np.cos(np.deg2rad(yaw) / 2))
        base.pose.orientation.z = float(np.sin(np.deg2rad(yaw) / 2))
        msg.poses.append(base)
        label = "BASE_ONLY"
        state = None
        local = None
        if 0 <= round_id < 10:
            if phase < 0.09:
                label = "A"
                state = before.sol(phase)
                local = phase
            elif phase < 0.105:
                label = "A_REJECT"
                state = np.array([1.0, -0.7, -0.2, 0.0, 0.0, 0.0, 0.0])
            elif phase < 0.2:
                label = "A_CONTINUED_REJECTED"
                state = before.sol(phase)
                local = phase
            elif round_id % 2 and phase < 0.7:
                label = "HELD"
                state = np.array([2.5, -0.7, 0.32, 0.0, 0.0, 0.0, 100 * phase])
            elif 0.7 <= phase < 0.7 + visible:
                label = "B"
                local = phase - 0.7
                state = before.sol(local) if local < bounce else after.sol(local)
        if state is not None:
            ball = NamedPose()
            ball.name = "Ball"
            ball.pose.position.x, ball.pose.position.y, ball.pose.position.z = map(
                float, state[:3]
            )
            if spin != "unknown":
                ball.pose.orientation.w = float(np.cos(state[6] / 2))
                ball.pose.orientation.y = float(np.sin(state[6] / 2))
            msg.poses.append(ball)
        # Frozen faults occur during A quarantine, outside B's independently
        # declared eligible window. Their exposure/acceptance remains recorded.
        fault = None
        if 0 <= round_id < 10 and 0.14 < phase < 0.15:
            fault = "wrong_frame"
            msg.header.frame_id = "wrong_frame"
        if 0 <= round_id < 10 and 0.16 < phase < 0.17:
            fault = "clock_unavailable"
            msg.latencies.clear()
        if 0 <= round_id < 10 and 0.18 < phase < 0.19 and previous_stamp:
            fault = "duplicate_or_reorder"
            bad = previous_stamp - (1000000 if index % 2 else 0)
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(bad, 10**9)
        publish_wall_ns = time.time_ns()
        publish_mono_ns = time.monotonic_ns()
        pub.publish(msg)
        published_mono_ns = time.monotonic_ns()
        rows.append(
            {
                "stamp_ns": stamp,
                "state_elapsed_s": elapsed,
                "index": index,
                "intended_mono_ns": intended_ns,
                "state_mono_ns": state_mono_ns,
                "stamp_mono_ns": stamp_mono_ns,
                "publish_wall_ns": publish_wall_ns,
                "publish_mono_ns": publish_mono_ns,
                "published_mono_ns": published_mono_ns,
                "wire_stamp_ns": int(msg.header.stamp.sec) * 10**9 + int(msg.header.stamp.nanosec),
                "poses": [{"name": pose.name,
                           "position": [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z],
                           "quaternion_wxyz": [pose.pose.orientation.w, pose.pose.orientation.x,
                                              pose.pose.orientation.y, pose.pose.orientation.z]}
                          for pose in msg.poses],
                "round": round_id,
                "phase": phase,
                "opportunity": label,
                "fault": fault,
            }
        )
        if label == "B" and 0.1 < local < plane_time - 0.13:
            eligible.append(stamp)
        previous_stamp = stamp
        index += 1
    queue.put(
        {
            "clock_origin_ns": clock_origin_ns,
            "rows": rows,
            "eligible_stamps": eligible,
            "plane_time_s": plane_time,
            "bounce_s": bounce,
            "base_x": base_x,
            "base_y": base_y,
            "base_yaw_deg": yaw,
            "cadence": cadence.receipt(),
            "source_intervals_ns": np.diff([r["stamp_ns"] for r in rows]).tolist(),
            "truth": "independent synthetic plant; not physical registration",
            "scored_rounds": 10,
            "launched_A": 10,
            "launched_B": 10,
        }
    )
    gc.enable()
    node.destroy_node()
    rclpy.shutdown()


def run(args):
    processes = []
    logs = []
    node = None
    stop = mp.Event()
    ready = mp.Event()
    queue = mp.Queue()
    seed = args.rate + (0 if args.policy == "FOLLOW_POSE" else 1) + len(args.spin)
    rng = np.random.default_rng(seed)
    bx = -0.43 + float(rng.uniform(-0.005, 0.005))
    by = -0.7 + float(rng.uniform(-0.005, 0.005))
    source = mp.Process(
        target=camera, args=(args.rate, args.spin, bx, by, args.yaw, stop, ready, queue)
    )
    wire = []
    decoded = []
    traces = []
    statuses = []
    observations = []
    source_stats = {}
    result = None
    lifecycle_path = args.output.with_name(args.output.stem + '-lifecycle.stages')

    def launch(exe, params=()):
        cmd = [str(exe), "--ros-args"]
        for k, v in [*params, *stage_parameters(args, Path(exe))]:
            cmd += ["-p", f"{k}:={v}"]
        log = args.output.with_name(
            args.output.stem + "-" + Path(exe).name + ".log"
        ).open("w")
        logs.append(log)
        p = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
        processes.append(p)
        return p

    try:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        registration = "SYNTHETIC_MCT_P1"
        world = "a" * 64
        launch(
            args.bringup / "optitrack_mct_relay",
            [
                ("publish_tf", "false"),
                ("input_history_depth", 8),
                ("ball_calibration_id", registration),
                ("ball_orientation_authoritative", str(args.spin != "unknown").lower()),
            ],
        )
        launch(
            args.bringup / "hope_base_pose_flat_relay",
            [
                ("extrinsic_calibrated", "true"),
                ("world_frame_calibrated", "true"),
                ("calibration_sha256", "b" * 64),
                ("world_frame_sha256", world),
                ("source_stamp_mode", "input_header"),
                ("policy_z_offset", 0.76),
            ],
        )
        planner = launch(
            args.planner,
            [
                ("prediction_backend", "v2"),
                ("registration_id", registration),
                ("world_frame_id", int(world[:13], 16)),
                ("plane_policy", args.policy),
                ("test.worker_delay_ms", args.worker_delay_ms),
                ("test.first_worker_delay_ms", args.first_worker_delay_ms),
                ("test.lifecycle_trace_path", str(lifecycle_path)),
            ],
        )
        launch(args.decoder)
        node = rclpy.create_node("continuation_observer")
        qos = QoSProfile(depth=2048, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(
            Float64MultiArray,
            "/hope/shadow/integrated_planner/racket_target",
            lambda m: wire.append(list(m.data)),
            2048,
        )
        node.create_subscription(
            Float64MultiArray,
            "/hope/test/integrated_runner/decoded",
            lambda m: decoded.append(list(m.data)),
            2048,
        )
        node.create_subscription(
            BallObservationV2,
            "/hope/ball/observation_v2",
            lambda m: observations.append(
                {
                    "ns": int(m.source_time_ns),
                    "omega_valid": bool(m.omega_valid),
                    "orientation_valid": bool(m.orientation_valid),
                }
            ),
            qos,
        )

        def trace(m):
            for d in m.status:
                traces.append(
                    {**{v.key: v.value for v in d.values}, "receipt_ns": time.time_ns()}
                )

        def status(m):
            for d in m.status:
                statuses.append(
                    {
                        **{v.key: v.value for v in d.values},
                        "reason": d.message,
                        "receipt_ns": time.time_ns(),
                    }
                )

        node.create_subscription(
            DiagnosticArray, "/hope/shadow/integrated_planner/trace", trace, qos
        )
        node.create_subscription(
            DiagnosticArray, "/hope/shadow/integrated_planner/status", status, 2048
        )
        until = time.monotonic() + 1
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.005)
        assert all(p.poll() is None for p in processes)
        source.start()
        until = time.monotonic() + 12
        while not ready.is_set() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.005)
        assert ready.is_set()
        start = time.monotonic()
        while time.monotonic() - start < 19.1:
            rclpy.spin_once(node, timeout_sec=0.001)
        stop.set()
        source_stats = queue.get(timeout=8)
        validate_source_epoch(source_stats)
        assert source_stats["cadence"]["valid"], ("source cadence", source_stats["cadence"])
        source.join(timeout=5)
        assert not source.is_alive()
        rows = source_stats["rows"]
        bounds = {
            i: (
                min(
                    r["stamp_ns"]
                    for r in rows
                    if r["round"] == i and r["opportunity"] == "B"
                ),
                max(
                    r["stamp_ns"]
                    for r in rows
                    if r["round"] == i and r["opportunity"] == "B"
                ),
            )
            for i in range(10)
        }
        by_seq = {int(t["command_sequence"]): t for t in traces}
        decoded_seqs = {int(d[0]) for d in decoded}
        rounds = []
        generation_order = []
        for i, (lo, hi) in bounds.items():
            # Acquisition/rejection/quarantine are checked from the complete
            # flight-matched trace after orderly process shutdown, below.
            b = [t for t in traces if lo <= int(t["source_time_ns"]) <= hi]
            assert b, (
                "B produced zero valid commands",
                i,
                Counter(s["reason"] for s in statuses),
            )
            generations = {int(t["physical_shot"]) for t in b}
            assert len(generations) == 1, (i, generations)
            gen = generations.pop()
            generation_order.append(gen)
            assert any(int(t["command_sequence"]) in decoded_seqs for t in b), (
                "B never decoded",
                i,
            )
            if args.spin == "unknown":
                assert all(t["spin_readiness"] == "UNAVAILABLE" for t in b)
            else:
                assert any(t["spin_readiness"] == "READY_MEASURED" for t in b), (
                    i,
                    Counter(t.get("spin_readiness") for t in b),
                )
            rounds.append(
                {
                    "round": i,
                    "B_generation": gen,
                    "B_valid_commands": len(b),
                    "B_decoded": sum(
                        int(t["command_sequence"]) in decoded_seqs for t in b
                    ),
                    "B_source_bounds_ns": [lo, hi],
                    "new_flight_evidence": "gap" if i % 2 == 0 else "held_release",
                }
            )
        assert all(
            b > a for a, b in zip(generation_order, generation_order[1:])
        ), generation_order
        # No solve begun for A may publish after a B command. Immutable source
        # cutoffs and monotonic physical/wire IDs prove the fence at the receiver.
        seq_order = sorted(traces, key=lambda t: int(t["command_sequence"]))
        assert all(
            int(b["physical_shot"]) >= int(a["physical_shot"])
            for a, b in zip(seq_order, seq_order[1:])
        )
        assert all(float(t["source_age_ms"]) <= 30 for t in traces)
        expected = set(source_stats["eligible_stamps"])
        published = {int(t["source_time_ns"]) for t in traces}
        covered = expected & published
        coverage = len(covered) / len(expected) if expected else 0
        if args.worker_delay_ms == 0:
            assert expected and coverage >= 0.95, (
                "coverage",
                len(covered),
                len(expected),
                coverage,
            )
        age = [
            (
                (int(d[13]) * 10**9 + int(d[14]))
                - int(by_seq[int(d[0])]["source_time_ns"])
            )
            / 1e6
            for d in decoded
            if d[10] == 0 and int(d[0]) in by_seq
        ]
        assert age and np.percentile(age, 95) <= 30, (
            "decoder p95",
            np.percentile(age, 95),
        )
        assert all(
            not o["omega_valid"] for o in observations
        ), "raw relay unexpectedly supplied producer omega"
        assert node.count_publishers("/racket/command_flat") == 0
        assert node.get_node_names().count("hope_planner") == 1
        assert node.count_publishers("/hope/ball/observation_v2") == 1
        assert statuses and max(int(s["candidate_samples"]) for s in statuses) <= 32
        assert all(s['observation_history_depth'] == '8' for s in statuses)
        assert max(int(s["retained_samples"]) for s in statuses) <= 192
        if args.first_worker_delay_ms:
            assert any(
                int(s.get("stale_work_discarded", "0")) > 0
                and int(s.get("last_discarded_generation", "0"))
                < int(s.get("generation_at_discard", "0"))
                for s in statuses
            ), "old-generation work did not cross B boundary"
        resource = {}
        for line in Path(f"/proc/{planner.pid}/status").read_text().splitlines():
            if line.startswith(("VmRSS:", "VmHWM:")):
                k, v = line.split(":")
                resource[k + "_kib"] = int(v.split()[0])
        result = {
            "status": "PASS",
            "rate_hz": args.rate,
            "plane_policy": args.policy,
            "spin": args.spin,
            "worker_delay_ms": args.worker_delay_ms,
            "first_worker_delay_ms": args.first_worker_delay_ms,
            "cross_generation_discard_observed": any(
                int(s.get("stale_work_discarded", "0")) > 0
                and int(s.get("last_discarded_generation", "0"))
                < int(s.get("generation_at_discard", "0"))
                for s in statuses
            ),
            "rounds": rounds,
            "source": source_stats,
            "command_coverage": coverage,
            "command_denominator": len(expected),
            "covered_command_epochs": len(covered),
            "coverage_gate": (
                "PASS"
                if args.worker_delay_ms == 0
                else "NOT_APPLICABLE_INJECTED_SLOW_WORKER"
            ),
            "decoder_ms": dict(
                zip(
                    ["p50", "p95", "p99", "max"],
                    map(float, np.percentile(age, [50, 95, 99, 100])),
                )
            ),
            "resources": resource,
            "reasons": dict(Counter(s["reason"] for s in statuses)),
            "candidate_bound": 32,
            "worker_bound": {"running": 1, "pending": 1},
            "physical_registration": False,
            "context_publisher": False,
        }
        return result
    finally:
        original_failure = sys.exc_info()[1]
        failure = sys.exc_info()[1]
        failure = None if failure is None else f'{type(failure).__name__}: {failure}'
        resource_receipt = capture_resources(processes)
        stop.set()
        if source.pid:
            if not source_stats:
                try:
                    source_stats = queue.get(timeout=5)
                except Empty:
                    source_stats = {}
            source.join(timeout=3)
            if source.is_alive():
                source.terminate()
                source.join(timeout=5)
                failure = (failure or '') + ' SOURCE_SHUTDOWN_TIMEOUT'
        if node is not None:
            node.destroy_node()
        for p in reversed(processes):
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
                try:
                    p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    p.kill()
                    p.wait(timeout=5)
                    failure = (failure or '') + ' CHILD_SHUTDOWN_TIMEOUT:' + Path(p.args[0]).name
            resource_receipt[Path(p.args[0]).name]['final_exit'] = p.returncode
            if p.returncode != 0:
                failure = (failure or '') + f' CHILD_EXIT:{Path(p.args[0]).name}:{p.returncode}'
        for log in logs:
            log.close()
        lifecycle = {}
        try:
            metadata, transitions = read_stages(lifecycle_path)
            lifecycle = verify_lifecycle(
                source_stats.get('rows', []), metadata, transitions, traces,
                {int(row[0]) for row in decoded})
        except (AssertionError, KeyError, ValueError) as error:
            lifecycle = {'status': 'FAIL', 'exception': repr(error)}
            failure = (failure or '') + ' LIFECYCLE:' + repr(error)
        write_receipt(args, {
            'wire': wire, 'decoded': decoded, 'traces': traces, 'statuses': statuses,
            'observations': observations, 'source': source_stats, 'lifecycle': lifecycle,
        }, resource_receipt, failure)
        if not failure and result is not None:
            result['lifecycle'] = lifecycle
            args.output.write_text(json.dumps(result, indent=2) + '\n')
        if failure and original_failure is None:
            raise RuntimeError(failure)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for key in ["planner", "decoder", "bringup", "output"]:
        p.add_argument("--" + key, type=Path, required=True)
    p.add_argument("--rate", type=int, choices=[150, 300], required=True)
    p.add_argument("--policy", choices=["FOLLOW_POSE", "ON_REFRESH"], required=True)
    p.add_argument(
        "--spin", choices=["unknown", "quaternion_only", "zero"], required=True
    )
    p.add_argument("--yaw", type=float, default=30)
    p.add_argument("--worker-delay-ms", type=int, default=0)
    p.add_argument("--first-worker-delay-ms", type=int, default=0)
    p.add_argument("--stage-accounting", action="store_true")
    args = p.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    try:
        r = run(args)
        print(json.dumps({k: v for k, v in r.items() if k not in ["source", "rounds"]}))
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    mp.set_start_method("spawn")
    main()
