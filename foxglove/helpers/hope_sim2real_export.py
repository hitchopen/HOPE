#!/usr/bin/env python3
"""Audit a collected session and extract unsynchronized native samples for MuJoCo.

Raw MCAP stays authoritative. NPZ arrays retain EACH source message, its header
stamp and the recorder timestamp; they are never resampled or silently joined.
"""
from __future__ import annotations

import argparse
import csv
from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path
import sys


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def audit_csv(path):
    counts, rows, malformed, first, last = Counter(), 0, 0, None, None
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if None in row or any(v is None for v in row.values()):
                malformed += 1
                continue
            rows += 1
            counts[row.get("mode", "unknown")] += 1
            t = row.get("wall_time_ns")
            first = first if first is not None else t
            last = t
    return {"rows": rows, "malformed_rows": malformed, "modes": dict(counts),
            "first_wall_ns": first, "last_wall_ns": last}


def export(session, output, converter, inventory_only=False):
    output.mkdir(parents=True, exist_ok=False)
    report = {"schema": 1, "session": str(session), "status": "NOT_PROVEN", "errors": [], "warnings": [],
              "capture_status": {}, "csv": {}, "csv_health": {}, "mcap": {}, "files": [],
              "alignment": "Source streams retained separately; clock fit and delay identification required.",
              "units": {"q": "rad", "dq": "rad/s", "effort": "vendor joint effort, nominal N*m",
                        "kp": "N*m/rad", "kd": "N*m*s/rad", "gyro": "rad/s", "accel": "m/s^2"},
              "limits": ["No motor current, voltage, temperature, drive saturation, or contact force in body ROS schema.",
                         "Effort and reconstructed PD torque are not calibrated motor torque.",
                         "Command header and state header may use different clocks. Use clock samples; do not subtract blindly.",
                         "Capture presence does not prove a physical hit or plant equivalence."]}
    sources = sorted(p for p in session.rglob("*") if p.is_file() and output not in p.parents)
    for path in sources:
        rel = str(path.relative_to(session))
        if path.name == "status.json" and (path.parent / "clock_resource_samples.csv").exists():
            status = json.loads(path.read_text())
            report["capture_status"][rel] = status
            if status.get("state") != "CLOSED" or status.get("errors") or status.get("forced_close"):
                report["errors"].append(rel + ": capture not cleanly complete")
        if path.name.endswith(".health.json"):
            health = json.loads(path.read_text())
            report["csv_health"][rel] = health
            if not health.get("closed") or health.get("dropped_bytes") or health.get("io_error"):
                report["errors"].append(rel + ": CSV incomplete or lost bytes")
        if path.name in {"runner_trace.csv", "policy_obs_actions.csv"}:
            report["csv"][rel] = audit_csv(path)
            if report["csv"][rel]["malformed_rows"]:
                report["errors"].append(rel + ": malformed CSV rows")
        report["files"].append({"path": rel, "bytes": path.stat().st_size, "sha256": digest(path)})
    if not inventory_only:
        import numpy as np
        from mcap.reader import make_reader
        spec = importlib.util.spec_from_file_location("a3_raw_converter", converter)
        conv = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = conv
        spec.loader.exec_module(conv)
        typestore = conv.get_ros2_typestore()
        report["joint_order_sdk31"] = conv.A3_JOINT_NAMES_31
        for index, path in enumerate(p for p in sources if p.suffix == ".mcap"):
            rel = str(path.relative_to(session))
            stats, streams = {}, {}
            try:
                with path.open("rb") as f:
                    reader = make_reader(f, validate_crcs=True)
                    if reader.get_summary() is None:
                        report["errors"].append(rel + ": missing MCAP summary/footer")
                    for schema, channel, msg in reader.iter_messages(log_time_order=False):
                        stat = stats.setdefault(channel.topic, {"count": 0, "first_log_ns": msg.log_time,
                            "last_log_ns": msg.log_time, "max_log_gap_ns": 0, "log_time_regressions": 0,
                            "joint_sequence_gaps": 0, "joint_sequence_duplicates": 0,
                            "joint_sequence_resets": 0, "source_stamp_regressions": 0})
                        gap = msg.log_time - stat["last_log_ns"]
                        stat["max_log_gap_ns"] = max(stat["max_log_gap_ns"], gap)
                        stat["log_time_regressions"] += gap < 0
                        stat["last_log_ns"] = msg.log_time
                        stat["count"] += 1
                        if not channel.topic.startswith("/body_drive/"):
                            continue
                        if schema is None:
                            raise ValueError("body topic lacks schema: " + channel.topic)
                        obj = typestore.deserialize_cdr(msg.data, schema.name)
                        source_ns = int(obj.header.stamp.sec) * 10**9 + int(obj.header.stamp.nanosec)
                        buf = streams.setdefault(channel.topic, {"log_time_ns": [], "header_stamp_ns": [],
                                                                 "mcap_sequence": [], "frame_id": []})
                        if buf["header_stamp_ns"] and source_ns < buf["header_stamp_ns"][-1]:
                            stat["source_stamp_regressions"] += 1
                        buf["log_time_ns"].append(msg.log_time)
                        buf["header_stamp_ns"].append(source_ns)
                        buf["mcap_sequence"].append(msg.sequence)
                        buf["frame_id"].append(obj.header.frame_id)
                        if hasattr(obj, "joints"):
                            names = [joint.name for joint in obj.joints]
                            if "joint_names" in buf and names != buf["joint_names"]:
                                raise ValueError("joint order changed within " + channel.topic)
                            buf["joint_names"] = names
                            seq = [int(joint.sequence) for joint in obj.joints]
                            old_seq = buf.get("joint_sequence", [])
                            if old_seq:
                                delta = [(a-b) % 2**32 for a, b in zip(seq, old_seq[-1])]
                                stat["joint_sequence_gaps"] += sum(d-1 for d in delta if 1 < d < 2**31)
                                stat["joint_sequence_duplicates"] += sum(d == 0 for d in delta)
                                stat["joint_sequence_resets"] += sum(d >= 2**31 for d in delta)
                            buf.setdefault("joint_sequence", []).append(seq)
                            for name in ("position", "velocity", "effort", "stiffness", "damping"):
                                if obj.joints and hasattr(obj.joints[0], name):
                                    buf.setdefault(name, []).append([getattr(j, name) for j in obj.joints])
                        else:
                            q, w, a = obj.orientation, obj.angular_velocity, obj.linear_acceleration
                            buf.setdefault("quat_wxyz", []).append([q.w, q.x, q.y, q.z])
                            buf.setdefault("gyro", []).append([w.x, w.y, w.z])
                            buf.setdefault("accel", []).append([a.x, a.y, a.z])
            except Exception as exc:
                report["errors"].append(rel + ": " + repr(exc))
            report["mcap"][rel] = stats
            for topic, arrays in streams.items():
                target = f"raw_{index:04d}_{topic.strip('/').replace('/', '_')}.npz"
                np.savez_compressed(output / target, **{k: np.asarray(v) for k, v in arrays.items()})
                stats[topic]["npz"] = target
                stamps = np.asarray(arrays["header_stamp_ns"], dtype=np.int64)
                if stamps.size > 1:
                    gaps = np.diff(stamps)
                    stats[topic]["source_period_ms_p50_p99_max"] = (np.quantile(gaps, [.5, .99, 1]) / 1e6).tolist()
                    stats[topic]["source_duration_s"] = float((stamps[-1] - stamps[0]) / 1e9)
                    threshold = max(5_000_000, 3 * float(np.median(gaps)))
                    indices = np.flatnonzero(gaps > threshold)
                    stats[topic]["source_gap_intervals_ns"] = [
                        [int(stamps[i]), int(stamps[i + 1])] for i in indices]
                    if indices.size:
                        report["warnings"].append(
                            rel + " " + topic + f": {indices.size} source gaps; exclude these intervals from identification, do not fill them.")
                if "joint_sequence" in arrays and len(arrays["joint_sequence"]) > 1:
                    seq = np.asarray(arrays["joint_sequence"])
                    constant = np.all(seq == seq[0], axis=0)
                    stats[topic]["constant_sequence_joints"] = [name for name, fixed in zip(arrays["joint_names"], constant) if fixed]
                    stats[topic]["sequence_note"] = "Constant vendor sequences cannot establish sample loss; check source timestamps too."
                if "effort" in arrays:
                    values = np.asarray(arrays["effort"])
                    stats[topic]["effort_all_zero"] = bool(np.all(values == 0))
                    stats[topic]["effort_has_nonfinite"] = bool(not np.all(np.isfinite(values)))
    required = {f"/body_drive/{group}_joint_{kind}" for group in ("waist", "neck", "arm", "leg")
                for kind in ("state", "command")} | {"/body_drive/pelvis_imu/data", "/body_drive/torso_imu/data"}
    present = {topic for counts in report["mcap"].values() for topic, info in counts.items() if info["count"] > 0}
    report["missing_body_topics"] = sorted(required - present)
    report["mocap_present"] = "/optitrack/poses" in present
    report["policy_rows"] = sum(x["rows"] for p, x in report["csv"].items() if p.endswith("policy_obs_actions.csv"))
    report["status"] = ("INVENTORY_ONLY" if inventory_only else
        "DATA_PRESENT_ALIGNMENT_NOT_PROVEN" if not report["errors"] and not report["missing_body_topics"]
        and report["mocap_present"] and report["policy_rows"] else "INCOMPLETE_FOR_POLICY_SIM2REAL")
    (output / "quality_report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps({"output": str(output), "status": report["status"], "errors": report["errors"]}, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--inventory-only", action="store_true")
    parser.add_argument("--converter", type=Path, default=Path(__file__).resolve().parents[2] /
                        "a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref/scripts/tools/a3_body_drive_debug_convert.py")
    args = parser.parse_args()
    export(args.session.resolve(), args.output.resolve(), args.converter, args.inventory_only)


if __name__ == "__main__":
    main()
