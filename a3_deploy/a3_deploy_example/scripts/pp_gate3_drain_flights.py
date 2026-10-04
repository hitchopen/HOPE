#!/usr/bin/env python3
"""Drain live flights; fail incomplete evidence when its producers stop progressing."""
import argparse
import json
import math
import os
from pathlib import Path
import re
import time


def pending_flights(report, launched):
    terminal = {int(row["shot_id"]) for row in report.get("rows", [])
                if row.get("terminal_event") is not None}
    observed = {int(row["shot_id"]) for row in report.get("rows", [])
                if row.get("samples", 0) > 0}
    return sorted((set(launched) | observed) - terminal)


def read_report(path):
    try:
        report = json.loads(Path(path).read_text())
        if not isinstance(report, dict) or not isinstance(report.get("rows"), list):
            raise ValueError("physical report must contain a rows array")
        # A corrupt snapshot is missing evidence, never an empty successful run.
        for row in report["rows"]:
            row["shot_id"] = int(row["shot_id"])
            row["samples"] = int(row.get("samples", 0))
            if row.get("last_stamp_ns") is not None:
                row["last_stamp_ns"] = int(row["last_stamp_ns"])
        return report, None
    except (OSError, ValueError, TypeError, KeyError) as exc:
        return {"rows": []}, str(exc)


def write_report(path, report):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    temporary.replace(path)


def process_alive(pid):
    """A zombie cannot produce more telemetry even though kill(pid, 0) succeeds."""
    try:
        fields = Path(f"/proc/{int(pid)}/stat").read_text().rsplit(")", 1)[1].split()
        return fields[0] not in {"Z", "X"}
    except (OSError, ValueError, IndexError):
        return False


class FlightDrain:
    def __init__(self, *, stale_s=10.0, dead_grace_s=2.0):
        if not all(math.isfinite(v) and v > 0 for v in (stale_s, dead_grace_s)):
            raise ValueError("drain stale/dead grace periods must be finite and positive")
        self.stale_s = stale_s
        self.dead_grace_s = dead_grace_s
        self.progress = {}
        self.launched = set()
        self.dead_since = None

    def observe(self, report, launched, producers, now, report_error=None):
        # Retain IDs even if a later log/snapshot is missing or corrupt.
        self.launched.update(launched)
        self.launched.update(int(row["shot_id"]) for row in report.get("rows", [])
                             if row.get("samples", 0) > 0)
        pending = pending_flights(report, self.launched)
        rows = {int(row["shot_id"]): row for row in report.get("rows", [])}
        stale = []
        for shot in pending:
            # Recorder timer writes and another shot's samples do not prove
            # this flight is advancing. Prefer the source timestamp, not mtime.
            row = rows.get(shot, {})
            value = row.get("last_stamp_ns")
            token = int(value) if value is not None else int(row.get("samples", 0))
            previous, changed_at = self.progress.setdefault(shot, (token, now))
            if token > previous:
                self.progress[shot] = (token, now)
                changed_at = now
            if now - changed_at >= self.stale_s:
                stale.append(shot)
        dead = sorted(name for name, alive in producers.items() if not alive)
        self.dead_since = (now if self.dead_since is None else self.dead_since) if dead else None
        reason = None
        if pending and dead and now - self.dead_since >= self.dead_grace_s:
            reason = "producer_exited"
        elif stale:
            reason = "telemetry_stalled"
        complete = not pending and not (report_error and self.launched)
        return {
            "status": "complete" if complete else "failed" if reason else "draining",
            "evidence_complete": complete, "failure_reason": reason,
            "launched_shot_ids": sorted(self.launched), "pending_shot_ids": pending,
            "stale_shot_ids": stale, "dead_producers": dead,
            "producer_alive": producers, "report_error": report_error,
            "updated_monotonic_s": now,
        }


def finalize_report(rally_path, drain_path):
    """Invalidate a provisional PASS without fabricating terminal ball events."""
    try:
        drain = json.loads(Path(drain_path).read_text())
        complete = drain.get("evidence_complete") is True and drain.get("status") == "complete"
    except (OSError, ValueError, AttributeError):
        drain = {"evidence_complete": False, "failure_reason": "drain_report_missing_or_invalid"}
        complete = False
    path = Path(rally_path)
    if path.exists() or not complete:
        try:
            report = json.loads(path.read_text())
            if not isinstance(report, dict):
                report = {}
        except (OSError, ValueError):
            report = {}
        report["flight_drain"] = drain
        if not complete:
            report["certification_pass"] = False
        write_report(path, report)
    return 0 if complete else 2


def main():
    Path(os.environ.get("PP_STOP_LAUNCHES_FILE", "/tmp/pp_gate3_stop_launches")).touch()
    report_path = Path(os.environ.get("PP_PHYSICAL_EVIDENCE_JSON", "/tmp/pp_physical_ball_report.json"))
    result_path = Path(os.environ.get("PP_DRAIN_REPORT_JSON", "/tmp/pp_gate3_drain_report.json"))
    log_path = Path(os.environ.get("PP_BALL_LOG", "/tmp/pp_ball.log"))
    pids = {name: os.environ[key] for name, key in
            (("simulator", "PP_SIM_PID"), ("recorder", "PP_EVIDENCE_PID"))
            if os.environ.get(key)}
    monitor = FlightDrain(stale_s=float(os.environ.get("PP_DRAIN_STALE_S", "10")),
                          dead_grace_s=float(os.environ.get("PP_DRAIN_DEAD_GRACE_S", "2")))
    # Allow an already queued launch and buffered final telemetry to be recorded.
    time.sleep(0.5)
    next_notice = time.monotonic() + 30.0
    while True:
        try:
            text = log_path.read_text(errors="replace")
        except OSError:
            text = ""
        launched = {int(v) for v in re.findall(r"serve \d+: shot_id=(\d+)", text)}
        report, error = read_report(report_path)
        result = monitor.observe(report, launched,
                                 {name: process_alive(pid) for name, pid in pids.items()},
                                 time.monotonic(), error)
        result["producer_pids"] = pids
        result["stale_timeout_s"] = monitor.stale_s
        result["dead_grace_s"] = monitor.dead_grace_s
        write_report(result_path, result)
        if result["status"] != "draining":
            print(f"[ball-drain] {result['status']}: pending={result['pending_shot_ids']} "
                  f"reason={result['failure_reason']}", flush=True)
            return 0 if result["evidence_complete"] else 2
        if time.monotonic() >= next_notice:
            print(f"[ball-drain] waiting for live flights: {result['pending_shot_ids']}", flush=True)
            next_notice = time.monotonic() + 30.0
        time.sleep(0.1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--finalize-report")
    args = parser.parse_args()
    raise SystemExit(finalize_report(args.finalize_report, os.environ.get(
        "PP_DRAIN_REPORT_JSON", "/tmp/pp_gate3_drain_report.json"))
        if args.finalize_report else main())
