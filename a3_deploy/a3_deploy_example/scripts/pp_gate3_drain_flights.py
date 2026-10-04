#!/usr/bin/env python3
"""Stop new launches, then wait for all launched physical flights to reach a natural endpoint."""
import json
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


def main():
    Path("/tmp/pp_gate3_stop_launches").touch()
    report_path = Path(os.environ.get("PP_PHYSICAL_EVIDENCE_JSON", "/tmp/pp_physical_ball_report.json"))
    # Allow an already queued launch to appear in the log and telemetry.
    time.sleep(0.5)
    next_notice = time.monotonic() + 30.0
    pending = []
    while True:
        try:
            text = Path("/tmp/pp_ball.log").read_text(errors="replace")
        except OSError:
            text = ""
        launched = {int(v) for v in re.findall(r"serve \d+: shot_id=(\d+)", text)}
        try:
            report = json.loads(report_path.read_text())
        except (OSError, ValueError):
            report = {}
        pending = pending_flights(report, launched)
        if not pending:
            print(f"[ball-drain] all {len(launched)} launched flights reached natural endpoints", flush=True)
            return 0
        if time.monotonic() >= next_notice:
            # A timeout must not become another way to erase an airborne ball.
            print(f"[ball-drain] still waiting for terminal telemetry: {pending}", flush=True)
            next_notice = time.monotonic() + 30.0
        time.sleep(0.1)


if __name__ == "__main__":
    raise SystemExit(main())
