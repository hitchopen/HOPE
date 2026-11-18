#!/usr/bin/env python3
"""Matched localhost CPU comparison; inert hardware backends, never A3 DDS.

Requires the same ROS_DOMAIN_ID=219/ROS_LOCALHOST_ONLY=1 safeguards as tests.
Reports parent plus worker CPU (100%=one core); it is not a field qualification.
Run after the regression suite, without other builds/tests running.
"""
import json
import math
import os
from pathlib import Path
import subprocess
import time

from test_native_operations_ros import NativeOperationsTests


def ticks(pid):
    text = Path(f"/proc/{pid}/stat").read_text().rsplit(")",1)[1].split()
    total = int(text[11]) + int(text[12])
    # posix_spawn runs on the main thread; include all descendants recursively.
    for child in Path(f"/proc/{pid}/task/{pid}/children").read_text().split():
        total += ticks(int(child))
    return total


def measure(reference, rate):
    test = NativeOperationsTests()
    test.reference = reference
    try:
        test.setUp()
        test.publishing = False
        binary = Path(os.environ["HOPE_NATIVE_BIN_DIR"]) / "hope-operations-test-ingress"
        with subprocess.Popen([str(binary),"--ros-args","-p","benchmark:=true","-p",f"rate_hz:={rate}",
                               "-p","duration_s:=25.0"],stdout=test.output,stderr=subprocess.STDOUT) as ingress:
            time.sleep(3.)
            before = ticks(test.process.pid);start = time.monotonic()
            time.sleep(20.)
            elapsed=time.monotonic()-start;used=ticks(test.process.pid)-before
            assert ingress.wait(timeout=10)==0
            assert test.process.poll() is None
            assert any(test.ball_live),"benchmark must actually deliver mocap"
            return dict(implementation="python" if reference else "native+worker",rate_hz=rate,
                        cpu_percent=used/os.sysconf("SC_CLK_TCK")/elapsed*100.,seconds=elapsed,player_rate_hz=30,
                        displayed_ball_rate=max((v for v in test.ball_rates if math.isfinite(v)),default=None),
                        markers=len(test.markers),
                        transport_metrics=test.metrics[-1] if test.metrics else None)
    finally:
        test.doCleanups()


if __name__ == "__main__":
    if os.environ.get("ROS_LOCALHOST_ONLY")!="1" or os.environ.get("ROS_DOMAIN_ID")!="219":
        raise SystemExit("isolated localhost domain 219 is required")
    for rate in (150,300):
        for reference in (True,False):
            print(json.dumps(measure(reference,rate)),flush=True)
