#!/usr/bin/env python3
"""Run the two unprivileged display services inside the Mac field VM container."""
import sys
if __name__ == "__main__":
    sys.path.pop(0)  # Avoid the sibling operator.py when loading the standard library.

import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path.home() / "HOPE"
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "foxglove/laptop"))
from hope_config_probe import read_receipt


def display_keys(values, hdu, laptop):
    if (values.get("HOPE_LIFECYCLE_HDU_IP") != hdu
            or values.get("HOPE_LIFECYCLE_LAPTOP_IP") != laptop
            or values.get("HOPE_LIFECYCLE_ROOT") != str(ROOT)
            or not values.get("HOPE_LIFECYCLE_SESSION_ID")):
        return {}
    return {"hope-host-bridge": (hdu, laptop),
            "hope-live-scene": tuple(sorted(values.items()))}


def stop(process):
    # ros2 run launches a child. Reap the entire display process group before
    # another instance attempts to bind the same WebSocket or mesh port.
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=3)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    process.wait()


def main():
    receipt = Path.home() / ".config/hope-foxglove/laptop-runtime.env"
    children, retry_at = {}, {}
    try:
        while True:
            try:
                keys = display_keys(read_receipt(receipt), os.environ["HOPE_FIELD_HDU_IP"],
                                    os.environ["HOPE_FIELD_LAPTOP_IP"])
            except (OSError, ValueError):
                keys = {}
            for name, (process, key) in list(children.items()):
                crashed = process.poll() is not None
                if crashed or keys.get(name) != key:
                    stop(process)
                    del children[name]
                    if crashed:
                        print(f"{name} exited with {process.returncode}, retrying in 5 seconds", flush=True)
                        retry_at[name] = time.monotonic() + 5
            for name, key in keys.items():
                if name not in children and time.monotonic() >= retry_at.get(name, 0):
                    process = subprocess.Popen(["bash", ROOT / "foxglove/helpers" / name], start_new_session=True)
                    children[name] = process, key
            time.sleep(1)
    finally:
        for process, _ in children.values():
            stop(process)


if __name__ == "__main__":
    def terminate(_signal, _frame):
        raise SystemExit(0)
    signal.signal(signal.SIGTERM, terminate)
    main()
