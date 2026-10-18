#!/usr/bin/env python3
"""Bounded catch-start cleanup of an explicit HDU background allowlist.

Never stops the process manager, HAL, local networking, clocks, or recording.
An explicitly installed Wi-Fi-only profile also stops the cellular watchdog.
Failure to establish a quiet state is a preflight failure, not a warning.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import urllib.request

APPS = ("agent", "hal_audio", "hal_hdu_camera")
UI = "agibot_ui.service"
CELLULAR = "zy_cellular.service"
WIFI_ONLY = Path("/etc/hope-foxglove/wifi-only")
WATCHDOGS = ("/scripts/agent/process_monitor.sh", "/scripts/hal_audio/hal_audio_monitor.sh")


def rpc(method, body):
    request = urllib.request.Request(
        "http://127.0.0.1:50081/rpc/aimdk.protocol.EmAppService/" + method,
        data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=3 if method == "GetAllAppInfos" else 35) as response:
        value = json.load(response)
    if str(value.get("header", {}).get("code", "0")) != "0":
        raise RuntimeError(f"{method}: {value}")
    return value


def unwanted(argv):
    if not argv:
        return False
    exe = Path(argv[0]).name
    if exe in {"bash", "sh"}:
        return any(arg.endswith(WATCHDOGS) for arg in argv[1:])
    return ((exe == "agent" and any("/config/agent/" in arg for arg in argv[1:]))
            or (exe == "hal_audio" and any("/config/hal_audio/" in arg for arg in argv[1:]))
            or (exe == "aimrt_main" and any("/config/hdu_camera/" in arg for arg in argv[1:])))


def unwanted_processes():
    result = {}
    for path in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            argv = path.read_bytes().decode().strip("\0").split("\0")
        except (OSError, UnicodeError):
            continue
        if unwanted(argv):
            result[int(path.parent.name)] = argv
    return result


def terminate(pid, argv):
    # Pin process identity before checking its command again; no broad pkill.
    try:
        fd = os.pidfd_open(pid)
    except ProcessLookupError:
        return
    try:
        current = Path(f"/proc/{pid}/cmdline").read_bytes().decode().strip("\0").split("\0")
        if current != argv or not unwanted(current):
            raise RuntimeError(f"process identity changed: {pid}")
        signal.pidfd_send_signal(fd, signal.SIGTERM)
    except (FileNotFoundError, ProcessLookupError):
        pass
    finally:
        os.close(fd)


def service_active(unit):
    result = subprocess.run(["systemctl", "is-active", unit], timeout=5,
                            capture_output=True, text=True)
    state = result.stdout.strip()
    if result.returncode == 0 or state in {"activating", "deactivating", "reloading"}:
        return True
    if result.returncode in {3, 4} and state in {"inactive", "failed", "unknown"} and not result.stderr.strip():
        return False
    raise RuntimeError(f"Cannot determine {unit} state: {result.stderr.strip() or state}")


def ui_active():
    return service_active(UI)


def wifi_only():
    try:
        value = WIFI_ONLY.read_text().strip()
    except FileNotFoundError:
        return False
    if value != "wifi-only-v1":
        raise RuntimeError("invalid Wi-Fi-only profile")
    return True


def stop_cellular(receipt):
    # A narrowly scoped polkit rule permits only this stop, even under NNP.
    active = service_active(CELLULAR)
    receipt["cellular"] = {"profile": "wifi-only-v1", "was_active": active}
    if active:
        subprocess.run(["systemctl", "--no-ask-password", "stop", CELLULAR],
                       check=True, timeout=30)


def clean(receipt):
    cellular_required = wifi_only()
    if cellular_required:
        stop_cellular(receipt)
    receipt["before"] = rpc("GetAllAppInfos", {})
    # Detached vendor watchdogs are not reaped by StopApp; stop them first.
    for pid, argv in unwanted_processes().items():
        if Path(argv[0]).name in {"bash", "sh"}:
            terminate(pid, argv)
            receipt["terminated_watchdogs"].append(pid)
    for info in receipt["before"]["infos"]:
        if info["app"] in APPS and info.get("pid", -1) > 0:
            result = rpc("StopApp", {"app": info["app"]})
            receipt["stop_results"].append(result)
            if result.get("result", {}).get("type") not in {"Type_SUCCESS", "Type_EXITED", "Type_STOPPING"}:
                raise RuntimeError(f"StopApp failed: {result}")
    if ui_active():
        subprocess.run(["sudo", "-n", "systemctl", "stop", UI], check=True, timeout=30)
    # Exact-name leftovers outside the manager also cannot survive preflight.
    for pid, argv in unwanted_processes().items():
        terminate(pid, argv)
        receipt["terminated_leftovers"].append(pid)
    quiet_since = None
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        after = rpc("GetAllAppInfos", {})
        running = [x for x in after["infos"] if x["app"] in APPS and x.get("pid", -1) > 0]
        remaining = unwanted_processes()
        cellular_active = cellular_required and service_active(CELLULAR)
        if not running and not remaining and not ui_active() and not cellular_active:
            if quiet_since is None:
                quiet_since = time.monotonic()
            if time.monotonic() - quiet_since >= 2:
                if cellular_required:
                    receipt["cellular"]["active_after"] = False
                receipt["after"] = after
                return
        else:
            quiet_since = None
        time.sleep(.25)
    raise RuntimeError(f"background still active or respawning: apps={running}, pids={list(remaining)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--receipt", type=Path, required=True)
    args = parser.parse_args()
    receipt = {"start_wall_ns": time.time_ns(), "allowed_apps": APPS,
               "terminated_watchdogs": [], "terminated_leftovers": [], "stop_results": []}
    try:
        clean(receipt)
        receipt["result"] = "PASS"
    except Exception as exc:
        receipt.update(result="FAIL", error=str(exc))
    receipt["end_wall_ns"] = time.time_ns()
    args.receipt.parent.mkdir(parents=True, exist_ok=True)
    args.receipt.write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps({"result": receipt["result"], "receipt": str(args.receipt), "error": receipt.get("error", "")}))
    return 0 if receipt["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
