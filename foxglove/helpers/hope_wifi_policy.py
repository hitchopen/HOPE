#!/usr/bin/env python3
"""HDU RTL8852BE policy: no power saving; scans only offline or on request.

Does not disconnect, reconnect, stop vendor applications, or control the robot.
The vendor's repeated rescan requests are rejected at the driver while associated.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import subprocess
import time

IFACE = "wifi_hdu"
IW = "/usr/sbin/iw"
SERVICE = "hope-wifi-policy.service"
SCAN = Path("/proc/net/rtl8852be/wifi_hdu/scan_deny")
DEVICE = Path("/sys/class/net/wifi_hdu")
WINDOW = Path("/run/hope-wifi-policy/manual-until")


def run(*argv, timeout=5):
    return subprocess.run(argv, capture_output=True, text=True, timeout=timeout,
                          check=True).stdout.strip()


def scan_blocked():
    line = next(iter(SCAN.read_text().splitlines()), "")
    if line not in ("scan_deny set by /proc is enable", "scan_deny set by /proc is disable"):
        raise RuntimeError("unrecognized RTL8852BE scan policy")
    return line.endswith("is enable")


def block_scan(block):
    if SCAN.exists() and scan_blocked() != block:
        SCAN.write_text("1\n" if block else "0\n")


def manual_window():
    try:
        remaining = float(WINDOW.read_text()) - time.monotonic()
        return 0 < remaining <= 30
    except (OSError, ValueError):
        return False


def snapshot():
    if not DEVICE.exists():
        return {"present": False, "connected": False}
    link = run(IW, "dev", IFACE, "link")
    if not (link.startswith("Connected to ") or link == "Not connected."):
        raise RuntimeError("unknown association state")
    power = run(IW, "dev", IFACE, "get", "power_save")
    if power not in ("Power save: on", "Power save: off"):
        raise RuntimeError("unknown Wi-Fi power state")
    return dict(present=True, connected=link.startswith("Connected to "),
                power_save=power.endswith("on"), scan_blocked=scan_blocked(),
                manual_scan=manual_window())


def enforce():
    state = snapshot()
    if not state["present"]:
        block_scan(False)
        return state
    # Release first on disconnect, even if updating power management fails.
    desired = state["connected"] and not state["manual_scan"]
    block_scan(desired)
    if state["power_save"]:
        run(IW, "dev", IFACE, "set", "power_save", "off")
    result = snapshot()
    if result.get("present") and result["connected"] == state["connected"]:
        if result["power_save"] or result["scan_blocked"] != desired:
            raise RuntimeError("Wi-Fi policy readback failed")
    return result


def verify():
    run("/usr/bin/systemctl", "is-active", "--quiet", SERVICE)
    run("/usr/bin/systemctl", "is-enabled", "--quiet", SERVICE)
    state = snapshot()
    if not state["present"]:
        raise RuntimeError("HDU Wi-Fi interface missing")
    if state["power_save"]:
        raise RuntimeError("Wi-Fi power saving enabled")
    if state["manual_scan"]:
        raise RuntimeError("manual Wi-Fi scan in progress; wait before START")
    if state["scan_blocked"] != state["connected"]:
        raise RuntimeError("Wi-Fi scan policy does not match association state")
    return state


def serve():
    previous = None
    while True:
        try:
            result = enforce()
        except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
            # Never strand reconnection because reading the driver failed.
            try:
                block_scan(False)
            except (OSError, RuntimeError):
                pass
            result = {"error": str(exc)}
        if result != previous:
            print(json.dumps(result, sort_keys=True), flush=True)
            previous = result
        time.sleep(1)


def rescan():
    run("/usr/bin/systemctl", "is-active", "--quiet", SERVICE)
    # Root-only directory and lock: unrelated users/processes cannot open a window.
    with (WINDOW.parent / "manual.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        deadline = time.monotonic() + 30
        WINDOW.write_text(str(deadline))
        try:
            block_scan(False)
            # iw waits for a completed scan; retry only when another scan owns it.
            while time.monotonic() < deadline - 5:
                try:
                    run(IW, "dev", IFACE, "scan", timeout=max(1, deadline - time.monotonic() - 2))
                    return run("/usr/bin/nmcli", "-f", "IN-USE,SSID,CHAN,SIGNAL,SECURITY",
                               "device", "wifi", "list", "ifname", IFACE, "--rescan", "no")
                except subprocess.CalledProcessError as exc:
                    if "busy" not in (exc.stderr or "").lower():
                        raise
                    time.sleep(1)
            raise RuntimeError("manual scan busy; retry later")
        finally:
            WINDOW.unlink(missing_ok=True)
            enforce()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("serve", "check", "rescan", "release"))
    args = parser.parse_args()
    if args.action != "check" and os.geteuid() != 0:
        parser.error("this action requires root")
    if args.action == "serve":
        serve()
    elif args.action == "release":
        block_scan(False)
    elif args.action == "rescan":
        print(rescan())
    else:
        deadline = time.monotonic() + 5
        while True:
            try:
                print(json.dumps(verify(), sort_keys=True))
                return
            except (OSError, RuntimeError, subprocess.SubprocessError):
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.5)


if __name__ == "__main__":
    main()
