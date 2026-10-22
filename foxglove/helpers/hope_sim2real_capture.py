#!/usr/bin/env python3
"""Session-owned, read-only capture companion. Never publishes robot commands.

Wrap a Runner/Planner command, or run standalone for Laptop mocap. Capture
failures are conspicuous diagnostics and never change the wrapped control flow.
"""
from __future__ import annotations

import argparse
import csv
import ctypes
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time
import re

from hope_sim2real_control import state_root, process_identity

ROS_TOPICS = [
    "/optitrack/poses", "/optitrack/rigid_body_markers", "/optitrack/tf",
    "/poses", "/rigid_body_markers", "/a3/base_pose_laptop_flat", "/a3/base_pose_flat",
    "/UCB_P1/pose", "/UCB_P2/pose", "/ball/pose", "/ball/flight_packet",
    "/racket/command_flat", "/hope/runner/state_flat", "/hope/runner/state_hdu_flat",
    "/hope/runner/control_request_flat", "/hope/runner/control_request_hdu_flat",
    "/hope/runner/summary", "/hope/planner/attempt", "/hope/planner/hit_plane", "/hope/lifecycle/summary",
    "/hope/lifecycle/session_id", "/hope/field/status", "/hope/clock/message_text",
    "/hope/ntp/text", "/hope/ntp/gate_pass", "/hope/base/summary", "/events/rosbag2_messages_lost",
]


def atomic_json(path, data):
    temp = path.with_name(path.name + ".tmp")
    temp.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n")
    temp.replace(path)


def sha256(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def snapshot(root, output, role):
    """Freeze actual local files; source files are not asserted to match binaries."""
    patterns = {
        "mdu": ["policy/**/*", "config/*.yaml", "config/*.xml", "motions/*.csv",
                "teleop_humanlike/*.yaml", "POLICY_DEPLOYMENT.json", "serve_hal_clock.sha256",
                "run_a3_pingpong.sh"],
        "hdu": ["src/hope_planner_cpp/config/*.yaml", "src/hope_planner_cpp/config/*.json"],
        "laptop": ["hope_ws/src/hope_bringup/config/*.yaml", "calibration/*.json",
                   "hope_ws/src/hope_planner_cpp/config/*flight_packetizer.yaml",
                   "hope_ws/calibration_receipts/*.json",
                   "hope_ws/src/hope_msgs/msg/*.msg",
                   "hope_ws/src/motion_capture_tracking_interfaces/msg/*.msg"],
    }[role]
    entries = []
    for relative in sorted({p.relative_to(root) for pattern in patterns for p in root.glob(pattern) if p.is_file()}):
        src, dst = root / relative, output / "configuration" / relative
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        entries.append({"source": str(src), "copy": str(dst.relative_to(output)),
                        "sha256": sha256(dst), "bytes": dst.stat().st_size})
    selection = (Path("/var/lib/hope-lifecycle/ball_physics.json") if role == "hdu" else
                 Path.home() / ".config/hope-foxglove/ball_physics.json")
    if role in {"hdu", "laptop"} and selection.is_file():
        dst = output / "configuration/selected_ball_physics.json"
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(selection, dst)
        entries.append({"source": str(selection), "copy": str(dst.relative_to(output)),
                        "sha256": sha256(dst), "bytes": dst.stat().st_size})
    if role == "mdu":
        for name in ("a3_deploy_onnx_ref_pingpong", "a3_body_drive_debug_record",
                     "libaimrt_record_playback_plugin.so", "liba3_body_drive_debug_ros2_ts.so"):
            p = root / name
            if p.is_file():
                entries.append({"source": str(p), "sha256": sha256(p), "bytes": p.stat().st_size})
        for p in (Path("/opt/agibot/config/hal_ethercat/hal_ethercat_a3_t3d0_cfg.yaml"),
                  Path("/opt/agibot/config/hal_elink/hal_elink_a3_cfg.yaml"),
                  Path("/agibot/data/info/model")):
            if p.is_file():
                dst = output / "configuration/vendor" / p.name
                dst.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(p, dst)
                entries.append({"source": str(p), "copy": str(dst.relative_to(output)),
                                "sha256": sha256(dst), "bytes": dst.stat().st_size})
        for p in sorted((root / "teleop_humanlike").glob("*.onnx")):
            entries.append({"source": str(p), "sha256": sha256(p), "bytes": p.stat().st_size})
    atomic_json(output / "configuration_manifest.json", entries)


def recorder_command(role, root, output):
    if role == "mdu":
        template = root / "config/a3_body_drive_debug_record.iceoryx.yaml"
        text = template.read_text()
        # AimRT: max_bag_num==0 disables deletion. Keep ALL 256 MB chunks.
        if text.count("max_bag_num: 1") != 1:
            raise ValueError("unexpected raw recorder retention template")
        text = text.replace("max_bag_num: 1", "max_bag_num: 0")
        text = text.replace('max_total_size: "approximately 1GB"', 'retention: "complete_session_no_rotation_deletion"')
        text = text.replace("__BAG_PATH__", str(output / "body_raw"))
        text = text.replace("/tmp/a3_body_drive_debug_logs", str(output / "recorder_logs"))
        cfg = output / "body_recorder.yaml"
        cfg.write_text(text)
        command = [str(root / "a3_body_drive_debug_record"), "--cfg_file_path", str(cfg)]
        # The controller owns cores 4..7. Independent recording uses little cores.
        cpus = sorted(set(os.sched_getaffinity(0)) & {0, 1, 2, 3})
        if cpus:
            command = ["taskset", "-c", ",".join(map(str, cpus))] + command
        return command
    return ["ros2", "bag", "record", "--storage", "mcap", "--output", str(output / "ros_raw"),
            "--storage-preset-profile", "none", "--max-cache-size", "67108864",
            "--max-bag-size", "268435456", "--disable-keyboard-controls",
            "--node-name", "hope_sim2real_" + role + "_recorder", "--topics", *ROS_TOPICS]


def recorder_environment(role, root, *, environ=None, ros_prefix=Path("/opt/ros")):
    env = dict(os.environ if environ is None else environ)
    if role == "mdu":
        # run_a3_pingpong.sh sources ROS for the Runner child only. Its sibling
        # recorder also needs ROS's transitive type-support libraries, even
        # when body messages arrive over iceoryx. Do not change controller env.
        distros = [env.get("ROS_DISTRO", ""), "jazzy", "humble"]
        setup = next((ros_prefix / distro / "setup.bash" for distro in distros
                      if distro and (ros_prefix / distro / "setup.bash").is_file()), None)
        if setup is not None:
            result = subprocess.run(
                ["bash", "--noprofile", "--norc", "-c", 'source "$1" >&2 && env -0',
                 "hope-recorder-env", str(setup)], env=env, capture_output=True,
                check=True, timeout=10)
            env = dict(item.decode().split("=", 1) for item in result.stdout.split(b"\0") if item)
    libraries = [str(root), str(root / "lib"), *env.get("LD_LIBRARY_PATH", "").split(":")]
    env["LD_LIBRARY_PATH"] = ":".join(dict.fromkeys(path for path in libraries if path))
    return env


def recorder_error_detail(output):
    try:
        with (output / "recorder.log").open("rb") as stream:
            stream.seek(0, os.SEEK_END)
            stream.seek(max(0, stream.tell() - 8192))
            text = re.sub(r"\x1b\[[0-9;]*m", "", stream.read().decode(errors="replace"))
        errors = [line for line in text.splitlines() if "[Error]" in line or " error:" in line]
        return (errors[0].split("] ", 1)[-1] if errors else text.strip().splitlines()[-1])[-600:]
    except (OSError, IndexError):
        return "inspect recorder.log"


def signal_group(process, sig):
    if process is not None and process.poll() is None:
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass


def recorder_parent_death_signal():
    # Linux-only robot/laptop runtime: don't orphan a disk recorder if the
    # owner is forcibly removed. SIGINT lets MCAP close its index.
    ctypes.CDLL(None, use_errno=True).prctl(1, signal.SIGINT, 0, 0, 0)


def finish_recorder(process):
    if process is None:
        return None, False
    forced = False
    for sig, timeout in ((signal.SIGINT, 15), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
        signal_group(process, sig)
        try:
            return process.wait(timeout=timeout), forced
        except subprocess.TimeoutExpired:
            forced = True
    return process.poll(), True


def clock_probe(output, name):
    # Read-only clock evidence. Offset is not assumed zero across machines.
    with (output / (name + ".txt")).open("w") as f:
        for cmd in (["chronyc", "tracking"], ["timedatectl", "show", "-p", "NTPSynchronized"]):
            try:
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=2)
                f.write("$ " + " ".join(cmd) + "\n" + result.stdout + result.stderr)
            except (OSError, subprocess.TimeoutExpired) as exc:
                f.write(str(exc) + "\n")


def run_controlled(args):
    """Keep controller ownership fixed while rotating independent recorders."""
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    root = state_root()
    control = root / "recording.request"
    stopping = False
    child = recorder = None
    segment = ""
    capture_output = None
    status_path = output / "monitor.json"
    registry = root / "workers" / f"{os.getpid()}.json"
    registry.parent.mkdir(exist_ok=True)
    status = {"role": args.role, "session_id": args.session_id, "state": "OFF", "segment": "", "errors": []}

    def on_signal(sig, _frame):
        nonlocal stopping
        if stopping:
            return
        stopping = True
        if child is not None and child.poll() is None:
            try:
                child.send_signal(sig)
            except ProcessLookupError:
                pass

    def close_segment():
        nonlocal recorder
        if recorder is not None:
            rc, forced = finish_recorder(recorder)
            if forced or rc not in (0, -signal.SIGINT):
                status["errors"].append(f"segment {segment} close: exit={rc} forced={forced}")
            recorder = None

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    atomic_json(status_path, status)
    atomic_json(registry, {"pid": os.getpid(), "identity": process_identity(os.getpid()),
                           "status_path": str(status_path), "output": str(output)})
    try:
        if args.command:
            child = subprocess.Popen(args.command, env={**os.environ, "HOPE_RECORD_CONTROL_FILE": str(control)})
        while not stopping and (child is None or child.poll() is None):
            requested = control.read_text().strip() if control.exists() else ""
            if requested and not re.fullmatch(r"record_[A-Za-z0-9_]{1,88}", requested):
                requested = ""
            if requested != segment:
                if recorder is not None:
                    status.update(state="STOPPING", updated_wall_ns=time.time_ns())
                    atomic_json(status_path, status)
                close_segment()
                segment = requested
                status.update(segment=segment, state="STARTING" if segment else "OFF")
                if segment:
                    capture_output = output / "recordings" / segment / "capture"
                    command = [sys.executable, str(Path(__file__).resolve()), "--role", args.role,
                               "--root", str(args.root), "--session-id", args.session_id,
                               "--output", str(capture_output), "--reserve-bytes", str(args.reserve_bytes)]
                    recorder = subprocess.Popen(command, start_new_session=True,
                                                preexec_fn=recorder_parent_death_signal)
            if recorder is not None:
                try:
                    capture = json.loads((capture_output / "status.json").read_text())
                    status["state"] = "ON" if capture["state"] == "RECORDING" else "STARTING"
                    status["errors"] = capture.get("errors", [])
                    if status["errors"] or recorder.poll() is not None:
                        status["state"] = "ERROR"
                except (OSError, ValueError):
                    if recorder.poll() is not None:
                        status.update(state="ERROR", errors=["recorder exited before opening storage"])
            csv_files = list(output.glob("*.recording.json")) + list(output.parent.glob("*.recording.json"))
            status["csv"] = {}
            for path in csv_files:
                try:
                    value = json.loads(path.read_text())
                    status["csv"][path.name] = value
                    if value["state"] == "ERROR":
                        status["state"] = "ERROR"
                    elif segment and (value["state"] != "ON" or value["segment"] != segment):
                        status["state"] = "STARTING"
                    elif not segment and value["state"] == "ON":
                        status["state"] = "STOPPING"
                except (OSError, ValueError):
                    status["state"] = "STARTING" if segment else "STOPPING"
            status["updated_wall_ns"] = time.time_ns()
            atomic_json(status_path, status)
            time.sleep(.1)
        if child is not None:
            child.wait()
    except OSError as exc:
        status.update(state="ERROR", errors=[str(exc)])
        print("[sim2real] CAPTURE ERROR: " + str(exc), flush=True)
        if child is not None:
            child.wait()  # capture failure never kills or restarts control
    finally:
        close_segment()
        status.update(state="CLOSED", command_exit_code=None if child is None else child.returncode)
        try:
            atomic_json(status_path, status)
            registry.unlink(missing_ok=True)
        except OSError:
            pass
    return 0 if child is None else child.returncode


def run(args):
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)  # no overwrite or mixed attempts
    root = args.root.resolve()
    status = {"schema": 1, "role": args.role, "session_id": args.session_id,
              "host": socket.gethostname(), "root": str(root), "start_wall_ns": time.time_ns(),
              "state": "STARTING", "errors": [], "command": args.command,
              "availability": "NOT_PROVEN_UNTIL_MESSAGE_COUNTS_AND_GAPS_CHECKED",
              "motor_current_voltage_temperature_drive_limits": "NOT_IN_BODY_DRIVE_ROS_SCHEMA",
              "effort_semantics": "vendor_reported_joint_effort_not_calibrated_motor_torque"}
    atomic_json(output / "status.json", status)
    recorder = child = None
    stopping = False

    def on_signal(sig, _frame):
        nonlocal stopping
        if stopping:
            return
        stopping = True
        if child is not None and child.poll() is None:
            try:
                child.send_signal(sig)
            except ProcessLookupError:
                pass

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    for operation in (lambda: snapshot(root, output, args.role),
                      lambda: clock_probe(output, "clock_start")):
        try:
            operation()
        except Exception as exc:
            status["errors"].append(str(exc))
    log = (output / "recorder.log").open("wb")
    try:
        try:
            if shutil.disk_usage(output).free < args.reserve_bytes:
                raise RuntimeError("disk reserve reached before recording")
            cmd = recorder_command(args.role, root, output)
            status["recorder_command"] = cmd
            env = recorder_environment(args.role, root)
            recorder = subprocess.Popen(cmd, cwd=root, env=env, stdout=log, stderr=subprocess.STDOUT,
                                        start_new_session=True, stdin=subprocess.DEVNULL,
                                        preexec_fn=recorder_parent_death_signal)
            status["recorder_pid"] = recorder.pid
        except Exception as exc:
            status["errors"].append("recorder startup: " + str(exc))
            print("[sim2real] CAPTURE ERROR: " + str(exc), flush=True)
        if args.command and not stopping:
            # Preserve the original controller's terminal/session membership.
            # tmux's final HUP must still reach it if graceful stop times out.
            child = subprocess.Popen(args.command)
        with (output / "clock_resource_samples.csv").open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["realtime_before_ns", "monotonic_ns", "realtime_after_ns", "disk_free_bytes",
                             "load_1m", "recorder_alive", "wrapped_process_alive", "wrapped_proc_stat"])
            while not stopping and (child is None or child.poll() is None):
                before, mono, after = time.time_ns(), time.monotonic_ns(), time.time_ns()
                free = shutil.disk_usage(output).free
                alive = recorder is not None and recorder.poll() is None
                if alive and free < args.reserve_bytes:
                    status["errors"].append("disk reserve reached; raw capture stopped, controller unchanged")
                    rc, forced = finish_recorder(recorder)
                    status.update(recorder_exit_code=rc, forced_close=forced)
                    alive = False
                if recorder is not None and not alive and "recorder_exit_code" not in status:
                    status["recorder_exit_code"] = recorder.poll()
                    status["errors"].append(
                        f"recorder exited before session end (exit={recorder.returncode}): "
                        + recorder_error_detail(output))
                    print("[sim2real] CAPTURE ERROR: recorder exited", flush=True)
                proc_stat = ""
                if child:
                    try:
                        proc_stat = Path(f"/proc/{child.pid}/stat").read_text().strip()
                    except FileNotFoundError:
                        pass
                writer.writerow([before, mono, after, free, os.getloadavg()[0], alive,
                                 child is not None and child.poll() is None, proc_stat])
                stream.flush()
                for health_file in output.glob("*.health.json"):
                    try:
                        health = json.loads(health_file.read_text())
                        status.setdefault("csv_health", {})[health_file.name] = health
                        if health.get("dropped_bytes") or health.get("io_error"):
                            error = health_file.name + ": CSV loss or I/O failure"
                            if error not in status["errors"]:
                                status["errors"].append(error)
                                print("[sim2real] CAPTURE ERROR: " + error, flush=True)
                    except (OSError, ValueError):
                        pass
                status.update(state="RECORDING" if alive else "CAPTURE_ERROR", last_wall_ns=after,
                              disk_free_bytes=free)
                atomic_json(output / "status.json", status)
                time.sleep(0.25)
        if child is not None:
            # Signals were forwarded above. Never terminate a running controller
            # merely because recording failed or the disk is getting full.
            child.wait()
    except OSError as exc:
        status["errors"].append("capture monitor I/O: " + str(exc))
        print("[sim2real] CAPTURE ERROR: " + str(exc), flush=True)
        if child is not None:
            # E.g. ENOSPC while writing health/clock files. Keep the original
            # control process and its signal handler alive; never restart it.
            child.wait()
        else:
            raise
    finally:
        rc, forced = finish_recorder(recorder)
        log.close()
        if rc not in (None, 0, -signal.SIGINT):
            status["errors"].append("recorder exit code " + str(rc))
        status.update(state="CLOSED", end_wall_ns=time.time_ns(), recorder_exit_code=rc,
                      forced_close=forced or status.get("forced_close", False),
                      command_exit_code=None if child is None else child.returncode)
        # The child closes its CSVs after our last periodic sample. Retain its
        # final health, including losses that happened during shutdown.
        for health_file in output.glob("*.health.json"):
            try:
                health = json.loads(health_file.read_text())
                status.setdefault("csv_health", {})[health_file.name] = health
                if not health.get("closed") or health.get("dropped_bytes") or health.get("io_error"):
                    status["errors"].append(health_file.name + ": CSV not cleanly complete")
            except (OSError, ValueError) as exc:
                status["errors"].append(health_file.name + ": " + str(exc))
        try:
            atomic_json(output / "status.json", status)
            clock_probe(output, "clock_end")
        except OSError as exc:
            print("[sim2real] CAPTURE ERROR: cannot finalize metadata: " + str(exc), flush=True)
    return 0 if child is None else child.returncode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--role", choices=["mdu", "hdu", "laptop"], required=True)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--reserve-bytes", type=int)
    parser.add_argument("--controlled", action="store_true", help="Use the Foxglove Record ON/OFF intent")
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.reserve_bytes is None:
        args.reserve_bytes = (1 if args.role == "hdu" else 2) * 1024**3
    if args.command[:1] == ["--"]:
        args.command = args.command[1:]
    return run_controlled(args) if args.controlled else run(args)


if __name__ == "__main__":
    raise SystemExit(main())
