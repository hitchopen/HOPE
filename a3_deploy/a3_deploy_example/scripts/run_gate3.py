#!/usr/bin/env python3
"""Two-file Gate3 entry. Native Runner owns policy compatibility validation."""
from __future__ import annotations

import argparse
import datetime as dt
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import sys
import time

import yaml

GEAR = Path(__file__).resolve().parents[1]
ROOT = GEAR.parents[1]
CONFIG = GEAR / "config/gate3"
ABI_MODES = {"small_station_112_v1": "small_station112", "small_station_324_v1": "compact324",
             "ball_clock_110_v1": "rally_v14"}


def digest(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n")


def resolve_path(value, base=ROOT):
    p = Path(value).expanduser()
    return (p if p.is_absolute() else base / p).resolve()


def runtime_config(path=None):
    source = resolve_path(path, Path.cwd()) if path else CONFIG / "runtime.local.yaml"
    if not source.exists() and not path:
        source = CONFIG / "runtime.yaml"
    cfg = yaml.safe_load(source.read_text())
    required = {"runner_dist", "sim_install", "hope_ws"}
    if not isinstance(cfg, dict) or not required <= cfg.keys() or cfg.keys() - required:
        raise ValueError("runtime config requires only runner_dist, sim_install, hope_ws")
    return {key: str(resolve_path(value)) for key, value in cfg.items()}


def native_inspect(dist, policy_dir):
    binary = Path(dist) / "a3_deploy_onnx_ref_pingpong"
    env = dict(os.environ, LD_LIBRARY_PATH=f"{dist}:/opt/ros/jazzy/lib")
    result = subprocess.run([str(binary), "--inspect-policy", "--policy-dir", str(policy_dir)],
                            env=env, capture_output=True, text=True, timeout=60)
    if result.returncode:
        raise ValueError("Runner could not load the two-file policy:\n" + result.stderr[-6000:])
    receipt = yaml.safe_load(result.stdout)
    if not isinstance(receipt, dict) or receipt.get("inspection_version") != 1:
        raise ValueError("Runner returned no supported policy inspection receipt")
    return receipt


def scene_environment(scene, balls=None):
    """Resolve a fixed-HOME test independently of policy/training provenance."""
    if not isinstance(scene, dict) or scene.get("schema_version") != 1 or scene.get("family") != "fixed_home":
        raise ValueError("scene requires schema_version: 1 and family: fixed_home")
    n = balls if balls is not None else scene["balls"]
    if type(n) is not int or n < 26 or n % 2:
        raise ValueError("fixed_home requires an even --balls >= 26")
    clean, cycle = scene["clean_serves"], scene["rapid_cycle"]
    if len(clean) != 6 or not cycle:
        raise ValueError("scene requires six clean serves and a nonempty rapid_cycle")
    for row in clean + cycle:
        if not isinstance(row, list) or len(row) != 6 or any(
                isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in row):
            raise ValueError("each serve must contain six finite x,y,z,vx,vy,vz values")
    timing = scene["timing"]
    for key in ("clean_flight_s", "rapid_flight_s", "final_flight_s", "clean_pause_s", "rapid_pause_s"):
        value = timing[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0 or (
                "flight" in key and value == 0):
            raise ValueError(f"invalid timing.{key}")
    rows = clean + [cycle[i % len(cycle)] for i in range(n - 6)]
    schedules = {}
    for name, fallback in (
        ("flight_s_list", [timing["clean_flight_s"]] * 6 +
         [timing["rapid_flight_s"]] * (n - 7) + [timing["final_flight_s"]]),
        ("pause_s_list", [timing["clean_pause_s"]] * 6 +
         [timing["rapid_pause_s"]] * (n - 6)),
    ):
        values = timing.get(name, fallback)
        if not isinstance(values, list) or len(values) != n or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or
            not math.isfinite(v) or v < 0 or (name == "flight_s_list" and v == 0)
            for v in values
        ):
            raise ValueError(f"timing.{name} must contain {n} finite "
                             "positive flight / nonnegative pause durations")
        schedules[name] = values
    env = {
        "PP_SERVES": str(n),
        "PP_SERVES_LIST": json.dumps([v for row in rows for v in row]),
        "PP_FLIGHT_S_LIST": json.dumps(schedules["flight_s_list"]),
        "PP_PAUSE_S_LIST": json.dumps(schedules["pause_s_list"]),
        "PP_CLEAN_FIXED_HOME_SERVES": "6",
        "PP_MIN_ENGAGED_SERVES": str(n), "PP_MIN_COMPLETED_SERVES": str(n),
        "PP_MIN_RAPID_ENGAGED": str(n - 6), "PP_MIN_RAPID_COMPLETED": str(n - 6),
    }
    # Named numerical report thresholds only; scene files cannot override paths,
    # process commands, policy identity, Planner ABI or the report implementation.
    allowed = {"PP_RECOVERY_RADIUS_M", "PP_RECOVERY_SPEED_MAX_MPS", "PP_RECOVERY_DWELL_S",
               "PP_RECOVERY_YAW_MAX_DEG", "PP_MOTION_IDLE_S", "PP_MIN_MOTION_IDLE_S",
               "PP_REQUIRED_POST_COMPLETION_TAIL_S", "PP_RAPID_MAX_ENGAGE_GAP_S",
               "PP_RAPID_CONTACT_TO_NEXT_COMMIT_MIN_S", "PP_RAPID_CONTACT_TO_NEXT_COMMIT_MAX_S",
               "PP_HOME_MAX_SUPPORT_MIDPOINT_END_M", "PP_HOME_MAX_FOOT_ANCHOR_END_M",
               "PP_HOME_MIN_SIGNED_WIDTH_M", "PP_HOME_MAX_SIGNED_WIDTH_M",
               "PP_MAX_END_YAW_DRIFT_DEG", "PP_MAX_PEAK_YAW_DRIFT_DEG"}
    for key, value in scene.get("metrics", {}).items():
        if key not in allowed or isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise ValueError(f"invalid scene metric: {key}")
        env[key] = str(value)
    return env


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--policy-dir", help="directory containing params/deploy.yaml and exported/policy.onnx")
    p.add_argument("--deploy", help="standalone deploy.yaml (use together with --onnx)")
    p.add_argument("--onnx", help="standalone policy ONNX")
    p.add_argument("--scenario", default="fixed-home", help="installed scene name or YAML path")
    p.add_argument("--balls", type=int)
    p.add_argument("--headless", action="store_true", help="run without viewer (default)")
    p.add_argument("--viewer", action="store_true")
    p.add_argument("--output", help="new result directory; generated automatically by default")
    p.add_argument("--runtime", help="one-time environment config override")
    p.add_argument("--check", action="store_true", help="load the actual Runner policy, print resolved run; do not start simulation")
    p.add_argument("--dry-run", action="store_true", help="alias for --check")
    p.add_argument("--configure", action="store_true", help="save installed environment paths once")
    p.add_argument("--runner-dist")
    p.add_argument("--sim-install")
    p.add_argument("--hope-ws")
    return p


def execute(args):
    cfg = runtime_config(args.runtime)
    for key in cfg:
        if value := getattr(args, key):
            cfg[key] = str(resolve_path(value, Path.cwd()))
    dist = Path(cfg["runner_dist"])
    for path in (dist / "a3_deploy_onnx_ref_pingpong", dist / "run_a3_pingpong.sh",
                 Path(cfg["sim_install"]) / "bin/aimrt_main",
                 Path(cfg["hope_ws"]) / "install/local_setup.bash"):
        if not path.is_file():
            raise ValueError(f"installed runtime asset missing: {path}")
    if args.configure:
        target = CONFIG / "runtime.local.yaml"
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(yaml.safe_dump(cfg, sort_keys=False))
        print(f"Saved installed environment: {target}")
        return 0
    if args.headless and args.viewer:
        raise ValueError("choose --headless or --viewer")
    if args.policy_dir and (args.deploy or args.onnx):
        raise ValueError("choose --policy-dir or the --deploy/--onnx pair")
    if args.policy_dir:
        source = resolve_path(args.policy_dir, Path.cwd())
        deploy, onnx = source / "params/deploy.yaml", source / "exported/policy.onnx"
    elif args.deploy and args.onnx:
        deploy, onnx = resolve_path(args.deploy, Path.cwd()), resolve_path(args.onnx, Path.cwd())
    else:
        raise ValueError("provide --policy-dir, or both --deploy and --onnx")
    for path in (deploy, onnx):
        if not path.is_file():
            raise ValueError(f"policy file missing: {path}")
    scene_path = Path(args.scenario).expanduser()
    if not scene_path.is_file():
        scene_path = CONFIG / "scenarios" / (args.scenario + ".yaml")
    scene_path = scene_path.resolve()
    scene = yaml.safe_load(scene_path.read_text())
    scene_env = scene_environment(scene, args.balls)
    out = resolve_path(args.output, Path.cwd()) if args.output else GEAR / "gate3/runs" / dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    out.mkdir(parents=True, exist_ok=False)
    policy = out / "policy"
    (policy / "params").mkdir(parents=True)
    (policy / "exported").mkdir()
    shutil.copy2(deploy, policy / "params/deploy.yaml")
    shutil.copy2(onnx, policy / "exported/policy.onnx")
    shutil.copy2(scene_path, out / "scene.yaml")
    inspection = native_inspect(dist, policy)
    write_json(out / "policy_inspection.json", inspection)
    # The generic scene engine currently shares the implemented schema-2
    # SmallStation ABI. Historical ABI adapters keep their existing entrypoints.
    if inspection["policy_abi"] not in ABI_MODES:
        raise ValueError("unified scene requires SmallStation 112, compact324 or ball-clock 110 ABI; "
                         "use the historical Gate3 entry for this legacy ABI")
    exports = out / "scene.env.sh"
    exports.write_text("\n".join(f"export {k}={shlex.quote(v)}" for k, v in scene_env.items()) + "\n")
    # Clear historical per-run knobs; every experiment below is reproducible
    # from its saved files and the one-time runtime configuration.
    env = {k: v for k, v in os.environ.items() if not k.startswith(("PP_", "A3_"))}
    env.update({
        "PP_DIST": str(dist), "PP_HOPE_WS": cfg["hope_ws"], "PP_SIM_INSTALL": cfg["sim_install"],
        "A3_SIM_INSTALL": cfg["sim_install"], "A3_SOURCE_ROBOT_ENV": "0",
        "A3_PINGPONG_RUNTIME_CFG": str(dist / "config/a3_runtime_config.pingpong.hitter_pingpong.yaml"),
        "PP_POLICY_ONNX": str(policy / "exported/policy.onnx"), "PP_POLICY_DIR": str(policy),
        "PP_SKIP_P1_CALIBRATION_SHA": "1", "PP_VIEWER": "1" if args.viewer else "0",
        "PP_SCENE_EXPORTS": str(exports), "PP_SERVES": scene_env["PP_SERVES"],
        "PP_MANAGED_RUN": "1", "PP_CHILD_PIDS_FILE": str(out / "child_pids.txt"),
        "A3_IOX_ROUDI_PID_FILE": str(out / "roudi.pid"),
    })
    receipt = {"status": "prepared", "evidence_scope": "simulator_truth", "output": str(out),
               "runtime": cfg, "policy": inspection, "scene": scene,
               "resolved_scene_environment": scene_env,
               "policy_sha256": digest(policy / "exported/policy.onnx"),
               "deploy_sha256": digest(policy / "params/deploy.yaml"),
               "runner_sha256": digest(dist / "a3_deploy_onnx_ref_pingpong"),
               "scene_sha256": digest(out / "scene.yaml")}
    write_json(out / "run.json", receipt)
    print(f"Gate3: {inspection['policy_abi']}, {scene_env['PP_SERVES']} balls, scene={scene_path.name}", flush=True)
    print(f"Results: {out}", flush=True)
    if args.check or args.dry_run:
        print("POLICY_CHECK_PASS (no simulation or transport started)")
        return 0
    # The inherited engine still uses fixed /tmp telemetry paths and shared
    # iceoryx. Serialize runs until that backend gains namespaced resources.
    with open("/tmp/hope_gate3_unified.lock", "a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise ValueError("another unified Gate3 run owns the simulator") from None
        reject_active_runtime()
        started = time.time()
        receipt["status"] = "running"
        write_json(out / "run.json", receipt)
        command = ["bash", str(GEAR / "scripts/pp_gate3_hitter_pingpong.sh")]
        process = None
        try:
            with (out / "console.log").open("w") as log:
                process = subprocess.Popen(command, cwd=GEAR, env=env, stdout=log, stderr=subprocess.STDOUT,
                                           start_new_session=True)
                while True:
                    try:
                        rc = process.wait(timeout=30)
                        break
                    except subprocess.TimeoutExpired:
                        print(f"Gate3 running ({int(time.time() - started)}s); {out / 'console.log'}", flush=True)
        except KeyboardInterrupt:
            rc = 130
            if process:
                os.killpg(process.pid, signal.SIGTERM)
                while True:
                    try:
                        process.wait(timeout=30)
                        break
                    except subprocess.TimeoutExpired:
                        print("Waiting for launched balls to finish before simulator cleanup", flush=True)
        finally:
            # Include diagnostics on failed setup/early falls as well as success.
            for path in Path("/tmp").glob("pp_*"):
                if path.is_file() and path.suffix in (".csv", ".json", ".log") and path.stat().st_mtime >= started:
                    shutil.copy2(path, out / path.name)
        receipt.update(status="finished", exit_code=rc, elapsed_s=time.time() - started)
        report_path = out / "pp_rally_report.json"
        if report_path.exists():
            receipt["certification_pass"] = json.loads(report_path.read_text()).get("certification_pass", False)
        write_json(out / "run.json", receipt)
        print(f"Gate3 exited {rc}; logs and reports: {out}")
        return rc


def reject_active_runtime():
    # The simulator and robot use shared channels. Never start a second owner
    # or use the old harness's broad pkill cleanup on an existing session.
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            words = (entry / "cmdline").read_bytes().split(b"\0")
            names = {Path(w.decode(errors="replace")).name for w in words[:2]}
            if names & {"aimrt_main", "a3_deploy_onnx_ref_pingpong", "hope_planner_cpp_node", "iox-roudi"}:
                raise ValueError(f"runtime already active (PID {entry.name}); existing process left running")
        except (OSError, ProcessLookupError):
            continue


def main():
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    try:
        return execute(parser().parse_args())
    except (ValueError, OSError, KeyError, TypeError, yaml.YAMLError, subprocess.TimeoutExpired) as exc:
        print(f"Gate3 setup failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
