#!/usr/bin/env python3
"""Allowlisted, restart-applied ball physics shared by Planner and packetizer."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import ipaddress
import os
from pathlib import Path
import stat
import sys
import time

import yaml

from hope_field_assets import atomic_write

PROFILES = {
    "berkeley": "ball_physics_real_20261010.yaml",
    "ppq002": "ball_physics_ppq002_20261015.yaml",
    "ppq004": "ball_physics_ppq004_20261020.yaml",
}
NODES = ("hope_planner", "hope_ball_flight_packetizer")
INSPECTION_CACHE_TTL_S = 60.0
INSPECTION_CACHE_SCHEMA = 1


def inspection_cache_path(role: str) -> Path:
    # HDU helpers inherit ProtectHome=read-only and ProtectSystem=strict from
    # operations. Its existing StateDirectory is writable without weakening
    # the service sandbox; Laptop helpers run in the operator's user runtime.
    if role == "hdu":
        return Path("/var/lib/hope-lifecycle/ball-model-cache/hdu.json")
    if role == "laptop":
        return Path.home() / ".cache/hope-ball-models/laptop.json"
    raise ValueError("unsupported ball model role")


def inspection_identity(path: Path, config_dir: Path, role: str, *,
                        base_config: Path | None = None, proc: Path = Path("/proc")) -> dict | None:
    """Cheap local identity only: a cache hit must not import ROS or discover DDS.

    This cannot establish the absence of a duplicate ROS name on another host.
    Explicit verification still checks the live graph before START.
    """
    executable = "hope_planner_cpp_node" if role == "hdu" else "hope_ball_flight_packetizer"
    processes = []
    for directory in proc.glob("[0-9]*"):
        try:
            argv = (directory / "cmdline").read_bytes()
        except OSError:
            continue
        if Path(os.fsdecode(argv.split(b"\0", 1)[0])).name != executable:
            continue
        # A matching but unreadable process makes the identity uncertain.
        starttime = (directory / "stat").read_text().rsplit(")", 1)[1].split()[19]
        binary = (directory / "exe").stat()
        processes.append({"pid": int(directory.name), "starttime": starttime,
                          "argv_sha256": hashlib.sha256(argv).hexdigest(),
                          "executable": [binary.st_dev, binary.st_ino, binary.st_size, binary.st_mtime_ns]})
    if len(processes) != 1:
        return None
    base_config = base_config or config_dir / (
        "model21800_hardware.yaml" if role == "hdu" else "model21800_flight_packetizer.yaml")
    files = [base_config, path, *(config_dir / name for name in PROFILES.values())]
    signatures = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else None for p in files}
    boot_id = (proc / "sys/kernel/random/boot_id").read_text().strip()
    if not boot_id:
        return None
    return {"boot_id": boot_id, "role": role, "process": processes[0], "files": signatures,
            "environment": {name: os.environ.get(name, "") for name in
                            ("ROS_DOMAIN_ID", "ROS_STATIC_PEERS", "RMW_IMPLEMENTATION")},
            "helper_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}


class BallModelInspectionCache:
    """Private, bounded diagnostic receipts; never a substitute for START checks."""

    def __init__(self, path: Path, identity, *, wall_time=time.time, monotonic=time.monotonic):
        self.path, self.identity = path, identity
        self.wall_time, self.monotonic = wall_time, monotonic

    def _identity(self):
        try:
            return self.identity()
        except Exception:
            # /proc/config/cache problems must not break the original live query.
            return None

    def invalidate(self):
        try:
            self.path.unlink(missing_ok=True)
        except OSError:
            pass

    def _read(self, identity):
        try:
            fd = os.open(self.path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
            with os.fdopen(fd, "r") as stream:
                info = os.fstat(stream.fileno())
                if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or
                        info.st_mode & 0o077 or info.st_size > 65536):
                    return None
                record = json.load(stream)
            if record["schema"] != INSPECTION_CACHE_SCHEMA or record["identity"] != identity:
                return None
            age = self.monotonic() - record["verified_monotonic_s"]
            wall_age = self.wall_time() - record["verified_at_unix_s"]
            if (not math.isfinite(age) or not 0 <= age < INSPECTION_CACHE_TTL_S or
                    not math.isfinite(wall_age) or wall_age < 0 or abs(age - wall_age) > 5):
                return None
            result = record["result"]
            if (result["runtime"] != "VERIFIED" or result["configured_model"] not in PROFILES or
                    not isinstance(result["parameters"], dict) or len(result["parameters"]) != 14 or
                    not isinstance(result["configured_parameters"], dict) or
                    result["configured_parameters"].keys() != result["parameters"].keys() or
                    result["runtime_model"] != result["configured_model"] or
                    compare_parameters(result["configured_parameters"], result["parameters"])):
                return None
            if self._identity() != identity:
                return None
            return {**result, "runtime": "CACHED_VERIFIED", "verification_cached": True,
                    "verification_age_s": age, "verified_at_unix_s": record["verified_at_unix_s"],
                    "verification_scope": "process_identity", "verification_cache_ttl_s": INSPECTION_CACHE_TTL_S}
        except Exception:
            return None

    def _write(self, identity, result, verified_wall, verified_monotonic):
        try:
            self.path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            parent = self.path.parent.lstat()
            if (not stat.S_ISDIR(parent.st_mode) or parent.st_uid != os.getuid() or
                    parent.st_mode & 0o077):
                return
            # atomic_write uses mkstemp (0600), fsync and same-directory rename.
            atomic_write(self.path, (json.dumps({"schema": INSPECTION_CACHE_SCHEMA, "identity": identity,
                "verified_at_unix_s": verified_wall, "verified_monotonic_s": verified_monotonic,
                "result": result}) + "\n").encode())
        except Exception:
            pass

    def inspect(self, read_live, *, force_live=False):
        identity = self._identity()
        if not force_live and identity is not None:
            cached = self._read(identity)
            if cached is not None:
                return cached
        # A failed or mismatched new query must not leave a reusable old success.
        self.invalidate()
        result = read_live()  # Exceptions here retain the original live semantics.
        if result["runtime"] != "VERIFIED":
            return result
        wall, monotonic = self.wall_time(), self.monotonic()
        result = {**result, "verified_at_unix_s": wall, "verification_age_s": 0.0,
                  "verification_cached": False, "verification_scope": "live_parameters"}
        if identity is not None and self._identity() == identity:
            self._write(identity, result, wall, monotonic)
        return result


def profile(config_dir: Path, model: str) -> dict:
    if model not in PROFILES:
        raise ValueError("ball model must be berkeley, ppq002 or ppq004")
    document = yaml.safe_load((config_dir / PROFILES[model]).read_text())
    params = document[NODES[0]]["ros__parameters"]
    if len(params) != 14 or params != document[NODES[1]]["ros__parameters"]:
        raise ValueError("ball profile must provide the same 14 parameters to both nodes")
    return {name: {"ros__parameters": params} for name in NODES}


def selected(path: Path, config_dir: Path) -> str | None:
    if not path.exists():
        return None
    document = json.loads(path.read_text())  # JSON is also a valid ROS YAML file.
    for model in PROFILES:
        if document == profile(config_dir, model):
            return model
    raise ValueError("saved ball physics does not match any reviewed profile")


def select(path: Path, config_dir: Path, model: str | None) -> dict:
    if model is None:  # Internal rollback to a previously absent override.
        path.unlink(missing_ok=True)
    else:
        atomic_write(path, (json.dumps(profile(config_dir, model), indent=2) + "\n").encode())
    return {"model": selected(path, config_dir), "applies": "NEXT_PROCESS_START"}


def compare_parameters(expected: dict, actual: dict) -> list[str]:
    # Detect both missing values and ROS type mismatches (bool is an int in Python).
    return [key for key, value in expected.items()
            if type(value) not in (bool, int, float) or type(actual.get(key)) is not type(value) or
            (actual[key] != value if isinstance(value, bool) else not math.isfinite(actual[key]) or abs(actual[key] - value) > 1e-12)]


def live_parameters(node_name: str, names: list[str]) -> dict | None:
    # ROS-generated messages import NumPy. This short-lived parameter query
    # needs no parallel BLAS work; keep each periodic check to one BLAS thread.
    # Set these before importing ROS, even if the parent requests more threads.
    for variable in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
        os.environ[variable] = "1"
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from rclpy.parameter import Parameter, parameter_value_to_python
    rclpy.init(args=[])
    node = rclpy.create_node("hope_ball_model_check", start_parameter_services=False, enable_rosout=False,
                            parameter_overrides=[Parameter("start_type_description_service", value=False)])
    try:
        client = node.create_client(GetParameters, f"/{node_name}/get_parameters")
        # Keep the same DDS participant while discovery catches up. Four attempts
        # bound discovery + replies to 20 s, below the caller's 30 s timeout.
        error = f"{node_name} parameters are unreachable"
        for attempt in range(4):
            available = client.wait_for_service(timeout_sec=2.0)
            found = sum(name == node_name for name, _ in node.get_node_names_and_namespaces())
            if found > 1:
                raise ValueError(f"duplicate {node_name} nodes; cannot verify ball physics")
            if not available:
                executable = "hope_planner_cpp_node" if node_name == "hope_planner" else node_name
                process_found = False
                for cmdline in Path("/proc").glob("[0-9]*/cmdline"):
                    try:
                        argv = cmdline.read_bytes().split(b"\0")
                        if argv and Path(argv[0].decode()).name == executable:
                            process_found = True
                            break
                    except (OSError, UnicodeError):
                        continue
                if not found and not process_found:
                    return None
                error = f"{node_name} exists but its parameters are unreachable"
                continue
            future = client.call_async(GetParameters.Request(names=names))
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
            if future.done() and future.result() is not None:
                if sum(name == node_name for name, _ in node.get_node_names_and_namespaces()) > 1:
                    raise ValueError(f"duplicate {node_name} nodes; cannot verify ball physics")
                return dict(zip(names, map(parameter_value_to_python, future.result().values)))
            client.remove_pending_request(future)
            future.cancel()
            error = f"{node_name} parameter read timed out"
        raise ValueError(f"{error} after 4 attempts (20 s maximum)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def identify_parameters(config_dir: Path, parameters: dict) -> str | None:
    for model in PROFILES:
        expected = profile(config_dir, model)[NODES[0]]["ros__parameters"]
        if not compare_parameters(expected, parameters):
            return model
    return None


def inspect_model(path: Path, config_dir: Path, role: str, *, base_config: Path | None = None,
                  read_live=live_parameters) -> dict:
    node_name = NODES[0 if role == "hdu" else 1]
    if base_config is None:
        base_config = config_dir / ("model21800_hardware.yaml" if role == "hdu" else
                                    "model21800_flight_packetizer.yaml")
    keys = list(profile(config_dir, "ppq004")[node_name]["ros__parameters"])
    configured = yaml.safe_load(base_config.read_text())[node_name]["ros__parameters"]
    configured = {key: configured.get(key) for key in keys}
    selection = selected(path, config_dir)
    if selection is not None:
        configured.update(profile(config_dir, selection)[node_name]["ros__parameters"])
    actual = read_live(node_name, keys)
    mismatch = compare_parameters(configured, actual) if actual is not None else []
    return {"model": selection, "configured_model": identify_parameters(config_dir, configured),
            "configured_parameters": configured, "base_config": str(base_config),
            "override_config": str(path) if path.exists() else None,
            "runtime": "NOT_RUNNING" if actual is None else "MISMATCH" if mismatch else "VERIFIED",
            "runtime_model": identify_parameters(config_dir, actual) if actual is not None else None,
            "parameters": actual, "mismatches": mismatch}


def handle(path: Path, config_dir: Path, role: str, request: dict, *, read_live=live_parameters,
           base_config: Path | None = None, inspection_cache: BallModelInspectionCache | None = None) -> dict:
    operation = request.get("op")
    if operation == "select_ball_model":
        if inspection_cache is not None:
            inspection_cache.invalidate()
        return select(path, config_dir, request.get("model"))
    model = selected(path, config_dir)
    if operation == "read_ball_model":
        return {"model": model}
    if operation not in {"verify_ball_model", "inspect_ball_model"}:
        raise ValueError("unsupported ball model operation")
    if operation == "verify_ball_model" and "model" in request and request["model"] != model:
        raise ValueError(f"{role} configured ball model differs from selection")
    inspect = lambda: inspect_model(path, config_dir, role, base_config=base_config, read_live=read_live)
    result = (inspection_cache.inspect(inspect, force_live=operation == "verify_ball_model")
              if inspection_cache is not None else inspect())
    if operation == "inspect_ball_model":
        return result
    if result["configured_model"] is None:
        raise ValueError(f"{role} configured physics does not match any reviewed model")
    if result["runtime"] == "MISMATCH":
        raise ValueError(f"{role} still uses different physics ({', '.join(result['mismatches'])}); "
                         "restart the affected process before START SYSTEM")
    if result["runtime"] == "NOT_RUNNING" and request.get("require_running"):
        raise ValueError(f"{role} is not running; physics was not verified")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--role", choices=("hdu", "laptop"), required=True)
    parser.add_argument("--config-dir", type=Path, required=True)
    parser.add_argument("--base-config", type=Path)
    args = parser.parse_args()
    path = (Path("/var/lib/hope-lifecycle/ball_physics.json") if args.role == "hdu" else
            Path.home() / ".config/hope-foxglove/ball_physics.json")
    request = json.loads(sys.stdin.read(4097))
    if request.get("op") in {"verify_ball_model", "inspect_ball_model"}:
        peer = str(ipaddress.IPv4Address(request["_peer"]))
        os.environ.update(ROS_DOMAIN_ID="232", ROS_AUTOMATIC_DISCOVERY_RANGE="SUBNET",
                          ROS_STATIC_PEERS=peer, ROS_LOCALHOST_ONLY="0")
    cache = BallModelInspectionCache(inspection_cache_path(args.role),
        lambda: inspection_identity(path, args.config_dir, args.role, base_config=args.base_config))
    print(json.dumps(handle(path, args.config_dir, args.role, request, base_config=args.base_config,
                            inspection_cache=cache)))


if __name__ == "__main__":
    main()
