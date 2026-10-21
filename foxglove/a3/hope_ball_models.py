#!/usr/bin/env python3
"""Allowlisted, restart-applied ball physics shared by Planner and packetizer."""
from __future__ import annotations

import argparse
import json
import math
import ipaddress
import os
from pathlib import Path
import sys

import yaml

from hope_field_assets import atomic_write

PROFILES = {
    "berkeley": "ball_physics_real_20261010.yaml",
    "ppq002": "ball_physics_ppq002_20261015.yaml",
    "ppq004": "ball_physics_ppq004_20261020.yaml",
}
NODES = ("hope_planner", "hope_ball_flight_packetizer")


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
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from rclpy.parameter import parameter_value_to_python
    rclpy.init(args=[])
    node = rclpy.create_node("hope_ball_model_check", start_parameter_services=False)
    try:
        client = node.create_client(GetParameters, f"/{node_name}/get_parameters")
        available = client.wait_for_service(timeout_sec=2.0)
        found = sum(name == node_name for name, _ in node.get_node_names_and_namespaces())
        if found > 1:
            raise ValueError(f"duplicate {node_name} nodes; cannot verify ball physics")
        if not available:
            executable = "hope_planner_cpp_node" if node_name == "hope_planner" else node_name
            for cmdline in Path("/proc").glob("[0-9]*/cmdline"):
                try:
                    argv = cmdline.read_bytes().split(b"\0")
                    if argv and Path(argv[0].decode()).name == executable:
                        raise ValueError(f"{node_name} exists but its parameters are unreachable")
                except (OSError, UnicodeError):
                    continue
            if found:
                raise ValueError(f"{node_name} discovered but parameters are unavailable")
            return None
        future = client.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        if not future.done() or future.result() is None:
            raise ValueError(f"{node_name} parameter read timed out")
        return dict(zip(names, map(parameter_value_to_python, future.result().values)))
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
           base_config: Path | None = None) -> dict:
    operation = request.get("op")
    if operation == "select_ball_model":
        return select(path, config_dir, request.get("model"))
    model = selected(path, config_dir)
    if operation == "read_ball_model":
        return {"model": model}
    if operation not in {"verify_ball_model", "inspect_ball_model"}:
        raise ValueError("unsupported ball model operation")
    if operation == "verify_ball_model" and "model" in request and request["model"] != model:
        raise ValueError(f"{role} configured ball model differs from selection")
    result = inspect_model(path, config_dir, role, base_config=base_config, read_live=read_live)
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
    print(json.dumps(handle(path, args.config_dir, args.role, request, base_config=args.base_config)))


if __name__ == "__main__":
    main()
