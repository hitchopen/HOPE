#!/usr/bin/env python3
"""Closed-loop MuJoCo check of the serve-pose dock.

The walking actor and the dock controller are the native Runner objects. A
delayed pelvis sample stands in for motion capture. Headless runs print a
trial report. Visual runs play at real time, and either mode can write a video.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

_MODE = "headless"
if "--mode" in sys.argv:
    _MODE = sys.argv[sys.argv.index("--mode") + 1]
if _MODE == "headless":
    os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco

sys.path.insert(0, str(Path(__file__).resolve().parent))
import validate_spin001_serve_mujoco as sim
from runner_sim_profile import apply_hope_contact_profile

REPO = Path(__file__).resolve().parents[3]
DEFAULT_POLICY = REPO / "a3_deploy/a3_deploy_example/assets/a3_runtime/teleop_humanlike"
DEFAULT_DOCK = REPO / "a3_deploy/a3_deploy_example/assets/a3_runtime/dock/dock.yaml"
DEFAULT_LIBRARY = Path("/tmp/hope_runner_transition_probe.so")
REASONS = {
    0: "none",
    1: "timeout",
    2: "mocap_stale",
    3: "mocap_invalid",
    4: "wrong_frame",
    5: "start_too_far",
    6: "attempts_exhausted",
    7: "stopped",
}
RESULTS = {0: "none", 1: "running", 2: "done", 3: "failed"}


def _load_numbers(path: Path) -> dict[str, float]:
    values: dict[str, float] = {}
    section = ""
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        if not line.startswith(" ") and line.endswith(":"):
            section = line.strip()[:-1]
            continue
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        try:
            values[f"{section}.{key.strip()}"] = float(value.strip())
        except ValueError:
            continue
    required = ("target.x", "target.y", "target.yaw", "timing.timeout_s", "timing.mocap_max_age_s")
    missing = [key for key in required if key not in values]
    if missing:
        raise RuntimeError(f"{path} is missing {', '.join(missing)}")
    return values


def _yaw(quat_wxyz: np.ndarray) -> float:
    w, x, y, z = quat_wxyz
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = np.sort(np.asarray(values, dtype=np.float64))
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return float(ordered[index])


def _ensure_library(path: Path) -> None:
    if path.is_file():
        return
    script = Path(__file__).resolve().parent / "build_runner_transition_probe.sh"
    subprocess.run(["bash", str(script), str(path)], check=True)


def _library(path: Path):
    lib = ctypes.CDLL(str(path))
    ptr = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
    lib.humanlike_create.argtypes = [ctypes.c_char_p]
    lib.humanlike_create.restype = ctypes.c_void_p
    lib.humanlike_destroy.argtypes = [ctypes.c_void_p]
    lib.humanlike_step.argtypes = [ctypes.c_void_p, ptr, ptr, ptr]
    lib.probe_create_locomotion.restype = ctypes.c_void_p
    lib.probe_destroy.argtypes = [ctypes.c_void_p]
    lib.probe_teleop_begin.argtypes = [ctypes.c_void_p, ptr, ptr]
    lib.probe_teleop_begin.restype = None
    lib.probe_teleop_step.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_double, ptr, ptr, ptr]
    lib.probe_begin.argtypes = [ctypes.c_void_p, ptr, ptr, ctypes.c_double]
    lib.probe_apply.argtypes = [ctypes.c_void_p, ctypes.c_double, ptr]
    lib.dock_create.argtypes = [ctypes.c_char_p]
    lib.dock_create.restype = ctypes.c_void_p
    lib.dock_destroy.argtypes = [ctypes.c_void_p]
    lib.dock_destroy.restype = None
    lib.dock_begin.argtypes = [ctypes.c_void_p, ctypes.c_double]
    lib.dock_begin.restype = None
    lib.dock_update.argtypes = [
        ctypes.c_void_p, ctypes.c_double, ptr, ctypes.c_int, ctypes.c_int, ctypes.c_int, ptr
    ]
    lib.dock_update.restype = None
    return lib


def _markers(scene, pelvis: np.ndarray, target: np.ndarray, yaw: float) -> None:
    if scene.ngeom + 2 > scene.maxgeom:
        return
    start = int(scene.ngeom)
    flat = np.array([target[0], target[1], pelvis[2]], dtype=np.float64)
    mujoco.mjv_connector(
        scene.geoms[start], int(mujoco.mjtGeom.mjGEOM_LINE), 0.008, pelvis, flat
    )
    scene.geoms[start].rgba[:] = (0.1, 0.85, 0.25, 1.0)
    tip = flat + np.array([math.cos(yaw), math.sin(yaw), 0.0]) * 0.45
    mujoco.mjv_connector(
        scene.geoms[start + 1], int(mujoco.mjtGeom.mjGEOM_ARROW), 0.02, flat, tip
    )
    scene.geoms[start + 1].rgba[:] = (0.95, 0.25, 0.15, 1.0)
    scene.ngeom = start + 2


def _spawn(rng: np.random.Generator, args) -> np.ndarray:
    low = np.array([args.x_range[0], args.y_range[0], math.radians(args.yaw_range_deg[0])])
    high = np.array([args.x_range[1], args.y_range[1], math.radians(args.yaw_range_deg[1])])
    if np.any(high <= low):
        raise RuntimeError("spawn ranges must be low < high")
    for _ in range(10000):
        pose = rng.uniform(low, high)
        if pose[0] < 0.0:
            return pose
    raise RuntimeError("every spawn landed on the table; widen the floor side of --x-range")


def _intersects_table(model, data) -> bool:
    table = int(model.geom("gate3_table_collision").id)
    mujoco.mj_forward(model, data)
    return any(table in (int(contact.geom1), int(contact.geom2)) for contact in data.contact[: data.ncon])


def _apply(model, data, cmd, bindings) -> None:
    qa, va, ac = np.array(bindings).T
    torque = cmd[3] * (cmd[0] - data.qpos[qa]) + cmd[4] * (cmd[1] - data.qvel[va]) + cmd[2]
    data.ctrl[ac] = np.clip(torque, model.actuator_ctrlrange[ac, 0], model.actuator_ctrlrange[ac, 1])
    for _ in range(2):
        mujoco.mj_step(model, data)


def _state(model, data, bindings, pelvis: int) -> np.ndarray:
    qa, va, _ = np.array(bindings).T
    spatial = np.zeros(6)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, pelvis, spatial, 1)
    return np.r_[data.qpos[qa], data.qvel[va], data.xquat[pelvis], spatial[:3]]


def run_trial(args, lib, model, data, bindings, stand, policy, probe, dock, rng, trial_seed, viewer, renderer, camera, video):
    target = np.array(args.target, dtype=np.float64)
    qa = np.array(bindings)[:, 0]
    initial = None
    for _ in range(10000):
        candidate = _spawn(rng, args)
        mujoco.mj_resetDataKeyframe(model, data, model.key("stand").id)
        data.qpos[0] = candidate[0]
        data.qpos[1] = candidate[1]
        half = candidate[2] / 2.0
        data.qpos[3:7] = [math.cos(half), 0.0, 0.0, math.sin(half)]
        data.qpos[qa] = stand[0]
        if not _intersects_table(model, data):
            initial = candidate
            break
    if initial is None:
        raise RuntimeError("every spawn intersected the table; widen the floor side of --x-range")
    for joint_id in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) or ""
        if name.startswith("gate3_ball_free_joint"):
            address = int(model.jnt_qposadr[joint_id])
            data.qpos[address:address + 3] = [100.0, 0.0, -10.0]
    mujoco.mj_forward(model, data)
    pelvis = model.body("pelvis_link").id
    cmd = stand.copy()
    fell = False
    max_tilt = 0.0
    min_height = float(data.qpos[2])
    dock_started = None
    finished = None
    terminal = None
    attempts = 0
    result = "running"
    reason = "none"
    history: list[tuple[float, np.ndarray]] = []
    next_sample = 0.0
    wall_origin = time.perf_counter()
    sim_origin = 0.0
    next_frame = 0.0

    def fallen() -> bool:
        tilt = float(np.arccos(np.clip(data.xmat[pelvis].reshape(3, 3)[2, 2], -1.0, 1.0)))
        nonlocal max_tilt, min_height
        max_tilt = max(max_tilt, tilt)
        min_height = min(min_height, float(data.qpos[2]))
        return tilt > 0.8 or data.qpos[2] < 0.55

    def mocap(now: float) -> np.ndarray:
        nonlocal next_sample
        measured = np.array([data.qpos[0], data.qpos[1], _yaw(data.xquat[pelvis])], dtype=np.float64)
        measured[:2] += rng.normal(0.0, args.mocap_noise_mm / 1000.0, size=2)
        if now + 1e-9 >= next_sample:
            history.append((now, measured))
            next_sample += 1.0 / args.mocap_rate_hz
        ready = now - args.mocap_latency_ms / 1000.0
        chosen = None
        for stamp, sample in history:
            if stamp <= ready:
                chosen = (stamp, sample)
            else:
                break
        packet = np.zeros(6, dtype=np.float64)
        if chosen is None:
            return packet
        packet[:3] = chosen[1]
        packet[3] = now - chosen[0]
        packet[4] = 1.0
        packet[5] = 1.0
        return packet

    def paint(now: float) -> None:
        nonlocal next_frame
        origin = np.array([data.qpos[0], data.qpos[1], data.qpos[2]], dtype=np.float64)
        goal = np.array([target[0], target[1], data.qpos[2]], dtype=np.float64)
        if viewer is not None and viewer.is_running():
            viewer.user_scn.ngeom = 0
            _markers(viewer.user_scn, origin, goal, target[2])
            viewer.sync()
            delay = (now - sim_origin) - (time.perf_counter() - wall_origin)
            if delay > 0.0:
                time.sleep(delay)
        if video is None or now + 1e-9 < next_frame:
            return
        renderer.update_scene(data, camera)
        _markers(renderer.scene, origin, goal, target[2])
        video.stdin.write(np.ascontiguousarray(renderer.render()).tobytes())
        next_frame += 1.0 / 30.0

    measured = np.array([data.qpos[0], data.qpos[1], _yaw(data.xquat[pelvis])], dtype=np.float64)
    history.append((-args.mocap_latency_ms / 1000.0, measured))
    tick = 0
    while tick < 500:
        mocap(float(data.time))
        _apply(model, data, cmd, bindings)
        paint(float(data.time))
        if fallen():
            fell = True
            break
        tick += 1
    if not fell:
        state = _state(model, data, bindings, pelvis)
        lib.probe_teleop_begin(probe, cmd, stand)
        lib.dock_begin(dock, float(data.time))
        dock_started = float(data.time)
        out = np.zeros(12, dtype=np.float64)
        velocity = np.zeros(3, dtype=np.float64)
        active = 1
        while float(data.time) - dock_started <= args.timeout + 2.5:
            now = float(data.time)
            state = _state(model, data, bindings, pelvis)
            packet = mocap(now)
            settled = 0
            if active:
                active = lib.probe_teleop_step(probe, policy, now - dock_started, state, velocity, cmd)
                if active < 0:
                    raise RuntimeError("teleop entry step failed")
            else:
                settled = lib.humanlike_step(policy, state, velocity, cmd)
                if settled < 0:
                    raise RuntimeError("walking policy rejected the state")
            quiet = int(np.max(np.abs(state[31:62])) < 0.5 and np.linalg.norm(state[66:69]) < 0.15)
            lib.dock_update(dock, now, packet, int(active > 0), int(settled == 1), quiet, out)
            attempts = int(out[4])
            result = RESULTS.get(int(out[10]), "unknown")
            reason = REASONS.get(int(out[11]), "unknown")
            velocity[:] = 0.0 if active or result in ("done", "failed") else out[:3]
            _apply(model, data, cmd, bindings)
            paint(float(data.time))
            if fallen():
                fell = True
                break
            if result in ("done", "failed") and active == 0:
                finished = float(data.time)
                terminal = out.copy()
                break
        if not fell and finished is not None:
            if lib.probe_begin(probe, cmd, stand, 0.5) != 0:
                raise RuntimeError("stand exit blend failed")
            exit_at = float(data.time)
            while float(data.time) - exit_at < 2.5:
                if float(data.time) - exit_at <= 0.5:
                    if lib.probe_apply(probe, float(data.time) - exit_at, cmd) != 0:
                        raise RuntimeError("stand exit blend apply failed")
                else:
                    cmd = stand.copy()
                _apply(model, data, cmd, bindings)
                paint(float(data.time))
                if fallen():
                    fell = True
                    break
    final_yaw = _yaw(data.xquat[pelvis])
    report = {
        "seed": trial_seed,
        "initial": {"x": float(initial[0]), "y": float(initial[1]), "yaw_deg": math.degrees(initial[2])},
        "target": {"x": float(target[0]), "y": float(target[1]), "yaw_deg": math.degrees(target[2])},
        "final": {"x": float(data.qpos[0]), "y": float(data.qpos[1]), "yaw_deg": math.degrees(final_yaw)},
        "position_error_m": float(np.hypot(data.qpos[0] - target[0], data.qpos[1] - target[1])),
        "yaw_error_deg": math.degrees(abs(_wrap(target[2] - final_yaw))),
        "docked_position_error_m": None if terminal is None else float(terminal[8]),
        "docked_yaw_error_deg": None if terminal is None else math.degrees(abs(terminal[7])),
        "time_s": None if dock_started is None or finished is None else finished - dock_started,
        "attempts": attempts,
        "fell": fell,
        "max_tilt_rad": max_tilt,
        "min_pelvis_height_m": min_height,
        "result": "fell" if fell else result,
        "reason": reason,
    }
    report["success"] = (not fell) and result == "done"
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("headless", "visual"), default="headless")
    parser.add_argument("--video", nargs="?", const="", default=None)
    parser.add_argument("--trials", type=int, default=1)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--x-range", type=float, nargs=2, default=None)
    parser.add_argument("--y-range", type=float, nargs=2, default=None)
    parser.add_argument("--yaw-range-deg", type=float, nargs=2, default=(-180.0, 180.0))
    parser.add_argument("--target", type=float, nargs=3, default=None, metavar=("X", "Y", "YAW_DEG"))
    parser.add_argument("--dock-config", type=Path, default=DEFAULT_DOCK)
    parser.add_argument("--mocap-latency-ms", type=float, default=10.0)
    parser.add_argument("--mocap-noise-mm", type=float, default=1.0)
    parser.add_argument("--mocap-rate-hz", type=float, default=120.0)
    parser.add_argument("--timeout", type=float, default=None)
    parser.add_argument("--library", type=Path, default=DEFAULT_LIBRARY)
    parser.add_argument("--policy-dir", type=Path, default=DEFAULT_POLICY)
    parser.add_argument("--json-out", type=Path, default=None)
    parser.add_argument("--min-success", type=float, default=0.95)
    args = parser.parse_args()
    if args.trials < 1 or args.mocap_rate_hz <= 0.0 or args.mocap_latency_ms < 0.0:
        raise RuntimeError("trials, mocap rate and latency must be positive")
    config = _load_numbers(args.dock_config)
    if args.target is None:
        args.target = [config["target.x"], config["target.y"], config["target.yaw"]]
    else:
        args.target = [args.target[0], args.target[1], math.radians(args.target[2])]
    if args.x_range is None:
        args.x_range = [args.target[0] - 2.0, args.target[0] + 2.0]
    if args.y_range is None:
        args.y_range = [args.target[1] - 2.0, args.target[1] + 2.0]
    if args.timeout is None:
        args.timeout = config["timing.timeout_s"]
    _ensure_library(args.library)
    lib = _library(args.library.resolve())
    model, bindings = sim._compile_model(sim.DEFAULT_MODEL)
    apply_hope_contact_profile(model)
    stand_q, kp, kd = sim._official_stand_vectors()
    stand = np.array([stand_q, np.zeros(31), np.zeros(31), kp, kd], dtype=np.float64)
    policy = lib.humanlike_create(str(args.policy_dir.resolve()).encode())
    probe = lib.probe_create_locomotion()
    dock = lib.dock_create(str(args.dock_config.resolve()).encode())
    if not policy or not probe or not dock:
        raise RuntimeError(
            "native dock controller failed to load "
            f"(policy={bool(policy)} probe={bool(probe)} dock={bool(dock)})"
        )
    video = None
    renderer = None
    camera = None
    if args.video is not None:
        path = Path(args.video) if args.video else REPO / "out" / f"auto_dock_{args.seed}.mp4"
        path.parent.mkdir(parents=True, exist_ok=True)
        model.vis.global_.offwidth = 1280
        model.vis.global_.offheight = 720
        renderer = mujoco.Renderer(model, height=720, width=1280)
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [args.target[0], args.target[1], 0.8]
        camera.distance = 8.0
        camera.azimuth = 140.0
        camera.elevation = -25.0
        video = subprocess.Popen(
            ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
             "-s", "1280x720", "-r", "30", "-i", "-", "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p",
             str(path)],
            stdin=subprocess.PIPE,
        )
    data = mujoco.MjData(model)
    viewer = None
    reports = []
    try:
        if args.mode == "visual":
            viewer = mujoco.viewer.launch_passive(model, data)
        rng = np.random.default_rng(args.seed)
        for index in range(args.trials):
            if viewer is not None and not viewer.is_running():
                break
            trial_seed = int(rng.integers(0, 2**31 - 1))
            report = run_trial(
                args, lib, model, data, bindings, stand, policy, probe, dock,
                np.random.default_rng(trial_seed), trial_seed, viewer, renderer, camera, video,
            )
            reports.append(report)
            print(json.dumps(report), flush=True)
    finally:
        if viewer is not None:
            viewer.close()
        if video is not None:
            video.stdin.close()
            video.wait()
        if renderer is not None:
            renderer.close()
        lib.dock_destroy(dock)
        lib.probe_destroy(probe)
        lib.humanlike_destroy(policy)
    times = [item["time_s"] for item in reports if item["success"] and item["time_s"] is not None]
    summary = {
        "count": len(reports),
        "successes": sum(item["success"] for item in reports),
        "falls": sum(item["fell"] for item in reports),
        "success_rate": 0.0 if not reports else sum(item["success"] for item in reports) / len(reports),
        "time_s_p50": _percentile(times, 0.50),
        "time_s_p95": _percentile(times, 0.95),
        "final_position_error_m_p95": _percentile(
            [item["position_error_m"] for item in reports if item["success"]], 0.95
        ),
        "final_yaw_error_deg_p95": _percentile(
            [item["yaw_error_deg"] for item in reports if item["success"]], 0.95
        ),
        "docked_position_error_m_p95": _percentile(
            [item["docked_position_error_m"] for item in reports if item["success"] and item["docked_position_error_m"] is not None],
            0.95,
        ),
        "docked_yaw_error_deg_p95": _percentile(
            [item["docked_yaw_error_deg"] for item in reports if item["success"] and item["docked_yaw_error_deg"] is not None],
            0.95,
        ),
    }
    payload = {"trials": reports, "summary": summary}
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary), flush=True)
    if not reports or summary["falls"] or summary["success_rate"] < args.min_success:
        return 1
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
