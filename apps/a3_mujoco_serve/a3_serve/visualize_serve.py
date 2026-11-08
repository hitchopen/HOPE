import argparse
import csv
import json
import math
import shutil
import struct
import subprocess
import time
import zlib
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from a3_serve.config import default_config_path, load_config
from a3_serve.constants import JOINT_NAMES
from a3_serve.csvio import MotionCsv
from a3_serve.mujoco_scene import A3ServeScene

# The model ships a second court. The serve scene adds the table the motion was
# solved against; hide the model court so the ball bounces on that one.
_MODEL_COURT_GEOMS = (
    "gate3_table_visual",
    "gate3_table_collision",
    "gate3_net_collision",
)
# Legs and pelvis only. The arms are allowed to reach over the table.
_STANCE_TOKENS = ("pelvis", "waist", "hip", "knee", "ankle", "foot")
# Drop the validated SDK serve was adjusted to (assets/validated/serve_policy.csv).
_IMPACT_VELOCITY_WORLD = (0.0, 0.0, -4.0)
# Runner clips leave the ball on the palmar side of the hand. +Z is that
# site's outward normal; 24 mm clears the palm proxy the same way the serve
# stability check does.
_PALM_SITE = "left_palm"
_PALM_RELEASE_OFFSET_M = 0.024
# HOPE placement: the pelvis stands this far behind the near table edge.
_ROOT_TO_TABLE_M = 0.5
# The tested serve meets the paddle 60 ms after NOMINAL_CONTACT. Aiming at the
# face that points nearest the center line instead sends a negative yaw into the
# follow-through, where that face has turned back toward the center.
_CONTACT_FACE_DELAY_S = 0.06
# Meet the ball this far on the incoming side of the face so the swing collects it.
_TOSS_LEAD_M = 0.04
# Three-quarter view: behind the robot and off to one side, aimed between
# the stance and the middle of the table.
_CAMERA_LOOKAT = (1.15, 0.0, 0.85)
_CAMERA_DISTANCE = 5.2
_CAMERA_AZIMUTH = 50.0
_CAMERA_ELEVATION = -22.0
# The free camera uses the model's vertical field of view. Keep 60% of that
# angle so the serve fills more of the frame without moving the camera.
_CAMERA_FOV_SCALE = 0.6
# Real-time playback. The model framebuffer is 640x480; the video raises it
# in memory so the file can be 720p.
_VIDEO_WIDTH = 1280
_VIDEO_HEIGHT = 720
_VIDEO_FPS = 30
_ERROR_NET = "net hit"
_ERROR_TWO_OURS = "two bounces in our side"
_ERROR_TWO_OPPONENT = "two bounces on opponents side"
_ERROR_NO_OURS = "no bounce in our side"
_ERROR_NO_OPPONENT = "no bounce in the opponent's side"
_ERROR_SIDE = (
    "ball flew over the side limits of the table instead of the player sides"
)


def _geom_id(model: mujoco.MjModel, name: str) -> int:
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if geom_id < 0:
        raise ValueError(f"required MuJoCo geom not found: {name}")
    return int(geom_id)


def hide_model_court(scene: A3ServeScene) -> None:
    """Hide the model court so only the serve-scene table and net remain."""

    model = scene.model
    for name in _MODEL_COURT_GEOMS:
        geom_id = _geom_id(model, name)
        model.geom_rgba[geom_id, 3] = 0.0
        model.geom_contype[geom_id] = 0
        model.geom_conaffinity[geom_id] = 0


def _stance_forward_and_sole(scene: A3ServeScene, table_top_z: float) -> tuple[float, float]:
    """Forward-most and lowest point of the legs and pelvis at the current pose."""

    model, data = scene.model, scene.data
    max_x = -1.0e9
    min_z = 1.0e9
    for geom_id in range(model.ngeom):
        if (int(model.geom_contype[geom_id]) & 1) == 0:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        if not any(token in name for token in _STANCE_TOKENS):
            continue
        center = model.geom_aabb[geom_id, :3]
        half = model.geom_aabb[geom_id, 3:]
        if not np.any(half > 0.0):
            continue
        rotation = data.geom_xmat[geom_id].reshape(3, 3)
        origin = data.geom_xpos[geom_id]
        for corner in (
            center + half * np.array([sx, sy, sz], dtype=np.float64)
            for sx in (-1.0, 1.0)
            for sy in (-1.0, 1.0)
            for sz in (-1.0, 1.0)
        ):
            world = origin + rotation @ corner
            min_z = min(min_z, float(world[2]))
            if world[2] <= table_top_z + 0.02:
                max_x = max(max_x, float(world[0]))
    if max_x < -1.0e8 or min_z > 1.0e8:
        raise RuntimeError("could not measure the robot stance against the table")
    return max_x, min_z


def plant_behind_table(
    scene: A3ServeScene,
    trajectory: np.ndarray,
    root_to_table_m: float | None = None,
) -> np.ndarray:
    """Set the feet down, and optionally place the pelvis behind the near edge.

    ``reset`` restores the keyframe, so the returned 7-vector is reapplied every
    frame. ``root_to_table_m`` is the pelvis-to-near-edge gap. Leave it unset for
    an SDK serve: that clip was solved at the keyframe pelvis, and shifting it
    moves the paddle off the aimed drop.
    """

    model = scene.model
    table_id = _geom_id(model, "gate3_table_collision")
    table_top_z = float(model.geom_pos[table_id, 2] + model.geom_size[table_id, 2])
    min_z = 1.0e9
    stride = max(1, len(trajectory) // 40)
    frames = list(range(0, len(trajectory), stride))
    frames.append(len(trajectory) - 1)
    for frame in frames:
        scene.reset(trajectory[frame])
        _, sole = _stance_forward_and_sole(scene, table_top_z)
        min_z = min(min_z, sole)
    scene.reset(trajectory[0])
    base = scene.data.qpos[scene.base_qpos_address:scene.base_qpos_address + 7].copy()
    # Drop a stance that is clear of the floor. Do not lift one that already touches.
    base[2] += min(0.0, -min_z)
    if root_to_table_m is not None:
        base[0] = scene.near_edge_x - float(root_to_table_m)
    return base


def _pose_racket(
    scene: A3ServeScene, joints: np.ndarray, base: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    scene.reset(joints)
    address = scene.base_qpos_address
    scene.data.qpos[address:address + 7] = base
    return scene.racket_pose()


def _release_state(
    contact: np.ndarray,
    flight_s: float,
    gravity: float,
    impact_velocity: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Back-integrate a drop so the ball is at ``contact`` with ``impact_velocity``."""

    acceleration = np.array([0.0, 0.0, -gravity])
    impact = np.asarray(impact_velocity, dtype=np.float64)
    velocity = impact - acceleration * flight_s
    position = contact - impact * flight_s + 0.5 * acceleration * flight_s * flight_s
    return position, velocity


def _csv_header(path: Path) -> list[str]:
    with path.open(newline="", encoding="utf-8") as stream:
        try:
            return next(csv.reader(stream))
        except StopIteration as exc:
            raise ValueError(f"CSV is empty: {path}") from exc


def _event_time(rows: list[dict[str, str]], times: np.ndarray, event: str) -> float | None:
    hits = [index for index, row in enumerate(rows) if row["event"] == event]
    if len(hits) != 1:
        return None
    return float(times[hits[0]])


def _load_runtime_timeline(path: Path) -> tuple[np.ndarray, float, int, float, float]:
    """Load a Runner serve timeline: FRAME rows, radians, contact, and toss."""

    with path.open(newline="", encoding="utf-8") as stream:
        rows = [row for row in csv.DictReader(stream) if row.get("row_kind") == "FRAME"]
    if not rows:
        raise ValueError(f"runtime CSV has no FRAME rows: {path}")
    missing = [name for name in JOINT_NAMES if f"q_rad::{name}" not in rows[0]]
    if missing or "time_s" not in rows[0] or "event" not in rows[0]:
        raise ValueError(f"runtime CSV is missing joint, time, or event columns: {path}")
    joints = np.asarray(
        [[float(row[f"q_rad::{name}"]) for name in JOINT_NAMES] for row in rows],
        dtype=np.float64,
    )
    times = np.asarray([float(row["time_s"]) for row in rows], dtype=np.float64)
    if joints.shape[0] < 2 or not np.all(np.isfinite(joints)) or not np.all(np.isfinite(times)):
        raise ValueError(f"runtime CSV needs at least two finite frames: {path}")
    if not np.all(np.diff(times) > 0.0):
        raise ValueError(f"runtime CSV frame times must increase: {path}")
    contacts = [index for index, row in enumerate(rows) if row["event"] == "NOMINAL_CONTACT"]
    if len(contacts) != 1:
        raise ValueError(f"runtime CSV needs exactly one NOMINAL_CONTACT row: {path}")
    release_time = _event_time(rows, times, "DETACH_TARGET")
    if release_time is None:
        release_time = _event_time(rows, times, "RELEASE")
    if release_time is None:
        raise ValueError(f"runtime CSV needs a DETACH_TARGET or RELEASE row: {path}")
    hz = 1.0 / float(np.median(np.diff(times)))
    return joints, hz, contacts[0], float(times[-1] - times[0]), release_time


def load_playback(csv_path: Path, config: dict) -> tuple[np.ndarray, float, int, float, float | None]:
    """Return joints, sample rate, strike index, clip length, and toss time.

    SDK exports are degrees and use the serve-config window. Their toss time is
    ``None``: the ball is the fixed drop aimed at the strike. Runner timelines
    (``sequence_index`` / ``q_rad::``) are already in radians, play in full, and
    release the ball from the left palm at ``DETACH_TARGET``.
    """

    header = _csv_header(csv_path)
    if header[:1] == ["sequence_index"]:
        return _load_runtime_timeline(csv_path)
    joints = MotionCsv.load(csv_path).joint_radians()
    hz = float(config["timing"]["source_hz"])
    start = int(config["timing"]["stroke_start_frame"])
    duration_s = 0.5 * float(config["planner"]["flight_duration_s"])
    played = int(math.ceil(duration_s * hz))
    trajectory = joints[start:start + played]
    if len(trajectory) == 0:
        raise ValueError(f"serve-config window is outside {csv_path}")
    strike = int(config["timing"]["strike_frame"]) - start
    return trajectory, hz, strike, duration_s, None


def _write_png(path: Path, image: np.ndarray) -> None:
    height, width, _ = image.shape
    raw = b"".join(b"\x00" + image[row].tobytes() for row in range(height))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data))
            + tag
            + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
        )

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw, 6))
        + chunk(b"IEND", b"")
    )


def _paint_disk(image: np.ndarray, x: int, y: int, radius: int, color: tuple[int, int, int]) -> None:
    height, width, _ = image.shape
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius:
                continue
            px = x + dx
            py = y + dy
            if 0 <= px < width and 0 <= py < height:
                image[py, px] = color


def _paint_segment(
    image: np.ndarray,
    start: tuple[int, int],
    end: tuple[int, int],
    radius: int,
    color: tuple[int, int, int],
) -> None:
    x0, y0 = start
    x1, y1 = end
    steps = max(abs(x1 - x0), abs(y1 - y0), 1)
    for step in range(steps + 1):
        fraction = step / steps
        _paint_disk(
            image,
            int(round(x0 + fraction * (x1 - x0))),
            int(round(y0 + fraction * (y1 - y0))),
            radius,
            color,
        )


def save_vertical_projection(
    scene: A3ServeScene,
    path_xy: np.ndarray,
    csv_path: Path,
) -> Path:
    """Save the ball path projected straight down onto the table plane.

    ``+x`` points toward the far side and ``+y`` points to the left. The table,
    the net, and the center line are drawn under the path.
    """

    width_px, height_px, margin = 960, 640, 36
    image = np.full((height_px, width_px, 3), 236, dtype=np.uint8)
    table_x0 = float(scene.near_edge_x)
    table_x1 = table_x0 + float(scene.length)
    table_y0 = -float(scene.width) / 2.0
    table_y1 = float(scene.width) / 2.0
    xs = path_xy[:, 0]
    ys = path_xy[:, 1]
    xmin = min(table_x0, float(xs.min())) - 0.35
    xmax = max(table_x1, float(xs.max())) + 0.35
    ymin = min(table_y0, float(ys.min())) - 0.35
    ymax = max(table_y1, float(ys.max())) + 0.35
    if xmax - xmin > 6.0:
        xmin, xmax = table_x0 - 1.0, table_x1 + 1.0
    if ymax - ymin > 4.0:
        ymin, ymax = table_y0 - 0.8, table_y1 + 0.8

    def to_pixel(x: float, y: float) -> tuple[int, int]:
        px = margin + (x - xmin) / (xmax - xmin) * (width_px - 2 * margin)
        py = margin + (ymax - y) / (ymax - ymin) * (height_px - 2 * margin)
        return int(round(px)), int(round(py))

    left, top = to_pixel(table_x0, table_y1)
    right, bottom = to_pixel(table_x1, table_y0)
    image[top:bottom, left:right] = (186, 214, 232)
    net_x = table_x0 + float(scene.net_x_table)
    _paint_segment(image, to_pixel(net_x, table_y0), to_pixel(net_x, table_y1), 1, (40, 40, 40))
    center_y = 0.0
    for step in range(0, 28, 2):
        x0 = table_x0 + (table_x1 - table_x0) * step / 28.0
        x1 = table_x0 + (table_x1 - table_x0) * (step + 1) / 28.0
        _paint_segment(image, to_pixel(x0, center_y), to_pixel(x1, center_y), 1, (255, 255, 255))
    pixels = [to_pixel(float(point[0]), float(point[1])) for point in path_xy]
    for start, end in zip(pixels, pixels[1:]):
        _paint_segment(image, start, end, 2, (214, 92, 16))
    _paint_disk(image, *pixels[0], 5, (32, 140, 64))
    _paint_disk(image, *pixels[-1], 5, (176, 32, 32))

    directory = Path(__file__).resolve().parents[3] / "out"
    output = directory / f"serve_{csv_path.stem}_vertical.png"
    _write_png(output, image)
    return output


def save_side_projection(
    scene: A3ServeScene,
    path_xyz: np.ndarray,
    csv_path: Path,
) -> Path:
    """Save the ball path projected onto the side plane.

    ``+x`` points toward the far side and ``+z`` points up. One meter is the
    same length on both axes. The table top and the net are drawn under the path.
    """

    width_px, height_px, margin = 960, 640, 36
    image = np.full((height_px, width_px, 3), 236, dtype=np.uint8)
    table_x0 = float(scene.near_edge_x)
    table_x1 = table_x0 + float(scene.length)
    table_z = float(scene.table_height)
    net_top = table_z + float(scene.net_height)
    xs = path_xyz[:, 0]
    zs = path_xyz[:, 2]
    xmin = min(table_x0, float(xs.min())) - 0.35
    xmax = max(table_x1, float(xs.max())) + 0.35
    zmin = min(0.0, float(zs.min())) - 0.15
    zmax = max(net_top, float(zs.max())) + 0.35
    if xmax - xmin > 6.0:
        xmin, xmax = table_x0 - 1.0, table_x1 + 1.0
    if zmax - zmin > 4.0:
        zmin, zmax = -0.2, max(net_top, float(zs.max())) + 0.4

    draw_w = width_px - 2 * margin
    draw_h = height_px - 2 * margin
    scale = min(draw_w / (xmax - xmin), draw_h / (zmax - zmin))
    used_w = (xmax - xmin) * scale
    used_h = (zmax - zmin) * scale
    origin_x = margin + (draw_w - used_w) / 2.0
    origin_y = margin + (draw_h - used_h) / 2.0

    def to_pixel(x: float, z: float) -> tuple[int, int]:
        px = origin_x + (x - xmin) * scale
        py = origin_y + (zmax - z) * scale
        return int(round(px)), int(round(py))

    _paint_segment(image, to_pixel(xmin, 0.0), to_pixel(xmax, 0.0), 1, (180, 180, 180))
    _paint_segment(
        image, to_pixel(table_x0, table_z), to_pixel(table_x1, table_z), 3, (90, 150, 184)
    )
    net_x = table_x0 + float(scene.net_x_table)
    _paint_segment(image, to_pixel(net_x, table_z), to_pixel(net_x, net_top), 2, (40, 40, 40))
    pixels = [to_pixel(float(point[0]), float(point[2])) for point in path_xyz]
    for start, end in zip(pixels, pixels[1:]):
        _paint_segment(image, start, end, 2, (214, 92, 16))
    _paint_disk(image, *pixels[0], 5, (32, 140, 64))
    _paint_disk(image, *pixels[-1], 5, (176, 32, 32))

    output = Path(__file__).resolve().parents[3] / "out" / f"serve_{csv_path.stem}_side.png"
    _write_png(output, image)
    return output


def _serve_camera() -> mujoco.MjvCamera:
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = _CAMERA_LOOKAT
    camera.distance = _CAMERA_DISTANCE
    camera.azimuth = _CAMERA_AZIMUTH
    camera.elevation = _CAMERA_ELEVATION
    return camera


class _SimulationVideo:
    """Encode RGB frames to an mp4 through ffmpeg."""

    def __init__(self, path: Path, width: int, height: int, fps: int) -> None:
        if shutil.which("ffmpeg") is None:
            raise RuntimeError("ffmpeg is required to save a simulation video")
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self._frames = 0
        self._proc: subprocess.Popen[bytes] | None = subprocess.Popen(
            [
                "ffmpeg",
                "-y",
                "-loglevel",
                "error",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "rgb24",
                "-s",
                f"{width}x{height}",
                "-r",
                str(fps),
                "-i",
                "-",
                "-an",
                "-c:v",
                "libx264",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(path),
            ],
            stdin=subprocess.PIPE,
        )

    def write(self, frame: np.ndarray) -> None:
        proc = self._proc
        if proc is None or proc.stdin is None:
            return
        proc.stdin.write(np.ascontiguousarray(frame).tobytes())
        self._frames += 1

    def close(self) -> Path | None:
        proc = self._proc
        if proc is None:
            return self.path if self._frames else None
        self._proc = None
        if proc.stdin is not None:
            proc.stdin.close()
        code = proc.wait()
        if self._frames == 0 or code != 0:
            self.path.unlink(missing_ok=True)
            if code != 0:
                raise RuntimeError(f"ffmpeg exited with status {code}")
            return None
        return self.path


def _ball_on_floor(scene: A3ServeScene) -> bool:
    """True when the simulated ball is touching the floor."""

    ball = scene.ball_geom_id
    floor = scene.floor_geom_id
    for index in range(scene.data.ncon):
        contact = scene.data.contact[index]
        if {int(contact.geom1), int(contact.geom2)} == {ball, floor}:
            return True
    return False


def _table_top_and_net(scene: A3ServeScene) -> tuple[bool, bool]:
    """Table-top and net contacts. The table's vertical faces are not bounces."""

    ball = scene.ball_geom_id
    table = scene.table_geom_id
    net = scene.net_geom_id
    top = float(scene.table_height) - 0.005
    table_hit = False
    net_hit = False
    for index in range(scene.data.ncon):
        contact = scene.data.contact[index]
        pair = {int(contact.geom1), int(contact.geom2)}
        if pair == {ball, net}:
            net_hit = True
        elif pair == {ball, table} and float(contact.pos[2]) >= top:
            table_hit = True
    return table_hit, net_hit


def _table_rect(scene: A3ServeScene) -> tuple[float, float, float, float]:
    x0 = float(scene.near_edge_x)
    half = float(scene.width) / 2.0
    return x0, x0 + float(scene.length), -half, half


def _over_table(point: np.ndarray, rect: tuple[float, float, float, float]) -> bool:
    x0, x1, y0, y1 = rect
    return x0 <= float(point[0]) <= x1 and y0 <= float(point[1]) <= y1


def _segment_hit(
    start: np.ndarray,
    end: np.ndarray,
    rect: tuple[float, float, float, float],
) -> tuple[float, str] | None:
    """Smallest t in (0, 1] where the segment meets a table edge, and which edge."""

    x0, x1, y0, y1 = rect
    dx = float(end[0] - start[0])
    dy = float(end[1] - start[1])
    hits: list[tuple[float, str]] = []
    if dx != 0.0:
        for edge, kind in ((x0, "near"), (x1, "far")):
            t = (edge - float(start[0])) / dx
            if 0.0 < t <= 1.0:
                y = float(start[1]) + t * dy
                if y0 - 1.0e-9 <= y <= y1 + 1.0e-9:
                    hits.append((t, kind))
    if dy != 0.0:
        for edge in (y0, y1):
            t = (edge - float(start[1])) / dy
            if 0.0 < t <= 1.0:
                x = float(start[0]) + t * dx
                if x0 - 1.0e-9 <= x <= x1 + 1.0e-9:
                    hits.append((t, "side"))
    if not hits:
        return None
    return min(hits)


def _interpolate(
    samples: list[tuple[float, np.ndarray]],
    index: int,
    fraction: float,
) -> tuple[float, np.ndarray]:
    time0, start = samples[index - 1]
    time1, end = samples[index]
    return time0 + fraction * (time1 - time0), start + fraction * (end - start)


def _leave_table(
    samples: list[tuple[float, np.ndarray]],
    rect: tuple[float, float, float, float],
) -> tuple[float, np.ndarray, str] | None:
    """First time the ball center's vertical projection leaves the table top."""

    was_inside = _over_table(samples[0][1], rect)
    for index in range(1, len(samples)):
        inside = _over_table(samples[index][1], rect)
        if was_inside and not inside:
            hit = _segment_hit(samples[index - 1][1], samples[index][1], rect)
            if hit is None:
                return samples[index][0], samples[index][1].copy(), "side"
            fraction, kind = hit
            time_s, position = _interpolate(samples, index, fraction)
            return time_s, position, kind
        was_inside = inside
    return None


def _pass_beside_table(
    samples: list[tuple[float, np.ndarray]],
    rect: tuple[float, float, float, float],
) -> tuple[float, np.ndarray] | None:
    """First point that passes the near edge while already outside a sideline."""

    x0, _, y0, y1 = rect
    for index in range(1, len(samples)):
        start = samples[index - 1][1]
        end = samples[index][1]
        if float(start[0]) < x0 <= float(end[0]) and float(end[0]) != float(start[0]):
            fraction = (x0 - float(start[0])) / float(end[0] - start[0])
            y = float(start[1]) + fraction * float(end[1] - start[1])
            if y < y0 or y > y1:
                return _interpolate(samples, index, fraction)
    return None


def _cross_net(
    samples: list[tuple[float, np.ndarray]],
    net_x: float,
) -> tuple[float, np.ndarray] | None:
    for index in range(1, len(samples)):
        start = samples[index - 1][1]
        end = samples[index][1]
        if float(start[0]) < net_x <= float(end[0]) and float(end[0]) != float(start[0]):
            fraction = (net_x - float(start[0])) / float(end[0] - start[0])
            return _interpolate(samples, index, fraction)
    return None


def _round_time(value: float) -> float:
    return round(float(value), 6)


def _round_position(position: np.ndarray) -> list[float]:
    return [round(float(position[0]), 6), round(float(position[1]), 6), round(float(position[2]), 6)]


def _serve_report(
    scene: A3ServeScene,
    samples: list[tuple[float, np.ndarray]],
    bounces: list[dict],
    errors: list[dict],
    ended_on_floor: bool,
) -> dict:
    """Faults, bounces, table exit, and the ball path from one serve."""

    rect = _table_rect(scene)
    net_x = float(scene.near_edge_x + scene.net_x_table)
    leave = _leave_table(samples, rect)
    our_count = sum(bounce["side"] == "our" for bounce in bounces)
    opponent_count = sum(bounce["side"] == "opponent" for bounce in bounces)
    last_time, last_position = samples[-1]

    def opportunity_passed(side: str) -> bool:
        count = our_count if side == "our" else opponent_count
        if count:
            return False
        if ended_on_floor or leave is not None:
            return True
        if float(last_position[2]) < float(scene.table_height):
            return True
        if side == "our" and any(float(point[0]) >= net_x for _, point in samples):
            return True
        return False

    def absence_moment(side: str) -> tuple[float, np.ndarray]:
        moments: list[tuple[float, np.ndarray]] = [(last_time, last_position)]
        if leave is not None:
            moments.append((leave[0], leave[1]))
        if side == "our":
            crossing = _cross_net(samples, net_x)
            if crossing is not None:
                moments.append(crossing)
        return min(moments, key=lambda item: item[0])

    if our_count == 0 and opportunity_passed("our"):
        time_s, position = absence_moment("our")
        errors.append({"time": time_s, "position": position, "type": _ERROR_NO_OURS})
    if opponent_count == 0 and opportunity_passed("opponent"):
        time_s, position = absence_moment("opponent")
        errors.append({"time": time_s, "position": position, "type": _ERROR_NO_OPPONENT})
    if leave is not None and leave[2] == "side":
        errors.append({"time": leave[0], "position": leave[1], "type": _ERROR_SIDE})
    elif leave is None:
        beside = _pass_beside_table(samples, rect)
        if beside is not None:
            errors.append({"time": beside[0], "position": beside[1], "type": _ERROR_SIDE})

    errors.sort(key=lambda item: float(item["time"]))
    table_exit = None
    if leave is not None:
        table_exit = {
            "time": _round_time(leave[0]),
            "position": _round_position(leave[1]),
        }
    return {
        "errors": [
            {
                "time": _round_time(error["time"]),
                "position": _round_position(error["position"]),
                "type": error["type"],
            }
            for error in errors
        ],
        "bounces": [
            {
                "time": _round_time(bounce["time"]),
                "position": _round_position(bounce["position"]),
            }
            for bounce in bounces
        ],
        "table_exit": table_exit,
        "trajectory": [
            {"time": _round_time(time_s), "position": _round_position(position)}
            for time_s, position in samples
        ],
    }


def _print_errors(errors: list[dict]) -> None:
    if not errors:
        print("no errors")
        return
    for error in errors:
        position = ", ".join(f"{value:.3f}" for value in error["position"])
        print(f"{error['time']:.3f}s  {error['type']}  [{position}]")


def save_serve_report(report: dict, csv_path: Path) -> Path:
    output = Path(__file__).resolve().parents[3] / "out" / f"serve_{csv_path.stem}.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=4) + "\n")
    return output


def hold_ball_on_palm(scene: A3ServeScene, palm_site_id: int) -> None:
    """Park the ball on the palmar face and clear any velocity from the last step."""

    scene.mj.mj_forward(scene.model, scene.data)
    rotation = scene.data.site_xmat[palm_site_id].reshape(3, 3)
    origin = scene.data.site_xpos[palm_site_id] + _PALM_RELEASE_OFFSET_M * rotation[:, 2]
    scene._set_ball(origin, (0.0, 0.0, 0.0))


def _table_facing_normal(rotation: np.ndarray, axis: int) -> np.ndarray:
    """Face normal with its horizontal part pointing toward the far side (+x)."""

    normal = np.array(rotation[:, axis], dtype=np.float64)
    if normal[0] < 0.0:
        normal *= -1.0
    return normal


def palm_release_velocity(
    scene: A3ServeScene,
    trajectory: np.ndarray,
    base: np.ndarray,
    hz: float,
    release_time: float,
    contact_index: int,
    palm_site_id: int,
) -> np.ndarray:
    """Ballistic velocity from the palm to the hitting face.

    The hand in these clips does not move, so a zero-velocity release falls
    short of the paddle. The toss is aimed at the face a fixed moment after
    ``NOMINAL_CONTACT``, the same moment for a centered swing and for either
    yaw. Choosing the face that points nearest the center line meets a negative
    yaw on the follow-through, after that face has turned back to center.
    """

    address = scene.base_qpos_address
    release_frame = min(int(round(release_time * hz)), len(trajectory) - 1)
    scene.reset(trajectory[release_frame])
    scene.data.qpos[address:address + 7] = base
    scene.mj.mj_forward(scene.model, scene.data)
    palm_rotation = scene.data.site_xmat[palm_site_id].reshape(3, 3)
    origin = (
        scene.data.site_xpos[palm_site_id] + _PALM_RELEASE_OFFSET_M * palm_rotation[:, 2]
    ).copy()
    frame = int(np.clip(contact_index + round(_CONTACT_FACE_DELAY_S * hz), 0, len(trajectory) - 1))
    flight = frame / hz - release_time
    if flight <= 0.0:
        raise RuntimeError("nominal contact is not after the palm release")
    scene.reset(trajectory[frame])
    scene.data.qpos[address:address + 7] = base
    scene.mj.mj_forward(scene.model, scene.data)
    position = scene.data.site_xpos[scene.racket_site_id].copy()
    normal = _table_facing_normal(
        scene.data.site_xmat[scene.racket_site_id].reshape(3, 3),
        scene.racket_normal_axis,
    )
    meet = position - normal * (scene.ball_radius + _TOSS_LEAD_M)
    return (meet - origin) / flight - 0.5 * scene.model.opt.gravity * flight


def ball_into_paddle(
    scene: A3ServeScene,
    trajectory: np.ndarray,
    base: np.ndarray,
    config: dict,
    hz: float,
    strike_index: int,
) -> tuple[list[float], list[float]]:
    """Drop the ball onto the paddle face at the strike.

    The face normal is the racket-site axis that points along the swing. The
    incoming speed is fixed from the validated reference serve, and the release
    position is set so the ball is on the face when the paddle arrives.
    """

    strike_index = min(max(int(strike_index), 0), len(trajectory) - 1)
    before = max(strike_index - 1, 0)
    after = min(strike_index + 1, len(trajectory) - 1)
    contact_position, rotation = _pose_racket(scene, trajectory[strike_index], base)
    previous, _ = _pose_racket(scene, trajectory[before], base)
    following, _ = _pose_racket(scene, trajectory[after], base)
    racket_velocity = (following - previous) / max((after - before) / hz, 1.0 / hz)
    normal = np.array(rotation[:, scene.racket_normal_axis], dtype=np.float64)
    if float(np.dot(racket_velocity, normal)) < 0.0:
        normal *= -1.0
    gap = scene.ball_radius + float(config["planner"]["racket_contact_gap_m"])
    contact = contact_position + normal * gap
    flight = strike_index / hz
    position, velocity = _release_state(
        contact,
        flight,
        float(config["physics"]["gravity"]),
        np.asarray(_IMPACT_VELOCITY_WORLD, dtype=np.float64),
    )
    return (
        [float(value) for value in position],
        [float(value) for value in velocity],
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Replay an A3 serve CSV and save the vertical ball-path image.",
    )
    parser.add_argument(
        "csv",
        type=Path,
        help="A3 SDK serve CSV or Runner timeline CSV to replay",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=default_config_path(),
        help="serve JSON (default: config/serve.json next to this package)",
    )
    parser.add_argument(
        "--visual",
        action="store_true",
        help="Open the MuJoCo viewer and keep replaying until the window closes. The default is one headless run.",
    )
    parser.add_argument(
        "--video",
        action="store_true",
        help="Save the first serve as a real-time mp4 next to the path images. Requires ffmpeg.",
    )
    args = parser.parse_args()

    csv_path = args.csv.expanduser().resolve()
    config = load_config(args.config)
    trajectory, hz, strike_index, duration_s, release_time = load_playback(csv_path, config)

    scene = A3ServeScene(config["model"]["xml"], config["physics"], config["model"])
    scene.model.vis.global_.fovy *= _CAMERA_FOV_SCALE
    hide_model_court(scene)
    base = plant_behind_table(
        scene,
        trajectory,
        root_to_table_m=None if release_time is None else _ROOT_TO_TABLE_M,
    )
    palm_site_id = -1
    ball_position: list[float] | None = None
    ball_velocity: list[float] | None = None
    if release_time is None:
        ball_position, ball_velocity = ball_into_paddle(
            scene, trajectory, base, config, hz, strike_index
        )
    else:
        palm_site_id = mujoco.mj_name2id(scene.model, mujoco.mjtObj.mjOBJ_SITE, _PALM_SITE)
        if palm_site_id < 0:
            raise ValueError(f"required MuJoCo site not found: {_PALM_SITE}")
    toss = (
        None
        if release_time is None
        else palm_release_velocity(
            scene, trajectory, base, hz, release_time, strike_index, palm_site_id
        )
    )
    dt = float(scene.model.opt.timestep)
    steps = int(math.ceil(duration_s / dt))
    tossed = False

    def apply_base() -> None:
        address = scene.base_qpos_address
        scene.data.qpos[address:address + 7] = base

    def place_ball(source_time: float) -> None:
        nonlocal tossed
        if release_time is None or toss is None:
            return
        if source_time <= release_time:
            hold_ball_on_palm(scene, palm_site_id)
            tossed = False
        else:
            if not tossed:
                velocity = scene.ball_dof_address
                scene.data.qvel[velocity:velocity + 3] = toss
                scene.data.qvel[velocity + 3:velocity + 6] = 0.0
                tossed = True
            scene._apply_drag()

    def begin_serve() -> None:
        scene.reset(trajectory[0])
        apply_base()
        if release_time is None:
            scene._set_ball(ball_position, ball_velocity)
            scene.mj.mj_forward(scene.model, scene.data)
        else:
            hold_ball_on_palm(scene, palm_site_id)

    image_written = False
    renderer: mujoco.Renderer | None = None
    video: _SimulationVideo | None = None
    if args.video:
        scene.model.vis.global_.offwidth = max(scene.model.vis.global_.offwidth, _VIDEO_WIDTH)
        scene.model.vis.global_.offheight = max(scene.model.vis.global_.offheight, _VIDEO_HEIGHT)
        renderer = mujoco.Renderer(scene.model, height=_VIDEO_HEIGHT, width=_VIDEO_WIDTH)
        video = _SimulationVideo(
            Path(__file__).resolve().parents[3] / "out" / f"serve_{csv_path.stem}.mp4",
            _VIDEO_WIDTH,
            _VIDEO_HEIGHT,
            _VIDEO_FPS,
        )
    camera = _serve_camera()
    frame_dt = 1.0 / _VIDEO_FPS

    def capture_frame() -> None:
        if renderer is None or video is None or image_written:
            return
        renderer.update_scene(scene.data, camera=camera)
        video.write(renderer.render())

    net_x = float(scene.near_edge_x + scene.net_x_table)
    ball_position_slice = slice(scene.ball_qpos_address, scene.ball_qpos_address + 3)

    def replay(viewer: object | None) -> None:
        nonlocal image_written
        begin_serve()
        samples: list[tuple[float, np.ndarray]] = [(0.0, scene.data.qpos[ball_position_slice].copy())]
        bounces: list[dict] = []
        errors: list[dict] = []
        our_bounces = 0
        opponent_bounces = 0
        last_bounce_time = -1.0
        last_net_time = -1.0
        previous_table = False
        previous_net = False
        sim_time = 0.0
        next_frame = 0.0

        def on_clock() -> None:
            nonlocal next_frame
            while next_frame <= sim_time + 1.0e-9:
                capture_frame()
                next_frame += frame_dt

        on_clock()
        for step in range(steps):
            if viewer is not None and not viewer.is_running():
                break
            tick = time.perf_counter()
            source = min(step * dt * hz, trajectory.shape[0] - 1.0)
            lower = int(math.floor(source))
            upper = min(lower + 1, trajectory.shape[0] - 1)
            fraction = source - lower
            q = trajectory[lower] + fraction * (trajectory[upper] - trajectory[lower])
            qd = (trajectory[upper] - trajectory[lower]) * hz
            scene._pin_robot(q, qd)
            apply_base()
            if release_time is None:
                scene._apply_drag()
            else:
                place_ball(step * dt)
            scene.mj.mj_step(scene.model, scene.data)
            sim_time += dt
            position = scene.data.qpos[ball_position_slice].copy()
            samples.append((sim_time, position))
            table_hit, net_hit = _table_top_and_net(scene)
            if table_hit and not previous_table and sim_time - last_bounce_time >= 0.05:
                side = "our" if float(position[0]) < net_x else "opponent"
                bounces.append({"time": sim_time, "position": position.copy(), "side": side})
                last_bounce_time = sim_time
                if side == "our":
                    our_bounces += 1
                    if our_bounces == 2:
                        errors.append({"time": sim_time, "position": position.copy(), "type": _ERROR_TWO_OURS})
                else:
                    opponent_bounces += 1
                    if opponent_bounces == 2:
                        errors.append(
                            {"time": sim_time, "position": position.copy(), "type": _ERROR_TWO_OPPONENT}
                        )
            if net_hit and not previous_net and sim_time - last_net_time >= 0.05:
                errors.append({"time": sim_time, "position": position.copy(), "type": _ERROR_NET})
                last_net_time = sim_time
            previous_table = table_hit
            previous_net = net_hit
            on_clock()
            if viewer is not None:
                viewer.sync()
                time.sleep(max(0.0, dt - (time.perf_counter() - tick)))
            if _ball_on_floor(scene):
                break
        if image_written or len(samples) < 2:
            return
        path = np.stack([sample[1] for sample in samples])
        print(save_vertical_projection(scene, path, csv_path))
        print(save_side_projection(scene, path, csv_path))
        report = _serve_report(scene, samples, bounces, errors, _ball_on_floor(scene))
        _print_errors(report["errors"])
        print(save_serve_report(report, csv_path))
        if video is not None:
            saved_video = video.close()
            if saved_video is not None:
                print(saved_video)
        image_written = True

    try:
        if not args.visual:
            replay(None)
            return

        with mujoco.viewer.launch_passive(scene.model, scene.data) as viewer:
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            viewer.cam.lookat[:] = _CAMERA_LOOKAT
            viewer.cam.distance = _CAMERA_DISTANCE
            viewer.cam.azimuth = _CAMERA_AZIMUTH
            viewer.cam.elevation = _CAMERA_ELEVATION
            while viewer.is_running():
                replay(viewer)
    finally:
        if video is not None:
            video.close()
        if renderer is not None:
            renderer.close()


if __name__ == "__main__":
    main()
