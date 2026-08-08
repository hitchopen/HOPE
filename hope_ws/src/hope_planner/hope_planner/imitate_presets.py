"""Pure-Python core for the ``planner_imitate`` fake-planner command source.

This module is ROS-free (numpy only) so it can be unit-tested without a ROS
environment, mirroring the ``planner.py`` (logic) / ``node.py`` (ROS wrapper)
split used by the real HOPE planner.

WHAT THIS IS
------------
A TEMPORARY fake planner for sim-to-real bring-up *before* motion capture and
real ball tracking are ready. It imitates the HITTER planner *output interface*
(``hope_msgs/RacketCommand``) with safe, repeatable, staged strike commands. It
does NOT predict real ball trajectories and is NOT the final ball planner.

It exists to validate, with the validated checkpoint ``model_15200``:
ONNX inference, deterministic (no-dither) execution, standing stability,
forehand/backhand swings, the target-command interface, safety-stop behaviour,
and logging for later comparison with MuJoCo.

FRAME (read carefully)
----------------------
``model_15200`` was trained/validated on ROBOT-RELATIVE targets:
  * +x = robot forward, +y = robot left, +z = up (height above the floor)
  * origin = the robot's ground point under ``base_link``
  * validated strike plane x = 0.40 m, |y| in [0.05, 0.45], z in [0.70, 1.05]
This is the DEFAULT output frame, tagged ``frame_id = "base_link"`` (the targets
are robot-relative; z is height above the floor, matching the env-origin
convention the policy was trained in).

The REAL ``hope_planner`` instead publishes in the HOPE canonical ``world``
(table) frame. ``planner_imitate`` can emit ``world`` too (``frame_id="world"``)
via an explicit, documented transform that needs the robot's measured world pose
(``robot_world_xyz`` + ``robot_world_yaw``). That pose is currently UNMEASURED
(``hope_world_frame.yaml: mocap_to_base_link`` is all-zero TODO), so ``world``
output prints a loud warning until you set it. See ``to_world()``.

Strike phase (forehand 0.36, backhand 0.50) is a CONTROLLER-side constant, not a
field of ``RacketCommand`` (the message conveys timing via ``time_to_strike`` /
``strike_time``). We carry it here for logging so the CSV matches what the
controller will use, and so the operator can see it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

import numpy as np

# ---------------------------------------------------------------------------
# model_15200 validated target distribution (robot-relative frame).
# Used for (a) the default presets and (b) the safety/training-range warning.
# Sourced from cfg/task/HOPEPingPong.yaml (uniform mode) + the MuJoCo sim2sim
# that validated model_15200: x fixed 0.40, |y| in [0.05,0.45], per-clip z.
# ---------------------------------------------------------------------------
MODEL_15200_RANGE = {
    "x": (0.30, 0.50),        # trained on fixed x=0.40; allow a small bring-up margin
    "y_abs": (0.05, 0.45),    # |y|; sign encodes swing (forehand -y, backhand +y)
    "z": (0.65, 1.30),        # forehand box ~[0.74,0.98], backhand box ~[1.05,1.30]
    "speed": (0.0, 3.50),     # target paddle speed at strike (training box |v|)
}
# ---------------------------------------------------------------------------
# model_9000 (2026-07-03, run 2026-07-03_02-01-17) target distribution — the
# 2026-07-02 blade re-plane PER-CLIP boxes (cfg/task/HOPEPingPongDeployParity.yaml).
# NOTE the schema differs from 15200: boxes are per-clip and SIGNED in y (the
# backhand box crosses y=0), so the shared y_abs form is only an approximation.
# ⚠ model_9000 is a WALK-AND-STRIKE policy (turns ~84 deg, steps 0.4-0.65 m to
# its target): its targets are only meaningful with real base localization; do
# NOT drive it through this base_link-frame fake planner without mocap.
# ---------------------------------------------------------------------------
MODEL_9000_RANGE = {
    "x": (0.56, 0.78),        # union of fh [0.58,0.78] / bh [0.56,0.76]
    "y_abs": (0.00, 0.64),    # fh y in [-0.64,-0.24]; bh y in [-0.07,0.33] (crosses 0)
    "z": (0.72, 1.13),        # fh z [0.72,0.92]; bh z [0.93,1.13]
    "speed": (0.0, 3.50),
    "pos_per_clip": {"forehand": ((0.58, 0.78), (-0.64, -0.24), (0.72, 0.92)),
                     "backhand": ((0.56, 0.76), (-0.07, 0.33), (0.93, 1.13))},
    "vel_per_clip": {"forehand": ((1.05, 2.05), (0.96, 1.96), (0.31, 1.11)),
                     "backhand": ((1.61, 2.61), (-1.21, -0.21), (0.00, 0.71))},
}
# Strike phases are PER-MODEL (baked in each ONNX's clip_strike_phases metadata):
# model_15200 era = (0.36, 0.50); model_9000 / v4 clips = (0.47, 0.333).
FOREHAND_STRIKE_PHASE = 0.36
BACKHAND_STRIKE_PHASE = 0.50
MODEL_9000_FOREHAND_STRIKE_PHASE = 0.47
MODEL_9000_BACKHAND_STRIKE_PHASE = 0.333
FOREHAND = "forehand"
BACKHAND = "backhand"


@dataclass
class SafetyLimits:
    """Conservative clamps applied to every generated command.

    DEFAULTS ARE model_15200-ERA (x capped at 0.50). For a model_9000 bring-up use
    ``SafetyLimits.for_model_9000()`` — the 15200 defaults hard-clamp every valid
    model_9000 target OOD (its boxes need x >= 0.56).
    """

    x_range: tuple = (0.30, 0.50)
    y_abs_range: tuple = (0.05, 0.45)
    z_range: tuple = (0.65, 1.30)
    max_speed: float = 3.0          # m/s hard ceiling on target paddle speed
    max_pos_step: float = 0.50      # m; max change in target position per published cycle (slew)
    backhand_disabled: bool = False  # hard-disable backhand swings (bring-up safety)

    @classmethod
    def for_model_9000(cls) -> "SafetyLimits":
        return cls(x_range=(0.56, 0.78), y_abs_range=(0.00, 0.64), z_range=(0.72, 1.13))


@dataclass
class ImitateConfig:
    """All tunables for the fake planner. Defaults are safe bring-up values."""

    level: int = 0                  # bring-up level 0..5 (see LEVELS)
    frame_id: str = "base_link"     # "base_link" (robot-relative, default) or "world"
    strike_period_s: float = 3.0    # seconds per fake strike (time_to_strike counts period -> 0)
    normal_speed: float = 2.5       # m/s target paddle speed for "normal" levels (2,4,5)
    slow_speed: float = 1.0         # m/s target paddle speed for "slow" levels (1,3)
    elevation_deg: float = 18.0     # target velocity/normal tilt above horizontal (forward+up)
    # world-frame transform inputs (only used when frame_id == "world"). The robot
    # base pose in the HOPE world frame: world = Rz(yaw) @ base + xyz. UNMEASURED today.
    robot_world_xyz: tuple = (0.0, 0.0, 0.0)
    robot_world_yaw: float = 0.0
    safety: SafetyLimits = field(default_factory=SafetyLimits)


@dataclass
class _LevelDef:
    name: str
    swings: tuple          # () => stand only; ("forehand",) etc; ("forehand","backhand") => alternate
    slow: bool


# Staged bring-up levels. Each level is selectable via ImitateConfig.level.
LEVELS = {
    0: _LevelDef("stand", (), slow=False),
    1: _LevelDef("forehand_slow", (FOREHAND,), slow=True),
    2: _LevelDef("forehand", (FOREHAND,), slow=False),
    3: _LevelDef("backhand_slow", (BACKHAND,), slow=True),
    4: _LevelDef("backhand", (BACKHAND,), slow=False),
    5: _LevelDef("alternate", (FOREHAND, BACKHAND), slow=False),
}


@dataclass
class ImitateCommand:
    """One generated planner command (frame-resolved, post-safety)."""

    seq: int
    t: float                       # generation time (s)
    swing_type: str                # "forehand" | "backhand" | "stand"
    frame_id: str                  # frame the pos/vel/normal are expressed in
    position: np.ndarray           # racket target position (3,)
    velocity: np.ndarray           # racket target velocity (3,)
    normal: np.ndarray             # racket target face normal, unit (3,)
    strike_phase: float            # controller-side clip phase of contact (0.36 / 0.50); NaN for stand
    time_to_strike: float          # seconds until strike (counts strike_period -> 0); NaN for stand
    strike_time: float             # absolute strike time = t + time_to_strike; NaN for stand
    clip_phase: float              # 0->1 progress through the current fake swing (debug)
    level: int
    level_name: str
    valid: bool                    # False for stand / estop -> controller should just stand
    clamped: bool                  # True if any field was clamped to the safety box
    out_of_training_range: bool    # True if the requested target left model_15200's range


def _swing_base_target(swing_type: str, slow: bool, cfg: ImitateConfig):
    """Robot-relative (base_link) target pos/vel/normal for one swing type.

    Returns (position[3], velocity[3], normal[3]) with +x forward, +y left,
    +z up (height above floor). Numbers track model_15200's validated box.
    """
    x = 0.40  # validated fixed strike plane (do NOT move to 0.70 for this checkpoint)
    speed = cfg.slow_speed if slow else cfg.normal_speed
    elev = math.radians(cfg.elevation_deg)
    # forward + slightly up swing; ball is returned toward +x (opponent).
    v_hat = np.array([math.cos(elev), 0.0, math.sin(elev)])
    if swing_type == FOREHAND:
        y = -0.22 if slow else -0.30     # forehand on -y (right-arm paddle)
        z = 0.82 if slow else 0.88       # forehand racket height band
    else:  # BACKHAND
        y = +0.22 if slow else +0.30     # backhand on +y
        z = 1.02 if slow else 1.12       # conservative-lower (slow) vs normal backhand height
    position = np.array([x, y, z])
    velocity = speed * v_hat
    normal = v_hat.copy()                # face normal ~ outgoing direction (informational for model_15200)
    return position, velocity, normal


def to_world(p_b, v_b, n_b, robot_world_xyz, robot_world_yaw):
    """Transform a base_link (robot-relative) target into the HOPE world frame.

    world = Rz(yaw) @ base + robot_world_xyz, with Rz a yaw-only rotation.

    ASSUMPTIONS (documented, must be verified before trusting world output):
      * base frame has no roll/pitch w.r.t. world (robot standing upright).
      * ``robot_world_xyz`` is the robot base origin in world coords INCLUDING any
        z offset between the base-frame z-origin (floor under the robot) and the
        world z-origin (HOPE table surface; floor is world z = -0.76).
      * ``robot_world_yaw`` is the robot's heading in world (radians).
    These come from the (currently TODO/unmeasured) mocap_to_base_link in
    hope_world_frame.yaml. Until measured, world output is NOT trustworthy.
    """
    c, s = math.cos(robot_world_yaw), math.sin(robot_world_yaw)
    Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    xyz = np.asarray(robot_world_xyz, dtype=float)
    return Rz @ np.asarray(p_b) + xyz, Rz @ np.asarray(v_b), Rz @ np.asarray(n_b)


def _apply_safety(position, velocity, normal, swing_type, safety: SafetyLimits):
    """Clamp position box + speed to the safety limits. Returns (pos, vel, clamped, oot)."""
    p = position.copy()
    v = velocity.copy()
    clamped = False
    # out-of-training-range check (pre-clamp, against model_15200's validated box)
    oot = (
        not (MODEL_15200_RANGE["x"][0] <= p[0] <= MODEL_15200_RANGE["x"][1])
        or not (MODEL_15200_RANGE["y_abs"][0] <= abs(p[1]) <= MODEL_15200_RANGE["y_abs"][1])
        or not (MODEL_15200_RANGE["z"][0] <= p[2] <= MODEL_15200_RANGE["z"][1])
        or float(np.linalg.norm(v)) > MODEL_15200_RANGE["speed"][1]
    )
    # position box clamp
    nx = float(np.clip(p[0], *safety.x_range))
    y_abs = float(np.clip(abs(p[1]), *safety.y_abs_range))
    ny = math.copysign(y_abs, p[1] if p[1] != 0.0 else (1.0 if swing_type == BACKHAND else -1.0))
    nz = float(np.clip(p[2], *safety.z_range))
    if (nx, ny, nz) != (p[0], p[1], p[2]):
        clamped = True
    p = np.array([nx, ny, nz])
    # speed clamp
    speed = float(np.linalg.norm(v))
    if speed > safety.max_speed and speed > 1e-9:
        v = v * (safety.max_speed / speed)
        clamped = True
    return p, v, clamped, oot


class ImitatePlanner:
    """Stateful generator of staged fake HITTER strike commands.

    Call :meth:`step(t)` at the control/command rate. Each fake swing lasts
    ``strike_period_s``; ``time_to_strike`` counts that period down to 0 (the
    strike instant), then a new swing begins (next swing type for alternating
    levels). Deterministic and repeatable: no randomness.
    """

    def __init__(self, cfg: ImitateConfig):
        self.cfg = cfg
        self._seq = 0
        self._swing_index = 0          # which swing in the level's tuple
        self._swing_start_t = None     # wall time the current swing began
        self._estop = False
        self._last_position = None     # for the per-cycle slew limit

    def set_estop(self, engaged: bool):
        self._estop = bool(engaged)

    def _stand_command(self, t):
        nan = float("nan")
        return ImitateCommand(
            seq=self._seq, t=t, swing_type="stand", frame_id=self.cfg.frame_id,
            position=np.zeros(3), velocity=np.zeros(3), normal=np.array([1.0, 0.0, 0.0]),
            strike_phase=nan, time_to_strike=nan, strike_time=nan, clip_phase=nan,
            level=self.cfg.level, level_name=LEVELS[self.cfg.level].name,
            valid=False, clamped=False, out_of_training_range=False,
        )

    def step(self, t: float) -> ImitateCommand:
        cfg = self.cfg
        level = LEVELS[cfg.level]

        # estop or Level 0 (stand): emit an invalid command -> controller stands.
        if self._estop or not level.swings:
            return self._stand_command(t)

        # swing scheduling: time_to_strike counts strike_period_s -> 0, then advance.
        if self._swing_start_t is None:
            self._swing_start_t = t
            self._seq += 1
        elapsed = t - self._swing_start_t
        if elapsed >= cfg.strike_period_s:
            self._swing_index += 1
            self._swing_start_t = t
            self._seq += 1
            elapsed = 0.0
        tts = max(0.0, cfg.strike_period_s - elapsed)
        clip_phase = min(1.0, elapsed / max(cfg.strike_period_s, 1e-6))

        # which swing this cycle (alternating levels cycle through the tuple)
        swings = level.swings
        swing_type = swings[self._swing_index % len(swings)]
        if swing_type == BACKHAND and cfg.safety.backhand_disabled:
            # backhand hard-disabled -> stand instead of swinging
            return self._stand_command(t)

        strike_phase = FOREHAND_STRIKE_PHASE if swing_type == FOREHAND else BACKHAND_STRIKE_PHASE

        # base_link target, then safety, then optional slew limit, then frame transform.
        p_b, v_b, n_b = _swing_base_target(swing_type, level.slow, cfg)
        p_b, v_b, clamped, oot = _apply_safety(p_b, v_b, n_b, swing_type, cfg.safety)

        # per-cycle slew limit on the (base-frame) target position
        if self._last_position is not None:
            step_vec = p_b - self._last_position
            step_norm = float(np.linalg.norm(step_vec))
            if step_norm > cfg.safety.max_pos_step and step_norm > 1e-9:
                p_b = self._last_position + step_vec * (cfg.safety.max_pos_step / step_norm)
                clamped = True
        self._last_position = p_b.copy()

        # frame resolution
        if cfg.frame_id == "world":
            pos, vel, nrm = to_world(p_b, v_b, n_b, cfg.robot_world_xyz, cfg.robot_world_yaw)
        else:
            pos, vel, nrm = p_b, v_b, n_b
        nn = float(np.linalg.norm(nrm))
        nrm = nrm / nn if nn > 1e-9 else np.array([1.0, 0.0, 0.0])

        return ImitateCommand(
            seq=self._seq, t=t, swing_type=swing_type, frame_id=cfg.frame_id,
            position=pos, velocity=vel, normal=nrm,
            strike_phase=strike_phase, time_to_strike=tts, strike_time=t + tts,
            clip_phase=clip_phase, level=cfg.level, level_name=level.name,
            valid=True, clamped=clamped, out_of_training_range=oot,
        )
