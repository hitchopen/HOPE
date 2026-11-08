#!/usr/bin/env python3
"""Write a serve025 timeline from the tested smooth-center clip.

The reference is ``a3p_op3_serve025_smooth_center_v14.csv``. With the defaults
below, the script writes that clip again: same events, same 468 frames, same
joint positions and the stored velocities. Those defaults are the parameters
of the tested file. v12 differs only in the right arm; the waist, neck, left
arm, and legs of v14 are what stayed up on the robot, so they are not part of
the adjustable set.

``--shift``, ``--rotation``, ``--advance``, and ``--smooth`` move only the
right arm during the swing. ``--rotation`` is one number in [-1, 1]. -1 aims at the left corner: +10
degrees about world Z, and that swing is two frames later so the ball meets
the flatter part of the stroke. 1 aims at the right corner: -17 degrees, with
no extra delay. At the labeled contact, +16.5 degrees lands short and bounces
a second time on the far side. The later, smaller left yaw lands once, at the
same depth as the right corner. Values in between blend straight through 0. The return to stand stays on the tested clip, and
the racket rotation is solved in place so the face turns without leaving the
swing. The last frame stays on the tested stand pose.

Run it with the MuJoCo virtualenv in ``apps/a3_mujoco_serve/.venv``. MuJoCo is
loaded only when ``--shift`` or ``--rotation`` is nonzero.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
MOTION_DIR = REPO / "a3_deploy/a3_deploy_example/assets/a3_runtime/serve/motions"
DEFAULT_SOURCE = MOTION_DIR / "a3p_op3_serve025_smooth_center_v14.csv"
# Full-scale yaw. -1 is the left corner, +1 is the right. The left corner is
# a smaller positive angle, met two frames later: +16.5 degrees at the nominal
# instant bounces twice on the far side, short of the right corner.
_YAW_AT_LEFT_DEG = 10.0
_YAW_AT_RIGHT_DEG = 17.0
_LEFT_HIT_DELAY_FRAMES = 2.0
# Half-scale yaw lands short and bounces twice on the far side. This world-Y
# loft is zero at 0 and at both corners, and peaks at ±0.5.
_CARRY_PITCH_DEG = 12.0
DEFAULT_MODEL = REPO / (
    "a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim/src/models/bin/cfg/model/"
    "a3_pingpong/a3_pingpong.xml"
)
SDK_ORDER = (
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "head_yaw_joint",
    "head_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
)
RIGHT_ARM = np.arange(12, 19)
# Reader accepts |qd| up to half of these SDK velocity limits. The step clip
# stays slightly inside that, matching retarget_serve025.py.
VELOCITY_LIMIT = np.array(
    [
        12.042771838760874, 22.7, 9.24785, 12.775810124598491,
        12.775810124598491, 13.613568165555769, 13.613568165555769,
        15.707963267948966, 15.707963267948966, 15.707963267948966,
        12.775810124598491, 12.775810124598491, 13.613568165555769,
        13.613568165555769, 15.707963267948966, 15.707963267948966,
        15.707963267948966, 12.775810124598491, 12.775810124598491,
        12.042771838760874, 12.042771838760874, 12.042771838760874,
        14.660765716752367, 10.8, 19.37, 12.042771838760874,
        12.042771838760874, 12.042771838760874, 14.660765716752367,
        10.8, 19.37,
    ],
    dtype=np.float64,
)
DT = 0.01
DETACH_FRAME = 52
RECOVERY_FRAME = 167
LIMIT_INSET = 0.021


def _smoothstep(x: np.ndarray) -> np.ndarray:
    clipped = np.clip(x, 0.0, 1.0)
    return clipped ** 3 * (10.0 - 15.0 * clipped + 6.0 * clipped ** 2)


def _blend_weight(frame_count: int) -> np.ndarray:
    """Full through the hit, then gone before the tested return to stand.

    The return is what kept the robot up. It stays on the reference, so a
    rotated swing cannot retarget that return into a new pose.
    """

    fade_frames = 40.0
    progress = (np.arange(frame_count) - (RECOVERY_FRAME - fade_frames)) / fade_frames
    weight = 1.0 - _smoothstep(progress)
    weight[RECOVERY_FRAME:] = 0.0
    return weight


def _rotvec(rotation: np.ndarray) -> np.ndarray:
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    angle = float(np.arccos(cosine))
    if angle < 1.0e-8:
        return np.zeros(3)
    scale = angle / (2.0 * np.sin(angle))
    return scale * np.array(
        [
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ]
    )


def _matrix(rotvec: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rotvec))
    if angle < 1.0e-12:
        return np.eye(3)
    axis = rotvec / angle
    x_axis, y_axis, z_axis = axis
    skew = np.array(
        [[0.0, -z_axis, y_axis], [z_axis, 0.0, -x_axis], [-y_axis, x_axis, 0.0]]
    )
    return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)


def _load(path: Path) -> tuple[list[str], list[dict[str, str]], np.ndarray, np.ndarray, np.ndarray]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError(f"CSV has no header: {path}")
        fields = list(reader.fieldnames)
        rows = list(reader)
    frames = [row for row in rows if row.get("row_kind") == "FRAME"]
    if len(frames) != 468:
        raise ValueError(f"serve025 timeline needs 468 FRAME rows, got {len(frames)}: {path}")
    missing = [name for name in SDK_ORDER if f"q_rad::{name}" not in frames[0]]
    if missing:
        raise ValueError(f"CSV is missing joint columns: {missing}")
    q = np.array([[float(row[f"q_rad::{name}"]) for name in SDK_ORDER] for row in frames])
    qd = np.array([[float(row[f"qd_rad_s::{name}"]) for name in SDK_ORDER] for row in frames])
    qdd = np.array([[float(row[f"qdd_rad_s2::{name}"]) for name in SDK_ORDER] for row in frames])
    return fields, rows, q, qd, qdd


def _neighbor_smooth(values: np.ndarray, passes: int) -> np.ndarray:
    smoothed = np.array(values, dtype=np.float64, copy=True)
    for _ in range(passes):
        blurred = smoothed.copy()
        blurred[1:-1] = 0.25 * smoothed[:-2] + 0.5 * smoothed[1:-1] + 0.25 * smoothed[2:]
        smoothed = blurred
    return smoothed


def _retarget_arm(
    model_path: Path,
    q: np.ndarray,
    shift: np.ndarray,
    rotation_deg: np.ndarray,
    weight: np.ndarray,
) -> float:
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    joint_ids = np.array(
        [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in SDK_ORDER]
    )
    if np.any(joint_ids < 0):
        raise ValueError(f"model is missing an SDK joint: {model_path}")
    qpos_adr = model.jnt_qposadr[joint_ids].astype(int)
    dof_adr = model.jnt_dofadr[joint_ids].astype(int)
    arm_qpos = qpos_adr[RIGHT_ARM]
    arm_dof = dof_adr[RIGHT_ARM]
    limits = model.jnt_range[joint_ids[RIGHT_ARM]]
    low = limits[:, 0] + LIMIT_INSET
    high = limits[:, 1] - LIMIT_INSET
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "right_racket")
    if site < 0:
        raise ValueError("model has no right_racket site")
    def residual_at(joints: np.ndarray, target: np.ndarray, orient: np.ndarray, reference: np.ndarray) -> np.ndarray:
        data.qpos[arm_qpos] = joints
        mujoco.mj_forward(model, data)
        position = data.site_xpos[site] - target
        orientation = _rotvec(data.site_xmat[site].reshape(3, 3) @ orient.T)
        return np.concatenate(
            [12.0 * position, 2.0 * orientation, 0.03 * (joints - reference)]
        )

    max_error = 0.0
    for index in range(len(q)):
        if weight[index] == 0.0:
            continue
        data.qpos[qpos_adr] = q[index]
        mujoco.mj_forward(model, data)
        target = data.site_xpos[site].copy() + weight[index] * shift
        orient = _matrix(np.radians(rotation_deg) * weight[index]) @ data.site_xmat[site].reshape(3, 3)
        reference = q[index, RIGHT_ARM].copy()
        current = reference.copy()
        # A plain damped step overshoots a 30 degree face rotation and leaves
        # the racket up to a meter off the swing. Accept a step only when it
        # lowers the residual, and shrink it until it does.
        for _ in range(25):
            residual = residual_at(current, target, orient, reference)
            if float(np.linalg.norm(residual[:3])) < 1.2e-3 and float(np.linalg.norm(residual[3:6])) < 2.0e-3:
                break
            jac_pos = np.zeros((3, model.nv))
            jac_rot = np.zeros((3, model.nv))
            mujoco.mj_jacSite(model, data, jac_pos, jac_rot, site)
            jacobian = np.vstack(
                [
                    12.0 * jac_pos[:, arm_dof],
                    2.0 * jac_rot[:, arm_dof],
                    0.03 * np.eye(len(RIGHT_ARM)),
                ]
            )
            cost = float(residual @ residual)
            improved = False
            for damping in (1.0e-4, 1.0e-2, 1.0e-1, 1.0, 10.0):
                gram = jacobian.T @ jacobian + damping * np.eye(len(RIGHT_ARM))
                direction = np.linalg.solve(gram, jacobian.T @ -residual)
                for scale in (1.0, 0.5, 0.25, 0.1):
                    trial = np.clip(current + scale * direction, low, high)
                    trial_residual = residual_at(trial, target, orient, reference)
                    if float(trial_residual @ trial_residual) < cost:
                        current = trial
                        improved = True
                        break
                if improved:
                    break
            if not improved or float(np.linalg.norm(direction)) < 1.0e-5:
                break
        q[index, RIGHT_ARM] = current
        position_error = float(np.linalg.norm(residual_at(current, target, orient, reference)[:3] / 12.0))
        max_error = max(max_error, position_error)
    return max_error


def _advance(arm: np.ndarray, frames: float) -> None:
    grid = np.arange(len(arm), dtype=np.float64)
    warp = frames * (1.0 - _smoothstep((grid - 110.0) / 57.0))
    for joint in range(arm.shape[1]):
        arm[:, joint] = np.interp(grid + warp, grid, arm[:, joint])


def _step_limit(index: int) -> np.ndarray:
    margin = 0.029 if index <= DETACH_FRAME or index >= RECOVERY_FRAME else 0.069
    return np.minimum(np.full(len(RIGHT_ARM), margin), 0.49 * VELOCITY_LIMIT[RIGHT_ARM] * DT)


def _clip_steps(q: np.ndarray, reference: np.ndarray) -> None:
    """Keep the swing inside the reader step envelope and meet the tested return.

    Recovery positions are the reference. The backward pass only repairs the
    junction, and the step budget is too small to drag that pose back into the hit.
    """

    q[RECOVERY_FRAME:] = reference[RECOVERY_FRAME:]
    arm = q[:, RIGHT_ARM]
    for index in range(1, RECOVERY_FRAME):
        limit = _step_limit(index)
        arm[index] = np.clip(arm[index], arm[index - 1] - limit, arm[index - 1] + limit)
    for index in range(RECOVERY_FRAME - 1, 0, -1):
        limit = _step_limit(index + 1)
        arm[index] = np.clip(arm[index], arm[index + 1] - limit, arm[index + 1] + limit)
    q[:, RIGHT_ARM] = arm
    q[RECOVERY_FRAME:] = reference[RECOVERY_FRAME:]


def _derivatives(q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    qd = np.gradient(q, DT, axis=0)
    qdd = np.gradient(qd, DT, axis=0)
    qd[[0, -1]] = 0.0
    qdd[[0, -1]] = 0.0
    qd = np.clip(qd, -0.49 * VELOCITY_LIMIT, 0.49 * VELOCITY_LIMIT)
    return qd, qdd


def generate(
    source: Path,
    output: Path,
    *,
    shift: tuple[float, float, float] = (0.0, 0.0, 0.0),
    rotation: tuple[float, float, float] = (0.0, 0.0, 0.0),
    advance: float = 0.0,
    smooth: float = 0.0,
    model: Path = DEFAULT_MODEL,
) -> dict[str, float]:
    """Return the max right-arm change and, when the arm is moved, the IK miss."""

    fields, rows, reference, reference_qd, reference_qdd = _load(source)
    if not 0.0 <= smooth <= 1.0:
        raise ValueError("--smooth must be between 0 and 1")
    q = reference.copy()
    weight = _blend_weight(len(q))
    shift_v = np.asarray(shift, dtype=np.float64)
    rotation_v = np.asarray(rotation, dtype=np.float64)
    ik_error = 0.0
    if float(np.linalg.norm(shift_v) + np.linalg.norm(rotation_v)) > 0.0:
        ik_error = _retarget_arm(model, q, shift_v, rotation_v, weight)
    if advance:
        # Column indexing with an index array returns a copy.
        delayed = np.array(q[:, RIGHT_ARM], copy=True)
        _advance(delayed, advance)
        q[:, RIGHT_ARM] = delayed
    if smooth > 0.0:
        passes = max(1, int(round(smooth * 30.0)))
        smoothed = _neighbor_smooth(q[:, RIGHT_ARM], passes)
        blend = (smooth * weight)[:, None]
        q[:, RIGHT_ARM] = q[:, RIGHT_ARM] + blend * (smoothed - q[:, RIGHT_ARM])
    q[RECOVERY_FRAME:] = reference[RECOVERY_FRAME:]
    edited = float(np.max(np.abs(q - reference))) > 1.0e-12
    if edited:
        _clip_steps(q, reference)
        qd, qdd = _derivatives(q)
        unchanged = np.abs(q - reference) <= 1.0e-12
        qd[unchanged] = reference_qd[unchanged]
        qdd[unchanged] = reference_qdd[unchanged]
    frames = [row for row in rows if row.get("row_kind") == "FRAME"]
    if edited:
        for index, row in enumerate(frames):
            for joint, name in enumerate(SDK_ORDER):
                if abs(float(q[index, joint]) - float(reference[index, joint])) <= 1.0e-12:
                    continue
                row[f"q_rad::{name}"] = format(float(q[index, joint]), ".12g")
                row[f"qd_rad_s::{name}"] = format(float(qd[index, joint]), ".12g")
                row[f"qdd_rad_s2::{name}"] = format(float(qdd[index, joint]), ".12g")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    return {
        "max_right_arm_delta_rad": float(np.max(np.abs(q[:, RIGHT_ARM] - reference[:, RIGHT_ARM]))),
        "max_other_joint_delta_rad": float(
            np.max(np.abs(np.delete(q - reference, RIGHT_ARM, axis=1)))
        ),
        "max_ik_position_error_m": ik_error,
    }


def _yaw_degrees(unit: float) -> float:
    """Map a [-1, 1] corner amount onto world-Z degrees.

    -1 is the left corner. 1 is the right corner.
    """

    if unit < 0.0:
        return -unit * _YAW_AT_LEFT_DEG
    if unit > 0.0:
        return -unit * _YAW_AT_RIGHT_DEG
    return 0.0


def _carry_pitch_deg(unit: float) -> float:
    """World-Y loft that lengthens the half-scale serves only."""

    span = abs(unit)
    return _CARRY_PITCH_DEG * span * (1.0 - span)


def _default_output(unit: float) -> Path:
    return MOTION_DIR / f"pablo_rz={unit:.2f}.csv"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="CSV path (default: motions/pablo_rz=<rotation>.csv, two decimal places)",
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument(
        "--shift",
        type=float,
        nargs=3,
        default=(0.0, 0.0, 0.0),
        metavar=("X", "Y", "Z"),
        help="right-racket translation in meters, faded out before stand (default 0 0 0)",
    )
    parser.add_argument(
        "--rotation",
        type=float,
        default=0.0,
        metavar="Z",
        help="right-racket yaw in [-1, 1]: -1 is left (+10 deg, delayed 2 frames), 1 is right (-17 deg) (default 0)",
    )
    parser.add_argument(
        "--advance",
        type=float,
        default=0.0,
        help="pull the right-arm swing earlier by this many frames (default 0)",
    )
    parser.add_argument(
        "--smooth",
        type=float,
        default=0.0,
        help="0 keeps the tested right arm; 1 blends it toward a neighbor smooth (default 0)",
    )
    args = parser.parse_args()
    if not -1.0 <= args.rotation <= 1.0:
        raise SystemExit("--rotation must be between -1 and 1")
    yaw_deg = _yaw_degrees(args.rotation)
    # Left yaw at the nominal contact pops the ball short. Delay that swing
    # so the toss meets the same face a moment earlier in the stroke, where it
    # carries as far as the right corner and bounces once.
    hit_delay = _LEFT_HIT_DELAY_FRAMES * args.rotation if args.rotation < 0.0 else 0.0
    source = args.source.expanduser().resolve()
    output = _default_output(args.rotation) if args.output is None else args.output.expanduser().resolve()
    if output == source:
        raise SystemExit("refusing to overwrite the reference CSV; choose another --output")
    summary = generate(
        source,
        output,
        shift=tuple(args.shift),
        rotation=(0.0, _carry_pitch_deg(args.rotation), yaw_deg),
        advance=args.advance + hit_delay,
        smooth=args.smooth,
        model=args.model.expanduser().resolve(),
    )
    print(
        f"wrote {output}\n"
        f"yaw {yaw_deg:.6g} deg\n"
        f"max right-arm change {summary['max_right_arm_delta_rad']:.6g} rad\n"
        f"max other-joint change {summary['max_other_joint_delta_rad']:.6g} rad\n"
        f"max racket IK error {summary['max_ik_position_error_m']:.6g} m"
    )


if __name__ == "__main__":
    main()
