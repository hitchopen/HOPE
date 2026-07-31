"""Cartesian strike schedule and damped-least-squares IK."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from .constants import RIGHT_ARM_HI, RIGHT_ARM_LO, RIGHT_ARM_JOINTS
from .math3d import align_local_axis, minimum_jerk, rotation_error, slerp

if TYPE_CHECKING:
    from .mujoco_scene import A3ServeScene
    from .physics import ServeCandidate


@dataclass(frozen=True)
class CartesianSchedule:
    position_world: np.ndarray
    rotation_world: np.ndarray
    planned_ready_position_world: np.ndarray
    planned_ready_rotation_world: np.ndarray


@dataclass(frozen=True)
class IkResult:
    joint_trajectory: np.ndarray
    right_arm_trajectory: np.ndarray
    max_position_error_m: float
    max_rotation_error_rad: float
    max_joint_step_rad: float
    max_joint_speed_rad_s: float
    unconverged_frames: tuple[int, ...]


def build_cartesian_schedule(
    *,
    frame_count: int,
    ready_frame: int,
    stroke_start_frame: int,
    strike_frame: int,
    follow_end_frame: int,
    return_end_frame: int,
    initial_position: np.ndarray,
    initial_rotation: np.ndarray,
    ready_position: np.ndarray,
    ready_rotation: np.ndarray,
    candidate: "ServeCandidate",
    normal_axis: int,
) -> CartesianSchedule:
    """Build a hold-compatible strike with the requested velocity at impact."""

    positions = np.zeros((frame_count, 3), dtype=np.float64)
    rotations = np.zeros((frame_count, 3, 3), dtype=np.float64)
    # A racket face is a two-sided plane: +n and -n describe the same physical
    # face orientation in the contact law.  Choose the sign closest to READY so
    # IK never inserts an unnecessary 180-degree wrist/arm flip.
    target_normal = np.asarray(candidate.racket_normal_world, dtype=np.float64)
    if float(np.dot(ready_rotation[:, normal_axis], target_normal)) < 0.0:
        target_normal = -target_normal
    planned_ready_rotation = align_local_axis(
        ready_rotation, normal_axis, target_normal
    )
    planned_ready_position = np.asarray(ready_position, dtype=np.float64)
    contact = np.asarray(candidate.racket_contact_position_world, dtype=np.float64)
    velocity = np.asarray(candidate.racket_velocity_world, dtype=np.float64)

    for frame in range(frame_count):
        if frame <= ready_frame:
            fraction = minimum_jerk(frame / max(ready_frame, 1))
            positions[frame] = (
                np.asarray(initial_position) * (1.0 - fraction)
                + planned_ready_position * fraction
            )
            rotations[frame] = slerp(
                initial_rotation, planned_ready_rotation, fraction
            )
        elif frame <= stroke_start_frame:
            positions[frame] = planned_ready_position
            rotations[frame] = planned_ready_rotation
        elif frame <= strike_frame:
            # Constant acceleration from rest.  The candidate's contact point
            # was constructed with displacement 1/2*u*T.
            fraction = (frame - stroke_start_frame) / max(
                strike_frame - stroke_start_frame, 1
            )
            positions[frame] = (
                planned_ready_position
                + (contact - planned_ready_position) * fraction * fraction
            )
            rotations[frame] = planned_ready_rotation
        elif frame <= follow_end_frame:
            fraction = (frame - strike_frame) / max(follow_end_frame - strike_frame, 1)
            duration_ratio = (follow_end_frame - strike_frame) / max(
                strike_frame - stroke_start_frame, 1
            )
            follow_displacement = (
                contact - planned_ready_position
            ) * duration_ratio
            positions[frame] = contact + follow_displacement * (
                2.0 * fraction - fraction * fraction
            )
            rotations[frame] = planned_ready_rotation
        elif frame <= return_end_frame:
            fraction = minimum_jerk(
                (frame - follow_end_frame) / max(return_end_frame - follow_end_frame, 1)
            )
            follow_position = positions[follow_end_frame]
            positions[frame] = (
                follow_position * (1.0 - fraction)
                + planned_ready_position * fraction
            )
            rotations[frame] = planned_ready_rotation
        else:
            positions[frame] = planned_ready_position
            rotations[frame] = planned_ready_rotation
    return CartesianSchedule(
        position_world=positions,
        rotation_world=rotations,
        planned_ready_position_world=planned_ready_position,
        planned_ready_rotation_world=planned_ready_rotation,
    )


def solve_dls_trajectory(
    scene: "A3ServeScene",
    reference_joint_trajectory: np.ndarray,
    schedule: CartesianSchedule,
    ik_cfg: dict,
    *,
    source_hz: float,
) -> IkResult:
    """Convert every Cartesian sample to A3 joints with DLS IK.

    The seed for each frame is the preceding solved frame.  The code never
    contains a hand-written strike joint pose; only limits and numerical solver
    parameters are configured.
    """

    reference = np.asarray(reference_joint_trajectory, dtype=np.float64)
    output = reference.copy()
    right = reference[0, scene.right_indices].copy()
    damping = float(ik_cfg["damping"])
    orientation_weight = float(ik_cfg["orientation_weight"])
    max_iterations = int(ik_cfg["max_iterations"])
    max_step = float(ik_cfg["max_step_rad"])
    position_tolerance = float(ik_cfg["position_tolerance_m"])
    rotation_tolerance = float(ik_cfg["rotation_tolerance_rad"])
    posture_gain = float(ik_cfg.get("posture_gain", 0.0))
    fail_on_unconverged = bool(ik_cfg.get("fail_on_unconverged", True))
    unconverged: list[int] = []
    max_position_error = 0.0
    max_rotation_error = 0.0

    scene.reset(reference[0])
    previous_position: np.ndarray | None = None
    previous_rotation: np.ndarray | None = None
    for frame in range(reference.shape[0]):
        target_position = schedule.position_world[frame]
        target_rotation = schedule.rotation_world[frame]
        if (
            previous_position is not None
            and np.array_equal(target_position, previous_position)
            and np.array_equal(target_rotation, previous_rotation)
        ):
            output[frame, scene.right_indices] = right
            continue

        seed = reference[frame, scene.right_indices]
        position_error_norm = float("inf")
        rotation_error_norm = float("inf")
        for _ in range(max_iterations):
            full = reference[frame].copy()
            full[scene.right_indices] = right
            scene.set_joint_positions(full)
            scene.data.qvel[:] = 0.0
            scene.mj.mj_forward(scene.model, scene.data)
            current_position, current_rotation = scene.racket_pose()
            position_error = target_position - current_position
            orientation_error = rotation_error(current_rotation, target_rotation)
            position_error_norm = float(np.linalg.norm(position_error))
            rotation_error_norm = float(np.linalg.norm(orientation_error))
            if (
                position_error_norm <= position_tolerance
                and rotation_error_norm <= rotation_tolerance
            ):
                break
            jacobian = scene.racket_jacobian().copy()
            jacobian[3:] *= orientation_weight
            error = np.concatenate(
                (position_error, orientation_error * orientation_weight)
            )
            system = jacobian @ jacobian.T + damping * damping * np.eye(6)
            delta = jacobian.T @ np.linalg.solve(system, error)
            if posture_gain > 0.0:
                pseudo = jacobian.T @ np.linalg.solve(system, jacobian)
                delta += posture_gain * (np.eye(len(RIGHT_ARM_JOINTS)) - pseudo) @ (
                    seed - right
                )
            largest = float(np.max(np.abs(delta)))
            if largest > max_step:
                delta *= max_step / largest
            right = np.clip(right + delta, RIGHT_ARM_LO, RIGHT_ARM_HI)

        output[frame, scene.right_indices] = right
        max_position_error = max(max_position_error, position_error_norm)
        max_rotation_error = max(max_rotation_error, rotation_error_norm)
        if (
            position_error_norm > position_tolerance
            or rotation_error_norm > rotation_tolerance
        ):
            unconverged.append(frame)
        previous_position = target_position.copy()
        previous_rotation = target_rotation.copy()

    if unconverged and fail_on_unconverged:
        preview = ", ".join(str(frame) for frame in unconverged[:10])
        raise RuntimeError(
            f"DLS IK did not converge at {len(unconverged)} frames; first: {preview}"
        )
    steps = np.abs(np.diff(output[:, scene.right_indices], axis=0))
    max_joint_step = float(np.max(steps)) if steps.size else 0.0
    return IkResult(
        joint_trajectory=output,
        right_arm_trajectory=output[:, scene.right_indices].copy(),
        max_position_error_m=max_position_error,
        max_rotation_error_rad=max_rotation_error,
        max_joint_step_rad=max_joint_step,
        max_joint_speed_rad_s=max_joint_step * float(source_hz),
        unconverged_frames=tuple(unconverged),
    )
