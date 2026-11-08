"""Pure schema-2 mocap base-pose wire helpers owned by hope_bringup."""

import math
from typing import Sequence


BASE_SCHEMA_V2_SIZE = 16
SOURCE_STAMP_INPUT_HEADER = "input_header"
SOURCE_STAMP_LOCAL_RECEIPT = "local_receipt"
FLAG_TRACKING_VALID = 1 << 0
FLAG_QUATERNION_VALID = 1 << 1
FLAG_EXTRINSIC_CALIBRATED = 1 << 2
FLAG_SOURCE_STAMP_HDU_ROS = 1 << 3
FLAG_POLICY_Z_OFFSET_APPLIED = 1 << 4
FLAG_WORLD_FRAME_CALIBRATED = 1 << 5
V17_REQUIRED_FLAGS = (
    FLAG_TRACKING_VALID
    | FLAG_QUATERNION_VALID
    | FLAG_EXTRINSIC_CALIBRATED
    | FLAG_SOURCE_STAMP_HDU_ROS
    | FLAG_WORLD_FRAME_CALIBRATED
)


class BasePoseJumpGuard:
    """Bound pose changes with an absolute noise budget plus allowed motion.

    VRPN callbacks can drain queued frames microseconds apart. Rate alone
    mistakes tiny measured changes for impossible motion. Compare displacement
    directly to tolerance + speed * elapsed time, without inventing a dt.
    A retained 20 ms anchor prevents spending a fresh tolerance on every frame
    of a burst to admit a cumulatively large jump. Rejections never move it.
    Freshness, calibration and tracking validity remain the relay's checks.
    """

    def __init__(self, max_linear_speed=3.0, max_angular_speed=8.0,
                 linear_tolerance_m=0.01, angular_tolerance_rad=math.radians(3.0),
                 window_s=0.020):
        values = (max_linear_speed, max_angular_speed, linear_tolerance_m,
                  angular_tolerance_rad, window_s)
        if not all(math.isfinite(v) and v >= 0.0 for v in values) or window_s <= 0.0:
            raise ValueError("base jump limits must be finite/nonnegative and window positive")
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.linear_tolerance_m = linear_tolerance_m
        self.angular_tolerance_rad = angular_tolerance_rad
        self.window_ns = round(window_s * 1e9)
        if self.window_ns < 1:
            raise ValueError("base jump window must be at least one nanosecond")
        self.reset()

    def reset(self):
        self._last = self._anchor = None

    def observe(self, stamp_ns, position, quaternion):
        position = tuple(float(v) for v in position)
        if len(position) != 3 or not all(math.isfinite(v) for v in position):
            raise ValueError("base position must contain three finite values")
        quaternion = _normalize_quaternion(quaternion, "base")
        if not isinstance(stamp_ns, int) or stamp_ns <= 0:
            raise ValueError("base jump timestamp must be a positive integer")
        for reference in (self._last, self._anchor):
            if reference is None:
                continue
            ns, old_position, old_quaternion = reference
            dt = (stamp_ns - ns) * 1e-9
            if dt <= 0.0:
                raise ValueError("non-positive source timestep")
            distance = math.dist(position, old_position)
            linear_limit = self.linear_tolerance_m + self.max_linear_speed * dt
            if distance > linear_limit:
                raise ValueError(
                    f"base linear jump {distance:.4f}m exceeds {linear_limit:.4f}m "
                    f"over {dt:.6f}s (including noise tolerance)")
            dot = min(1.0, abs(sum(a * b for a, b in zip(quaternion, old_quaternion))))
            angle = 2.0 * math.acos(dot)
            angular_limit = self.angular_tolerance_rad + self.max_angular_speed * dt
            if angle > angular_limit:
                raise ValueError(
                    f"base angular jump {math.degrees(angle):.3f}deg exceeds "
                    f"{math.degrees(angular_limit):.3f}deg over {dt:.6f}s "
                    "(including noise tolerance)")
        sample = (stamp_ns, position, quaternion)
        self._last = sample
        if self._anchor is None or stamp_ns - self._anchor[0] >= self.window_ns:
            self._anchor = sample


def resolve_wire_source_stamp_ns(
    input_sec: int,
    input_nsec: int,
    receipt_ns: int,
    mode: str,
) -> int:
    """Select the schema-2 source stamp without mixing clock domains.

    ``input_header`` is valid only when the pose producer and relay share a
    synchronized ROS clock.  ``local_receipt`` is the production HDU contract:
    the upstream header must still be valid and monotonic, while the wire stamp
    is the HDU ROS receipt time consumed by the MDU freshness gate.
    """
    input_sec_i = int(input_sec)
    input_nsec_i = int(input_nsec)
    receipt_ns_i = int(receipt_ns)
    if input_sec_i <= 0 or input_nsec_i < 0 or input_nsec_i >= 1_000_000_000:
        raise ValueError("input source timestamp is invalid")
    if receipt_ns_i <= 0:
        raise ValueError("local receipt timestamp is invalid")
    if mode == SOURCE_STAMP_INPUT_HEADER:
        return input_sec_i * 1_000_000_000 + input_nsec_i
    if mode == SOURCE_STAMP_LOCAL_RECEIPT:
        return receipt_ns_i
    raise ValueError(
        "source_stamp_mode must be 'input_header' or 'local_receipt'"
    )


def receipt_id_u52(receipt_sha256: str) -> int:
    """Derive an exactly representable wire correlation ID from a full receipt."""
    value = str(receipt_sha256).strip().lower()
    if len(value) != 64 or any(c not in "0123456789abcdef" for c in value):
        raise ValueError("calibration receipt must be exactly 64 hexadecimal characters")
    return int(value[:13], 16)


def _normalize_quaternion(
    quaternion_wxyz: Sequence[float], name: str
) -> tuple[float, float, float, float]:
    values = tuple(float(v) for v in quaternion_wxyz)
    if len(values) != 4 or not all(math.isfinite(v) for v in values):
        raise ValueError(f"{name} quaternion must contain four finite values")
    norm = math.sqrt(sum(v * v for v in values))
    if norm < 0.5 or norm > 1.5:
        raise ValueError(f"{name} quaternion norm is outside [0.5,1.5]")
    return tuple(v / norm for v in values)


def _quat_mul(
    lhs: Sequence[float], rhs: Sequence[float]
) -> tuple[float, float, float, float]:
    aw, ax, ay, az = lhs
    bw, bx, by, bz = rhs
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _quat_rotate(
    quaternion_wxyz: Sequence[float], vector_xyz: Sequence[float]
) -> tuple[float, float, float]:
    qw, qx, qy, qz = quaternion_wxyz
    vx, vy, vz = vector_xyz
    return (
        (1.0 - 2.0 * (qy * qy + qz * qz)) * vx
        + 2.0 * (qx * qy - qw * qz) * vy
        + 2.0 * (qx * qz + qw * qy) * vz,
        2.0 * (qx * qy + qw * qz) * vx
        + (1.0 - 2.0 * (qx * qx + qz * qz)) * vy
        + 2.0 * (qy * qz - qw * qx) * vz,
        2.0 * (qx * qz - qw * qy) * vx
        + 2.0 * (qy * qz + qw * qx) * vy
        + (1.0 - 2.0 * (qx * qx + qy * qy)) * vz,
    )


def compose_marker_to_base_pose(
    position_xyz: Sequence[float],
    marker_quaternion_wxyz: Sequence[float],
    marker_to_base_xyz: Sequence[float],
    marker_to_base_quaternion_wxyz: Sequence[float],
    *,
    previous_base_quaternion_wxyz: Sequence[float] | None = None,
) -> tuple[
    tuple[float, float, float], tuple[float, float, float, float]
]:
    """Compose T_world_marker * T_marker_base without a policy z offset."""

    position = tuple(float(v) for v in position_xyz)
    offset = tuple(float(v) for v in marker_to_base_xyz)
    if len(position) != 3 or len(offset) != 3:
        raise ValueError("position/offset dimensions must be 3/3")
    if not all(math.isfinite(v) for v in (*position, *offset)):
        raise ValueError("position/offset contains a non-finite value")

    q_world_marker = _normalize_quaternion(marker_quaternion_wxyz, "marker")
    q_marker_base = _normalize_quaternion(
        marker_to_base_quaternion_wxyz, "marker-to-base"
    )
    q_world_base = _normalize_quaternion(
        _quat_mul(q_world_marker, q_marker_base), "world-to-base"
    )
    if previous_base_quaternion_wxyz is not None:
        previous = _normalize_quaternion(
            previous_base_quaternion_wxyz, "previous base"
        )
        if sum(a * b for a, b in zip(q_world_base, previous)) < 0.0:
            q_world_base = tuple(-v for v in q_world_base)

    rotated_offset = _quat_rotate(q_world_marker, offset)
    return (
        (
            position[0] + rotated_offset[0],
            position[1] + rotated_offset[1],
            position[2] + rotated_offset[2],
        ),
        q_world_base,
    )


def pose_to_base_flat(
    position_xyz: Sequence[float],
    marker_quaternion_wxyz: Sequence[float],
    marker_to_base_xyz: Sequence[float],
    marker_to_base_quaternion_wxyz: Sequence[float],
    policy_z_offset: float,
    *,
    sequence: int,
    source_sec: int,
    source_nsec: int,
    tracking_quality: float,
    flags: int,
    calibration_id: int,
    world_frame_id: int,
    previous_base_quaternion_wxyz: Sequence[float] | None = None,
) -> list[float]:
    """Build schema 2 using T_world_base = T_world_marker * T_marker_base."""
    z_offset = float(policy_z_offset)
    if not math.isfinite(z_offset):
        raise ValueError("policy z offset is non-finite")
    base_position, q_world_base = compose_marker_to_base_pose(
        position_xyz,
        marker_quaternion_wxyz,
        marker_to_base_xyz,
        marker_to_base_quaternion_wxyz,
        previous_base_quaternion_wxyz=previous_base_quaternion_wxyz,
    )

    sequence_i = int(sequence)
    source_sec_i = int(source_sec)
    source_nsec_i = int(source_nsec)
    quality = float(tracking_quality)
    flags_i = int(flags)
    calibration_i = int(calibration_id)
    world_i = int(world_frame_id)
    if sequence_i < 0 or sequence_i > (1 << 52):
        raise ValueError("sequence is outside the exact Float64 integer range")
    if source_sec_i <= 0 or source_nsec_i < 0 or source_nsec_i >= 1_000_000_000:
        raise ValueError("source timestamp is invalid")
    if not math.isfinite(quality) or quality < 0.0 or quality > 1.0:
        raise ValueError("tracking quality must be finite and in [0,1]")
    if flags_i < 0 or flags_i > (1 << 20):
        raise ValueError("flags are invalid")
    if flags_i & V17_REQUIRED_FLAGS != V17_REQUIRED_FLAGS:
        raise ValueError("required V17 validity/calibration flags are missing")
    for value, name in (
        (calibration_i, "calibration id"),
        (world_i, "world-frame id"),
    ):
        if value <= 0 or value > (1 << 52):
            raise ValueError(f"{name} is outside the exact Float64 integer range")

    return [
        2.0,
        1.0,
        float(sequence_i),
        float(source_sec_i),
        float(source_nsec_i),
        base_position[0],
        base_position[1],
        base_position[2] + z_offset,
        *q_world_base,
        quality,
        float(flags_i),
        float(calibration_i),
        float(world_i),
    ]


def invalid_base_flat(
    *, sequence: int, source_sec: int = 0, source_nsec: int = 0, flags: int = 0
) -> list[float]:
    """Build an explicit, finite schema-2 invalidation packet."""
    return [
        2.0,
        0.0,
        float(max(0, int(sequence))),
        float(max(0, int(source_sec))),
        float(max(0, int(source_nsec))),
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        float(max(0, int(flags))),
        0.0,
        0.0,
    ]
