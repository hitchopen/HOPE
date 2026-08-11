"""ROS-free decoders for the model_21800 Foxglove observer.

The observer is deliberately read-only.  These helpers decode the two frozen
schema-2 wire contracts and validate local audit-file identifiers without
importing ROS or mutating any runtime state.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import re
from typing import Sequence


BASE_SCHEMA_V2_SIZE = 16
RACKET_SCHEMA_V2_SIZE = 19
MAX_EXACT_FLOAT_INTEGER = 1 << 52

FLAG_TRACKING_VALID = 1 << 0
FLAG_QUATERNION_VALID = 1 << 1
FLAG_EXTRINSIC_CALIBRATED = 1 << 2
FLAG_SOURCE_STAMP_HDU_ROS = 1 << 3
FLAG_WORLD_FRAME_CALIBRATED = 1 << 5
REQUIRED_BASE_FLAGS = (
    FLAG_TRACKING_VALID
    | FLAG_QUATERNION_VALID
    | FLAG_EXTRINSIC_CALIBRATED
    | FLAG_SOURCE_STAMP_HDU_ROS
    | FLAG_WORLD_FRAME_CALIBRATED
)

SESSION_ID_PATTERN = re.compile(r"model21800_[0-9]{8}T[0-9]{6}Z")
PLANNER_ATTEMPT_PATTERN = re.compile(r"planner_attempt_[0-9]{3}")
X_HIT_PATTERN = re.compile(
    r"(?:^|[ ;])x_hit=([-+]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)"
    r"(?:[eE][-+]?[0-9]+)?)"
)


class DecodeError(ValueError):
    """Raised when a wire packet or local status record is malformed."""


@dataclass(frozen=True)
class BasePacket:
    valid: bool
    sequence: int
    source_time_ns: int
    position_xyz: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]
    tracking_quality: float
    flags: int
    calibration_id: int
    world_frame_id: int
    reason: str


@dataclass(frozen=True)
class RacketPacket:
    valid: bool
    swing_sign: float
    position_xyz: tuple[float, float, float]
    velocity_xyz: tuple[float, float, float]
    published_time_to_strike_s: float
    strike_deadline_wall_s: float
    producer_time_ns: int
    command_sequence: int
    flight_id: int
    revision_id: int
    estimator_sample_count: int
    estimator_span_s: float
    reason: str


@dataclass(frozen=True)
class XHitStatus:
    request_id: str
    success: bool
    message: str
    x_hit_m: float | None


def _finite_values(values: Sequence[float], expected_size: int, label: str) -> tuple[float, ...]:
    try:
        decoded = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise DecodeError(f"{label} contains a non-numeric value") from exc
    if len(decoded) != expected_size:
        raise DecodeError(
            f"{label} has {len(decoded)} values; expected {expected_size}"
        )
    if not all(math.isfinite(value) for value in decoded):
        raise DecodeError(f"{label} contains a non-finite value")
    if decoded[0] != 2.0:
        raise DecodeError(f"{label} schema is {decoded[0]!r}; expected 2")
    if decoded[1] not in (0.0, 1.0):
        raise DecodeError(f"{label} validity field is not 0 or 1")
    return decoded


def _exact_integer(
    value: float,
    label: str,
    *,
    minimum: int = 0,
    maximum: int = MAX_EXACT_FLOAT_INTEGER,
) -> int:
    decoded = int(value)
    if float(decoded) != value or decoded < minimum or decoded > maximum:
        raise DecodeError(
            f"{label} must be an exact integer in [{minimum}, {maximum}]"
        )
    return decoded


def _timestamp_ns(seconds: float, nanoseconds: float, label: str) -> int:
    sec = _exact_integer(seconds, f"{label} seconds", minimum=1)
    nsec = _exact_integer(
        nanoseconds,
        f"{label} nanoseconds",
        minimum=0,
        maximum=999_999_999,
    )
    return sec * 1_000_000_000 + nsec


def decode_base_packet(values: Sequence[float]) -> BasePacket:
    """Decode the 16-double authoritative base schema.

    The source stamp belongs to the HDU ROS/system-clock domain when the
    required flag set is present.  Receipt freshness is intentionally not
    inferred here; the ROS observer measures it with its local monotonic clock.
    """

    value = _finite_values(values, BASE_SCHEMA_V2_SIZE, "base schema-2 packet")
    sequence = _exact_integer(value[2], "base sequence")
    if value[1] == 0.0:
        return BasePacket(
            valid=False,
            sequence=sequence,
            source_time_ns=0,
            position_xyz=(0.0, 0.0, 0.0),
            quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
            tracking_quality=0.0,
            flags=_exact_integer(value[13], "base flags", maximum=1 << 20),
            calibration_id=0,
            world_frame_id=0,
            reason="producer marked base packet invalid",
        )

    source_time_ns = _timestamp_ns(value[3], value[4], "base source stamp")
    quaternion = (value[8], value[9], value[10], value[11])
    quaternion_norm = math.sqrt(sum(component * component for component in quaternion))
    if quaternion_norm < 0.5 or quaternion_norm > 1.5:
        raise DecodeError("base quaternion norm is outside [0.5, 1.5]")
    quality = value[12]
    if quality < 0.0 or quality > 1.0:
        raise DecodeError("base tracking quality is outside [0, 1]")
    flags = _exact_integer(value[13], "base flags", maximum=1 << 20)
    if flags & REQUIRED_BASE_FLAGS != REQUIRED_BASE_FLAGS:
        raise DecodeError("base packet is missing required base flags")
    calibration_id = _exact_integer(value[14], "base calibration id", minimum=1)
    world_frame_id = _exact_integer(value[15], "base world-frame id", minimum=1)
    return BasePacket(
        valid=True,
        sequence=sequence,
        source_time_ns=source_time_ns,
        position_xyz=(value[5], value[6], value[7]),
        quaternion_wxyz=quaternion,
        tracking_quality=quality,
        flags=flags,
        calibration_id=calibration_id,
        world_frame_id=world_frame_id,
        reason="valid",
    )


def decode_racket_packet(values: Sequence[float]) -> RacketPacket:
    """Decode the frozen 19-double model_21800 racket-command schema."""

    value = _finite_values(values, RACKET_SCHEMA_V2_SIZE, "racket schema-2 packet")
    command_sequence = _exact_integer(value[14], "command sequence")
    if value[1] == 0.0:
        return RacketPacket(
            valid=False,
            swing_sign=0.0,
            position_xyz=(0.0, 0.0, 0.0),
            velocity_xyz=(0.0, 0.0, 0.0),
            published_time_to_strike_s=0.0,
            strike_deadline_wall_s=0.0,
            producer_time_ns=0,
            command_sequence=command_sequence,
            flight_id=0,
            revision_id=0,
            estimator_sample_count=0,
            estimator_span_s=0.0,
            reason="producer marked racket packet invalid",
        )

    if value[9] <= 0.0:
        raise DecodeError("published time-to-strike is not positive")
    if value[10] <= 0.0:
        raise DecodeError("strike deadline is not positive")
    producer_time_ns = _timestamp_ns(value[12], value[13], "command producer stamp")
    flight_id = _exact_integer(value[15], "flight id", minimum=1)
    revision_id = _exact_integer(value[16], "revision id", minimum=1)
    sample_count = _exact_integer(value[17], "estimator sample count")
    if value[18] < 0.0:
        raise DecodeError("estimator span is negative")
    return RacketPacket(
        valid=True,
        swing_sign=value[2],
        position_xyz=(value[3], value[4], value[5]),
        velocity_xyz=(value[6], value[7], value[8]),
        published_time_to_strike_s=value[9],
        strike_deadline_wall_s=value[10],
        producer_time_ns=producer_time_ns,
        command_sequence=command_sequence,
        flight_id=flight_id,
        revision_id=revision_id,
        estimator_sample_count=sample_count,
        estimator_span_s=value[18],
        reason="valid",
    )


def parse_session_id(text: str) -> str:
    value = str(text).strip()
    if SESSION_ID_PATTERN.fullmatch(value) is None:
        raise DecodeError("session id does not match model21800_YYYYMMDDTHHMMSSZ")
    return value


def parse_planner_attempt(text: str) -> str:
    value = str(text).strip()
    if PLANNER_ATTEMPT_PATTERN.fullmatch(value) is None:
        raise DecodeError("planner attempt does not match planner_attempt_NNN")
    return value


def parse_positive_pid(text: str) -> int:
    value = str(text).strip()
    if not value.isascii() or not value.isdecimal():
        raise DecodeError("planner pid is not a decimal integer")
    pid = int(value)
    if pid <= 0:
        raise DecodeError("planner pid is not positive")
    return pid


def process_cmdline_matches(cmdline: bytes, expected_fragment: str) -> bool:
    if not expected_fragment or not cmdline:
        return False
    decoded = cmdline.replace(b"\0", b" ").decode("utf-8", errors="replace")
    return expected_fragment in decoded


def parse_x_hit_status(text: str) -> XHitStatus:
    fields: dict[str, str] = {}
    for raw_line in str(text).splitlines():
        if not raw_line.strip():
            continue
        key, separator, value = raw_line.partition("=")
        if not separator or not key:
            raise DecodeError("x_hit status contains a malformed line")
        if key in fields:
            raise DecodeError(f"x_hit status repeats field {key!r}")
        fields[key] = value
    missing = {"request", "success", "message"} - fields.keys()
    if missing:
        raise DecodeError(f"x_hit status is missing fields: {', '.join(sorted(missing))}")
    if fields["success"] not in ("0", "1"):
        raise DecodeError("x_hit success is not 0 or 1")
    match = X_HIT_PATTERN.search(fields["message"])
    x_hit_m = float(match.group(1)) if match else None
    if x_hit_m is not None and not math.isfinite(x_hit_m):
        raise DecodeError("x_hit value is non-finite")
    return XHitStatus(
        request_id=fields["request"],
        success=fields["success"] == "1",
        message=fields["message"],
        x_hit_m=x_hit_m,
    )
