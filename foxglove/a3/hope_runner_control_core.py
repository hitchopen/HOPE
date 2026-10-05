"""ROS-free codec for the local Runner's fixed operator contract.

The browser never publishes this wire directly.  ``hope_command_proxy``
maps ten exact Trigger services to the fixed request array and waits for the
authoritative Runner state response.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from hope_observer_core import DecodeError


RUNNER_SCHEMA_VERSION = 2
RUNNER_REQUEST_SIZE = 4
RUNNER_STATE_SIZE = 21
MAX_EXACT_FLOAT_INTEGER = 1 << 52

ACTION_CODES = {
    "SET_SERVER": 1,
    "SET_RECEIVER": 2,
    "ENTER_PD_STAND": 3,
    "ENTER_MOTION": 4,
    "EMERGENCY_PASSIVE": 5,
    "PREPARE_SERVE": 7,
    "CONFIRM_BALL_LOADED": 8,
    "READY_TO_SERVE": 9,
    "OPEN_GRIPPER": 10,
    "CONFIRM_GRIP_SECURE": 12,
    "ENTER_TELEOP": 13,
}
ACTION_NAMES = {value: key for key, value in ACTION_CODES.items()}
ACTION_NAMES[0] = "NONE"
ACTION_NAMES[6] = "ENTER_SHADOW"

MODE_NAMES = {
    0: "PASSIVE",
    1: "PD_STAND",
    2: "SHADOW",
    3: "MOTION",
    4: "REFERENCE_PLAYBACK",
    5: "SERVE",
    6: "TELEOP",
}
ROLE_NAMES = {0: "UNASSIGNED", 1: "SERVER", 2: "RECEIVER"}
RESULT_NAMES = {
    0: "NONE",
    1: "APPLIED",
    2: "ALREADY_SET",
    3: "ACCEPTED_PENDING",
    4: "REJECTED_WRONG_MODE",
    5: "REJECTED_RUNNER_FAULT",
    6: "REJECTED_SERVE_ACTIVE",
    7: "INVALID_REQUEST",
    8: "QUEUE_FULL",
    9: "REJECTED_SERVE_UNAVAILABLE",
    10: "REJECTED_SERVE_NOT_READY",
    11: "REJECTED_GAIN_SCALE",
    12: "REJECTED_WRONG_ROLE",
    13: "REJECTED_CLEANUP_REQUIRED",
    14: "REJECTED_GRIPPER_STATE",
    15: "REJECTED_TELEOP",
}
REASON_NAMES = {
    0: "NONE",
    1: "ROLE_CHANGED",
    2: "ROLE_UNCHANGED",
    3: "MODE_CHANGED",
    4: "MODE_UNCHANGED",
    5: "SERVE_ABORT_REQUESTED",
    6: "ROLE_CHANGE_REQUIRES_PASSIVE_OR_PD_STAND",
    7: "RUNNER_COMMAND_FAULT_LATCHED",
    8: "SERVE_OWNS_COMMAND",
    9: "MALFORMED_REQUEST",
    10: "ACTION_QUEUE_FULL",
    11: "SERVE_PREPARE_REQUESTED",
    12: "BALL_LOADED_CONFIRM_REQUESTED",
    13: "SERVE_CONTROLLER_UNAVAILABLE",
    14: "SERVE_PHASE_MISMATCH",
    15: "SERVE_GAIN_SCALES_MUST_BE_ONE",
    16: "SERVE_FAULT_LATCHED",
    17: "SERVER_ROLE_REQUIRED",
    18: "RECEIVER_ROLE_REQUIRED",
    19: "GRIPPER_CLEANUP_REQUIRED",
    20: "READY_TO_SERVE_PLAY_REQUESTED",
    21: "GRIPPER_OPEN_REQUESTED",
    22: "LOADING_ZONE_CLEAR_REQUESTED",
    23: "PD_STAND_REQUIRED",
    24: "GRIPPER_MUST_BE_GRABBED",
    25: "GRIP_SECURE_CONFIRM_REQUESTED",
    26: "BALL_LOAD_MERGED_INTO_PREPARE",
    27: "LOADING_ZONE_CLEAR_REMOVED",
    28: "PURE_SERVE_ONLY",
    29: "TELEOP_UNAVAILABLE",
    30: "TELEOP_INPUT_NOT_READY",
    31: "TELEOP_STOP_REQUESTED",
}
SERVE_STATE_NAMES = {
    -1: "UNAVAILABLE",
    0: "IDLE",
    1: "PREPARING_STAND",
    2: "TRANSITION_TO_LOAD",
    3: "WAIT_BALL_LOAD",
    4: "GRIP_CLOSING",
    5: "WAIT_READY_TO_SERVE",
    6: "PLAYING_PRE_RELEASE",
    7: "RELEASE_PENDING",
    8: "STRIKE",
    9: "FOLLOW_THROUGH",
    10: "RECOVERY",
    11: "HANDOFF_READY",
    12: "COMPLETE",
    13: "ABORT_WAIT_HUMAN_CLEAR",
    14: "ABORT_RETURN",
    15: "ABORTED",
    16: "FAULT",
    17: "CLEANUP_OPENING",
    18: "WAIT_GRIP_SECURE",
}
GRIPPER_STATE_NAMES = {
    -1: "UNAVAILABLE",
    0: "UNKNOWN",
    1: "OPENING",
    2: "OPEN",
    3: "CLOSING",
    4: "GRABBED",
    5: "RELEASING",
    6: "RELEASED",
    7: "FAULT",
}
SUCCESS_RESULTS = frozenset({"APPLIED", "ALREADY_SET", "ACCEPTED_PENDING"})


@dataclass(frozen=True)
class RunnerState:
    boot_id: int
    state_sequence: int
    run_mode: str
    command_publishing: bool
    policy_native: bool
    command_fault_latched: bool
    local_role: str
    role_epoch: int
    role_change_allowed: bool
    role_last_result: str
    role_last_reason: str
    serve_capability: str
    serve_state: str
    gripper_state: str
    serve_cleanup_required: bool
    last_action_id: int
    last_action: str
    last_action_result: str
    last_action_reason: str
    session_fingerprint: int


def _exact_integer(
    value: float,
    label: str,
    *,
    minimum: int = 0,
    maximum: int = MAX_EXACT_FLOAT_INTEGER,
) -> int:
    if not math.isfinite(value):
        raise DecodeError(f"{label} is not finite")
    decoded = int(value)
    if float(decoded) != value or decoded < minimum or decoded > maximum:
        raise DecodeError(
            f"{label} must be an exact integer in [{minimum}, {maximum}]"
        )
    return decoded


def _enum_name(value: float, names: dict[int, str], label: str) -> str:
    minimum = min(names)
    maximum = max(names)
    decoded = _exact_integer(value, label, minimum=minimum, maximum=maximum)
    try:
        return names[decoded]
    except KeyError as exc:
        raise DecodeError(f"{label} has unknown code {decoded}") from exc


def _flag(value: float, label: str) -> bool:
    decoded = _exact_integer(value, label, maximum=1)
    return bool(decoded)


def encode_runner_request(request_id: int, action_name: str) -> list[float]:
    request_id = _exact_integer(
        float(request_id), "runner request id", minimum=1
    )
    try:
        action_code = ACTION_CODES[str(action_name)]
    except KeyError as exc:
        raise DecodeError(f"unsupported Runner action {action_name!r}") from exc
    return [
        float(RUNNER_SCHEMA_VERSION),
        float(request_id),
        float(action_code),
        0.0,
    ]


def decode_runner_state(values: Sequence[float]) -> RunnerState:
    try:
        decoded = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise DecodeError("Runner state contains a non-numeric value") from exc
    if len(decoded) != RUNNER_STATE_SIZE:
        raise DecodeError(
            f"Runner state has {len(decoded)} values; expected {RUNNER_STATE_SIZE}"
        )
    if not all(math.isfinite(value) for value in decoded):
        raise DecodeError("Runner state contains a non-finite value")
    if decoded[0] != float(RUNNER_SCHEMA_VERSION):
        raise DecodeError(
            f"Runner state schema is {decoded[0]!r}; expected {RUNNER_SCHEMA_VERSION}"
        )

    serve_capability = _flag(decoded[12], "serve capability")
    serve_state = _enum_name(decoded[13], SERVE_STATE_NAMES, "serve state")
    gripper_state = _enum_name(
        decoded[14], GRIPPER_STATE_NAMES, "gripper state"
    )
    cleanup_required = _flag(decoded[15], "serve cleanup required")
    if not serve_capability and serve_state != "UNAVAILABLE":
        raise DecodeError(
            "serve state must be UNAVAILABLE when serve capability is unavailable"
        )
    if serve_capability and serve_state == "UNAVAILABLE":
        raise DecodeError(
            "serve state cannot be UNAVAILABLE when serve capability is available"
        )
    if not serve_capability and gripper_state != "UNAVAILABLE":
        raise DecodeError(
            "gripper state must be UNAVAILABLE when serve capability is unavailable"
        )
    if not serve_capability and cleanup_required:
        raise DecodeError(
            "serve cleanup cannot be required when serve capability is unavailable"
        )

    return RunnerState(
        boot_id=_exact_integer(decoded[1], "Runner boot id", minimum=1),
        state_sequence=_exact_integer(
            decoded[2], "Runner state sequence", minimum=1
        ),
        run_mode=_enum_name(decoded[3], MODE_NAMES, "Runner mode"),
        command_publishing=_flag(decoded[4], "command publishing"),
        policy_native=_flag(decoded[5], "policy native"),
        command_fault_latched=_flag(decoded[6], "command fault"),
        local_role=_enum_name(decoded[7], ROLE_NAMES, "local role"),
        role_epoch=_exact_integer(decoded[8], "role epoch"),
        role_change_allowed=_flag(decoded[9], "role change allowed"),
        role_last_result=_enum_name(
            decoded[10], RESULT_NAMES, "role last result"
        ),
        role_last_reason=_enum_name(
            decoded[11], REASON_NAMES, "role last reason"
        ),
        serve_capability="AVAILABLE" if serve_capability else "UNAVAILABLE",
        serve_state=serve_state,
        gripper_state=gripper_state,
        serve_cleanup_required=cleanup_required,
        last_action_id=_exact_integer(decoded[16], "last action id"),
        last_action=_enum_name(decoded[17], ACTION_NAMES, "last action"),
        last_action_result=_enum_name(
            decoded[18], RESULT_NAMES, "last action result"
        ),
        last_action_reason=_enum_name(
            decoded[19], REASON_NAMES, "last action reason"
        ),
        session_fingerprint=_exact_integer(
            decoded[20], "session fingerprint", minimum=1
        ),
    )


def runner_session_fingerprint(session_id: str) -> int:
    value = 1469598103934665603
    for character in str(session_id).encode("utf-8"):
        value ^= character
        value = (value * 1099511628211) & ((1 << 64) - 1)
    value &= MAX_EXACT_FLOAT_INTEGER - 1
    return value or 1


def opponent_expected_role(local_role: str) -> str:
    if local_role == "SERVER":
        return "RECEIVER"
    if local_role == "RECEIVER":
        return "SERVER"
    return "UNKNOWN"


def action_succeeded(result: str) -> bool:
    return result in SUCCESS_RESULTS
