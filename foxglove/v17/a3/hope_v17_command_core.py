"""Narrow file-contract helpers for explicit V17 Foxglove actions."""

from __future__ import annotations

import os
from pathlib import Path
import re
import time
from typing import Callable

from hope_v17_observer_core import DecodeError, XHitStatus, parse_x_hit_status


REQUEST_ID_PATTERN = re.compile(r"[0-9]{1,32}")


def validate_request_id(request_id: str) -> str:
    value = str(request_id).strip()
    if REQUEST_ID_PATTERN.fullmatch(value) is None:
        raise DecodeError("request id must contain 1 to 32 decimal digits")
    return value


def publish_x_hit_request(request_path: Path, request_id: str) -> None:
    """Publish one fully written request without overwriting an existing one.

    The temporary file is closed and fsynced before an atomic hard-link makes
    the request visible to Planner.  ``os.link`` fails if ``request_path``
    already exists, so concurrent/stale requests are never replaced.
    """

    request_path = Path(request_path)
    request_id = validate_request_id(request_id)
    if not request_path.parent.is_dir():
        raise FileNotFoundError(
            f"x_hit request directory does not exist: {request_path.parent}"
        )
    temporary = request_path.with_name(
        f".{request_path.name}.{os.getpid()}.{request_id}.tmp"
    )
    descriptor = os.open(
        temporary,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC,
        0o600,
    )
    try:
        payload = f"{request_id}\n".encode("ascii")
        offset = 0
        while offset < len(payload):
            written = os.write(descriptor, payload[offset:])
            if written <= 0:
                raise OSError("short write while preparing x_hit request")
            offset += written
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    try:
        os.link(temporary, request_path)
    finally:
        temporary.unlink(missing_ok=True)


def wait_for_x_hit_status(
    status_path: Path,
    request_id: str,
    *,
    timeout_s: float,
    poll_period_s: float = 0.05,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> XHitStatus:
    request_id = validate_request_id(request_id)
    timeout_s = float(timeout_s)
    poll_period_s = float(poll_period_s)
    if timeout_s < 0.0:
        raise ValueError("timeout_s must not be negative")
    if poll_period_s <= 0.0:
        raise ValueError("poll_period_s must be positive")
    deadline = monotonic() + timeout_s
    last_error = "status file not seen"
    while True:
        try:
            status = parse_x_hit_status(Path(status_path).read_text(encoding="utf-8"))
            if status.request_id == request_id:
                return status
            last_error = f"latest status belongs to request {status.request_id!r}"
        except (OSError, DecodeError) as exc:
            last_error = str(exc)
        now = monotonic()
        if now >= deadline:
            raise TimeoutError(
                f"no matching x_hit status for request {request_id}: {last_error}"
            )
        sleep(min(poll_period_s, deadline - now))
