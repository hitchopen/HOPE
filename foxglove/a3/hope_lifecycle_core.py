"""ROS-free contracts for the fixed three-machine Runner lifecycle supervisor."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import fcntl
import ipaddress
import json
import os
from pathlib import Path
import re
import tempfile
from typing import IO, Mapping, Sequence


CONFIG_SCHEMA_VERSION = 2
NETWORK_CONFIG_FIELDS = (
    "laptop_wifi_ip",
    "hdu_wifi_ip",
    "mdu_internal_ip",
    "motive_ip",
)
REQUIRED_NETWORK_CONFIG_FIELDS = NETWORK_CONFIG_FIELDS[:-1]
CONFIG_FIELDS = (*NETWORK_CONFIG_FIELDS, "table_side")
PRIVATE_NETWORKS = tuple(
    ipaddress.IPv4Network(cidr) for cidr in ("10.0.0.0/8", "172.16.0.0/12", "192.168.0.0/16")
)
SESSION_PATTERN = re.compile(r"model21800_[0-9]{8}T[0-9]{6}Z")
HELPER_EVENT_PATTERN = re.compile(
    r"HOPE_LIFECYCLE_V1 step=([A-Z0-9_]+) state=([A-Z]+) reason=([A-Z0-9_]+)"
)
HARDWARE_OPERATION_LOCK_PATH = Path(
    "/var/lib/hope-lifecycle/hardware-operation.lock"
)


@dataclass(frozen=True)
class LifecycleConfig:
    # A fresh public install must not inherit one lab's network coordinates.
    # The operator confirms the three Runner transport addresses plus the
    # table-side selector through the fixed apply_config API. Motive is optional
    # metadata for an independently managed OptiTrack stack.
    laptop_wifi_ip: str = ""
    hdu_wifi_ip: str = ""
    mdu_internal_ip: str = ""
    motive_ip: str = ""
    table_side: str = "P1"
    revision: int = 0

    def values(self) -> dict[str, str]:
        return {name: str(getattr(self, name)) for name in CONFIG_FIELDS}


@dataclass(frozen=True)
class HelperEvent:
    step: str
    state: str
    reason: str


def try_acquire_hardware_operation_lock(
    path: Path = HARDWARE_OPERATION_LOCK_PATH,
) -> IO[str] | None:
    """Acquire the shared lifecycle/maintenance interlock without waiting."""

    stream = path.open("r+", encoding="ascii")
    try:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        stream.close()
        return None
    return stream


def release_hardware_operation_lock(stream: IO[str] | None) -> None:
    if stream is None:
        return
    try:
        fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
    finally:
        stream.close()


def validate_ipv4(name: str, value: object) -> str:
    if name not in NETWORK_CONFIG_FIELDS:
        raise ValueError(f"unsupported configuration field: {name}")
    if not isinstance(value, str):
        raise ValueError(f"{name} must be a string")
    if value != value.strip() or not value:
        raise ValueError(f"{name} must not contain surrounding whitespace")
    try:
        address = ipaddress.IPv4Address(value)
    except ipaddress.AddressValueError as exc:
        raise ValueError(f"{name} is not a valid IPv4 address") from exc
    if (
        address.is_unspecified
        or address.is_multicast
        or address.is_loopback
        or address.is_link_local
    ):
        raise ValueError(f"{name} is not a usable unicast IPv4 address")
    if not any(address in network for network in PRIVATE_NETWORKS):
        raise ValueError(f"{name} must be an RFC1918 private IPv4 address")
    return str(address)


def validate_table_side(value: object) -> str:
    if not isinstance(value, str):
        raise ValueError("table_side must be a string")
    if value not in {"P1", "P2"}:
        raise ValueError("table_side must be exactly P1 or P2")
    return value


def validate_config_value(name: str, value: object) -> str:
    if name == "motive_ip":
        if value == "":
            return ""
        return validate_ipv4(name, value)
    if name in REQUIRED_NETWORK_CONFIG_FIELDS:
        return validate_ipv4(name, value)
    if name == "table_side":
        return validate_table_side(value)
    raise ValueError(f"unsupported configuration field: {name}")


def apply_config_updates(
    current: LifecycleConfig,
    updates: Sequence[tuple[str, object]],
) -> LifecycleConfig:
    if len(updates) != len(CONFIG_FIELDS):
        raise ValueError(
            "configuration request must contain three required IPv4 fields, "
            "optional motive_ip, and table_side"
        )
    names = [name for name, _value in updates]
    if len(set(names)) != len(names):
        raise ValueError("configuration request contains duplicate fields")
    if set(names) != set(CONFIG_FIELDS):
        unknown = sorted(set(names) - set(CONFIG_FIELDS))
        missing = sorted(set(CONFIG_FIELDS) - set(names))
        detail = []
        if unknown:
            detail.append(f"unknown={','.join(unknown)}")
        if missing:
            detail.append(f"missing={','.join(missing)}")
        raise ValueError("configuration fields mismatch: " + " ".join(detail))
    validated = {
        name: validate_config_value(name, value) for name, value in updates
    }
    return LifecycleConfig(**validated, revision=current.revision + 1)


def config_to_document(config: LifecycleConfig) -> dict[str, object]:
    return {"schema_version": CONFIG_SCHEMA_VERSION, **asdict(config)}


def config_from_document(document: Mapping[str, object]) -> LifecycleConfig:
    schema_version = document.get("schema_version")
    if schema_version not in {1, CONFIG_SCHEMA_VERSION}:
        raise ValueError("unsupported lifecycle configuration schema")
    revision = document.get("revision")
    if not isinstance(revision, int) or isinstance(revision, bool) or revision < 1:
        raise ValueError("configuration revision must be a positive integer")
    values = {
        name: validate_config_value(name, document.get(name))
        for name in NETWORK_CONFIG_FIELDS
    }
    # A deployed schema-1 configuration predates table-side selection and is
    # therefore exactly the historical P1/direct mode.  The next confirmation
    # writes schema 2 explicitly.
    values["table_side"] = (
        "P1"
        if schema_version == 1
        else validate_table_side(document.get("table_side"))
    )
    return LifecycleConfig(**values, revision=revision)


def load_config(path: Path) -> LifecycleConfig:
    if not path.exists():
        return LifecycleConfig()
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("lifecycle configuration must be a JSON object")
    return config_from_document(document)


def save_config_atomic(path: Path, config: LifecycleConfig) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", dir=str(path.parent), text=True
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(config_to_document(config), stream, sort_keys=True, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, 0o600)
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def validate_session_id(value: str) -> str:
    if SESSION_PATTERN.fullmatch(value) is None:
        raise ValueError("invalid model21800 session id")
    return value


def parse_helper_event(line: str) -> HelperEvent | None:
    match = HELPER_EVENT_PATTERN.fullmatch(line.strip())
    if match is None:
        return None
    return HelperEvent(step=match.group(1), state=match.group(2), reason=match.group(3))
