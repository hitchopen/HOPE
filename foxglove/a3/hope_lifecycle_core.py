"""ROS-free contracts for the fixed three-machine Runner lifecycle supervisor."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import ipaddress
import json
import os
from pathlib import Path
import re
import tempfile
from typing import Mapping, Sequence


CONFIG_SCHEMA_VERSION = 1
CONFIG_FIELDS = (
    "laptop_wifi_ip",
    "hdu_wifi_ip",
    "mdu_internal_ip",
    "motive_ip",
)
PRIVATE_NETWORKS = tuple(
    ipaddress.IPv4Network(cidr) for cidr in ("10.0.0.0/8", "172.16.0.0/12", "192.168.0.0/16")
)
SESSION_PATTERN = re.compile(r"model21800_[0-9]{8}T[0-9]{6}Z")
HELPER_EVENT_PATTERN = re.compile(
    r"HOPE_LIFECYCLE_V1 step=([A-Z0-9_]+) state=([A-Z]+) reason=([A-Z0-9_]+)"
)


@dataclass(frozen=True)
class LifecycleConfig:
    laptop_wifi_ip: str = "172.23.20.46"
    hdu_wifi_ip: str = "172.23.20.135"
    mdu_internal_ip: str = "10.42.10.12"
    motive_ip: str = "192.168.100.111"
    revision: int = 0

    def values(self) -> dict[str, str]:
        return {name: str(getattr(self, name)) for name in CONFIG_FIELDS}


@dataclass(frozen=True)
class HelperEvent:
    step: str
    state: str
    reason: str


def validate_ipv4(name: str, value: object) -> str:
    if name not in CONFIG_FIELDS:
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


def apply_config_updates(
    current: LifecycleConfig,
    updates: Sequence[tuple[str, object]],
) -> LifecycleConfig:
    if len(updates) != len(CONFIG_FIELDS):
        raise ValueError("configuration request must contain all four IPv4 fields")
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
    validated = {name: validate_ipv4(name, value) for name, value in updates}
    return LifecycleConfig(**validated, revision=current.revision + 1)


def config_to_document(config: LifecycleConfig) -> dict[str, object]:
    return {"schema_version": CONFIG_SCHEMA_VERSION, **asdict(config)}


def config_from_document(document: Mapping[str, object]) -> LifecycleConfig:
    if document.get("schema_version") != CONFIG_SCHEMA_VERSION:
        raise ValueError("unsupported lifecycle configuration schema")
    revision = document.get("revision")
    if not isinstance(revision, int) or isinstance(revision, bool) or revision < 1:
        raise ValueError("configuration revision must be a positive integer")
    values = {
        name: validate_ipv4(name, document.get(name)) for name in CONFIG_FIELDS
    }
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
