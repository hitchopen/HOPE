#!/usr/bin/env python3
"""Pre-launch NatNet checks against Motive; ROS is not required.

The probe separates three failures that otherwise all look like silent ROS
topics: no NAT_SERVERINFO, no masked NAT_MODELDEF, and no FRAMEOFDATA.  It
also checks that the model definition contains the HOPE ball (`Ball` or the
field alias `ball`) and selected UCB_P1/UCB_P2 robot asset.

Exit status 0 means the NatNet side is ready for the patched bridge.  Status 1
means at least one launch blocker was found.
"""

import argparse
import re
import socket
import struct
import subprocess
import sys
import time
from typing import Dict, List, Optional, Tuple


NAT_CONNECT = 0
NAT_SERVERINFO = 1
NAT_REQUEST_MODELDEF = 4
NAT_MODELDEF = 5
NAT_FRAMEOFDATA = 7

# Keep this identical to MODELDEF_TYPES in libmotioncapture/src/optitrack.cpp.
# bit 0 = MarkerSet, bit 1 = RigidBody.
MODELDEF_TYPES = 0x3

SERVERINFO_MIN_LEN = 283
MAX_PACKET_SIZE = 65535

# Keep this aligned with hope_bringup/config/optitrack_mct.yaml. Motive asset
# spelling is normalized only at the mocap boundary; every ROS consumer keeps
# the canonical `Ball` contract.
DEFAULT_ASSET_ALIAS_ENTRIES = ("ball=Ball",)


def packet_message_id(packet: bytes) -> Optional[int]:
    if len(packet) < 2:
        return None
    return struct.unpack_from("<H", packet)[0]


def packet_is_complete(packet: bytes, minimum_len: int = 4) -> bool:
    if len(packet) < max(4, minimum_len):
        return False
    payload_len = struct.unpack_from("<H", packet, 2)[0]
    return payload_len <= len(packet) - 4


def route_of(destination: str) -> Tuple[Optional[str], Optional[str]]:
    """Return the (device, source IP) selected by the kernel."""
    try:
        result = subprocess.run(
            ["ip", "-o", "route", "get", destination],
            capture_output=True,
            check=False,
            text=True,
            timeout=5,
        )
    except (OSError, subprocess.SubprocessError):
        return None, None
    device = re.search(r"\bdev (\S+)", result.stdout)
    source = re.search(r"\bsrc (\S+)", result.stdout)
    return (
        device.group(1) if device else None,
        source.group(1) if source else None,
    )


def receive_expected(
    sock: socket.socket, message_id: int, wait: float, minimum_len: int
) -> Optional[bytes]:
    """Receive one complete NatNet packet before a monotonic deadline."""
    deadline = time.monotonic() + wait
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        sock.settimeout(remaining)
        try:
            packet, _ = sock.recvfrom(MAX_PACKET_SIZE)
        except socket.timeout:
            return None
        if (
            packet_message_id(packet) == message_id
            and packet_is_complete(packet, minimum_len)
        ):
            return packet


def natnet_connect(
    host: str, port: int, wait: float = 3.0
) -> Tuple[socket.socket, Optional[bytes]]:
    """Send NAT_CONNECT and retain its socket for unicast frame reception."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
    sock.bind(("0.0.0.0", 0))
    sock.sendto(struct.pack("<HH", NAT_CONNECT, 0), (host, port))
    return sock, receive_expected(sock, NAT_SERVERINFO, wait, SERVERINFO_MIN_LEN)


def ask_modeldef(
    host: str, port: int, payload: bytes, wait: float = 3.0
) -> Optional[bytes]:
    """Perform one MODELDEF round trip on a fresh command connection."""
    sock, server_info = natnet_connect(host, port, wait)
    try:
        if server_info is None:
            return None
        request = struct.pack("<HH", NAT_REQUEST_MODELDEF, len(payload)) + payload
        sock.sendto(request, (host, port))
        return receive_expected(sock, NAT_MODELDEF, wait, 8)
    finally:
        sock.close()


def modeldef_contract_blocker(
    masked: Optional[bytes], bare: Optional[bytes]
) -> Optional[str]:
    """Return why the patched driver's exact request contract would fail."""
    if masked is not None:
        return None
    if bare is not None:
        return (
            "Motive answers only payload-less MODELDEF, but this bridge sends "
            "the required 0x3 descriptor mask"
        )
    return (
        "Motive returns no model definition; toggle Broadcast Frame Data "
        "off/on, then restart Motive"
    )




def parse_asset_aliases(entries: List[str]) -> Dict[str, str]:
    """Parse unique SOURCE=CANONICAL asset aliases."""
    aliases: Dict[str, str] = {}
    canonical_names = set()
    for entry in entries:
        if entry.count("=") != 1:
            raise ValueError(
                f"asset alias must be SOURCE=CANONICAL: {entry!r}"
            )
        source, canonical = entry.split("=", 1)
        if not source or not canonical or source == canonical:
            raise ValueError(
                f"asset alias must have distinct non-empty names: {entry!r}"
            )
        if source in aliases:
            raise ValueError(f"duplicate asset alias source: {source!r}")
        if canonical in canonical_names:
            raise ValueError(
                f"duplicate asset alias canonical name: {canonical!r}"
            )
        aliases[source] = canonical
        canonical_names.add(canonical)
    return aliases


def canonical_assets(
    assets: List[str], aliases: Dict[str, str]
) -> List[str]:
    return sorted({aliases.get(asset, asset) for asset in assets})


def robot_asset_for_table_side(table_side: str) -> str:
    if table_side not in {"P1", "P2"}:
        raise ValueError(f"unsupported table side: {table_side!r}")
    return f"UCB_{table_side}"


def _packet_limit(packet: bytes) -> int:
    if not packet_is_complete(packet):
        raise ValueError("truncated NatNet packet")
    return 4 + struct.unpack_from("<H", packet, 2)[0]


def _read_i32(packet: bytes, offset: int, limit: int) -> Tuple[int, int]:
    if offset + 4 > limit:
        raise ValueError("truncated NatNet int32")
    return struct.unpack_from("<i", packet, offset)[0], offset + 4


def _read_c_string(packet: bytes, offset: int, limit: int) -> Tuple[str, int]:
    end = packet.find(b"\x00", offset, limit)
    if end < 0:
        raise ValueError("unterminated NatNet string")
    return packet[offset:end].decode(errors="replace"), end + 1


def frame_uses_sized_sections(
    packet: bytes, major: int, minor: int
) -> bool:
    """Infer whether this FRAMEOFDATA uses 4.1+ section byte counts.

    Motive 3.5.0.1 can advertise NatNet 4.5 while multicasting the legacy
    unsized frame layout. A non-empty marker-set section makes the layouts
    unambiguous: a valid byte count is bounded by the packet, while an asset
    name's first four bytes are not.
    """
    if not (major > 4 or (major == 4 and minor >= 1)):
        return False
    limit = _packet_limit(packet)
    marker_set_count, offset = _read_i32(packet, 8, limit)
    if marker_set_count < 0 or marker_set_count > 100000:
        raise ValueError("invalid NatNet marker-set count")
    if marker_set_count == 0:
        return True
    candidate_size, _ = _read_i32(packet, offset, limit)
    return 0 <= candidate_size <= limit - offset - 4


def _unsized_modeldef_ids(
    packet: bytes, major: int, minor: int, aliases: Dict[str, str]
) -> Dict[str, int]:
    """Return canonical rigid-body name -> ID from MODELDEF descriptions.

    NatNet 4.1+ FRAMEOFDATA sections carry byte counts, but MODELDEF
    descriptions remain unsized.  In particular, Motive 3.5.0.1 / NatNet
    4.5.0.0 places the description directly after the dataset type.
    """
    limit = _packet_limit(packet)
    dataset_count, offset = _read_i32(packet, 4, limit)
    if dataset_count < 0 or dataset_count > 10000:
        raise ValueError("invalid NatNet MODELDEF dataset count")
    raw_result: Dict[str, int] = {}
    for dataset_index in range(dataset_count):
        dataset_type, offset = _read_i32(packet, offset, limit)
        if dataset_type == 0:
            _, offset = _read_c_string(packet, offset, limit)
            marker_count, offset = _read_i32(packet, offset, limit)
            if marker_count < 0 or marker_count > 100000:
                raise ValueError("invalid NatNet marker-set marker count")
            for _ in range(marker_count):
                _, offset = _read_c_string(packet, offset, limit)
        elif dataset_type == 1:
            name, offset = _read_c_string(packet, offset, limit)
            rigid_body_id, offset = _read_i32(packet, offset, limit)
            # parent ID + parent translation
            if offset + 16 > limit:
                raise ValueError("truncated NatNet rigid-body parent transform")
            offset += 16
            existing = raw_result.get(name)
            if existing is not None and existing != rigid_body_id:
                raise ValueError(
                    f"duplicate rigid-body name {name!r}"
                )
            raw_result[name] = rigid_body_id
            if major >= 3:
                marker_count, offset = _read_i32(packet, offset, limit)
                if marker_count < 0 or marker_count > 10000:
                    raise ValueError("invalid NatNet rigid-body marker count")
                marker_array_bytes = marker_count * 16
                if offset + marker_array_bytes > limit:
                    raise ValueError("truncated NatNet rigid-body marker arrays")
                offset += marker_array_bytes
                if major >= 4:
                    for _ in range(marker_count):
                        _, offset = _read_c_string(packet, offset, limit)
        else:
            raise ValueError(
                "unsupported NatNet MODELDEF dataset type "
                f"{dataset_type} at index {dataset_index}"
            )
    if offset != limit:
        raise ValueError(
            f"unexpected trailing NatNet MODELDEF bytes: {limit - offset}"
        )
    result: Dict[str, int] = {}
    for name, rigid_body_id in raw_result.items():
        canonical_name = aliases.get(name, name)
        # Match the C++ adapter: if both spellings exist, the exact canonical
        # Motive asset is authoritative and the alias is ignored.
        if name != canonical_name and canonical_name in raw_result:
            continue
        existing = result.get(canonical_name)
        if existing is not None and existing != rigid_body_id:
            raise ValueError(
                f"duplicate canonical rigid-body name {canonical_name!r}"
            )
        result[canonical_name] = rigid_body_id
    return result


def frame_rigid_body_tracking(
    packet: bytes, major: int, minor: int
) -> Dict[int, bool]:
    """Extract rigid-body tracking-valid flags from one FRAMEOFDATA packet."""
    limit = _packet_limit(packet)
    if packet_message_id(packet) != NAT_FRAMEOFDATA:
        raise ValueError("packet is not FRAMEOFDATA")
    sized_sections = frame_uses_sized_sections(packet, major, minor)
    offset = 8  # packet header + frame number

    marker_set_count, offset = _read_i32(packet, offset, limit)
    if marker_set_count < 0 or marker_set_count > 100000:
        raise ValueError("invalid NatNet marker-set count")
    if sized_sections:
        _, offset = _read_i32(packet, offset, limit)
    for _ in range(marker_set_count):
        _, offset = _read_c_string(packet, offset, limit)
        marker_count, offset = _read_i32(packet, offset, limit)
        marker_bytes = marker_count * 12
        if marker_count < 0 or offset + marker_bytes > limit:
            raise ValueError("invalid NatNet marker-set payload")
        offset += marker_bytes

    other_marker_count, offset = _read_i32(packet, offset, limit)
    if sized_sections:
        _, offset = _read_i32(packet, offset, limit)
    other_marker_bytes = other_marker_count * 12
    if other_marker_count < 0 or offset + other_marker_bytes > limit:
        raise ValueError("invalid NatNet unlabeled-marker payload")
    offset += other_marker_bytes

    rigid_body_count, offset = _read_i32(packet, offset, limit)
    if sized_sections:
        _, offset = _read_i32(packet, offset, limit)
    if rigid_body_count < 0 or rigid_body_count > 100000:
        raise ValueError("invalid NatNet rigid-body count")
    result: Dict[int, bool] = {}
    for _ in range(rigid_body_count):
        rigid_body_id, offset = _read_i32(packet, offset, limit)
        fixed_pose_bytes = 7 * 4
        if offset + fixed_pose_bytes > limit:
            raise ValueError("truncated NatNet rigid-body pose")
        offset += fixed_pose_bytes
        if major >= 2:
            if offset + 4 > limit:
                raise ValueError("truncated NatNet rigid-body mean error")
            offset += 4
        has_params = major > 2 or (major == 2 and minor >= 6) or major == 0
        if has_params:
            if offset + 2 > limit:
                raise ValueError("truncated NatNet rigid-body params")
            params = struct.unpack_from("<H", packet, offset)[0]
            offset += 2
            result[rigid_body_id] = bool(params & 0x01)
        else:
            result[rigid_body_id] = True
    return result


def frame_labeled_marker_model_counts(
    packet: bytes, major: int, minor: int
) -> Tuple[int, Dict[int, int]]:
    """Return total labeled markers and per-model physical unique counts.

    Sized and legacy-unsized FRAMEOFDATA layouts are both accepted. This lets
    the preflight inspect labeled samples without treating deprecated
    MarkerSet positions as physical marker receipts.
    """
    limit = _packet_limit(packet)
    if packet_message_id(packet) != NAT_FRAMEOFDATA:
        raise ValueError("packet is not FRAMEOFDATA")
    offset = 8  # packet header + frame number
    sized_sections = frame_uses_sized_sections(packet, major, minor)
    if sized_sections:
        for section_name in (
            "marker-set",
            "unlabeled-marker",
            "rigid-body",
            "skeleton",
            "asset",
        ):
            section_count, offset = _read_i32(packet, offset, limit)
            section_size, offset = _read_i32(packet, offset, limit)
            if section_count < 0 or section_count > 100000:
                raise ValueError(f"invalid NatNet {section_name} count")
            if section_size < 0 or offset + section_size > limit:
                raise ValueError(f"invalid NatNet {section_name} section size")
            offset += section_size
    else:
        marker_set_count, offset = _read_i32(packet, offset, limit)
        if marker_set_count < 0 or marker_set_count > 100000:
            raise ValueError("invalid NatNet marker-set count")
        for _ in range(marker_set_count):
            _, offset = _read_c_string(packet, offset, limit)
            marker_count, offset = _read_i32(packet, offset, limit)
            marker_bytes = marker_count * 12
            if marker_count < 0 or offset + marker_bytes > limit:
                raise ValueError("invalid NatNet marker-set payload")
            offset += marker_bytes

        other_marker_count, offset = _read_i32(packet, offset, limit)
        other_marker_bytes = other_marker_count * 12
        if other_marker_count < 0 or offset + other_marker_bytes > limit:
            raise ValueError("invalid NatNet unlabeled-marker payload")
        offset += other_marker_bytes

        rigid_body_count, offset = _read_i32(packet, offset, limit)
        rigid_body_bytes = rigid_body_count * 38
        if rigid_body_count < 0 or offset + rigid_body_bytes > limit:
            raise ValueError("invalid NatNet rigid-body payload")
        offset += rigid_body_bytes

        skeleton_count, offset = _read_i32(packet, offset, limit)
        if skeleton_count != 0:
            raise ValueError(
                "unsized NatNet skeleton payload is unsupported by preflight"
            )

    labeled_count, offset = _read_i32(packet, offset, limit)
    if sized_sections:
        labeled_size, offset = _read_i32(packet, offset, limit)
    else:
        marker_bytes = 4 + 4 * 4 + 2 + (4 if major >= 3 else 0)
        labeled_size = labeled_count * marker_bytes
    if labeled_count < 0 or labeled_count > 100000:
        raise ValueError("invalid NatNet labeled-marker count")
    labeled_end = offset + labeled_size
    if labeled_size < 0 or labeled_end > limit:
        raise ValueError("invalid NatNet labeled-marker section size")

    physical_members: Dict[int, set[int]] = {}
    marker_bytes = 4 + 4 * 4  # ID + xyz + size
    if major > 2 or (major == 2 and minor >= 6):
        marker_bytes += 2  # params
    if major >= 3:
        marker_bytes += 4  # residual
    for _ in range(labeled_count):
        if offset + marker_bytes > labeled_end:
            raise ValueError("truncated NatNet labeled-marker payload")
        marker_id = struct.unpack_from("<I", packet, offset)[0]
        model_id = (marker_id >> 16) & 0xFFFF
        member_id = marker_id & 0xFFFF
        params = struct.unpack_from("<H", packet, offset + 20)[0]
        # Match rigid_body_marker_association.hpp: an occluded or purely
        # model-filled position must not satisfy a physical calibration gate.
        if (params & 0x01) == 0 and (params & 0x02) != 0:
            physical_members.setdefault(model_id, set()).add(member_id)
        offset += marker_bytes
    if offset != labeled_end:
        raise ValueError("unexpected NatNet labeled-marker payload size")
    return labeled_count, {
        model_id: len(member_ids)
        for model_id, member_ids in physical_members.items()
    }


def count_frames_with_tracking(
    sock: socket.socket,
    seconds: float,
    tracked_ids: set[int],
    major: int,
    minor: int,
) -> Tuple[
    int,
    float,
    int,
    Dict[int, int],
    Dict[int, int],
    Dict[int, int],
    Dict[int, int],
    int,
    int,
]:
    """Count frames plus per-ID presence/tracking-valid samples."""
    previous = None
    frames = 0
    missing = 0
    seen = {rigid_body_id: 0 for rigid_body_id in tracked_ids}
    valid = {rigid_body_id: 0 for rigid_body_id in tracked_ids}
    marker_peak = {rigid_body_id: 0 for rigid_body_id in tracked_ids}
    marker_frames = {rigid_body_id: 0 for rigid_body_id in tracked_ids}
    tracking_parse_errors = 0
    marker_parse_errors = 0
    started = time.monotonic()
    deadline = started + seconds
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        sock.settimeout(min(3.0, remaining))
        try:
            packet, _ = sock.recvfrom(MAX_PACKET_SIZE)
        except socket.timeout:
            break
        if (
            packet_message_id(packet) != NAT_FRAMEOFDATA
            or not packet_is_complete(packet, 8)
        ):
            continue
        frame_number = struct.unpack_from("<I", packet, 4)[0]
        frames += 1
        if previous is not None:
            delta = (frame_number - previous) & 0xFFFFFFFF
            if 1 < delta < 0x80000000:
                missing += delta - 1
        previous = frame_number
        if tracked_ids:
            try:
                tracking = frame_rigid_body_tracking(
                    packet, major, minor
                )
            except ValueError:
                tracking_parse_errors += 1
            else:
                for rigid_body_id in tracked_ids:
                    if rigid_body_id in tracking:
                        seen[rigid_body_id] += 1
                        if tracking[rigid_body_id]:
                            valid[rigid_body_id] += 1
            if major > 4 or (major == 4 and minor >= 1):
                try:
                    _, marker_counts = frame_labeled_marker_model_counts(
                        packet, major, minor
                    )
                except ValueError:
                    marker_parse_errors += 1
                else:
                    for rigid_body_id in tracked_ids:
                        count = marker_counts.get(rigid_body_id, 0)
                        marker_peak[rigid_body_id] = max(
                            marker_peak[rigid_body_id], count
                        )
                        if count > 0:
                            marker_frames[rigid_body_id] += 1
    elapsed = max(1e-6, time.monotonic() - started)
    return (
        frames,
        frames / elapsed,
        missing,
        seen,
        valid,
        marker_peak,
        marker_frames,
        tracking_parse_errors,
        marker_parse_errors,
    )


def count_frames(
    sock: socket.socket, seconds: float
) -> Tuple[int, float, int]:
    """Return received frame count, receive rate, and forward sequence gaps."""
    frames, hz, missing, _, _, _, _, _, _ = count_frames_with_tracking(
        sock, seconds, set(), 0, 0
    )
    return frames, hz, missing


def multicast_socket(group: str, port: int, interface_ip: str) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
    sock.bind(("0.0.0.0", port))
    membership = socket.inet_aton(group) + socket.inet_aton(interface_ip)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
    return sock


NAT_ECHOREQUEST = 12
NAT_ECHORESPONSE = 13
EXPECTED_APP_NAMES = ("Motive", "MotiveBody")
EXPECTED_APP_VERSION = (3, 5, 0, 1)
EXPECTED_NATNET_VERSION_PREFIX = (4, 5)

def competition_version_warnings(app: str, app_version, nat_version):
    """Report deviations from the validated profile without rejecting them."""
    warnings = []
    app_version = tuple(app_version)
    nat_version = tuple(nat_version)
    if app not in EXPECTED_APP_NAMES or app_version != EXPECTED_APP_VERSION:
        warnings.append('validated Motive profile is MotiveBody 3.5.0.1 Beta '
                        '1; server reports %s %s; continuing because software '
                        'versions are advisory' %
                        (app, '.'.join(str(b) for b in app_version)))
    if nat_version[:2] != EXPECTED_NATNET_VERSION_PREFIX:
        warnings.append('validated profile is NatNet 4.5.x; server reports %s; '
                        'continuing because the live decode and stream checks '
                        'are authoritative' %
                        '.'.join(str(b) for b in nat_version))
    return warnings

def clock_sync_samples(host: str, port: int, count: int = 10):
    """Return NatNet echo RTTs, or fewer entries when responses are missing."""
    sock, info = natnet_connect(host, port)
    samples = []
    server_ip = socket.gethostbyname(host)
    try:
        if info is None:
            return samples
        sock.settimeout(0.1)
        for _ in range(count):
            token = time.monotonic_ns()
            sent = time.monotonic()
            sock.sendto(struct.pack('<HHQ', NAT_ECHOREQUEST, 8, token),
                        (host, port))
            deadline = sent + 0.1
            while time.monotonic() < deadline:
                try:
                    data, sender = sock.recvfrom(65535)
                except socket.timeout:
                    break
                received = time.monotonic()
                if (sender[0] != server_ip or len(data) < 20
                        or packet_message_id(data) != NAT_ECHORESPONSE):
                    continue
                message_id, payload_size, echoed, server_ticks = \
                    struct.unpack_from('<HHQQ', data)
                if (message_id == NAT_ECHORESPONSE and payload_size >= 16
                        and echoed == token):
                    samples.append((received - sent, server_ticks))
                    break
        return samples
    finally:
        sock.close()

class ModelDefReader:
    """Length-checked little-endian reader for one MODELDEF region."""

    def __init__(self, packet: bytes, start: int, end: int):
        if start < 0 or end < start or end > len(packet):
            raise ValueError('invalid MODELDEF bounds')
        self.packet = packet
        self.offset = start
        self.end = end

    def unpack(self, fmt: str, field: str):
        size = struct.calcsize('<' + fmt)
        if self.offset + size > self.end:
            raise ValueError('MODELDEF truncated at %s' % field)
        values = struct.unpack_from('<' + fmt, self.packet, self.offset)
        self.offset += size
        return values[0] if len(values) == 1 else values

    def string(self, field: str):
        end = self.packet.find(b'\x00', self.offset, self.end)
        if end < 0:
            raise ValueError('MODELDEF unterminated string at %s' % field)
        value = self.packet[self.offset:end].decode(errors='replace')
        self.offset = end + 1
        return value

    def skip(self, size: int, field: str):
        if size < 0 or self.offset + size > self.end:
            raise ValueError('MODELDEF truncated at %s' % field)
        self.offset += size

def _sized_modeldef_ids(packet: bytes, nat_major: int, nat_minor: int):
    """Decode rigid-body names using the sized NatNet 4.1+ MODELDEF schema.

    NatNet 4.5 IMU/GPIO/anchor descriptions are intentionally skipped using
    description_size. Keep this parser and the C++ natnet_modeldef.h fixtures
    updated together whenever OptiTrack changes the schema.
    """
    if len(packet) < 8:
        raise ValueError('MODELDEF is shorter than its header')
    message_id, payload_size = struct.unpack_from('<HH', packet)
    if message_id != NAT_MODELDEF:
        raise ValueError('packet is not MODELDEF')
    if payload_size > len(packet) - 4:
        raise ValueError('MODELDEF payload exceeds datagram')

    reader = ModelDefReader(packet, 4, 4 + payload_size)
    dataset_count = reader.unpack('i', 'dataset count')
    if dataset_count < 0 or dataset_count > 10000:
        raise ValueError('invalid MODELDEF dataset count')
    sized = nat_major > 4 or (nat_major == 4 and nat_minor >= 1)
    rotation_offset = nat_major > 4 or (nat_major == 4 and nat_minor >= 2)
    assets = {}

    for _ in range(dataset_count):
        dataset_type = reader.unpack('i', 'dataset type')
        dataset_end = reader.end
        if sized:
            description_size = reader.unpack('i', 'description size')
            if description_size < 0 or reader.offset + description_size > reader.end:
                raise ValueError('invalid MODELDEF description size')
            dataset_end = reader.offset + description_size
        dataset = ModelDefReader(packet, reader.offset, dataset_end)

        if dataset_type == 0:
            dataset.string('marker-set name')
            marker_count = dataset.unpack('i', 'marker-set marker count')
            if marker_count < 0 or marker_count > 10000:
                raise ValueError('invalid marker-set marker count')
            for _ in range(marker_count):
                dataset.string('marker-set marker name')
        elif dataset_type == 1:
            name = dataset.string('rigid-body name') if nat_major >= 2 else ''
            rigid_body_id = dataset.unpack('i', 'rigid-body id')
            if name in assets and assets[name] != rigid_body_id:
                raise ValueError('duplicate rigid-body name')
            assets[name] = rigid_body_id
            dataset.unpack('i', 'rigid-body parent id')
            dataset.skip(3 * 4, 'rigid-body position offset')
            if rotation_offset:
                dataset.skip(4 * 4, 'rigid-body rotation offset')
            if nat_major >= 3:
                marker_count = dataset.unpack('i', 'rigid-body marker count')
                if marker_count < 0 or marker_count > 10000:
                    raise ValueError('invalid rigid-body marker count')
                dataset.skip(marker_count * 3 * 4, 'marker positions')
                dataset.skip(marker_count * 4, 'marker active labels')
                if nat_major >= 4:
                    for _ in range(marker_count):
                        dataset.string('rigid-body marker name')
        elif not sized:
            raise ValueError('unsupported unsized MODELDEF dataset type %d'
                             % dataset_type)

        reader.offset = dataset_end if sized else dataset.offset

    if reader.offset != reader.end:
        raise ValueError("unexpected trailing MODELDEF bytes")
    return assets

def modeldef_rigid_body_ids(packet: bytes, major: int, minor: int,
                            aliases: Dict[str, str]) -> Dict[str, int]:
    # Both layouts occur in advertised NatNet 4.5 streams. Each parser must
    # validate its complete datagram; never accept a regex name scan.
    try:
        raw = _sized_modeldef_ids(packet, major, minor)
    except ValueError:
        raw = _unsized_modeldef_ids(packet, major, minor, {})
    result = {}
    for name, identifier in raw.items():
        canonical = aliases.get(name, name)
        if canonical != name and canonical in raw:
            continue
        if canonical in result and result[canonical] != identifier:
            raise ValueError(f"duplicate canonical rigid-body name {canonical!r}")
        result[canonical] = identifier
    return result

def modeldef_assets(packet: bytes, nat_major: int = 4, nat_minor: int = 5) -> List[str]:
    return sorted(modeldef_rigid_body_ids(packet, nat_major, nat_minor, {}))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--hostname",
        default="192.168.50.1",
        help="Motive PC IP address",
    )
    parser.add_argument(
        "--table-side",
        choices=("P1", "P2"),
        default="P1",
        help="table end; the required robot rigid body is UCB_P1 or UCB_P2",
    )
    parser.add_argument(
        "--asset-alias",
        action="append",
        default=[],
        metavar="SOURCE=CANONICAL",
        help=(
            "additional Motive rigid-body alias; repeatable; lowercase "
            "ball=Ball is always enabled to match the production bridge"
        ),
    )
    parser.add_argument("--port-command", type=int, default=1510)
    parser.add_argument(
        "--window", type=float, default=8.0, help="FRAMEOFDATA sample seconds"
    )
    parser.add_argument("--max-clock-sync-uncertainty-ms", type=float, default=2.0)
    parser.add_argument("--min-hz", type=float, default=250.0)
    parser.add_argument(
        "--max-loss",
        type=float,
        default=20.0,
        help="maximum allowed missing frame-number percentage",
    )
    args = parser.parse_args()

    try:
        # De-duplicate an explicitly repeated built-in spelling while still
        # rejecting conflicting aliases through parse_asset_aliases().
        alias_entries = list(DEFAULT_ASSET_ALIAS_ENTRIES)
        alias_entries.extend(
            entry for entry in args.asset_alias if entry not in alias_entries
        )
        asset_aliases = parse_asset_aliases(alias_entries)
    except ValueError as error:
        parser.error(str(error))

    if args.window <= 0 or args.min_hz < 0:
        parser.error("--window must be positive and --min-hz non-negative")
    if not 0 <= args.max_loss <= 100:
        parser.error("--max-loss must be in [0, 100]")

    blockers: List[str] = []
    print(f"== NatNet preflight: {args.hostname}:{args.port_command} ==")

    host_device, host_source = route_of(args.hostname)
    print(f"  route to Motive     : dev={host_device} src={host_source}")
    if host_source is None:
        print(f"  FAIL no route to {args.hostname}")
        return 1

    command_sock, server_info = natnet_connect(args.hostname, args.port_command)
    command_sock.close()
    if server_info is None:
        print("  FAIL NAT_CONNECT    : no NAT_SERVERINFO reply")
        print(
            f"\nBLOCKER: Motive is not answering on udp/{args.port_command}. "
            "Check that Motive is running, Broadcast Frame Data is enabled, "
            f"its streaming interface is {args.hostname}, and Windows Firewall "
            f"allows udp/{args.port_command}."
        )
        return 1

    app = server_info[4:260].split(b"\x00")[0].decode(errors="replace")
    app_version = ".".join(str(byte) for byte in server_info[260:264])
    natnet_version = ".".join(str(byte) for byte in server_info[264:268])
    natnet_major = int(server_info[264])
    natnet_minor = int(server_info[265])
    data_port, is_multicast = struct.unpack_from("<H?", server_info, 276)
    multicast_group = ".".join(str(byte) for byte in server_info[279:283])
    print(
        f"  PASS NAT_CONNECT    : {app} {app_version}, NatNet {natnet_version}"
    )
    transmission = "MULTICAST" if is_multicast else "UNICAST"
    suffix = f", group {multicast_group}" if is_multicast else ""
    print(f"  transmission        : {transmission}, data port {data_port}{suffix}")

    for warning in competition_version_warnings(app, server_info[260:264], server_info[264:268]):
        print(f"  WARN VERSION        : {warning}")
    echo_samples = clock_sync_samples(args.hostname, args.port_command)
    frequency = struct.unpack_from("<Q", server_info, 268)[0]
    if len(echo_samples) < 5 or frequency <= 0:
        blockers.append("NatNet echo clock synchronization unavailable")
    elif min(row[0] for row in echo_samples) * 500.0 > args.max_clock_sync_uncertainty_ms:
        blockers.append("NatNet clock-sync midpoint uncertainty exceeds publication limit")

    # Query both forms for diagnosis, but pass only if the exact masked request
    # used by the C++ driver succeeds.
    bare = ask_modeldef(args.hostname, args.port_command, b"")
    masked = ask_modeldef(
        args.hostname,
        args.port_command,
        struct.pack("<I", MODELDEF_TYPES),
    )
    blocker = modeldef_contract_blocker(masked, bare)
    robot_asset = robot_asset_for_table_side(args.table_side)
    rigid_body_ids: Dict[str, int] = {}
    if masked is not None:
        print(
            f"  PASS MODELDEF       : {len(masked)} bytes "
            f"(type mask 0x{MODELDEF_TYPES:x})"
        )
        if bare is None:
            print(
                "  NOTE payload-less MODELDEF is ignored; an unpatched "
                "bridge would hang before creating ROS publishers."
            )
        try:
            raw_rigid_body_ids = modeldef_rigid_body_ids(
                masked, natnet_major, natnet_minor, {}
            )
            rigid_body_ids = modeldef_rigid_body_ids(
                masked, natnet_major, natnet_minor, asset_aliases
            )
        except ValueError as error:
            raw_rigid_body_ids = {}
            blockers.append(f"cannot parse rigid-body IDs from MODELDEF: {error}")
        raw_assets = (
            sorted(raw_rigid_body_ids)
            if raw_rigid_body_ids
            else []
        )
        assets = (
            sorted(rigid_body_ids)
            if rigid_body_ids
            else canonical_assets(raw_assets, asset_aliases)
        )
        print(f"  assets in modeldef  : {', '.join(raw_assets) or '(none)'}")
        if assets != raw_assets:
            print(f"  canonical assets    : {', '.join(assets) or '(none)'}")
        for required in ("Ball", robot_asset):
            if required not in assets:
                if required == "Ball":
                    remedy = "create/enable 'Ball' or 'ball' in Motive"
                else:
                    remedy = "create/rename it exactly in Motive"
                if required != "Ball" and args.asset_alias:
                    remedy += " or correct the explicitly requested asset alias"
                blockers.append(
                    f"rigid body '{required}' is absent from MODELDEF; {remedy}"
                )
    else:
        detail = (
            "bare request replies, masked request is silent"
            if bare is not None
            else "both request forms are silent"
        )
        print(f"  FAIL MODELDEF       : {detail}")
    if blocker:
        blockers.append(blocker)

    if is_multicast:
        group_device, _ = route_of(multicast_group)
        print(
            "  multicast join iface: "
            f"dev={host_device} src={host_source} (selected by Motive route)"
        )
        if group_device != host_device:
            print(
                f"  NOTE group default route: dev={group_device}; ignored "
                "because the client joins on the Motive interface explicitly."
            )
        frame_sock = multicast_socket(
            multicast_group, data_port, host_source
        )
    else:
        frame_sock, unicast_info = natnet_connect(
            args.hostname, args.port_command
        )
        if unicast_info is None:
            blockers.append(
                "second NAT_CONNECT failed while registering the unicast "
                "FRAMEOFDATA receiver"
            )

    ball_id = rigid_body_ids.get("Ball")
    robot_id = rigid_body_ids.get(robot_asset)
    tracked_ids = {
        rigid_body_id
        for rigid_body_id in (ball_id, robot_id)
        if rigid_body_id is not None
    }
    (
        frames,
        hz,
        missing,
        seen,
        valid,
        marker_peak,
        marker_frames,
        tracking_parse_errors,
        marker_parse_errors,
    ) = (
        count_frames_with_tracking(
            frame_sock,
            args.window,
            tracked_ids,
            natnet_major,
            natnet_minor,
        )
    )
    frame_sock.close()
    loss = 100.0 * missing / max(1, frames + missing)
    if frames == 0:
        print(f"  FAIL FRAMEOFDATA    : none received in {args.window:.1f}s")
        blockers.append("no mocap frames reach this host")
    else:
        frame_result = (
            "PASS"
            if hz >= args.min_hz and loss < args.max_loss
            else "FAIL"
        )
        print(
            f"  {frame_result} FRAMEOFDATA    : {frames} frames, "
            f"{hz:.1f} Hz, {loss:.1f}% sequence gaps"
        )
        if hz < args.min_hz:
            blockers.append(
                f"frame rate {hz:.1f} Hz is below --min-hz {args.min_hz:.1f}"
            )
        if loss >= args.max_loss:
            blockers.append(
                f"{loss:.1f}% frame-number gaps exceed "
                f"--max-loss {args.max_loss:.1f}%"
            )
        elif loss >= 1.0:
            print(
                "  NOTE frame numbers are missing at this receiver; use "
                "kernel counters or a packet capture to locate the loss."
            )
        if ball_id is not None:
            ball_seen = seen.get(ball_id, 0)
            ball_valid = valid.get(ball_id, 0)
            print(
                f"  selected ball live  : canonical Ball id={ball_id}, "
                f"tracking-valid {ball_valid}/{ball_seen} frames"
            )
            if not tracking_parse_errors and (
                ball_seen == 0 or ball_valid == 0
            ):
                print(
                    "  NOTE the Ball/ball asset exists but is not currently "
                    "tracking-valid; this is not a launch blocker while the "
                    "ball is outside the capture volume or occluded."
                )
        if robot_id is not None:
            robot_seen = seen.get(robot_id, 0)
            robot_valid = valid.get(robot_id, 0)
            print(
                f"  selected robot live : {robot_asset} id={robot_id}, "
                f"tracking-valid {robot_valid}/{robot_seen} frames"
            )
            if tracking_parse_errors:
                blockers.append(
                    "could not parse rigid-body tracking validity in "
                    f"{tracking_parse_errors}/{frames} FRAMEOFDATA packets"
                )
            elif robot_seen == 0:
                blockers.append(
                    f"rigid body '{robot_asset}' id={robot_id} is absent from "
                    "live FRAMEOFDATA"
                )
            elif robot_valid == 0:
                blockers.append(
                    f"rigid body '{robot_asset}' is present but never "
                    "tracking-valid; enable the asset and restore marker visibility"
                )
            if natnet_major > 4 or (
                natnet_major == 4 and natnet_minor >= 1
            ):
                robot_marker_peak = marker_peak.get(robot_id, 0)
                robot_marker_frames = marker_frames.get(robot_id, 0)
                print(
                    f"  selected marker data: {robot_asset} id={robot_id}, "
                    f"peak {robot_marker_peak}/10 physical labeled markers, "
                    f"present in {robot_marker_frames}/{frames} frames"
                )
                if marker_parse_errors:
                    blockers.append(
                        "could not parse labeled-marker data in "
                        f"{marker_parse_errors}/{frames} FRAMEOFDATA packets"
                    )
                elif robot_marker_peak == 0:
                    blockers.append(
                        f"rigid body '{robot_asset}' supplies no physical "
                        "labeled markers; enable "
                        "Stream Labeled Markers in Motive Data Streaming"
                    )
                elif robot_marker_peak < 10:
                    blockers.append(
                        f"rigid body '{robot_asset}' supplies only "
                        f"{robot_marker_peak}/10 physical labeled markers; "
                        "restore visibility and tracking for the missing "
                        "markers"
                    )
        elif natnet_major < 4 or (natnet_major == 4 and natnet_minor < 1):
            print(
                "  NOTE selected-rigid-body live validity is not decoded for "
                "NatNet versions older than 4.1"
            )

    print()
    if blockers:
        for item in blockers:
            print(f"BLOCKER: {item}")
        return 1
    print("All NatNet checks passed; the bridge should be able to start.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
