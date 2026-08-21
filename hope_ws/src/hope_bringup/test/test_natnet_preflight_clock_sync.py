import importlib.util
from pathlib import Path
import socket
import struct
import threading
import time

SCRIPT = Path(__file__).parents[1] / "scripts" / "natnet_preflight.py"
SPEC = importlib.util.spec_from_file_location("natnet_preflight", SCRIPT)
natnet_preflight = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(natnet_preflight)


class FakeMotive:
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", 0))
        self.socket.settimeout(0.1)
        self.port = self.socket.getsockname()[1]
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._serve, daemon=True)

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *_):
        self.stop.set()
        self.thread.join(timeout=1.0)
        self.socket.close()

    @staticmethod
    def _server_info():
        packet = bytearray(natnet_preflight.SERVERINFO_MIN_LEN)
        struct.pack_into("<HH", packet, 0, natnet_preflight.NAT_SERVERINFO,
                         len(packet) - 4)
        packet[4:10] = b"Motive"
        packet[260:264] = bytes((3, 1, 0, 4))
        packet[264:268] = bytes((4, 1, 0, 0))
        struct.pack_into("<Q", packet, 268, 1_000_000_000)
        struct.pack_into("<H?", packet, 276, 1511, False)
        return packet

    def _serve(self):
        while not self.stop.is_set():
            try:
                data, peer = self.socket.recvfrom(65535)
            except socket.timeout:
                continue
            if len(data) < 4:
                continue
            message_id = struct.unpack_from("<H", data)[0]
            if message_id == natnet_preflight.NAT_CONNECT:
                self.socket.sendto(self._server_info(), peer)
            elif (message_id == natnet_preflight.NAT_ECHOREQUEST
                  and len(data) >= 12):
                token = struct.unpack_from("<Q", data, 4)[0]
                response = struct.pack(
                    "<HHQQ", natnet_preflight.NAT_ECHORESPONSE, 16,
                    token, time.monotonic_ns())
                self.socket.sendto(response, peer)


def test_clock_sync_samples_match_echo_tokens():
    with FakeMotive() as motive:
        samples = natnet_preflight.clock_sync_samples(
            "127.0.0.1", motive.port, count=10)

    assert len(samples) == 10
    assert all(rtt >= 0.0 for rtt, _ in samples)
    assert all(server_ticks > 0 for _, server_ticks in samples)


def test_competition_profile_accepts_motivebody_natnet_45():
    assert natnet_preflight.competition_version_blockers(
        "MotiveBody", (3, 5, 0, 1), (4, 5, 0, 0)) == []
    # SERVERINFO may retain the legacy application name even for MotiveBody.
    assert natnet_preflight.competition_version_blockers(
        "Motive", (3, 5, 0, 1), (4, 5, 0, 0)) == []


def test_competition_profile_rejects_old_natnet_42():
    blockers = natnet_preflight.competition_version_blockers(
        "Motive", (3, 2, 0, 2), (4, 2, 0, 0))
    assert len(blockers) == 2
    assert any("NatNet 4.5" in blocker for blocker in blockers)


def make_rigid_body_description(name, rigid_body_id, include_rotation):
    description = bytearray()
    description.extend(name.encode() + b"\x00")
    description.extend(struct.pack("<ii3f", rigid_body_id, -1,
                                   0.1, -0.2, 0.3))
    if include_rotation:
        description.extend(struct.pack("<4f", 0.1, 0.2, 0.3, 0.9))
    description.extend(struct.pack("<i", 1))
    description.extend(struct.pack("<3fi", 1.0, 2.0, 3.0, 1001))
    description.extend(b"marker_0\x00")
    return struct.pack("<ii", 1, len(description)) + description


def make_modeldef_42():
    unknown = struct.pack("<iii", 99, 4, 0x12345678)
    datasets = unknown + b"".join([
        make_rigid_body_description("Ball", 101, True),
        make_rigid_body_description("P1", 102, True),
        make_rigid_body_description("P2", 103, True),
    ])
    payload = struct.pack("<i", 4) + datasets
    return struct.pack("<HH", natnet_preflight.NAT_MODELDEF, len(payload)) + payload


def make_modeldef_45():
    extensions = b"".join(
        struct.pack("<iii", dataset_type, 4, 0x12340000 + dataset_type)
        for dataset_type in (7, 8)
    )
    anchor = struct.pack("<iii", 9, 4, 0x12340009)
    datasets = extensions + b"".join([
        make_rigid_body_description("Ball", 101, True),
        make_rigid_body_description("P1", 102, True),
        make_rigid_body_description("P2", 103, True),
    ]) + anchor
    payload = struct.pack("<i", 6) + datasets
    return struct.pack("<HH", natnet_preflight.NAT_MODELDEF, len(payload)) + payload


def test_modeldef_assets_decodes_natnet_42_rotation_offsets():
    packet = make_modeldef_42()
    assert natnet_preflight.modeldef_assets(packet, 4, 2) == ["Ball", "P1", "P2"]


def test_modeldef_assets_skips_natnet_45_extension_descriptions():
    packet = make_modeldef_45()
    assert natnet_preflight.modeldef_assets(packet, 4, 5) == ["Ball", "P1", "P2"]


def test_modeldef_assets_rejects_truncated_natnet_42_packet():
    packet = make_modeldef_42()
    try:
        natnet_preflight.modeldef_assets(packet[:-1], 4, 2)
    except ValueError:
        return
    raise AssertionError("truncated sized NatNet MODELDEF was accepted")
