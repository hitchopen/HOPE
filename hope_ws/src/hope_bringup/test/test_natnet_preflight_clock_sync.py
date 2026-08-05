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
