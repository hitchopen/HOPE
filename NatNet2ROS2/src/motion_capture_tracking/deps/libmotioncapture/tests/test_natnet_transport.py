"""Exercise the production C++ driver against a loopback-only fake Motive."""

import socket
import struct
import subprocess
import sys
import threading
import unittest


CLIENT = sys.argv.pop(1) if __name__ == "__main__" else None
GROUP = "239.255.42.99"


def packet(message_id, payload):
    return struct.pack("<HH", message_id, len(payload)) + payload


def frame():
    payload = struct.pack("<i", 4242)
    payload += struct.pack("<ii", 0, 0)  # marker sets
    payload += struct.pack("<ii3f", 1, 12, 1.0, 2.0, 3.0)  # unlabeled
    payload += struct.pack("<ii", 0, 0) * 6  # remaining 4.2 sections
    payload += struct.pack("<IIdQQQIIHi", 0, 0, 12.5, 1000, 1100, 1200,
                           123, 456, 0, 0)
    return packet(7, payload)


class FakeMotive:
    def __init__(self, multicast):
        self.multicast = multicast
        self.command = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.command.bind(("127.0.0.1", 0))
        self.command.settimeout(0.1)
        self.port = self.command.getsockname()[1]
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
            reservation.bind(("127.0.0.1", 0))
            self.data_port = reservation.getsockname()[1]
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sender.bind(("127.0.0.1", 0))
        self.sender.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF,
                               socket.inet_aton("127.0.0.1"))
        self.sender.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 0)
        self.sender.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        self.stop = threading.Event()
        self.errors = []
        self.thread = threading.Thread(target=self.serve, daemon=True)

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *args):
        self.stop.set()
        self.thread.join(timeout=2)
        self.command.close()
        self.sender.close()

    def server_info(self):
        payload = bytearray(279)
        payload[:6] = b"Motive"
        payload[256:260] = bytes((3, 2, 0, 0))
        payload[260:264] = bytes((4, 2, 0, 0))
        struct.pack_into("<QH?", payload, 264, 1_000_000_000,
                         self.data_port, self.multicast)
        payload[275:279] = socket.inet_aton(GROUP)
        return packet(1, payload)

    def serve(self):
        try:
            while not self.stop.is_set():
                try:
                    data, peer = self.command.recvfrom(65535)
                except socket.timeout:
                    continue
                message_id = struct.unpack_from("<H", data)[0]
                if message_id == 0:
                    if peer[1] == self.data_port:
                        target = (GROUP, self.data_port) if self.multicast else peer
                        self.sender.sendto(frame(), target)
                    else:
                        self.command.sendto(self.server_info(), peer)
                elif message_id == 4:
                    self.command.sendto(packet(5, struct.pack("<i", 0)), peer)
        except Exception as error:
            self.errors.append(error)


class NatNetTransportTest(unittest.TestCase):
    def check_transport(self, multicast, interface, selected=None):
        with FakeMotive(multicast) as motive:
            result = subprocess.run(
                [CLIENT, str(motive.port), interface], text=True,
                capture_output=True, timeout=5,
            )
        self.assertEqual(motive.errors, [])
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("FRAME_DECODED", result.stdout)
        if selected:
            self.assertIn(f"via local interface {selected},", result.stdout)

    def test_multicast_empty_uses_motive_route(self):
        self.check_transport(True, "", "127.0.0.1")

    def test_multicast_wildcard_uses_motive_route(self):
        self.check_transport(True, "0.0.0.0", "127.0.0.1")

    def test_multicast_explicit_nic(self):
        self.check_transport(True, "127.0.0.1", "127.0.0.1")

    def test_multicast_explicit_nic_overrides_route(self):
        self.check_transport(True, "127.0.0.2", "127.0.0.2")

    def test_unicast_empty_interface(self):
        self.check_transport(False, "")


if __name__ == "__main__":
    unittest.main(verbosity=2)
