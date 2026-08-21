#!/usr/bin/env python3
"""Pre-launch NatNet preflight: talks to Motive directly, no ROS required.

Complements mocap_rate_probe.py, which proves mocap liveness *after* the bridge
is up by counting ROS messages. This probe runs *before* `ros2 launch` and
speaks the NatNet command protocol itself, so it can distinguish the failure
modes that all look identical from the ROS side (every HOPE topic silent):

  1. Motive unreachable / streaming disabled  -> no NAT_SERVERINFO;
  2. Motive answers NAT_CONNECT but ignores NAT_REQUEST_MODELDEF;
  3. Motive does not support the NatNet echo exchange required for camera_utc;
  4. handshake fine, but no FRAMEOFDATA reaches this host (wrong interface,
     multicast routed out a VPN tunnel, firewall).

Mode 2 is why this script exists. On 2026-07-30 a venue Motive (3.1.0.4 /
NatNet 4.1) silently dropped every payload-less NAT_REQUEST_MODELDEF; the
vendored driver's unbounded blocking receive then deadlocked in the
MotionCaptureOptitrack constructor, *before* create_publisher(), so
/optitrack/poses existed with `Publisher count: 0` and every downstream topic
stayed silent with no error logged anywhere. See the driver fix in
deps/libmotioncapture/src/optitrack.cpp (MODELDEF_TYPES; PIN.md patch #9).
This probe reports that condition in ten seconds instead of leaving the
operator to guess.

For multicast, --interface-ip is the exact local wired-NIC IPv4 address passed
to NatNet2ROS2. The probe and driver therefore join the same group on the same
interface; neither lets a VPN/default route choose implicitly.

Exit codes: 0 = the bridge should come up, 1 = it cannot start or publish
camera_utc data safely.
"""

import argparse
import ipaddress
import re
import socket
import struct
import subprocess
import sys
import time

NAT_CONNECT = 0
NAT_SERVERINFO = 1
NAT_REQUEST_MODELDEF = 4
NAT_MODELDEF = 5
NAT_FRAMEOFDATA = 7
NAT_ECHOREQUEST = 12
NAT_ECHORESPONSE = 13

# Must match MODELDEF_TYPES in deps/libmotioncapture/src/optitrack.cpp.
# bit0 = MarkerSet, bit1 = RigidBody. Masks with undefined bits set (0x7f,
# 0xff, ~0) are dropped by Motive, so request exactly what the driver parses.
MODELDEF_TYPES = 0x3

SERVERINFO_MIN_LEN = 283
REQUIRED_ASSETS = ('Ball', 'P1', 'P2')
EXPECTED_APP_NAMES = ('Motive', 'MotiveBody')
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


def route_of(dst: str):
    """Return (dev, src) the kernel would use for dst, or (None, None)."""
    try:
        out = subprocess.run(['ip', '-o', 'route', 'get', dst],
                             capture_output=True, text=True,
                             timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return None, None
    dev = re.search(r'\bdev (\S+)', out)
    src = re.search(r'\bsrc (\S+)', out)
    return (dev.group(1) if dev else None), (src.group(1) if src else None)


def packet_message_id(data: bytes):
    """Return a NatNet message id only when the declared payload fits."""
    if len(data) < 4:
        return None
    message_id, payload_size = struct.unpack_from('<HH', data)
    if payload_size > len(data) - 4:
        return None
    return message_id


def natnet_connect(host: str, port: int, wait: float = 3.0):
    """Send NAT_CONNECT; return (socket, serverinfo_packet_or_None).

    The socket stays open: in unicast mode Motive streams FRAMEOFDATA back to
    the source endpoint of this NAT_CONNECT, so the caller reuses it to count
    frames.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
    sock.settimeout(wait)
    sock.bind(('0.0.0.0', 0))
    sock.sendto(struct.pack('<HH', NAT_CONNECT, 0), (host, port))
    server_ip = socket.gethostbyname(host)
    deadline = time.time() + wait
    while time.time() < deadline:
        try:
            data, sender = sock.recvfrom(65535)
        except socket.timeout:
            break
        if (sender[0] == server_ip
                and packet_message_id(data) == NAT_SERVERINFO
                and len(data) >= SERVERINFO_MIN_LEN):
            return sock, data
    return sock, None


def ask_modeldef(host: str, port: int, payload: bytes, wait: float = 3.0):
    """One NAT_REQUEST_MODELDEF round trip on a fresh connection."""
    sock, info = natnet_connect(host, port, wait)
    try:
        if info is None:
            return None
        sock.sendto(
            struct.pack('<HH', NAT_REQUEST_MODELDEF, len(payload)) + payload,
            (host, port))
        server_ip = socket.gethostbyname(host)
        deadline = time.time() + wait
        while time.time() < deadline:
            try:
                data, sender = sock.recvfrom(65535)
            except socket.timeout:
                break
            if (sender[0] == server_ip
                    and packet_message_id(data) == NAT_MODELDEF):
                return data
        return None
    finally:
        sock.close()


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


def modeldef_assets(packet: bytes, nat_major: int, nat_minor: int):
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
    assets = []

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
            assets.append(name)
            dataset.unpack('i', 'rigid-body id')
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

    return sorted(name for name in assets if name)


def count_frames(sock: socket.socket, seconds: float, server_ip: str):
    """Return (n_frames, hz, missing) using NatNet frame numbers for loss."""
    sock.settimeout(3.0)
    prev = None
    n = 0
    missing = 0
    start = time.time()
    while time.time() - start < seconds:
        try:
            data, sender = sock.recvfrom(65535)
        except socket.timeout:
            break
        if sender[0] != server_ip or len(data) < 8:
            continue
        message_id, payload_size = struct.unpack_from('<HH', data)
        if payload_size > len(data) - 4 or message_id != NAT_FRAMEOFDATA:
            continue
        number = struct.unpack_from('<i', data, 4)[0]
        n += 1
        if prev is not None and number != prev + 1:
            missing += max(0, number - prev - 1)
        prev = number
    elapsed = max(1e-6, time.time() - start)
    return n, n / elapsed, missing


def multicast_socket(group: str, port: int, iface_ip: str):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
    sock.bind(('0.0.0.0', port))
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                    socket.inet_aton(group) + socket.inet_aton(iface_ip))
    return sock


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--hostname', required=True,
                        help='Motive PC IP (NatNet server); venue fact, '
                             'always passed explicitly')
    parser.add_argument('--interface-ip', required=True,
                        help="this computer's wired Motive-network NIC IPv4; "
                             'passed unchanged to NatNet2ROS2')
    parser.add_argument('--port-command', type=int, default=1510)
    parser.add_argument('--window', type=float, default=8.0,
                        help='seconds to count FRAMEOFDATA')
    parser.add_argument('--min-hz', type=float, default=270.0,
                        help='minimum accepted direct NatNet rate; competition '
                             'Motive is configured for 300 Hz')
    parser.add_argument('--max-loss', type=float, default=20.0,
                        help='percent of frame numbers allowed to go missing')
    parser.add_argument('--max-clock-sync-uncertainty-ms', type=float,
                        default=2.0,
                        help='maximum NatNet echo midpoint uncertainty')
    args = parser.parse_args()
    if args.max_clock_sync_uncertainty_ms <= 0.0:
        parser.error('--max-clock-sync-uncertainty-ms must be positive')

    try:
        interface_address = ipaddress.ip_address(args.interface_ip)
    except ValueError:
        parser.error('--interface-ip must be an IPv4 address')
    if (interface_address.version != 4 or interface_address.is_unspecified
            or interface_address.is_multicast):
        parser.error('--interface-ip must be an explicit unicast IPv4 address')

    blockers = []
    print('== NatNet preflight: %s:%d ==' % (args.hostname, args.port_command))

    host_dev, host_src = route_of(args.hostname)
    print('  route to Motive     : dev=%s src=%s' % (host_dev, host_src))
    if host_src is None:
        print('  FAIL no route to %s' % args.hostname)
        return 1
    if host_src != args.interface_ip:
        blockers.append('--interface-ip %s does not match the route source %s '
                        'used to reach Motive on %s'
                        % (args.interface_ip, host_src, host_dev))

    sock, info = natnet_connect(args.hostname, args.port_command)
    sock.close()
    if info is None:
        print('  FAIL NAT_CONNECT    : no NAT_SERVERINFO reply')
        print('\nBLOCKER: Motive is not answering on udp/%d. Check that Motive '
              'is running, Broadcast Frame Data is ON, the streaming interface '
              'is %s, and the Windows firewall allows udp/%d.'
              % (args.port_command, args.hostname, args.port_command))
        return 1

    app = info[4:260].split(b'\x00')[0].decode(errors='replace')
    app_version_bytes = tuple(info[260:264])
    app_version = '.'.join(str(b) for b in app_version_bytes)
    nat_version_bytes = tuple(info[264:268])
    nat_version = '.'.join(str(b) for b in nat_version_bytes)
    data_port, is_multicast = struct.unpack_from('<H?', info, 276)
    high_res_clock_frequency = struct.unpack_from('<Q', info, 268)[0]
    group = '.'.join(str(b) for b in info[279:283])
    print('  PASS NAT_CONNECT    : %s %s, NatNet %s'
          % (app, app_version, nat_version))
    print('  transmission        : %s, data port %d%s'
          % ('MULTICAST' if is_multicast else 'UNICAST', data_port,
             ', group ' + group if is_multicast else ''))
    for warning in competition_version_warnings(
            app, app_version_bytes, nat_version_bytes):
        print('  WARN VERSION        : %s' % warning)
    if not is_multicast:
        blockers.append('competition Motive transmission must be MULTICAST')
    if is_multicast and group != '239.255.42.99':
        blockers.append('unexpected multicast group %s; expected 239.255.42.99'
                        % group)
    if data_port != 1511:
        blockers.append('unexpected NatNet data port %d; expected 1511'
                        % data_port)

    echo_samples = clock_sync_samples(args.hostname, args.port_command)
    if len(echo_samples) >= 5 and high_res_clock_frequency > 0:
        min_rtt = min(sample[0] for sample in echo_samples)
        midpoint_uncertainty_ms = min_rtt * 500.0
        clock_ok = (midpoint_uncertainty_ms
                    <= args.max_clock_sync_uncertainty_ms)
        print('  %s CLOCK SYNC     : %d echoes, min RTT %.3f ms, '
              'midpoint uncertainty <= %.3f ms (limit %.3f ms)'
              % ('PASS' if clock_ok else 'FAIL', len(echo_samples),
                 min_rtt * 1e3, midpoint_uncertainty_ms,
                 args.max_clock_sync_uncertainty_ms))
        if not clock_ok:
            blockers.append('NatNet clock-sync midpoint uncertainty exceeds '
                            'the camera_utc publication limit')
    else:
        print('  FAIL CLOCK SYNC     : %d/10 echo replies, QPC frequency %d'
              % (len(echo_samples), high_res_clock_frequency))
        blockers.append('camera_utc cannot map Motive QPC to adapter time; '
                        'NatNet echo clock synchronization is unavailable')

    bare = ask_modeldef(args.hostname, args.port_command, b'')
    masked = ask_modeldef(args.hostname, args.port_command,
                          struct.pack('<i', MODELDEF_TYPES))
    if masked is not None:
        print('  PASS MODELDEF       : %d bytes (type mask 0x%x)'
              % (len(masked), MODELDEF_TYPES))
        if bare is None:
            print('  NOTE this Motive ignores payload-less MODELDEF requests; '
                  'the patched driver sends the type mask, an unpatched one '
                  'would hang in its constructor.')
    elif bare is not None:
        print('  PASS MODELDEF       : %d bytes (no type mask needed)'
              % len(bare))
    else:
        print('  FAIL MODELDEF       : silent for both request forms')
        blockers.append('Motive returns no model definition. Toggle Broadcast '
                        'Frame Data off/on, then restart Motive.')

    packet = masked or bare
    if packet is not None:
        try:
            assets = modeldef_assets(
                packet, nat_version_bytes[0], nat_version_bytes[1])
        except ValueError as exc:
            assets = []
            blockers.append('MODELDEF decode failed for NatNet %s: %s'
                            % (nat_version, exc))
        else:
            print('  assets in modeldef  : %s'
                  % (', '.join(assets) or '(none)'))
            for want in REQUIRED_ASSETS:
                if want not in assets:
                    blockers.append("rigid body '%s' is not in the model "
                                    'definition; create/rename it in Motive'
                                    % want)

    try:
        if is_multicast:
            print('  multicast join iface: dev=%s interface_ip=%s'
                  % (host_dev, args.interface_ip))
            frame_sock = multicast_socket(group, data_port, args.interface_ip)
        else:
            frame_sock, _ = natnet_connect(args.hostname, args.port_command)
    except OSError as exc:
        frame_sock = None
        blockers.append('cannot open NatNet data socket: %s' % exc)

    if frame_sock is None:
        frames, hz, missing = 0, 0.0, 0
    else:
        server_ip = socket.gethostbyname(args.hostname)
        frames, hz, missing = count_frames(frame_sock, args.window, server_ip)
        frame_sock.close()
    loss = 100.0 * missing / max(1, frames + missing)
    if frames == 0:
        print('  FAIL FRAMEOFDATA    : none received in %.0fs' % args.window)
        blockers.append('no mocap frames reach this host')
    else:
        print('  %s FRAMEOFDATA    : %d frames, %.1f Hz, %.1f%% loss'
              % ('PASS' if hz >= args.min_hz else 'FAIL', frames, hz, loss))
        if hz < args.min_hz:
            blockers.append('frame rate %.1f Hz is below --min-hz %.1f'
                            % (hz, args.min_hz))
        if loss >= args.max_loss:
            blockers.append('%.1f%% of frame numbers are missing (threshold '
                            '%.1f%%)' % (loss, args.max_loss))
        elif loss >= 1.0:
            print('  NOTE %.1f%% of frame numbers never left Motive; isolated '
                  'gaps at this level are tolerated but indicate Motive-side '
                  'load, not network loss.' % loss)

    print()
    if blockers:
        for item in blockers:
            print('BLOCKER: %s' % item)
        return 1
    print('All checks passed; the bridge should come up.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
