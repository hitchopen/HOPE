#!/usr/bin/env python3
"""Pre-launch NatNet preflight: talks to Motive directly, no ROS required.

Complements mocap_rate_probe.py, which proves mocap liveness *after* the bridge
is up by counting ROS messages. This probe runs *before* `ros2 launch` and
speaks the NatNet command protocol itself, so it can distinguish the failure
modes that all look identical from the ROS side (every HOPE topic silent):

  1. Motive unreachable / streaming disabled  -> no NAT_SERVERINFO;
  2. Motive answers NAT_CONNECT but ignores NAT_REQUEST_MODELDEF;
  3. handshake fine, but no FRAMEOFDATA reaches this host (wrong interface,
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

Multicast note: libmotioncapture passes interface_ip="0.0.0.0", so the data
socket's IP_ADD_MEMBERSHIP lets the kernel pick the interface by route. When a
VPN default route wins over the arena NIC, the join lands on the tunnel and no
frames arrive. This probe compares the two routes and fails loudly.

Exit codes: 0 = the bridge should come up, 1 = it will hang or see no data.
"""

import argparse
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

# Must match MODELDEF_TYPES in deps/libmotioncapture/src/optitrack.cpp.
# bit0 = MarkerSet, bit1 = RigidBody. Masks with undefined bits set (0x7f,
# 0xff, ~0) are dropped by Motive, so request exactly what the driver parses.
MODELDEF_TYPES = 0x3

SERVERINFO_MIN_LEN = 283
REQUIRED_ASSETS = ('Ball', 'P1')


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
    deadline = time.time() + wait
    while time.time() < deadline:
        try:
            data, _ = sock.recvfrom(65535)
        except socket.timeout:
            break
        if (struct.unpack_from('<H', data)[0] == NAT_SERVERINFO
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
        deadline = time.time() + wait
        while time.time() < deadline:
            try:
                data, _ = sock.recvfrom(65535)
            except socket.timeout:
                break
            if struct.unpack_from('<H', data)[0] == NAT_MODELDEF:
                return data
        return None
    finally:
        sock.close()


def modeldef_assets(packet: bytes):
    """Asset names in a model definition, ignoring per-marker labels.

    A byte scan rather than a full parse: the dataset layout is NatNet
    version dependent and all this needs to answer is "are Ball and P1 in
    here". Scanning can start mid-token when the preceding byte happens to be
    a letter, so drop any name that is a strict suffix of another ('all' from
    'Ball'). Real HOPE assets (Ball, P1, P2, Table) are never suffixes of each
    other.
    """
    names = {m.decode(errors='replace').rstrip('\x00')
             for m in re.findall(rb'[A-Za-z][A-Za-z0-9_]{1,30}\x00', packet)}
    names = {n for n in names if 'Marker' not in n}
    return sorted(n for n in names
                  if not any(o != n and o.endswith(n) for o in names))


def count_frames(sock: socket.socket, seconds: float):
    """Return (n_frames, hz, missing) using NatNet frame numbers for loss."""
    sock.settimeout(3.0)
    prev = None
    n = 0
    missing = 0
    start = time.time()
    while time.time() - start < seconds:
        try:
            data, _ = sock.recvfrom(65535)
        except socket.timeout:
            break
        if struct.unpack_from('<H', data)[0] != NAT_FRAMEOFDATA:
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
    parser.add_argument('--port-command', type=int, default=1510)
    parser.add_argument('--window', type=float, default=8.0,
                        help='seconds to count FRAMEOFDATA')
    parser.add_argument('--min-hz', type=float, default=250.0)
    parser.add_argument('--max-loss', type=float, default=20.0,
                        help='percent of frame numbers allowed to go missing')
    args = parser.parse_args()

    blockers = []
    print('== NatNet preflight: %s:%d ==' % (args.hostname, args.port_command))

    host_dev, host_src = route_of(args.hostname)
    print('  route to Motive     : dev=%s src=%s' % (host_dev, host_src))
    if host_src is None:
        print('  FAIL no route to %s' % args.hostname)
        return 1

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
    app_version = '.'.join(str(b) for b in info[260:264])
    nat_version = '.'.join(str(b) for b in info[264:268])
    data_port, is_multicast = struct.unpack_from('<H?', info, 276)
    group = '.'.join(str(b) for b in info[279:283])
    print('  PASS NAT_CONNECT    : %s %s, NatNet %s'
          % (app, app_version, nat_version))
    print('  transmission        : %s, data port %d%s'
          % ('MULTICAST' if is_multicast else 'UNICAST', data_port,
             ', group ' + group if is_multicast else ''))
    if is_multicast:
        print('  NOTE Motive is in MULTICAST; docs/OPTITRACK.md recommends '
              'Unicast (venue switches handle it better).')

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
        assets = modeldef_assets(packet)
        print('  assets in modeldef  : %s' % (', '.join(assets) or '(none)'))
        for want in REQUIRED_ASSETS:
            if want not in assets:
                blockers.append("rigid body '%s' is not in the model "
                                'definition; create/rename it in Motive' % want)

    if is_multicast:
        group_dev, _ = route_of(group)
        print('  multicast join iface: dev=%s (driver uses interface_ip='
              '0.0.0.0, so the kernel route decides)' % group_dev)
        if group_dev != host_dev:
            blockers.append('multicast group %s routes out %s but Motive is on '
                            '%s; the data socket would join the wrong '
                            'interface. Switch Motive to Unicast.'
                            % (group, group_dev, host_dev))
        frame_sock = multicast_socket(group, data_port, host_src)
    else:
        frame_sock, _ = natnet_connect(args.hostname, args.port_command)

    frames, hz, missing = count_frames(frame_sock, args.window)
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
