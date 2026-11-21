"""Read-only control-connection evidence; never infer robot identity from an IP."""
import ipaddress
import json
import os
from pathlib import Path
import re
import struct
import subprocess

PRIVATE = tuple(ipaddress.ip_network(n) for n in ('10.0.0.0/8', '172.16.0.0/12', '192.168.0.0/16'))


def control_connections(text):
    pairs = set()
    for line in text.splitlines()[1:]:
        fields = line.split()
        if len(fields) < 4 or fields[3] != '01':
            continue
        try:
            local, port = fields[1].split(':')
            peer, _ = fields[2].split(':')
            if int(port, 16) != 8766:
                continue
            addresses = [str(ipaddress.ip_address(struct.pack('<I', int(a, 16)))) for a in (local, peer)]
            if all(any(ipaddress.ip_address(a) in n for n in PRIVATE) for a in addresses):
                pairs.add(tuple(addresses))
        except (ValueError, struct.error):
            continue
    return [{'hdu_ip': h, 'laptop_ip': l} for h, l in sorted(pairs)]


def connection_identity():
    try:
        machine = Path('/etc/machine-id').read_text().strip()
        boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        pairs = control_connections(Path('/proc/net/tcp').read_text())
        return {'machine_id': machine, 'boot_id': boot, 'connections': pairs,
                'connection': pairs[0] if len(pairs) == 1 else None}
    except OSError:
        return None


def validate_network_migration(current, updated, raw, *, observed=None, run=subprocess.run):
    """Called under the hardware lock, before any saved-config write.

    Only the unique live control pair and the installed operator's pinned machine
    are eligible. No browser-supplied command, host identity or SSH trust override.
    """
    actual = connection_identity() if observed is None else observed
    try:
        request = json.loads(raw)
        expected = {'machine_id': actual['machine_id'], 'boot_id': actual['boot_id'],
                    'revision': current.revision}
        if request != expected or isinstance(request.get('revision'), bool):
            raise ValueError('Robot identity or configuration revision changed; refresh and retry')
        pair = actual['connection']
        if pair != {'hdu_ip': updated.hdu_wifi_ip, 'laptop_ip': updated.laptop_wifi_ip}:
            raise ValueError('Network migration requires one unambiguous live control connection')
    except (TypeError, KeyError, json.JSONDecodeError) as exc:
        raise ValueError('Fresh robot identity and control connection evidence are required') from exc
    if updated.operator_profile != 'linux' or current.operator_profile != 'linux':
        raise ValueError('This network repair requires the registered Linux operator')
    pinned = os.environ.get('HOPE_LAPTOP_MACHINE_ID', '')
    user = os.environ.get('HOPE_LAPTOP_USER', '')
    if not re.fullmatch('[0-9a-f]{32}', pinned) or not re.fullmatch('[a-z_][a-z0-9_-]{0,31}', user):
        raise ValueError('Register the operator machine identity before network migration')
    result = run(['/usr/bin/ssh', '-T', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
                  '-o', 'ConnectTimeout=3', user + '@' + updated.laptop_wifi_ip,
                  '/usr/bin/cat', '/etc/machine-id'], text=True, capture_output=True, timeout=5, check=True)
    if result.stdout.strip() != pinned:
        raise ValueError('Connected computer differs from the registered operator; unchanged')
    # Refuse a connection/boot change during SSH verification.
    if observed is None and connection_identity() != actual:
        raise ValueError('Control connection changed during verification; retry')
