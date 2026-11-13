#!/usr/bin/env python3
"""Canonical network environment and evidence of the configuration in use.

Only config.json is editable. Reports are derived evidence. Reconciliation never
starts an inactive unit and requires the shared hardware lock and idle owners.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time
import xml.etree.ElementTree as ET

from hope_lifecycle_core import config_to_document, load_config, try_acquire_hardware_operation_lock, release_hardware_operation_lock

ROOT = Path('/var/lib/hope-lifecycle')
CONFIG, REPORT, OWNER = (ROOT / n for n in ('config.json', 'config-runtime.json', 'config-owner.json'))
CALIBRATION = Path('/var/lib/hope-time-calibration/status.json')
BOOT = Path('/proc/sys/kernel/random/boot_id')
UNITS = ('hope-foxglove-bridge.service', 'hope-foxglove-control-bridge.service',
         'hope-mocap-config-transport.service', 'hope-monitor.service',
         'hope-imu-telemetry.service', 'hope-time-calibration.service',
         'hope-lifecycle-supervisor.service')  # owner last
NETWORK_FIELDS = ('laptop_wifi_ip', 'hdu_wifi_ip', 'mdu_internal_ip')


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def identity(config):
    return digest(config_to_document(config))


def network(config):
    peers = [config.laptop_wifi_ip, config.mdu_internal_ip] if config.revision > 0 else []
    return {'peers': ';'.join(dict.fromkeys(p for p in peers if p)),
            'sha256': digest({k: getattr(config, k) for k in NETWORK_FIELDS})}


def environment(config, inherited):
    # Bootstrap stays reachable for apply_config; it can never report APPLIED.
    result = dict(inherited)
    result.update(ROS_STATIC_PEERS=network(config)['peers'], HOPE_CONFIG_SOURCE=str(CONFIG),
                  HOPE_CONFIG_NETWORK_SHA256=network(config)['sha256'])
    return result


def atomic_json(path, value):
    path = Path(path)
    tmp = path.with_name(path.name + f'.{os.getpid()}.tmp')
    try:
        with tmp.open('w') as stream:
            json.dump(value, stream, sort_keys=True)
            stream.write('\n')
        tmp.chmod(0o644)
        tmp.replace(path)
    finally:
        tmp.unlink(missing_ok=True)


def owner_status(config, state, busy, path=OWNER, now=None):
    atomic_json(path, {'boot_id': BOOT.read_text().strip(), 'monotonic_s': time.monotonic() if now is None else now,
                      'config_sha256': identity(config), 'state': state, 'busy': bool(busy)})


def read_status(config, path=REPORT, now=None):
    result = {'schema_version': 1, 'source': str(CONFIG), 'revision': config.revision,
              'values': config.values(), 'config_sha256': identity(config), 'applied': False,
              'state': 'UNKNOWN', 'reason': 'Waiting for configuration runtime audit', 'services': [], 'laptop': {}}
    try:
        report = json.loads(Path(path).read_text())
        age = (time.monotonic() if now is None else now) - report['monotonic_s']
        if report['boot_id'] != BOOT.read_text().strip() or not 0 <= age <= 12:
            raise ValueError('Configuration runtime audit is stale')
        if report['config_sha256'] != identity(config):
            raise ValueError('Saved configuration is awaiting runtime reconciliation')
        services, laptop = report['services'], report['laptop']
        if (not isinstance(services, list) or {s['service'] for s in services} != set(UNITS)
                or len(services) != len(UNITS) or not isinstance(laptop, dict)):
            raise ValueError('Incomplete configuration runtime audit')
        applied = (config.revision > 0 and report['applied'] is True and report['state'] == 'APPLIED'
                   and all(s.get('matches') is True for s in services) and laptop.get('matches') is True)
        result.update(applied=applied, state='APPLIED' if applied else 'PENDING',
                      reason=str(report['reason']), services=services, laptop=laptop, age_s=age)
    except (OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
        result['reason'] = str(exc)
    return result


def require_applied(config):
    status = read_status(config)
    if not status['applied']:
        raise ValueError('CONFIG_NOT_APPLIED: ' + status['reason'])


def process_status(pid, config, proc=Path('/proc')):
    path = proc / str(pid)
    before = (path / 'stat').read_text()
    env = dict(v.split('=', 1) for v in (path / 'environ').read_bytes().decode().split('\0') if '=' in v)
    expected = network(config)
    result = {'pid': pid, 'actual_peers': env.get('ROS_STATIC_PEERS', ''),
              'source': env.get('HOPE_CONFIG_SOURCE', ''),
              'network_sha256': env.get('HOPE_CONFIG_NETWORK_SHA256', '')}
    result['matches'] = (result['actual_peers'] == expected['peers'] and result['source'] == str(CONFIG)
                         and result['network_sha256'] == expected['sha256'])
    # A generated Fast DDS profile can override an otherwise correct environment.
    profile = env.get('FASTDDS_DEFAULT_PROFILES_FILE') or env.get('FASTRTPS_DEFAULT_PROFILES_FILE')
    if profile:
        tree = ET.parse(profile)
        addresses = [a.text for peers in tree.iter() if peers.tag.split('}')[-1] == 'initialPeersList'
                     for a in peers.iter() if a.tag.split('}')[-1] == 'address']
        result['profile_peers'] = addresses
        if addresses and set(addresses) != set(expected['peers'].split(';')):
            result['matches'] = False
    # starttime is field 22, after the potentially spaced parenthesized comm.
    if before.rsplit(')', 1)[1].split()[19] != (path / 'stat').read_text().rsplit(')', 1)[1].split()[19]:
        raise ValueError('Process identity changed during audit')
    return result


def service_status(unit, config, run=subprocess.run, proc=Path('/proc')):
    r = run(['systemctl', 'show', unit, '-p', 'LoadState,ActiveState,MainPID'], text=True, capture_output=True, check=True, timeout=3)
    values = dict(line.split('=', 1) for line in r.stdout.splitlines() if '=' in line)
    result = {'service': unit, 'active': values.get('ActiveState'), 'pid': int(values.get('MainPID', '0')),
              'expected_peers': network(config)['peers'], 'actual_peers': None, 'matches': False, 'processes': []}
    if values.get('LoadState') != 'loaded' or result['active'] != 'active' or not result['pid']:
        result['reason'] = 'Service is not active'
        return result
    try:
        queue, seen = [result['pid']], set()
        while queue:
            pid = queue.pop()
            if pid in seen:
                continue
            seen.add(pid)
            result['processes'].append(process_status(pid, config, proc))
            tasks = list((proc / str(pid) / 'task').glob('*'))
            try:
                for task in tasks:
                    queue.extend(int(v) for v in (task / 'children').read_text().split())
            except FileNotFoundError:
                # Some robot kernels omit CONFIG_CHECKPOINT_RESTORE and expose
                # no task/children files. PPid still identifies real children.
                if not (proc / str(pid) / 'stat').exists():
                    raise ValueError('Service process exited during audit')
                for status in proc.glob('[0-9]*/status'):
                    try:
                        parent = next(line.split(':', 1)[1].strip() for line in
                                      status.read_text().splitlines() if line.startswith('PPid:'))
                        if int(parent) == pid:
                            queue.append(int(status.parent.name))
                    except FileNotFoundError:
                        pass  # unrelated short-lived process exited
        result['actual_peers'] = result['processes'][0]['actual_peers']
        result['matches'] = all(p['matches'] for p in result['processes'])
        result['reason'] = '' if result['matches'] else 'Process environment or DDS profile differs from saved configuration'
    except (OSError, ValueError, ET.ParseError) as exc:
        result['reason'] = str(exc)
    return result


def can_reconcile(owner, calibration, now, boot, config_sha):
    stamp = owner.get('monotonic_s')
    return (isinstance(stamp, (int, float)) and math.isfinite(stamp)
            and owner.get('boot_id') == boot and 0 <= now - stamp <= 3
            and owner.get('config_sha256') == config_sha
            and owner.get('state') == 'STOPPED' and owner.get('busy') is False
            and calibration.get('state') in {'IDLE', 'REJECTED', 'COMPLETE'})


def no_control_processes(proc=Path('/proc')):
    for p in proc.glob('[0-9]*/exe'):
        try:
            if p.resolve(strict=True).name in {'a3_deploy_onnx_ref_pingpong', 'hope_planner_cpp_node'}:
                return False
        except FileNotFoundError:
            pass  # process exited
        except PermissionError:
            return False
    return True


def no_mdu_control_processes(config, run=subprocess.run):
    robot_user = os.environ.get('HOPE_ROBOT_USER', 'agi')
    if not re.fullmatch(r'[a-z_][a-z0-9_-]{0,31}', robot_user):
        return False
    # pgrep exit 1 alone proves absence. SSH failure/timeout is not an idle MDU.
    result = run(['runuser', '-u', 'agi', '--', '/usr/bin/ssh', '-T', '-o', 'BatchMode=yes',
                  '-o', 'ConnectTimeout=2', robot_user + '@' + config.mdu_internal_ip,
                  "pgrep -f '(^|/)a3_deploy_onnx_ref_pingpong([[:space:]]|$)'"],
                 capture_output=True, text=True, timeout=3, check=False)
    return result.returncode == 1


def laptop_matches(config, result):
    expected = {'HOPE_LIFECYCLE_HDU_IP': config.hdu_wifi_ip, 'HOPE_LIFECYCLE_TABLE_SIDE': config.table_side,
                'HOPE_LIFECYCLE_LAPTOP_IP': config.laptop_wifi_ip, 'HOPE_LIFECYCLE_MDU_IP': config.mdu_internal_ip,
                'HOPE_LIFECYCLE_MOCAP_SOURCE': config.mocap_source, 'HOPE_LIFECYCLE_PANTS_PROFILE': config.pants_profile,
                'HOPE_LIFECYCLE_VRPN_IP': config.vrpn_ip, 'HOPE_LIFECYCLE_VRPN_PORT': config.vrpn_port,
                'HOPE_LIFECYCLE_VRPN_UNITS': config.vrpn_units, 'HOPE_LIFECYCLE_MOTIVE_IP': config.motive_ip,
                'HOPE_VRPN_PELVIS_ASSET_CONFIRMED': config.vrpn_pelvis_asset_confirmed,
                'HOPE_VRPN_WORLD_FRAME_CONFIRMED': config.vrpn_world_frame_confirmed}
    mismatches = {k: {'expected': v, 'actual': result.get('receipt', {}).get(k)} for k, v in expected.items()
                  if result.get('receipt', {}).get(k) != v}
    processes = result.get('processes', [])
    for process in processes:
        if process.get('matches') is not True:
            mismatches[str(process['pid'])] = process
    kinds = {p.get('kind') for p in processes}
    required = {'display'} | ({'vrpn', 'vrpn_relay', 'base'} if config.mocap_source == 'vrpn' else {'motive', 'motive_relay', 'base'} if config.mocap_source == 'motive' else set())
    if required - kinds:
        mismatches['missing_processes'] = sorted(required - kinds)
    if config.mocap_source == 'none' and kinds & {'vrpn', 'motive'}:
        mismatches['mocap_disabled'] = 'Receiver still running'
    result['mismatches'] = mismatches
    result['matches'] = not mismatches and result.get('display_active') is True
    return result


def laptop_status(config, run=subprocess.run):
    mac = config.operator_profile == 'mac'
    user = os.environ.get('HOPE_MAC_USER' if mac else 'HOPE_LAPTOP_USER', 'operator')
    helper = os.environ.get('HOPE_MAC_LIFECYCLE_HELPER' if mac else 'HOPE_LAPTOP_LIFECYCLE_HELPER',
                            '/home/operator/.local/libexec/hope-lifecycle' if mac else '/usr/local/libexec/hope-lifecycle')
    if not re.fullmatch(r'[a-z_][a-z0-9_-]{0,31}', user) or not re.fullmatch(r'/[A-Za-z0-9_./-]+', helper):
        raise ValueError('Invalid operator installation metadata')
    target = user + '@' + ('hope-mac-operator' if mac else config.laptop_wifi_ip)
    r = run(['runuser', '-u', 'agi', '--', '/usr/bin/ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=3',
             target, helper, 'config-status-laptop'], capture_output=True, text=True, timeout=5, check=True)
    return laptop_matches(config, json.loads(r.stdout))


def audit(config, run=subprocess.run):
    services = [service_status(unit, config, run) for unit in UNITS]
    try:
        laptop = laptop_status(config, run) if config.revision > 0 else {'matches': False, 'error': 'Configuration not saved'}
    except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as exc:
        laptop = {'matches': False, 'error': str(exc)[-500:]}
    stale = [s for s in services if not s['matches']]
    reason = '; '.join(s['service'] + ': ' + s.get('reason', '') for s in stale)
    if not laptop['matches']:
        reason += '; Laptop: ' + str(laptop.get('mismatches') or laptop.get('error') or 'display unavailable')
    applied = config.revision > 0 and not stale and laptop['matches']
    return {'boot_id': BOOT.read_text().strip(), 'monotonic_s': time.monotonic(), 'config_sha256': identity(config),
            'revision': config.revision, 'applied': applied, 'state': 'APPLIED' if applied else 'PENDING',
            'reason': reason.strip('; ') if not applied else 'All audited consumers match saved configuration',
            'services': services, 'laptop': laptop}


def reconcile(config, report, run=subprocess.run, attempted=None):
    units = [s['service'] for s in report['services'] if not s['matches'] and s['active'] == 'active']
    if not units or config.revision < 1:
        return
    lock = try_acquire_hardware_operation_lock()
    if lock is None:
        return
    try:
        if not no_mdu_control_processes(config, run):
            return
        for unit in units:
            attempt_key = (unit, identity(config))
            if attempted is not None and attempt_key in attempted:
                continue
            if unit not in UNITS:
                raise ValueError('Unexpected service in runtime audit')
            owner, calibration = json.loads(OWNER.read_text()), json.loads(CALIBRATION.read_text())
            if (not can_reconcile(owner, calibration, time.monotonic(), report['boot_id'], identity(config))
                    or identity(load_config(CONFIG)) != identity(config) or not no_control_processes()):
                return
            # try-restart does not activate a unit that stopped since the audit.
            if attempted is not None:
                attempted.add(attempt_key)
            run(['systemctl', 'try-restart', unit], check=True, timeout=30)
    finally:
        release_hardware_operation_lock(lock)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    ex = sub.add_parser('exec'); ex.add_argument('command', nargs=argparse.REMAINDER)
    sub.add_parser('audit'); sub.add_parser('serve'); sub.add_parser('peers')
    args = parser.parse_args()
    if args.action == 'exec':
        command = args.command[1:] if args.command[:1] == ['--'] else args.command
        if not command:
            parser.error('missing service command')
        os.execvpe(command[0], command, environment(load_config(CONFIG), os.environ))
    elif args.action == 'peers':
        print(network(load_config(CONFIG))['peers'])
    elif args.action == 'audit':
        print(json.dumps(audit(load_config(CONFIG)), sort_keys=True))
    else:
        attempted = set()  # At most one refresh per saved revision; no restart storm.
        while True:
            try:
                config = load_config(CONFIG)
                report = audit(config)
                atomic_json(REPORT, report)
                reconcile(config, report, attempted=attempted)
            except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as exc:
                print('CONFIG_RUNTIME_AUDIT_FAILED: ' + str(exc), flush=True)
            time.sleep(2)


if __name__ == '__main__':
    main()
