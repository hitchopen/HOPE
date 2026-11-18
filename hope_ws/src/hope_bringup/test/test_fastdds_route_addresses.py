"""Exercise generated DDS profiles without creating ROS/network participants."""
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest

WRAPPER = Path(__file__).resolve().parents[1] / 'scripts/with_fastdds_unicast.sh'


def run_wrapper(tmp_path, args, *, missing_source=False):
    routes = {
        '192.168.100.250': '192.168.100.250 via 192.168.200.1 dev wifi_hdu src 192.168.200.202',
        '10.42.10.12': '10.42.10.12 dev eth_hdu src 10.42.10.10',
        '127.0.0.1': 'local 127.0.0.1 dev lo src 127.0.0.1',
        '192.168.200.1': '192.168.200.1 dev wifi_hdu src 192.168.200.202',
    }
    if missing_source:
        routes['10.42.10.12'] = '10.42.10.12 dev eth_hdu'
    fake_ip = tmp_path / 'ip'
    fake_ip.write_text('#!' + sys.executable + '\nimport sys\nroutes=' + repr(routes) +
                      '\nif sys.argv[1:3] == ["link", "show"]: sys.exit(0)\n'
                      'print(routes[sys.argv[-1]])\n')
    fake_ip.chmod(0o755)
    output = tmp_path / 'profile.xml'
    return subprocess.run(['bash', str(WRAPPER), *args, '--', sys.executable, '-c',
        'import os,pathlib,sys;pathlib.Path(sys.argv[1]).write_bytes('
        'pathlib.Path(os.environ["FASTDDS_DEFAULT_PROFILES_FILE"]).read_bytes())', str(output)],
        env={**os.environ, 'PATH': str(tmp_path) + os.pathsep + os.environ['PATH']},
        text=True, capture_output=True), output


def values(output, tag):
    root = ET.parse(output).getroot()
    return [e.text for e in root.iter() if e.tag.rsplit('}', 1)[-1] == tag]


def test_route_mode_keeps_all_peer_paths_and_excludes_device_aliases(tmp_path):
    result, output = run_wrapper(tmp_path, [
        '--route-source-addresses', '--peer', '192.168.100.250', '--peer', '10.42.10.12',
        '--peer', '127.0.0.1', '--peer', '10.42.10.12', '--flight-packet-qos-topic', '/ball/flight_packet'])
    assert result.returncode == 0, result.stderr
    assert values(output, 'interface') == ['192.168.200.202', '10.42.10.10', '127.0.0.1']
    assert set(values(output, 'address')) == {'192.168.100.250', '10.42.10.12', '127.0.0.1'}
    assert values(output, 'maxMessageSize') == []
    assert 'ASYNCHRONOUS' in values(output, 'kind')


def test_existing_default_and_explicit_interface_behavior_is_preserved(tmp_path):
    for arguments, expected in [
        (['--peer', '192.168.100.250', '--peer', '10.42.10.12'], ['wifi_hdu', 'eth_hdu']),
        (['--peer', '192.168.100.250', '--interface', 'wifi_hdu'], ['wifi_hdu']),
    ]:
        result, output = run_wrapper(tmp_path, arguments)
        assert result.returncode == 0, result.stderr
        assert values(output, 'interface') == expected


@pytest.mark.parametrize('peers', [
    ['192.168.200.1', '192.168.100.250', '10.42.10.12'],
    ['192.168.100.250', '192.168.200.1', '10.42.10.12'],
])
def test_direct_link_filter_never_filters_a_shared_routed_source(tmp_path, peers):
    arguments = ['--route-source-addresses', '--filter-direct-routes']
    for peer in peers:
        arguments.extend(['--peer', peer])
    result, output = run_wrapper(tmp_path, arguments)
    assert result.returncode == 0, result.stderr
    root = ET.parse(output).getroot()
    entries = [e.attrib for e in root.iter() if e.tag.endswith('}interface')]
    assert entries == [
        {'name': '192.168.200.202', 'netmask_filter': 'OFF'},
        {'name': '10.42.10.10', 'netmask_filter': 'ON'},
    ]
    assert values(output, 'maxMessageSize') == []


@pytest.mark.parametrize('arguments,missing_source,error', [
    (['--route-source-addresses', '--peer', '10.42.10.12'], True, 'no valid IPv4 source'),
    (['--route-source-addresses', '--peer', '10.42.10.12', '--interface', 'eth_hdu'], False, 'cannot be combined'),
    (['--filter-direct-routes', '--peer', '10.42.10.12'], False, 'requires --route-source-addresses'),
])
def test_incomplete_route_or_ambiguous_mode_never_starts_command(tmp_path, arguments, missing_source, error):
    result, output = run_wrapper(tmp_path, arguments, missing_source=missing_source)
    assert result.returncode != 0
    assert error in result.stderr
    assert not output.exists()
