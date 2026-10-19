"""Configuration and actual Laptop dispatch, with no ROS/hardware processes."""
import os
from pathlib import Path
import subprocess
import sys

import pytest

FOXGLOVE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FOXGLOVE / "a3"))
from hope_lifecycle_core import LifecycleConfig, apply_config_updates, config_from_document, config_to_document


def configured(**updates):
    base = LifecycleConfig(laptop_wifi_ip="10.42.20.1", hdu_wifi_ip="10.42.20.10",
                           mdu_internal_ip="10.42.10.12", revision=37,
                           gripper_profile="new_20261015", pants_profile="v4")
    return apply_config_updates(base, list({**base.values(), **updates}.items()))


@pytest.mark.parametrize("source", ["none", "motive", "vrpn"])
def test_source_roundtrip_and_separate_endpoints(source):
    config = configured(mocap_source=source, motive_ip="169.254.10.221",
                        vrpn_ip="192.168.100.99", vrpn_port="3884", vrpn_units="m")
    assert config_from_document(config_to_document(config)) == config
    assert config.revision == 38 and config.pants_profile == "v4"
    assert config.gripper_profile == "new_20261015"


@pytest.mark.parametrize("version", [1, 2, 3, 4])
@pytest.mark.parametrize("motive_ip", ["", "169.254.10.221"])
def test_legacy_upgrade_preserves_settings_without_guessing_vrpn(version, motive_ip):
    document = config_to_document(configured(motive_ip=motive_ip))
    document["schema_version"] = version
    for name in ("operator_profile", "mocap_source", "vrpn_ip", "vrpn_port", "vrpn_units"):
        document.pop(name)
    migrated = config_from_document(document)
    assert migrated.mocap_source == "none"
    assert migrated.revision == 38 and migrated.motive_ip == motive_ip
    assert migrated.gripper_profile == ("legacy" if version < 3 else "new_20261015")
    assert migrated.pants_profile == ("v2" if version < 4 else "v4")


@pytest.mark.parametrize("changes", [
    {"mocap_source": "auto"}, {"mocap_source": "motive"}, {"mocap_source": "vrpn"},
    {"vrpn_port": "0"}, {"vrpn_port": "65536"}, {"vrpn_port": "03883"},
    {"vrpn_port": "$(whoami)"}, {"vrpn_port": 3883}, {"vrpn_units": "cm"},
    {"vrpn_ip": "127.0.0.1"},
])
def test_invalid_or_incomplete_setup_rejected(changes):
    with pytest.raises(ValueError):
        configured(**changes)


def test_helper_dispatch_keeps_selected_setup_and_rejects_second_stack():
    helper = (FOXGLOVE / "helpers/hope-lifecycle").read_text()
    parser = helper[helper.index("valid_ipv4() {"):helper.index("require_user() {")]
    starter = helper[helper.index("validate_laptop_mocap() {"):helper.index("run_base() {")]
    stubs = '''
LAPTOP_USER=unused LAPTOP_SESSION=hope-optitrack SELF=helper
require_user() { :; }
die() { echo "$*"; exit 1; }
pgrep() { return "${EXISTING_STACK:-1}"; }
start_tmux() { printf '%s\n' "$@"; }
sleep() { :; }
tmux_exists() { return 0; }
emit() { :; }
'''
    common = ["model21800_20261015T010203Z", "10.42.20.1", "10.42.20.10", "10.42.10.12",
              "NONE", "P2", "OPTITRACK", "DEFAULT", "new_20261015", "vrpn",
              "192.168.100.99", "3884", "m", "v4"]
    code = "set -eu\n" + parser + starter + stubs + '\nparse_common "$@"; start_laptop'
    result = subprocess.run(["bash", "-c", code, "test", *common], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines()[-5:] == common[-5:]
    result = subprocess.run(["bash", "-c", code, "test", *common], capture_output=True,
                            text=True, env={**os.environ, "EXISTING_STACK": "0"})
    assert result.returncode != 0 and "UNMANAGED_BRIDGE_PRESENT" in result.stdout


@pytest.mark.parametrize("units,scale", [("mm", "0.001"), ("m", "1.0")])
def test_real_runtime_selects_only_vrpn_without_natnet_assets(tmp_path, units, scale):
    root = tmp_path / "repo"
    ws = root / "hope_ws"
    (ws / "install").mkdir(parents=True)
    (ws / "install/local_setup.bash").write_text("")
    ros_setup = tmp_path / "ros_setup.bash"
    ros_setup.write_text("")
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    capture = tmp_path / "args"
    for name, body in {
        "ros2": '[[ "$1" == daemon ]] || printf "%s\\n" "$@" > "$MOCAP_TEST_ARGS"',
        "sha256sum": "exit 0",
    }.items():
        script = bin_dir / name
        script.write_text("#!/bin/bash\n" + body + "\n")
        script.chmod(0o755)
    wrap = ws / "install/hope_bringup/lib/hope_bringup/with_fastdds_unicast.sh"
    wrap.parent.mkdir(parents=True)
    wrap.write_text('#!/bin/bash\nshift 5\nexec "$@"\n')
    wrap.chmod(0o755)
    env_file = tmp_path / "runtime.env"
    env_file.write_text(f'''HOPE_LIFECYCLE_ROOT='{root}'
HOPE_LIFECYCLE_SESSION_ID=model21800_20261015T010203Z
HOPE_LIFECYCLE_MOCAP_SOURCE=vrpn
HOPE_LIFECYCLE_HDU_IP=10.42.20.10
HOPE_LIFECYCLE_VRPN_IP=192.168.100.99
HOPE_LIFECYCLE_VRPN_PORT=3884
HOPE_LIFECYCLE_VRPN_UNITS={units}
HOPE_LIFECYCLE_PANTS_PROFILE=v4
HOPE_LIFECYCLE_TABLE_SIDE=P2
''')
    source = (FOXGLOVE / "helpers/hope-laptop-container-runtime").read_text()
    source = source.replace('"$HOME/.config/hope-foxglove/laptop-runtime.env"', f"'{env_file}'")
    source = source.replace("/opt/ros/jazzy/setup.bash", str(ros_setup))
    result = subprocess.run(["bash", "-c", source], capture_output=True, text=True,
                            env={**os.environ, "PATH": f"{bin_dir}:{os.environ['PATH']}",
                                 "MOCAP_TEST_ARGS": str(capture)}, timeout=10)
    assert result.returncode == 0, result.stderr
    args = capture.read_text().splitlines()
    assert args[:3] == ["launch", "hope_bringup", "avatar_pro_hope_bridge.launch.py"]
    for arg in ("server:=192.168.100.99", "port:=3884", f"position_scale:={scale}",
                "pants_profile:=v4", "table_side:=P2", "pelvis_asset_confirmed:=false",
                "world_frame_confirmed:=false", "follow_lifecycle_config:=true"):
        assert arg in args

@pytest.mark.parametrize('version', [3, 4])
def test_main_schema_migration_preserves_mac_natnet_and_gripper(version):
    document = dict(schema_version=version, revision=7, laptop_wifi_ip='10.42.20.1',
                    hdu_wifi_ip='10.42.20.10', mdu_internal_ip='10.42.10.12',
                    motive_ip='169.254.10.221', table_side='P2', operator_profile='mac')
    if version == 4:
        document['gripper_profile'] = 'new_20261015'
    config = config_from_document(document)
    assert config.operator_profile == 'mac' and config.table_side == 'P2'
    assert config.motive_ip == '169.254.10.221' and config.mocap_source == 'none'
    assert config.gripper_profile == ('legacy' if version == 3 else 'new_20261015')
    assert config.pants_profile == 'v2'


def test_field_schema_five_and_old_mac_request_keep_new_choices():
    from hope_lifecycle_core import OPERATOR_CONFIG_FIELDS
    config = configured(operator_profile='mac', mocap_source='vrpn', vrpn_ip='192.168.100.99')
    doc = config_to_document(config)
    doc['schema_version'] = 5
    del doc['operator_profile']
    migrated = config_from_document(doc)
    assert migrated.operator_profile == 'linux' and migrated.pants_profile == 'v4'
    assert migrated.mocap_source == 'vrpn'
    old = [(name, config.values()[name]) for name in OPERATOR_CONFIG_FIELDS]
    changed = apply_config_updates(config, old)
    assert changed.values() == config.values()
