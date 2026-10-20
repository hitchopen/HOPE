#!/usr/bin/env python3
"""Mac operator services; never starts HAL, Runner, motion, or time calibration."""
from __future__ import annotations

# This CLI's filename matches a stdlib module imported by argparse. Keep the
# script directory out of import lookup when executed directly on any Mac.
import sys
if __name__ == "__main__" and sys.path:
    sys.path.pop(0)

import argparse
import hashlib
import ipaddress
import json
from pathlib import Path
import re
import subprocess

CONFIG = Path.home() / ".config/hope-macos"
CONTRACT_LABEL = "org.hope.operator.contract"
CONTRACT_VERSION = "3"


def validate_identity(user, helper):
    if not isinstance(user, str) or user == "root" or not re.fullmatch(r"[a-z_][a-z0-9_-]{0,31}", user):
        raise ValueError("laptop_user must be a non-root POSIX account matching the HDU supervisor")
    if (not isinstance(helper, str) or not re.fullmatch(r"/[A-Za-z0-9_./-]+", helper)
            or any(part in (".", "..", "") for part in helper.split("/")[1:])):
        raise ValueError("laptop_helper must be the supervisor's exact absolute helper path, without traversal")


def validate_config(config):
    # These are container identities, never inferred from the Mac login/admin.
    for key in ("laptop_user", "laptop_helper"):
        if key not in config:
            raise ValueError(f"Missing {key} in operator.json; match the supervisor's Mac profile (see lifecycle README)")
    validate_identity(config["laptop_user"], config["laptop_helper"])
    for key in ("hdu_ip", "mac_ip", "linux_ip", "mdu_ip"):
        ipaddress.IPv4Address(config[key])
    # Sequential operators may reuse an address: profile/SSH alias, not IP,
    # determines identity. Simultaneously connected hosts still need unique IPs.
    if not re.fullmatch(r"[a-z_][a-z0-9_-]{0,31}", config["ssh_user"]):
        raise ValueError("ssh_user must be a POSIX account")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", config["container"]):
        raise ValueError("Invalid Docker container name")
    if type(config["ssh_port"]) is not int or not 1024 <= config["ssh_port"] <= 65535:
        raise ValueError("ssh_port must be an integer between 1024 and 65535")
    root = Path(config["root"]).expanduser()
    if not root.is_absolute() or not (root / "foxglove/helpers/hope-lifecycle").is_file():
        raise ValueError("root must be the absolute Mac path to this HOPE checkout")
    config["root"] = str(root.resolve())
    return config


def run(args, **kwargs):
    return subprocess.run([str(arg) for arg in args], check=True, **kwargs)


def tunnel_socket(config):
    # An explicit ControlPath otherwise reuses a master for the old HDU/port
    # after operator.json changes, regardless of the destination on ssh's argv.
    identity = json.dumps([config["ssh_user"], config["hdu_ip"], config["ssh_port"]])
    suffix = hashlib.sha256(identity.encode()).hexdigest()[:16]
    return CONFIG / f"tunnel-{suffix}.sock"


def ssh_args(config):
    return ["ssh", "-S", tunnel_socket(config), "-i", CONFIG / "client_key", "-o", "IdentitiesOnly=yes",
            "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
            "-o", f"UserKnownHostsFile={CONFIG / 'known_hosts'}",
            "-o", "ConnectTimeout=8", f"{config['ssh_user']}@{config['hdu_ip']}"]


def remote(config, script, **kwargs):
    return run([*ssh_args(config), "bash -s"], input=script, text=True, **kwargs)


SUPERVISOR_CONTRACT_SCRIPT = """python3 - <<'PY'
import json
from pathlib import Path
import subprocess
pid = int(subprocess.check_output([
    'systemctl', 'show', '--property=MainPID', '--value',
    'hope-lifecycle-supervisor.service'], text=True).strip())
if pid <= 0:
    raise RuntimeError('The HDU lifecycle supervisor must be running to verify its Mac profile identity')
try:
    entries = Path(f'/proc/{pid}/environ').read_bytes().split(b'\\0')
except OSError as error:
    raise RuntimeError('Cannot read the live supervisor identity; run as its service account') from error
env = dict(entry.split(b'=', 1) for entry in entries if b'=' in entry)
# Only these non-secret contract fields leave the HDU, never the full environment.
print(json.dumps({
    'laptop_user': env.get(b'HOPE_MAC_USER', b'operator').decode().strip(),
    'laptop_helper': env.get(b'HOPE_MAC_LIFECYCLE_HELPER',
                           b'/home/operator/.local/libexec/hope-lifecycle').decode()}))
PY
"""


def verify_supervisor_contract(config):
    result = remote(config, SUPERVISOR_CONTRACT_SCRIPT, capture_output=True)
    actual = json.loads(result.stdout)
    expected = {key: config[key] for key in ("laptop_user", "laptop_helper")}
    if actual != expected:
        raise RuntimeError(f"Mac laptop identity {expected} differs from the running supervisor {actual}. "
                           "Match operator.json to HOPE_MAC_USER/HOPE_MAC_LIFECYCLE_HELPER; do not change Linux settings.")
    return actual


def container_info(config):
    probe = subprocess.run(["docker", "inspect", "--format", "{{json .}}", config["container"]],
                           capture_output=True, text=True)
    return None if probe.returncode else json.loads(probe.stdout)


def verify_container_contract(config, info):
    if info is None:
        raise RuntimeError("Mac operator container is missing; run operator.py up first")
    env = dict(value.split("=", 1) for value in info["Config"].get("Env", []) if "=" in value)
    mounts = {value["Destination"]: value for value in info.get("Mounts", [])}
    expected_home = f"/home/{config['laptop_user']}"
    labels = info["Config"].get("Labels") or {}
    valid = (labels.get(CONTRACT_LABEL) == CONTRACT_VERSION
             and env.get("HOPE_OPERATOR_USER") == config["laptop_user"]
             and env.get("HOPE_OPERATOR_HELPER") == config["laptop_helper"]
             and mounts.get("/work", {}).get("Source") == str(Path(config["root"]).resolve())
             and mounts.get("/operator-config", {}).get("Source") == str(CONFIG.resolve())
             and mounts.get(expected_home, {}).get("Name") == "hope-macos-operator-home")
    ports = info.get("HostConfig", {}).get("PortBindings", {}).get("2222/tcp")
    valid = valid and ports == [{"HostIp": "127.0.0.1", "HostPort": str(config["ssh_port"])}]
    if not valid:
        raise RuntimeError("Existing Mac container has an old/different account, helper, checkout or port. "
                           "It was not changed. While STOPPED, select Linux, stop/remove only the named "
                           "container (keep its volumes), rebuild the image and run up; see the lifecycle README.")


def up(config, *, scheduled=False):
    disabled = CONFIG / "disabled"
    if scheduled and disabled.exists():
        return
    info = container_info(config)
    if info is not None:
        verify_container_contract(config, info)
    if not scheduled:
        disabled.unlink(missing_ok=True)
    name = config["container"]
    if info is None:
        image = run(["docker", "image", "inspect", "--format", "{{json .Config.Labels}}",
                     "hope-operator:jazzy-arm64"], capture_output=True, text=True)
        if (json.loads(image.stdout) or {}).get(CONTRACT_LABEL) != CONTRACT_VERSION:
            raise RuntimeError("Rebuild the Mac operator image with this checkout before running up")
        run(["docker", "run", "-d", "--init", "--restart", "unless-stopped", "--name", name,
             "-e", f"HOPE_OPERATOR_USER={config['laptop_user']}",
             "-e", f"HOPE_OPERATOR_HELPER={config['laptop_helper']}",
             "--platform", "linux/arm64", "-p", f"127.0.0.1:{config['ssh_port']}:2222",
             "--mount", f"type=bind,src={config['root']},dst=/work",
             "--mount", f"type=bind,src={CONFIG},dst=/operator-config,readonly",
             "--mount", "type=volume,src=hope-macos-arm64,dst=/opt/hope",
             "--mount", f"type=volume,src=hope-macos-operator-home,dst=/home/{config['laptop_user']}",
             "--mount", "type=volume,src=hope-macos-operator-state,dst=/operator-state",
             "hope-operator:jazzy-arm64"])
    elif not info["State"]["Running"]:
        run(["docker", "start", name])
    control = tunnel_socket(config)
    check = subprocess.run(["ssh", "-S", str(control), "-O", "check",
                            f"{config['ssh_user']}@{config['hdu_ip']}"], capture_output=True)
    if check.returncode:
        base = ssh_args(config)
        run([*base[:-1], "-M", "-S", control, "-fNT", "-o", "ExitOnForwardFailure=yes",
             "-o", "ServerAliveInterval=15", "-o", "ServerAliveCountMax=3",
             "-R", f"127.0.0.1:{config['ssh_port']}:127.0.0.1:{config['ssh_port']}", base[-1]])
    print("Mac operator container and loopback-only reverse SSH tunnel are running.")


def bind(config):
    """Add only the Mac route; retain the Linux address/account and existing entries."""
    verify_supervisor_contract(config)
    verify_container_contract(config, container_info(config))
    result = run(["docker", "exec", config["container"], "cat",
                  "/operator-state/hostkeys/ssh_host_ed25519_key.pub"], capture_output=True, text=True)
    hostkey = " ".join(result.stdout.split()[:2])
    settings = {key: config[key] for key in ("mac_ip", "linux_ip", "mdu_ip", "ssh_port", "laptop_user")}
    settings["hostkey"] = hostkey
    remote(config, _bind_script(settings))
    # Preserve Linux discovery while admitting the Mac address for future sessions.
    remote(config, "sudo -n python3 - <<'PY'\npeers=" + repr(
        [config['linux_ip'], config['mac_ip'], config['mdu_ip']]) + "\n" + '''
from pathlib import Path
from datetime import datetime, timezone
import shutil
p=Path('/etc/hope-foxglove/network.env');old=p.read_text();lines=old.splitlines()
for i,line in enumerate(lines):
    if line.startswith('ROS_STATIC_PEERS='):
        current=line.split('=',1)[1].split(';')
        lines[i]='ROS_STATIC_PEERS='+';'.join(dict.fromkeys(current+peers))
        break
else:raise RuntimeError('Missing ROS_STATIC_PEERS; refusing to guess the service configuration')
new='\\n'.join(lines)+'\\n'
if old!=new:
    shutil.copy2(p,str(p)+'.before-macos-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    p.write_text(new)
print('DDS configuration includes both operator addresses; existing services were not restarted.')
PY
''')


def _bind_script(settings):
    # Serialize input as Python data, then send the script on SSH stdin, not shell interpolation.
    return "python3 - <<'PY'\ncfg=" + repr(settings) + "\n" + '''
from pathlib import Path
import datetime
ssh=Path.home()/'.ssh'; ssh.mkdir(mode=0o700,exist_ok=True)
stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
path=ssh/'config'; original=path.read_text() if path.exists() else ''; old=original
start='# BEGIN HOPE MAC OPERATOR'; end='# END HOPE MAC OPERATOR'
if start in old:
    begin=old.index(start); finish=old.index(end,begin)+len(end)
    old=old[:begin]+old[finish:].lstrip('\\n')
block=(f'{start}\\nHost hope-mac-operator\\n'
       f'  HostName 127.0.0.1\\n  Port {cfg["ssh_port"]}\\n  User {cfg["laptop_user"]}\\n'
       '  IdentityFile ~/.ssh/id_ed25519\\n  IdentitiesOnly yes\\n'
       '  ControlMaster auto\\n  ControlPersist 60\\n'
       '  ControlPath /var/lib/hope-lifecycle/mac-ssh-%C\\n'
       f'  HostKeyAlias hope-mac-operator\\n  StrictHostKeyChecking yes\\n{end}\\n')
if path.exists(): (ssh/f'config.before-macos-{stamp}').write_text(original)
path.write_text(block+'\\n'+old); path.chmod(0o600)
known=ssh/'known_hosts'; entries=known.read_text().splitlines() if known.exists() else []
entries=[line for line in entries if not line.startswith('hope-mac-operator ')]
entries.append('hope-mac-operator '+cfg['hostkey']);known.write_text('\\n'.join(entries)+'\\n');known.chmod(0o600)
print('HDU Mac route installed; original Linux route retained.')
PY
'''


def select(config, profile):
    if profile not in {"mac", "linux"}:
        raise ValueError("operator profile must be mac or linux")
    if profile == "mac":
        verify_supervisor_contract(config)
        verify_container_contract(config, container_info(config))
    ip = config[profile + "_ip"]
    remote(config, _select_script(ip, profile))


def _select_script(ip, profile):
    if profile not in {"mac", "linux"}:
        raise ValueError("operator profile must be mac or linux")
    return ("source /opt/ros/jazzy/setup.bash\nexport ROS_DOMAIN_ID=232\n" +
           "python3 - <<'PY'\n" + "laptop=" + repr(ip) + "\nprofile=" + repr(profile) + "\n" + '''
import time
import rclpy
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
rclpy.init(); node=rclpy.create_node('hope_mac_select_operator')
values={}; sub=node.create_subscription(String,'/hope/lifecycle/state',lambda m:values.update(state=m.data),10)
deadline=time.monotonic()+8
while 'state' not in values and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.2)
if values.get('state')!='STOPPED':raise RuntimeError('operator change requires fresh STOPPED state')
# Retain all other venue fields from the supervisor configuration.
import json
from pathlib import Path
cfg=json.loads(Path('/var/lib/hope-lifecycle/config.json').read_text())
if cfg.get('schema_version', 0) < 6:
    raise RuntimeError('HDU operator selection needs the combined schema-6 backend; '
                       'schema-5 venue settings were left unchanged. Build/stage the matching '
                       'release first; do not replace it with the old schema-3 files.')
cfg.update(laptop_wifi_ip=laptop, operator_profile=profile)
request=SetParameters.Request()
request.parameters=[Parameter(name=k,value=ParameterValue(type=4,string_value=cfg[k])) for k in
 ('laptop_wifi_ip','hdu_wifi_ip','mdu_internal_ip','motive_ip','vrpn_ip','table_side',
  'gripper_profile','pants_profile','mocap_source','vrpn_port','vrpn_units','operator_profile')]
client=node.create_client(SetParameters,'/hope/lifecycle/apply_config')
if not client.wait_for_service(timeout_sec=5):raise RuntimeError('configuration service unavailable')
future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=10)
result=future.result()
if result is None or not result.results or not all(x.successful for x in result.results):raise RuntimeError(str(result))
print(result);node.destroy_node();rclpy.shutdown()
PY
''')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action",choices=["up","ensure","bind","status","select-mac","select-linux","down"])
    args=parser.parse_args()
    config=validate_config(json.loads((CONFIG/"operator.json").read_text()))
    if args.action=="up":up(config)
    elif args.action=="ensure":up(config, scheduled=True)
    elif args.action=="bind":bind(config)
    elif args.action.startswith("select-"):select(config,args.action.split('-')[1])
    elif args.action=="status":
        run(["docker","inspect","--format","{{.State.Status}}",config['container']])
        run(["ssh","-S",tunnel_socket(config),"-O","check",f"{config['ssh_user']}@{config['hdu_ip']}"])
        remote(config,"cat /var/lib/hope-lifecycle/config.json\n")
    else:
        remote(config,"python3 - <<'PY'\nimport json\nfrom pathlib import Path\nc=json.loads(Path('/var/lib/hope-lifecycle/config.json').read_text())\nassert c.get('operator_profile') == 'linux', 'Select the Linux operator before stopping the Mac backend (updated supervisor required)'\nPY\n")
        (CONFIG/'disabled').touch(mode=0o600)
        control = ["ssh", "-S", tunnel_socket(config), "-O"]
        host = f"{config['ssh_user']}@{config['hdu_ip']}"
        if subprocess.run([*map(str, control), "check", host], capture_output=True).returncode == 0:
            try:
                run([*control, "exit", host])
            finally:
                # The master can disappear after check; still stop this backend.
                run(["docker","stop",config['container']])
        else:
            run(["docker","stop",config['container']])


if __name__=="__main__":
    main()
