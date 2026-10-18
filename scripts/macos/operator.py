#!/usr/bin/env python3
"""Mac operator services; never starts HAL, Runner, motion, or time calibration."""
from __future__ import annotations

import argparse
import ipaddress
import json
from pathlib import Path
import subprocess

CONFIG = Path.home() / ".config/hope-macos"


def run(args, **kwargs):
    return subprocess.run([str(arg) for arg in args], check=True, **kwargs)


def ssh_args(config):
    return ["ssh", "-S", CONFIG / "tunnel.sock", "-i", CONFIG / "client_key", "-o", "IdentitiesOnly=yes",
            "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
            "-o", f"UserKnownHostsFile={CONFIG / 'known_hosts'}",
            "-o", "ConnectTimeout=8", f"{config['ssh_user']}@{config['hdu_ip']}"]


def remote(config, script, **kwargs):
    return run([*ssh_args(config), "bash -s"], input=script, text=True, **kwargs)


def up(config, *, scheduled=False):
    disabled = CONFIG / "disabled"
    if scheduled and disabled.exists():
        return
    if not scheduled:
        disabled.unlink(missing_ok=True)
    name = config["container"]
    probe = subprocess.run(["docker", "inspect", "--format", "{{.State.Running}}", name],
                           capture_output=True, text=True)
    if probe.returncode:
        run(["docker", "run", "-d", "--init", "--restart", "unless-stopped", "--name", name,
             "--platform", "linux/arm64", "-p", f"127.0.0.1:{config['ssh_port']}:2222",
             "--mount", f"type=bind,src={config['root']},dst=/work",
             "--mount", f"type=bind,src={CONFIG},dst=/operator-config,readonly",
             "--mount", "type=volume,src=hope-macos-arm64,dst=/opt/hope",
             "--mount", "type=volume,src=hope-macos-operator-home,dst=/home/dongc1",
             "--mount", "type=volume,src=hope-macos-operator-state,dst=/operator-state",
             "hope-operator:jazzy-arm64"])
    elif probe.stdout.strip() != "true":
        run(["docker", "start", name])
    control = CONFIG / "tunnel.sock"
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
    result = run(["docker", "exec", config["container"], "cat",
                  "/operator-state/hostkeys/ssh_host_ed25519_key.pub"], capture_output=True, text=True)
    hostkey = " ".join(result.stdout.split()[:2])
    settings = {key: config[key] for key in ("mac_ip", "linux_ip", "mdu_ip", "ssh_port")}
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
       f'  HostName 127.0.0.1\\n  Port {cfg["ssh_port"]}\\n  User dongc1\\n'
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
cfg.update(laptop_wifi_ip=laptop, operator_profile=profile)
request=SetParameters.Request()
request.parameters=[Parameter(name=k,value=ParameterValue(type=4,string_value=cfg[k])) for k in
 ('laptop_wifi_ip','hdu_wifi_ip','mdu_internal_ip','motive_ip','table_side','operator_profile')]
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
    config=json.loads((CONFIG/"operator.json").read_text())
    for key in ("hdu_ip","mac_ip","linux_ip","mdu_ip"):
        ipaddress.IPv4Address(config[key])
    if args.action=="up":up(config)
    elif args.action=="ensure":up(config, scheduled=True)
    elif args.action=="bind":bind(config)
    elif args.action.startswith("select-"):select(config,args.action.split('-')[1])
    elif args.action=="status":
        run(["docker","inspect","--format","{{.State.Status}}",config['container']])
        run(["ssh","-S",CONFIG/'tunnel.sock',"-O","check",f"{config['ssh_user']}@{config['hdu_ip']}"])
        remote(config,"cat /var/lib/hope-lifecycle/config.json\n")
    else:
        remote(config,"python3 - <<'PY'\nimport json\nfrom pathlib import Path\nc=json.loads(Path('/var/lib/hope-lifecycle/config.json').read_text())\nassert c.get('operator_profile') == 'linux', 'Select the Linux operator with the updated supervisor before stopping the Mac backend'\nPY\n")
        (CONFIG/'disabled').touch(mode=0o600)
        run(["ssh","-S",CONFIG/'tunnel.sock',"-O","exit",f"{config['ssh_user']}@{config['hdu_ip']}"])
        run(["docker","stop",config['container']])


if __name__=="__main__":
    main()
