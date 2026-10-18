#!/bin/bash
set -euo pipefail
operator_home=/home/dongc1
operator_root="$operator_home/HOPE"
mkdir -p /run/sshd "$operator_home/.ssh" "$operator_home/.config/hope-foxglove" \
  "$operator_home/.local/libexec" "$operator_home/.local/state/hope-sim2real" \
  "$operator_root/hope_ws" /work/real_logs /work/artifacts/macos_setup/operator-calibration /operator-state/hostkeys
for path in /work/* /work/.[!.]*; do
  [[ -e "$path" || -L "$path" ]] || continue
  name="$(basename "$path")"
  [[ "$name" == hope_ws || "$name" == calibration ]] && continue
  ln -sfn "$path" "$operator_root/$name"
done
# A checkout's calibration may belong to another physical robot. Start with a
# separate persistent folder and populate it only through attended calibration.
ln -sfn /work/artifacts/macos_setup/operator-calibration "$operator_root/calibration"
for name in src calibration_receipts; do
  ln -sfn "/work/hope_ws/$name" "$operator_root/hope_ws/$name"
done
for name in install build log; do
  ln -sfn "/opt/hope/ros/$name" "$operator_root/hope_ws/$name"
done
ln -sfn /work/foxglove/helpers/hope-lifecycle "$operator_home/.local/libexec/hope-lifecycle"
cat > "$operator_home/.config/hope-foxglove/lifecycle.env" <<'EOF'
export HOPE_ROOT=/home/dongc1/HOPE
export HOPE_LAPTOP_USER=dongc1
export HOPE_ROBOT_USER=agi
export HOPE_LAPTOP_RUNTIME=direct
export HOPE_LAPTOP_SHARE=/work/foxglove/laptop
EOF
install -m 600 /operator-config/client_key "$operator_home/.ssh/id_ed25519"
install -m 600 /operator-config/known_hosts "$operator_home/.ssh/known_hosts"
printf 'Host *\n  BatchMode yes\n  StrictHostKeyChecking yes\n  ConnectTimeout 5\n' > "$operator_home/.ssh/config"
printf 'restrict,command="/usr/local/bin/hope-operator-dispatch" %s\n' \
  "$(cat /operator-config/hdu.pub)" > "$operator_home/.ssh/authorized_keys"
chmod 700 "$operator_home/.ssh"
chmod 600 "$operator_home/.ssh/authorized_keys" "$operator_home/.ssh/config"
chown -R dongc1:dongc1 "$operator_home"
if [[ ! -f /operator-state/hostkeys/ssh_host_ed25519_key ]]; then
  ssh-keygen -q -t ed25519 -N '' -f /operator-state/hostkeys/ssh_host_ed25519_key
fi
cat > /run/hope-sshd.conf <<'EOF'
Port 2222
ListenAddress 0.0.0.0
HostKey /operator-state/hostkeys/ssh_host_ed25519_key
PasswordAuthentication no
KbdInteractiveAuthentication no
PermitEmptyPasswords no
PermitRootLogin no
AllowUsers dongc1
UsePAM no
AllowTcpForwarding no
X11Forwarding no
PermitTTY no
AuthorizedKeysFile .ssh/authorized_keys
PidFile /run/hope-sshd.pid
EOF
exec /usr/sbin/sshd -D -e -f /run/hope-sshd.conf
