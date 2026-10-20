#!/bin/bash
set -euo pipefail

[[ "$EUID" == 0 ]] || { echo 'Operator initialization requires root.' >&2; exit 64; }
# Both values are mandatory. The same validator is used before any account or
# filesystem setup and every time the SSH dispatcher reads its root-owned copy.
/usr/local/bin/hope-operator-dispatch --configure
operator_user="${HOPE_OPERATOR_USER:?missing HOPE_OPERATOR_USER}"
operator_home="/home/$operator_user"
operator_root="$operator_home/HOPE"
operator_group=hope
if ! getent group "$operator_group" >/dev/null; then
  groupadd "$operator_group"
fi
if ! id "$operator_user" >/dev/null 2>&1; then
  # Use a dedicated group: Ubuntu's pre-existing "operator" system group is
  # unrelated to the HOPE login account and must not be reused or replaced.
  useradd --create-home --gid "$operator_group" --home-dir "$operator_home" --shell /bin/bash "$operator_user"
fi
[[ "$(id -u "$operator_user")" -ge 1000 ]] || {
  echo 'Refusing a system account for the operator.' >&2; exit 64;
}
[[ "$(getent passwd "$operator_user" | cut -d: -f6)" == "$operator_home" ]] || {
  echo 'Existing operator home does not match the configured volume.' >&2; exit 64;
}
if [[ "$(id -gn "$operator_user")" != "$operator_group" ]]; then
  usermod --gid "$operator_group" "$operator_user"
fi
# Unlock the account for public-key authentication only. sshd explicitly rejects
# passwords and empty passwords, as well as root, PTYs and forwarding.
passwd -d "$operator_user" >/dev/null
mkdir -p /run/sshd "$operator_home/.ssh" "$operator_home/.config/hope-foxglove" \
  "$operator_home/.local/libexec" "$operator_home/.local/state/hope-sim2real" \
  "$operator_root/hope_ws" /work/real_logs /work/artifacts/macos_setup/operator-calibration /operator-state/hostkeys
for path in /work/* /work/.[!.]*; do
  [[ -e "$path" || -L "$path" ]] || continue
  name="$(basename "$path")"
  [[ "$name" == hope_ws || "$name" == calibration ]] && continue
  ln -sfn "$path" "$operator_root/$name"
done
# Keep current-machine calibration separate from any checkout's calibration.
ln -sfn /work/artifacts/macos_setup/operator-calibration "$operator_root/calibration"
for name in src calibration_receipts; do
  ln -sfn "/work/hope_ws/$name" "$operator_root/hope_ws/$name"
done
for name in install build log; do
  ln -sfn "/opt/hope/ros/$name" "$operator_root/hope_ws/$name"
done
ln -sfn /work/foxglove/helpers/hope-lifecycle "$operator_home/.local/libexec/hope-lifecycle"
{
  printf 'export HOPE_ROOT=%q\n' "$operator_root"
  printf 'export HOPE_LAPTOP_USER=%q\n' "$operator_user"
  printf 'export HOPE_ROBOT_USER=agi\n'
  printf 'export HOPE_LAPTOP_RUNTIME=direct\n'
  printf 'export HOPE_LAPTOP_SHARE=/work/foxglove/laptop\n'
} > "$operator_home/.config/hope-foxglove/lifecycle.env"
install -m 600 /operator-config/client_key "$operator_home/.ssh/id_ed25519"
install -m 600 /operator-config/known_hosts "$operator_home/.ssh/known_hosts"
printf 'Host *\n  BatchMode yes\n  StrictHostKeyChecking yes\n  ConnectTimeout 5\n' > "$operator_home/.ssh/config"
printf 'restrict,command="/usr/local/bin/hope-operator-dispatch" %s\n' \
  "$(cat /operator-config/hdu.pub)" > "$operator_home/.ssh/authorized_keys"
chmod 700 "$operator_home/.ssh"
chmod 600 "$operator_home/.ssh/authorized_keys" "$operator_home/.ssh/config"
chown -R "$operator_user:$operator_group" "$operator_home"
if [[ ! -f /operator-state/hostkeys/ssh_host_ed25519_key ]]; then
  ssh-keygen -q -t ed25519 -N '' -f /operator-state/hostkeys/ssh_host_ed25519_key
fi
{
  printf 'AllowUsers %s\n' "$operator_user"
  cat <<'EOF'
Port 2222
ListenAddress 0.0.0.0
HostKey /operator-state/hostkeys/ssh_host_ed25519_key
PasswordAuthentication no
KbdInteractiveAuthentication no
PermitEmptyPasswords no
PermitRootLogin no
UsePAM no
AllowTcpForwarding no
X11Forwarding no
PermitTTY no
AuthorizedKeysFile .ssh/authorized_keys
PidFile /run/hope-sshd.pid
EOF
} > /run/hope-sshd.conf
exec /usr/sbin/sshd -D -e -f /run/hope-sshd.conf
