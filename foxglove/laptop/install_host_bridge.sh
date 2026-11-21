#!/usr/bin/env bash
# Install only the Host endpoint; never stop robot services or switch clients.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
command -v distrobox >/dev/null
[[ -r "$HOME/.config/hope-foxglove/laptop-runtime.env" ]]
distrobox enter hope -- bash -lc 'source /opt/ros/jazzy/setup.bash; ros2 pkg prefix foxglove_bridge >/dev/null; python3 -c "import yaml"'
mkdir -p "$HOME/.config/systemd/user" "$HOME/.config/hope-foxglove"
if [[ -e "$HOME/.config/hope-foxglove/host-bridge.env" ]]; then
  cp -p "$HOME/.config/hope-foxglove/host-bridge.env" "$HOME/.config/hope-foxglove/host-bridge.env.before-$(date +%Y%m%dT%H%M%S)"
fi
printf 'HOPE_ROOT=%s\n' "$root" > "$HOME/.config/hope-foxglove/host-bridge.env"
install -m 0644 "$root/foxglove/laptop/hope-host-bridge.service" "$HOME/.config/systemd/user/hope-host-bridge.service"
systemctl --user daemon-reload
systemctl --user enable --now hope-host-bridge.service
printf 'Host endpoint: ws://127.0.0.1:8767 (current lifecycle robot). Switch Foxglove only in a non-MOTION window.\n'
