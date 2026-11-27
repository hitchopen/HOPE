#!/bin/bash
# The named SSH host is the bridged Ubuntu guest running on this Mac.
set -euo pipefail
mode=${1:-view}
guest=${HOPE_FIELD_GUEST:-hope-field}
[[ "$guest" =~ ^[A-Za-z0-9][A-Za-z0-9_.@-]*$ ]] || { echo 'Invalid SSH host' >&2; exit 2; }
case "$mode" in
  view)
    port=8767
    forwards=(-L 127.0.0.1:8767:127.0.0.1:8767 -L 127.0.0.1:8768:127.0.0.1:8768 -L 127.0.0.1:8769:127.0.0.1:8769)
    ;;
  commission)
    hdu=${2:?pass the HDU IPv4 from the current venue}
    /usr/bin/python3 -c 'import ipaddress,sys; ipaddress.IPv4Address(sys.argv[1])' "$hdu"
    port=8766
    forwards=(-L "127.0.0.1:8766:$hdu:8766")
    ;;
  *) echo 'Usage: open_field.command [view|commission HDU-IP]' >&2; exit 2 ;;
esac
socket_dir=$(mktemp -d /tmp/hope-field.XXXXXXXX)
socket="$socket_dir/control"
cleanup() {
  ssh -S "$socket" -O exit "$guest" >/dev/null 2>&1 || true
  rmdir "$socket_dir" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
ssh -M -S "$socket" -o ControlPersist=no -o StrictHostKeyChecking=yes \
    -o ExitOnForwardFailure=yes -o ServerAliveInterval=10 -o ServerAliveCountMax=3 \
    -fNT "${forwards[@]}" "$guest"
open -a Foxglove "foxglove://open?ds=foxglove-websocket&ds.url=ws%3A%2F%2F127.0.0.1%3A$port"
echo 'Leave this window open for the field connection. Press Ctrl-C to disconnect.'
while ssh -S "$socket" -O check "$guest" >/dev/null 2>&1; do sleep 2; done
