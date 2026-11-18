#!/usr/bin/env bash
# Apply the packet transport profile to this node only. Other mocap nodes keep
# their existing participants and transport configuration.
set -euo pipefail
[[ $# -ge 3 && "$2" == -- ]] || { echo 'Usage: with_flight_packet_qos.sh TOPIC -- COMMAND [ARG ...]' >&2; exit 64; }
topic="$1"
shift 2
if [[ -z "${ROS_STATIC_PEERS:-}" ]]; then
  # Standalone launches retain their normal discovery configuration.
  exec "$@"
fi
IFS=';' read -r -a peers <<< "$ROS_STATIC_PEERS"
args=()
for peer in "${peers[@]}"; do
  [[ -z "$peer" ]] || args+=(--peer "$peer")
done
exec "$(dirname -- "$0")/with_fastdds_unicast.sh" \
  --domain-id "${ROS_DOMAIN_ID:-232}" "${args[@]}" \
  --flight-packet-qos-topic "$topic" -- "$@"
