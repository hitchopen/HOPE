#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  with_fastdds_unicast.sh [options] -- COMMAND [ARG ...]

Runs COMMAND with a generated Fast DDS profile that adds an explicit list of
unicast peers (multicast discovery itself stays enabled at SUBNET range; set
ROS_AUTOMATIC_DISCOVERY_RANGE=OFF yourself to fully disable it) — for venue Wi-Fi /
segmented LANs where DDS multicast discovery does not work (a common setup
when a laptop bridges the OptiTrack/Motive LAN to the robot's network).

Options:
  --peer IPV4              Remote Fast DDS peer. Repeat for every remote box.
  --interface NAME         Local DDS interface. Repeat as needed. By default,
                           interfaces are derived from `ip route get PEER`.
  --domain-id N            ROS domain (default: $ROS_DOMAIN_ID or 0).
  --max-initial-peers N    Participant ports probed per peer (default: 32).
  -h, --help               Show this help.

Example for the OptiTrack laptop-bridge topology:
  # Laptop: independently built NatNet2ROS2 adapter -> robot host.
  with_fastdds_unicast.sh --peer <ROBOT_HOST_IP> -- \
    ros2 launch motion_capture_tracking natnet2ros2.launch.py \
      hostname:=<MOTIVE_PC_IP> interface_ip:=<ADAPTER_MOTIVE_NIC_IP>

  # Robot host: source NatNet2ROS2 interfaces, then run the HOPE relay/planner.
  with_fastdds_unicast.sh --peer <LAPTOP_IP> -- \
    ros2 launch hope_bringup hope_bringup.launch.py mocap_backend:=optitrack

When the venue Wi-Fi changes, replace only the peer IPs. The wrapper derives
each local interface and address from the active route; no XML edit is needed.
EOF
}

die() {
  echo "[hope-fastdds] ERROR: $*" >&2
  exit 1
}

is_ipv4() {
  local value="$1"
  local a b c d extra octet
  IFS='.' read -r a b c d extra <<<"${value}"
  [[ -n "${a:-}" && -n "${b:-}" && -n "${c:-}" && -n "${d:-}" && -z "${extra:-}" ]] ||
    return 1
  for octet in "${a}" "${b}" "${c}" "${d}"; do
    [[ "${octet}" =~ ^[0-9]{1,3}$ ]] || return 1
    ((10#${octet} <= 255)) || return 1
  done
}

declare -a PEERS=()
declare -a REQUESTED_INTERFACES=()
declare -a COMMAND=()
DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
MAX_INITIAL_PEERS="32"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --peer)
      [[ $# -ge 2 ]] || die "--peer requires an IPv4 address"
      PEERS+=("$2")
      shift 2
      ;;
    --interface)
      [[ $# -ge 2 ]] || die "--interface requires a network interface name"
      REQUESTED_INTERFACES+=("$2")
      shift 2
      ;;
    --domain-id)
      [[ $# -ge 2 ]] || die "--domain-id requires an integer"
      DOMAIN_ID="$2"
      shift 2
      ;;
    --max-initial-peers)
      [[ $# -ge 2 ]] || die "--max-initial-peers requires an integer"
      MAX_INITIAL_PEERS="$2"
      shift 2
      ;;
    --)
      shift
      COMMAND=("$@")
      break
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown argument '$1' (COMMAND must follow --)"
      ;;
  esac
done

[[ ${#PEERS[@]} -gt 0 ]] || die "at least one --peer is required"
[[ ${#COMMAND[@]} -gt 0 ]] || die "COMMAND is required after --"
[[ "${DOMAIN_ID}" =~ ^[0-9]+$ ]] || die "--domain-id must be an integer"
((10#${DOMAIN_ID} <= 232)) || die "--domain-id must be in [0, 232] for UDP port safety"
[[ "${MAX_INITIAL_PEERS}" =~ ^[1-9][0-9]*$ ]] ||
  die "--max-initial-peers must be a positive integer"

# Fast DDS discovery-unicast port for the last probed participant must remain
# within uint16. This matters near domain 232, whose base port is already
# 65400.
LAST_DISCOVERY_PORT=$((7400 + 250 * 10#${DOMAIN_ID} + 10 + 2 * (10#${MAX_INITIAL_PEERS} - 1)))
((LAST_DISCOVERY_PORT <= 65535)) ||
  die "domain ${DOMAIN_ID} with max-initial-peers ${MAX_INITIAL_PEERS} reaches invalid UDP port ${LAST_DISCOVERY_PORT}"

command -v ip >/dev/null || die "iproute2 is required"
command -v mktemp >/dev/null || die "mktemp is required"

for peer in "${PEERS[@]}"; do
  is_ipv4 "${peer}" || die "invalid IPv4 peer '${peer}'"
done

declare -a DDS_INTERFACES=()
declare -A SEEN_INTERFACES=()
add_interface() {
  local interface="$1"
  [[ "${interface}" =~ ^[a-zA-Z0-9_.:-]+$ ]] ||
    die "unsafe or unsupported interface name '${interface}'"
  ip link show dev "${interface}" >/dev/null 2>&1 ||
    die "interface '${interface}' does not exist"
  if [[ -z "${SEEN_INTERFACES[${interface}]:-}" ]]; then
    DDS_INTERFACES+=("${interface}")
    SEEN_INTERFACES["${interface}"]=1
  fi
}

if [[ ${#REQUESTED_INTERFACES[@]} -gt 0 ]]; then
  for interface in "${REQUESTED_INTERFACES[@]}"; do
    add_interface "${interface}"
  done
else
  for peer in "${PEERS[@]}"; do
    route_line="$(ip -o route get "${peer}" 2>/dev/null | head -n 1)"
    [[ -n "${route_line}" ]] || die "no route to peer ${peer}"
    interface="$(awk '{for (i=1; i<=NF; ++i) if ($i == "dev") {print $(i+1); exit}}' <<<"${route_line}")"
    [[ -n "${interface}" ]] || die "route to ${peer} has no interface: ${route_line}"
    add_interface "${interface}"
  done
fi

STATIC_PEERS=""
for peer in "${PEERS[@]}"; do
  if [[ -n "${STATIC_PEERS}" ]]; then
    STATIC_PEERS+=";"
  fi
  STATIC_PEERS+="${peer}"
done

HOPE_FASTDDS_TMP="$(mktemp -d "${TMPDIR:-/tmp}/hope-fastdds-unicast.XXXXXX")"
DDS_PROFILE="${HOPE_FASTDDS_TMP}/fastdds_unicast.xml"
cleanup() {
  rm -f -- "${DDS_PROFILE}"
  rmdir -- "${HOPE_FASTDDS_TMP}" 2>/dev/null || true
}
trap cleanup EXIT

{
  cat <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>hope_static_peer_udp</transport_id>
        <type>UDPv4</type>
        <maxInitialPeersRange>${MAX_INITIAL_PEERS}</maxInitialPeersRange>
        <interfaceWhiteList>
EOF
  for interface in "${DDS_INTERFACES[@]}"; do
    printf '          <interface>%s</interface>\n' "${interface}"
  done
  cat <<'EOF'
        </interfaceWhiteList>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="hope_static_peer" is_default_profile="true">
      <rtps>
        <builtin>
          <initialPeersList>
EOF
  for peer in "${PEERS[@]}"; do
    cat <<EOF
            <locator>
              <udpv4>
                <address>${peer}</address>
              </udpv4>
            </locator>
EOF
  done
  cat <<'EOF'
          </initialPeersList>
        </builtin>
        <useBuiltinTransports>false</useBuiltinTransports>
        <userTransports>
          <transport_id>hope_static_peer_udp</transport_id>
        </userTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
EOF
} >"${DDS_PROFILE}"

export ROS_DOMAIN_ID="${DOMAIN_ID}"
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_STATIC_PEERS="${STATIC_PEERS}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="${DDS_PROFILE}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${DDS_PROFILE}"

echo "[hope-fastdds] domain=${ROS_DOMAIN_ID} peers=${ROS_STATIC_PEERS} max_initial_peers=${MAX_INITIAL_PEERS}"
echo "[hope-fastdds] interfaces=${DDS_INTERFACES[*]} profile=${DDS_PROFILE}"
printf '[hope-fastdds] exec:'
printf ' %q' "${COMMAND[@]}"
printf '\n'

"${COMMAND[@]}"
