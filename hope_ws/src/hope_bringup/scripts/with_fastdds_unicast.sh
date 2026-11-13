#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  with_fastdds_unicast.sh [options] -- COMMAND [ARG ...]

Options:
  --peer IPV4              Remote Fast DDS peer. Repeat for every remote box.
  --interface NAME         Local DDS interface. Repeat as needed. By default,
                           interfaces are derived from `ip route get PEER`.
  --route-source-addresses Advertise only the source IPv4 selected for each
                           peer route, excluding unrelated interface aliases.
                           Cannot be combined with --interface.
  --filter-direct-routes   With route-source addresses, restrict direct-link
                           sources to their subnet. Routed sources stay open.
                           Requires Fast DDS >= 2.14.
  --domain-id N            ROS domain (default: $ROS_DOMAIN_ID or 232).
  --max-initial-peers N    Participant ports probed per peer (default: 32).
  --flight-packet-qos-topic TOPIC
                          Bounded asynchronous flight-packet DDS profile.
  -h, --help               Show this help.

Examples for the OptiTrack laptop-bridge topology:
  # Laptop: Motive bridge -> HDU over the laptop Wi-Fi route.
  with_fastdds_unicast.sh --peer 172.23.20.135 -- \
    ros2 launch hope_bringup optitrack_hope_bridge.launch.py \
      hostname:=192.168.100.111

  # HDU: Laptop input plus MDU output; routes select wifi_hdu and eth_hdu.
  with_fastdds_unicast.sh --peer 172.23.20.46 --peer 10.42.10.12 -- \
    ros2 run hope_planner_cpp hope_planner_cpp_node ...

When the venue Wi-Fi changes, replace only the Wi-Fi peer IPs. The wrapper
derives each local interface and address from the active route; no XML edit is
needed. Keep the MDU peer at 10.42.10.12 unless the robot-internal network itself
changes.
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
DOMAIN_ID="${ROS_DOMAIN_ID:-232}"
MAX_INITIAL_PEERS="32"
FLIGHT_PACKET_QOS_TOPIC=""
ROUTE_SOURCE_ADDRESSES=false
FILTER_DIRECT_ROUTES=false

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
    --route-source-addresses)
      ROUTE_SOURCE_ADDRESSES=true
      shift
      ;;
    --filter-direct-routes)
      FILTER_DIRECT_ROUTES=true
      shift
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
    --flight-packet-qos-topic)
      [[ $# -ge 2 ]] || die "--flight-packet-qos-topic requires a ROS topic"
      FLIGHT_PACKET_QOS_TOPIC="$2"
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
# within uint16. This matters at the production domain 232, whose base port is
# already 65400.
LAST_DISCOVERY_PORT=$((7400 + 250 * 10#${DOMAIN_ID} + 10 + 2 * (10#${MAX_INITIAL_PEERS} - 1)))
((LAST_DISCOVERY_PORT <= 65535)) ||
  die "domain ${DOMAIN_ID} with max-initial-peers ${MAX_INITIAL_PEERS} reaches invalid UDP port ${LAST_DISCOVERY_PORT}"
[[ -z "$FLIGHT_PACKET_QOS_TOPIC" || "$FLIGHT_PACKET_QOS_TOPIC" =~ ^/[a-zA-Z0-9_/]+$ ]] ||
  die "invalid flight packet topic"
if "$ROUTE_SOURCE_ADDRESSES" && [[ ${#REQUESTED_INTERFACES[@]} -gt 0 ]]; then
  die "--route-source-addresses cannot be combined with --interface"
fi
if "$FILTER_DIRECT_ROUTES" && ! "$ROUTE_SOURCE_ADDRESSES"; then
  die "--filter-direct-routes requires --route-source-addresses"
fi

command -v ip >/dev/null || die "iproute2 is required"
command -v mktemp >/dev/null || die "mktemp is required"

for peer in "${PEERS[@]}"; do
  is_ipv4 "${peer}" || die "invalid IPv4 peer '${peer}'"
done

declare -a DDS_INTERFACES=()
declare -A SEEN_INTERFACES=()
declare -A SOURCE_FILTERS=()
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
    if "$ROUTE_SOURCE_ADDRESSES"; then
      # A device name includes every IPv4 alias. Fast DDS then sends through
      # each local address to each advertised reader address. Bind only the
      # OS-selected route source; retain separate paths for distinct peers.
      source_address="$(awk '{for (i=1; i<=NF; ++i) if ($i == "src") {print $(i+1); exit}}' <<<"${route_line}")"
      is_ipv4 "${source_address}" || die "route to ${peer} has no valid IPv4 source: ${route_line}"
      # An address serving any routed peer must remain unrestricted, even if
      # another peer using the same address is directly connected. Otherwise
      # cross-subnet laptop discovery and data would be silently filtered.
      if [[ " ${route_line} " == *" via "* ]]; then
        SOURCE_FILTERS["${source_address}"]=OFF
      elif [[ -z "${SOURCE_FILTERS[${source_address}]:-}" ]]; then
        SOURCE_FILTERS["${source_address}"]=ON
      fi
      if [[ -z "${SEEN_INTERFACES[${source_address}]:-}" ]]; then
        DDS_INTERFACES+=("${source_address}")
        SEEN_INTERFACES["${source_address}"]=1
      fi
    else
      add_interface "${interface}"
    fi
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
EOF
  if "$FILTER_DIRECT_ROUTES"; then
    printf '        <interfaces><allowlist>\n'
    for interface in "${DDS_INTERFACES[@]}"; do
      printf '          <interface name="%s" netmask_filter="%s"/>\n' "${interface}" "${SOURCE_FILTERS[${interface}]}"
    done
    printf '        </allowlist></interfaces>\n'
  else
    printf '        <interfaceWhiteList>\n'
    for interface in "${DDS_INTERFACES[@]}"; do
      printf '          <interface>%s</interface>\n' "${interface}"
    done
    printf '        </interfaceWhiteList>\n'
  fi
  cat <<'EOF'
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
EOF
  if [[ -n "$FLIGHT_PACKET_QOS_TOPIC" ]]; then
    cat <<EOF
    <!-- Preserve rmw_fastrtps defaults for all unrelated endpoints. -->
    <publisher profile_name="hope_default_writer" is_default_profile="true">
      <qos><publishMode><kind>SYNCHRONOUS</kind></publishMode><data_sharing><kind>OFF</kind></data_sharing></qos>
      <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
    </publisher>
    <subscriber profile_name="hope_default_reader" is_default_profile="true">
      <qos><data_sharing><kind>OFF</kind></data_sharing></qos>
      <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
    </subscriber>
    <publisher profile_name="$FLIGHT_PACKET_QOS_TOPIC">
      <qos>
        <publishMode><kind>ASYNCHRONOUS</kind></publishMode>
        <data_sharing><kind>OFF</kind></data_sharing>
        <reliability><kind>RELIABLE</kind><max_blocking_time><sec>0</sec><nanosec>1000000</nanosec></max_blocking_time></reliability>
      </qos>
      <times>
        <initialHeartbeatDelay><sec>0</sec><nanosec>1000000</nanosec></initialHeartbeatDelay>
        <heartbeatPeriod><sec>0</sec><nanosec>10000000</nanosec></heartbeatPeriod>
        <nackResponseDelay><sec>0</sec><nanosec>1000000</nanosec></nackResponseDelay>
        <nackSupressionDuration><sec>0</sec><nanosec>0</nanosec></nackSupressionDuration>
      </times>
      <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
    </publisher>
    <subscriber profile_name="$FLIGHT_PACKET_QOS_TOPIC">
      <qos><data_sharing><kind>OFF</kind></data_sharing></qos>
      <times><initialAcknackDelay><sec>0</sec><nanosec>1000000</nanosec></initialAcknackDelay><heartbeatResponseDelay><sec>0</sec><nanosec>1000000</nanosec></heartbeatResponseDelay></times>
      <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
    </subscriber>
EOF
  fi
  cat <<'EOF'
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
if [[ -n "$FLIGHT_PACKET_QOS_TOPIC" ]]; then
  export RMW_FASTRTPS_USE_QOS_FROM_XML=1
fi

echo "[hope-fastdds] domain=${ROS_DOMAIN_ID} peers=${ROS_STATIC_PEERS} max_initial_peers=${MAX_INITIAL_PEERS}"
echo "[hope-fastdds] interfaces=${DDS_INTERFACES[*]} profile=${DDS_PROFILE}"
printf '[hope-fastdds] exec:'
printf ' %q' "${COMMAND[@]}"
printf '\n'

"${COMMAND[@]}"
