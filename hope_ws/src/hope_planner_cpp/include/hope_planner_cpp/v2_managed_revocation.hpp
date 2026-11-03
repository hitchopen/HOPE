#pragma once
#include <algorithm>
#include "hope_planner_cpp/schema2_packer.hpp"
#include "hope_planner_cpp/v2_strike_adapter.hpp"

namespace hope_planner_cpp {
// Existing invalid schema-4/5 packet with explicit identity. Only an opt-in
// Runner treats it as a pending-flight veto; active swing behavior is unchanged.
inline std::optional<std::vector<double>> managed_revocation(
    const V2StrikeCandidate& status, const WireIdentity& transport, std::int64_t now_ns) {
  constexpr std::uint64_t limit = 4503599627370496ULL;
  if (status.command_revision_id >= limit) return {};
  const auto revision = std::max(status.revision, status.command_revision_id + 1);
  if ((status.status != IncomingStatus::Revoked && status.status != IncomingStatus::Expired) ||
      !status.command_sequence || !status.command_flight_id || !status.plane.selection.epoch ||
      transport.command_sequence <= status.command_sequence || transport.command_sequence > limit ||
      status.command_flight_id > limit || status.plane.selection.epoch > limit ||
      !revision || revision > limit || now_ns <= 0) return {};
  const auto packet = Schema2Packer::pack(nullptr, 0, 0, 0, now_ns, transport, 0, 0);
  std::vector<double> wire(packet.values.begin(), packet.values.end());
  wire[0] = status.plane.selection.mode == PlaneMode::Fixed ? 4 : 5;
  wire[15] = status.command_flight_id; wire[16] = revision;
  wire.push_back(status.plane.selection.epoch);
  if (status.plane.selection.mode != PlaneMode::Fixed) {
    wire.push_back(status.plane.selection.reference_x);
    wire.push_back(static_cast<int>(status.plane.selection.mode));
  }
  return wire;
}
}  // namespace hope_planner_cpp
