#include "hope_planner_cpp/schema3_packer.hpp"

#include <algorithm>

namespace hope_planner_cpp {

Schema3Packet Schema3Packer::pack(
    const RacketCommand* command,
    double swing_sign,
    double strike_deadline_wall_s,
    double policy_z_offset,
    std::int64_t producer_wall_ns,
    const WireIdentity& identity,
    std::size_t estimator_sample_count,
    double estimator_span_s,
    const ReachPermission& permission) noexcept {
  Schema3Packet packet;
  const bool permission_valid = legal_reach_permission(permission);
  const Schema2Packet schema2 = Schema2Packer::pack(
      permission_valid ? command : nullptr,
      swing_sign,
      strike_deadline_wall_s,
      policy_z_offset,
      producer_wall_ns,
      identity,
      estimator_sample_count,
      estimator_span_s);
  std::copy(schema2.values.begin(), schema2.values.end(), packet.values.begin());
  packet.values[0] = 3.0;
  packet.values[19] = schema2.valid ? permission.reach_level : 0.0;
  packet.values[20] = schema2.valid ? permission.swing_foot_sign : 0.0;
  packet.identity = identity;
  packet.valid = schema2.valid;
  return packet;
}

}  // namespace hope_planner_cpp
