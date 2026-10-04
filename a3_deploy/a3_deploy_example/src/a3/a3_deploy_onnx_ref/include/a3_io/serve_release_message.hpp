#pragma once
#include "aimdk/protocol/hal/hand/serve_hand_channel.pb.h"
#include <cstdint>

namespace a3_io {
// Same OP3 preset and wire format as scripts/a3p_gripper_bridge.py. The
// native publisher bypasses Python/IPC scheduling, not HAL or its safeguards.
inline void FillServeReleaseMessage(aimdk::protocol::HandCommandChannel& msg,
                                    std::uint32_t sequence, std::int64_t wall_ns) {
  auto* h = msg.mutable_header();
  h->set_seq(sequence);
  h->set_control_source(static_cast<aimdk::protocol::ControlSource>(1));
  auto* t = h->mutable_timestamp();
  t->set_seconds(wall_ns / 1'000'000'000);
  t->set_nanos(wall_ns % 1'000'000'000);
  t->set_ms_since_epoch(wall_ns / 1'000'000);
  auto* claw = msg.mutable_data()->mutable_left()->mutable_agi_claw_cmd();
  claw->set_cmd(0);
  claw->set_pos(2000);
  claw->set_force(20);
  claw->set_clamp_method(1);
  claw->set_vel(60);
  msg.mutable_data()->mutable_right()->mutable_agi_claw_cmd();
}
}  // namespace a3_io
