#include <cmath>
#include <chrono>
#include <cstdint>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "a3_pingpong/pp_planner_input.hpp"

namespace a3_pingpong {
namespace {

std::vector<double> Schema2Racket(std::uint64_t command_seq,
                                  std::uint64_t revision,
                                  double x_offset = 0.0,
                                  double vx_offset = 0.0,
                                  double strike_offset_s = 0.0,
                                  double producer_offset_s = 0.0) {
  const double producer = PpNowWallSec() + producer_offset_s;
  const double sec = std::floor(producer);
  const double nsec = std::floor((producer - sec) * 1.0e9);
  const double producer_wire = sec + nsec * 1.0e-9;
  const double strike = producer_wire + 1.0 + strike_offset_s;
  const double tts = strike - producer_wire;
  return {
      2.0, 1.0, 1.0,
      0.58 + x_offset, -0.44, 1.0,
      2.0 + vx_offset, 0.4, 0.8,
      tts, strike, 0.0,
      sec, nsec, static_cast<double>(command_seq),
      77.0, static_cast<double>(revision), 8.0, 0.12};
}

TEST(PpRacketTargetInput, Schema2TracksRevisionStabilityAsTelemetry) {
  PpRacketTargetInput input;
  input.SetFromFlat(Schema2Racket(1, 1));
  EXPECT_EQ(input.Latest().cmd.stable_revision_count, 1);
  input.SetFromFlat(Schema2Racket(2, 2, 0.005, 0.02));
  EXPECT_EQ(input.Latest().cmd.stable_revision_count, 2);
  input.SetFromFlat(Schema2Racket(3, 3, 0.010, 0.04));
  const auto latest = input.Latest();
  ASSERT_TRUE(latest.has_valid);
  EXPECT_EQ(latest.cmd.schema, 2);
  EXPECT_EQ(latest.cmd.flight_id, 77U);
  EXPECT_EQ(latest.cmd.revision_id, 3U);
  EXPECT_EQ(latest.cmd.stable_revision_count, 3);
  EXPECT_GE(latest.valid_age_s, 0.0);
  EXPECT_NEAR(latest.control_time_to_strike_s, 1.0, 0.05);
}

TEST(PpRacketTargetInput, LatestSnapshotKeepsOneRevisionTupleCoherent) {
  PpRacketTargetInput input;
  input.SetFromFlat(Schema2Racket(1, 1, 0.001, 0.010, 0.020));
  input.SetFromFlat(Schema2Racket(2, 2, 0.012, 0.120, 0.080));

  const auto latest = input.Latest();
  ASSERT_TRUE(latest.has_valid);
  EXPECT_EQ(latest.cmd.command_seq, 2U);
  EXPECT_EQ(latest.cmd.flight_id, 77U);
  EXPECT_EQ(latest.cmd.revision_id, 2U);
  EXPECT_NEAR(latest.cmd.pos_w.x(), 0.592, 1.0e-12);
  EXPECT_NEAR(latest.cmd.vel_w.x(), 2.120, 1.0e-12);
  // Unix-epoch doubles have roughly 0.2 us resolution in 2026; this tests one
  // coherent packet tuple rather than claiming nanosecond precision.
  EXPECT_NEAR(latest.cmd.time_to_strike, 1.080, 1.0e-6);
  EXPECT_NEAR(
      latest.cmd.strike_time - latest.cmd.producer_wall_s,
      latest.cmd.time_to_strike, 1.0e-6);
}

TEST(PpRacketTargetInput, CrossHostClockLeadIsAuditOnly) {
  PpRacketTargetInput input;
  input.SetFromFlat(Schema2Racket(1, 1, 0.0, 0.0, 0.0, 0.5));
  const auto latest = input.Latest();
  ASSERT_TRUE(latest.has_valid);
  EXPECT_GE(latest.valid_age_s, 0.0);
  EXPECT_LT(latest.valid_age_s, 0.05);
  EXPECT_LT(latest.producer_age_s, -0.4);
  EXPECT_LT(latest.cmd.producer_clock_delta_s, -0.4);
}

TEST(PpRacketTargetInput, AbsoluteDeadlineIncludesTransportBeforeMduReceipt) {
  PpRacketTargetInput input;
  // The HDU produced a one-second TTS 200 ms ago. The MDU countdown must start
  // near 0.8 s, not restart from one second at local receipt.
  input.SetFromFlat(
      Schema2Racket(1, 1, 0.0, 0.0, 0.0, -0.2));
  const auto latest = input.Latest();
  ASSERT_TRUE(latest.has_valid);
  EXPECT_NEAR(latest.cmd.time_to_strike, 1.0, 1.0e-9);
  EXPECT_NEAR(latest.control_time_to_strike_s, 0.8, 0.05);
  EXPECT_GE(latest.valid_age_s, 0.0);
}

TEST(PpRacketTargetInput, Schema2LargeRevisionJumpRestartsStability) {
  PpRacketTargetInput input;
  input.SetFromFlat(Schema2Racket(1, 1));
  input.SetFromFlat(Schema2Racket(2, 2, 0.005));
  input.SetFromFlat(Schema2Racket(3, 3, 0.10));
  EXPECT_EQ(input.Latest().cmd.stable_revision_count, 1);
}

TEST(PpRacketTargetInput, Schema2DuplicateRevisionDoesNotPoisonRetainedCommand) {
  PpRacketTargetInput input;
  input.SetFromFlat(Schema2Racket(1, 1));
  input.SetFromFlat(Schema2Racket(2, 1));
  const auto latest = input.Latest();
  ASSERT_TRUE(latest.has_valid);
  EXPECT_FALSE(latest.invalid_after);
  EXPECT_EQ(latest.cmd.stable_revision_count, 1);
  EXPECT_EQ(latest.cmd.command_seq, 1U);
}

TEST(PpRacketTargetInput, PlannerRestartOpensNewSequenceEpoch) {
  PpRacketTargetInput input;
  input.SetFromFlat(Schema2Racket(100, 100));
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  input.SetFromFlat(Schema2Racket(1, 1, 0.02));
  const auto latest = input.Latest();
  ASSERT_TRUE(latest.has_valid);
  EXPECT_EQ(latest.cmd.command_seq, 1U);
  EXPECT_EQ(latest.cmd.revision_id, 1U);
  EXPECT_NEAR(latest.cmd.pos_w[0], 0.60, 1.0e-12);
}

TEST(PpRacketTargetInput, LegacySchemaRemainsReadableButUnrevisioned) {
  PpRacketTargetInput input;
  input.SetFromFlat(
      {1.0, 1.0, 1.0, 0.58, -0.44, 1.0,
       2.0, 0.4, 0.8, 1.0, PpNowWallSec() + 1.0, 0.0});
  const auto latest = input.Latest();
  ASSERT_TRUE(latest.has_valid);
  EXPECT_EQ(latest.cmd.schema, 1);
  EXPECT_EQ(latest.cmd.stable_revision_count, 0);
}

}  // namespace
}  // namespace a3_pingpong
