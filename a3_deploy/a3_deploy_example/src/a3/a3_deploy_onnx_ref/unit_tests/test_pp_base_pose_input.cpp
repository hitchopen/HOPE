#include <cmath>
#include <chrono>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "a3_pingpong/pp_planner_input.hpp"

namespace a3_pingpong {
namespace {

std::vector<double> Schema2Base(std::uint64_t sequence = 1,
                                double source_offset_s = 0.0) {
  const double now = PpNowWallSec() + source_offset_s;
  const double sec = std::floor(now);
  const double nsec = std::floor((now - sec) * 1.0e9);
  return {
      2.0, 1.0, static_cast<double>(sequence), sec, nsec,
      1.0, 2.0, 0.95,
      1.0, 0.0, 0.0, 0.0,
      1.0, static_cast<double>(kV17RequiredBaseFlags),
      11.0, 22.0};
}

TEST(PpBasePoseInput, AuthoritativeSchema2PreservesPoseAndSourceTime) {
  PpBasePoseInput input;
  input.SetFromFlat(Schema2Base());
  PpBaseSample sample;
  ASSERT_TRUE(input.Latest(sample, 0.2, true));
  EXPECT_EQ(sample.schema, 2);
  EXPECT_TRUE(sample.authoritative);
  EXPECT_EQ(sample.seq, 1U);
  EXPECT_EQ(sample.calibration_id, 11U);
  EXPECT_EQ(sample.world_frame_id, 22U);
  EXPECT_EQ(sample.flags, kV17RequiredBaseFlags);
  EXPECT_NEAR(sample.pos[0], 1.0, 1.0e-12);
  EXPECT_NEAR(sample.pos[1], 2.0, 1.0e-12);
  EXPECT_NEAR(sample.pos[2], 0.95, 1.0e-12);
  EXPECT_NEAR(sample.quat.norm(), 1.0, 1.0e-12);
  EXPECT_GE(sample.age_s, 0.0);
}

TEST(PpBasePoseInput, CrossHostClockLeadIsAuditOnly) {
  PpBasePoseInput input;
  input.SetFromFlat(Schema2Base(1, 0.5));
  PpBaseSample sample;
  ASSERT_TRUE(input.Latest(sample, 0.2, true));
  EXPECT_GE(sample.age_s, 0.0);
  EXPECT_LT(sample.age_s, 0.05);
  EXPECT_LT(sample.source_age_s, -0.4);
  EXPECT_LT(sample.source_clock_delta_s, -0.4);
}

TEST(PpBasePoseInput, LegacySchemaCannotSatisfyV17Authority) {
  PpBasePoseInput input;
  input.SetFromFlat(
      {1.0, 1.0, 1.0, 2.0, 0.95, 1.0, 0.0, 0.0, 0.0});
  PpBaseSample sample;
  EXPECT_TRUE(input.Latest(sample, 0.2));
  EXPECT_FALSE(input.Latest(sample, 0.2, true));
}

TEST(PpBasePoseInput, MissingCalibrationFlagCannotSatisfyV17Authority) {
  PpBasePoseInput input;
  auto packet = Schema2Base();
  packet[13] = static_cast<double>(
      kBaseFlagTrackingValid | kBaseFlagQuaternionValid |
      kBaseFlagSourceStampHduRos);
  input.SetFromFlat(packet);
  PpBaseSample sample;
  EXPECT_TRUE(input.Latest(sample, 0.2));
  EXPECT_FALSE(input.Latest(sample, 0.2, true));
}

TEST(PpBasePoseInput, InvalidQuaternionFailsClosed) {
  PpBasePoseInput input;
  auto packet = Schema2Base();
  packet[8] = packet[9] = packet[10] = packet[11] = 0.0;
  input.SetFromFlat(packet);
  PpBaseSample sample;
  EXPECT_FALSE(input.Latest(sample, 0.2, true));
}

TEST(PpBasePoseInput, ReorderedSequenceDoesNotPoisonRetainedPose) {
  PpBasePoseInput input;
  input.SetFromFlat(Schema2Base(2));
  PpBaseSample sample;
  ASSERT_TRUE(input.Latest(sample, 0.2, true));
  input.SetFromFlat(Schema2Base(1));
  ASSERT_TRUE(input.Latest(sample, 0.2, true));
  EXPECT_EQ(sample.seq, 2U);
}

TEST(PpBasePoseInput, SourceWallRegressionDoesNotRefreshOrReplacePose) {
  PpBasePoseInput input;
  input.SetFromFlat(Schema2Base(1));
  auto regressed = Schema2Base(2, -0.010);
  regressed[5] = 1.25;
  input.SetFromFlat(regressed);
  PpBaseSample sample;
  ASSERT_TRUE(input.Latest(sample, 0.2, true));
  EXPECT_EQ(sample.seq, 1U);
  EXPECT_NEAR(sample.pos[0], 1.0, 1.0e-12);
}

TEST(PpBasePoseInput, RelayRestartOpensNewSequenceEpoch) {
  PpBasePoseInput input;
  input.SetFromFlat(Schema2Base(100));
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  auto restarted = Schema2Base(1);
  restarted[5] = 1.5;
  input.SetFromFlat(restarted);
  PpBaseSample sample;
  ASSERT_TRUE(input.Latest(sample, 0.2, true));
  EXPECT_EQ(sample.seq, 1U);
  EXPECT_NEAR(sample.pos[0], 1.5, 1.0e-12);
}

TEST(PpBasePoseInput, StaleColdStartCannotSeedPose) {
  PpBasePoseInput input;
  input.SetFromFlat(Schema2Base(1, -0.353698));  // Recorded MDU delay.
  PpBaseSample sample;
  EXPECT_FALSE(input.LatestForPolicy(sample, 1., true));
  input.SetFromFlat(Schema2Base(2));
  ASSERT_TRUE(input.LatestForPolicy(sample, 1., true));
  EXPECT_EQ(sample.seq, 2U);
}

TEST(PpBasePoseInput, DelayedBurstCannotRefreshDeadlineOrPoisonRestartEpoch) {
  PpBasePoseInput input;
  input.SetFromFlat(Schema2Base(100));
  PpBaseSample original, retained;
  ASSERT_TRUE(input.LatestForPolicy(original, 1., true));
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  for (int i = 0; i < 250; ++i) {
    auto late = Schema2Base(101 + i, -0.353698);
    late[5] = 99.; late[8] = -1.;
    input.SetFromFlat(late);
  }
  input.SetFromFlat(Schema2Base(1, -0.353698));
  ASSERT_TRUE(input.LatestForPolicy(retained, 1., true));
  EXPECT_TRUE(retained.held_for_policy);
  EXPECT_EQ(retained.seq, original.seq);
  EXPECT_EQ(retained.pos, original.pos);
  EXPECT_EQ(retained.quat, original.quat);
  EXPECT_EQ(retained.source_wall_s, original.source_wall_s);
  EXPECT_EQ(retained.receipt_wall_s, original.receipt_wall_s);
  EXPECT_EQ(retained.receipt_steady_s, original.receipt_steady_s);
  // A fresh packet resumes immediately, even below the discarded sequence IDs.
  input.SetFromFlat(Schema2Base(101));
  ASSERT_TRUE(input.LatestForPolicy(retained, 1., true));
  EXPECT_EQ(retained.seq, 101U);
  EXPECT_FALSE(retained.held_for_policy);
}

TEST(PpBasePoseInput, HigherSequenceWithIdenticalSourceTimeIsNotProgress) {
  PpBasePoseInput input;
  auto wire = Schema2Base(1);
  input.SetFromFlat(wire);
  PpBaseSample original, retained;
  ASSERT_TRUE(input.Latest(original, 1., true));
  wire[2] = 2.; wire[5] = 99.;
  input.SetFromFlat(wire);
  ASSERT_TRUE(input.Latest(retained, 1., true));
  EXPECT_EQ(retained.seq, original.seq);
  EXPECT_EQ(retained.pos, original.pos);
  EXPECT_EQ(retained.receipt_steady_s, original.receipt_steady_s);
}

}  // namespace
}  // namespace a3_pingpong
