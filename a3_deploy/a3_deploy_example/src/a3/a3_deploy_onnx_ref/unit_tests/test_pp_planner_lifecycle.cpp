#include "a3_pingpong/pp_planner_lifecycle.hpp"

#include <limits>

#include <gtest/gtest.h>

namespace a3_pingpong {

TEST(PpPlannerLifecycle, PrefixCommitIsDeterministicAndBeforeDynamicOnset) {
  EXPECT_DOUBLE_EQ(planner_prefix_commit_tts(0.82, 0.10), 0.72);
  EXPECT_DOUBLE_EQ(planner_prefix_commit_tts(0.96, 0.10), 0.86);
  EXPECT_DOUBLE_EQ(planner_prefix_commit_tts(0.82, 0.20), 0.62);
  EXPECT_DOUBLE_EQ(planner_prefix_commit_tts(0.96, 0.20), 0.76);
  EXPECT_DOUBLE_EQ(planner_prefix_hard_late_tts(0.82), 0.451);
  EXPECT_DOUBLE_EQ(planner_prefix_hard_late_tts(0.96), 0.528);
  EXPECT_DOUBLE_EQ(planner_prefix_commit_tts(0.82, 10.0), 0.451);
  EXPECT_DOUBLE_EQ(planner_prefix_commit_tts(0.82, 0.00), 0.82);
  EXPECT_DOUBLE_EQ(planner_prefix_commit_tts(0.82, 0.05), 0.77);
}

TEST(PpPlannerLifecycle, Model21800SamplesAtFixedDynamicBoundary) {
  EXPECT_DOUBLE_EQ(planner_target_sample_tts(true, 0.82, 0.0), 0.451);
  EXPECT_DOUBLE_EQ(planner_target_sample_tts(true, 0.82, 0.10), 0.451);
  EXPECT_DOUBLE_EQ(planner_target_sample_tts(true, 0.82, 10.0), 0.451);
  EXPECT_DOUBLE_EQ(planner_target_sample_tts(true, 0.96, 0.10), 0.528);
  EXPECT_DOUBLE_EQ(planner_target_sample_tts(false, 0.82, 0.10), 0.72);
}

TEST(PpPlannerLifecycle, PositiveLateCommandStartsAtDeepestSupportedPrefix) {
  const auto forehand = planner_phase_continuous_start(true, 0.040, 0.451);
  EXPECT_DOUBLE_EQ(forehand.clock_tts_s, 0.451);
  EXPECT_DOUBLE_EQ(forehand.expected_strike_lateness_s, 0.411);
  EXPECT_TRUE(forehand.late_phase_clamped);

  const auto already_in_prefix =
      planner_phase_continuous_start(true, 0.500, 0.451);
  EXPECT_DOUBLE_EQ(already_in_prefix.clock_tts_s, 0.500);
  EXPECT_DOUBLE_EQ(already_in_prefix.expected_strike_lateness_s, 0.0);
  EXPECT_FALSE(already_in_prefix.late_phase_clamped);

  const auto legacy = planner_phase_continuous_start(false, 0.040, 0.451);
  EXPECT_DOUBLE_EQ(legacy.clock_tts_s, 0.040);
  EXPECT_DOUBLE_EQ(legacy.expected_strike_lateness_s, 0.0);
  EXPECT_FALSE(legacy.late_phase_clamped);
}

TEST(PpPlannerLifecycle, RevisionStabilityIsAuditOnly) {
  EXPECT_FALSE(planner_revision_release_blocked(true, 2, 0));
  EXPECT_FALSE(planner_revision_release_blocked(true, 2, 1));
  EXPECT_FALSE(planner_revision_release_blocked(true, 2, 2));
  EXPECT_FALSE(planner_revision_release_blocked(true, 2, 3));
  EXPECT_TRUE(planner_revision_release_blocked(true, 1, 99));
  EXPECT_FALSE(planner_revision_release_blocked(false, 1, 0));
}

TEST(PpPlannerLifecycle, SamePhysicalShotCannotBeConsumedTwice) {
  EXPECT_TRUE(same_planner_shot(101.12, 101.00, 0.25));
  EXPECT_FALSE(same_planner_shot(101.30, 101.00, 0.25));
  EXPECT_FALSE(same_planner_shot(0.0, 101.00, 0.25));
  EXPECT_FALSE(same_planner_shot(
      std::numeric_limits<double>::quiet_NaN(), 101.00, 0.25));
}

TEST(PpPlannerLifecycle, PendingStationSurvivesTransientGapUntilShotExpires) {
  EXPECT_EQ(pending_station_gap_decision(false, 1.0, 0.25),
            PendingStationGapDecision::kNoPending);
  EXPECT_EQ(pending_station_gap_decision(true, 0.70, 0.25),
            PendingStationGapDecision::kHoldBlocked);
  EXPECT_EQ(pending_station_gap_decision(true, -0.20, 0.25),
            PendingStationGapDecision::kHoldBlocked);
  EXPECT_EQ(pending_station_gap_decision(true, -0.30, 0.25),
            PendingStationGapDecision::kExpire);
  EXPECT_EQ(pending_station_gap_decision(
                true, std::numeric_limits<double>::quiet_NaN(), 0.25),
            PendingStationGapDecision::kHoldBlocked);
}

TEST(PpPlannerLifecycle, NewShotIdentityResetsEvenAtSameStation) {
  EXPECT_FALSE(planner_shot_changed(101.12, 101.00, 0.25));
  EXPECT_TRUE(planner_shot_changed(102.00, 101.00, 0.25));
  EXPECT_FALSE(planner_shot_changed(0.0, 101.00, 0.25));
}

TEST(PpPlannerLifecycle, PolicyNativeHeadingIsTelemetryOnly) {
  EXPECT_FALSE(planner_heading_blocks_release(true, true));
  EXPECT_TRUE(planner_heading_blocks_release(false, true));
  EXPECT_FALSE(planner_heading_blocks_release(false, false));
}

TEST(PpPlannerLifecycle, PolicyNativeReadyDoesNotBlockBallClock) {
  EXPECT_FALSE(planner_station_blocks_release(true, false, false));
  EXPECT_TRUE(planner_station_blocks_release(false, false, false));
  EXPECT_FALSE(planner_station_blocks_release(false, false, true));
  EXPECT_TRUE(planner_station_blocks_release(true, true, true));
}

TEST(PpPlannerLifecycle, PolicyNativeTargetSupportIsTelemetryOnly) {
  EXPECT_FALSE(planner_target_blocks_release(true, false));
  EXPECT_TRUE(planner_target_blocks_release(false, false));
  EXPECT_FALSE(planner_target_blocks_release(false, true));
}

TEST(PpPlannerLifecycle, PolicyNativeCommandHealthIsTelemetryOnly) {
  EXPECT_FALSE(planner_command_health_blocks_release(true, true));
  EXPECT_TRUE(planner_command_health_blocks_release(false, true));
  EXPECT_FALSE(planner_command_health_blocks_release(false, false));
}

TEST(PpPlannerLifecycle, PolicyNativeInterSwingRestIsTelemetryOnly) {
  EXPECT_FALSE(planner_rest_blocks_release(true, true));
  EXPECT_TRUE(planner_rest_blocks_release(false, true));
  EXPECT_FALSE(planner_rest_blocks_release(false, false));
}

TEST(PpPlannerLifecycle, PolicyNativeExecutesPositiveLateTts) {
  EXPECT_FALSE(planner_timing_blocks_release(true, 0.69, 0.72));
  EXPECT_TRUE(planner_timing_blocks_release(false, 0.69, 0.72));
  EXPECT_TRUE(planner_timing_blocks_release(true, 0.0, 0.72));
  EXPECT_TRUE(planner_timing_blocks_release(true, -0.01, 0.72));
}

TEST(PpPlannerLifecycle, SamePendingStationKeepsSettlingAcrossTargetGap) {
  EXPECT_TRUE(pending_station_can_progress_during_target_gap(true, true, 0.02));
  EXPECT_TRUE(pending_station_can_progress_during_target_gap(true, true, 0.05));
  EXPECT_FALSE(pending_station_can_progress_during_target_gap(false, true, 0.0));
  EXPECT_FALSE(pending_station_can_progress_during_target_gap(true, false, 0.0));
  EXPECT_FALSE(pending_station_can_progress_during_target_gap(true, true, 0.051));
  EXPECT_FALSE(pending_station_can_progress_during_target_gap(
      true, true, std::numeric_limits<double>::quiet_NaN()));
}

TEST(PpPlannerLifecycle, ReadyPendingShotMayUseRecentSupportedTargetLatch) {
  EXPECT_TRUE(pending_target_latch_can_release(true, true, true, 0.12, 0.25));
  EXPECT_TRUE(pending_target_latch_can_release(true, true, true, 0.25, 0.25));
  EXPECT_FALSE(pending_target_latch_can_release(false, true, true, 0.12, 0.25));
  EXPECT_FALSE(pending_target_latch_can_release(true, false, true, 0.12, 0.25));
  EXPECT_FALSE(pending_target_latch_can_release(true, true, false, 0.12, 0.25));
  EXPECT_FALSE(pending_target_latch_can_release(true, true, true, 0.251, 0.25));
  EXPECT_FALSE(pending_target_latch_can_release(
      true, true, true, std::numeric_limits<double>::quiet_NaN(), 0.25));
}

}  // namespace a3_pingpong
