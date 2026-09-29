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

TEST(PpPlannerLifecycle, HitterRepairCommitsAtActorEntrySupportUpperEdge) {
  EXPECT_DOUBLE_EQ(planner_actor_entry_commit_tts(0.40, 0.55), 0.55);
  EXPECT_DOUBLE_EQ(planner_actor_entry_commit_tts(0.45, 0.55), 0.55);
  EXPECT_DOUBLE_EQ(planner_actor_entry_commit_tts(0.82, 0.82), 0.82);
  EXPECT_DOUBLE_EQ(planner_actor_entry_commit_tts(0.55, 0.40), 0.0);
  EXPECT_DOUBLE_EQ(planner_actor_entry_commit_tts(0.0, 0.55), 0.0);
  EXPECT_DOUBLE_EQ(
      planner_actor_entry_commit_tts(
          std::numeric_limits<double>::quiet_NaN(), 0.55),
      0.0);
}

TEST(PpPlannerLifecycle, ContinuousV2UsesClosedPerSideTtsSupport) {
  EXPECT_EQ(planner_actor_entry_tts_decision(0.83, 0.40, 0.82),
            PlannerActorEntryTtsDecision::kWaitAboveSupport);
  EXPECT_EQ(planner_actor_entry_tts_decision(0.82, 0.40, 0.82),
            PlannerActorEntryTtsDecision::kCommitInSupport);
  EXPECT_EQ(planner_actor_entry_tts_decision(0.70, 0.40, 0.82),
            PlannerActorEntryTtsDecision::kCommitInSupport);
  EXPECT_EQ(planner_actor_entry_tts_decision(0.46, 0.40, 0.82),
            PlannerActorEntryTtsDecision::kCommitInSupport);
  EXPECT_EQ(planner_actor_entry_tts_decision(0.40, 0.40, 0.82),
            PlannerActorEntryTtsDecision::kCommitInSupport);
  EXPECT_EQ(planner_actor_entry_tts_decision(0.399, 0.40, 0.82),
            PlannerActorEntryTtsDecision::kRejectBelowSupport);

  EXPECT_EQ(planner_actor_entry_tts_decision(0.56, 0.40, 0.55),
            PlannerActorEntryTtsDecision::kWaitAboveSupport);
  EXPECT_EQ(planner_actor_entry_tts_decision(0.55, 0.40, 0.55),
            PlannerActorEntryTtsDecision::kCommitInSupport);
  EXPECT_EQ(planner_actor_entry_tts_decision(0.40, 0.40, 0.55),
            PlannerActorEntryTtsDecision::kCommitInSupport);
  EXPECT_EQ(planner_actor_entry_tts_decision(0.39, 0.40, 0.55),
            PlannerActorEntryTtsDecision::kRejectBelowSupport);
  EXPECT_EQ(
      planner_actor_entry_tts_decision(
          std::numeric_limits<double>::quiet_NaN(), 0.40, 0.82),
      PlannerActorEntryTtsDecision::kRejectBelowSupport);
}

TEST(PpPlannerLifecycle, Schema22ColdPositiveTtsOutsideSupportIsTelemetry) {
  EXPECT_EQ(planner_schema22_cold_tts_decision(0.83, 0.40, 0.82),
            PlannerSchema22ColdTtsDecision::kWaitAboveSupport);
  EXPECT_EQ(planner_schema22_cold_tts_decision(0.40, 0.40, 0.82),
            PlannerSchema22ColdTtsDecision::kCommitInSupport);
  // Latest real flight was only 3.432 ms below the old hard lower edge.
  EXPECT_EQ(planner_schema22_cold_tts_decision(0.396568, 0.40, 0.82),
            PlannerSchema22ColdTtsDecision::kCommitBelowSupportTelemetry);
  EXPECT_EQ(planner_schema22_cold_tts_decision(1.0e-6, 0.40, 0.82),
            PlannerSchema22ColdTtsDecision::kCommitBelowSupportTelemetry);
  EXPECT_EQ(planner_schema22_cold_tts_decision(0.0, 0.40, 0.82),
            PlannerSchema22ColdTtsDecision::kRejectExpiredOrInvalid);
  EXPECT_EQ(planner_schema22_cold_tts_decision(-0.01, 0.40, 0.82),
            PlannerSchema22ColdTtsDecision::kRejectExpiredOrInvalid);
  EXPECT_EQ(
      planner_schema22_cold_tts_decision(
          std::numeric_limits<double>::quiet_NaN(), 0.40, 0.82),
      PlannerSchema22ColdTtsDecision::kRejectExpiredOrInvalid);
}

TEST(PpPlannerLifecycle, Schema22ColdRetainsFreshFlightAcrossMailboxTimeout) {
  // Real BH command arrives fresh at TTS=1.164 s, then waits until the 0.55 s
  // actor-entry boundary. The same retained identity is now older than the
  // 0.50 s mailbox timeout, but that age is scheduling rather than a veto.
  EXPECT_EQ(planner_schema22_cold_tts_decision(1.164, 0.40, 0.55),
            PlannerSchema22ColdTtsDecision::kWaitAboveSupport);
  EXPECT_EQ(planner_schema22_cold_receipt_decision(
                false, 0.01, 0.50, false),
            PlannerSchema22ColdReceiptDecision::kAcceptFreshReceipt);
  EXPECT_EQ(planner_schema22_cold_receipt_decision(
                true, 0.624, 0.50, true),
            PlannerSchema22ColdReceiptDecision::kAcceptRetainedFlight);
  EXPECT_EQ(planner_schema22_cold_tts_decision(0.55, 0.40, 0.55),
            PlannerSchema22ColdTtsDecision::kCommitInSupport);

  // A different flight first observed at the same age is not grandfathered.
  EXPECT_EQ(planner_schema22_cold_receipt_decision(
                false, 0.624, 0.50, false),
            PlannerSchema22ColdReceiptDecision::kRejectStaleInitialReceipt);
  EXPECT_EQ(planner_schema22_cold_receipt_decision(
                false, 0.01, 0.50, true),
            PlannerSchema22ColdReceiptDecision::kRejectInvalidInitialReceipt);
  EXPECT_EQ(
      planner_schema22_cold_receipt_decision(
          true, std::numeric_limits<double>::quiet_NaN(), 0.50, false),
      PlannerSchema22ColdReceiptDecision::kRejectInvalidAge);
}

TEST(PpPlannerLifecycle, ContinuousV2WaitClockExposesElapsedWithoutNewChannel) {
  constexpr std::uint64_t kOrigin = 100;
  constexpr double kDt = 0.02;
  EXPECT_DOUBLE_EQ(planner_wait_tts(1.0, false, kOrigin + 12, kOrigin, kDt),
                   1.0);
  EXPECT_DOUBLE_EQ(planner_wait_tts(1.0, true, kOrigin, kOrigin, kDt), 1.0);
  EXPECT_DOUBLE_EQ(planner_wait_tts(1.0, true, kOrigin + 12, kOrigin, kDt),
                   0.76);
  EXPECT_DOUBLE_EQ(planner_wait_tts(1.0, true, kOrigin + 60, kOrigin, kDt),
                   -0.2);
  EXPECT_DOUBLE_EQ(planner_wait_tts(1.0, true, kOrigin - 1, kOrigin, kDt),
                   1.0);
}

TEST(PpPlannerLifecycle, ContinuousV3WaitClockStaysAtMemorylessPendingSentinel) {
  constexpr std::uint64_t kOrigin = 100;
  constexpr double kDt = 0.02;
  EXPECT_DOUBLE_EQ(planner_wait_tts(0.85, false, kOrigin, kOrigin, kDt),
                   0.85);
  EXPECT_DOUBLE_EQ(
      planner_wait_tts(0.85, false, kOrigin + 50, kOrigin, kDt), 0.85);
  EXPECT_DOUBLE_EQ(
      planner_wait_tts(0.85, false, kOrigin + 30000, kOrigin, kDt), 0.85);
}

TEST(PpPlannerLifecycle, ContinuousV2PreemptsAfterFollowthroughWithoutRestGate) {
  EXPECT_FALSE(planner_followthrough_complete(-0.119, 0.12));
  EXPECT_TRUE(planner_followthrough_complete(-0.12, 0.12));
  EXPECT_TRUE(planner_followthrough_complete(-0.11999995, 0.12));
  EXPECT_FALSE(planner_followthrough_complete(-0.119998, 0.12));
  EXPECT_TRUE(planner_followthrough_complete(-0.20, 0.12));
  EXPECT_FALSE(planner_followthrough_complete(
      std::numeric_limits<double>::quiet_NaN(), 0.12));

  // At the first trained TTS tick a fresh external event can commit; a legacy
  // swing-rest timer is advisory in policy-native mode and adds no 0.40 s gate.
  EXPECT_EQ(planner_actor_entry_tts_decision(0.40, 0.40, 0.82),
            PlannerActorEntryTtsDecision::kCommitInSupport);
  EXPECT_FALSE(planner_rest_blocks_release(true, true));
}

TEST(PpPlannerLifecycle, Schema22RapidPreemptRequiresNewFlightProtectedTailAndClosedSupport) {
  const auto flight = [](std::uint64_t epoch, std::uint64_t id) {
    return PlannerFlightIdentity{epoch, id};
  };
  EXPECT_STREQ(kPlannerEngageLogPrefix, "[pp engage]");
  EXPECT_EQ(
      planner_rapid_preempt_decision(
          true, true, 2, flight(1, 42), flight(1, 42), -0.50, 0.12,
          0.55, 0.40, 0.82,
          0.40, 0.55),
      PlannerRapidPreemptDecision::kSameFlight);
  EXPECT_EQ(
      planner_rapid_preempt_decision(
          true, true, 2, flight(1, 43), flight(1, 42), -0.119, 0.12,
          0.65, 0.60, 0.75,
          0.40, 0.55),
      PlannerRapidPreemptDecision::kWaitProtectedFollowthrough);
  EXPECT_EQ(
      planner_rapid_preempt_decision(
          true, true, 2, flight(1, 43), flight(1, 42), -0.399, 0.12,
          0.65, 0.60, 0.75,
          0.40, 0.55),
      PlannerRapidPreemptDecision::kWaitBeforeCommitDelaySupport);
  EXPECT_EQ(
      planner_rapid_preempt_decision(
          true, true, 2, flight(1, 43), flight(1, 42), -0.400, 0.12,
          0.60, 0.60, 0.75,
          0.40, 0.55),
      PlannerRapidPreemptDecision::kCommit);
  EXPECT_EQ(
      planner_rapid_preempt_decision(
          true, true, 2, flight(1, 43), flight(1, 42), -0.39999995,
          0.12, 0.59999995, 0.60, 0.75, 0.40, 0.55),
      PlannerRapidPreemptDecision::kCommit);
  EXPECT_EQ(
      planner_rapid_preempt_decision(
          true, true, 2, flight(1, 43), flight(1, 42), -0.550, 0.12,
          0.75, 0.60, 0.75,
          0.40, 0.55),
      PlannerRapidPreemptDecision::kCommit);
  EXPECT_EQ(
      planner_rapid_preempt_decision(
          true, true, 2, flight(1, 43), flight(1, 42), -0.551, 0.12,
          0.65, 0.60, 0.75,
          0.40, 0.55),
      PlannerRapidPreemptDecision::kExpiredAfterCommitDelaySupport);
  EXPECT_EQ(
      planner_rapid_preempt_decision(
          true, true, 2, flight(1, 43), flight(1, 42), -0.55000005,
          0.12, 0.75000005, 0.60, 0.75, 0.40, 0.55),
      PlannerRapidPreemptDecision::kCommit);
  EXPECT_EQ(
      planner_rapid_preempt_decision(
          true, true, 2, flight(1, 43), flight(1, 42), -0.50, 0.12,
          0.751, 0.60, 0.75,
          0.40, 0.55),
      PlannerRapidPreemptDecision::kWaitAboveSupport);
  EXPECT_EQ(
      planner_rapid_preempt_decision(
          true, true, 2, flight(1, 43), flight(1, 42), -0.50, 0.12,
          0.599, 0.60, 0.75,
          0.40, 0.55),
      PlannerRapidPreemptDecision::kExpiredBelowSupport);
  EXPECT_EQ(
      planner_rapid_preempt_decision(
          true, true, 1, flight(1, 43), flight(1, 42), -0.50, 0.12,
          0.65, 0.60, 0.75,
          0.40, 0.55),
      PlannerRapidPreemptDecision::kInvalidCandidate);
  EXPECT_EQ(
      planner_rapid_preempt_decision(
          false, true, 2, flight(1, 43), flight(1, 42), -0.50, 0.12,
          0.65, 0.60, 0.75,
          0.40, 0.55),
      PlannerRapidPreemptDecision::kDisabled);
}

TEST(PpPlannerLifecycle, Gate3ForceEveryFlightBypassesOnlyTemporalAdmission) {
  const auto flight = [](std::uint64_t epoch, std::uint64_t id) {
    return PlannerFlightIdentity{epoch, id};
  };
  EXPECT_EQ(
      planner_gate3_force_every_flight_decision(
          true, true, 2, flight(1, 43), flight(1, 42), 0.481),
      PlannerRapidPreemptDecision::kCommit);
  EXPECT_EQ(
      planner_gate3_force_every_flight_decision(
          true, true, 2, flight(1, 43), flight(1, 42), 0.001),
      PlannerRapidPreemptDecision::kCommit);
  EXPECT_EQ(
      planner_gate3_force_every_flight_decision(
          true, true, 2, flight(1, 42), flight(1, 42), 0.481),
      PlannerRapidPreemptDecision::kSameFlight);
  EXPECT_EQ(
      planner_gate3_force_every_flight_decision(
          true, true, 2, flight(1, 43), flight(1, 42), 0.0),
      PlannerRapidPreemptDecision::kInvalidCandidate);
  EXPECT_EQ(
      planner_gate3_force_every_flight_decision(
          true, true, 3, flight(1, 43), flight(1, 42), 0.481),
      PlannerRapidPreemptDecision::kInvalidCandidate);
  EXPECT_EQ(
      planner_gate3_force_every_flight_decision(
          false, true, 2, flight(1, 43), flight(1, 42), 0.481),
      PlannerRapidPreemptDecision::kDisabled);
}

TEST(PpPlannerLifecycle, Schema22CompletionLedgerIsIdentityAwareAndExactlyOnce) {
  const auto flight = [](std::uint64_t epoch, std::uint64_t id) {
    return PlannerFlightIdentity{epoch, id};
  };
  PlannerFlightCompletionState state;
  EXPECT_STREQ(kPlannerCompletionLogPrefix, "[pp completion]");
  EXPECT_STREQ(
      planner_flight_completion_kind_name(
          PlannerFlightCompletionKind::kNone),
      "none");
  EXPECT_STREQ(
      planner_flight_completion_kind_name(
          PlannerFlightCompletionKind::kRapidPreempt),
      "preempt");
  EXPECT_STREQ(
      planner_flight_completion_kind_name(
          PlannerFlightCompletionKind::kNativeEnd),
      "native");

  EXPECT_FALSE(planner_record_flight_completion_once(
      false, flight(1, 34), PlannerFlightCompletionKind::kRapidPreempt,
      state));
  EXPECT_FALSE(planner_record_flight_completion_once(
      true, flight(0, 34), PlannerFlightCompletionKind::kRapidPreempt,
      state));
  EXPECT_FALSE(planner_record_flight_completion_once(
      true, flight(1, 34), PlannerFlightCompletionKind::kNone, state));
  EXPECT_EQ(state.sequence, 0u);

  EXPECT_TRUE(planner_record_flight_completion_once(
      true, flight(1, 34), PlannerFlightCompletionKind::kRapidPreempt,
      state));
  EXPECT_EQ(state.sequence, 1u);
  EXPECT_EQ(state.completed_identity.producer_epoch, 1u);
  EXPECT_EQ(state.completed_identity.flight_id, 34u);
  EXPECT_EQ(state.kind, PlannerFlightCompletionKind::kRapidPreempt);

  // A duplicate native-end path and any older identity cannot create another
  // edge or mutate the latched event.
  EXPECT_FALSE(planner_record_flight_completion_once(
      true, flight(1, 34), PlannerFlightCompletionKind::kNativeEnd, state));
  EXPECT_FALSE(planner_record_flight_completion_once(
      true, flight(1, 33), PlannerFlightCompletionKind::kNativeEnd, state));
  EXPECT_EQ(state.sequence, 1u);
  EXPECT_EQ(state.completed_identity.flight_id, 34u);
  EXPECT_EQ(state.kind, PlannerFlightCompletionKind::kRapidPreempt);

  EXPECT_TRUE(planner_record_flight_completion_once(
      true, flight(1, 35), PlannerFlightCompletionKind::kNativeEnd, state));
  EXPECT_EQ(state.sequence, 2u);
  EXPECT_EQ(state.completed_identity.flight_id, 35u);
  EXPECT_EQ(state.kind, PlannerFlightCompletionKind::kNativeEnd);

  // A producer restart makes (epoch 2, flight 1) a genuinely fresh identity;
  // an old-epoch high flight ID remains stale after that transition.
  EXPECT_TRUE(planner_record_flight_completion_once(
      true, flight(2, 1), PlannerFlightCompletionKind::kRapidPreempt, state));
  EXPECT_EQ(state.sequence, 3u);
  EXPECT_EQ(state.completed_identity.producer_epoch, 2u);
  EXPECT_EQ(state.completed_identity.flight_id, 1u);
  EXPECT_FALSE(planner_record_flight_completion_once(
      true, flight(1, 99), PlannerFlightCompletionKind::kNativeEnd, state));
  EXPECT_EQ(state.sequence, 3u);
}

TEST(PpPlannerLifecycle, Schema22RapidCommandSupportIsStaticAndWorldFrameOnly) {
  EXPECT_EQ(kPlannerRapidHomeStationRange,
            (std::array<double, 4>{0.0, 0.0, -0.06, 0.06}));
  EXPECT_DOUBLE_EQ(kPlannerRapidCommandStepMaxM, 0.06);
  EXPECT_DOUBLE_EQ(kPlannerRapidSupportNumericTolerance, 1.0e-9);
  auto check = [&](int frame_code, double station_x, double station_y,
                   const std::array<double, 4>& station_range,
                   double command_anchor_y, double command_step_max) {
    return planner_rapid_static_command_support(
        0, frame_code, 1.05, 2.0, true, station_x, station_y,
        true, 0.0, 0.0, station_range,
        kPlannerRapidSupportNumericTolerance, 0.0, command_anchor_y,
        command_step_max, 0.90, 1.20,
        kPlannerRapidSupportNumericTolerance, 3.0);
  };

  // The measured robot may be far from the legal HOME-relative command after a miss. It is
  // deliberately not an input to the static admission function and therefore cannot veto D.
  constexpr double current_base_x = 0.42;
  constexpr double current_base_y = -0.38;
  const auto legal = check(
      0, 0.0, 0.05, kPlannerRapidHomeStationRange, 0.0,
      kPlannerRapidCommandStepMaxM + kPlannerRapidSupportNumericTolerance);
  EXPECT_GT(std::hypot(current_base_x, current_base_y - 0.05), 0.21);
  EXPECT_TRUE(legal.ok);
  EXPECT_TRUE(legal.absolute_station_ok);
  EXPECT_TRUE(legal.command_step_ok);

  const auto absolute_out =
      check(0, 0.0, 0.060000002, kPlannerRapidHomeStationRange, 0.0, 0.20);
  EXPECT_FALSE(absolute_out.ok);
  EXPECT_FALSE(absolute_out.absolute_station_ok);
  EXPECT_TRUE(absolute_out.command_step_ok);

  const std::array<double, 4> wide_station_range = {
      -1.0, 1.0, -1.0, 1.0};
  const auto command_step_out =
      check(0, 0.0, 0.000000002, wide_station_range, -0.06,
            kPlannerRapidCommandStepMaxM +
                kPlannerRapidSupportNumericTolerance);
  EXPECT_FALSE(command_step_out.ok);
  EXPECT_TRUE(command_step_out.absolute_station_ok);
  EXPECT_FALSE(command_step_out.command_step_ok);

  const auto base_link_frame =
      check(1, 0.0, 0.05, kPlannerRapidHomeStationRange, 0.0,
            kPlannerRapidCommandStepMaxM +
                kPlannerRapidSupportNumericTolerance);
  EXPECT_FALSE(base_link_frame.ok);
}

TEST(PpPlannerLifecycle, Schema22ExactEnvelopeIsDistributionTelemetryOnly) {
  auto exact = [](int frame_code, double station_x, double station_y,
                  double z, double speed, bool velocity_ok) {
    return planner_schema22_exact_command_support(
        0, frame_code, z, speed, velocity_ok, station_x, station_y,
        true, 0.0, 0.0, 0.90, 1.20, 3.0);
  };

  // Representative Gate3/world-frame first-shot tuple at the closed HOME boundary.
  const auto legal = exact(0, 0.0, 0.06, 1.05, 2.0, true);
  EXPECT_TRUE(legal.ok);
  EXPECT_TRUE(legal.absolute_station_ok);
  EXPECT_TRUE(legal.command_step_ok);

  // No CLI-style margin exists in this exact classifier. These misses quantify
  // distribution drift; they no longer veto an otherwise valid flight.
  EXPECT_FALSE(exact(1, 0.0, 0.05, 1.05, 2.0, true).ok);
  EXPECT_FALSE(exact(0, 0.001, 0.05, 1.05, 2.0, true).ok);
  EXPECT_FALSE(exact(0, 0.0, 0.061, 1.05, 2.0, true).ok);
  EXPECT_FALSE(exact(0, 0.0, 0.05, 1.201, 2.0, true).ok);
  EXPECT_FALSE(exact(0, 0.0, 0.05, 1.05, 2.0, false).ok);
  EXPECT_FALSE(exact(0, 0.0, 0.05, 1.05, 3.001, true).ok);
  EXPECT_FALSE(planner_schema22_exact_command_support(
      0, 0, 1.05, 2.0, true, 0.0, 0.05, false, 0.0, 0.0,
      0.90, 1.20, 3.0).ok);

  // HOME comes from the fresh localized MOTION-entry base and is not inferred
  // from the first ball. A normal command-space offset is therefore reported,
  // while admission is owned by wire validity rather than this classifier.
  constexpr double first_command_station_x = -0.500;
  constexpr double first_command_station_y = -0.762;
  constexpr double measured_base_x = -0.495;
  const auto localized_home_classification =
      planner_schema22_exact_command_support(
          0, 0, 1.05, 2.0, true,
          first_command_station_x, first_command_station_y, true,
          measured_base_x, first_command_station_y,
          0.90, 1.20, 3.0);
  EXPECT_GT(std::fabs(measured_base_x - first_command_station_x), 0.004);
  EXPECT_FALSE(localized_home_classification.ok);
  EXPECT_FALSE(localized_home_classification.absolute_station_ok);
  EXPECT_EQ(
      planner_schema22_core_box_coverage_disposition(
          localized_home_classification),
      PlannerSchema22CoreBoxCoverageDisposition::kTelemetryOnly);

  // Real Aug-22 tuples that the old Runner swallowed: high FH z/velocity,
  // negative-BH vz, and a >6 cm derived station are all telemetry now.
  constexpr std::array<double, 3> real_fh_velocity = {
      2.413, 0.770, 0.686};
  const double real_fh_speed = std::sqrt(
      real_fh_velocity[0] * real_fh_velocity[0] +
      real_fh_velocity[1] * real_fh_velocity[1] +
      real_fh_velocity[2] * real_fh_velocity[2]);
  const auto real_fh = exact(0, 0.0, 0.0, 1.388, real_fh_speed, false);
  EXPECT_FALSE(real_fh.ok);
  EXPECT_FALSE(real_fh.z_ok);
  EXPECT_FALSE(real_fh.velocity_ok);
  EXPECT_EQ(planner_schema22_core_box_coverage_disposition(real_fh),
            PlannerSchema22CoreBoxCoverageDisposition::kTelemetryOnly);

  constexpr std::array<double, 3> real_bh_velocity = {
      2.095, 0.0649, -0.259};
  const double real_bh_speed = std::sqrt(
      real_bh_velocity[0] * real_bh_velocity[0] +
      real_bh_velocity[1] * real_bh_velocity[1] +
      real_bh_velocity[2] * real_bh_velocity[2]);
  EXPECT_LT(real_bh_velocity[2], 0.0);
  const auto real_bh = exact(0, 0.0, 0.0, 1.032, real_bh_speed, false);
  EXPECT_FALSE(real_bh.ok);
  EXPECT_TRUE(real_bh.z_ok);
  EXPECT_FALSE(real_bh.velocity_ok);
  EXPECT_EQ(planner_schema22_core_box_coverage_disposition(real_bh),
            PlannerSchema22CoreBoxCoverageDisposition::kTelemetryOnly);

  const auto real_station_step = exact(0, 0.0, 0.120, 1.05, 2.0, true);
  EXPECT_FALSE(real_station_step.ok);
  EXPECT_FALSE(real_station_step.absolute_station_ok);
  EXPECT_FALSE(real_station_step.command_step_ok);
  EXPECT_EQ(planner_schema22_core_box_coverage_disposition(real_station_step),
            PlannerSchema22CoreBoxCoverageDisposition::kTelemetryOnly);
}

TEST(PpPlannerLifecycle, Schema22ProjectsLegacyBaseCommandIntoHomeSupport) {
  const auto positive_tail = planner_schema22_project_station_to_home(
      -0.31, -0.642, true, -0.495, -0.762);
  EXPECT_DOUBLE_EQ(positive_tail[0], -0.495);
  EXPECT_DOUBLE_EQ(positive_tail[1], -0.702);

  const auto negative_tail = planner_schema22_project_station_to_home(
      -0.70, -0.986, true, -0.495, -0.762);
  EXPECT_DOUBLE_EQ(negative_tail[0], -0.495);
  EXPECT_DOUBLE_EQ(negative_tail[1], -0.822);

  const auto inside = planner_schema22_project_station_to_home(
      -0.51, -0.750, true, -0.495, -0.762);
  EXPECT_DOUBLE_EQ(inside[0], -0.495);
  EXPECT_DOUBLE_EQ(inside[1], -0.750);

  // The Runner blocks actor entry until localized HOME exists; the pure helper
  // itself leaves the request untouched so it cannot fabricate an anchor.
  const auto unset = planner_schema22_project_station_to_home(
      -0.31, -0.642, false, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(unset[0], -0.31);
  EXPECT_DOUBLE_EQ(unset[1], -0.642);
}

TEST(PpPlannerLifecycle, Schema22OppositeSidePreemptIsAtomicAndDoesNotQueueDirection) {
  const auto edge = planner_rapid_preempt_edge(900, -1.0, 17);
  EXPECT_EQ(edge.swing_direction, -1);
  EXPECT_EQ(edge.pending_swing_direction, 0);
  EXPECT_EQ(edge.clock_origin_tick, 900u);
  EXPECT_EQ(edge.shot_sequence, 18u);
  EXPECT_EQ(planner_swing_clock_origin_after_engage(
                900, 123, 1, 1, edge.swing_direction, 1, 17,
                edge.shot_sequence),
            900u);
  const auto same_side_edge = planner_rapid_preempt_edge(901, 1.0, 18);
  EXPECT_EQ(planner_swing_clock_origin_after_engage(
                901, 123, 1, 1, 1, 1, 18,
                same_side_edge.shot_sequence),
            901u);
  // The old clip's already-complete tail cannot complete the replacement
  // shot on this same tick: the atomic edge installs a fresh positive clock.
  EXPECT_FALSE(planner_native_clip_end_complete(0.60, -1.10));
}

TEST(PpPlannerLifecycle, Schema22CandidateLatchIsMonotonicAndCannotBeOverwritten) {
  using D = PlannerRapidCandidateIdentityDecision;
  const auto flight = [](std::uint64_t epoch, std::uint64_t id) {
    return PlannerFlightIdentity{epoch, id};
  };
  EXPECT_EQ(planner_rapid_candidate_identity_decision(
                flight(1, 42), 9, 100, flight(1, 42), flight(1, 42),
                flight(0, 0), false, flight(0, 0), 0, 0),
            D::kSameFrozenFlight);
  EXPECT_EQ(planner_rapid_candidate_identity_decision(
                flight(1, 41), 9, 100, flight(1, 42), flight(1, 41),
                flight(0, 0), false, flight(0, 0), 0, 0),
            D::kRejectConsumedFlight);
  EXPECT_EQ(planner_rapid_candidate_identity_decision(
                flight(1, 43), 1, 101, flight(1, 42), flight(1, 42),
                flight(0, 0), false, flight(0, 0), 0, 0),
            D::kLatchFreshFlight);
  EXPECT_EQ(planner_rapid_candidate_identity_decision(
                flight(1, 41), 9, 102, flight(1, 42), flight(1, 42),
                flight(0, 0), false, flight(0, 0), 0, 0),
            D::kRejectNonmonotonicFlight);
  EXPECT_EQ(planner_rapid_candidate_identity_decision(
                flight(1, 43), 2, 103, flight(1, 42), flight(1, 42),
                flight(1, 43), false, flight(0, 0), 0, 0),
            D::kRejectExpiredFlight);
  EXPECT_EQ(planner_rapid_candidate_identity_decision(
                flight(1, 43), 3, 104, flight(1, 42), flight(1, 42),
                flight(1, 44), false, flight(0, 0), 0, 0),
            D::kRejectExpiredFlight);
  EXPECT_EQ(planner_rapid_candidate_identity_decision(
                flight(1, 43), 1, 101, flight(1, 42), flight(1, 42),
                flight(0, 0), true, flight(1, 43), 1, 101),
            D::kKeepPendingRevision);
  EXPECT_EQ(planner_rapid_candidate_identity_decision(
                flight(1, 43), 1, 102, flight(1, 42), flight(1, 42),
                flight(0, 0), true, flight(1, 43), 1, 101),
            D::kKeepPendingRevision);
  EXPECT_EQ(planner_rapid_candidate_identity_decision(
                flight(1, 43), 2, 101, flight(1, 42), flight(1, 42),
                flight(0, 0), true, flight(1, 43), 1, 101),
            D::kKeepPendingRevision);
  EXPECT_EQ(planner_rapid_candidate_identity_decision(
                flight(1, 43), 2, 102, flight(1, 42), flight(1, 42),
                flight(0, 0), true, flight(1, 43), 1, 101),
            D::kUpdatePendingRevision);
  EXPECT_EQ(planner_rapid_candidate_identity_decision(
                flight(1, 44), 1, 103, flight(1, 42), flight(1, 42),
                flight(0, 0), true, flight(1, 43), 1, 101),
            D::kBusyDropDifferentFlight);
  EXPECT_EQ(planner_rapid_candidate_identity_decision(
                flight(1, 40), 1, 104, flight(1, 42), flight(1, 42),
                flight(0, 0), true, flight(1, 43), 1, 101),
            D::kRejectNonmonotonicFlight);
}

TEST(PpPlannerLifecycle, Schema22ExplicitEpochTransitionBaselinesFirstActiveFlight) {
  using E = PlannerProducerEpochDecision;
  using D = PlannerRapidCandidateIdentityDecision;
  const auto flight = [](std::uint64_t epoch, std::uint64_t id) {
    return PlannerFlightIdentity{epoch, id};
  };

  EXPECT_EQ(planner_producer_epoch_decision(0, 0, false), E::kInvalid);
  EXPECT_EQ(planner_producer_epoch_decision(1, 0, false), E::kInitialize);
  EXPECT_EQ(planner_producer_epoch_decision(1, 1, true), E::kSame);
  EXPECT_EQ(planner_producer_epoch_decision(1, 2, true), E::kRejectOlder);
  EXPECT_EQ(planner_producer_epoch_decision(2, 1, false), E::kResetIdle);
  EXPECT_EQ(planner_producer_epoch_decision(2, 1, true),
            E::kResetActiveBaseline);

  // Current schema-2 input pins epoch=1 because its wire has no boot nonce. If
  // a future schema supplies an authoritative epoch transition, active old
  // epoch flight 34 is preserved: epoch 2 flight 1 becomes an explicit consumed
  // baseline and only epoch 2 flight 2 is eligible as the fresh next ball.
  EXPECT_EQ(planner_rapid_candidate_identity_decision(
                flight(2, 1), 1, 1, flight(1, 34), flight(2, 1),
                flight(0, 0), false, flight(0, 0), 0, 0),
            D::kRejectConsumedFlight);
  EXPECT_EQ(planner_rapid_candidate_identity_decision(
                flight(2, 2), 1, 2, flight(1, 34), flight(2, 1),
                flight(0, 0), false, flight(0, 0), 0, 0),
            D::kLatchFreshFlight);
  EXPECT_EQ(planner_rapid_candidate_identity_decision(
                flight(1, 35), 1, 101, flight(1, 34), flight(2, 1),
                flight(0, 0), false, flight(0, 0), 0, 0),
            D::kRejectNonmonotonicFlight);
}

TEST(PpPlannerLifecycle, Schema22StreamingCannotConsumeAnotherFlight) {
  const auto flight = [](std::uint64_t epoch, std::uint64_t id) {
    return PlannerFlightIdentity{epoch, id};
  };
  EXPECT_TRUE(planner_stream_target_matches_frozen_flight(
      true, flight(1, 71), flight(1, 71)));
  EXPECT_FALSE(planner_stream_target_matches_frozen_flight(
      true, flight(1, 72), flight(1, 71)));
  EXPECT_FALSE(planner_stream_target_matches_frozen_flight(
      true, flight(2, 71), flight(1, 71)));
  EXPECT_FALSE(planner_stream_target_matches_frozen_flight(
      true, flight(0, 0), flight(1, 71)));
  // Every pre-schema22 runtime keeps its per-tick streaming decision unchanged.
  EXPECT_TRUE(planner_stream_target_matches_frozen_flight(
      false, flight(2, 72), flight(1, 71)));
}

TEST(PpPlannerLifecycle, Schema22PendingLatchDoesNotGrantInfiniteFreshness) {
  EXPECT_TRUE(planner_rapid_pending_is_fresh(10.20, 10.00, 0.20));
  EXPECT_FALSE(planner_rapid_pending_is_fresh(10.201, 10.00, 0.20));
  EXPECT_FALSE(planner_rapid_pending_is_fresh(9.99, 10.00, 0.20));
  EXPECT_FALSE(planner_rapid_pending_is_fresh(
      std::numeric_limits<double>::quiet_NaN(), 10.00, 0.20));
}

TEST(PpPlannerLifecycle, Build2RootfixPublishesHomeAtContactTailButCompletesAtNativeEnd) {
  constexpr double kHomeEdgeS = 0.12;
  constexpr double kNativeClipMinTtsS = -1.10;

  // Schema21 uses the same signed-TTS contact-frame owner as V4 without
  // inheriting V4's observation ABI. A 0.40 s late COMMIT therefore starts the
  // actor and native reference at exactly 0.40 s with zero synthetic lateness.
  const auto rootfix_entry = planner_contact_frame_start(
      true, false, 0.40, 0.15);
  EXPECT_DOUBLE_EQ(rootfix_entry.clock_tts_s, 0.40);
  EXPECT_DOUBLE_EQ(rootfix_entry.expected_strike_lateness_s, 0.0);
  EXPECT_FALSE(rootfix_entry.late_phase_clamped);
  EXPECT_DOUBLE_EQ(planner_motion_reference_tts(true, 0.40, 0.451), 0.40);

  auto decision = planner_native_tail_home_step(
      true, true, 1, true, true, -0.119, kHomeEdgeS, false);
  EXPECT_FALSE(decision.latch_recovery);
  EXPECT_FALSE(decision.publish_home_base_target);

  decision = planner_native_tail_home_step(
      true, true, 1, true, true, -0.12, kHomeEdgeS, false);
  EXPECT_TRUE(decision.latch_recovery);
  EXPECT_TRUE(decision.publish_home_base_target);
  decision = planner_native_tail_home_step(
      true, true, 1, true, true, -0.11999995, kHomeEdgeS, false);
  EXPECT_TRUE(decision.latch_recovery);
  EXPECT_TRUE(decision.publish_home_base_target);
  // The HOME edge must not terminate level 1 or resample a clip: native
  // completion remains much later, at the true segment end.
  EXPECT_FALSE(planner_native_clip_end_complete(-0.12, kNativeClipMinTtsS));

  decision = planner_native_tail_home_step(
      true, true, 1, true, true,
      std::numeric_limits<double>::quiet_NaN(), kHomeEdgeS, true);
  EXPECT_FALSE(decision.latch_recovery);
  EXPECT_TRUE(decision.publish_home_base_target);

  EXPECT_FALSE(planner_native_clip_end_complete(kNativeClipMinTtsS,
                                                kNativeClipMinTtsS));
  EXPECT_TRUE(planner_native_clip_end_complete(-1.101,
                                               kNativeClipMinTtsS));
  EXPECT_FALSE(planner_native_clip_end_complete(
      std::numeric_limits<double>::quiet_NaN(), kNativeClipMinTtsS));

  // Cold HOLD, missing session HOME, or legacy contracts cannot activate this
  // path. Level 0 retains the already-copied hold station in PpPolicy.
  EXPECT_FALSE(planner_native_tail_home_step(
                   true, true, 0, false, true, -0.20, kHomeEdgeS, false)
                   .publish_home_base_target);
  EXPECT_FALSE(planner_native_tail_home_step(
                   true, true, 1, true, false, -0.20, kHomeEdgeS, false)
                   .publish_home_base_target);
  EXPECT_FALSE(planner_native_tail_home_step(
                   false, true, 1, true, true, -0.20, kHomeEdgeS, false)
                   .publish_home_base_target);
}

TEST(PpPlannerLifecycle, LateRevealPrefixCapMatchesTrainingSemantics) {
  EXPECT_DOUBLE_EQ(planner_late_reveal_reference_floor_tts(0.82, 1.0), 0.0);
  EXPECT_NEAR(planner_late_reveal_reference_floor_tts(0.82, 0.55), 0.369,
              1.0e-12);
  EXPECT_DOUBLE_EQ(planner_late_reveal_reference_floor_tts(0.96, 0.0), 0.96);
  EXPECT_DOUBLE_EQ(planner_late_reveal_reference_floor_tts(0.96, 2.0), 0.0);
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

TEST(PpPlannerLifecycle, ContinuousV4MotionFrameIsOnlySignedActorTts) {
  const auto v4_early = planner_contact_frame_start(
      true, true, 0.40, 0.451);
  EXPECT_DOUBLE_EQ(v4_early.clock_tts_s, 0.40);
  EXPECT_DOUBLE_EQ(v4_early.expected_strike_lateness_s, 0.0);
  EXPECT_FALSE(v4_early.late_phase_clamped);

  const auto v4_nominal = planner_contact_frame_start(
      true, true, 0.82, 0.451);
  EXPECT_DOUBLE_EQ(v4_nominal.clock_tts_s, 0.82);
  EXPECT_DOUBLE_EQ(
      planner_motion_reference_tts(true, 0.40, 0.451), 0.40);
  EXPECT_DOUBLE_EQ(
      planner_motion_reference_tts(true, -0.12, 0.0), -0.12);

  // V1-v3 retain the frozen-clip prefix behavior byte-for-byte.
  const auto legacy = planner_contact_frame_start(
      false, true, 0.40, 0.451);
  EXPECT_DOUBLE_EQ(legacy.clock_tts_s, 0.451);
  EXPECT_TRUE(legacy.late_phase_clamped);
  EXPECT_DOUBLE_EQ(
      planner_motion_reference_tts(false, 0.40, 0.451), 0.451);
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

TEST(PpPlannerLifecycle, ContinuousV2UsesFreshExternalFlightIdentity) {
  // Revisions of one external flight remain one-shot even if strike time moves
  // farther than v1's heuristic tolerance.
  EXPECT_TRUE(same_planner_shot_identity(
      true, 17, 17, 101.40, 101.00, 0.25));
  // A genuinely fresh rapid ball is not mistaken for the consumed shot merely
  // because its strike time is less than 0.25 s away.
  EXPECT_FALSE(same_planner_shot_identity(
      true, 18, 17, 101.12, 101.00, 0.25));
  EXPECT_FALSE(same_planner_shot_identity(
      true, 0, 17, 101.00, 101.00, 0.25));
  // v1 remains compatible with its strike-time identity fallback.
  EXPECT_TRUE(same_planner_shot_identity(
      false, 18, 17, 101.12, 101.00, 0.25));
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

TEST(PpPlannerLifecycle, NewShotSequenceResetsClockAcrossSameSideCompletion) {
  EXPECT_TRUE(planner_swing_clock_needs_reset(1, 1, 1, 1, 5, 6));
  EXPECT_FALSE(planner_swing_clock_needs_reset(1, 1, 1, 1, 5, 5));
  EXPECT_TRUE(planner_swing_clock_needs_reset(1, 0, 1, 1, 5, 5));
  EXPECT_TRUE(planner_swing_clock_needs_reset(1, 1, -1, 1, 5, 5));
}

TEST(PpPlannerLifecycle, CompletedShotThenNewFlightCannotReuseExpiredClock) {
  constexpr double kDt = 0.02;
  // 20260818T002302Z hardware trace (artifact-provenance model_7800): completion at tick 5003,
  // followed by the conflicting same-side flight-6 engage at tick 5004 (CSV rows 5005/5006).
  constexpr std::uint64_t kCompletionTick = 5003;
  constexpr std::uint64_t kEngageTick = 5004;
  int cached_level = 1;
  planner_sync_completion_cache(cached_level);
  EXPECT_EQ(cached_level, 0);

  const std::uint64_t old_origin = 4900;
  const double old_elapsed = planner_swing_elapsed_s(
      kCompletionTick, old_origin, kDt, 1.0);
  EXPECT_GT(old_elapsed, 2.0);

  // The retained mailbox entry is the consumed physical shot. Production returns before engage,
  // so neither sequence nor origin changes.
  EXPECT_TRUE(same_planner_shot(101.12, 101.00, 0.25));
  const std::uint64_t retained_origin =
      planner_swing_clock_origin_after_engage(
          kEngageTick - 1, old_origin, 0, cached_level, 1, 1, 5, 5);
  EXPECT_EQ(retained_origin, old_origin);

  // A genuinely new flight increments the sequence once. This is the exact helper called by
  // ComputeCommand before ScriptedTarget(), so a same-side engage gets a fresh origin.
  EXPECT_FALSE(same_planner_shot(102.0, 101.0, 0.25));
  const std::uint64_t new_origin =
      planner_swing_clock_origin_after_engage(
          kEngageTick, retained_origin, 1, cached_level, 1, 1, 5, 6);
  EXPECT_EQ(new_origin, kEngageTick);
  constexpr double kActorEntryTts = 0.303842;
  EXPECT_DOUBLE_EQ(
      planner_running_tts(kActorEntryTts, kEngageTick, new_origin, kDt, 1.0),
      kActorEntryTts);
  EXPECT_FALSE(planner_swing_clock_needs_reset(1, 1, 1, 1, 6, 6));
  EXPECT_NEAR(
      planner_running_tts(kActorEntryTts, kEngageTick + 1, new_origin, kDt, 1.0),
      0.283842, 1.0e-12);
  EXPECT_EQ(planner_swing_clock_origin_after_engage(
                kEngageTick + 1, new_origin, 1, 1, 1, 1, 6, 6),
            new_origin);
}

TEST(PpPlannerLifecycle, StaleLocalizationPreservesVerifiedStationAnchor) {
  EXPECT_DOUBLE_EQ(planner_stale_station_coordinate(true, -0.42, -0.10),
                   -0.42);
  EXPECT_DOUBLE_EQ(planner_stale_station_coordinate(false, -0.42, -0.10),
                   -0.10);
  EXPECT_DOUBLE_EQ(
      planner_stale_station_coordinate(
          true, std::numeric_limits<double>::quiet_NaN(), -0.10),
      -0.10);
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

TEST(PpPlannerLifecycle, ContinuousV3AuthoritativeCommandHealthFailsClosed) {
  EXPECT_TRUE(planner_command_health_blocks_release(true, true, true));
  EXPECT_TRUE(planner_command_health_blocks_release(false, true, true));
  EXPECT_FALSE(planner_command_health_blocks_release(true, false, true));
  EXPECT_TRUE(planner_command_age_unhealthy(0.051, 0.05, true));
  EXPECT_TRUE(planner_command_age_unhealthy(
      std::numeric_limits<double>::quiet_NaN(), 0.05, true));
  EXPECT_TRUE(planner_command_age_unhealthy(-0.01, 0.05, true));
  EXPECT_TRUE(planner_command_age_unhealthy(
      0.01, std::numeric_limits<double>::quiet_NaN(), true));
  EXPECT_FALSE(planner_command_age_unhealthy(
      std::numeric_limits<double>::quiet_NaN(), 0.05, false));
  EXPECT_TRUE(planner_invalid_revision_unhealthy(
      true, 0.0, 0.10, true));
  EXPECT_FALSE(planner_invalid_revision_unhealthy(
      true, 0.0, 0.10, false));
  EXPECT_TRUE(planner_invalid_revision_unhealthy(
      true, 0.11, 0.10, false));
  EXPECT_FALSE(planner_invalid_revision_unhealthy(
      false, 1.0, 0.10, true));
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
