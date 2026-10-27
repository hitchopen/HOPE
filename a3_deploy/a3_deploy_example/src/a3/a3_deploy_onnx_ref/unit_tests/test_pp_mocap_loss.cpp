#include "a3_pingpong/pp_runner_control.hpp"
#include "a3_pingpong/pp_planner_input.hpp"
#include <chrono>
#include <thread>
#include <gtest/gtest.h>

namespace {
using namespace a3_pingpong;

TEST(PpMocapLoss, RallyStopsOnceAndRecoveryRequiresExplicitReentry) {
  PpRunnerControl control(RunnerMode::kMotion, 1, "test");
  control.RequireMocap(true);
  EXPECT_FALSE(control.ObserveMocap(true));
  EXPECT_TRUE(control.ObserveMocap(false));
  EXPECT_EQ(control.mode(), RunnerMode::kPdStand);
  EXPECT_FALSE(control.ObserveMocap(false));
  control.SetRuntimeMode(RunnerMode::kMotion);
  EXPECT_EQ(control.mode(), RunnerMode::kPdStand);
  EXPECT_FALSE(control.ObserveMocap(true));
  EXPECT_EQ(control.mode(), RunnerMode::kPdStand);
  control.SetRuntimeMode(RunnerMode::kMotion);
  EXPECT_EQ(control.mode(), RunnerMode::kMotion);
}

TEST(PpMocapLoss, ServeAbortsThroughValidatedRecoveryEvenIfStreamReturns) {
  PpRunnerControl control(RunnerMode::kServe, 1, "test");
  control.RequireMocap(true);
  EXPECT_TRUE(control.ObserveMocap(false));
  EXPECT_TRUE(control.MocapStopRequested());
  EXPECT_EQ(control.mode(), RunnerMode::kServe);
  control.ObserveMocap(true);
  EXPECT_TRUE(control.MocapStopRequested());
  EXPECT_FALSE(control.RecoverTerminalServeToStand(15, false));
  EXPECT_TRUE(control.RecoverTerminalServeToStand(15, true));
  EXPECT_EQ(control.mode(), RunnerMode::kPdStand);
}

TEST(PpMocapLoss, LateServeCompletionCannotRestartReceive) {
  PpRunnerControl control(RunnerMode::kServe, 1, "test");
  control.RequireMocap(true);
  control.ObserveMocap(false);
  control.ObserveMocap(true);
  ASSERT_TRUE(control.CompleteServe());
  EXPECT_EQ(control.mode(), RunnerMode::kPdStand);
  EXPECT_EQ(control.EncodeMocapSafety()[6], 1.);
  EXPECT_EQ(control.EncodeMocapSafety()[7], 1.);
}

TEST(PpMocapLoss, JitterWithinDeadlineDoesNotRejectReceiveOrServeCompletion) {
  for (double age : {.005, .025, .060, .099}) {
    SCOPED_TRACE(age);
    PpRunnerControl control(RunnerMode::kPdStand, 1, "test");
    control.RequireMocap(true);
    const bool held = age > kBasePoseHoldAfterS;
    ASSERT_FALSE(control.ObserveMocap(true, held, age));
    EXPECT_EQ(control.EncodeMocapSafety()[5], held ? 0. : 1.);
    EXPECT_EQ(control.EncodeMocapSafety()[8], held ? 1. : 0.);
    ASSERT_TRUE(control.EnqueueLocalAction(RunnerAction::kSetReceiver));
    ASSERT_EQ(control.ProcessPending(false, false)[0].result, RunnerActionResult::kApplied);
    ASSERT_TRUE(control.EnqueueLocalAction(RunnerAction::kEnterMotion));
    ASSERT_EQ(control.ProcessPending(false, false)[0].result, RunnerActionResult::kApplied);
    EXPECT_EQ(control.mode(), RunnerMode::kMotion);
    ASSERT_TRUE(control.SetRuntimeMode(RunnerMode::kPdStand));
    ASSERT_TRUE(control.EnqueueLocalAction(RunnerAction::kSetServer));
    ASSERT_EQ(control.ProcessPending(false, false)[0].result, RunnerActionResult::kApplied);
    ASSERT_TRUE(control.EnqueueLocalAction(RunnerAction::kPrepareServe));
    ASSERT_EQ(control.ProcessPending(false, false, true, 0)[0].result, RunnerActionResult::kApplied);
    EXPECT_TRUE(control.ConsumeServePrepare());
    ASSERT_TRUE(control.CompleteServe());
    EXPECT_EQ(control.mode(), RunnerMode::kMotion);
    EXPECT_EQ(control.EncodeMocapSafety()[6], 0.);
    EXPECT_EQ(control.EncodeMocapSafety()[7], 0.);
  }
}

TEST(PpMocapLoss, CompletionAdmissionFailureRecordsSafetyReasonWithoutAnOperatorAction) {
  // Completion can run before the first watchdog observation. It must still
  // expose why normal receive was not resumed, without changing last action.
  for (bool missing_mocap : {true, false}) {
    SCOPED_TRACE(missing_mocap);
    PpRunnerControl control(RunnerMode::kServe, 1, "test");
    control.RequireMocap(true);
    if (!missing_mocap) {
      control.ObserveMocap(true);
      control.RequireOperatorHeartbeat(true);
    }
    const auto action = control.EncodeState();
    ASSERT_TRUE(control.CompleteServe());
    EXPECT_EQ(control.mode(), RunnerMode::kPdStand);
    const auto safety = control.EncodeMocapSafety();
    EXPECT_EQ(safety[6], missing_mocap ? 1. : 0.);
    EXPECT_EQ(safety[7], missing_mocap ? 1. : 0.);
    EXPECT_EQ(safety[13], missing_mocap ? 0. : 36.);
    EXPECT_EQ(safety[14], missing_mocap ? 0. : 1.);
    EXPECT_EQ(control.EncodeState()[19], action[19]);
    EXPECT_FALSE(control.CompleteServe());
    EXPECT_EQ(control.EncodeMocapSafety()[6], safety[6]);
    EXPECT_EQ(control.EncodeMocapSafety()[14], safety[14]);
  }
}

TEST(PpMocapLoss, PassiveAndTeleopAreNeverEnergizedOrOverridden) {
  for (auto mode : {RunnerMode::kPassive, RunnerMode::kPdStand, RunnerMode::kTeleop}) {
    PpRunnerControl control(mode, 1, "test");
    control.RequireMocap(true);
    EXPECT_FALSE(control.ObserveMocap(false));
    EXPECT_EQ(control.mode(), mode);
  }
  PpRunnerControl control(RunnerMode::kServe, 1, "test");
  control.RequireMocap(true);
  control.ObserveMocap(false);
  control.SetRuntimeMode(RunnerMode::kPassive);
  EXPECT_FALSE(control.CompleteServe());
  EXPECT_FALSE(control.RecoverTerminalServeToStand(15, true));
  EXPECT_EQ(control.mode(), RunnerMode::kPassive);
}

TEST(PpMocapLoss, EveryPlayEntryIsBlockedButStandAndGripperCleanupRemainAvailable) {
  PpRunnerControl control(RunnerMode::kPdStand, 1, "test");
  control.RequireMocap(true);
  for (auto action : {RunnerAction::kEnterMotion, RunnerAction::kPrepareServe, RunnerAction::kReadyToServe}) {
    ASSERT_TRUE(control.EnqueueLocalAction(action));
    auto decisions = control.ProcessPending(false, false, true);
    ASSERT_EQ(decisions.size(), 1U);
    EXPECT_EQ(decisions[0].reason, RunnerActionReason::kMocapUnavailable);
    EXPECT_EQ(control.mode(), RunnerMode::kPdStand);
  }
  ASSERT_TRUE(control.EnqueueLocalAction(RunnerAction::kEnterPdStand));
  EXPECT_EQ(control.ProcessPending(false, false)[0].result, RunnerActionResult::kAlreadySet);
  ASSERT_TRUE(control.EnqueueLocalAction(RunnerAction::kSetServer));
  EXPECT_EQ(control.ProcessPending(false, false)[0].result, RunnerActionResult::kApplied);
  ASSERT_TRUE(control.EnqueueLocalAction(RunnerAction::kOpenGripper));
  EXPECT_TRUE(control.ProcessPending(false, false, true, 0, true, -1, true)[0].request_open_gripper);
}

TEST(PpMocapLoss, OfflineServeDoesNotNeedMocap) {
  PpRunnerControl control(RunnerMode::kServe, 1, "test", 16, true);
  control.RequireMocap(false);
  EXPECT_FALSE(control.ObserveMocap(false));
  EXPECT_FALSE(control.MocapStopRequested());
  EXPECT_TRUE(control.CompleteServe());
  EXPECT_EQ(control.mode(), RunnerMode::kPdStand);
}

TEST(PpMocapLoss, MonotonicMailboxExpiresAndDuplicateFramesCannotRenewIt) {
  PpBasePoseInput input;
  PpBaseSample sample;
  const double now = PpNowWallSec();
  const auto sec = std::floor(now);
  const std::vector<double> packet{2,1,1,sec,std::floor((now-sec)*1e9),
      -.5,0,1,1,0,0,0,1,double(kV17RequiredBaseFlags),1,1};
  input.SetFromFlat(packet);
  EXPECT_TRUE(input.Latest(sample, .100, true));
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  input.SetFromFlat(packet);
  EXPECT_FALSE(input.Latest(sample, .100, true));
}

TEST(PpMocapLoss, RefusedModeChangeReturnsFalseAndNeverQueuesServePrepare) {
  PpRunnerControl control(RunnerMode::kPdStand, 1, "test");
  control.RequireMocap(true);
  EXPECT_FALSE(control.SetRuntimeMode(RunnerMode::kMotion));
  EXPECT_FALSE(control.SetRuntimeMode(RunnerMode::kServe, true));
  EXPECT_FALSE(control.ConsumeServePrepare());
  EXPECT_EQ(control.mode(), RunnerMode::kPdStand);
  control.ObserveMocap(true);
  EXPECT_TRUE(control.SetRuntimeMode(RunnerMode::kServe, true));
  EXPECT_TRUE(control.ConsumeServePrepare());
}

TEST(PpMocapLoss, ConcurrentFreshnessNeverAcknowledgesAnUncommittedPrepare) {
  PpRunnerControl control(RunnerMode::kPdStand, 1, "test");
  control.RequireMocap(true);
  ASSERT_TRUE(control.EnqueueLocalAction(RunnerAction::kSetServer));
  control.ProcessPending(false, false);
  std::atomic<bool> done{false};
  std::thread flicker([&] {
    while (!done.load()) {
      control.ObserveMocap(false);
      control.ObserveMocap(true);
    }
  });
  std::size_t mismatches = 0;
  for (int i = 0; i < 100000; ++i) {
    control.SetRuntimeMode(RunnerMode::kPdStand);
    control.ObserveMocap(true);
    control.EnqueueLocalAction(RunnerAction::kPrepareServe);
    const auto decision = control.ProcessPending(false, false, true, 0)[0];
    const bool applied = decision.result == RunnerActionResult::kApplied;
    if (applied != control.ConsumeServePrepare() || applied != decision.request_prepare_serve)
      ++mismatches;
  }
  done.store(true);
  flicker.join();
  EXPECT_EQ(mismatches, 0U);
}

TEST(PpMocapLoss, SafetyTelemetryRetainsStopReasonWithoutReplacingOperatorResult) {
  PpRunnerControl control(RunnerMode::kPdStand, 1, "test");
  control.RequireMocap(true);
  control.ObserveMocap(true);
  control.EnqueueLocalAction(RunnerAction::kSetReceiver);
  control.ProcessPending(false, false);
  control.EnqueueLocalAction(RunnerAction::kEnterMotion);
  ASSERT_EQ(control.ProcessPending(false, false)[0].result, RunnerActionResult::kApplied);
  const auto action = control.EncodeState();
  ASSERT_TRUE(control.ObserveMocap(false));
  auto safety = control.EncodeMocapSafety();
  ASSERT_EQ(safety.size(), 15U);
  EXPECT_EQ(safety[4], 1.);
  EXPECT_EQ(safety[5], 0.);
  EXPECT_EQ(safety[6], 1.);
  EXPECT_EQ(safety[7], 1.);
  EXPECT_EQ(control.EncodeState()[19], action[19]);
  control.ObserveMocap(true);
  EXPECT_EQ(control.EncodeMocapSafety()[7], 1.);
  ASSERT_TRUE(control.SetRuntimeMode(RunnerMode::kMotion));
  EXPECT_EQ(control.EncodeMocapSafety()[7], 0.);
  EXPECT_EQ(control.EncodeState().size(), kRunnerStateSize);
}
}  // namespace
