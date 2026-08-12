#include "a3_pingpong/pp_runner_control.hpp"

#include "gtest/gtest.h"

namespace {

using a3_pingpong::LocalRole;
using a3_pingpong::PpRunnerControl;
using a3_pingpong::RunnerAction;
using a3_pingpong::RunnerActionReason;
using a3_pingpong::RunnerActionRequest;
using a3_pingpong::RunnerActionResult;
using a3_pingpong::RunnerMode;

TEST(PpRunnerControl, StartsUnassignedAndEncodesFrozenStateSchema) {
  PpRunnerControl control(RunnerMode::kPassive, 1234, "model21800_20260811T010203Z");
  control.ObserveExternalState(true, true, false, false, -1);
  const auto state = control.EncodeState();

  ASSERT_EQ(state.size(), a3_pingpong::kRunnerStateSize);
  EXPECT_EQ(state[0], 1.0);
  EXPECT_EQ(state[1], 1234.0);
  EXPECT_EQ(state[3], static_cast<double>(static_cast<int>(RunnerMode::kPassive)));
  EXPECT_EQ(state[4], 1.0);
  EXPECT_EQ(state[5], 1.0);
  EXPECT_EQ(state[7], static_cast<double>(static_cast<int>(LocalRole::kUnassigned)));
  EXPECT_EQ(state[8], 0.0);
  EXPECT_EQ(state[9], 1.0);
  EXPECT_EQ(state[12], 0.0);
  EXPECT_EQ(state[13], -1.0);
  EXPECT_GT(state[18], 0.0);
}

TEST(PpRunnerControl, RoleChangesAreIdempotentAndNeverChangeMode) {
  PpRunnerControl control(RunnerMode::kPdStand, 1, "session");
  ASSERT_TRUE(control.Enqueue({10, RunnerAction::kSetReceiver, true}));
  auto decisions = control.ProcessPending(false, false);
  ASSERT_EQ(decisions.size(), 1U);
  EXPECT_EQ(decisions[0].result, RunnerActionResult::kApplied);
  EXPECT_EQ(decisions[0].reason, RunnerActionReason::kRoleChanged);
  EXPECT_EQ(control.local_role(), LocalRole::kReceiver);
  EXPECT_EQ(control.role_epoch(), 1U);
  EXPECT_EQ(control.mode(), RunnerMode::kPdStand);

  ASSERT_TRUE(control.Enqueue({11, RunnerAction::kSetReceiver, true}));
  decisions = control.ProcessPending(false, false);
  ASSERT_EQ(decisions.size(), 1U);
  EXPECT_EQ(decisions[0].result, RunnerActionResult::kAlreadySet);
  EXPECT_EQ(decisions[0].reason, RunnerActionReason::kRoleUnchanged);
  EXPECT_EQ(control.role_epoch(), 1U);
  EXPECT_EQ(control.mode(), RunnerMode::kPdStand);
}

TEST(PpRunnerControl, RoleChangeRejectsMotionServeShadowAndFault) {
  for (const RunnerMode mode : {RunnerMode::kMotion, RunnerMode::kServe,
                                RunnerMode::kShadow,
                                RunnerMode::kReferencePlayback}) {
    PpRunnerControl control(mode, 1, "session");
    ASSERT_TRUE(control.Enqueue({20, RunnerAction::kSetServer, true}));
    const auto decisions = control.ProcessPending(false, mode == RunnerMode::kServe);
    ASSERT_EQ(decisions.size(), 1U);
    EXPECT_EQ(decisions[0].result, RunnerActionResult::kRejectedWrongMode);
    EXPECT_EQ(control.local_role(), LocalRole::kUnassigned);
  }

  PpRunnerControl faulted(RunnerMode::kPassive, 1, "session");
  ASSERT_TRUE(faulted.Enqueue({21, RunnerAction::kSetServer, true}));
  const auto decisions = faulted.ProcessPending(true, false);
  ASSERT_EQ(decisions.size(), 1U);
  EXPECT_EQ(decisions[0].result, RunnerActionResult::kRejectedRunnerFault);
  EXPECT_EQ(faulted.local_role(), LocalRole::kUnassigned);

  PpRunnerControl emergency_serve(RunnerMode::kPassive, 1, "session");
  ASSERT_TRUE(emergency_serve.Enqueue({22, RunnerAction::kSetServer, true}));
  const auto serve_decisions = emergency_serve.ProcessPending(false, true);
  ASSERT_EQ(serve_decisions.size(), 1U);
  EXPECT_EQ(serve_decisions[0].result,
            RunnerActionResult::kRejectedWrongMode);
  EXPECT_EQ(emergency_serve.local_role(), LocalRole::kUnassigned);
}

TEST(PpRunnerControl, ModeActionsMatchKeyboardSemantics) {
  PpRunnerControl control(RunnerMode::kPassive, 1, "session");
  ASSERT_TRUE(control.EnqueueLocalAction(RunnerAction::kEnterPdStand));
  auto decisions = control.ProcessPending(false, false);
  ASSERT_EQ(decisions.size(), 1U);
  EXPECT_TRUE(decisions[0].hold_reference);
  EXPECT_EQ(control.mode(), RunnerMode::kPdStand);

  ASSERT_TRUE(control.Enqueue({30, RunnerAction::kEnterMotion, true}));
  decisions = control.ProcessPending(false, false);
  EXPECT_EQ(decisions[0].result, RunnerActionResult::kApplied);
  EXPECT_EQ(control.mode(), RunnerMode::kMotion);

  ASSERT_TRUE(control.Enqueue({31, RunnerAction::kEmergencyPassive, true}));
  decisions = control.ProcessPending(false, false);
  EXPECT_TRUE(decisions[0].hold_reference);
  EXPECT_EQ(control.mode(), RunnerMode::kPassive);
}

TEST(PpRunnerControl, StandRequestsPhaseAwareAbortWhileServeOwnsCommand) {
  PpRunnerControl control(RunnerMode::kServe, 1, "session");
  ASSERT_TRUE(control.Enqueue({40, RunnerAction::kEnterPdStand, true}));
  auto decisions = control.ProcessPending(false, true);
  ASSERT_EQ(decisions.size(), 1U);
  EXPECT_EQ(decisions[0].result, RunnerActionResult::kAcceptedPending);
  EXPECT_TRUE(decisions[0].request_serve_abort);
  EXPECT_EQ(control.mode(), RunnerMode::kServe);

  ASSERT_TRUE(control.Enqueue({41, RunnerAction::kEnterMotion, true}));
  decisions = control.ProcessPending(false, true);
  EXPECT_EQ(decisions[0].result, RunnerActionResult::kRejectedServeActive);
  EXPECT_EQ(control.mode(), RunnerMode::kServe);

  ASSERT_TRUE(control.Enqueue({42, RunnerAction::kEmergencyPassive, true}));
  decisions = control.ProcessPending(false, true);
  EXPECT_EQ(decisions[0].result, RunnerActionResult::kApplied);
  EXPECT_EQ(control.mode(), RunnerMode::kPassive);
}

TEST(PpRunnerControl, FlatWireRejectsMalformedOrNonServiceActions) {
  PpRunnerControl control(RunnerMode::kPassive, 1, "session");
  EXPECT_FALSE(control.EnqueueFlatRequest({1.0, 50.0, 6.0, 0.0}));
  EXPECT_FALSE(control.EnqueueFlatRequest({1.0, 50.5, 1.0, 0.0}));
  EXPECT_FALSE(control.EnqueueFlatRequest({2.0, 50.0, 1.0, 0.0}));
  EXPECT_TRUE(control.ProcessPending(false, false).empty());
  const auto state = control.EncodeState();
  EXPECT_EQ(state[16], static_cast<double>(
                           static_cast<int>(RunnerActionResult::kInvalidRequest)));
  EXPECT_EQ(state[17], static_cast<double>(
                           static_cast<int>(RunnerActionReason::kMalformedRequest)));
}

TEST(PpRunnerControl, ReadyToServeUsesTheExistingTwoStageServeContract) {
  PpRunnerControl control(RunnerMode::kPdStand, 1, "session");
  ASSERT_TRUE(control.EnqueueFlatRequest({1.0, 80.0, 7.0, 0.0}));
  auto decisions = control.ProcessPending(false, false, true, 0, true);
  ASSERT_EQ(decisions.size(), 1U);
  EXPECT_EQ(decisions[0].result, RunnerActionResult::kApplied);
  EXPECT_EQ(decisions[0].reason,
            RunnerActionReason::kServeStartRequested);
  EXPECT_TRUE(decisions[0].request_serve_start);
  EXPECT_FALSE(decisions[0].request_serve_confirm);
  EXPECT_EQ(control.mode(), RunnerMode::kServe);

  ASSERT_TRUE(control.EnqueueFlatRequest({1.0, 81.0, 8.0, 0.0}));
  decisions = control.ProcessPending(false, true, true, 3, true);
  ASSERT_EQ(decisions.size(), 1U);
  EXPECT_EQ(decisions[0].result, RunnerActionResult::kAcceptedPending);
  EXPECT_EQ(decisions[0].reason,
            RunnerActionReason::kBallOnPalmConfirmRequested);
  EXPECT_FALSE(decisions[0].request_serve_start);
  EXPECT_TRUE(decisions[0].request_serve_confirm);
}

TEST(PpRunnerControl, ServeActionsFailClosedOnCapabilityPhaseFaultAndGains) {
  PpRunnerControl unavailable(RunnerMode::kPdStand, 1, "session");
  ASSERT_TRUE(unavailable.Enqueue({90, RunnerAction::kReadyToServe, true}));
  auto decisions = unavailable.ProcessPending(false, false, false, -1, true);
  EXPECT_EQ(decisions[0].result,
            RunnerActionResult::kRejectedServeUnavailable);
  EXPECT_EQ(unavailable.mode(), RunnerMode::kPdStand);

  PpRunnerControl scaled(RunnerMode::kPdStand, 1, "session");
  ASSERT_TRUE(scaled.Enqueue({91, RunnerAction::kReadyToServe, true}));
  decisions = scaled.ProcessPending(false, false, true, 0, false);
  EXPECT_EQ(decisions[0].result, RunnerActionResult::kRejectedGainScale);
  EXPECT_EQ(scaled.mode(), RunnerMode::kPdStand);

  PpRunnerControl wrong_phase(RunnerMode::kServe, 1, "session");
  ASSERT_TRUE(wrong_phase.Enqueue({92, RunnerAction::kServe, true}));
  decisions = wrong_phase.ProcessPending(false, true, true, 2, true);
  EXPECT_EQ(decisions[0].result,
            RunnerActionResult::kRejectedServeNotReady);
  EXPECT_FALSE(decisions[0].request_serve_confirm);

  PpRunnerControl faulted(RunnerMode::kServe, 1, "session");
  ASSERT_TRUE(faulted.Enqueue({93, RunnerAction::kServe, true}));
  decisions = faulted.ProcessPending(false, false, true, 8, true);
  EXPECT_EQ(decisions[0].reason, RunnerActionReason::kServeFaultLatched);
}

TEST(PpRunnerControl, QueueBoundRejectsNormalRequestButKeepsEmergencyPassive) {
  PpRunnerControl control(RunnerMode::kMotion, 1, "session", 1);
  ASSERT_TRUE(control.Enqueue({60, RunnerAction::kSetServer, true}));
  EXPECT_FALSE(control.Enqueue({61, RunnerAction::kSetReceiver, true}));
  EXPECT_TRUE(control.Enqueue({62, RunnerAction::kEmergencyPassive, true}));
  const auto decisions = control.ProcessPending(false, false);
  ASSERT_EQ(decisions.size(), 1U);
  EXPECT_EQ(decisions[0].request.request_id, 62U);
  EXPECT_EQ(control.mode(), RunnerMode::kPassive);
}

TEST(PpRunnerControl, EmergencyPassiveRunsAheadOfQueuedNormalActions) {
  PpRunnerControl control(RunnerMode::kMotion, 1, "session", 4);
  ASSERT_TRUE(control.Enqueue({70, RunnerAction::kSetServer, true}));
  ASSERT_TRUE(control.Enqueue({71, RunnerAction::kEmergencyPassive, true}));
  const auto decisions = control.ProcessPending(false, false);
  ASSERT_EQ(decisions.size(), 2U);
  EXPECT_EQ(decisions[0].request.request_id, 71U);
  EXPECT_EQ(decisions[0].request.action, RunnerAction::kEmergencyPassive);
  EXPECT_EQ(control.mode(), RunnerMode::kPassive);
  EXPECT_EQ(control.local_role(), LocalRole::kServer);
}

TEST(PpRunnerControl, SessionFingerprintIsStableAndBounded) {
  const auto a = a3_pingpong::RunnerSessionFingerprint(
      "model21800_20260811T010203Z");
  const auto b = a3_pingpong::RunnerSessionFingerprint(
      "model21800_20260811T010203Z");
  const auto c = a3_pingpong::RunnerSessionFingerprint(
      "model21800_20260811T010204Z");
  EXPECT_EQ(a, b);
  EXPECT_EQ(a, 47246369472706ULL);  // Cross-language wire fixture.
  EXPECT_NE(a, c);
  EXPECT_GT(a, 0U);
  EXPECT_LT(a, a3_pingpong::kRunnerMaxExactFloatInteger);
}

}  // namespace
