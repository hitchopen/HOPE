#include "a3_pingpong/pp_runner_control.hpp"

#include "gtest/gtest.h"

namespace {

using a3_pingpong::LocalRole;
using a3_pingpong::PpRunnerControl;
using a3_pingpong::RunnerAction;
using a3_pingpong::RunnerActionReason;
using a3_pingpong::RunnerActionResult;
using a3_pingpong::RunnerMode;

void SetRole(PpRunnerControl& control, RunnerAction action,
             LocalRole expected) {
  ASSERT_TRUE(control.Enqueue({1, action, true}));
  const auto decisions = control.ProcessPending(false, false);
  ASSERT_EQ(decisions.size(), 1U);
  ASSERT_EQ(decisions.front().result, RunnerActionResult::kApplied);
  ASSERT_EQ(control.local_role(), expected);
}

TEST(PpRunnerControl, PureServeRejectsReceiveAndPolicyActions) {
  PpRunnerControl control(RunnerMode::kPdStand, 123, "session", 16, true);
  SetRole(control, RunnerAction::kSetServer, LocalRole::kServer);
  std::uint64_t request_id = 2;
  for (auto action : {RunnerAction::kSetReceiver, RunnerAction::kEnterMotion,
                      RunnerAction::kEnterShadow}) {
    ASSERT_TRUE(control.Enqueue({request_id++, action, action != RunnerAction::kEnterShadow}));
    const auto decisions = control.ProcessPending(false, false);
    ASSERT_EQ(decisions.size(), 1U);
    EXPECT_EQ(decisions[0].reason, RunnerActionReason::kPureServeOnly);
    EXPECT_EQ(control.mode(), RunnerMode::kPdStand);
    EXPECT_EQ(control.local_role(), LocalRole::kServer);
  }
  for (auto mode : {RunnerMode::kMotion, RunnerMode::kShadow,
                    RunnerMode::kReferencePlayback}) {
    control.SetRuntimeMode(mode);
    EXPECT_EQ(control.mode(), RunnerMode::kPdStand);
  }
}

TEST(PpRunnerControl, PureServeCompletesInStandAndNormalServeStillEntersMotion) {
  for (bool pure : {false, true}) {
    PpRunnerControl control(RunnerMode::kPdStand, 123, "session", 16, pure);
    control.SetRuntimeMode(RunnerMode::kServe);
    control.CompleteServe();
    EXPECT_EQ(control.mode(), pure ? RunnerMode::kPdStand : RunnerMode::kMotion);
    control.SetRuntimeMode(RunnerMode::kPassive);
    EXPECT_EQ(control.mode(), RunnerMode::kPassive);
  }
}

TEST(PpRunnerControl, EncodesFrozenSchema2StateIncludingGripperCleanup) {
  PpRunnerControl control(RunnerMode::kPdStand, 1234,
                          "model21800_20260811T010203Z");
  control.ObserveExternalState(true, true, false, true, 5, 4, true);
  const auto state = control.EncodeState();

  ASSERT_EQ(state.size(), a3_pingpong::kRunnerStateSize);
  EXPECT_EQ(state[0], 2.0);
  EXPECT_EQ(state[1], 1234.0);
  EXPECT_EQ(state[3], 1.0);
  EXPECT_EQ(state[4], 1.0);
  EXPECT_EQ(state[5], 1.0);
  EXPECT_EQ(state[7], 0.0);
  EXPECT_EQ(state[12], 1.0);
  EXPECT_EQ(state[13], 5.0);
  EXPECT_EQ(state[14], 4.0);
  EXPECT_EQ(state[15], 1.0);
  EXPECT_GT(state[20], 0.0);
}

TEST(PpRunnerControl, RoleChangeIsStoppedOnlyAndIgnoresGripperTelemetry) {
  PpRunnerControl control(RunnerMode::kPdStand, 1, "session");
  SetRole(control, RunnerAction::kSetServer, LocalRole::kServer);
  EXPECT_EQ(control.role_epoch(), 1U);

  ASSERT_TRUE(control.Enqueue({2, RunnerAction::kSetServer, true}));
  auto decisions = control.ProcessPending(false, false);
  ASSERT_EQ(decisions.size(), 1U);
  EXPECT_EQ(decisions.front().result, RunnerActionResult::kAlreadySet);
  EXPECT_EQ(control.role_epoch(), 1U);

  ASSERT_TRUE(control.Enqueue({3, RunnerAction::kSetReceiver, true}));
  decisions =
      control.ProcessPending(false, false, true, 15, true, 4, true);
  ASSERT_EQ(decisions.size(), 1U);
  EXPECT_EQ(decisions.front().result, RunnerActionResult::kApplied);
  EXPECT_EQ(control.local_role(), LocalRole::kReceiver);

  control.SetRuntimeMode(RunnerMode::kMotion);
  ASSERT_TRUE(control.Enqueue({4, RunnerAction::kSetReceiver, true}));
  decisions = control.ProcessPending(false, false);
  EXPECT_EQ(decisions.front().result,
            RunnerActionResult::kRejectedWrongMode);
}

TEST(PpRunnerControl, ReceiverAloneMayUseExternalEnterMotion) {
  PpRunnerControl server(RunnerMode::kPdStand, 1, "server");
  SetRole(server, RunnerAction::kSetServer, LocalRole::kServer);
  ASSERT_TRUE(server.Enqueue({2, RunnerAction::kEnterMotion, true}));
  auto decisions = server.ProcessPending(false, false);
  ASSERT_EQ(decisions.size(), 1U);
  EXPECT_EQ(decisions.front().result,
            RunnerActionResult::kRejectedWrongRole);
  EXPECT_EQ(decisions.front().reason,
            RunnerActionReason::kReceiverRoleRequired);
  EXPECT_EQ(server.mode(), RunnerMode::kPdStand);

  PpRunnerControl receiver(RunnerMode::kPdStand, 1, "receiver");
  SetRole(receiver, RunnerAction::kSetReceiver, LocalRole::kReceiver);
  ASSERT_TRUE(receiver.Enqueue({2, RunnerAction::kEnterMotion, true}));
  decisions = receiver.ProcessPending(false, false);
  ASSERT_EQ(decisions.size(), 1U);
  EXPECT_EQ(decisions.front().result, RunnerActionResult::kApplied);
  EXPECT_EQ(receiver.mode(), RunnerMode::kMotion);
}

TEST(PpRunnerControl, ServerRepeatsServeFromReadyWithoutIntermediateStand) {
  PpRunnerControl control(RunnerMode::kPdStand, 1, "loop");
  SetRole(control, RunnerAction::kSetServer, LocalRole::kServer);
  for (int cycle = 0; cycle < 5; ++cycle) {
    control.EnqueueLocalAction(RunnerAction::kPrepareServe);
    const auto decisions = control.ProcessPending(false, false, true, cycle == 0 ? 0 : 12);
    ASSERT_EQ(decisions.size(), 1U);
    ASSERT_EQ(decisions[0].result, RunnerActionResult::kApplied);
    ASSERT_EQ(control.mode(), RunnerMode::kServe);
    // Start is ready for the command thread as soon as SERVE is visible,
    // before an action worker could call any controller method.
    EXPECT_TRUE(control.ConsumeServePrepare());
    EXPECT_FALSE(control.ConsumeServePrepare());
    control.CompleteServe();
    EXPECT_EQ(control.mode(), RunnerMode::kMotion);
  }
}

TEST(PpRunnerControl, PrepareServeStillRejectsTeleopPassiveAndReceiver) {
  PpRunnerControl control(RunnerMode::kPdStand, 1, "loop");
  SetRole(control, RunnerAction::kSetServer, LocalRole::kServer);
  for (auto mode : {RunnerMode::kTeleop, RunnerMode::kPassive, RunnerMode::kShadow}) {
    control.SetRuntimeMode(mode);
    control.EnqueueLocalAction(RunnerAction::kPrepareServe);
    EXPECT_EQ(control.ProcessPending(false, false, true, 12)[0].result,
              RunnerActionResult::kRejectedWrongMode);
    EXPECT_FALSE(control.ConsumeServePrepare());
  }
  control.SetRuntimeMode(RunnerMode::kMotion);
  control.EnqueueLocalAction(RunnerAction::kPrepareServe);
  EXPECT_EQ(control.ProcessPending(false, true, true, 5)[0].result,
            RunnerActionResult::kRejectedServeActive);
  EXPECT_FALSE(control.ConsumeServePrepare());
  PpRunnerControl receiver(RunnerMode::kPdStand, 2, "receiver");
  SetRole(receiver, RunnerAction::kSetReceiver, LocalRole::kReceiver);
  receiver.SetRuntimeMode(RunnerMode::kMotion);
  receiver.EnqueueLocalAction(RunnerAction::kPrepareServe);
  EXPECT_EQ(receiver.ProcessPending(false, false, true, 12)[0].result,
            RunnerActionResult::kRejectedWrongRole);
}

TEST(PpRunnerControl, ServerServeActionsFollowExactPhases) {
  PpRunnerControl control(RunnerMode::kPdStand, 1, "session");
  SetRole(control, RunnerAction::kSetServer, LocalRole::kServer);

  ASSERT_TRUE(control.EnqueueFlatRequest({2.0, 10.0, 7.0, 0.0}));
  auto decisions =
      control.ProcessPending(false, false, true, 0, true, 0, false);
  ASSERT_EQ(decisions.size(), 1U);
  EXPECT_EQ(decisions.front().result, RunnerActionResult::kApplied);
  EXPECT_TRUE(decisions.front().request_prepare_serve);
  EXPECT_EQ(control.mode(), RunnerMode::kServe);

  ASSERT_TRUE(control.EnqueueFlatRequest({2.0, 11.0, 8.0, 0.0}));
  decisions =
      control.ProcessPending(false, true, true, 3, true, 2, false);
  ASSERT_EQ(decisions.size(), 1U);
  EXPECT_EQ(decisions.front().result,
            RunnerActionResult::kAcceptedPending);
  EXPECT_TRUE(decisions.front().request_confirm_ball_loaded);

  // The active two-step flow enters WAIT_READY directly after PREPARE_SERVE.
  ASSERT_TRUE(control.EnqueueFlatRequest({2.0, 12.0, 9.0, 0.0}));
  decisions =
      control.ProcessPending(false, true, true, 5, true, -1, true);
  ASSERT_EQ(decisions.size(), 1U);
  EXPECT_EQ(decisions.front().result,
            RunnerActionResult::kAcceptedPending);
  EXPECT_TRUE(decisions.front().request_ready_to_serve);

  ASSERT_TRUE(control.EnqueueFlatRequest({2.0, 13.0, 12.0, 0.0}));
  decisions =
      control.ProcessPending(false, true, true, 5, true, -1, true);
  ASSERT_EQ(decisions.size(), 1U);
  EXPECT_EQ(decisions.front().result,
            RunnerActionResult::kAcceptedPending);
  EXPECT_TRUE(decisions.front().request_confirm_grip_secure);

  ASSERT_TRUE(control.EnqueueFlatRequest({2.0, 14.0, 9.0, 0.0}));
  decisions =
      control.ProcessPending(false, true, true, 5, true, 4, true);
  ASSERT_EQ(decisions.size(), 1U);
  EXPECT_EQ(decisions.front().result,
            RunnerActionResult::kAcceptedPending);
  EXPECT_TRUE(decisions.front().request_ready_to_serve);

  ASSERT_TRUE(control.Enqueue({15, RunnerAction::kConfirmBallLoaded, true}));
  decisions =
      control.ProcessPending(false, true, true, 5, true, 4, true);
  EXPECT_EQ(decisions.front().result, RunnerActionResult::kAlreadySet);
  EXPECT_EQ(decisions.front().reason,
            RunnerActionReason::kBallLoadMergedIntoPrepare);
  EXPECT_FALSE(decisions.front().request_confirm_ball_loaded);
}

TEST(PpRunnerControl, GripperCommandsIgnoreReceiptAndCleanupTelemetry) {
  PpRunnerControl control(RunnerMode::kPdStand, 1, "session");
  SetRole(control, RunnerAction::kSetServer, LocalRole::kServer);

  control.SetRuntimeMode(RunnerMode::kServe);
  ASSERT_TRUE(control.Enqueue(
      {10, RunnerAction::kConfirmLoadingZoneClear, true}));
  auto decisions =
      control.ProcessPending(false, true, true, 13, true, 2, false);
  ASSERT_EQ(decisions.size(), 1U);
  EXPECT_EQ(decisions.front().result, RunnerActionResult::kAlreadySet);
  EXPECT_EQ(decisions.front().reason,
            RunnerActionReason::kLoadingZoneClearRemoved);

  control.SetRuntimeMode(RunnerMode::kPdStand);
  ASSERT_TRUE(control.Enqueue({11, RunnerAction::kOpenGripper, true}));
  decisions =
      control.ProcessPending(false, false, true, 15, true, 4, true);
  ASSERT_EQ(decisions.size(), 1U);
  EXPECT_TRUE(decisions.front().request_open_gripper);

  ASSERT_TRUE(control.Enqueue({12, RunnerAction::kOpenGripper, true}));
  decisions =
      control.ProcessPending(false, false, true, 15, true, 2, true);
  EXPECT_EQ(decisions.front().result,
            RunnerActionResult::kAcceptedPending);
  EXPECT_TRUE(decisions.front().request_open_gripper);
}

TEST(PpRunnerControl, StandAbortsServeButEmergencyPassivePreemptsQueue) {
  PpRunnerControl serving(RunnerMode::kServe, 1, "serve");
  ASSERT_TRUE(serving.Enqueue(
      {20, RunnerAction::kEnterPdStand, true}));
  auto decisions =
      serving.ProcessPending(false, true, true, 3, true, 2, false);
  ASSERT_EQ(decisions.size(), 1U);
  EXPECT_TRUE(decisions.front().request_serve_abort);
  EXPECT_EQ(serving.mode(), RunnerMode::kServe);

  PpRunnerControl control(RunnerMode::kMotion, 1, "queue", 2);
  ASSERT_TRUE(control.Enqueue({21, RunnerAction::kSetServer, true}));
  ASSERT_TRUE(control.Enqueue({22, RunnerAction::kEmergencyPassive, true}));
  decisions = control.ProcessPending(false, false);
  ASSERT_EQ(decisions.size(), 2U);
  EXPECT_EQ(decisions[0].request.action,
            RunnerAction::kEmergencyPassive);
  EXPECT_EQ(control.mode(), RunnerMode::kPassive);
  EXPECT_EQ(decisions[1].request.action, RunnerAction::kSetServer);
  EXPECT_EQ(control.local_role(), LocalRole::kServer);
}

TEST(PpRunnerControl, FlatWireRejectsShadowWrongSchemaAndFractions) {
  PpRunnerControl control(RunnerMode::kPassive, 1, "session");
  EXPECT_FALSE(control.EnqueueFlatRequest({2.0, 50.0, 6.0, 0.0}));
  EXPECT_FALSE(control.EnqueueFlatRequest({1.0, 50.0, 1.0, 0.0}));
  EXPECT_FALSE(control.EnqueueFlatRequest({2.0, 50.5, 1.0, 0.0}));
  EXPECT_TRUE(control.ProcessPending(false, false).empty());
  const auto state = control.EncodeState();
  EXPECT_EQ(state[18], static_cast<double>(
                           static_cast<int>(
                               RunnerActionResult::kInvalidRequest)));
  EXPECT_EQ(state[19], static_cast<double>(
                           static_cast<int>(
                               RunnerActionReason::kMalformedRequest)));
}

TEST(PpRunnerControl, SessionFingerprintMatchesPythonFixture) {
  const auto fingerprint = a3_pingpong::RunnerSessionFingerprint(
      "model21800_20260811T010203Z");
  EXPECT_EQ(fingerprint, 47246369472706ULL);
  EXPECT_LT(fingerprint, a3_pingpong::kRunnerMaxExactFloatInteger);
}

}  // namespace

TEST(PpRunnerControl, KernelServeCompletesInStandAndKeepsServerRole) {
  PpRunnerControl control(RunnerMode::kPdStand, 1, "kernel");
  SetRole(control, RunnerAction::kSetServer, LocalRole::kServer);
  control.SetRuntimeMode(RunnerMode::kServe);
  control.CompleteServe(true);
  EXPECT_EQ(control.mode(), RunnerMode::kPdStand);
  EXPECT_EQ(control.local_role(), LocalRole::kServer);
  control.EnqueueLocalAction(RunnerAction::kPrepareServe);
  ASSERT_EQ(control.ProcessPending(false, false, true, 12)[0].result,
            RunnerActionResult::kApplied);
  EXPECT_EQ(control.mode(), RunnerMode::kServe);
}

TEST(PpRunnerControl, NormalAndPureServeKeepExistingCompletionModes) {
  for (bool pure : {false, true}) {
    PpRunnerControl control(RunnerMode::kServe, 1, "normal", 16, pure);
    control.CompleteServe(false);
    EXPECT_EQ(control.mode(), pure ? RunnerMode::kPdStand : RunnerMode::kMotion);
  }
}
