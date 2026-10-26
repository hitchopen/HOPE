// Executable action/recovery regressions plus source integration contracts.
// Source checks do not replace a full ONNX/AimRT build or robot validation.
#include "a3_pingpong/pp_runner_control.hpp"
#include "a3_pingpong/pp_serve_recovery.hpp"

#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <gtest/gtest.h>

namespace {
using a3_pingpong::LocalRole;
using a3_pingpong::PpRunnerControl;
using a3_pingpong::RunnerAction;
using a3_pingpong::RunnerActionReason;
using a3_pingpong::RunnerActionResult;
using a3_pingpong::RunnerMode;

std::string Source(const char* relative) {
  // CMake supplies absolute TEST_SRCS paths: source-contract checks resolve
  // against this checkout, independently of run_tests' working directory.
  const auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
  const auto path = root / relative;
  std::ifstream input(path);
  if (!input) throw std::runtime_error("integration source is unavailable: " + path.string());
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}
std::string Between(const std::string& text, const char* first, const char* last) {
  const auto begin = text.find(first);
  if (begin == std::string::npos) throw std::runtime_error("source start anchor missing");
  const auto end = text.find(last, begin + std::string(first).size());
  if (end == std::string::npos) throw std::runtime_error("source end anchor missing");
  return text.substr(begin, end - begin);
}

TEST(PpOperability, StandRoleAndReceiveWorkWithoutPermitOrManagedSession) {
  PpRunnerControl control(RunnerMode::kPassive, 1, "");
  ASSERT_TRUE(control.EnqueueLocalAction(RunnerAction::kEnterPdStand));
  auto decisions = control.ProcessPending(false, false);
  ASSERT_EQ(decisions.size(), 1U);
  EXPECT_EQ(decisions[0].result, RunnerActionResult::kApplied);
  EXPECT_EQ(control.mode(), RunnerMode::kPdStand);
  ASSERT_TRUE(control.EnqueueLocalAction(RunnerAction::kSetReceiver));
  decisions = control.ProcessPending(false, false);
  EXPECT_EQ(decisions[0].result, RunnerActionResult::kApplied);
  EXPECT_EQ(control.local_role(), LocalRole::kReceiver);
  ASSERT_TRUE(control.EnqueueLocalAction(RunnerAction::kEnterMotion));
  decisions = control.ProcessPending(false, false);
  EXPECT_EQ(decisions[0].result, RunnerActionResult::kApplied);
  EXPECT_EQ(control.mode(), RunnerMode::kMotion);
}

TEST(PpOperability, ServeCyclesAndNormalCompletionNeedNoContinuationAction) {
  PpRunnerControl control(RunnerMode::kPdStand, 1, "");
  ASSERT_TRUE(control.EnqueueLocalAction(RunnerAction::kSetServer));
  ASSERT_EQ(control.ProcessPending(false, false)[0].result, RunnerActionResult::kApplied);
  ASSERT_TRUE(control.EnqueueLocalAction(RunnerAction::kOpenGripper));
  auto decisions = control.ProcessPending(false, false, true, 0, true, -1, true);
  ASSERT_TRUE(decisions[0].request_open_gripper);
  for (int cycle = 0; cycle < 20; ++cycle) {
    ASSERT_TRUE(control.EnqueueLocalAction(RunnerAction::kPrepareServe));
    decisions = control.ProcessPending(false, false, true, cycle == 0 ? 0 : 12,
                                       true, -1, true);
    ASSERT_EQ(decisions[0].result, RunnerActionResult::kApplied);
    ASSERT_EQ(control.mode(), RunnerMode::kServe);
    ASSERT_TRUE(control.ConsumeServePrepare());
    ASSERT_TRUE(control.EnqueueLocalAction(RunnerAction::kReadyToServe));
    decisions = control.ProcessPending(false, true, true, 5, true, -1, true);
    ASSERT_EQ(decisions[0].result, RunnerActionResult::kAcceptedPending);
    ASSERT_TRUE(decisions[0].request_ready_to_serve);
    ASSERT_TRUE(control.CompleteServe());
    EXPECT_EQ(control.mode(), RunnerMode::kMotion);
  }
}

TEST(PpOperability, PureAndKernelServeStillCompleteInStand) {
  for (bool pure : {false, true}) {
    for (bool kernel : {false, true}) {
      PpRunnerControl control(RunnerMode::kServe, 1, "", 16, pure);
      ASSERT_TRUE(control.CompleteServe(kernel));
      EXPECT_EQ(control.mode(), pure || kernel ? RunnerMode::kPdStand : RunnerMode::kMotion);
    }
  }
}

TEST(PpOperability, TerminalFaultAndAbortRecoverWithCurrentValidCommand) {
  for (int terminal : {15, 16}) {
    PpRunnerControl control(RunnerMode::kServe, 1, "");
    ASSERT_FALSE(control.RecoverTerminalServeToStand(terminal, false));
    EXPECT_EQ(control.mode(), RunnerMode::kServe);
    ASSERT_TRUE(control.EnqueueLocalAction(RunnerAction::kEnterPdStand));
    const auto decisions = control.ProcessPending(false, false, true, terminal);
    ASSERT_EQ(decisions[0].result, RunnerActionResult::kAcceptedPending);
    EXPECT_EQ(decisions[0].reason, RunnerActionReason::kServeRecoveryPending);
    EXPECT_EQ(control.mode(), RunnerMode::kServe);
    ASSERT_TRUE(control.RecoverTerminalServeToStand(terminal, true));
    EXPECT_EQ(control.mode(), RunnerMode::kPdStand);
    EXPECT_FALSE(control.RecoverTerminalServeToStand(terminal, true));
  }
  PpRunnerControl active(RunnerMode::kServe, 1, "");
  EXPECT_FALSE(active.RecoverTerminalServeToStand(5, true));
  EXPECT_EQ(active.mode(), RunnerMode::kServe);
}

TEST(PpOperability, EmergencyPassiveCannotBeUndoneByLateServeResult) {
  for (int terminal : {12, 15, 16}) {
    PpRunnerControl control(RunnerMode::kServe, 1, "");
    ASSERT_TRUE(control.EnqueueLocalAction(RunnerAction::kEmergencyPassive));
    ASSERT_EQ(control.ProcessPending(false, false)[0].result, RunnerActionResult::kApplied);
    EXPECT_FALSE(control.CompleteServe());
    EXPECT_FALSE(control.RecoverTerminalServeToStand(terminal, true));
    EXPECT_EQ(control.mode(), RunnerMode::kPassive);
  }
}

TEST(PpOperability, ExistingCommandFaultAndRoleGuardsRemainEffective) {
  PpRunnerControl faulted(RunnerMode::kPassive, 1, "");
  ASSERT_TRUE(faulted.EnqueueLocalAction(RunnerAction::kEnterPdStand));
  EXPECT_EQ(faulted.ProcessPending(true, false)[0].result,
            RunnerActionResult::kApplied);
  EXPECT_EQ(faulted.mode(), RunnerMode::kPdStand);
  ASSERT_TRUE(faulted.EnqueueLocalAction(RunnerAction::kEnterMotion));
  EXPECT_EQ(faulted.ProcessPending(true, false)[0].result,
            RunnerActionResult::kRejectedRunnerFault);
  PpRunnerControl server(RunnerMode::kPdStand, 1, "");
  ASSERT_TRUE(server.EnqueueLocalAction(RunnerAction::kSetServer));
  ASSERT_EQ(server.ProcessPending(false, false)[0].result, RunnerActionResult::kApplied);
  ASSERT_TRUE(server.EnqueueLocalAction(RunnerAction::kEnterMotion));
  EXPECT_EQ(server.ProcessPending(false, false)[0].result,
            RunnerActionResult::kRejectedWrongRole);
  server.SetRuntimeMode(RunnerMode::kServe);
  ASSERT_TRUE(server.EnqueueLocalAction(RunnerAction::kEnterPdStand));
  EXPECT_EQ(server.ProcessPending(true, false, true, 16)[0].result,
            RunnerActionResult::kAcceptedPending);
}

TEST(PpOperability, TerminalRecoveryLoggingIsEdgeBounded) {
  a3_pingpong::ServeTerminalRecoveryEvents events;
  EXPECT_TRUE(events.Observe(16, false));
  for (int tick = 0; tick < 10000; ++tick) EXPECT_FALSE(events.Observe(16, false));
  EXPECT_TRUE(events.Observe(16, true));
  for (int tick = 0; tick < 10000; ++tick) EXPECT_FALSE(events.Observe(16, true));
  events.Reset();
  EXPECT_TRUE(events.Observe(16, true));
  EXPECT_TRUE(events.Observe(15, true));
}

TEST(PpOperability, NativeStartupAndActionsHaveNoNewTelemetryAdmission) {
  const auto main = Source("src/a3_deploy/a3_pingpong_main.cpp");
  const auto control = Source("include/a3_pingpong/pp_runner_control.hpp");
  const auto serve = Source("include/a3_pingpong/pp_serve_controller.hpp");
  for (const auto* source : {&main, &control, &serve}) {
    for (const char* removed : {"PpElinkPermit", "SetAdmissionCheck", "NewActionsAllowed",
         "SERVE_CONTINUATION_REQUIRED", "ServeContinuationReadiness", "elink_challenge"})
      EXPECT_EQ(source->find(removed), std::string::npos) << removed;
  }
  EXPECT_NE(main.find("warmup_sec > 0 ? Mode::kPdStand : default_mode"), std::string::npos);
  EXPECT_NE(main.find("runner_control.SetRuntimeMode(target_mode)"), std::string::npos);
  EXPECT_NE(main.find("runner_control.SetRuntimeMode(Mode::kReferencePlayback)"), std::string::npos);
}

TEST(PpOperability, PolicyRetainsSameFlightStreamingAndPeriodicRearm) {
  const auto policy = Source("include/a3_pingpong/pp_policy.hpp");
  EXPECT_EQ(policy.find("NewActionsAllowed"), std::string::npos);
  EXPECT_EQ(policy.find("SetAdmissionCheck"), std::string::npos);
  const auto rapid = Between(policy, "bool PlannerRapidPreemptStep_", "// One driver tick owns the complete edge");
  EXPECT_NE(rapid.find("if (cfg_.stream_target) StreamTargetStep_(tick_idx)"), std::string::npos);
  const auto rest = Between(policy, "// Auto re-arm after the rest", "// MIN-side OBS clamp");
  EXPECT_NE(rest.find("tick_idx >= rest_rearm_tick_"), std::string::npos);
  EXPECT_NE(rest.find("level_.store(1)"), std::string::npos);
}

TEST(PpOperability, TerminalRecoveryFollowsValidationAndKeepsExistingBlend) {
  const auto main = Source("src/a3_deploy/a3_pingpong_main.cpp");
  const auto serve = Between(main, "const bool serve_command_valid", "} else {  // SHADOW or MOTION");
  const auto validation = serve.find("if (!serve_command_valid) return false");
  const auto recovery = serve.find("RecoverTerminalServeToStand");
  ASSERT_NE(validation, std::string::npos);
  ASSERT_NE(recovery, std::string::npos);
  EXPECT_LT(validation, recovery);
  EXPECT_EQ(serve.find("SetRuntimeMode(Mode::kPdStand)"), std::string::npos);
  EXPECT_NE(serve.find("static_cast<int>(terminal), serve_command_valid"), std::string::npos);
  const auto event = serve.find("serve_terminal_events.Observe");
  const auto diagnostic = serve.find("terminal recovery not applied");
  ASSERT_NE(event, std::string::npos);
  ASSERT_NE(diagnostic, std::string::npos);
  EXPECT_LT(event, diagnostic);
  EXPECT_NE(serve.find("RunnerModeName(runner_control.mode())"), std::string::npos);
  const auto blend = Between(main, "if (publish && (m == Mode::kPdStand", "// Final command safety");
  EXPECT_NE(blend.find("source = last_delivered_command"), std::string::npos);
  EXPECT_NE(blend.find("mode_transition.Begin(source, cmd, duration)"), std::string::npos);
}
}  // namespace
