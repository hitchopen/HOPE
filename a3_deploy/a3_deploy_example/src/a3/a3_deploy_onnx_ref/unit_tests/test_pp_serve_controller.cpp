#include "a3_pingpong/pp_serve_controller.hpp"
#include "robot_io/a3_layout_extra.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

using a3_pingpong::PpGripperCommand;
using a3_pingpong::PpGripperReceipt;
using a3_pingpong::PpGripperWorker;
using a3_pingpong::PpGripperWorkerTransport;
using a3_pingpong::PpServe025FullbodyTimeline;
using a3_pingpong::PpServeController;
using a3_pingpong::ServeControllerState;
using a3_pingpong::ServeGripperState;

std::filesystem::path A3Root() {
  return std::filesystem::path(__FILE__)
      .parent_path()
      .parent_path()
      .parent_path()
      .parent_path()
      .parent_path();
}

std::filesystem::path TimelinePath() {
  return A3Root() / "assets/a3_runtime/serve/motions" /
         "a3p_op3_serve025_photo_right30_advance20_v12.csv";
}

PpServe025FullbodyTimeline LoadTimeline() {
  PpServe025FullbodyTimeline timeline;
  std::string error;
  EXPECT_TRUE(timeline.LoadCsv(TimelinePath().string(), error)) << error;
  return timeline;
}

Eigen::VectorXd PositiveGains(double value) {
  return Eigen::VectorXd::Constant(robot_io::kA3Dof, value);
}

robot_io::RobotState StateAt(const Eigen::VectorXd& q) {
  robot_io::RobotState state;
  state.q = q;
  state.dq = Eigen::VectorXd::Zero(robot_io::kA3Dof);
  state.tau_est = Eigen::VectorXd::Zero(robot_io::kA3Dof);
  state.imu_quat_wxyz = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
  state.imu_gyro = Eigen::Vector3d::Zero();
  state.sync_complete = true;
  state.sync_aligned = true;
  return state;
}

void ExpectPdCommand(const robot_io::RobotCommand& actual,
                     const Eigen::VectorXd& q_des,
                     const Eigen::VectorXd& dq_des,
                     const Eigen::VectorXd& kp,
                     const Eigen::VectorXd& kd,
                     double tolerance = 0.0) {
  EXPECT_TRUE(actual.q_des.isApprox(q_des, tolerance));
  EXPECT_TRUE(actual.dq_des.isApprox(dq_des, tolerance));
  EXPECT_TRUE(actual.tau_ff.isZero(0.0));
  EXPECT_TRUE(actual.kp.isApprox(kp, 0.0));
  EXPECT_TRUE(actual.kd.isApprox(kd, 0.0));
}

void AdvanceToReady(PpServeController& controller,
                    const robot_io::RobotState& state,
                    robot_io::RobotCommand& command) {
  controller.Start();
  for (std::size_t tick = 0;
       tick < a3_pingpong::kServe025FullbodyTransitionTicks; ++tick) {
    ASSERT_TRUE(controller.ComputeCommand(tick, state, command));
  }
  ASSERT_EQ(controller.state(), ServeControllerState::kWaitReadyToServe);
}

PpGripperReceipt SuccessfulReceipt(PpGripperCommand command) {
  PpGripperReceipt receipt;
  receipt.ok = true;
  receipt.command = command;
  receipt.state = command == PpGripperCommand::kOpen
                      ? "OPEN"
                      : command == PpGripperCommand::kGrab ? "GRABBED"
                                                           : "RELEASED";
  receipt.first_publish_monotonic_ns = 100;
  receipt.published_before_ack = 5;
  receipt.planned_publish_count = 5;
  receipt.matched_subscribers = 1;
  return receipt;
}

TEST(PpServeController, Full31TimelineHasFrozenExecutionShape) {
  const auto timeline = LoadTimeline();
  EXPECT_EQ(timeline.size(), a3_pingpong::kServe025FullbodyFrames);
  EXPECT_EQ(timeline.ready().frame_index, a3_pingpong::kServe025SpaceFrame);
  EXPECT_EQ(timeline.complete().frame_index,
            a3_pingpong::kServe025CompleteFrame);
  EXPECT_EQ(a3_pingpong::kServePolicyHz, 100.0);
  EXPECT_EQ(a3_pingpong::kServe025FullbodyCommandTicks, 468U);
  EXPECT_EQ(a3_pingpong::kServe025FullbodyTransitionTicks, 501U);
  EXPECT_DOUBLE_EQ(timeline.ready().time_s, 0.0);
  EXPECT_DOUBLE_EQ(timeline.complete().time_s, 4.67);
}

TEST(PpServeController, RaiseHandTransitionsFromStandToCsvFrameZero) {
  auto timeline = LoadTimeline();
  const Eigen::VectorXd ready_q = timeline.ready().q_sdk;
  const Eigen::VectorXd ready_dq = timeline.ready().qd_sdk;
  const Eigen::VectorXd stand = timeline.complete().q_sdk;
  const Eigen::VectorXd kp = PositiveGains(80.0);
  const Eigen::VectorXd kd = PositiveGains(2.0);
  PpServeController controller(
      std::move(timeline), stand, kp, kd, nullptr);
  const auto state = StateAt(stand);
  robot_io::RobotCommand command;

  controller.Start();
  ASSERT_TRUE(controller.ComputeCommand(0, state, command));
  ExpectPdCommand(command, stand, Eigen::VectorXd::Zero(robot_io::kA3Dof),
                  kp, kd);
  for (std::size_t tick = 1;
       tick < a3_pingpong::kServe025FullbodyTransitionTicks; ++tick) {
    ASSERT_TRUE(controller.ComputeCommand(tick, state, command));
  }
  EXPECT_EQ(controller.state(), ServeControllerState::kWaitReadyToServe);
  ExpectPdCommand(command, ready_q,
                  Eigen::VectorXd::Zero(robot_io::kA3Dof), kp, kd, 1.0e-14);

  ASSERT_TRUE(controller.ComputeCommand(999, state, command));
  ExpectPdCommand(command, ready_q, ready_dq, kp, kd);
}

TEST(PpServeController, ReadyConsumesSelectedForwardHitFramesExactly) {
  auto timeline = LoadTimeline();
  const auto expected = timeline;
  const Eigen::VectorXd stand = timeline.complete().q_sdk;
  const Eigen::VectorXd kp = PositiveGains(90.0);
  const Eigen::VectorXd kd = PositiveGains(3.0);
  PpServeController controller(
      std::move(timeline), stand, kp, kd, nullptr);
  const auto state = StateAt(stand);
  robot_io::RobotCommand command;
  AdvanceToReady(controller, state, command);

  controller.TriggerReadyToServe();
  for (std::size_t tick = 0;
       tick < a3_pingpong::kServe025FullbodyCommandTicks; ++tick) {
    ASSERT_TRUE(controller.ComputeCommand(1000 + tick, state, command));
    const auto& value = expected.At(tick);
    ExpectPdCommand(command, value.q_sdk, value.qd_sdk, kp, kd);
  }
  EXPECT_EQ(controller.state(), ServeControllerState::kComplete);
  EXPECT_FALSE(controller.active());
}

TEST(PpServeController, LoadingTrajectoryVelocityMatchesPositionAndHasQuietEndpoints) {
  auto timeline = LoadTimeline();
  const auto stand = timeline.complete().q_sdk;
  const auto ready = timeline.ready().q_sdk;
  PpServeController controller(std::move(timeline), stand,
      PositiveGains(90), PositiveGains(3), nullptr);
  const auto state = StateAt(stand);
  robot_io::RobotCommand command, previous;
  controller.Start();
  for (std::size_t tick = 0; tick < a3_pingpong::kServe025FullbodyTransitionTicks; ++tick) {
    ASSERT_TRUE(controller.ComputeCommand(tick, state, command));
    ASSERT_TRUE(command.q_des.allFinite());
    EXPECT_LT(command.dq_des.cwiseAbs().maxCoeff(), 1.5);
    if (tick > 0) {
      const Eigen::VectorXd secant = (command.q_des - previous.q_des) / .01;
      EXPECT_LT((secant - .5*(command.dq_des + previous.dq_des)).cwiseAbs().maxCoeff(), 1e-4);
      EXPECT_LT((command.q_des - previous.q_des).cwiseAbs().maxCoeff(), .015);
    } else {
      EXPECT_TRUE(command.q_des.isApprox(stand, 1e-14));
      EXPECT_TRUE(command.dq_des.isZero(1e-14));
    }
    previous = command;
  }
  EXPECT_TRUE(command.q_des.isApprox(ready, 1e-14));
  EXPECT_TRUE(command.dq_des.isZero(1e-14));
}

TEST(PpServeController, SelectedCsvCanOnlyChangeSymmetricLateralStand) {
  const auto stand = LoadTimeline().complete().q_sdk;
  auto wide = stand;
  wide[20] += .03; wide[24] -= .03;
  wide[26] -= .03; wide[30] += .03;
  EXPECT_NEAR(a3_pingpong::ServeStandLateralOffset(stand, wide), .03, 1e-12);
  EXPECT_DOUBLE_EQ(a3_pingpong::ServeStandLateralOffset(stand, stand), 0.);
  auto bad = wide; bad[12] += .001;
  EXPECT_THROW(a3_pingpong::ServeStandLateralOffset(stand, bad), std::invalid_argument);
  bad = wide; bad[26] += .001;
  EXPECT_THROW(a3_pingpong::ServeStandLateralOffset(stand, bad), std::invalid_argument);
  bad = stand; bad[20] += .07;
  EXPECT_THROW(a3_pingpong::ServeStandLateralOffset(stand, bad), std::invalid_argument);
  bad = stand; bad[20] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(a3_pingpong::ServeStandLateralOffset(stand, bad), std::invalid_argument);
}

TEST(PpServeController, StandStartsLoadingWithoutAnAdditionalSettlementGate) {
  auto timeline = LoadTimeline();
  const auto stand = timeline.complete().q_sdk;
  const auto kp = PositiveGains(90), kd = PositiveGains(3);
  PpServeController controller(std::move(timeline), stand, kp, kd, nullptr);
  auto state = StateAt(stand);
  robot_io::RobotCommand source, command;
  source.q_des = stand;
  source.dq_des = Eigen::VectorXd::Zero(31);
  source.tau_ff = Eigen::VectorXd::Zero(31);
  source.kp = kp;
  source.kd = kd;
  controller.SetEntryCommand(source);
  state.imu_gyro[1] = .25;
  controller.Start();
  state.q[22] += .15;
  ASSERT_TRUE(controller.ComputeCommand(0, state, command));
  EXPECT_EQ(controller.state(), ServeControllerState::kTransitionToLoad);
  ExpectPdCommand(command, stand, Eigen::VectorXd::Zero(31), kp, kd);
}

TEST(PpServeController, EntryPreservesMovingCommandAndReplacesSeedOnReentry) {
  auto timeline = LoadTimeline();
  const auto ready = timeline.ready().q_sdk;
  const auto stand = timeline.complete().q_sdk;
  const auto kp = PositiveGains(90), kd = PositiveGains(3);
  PpServeController controller(std::move(timeline), stand, kp, kd, nullptr);
  auto state = StateAt(stand);
  robot_io::RobotCommand source, command;
  source.q_des = stand.array() + .02;
  source.dq_des = PositiveGains(.1);
  source.kp = PositiveGains(50);
  source.kd = PositiveGains(2);
  source.tau_ff = PositiveGains(.3);
  for (int entry = 0; entry < 2; ++entry) {
    source.q_des.array() += .01;
    controller.SetEntryCommand(source);
    controller.Start();
    ASSERT_TRUE(controller.ComputeCommand(0, state, command));
    EXPECT_TRUE(command.q_des.isApprox(source.q_des, 0));
    EXPECT_TRUE(command.dq_des.isApprox(source.dq_des, 0));
    EXPECT_TRUE(command.kp.isApprox(source.kp, 0));
    EXPECT_TRUE(command.kd.isApprox(source.kd, 0));
    EXPECT_TRUE(command.tau_ff.isApprox(source.tau_ff, 0));
    EXPECT_EQ(controller.state(), ServeControllerState::kPreparingStand);
    EXPECT_TRUE(controller.active());
    state.imu_gyro[1] = .2;
    for (std::size_t t = 1; t < 100; ++t)
      ASSERT_TRUE(controller.ComputeCommand(t, state, command));
    EXPECT_EQ(controller.state(), ServeControllerState::kPreparingStand);
    ASSERT_TRUE(controller.ComputeCommand(100, state, command));
    ExpectPdCommand(command, stand, Eigen::VectorXd::Zero(31), kp, kd, 1e-12);
    EXPECT_EQ(controller.state(), ServeControllerState::kTransitionToLoad);
    state.imu_gyro.setZero();
    for (std::size_t t = 0; t < a3_pingpong::kServe025FullbodyTransitionTicks; ++t)
      ASSERT_TRUE(controller.ComputeCommand(t, state, command));
    ExpectPdCommand(command, ready, Eigen::VectorXd::Zero(31), kp, kd, 1e-12);
    controller.RequestAbort();
    ASSERT_TRUE(controller.ComputeCommand(502, state, command));
    ASSERT_EQ(controller.state(), ServeControllerState::kAborted);
  }
}

TEST(PpServeController, PreservesForwardHitRightArmVelocityExactly) {
  auto timeline = LoadTimeline();
  const auto expected = timeline;
  const Eigen::VectorXd stand = timeline.complete().q_sdk;
  const Eigen::VectorXd kp = PositiveGains(90.0);
  const Eigen::VectorXd kd = PositiveGains(3.0);
  PpServeController controller(
      std::move(timeline), stand, kp, kd, nullptr);
  const auto state = StateAt(stand);
  robot_io::RobotCommand command;
  AdvanceToReady(controller, state, command);
  controller.TriggerReadyToServe();

  for (std::size_t tick = 0; tick <= 82; ++tick) {
    ASSERT_TRUE(controller.ComputeCommand(1000 + tick, state, command));
  }
  ASSERT_EQ(controller.TakeDiag().frame, 82U);
  const auto& strike = expected.At(82);
  EXPECT_TRUE(command.q_des.isApprox(strike.q_sdk, 0.0));
  EXPECT_TRUE(command.dq_des.isApprox(strike.qd_sdk, 0.0));
  // Keep this a moving strike sample so a dropped velocity command cannot pass.
  EXPECT_GT(std::abs(strike.qd_sdk[15]), 1.0);
}

TEST(PpServeController, MissingGripperNeverBlocksBodyTimeline) {
  auto timeline = LoadTimeline();
  const Eigen::VectorXd stand = timeline.complete().q_sdk;
  PpServeController controller(
      std::move(timeline), stand, PositiveGains(80.0),
      PositiveGains(2.0), nullptr);
  const auto state = StateAt(stand);
  robot_io::RobotCommand command;
  AdvanceToReady(controller, state, command);
  EXPECT_EQ(controller.gripper_state(), ServeGripperState::kUnavailable);

  controller.TriggerReadyToServe();
  for (std::size_t tick = 0;
       tick < a3_pingpong::kServe025FullbodyCommandTicks; ++tick) {
    ASSERT_TRUE(controller.ComputeCommand(tick, state, command));
  }
  EXPECT_EQ(controller.state(), ServeControllerState::kComplete);
}

TEST(PpServeController, ReleaseAcceptsAnyReceiptInsideFivePublishBurst) {
  for (std::size_t lead : {0U, 1U, 2U, 3U}) {
    struct Shared {
      std::mutex mutex;
      std::vector<PpGripperCommand> commands;
    } shared;
    PpGripperWorkerTransport transport;
    transport.exchange = [&shared](PpGripperCommand command,
                                   std::chrono::milliseconds,
                                   PpGripperReceipt& receipt,
                                   std::string& error) {
      {
        std::lock_guard<std::mutex> lock(shared.mutex);
        shared.commands.push_back(command);
      }
      receipt = SuccessfulReceipt(command);
      error.clear();
      return true;
    };
    auto worker = std::make_unique<PpGripperWorker>(std::move(transport));
    std::string error;
    ASSERT_TRUE(worker->Start(error)) << error;

    auto timeline = LoadTimeline();
    const auto expected = timeline;
    const Eigen::VectorXd stand = timeline.complete().q_sdk;
    PpServeController controller(
        std::move(timeline), stand, PositiveGains(80.0),
        PositiveGains(2.0), std::move(worker));
    const auto state = StateAt(stand);
    robot_io::RobotCommand command;
    controller.SetReleaseLeadFrames(lead);
    AdvanceToReady(controller, state, command);
    for (int attempt = 0; attempt < 100 &&
         controller.gripper_state() != ServeGripperState::kGrabbed; ++attempt) {
      controller.PollAsync();
      ASSERT_TRUE(controller.ComputeCommand(500 + attempt, state, command));
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    EXPECT_EQ(controller.gripper_state(), ServeGripperState::kGrabbed);

    controller.TriggerReadyToServe();
    for (std::size_t tick = 0; tick <= 60; ++tick) {
      ASSERT_TRUE(controller.ComputeCommand(1000 + tick, state, command));
      EXPECT_TRUE(command.q_des.isApprox(expected.At(tick).q_sdk, 0.));
      EXPECT_TRUE(command.dq_des.isApprox(expected.At(tick).qd_sdk, 0.));
      if (tick < 48 - lead) EXPECT_EQ(controller.TakeDiag().release_dispatch_monotonic_ns, 0U);
      else EXPECT_GT(controller.TakeDiag().release_dispatch_monotonic_ns, 0U);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    controller.PollAsync();
    EXPECT_EQ(controller.gripper_state(), ServeGripperState::kReleased);
    std::lock_guard<std::mutex> lock(shared.mutex);
    ASSERT_EQ(shared.commands.size(), 3U);
    EXPECT_EQ(shared.commands[0], PpGripperCommand::kOpen);
    EXPECT_EQ(shared.commands[1], PpGripperCommand::kGrab);
    EXPECT_EQ(shared.commands[2], PpGripperCommand::kRelease);
  }
}

TEST(PpServeController, NewPrepareRetriesOpenAfterPriorTransportFault) {
  struct Shared {
    std::mutex mutex;
    std::vector<PpGripperCommand> commands;
    bool fail_first_open{true};
  } shared;
  PpGripperWorkerTransport transport;
  transport.exchange = [&shared](PpGripperCommand command,
                                 std::chrono::milliseconds,
                                 PpGripperReceipt& receipt,
                                 std::string& error) {
    std::lock_guard<std::mutex> lock(shared.mutex);
    shared.commands.push_back(command);
    if (command == PpGripperCommand::kOpen && shared.fail_first_open) {
      shared.fail_first_open = false;
      error = "injected first OPEN failure";
      return false;
    }
    receipt = SuccessfulReceipt(command);
    error.clear();
    return true;
  };
  auto worker = std::make_unique<PpGripperWorker>(std::move(transport));
  std::string error;
  ASSERT_TRUE(worker->Start(error)) << error;

  auto timeline = LoadTimeline();
  const Eigen::VectorXd stand = timeline.complete().q_sdk;
  PpServeController controller(
      std::move(timeline), stand, PositiveGains(80.0),
      PositiveGains(2.0), std::move(worker));
  const auto state = StateAt(stand);
  robot_io::RobotCommand command;

  controller.Start();
  ASSERT_TRUE(controller.ComputeCommand(0, state, command));
  for (int attempt = 0; attempt < 100 &&
       controller.gripper_state() != ServeGripperState::kFault; ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    controller.PollAsync();
  }
  EXPECT_EQ(controller.gripper_state(), ServeGripperState::kFault);
  controller.RequestAbort();
  ASSERT_TRUE(controller.ComputeCommand(1, state, command));
  EXPECT_EQ(controller.state(), ServeControllerState::kAborted);

  controller.Start();
  ASSERT_TRUE(controller.ComputeCommand(2, state, command));
  for (int attempt = 0; attempt < 100 &&
       controller.gripper_state() != ServeGripperState::kGrabbed; ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    controller.PollAsync();
    ASSERT_TRUE(controller.ComputeCommand(3 + attempt, state, command));
  }
  EXPECT_EQ(controller.gripper_state(), ServeGripperState::kGrabbed);
  std::lock_guard<std::mutex> lock(shared.mutex);
  ASSERT_GE(shared.commands.size(), 3U);
  EXPECT_EQ(shared.commands[0], PpGripperCommand::kOpen);
  EXPECT_EQ(shared.commands[1], PpGripperCommand::kOpen);
  EXPECT_EQ(shared.commands[2], PpGripperCommand::kGrab);
}

TEST(PpServeController, InvalidActualStateStillFailsClosed) {
  auto timeline = LoadTimeline();
  const Eigen::VectorXd stand = timeline.complete().q_sdk;
  PpServeController controller(
      std::move(timeline), stand, PositiveGains(80.0),
      PositiveGains(2.0), nullptr);
  auto state = StateAt(stand);
  state.q[0] = std::numeric_limits<double>::quiet_NaN();
  robot_io::RobotCommand command;

  controller.Start();
  EXPECT_FALSE(controller.ComputeCommand(0, state, command));
  EXPECT_EQ(controller.state(), ServeControllerState::kFault);
  EXPECT_FALSE(controller.fault_reason().empty());
}

TEST(PpServeController, AbortHoldsLastRunnerOwnedCommand) {
  auto timeline = LoadTimeline();
  const Eigen::VectorXd stand = timeline.complete().q_sdk;
  PpServeController controller(
      std::move(timeline), stand, PositiveGains(80.0),
      PositiveGains(2.0), nullptr);
  const auto state = StateAt(stand);
  robot_io::RobotCommand command;

  controller.Start();
  ASSERT_TRUE(controller.ComputeCommand(0, state, command));
  const robot_io::RobotCommand before_abort = command;
  controller.RequestAbort();
  ASSERT_TRUE(controller.ComputeCommand(1, state, command));
  EXPECT_EQ(controller.state(), ServeControllerState::kAborted);
  ExpectPdCommand(command, before_abort.q_des, before_abort.dq_des,
                  before_abort.kp, before_abort.kd);
}

}  // namespace

TEST(PpServeController, EarlyPolicyHandoffPreservesReleaseContactAndPlayedFrames) {
  auto timeline = LoadTimeline();
  const auto expected = timeline;
  const auto stand = timeline.complete().q_sdk;
  const auto kp = PositiveGains(90), kd = PositiveGains(3);
  PpServeController controller(std::move(timeline), stand, kp, kd, nullptr);
  EXPECT_THROW(controller.SetPolicyHandoffFrame(78), std::invalid_argument);
  controller.SetPolicyHandoffFrame(110);
  auto state = StateAt(stand);
  robot_io::RobotCommand command;
  AdvanceToReady(controller, state, command);
  EXPECT_THROW(controller.SetPolicyHandoffFrame(120), std::invalid_argument);
  controller.TriggerReadyToServe();
  for (std::size_t tick=0; tick<=110; ++tick) {
    ASSERT_TRUE(controller.ComputeCommand(1000+tick,state,command));
    ExpectPdCommand(command,expected.At(tick).q_sdk,expected.At(tick).qd_sdk,kp,kd);
    if (tick<110) EXPECT_NE(controller.state(),ServeControllerState::kComplete);
  }
  EXPECT_EQ(controller.state(),ServeControllerState::kHandoffReady);
  EXPECT_TRUE(controller.active());
  const auto source = command;
  state = StateAt(expected.At(110).q_sdk);
  ASSERT_TRUE(controller.ComputeCommand(1111,state,command));
  ExpectPdCommand(command,source.q_des,source.dq_des,kp,kd);
  // Expired return clock must not hand a stalled arm pose to the actor.
  for (int i=0;i<150;++i) ASSERT_TRUE(controller.ComputeCommand(1112+i,state,command));
  EXPECT_EQ(controller.state(),ServeControllerState::kHandoffReady);
  ExpectPdCommand(command,stand,Eigen::VectorXd::Zero(31),kp,kd);
  state = StateAt(stand);
  state.dq[0] = .7;
  for (int i=0;i<20;++i) ASSERT_TRUE(controller.ComputeCommand(1300+i,state,command));
  EXPECT_EQ(controller.state(),ServeControllerState::kHandoffReady);
  state.dq.setZero();
  for (int i=0;i<9;++i) ASSERT_TRUE(controller.ComputeCommand(1400+i,state,command));
  EXPECT_EQ(controller.state(),ServeControllerState::kHandoffReady);
  state.imu_gyro[0] = .4;
  ASSERT_TRUE(controller.ComputeCommand(1409,state,command));
  state.imu_gyro.setZero();
  for (int i=0;i<20;++i) ASSERT_TRUE(controller.ComputeCommand(1410+i,state,command));
  EXPECT_EQ(controller.state(),ServeControllerState::kComplete);
  EXPECT_FALSE(controller.active());
}

TEST(PpServeController, AbortRemainsAvailableDuringPostContactStandReturn) {
  auto timeline = LoadTimeline();
  const auto stand = timeline.complete().q_sdk;
  PpServeController controller(std::move(timeline), stand, PositiveGains(90), PositiveGains(3), nullptr);
  controller.SetPolicyHandoffFrame(110);
  auto state = StateAt(stand);
  robot_io::RobotCommand command;
  AdvanceToReady(controller,state,command);
  controller.TriggerReadyToServe();
  for (int i=0;i<=110;++i) ASSERT_TRUE(controller.ComputeCommand(i,state,command));
  ASSERT_EQ(controller.state(),ServeControllerState::kHandoffReady);
  controller.RequestAbort();
  ASSERT_TRUE(controller.ComputeCommand(111,state,command));
  EXPECT_EQ(controller.state(),ServeControllerState::kAborted);
}

TEST(PpServeController, ReleaseLeadIsBoundedAndCannotChangeDuringAction) {
  auto timeline = LoadTimeline();
  const auto stand = timeline.complete().q_sdk;
  PpServeController controller(std::move(timeline), stand, PositiveGains(80), PositiveGains(2), nullptr);
  EXPECT_EQ(controller.release_frame(), 48U);
  EXPECT_THROW(controller.SetReleaseLeadFrames(4), std::invalid_argument);
  controller.SetReleaseLeadFrames(2);
  EXPECT_EQ(controller.release_frame(), 46U);
  robot_io::RobotCommand command;
  controller.Start();
  ASSERT_TRUE(controller.ComputeCommand(0, StateAt(stand), command));
  EXPECT_THROW(controller.SetReleaseLeadFrames(0), std::invalid_argument);
}

TEST(PpServeController, PitchSupportContinuesThroughReadyAndPreservesCsvArms) {
  auto timeline = LoadTimeline();
  const auto stand = timeline.complete().q_sdk;
  const auto ready = timeline.ready().q_sdk;
  const auto kp = PositiveGains(90), kd = PositiveGains(3);
  PpServeController level(timeline, stand, kp, kd, nullptr);
  PpServeController tilted(std::move(timeline), stand, kp, kd, nullptr);
  auto state = StateAt(stand), lean = state;
  const Eigen::Quaterniond orientation = Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-.1, Eigen::Vector3d::UnitY());
  lean.imu_quat_wxyz << orientation.w(), orientation.x(), orientation.y(), orientation.z();
  lean.imu_gyro[1] = -.2;
  robot_io::RobotCommand a, b;
  level.Start(); tilted.Start();
  double maximum = 0.;
  for (std::size_t t = 0; t < a3_pingpong::kServe025FullbodyTransitionTicks; ++t) {
    ASSERT_TRUE(level.ComputeCommand(t, state, a));
    ASSERT_TRUE(tilted.ComputeCommand(t, lean, b));
    for (int j = 0; j < 31; ++j) {
      if (j == 23 || j == 29) {
        const double delta = b.q_des[j] - a.q_des[j];
        EXPECT_GE(delta, -.080001);
        EXPECT_LE(delta, 1e-12);
        EXPECT_LE(std::abs(b.dq_des[j] - a.dq_des[j]), .3);
        maximum = std::max(maximum, std::abs(delta));
      } else EXPECT_DOUBLE_EQ(a.q_des[j], b.q_des[j]);
    }
    if (t == 0) ExpectPdCommand(b, a.q_des, a.dq_des, kp, kd, 1e-12);
  }
  EXPECT_GT(maximum, .05);
  EXPECT_LT(b.q_des[23] - ready[23], -.05);
  const auto last = b;
  ASSERT_TRUE(tilted.ComputeCommand(600, lean, b));
  EXPECT_LT((b.q_des - last.q_des).cwiseAbs().maxCoeff(), .0013);
  tilted.TriggerReadyToServe();
  auto expected = LoadTimeline();
  for (std::size_t frame = 0; frame < 100; ++frame) {
    ASSERT_TRUE(tilted.ComputeCommand(601 + frame, lean, b));
    for (int j = 0; j < 31; ++j) {
      if (j == 23 || j == 29) {
        EXPECT_NEAR(b.q_des[j] - expected.At(frame).q_sdk[j], -.08, 1e-9);
      } else {
        EXPECT_DOUBLE_EQ(b.q_des[j], expected.At(frame).q_sdk[j]);
        EXPECT_DOUBLE_EQ(b.dq_des[j], expected.At(frame).qd_sdk[j]);
      }
    }
  }
  EXPECT_EQ(tilted.state(), ServeControllerState::kFollowThrough);
}

TEST(PpServeController, MovingPitchSupportIsContinuousOnServeButton) {
  auto timeline = LoadTimeline();
  const auto stand = timeline.complete().q_sdk;
  const auto kp = PositiveGains(90), kd = PositiveGains(3);
  PpServeController controller(std::move(timeline), stand, kp, kd, nullptr);
  auto state = StateAt(stand);
  robot_io::RobotCommand command;
  controller.Start();
  for (std::size_t t = 0; t < a3_pingpong::kServe025FullbodyTransitionTicks; ++t)
    ASSERT_TRUE(controller.ComputeCommand(t, state, command));
  const Eigen::Quaterniond q(Eigen::AngleAxisd(-.04, Eigen::Vector3d::UnitY()));
  state.imu_quat_wxyz << q.w(), q.x(), q.y(), q.z();
  state.imu_gyro[1] = -.08;
  ASSERT_TRUE(controller.ComputeCommand(600, state, command));
  const auto before = command;
  ASSERT_LT(before.dq_des[23], -.05);
  controller.TriggerReadyToServe();
  ASSERT_TRUE(controller.ComputeCommand(601, state, command));
  EXPECT_LT((command.q_des-before.q_des).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT((command.dq_des-before.dq_des).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT((command.kp-before.kp).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT((command.kd-before.kd).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT((command.tau_ff-before.tau_ff).cwiseAbs().maxCoeff(), 1e-12);
}

TEST(PpServeController, SlowStandReturnKeepsEntryContinuousAndLowersArmsOverTwoAndHalfSeconds) {
  auto timeline = LoadTimeline();
  const auto stand = timeline.complete().q_sdk;
  const auto kp = PositiveGains(90), kd = PositiveGains(3);
  PpServeController controller(std::move(timeline), stand, kp, kd, nullptr);
  controller.SetPolicyHandoffFrame(110);
  controller.SetPolicyReturnSeconds(2.5);
  auto state = StateAt(stand);
  robot_io::RobotCommand command;
  AdvanceToReady(controller, state, command);
  controller.TriggerReadyToServe();
  for (unsigned t = 0; t <= 110; ++t)
    ASSERT_TRUE(controller.ComputeCommand(t, state, command));
  const auto source = command;
  ASSERT_TRUE(controller.ComputeCommand(111, state, command));
  ExpectPdCommand(command, source.q_des, source.dq_des, kp, kd);
  for (unsigned t = 1; t <= 100; ++t)
    ASSERT_TRUE(controller.ComputeCommand(111+t, state, command));
  EXPECT_EQ(controller.state(), ServeControllerState::kHandoffReady);
  EXPECT_GT((command.q_des.segment(5,14)-stand.segment(5,14)).cwiseAbs().maxCoeff(), .1);
  for (unsigned t = 101; t <= 250; ++t)
    ASSERT_TRUE(controller.ComputeCommand(111+t, state, command));
  ExpectPdCommand(command, stand, Eigen::VectorXd::Zero(31), kp, kd);
  for (unsigned t = 0; t < 20; ++t)
    ASSERT_TRUE(controller.ComputeCommand(400+t, state, command));
  EXPECT_EQ(controller.state(), ServeControllerState::kComplete);
}
