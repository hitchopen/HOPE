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
         "a3p_op3_serve025_new_build4_deep_1p07_compact50_lowdrop35_"
         "strikewindow180_full31_balanced_face20deg_forwardhit_v4.csv";
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
    for (std::size_t t = 1; t <= 50; ++t)
      ASSERT_TRUE(controller.ComputeCommand(t, state, command));
    ExpectPdCommand(command, stand, Eigen::VectorXd::Zero(31), kp, kd, 1e-12);
    EXPECT_EQ(controller.state(), ServeControllerState::kPreparingStand);
    // Joint positions/velocities are nominal, but the base is still rocking.
    // The former 0.15 rad/s envelope incorrectly admitted this state.
    state.imu_gyro[1] = .10;
    for (std::size_t t = 0; t < 30; ++t)
      ASSERT_TRUE(controller.ComputeCommand(t, state, command));
    EXPECT_EQ(controller.state(), ServeControllerState::kPreparingStand);
    state.imu_gyro.setZero();
    // A turning point has low angular speed without a settled support pose.
    state.imu_quat_wxyz = Eigen::Vector4d(std::cos(.06 / 2), 0, std::sin(.06 / 2), 0);
    for (std::size_t t = 0; t < 30; ++t)
      ASSERT_TRUE(controller.ComputeCommand(t, state, command));
    EXPECT_EQ(controller.state(), ServeControllerState::kPreparingStand);
    // Heading must not affect the check; a disturbance resets the quiet dwell.
    state.imu_quat_wxyz = Eigen::Vector4d(0, 0, 0, 1);
    for (std::size_t t = 0; t < 20; ++t)
      ASSERT_TRUE(controller.ComputeCommand(t, state, command));
    EXPECT_EQ(controller.state(), ServeControllerState::kPreparingStand);
    state.imu_gyro[0] = .10;
    ASSERT_TRUE(controller.ComputeCommand(0, state, command));
    state.imu_gyro.setZero();
    for (std::size_t t = 0; t < 29; ++t)
      ASSERT_TRUE(controller.ComputeCommand(t, state, command));
    EXPECT_EQ(controller.state(), ServeControllerState::kPreparingStand);
    ASSERT_TRUE(controller.ComputeCommand(29, state, command));
    EXPECT_EQ(controller.state(), ServeControllerState::kTransitionToLoad);
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
  // Current forwardhit_v4 CSV caps frame 82 elbow speed at 6.9 rad/s.
  EXPECT_NEAR(std::abs(command.dq_des[15]), 6.9, 1.0e-9);
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
  const Eigen::VectorXd stand = timeline.complete().q_sdk;
  PpServeController controller(
      std::move(timeline), stand, PositiveGains(80.0),
      PositiveGains(2.0), std::move(worker));
  const auto state = StateAt(stand);
  robot_io::RobotCommand command;
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
