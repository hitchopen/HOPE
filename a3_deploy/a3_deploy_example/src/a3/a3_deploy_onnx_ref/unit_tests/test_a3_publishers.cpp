// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_backend_plan.md §? / PR 6
//
// Unit tests for PR 6: publish_helpers + 4 command publishers +
// A3PublisherManager.
//
// Part A (always compiled): pure BuildJointCmdRow<DOF> tests.
// Part B (#ifdef HAS_A3_ROS_MSGS): real joint_msgs::msg::JointCommand
// construction + lambda-captured publish + A3PublisherManager.PublishAll.

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "a3_io/publish_helpers.hpp"
#include "a3_io/publisher_manager.hpp"
#include "a3_io/publishers/arm_cmd_pub.hpp"
#include "a3_io/publishers/leg_cmd_pub.hpp"
#include "a3_io/publishers/neck_cmd_pub.hpp"
#include "a3_io/publishers/waist_cmd_pub.hpp"
#include "robot_io/a3_layout_extra.hpp"
#include "robot_io/layouts.hpp"
#include "robot_io/robot_io_backend.hpp"

namespace {

// Build a deterministic 31-DOF RobotCommand where every field encodes
// its flat index (so tests can recompute expected values).
robot_io::RobotCommand MakeDeterministicCmd31() {
  constexpr int N = robot_io::kA3Dof;  // 31
  robot_io::RobotCommand cmd;
  cmd.q_des  = Eigen::VectorXd::Zero(N);
  cmd.dq_des = Eigen::VectorXd::Zero(N);
  cmd.tau_ff = Eigen::VectorXd::Zero(N);
  cmd.kp     = Eigen::VectorXd::Zero(N);
  cmd.kd     = Eigen::VectorXd::Zero(N);
  for (int i = 0; i < N; ++i) {
    cmd.q_des[i]  = static_cast<double>(i) * 0.01;
    cmd.dq_des[i] = static_cast<double>(i) * 0.1;
    cmd.tau_ff[i] = static_cast<double>(i) + 0.5;
    cmd.kp[i]     = 100.0 + static_cast<double>(i);
    cmd.kd[i]     = 1.0 + static_cast<double>(i) * 0.01;
  }
  return cmd;
}

}  // namespace

// ============================================================================
// Part A — BuildJointCmdRow pure tests
// ============================================================================

TEST(BuildJointCmdRow, WaistExtractsCorrectSlice) {
  const auto cmd = MakeDeterministicCmd31();
  // Waist starts at A3 SDK slot 0; DOF=3.
  for (int sub = 0; sub < robot_io::kA3WaistCount; ++sub) {
    const auto row = a3_io::BuildJointCmdRow<robot_io::kA3WaistCount>(
        cmd, robot_io::kA3WaistStart, sub, robot_io::kA3WaistJointNames,
        /*seq=*/42u);
    const int flat = robot_io::kA3WaistStart + sub;
    EXPECT_EQ(row.name, robot_io::kA3WaistJointNames[sub]);
    EXPECT_EQ(row.sequence, 42u);
    EXPECT_DOUBLE_EQ(row.position, static_cast<double>(flat) * 0.01);
    EXPECT_DOUBLE_EQ(row.velocity, static_cast<double>(flat) * 0.1);
    EXPECT_DOUBLE_EQ(row.effort, static_cast<double>(flat) + 0.5);
    EXPECT_DOUBLE_EQ(row.stiffness, 100.0 + static_cast<double>(flat));
    EXPECT_DOUBLE_EQ(row.damping, 1.0 + static_cast<double>(flat) * 0.01);
  }
}

TEST(BuildJointCmdRow, LegSpotChecks) {
  const auto cmd = MakeDeterministicCmd31();
  // Leg starts at A3 SDK slot 19; DOF=12. Spot check sub 0, 5, 11.
  for (int sub : {0, 5, 11}) {
    const auto row = a3_io::BuildJointCmdRow<robot_io::kA3LegCount>(
        cmd, robot_io::kA3LegStart, sub, robot_io::kA3LegJointNames, 7u);
    const int flat = robot_io::kA3LegStart + sub;
    EXPECT_EQ(row.name, robot_io::kA3LegJointNames[sub]);
    EXPECT_DOUBLE_EQ(row.position, static_cast<double>(flat) * 0.01);
    EXPECT_DOUBLE_EQ(row.stiffness, 100.0 + static_cast<double>(flat));
  }
}

TEST(BuildJointCmdRow, ArmSpotChecks) {
  const auto cmd = MakeDeterministicCmd31();
  // Arm starts at A3 SDK slot 5; DOF=14.
  for (int sub : {0, 7, 13}) {
    const auto row = a3_io::BuildJointCmdRow<robot_io::kA3ArmCount>(
        cmd, robot_io::kA3ArmStart, sub, robot_io::kA3ArmJointNames, 99u);
    const int flat = robot_io::kA3ArmStart + sub;
    EXPECT_EQ(row.name, robot_io::kA3ArmJointNames[sub]);
    EXPECT_EQ(row.sequence, 99u);
    EXPECT_DOUBLE_EQ(row.velocity, static_cast<double>(flat) * 0.1);
    EXPECT_DOUBLE_EQ(row.damping, 1.0 + static_cast<double>(flat) * 0.01);
  }
}

TEST(BuildJointCmdRow, NeckBothJoints) {
  const auto cmd = MakeDeterministicCmd31();
  for (int sub = 0; sub < robot_io::kA3NeckCount; ++sub) {
    const auto row = a3_io::BuildJointCmdRow<robot_io::kA3NeckCount>(
        cmd, robot_io::kA3NeckStart, sub, robot_io::kA3NeckJointNames, 0u);
    const int flat = robot_io::kA3NeckStart + sub;
    EXPECT_EQ(row.name, robot_io::kA3NeckJointNames[sub]);
    EXPECT_DOUBLE_EQ(row.effort, static_cast<double>(flat) + 0.5);
  }
}

TEST(BuildJointCmdRow, SubIdxOutOfRangeThrows) {
  const auto cmd = MakeDeterministicCmd31();
  EXPECT_THROW((a3_io::BuildJointCmdRow<robot_io::kA3NeckCount>(
                   cmd, robot_io::kA3NeckStart, /*sub=*/2,
                   robot_io::kA3NeckJointNames, 0u)),
               std::out_of_range);
  EXPECT_THROW((a3_io::BuildJointCmdRow<robot_io::kA3NeckCount>(
                   cmd, robot_io::kA3NeckStart, /*sub=*/-1,
                   robot_io::kA3NeckJointNames, 0u)),
               std::out_of_range);
}

TEST(BuildJointCmdRow, RobotCommandTooShortThrows) {
  robot_io::RobotCommand tiny;
  tiny.q_des  = Eigen::VectorXd::Zero(5);
  tiny.dq_des = Eigen::VectorXd::Zero(5);
  tiny.tau_ff = Eigen::VectorXd::Zero(5);
  tiny.kp     = Eigen::VectorXd::Zero(5);
  tiny.kd     = Eigen::VectorXd::Zero(5);
  // Arm starts at A3 SDK slot 5 → slot 5+13 = 18 is beyond 5 entries.
  EXPECT_THROW((a3_io::BuildJointCmdRow<robot_io::kA3ArmCount>(
                   tiny, robot_io::kA3ArmStart, 0,
                   robot_io::kA3ArmJointNames, 0u)),
               std::out_of_range);
}

TEST(BuildJointCmdRow, SequencePropagates) {
  const auto cmd = MakeDeterministicCmd31();
  for (std::uint32_t seq : {0u, 1u, 1234u, 0xFFFFFFFFu}) {
    const auto row = a3_io::BuildJointCmdRow<robot_io::kA3ArmCount>(
        cmd, robot_io::kA3ArmStart, 3, robot_io::kA3ArmJointNames, seq);
    EXPECT_EQ(row.sequence, seq);
  }
}

// ============================================================================
// Part B — ROS2 joint_msgs JointCommand + lambda publish + manager
// ============================================================================

#ifdef HAS_A3_ROS_MSGS

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

using joint_msgs::msg::JointCommand;

TEST(A3PubMsg, WaistBuildMessageFillsAllFields) {
  const auto cmd = MakeDeterministicCmd31();
  JointCommand msg;
  a3_io::WaistCmdPub::BuildMessage(cmd, /*stamp_ns=*/1'777'000'000'000LL,
                                   /*seq=*/11u, msg);
  ASSERT_EQ(msg.joints.size(), robot_io::kA3WaistCount);
  EXPECT_EQ(msg.header.stamp.sec, 1777);
  EXPECT_EQ(msg.header.stamp.nanosec, 0u);
  for (std::size_t i = 0; i < msg.joints.size(); ++i) {
    const int flat = robot_io::kA3WaistStart + static_cast<int>(i);
    EXPECT_EQ(msg.joints[i].name, robot_io::kA3WaistJointNames[i]);
    EXPECT_EQ(msg.joints[i].sequence, 11u);
    EXPECT_DOUBLE_EQ(msg.joints[i].position, flat * 0.01);
    EXPECT_DOUBLE_EQ(msg.joints[i].velocity, flat * 0.1);
    EXPECT_DOUBLE_EQ(msg.joints[i].effort, flat + 0.5);
    EXPECT_DOUBLE_EQ(msg.joints[i].stiffness, 100.0 + flat);
    EXPECT_DOUBLE_EQ(msg.joints[i].damping, 1.0 + flat * 0.01);
  }
}

TEST(A3PubMsg, LegPublishViaLambdaCaptures12Joints) {
  const auto cmd = MakeDeterministicCmd31();
  std::vector<JointCommand> captured;
  a3_io::LegCmdPub pub(
      [&captured](const JointCommand& m) { captured.push_back(m); });

  pub.Publish(/*stamp_ns=*/1'500'000'000LL, /*seq=*/3u, cmd);
  ASSERT_EQ(captured.size(), 1u);
  ASSERT_EQ(captured[0].joints.size(), robot_io::kA3LegCount);
  EXPECT_EQ(captured[0].header.stamp.sec, 1);
  EXPECT_EQ(captured[0].header.stamp.nanosec, 500'000'000u);
  for (std::size_t i = 0; i < captured[0].joints.size(); ++i) {
    EXPECT_EQ(captured[0].joints[i].name, robot_io::kA3LegJointNames[i]);
    EXPECT_DOUBLE_EQ(captured[0].joints[i].position,
                     (robot_io::kA3LegStart + static_cast<int>(i)) * 0.01);
    EXPECT_EQ(captured[0].joints[i].sequence, 3u);
  }
}

TEST(A3PubMsg, ArmPublishViaLambdaCaptures14Joints) {
  const auto cmd = MakeDeterministicCmd31();
  std::vector<JointCommand> captured;
  a3_io::ArmCmdPub pub(
      [&captured](const JointCommand& m) { captured.push_back(m); });
  pub.Publish(0, /*seq=*/0u, cmd);
  ASSERT_EQ(captured.size(), 1u);
  ASSERT_EQ(captured[0].joints.size(), robot_io::kA3ArmCount);
  for (std::size_t i = 0; i < robot_io::kA3ArmCount; ++i) {
    EXPECT_EQ(captured[0].joints[i].name, robot_io::kA3ArmJointNames[i]);
    EXPECT_DOUBLE_EQ(captured[0].joints[i].stiffness,
                     100.0 + (robot_io::kA3ArmStart + static_cast<int>(i)));
  }
}

TEST(A3PubMsg, NeckPublishViaLambdaCaptures2Joints) {
  const auto cmd = MakeDeterministicCmd31();
  std::vector<JointCommand> captured;
  a3_io::NeckCmdPub pub(
      [&captured](const JointCommand& m) { captured.push_back(m); });
  pub.Publish(/*stamp_ns=*/12LL, /*seq=*/9u, cmd);
  ASSERT_EQ(captured.size(), 1u);
  ASSERT_EQ(captured[0].joints.size(), robot_io::kA3NeckCount);
  EXPECT_EQ(captured[0].header.stamp.sec, 0);
  EXPECT_EQ(captured[0].header.stamp.nanosec, 12u);
  EXPECT_EQ(captured[0].joints[0].name, robot_io::kA3NeckJointNames[0]);
  EXPECT_EQ(captured[0].joints[1].name, robot_io::kA3NeckJointNames[1]);
}

TEST(A3PubMsg, EmptyPublishFnIsNoop) {
  const auto cmd = MakeDeterministicCmd31();
  a3_io::ArmCmdPub pub;  // no publish_fn
  // Should not crash.
  pub.Publish(0, 0u, cmd);
  SUCCEED();
}

TEST(A3PubMsg, JointCommandSerializesWithRos2TypeSupport) {
  const auto cmd = MakeDeterministicCmd31();
  JointCommand msg;
  a3_io::LegCmdPub::BuildMessage(cmd, /*stamp_ns=*/1'500'000'000LL,
                                 /*seq=*/3u, msg);

  rclcpp::Serialization<JointCommand> serializer;
  rclcpp::SerializedMessage serialized;
  EXPECT_NO_THROW(serializer.serialize_message(&msg, &serialized));
  EXPECT_GT(serialized.size(), 0u);
}

TEST(A3PublisherManager, PublishAllFansOutToAllFour) {
  const auto cmd = MakeDeterministicCmd31();
  std::vector<JointCommand> w, l, a, n;

  a3_io::A3PublisherManager::Options opt;
  opt.waist_publish_fn = [&w](const JointCommand& m) { w.push_back(m); };
  opt.leg_publish_fn   = [&l](const JointCommand& m) { l.push_back(m); };
  opt.arm_publish_fn   = [&a](const JointCommand& m) { a.push_back(m); };
  opt.neck_publish_fn  = [&n](const JointCommand& m) { n.push_back(m); };

  a3_io::A3PublisherManager mgr(std::move(opt));

  mgr.PublishAll(/*stamp_ns=*/2'000'000'000LL, /*seq=*/5u, cmd);

  ASSERT_EQ(w.size(), 1u);
  ASSERT_EQ(l.size(), 1u);
  ASSERT_EQ(a.size(), 1u);
  ASSERT_EQ(n.size(), 1u);
  EXPECT_EQ(w[0].joints.size(), robot_io::kA3WaistCount);
  EXPECT_EQ(l[0].joints.size(), robot_io::kA3LegCount);
  EXPECT_EQ(a[0].joints.size(), robot_io::kA3ArmCount);
  EXPECT_EQ(n[0].joints.size(), robot_io::kA3NeckCount);

  // All four should carry the same stamp (sec=2, nanosec=0).
  for (const auto* v : {&w, &l, &a, &n}) {
    EXPECT_EQ((*v)[0].header.stamp.sec, 2);
    EXPECT_EQ((*v)[0].header.stamp.nanosec, 0u);
  }
}

TEST(A3PublisherManager, MultiplePublishAllIncrementsCaptures) {
  const auto cmd = MakeDeterministicCmd31();
  std::vector<JointCommand> w, l, a, n;
  a3_io::A3PublisherManager::Options opt;
  opt.waist_publish_fn = [&w](const JointCommand& m) { w.push_back(m); };
  opt.leg_publish_fn   = [&l](const JointCommand& m) { l.push_back(m); };
  opt.arm_publish_fn   = [&a](const JointCommand& m) { a.push_back(m); };
  opt.neck_publish_fn  = [&n](const JointCommand& m) { n.push_back(m); };
  a3_io::A3PublisherManager mgr(std::move(opt));

  for (std::uint32_t s = 0; s < 5; ++s) {
    mgr.PublishAll(static_cast<std::int64_t>(s) * 1'000'000'000LL, s, cmd);
  }
  EXPECT_EQ(w.size(), 5u);
  EXPECT_EQ(l.size(), 5u);
  EXPECT_EQ(a.size(), 5u);
  EXPECT_EQ(n.size(), 5u);
  for (std::uint32_t s = 0; s < 5; ++s) {
    EXPECT_EQ(w[s].header.stamp.sec, static_cast<int>(s));
    EXPECT_EQ(a[s].joints.front().sequence, s);
  }
}

#endif  // HAS_A3_ROS_MSGS
