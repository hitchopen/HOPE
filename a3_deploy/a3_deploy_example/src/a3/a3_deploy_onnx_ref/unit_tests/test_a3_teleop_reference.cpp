// Copyright (c) 2026, AgiBot Inc. All rights reserved.
#include "a3_deploy/a3_teleop_reference.hpp"

#include "a3_deploy/a3_obs_builder.hpp"
#include "a3_policy_parameters.hpp"

#ifdef HAS_A3_TA_PROTO
#include "aimdk/protocol/ta/ta_channel.pb.h"
#endif

#include <gtest/gtest.h>

#include <cmath>

namespace {

constexpr double kDegToRad = 3.141592653589793238462643383279502884 / 180.0;
constexpr std::size_t kFutureFrames = 10;
constexpr std::size_t kPolicyDof = 29;
constexpr std::size_t kCommandDofFloats = kFutureFrames * kPolicyDof;

std::array<double, 4> QuatFromYawDeg(double yaw_deg) {
  const double half = 0.5 * yaw_deg * kDegToRad;
  return {std::cos(half), 0.0, 0.0, std::sin(half)};
}

a3_deploy::A3TeleopFrame MakeFrame(std::int64_t stamp_ns, double q0) {
  a3_deploy::A3TeleopFrame f{};
  f.stamp_ns = stamp_ns;
  f.pelvis_quat_wxyz = {1.0, 0.0, 0.0, 0.0};
  for (std::size_t i = 0; i < kPolicyDof; ++i) {
    f.q_mujoco[i] = q0 + static_cast<double>(i) * 0.01;
    f.dq_mujoco[i] = 10.0 + q0 + static_cast<double>(i) * 0.01;
  }
  return f;
}

a3_deploy::A3TeleopFrame MakeFrameWithYaw(std::int64_t stamp_ns,
                                          double q0,
                                          double yaw_deg) {
  auto f = MakeFrame(stamp_ns, q0);
  f.pelvis_quat_wxyz = QuatFromYawDeg(yaw_deg);
  return f;
}

}  // namespace

TEST(A3TeleopReference, DefaultStandTokenizerUsesDefaultPoseAndZeroVelocity) {
  std::array<float, a3_deploy::kA3TokenizerFloatsPerTick> out{};
  a3_deploy::BuildDefaultStandTokenizerSlice({1.0, 0.0, 0.0, 0.0}, out);

  for (std::size_t k = 0; k < kFutureFrames; ++k) {
    for (std::size_t i_il = 0; i_il < kPolicyDof; ++i_il) {
      EXPECT_FLOAT_EQ(out[k * kPolicyDof + i_il],
                      static_cast<float>(
                          a3_default_angles[a3_mujoco_to_isaaclab[i_il]]));
      EXPECT_FLOAT_EQ(out[kCommandDofFloats + k * kPolicyDof + i_il], 0.0f);
    }

    const std::size_t o = a3_deploy::kA3CommandMultiFutureFloats + k * 6;
    EXPECT_FLOAT_EQ(out[o + 0], 1.0f);
    EXPECT_FLOAT_EQ(out[o + 1], 0.0f);
    EXPECT_FLOAT_EQ(out[o + 2], 0.0f);
    EXPECT_FLOAT_EQ(out[o + 3], 1.0f);
    EXPECT_FLOAT_EQ(out[o + 4], 0.0f);
    EXPECT_FLOAT_EQ(out[o + 5], 0.0f);
  }
}

TEST(A3TeleopReference, StandFallbackCommandFilterBlendsAndClamps) {
  std::array<double, 29> q_des{};
  for (std::size_t i = 0; i < kPolicyDof; ++i) {
    q_des[i] = a3_default_angles[i] + 1.0;
  }

  a3_deploy::ApplyStandFallbackCommandFilter(/*policy_blend=*/0.25,
                                             /*max_delta_rad=*/0.08, q_des);

  for (std::size_t i = 0; i < kPolicyDof; ++i) {
    EXPECT_DOUBLE_EQ(q_des[i], a3_default_angles[i] + 0.08);
  }

  a3_deploy::ApplyStandFallbackCommandFilter(/*policy_blend=*/0.0,
                                             /*max_delta_rad=*/0.08, q_des);
  for (std::size_t i = 0; i < kPolicyDof; ++i) {
    EXPECT_DOUBLE_EQ(q_des[i], a3_default_angles[i]);
  }
}

TEST(A3TeleopReference, ReportsBufferingUntilDelayWindowExists) {
  a3_deploy::A3TeleopReferenceOptions opt;
  opt.delay_ns = 900'000'000;
  opt.policy_hz = 10.0;
  opt.future_frame_skip = 1;
  a3_deploy::A3TeleopReferenceBuffer buffer(opt);
  buffer.PushFrame(MakeFrame(1'000'000'000, 1.0));

  std::array<float, a3_deploy::kA3TokenizerFloatsPerTick> out{};
  a3_deploy::A3TeleopTokenizerStatus status =
      a3_deploy::A3TeleopTokenizerStatus::kNoData;
  EXPECT_FALSE(buffer.BuildTokenizerSlice(1'500'000'000,
                                          {1.0, 0.0, 0.0, 0.0},
                                          0.0, out, &status));
  EXPECT_EQ(status, a3_deploy::A3TeleopTokenizerStatus::kBuffering);

  EXPECT_FALSE(buffer.BuildTokenizerSlice(1'900'000'000,
                                          {1.0, 0.0, 0.0, 0.0},
                                          0.0, out, &status));
  EXPECT_EQ(status, a3_deploy::A3TeleopTokenizerStatus::kBuffering);
}

TEST(A3TeleopReference, BuildsDelayedFutureWindow) {
  a3_deploy::A3TeleopReferenceOptions opt;
  opt.delay_ns = 900'000'000;
  opt.policy_hz = 10.0;
  opt.future_frame_skip = 1;
  a3_deploy::A3TeleopReferenceBuffer buffer(opt);

  for (int i = 0; i <= 9; ++i) {
    buffer.PushFrame(MakeFrame(1'000'000'000 + i * 100'000'000LL,
                               static_cast<double>(i)));
  }

  std::array<float, a3_deploy::kA3TokenizerFloatsPerTick> out{};
  a3_deploy::A3TeleopTokenizerStatus status =
      a3_deploy::A3TeleopTokenizerStatus::kNoData;
  ASSERT_TRUE(buffer.BuildTokenizerSlice(1'900'000'000,
                                         {1.0, 0.0, 0.0, 0.0},
                                         0.0, out, &status));
  EXPECT_EQ(status, a3_deploy::A3TeleopTokenizerStatus::kRunning);

  for (std::size_t k = 0; k < kFutureFrames; ++k) {
    for (std::size_t i_il = 0; i_il < kPolicyDof; ++i_il) {
      const int i_mj = a3_mujoco_to_isaaclab[i_il];
      const float expected_q =
          static_cast<float>(static_cast<double>(k) +
                             static_cast<double>(i_mj) * 0.01);
      const float expected_dq = static_cast<float>(
          10.0 + static_cast<double>(k) + static_cast<double>(i_mj) * 0.01);
      EXPECT_FLOAT_EQ(out[k * kPolicyDof + i_il], expected_q);
      EXPECT_FLOAT_EQ(out[kCommandDofFloats + k * kPolicyDof + i_il],
                      expected_dq);
    }
  }
}

TEST(A3TeleopReference, StartsWhenLatestFrameSlightlyLagsPolicyTick) {
  a3_deploy::A3TeleopReferenceOptions opt;
  opt.delay_ns = 900'000'000;
  opt.policy_hz = 50.0;
  opt.future_frame_skip = 5;
  a3_deploy::A3TeleopReferenceBuffer buffer(opt);

  for (int i = 0; i <= 89; ++i) {
    buffer.PushFrame(MakeFrame(1'000'000'000 + i * 10'000'000LL,
                               static_cast<double>(i)));
  }

  constexpr std::int64_t policy_now_ns = 1'900'000'000;
  EXPECT_TRUE(buffer.HasReadyWindow(policy_now_ns));

  std::array<float, a3_deploy::kA3TokenizerFloatsPerTick> out{};
  a3_deploy::A3TeleopTokenizerStatus status =
      a3_deploy::A3TeleopTokenizerStatus::kNoData;
  ASSERT_TRUE(buffer.BuildTokenizerSlice(policy_now_ns,
                                         {1.0, 0.0, 0.0, 0.0},
                                         0.0, out, &status));
  EXPECT_EQ(status, a3_deploy::A3TeleopTokenizerStatus::kRunning);

  const int last_mj = a3_mujoco_to_isaaclab[0];
  EXPECT_FLOAT_EQ(out[9 * kPolicyDof],
                  static_cast<float>(89.0 +
                                     static_cast<double>(last_mj) * 0.01));
}

TEST(A3TeleopReference, HoldsLastOnlyAfterRunningWhenStreamStale) {
  a3_deploy::A3TeleopReferenceOptions opt;
  opt.delay_ns = 900'000'000;
  opt.policy_hz = 10.0;
  opt.future_frame_skip = 1;
  a3_deploy::A3TeleopReferenceBuffer buffer(opt);

  for (int i = 0; i <= 9; ++i) {
    buffer.PushFrame(MakeFrame(1'000'000'000 + i * 100'000'000LL,
                               static_cast<double>(i)));
  }

  std::array<float, a3_deploy::kA3TokenizerFloatsPerTick> out{};
  a3_deploy::A3TeleopTokenizerStatus status =
      a3_deploy::A3TeleopTokenizerStatus::kNoData;
  ASSERT_TRUE(buffer.BuildTokenizerSlice(1'900'000'000,
                                         {1.0, 0.0, 0.0, 0.0},
                                         0.0, out, &status));
  EXPECT_EQ(status, a3_deploy::A3TeleopTokenizerStatus::kRunning);

  ASSERT_TRUE(buffer.BuildTokenizerSlice(2'200'000'000,
                                         {1.0, 0.0, 0.0, 0.0},
                                         0.0, out, &status));
  EXPECT_EQ(status, a3_deploy::A3TeleopTokenizerStatus::kRunning);
  for (std::size_t i_il = 0; i_il < kPolicyDof; ++i_il) {
    const int i_mj = a3_mujoco_to_isaaclab[i_il];
    const float expected_q =
        static_cast<float>(9.0 + static_cast<double>(i_mj) * 0.01);
    const float expected_dq =
        static_cast<float>(19.0 + static_cast<double>(i_mj) * 0.01);
    EXPECT_FLOAT_EQ(out[9 * kPolicyDof + i_il], expected_q);
    EXPECT_FLOAT_EQ(out[kCommandDofFloats + 9 * kPolicyDof + i_il],
                    expected_dq);
  }

  ASSERT_TRUE(buffer.BuildTokenizerSlice(3'500'000'000,
                                         {1.0, 0.0, 0.0, 0.0},
                                         0.0, out, &status));
  EXPECT_EQ(status, a3_deploy::A3TeleopTokenizerStatus::kRunning);
  for (std::size_t i_il = 0; i_il < kPolicyDof; ++i_il) {
    const int i_mj = a3_mujoco_to_isaaclab[i_il];
    const float expected_q =
        static_cast<float>(9.0 + static_cast<double>(i_mj) * 0.01);
    const float expected_dq =
        static_cast<float>(19.0 + static_cast<double>(i_mj) * 0.01);
    EXPECT_FLOAT_EQ(out[i_il], expected_q);
    EXPECT_FLOAT_EQ(out[kCommandDofFloats + i_il], expected_dq);
    EXPECT_FLOAT_EQ(out[9 * kPolicyDof + i_il], expected_q);
    EXPECT_FLOAT_EQ(out[kCommandDofFloats + 9 * kPolicyDof + i_il],
                    expected_dq);
  }
}

TEST(A3TeleopReference, LatestYawOffsetUsesFreshestFrame) {
  a3_deploy::A3TeleopReferenceOptions opt;
  opt.delay_ns = 900'000'000;
  opt.policy_hz = 10.0;
  opt.future_frame_skip = 1;
  a3_deploy::A3TeleopReferenceBuffer buffer(opt);

  buffer.PushFrame(MakeFrameWithYaw(1'000'000'000, 0.0, 0.0));
  buffer.PushFrame(MakeFrameWithYaw(1'900'000'000, 1.0, 45.0));

  const auto robot_yaw_90 = QuatFromYawDeg(90.0);
  EXPECT_NEAR(buffer.ComputeYawOffsetRad(1'900'000'000, robot_yaw_90),
              90.0 * kDegToRad, 1e-12);
  EXPECT_NEAR(buffer.ComputeLatestYawOffsetRad(robot_yaw_90),
              45.0 * kDegToRad, 1e-12);
}

TEST(A3TeleopReference, ReconnectOffsetAlignsLatestTaYawToRobotYaw) {
  a3_deploy::A3TeleopReferenceOptions opt;
  opt.delay_ns = 900'000'000;
  opt.policy_hz = 10.0;
  opt.future_frame_skip = 1;
  a3_deploy::A3TeleopReferenceBuffer buffer(opt);

  for (int i = 0; i <= 9; ++i) {
    buffer.PushFrame(MakeFrameWithYaw(1'000'000'000 + i * 100'000'000LL,
                                      static_cast<double>(i), 0.0));
  }

  std::array<float, a3_deploy::kA3TokenizerFloatsPerTick> out{};
  a3_deploy::A3TeleopTokenizerStatus status =
      a3_deploy::A3TeleopTokenizerStatus::kNoData;
  ASSERT_TRUE(buffer.BuildTokenizerSlice(1'900'000'000,
                                         QuatFromYawDeg(0.0),
                                         0.0, out, &status));
  EXPECT_EQ(status, a3_deploy::A3TeleopTokenizerStatus::kRunning);

  ASSERT_TRUE(buffer.BuildTokenizerSlice(3'500'000'000,
                                         QuatFromYawDeg(0.0),
                                         0.0, out, &status));
  EXPECT_EQ(status, a3_deploy::A3TeleopTokenizerStatus::kRunning);

  buffer.PushFrame(MakeFrameWithYaw(3'500'000'000, 30.0, 180.0));
  const double reconnect_offset =
      buffer.ComputeLatestYawOffsetRad(QuatFromYawDeg(0.0));
  EXPECT_NEAR(reconnect_offset, -180.0 * kDegToRad, 1e-12);
  ASSERT_TRUE(buffer.ResetToLatestFrame());
  for (int i = 1; i <= 9; ++i) {
    buffer.PushFrame(MakeFrameWithYaw(3'500'000'000 + i * 100'000'000LL,
                                      30.0 + static_cast<double>(i), 180.0));
  }
  ASSERT_TRUE(buffer.BuildTokenizerSlice(4'400'000'000,
                                         QuatFromYawDeg(0.0),
                                         reconnect_offset, out, &status));
  EXPECT_EQ(status, a3_deploy::A3TeleopTokenizerStatus::kRunning);

  const std::size_t o = a3_deploy::kA3CommandMultiFutureFloats + 9 * 6;
  EXPECT_NEAR(out[o + 0], 1.0f, 1e-5f);
  EXPECT_NEAR(out[o + 1], 0.0f, 1e-5f);
  EXPECT_NEAR(out[o + 2], 0.0f, 1e-5f);
  EXPECT_NEAR(out[o + 3], 1.0f, 1e-5f);
  EXPECT_NEAR(out[o + 4], 0.0f, 1e-5f);
  EXPECT_NEAR(out[o + 5], 0.0f, 1e-5f);
}

TEST(A3TeleopReference, ResetToLatestFrameBuffersUntilNewSegmentWindowExists) {
  a3_deploy::A3TeleopReferenceOptions opt;
  opt.delay_ns = 900'000'000;
  opt.policy_hz = 10.0;
  opt.future_frame_skip = 1;
  a3_deploy::A3TeleopReferenceBuffer buffer(opt);

  for (int i = 0; i <= 9; ++i) {
    buffer.PushFrame(MakeFrame(1'000'000'000 + i * 100'000'000LL,
                               static_cast<double>(i)));
  }

  std::array<float, a3_deploy::kA3TokenizerFloatsPerTick> out{};
  a3_deploy::A3TeleopTokenizerStatus status =
      a3_deploy::A3TeleopTokenizerStatus::kNoData;
  ASSERT_TRUE(buffer.BuildTokenizerSlice(1'900'000'000,
                                         {1.0, 0.0, 0.0, 0.0},
                                         0.0, out, &status));

  buffer.PushFrame(MakeFrame(3'500'000'000, 30.0));
  ASSERT_TRUE(buffer.ResetToLatestFrame());
  EXPECT_EQ(buffer.FrameCount(), 1u);
  EXPECT_FALSE(buffer.BuildTokenizerSlice(3'500'000'000,
                                          {1.0, 0.0, 0.0, 0.0},
                                          0.0, out, &status));
  EXPECT_EQ(status, a3_deploy::A3TeleopTokenizerStatus::kBuffering);

  buffer.PushFrame(MakeFrame(3'600'000'000, 31.0));
  ASSERT_TRUE(buffer.BuildTokenizerSlice(4'400'000'000,
                                         {1.0, 0.0, 0.0, 0.0},
                                         0.0, out, &status));
  EXPECT_EQ(status, a3_deploy::A3TeleopTokenizerStatus::kRunning);
  const int first_mj = a3_mujoco_to_isaaclab[0];
  EXPECT_FLOAT_EQ(out[0],
                  static_cast<float>(30.0 +
                                     static_cast<double>(first_mj) * 0.01));
}

#ifdef HAS_A3_TA_PROTO
TEST(A3TeleopReference, ConvertsTaWholeBodyCommandToPolicyOrder) {
  aimdk::protocol::TaWholeBodyCommandChannel msg;
  msg.mutable_header()->mutable_timestamp()->set_seconds(12);
  msg.mutable_header()->mutable_timestamp()->set_nanos(34);

  auto* data = msg.mutable_data();
  data->set_joint_layout(aimdk::protocol::TaJointLayout_BODY_30);
  for (double v : {1.0, 0.0, 0.0, 0.0}) {
    data->mutable_pelvis_pose()->add_quat_wxyz(v);
  }
  for (int i = 0; i < 12; ++i) {
    data->mutable_leg_command()->add_angles_rad(200.0 + i);
  }
  for (int i = 0; i < 3; ++i) {
    data->mutable_waist_command()->add_angles_rad(10.0 + i);
  }
  for (int i = 0; i < 14; ++i) {
    data->mutable_arm_command()->add_angles_rad(100.0 + i);
  }
  for (int i = 0; i < 30; ++i) {
    data->mutable_joint_velocities()->add_velocities_rad_s(300.0 + i);
  }

  a3_deploy::A3TeleopFrame frame{};
  std::string error;
  ASSERT_TRUE(a3_deploy::ConvertTaWholeBodyCommand(
      msg, 99, frame, &error)) << error;

  EXPECT_EQ(frame.stamp_ns, 12'000'000'034LL);
  EXPECT_DOUBLE_EQ(frame.q_mujoco[0], 10.0);
  EXPECT_DOUBLE_EQ(frame.q_mujoco[3], 100.0);
  EXPECT_DOUBLE_EQ(frame.q_mujoco[17], 200.0);
  EXPECT_DOUBLE_EQ(frame.dq_mujoco[0], 312.0);
  EXPECT_DOUBLE_EQ(frame.dq_mujoco[3], 316.0);
  EXPECT_DOUBLE_EQ(frame.dq_mujoco[17], 300.0);
}

TEST(A3TeleopReference, ConvertsTaWholeBodyCommandWithUrdf31VelocityOrder) {
  aimdk::protocol::TaWholeBodyCommandChannel msg;
  msg.mutable_header()->mutable_timestamp()->set_seconds(12);

  auto* data = msg.mutable_data();
  for (double v : {1.0, 0.0, 0.0, 0.0}) {
    data->mutable_pelvis_pose()->add_quat_wxyz(v);
  }
  for (int i = 0; i < 12; ++i) {
    data->mutable_leg_command()->add_angles_rad(200.0 + i);
  }
  for (int i = 0; i < 3; ++i) {
    data->mutable_waist_command()->add_angles_rad(10.0 + i);
  }
  for (int i = 0; i < 14; ++i) {
    data->mutable_arm_command()->add_angles_rad(100.0 + i);
  }
  for (int i = 0; i < 31; ++i) {
    data->mutable_joint_velocities()->add_velocities_rad_s(400.0 + i);
  }

  a3_deploy::A3TeleopFrame frame{};
  std::string error;
  ASSERT_TRUE(a3_deploy::ConvertTaWholeBodyCommand(
      msg, 99, frame, &error)) << error;

  EXPECT_DOUBLE_EQ(frame.dq_mujoco[0], 412.0);
  EXPECT_DOUBLE_EQ(frame.dq_mujoco[3], 417.0);
  EXPECT_DOUBLE_EQ(frame.dq_mujoco[17], 400.0);
}

TEST(A3TeleopReference, ConvertsTaWholeBodyCommandWithMissingVelocityAsZero) {
  aimdk::protocol::TaWholeBodyCommandChannel msg;
  msg.mutable_header()->mutable_timestamp()->set_seconds(12);

  auto* data = msg.mutable_data();
  for (double v : {1.0, 0.0, 0.0, 0.0}) {
    data->mutable_pelvis_pose()->add_quat_wxyz(v);
  }
  for (int i = 0; i < 12; ++i) {
    data->mutable_leg_command()->add_angles_rad(200.0 + i);
  }
  for (int i = 0; i < 3; ++i) {
    data->mutable_waist_command()->add_angles_rad(10.0 + i);
  }
  for (int i = 0; i < 14; ++i) {
    data->mutable_arm_command()->add_angles_rad(100.0 + i);
  }

  a3_deploy::A3TeleopFrame frame{};
  std::string error;
  ASSERT_TRUE(a3_deploy::ConvertTaWholeBodyCommand(
      msg, 99, frame, &error)) << error;

  for (double v : frame.dq_mujoco) {
    EXPECT_DOUBLE_EQ(v, 0.0);
  }
}
#endif
