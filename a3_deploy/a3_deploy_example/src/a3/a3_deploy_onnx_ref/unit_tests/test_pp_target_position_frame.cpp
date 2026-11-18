#include <gtest/gtest.h>
#include <limits>
#include "a3_pingpong/pp_obs_builder.hpp"

namespace a3_pingpong {
namespace {
PpRobotState pose() {
  PpRobotState s{};
  s.base_pos_w = Vec3(2.0, -3.0, 0.9);
  s.base_quat_w = Vec4(1, 0, 0, 0);
  s.q = s.qd = Eigen::VectorXd::Zero(kNumJoints);
  s.base_ang_vel_b.setZero();
  return s;
}
Vec4 quat(const Eigen::Quaterniond& q) { return Vec4(q.w(), q.x(), q.y(), q.z()); }
Eigen::VectorXd observation(const PpRobotState& s, const PpRacketTarget& t) {
  return build_obs_110(s, t, Eigen::VectorXd::Zero(kNumJoints),
                       Eigen::VectorXd::Zero(kNumJoints));
}

TEST(PpTargetFrame, ExplicitConfigAndLegacyDefault) {
  EXPECT_EQ(parse_target_frame("training"), PpTargetFrame::kTraining);
  EXPECT_EQ(parse_target_frame("body"), PpTargetFrame::kBody);
  EXPECT_EQ(parse_target_frame("body_once"), PpTargetFrame::kBodyOnce);
  EXPECT_THROW(parse_target_frame("boddy"), std::invalid_argument);
  auto s = pose();
  PpRacketTarget t{}; t.vel_w.setZero(); t.pos_w = Vec3(4, 5, 6);
  auto obs = observation(s, t); const auto original = obs;
  apply_target_frame(obs, s, t, PpTargetFrame::kTraining);
  EXPECT_TRUE(obs == original);
}

TEST(PpTargetFrame, TranslationThenInverseYawTransformsPositionAndVelocity) {
  auto s = pose(); s.base_quat_w = quat(Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Vec3::UnitZ())));
  PpRacketTarget t{}; t.vel_w.setZero();
  t.pos_w = s.base_pos_w + Vec3(1, 2, 3);
  t.vel_w = Vec3(7, 8, 9); t.time_to_strike = 0.41;
  auto obs = observation(s, t); const auto original = obs;
  apply_target_frame(obs, s, t, PpTargetFrame::kBody);
  EXPECT_TRUE(obs.segment<3>(103).isApprox(Vec3(2, -1, 3), 1e-12));
  EXPECT_TRUE(obs.segment<3>(106).isApprox(Vec3(8, -7, 9), 1e-12));
  for (int i = 0; i < obs.size(); ++i)
    if (i < 103 || i >= 109) EXPECT_DOUBLE_EQ(obs[i], original[i]);
}

TEST(PpTargetFrame, FullRollPitchAndQuaternionSignScale) {
  auto s = pose();
  PpRacketTarget t{}; t.vel_w.setZero(); t.pos_w = s.base_pos_w + Vec3(1, 2, 3);
  for (const auto& axis : {Vec3::UnitX().eval(), Vec3::UnitY().eval()}) {
    Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 2, axis));
    s.base_quat_w = -2.0 * quat(q);
    auto obs = observation(s, t);
    apply_target_frame(obs, s, t, PpTargetFrame::kBody);
    Vec3 expected = axis[0] == 1 ? Vec3(1, 3, -2) : Vec3(-3, 2, 1);
    EXPECT_TRUE(obs.segment<3>(103).isApprox(expected, 1e-12));
  }
}

TEST(PpTargetFrame, SameBodyTargetAcrossChangingWorldPoses) {
  auto s = pose(); PpRacketTarget t{}; t.vel_w.setZero();
  const Vec3 body_target(0.57, -0.35, 0.18);
  const Vec3 body_velocity(1.5, -0.7, 0.3);
  const Vec3 body_normal = Vec3(1, 2, 3).normalized();
  for (int i = 0; i < 100; ++i) {
    Eigen::Quaterniond q = Eigen::AngleAxisd(i * .031, Vec3::UnitZ()) *
        Eigen::AngleAxisd(i * -.007, Vec3::UnitY()) *
        Eigen::AngleAxisd(i * .005, Vec3::UnitX());
    s.base_quat_w = quat(q); s.base_pos_w = Vec3(i * .07, i * -.04, .9);
    t.pos_w = s.base_pos_w + q * body_target;
    t.vel_w = q * body_velocity;
    t.normal_w = q * body_normal;
    s.base_velocity_xy_w = Vec2(10, -20);  // coordinate expression, not relative motion
    auto obs = observation(s, t);
    apply_target_frame(obs, s, t, PpTargetFrame::kBody);
    EXPECT_TRUE(obs.segment<3>(103).isApprox(body_target, 1e-12));
    EXPECT_TRUE(obs.segment<3>(106).isApprox(body_velocity, 1e-12));
    const auto body = racket_target_to_body(s, t);
    ASSERT_TRUE(body.normal_b.has_value());
    EXPECT_TRUE(body.normal_b->isApprox(body_normal, 1e-12));
    EXPECT_NEAR(body.vel_b.norm(), t.vel_w.norm(), 1e-12);
  }
}

TEST(PpTargetFrame, NormalIsOptionalNormalizedAndDoesNotEnterActor) {
  auto s = pose(); s.base_quat_w = quat(Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Vec3::UnitZ())));
  PpRacketTarget t{}; t.pos_w = Vec3(2, -2, 1); t.vel_w = Vec3(1, 2, 3);
  t.time_to_strike = .37;
  EXPECT_FALSE(racket_target_to_body(s, t).normal_b.has_value());
  auto without_normal = observation(s, t);
  apply_target_frame(without_normal, s, t, PpTargetFrame::kBody);
  t.normal_w = Vec3(4, 0, 0);
  const auto body = racket_target_to_body(s, t);
  ASSERT_TRUE(body.normal_b.has_value());
  EXPECT_TRUE(body.normal_b->isApprox(Vec3(0, -1, 0), 1e-12));
  auto with_normal = observation(s, t);
  apply_target_frame(with_normal, s, t, PpTargetFrame::kBody);
  EXPECT_TRUE(with_normal == without_normal);
  EXPECT_DOUBLE_EQ(with_normal[109], .37);
}

TEST(PpTargetFrame, RejectsInvalidVelocityAndPresentInvalidNormal) {
  auto s = pose(); PpRacketTarget t{}; t.pos_w = Vec3::Zero(); t.vel_w = Vec3::Zero();
  t.vel_w[1] = std::numeric_limits<double>::infinity();
  EXPECT_THROW(racket_target_to_body(s, t), std::invalid_argument);
  t.vel_w.setZero(); t.normal_w = Vec3::Zero();
  EXPECT_THROW(racket_target_to_body(s, t), std::invalid_argument);
  t.normal_w = Vec3(1, 0, std::numeric_limits<double>::quiet_NaN());
  EXPECT_THROW(racket_target_to_body(s, t), std::invalid_argument);
}

TEST(PpTargetFrame, RejectsInvalidPoseAndWrongLayout) {
  auto s = pose(); PpRacketTarget t{}; t.vel_w.setZero(); t.pos_w.setZero();
  auto obs = observation(s, t); s.base_quat_w.setZero();
  EXPECT_THROW(apply_target_frame(obs, s, t, PpTargetFrame::kBody), std::invalid_argument);
  s = pose(); s.base_pos_w[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(apply_target_frame(obs, s, t, PpTargetFrame::kBody), std::invalid_argument);
  s = pose(); obs = Eigen::VectorXd::Zero(112);
  EXPECT_THROW(apply_target_frame(obs, s, t, PpTargetFrame::kBody), std::invalid_argument);
}
TEST(PpTargetFrame, CommitFreezesVectorsWhilePoseAndTimeChange) {
  auto s = pose();
  s.base_quat_w = quat(Eigen::Quaterniond(
      Eigen::AngleAxisd(.2, Vec3::UnitZ()) *
      Eigen::AngleAxisd(.1, Vec3::UnitY())));
  PpRacketTarget t{};
  t.pos_w = s.base_pos_w + Vec3(.57, -.2, .15);
  t.vel_w = Vec3(2, .3, -.1);
  t.normal_w = Vec3(1, 0, 0);
  PpCommittedBodyTarget latch;
  auto obs = observation(s, t);
  ASSERT_TRUE(latch.Apply(obs, s, t, true, 1));
  const Vec3 position = obs.segment<3>(103), velocity = obs.segment<3>(106);
  const Vec3 normal = *latch.body()->normal_b;
  const Mat3 inverse = mat_from_quat(s.base_quat_w).transpose();
  EXPECT_TRUE(position.isApprox(inverse * (t.pos_w - s.base_pos_w), 1e-12));
  EXPECT_TRUE(velocity.isApprox(inverse * t.vel_w, 1e-12));
  EXPECT_TRUE(normal.isApprox(inverse * *t.normal_w, 1e-12));
  for (int i = 0; i < 30; ++i) {
    s.base_pos_w += Vec3(.003, -.001, .002);
    s.base_quat_w = quat(Eigen::Quaterniond(Eigen::AngleAxisd(.3 + i*.01, Vec3::UnitZ())));
    t.time_to_strike = .4 - i*.02;
    obs = observation(s, t);
    const auto before = obs;
    EXPECT_FALSE(latch.Apply(obs, s, t, true, 1));
    EXPECT_TRUE(obs.segment<3>(103) == position);
    EXPECT_TRUE(obs.segment<3>(106) == velocity);
    EXPECT_TRUE(*latch.body()->normal_b == normal);
    for (int k = 0; k < obs.size(); ++k)
      if (k < 103 || k >= 109) EXPECT_DOUBLE_EQ(obs[k], before[k]);
  }
}

TEST(PpTargetFrame, NewShotRecapturesWithoutWaitAndSameShotDoesNot) {
  auto s = pose(); PpRacketTarget t{};
  t.pos_w = s.base_pos_w + Vec3(1, 2, 3); t.vel_w = Vec3(4, 5, 6);
  PpCommittedBodyTarget latch; auto obs = observation(s, t);
  ASSERT_TRUE(latch.Apply(obs, s, t, true, 10));
  const auto first = obs;
  s.base_quat_w = quat(Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2, Vec3::UnitZ())));
  t.pos_w += Vec3(.1, 0, 0); t.vel_w += Vec3(0, .2, 0);
  EXPECT_FALSE(latch.Apply(obs, s, t, true, 10));
  EXPECT_TRUE(obs == first);
  ASSERT_TRUE(latch.Apply(obs, s, t, true, 11));
  EXPECT_TRUE(obs.segment<3>(103).isApprox(Vec3(2, -1.1, 3), 1e-12));
  EXPECT_TRUE(obs.segment<3>(106).isApprox(Vec3(5.2, -4, 6), 1e-12));
}

TEST(PpTargetFrame, WaitAndModeResetClearCachedShot) {
  auto s = pose(); PpRacketTarget t{}; t.pos_w = Vec3(4, 5, 6); t.vel_w.setZero();
  PpCommittedBodyTarget latch; auto obs = observation(s, t);
  ASSERT_TRUE(latch.Apply(obs, s, t, true, 7));
  const Eigen::VectorXd wait = Eigen::VectorXd::Constant(110, .123);
  obs = wait;
  EXPECT_FALSE(latch.Apply(obs, s, t, false, 7));
  EXPECT_TRUE(obs == wait); EXPECT_FALSE(latch.body().has_value());
  s.base_pos_w[0] += .2;
  EXPECT_TRUE(latch.Apply(obs, s, t, true, 7));
  EXPECT_TRUE(obs.segment<3>(103).isApprox(t.pos_w-s.base_pos_w, 1e-12));
  latch.Reset(); EXPECT_FALSE(latch.body().has_value());
  EXPECT_TRUE(latch.Apply(obs, s, t, true, 7));
}

TEST(PpTargetFrame, InvalidNewShotDoesNotPartiallyCommit) {
  auto s = pose(); PpRacketTarget t{}; t.pos_w.setZero(); t.vel_w.setZero();
  PpCommittedBodyTarget latch; auto obs = observation(s, t);
  ASSERT_TRUE(latch.Apply(obs, s, t, true, 1));
  const auto old = obs;
  s.base_quat_w.setZero();
  EXPECT_THROW(latch.Apply(obs, s, t, true, 2), std::invalid_argument);
  EXPECT_TRUE(obs == old);
  s = pose(); EXPECT_TRUE(latch.Apply(obs, s, t, true, 2));
  EXPECT_THROW(apply_target_frame(obs, s, t, PpTargetFrame::kBodyOnce), std::invalid_argument);
}
TEST(PpTargetFrame, FixedHeadingPreservesRealTiltAndCommitTransformInAllPhases) {
  for (bool active : {false, true}) {
    for (double yaw : {-.8, -.2, .0, .3, 1.5}) {
      auto s = pose();
      s.base_quat_w = quat(Eigen::Quaterniond(
          Eigen::AngleAxisd(yaw, Vec3::UnitZ()) *
          Eigen::AngleAxisd(.12, Vec3::UnitY()) *
          Eigen::AngleAxisd(-.08, Vec3::UnitX())));
      s.base_ang_vel_b = Vec3(.1, -.2, .3);
      const Vec4 real_quaternion = s.base_quat_w;
      PpRacketTarget t{}; t.pos_w = s.base_pos_w + Vec3(.57, -.2, .1);
      t.vel_w = Vec3(2, -.3, .1); t.time_to_strike = .28;
      PpCommittedBodyTarget latch;
      auto obs = observation(s, t);
      latch.Apply(obs, s, t, active, 1);
      const auto before_heading = obs;
      apply_forward_heading_110(obs);
      EXPECT_DOUBLE_EQ(obs[99], 1.0); EXPECT_DOUBLE_EQ(obs[100], 0.0);
      for (int k = 0; k < obs.size(); ++k)
        if (k != 99 && k != 100) EXPECT_DOUBLE_EQ(obs[k], before_heading[k]);
      EXPECT_TRUE(s.base_quat_w == real_quaternion);
      EXPECT_TRUE(obs.segment<3>(96).isApprox(projected_gravity_body(real_quaternion), 1e-12));
      EXPECT_TRUE(obs.segment<3>(0) == s.base_ang_vel_b);
      if (active) {
        const Mat3 inverse = mat_from_quat(real_quaternion).transpose();
        EXPECT_TRUE(obs.segment<3>(103).isApprox(inverse * (t.pos_w-s.base_pos_w), 1e-12));
        EXPECT_TRUE(obs.segment<3>(106).isApprox(inverse * t.vel_w, 1e-12));
      }
    }
  }
}

TEST(PpTargetFrame, FixedHeadingRejectsOtherObservationLayouts) {
  Eigen::VectorXd obs = Eigen::VectorXd::Zero(112);
  EXPECT_THROW(apply_forward_heading_110(obs), std::invalid_argument);
}
}  // namespace
}  // namespace a3_pingpong
