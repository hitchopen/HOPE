#include <gtest/gtest.h>
#include "a3_pingpong/pp_footstep_task.hpp"
using namespace a3_pingpong;

TEST(ObservableFootstepTask, AnkleVelocityMatchesMovingBaseEncoderFiniteDifference) {
  constexpr double dt = 1e-7;
  for (int sample = 0; sample < 32; ++sample) {
    PpRobotState s;
    s.base_pos_w = Vec3(.1, -.2, 1.);
    const Eigen::Quaterniond orientation(Eigen::AngleAxisd(.01 * sample, Vec3(1., 2., 3.).normalized()));
    s.base_quat_w = Vec4(orientation.w(), orientation.x(), orientation.y(), orientation.z());
    s.base_ang_vel_b = Vec3(.2, -.3, .4);
    s.base_velocity_xy_w = Vec2(.13, -.21);
    s.q = Eigen::VectorXd::Zero(31); s.qd = s.q;
    for (int i = 0; i < 31; ++i) {
      s.q[i] = .2 * std::sin(i + .3 * sample);
      s.qd[i] = .4 * std::cos(i + .2 * sample);
    }
    const auto current = footstep_kinematics(s);
    PpRobotState next = s;
    next.q += dt * s.qd;
    next.base_pos_w.head<2>() += dt * s.base_velocity_xy_w;
    const Eigen::Quaterniond rotated(orientation.toRotationMatrix() * Eigen::AngleAxisd(
        dt * s.base_ang_vel_b.norm(), s.base_ang_vel_b.normalized()).toRotationMatrix());
    next.base_quat_w = Vec4(rotated.w(), rotated.x(), rotated.y(), rotated.z());
    const auto future = footstep_kinematics(next);
    for (int foot = 0; foot < 2; ++foot)
      EXPECT_NEAR(current.speed[foot], ((future.xy[foot] - current.xy[foot]) / dt).norm(), 1e-6);
  }
}

TEST(ObservableFootstepTask, SlidingAndTokenLiftCannotDischargeTask) {
  ObservableFootstepTask task;
  FootstepKinematics k;
  k.xy = {Vec2(0., .134), Vec2(0., -.134)};
  auto step = [&] { task.Step(k, Vec2::Zero(), 0., 2., 1., .7, .02); };
  step();
  k.xy[0][1] += .06; step();
  k.minimum_height[0] = .035; step();
  EXPECT_DOUBLE_EQ(task.signed_phase(), .25);
  EXPECT_NEAR(task.errors(k)[1], -.06, 1e-12);
}

TEST(ObservableFootstepTask, NewBallRetainsTaskAndCompleteLiftLandReturnsHome) {
  ObservableFootstepTask task;
  FootstepKinematics k;
  k.xy = {Vec2(0., .134), Vec2(0., -.134)};
  task.Step(k, Vec2::Zero(), 0., 2., 1., .7, .02);
  task.Step(k, Vec2::Zero(), 0., 2., -1., .6, .02);
  EXPECT_DOUBLE_EQ(task.signed_phase(), .25);
  EXPECT_NEAR(task.errors(k)[3], .06, 1e-12);
  k.minimum_height[0] = .035;
  task.Step(k, Vec2::Zero(), 0., 0., 0., 1., .02);
  EXPECT_DOUBLE_EQ(task.signed_phase(), .5);
  k.xy[0][1] += .06;
  task.Step(k, Vec2::Zero(), 0., 0., 0., 1., .02);
  EXPECT_DOUBLE_EQ(task.signed_phase(), .75);
  k.minimum_height[0] = .003;
  for (int i = 0; i < 4; ++i) task.Step(k, Vec2::Zero(), 0., 0., 0., 1., .02);
  EXPECT_DOUBLE_EQ(task.signed_phase(), 0.);
  task.Step(k, Vec2::Zero(), 0., 0., 0., 1., .02);
  EXPECT_DOUBLE_EQ(task.signed_phase(), .25);
  EXPECT_NEAR(task.errors(k)[3], -.06, 1e-12);
}

TEST(ObservableFootstepTask, ObservationReassignsOnlyDeclaredPassiveHeadSlots) {
  PpRobotState s;
  s.base_pos_w = Vec3(0, 0, 1); s.base_quat_w = Vec4(1, 0, 0, 0);
  s.base_ang_vel_b.setZero(); s.q = Eigen::VectorXd::Zero(31); s.qd = s.q;
  PpRacketTarget t; t.pos_w = Vec3(1, .2, 1.1); t.vel_w = Vec3(2, 0, 0);
  t.time_to_strike = .6; t.signed_external_reach = 2.; t.signed_replant_need = .5;
  t.footstep_task_errors << .01, .02, .03, .04;
  const auto old = build_obs_112_headslots_vxy_signed_support(s, t, s.q, s.q);
  const auto obs = build_obs_112_headslots_step_task(s, t, s.q, s.q);
  for (int i = 0; i < 112; ++i) {
    if (i == 14) EXPECT_DOUBLE_EQ(obs[i], .01);
    else if (i == 19) EXPECT_DOUBLE_EQ(obs[i], .02);
    else if (i == 45) EXPECT_DOUBLE_EQ(obs[i], .03);
    else if (i == 50) EXPECT_DOUBLE_EQ(obs[i], .04);
    else EXPECT_DOUBLE_EQ(obs[i], old[i]);
  }
}
