#include "a3_pingpong/pp_command_transition.hpp"
#include <gtest/gtest.h>
#include <limits>

namespace {
robot_io::RobotCommand Command(double q, double dq, double kp) {
  robot_io::RobotCommand c;
  c.q_des = Eigen::VectorXd::Constant(31, q);
  c.dq_des = Eigen::VectorXd::Constant(31, dq);
  c.kp = Eigen::VectorXd::Constant(31, kp);
  c.kd = Eigen::VectorXd::Constant(31, kp / 10);
  c.tau_ff = Eigen::VectorXd::Constant(31, kp / 20);
  return c;
}

TEST(PpCommandTransition, EntryPreservesEntireLastDeliveredCommand) {
  const auto source = Command(.2, .3, 40);
  auto target = Command(-.4, -.2, 200);
  a3_pingpong::PpCommandTransition t;
  t.Begin(source, target, .8);
  t.Apply(0, target);
  EXPECT_TRUE(target.q_des.isApprox(source.q_des));
  EXPECT_TRUE(target.dq_des.isApprox(source.dq_des));
  EXPECT_TRUE(target.kp.isApprox(source.kp));
  EXPECT_TRUE(target.kd.isApprox(source.kd));
  EXPECT_TRUE(target.tau_ff.isApprox(source.tau_ff));
}

TEST(PpCommandTransition, VelocityMatchesDerivativeForMovingDestination) {
  a3_pingpong::PpCommandTransition t;
  const auto source = Command(.2, .3, 40);
  const auto initial = Command(-.4, -.2, 200);
  t.Begin(source, initial, .8);
  auto sample = [&](double seconds) {
    auto c = initial;
    c.q_des.array() += seconds * -.2;
    t.Apply(seconds, c);
    return c;
  };
  for (double s : {.0001, .1, .4, .7999}) {
    const auto a = sample(s - 1e-6), b = sample(s + 1e-6), c = sample(s);
    EXPECT_NEAR((b.q_des[0] - a.q_des[0]) / 2e-6, c.dq_des[0], 1e-7);
    EXPECT_GE(c.kp[0], 40);
    EXPECT_LE(c.kp[0], 200);
  }
  auto end = sample(.8);
  EXPECT_DOUBLE_EQ(end.q_des[0], -.4 - .8 * .2);
  EXPECT_DOUBLE_EQ(end.dq_des[0], -.2);
  EXPECT_DOUBLE_EQ(end.kp[0], 200);
  EXPECT_FALSE(t.active());
}

TEST(PpCommandTransition, InterruptedHandoffSeedsFromCurrentCommand) {
  a3_pingpong::PpCommandTransition t;
  auto source = Command(.2, .3, 40), target = Command(-.4, 0, 200);
  t.Begin(source, target, 1);
  t.Apply(.3, target);
  const auto interrupted = target;
  auto next = Command(.1, 0, 90);
  t.Begin(interrupted, next, .5);
  t.Apply(0, next);
  EXPECT_TRUE(next.q_des.isApprox(interrupted.q_des));
  EXPECT_TRUE(next.dq_des.isApprox(interrupted.dq_des));
  EXPECT_TRUE(next.kp.isApprox(interrupted.kp));
  t.Reset();
  next = source;
  t.Apply(.1, next);
  EXPECT_TRUE(next.q_des.isApprox(source.q_des));
}

TEST(PpCommandTransition, AttenuatesChangingPolicyTargetsDuringEntry) {
  a3_pingpong::PpCommandTransition t;
  auto source = Command(0, 0, 40), initial = source;
  t.Begin(source, initial, 1);
  auto changed = Command(1, 0, 100);
  t.Apply(.1, changed);
  EXPECT_NEAR(changed.q_des[0], .00856, 1e-12);
  EXPECT_NEAR(changed.dq_des[0], .243, 1e-12);
}

TEST(PpCommandTransition, ImpedanceHandoffBlendsActualPdTorqueAtEveryState) {
  const auto source = Command(0, .2, 500);
  auto destination = Command(.35, -.1, 50);
  a3_pingpong::PpCommandTransition transition;
  transition.BeginImpedance(source, destination, .5);
  for (double t : {0., .02, .1, .2, .3, .48, .5}) {
    auto target = destination;
    target.q_des.array() += .1 * t;  // live actor keeps updating during handoff
    const auto original = target;
    transition.Apply(t, target);
    const double u = t / .5;
    const double w = u*u*u * (10 + u*(-15 + 6*u));
    for (double q : {-.2, 0., .4}) {
      for (double dq : {-2., 0., 1.}) {
        const auto torque = [q,dq](const robot_io::RobotCommand& c) {
          return c.kp[0]*(c.q_des[0]-q) + c.kd[0]*(c.dq_des[0]-dq) + c.tau_ff[0];
        };
        EXPECT_NEAR(torque(target), (1-w)*torque(source)+w*torque(original), 1e-10);
      }
    }
  }
  EXPECT_FALSE(transition.active());
}

TEST(PpCommandTransition, ImpedanceHandoffWithZeroGainsStaysFiniteAndContinuous) {
  const auto source = Command(.2, .3, 0);
  auto target = Command(-.4, -.2, 0);
  a3_pingpong::PpCommandTransition transition;
  transition.BeginImpedance(source, target, .5);
  transition.Apply(.25, target);
  EXPECT_TRUE(a3_pingpong::PpCommandTransition::Valid(target));
  EXPECT_NEAR(target.q_des[0], -.1, 1e-12);
  EXPECT_NEAR(target.dq_des[0], .05, 1e-12);
}

TEST(PpCommandTransition, LongPoseBlendSettlesEntryVelocityWithoutLongExtrapolation) {
  a3_pingpong::PpCommandTransition t;
  const auto source = Command(.2, 2., 40), target = Command(.2, 0., 200);
  t.Begin(source, target, 5., .25);
  auto sample = [&](double seconds) {
    auto c = target;
    t.Apply(seconds, c);
    return c;
  };
  EXPECT_DOUBLE_EQ(sample(0).dq_des[0], 2.);
  for (double s : {.0001, .05, .125, .2499, .2501, 1.}) {
    const auto a = sample(s - 1e-6), b = sample(s + 1e-6), c = sample(s);
    EXPECT_NEAR((b.q_des[0] - a.q_des[0]) / 2e-6, c.dq_des[0], 1e-7);
    EXPECT_LT(std::abs(c.q_des[0] - .2), .1);
  }
  EXPECT_DOUBLE_EQ(sample(.25).dq_des[0], 0.);
  EXPECT_NEAR(sample(.25).q_des[0], .2, 1e-12);
  EXPECT_TRUE(t.active());  // pose/gain transition continues for 5 seconds
  EXPECT_EQ(sample(5).kp[0], 200.);
}

TEST(PpCommandTransition, RejectsNonFiniteOrMalformedInputs) {
  a3_pingpong::PpCommandTransition t;
  auto source = Command(0, 0, 40), target = source;
  EXPECT_THROW(t.Begin(source, target, 0), std::invalid_argument);
  target.kp[0] = -1;
  EXPECT_THROW(t.Begin(source, target, 1), std::invalid_argument);
  target = source;
  target.dq_des.resize(30);
  EXPECT_THROW(t.Begin(source, target, 1), std::invalid_argument);
  target = source;
  t.Begin(source, target, 1);
  EXPECT_THROW(t.Apply(std::numeric_limits<double>::quiet_NaN(), target), std::invalid_argument);
}
} // namespace
