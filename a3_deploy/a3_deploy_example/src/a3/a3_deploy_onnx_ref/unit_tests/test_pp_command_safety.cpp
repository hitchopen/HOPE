#include <gtest/gtest.h>

#include "a3_pingpong/pp_command_safety.hpp"

namespace a3_pingpong {
namespace {

TEST(PpCommandSafetyMonitor, AcceptsBoundedFinalCommand) {
  PpCommandSafetyMonitor monitor;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(31);
  monitor.Seed(q);
  q[0] = 0.06;
  q[5] = 0.15;
  EXPECT_NO_THROW(monitor.ValidateAndAdvance(
      q, Eigen::VectorXd::Zero(31), Eigen::VectorXd::Zero(31)));
}

TEST(PpCommandSafetyMonitor, RejectsWaistStepBeforePublish) {
  PpCommandSafetyMonitor monitor;
  monitor.Seed(Eigen::VectorXd::Zero(31));
  Eigen::VectorXd q_des = Eigen::VectorXd::Zero(31);
  q_des[0] = 0.071;
  EXPECT_THROW(
      monitor.ValidateAndAdvance(
          q_des, Eigen::VectorXd::Zero(31), Eigen::VectorXd::Zero(31)),
      std::runtime_error);
}

TEST(PpCommandSafetyMonitor, RejectsTrackingDebtAndVelocity) {
  PpCommandSafetyMonitor monitor;
  monitor.Seed(Eigen::VectorXd::Zero(31));
  Eigen::VectorXd q_des = Eigen::VectorXd::Zero(31);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(31);
  q[0] = -0.50;
  EXPECT_THROW(
      monitor.ValidateAndAdvance(q_des, q, Eigen::VectorXd::Zero(31)),
      std::runtime_error);

  monitor.Seed(Eigen::VectorXd::Zero(31));
  Eigen::VectorXd dq = Eigen::VectorXd::Zero(31);
  dq[5] = 12.1;
  EXPECT_THROW(
      monitor.ValidateAndAdvance(
          Eigen::VectorXd::Zero(31), Eigen::VectorXd::Zero(31), dq),
      std::runtime_error);
}

}  // namespace
}  // namespace a3_pingpong
