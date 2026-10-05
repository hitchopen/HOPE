#include "a3_pingpong/pp_humanlike_contract.hpp"
#include <gtest/gtest.h>

TEST(HumanLikeStop, ParksAfterHalfStepWithoutWaitingForFeetAlignment) {
  a3_pingpong::HumanLikeGaitParameters p;
  p.finish_step_on_stop = true;
  a3_pingpong::HumanLikeGait runner(p), original;
  const Eigen::Vector4d upright(1, 0, 0, 0);
  const Eigen::Vector3d left(.12, .125, -.9), right(-.12, -.125, -.9);
  for (int i = 0; i < 150; ++i) {
    runner.Update({-.2, 0, 0}, upright, left, right);
    original.Update({-.2, 0, 0}, upright, left, right);
  }
  for (int i = 0; i < 60; ++i) {
    runner.Update(Eigen::Vector3d::Zero(), upright, left, right);
    original.Update(Eigen::Vector3d::Zero(), upright, left, right);
  }
  EXPECT_TRUE(runner.settled());
  EXPECT_FALSE(original.settled());
  const auto parked = runner.features();
  for (int i = 0; i < 100; ++i)
    EXPECT_EQ(runner.Update(Eigen::Vector3d::Zero(), upright, left, right), parked);
  runner.Update({-.2, 0, 0}, upright, left, right);
  EXPECT_FALSE(runner.settled());
}

TEST(HumanLikeStop, DoesNotParkWhileTilted) {
  a3_pingpong::HumanLikeGaitParameters p;
  p.finish_step_on_stop = true;
  a3_pingpong::HumanLikeGait gait(p);
  const Eigen::Vector4d tilted(std::cos(.1), std::sin(.1), 0, 0);
  for (int i = 0; i < 100; ++i)
    gait.Update(Eigen::Vector3d::Zero(), tilted, {0, .125, -.9}, {0, -.125, -.9});
  EXPECT_FALSE(gait.settled());
}
