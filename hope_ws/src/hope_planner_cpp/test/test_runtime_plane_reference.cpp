#include "hope_planner_cpp/runtime_plane_reference.hpp"
#include <gtest/gtest.h>
using namespace hope_planner_cpp;
TEST(RuntimePlaneReference, RunningSwitchWaitsForNextFlightAndFreezesCurrentPose) {
  RuntimePlaneReference plane;
  const auto first = plane.select(10, 1, -.5, .58, -.4, true, true, true);
  ASSERT_TRUE(first.valid); EXPECT_NEAR(first.plane_x, .08, 1e-10);
  plane.commit(first);
  plane.request(PlaneMode::PerBall);
  const auto same = plane.select(10, 1, -.5, .58, -.35, true, true, true);
  EXPECT_EQ(same.mode, PlaneMode::Fixed); EXPECT_DOUBLE_EQ(same.plane_x, first.plane_x);
  auto next = plane.select(10, 2, -.5, .58, -.4, true, true, false);
  EXPECT_EQ(next.mode, PlaneMode::PerBall); EXPECT_NEAR(next.plane_x, .18, 1e-10);
  EXPECT_NEAR(plane.select(10, 2, -.5, .58, -.3, true, true, true).plane_x, .18, 1e-10);
  plane.commit(next);
  plane.request(PlaneMode::StablePerBall);
  auto held = plane.select(10, 3, -.5, .58, -.3, true, true, false);
  EXPECT_NEAR(held.plane_x, .18, 1e-10); EXPECT_EQ(held.reason, "posture_unstable_keep_previous");
  plane.commit(held);
  auto stable = plane.select(10, 4, -.5, .58, -.3, true, true, true);
  EXPECT_NEAR(stable.plane_x, .28, 1e-10);
  plane.commit(stable);
  plane.request(PlaneMode::Fixed);
  EXPECT_NEAR(plane.select(10, 5, -.5, .58, -.3, true, true, true).plane_x, .08, 1e-10);
}
TEST(RuntimePlaneReference, RejectsStalePoseUnsupportedRunnerAndUsesEachModelsOffset) {
  RuntimePlaneReference plane; plane.request(PlaneMode::PerBall);
  EXPECT_FALSE(plane.select(1, 1, -.5, .58, -.4, false, true, true).valid);
  EXPECT_FALSE(plane.select(1, 2, -.5, .58, -.4, true, false, true).valid);
  EXPECT_NEAR(plane.select(2, 1, -.7, .62, -.6, true, true, true).plane_x, .02, 1e-10);
  plane.request(PlaneMode::StablePerBall);
  EXPECT_NEAR(plane.select(3, 1, -.7, .62, -.4, true, true, false).plane_x, -.08, 1e-10);
  EXPECT_THROW(parse_plane_mode("continuous"), std::invalid_argument);
}
