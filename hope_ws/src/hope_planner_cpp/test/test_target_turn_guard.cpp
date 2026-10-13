#include "hope_planner_cpp/target_turn_guard.hpp"
#include <gtest/gtest.h>

namespace hope_planner_cpp {
namespace {
constexpr double rad = 3.14159265358979323846 / 180.0;
Eigen::Quaterniond yaw(double degrees) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(degrees * rad, Vec3::UnitZ()));
}
Vec3 point(double degrees) { return Vec3(std::cos(degrees * rad), std::sin(degrees * rad), 1.0); }
TEST(TargetTurnGuard, SymmetricInclusiveBoundaryAndBehind) {
  for (double angle : {-40.0, 0.0, 40.0}) {
    auto r = check_target_turn(point(angle), Vec3::Zero(), yaw(0), true);
    EXPECT_TRUE(r.allowed);
    EXPECT_NEAR(r.turn_deg, angle, 1e-10);
  }
  for (double angle : {-180.0, -40.00001, 40.00001, 180.0}) {
    auto r = check_target_turn(point(angle), Vec3::Zero(), yaw(0), true);
    EXPECT_FALSE(r.allowed);
    EXPECT_STREQ(r.reason, "target_turn_exceeds_limit");
  }
}
TEST(TargetTurnGuard, UsesCurrentHeadingAndShortestWrappedAngle) {
  EXPECT_NEAR(check_target_turn(point(-179), Vec3::Zero(), yaw(179), true).turn_deg, 2, 1e-10);
  EXPECT_NEAR(check_target_turn(point(179), Vec3::Zero(), yaw(-179), true).turn_deg, -2, 1e-10);
  EXPECT_TRUE(check_target_turn(point(100), Vec3::Zero(), yaw(70), true).allowed);
  EXPECT_FALSE(check_target_turn(point(0), Vec3::Zero(), yaw(70), true).allowed);
}
TEST(TargetTurnGuard, CommonWorldTransformAndQuaternionSignPreserveDecision) {
  const Vec3 base(2.3, -0.8, 0.76);
  const auto rotate = yaw(-90);
  auto q = rotate * yaw(35);
  q.coeffs() *= -1.1;
  auto r = check_target_turn(base + rotate * point(70), base, q, true);
  EXPECT_TRUE(r.allowed);
  EXPECT_NEAR(r.turn_deg, 35.0, 1e-10);
}
TEST(TargetTurnGuard, MissingInvalidAndUndefinedDirectionAreRejected) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(check_target_turn(point(0), Vec3::Zero(), yaw(0), false).allowed);
  EXPECT_FALSE(check_target_turn(point(0), Vec3(nan,0,0), yaw(0), true).allowed);
  EXPECT_FALSE(check_target_turn(Vec3(nan,0,0), Vec3::Zero(), yaw(0), true).allowed);
  EXPECT_FALSE(check_target_turn(point(0), Vec3::Zero(), Eigen::Quaterniond(0,0,0,0), true).allowed);
  EXPECT_FALSE(check_target_turn(point(0), Vec3::Zero(), Eigen::Quaterniond(nan,0,0,0), true).allowed);
  EXPECT_FALSE(check_target_turn(Vec3(0,0,1), Vec3::Zero(), yaw(0), true).allowed);
  EXPECT_FALSE(check_target_turn(point(0), Vec3::Zero(), Eigen::Quaterniond(Eigen::AngleAxisd(90*rad,Vec3::UnitY())), true).allowed);
  for (double limit : {0.0, -40.0, 181.0, nan})
    EXPECT_FALSE(check_target_turn(point(0), Vec3::Zero(), yaw(0), true, limit).allowed);
}
} // namespace
} // namespace hope_planner_cpp
