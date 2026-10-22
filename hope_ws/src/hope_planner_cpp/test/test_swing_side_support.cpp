#include "hope_planner_cpp/swing_side_support.hpp"
#include <gtest/gtest.h>

namespace hope_planner_cpp {
namespace {
const std::vector<double> kFh{-0.68, -0.24};
const std::vector<double> kBh{-0.24, 0.28};

TEST(SwingSideSupport, ReplaysAll26Gate3LanesWithoutHistoryDependentHoles) {
  constexpr double home = -0.7625;
  const std::vector<double> lanes{
      -1.0225, -.7125, -1.0575, -.6575, -1.0925, -.6125,
      -1.0225, -1.0575, -.7125, -.6575, -1.0925, -.6125,
      -1.0225, -.7125, -1.0575, -.6575, -1.0925, -.6125,
      -1.0225, -.7125, -1.0575, -.6575, -1.0925, -.6125, -1.0225, -.7125};
  for (double initial : {-1.0, 0.0, 1.0}) {
    double previous = initial;
    for (std::size_t i = 0; i < lanes.size(); ++i) {
      previous = select_swing_sign_with_support(lanes[i], home, -.25, .04, previous, kFh, kBh);
      EXPECT_DOUBLE_EQ(previous, lanes[i] - home < -.24 ? 1.0 : -1.0) << "shot " << i + 1;
    }
  }
}

TEST(SwingSideSupport, OverridesWrongSideInBothHysteresisDirections) {
  EXPECT_DOUBLE_EQ(select_swing_sign_with_support(-.261, 0, -.25, .04, -1, kFh, kBh), 1);
  EXPECT_DOUBLE_EQ(select_swing_sign_with_support(-.230, 0, -.25, .04, 1, kFh, kBh), -1);
  EXPECT_DOUBLE_EQ(select_swing_sign_with_support(-.261, 0, -.25, .04, -1, {}, {}), -1);
}

TEST(SwingSideSupport, KeepsHysteresisOnlyInsideOverlappingSupport) {
  const std::vector<double> fh{-.68, -.20}, bh{-.30, .28};
  EXPECT_DOUBLE_EQ(select_swing_sign_with_support(-.26, 0, -.25, .04, 1, fh, bh), 1);
  EXPECT_DOUBLE_EQ(select_swing_sign_with_support(-.26, 0, -.25, .04, -1, fh, bh), -1);
  EXPECT_DOUBLE_EQ(select_swing_sign_with_support(-.24, 0, -.25, .04, 1, kFh, kBh), 1);
  EXPECT_DOUBLE_EQ(select_swing_sign_with_support(-.24, 0, -.25, .04, -1, kFh, kBh), -1);
}

TEST(SwingSideSupport, FailsClosedOutsideBothBandsOrOnBadContract) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (double target : {-.69, .29, nan}) {
    EXPECT_TRUE(std::isnan(select_swing_sign_with_support(target, 0, -.25, .04, 1, kFh, kBh)));
  }
  EXPECT_TRUE(std::isnan(select_swing_sign_with_support(0, nan, -.25, .04, 1, kFh, kBh)));
  EXPECT_FALSE(valid_swing_side_support_ranges(kFh, {}));
  EXPECT_FALSE(valid_swing_side_support_ranges({0, -1}, kBh));
  EXPECT_FALSE(valid_swing_side_support_ranges({nan, 1}, kBh));
  EXPECT_TRUE(std::isnan(select_swing_sign_with_support(-.25, 0, -.25, .04, 1, {-.68, -.30}, {-.20, .28})));
}

TEST(SwingSideSupport, UsesRunnerHomeInsteadOfDriftedFirstSolve) {
  const std::vector<double> fh{-.9002, -.275}, bh{-.275, .4598};
  const double home = runner_home_y_from_flat({1., 1., -.494, -.562});
  for (double previous : {-1., 0., 1.}) {
    EXPECT_DOUBLE_EQ(select_swing_sign_with_support(-1.4558, home, -.275, 0., previous, fh, bh), 1.);
    EXPECT_DOUBLE_EQ(select_swing_sign_with_support(-.8077, home, -.275, 0., previous, fh, bh), -1.);
    EXPECT_DOUBLE_EQ(select_swing_sign_with_support(-.1192, home, -.275, 0., previous, fh, bh), -1.);
    EXPECT_DOUBLE_EQ(select_swing_sign_with_support(-.275, 0., -.275, 0., previous, fh, bh), -1.);
  }
  EXPECT_TRUE(std::isnan(select_swing_sign_with_support(-1.4558, -.5515, -.275, 0., 1., fh, bh)));
  EXPECT_TRUE(std::isnan(runner_home_y_from_flat({1., 0., -.494, -.562})));
  EXPECT_TRUE(std::isnan(runner_home_y_from_flat({2., 1., -.494, -.562})));
  EXPECT_TRUE(std::isnan(runner_home_y_from_flat({1., 1., -.494})));
}
TEST(SwingSideSupport, KeepsFixedInterceptInsideImmutableHomeXSupport) {
  const std::vector<double> support{.36, .58};
  // Measured Home is 2 mm behind the nominal reset; the old world-fixed
  // x=0.08 generated relative x=0.582 and every swing was rejected.
  const double home = -.502;
  const double x = supported_home_x_hit(.08, home, support);
  EXPECT_LT(x - home, .58);
  EXPECT_GT(x - home, .57999);
  EXPECT_NEAR(supported_home_x_hit(.08, -.494, support), .08, 1e-15);
  EXPECT_GT(supported_home_x_hit(-1., home, support) - home, .36);
  EXPECT_DOUBLE_EQ(supported_home_x_hit(.08, home, {}), .08);
  EXPECT_TRUE(std::isnan(supported_home_x_hit(.08,
      std::numeric_limits<double>::quiet_NaN(), support)));
  EXPECT_TRUE(std::isnan(supported_home_x_hit(.08, home, {.58, .36})));
}
}  // namespace
}  // namespace hope_planner_cpp
