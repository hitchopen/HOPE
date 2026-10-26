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

TEST(SwingSideSupport, Model14600FieldProfileDoesNotLatchPreviousSide) {
  // Exported arm bands plus the immutable-HOME station range [-0.35, 0.35].
  const std::vector<double> fh{-.83, -.05}, bh{-.48, .30};
  for (double previous : {-1., 0., 1.}) {
    EXPECT_DOUBLE_EQ(select_swing_sign_with_support(-.261, 0., -.25, 0., previous, fh, bh), 1.);
    EXPECT_DOUBLE_EQ(select_swing_sign_with_support(-.230, 0., -.25, 0., previous, fh, bh), -1.);
    EXPECT_DOUBLE_EQ(select_swing_sign_with_support(-.25, 0., -.25, 0., previous, fh, bh), -1.);
    EXPECT_DOUBLE_EQ(select_swing_sign_with_support(-.80, 0., -.25, 0., previous, fh, bh), 1.);
    EXPECT_DOUBLE_EQ(select_swing_sign_with_support(.29, 0., -.25, 0., previous, fh, bh), -1.);
  }
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

TEST(SwingSideSupport, EmptySupportPreservesLegacyIncludingExactBoundaries) {
  for (double previous : {-1., 0., 1.})
    for (double hysteresis : {0., .04})
      for (double y : {-.70, -.29, -.25, -.21, .28})
        EXPECT_DOUBLE_EQ(
            select_swing_sign_with_support(y, 0., -.25, hysteresis, previous, {}, {}),
            select_swing_sign_with_hysteresis(y, 0., -.25, hysteresis, previous));
}
}  // namespace
}  // namespace hope_planner_cpp
