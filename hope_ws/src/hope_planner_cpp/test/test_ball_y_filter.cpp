#include "hope_planner_cpp/ball_y_filter.hpp"
#include <gtest/gtest.h>

namespace hope_planner_cpp {
TEST(BallYFilter, InclusiveBoundsAndBothPositions) {
  const BallYFilter f{true, -1.3895904353652275, -0.5543593239924284};
  EXPECT_EQ(f.rejection(f.min_y, f.max_y), nullptr);
  EXPECT_EQ(f.rejection(f.max_y, f.min_y), nullptr);
  EXPECT_EQ(f.rejection(-0.9, -0.8), nullptr);
  for (double y : {f.min_y - 1e-9, f.max_y + 1e-9}) {
    EXPECT_STREQ(f.rejection(y, -0.8), "ball_y_filter_observed_outside");
    EXPECT_STREQ(f.rejection(-0.8, y), "ball_y_filter_contact_outside");
  }
}
TEST(BallYFilter, FailsClosedOnInvalidInputsAndBounds) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const BallYFilter f{true, -1.4, -0.55};
  for (double y : {nan, inf, -inf}) {
    EXPECT_STREQ(f.rejection(y, -0.8), "ball_y_filter_position_invalid");
    EXPECT_STREQ(f.rejection(-0.8, y), "ball_y_filter_position_invalid");
  }
  for (const auto bounds : {BallYFilter{true, nan, -0.5},
                           BallYFilter{true, -1.4, inf},
                           BallYFilter{true, -0.5, -1.4},
                           BallYFilter{true, -0.5, -0.5}}) {
    EXPECT_FALSE(bounds.valid_bounds());
    EXPECT_STREQ(bounds.rejection(-0.8, -0.8), "ball_y_filter_bounds_invalid");
  }
  EXPECT_EQ(BallYFilter{}.rejection(nan, nan), nullptr);
}
}  // namespace hope_planner_cpp
