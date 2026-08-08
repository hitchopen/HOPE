#include "a3_pingpong/pp_static_handoff.hpp"

#include <gtest/gtest.h>

namespace a3_pingpong {

TEST(PpStaticHandoff, V11AffineResetsToZeroActionHistory) {
  const RuntimeHandoffReset reset = runtime_handoff_reset(false);
  EXPECT_FALSE(reset.seed_measured_qdes_feedback);
  EXPECT_TRUE(reset.force_exact_default_static);
}

TEST(PpStaticHandoff, BoundedContractDefersMeasuredFeedbackToFirstTick) {
  const RuntimeHandoffReset reset = runtime_handoff_reset(true);
  EXPECT_TRUE(reset.seed_measured_qdes_feedback);
  EXPECT_TRUE(reset.force_exact_default_static);
}

TEST(PpStaticHandoff, UprightTrueColdBootDoesNotRequireLocalizationSpeed) {
  EXPECT_TRUE(prefirst_static_allowed(
      false, true, false, false, true, false));
}

TEST(PpStaticHandoff, ColdBootStillRequiresUprightQuietRobot) {
  EXPECT_FALSE(prefirst_static_allowed(
      false, true, false, false, false, true));
}

TEST(PpStaticHandoff, AbortedActiveTrackingRequiresBaseSettle) {
  EXPECT_FALSE(prefirst_static_allowed(
      false, true, true, false, true, true));
  EXPECT_TRUE(prefirst_static_allowed(
      false, true, true, true, true, false));
}

TEST(PpStaticHandoff, NeverOverridesPostSwingOrOffStationPath) {
  EXPECT_FALSE(prefirst_static_allowed(
      true, true, false, true, true, true));
  EXPECT_FALSE(prefirst_static_allowed(
      false, false, false, true, true, true));
}

}  // namespace a3_pingpong
