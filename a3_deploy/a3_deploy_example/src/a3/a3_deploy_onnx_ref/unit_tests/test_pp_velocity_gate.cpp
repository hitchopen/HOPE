#include <array>

#include <gtest/gtest.h>

#include "a3_pingpong/pp_velocity_gate.hpp"

namespace a3_pingpong {
namespace {

constexpr std::array<double, 6> kForehandCore =
    {1.24, 2.24, -0.31, 0.69, 0.66, 1.66};
constexpr std::array<double, 6> kForehandPlanner =
    {1.57, 2.55, 0.10, 0.52, 0.41, 1.35};
constexpr std::array<double, 6> kForehandUnion =
    {1.24, 2.60, -0.31, 0.69, 0.40, 1.66};

TEST(PpVelocityGate, AcceptsEitherSampledComponent) {
  EXPECT_TRUE(velocity_in_component_support(
      kForehandCore, kForehandPlanner, 1.30, -0.20, 1.50));
  EXPECT_TRUE(velocity_in_component_support(
      kForehandCore, kForehandPlanner, 2.50, 0.50, 0.50));
}

TEST(PpVelocityGate, RejectsUnionOnlyCartesianCorner) {
  // This command is inside the exported safety union, but its high-x/high-z combination was
  // sampled by neither component.  It remains rejected even with the runner's default 0.30
  // m/s per-axis tolerance.
  EXPECT_TRUE(velocity_in_box(kForehandUnion, 2.60, 0.20, 1.66));
  EXPECT_FALSE(velocity_in_component_support(
      kForehandCore, kForehandPlanner, 2.60, 0.20, 1.66, 0.30));
}

TEST(PpVelocityGate, RejectsNegativeZAndOverspeedAxis) {
  EXPECT_FALSE(velocity_in_component_support(
      kForehandCore, kForehandPlanner, 2.00, 0.20, -0.01));
  EXPECT_FALSE(velocity_in_component_support(
      kForehandCore, kForehandPlanner, 3.60, 0.20, 1.00));
}

}  // namespace
}  // namespace a3_pingpong
