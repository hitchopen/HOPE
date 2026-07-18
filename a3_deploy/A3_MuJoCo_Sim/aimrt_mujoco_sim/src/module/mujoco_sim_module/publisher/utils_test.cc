// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "mujoco_sim_module/publisher/utils.h"
#include <gtest/gtest.h>

namespace aimrt_mujoco_sim::mujoco_sim_module::publisher {

TEST(GetAvgIntervalBaseTest, BoundaryValidation) {
  EXPECT_ANY_THROW(GetAvgIntervalBase(0));
  EXPECT_ANY_THROW(GetAvgIntervalBase(1001));
}

TEST(GetAvgIntervalBaseTest, ExactFrequency) {
  const std::vector<std::pair<uint32_t, double>> cases = {
      {1, 1000.0},
      {10, 100.0},
      {1000, 1.0}};

  for (const auto& [freq, expected] : cases) {
    EXPECT_DOUBLE_EQ(GetAvgIntervalBase(freq), expected);
  }
}

TEST(GetAvgIntervalBaseTest, ApproximateFrequency) {
  // valid_cases
  {
    const std::vector<uint32_t> valid_cases = {48, 100};
    for (auto freq : valid_cases) {
      EXPECT_NO_THROW(GetAvgIntervalBase(freq));
    }
  }

  // invalid_cases
  {
    const std::vector<uint32_t> invalid_cases = {999};
    for (auto freq : invalid_cases) {
      EXPECT_ANY_THROW(GetAvgIntervalBase(freq));
    }
  }
}

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::publisher