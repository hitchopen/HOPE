// Copyright (c) 2026, AgiBot Inc.
// All rights reserved.

#include "mujoco_sim_module/common/gate3_ball_contact_model.h"

#include <gtest/gtest.h>

namespace aimrt_mujoco_sim::mujoco_sim_module::common::gate3_ball {

TEST(Gate3BallContactModel, MatchesPlannerNormalContactReference) {
  const auto result = PredictPaddleContact(
      Vec3{-2.0, 0.1, -0.3}, Vec3{2.0, 0.0, 0.0},
      Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 0.0, 0.0}, 0.020);

  EXPECT_NEAR(result.linear_velocity[0], 4.54502597, 1.0e-8);
  EXPECT_NEAR(result.linear_velocity[1], 0.048, 1.0e-9);
  EXPECT_NEAR(result.linear_velocity[2], -0.144, 1.0e-9);
  // These values pass through normalized floating-point vectors.  GCC 13
  // differs from the original compiler by less than 5e-8 here; 1e-7 remains
  // far below any physical or telemetry resolution used by Gate3.
  EXPECT_NEAR(result.angular_velocity[1], 11.7, 1.0e-7);
  EXPECT_NEAR(result.angular_velocity[2], 3.9, 1.0e-7);
}

TEST(Gate3BallContactModel, NormalOrientationIsSignInvariant) {
  const auto positive = PredictPaddleContact(
      Vec3{-2.0, 0.0, 0.0}, Vec3{2.0, 0.0, 0.0},
      Vec3{1.0, 0.0, 0.0}, Vec3{}, 0.020);
  const auto negative = PredictPaddleContact(
      Vec3{-2.0, 0.0, 0.0}, Vec3{2.0, 0.0, 0.0},
      Vec3{-1.0, 0.0, 0.0}, Vec3{}, 0.020);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    EXPECT_NEAR(
        positive.linear_velocity[axis],
        negative.linear_velocity[axis], 1.0e-12);
  }
}

TEST(Gate3BallContactModel, FlightAccelerationContainsDragGravityFreeMagnus) {
  const auto acceleration = FlightAcceleration(
      Vec3{4.0, 0.0, 0.0}, Vec3{0.0, 10.0, 0.0}, 0.1261);
  EXPECT_NEAR(acceleration[0], -2.0176, 1.0e-12);
  EXPECT_NEAR(acceleration[1], 0.0, 1.0e-12);
  EXPECT_NEAR(acceleration[2], -0.1776, 1.0e-12);
}

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::common::gate3_ball
