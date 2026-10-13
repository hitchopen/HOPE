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
  // The normal and tangential direction retain the reference 1e-9 epsilon.
  EXPECT_NEAR(result.angular_velocity[1], 11.699999951301349, 1.0e-8);
  EXPECT_NEAR(result.angular_velocity[2], 3.899999983767117, 1.0e-8);
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
      Vec3{4.0, 0.0, 0.0}, Vec3{0.0, 10.0, 0.0}, 0.1220);
  EXPECT_NEAR(acceleration[0], -1.952, 1.0e-12);
  EXPECT_NEAR(acceleration[1], 0.0, 1.0e-12);
  // October 10 calibration: Magnus coefficient 0.00379260504480258.
  EXPECT_NEAR(acceleration[2], -0.1517042017921032, 1.0e-12);
}

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::common::gate3_ball
