#include "hope_planner_cpp/spin_estimator.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace hope_planner_cpp {
namespace {

constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

Eigen::Quaterniond orientation_at(
    const Vec3& omega_rad_s, double time_s,
    const Eigen::Quaterniond& body_offset = Eigen::Quaterniond::Identity()) {
  const Vec3 rotation_vector = omega_rad_s * time_s;
  const double angle = rotation_vector.norm();
  const Eigen::Quaterniond world_rotation = angle < 1.0e-12
      ? Eigen::Quaterniond::Identity()
      : Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation_vector / angle));
  return world_rotation * body_offset;
}

BallSample sample(double time_s, const Eigen::Quaterniond& orientation) {
  BallSample output;
  output.source_time_s = time_s;
  output.orientation = orientation;
  output.orientation_valid = true;
  return output;
}

TEST(SpinEstimator, RecoversConstantWorldSpinWithoutNoiseGate) {
  SpinEstimator estimator;
  const Vec3 expected(0.0, -6.0 * kTwoPi, 1.5 * kTwoPi);
  for (int index = 0; index < 50; ++index) {
    const double time_s = index / 360.0;
    estimator.push(sample(time_s, orientation_at(expected, time_s)));
  }
  const SpinEstimate result = estimator.estimate();
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_NEAR(result.omega_rad_s.x(), expected.x(), 1.0e-9);
  EXPECT_NEAR(result.omega_rad_s.y(), expected.y(), 1.0e-9);
  EXPECT_NEAR(result.omega_rad_s.z(), expected.z(), 1.0e-9);
  EXPECT_NEAR(result.coherence, 1.0, 1.0e-12);
}

TEST(SpinEstimator, RejectsOneRelockIncrementButKeepsFollowingMotion) {
  SpinEstimator estimator;
  const Vec3 expected(1.0 * kTwoPi, -5.0 * kTwoPi, 0.0);
  const Eigen::Quaterniond relock(
      Eigen::AngleAxisd(2.4, Vec3(0.3, 0.8, -0.2).normalized()));
  for (int index = 0; index < 55; ++index) {
    const double time_s = index / 360.0;
    const auto offset = index < 20 ? Eigen::Quaterniond::Identity() : relock;
    estimator.push(sample(time_s, orientation_at(expected, time_s, offset)));
  }
  const SpinEstimate result = estimator.estimate();
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_GE(result.rejected_increments, 1u);
  EXPECT_NEAR(result.omega_rad_s.x(), expected.x(), 1.0e-8);
  EXPECT_NEAR(result.omega_rad_s.y(), expected.y(), 1.0e-8);
  EXPECT_NEAR(result.omega_rad_s.z(), expected.z(), 1.0e-8);
}

TEST(SpinEstimator, MissingOrientationDoesNotInventSpin) {
  SpinEstimator estimator;
  BallSample invalid;
  invalid.source_time_s = 1.0;
  invalid.orientation_valid = false;
  estimator.push(invalid);
  const SpinEstimate result = estimator.estimate();
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "spin_not_ready");
}

TEST(SpinEstimator, InvalidOrientationBreaksDeltaAcrossReacquisition) {
  for (const int hz : {150, 300}) {
    for (const bool p2 : {false, true}) {
      SCOPED_TRACE(::testing::Message() << "hz=" << hz << " p2=" << p2);
      SpinEstimator estimator;
      const Eigen::Quaterniond side = p2
          ? Eigen::Quaterniond(Eigen::AngleAxisd(kTwoPi / 2, Vec3::UnitZ()))
          : Eigen::Quaterniond::Identity();
      const Vec3 expected = side * Vec3(10., 20., 30.);
      for (int i = 0; i <= hz / 5; ++i) {
        const double time = static_cast<double>(i) / hz;
        estimator.push(sample(time, side * orientation_at(Vec3(10., 20., 30.), time)));
      }
      ASSERT_TRUE(estimator.estimate().valid);
      BallSample missing;
      missing.source_time_s = 0.2 + 1.0 / hz;
      missing.orientation = Eigen::Quaterniond(0., 0., 0., 0.);
      missing.orientation_valid = false;
      estimator.push(missing);
      EXPECT_FALSE(estimator.estimate().valid);

      const double resumed = missing.source_time_s + 1.0 / hz;
      const Eigen::Quaterniond new_offset(Eigen::AngleAxisd(.15, Vec3::UnitX()));
      estimator.push(sample(resumed, side * new_offset));
      EXPECT_FALSE(estimator.estimate().valid);  // no delta from the sentinel
      for (int i = 1; i <= hz / 5; ++i) {
        const double elapsed = static_cast<double>(i) / hz;
        estimator.push(sample(resumed + elapsed,
            side * orientation_at(Vec3(10., 20., 30.), elapsed, new_offset)));
      }
      const auto recovered = estimator.estimate();
      ASSERT_TRUE(recovered.valid) << recovered.reason;
      EXPECT_LT((recovered.omega_rad_s - expected).norm(), 1e-8);
    }
  }
}

}  // namespace
}  // namespace hope_planner_cpp
