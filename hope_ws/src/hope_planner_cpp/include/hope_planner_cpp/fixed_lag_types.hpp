#pragma once
#include "hope_planner_cpp/probabilistic_types.hpp"
#include <chrono>
#include <optional>
namespace hope_planner_cpp {
using State9 = Eigen::Matrix<double,9,1>;
using Matrix9 = Eigen::Matrix<double,9,9>;
struct FullTrackerCheckpoint {
  StateBelief belief;
  Vec3 bias_mean = Vec3::Zero();
  Eigen::Matrix<double,6,3> state_bias_covariance = Eigen::Matrix<double,6,3>::Zero();
  Matrix3 bias_covariance = Matrix3::Zero();
  std::size_t consecutive_rejections = 0;
  double innovation_sum_squared = 0.;
  std::string noise_fingerprint;
  State9 mean() const;
  Matrix9 covariance() const;
  bool valid() const;
};
std::string noise_model_fingerprint(const ProbabilisticConfig &);
} // namespace hope_planner_cpp
