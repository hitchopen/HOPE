#pragma once
#include "hope_planner_cpp/probabilistic_types.hpp"
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
namespace hope_planner_cpp {
// Forecast-only immutable-by-value snapshot. Never passed to the CKF.
struct ForecastNoiseConfig {
  double acceleration_spectral_density=.10;
  Vec3 bounce_velocity_std=Vec3(.65,.30,.45);
  bool attribute_covariance=false; // Offline diagnostics; no transport schema change.
  static ForecastNoiseConfig inherited(const ProbabilisticConfig &c) {
    ForecastNoiseConfig n;n.acceleration_spectral_density=c.acceleration_spectral_density;
    n.bounce_velocity_std=c.bounce_velocity_std;return n;
  }
  void validate() const {
    if(!std::isfinite(acceleration_spectral_density) || acceleration_spectral_density<0 ||
       !bounce_velocity_std.allFinite() || bounce_velocity_std.minCoeff()<0)
      throw std::invalid_argument("finite nonnegative forecast noise required");
  }
  std::string identity() const {
    validate();std::ostringstream s;s.imbue(std::locale::classic());s<<std::setprecision(17)<<"forecast-noise-v1;q="<<acceleration_spectral_density;
    for(int i=0;i<3;++i)s<<";bounce"<<i<<'='<<bounce_velocity_std[i];
    return s.str();
  }
};
template<int N> struct ForecastCovarianceBudget {
  using Matrix=Eigen::Matrix<double,N,N>;
  bool available=false;
  Matrix posterior=Matrix::Zero(), free_flight=Matrix::Zero(), bounce=Matrix::Zero();
  Matrix total() const {return posterior+free_flight+bounce;}
};
} // namespace hope_planner_cpp
