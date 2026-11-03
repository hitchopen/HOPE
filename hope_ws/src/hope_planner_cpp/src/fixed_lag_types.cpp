#include "hope_planner_cpp/fixed_lag_types.hpp"
#include "hope_planner_cpp/return_region.hpp"
#include <Eigen/Eigenvalues>
#include <sstream>
#include <locale>
#include <iomanip>
#include <cmath>
#include <stdexcept>
namespace hope_planner_cpp {
State9 FullTrackerCheckpoint::mean() const { State9 s; s << belief.mean,bias_mean; return s; }
Matrix9 FullTrackerCheckpoint::covariance() const {
  Matrix9 p; p << belief.covariance,state_bias_covariance,state_bias_covariance.transpose(),bias_covariance;
  return p;
}
bool FullTrackerCheckpoint::valid() const {
  if (!mean().allFinite() || !covariance().allFinite() ||
      !std::isfinite(belief.source_time_s) || !std::isfinite(innovation_sum_squared) ||
      !std::isfinite(belief.first_measurement_time_s) || !std::isfinite(belief.last_measurement_time_s) ||
      !std::isfinite(belief.last_measurement_gap_s) || !std::isfinite(belief.innovation_squared) ||
      !std::isfinite(belief.innovation_rms_m) || !std::isfinite(belief.innovation_max_m) ||
      innovation_sum_squared<0. || belief.last_measurement_time_s>belief.source_time_s+1e-10) return false;
  if (!belief.initialized) return true;
  const Matrix9 p=covariance();
  if ((p-p.transpose()).norm()>1e-9*std::max(1.,p.norm())) return false;
  Eigen::SelfAdjointEigenSolver<Matrix9> eig(p);
  return eig.info()==Eigen::Success && eig.eigenvalues().minCoeff()>=-1e-12;
}
std::string noise_model_fingerprint(const ProbabilisticConfig &c) {
  std::ostringstream o; o.imbue(std::locale::classic());o << physics_fingerprint(c) << ":tracker-v1:" << std::hexfloat;
  for(double x:{c.measurement_std_m[0],c.measurement_std_m[1],c.measurement_std_m[2],
      c.measurement_bias_std_m[0],c.measurement_bias_std_m[1],c.measurement_bias_std_m[2],
      c.measurement_bias_tau_s,c.velocity_prior_mean[0],c.velocity_prior_mean[1],c.velocity_prior_mean[2],
      c.velocity_prior_std[0],c.velocity_prior_std[1],c.velocity_prior_std[2],
      c.innovation_gate_squared,c.max_tracking_gap_s,c.clock_reset_backstep_s}) o << x << ':';
  o << c.reacquire_rejections;return o.str();
}
} // namespace hope_planner_cpp
