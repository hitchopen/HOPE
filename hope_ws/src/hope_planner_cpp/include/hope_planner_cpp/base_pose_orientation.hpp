#pragma once
#include <Eigen/Geometry>
#include <optional>
#include <vector>
namespace hope_planner_cpp {
// Schema 2 is owned by hope_bringup/base_pose_contract.py: slots 8:12
// contain w,x,y,z. Callers still require the registration/flags/time checks.
inline std::optional<Eigen::Quaterniond>
base_orientation_wxyz(const std::vector<double> &w) {
  if (w.size() != 16)
    return {};
  Eigen::Quaterniond q(w[8], w[9], w[10], w[11]);
  if (!q.coeffs().allFinite() || q.norm() < .5 || q.norm() > 1.5)
    return {};
  return q.normalized();
}
} // namespace hope_planner_cpp
