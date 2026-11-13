#pragma once

#include "hope_planner_cpp/types.hpp"
#include <algorithm>

namespace hope_planner_cpp {

inline Vec3 ball_acceleration(const BallPhysics& p, const Vec3& v,
                              const Vec3& omega) noexcept {
  return p.gravity - p.drag_k * v.norm() * v + p.magnus_k * omega.cross(v);
}

// Static-table impulse, shared by observed-contact state fitting and future
// prediction. Angular velocity is in world coordinates, in rad/s.
inline void table_contact(const BallPhysics& p, Vec3& v, Vec3& omega) noexcept {
  if (!(p.radius > 0.0)) {
    v = Vec3(p.restitution_h * v.x(), p.restitution_h * v.y(),
             -p.restitution_v * v.z());
    return;
  }
  const Vec3 slip(v.x() - p.radius * omega.y(), v.y() + p.radius * omega.x(), 0.0);
  const double speed = slip.norm();
  const double impulse = std::min(std::max(0.0, p.table_tangential_gain) * speed,
      std::max(0.0, p.table_friction_cap_mu) * (1.0 + p.restitution_v) * std::abs(v.z()));
  Vec3 dv = Vec3::Zero();
  if (speed > 1.0e-12) dv = -impulse / speed * slip;
  v += dv;
  v.z() = -p.restitution_v * v.z();
  omega += Vec3(dv.y(), -dv.x(), 0.0) / ((2.0 / 3.0) * p.radius);
}

}  // namespace hope_planner_cpp
