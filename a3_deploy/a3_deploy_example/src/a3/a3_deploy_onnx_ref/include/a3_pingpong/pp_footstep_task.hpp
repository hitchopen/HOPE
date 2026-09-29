// Observable task progress only. No action, q_des, contact force or recovery actuator.
#pragma once
#include "a3_pingpong/pp_base_estimator.hpp"
#include "a3_pingpong/pp_obs_builder.hpp"

namespace a3_pingpong {
struct FootstepKinematics {
  std::array<Vec2, 2> xy;
  std::array<double, 2> minimum_height{}, tilt{}, speed{};
};

inline FootstepKinematics footstep_kinematics(const PpRobotState& s) {
  FootstepKinematics out;
  const Mat3 rb = mat_from_quat(s.base_quat_w);
  for (int f = 0; f < 2; ++f) {
    const auto& leg = f == 0 ? leg_fk_detail::left_leg() : leg_fk_detail::right_leg();
    Mat3 r = Mat3::Identity(); Vec3 p = Vec3::Zero();
    std::array<Vec3, 6> axes, origins;
    for (int j = 0; j < 6; ++j) {
      const auto& link = leg[j];
      p += r * link.pos;
      r *= mat_from_quat(link.quat);
      origins[j] = p;
      axes[j] = r.col(link.axis);
      r *= axis_rot(link.axis, s.q[link.qidx]);
    }
    Vec3 v = s.base_ang_vel_b.cross(p);
    for (int j = 0; j < 6; ++j) v += axes[j].cross(p - origins[j]) * s.qd[leg[j].qidx];
    v = (rb * v).eval();
    v.head<2>() += s.base_velocity_xy_w;
    const Vec3 world = s.base_pos_w + rb * p;
    const Mat3 rotation = rb * r;
    out.xy[f] = world.head<2>(); out.speed[f] = v.head<2>().norm();
    out.tilt[f] = std::acos(std::clamp(rotation(2, 2), -1., 1.));
    out.minimum_height[f] = INFINITY;
    for (double x : {-.05, .13}) for (double y : {-.05, .05})
      out.minimum_height[f] = std::min(out.minimum_height[f], (world + rotation * Vec3(x, y, -.067))[2]);
  }
  return out;
}

class ObservableFootstepTask {
 public:
  void Reset() { phase_ = index_ = 0; dwell_ = 0.; origin_.setZero(); destination_.setZero(); }
  void Step(const FootstepKinematics& k, const Vec2& home, double yaw,
            double level, double sign, double tts, double dt, bool body_ready = true, bool advance = true) {
    if (!(dt > 0. && dt <= .1)) throw std::runtime_error("invalid footstep task dt");
    const Vec2 left(-std::sin(yaw), std::cos(yaw));
    std::array<Vec2, 2> goal{home + .134 * left, home - .134 * left};
    const bool outward = (level == 1. || level == 2.) && std::abs(sign) == 1. && tts <= .75 && tts >= -.12;
    if (outward) goal[sign > 0. ? 0 : 1] += sign * (level == 2. ? .06 : .02) * left;
    const double hmin = std::min(k.minimum_height[0], k.minimum_height[1]);
    const std::array<double, 2> h{k.minimum_height[0] - hmin, k.minimum_height[1] - hmin};
    const double e0 = (k.xy[0] - goal[0]).norm(), e1 = (k.xy[1] - goal[1]).norm();
    const Vec2 d0 = k.xy[0] - goal[0], d1 = k.xy[1] - goal[1];
    const Vec2 midpoint = .5 * (d0 + d1);
    const double shape_error = std::max((d0 - midpoint).norm(), (d1 - midpoint).norm());
    if (phase_ == 0 && (shape_error > .012 || midpoint.norm() > .04 ||
        (outward && std::max(e0, e1) > .012) || std::max(h[0], h[1]) > .012)) {
      index_ = std::max(h[0], h[1]) > .012 ? (h[0] >= h[1] ? 0 : 1) : (e0 >= e1 ? 0 : 1);
      origin_ = k.xy[index_]; destination_ = goal[index_]; phase_ = 1;
    }
    const int old = phase_;
    if (!advance) return;  // reset/replay observation has no physical landing dwell
    const double origin_error = (k.xy[index_] - origin_).norm();
    const double destination_error = (k.xy[index_] - destination_).norm();
    const int stance = 1 - index_;
    const bool planted = std::abs(k.minimum_height[stance]) <= .012 && k.tilt[stance] <= .15 && k.speed[stance] <= .08;
    if (old == 1 && h[index_] >= .025 && origin_error <= .008 && k.tilt[index_] <= .12 && planted) phase_ = 2;
    if (old == 2 && (h[index_] < .018 || !planted)) phase_ = 1;
    if (old == 2 && h[index_] >= .025 && destination_error <= .008 && k.speed[index_] <= .12 && planted) phase_ = 3;
    if (old == 3 && (destination_error > .018 || h[index_] > .07)) phase_ = 2;
    const bool settled = phase_ == 3 && old == 3 && destination_error <= .008 && h[index_] <= .006 && k.tilt[index_] <= .10 && k.speed[index_] <= .06 && planted && body_ready;
    dwell_ = settled ? dwell_ + dt : 0.;
    if (phase_ == 3 && dwell_ >= .08 - 1e-6) { phase_ = 0; dwell_ = 0.; }
  }
  double signed_phase() const {
    return (index_ == 0 ? 1. : -1.) * (.25 * phase_ + (phase_ == 3 ? .20 * std::clamp(dwell_ / .08, 0., 1.) : 0.));
  }
  Eigen::Vector4d errors(const FootstepKinematics& k) const {
    Eigen::Vector4d result = Eigen::Vector4d::Zero();
    if (phase_ != 0) { result.head<2>() = origin_ - k.xy[index_]; result.tail<2>() = destination_ - k.xy[index_]; }
    return result;
  }
 private:
  int phase_ = 0, index_ = 0;
  double dwell_ = 0.;
  Vec2 origin_ = Vec2::Zero(), destination_ = Vec2::Zero();
};
}  // namespace a3_pingpong
