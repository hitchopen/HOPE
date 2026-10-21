#include "hope_planner_cpp/nominal_event_forecast.hpp"
#include "hope_planner_cpp/common_ball_physics.hpp"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
namespace hope_planner_cpp {
namespace {
State6 pack(const ModelState &s) {
  State6 x;
  x << s.position, s.velocity;
  return x;
}
ModelState unpack(const State6 &x, const EffectiveSpin &spin) {
  ModelState s;
  s.position = x.head<3>();
  s.velocity = x.tail<3>();
  s.spin = spin;
  return s;
}
State6 flow(const State6 &s, const BallPhysics &p, const EffectiveSpin &spin) {
  State6 f;
  f.head<3>() = s.tail<3>();
  f.tail<3>() = ball_acceleration(p, s.tail<3>(),
                                  spin.usable() ? *spin.omega : Vec3::Zero());
  return f;
}
bool covariance_ok(const Matrix6 &p) {
  if (!p.allFinite() ||
      (p - p.transpose()).norm() > 1e-9 * std::max(1., p.norm()))
    return false;
  Eigen::SelfAdjointEigenSolver<Matrix6> e(p, Eigen::EigenvaluesOnly);
  return e.info() == Eigen::Success &&
         e.eigenvalues().minCoeff() >= -1e-10 * std::max(1., p.norm());
}
Eigen::Matrix3d acceleration_jacobian(const Vec3 &v, const BallPhysics &p,
                                      const EffectiveSpin &spin) {
  Eigen::Matrix3d a = Eigen::Matrix3d::Zero();
  const double n = v.norm();
  if (n > 1e-15)
    a = -p.drag_k * (n * Eigen::Matrix3d::Identity() + v * v.transpose() / n);
  if (spin.usable()) {
    const Vec3 w = p.magnus_k * *spin.omega;
    Eigen::Matrix3d cross;
    cross << 0, -w.z(), w.y(), w.z(), 0, -w.x(), -w.y(), w.x(), 0;
    a += cross;
  }
  return a;
}
} // namespace
const char *quality_name(LinearizationQuality q) {
  switch (q) {
  case LinearizationQuality::Valid:
    return "valid_linearized";
  case LinearizationQuality::BranchSensitive:
    return "branch_sensitive";
  case LinearizationQuality::IllConditioned:
    return "ill_conditioned";
  default:
    return "unavailable";
  }
}
Matrix6 free_flight_jacobian(const State6 &s, double h, const BallPhysics &p,
                             const EffectiveSpin &spin) {
  // Differentiate the exact RK4 stages, holding the resolved auxiliary spin
  // fixed.
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  const Vec3 v = s.tail<3>();
  const auto acceleration = [&](const Vec3 &z) {
    return ball_acceleration(p, z, spin.usable() ? *spin.omega : Vec3::Zero());
  };
  const Vec3 v2 = v + .5 * h * acceleration(v),
             v3 = v + .5 * h * acceleration(v2), v4 = v + h * acceleration(v3);
  const Eigen::Matrix3d k1 = acceleration_jacobian(v, p, spin);
  const Eigen::Matrix3d m2 = I + .5 * h * k1,
                        k2 = acceleration_jacobian(v2, p, spin) * m2;
  const Eigen::Matrix3d m3 = I + .5 * h * k2,
                        k3 = acceleration_jacobian(v3, p, spin) * m3;
  const Eigen::Matrix3d m4 = I + h * k3,
                        k4 = acceleration_jacobian(v4, p, spin) * m4;
  Matrix6 j = Matrix6::Identity();
  j.topRightCorner<3, 3>() = h / 6 * (I + 2 * m2 + 2 * m3 + m4);
  j.bottomRightCorner<3, 3>() = I + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
  return j;
}
Matrix6 physical_step_jacobian(const State6 &state, double h,
                               const ProbabilisticConfig &c, bool &branch) {
  auto nominal = state;
  auto event = physical_step(nominal, h, c);
  branch = false;
  if (!event.bounced)
    return free_flight_jacobian(state, h, c.physics, c.effective_spin);
  Matrix6 j;
  for (int k = 0; k < 6; ++k) {
    const double eps = 1e-6 * std::max(1., std::abs(state[k]));
    State6 a = state, b = state;
    a[k] += eps;
    b[k] -= eps;
    const auto ea = physical_step(a, h, c), eb = physical_step(b, h, c);
    branch |= ea.bounced != event.bounced || eb.bounced != event.bounced;
    j.col(k) = (a - b) / (2 * eps);
  }
  return j;
}
Matrix6 bounce_saltation(const State6 &pre, const ProbabilisticConfig &c) {
  auto reset = [&](const State6 &x) {
    auto s = unpack(x, c.effective_spin);
    apply_model_contact(s, c.physics);
    return pack(s);
  };
  if (std::abs(pre[5]) < 1e-8)
    return Matrix6::Constant(NAN);
  Matrix6 dr;
  for (int k = 0; k < 6; ++k) {
    State6 a = pre, b = pre;
    const double eps = 1e-6 * std::max(1., std::abs(pre[k]));
    a[k] += eps;
    b[k] -= eps;
    dr.col(k) = (reset(a) - reset(b)) / (2 * eps);
  }
  auto post = unpack(pre, c.effective_spin);
  apply_model_contact(post, c.physics);
  Matrix6 salt = dr;
  salt.col(2) += (flow(pack(post), c.physics, post.spin) -
                  dr * flow(pre, c.physics, c.effective_spin)) /
                 pre[5];
  return salt;
}
Matrix7 spatial_event_covariance(const State6 &s, const Matrix6 &p, int axis,
                                 const BallPhysics &physics,
                                 const EffectiveSpin &spin) {
  if (std::abs(s[axis + 3]) < 1e-10)
    return Matrix7::Constant(NAN);
  Eigen::Matrix<double, 7, 6> b = Eigen::Matrix<double, 7, 6>::Zero();
  b.topRows<6>() = Matrix6::Identity();
  b(6, axis) = -1 / s[axis + 3];
  b.topRows<6>() += flow(s, physics, spin) * b.row(6);
  Matrix7 result = b * p * b.transpose();
  return .5 * (result + result.transpose()).eval();
}
NominalBallForecast forecast_nominal(const StateBelief &b,
                                     const ProbabilisticConfig &c) {
  return forecast_nominal(b, c, ForecastNoiseConfig::inherited(c));
}
NominalBallForecast forecast_nominal(const StateBelief &b,
                                     const ProbabilisticConfig &c,
                                     const ForecastNoiseConfig &noise) {
  c.validate();
  noise.validate();
  NominalBallForecast out;
  out.source = b;
  out.uncertainty_provenance =
      "thin_shell_review_v4;ordered_saltation_event_covariance;conditional_pv_linearization;auxiliary_spin_model_plane_"
      "clock_uncertainty_unmodelled;uncalibrated";
  if (!b.initialized || !b.mean.allFinite())
    return out;
  out.available = true;
  out.termination_reason = "horizon";
  ModelState model = unpack(b.mean, c.effective_spin);
  Matrix6 covariance = b.covariance;
  auto quality = covariance_ok(covariance) ? LinearizationQuality::Valid
                                           : LinearizationQuality::Unavailable;
  ForecastCovarianceBudget<6> budget;
  budget.available = noise.attribute_covariance &&
                     quality != LinearizationQuality::Unavailable;
  if (budget.available)
    budget.posterior = covariance;
  auto transform = [](const ForecastCovarianceBudget<6> &a, const Matrix6 &j) {
    auto z = a;
    if (a.available) {
      z.posterior = j * a.posterior * j.transpose();
      z.free_flight = j * a.free_flight * j.transpose();
      z.bounce = j * a.bounce * j.transpose();
    }
    return z;
  };
  auto waypoint = [&](double t) {
    ForecastWaypoint w;
    w.spin = model.spin;
    w.dt_s = t;
    w.state = pack(model);
    w.quality = quality;
    w.covariance = covariance;
    w.covariance_budget = budget;
    out.waypoints.push_back(w);
  };
  EventGeometry geometry;
  geometry.table = c.table;
  geometry.receiving_x = c.x_hit;
  unsigned contacts = 0;
  waypoint(0);
  double next_waypoint = c.trajectory_dt_s;
  for (double t = 0; t < c.horizon_s - 1e-12;) {
    const double h =
        std::min({c.integration_dt_s, c.horizon_s - t, next_waypoint - t});
    const auto before = model;
    const auto step = advance_ball(model, h, c.physics, geometry);
    auto segment_start = before;
    double cursor = 0;
    auto propagate = [&](const ModelState &start, double duration) {
      if (quality == LinearizationQuality::Unavailable) return;
      const Matrix6 j = free_flight_jacobian(pack(start), duration, c.physics, start.spin);
      const Matrix6 q = process_covariance(duration, noise.acceleration_spectral_density);
      covariance = j * covariance * j.transpose() + q;
      budget = transform(budget, j);
      if (budget.available) budget.free_flight += q;
    };
    for (const auto &e : step.events) {
      ForecastPhysicsEvent event;
      event.pre_spin = e.pre.spin;
      event.post_spin = e.post.spin;
      event.kind = name(e.kind);
      event.dt_s = t + e.dt_s;
      event.pre = pack(e.pre);
      event.post = pack(e.post);
      event.quality = quality;
      propagate(segment_start, e.dt_s - cursor);
      cursor = e.dt_s;
      segment_start = e.post;
      const Matrix6 pc = covariance;
      if (quality != LinearizationQuality::Unavailable) {
        if (e.kind == PhysicalEventKind::APEX) {
          const auto f = flow(event.pre, c.physics, e.pre.spin);
          if (std::abs(f[5]) < .05)
            event.quality = LinearizationQuality::IllConditioned;
          else {
            Eigen::Matrix<double, 7, 6> a = Eigen::Matrix<double, 7, 6>::Zero();
            a.topRows<6>() = Matrix6::Identity();
            a(6, 5) = -1 / f[5];
            a.topRows<6>() += f * a.row(6);
            event.covariance = a * pc * a.transpose();
          }
        } else {
          const int axis = (e.kind == PhysicalEventKind::TABLE ||
                            e.kind == PhysicalEventKind::GROUND ||
                            e.kind == PhysicalEventKind::EDGE_AMBIGUOUS)
                               ? 2
                               : 0;
          event.covariance = spatial_event_covariance(event.pre, pc, axis,
                                                      c.physics, e.pre.spin);
          if (std::abs(event.pre[axis + 3]) < .05)
            event.quality = LinearizationQuality::IllConditioned;
          if (budget.available) {
            const auto &terms = budget;
            event.covariance_budget.available = true;
            event.covariance_budget.posterior = spatial_event_covariance(
                event.pre, terms.posterior, axis, c.physics, e.pre.spin);
            event.covariance_budget.free_flight = spatial_event_covariance(
                event.pre, terms.free_flight, axis, c.physics, e.pre.spin);
            event.covariance_budget.bounce = spatial_event_covariance(
                event.pre, terms.bounce, axis, c.physics, e.pre.spin);
          }
        }
        if (event.covariance.allFinite())
          event.time_std_s = std::sqrt(std::max(0., event.covariance(6, 6)));
        else
          event.quality = LinearizationQuality::Unavailable;
      }
      if (e.kind == PhysicalEventKind::TABLE) {
        ++contacts;
        Matrix6 qb = Matrix6::Zero();
        qb.bottomRightCorner<3, 3>() =
            noise.bounce_velocity_std.array().square().matrix().asDiagonal();
        auto local = c;
        local.effective_spin = e.pre.spin;
        const Matrix6 reset = bounce_saltation(event.pre, local);
        if (reset.allFinite() && quality != LinearizationQuality::Unavailable) {
          covariance = reset * covariance * reset.transpose() + qb;
          budget = transform(budget, reset);
          if (budget.available) budget.bounce += qb;
        } else quality = event.quality = LinearizationQuality::Unavailable;
        const double sx = 3 * std::sqrt(std::max(0., pc(0, 0))),
                     sy = 3 * std::sqrt(std::max(0., pc(1, 1)));
        if (e.pre.position.x() - sx <= 0 ||
            e.pre.position.x() + sx >= c.table.length ||
            e.pre.position.y() - sy <= c.table.y_max - c.table.width ||
            e.pre.position.y() + sy >= c.table.y_max)
          event.quality = quality = LinearizationQuality::BranchSensitive;
      }
      event.physically_usable = e.kind != PhysicalEventKind::NET_COLLISION &&
                                e.kind != PhysicalEventKind::GROUND &&
                                e.kind != PhysicalEventKind::EDGE_AMBIGUOUS;
      if (e.kind == PhysicalEventKind::PLANE && !out.hit_plane)
        out.hit_plane = event;
      if (out.events.size() < 128)
        out.events.push_back(event);
    }
    propagate(segment_start, step.elapsed_s - cursor);
    covariance = .5 * (covariance + covariance.transpose()).eval();
    if (!covariance.allFinite())
      quality = LinearizationQuality::Unavailable;
    t += step.elapsed_s;
    if (t + 1e-10 >= next_waypoint || step.stopped ||
        t + 1e-10 >= c.horizon_s) {
      waypoint(t);
      next_waypoint += c.trajectory_dt_s;
    }
    if (step.stopped) {
      out.termination_reason = step.reason;
      if (t == 0)
        out.available = false;
      break;
    }
    if (contacts >= c.max_rollout_bounces) {
      out.termination_reason = "bounce_limit";
      break;
    }
    if (out.waypoints.size() >= 2048) {
      out.termination_reason = "waypoint_limit";
      break;
    }
  }
  return out;
}
} // namespace hope_planner_cpp
