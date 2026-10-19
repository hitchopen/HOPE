#include "hope_planner_cpp/common_ball_physics.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace hope_planner_cpp {
void validate_ball_physics(const BallPhysics &p) {
  if (!p.gravity.allFinite() || !std::isfinite(p.radius) || p.radius <= 0 ||
      !std::isfinite(p.drag_k) || p.drag_k < 0 || !std::isfinite(p.magnus_k) ||
      p.magnus_k < 0 || !std::isfinite(p.restitution_v) ||
      p.restitution_v < 0 || p.restitution_v > 1 ||
      !std::isfinite(p.table_tangential_gain) || p.table_tangential_gain < 0 ||
      p.table_tangential_gain > .4 || !std::isfinite(p.table_friction_cap_mu) ||
      p.table_friction_cap_mu < 0)
    throw std::invalid_argument("invalid thin-shell-v2 physics (R>0, "
                                "kd/kM/mu>=0, e in [0,1], kt in [0,.4])");
}
const char *name(PhysicalEventKind k) {
  switch (k) {
  case PhysicalEventKind::TABLE:
    return "table_bounce";
  case PhysicalEventKind::NET_COLLISION:
    return "net_collision";
  case PhysicalEventKind::NET_CROSSING:
    return "net_crossing";
  case PhysicalEventKind::PLANE:
    return "receiving_plane";
  case PhysicalEventKind::END:
    return "opponent_end";
  case PhysicalEventKind::APEX:
    return "apex";
  case PhysicalEventKind::EDGE_AMBIGUOUS:
    return "edge_ambiguous";
  default:
    return "ground";
  }
}
void integrate_ball(ModelState &s, double h, const BallPhysics &p) {
  const Vec3 w = s.spin.usable() ? *s.spin.omega : Vec3::Zero();
  const Vec3 v = s.velocity;
  const Vec3 a1 = ball_acceleration(p, v, w);
  const Vec3 v2 = v + .5 * h * a1, a2 = ball_acceleration(p, v2, w);
  const Vec3 v3 = v + .5 * h * a2, a3 = ball_acceleration(p, v3, w);
  const Vec3 v4 = v + h * a3, a4 = ball_acceleration(p, v4, w);
  s.position += h / 6 * (v + 2 * v2 + 2 * v3 + v4);
  s.velocity += h / 6 * (a1 + 2 * a2 + 2 * a3 + a4);
}
void apply_model_contact(ModelState &s, const BallPhysics &p) {
  if (s.spin.usable()) {
    Vec3 w = *s.spin.omega;
    table_contact(p, s.velocity, w);
    s.spin.omega = w;
    s.spin.mode = EffectiveSpinMode::CONTACT_PROPAGATED;
    s.spin.provenance = "thin_shell_contact;constant_free_flight_spin_prior;"
                        "conditional_uncertainty";
  } else {
    const Vec3 slip(s.velocity.x(), s.velocity.y(), 0);
    const double speed = slip.norm();
    const double impulse =
        std::min(p.table_tangential_gain * speed, p.table_friction_cap_mu *
                                                      (1 + p.restitution_v) *
                                                      std::abs(s.velocity.z()));
    if (speed > 1e-12)
      s.velocity -= impulse / speed * slip;
    s.velocity.z() *= -p.restitution_v;
    s.spin = EffectiveSpin{};
  }
}
namespace {
bool top(const Vec3 &p, const TableParams &t) {
  return p.x() >= 0 && p.x() <= t.length && p.y() >= t.y_max - t.width &&
         p.y() <= t.y_max;
}
bool edge(const Vec3 &p, const TableParams &t, double r) {
  const double dx = std::max({-p.x(), p.x() - t.length, 0.});
  const double dy = std::max({t.y_max - t.width - p.y(), p.y() - t.y_max, 0.});
  return dx * dx + dy * dy <= r * r;
}
ModelState at(const ModelState &s, double h, const BallPhysics &p) {
  auto z = s;
  integrate_ball(z, h, p);
  return z;
}
double root(const ModelState &s, double h, const BallPhysics &p, int axis,
            double value, bool velocity = false) {
  double lo = 0, hi = h;
  auto f = [&](double t) {
    auto z = at(s, t, p);
    return (velocity ? z.velocity[axis] : z.position[axis]) - value;
  };
  const double a = f(0);
  for (int i = 0; i < 36; ++i) {
    double m = (lo + hi) * .5;
    if ((f(m) > 0) == (a > 0))
      lo = m;
    else
      hi = m;
  }
  return (lo + hi) * .5;
}
} // namespace
ModelStep advance_ball(ModelState &s, double h, const BallPhysics &p,
                       const EventGeometry &g) {
  ModelStep out;
  if (!std::isfinite(h) || h < 0 || h > .1 || !s.position.allFinite() ||
      !s.velocity.allFinite()) {
    out.stopped = true;
    out.reason = "invalid_state_or_duration";
    return out;
  }
  if (top(s.position, g.table) && s.position.z() < p.radius - 1e-8) {
    out.stopped = true;
    out.reason = "below_table_initial_state";
    return out;
  }
  for (int iteration = 0; out.elapsed_s < h - 1e-12 && iteration < 16;
       ++iteration) {
    const double remaining = h - out.elapsed_s;
    const auto initial = s, next = at(s, remaining, p);
    std::vector<PhysicalEvent> candidates;
    auto add = [&](PhysicalEventKind kind, double t) {
      PhysicalEvent e;
      e.kind = kind;
      e.dt_s = t;
      e.pre = at(initial, t, p);
      e.post = e.pre;
      candidates.push_back(e);
    };
    if (initial.velocity.z() < 0 && initial.position.z() >= p.radius - 1e-10 &&
        next.position.z() <= p.radius) {
      const double t = root(initial, remaining, p, 2, p.radius);
      const auto hit = at(initial, t, p);
      if (top(hit.position, g.table))
        add(PhysicalEventKind::TABLE, t);
      else if (edge(hit.position, g.table, p.radius))
        add(PhysicalEventKind::EDGE_AMBIGUOUS, t);
    }
    // Swept finite radius-expanded net box. Axis roots handle entry through the
    // top/side too.
    const Vec3 lower(g.table.net_x - p.radius,
                     g.table.y_max - g.table.width - g.table.net_overhang -
                         p.radius,
                     -p.radius);
    const Vec3 upper(g.table.net_x + p.radius,
                     g.table.y_max + g.table.net_overhang + p.radius,
                     g.table.net_height + p.radius);
    double enter = 0, exit = remaining;
    bool intersects = true;
    for (int axis = 0; axis < 3; ++axis) {
      const double a = initial.position[axis], b = next.position[axis];
      if (std::max(a, b) < lower[axis] || std::min(a, b) > upper[axis]) {
        intersects = false;
        break;
      }
      if (a < lower[axis])
        enter = std::max(enter, root(initial, remaining, p, axis, lower[axis]));
      if (a > upper[axis])
        enter = std::max(enter, root(initial, remaining, p, axis, upper[axis]));
      if (b < lower[axis])
        exit = std::min(exit, root(initial, remaining, p, axis, lower[axis]));
      if (b > upper[axis])
        exit = std::min(exit, root(initial, remaining, p, axis, upper[axis]));
    }
    if (intersects && enter <= exit + 1e-12)
      add(PhysicalEventKind::NET_COLLISION, enter);
    auto crossing = [&](double x, PhysicalEventKind kind, int direction) {
      const double a = initial.position.x() - x, b = next.position.x() - x;
      if ((direction < 0 && a > 0 && b <= 0) ||
          (direction > 0 && a < 0 && b >= 0) ||
          (direction == 0 && a * b <= 0 && a != 0 && a != b))
        add(kind, root(initial, remaining, p, 0, x));
    };
    crossing(g.table.net_x, PhysicalEventKind::NET_CROSSING, 0);
    if (g.receiving_x)
      crossing(*g.receiving_x, PhysicalEventKind::PLANE, -1);
    if (g.record_end)
      crossing(g.table.length, PhysicalEventKind::END, 1);
    if (g.record_apex && initial.velocity.z() > 1e-10 && next.velocity.z() <= 0)
      add(PhysicalEventKind::APEX, root(initial, remaining, p, 2, 0, true));
    if (initial.position.z() > 0 && next.position.z() <= 0)
      add(PhysicalEventKind::GROUND, root(initial, remaining, p, 2, 0));
    std::stable_sort(
        candidates.begin(), candidates.end(), [](const auto &a, const auto &b) {
          const bool ac = a.kind == PhysicalEventKind::TABLE ||
                          a.kind == PhysicalEventKind::NET_COLLISION ||
                          a.kind == PhysicalEventKind::EDGE_AMBIGUOUS;
          const bool bc = b.kind == PhysicalEventKind::TABLE ||
                          b.kind == PhysicalEventKind::NET_COLLISION ||
                          b.kind == PhysicalEventKind::EDGE_AMBIGUOUS;
          const auto at = std::llround(a.dt_s * 1e9),
                     bt = std::llround(b.dt_s * 1e9);
          if (at == bt && ac != bc)
            return ac;
          return a.dt_s < b.dt_s;
        });
    auto collision =
        std::find_if(candidates.begin(), candidates.end(), [](const auto &e) {
          return e.kind == PhysicalEventKind::TABLE ||
                 e.kind == PhysicalEventKind::NET_COLLISION ||
                 e.kind == PhysicalEventKind::GROUND ||
                 e.kind == PhysicalEventKind::EDGE_AMBIGUOUS;
        });
    const double limit =
        collision == candidates.end() ? remaining : collision->dt_s;
    for (auto e : candidates) {
      if (e.dt_s > limit + 1e-10)
        break;
      if (e.kind == PhysicalEventKind::TABLE) {
        e.pre.position.z() = p.radius;
        e.post = e.pre;
        apply_model_contact(e.post, p);
      }
      e.dt_s += out.elapsed_s;
      out.events.push_back(e);
      if (e.kind == PhysicalEventKind::TABLE ||
          e.kind == PhysicalEventKind::NET_COLLISION ||
          e.kind == PhysicalEventKind::GROUND ||
          e.kind == PhysicalEventKind::EDGE_AMBIGUOUS)
        break;
    }
    if (collision == candidates.end()) {
      s = next;
      out.elapsed_s = h;
      return out;
    }
    s = out.events.back().post;
    out.elapsed_s += limit;
    if (collision->kind != PhysicalEventKind::TABLE) {
      out.stopped = true;
      out.reason = name(collision->kind);
      return out;
    }
    if (limit <= 1e-12 && s.velocity.z() <= 1e-9) {
      out.stopped = true;
      out.reason = "contact_rest_ambiguous";
      return out;
    }
  }
  if (out.elapsed_s < h - 1e-10) {
    out.stopped = true;
    out.reason = "contact_work_limit";
  }
  return out;
}
} // namespace hope_planner_cpp
