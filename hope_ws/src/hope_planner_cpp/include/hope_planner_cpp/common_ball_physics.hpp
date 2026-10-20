#pragma once
#include "hope_planner_cpp/ball_dynamics.hpp"
#include <vector>

namespace hope_planner_cpp {
// Shared C++17 mean/event kernel. Coefficients are acceleration-law
// coefficients.
void validate_ball_physics(const BallPhysics &);
struct ModelState {
  Vec3 position = Vec3::Zero(), velocity = Vec3::Zero();
  EffectiveSpin spin;
};
void integrate_ball(ModelState &, double duration_s, const BallPhysics &);
void apply_model_contact(ModelState &, const BallPhysics &);
enum class PhysicalEventKind {
  TABLE,
  NET_COLLISION,
  NET_CROSSING,
  PLANE,
  END,
  APEX,
  GROUND,
  EDGE_AMBIGUOUS
};
const char *name(PhysicalEventKind);
struct PhysicalEvent {
  PhysicalEventKind kind = PhysicalEventKind::GROUND;
  double dt_s = 0;
  ModelState pre, post;
};
struct EventGeometry {
  TableParams table;
  std::optional<double> receiving_x;
  bool record_end = true, record_apex = true;
};
struct ModelStep {
  std::vector<PhysicalEvent> events;
  bool stopped = false;
  std::string reason;
  double elapsed_s = 0;
};
// Resolves contacts and targets in physical time; collision wins a 1 ns tie.
ModelStep advance_ball(ModelState &, double duration_s, const BallPhysics &,
                       const EventGeometry &);
} // namespace hope_planner_cpp
