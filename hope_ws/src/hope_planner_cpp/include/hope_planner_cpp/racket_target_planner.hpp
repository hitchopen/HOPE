#pragma once

#include "hope_planner_cpp/types.hpp"

namespace hope_planner_cpp {

struct ReturnPrediction {
  Vec3 racket_normal = Vec3::Zero();
  Vec3 outgoing_velocity = Vec3::Zero();
  Vec3 outgoing_spin_rad_s = Vec3::Zero();
  Vec3 bounce_position = Vec3::Zero();
  Vec3 bounce_velocity = Vec3::Zero();
  Vec3 net_position = Vec3::Zero();
  double flight_time_s = 0.0;
  bool net_crossed = false;
  bool valid = false;
};

class RacketTargetPlanner {
 public:
  RacketTargetPlanner(BallPhysics physics, PlannerConfig config, TableParams table);
  RacketCommand plan(const StrikeTarget& strike, const Vec3& target_land,
                     double delta_t_flight_s) const noexcept;
  // The inverse and qualification use this same full forward physics path.
  ReturnPrediction evaluate_return(const StrikeTarget& strike,
                                   const Vec3& racket_velocity) const noexcept;

 private:
  bool valid_parameters() const noexcept;
  bool solve_racket(const StrikeTarget& strike, const Vec3& target_land,
                    double duration_s, Vec3& racket_velocity,
                    ReturnPrediction& prediction) const noexcept;
  Vec3 initial_racket_velocity(const StrikeTarget& strike, const Vec3& target_land,
                               double duration_s) const noexcept;
  void rk4_step(Vec3& position, Vec3& velocity, const Vec3& omega,
                double h) const noexcept;
  void check_net_clearance(const ReturnPrediction& prediction,
                           bool& clears_net, bool& bypasses_posts) const noexcept;
  BallPhysics physics_;
  PlannerConfig config_;
  TableParams table_;
};

}  // namespace hope_planner_cpp
