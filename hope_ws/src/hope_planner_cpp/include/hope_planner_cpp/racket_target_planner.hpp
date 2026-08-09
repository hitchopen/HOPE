#pragma once

#include "hope_planner_cpp/types.hpp"

#include <array>

namespace hope_planner_cpp {

class RacketTargetPlanner {
 public:
  RacketTargetPlanner(BallPhysics physics, PlannerConfig config, TableParams table);

  RacketCommand plan(
      const StrikeTarget& strike,
      const Vec3& target_land,
      double delta_t_flight_s) const noexcept;

 private:
  bool compute_outgoing_velocity(
      const Vec3& strike_position,
      const Vec3& target_land,
      double flight_time_s,
      Vec3& outgoing_velocity) const noexcept;
  bool integrate_free_flight(
      const Vec3& initial_position,
      const Vec3& initial_velocity,
      double duration_s,
      Vec3& final_position,
      Vec3& final_velocity) const noexcept;
  bool integrate_free_flight_batch(
      const Vec3& initial_position,
      const std::array<Vec3, 4>& initial_velocity,
      double duration_s,
      std::array<Vec3, 4>& final_position,
      std::array<Vec3, 4>& final_velocity) const noexcept;
  void compute_racket_velocity(
      const Vec3& incoming,
      const Vec3& outgoing,
      Vec3& racket_velocity,
      Vec3& racket_normal) const noexcept;
  bool free_flight_position_at_x(
      const Vec3& initial_position,
      const Vec3& initial_velocity,
      double target_x,
      Vec3& position_at_x) const noexcept;
  void check_net_clearance(
      const Vec3& strike_position,
      const Vec3& outgoing_velocity,
      bool& clears_net,
      bool& bypasses_posts) const noexcept;

  BallPhysics physics_;
  PlannerConfig config_;
  TableParams table_;
};

}  // namespace hope_planner_cpp
