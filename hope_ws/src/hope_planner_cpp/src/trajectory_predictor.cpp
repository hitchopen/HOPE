#include "hope_planner_cpp/trajectory_predictor.hpp"
#include "hope_planner_cpp/common_ball_physics.hpp"

#include <algorithm>
#include <cmath>

namespace hope_planner_cpp {

TrajectoryPredictor::TrajectoryPredictor(
    BallPhysics physics, PlannerConfig config, TableParams table)
    : physics_(std::move(physics)), config_(std::move(config)), table_(std::move(table)) {validate_ball_physics(physics_);}

double TrajectoryPredictor::prediction_horizon_s(
    const BallState& state, double x_hit) const noexcept {
  const double base = std::max(0.0, config_.max_predict_time_s);
  if (!config_.adaptive_predict_horizon) {
    return base;
  }
  const double distance_x = state.position.x() - x_hit;
  const double incoming_speed_x = -state.velocity.x();
  if (distance_x <= 0.0 || incoming_speed_x <= 1.0e-6) {
    return base;
  }
  return std::max(base, config_.max_predict_time_cap_s);
}

namespace {
StrikeTarget shared_prediction(const BallState& state, double x, const BallPhysics& physics,
    const PlannerConfig& config, const TableParams& table, EffectiveSpin spin) {
  StrikeTarget out;
  if(!state.valid || !state.position.allFinite() || !state.velocity.allFinite() ||
      !std::isfinite(state.source_time_s) || !std::isfinite(x)) {out.reason="invalid_estimate";return out;}
  if(state.position.x()<=x){out.reason="no_hit_plane_crossing";return out;}
  if(state.velocity.x()>=0){out.reason="not_incoming";return out;}
  if(!std::isfinite(config.integrate_dt_s) || config.integrate_dt_s<=0) {out.reason="invalid_integrator_dt";return out;}
  ModelState s;s.position=state.position;s.velocity=state.velocity;s.spin=std::move(spin);
  EventGeometry g;g.table=table;g.receiving_x=x;
  const double horizon=config.adaptive_predict_horizon?std::max(config.max_predict_time_s,config.max_predict_time_cap_s):config.max_predict_time_s;
  int bounces=0;
  for(double t=0;t<horizon-1e-12;) {
    const double h=std::min(config.integrate_dt_s,horizon-t);
    const auto step=advance_ball(s,h,physics,g);
    for(const auto& e:step.events) {
      if(e.kind==PhysicalEventKind::TABLE)++bounces;
      if(e.kind==PhysicalEventKind::PLANE) {
        out.ball_position=e.pre.position;out.ball_velocity=e.pre.velocity;
        out.strike_source_time_s=state.source_time_s+t+e.dt_s;out.predicted_bounces=bounces;
        const bool dead=e.pre.position.z()<.05 && e.pre.velocity.z()<0;
        out.valid=!dead;out.reason=dead?"dead_ball":"prediction_valid";return out;
      }
    }
    if(step.stopped){out.reason=step.reason;return out;}
    t+=h;
  }
  out.reason="hit_plane_not_reached";return out;
}
} // namespace
StrikeTarget TrajectoryPredictor::predict(const BallState& state,double x) const noexcept {
  try {return shared_prediction(state,x,physics_,config_,table_,{});} catch(...) {StrikeTarget s;s.reason="physics_error";return s;}
}
StrikeTarget TrajectoryPredictor::predict_with_spin(const BallState& state,double x,const Vec3& omega,SpinPhysicsMode mode) const noexcept {
  try {
    if(mode==SpinPhysicsMode::kLegacyNoSpin)return predict(state,x);
    EffectiveSpin spin;if(omega.allFinite() && omega.squaredNorm()>0){spin.omega=omega;spin.mode=EffectiveSpinMode::MEASURED;}
    auto p=physics_;
    if(mode==SpinPhysicsMode::kNakashimaBounce || mode==SpinPhysicsMode::kVenueGripBounce)p.magnus_k=0;
    return shared_prediction(state,x,p,config_,table_,spin);
  } catch(...) {StrikeTarget s;s.reason="physics_error";return s;}
}
} // namespace hope_planner_cpp
