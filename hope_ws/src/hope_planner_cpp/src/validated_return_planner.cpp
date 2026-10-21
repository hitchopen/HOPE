#include "hope_planner_cpp/validated_return_planner.hpp"
#include <Eigen/LU>
#include <cmath>
#include <stdexcept>
#include <sstream>
#include <locale>

namespace hope_planner_cpp {
PaddleImpact reconstruct_paddle_impact(const Vec3 &in, const Vec3 &r, const Vec3 &n, double e) {
  PaddleImpact out;
  if (!in.allFinite() || !r.allFinite() || !n.allFinite() ||
      !std::isfinite(e) || e < 0 || e > 1 || std::abs(n.norm()-1) > 1e-9 || n.x() <= 0)
    return out;
  const double relative = (in-r).dot(n);
  if (relative >= -1e-6) { out.reason = "PADDLE_NOT_APPROACHING"; return out; }
  out.outgoing = in - (1+e)*relative*n;
  out.valid = out.outgoing.allFinite(); out.reason = out.valid ? "IMPACT_VALID" : "IMPACT_NONFINITE";
  return out;
}
void ValidatedReturnSettings::validate(const ProbabilisticConfig &c) const {
  if (!landing_surface.allFinite() || landing_surface.z() != 0 ||
      landing_surface.x() <= c.table.net_x+c.physics.radius ||
      landing_surface.x() >= c.table.length-c.physics.radius ||
      landing_surface.y() <= c.table.y_max-c.table.width+c.physics.radius ||
      landing_surface.y() >= c.table.y_max-c.physics.radius ||
      max_iterations < 1 || max_iterations > 40 || durations_s.empty() || durations_s.size() > 8)
    throw std::invalid_argument("invalid return objective/work limit");
  for (double v : {endpoint_tolerance_m, landing_tolerance_m, time_tolerance_s,
                   impact_tolerance_mps, restitution, max_racket_speed_mps, max_ball_speed_mps})
    if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("invalid return limits");
  if (restitution > 1 || endpoint_tolerance_m > .001 || landing_tolerance_m > .001 ||
      time_tolerance_s > .0005 || impact_tolerance_mps > 1e-5)
    throw std::invalid_argument("return numerical budget exceeded");
  for (double t : durations_s)
    if (!std::isfinite(t) || t < .1 || t > 1.5) throw std::invalid_argument("invalid outgoing duration");
}
ValidatedReturnPlanner::ValidatedReturnPlanner(ProbabilisticConfig c, ValidatedReturnSettings s)
  : config_(std::move(c)), settings_(std::move(s)) {
  config_.validate(); settings_.validate(config_);
  std::ostringstream id;id.imbue(std::locale::classic());id<<std::hexfloat;
  id<<flight_model_config_id(config_,{},"table")<<";normal-only-constant-e-v1;outgoing-POSITION_ONLY;";
  for (double value : {settings_.restitution, settings_.endpoint_tolerance_m,
      settings_.landing_tolerance_m, settings_.time_tolerance_s, settings_.impact_tolerance_mps,
      settings_.max_racket_speed_mps, settings_.max_ball_speed_mps, settings_.landing_surface.x(),
      settings_.landing_surface.y()}) id<<value<<';';
  id<<settings_.max_iterations<<';';
  for (double duration : settings_.durations_s) id<<duration<<';';
  return_config_id_="shared-return-v1:sha256:"+sha256(id.str());
}
Vec3 ValidatedReturnPlanner::endpoint(const Vec3 &p, const Vec3 &v, double t) const {
  ModelState state; state.position=p; state.velocity=v; // POSITION_ONLY after paddle.
  for (double elapsed=0; elapsed < t-1e-12;) {
    const double h=std::min(config_.integration_dt_s,t-elapsed);
    integrate_ball(state,h,config_.physics); elapsed+=h;
  }
  return state.position;
}
bool ValidatedReturnPlanner::validate_action(const ContactPlanningInput &input, ValidatedReturn &out) const {
  out.command.valid=false; out.impact_ok=false; out.return_ok=false;
  auto impact=reconstruct_paddle_impact(input.contact.pre.velocity,out.command.velocity,
                                        out.command.normal,settings_.restitution);
  if (!impact.valid) { out.reason=impact.reason; return false; }
  out.impact_residual_mps=(impact.outgoing-out.command.outgoing_ball_velocity).norm();
  if (out.impact_residual_mps > settings_.impact_tolerance_mps ||
      impact.outgoing.norm()>settings_.max_ball_speed_mps ||
      out.command.velocity.norm()>settings_.max_racket_speed_mps ||
      impact.outgoing.x()<=0 || out.command.velocity.x()<=0 ||
      (out.command.normal-out.command.velocity.normalized()).norm()>1e-9) {
    out.reason="IMPACT_OR_VELOCITY_CONTRACT"; return false;
  }
  out.impact_ok=true;
  ModelState state; state.position=input.contact.pre.position; state.velocity=impact.outgoing;
  state.spin.provenance="paddle_unknown_omega;POSITION_ONLY_effective_outgoing";
  out.reconstructed=state;
  EventGeometry geometry; geometry.table=config_.table; geometry.record_end=false;
  bool crossed=false;
  for (double t=0; t<1.5-1e-12;) {
    const double h=std::min(config_.integration_dt_s,1.5-t);
    auto step=advance_ball(state,h,config_.physics,geometry);
    for (const auto &e : step.events) {
      if (e.kind==PhysicalEventKind::NET_COLLISION || e.kind==PhysicalEventKind::EDGE_AMBIGUOUS ||
          e.kind==PhysicalEventKind::GROUND) { out.reason=name(e.kind); return false; }
      if (e.kind==PhysicalEventKind::NET_CROSSING) {
        // Baseline deliberately excludes around-post paths.
        if (e.pre.position.y()<config_.table.y_max-config_.table.width ||
            e.pre.position.y()>config_.table.y_max) { out.reason="AROUND_POST_UNSUPPORTED"; return false; }
        crossed=true; out.net_clearance_m=e.pre.position.z()-config_.table.net_height-config_.physics.radius;
      }
      if (e.kind==PhysicalEventKind::TABLE) {
        out.first_landing=e.pre.position; out.landing_time_s=t+e.dt_s;
        const Vec3 target=settings_.landing_surface+Vec3(0,0,config_.physics.radius);
        if (!crossed || e.pre.position.x()<=config_.table.net_x ||
            e.pre.velocity.z()>=0 || (out.first_landing-target).norm()>settings_.landing_tolerance_m ||
            std::abs(out.landing_time_s-out.selected_duration_s)>settings_.time_tolerance_s) {
          out.reason="FIRST_LANDING_MISMATCH"; return false;
        }
        out.return_ok=true; out.command.clears_net=true;
        out.command.valid=out.solve_ok && out.impact_ok; out.reason="RETURN_VALID";
        out.command.reason=out.reason;
        return out.command.valid;
      }
    }
    if (step.stopped) { out.reason=step.reason; return false; }
    t+=h;
  }
  out.reason="LANDING_HORIZON_EXHAUSTED"; return false;
}
ValidatedReturn ValidatedReturnPlanner::solve(const ContactPlanningInput &input) const {
  ValidatedReturn out; out.model_config_id=input.model_config_id+";"+return_config_id_;
  out.command.target_land=settings_.landing_surface;
  out.command.predicted_bounces=1;
  out.command.strike_source_time_s=static_cast<double>(input.arrival_ns)*1e-9;
  const Vec3 p=input.contact.pre.position, in=input.contact.pre.velocity;
  if (!p.allFinite() || !in.allFinite() || p.z()<=config_.physics.radius || in.x()>=-.5) {
    out.reason="INVALID_INCOMING_CONTACT"; return out;
  }
  const Vec3 target=settings_.landing_surface+Vec3(0,0,config_.physics.radius);
  for (double duration : settings_.durations_s) {
    ++out.attempts; out.solve_ok=false; out.impact_ok=false; out.return_ok=false;
    out.selected_duration_s=duration;
    Vec3 velocity=(target-p-.5*config_.physics.gravity*duration*duration)/duration;
    for (int iteration=0; iteration<settings_.max_iterations; ++iteration) {
      ++out.iterations;
      const Vec3 residual=endpoint(p,velocity,duration)-target;
      if (!residual.allFinite()) break;
      if (residual.norm()<=settings_.endpoint_tolerance_m) break;
      Eigen::Matrix3d jacobian;
      for (int axis=0;axis<3;++axis) {
        Vec3 shifted=velocity; shifted[axis]+=1e-4;
        jacobian.col(axis)=(endpoint(p,shifted,duration)-target-residual)/1e-4;
      }
      auto lu=jacobian.fullPivLu();
      if (!lu.isInvertible()) break;
      Vec3 correction=lu.solve(residual);
      if (!correction.allFinite() || correction.norm()>20) break;
      velocity-=correction;
      if (!velocity.allFinite() || velocity.norm()>settings_.max_ball_speed_mps) break;
    }
    // Never accept a finite final iterate without this independent final test.
    out.solver_residual_m=(endpoint(p,velocity,duration)-target).norm();
    if (!std::isfinite(out.solver_residual_m) || out.solver_residual_m>settings_.endpoint_tolerance_m) {
      out.reason="INVERSE_NONCONVERGENCE"; continue;
    }
    out.solve_ok=true;
    Vec3 normal=velocity-in;
    if (normal.norm()<1e-8) { out.reason="ZERO_IMPULSE"; continue; }
    normal.normalize(); if (normal.x()<0) normal=-normal;
    const double speed=(velocity.dot(normal)+settings_.restitution*in.dot(normal))/(1+settings_.restitution);
    out.command.position=p; // Actor contract uses ball-center intercept, not face center.
    out.command.velocity=speed*normal; out.command.normal=normal;
    out.command.outgoing_ball_velocity=velocity;
    if (validate_action(input,out)) return out;
  }
  out.command.valid=false; return out;
}
} // namespace hope_planner_cpp
