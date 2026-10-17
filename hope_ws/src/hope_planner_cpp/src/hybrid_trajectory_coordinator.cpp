#include "hope_planner_cpp/hybrid_trajectory_coordinator.hpp"
#include <cmath>
#include <chrono>
#include <stdexcept>
#include <limits>
namespace hope_planner_cpp {
HybridTrajectoryCoordinator::HybridTrajectoryCoordinator(
    ProbabilisticConfig c, ReturnRegionPolicy p, std::string frame, Sink sink, Sink projection_sink, std::uint64_t generation, std::optional<ForecastNoiseConfig> forecast_noise)
    : config_(std::move(c)), forecast_noise_(forecast_noise.value_or(ForecastNoiseConfig::inherited(config_))), policy_(p), frame_(std::move(frame)),
      config_id_(return_region_config_identity(config_, policy_, frame_)),
      sink_(std::move(sink)), projection_sink_(std::move(projection_sink)),
      plane_generation_(generation), tracker_(config_), machine_(config_) {
  config_id_="nominal-event-v1:sha256:"+sha256(config_id_+";cov=discrete-event-v1;fd_rel=1e-6;quality=3sigma-edge;min_speed=.05");
  config_.validate(); policy_.validate();forecast_noise_.validate();
  if((forecast_noise_.acceleration_spectral_density!=config_.acceleration_spectral_density ||
      forecast_noise_.bounce_velocity_std!=config_.bounce_velocity_std))
    config_id_="nominal-event-v1:sha256:"+sha256(config_id_+";"+forecast_noise_.identity());
  if(generation) config_id_ += ";hit-plane-generation="+std::to_string(generation);
  if (frame_.empty() || frame_.size() > 128 || !sink_)
    throw std::invalid_argument("registered frame and event sink required");
}
double HybridTrajectoryCoordinator::local_time(std::int64_t ns) {
  if (!origin_) origin_ = ns;
  return double((static_cast<long double>(ns) - *origin_) * 1e-9L);
}
StateBelief HybridTrajectoryCoordinator::envelope(std::int64_t ns) const {
  auto b = tracker_.belief();
  b.source_time_origin_ns = origin_.value_or(ns);
  b.source_time_ns = ns;
  b.last_measurement_time_ns = last_observed_ns_.value_or(ns);
  b.frame_id = frame_;
  return b;
}
FlightState HybridTrajectoryCoordinator::output_flight() const {
  auto f=machine_.state();
  f.trajectory_epoch=forecast_epoch_;f.revision_id=forecast_revision_;
  return f;
}
bool HybridTrajectoryCoordinator::emit(ForecastUpdate e) {
  e.config_id = config_id_; return sink_(e);
}
void HybridTrajectoryCoordinator::invalidate(const std::string &reason, std::int64_t ns) {
  ForecastUpdate e;
  e.kind = ForecastUpdate::Invalidation; e.reason = reason;
  e.state = envelope(ns); e.flight = output_flight(); e.history = machine_.bounce_context();
  forecast_expiry_.reset();
  e.config_id=config_id_; if(projection_sink_) projection_sink_(e);
  emit(std::move(e));
}
void HybridTrajectoryCoordinator::reset(std::int64_t ns, const std::string &reason) {
  tracker_.reset();
  machine_.update(nullptr, tracker_.belief(), {true,false,false,true,false,reason});
  last_observed_ns_.reset(); last_output_ = -1e300;
  invalidate(reason, ns);
}
void HybridTrajectoryCoordinator::begin(std::int64_t ns, std::uint64_t rally,
                                        bool serving, bool complete) {
  local_time(ns);
  machine_.begin_rally(rally, serving);
  if(complete)machine_.set_history_continuity(HistoryContinuity::kComplete);
  // Context is metadata: it cannot reset CKF state or publication cadence.
}
bool HybridTrajectoryCoordinator::verified_contact(std::int64_t ns, std::uint64_t rally,
    ContactSide side, const Vec3 &position, const std::string &source) {
  // Verified evidence may describe an earlier contact, but it changes only
  // current history, never past CKF states. Future-time checks belong to ROS.
  if (rally != machine_.state().rally_id ||
      side == ContactSide::kAmbiguous || source.empty()) return false;
  ContactHistoryEvent e;
  e.source_time_ns = ns; e.source_time_s = local_time(ns); e.position = position;
  e.side = side; e.source_id = source; e.epoch = machine_.state().trajectory_epoch;
  e.provenance = ContactProvenance::kExternalSideVerified;
  machine_.add_verified_contact(e);
  return true;
}
void HybridTrajectoryCoordinator::observe(BallSample sample, const std::string &frame,
                                          double age) {
  const auto ns = sample.source_time_ns;
  if (frame != frame_) { reset(ns, "frame_unknown_or_mismatch"); return; }
  if (latest_ns_ && (static_cast<long double>(ns)-*latest_ns_)*1e-9L <
      -config_.clock_reset_backstep_s) {
    reset(ns, "source_epoch_reset"); origin_ = ns; latest_ns_ = ns;
  } else if (!latest_ns_ || ns > *latest_ns_) latest_ns_ = ns;
  sample.source_time_s = local_time(ns);
  auto u = tracker_.observe(sample);
  if(u.changed) {
    if(u.new_track){++forecast_epoch_;forecast_revision_=0;}
    ++forecast_revision_;
  }
  machine_.update(u.accepted ? &sample : nullptr, tracker_.belief(), u);
  if (u.accepted) last_observed_ns_ = ns;
  auto b = envelope(ns);
  b.receipt_steady_ns = sample.receipt_steady_ns; b.receipt_wall_ns = sample.receipt_wall_ns;
  if (!b.initialized) b.source_time_s = sample.source_time_s;
  if (u.changed) {
    ForecastUpdate e; e.state = b; e.flight = output_flight();
    e.update = u; e.history = machine_.bounce_context(); emit(std::move(e));
  }
  if (u.new_track) { invalidate(u.reason, ns); last_output_ = -1e300; }
  if (u.terminated) { invalidate(u.reason, ns); return; }
  if (!u.changed) return;
  if (!b.initialized) {
    invalidate("outgoing_or_ready_stand", ns);
    return;
  }
  if (!std::isfinite(age) || age < -.001 || age > policy_.max_source_age_s) {
    invalidate("source_age_expired", ns); return;
  }
  const double accepted_age = std::max(0.,age) +
      double((static_cast<long double>(ns)-b.last_measurement_time_ns)*1e-9L);
  if (accepted_age > policy_.max_source_age_s) {
    invalidate("accepted_measurement_expired",ns); return;
  }
  {
    auto forecast=forecast_nominal(b,config_,forecast_noise_);
    ForecastUpdate e;e.kind=ForecastUpdate::Trajectory;
    e.state=b;e.flight=output_flight();e.history=machine_.bounce_context();e.update=u;
    e.config_id=config_id_;e.forecast=std::move(forecast);
    e.rollout_complete_steady_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    const long double expiry=static_cast<long double>(b.last_measurement_time_ns)+policy_.max_source_age_s*1e9L;
    forecast_expiry_=expiry<std::numeric_limits<std::int64_t>::max()?
      static_cast<std::int64_t>(expiry):std::numeric_limits<std::int64_t>::max();
    if(projection_sink_) projection_sink_(e);
    if(sample.source_time_s-last_output_+1e-12>=policy_.publication_period_s) {
      last_output_=sample.source_time_s;emit(std::move(e));
    }
    return;
  }
}

void HybridTrajectoryCoordinator::expire(std::int64_t ns) {
  if (!origin_) return;
  if(forecast_expiry_ && ns>=*forecast_expiry_) invalidate("forecast_expired",ns);
  const double t = double((static_cast<long double>(ns)-*origin_)*1e-9L);
  if (tracker_.belief().initialized &&
      t-tracker_.belief().last_measurement_time_s > config_.max_tracking_gap_s) {
    auto u = tracker_.advance_time(t);
    machine_.update(nullptr, tracker_.belief(),u);
    if (u.terminated) invalidate(u.reason,ns);
  }
}
} // namespace hope_planner_cpp
