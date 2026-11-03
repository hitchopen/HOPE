#include "hope_planner_cpp/flight_state_v2.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
namespace hope_planner_cpp {
const char *name(PlayState s) {
  switch (s) {
  case PlayState::OPPONENT_RALLY:
    return "OPPONENT_RALLY";
  case PlayState::RALLY:
    return "RALLY";
  case PlayState::OPPONENT_SERVE:
    return "OPPONENT_SERVE";
  case PlayState::SERVE:
    return "SERVE";
  default:
    return "INVALID";
  }
}
void ShotSettings::validate() const {
  if (observation_capacity < 6 || observation_capacity > 4096 ||
      candidate_capacity < 6 || candidate_capacity > 4096 ||
      event_capacity < 8 || event_capacity > 4096)
    throw std::invalid_argument("invalid shot capacities");
  for (double v :
       {observation_window_s, qualification_span_s, direction_speed_mps,
        launch_speed_mps, ballistic_residual_mps2, contact_association_s,
        contact_association_m, contact_exclusion_s, held_span_s,
        held_displacement_m, max_source_age_s, supported_sigma,
        supported_time_margin_s})
    if (!std::isfinite(v) || v <= 0)
      throw std::invalid_argument("invalid shot threshold");
}
bool checked_arrival(std::int64_t r, std::int64_t d, std::int64_t &a) {
  if (d < 0 || r > std::numeric_limits<std::int64_t>::max() - d)
    return false;
  a = r + d;
  return true;
}
std::string flight_model_config_id(const ProbabilisticConfig &c,
                                   const ShotSettings &s,
                                   const std::string &frame) {
  c.validate();
  s.validate();
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::hexfloat;
  out << kFlightPhysicsProfile << ';'
      << return_region_config_identity(c, {}, frame);
  for (double value :
       {s.observation_window_s, s.qualification_span_s, s.direction_speed_mps,
        s.launch_speed_mps, s.ballistic_residual_mps2, s.contact_association_s,
        s.contact_association_m, s.contact_exclusion_s, s.held_span_s,
        s.held_displacement_m, s.max_source_age_s, s.supported_sigma,
        s.supported_time_margin_s})
    out << ';' << value;
  out << ';' << s.observation_capacity << ';' << s.candidate_capacity << ';'
      << s.event_capacity << ';' << s.service_armed;
  if (s.game_legality_advisory) out << ";game_legality_advisory";
  if (s.automatic_rearm) out << ";automatic_rearm";
  out << ";bounded-causal-candidate-spin-v2";
  return "flight-model-v3:sha256:" + sha256(out.str());
}
void set_flight_parameter(ProbabilisticConfig &c, ShotSettings &s,
                          const std::string &key, double value) {
  if (!std::isfinite(value))
    throw std::invalid_argument("nonfinite flight parameter");
  if (key == "automatic_rearm_enabled") {
    if (value != 0 && value != 1) throw std::invalid_argument("automatic_rearm_enabled must be 0 or 1");
    s.automatic_rearm = value == 1;
    return;
  }
  double *target = nullptr;
  const std::pair<const char *, double *> options[] = {
      {"physics.drag_k", &c.physics.drag_k},
      {"physics.magnus_k", &c.physics.magnus_k},
      {"physics.restitution_v", &c.physics.restitution_v},
      {"physics.table_tangential_gain", &c.physics.table_tangential_gain},
      {"physics.table_friction_cap_mu", &c.physics.table_friction_cap_mu},
      {"physics.radius", &c.physics.radius},
      {"physics.gravity_x", &c.physics.gravity.x()},
      {"physics.gravity_y", &c.physics.gravity.y()},
      {"physics.gravity_z", &c.physics.gravity.z()},
      {"integration_dt_s", &c.integration_dt_s},
      {"trajectory_dt_s", &c.trajectory_dt_s},
      {"horizon_s", &c.horizon_s},
      {"x_hit", &c.x_hit},
      {"table.length", &c.table.length},
      {"table.width", &c.table.width},
      {"table.y_max", &c.table.y_max},
      {"table.net_x", &c.table.net_x},
      {"table.net_height", &c.table.net_height},
      {"table.net_overhang", &c.table.net_overhang},
      {"max_source_age_s", &s.max_source_age_s}};
  for (const auto &entry : options)
    if (key == entry.first)
      target = entry.second;
  if (!target)
    throw std::invalid_argument("unknown flight parameter: " + key);
  const double previous = *target;
  *target = value;
  try {
    c.validate();
    s.validate();
  } catch (...) {
    *target = previous;
    throw;
  }
}
std::optional<std::int64_t> duration_ns(double s) {
  const long double n = static_cast<long double>(s) * 1000000000.L;
  if (!std::isfinite(s) || n < 0 ||
      n > std::numeric_limits<std::int64_t>::max() - .5L)
    return {};
  return static_cast<std::int64_t>(std::round(n));
}
ShotContext::ShotContext(ProbabilisticConfig c, ShotSettings s)
    : config_(std::move(c)), settings_(s) {
  config_.validate();
  s.validate();
}
void ShotContext::changed(const std::string &r) {
  ++revision_;
  reason_ = r;
}
void ShotContext::start(std::int64_t ns, int d, BoundaryEvidence evidence,
                        HistoryContinuity continuity, const std::string &why) {
  if (d != 1 && d != -1)
    throw std::invalid_argument("qualified boundary requires direction");
  if (rejected_latch_ && evidence == BoundaryEvidence::ACQUISITION &&
      !(settings_.automatic_rearm && rearm_candidate_)) return;
  const bool acquisition = evidence == BoundaryEvidence::ACQUISITION;
  auto angular_history = candidate_spin_;
  ++generation_;
  revision_ = 1;
  published_sequence_ = 0;
  boundary_ns_ = ns;
  boundary_qualified_ = evidence != BoundaryEvidence::ACQUISITION;
  direction_ = d;
  net_crossed_ = false;
  rejected_latch_ = false;
  rearm_candidate_ = false;
  rejection_fence_.reset(); rejection_reason_.clear();
  candidate_last_.reset(); quiet_anchor_.reset();
  acquisition_spin_ = {}; acquisition_spin_cutoff_ns_ = 0;
  samples_.clear();
  events_.clear();
  spin_.reset();
  match_ = MatchContext::UNKNOWN;
  match_source_.clear();
  match_time_ns_ = 0;
  state_ = route_ = d > 0 ? PlayState::RALLY : PlayState::OPPONENT_RALLY;
  classification_ = ClassificationStatus::PROVISIONAL;
  continuity_ = continuity;
  reason_ = why;
  auto candidates = std::move(candidates_);
  candidates_.clear();
  for (auto sample : candidates)
    if (sample.source_time_ns >= ns)
      observe(std::move(sample));
  if (acquisition && active()) spin_ = std::move(angular_history);
  candidate_spin_.reset();
}
void ShotContext::invalidate(const std::string &why) {
  // Retain only a source-identity/time fence, never the old estimation ledger.
  if (!samples_.empty()) rejection_fence_ = samples_.back();
  rejection_reason_ = why;
  candidate_last_ = rejection_fence_;
  quiet_anchor_.reset(); rearm_candidate_ = false;
  candidate_spin_.reset(); acquisition_spin_ = {}; acquisition_spin_cutoff_ns_ = 0;
  ++generation_;
  ++revision_;
  state_ = route_ = PlayState::INVALID;
  direction_ = 0;
  net_crossed_ = false;
  samples_.clear();
  events_.clear();
  candidates_.clear();
  spin_.reset();
  match_ = MatchContext::UNKNOWN;
  continuity_ = HistoryContinuity::kUnknown;
  rejected_latch_ = true;
  reason_ = why;
}
bool ShotContext::reject_supported_prediction(
    PhysicalEventKind kind, const PredictedViolationBound &bound) {
  if (!active() || !bound.all_relevant_uncertainty_bounded ||
      !std::isfinite(bound.forbidden_latest_s) ||
      !std::isfinite(bound.target_earliest_s) || bound.forbidden_latest_s < 0 ||
      bound.forbidden_latest_s + settings_.supported_time_margin_s >=
          bound.target_earliest_s)
    return false;
  if (kind == PhysicalEventKind::NET_COLLISION &&
      (!bound.finite_net_geometry ||
       !std::isfinite(bound.net_clearance_upper_m) ||
       bound.net_clearance_upper_m >= 0))
    return false;
  if (kind != PhysicalEventKind::NET_COLLISION &&
      kind != PhysicalEventKind::TABLE)
    return false;
  invalidate("supported_predicted_forbidden_event");
  return true;
}
bool ShotContext::compatible(const BallSample &a, const BallSample &b) const {
  return a.frame_id == b.frame_id && a.calibration_id == b.calibration_id &&
         a.clock_domain == b.clock_domain &&
         a.clock_mapping_id == b.clock_mapping_id && a.ball_id == b.ball_id;
}
bool ShotContext::collect_candidate(const BallSample &s) {
  if (!s.clock_mapping_valid || !std::isfinite(s.clock_uncertainty_s) ||
      s.clock_uncertainty_s < 0 || s.clock_uncertainty_s > .001) return false;
  if (rejected_latch_ && rejection_fence_ && !compatible(*rejection_fence_,s)) return false;
  if (candidate_last_ && (s.source_time_ns <= candidate_last_->source_time_ns ||
      !compatible(*candidate_last_,s))) return false;
  const double gap = candidate_last_ ? double((static_cast<long double>(s.source_time_ns)-
      candidate_last_->source_time_ns)*1e-9L) : 0;
  bool quiet_launch = false, angular_boundary = false;
  if (quiet_anchor_ && (s.position-quiet_anchor_->position).norm() > settings_.held_displacement_m) {
    const double quiet_span = candidate_last_ ? double((static_cast<long double>(candidate_last_->source_time_ns)-
        quiet_anchor_->source_time_ns)*1e-9L) : 0;
    quiet_launch = quiet_span >= settings_.held_span_s;
    angular_boundary = quiet_span >= settings_.qualification_span_s;
    quiet_anchor_ = s;
  }
  if (!quiet_anchor_) quiet_anchor_ = s;
  candidate_last_ = s;
  bool new_opportunity = false;
  if (rejected_latch_ && settings_.automatic_rearm && !rearm_candidate_) {
    if (!rejection_fence_ || rejection_reason_ == "source_clock_discontinuity") return false;
    new_opportunity = gap > config_.max_tracking_gap_s || quiet_launch;
    if (!new_opportunity) return false;
    rearm_candidate_ = true;
    reason_ = gap > config_.max_tracking_gap_s ? "rearm_qualified_source_gap" : "rearm_held_to_launch";
  }
  if (new_opportunity || gap > config_.gap_warning_s || angular_boundary ||
      s.position.z() <= config_.physics.radius + .03) {
    candidates_.clear(); candidate_spin_.reset();
    quiet_anchor_ = s;
  }
  candidate_spin_.observe(s);
  candidates_.push_back(s);
  while (candidates_.size() > settings_.candidate_capacity) candidates_.pop_front();
  return true;
}
void ShotContext::rebase_times(std::int64_t origin) {
  if (samples_.empty()) return;
  const double offset = samples_.front().source_time_s - double(
      (static_cast<long double>(samples_.front().source_time_ns)-origin)*1e-9L);
  for (auto &s : samples_) s.source_time_s -= offset;
  for (auto &s : candidates_) s.source_time_s -= offset;
  spin_.rebase_time(offset); candidate_spin_.rebase_time(offset);
}
bool ShotContext::observe(BallSample s) {
  if (!s.source_time_valid || !s.position_valid || !s.position.allFinite() ||
      !std::isfinite(s.source_time_s))
    return false;
  if (active() && !samples_.empty()) {
    if (!compatible(samples_.back(), s)) {
      invalidate("incompatible_provenance");
      return false;
    }
    if (s.source_time_ns <= samples_.back().source_time_ns) {
      const long double delta =
          static_cast<long double>(samples_.back().source_time_ns) -
          s.source_time_ns;
      if (delta > config_.clock_reset_backstep_s * 1e9L)
        invalidate("source_clock_discontinuity");
      return false;
    }
    if ((static_cast<long double>(s.source_time_ns)-samples_.back().source_time_ns)*1e-9L >
        config_.max_tracking_gap_s) {
      // Beyond the track's supported gap this is a new acquisition, not an
      // extra bounce appended to the old shot. Resume only after the ordinary
      // multi-sample ballistic qualifier; a single resumed point is no launch.
      invalidate("source_gap_requires_new_acquisition");
      // Standalone retains the established active-gap recovery contract.
      // Integrated automatic rearm goes through the same fenced helper as an
      // already-rejected shot, rather than clearing the latch here.
      if (!settings_.automatic_rearm) rejected_latch_ = false;
    }
  }
  if (!active()) {
    collect_candidate(s);
    return false;
  }
  if (s.source_time_ns < boundary_ns_)
    return false;
  s.shot_generation = generation_;
  if (!s.clock_mapping_valid) {
    changed("clock_mapping_unavailable");
    return false;
  }
  if (s.position.z() +
              settings_.supported_sigma * config_.measurement_std_m.z() <
          0 &&
      s.position.x() >= 0 && s.position.x() <= config_.table.length &&
      s.position.y() >= config_.table.y_max - config_.table.width &&
      s.position.y() <= config_.table.y_max) {
    invalidate("observed_under_table");
    return false;
  }
  if (!samples_.empty() && continuity_ == HistoryContinuity::kComplete &&
      (static_cast<long double>(s.source_time_ns) -
       samples_.back().source_time_ns) *
              1e-9L >
          config_.gap_warning_s)
    set_history_continuity(HistoryContinuity::kIncomplete);
  spin_.observe(s);
  samples_.push_back(s);
  while (samples_.size() > settings_.observation_capacity ||
         (samples_.size() > 1 && (static_cast<long double>(s.source_time_ns) -
                                  samples_.front().source_time_ns) *
                                         1e-9L >
                                     settings_.observation_window_s))
    samples_.pop_front();
  if (samples_.size() >= 2) {
    const auto &before = samples_[samples_.size() - 2];
    if ((direction_ > 0 && before.position.x() < config_.table.net_x &&
         s.position.x() >= config_.table.net_x) ||
        (direction_ < 0 && before.position.x() > config_.table.net_x &&
         s.position.x() <= config_.table.net_x)) {
      const double f = (config_.table.net_x - before.position.x()) /
                       (s.position.x() - before.position.x());
      ModelState net;
      net.position = before.position + f * (s.position - before.position);
      const long double interval =
          static_cast<long double>(s.source_time_ns) - before.source_time_ns;
      net.velocity = (s.position - before.position) / (double(interval) * 1e-9);
      const auto ns = before.source_time_ns +
                      static_cast<std::int64_t>(std::round(f * interval));
      const bool finite_net = finite_net_distance(net.position, config_.table) <=
                              config_.physics.radius + 1e-10;
      confirm(finite_net ? PhysicalEventKind::NET_COLLISION
                         : PhysicalEventKind::NET_CROSSING,
              ns, s.source_time_ns, net, "source_bracket");
      if (!active())
        return false;
    }
  }
  if (samples_.size() >= 3) {
    const auto a = samples_[samples_.size() - 3],
               b = samples_[samples_.size() - 2], c = samples_.back();
    const double h1 =
        double((static_cast<long double>(b.source_time_ns) - a.source_time_ns) *
               1e-9L);
    const double h2 =
        double((static_cast<long double>(c.source_time_ns) - b.source_time_ns) *
               1e-9L);
    if (h1 > 0 && h2 > 0) {
      const Vec3 v1 = (b.position - a.position) / h1,
                 v2 = (c.position - b.position) / h2;
      const Vec3 acceleration = 2 * (v2 - v1) / (h1 + h2),
                 vb = v1 + .5 * h1 * acceleration;
      if (v1.z() > 0 && v2.z() < 0 && acceleration.z() < -.1) {
        const double dt = -vb.z() / acceleration.z();
        if (dt >= -h1 && dt <= h2) {
          ModelState apex;
          apex.position = b.position + dt * vb + .5 * dt * dt * acceleration;
          apex.velocity = vb + dt * acceleration;
          apex.spin = spin_.value();
          const auto occurrence = b.source_time_ns + static_cast<std::int64_t>(
                                                         std::round(dt * 1e9));
          confirm(PhysicalEventKind::APEX, occurrence, c.source_time_ns, apex,
                  "source_velocity_root");
        }
      }
    }
  }
  // Elapsed source time and spatial dispersion distinguish a held ball from an
  // apex.
  if (samples_.size() > 3 && (static_cast<long double>(s.source_time_ns) -
                              samples_.front().source_time_ns) *
                                     1e-9L >=
                                 settings_.held_span_s) {
    double movement = 0;
    for (const auto &x : samples_)
      movement = std::max(movement, (x.position - s.position).norm());
    if (movement < settings_.held_displacement_m) {
      invalidate("qualified_held_non_ballistic");
      return false;
    }
    const auto a = samples_.front(), b = samples_[samples_.size() / 2],
               c = samples_.back();
    const double h1 = double((static_cast<long double>(b.source_time_ns) -
                              a.source_time_ns) *
                             1e-9L),
                 h2 = double((static_cast<long double>(c.source_time_ns) -
                              b.source_time_ns) *
                             1e-9L);
    if (h1 > 0 && h2 > 0) {
      const Vec3 v1 = (b.position - a.position) / h1,
                 v2 = (c.position - b.position) / h2, velocity = (v1 + v2) * .5;
      const Vec3 acceleration = 2 * (v2 - v1) / (h1 + h2);
      const auto omega =
          spin_.value().usable() ? *spin_.value().omega : Vec3::Zero();
      const Vec3 expected = ball_acceleration(config_.physics, velocity, omega);
      bool contact_window = v1.z() < 0 && v2.z() > 0;
      // A smooth-window acceleration test cannot classify an interval that
      // contains a plausible unresolved top contact. Keep forecasting until
      // reversal evidence resolves it; held-ball rejection above still applies.
      for (const auto &point : samples_)
        if (point.position.z() >= config_.physics.radius - .01 &&
            point.position.z() <= config_.physics.radius + .03 &&
            point.position.x() >= 0 &&
            point.position.x() <= config_.table.length &&
            point.position.y() >= config_.table.y_max - config_.table.width &&
            point.position.y() <= config_.table.y_max)
          contact_window = true;
      for (const auto &event : events_)
        if (event.status == EventStatus::CONFIRMED &&
            event.kind == PhysicalEventKind::TABLE &&
            event.event_time_ns >= a.source_time_ns &&
            event.event_time_ns <= c.source_time_ns)
          contact_window = true;
      // A tracking gap can hide a contact without a measured reversal or a
      // sample near table height. Its secant velocities do not establish one
      // smooth acceleration. History is already incomplete; preserve XYZ and
      // withhold this rejection until the gap leaves the retained window.
      bool continuous_window = true;
      for (std::size_t i = 1; i < samples_.size(); ++i)
        if ((static_cast<long double>(samples_[i].source_time_ns) -
             samples_[i - 1].source_time_ns) * 1e-9L > config_.gap_warning_s) {
          continuous_window = false;
          break;
        }
      if (continuous_window && !contact_window &&
          (acceleration - expected).norm() > settings_.ballistic_residual_mps2) {
        invalidate("qualified_non_ballistic_motion");
        return false;
      }
    }
  }
  return true;
}
bool ShotContext::qualify_candidate() {
  if (active() || (rejected_latch_ && !(settings_.automatic_rearm && rearm_candidate_)) || settings_.service_armed ||
      candidates_.size() < 6)
    return false;
  // Qualify a continuous recent launch window. Older held-ball candidates are
  // not observations of this flight and must not dilute its curvature for an
  // entire candidate-capacity interval after release.
  std::size_t first = candidates_.size()-6;
  while (first > 0 &&
         (static_cast<long double>(candidates_.back().source_time_ns)-candidates_[first].source_time_ns)*1e-9L <
             settings_.qualification_span_s) --first;
  const auto &a = candidates_[first];
  const auto &b = candidates_[first+(candidates_.size()-first)/2];
  const auto &c = candidates_.back();
  const double ab = double(
      (static_cast<long double>(b.source_time_ns) - a.source_time_ns) * 1e-9L);
  const double bc = double(
      (static_cast<long double>(c.source_time_ns) - b.source_time_ns) * 1e-9L);
  if (ab <= 0 || bc <= 0 || ab + bc < settings_.qualification_span_s)
    return false;
  const Vec3 v1 = (b.position - a.position) / ab,
             v2 = (c.position - b.position) / bc;
  const Vec3 v = (v1 + v2) * .5, accel = 2 * (v2 - v1) / (ab + bc);
  if (std::abs(v.x()) < settings_.launch_speed_mps ||
      v.norm() < settings_.launch_speed_mps)
    return false;
  // A smooth acceleration hypothesis requires continuous contact-free data.
  // Only spin qualified by the middle source cutoff can explain its curvature.
  // Candidate angular history is independently retained across short dynamic windows.
  for (std::size_t i = first; i < candidates_.size(); ++i) {
    const auto &point = candidates_[i];
    if ((i > first && (static_cast<long double>(point.source_time_ns) -
               candidates_[i - 1].source_time_ns) * 1e-9L > config_.gap_warning_s) ||
        point.position.z() <= config_.physics.radius + .03) {
      reason_ = "acquisition_discontinuous_or_contact_window";
      return false;
    }

  }
  const auto spin = candidate_spin_.at(b.source_time_ns);
  const auto cutoff = b.source_time_ns;
  const Vec3 expected = ball_acceleration(config_.physics, v,
      spin.usable() ? *spin.omega : Vec3::Zero());
  if ((accel - expected).norm() > settings_.ballistic_residual_mps2) {
    reason_ = spin.usable() ? "acquisition_qualified_spin_model_residual" :
        candidate_spin_.readiness().state == "COLLECTING" ? "SPIN_COLLECTING_MODEL_NOT_YET_SUPPORTED" :
        "MODEL_UNSUPPORTED_WHILE_SPIN_UNKNOWN";
    return false;
  }
  // Acquisition does not establish racket contact or complete prehistory.
  const auto boundary = candidates_.front().source_time_ns;
  const int d = v.x() > 0 ? 1 : -1;
  start(boundary, d, BoundaryEvidence::ACQUISITION, HistoryContinuity::kUnknown,
        "ballistic_acquisition_incomplete_history");
  if (active()) { acquisition_spin_ = spin; acquisition_spin_cutoff_ns_ = cutoff; }
  return active();
}
bool ShotContext::context(MatchContext c, std::int64_t ns,
                          const std::string &source, std::uint64_t gen) {
  if (!active() || gen != generation_ || source.empty() || ns < boundary_ns_ ||
      (!match_source_.empty() &&
       (source != match_source_ || ns < match_time_ns_)))
    return false;
  const bool semantic_change = match_ != c;
  match_ = c;
  match_time_ns_ = ns;
  match_source_ = source;
  if (semantic_change)
    changed("authoritative_context");
  reclassify();
  return true;
}
int ShotContext::observed_contacts() const {
  int n = 0;
  for (const auto &e : events_)
    if (e.status == EventStatus::CONFIRMED &&
        e.kind == PhysicalEventKind::TABLE)
      ++n;
  return n;
}
bool ShotContext::route_supported() const {
  // Match type constrains the route; it does not certify unobserved bounce
  // history.
  return active() && continuity_ == HistoryContinuity::kComplete;
}
void ShotContext::set_history_continuity(HistoryContinuity continuity) {
  if (active() && continuity_ != continuity) {
    continuity_ = continuity;
    changed("history_qualification_changed");
    reclassify();
  }
}
void ShotContext::reclassify() {
  if (!active())
    return;
  if (settings_.game_legality_advisory) {
    bool striker_bounce = false;
    for (const auto &event : events_)
      if (event.status == EventStatus::CONFIRMED) {
        if (event.kind == PhysicalEventKind::NET_CROSSING) net_crossed_ = true;
        if (event.kind == PhysicalEventKind::TABLE &&
            ((direction_ < 0 && event.side == ContactSide::kOpponent) ||
             (direction_ > 0 && event.side == ContactSide::kReceiver))) striker_bounce = true;
      }
    state_ = direction_ < 0 ? (striker_bounce ? PlayState::OPPONENT_SERVE : PlayState::OPPONENT_RALLY)
                            : (striker_bounce ? PlayState::SERVE : PlayState::RALLY);
    route_ = state_;
    classification_ = striker_bounce ? ClassificationStatus::CONFIRMED : ClassificationStatus::PROVISIONAL;
    return;
  }
  const auto striker =
      direction_ > 0 ? ContactSide::kReceiver : ContactSide::kOpponent;
  int striker_contacts = 0, receiver_contacts = 0;
  bool net = false, serve = match_ == MatchContext::SERVE;
  const bool complete = continuity_ == HistoryContinuity::kComplete;
  for (const auto &e : events_)
    if (e.status == EventStatus::CONFIRMED) {
      if (e.kind == PhysicalEventKind::NET_COLLISION) {
        invalidate("confirmed_net_collision");
        return;
      }
      if (e.kind == PhysicalEventKind::NET_CROSSING) {
        if (serve && complete && striker_contacts != 1) {
          invalidate("serve_net_before_striker_bounce");
          return;
        }
        net = true;
      }
      if (e.kind == PhysicalEventKind::TABLE) {
        if (e.side == ContactSide::kAmbiguous) {
          changed("ambiguous_contact_side");
          continuity_ = HistoryContinuity::kUnknown;
          return;
        }
        if (e.side == striker) {
          if (match_ == MatchContext::NORMAL_RALLY || net ||
              receiver_contacts || ++striker_contacts > 1) {
            invalidate("forbidden_striker_contact");
            return;
          }
          serve = true;
        } else {
          ++receiver_contacts;
          // With missing prehistory, an absent net/B1 record is not evidence
          // that the ball never crossed/bounced there. Positive forbidden
          // ordering/count evidence still rejects the shot immediately.
          if ((complete && (!net || (serve && striker_contacts != 1))) ||
              ((serve || direction_ < 0) && receiver_contacts > 1)) {
            invalidate("forbidden_contact_order_or_count");
            return;
          }
        }
      }
    }
  net_crossed_ = net;
  // Match context constrains a hypothesis; only physical evidence confirms a
  // public serve. Recompute on every history/context change (no sticky status).
  const bool observed_serve = striker_contacts == 1;
  state_ = direction_ > 0
               ? (observed_serve ? PlayState::SERVE : PlayState::RALLY)
               : (observed_serve ? PlayState::OPPONENT_SERVE
                                 : PlayState::OPPONENT_RALLY);
  route_ = direction_ > 0 ? (serve ? PlayState::SERVE : PlayState::RALLY)
                          : (serve ? PlayState::OPPONENT_SERVE
                                   : PlayState::OPPONENT_RALLY);
  classification_ = (observed_serve || (net && complete))
                        ? ClassificationStatus::CONFIRMED
                        : ClassificationStatus::PROVISIONAL;
}
bool ShotContext::confirm(PhysicalEventKind kind, std::int64_t ns,
                          std::int64_t confirmed, const ModelState &pre,
                          const std::string &source, std::int64_t uncertainty,
                          std::optional<std::int64_t> reversal_ns,
                          std::int64_t bracket_begin_ns, std::int64_t bracket_end_ns) {
  if (!active() || (boundary_qualified_ && ns < boundary_ns_) ||
      confirmed < ns || source.empty() || !pre.position.allFinite() ||
      uncertainty < 0)
    return false;
  if (kind == PhysicalEventKind::TABLE &&
      (!pre.velocity.allFinite() || pre.velocity.z() >= 0 ||
       std::abs(pre.position.z() - config_.physics.radius) > .06 ||
       pre.position.x() < 0 || pre.position.x() > config_.table.length ||
       pre.position.y() < config_.table.y_max - config_.table.width ||
       pre.position.y() > config_.table.y_max))
    return false;
  ShotEvent *found = nullptr;
  int matches = 0;
  if (reversal_ns) {
    if (bracket_begin_ns > *reversal_ns || bracket_end_ns < *reversal_ns ||
        bracket_begin_ns > ns || bracket_end_ns < ns) return false;
    ShotEvent *corroborated = nullptr;
    for (auto &e : events_)
      if (e.kind == kind && e.status == EventStatus::CONFIRMED &&
          e.reversal_sample_ns == reversal_ns) return true;
      else if (e.kind == kind && e.status == EventStatus::CONFIRMED &&
               !e.reversal_sample_ns && e.event_time_ns >= bracket_begin_ns &&
               e.event_time_ns <= bracket_end_ns) {
        if (corroborated) return false; // Two existing contacts are unresolved.
        corroborated = &e;
      }
    if (corroborated) {
      // Corroboration of an authoritative contact (or a wire checkpoint
      // without internal bracket fields) keeps its ID and impulse. Distance
      // travelled at high speed is not a second observed physical contact.
      corroborated->reversal_sample_ns = reversal_ns;
      corroborated->bracket_begin_ns = bracket_begin_ns;
      corroborated->bracket_end_ns = bracket_end_ns;
      return true;
    }
    for (auto it = events_.rbegin(); it != events_.rend(); ++it)
      if (it->kind == kind && it->status == EventStatus::CONFIRMED && it->reversal_sample_ns) {
        // Rearm only after a verified intervening rise and descent. Time or
        // horizontal separation alone cannot turn overlapping minima into a
        // new bounce. Closely spaced real bounces still have their own apex.
        double peak = -std::numeric_limits<double>::infinity();
        for (const auto &s : samples_)
          if (s.source_time_ns > *it->reversal_sample_ns && s.source_time_ns < *reversal_ns)
            peak = std::max(peak, s.position.z());
        if (peak <= std::max(it->pre.position.z(), pre.position.z()) +
                    std::max(.001, 2*config_.measurement_std_m.z())) return false;
        break;
      }
  }
  auto compatible_event = [&](const ShotEvent &e) {
    // A different observed reversal is never the same event. Predicted
    // hypotheses associate by the physical time bracket, without a speed-
    // dependent 10 cm gate.
    if (reversal_ns) return e.status != EventStatus::CONFIRMED &&
        e.event_time_ns >= bracket_begin_ns && e.event_time_ns <= bracket_end_ns;
    return std::abs(static_cast<long double>(e.event_time_ns) - ns) <=
               settings_.contact_association_s * 1e9L &&
           (e.pre.position - pre.position).norm() <= settings_.contact_association_m;
  };
  for (auto &e : events_)
    if (e.kind == kind && e.status != EventStatus::CANCELLED &&
        e.status != EventStatus::SUPERSEDED &&
        compatible_event(e)) {
      found = &e;
      ++matches;
    }
  // Compare retired alternatives by the last source snapshot that supported
  // them. Older mutually superseded hypotheses must not make a later physical
  // confirmation ambiguous; two candidates from one snapshot still do.
  if (matches == 0) {
    std::optional<std::int64_t> latest;
    for (auto &e : events_)
      if (e.kind == kind && e.status == EventStatus::SUPERSEDED &&
          compatible_event(e)) {
        const auto reference = e.last_prediction_reference_ns.value_or(
            std::numeric_limits<std::int64_t>::min());
        if (!latest || reference > *latest) {
          latest = reference;
          found = &e;
          matches = 1;
        } else if (reference == *latest)
          ++matches;
      }
  }
  if (matches > 1) {
    for (auto &e : events_)
      if (e.kind == kind && e.status == EventStatus::PREDICTED)
        e.status = EventStatus::AMBIGUOUS;
    changed("ambiguous_event_association");
    return false;
  }
  if (found && found->status == EventStatus::CONFIRMED)
    return true;
  if (!found) {
    if (events_.size() >= settings_.event_capacity) {
      invalidate("event_ledger_capacity");
      return false;
    }
    events_.push_back({});
    found = &events_.back();
    found->id = next_event_id_++;
    found->generation = generation_;
    found->kind = kind;
  }
  found->event_time_ns = ns;
  found->confirmed_at_ns = confirmed;
  found->time_uncertainty_ns = uncertainty;
  found->reversal_sample_ns = reversal_ns;
  found->bracket_begin_ns = bracket_begin_ns;
  found->bracket_end_ns = bracket_end_ns;
  if (kind == PhysicalEventKind::TABLE && !spin_.can_apply_contact(ns)) {
    found->status = EventStatus::AMBIGUOUS;
    found->pre = found->post = pre;
    found->source = source;
    continuity_ = HistoryContinuity::kIncomplete;
    changed("late_contact_spin_order_ambiguous");
    return false;
  }
  found->status = EventStatus::CONFIRMED;
  found->source = source;
  found->pre = pre;
  // Resolve the checkpoint at the physical source cutoff, rather than throwing
  // away a too-new latest measurement and losing the usable historical one.
  found->pre.spin = spin_.at(reversal_ns ? bracket_begin_ns : ns);
  found->post = found->pre;
  found->side = pre.position.x() < config_.table.net_x ? ContactSide::kReceiver
                                                       : ContactSide::kOpponent;
  if (std::abs(pre.position.x() - config_.table.net_x) < config_.physics.radius)
    found->side = ContactSide::kAmbiguous;
  if (kind == PhysicalEventKind::TABLE) {
    apply_model_contact(found->post, config_.physics);
    found->post.spin.last_contact_id = found->id;
    spin_.contact(found->id, ns, pre.velocity, config_.physics, bracket_end_ns,
                  reversal_ns ? std::optional<std::int64_t>(bracket_begin_ns) : std::nullopt);
  }
  std::stable_sort(events_.begin(), events_.end(),
                   [](const auto &a, const auto &b) {
                     return a.event_time_ns < b.event_time_ns;
                   });
  changed("contact_or_crossing_confirmed");
  reclassify();
  return true;
}
void ShotContext::predict_events(const std::vector<PhysicalEvent> &predictions,
                                 std::int64_t reference) {
  if (!active())
    return;
  std::vector<std::uint64_t> retained;
  for (const auto &p : predictions) {
    auto d = duration_ns(p.dt_s);
    std::int64_t ns;
    if (!d || !checked_arrival(reference, *d, ns))
      continue;
    ShotEvent *found = nullptr;
    int matches = 0;
    for (auto &e : events_)
      if (e.kind == p.kind &&
          std::find(retained.begin(), retained.end(), e.id) == retained.end() &&
          e.status != EventStatus::CANCELLED &&
          e.status != EventStatus::SUPERSEDED &&
          std::abs(static_cast<long double>(e.event_time_ns) - ns) <=
              settings_.contact_association_s * 1e9L &&
          (e.pre.position - p.pre.position).norm() <=
              settings_.contact_association_m) {
        found = &e;
        ++matches;
      }
    if (matches == 0) {
      std::optional<std::int64_t> latest;
      for (auto &e : events_)
        if (e.kind == p.kind && e.status == EventStatus::SUPERSEDED &&
            std::find(retained.begin(), retained.end(), e.id) ==
                retained.end() &&
            std::abs(static_cast<long double>(e.event_time_ns) - ns) <=
                settings_.contact_association_s * 1e9L &&
            (e.pre.position - p.pre.position).norm() <=
                settings_.contact_association_m) {
          const auto previous = e.last_prediction_reference_ns.value_or(
              std::numeric_limits<std::int64_t>::min());
          if (!latest || previous > *latest) {
            latest = previous;
            found = &e;
            matches = 1;
          } else if (previous == *latest)
            ++matches;
        }
    }
    if (matches > 1)
      continue;
    if (!found) {
      if (events_.size() >= settings_.event_capacity) {
        auto obsolete =
            std::find_if(events_.begin(), events_.end(), [](const auto &e) {
              return e.status == EventStatus::SUPERSEDED ||
                     e.status == EventStatus::CANCELLED;
            });
        if (obsolete == events_.end())
          break;
        events_.erase(obsolete);
      }
      events_.push_back({});
      found = &events_.back();
      found->id = next_event_id_++;
      found->generation = generation_;
      found->kind = p.kind;
    }
    retained.push_back(found->id);
    if (found->status == EventStatus::CONFIRMED)
      continue;
    found->last_prediction_reference_ns = reference;
    found->event_time_ns = ns;
    found->pre = p.pre;
    found->post = p.post;
    found->status = EventStatus::PREDICTED;
    found->side = p.pre.position.x() < config_.table.net_x
                      ? ContactSide::kReceiver
                      : ContactSide::kOpponent;
  }
  for (auto &e : events_)
    if (e.status == EventStatus::PREDICTED &&
        std::find(retained.begin(), retained.end(), e.id) == retained.end())
      e.status = EventStatus::SUPERSEDED;
  route_ = match_ == MatchContext::SERVE
               ? (direction_ > 0 ? PlayState::SERVE : PlayState::OPPONENT_SERVE)
               : state_;
  // A speculative first striker bounce changes only the forecast hypothesis.
  if (observed_contacts() == 0 && match_ == MatchContext::UNKNOWN)
    for (const auto &p : predictions) {
      if (p.kind == PhysicalEventKind::NET_CROSSING)
        break;
      if (p.kind == PhysicalEventKind::TABLE) {
        const bool striker = direction_ > 0
                                 ? p.pre.position.x() < config_.table.net_x
                                 : p.pre.position.x() > config_.table.net_x;
        if (striker)
          route_ =
              direction_ > 0 ? PlayState::SERVE : PlayState::OPPONENT_SERVE;
        break;
      }
    }
}
ShotToken ShotContext::snapshot() {
  return {generation_, revision_, ++solve_sequence_,
          samples_.empty() ? boundary_ns_ : samples_.back().source_time_ns};
}
bool ShotContext::publishable(const ShotToken &t, std::int64_t now) {
  const long double age = (static_cast<long double>(now) - t.source_ns) * 1e-9L;
  if (!active() || t.generation != generation_ ||
      t.context_revision != revision_ ||
      t.solve_sequence <= published_sequence_ || age < -.001 ||
      age > settings_.max_source_age_s)
    return false;
  published_sequence_ = t.solve_sequence;
  return true;
}
ShotSnapshot ShotContext::export_snapshot() const {
  return {generation_,     revision_,     boundary_ns_,   boundary_qualified_,
          direction_,      state_,        match_,         continuity_,
          classification_, match_source_, match_time_ns_, samples_,
          events_,         spin_.value(), spin_};
}
void ShotContext::restore_snapshot(const ShotSnapshot &snapshot) {
  if (!snapshot.generation || snapshot.direction == 0 ||
      snapshot.state == PlayState::INVALID || snapshot.observations.empty() ||
      snapshot.observations.size() > settings_.observation_capacity ||
      snapshot.events.size() > settings_.event_capacity)
    throw std::invalid_argument("invalid shot snapshot");
  for (std::size_t i = 0; i < snapshot.observations.size(); ++i) {
    const auto &s = snapshot.observations[i];
    if (!s.source_time_valid || !s.position_valid || !s.position.allFinite() ||
        s.source_time_ns < snapshot.boundary_ns ||
        (i &&
         (s.source_time_ns <= snapshot.observations[i - 1].source_time_ns ||
          !compatible(s, snapshot.observations[i - 1]))))
      throw std::invalid_argument(
          "invalid snapshot observation identity/order");
  }
  generation_ = snapshot.generation;
  revision_ = snapshot.revision;
  boundary_ns_ = snapshot.boundary_ns;
  boundary_qualified_ = snapshot.boundary_qualified;
  direction_ = snapshot.direction;
  state_ = route_ = snapshot.state;
  match_ = snapshot.match;
  match_source_ = snapshot.context_source;
  match_time_ns_ = snapshot.context_time_ns;
  continuity_ = snapshot.continuity;
  classification_ = snapshot.classification;
  samples_ = snapshot.observations;
  events_ = snapshot.events;
  if (snapshot.spin_checkpoint) spin_ = *snapshot.spin_checkpoint;
  else spin_.restore(snapshot.spin);
  candidates_.clear();
  net_crossed_ = false;
  next_event_id_ = 1;
  for (auto &e : events_) {
    if (e.generation != generation_ || !e.id ||
        e.time_uncertainty_ns < 0 ||
        (boundary_qualified_ && e.status == EventStatus::CONFIRMED &&
         e.event_time_ns < boundary_ns_) ||
        (e.status == EventStatus::CONFIRMED &&
         e.confirmed_at_ns < e.event_time_ns))
      throw std::invalid_argument("invalid snapshot ledger");
    if (e.kind == PhysicalEventKind::TABLE && e.status == EventStatus::CONFIRMED &&
        e.source == "source_reversal_bracket" && !e.reversal_sample_ns) {
      // The unchanged wire already carries the exact confirmation sample and
      // bounded raw history. Reconstruct the original five-point window's
      // reversal key; packet resume must not mint a second physical contact.
      auto end = std::find_if(samples_.begin(),samples_.end(),[&](const auto &s) {
        return s.source_time_ns == e.confirmed_at_ns;
      });
      if (end != samples_.end() && end-samples_.begin() >= 4) {
        const auto first=end-4;
        const auto low=std::min_element(first,end+1,[](const auto &a,const auto &b) {
          return a.position.z()<b.position.z();
        });
        if (low!=first && low!=end && e.event_time_ns >= (low-1)->source_time_ns &&
            e.event_time_ns <= (low+1)->source_time_ns) {
          e.reversal_sample_ns=low->source_time_ns;
          e.bracket_begin_ns=(low-1)->source_time_ns;
          e.bracket_end_ns=(low+1)->source_time_ns;
        }
      }
    }
    if (e.kind == PhysicalEventKind::TABLE &&
        e.status == EventStatus::CONFIRMED)
      spin_.remember_contact(e.id, e.event_time_ns);
    next_event_id_ = std::max(next_event_id_, e.id + 1);
    if (e.kind == PhysicalEventKind::NET_CROSSING &&
        e.status == EventStatus::CONFIRMED)
      net_crossed_ = true;
  }
  if (!snapshot.spin_checkpoint) {
    // Wire checkpoints contain raw observations and the event ledger, but no
    // in-memory quaternion estimator. Recover a causal suffix when the raw
    // evidence supports the exact checkpoint; otherwise retain the explicit
    // latest checkpoint without inventing unavailable older spin.
    TrustedSpin recovered;
    std::vector<std::uint64_t> applied;
    for (const auto &e : events_)
      if (e.kind == PhysicalEventKind::TABLE && e.status == EventStatus::CONFIRMED &&
          e.event_time_ns < samples_.front().source_time_ns) {
        recovered.restore(e.post.spin);
        recovered.remember_contact(e.id,e.event_time_ns);
        applied.push_back(e.id);
      }
    for (const auto &sample : samples_) {
      recovered.observe(sample);
      for (const auto &e : events_)
        if (e.kind == PhysicalEventKind::TABLE && e.status == EventStatus::CONFIRMED &&
            e.confirmed_at_ns <= sample.source_time_ns &&
            std::find(applied.begin(),applied.end(),e.id)==applied.end()) {
          std::int64_t end=e.event_time_ns;
          if (e.bracket_end_ns) end=e.bracket_end_ns;
          else if (!checked_arrival(e.event_time_ns,e.time_uncertainty_ns,end))
            end=std::numeric_limits<std::int64_t>::max();
          recovered.contact(e.id,e.event_time_ns,e.pre.velocity,config_.physics,end,
              e.reversal_sample_ns ? std::optional<std::int64_t>(e.bracket_begin_ns) : std::nullopt);
          applied.push_back(e.id);
        }
    }
    const auto &r = recovered.value(), &s = snapshot.spin;
    if (r.usable()==s.usable() && (!s.usable() ||
        ((r.omega.value()-s.omega.value()).norm()<1e-8 &&
         r.measurement_time_ns==s.measurement_time_ns && r.last_contact_id==s.last_contact_id)))
      spin_ = std::move(recovered);
  }
  solve_sequence_ = published_sequence_ = 0;
  rejected_latch_ = false;
  candidate_spin_.reset(); candidate_last_.reset(); quiet_anchor_.reset();
  rejection_fence_.reset(); rejection_reason_.clear(); rearm_candidate_ = false;
  acquisition_spin_ = {}; acquisition_spin_cutoff_ns_ = 0;
  reason_ = "restored_shot_checkpoint";
}
void FlightPipeline::restore(const ShotSnapshot &snapshot,
                             const FullTrackerCheckpoint &checkpoint,
                             std::int64_t origin) {
  if (snapshot.observations.empty() ||
      checkpoint.belief.source_time_ns !=
          snapshot.observations.back().source_time_ns)
    throw std::invalid_argument("snapshot/filter reference mismatch");
  // Validate in temporary owners before committing either component.
  ShotContext shot(config_, settings_);
  shot.restore_snapshot(snapshot);
  ProbabilisticBallTracker tracker(config_);
  auto resolved = checkpoint;
  auto &spin = resolved.belief.effective_spin;
  if (spin.usable() && shot.spin().usable() &&
      spin.measurement_time_ns==shot.spin().measurement_time_ns &&
      spin.last_contact_id==shot.spin().last_contact_id &&
      (*spin.omega-*shot.spin().omega).norm()<1e-8 && !shot.spin().provenance.empty())
    spin.provenance = shot.spin().provenance;
  tracker.restore_checkpoint(resolved);
  tracker.set_effective_spin(spin);
  shot_ = std::move(shot);
  tracker_ = std::move(tracker);
  origin_ = origin;
  tracker_generation_ = shot_.generation();
  tracker_history_.clear();
}
RouteTarget select_route_target(const ShotContext &shot,
                                const std::vector<PhysicalEvent> &events,
                                const ProbabilisticConfig &config,
                                bool plane_available) {
  RouteTarget out;
  out.status = ForecastStatus::HORIZON_EXHAUSTED;
  out.reason = "route_horizon_exhausted";
  if (!shot.active()) {
    out.status = ForecastStatus::REVOKED;
    out.reason = shot.reason();
    return out;
  }
  const bool incoming = shot.direction() < 0;
  const bool serve = shot.route() == PlayState::SERVE ||
                     shot.route() == PlayState::OPPONENT_SERVE;
  const auto striker =
      incoming ? ContactSide::kOpponent : ContactSide::kReceiver;
  int count = 0;
  bool net = false;
  std::optional<PhysicalEvent> apex;
  for (const auto &e : shot.events())
    if (e.status == EventStatus::CONFIRMED) {
      if (e.kind == PhysicalEventKind::TABLE)
        ++count;
      if (e.kind == PhysicalEventKind::NET_CROSSING)
        net = true;
      if (e.kind == PhysicalEventKind::APEX && count == 1 && !incoming &&
          !serve) {
        PhysicalEvent p;
        p.kind = e.kind;
        p.pre = e.pre;
        p.post = e.post;
        p.dt_s = -1;
        apex = p;
      }
    }
  auto forbidden = [&](const char *reason) {
    out.status = ForecastStatus::UNCERTAIN;
    out.known_forbidden = true;
    out.reason = reason;
    // Default uncertainty is conditional: no permanent rejection from a nominal
    // branch.
    return out;
  };
  auto target = [&](const PhysicalEvent &e) {
    out.event = e;
    out.incoming = incoming;
    out.historical = e.dt_s < 0;
    out.valid = shot.route_supported() && !out.historical &&
                (!incoming || plane_available);
    out.status = out.valid ? ForecastStatus::READY : ForecastStatus::UNCERTAIN;
    out.reason = out.historical                 ? "historical_apex"
                 : !shot.route_supported()      ? "incomplete_history"
                 : incoming && !plane_available ? "receiving_plane_unavailable"
                                                : "route_target_ready";
    return out;
  };
  for (const auto &e : events) {
    if (e.kind == PhysicalEventKind::NET_COLLISION)
      return forbidden("predicted_net_collision_conditional");
    if (e.kind == PhysicalEventKind::EDGE_AMBIGUOUS) {
      out.status = ForecastStatus::UNCERTAIN;
      out.reason = "edge_ambiguous";
      return out;
    }
    if (e.kind == PhysicalEventKind::GROUND)
      return forbidden("predicted_ground_before_target_conditional");
    if (e.kind == PhysicalEventKind::NET_CROSSING) {
      if (serve && count != 1)
        return forbidden("serve_net_before_b1_conditional");
      net = true;
    }
    if (e.kind == PhysicalEventKind::TABLE) {
      const auto side = e.pre.position.x() < config.table.net_x
                            ? ContactSide::kReceiver
                            : ContactSide::kOpponent;
      ++count;
      if (std::abs(e.pre.position.x() - config.table.net_x) <
          config.physics.radius) {
        out.status = ForecastStatus::UNCERTAIN;
        out.reason = "contact_side_ambiguous";
        return out;
      }
      if (serve) {
        if (count == 1 && (net || side != striker))
          return forbidden("serve_b1_order_conditional");
        if (count == 2 && (!net || side == striker))
          return forbidden("serve_b2_order_conditional");
        if (count > 2)
          return forbidden("serve_b3_before_target_conditional");
      } else {
        if (side == striker || !net)
          return forbidden("rally_wrong_side_conditional");
        if (count >= 2) {
          if (!incoming && apex)
            return target(*apex);
          return forbidden("rally_b2_before_target_conditional");
        }
      }
    }
    if (e.kind == PhysicalEventKind::APEX && count == 1 && !incoming && !serve)
      apex = e;
    if ((incoming && e.kind == PhysicalEventKind::PLANE) ||
        (!incoming && e.kind == PhysicalEventKind::END)) {
      if (count == (serve ? 2 : 1))
        return target(e);
      return forbidden("target_before_required_contact_conditional");
    }
  }
  return out;
}
FlightPipeline::FlightPipeline(ProbabilisticConfig c, ShotSettings s)
    : config_(std::move(c)), settings_(s), shot_(config_, s),
      tracker_(config_) {}
void FlightPipeline::start(std::int64_t ns, int direction,
                           BoundaryEvidence evidence,
                           HistoryContinuity continuity) {
  shot_.start(ns, direction, evidence, continuity);
  tracker_.set_effective_spin({});
  tracker_.reset();
  origin_.reset();
  tracker_history_.clear();
  tracker_generation_ = shot_.generation();
}
void FlightPipeline::invalidate(const std::string &why) {
  shot_.invalidate(why);
  tracker_.set_effective_spin({});
  tracker_.reset();
  origin_.reset();
  tracker_history_.clear();
  tracker_generation_ = shot_.generation();
}
bool FlightPipeline::observe(BallSample sample) {
  if (!origin_)
    origin_ = sample.source_time_ns;
  sample.source_time_s = double(
      (static_cast<long double>(sample.source_time_ns) - *origin_) * 1e-9L);
  const auto before = shot_.generation();
  const bool admitted = shot_.observe(sample);
  if (!admitted && !shot_.active())
    shot_.qualify_candidate();
  if (shot_.generation() != before ||
      tracker_generation_ != shot_.generation()) {
    tracker_.set_effective_spin({});
    tracker_.reset();
    tracker_history_.clear();
    tracker_generation_ = shot_.generation();
    if (!shot_.active()) {
      return false;
    }
    if (settings_.automatic_rearm && !shot_.samples().empty()) {
      origin_ = shot_.samples().front().source_time_ns;
      shot_.rebase_times(*origin_);
    }
    for (const auto &s : shot_.samples()) {
      tracker_history_.push_back({s.source_time_ns, tracker_.export_checkpoint()});
      tracker_.observe(s);
      // Historical lookup includes candidate angular evidence older than the
      // short dynamic suffix, and never pulls a future measurement backward.
      tracker_.set_effective_spin(shot_.spin_at(s.source_time_ns));
    }
    return !shot_.samples().empty();
  }
  if (!admitted)
    return false;
  tracker_history_.push_back({sample.source_time_ns, tracker_.export_checkpoint()});
  while (!tracker_history_.empty() && !shot_.samples().empty() &&
         tracker_history_.front().next_observation_ns < shot_.samples().front().source_time_ns)
    tracker_history_.pop_front();
  const auto update = tracker_.observe(sample);
  if (update.new_track && update.reason != "initial_observation")
    shot_.set_history_continuity(HistoryContinuity::kIncomplete);
  tracker_.set_effective_spin(shot_.spin());
  // Confirm a bracketed descending/rising top contact using source-time slopes.
  const auto &samples = shot_.samples();
  if (samples.size() >= 5) {
    const std::size_t n = samples.size();
    const auto &a = samples[n - 5];
    // All overlapping windows resolve the same adjacent source reversal, not
    // whichever sample happens to be the middle of today's five-point window.
    const auto minimum = std::min_element(samples.begin() + (n - 5), samples.end(),
        [](const auto &x, const auto &y) { return x.position.z() < y.position.z(); });
    if (minimum == samples.begin() + (n - 5) || minimum == samples.end() - 1)
      return update.accepted;
    const auto &low = *minimum;
    const auto &b = samples[n - 1];
    for (std::size_t i=n-4; i<n; ++i)
      if ((static_cast<long double>(samples[i].source_time_ns)-samples[i-1].source_time_ns)*1e-9L >
          config_.gap_warning_s) return update.accepted;
    const double ta = double(
        (static_cast<long double>(low.source_time_ns) - a.source_time_ns) *
        1e-9L);
    const double tb = double(
        (static_cast<long double>(b.source_time_ns) - low.source_time_ns) *
        1e-9L);
    const double excursion = std::max(.001, 2 * config_.measurement_std_m.z());
    if (ta > 0 && tb > 0 && ta + tb < .08 && low.position.z() >= 0 &&
        low.position.z() < config_.physics.radius + .03 &&
        a.position.z() - low.position.z() > excursion &&
        b.position.z() - low.position.z() > excursion &&
        low.position.x() >= 0 && low.position.x() <= config_.table.length &&
        low.position.y() >= config_.table.y_max - config_.table.width &&
        low.position.y() <= config_.table.y_max) {
      // A single low outlier produces both endpoint slopes even when the CKF
      // rejects it. Require independent post-contact motion, excluding that
      // sample, to place an ascending surface crossing inside the source
      // bracket. Do not simply require CKF acceptance: real impacts can exceed
      // the innovation gate when the contact model is imperfect.
      if (minimum + 2 >= samples.end()) return update.accepted;
      const auto &right = *(minimum + 1), &later = *(minimum + 2);
      const double rise_dt = static_cast<double>(
          (static_cast<long double>(later.source_time_ns) - right.source_time_ns) * 1e-9L);
      const Vec3 rise = (later.position - right.position) / rise_dt;
      const auto corroborating_spin = shot_.spin_at(right.source_time_ns);
      const double az = ball_acceleration(config_.physics, rise,
          corroborating_spin.usable() ? *corroborating_spin.omega : Vec3::Zero()).z();
      const double rise_vz = rise.z() - .5 * az * rise_dt;
      const double bracket_dt = static_cast<double>(
          (static_cast<long double>((minimum - 1)->source_time_ns) - right.source_time_ns) * 1e-9L);
      const double bracket_height = right.position.z() + rise_vz * bracket_dt +
          .5 * az * bracket_dt * bracket_dt;
      const double height_tolerance = std::max(.001, 3 * config_.measurement_std_m.z());
      if (!std::isfinite(rise_vz) || rise_vz <= 0 ||
          !std::isfinite(bracket_height) ||
          bracket_height > config_.physics.radius + height_tolerance)
        return update.accepted;
      if (minimum >= samples.begin() + 2) {
        const auto &left = *(minimum - 1), &older = *(minimum - 2);
        const double dt = static_cast<double>(
            (static_cast<long double>(left.source_time_ns) - older.source_time_ns) * 1e-9L);
        const Vec3 descending = (left.position - older.position) / dt;
        const double vz = descending.z() + .5 * dt * ball_acceleration(
            config_.physics, descending, Vec3::Zero()).z();
        if (!std::isfinite(vz) || vz >= 0) return update.accepted;
      }
      ModelState pre;
      pre.position = low.position;
      pre.velocity = (low.position - a.position) / ta;
      auto occurrence = low.source_time_ns;
      pre.spin = shot_.spin_at((minimum - 1)->source_time_ns);
      if (minimum >= samples.begin() + 2) {
        // Use two strictly descending-side samples, then find the surface
        // root. A sampled minimum after impact is never a pre-impact velocity
        // or angular measurement. The full adjacent bracket remains uncertain.
        const auto &left = *(minimum - 1), &older = *(minimum - 2);
        const double dt = static_cast<double>((static_cast<long double>(left.source_time_ns)-older.source_time_ns)*1e-9L);
        const double span = static_cast<double>((static_cast<long double>((minimum+1)->source_time_ns)-left.source_time_ns)*1e-9L);
        ModelState prior;
        prior.position = left.position; prior.spin = pre.spin;
        prior.velocity = (left.position-older.position)/dt;
        prior.velocity += .5*dt*ball_acceleration(config_.physics,prior.velocity,
            prior.spin.usable()?*prior.spin.omega:Vec3::Zero());
        auto end = prior; integrate_ball(end,span,config_.physics);
        if (prior.velocity.z()<0 && prior.position.z()>=config_.physics.radius &&
            end.position.z()<=config_.physics.radius) {
          double lo=0,hi=span;
          for(int k=0;k<32;++k) {
            auto trial=prior; const double mid=(lo+hi)*.5;
            integrate_ball(trial,mid,config_.physics);
            if(trial.position.z()>config_.physics.radius)lo=mid;else hi=mid;
          }
          pre=prior;integrate_ball(pre,(lo+hi)*.5,config_.physics);
          pre.position.z()=config_.physics.radius;
          occurrence=left.source_time_ns+std::llround((lo+hi)*.5*1e9);
        }
      }
      const auto count_before = shot_.observed_contacts();
      shot_.confirm(
          PhysicalEventKind::TABLE, occurrence, b.source_time_ns, pre,
          "source_reversal_bracket",
          static_cast<std::int64_t>(std::round(std::max(ta, tb) * 1e9)),
          low.source_time_ns, (minimum - 1)->source_time_ns,
          (minimum + 1)->source_time_ns);
      if (!shot_.active()) {
        tracker_.reset();
        return false;
      }
      if (shot_.observed_contacts() > count_before) {
        // Delayed observation of a contact can invalidate angular increments
        // used by the last few CKF propagations. Reapply only that bounded
        // source suffix from an actual saved checkpoint, never restart a shot
        // or inject the post-contact velocity as a position measurement.
        auto checkpoint = tracker_history_.end();
        for (auto it = tracker_history_.begin(); it != tracker_history_.end(); ++it)
          if (it->next_observation_ns <= (minimum - 1)->source_time_ns) checkpoint = it;
        if (checkpoint != tracker_history_.end()) {
          const auto resume = checkpoint->next_observation_ns;
          tracker_.restore_checkpoint(checkpoint->before);
          tracker_.set_effective_spin(checkpoint->before.belief.effective_spin);
          tracker_history_.erase(checkpoint, tracker_history_.end());
          for (const auto &s : samples) if (s.source_time_ns >= resume) {
            tracker_history_.push_back({s.source_time_ns, tracker_.export_checkpoint()});
            tracker_.observe(s);
            tracker_.set_effective_spin(shot_.spin_at(s.source_time_ns));
          }
        }
      }
      tracker_.set_effective_spin(shot_.spin());
    }
  }
  return update.accepted;
}
FlightPrediction FlightPipeline::forecast(const HitPlaneSnapshot &plane,
                                          std::int64_t now) {
  FlightPrediction out;
  out.model_config_id = flight_model_config_id(
      config_, settings_,
      shot_.samples().empty() ? "table" : shot_.samples().back().frame_id);
  out.token = shot_.snapshot();
  out.state = shot_.state();
  out.route = shot_.route();
  out.reference_ns = out.token.source_ns;
  out.plane = plane;
  out.reason = shot_.reason();
  const auto expiry = duration_ns(settings_.max_source_age_s);
  if (expiry)
    checked_arrival(out.reference_ns, *expiry, out.expires_ns);
  if (!shot_.active())
    return out;
  const long double age =
      (static_cast<long double>(now) - out.reference_ns) * 1e-9L;
  if (age < -.001 || age > settings_.max_source_age_s) {
    out.reason = "source_age_expired";
    return out;
  }
  out.belief = tracker_.belief();
  out.belief.source_time_ns = out.reference_ns;
  out.belief.source_time_origin_ns = origin_.value_or(0);
  if (!out.belief.initialized || out.belief.observation_count < 3) {
    out.status = ForecastStatus::WARMING_UP;
    return out;
  }
  auto c = config_;
  c.effective_spin = out.belief.effective_spin;
  if (plane.calibrated)
    c.x_hit = plane.x_hit_m;
  out.nominal = forecast_nominal(out.belief, c);
  if (!plane.calibrated) {
    out.nominal.hit_plane.reset();
    out.nominal.events.erase(
        std::remove_if(
            out.nominal.events.begin(), out.nominal.events.end(),
            [](const auto &event) { return event.kind == "receiving_plane"; }),
        out.nominal.events.end());
  }
  out.physical_available = out.nominal.available;
  for (const auto &event : out.nominal.events) {
    PhysicalEvent e;
    e.dt_s = event.dt_s;
    e.pre.position = event.pre.head<3>();
    e.pre.velocity = event.pre.tail<3>();
    e.pre.spin = event.pre_spin;
    e.post.position = event.post.head<3>();
    e.post.velocity = event.post.tail<3>();
    e.post.spin = event.post_spin;
    e.kind = event.kind == "table_bounce"    ? PhysicalEventKind::TABLE
             : event.kind == "net_crossing"  ? PhysicalEventKind::NET_CROSSING
             : event.kind == "net_collision" ? PhysicalEventKind::NET_COLLISION
             : event.kind == "receiving_plane" ? PhysicalEventKind::PLANE
             : event.kind == "opponent_end"    ? PhysicalEventKind::END
             : event.kind == "apex"            ? PhysicalEventKind::APEX
             : event.kind == "edge_ambiguous"
                 ? PhysicalEventKind::EDGE_AMBIGUOUS
                 : PhysicalEventKind::GROUND;
    out.events.push_back(e);
  }
  shot_.predict_events(out.events, out.reference_ns);
  out.route = shot_.route();
  out.target = select_route_target(shot_, out.events, c, plane.calibrated);
  if (out.target.valid && out.target.event) {
    const ForecastPhysicsEvent *target = nullptr;
    for (const auto &event : out.nominal.events)
      if (event.kind == name(out.target.event->kind) &&
          std::abs(event.dt_s - out.target.event->dt_s) < 1e-6) {
        target = &event;
        break;
      }
    bool ambiguous = target && target->quality != LinearizationQuality::Valid;
    if (target)
      for (const auto &event : out.nominal.events) {
        if (event.kind == "table_bounce" && event.dt_s < target->dt_s &&
            std::isfinite(event.time_std_s) &&
            std::isfinite(target->time_std_s) &&
            event.dt_s + settings_.supported_sigma * event.time_std_s >=
                target->dt_s - settings_.supported_sigma * target->time_std_s)
          ambiguous = true;
        if (event.kind == "net_crossing" && event.dt_s < target->dt_s &&
            event.covariance.allFinite()) {
          const double lower =
              event.pre[2] -
              settings_.supported_sigma *
                  std::sqrt(std::max(0., event.covariance(2, 2)));
          const double y = event.pre[1];
          if (y >= c.table.y_max - c.table.width - c.table.net_overhang -
                       c.physics.radius &&
              y <= c.table.y_max + c.table.net_overhang + c.physics.radius &&
              lower <= c.table.net_height + c.physics.radius)
            ambiguous = true;
        }
      }
    if (ambiguous) {
      out.target.valid = false;
      out.target.status = ForecastStatus::UNCERTAIN;
      out.target.reason = "event_order_or_clearance_uncertain";
    }
  }
  out.status = out.target.status;
  out.reason = out.target.reason;
  if (out.target.event && out.target.event->dt_s >= 0) {
    auto duration = duration_ns(out.target.event->dt_s);
    if (!duration ||
        !checked_arrival(out.reference_ns, *duration, out.arrival_ns)) {
      out.target.valid = false;
      out.status = ForecastStatus::REVOKED;
      out.reason = "arrival_overflow";
    } else
      out.arrival_dt_ns = *duration;
  }
  return out;
}
} // namespace hope_planner_cpp
