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
  return "flight-model-v3:sha256:" + sha256(out.str());
}
void set_flight_parameter(ProbabilisticConfig &c, ShotSettings &s,
                          const std::string &key, double value) {
  if (!std::isfinite(value))
    throw std::invalid_argument("nonfinite flight parameter");
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
  if (rejected_latch_ && evidence == BoundaryEvidence::ACQUISITION)
    return;
  ++generation_;
  revision_ = 1;
  published_sequence_ = 0;
  boundary_ns_ = ns;
  boundary_qualified_ = evidence != BoundaryEvidence::ACQUISITION;
  direction_ = d;
  net_crossed_ = false;
  rejected_latch_ = false;
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
}
void ShotContext::invalidate(const std::string &why) {
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
  }
  if (!active()) {
    if (!candidates_.empty() &&
        (s.source_time_ns <= candidates_.back().source_time_ns ||
         !compatible(candidates_.back(), s)))
      return false;
    candidates_.push_back(s);
    while (candidates_.size() > settings_.candidate_capacity)
      candidates_.pop_front();
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
      const bool finite_net =
          net.position.y() >= config_.table.y_max - config_.table.width -
                                  config_.table.net_overhang -
                                  config_.physics.radius &&
          net.position.y() <= config_.table.y_max + config_.table.net_overhang +
                                  config_.physics.radius &&
          net.position.z() <=
              config_.table.net_height + config_.physics.radius &&
          net.position.z() >= -config_.physics.radius;
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
      if (!contact_window && (acceleration - expected).norm() >
                                 settings_.ballistic_residual_mps2) {
        invalidate("qualified_non_ballistic_motion");
        return false;
      }
    }
  }
  return true;
}
bool ShotContext::qualify_candidate() {
  if (active() || rejected_latch_ || settings_.service_armed ||
      candidates_.size() < 6)
    return false;
  const auto &a = candidates_.front();
  const auto &b = candidates_[candidates_.size() / 2];
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
      v.norm() < settings_.launch_speed_mps ||
      (accel - config_.physics.gravity).norm() >
          settings_.ballistic_residual_mps2)
    return false;
  // Acquisition does not establish racket contact or complete prehistory.
  const auto boundary = a.source_time_ns;
  const int d = v.x() > 0 ? 1 : -1;
  start(boundary, d, BoundaryEvidence::ACQUISITION, HistoryContinuity::kUnknown,
        "ballistic_acquisition_incomplete_history");
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
                          const std::string &source, std::int64_t uncertainty) {
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
  for (auto &e : events_)
    if (e.kind == kind && e.status != EventStatus::CANCELLED &&
        e.status != EventStatus::SUPERSEDED &&
        std::abs(static_cast<long double>(e.event_time_ns) - ns) <=
            settings_.contact_association_s * 1e9L &&
        (e.pre.position - pre.position).norm() <=
            settings_.contact_association_m) {
      found = &e;
      ++matches;
    }
  // Source evidence can confirm a contact several exposures after its
  // predicted occurrence. The next forecast may already have superseded that
  // prediction; retirement is not cancellation of its physical identity.
  if (matches == 0)
    for (auto &e : events_)
      if (e.kind == kind && e.status == EventStatus::SUPERSEDED &&
          std::abs(static_cast<long double>(e.event_time_ns) - ns) <=
              settings_.contact_association_s * 1e9L &&
          (e.pre.position - pre.position).norm() <=
              settings_.contact_association_m) {
        found = &e;
        ++matches;
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
  if (found->pre.spin.usable() && found->pre.spin.measurement_time_ns > ns)
    found->pre.spin = {};
  found->post = pre;
  if (!found->pre.spin.usable() && spin_.value().usable() &&
      spin_.value().measurement_time_ns <= ns)
    found->pre.spin = spin_.value();
  found->post = found->pre;
  found->side = pre.position.x() < config_.table.net_x ? ContactSide::kReceiver
                                                       : ContactSide::kOpponent;
  if (std::abs(pre.position.x() - config_.table.net_x) < config_.physics.radius)
    found->side = ContactSide::kAmbiguous;
  if (kind == PhysicalEventKind::TABLE) {
    apply_model_contact(found->post, config_.physics);
    spin_.contact(found->id, ns, pre.velocity, config_.physics);
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
      if (e.kind == p.kind && e.status != EventStatus::CANCELLED &&
          e.status != EventStatus::SUPERSEDED &&
          std::abs(static_cast<long double>(e.event_time_ns) - ns) <=
              settings_.contact_association_s * 1e9L &&
          (e.pre.position - p.pre.position).norm() <=
              settings_.contact_association_m) {
        found = &e;
        ++matches;
      }
    // A transient unavailable horizon can retire a future hypothesis before
    // the next solve recovers it. Reuse its identity when source-time and
    // geometry still support the same event, rather than creating duplicates.
    if (matches == 0)
      for (auto &e : events_)
        if (e.kind == p.kind && e.status == EventStatus::SUPERSEDED &&
            std::abs(static_cast<long double>(e.event_time_ns) - ns) <=
                settings_.contact_association_s * 1e9L &&
            (e.pre.position - p.pre.position).norm() <=
                settings_.contact_association_m) {
          found = &e;
          ++matches;
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
          events_,         spin_.value()};
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
  spin_.restore(snapshot.spin);
  candidates_.clear();
  net_crossed_ = false;
  next_event_id_ = 1;
  for (const auto &e : events_) {
    if (e.generation != generation_ || !e.id ||
        (boundary_qualified_ && e.status == EventStatus::CONFIRMED &&
         e.event_time_ns < boundary_ns_) ||
        (e.status == EventStatus::CONFIRMED &&
         e.confirmed_at_ns < e.event_time_ns))
      throw std::invalid_argument("invalid snapshot ledger");
    if (e.kind == PhysicalEventKind::TABLE &&
        e.status == EventStatus::CONFIRMED)
      spin_.remember_contact(e.id, e.event_time_ns);
    next_event_id_ = std::max(next_event_id_, e.id + 1);
    if (e.kind == PhysicalEventKind::NET_CROSSING &&
        e.status == EventStatus::CONFIRMED)
      net_crossed_ = true;
  }
  solve_sequence_ = published_sequence_ = 0;
  rejected_latch_ = false;
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
  tracker.restore_checkpoint(checkpoint);
  shot_ = std::move(shot);
  tracker_ = std::move(tracker);
  origin_ = origin;
  tracker_generation_ = shot_.generation();
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
  tracker_generation_ = shot_.generation();
}
void FlightPipeline::invalidate(const std::string &why) {
  shot_.invalidate(why);
  tracker_.set_effective_spin({});
  tracker_.reset();
  origin_.reset();
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
    tracker_generation_ = shot_.generation();
    if (!shot_.active()) {
      origin_.reset();
      return false;
    }
    TrustedSpin replay_spin;
    for (const auto &s : shot_.samples()) {
      tracker_.observe(s);
      replay_spin.observe(s);
      for (const auto &event : shot_.events())
        if (event.kind == PhysicalEventKind::TABLE &&
            event.status == EventStatus::CONFIRMED &&
            event.event_time_ns <= s.source_time_ns)
          replay_spin.contact(event.id, event.event_time_ns, event.pre.velocity,
                              config_.physics);
      // Never assign the acquisition window's final spin to earlier intervals.
      tracker_.set_effective_spin(replay_spin.value());
    }
    return !shot_.samples().empty();
  }
  if (!admitted)
    return false;
  const auto update = tracker_.observe(sample);
  if (update.new_track && update.reason != "initial_observation")
    shot_.set_history_continuity(HistoryContinuity::kIncomplete);
  tracker_.set_effective_spin(shot_.spin());
  // Confirm a bracketed descending/rising top contact using source-time slopes.
  const auto &samples = shot_.samples();
  if (samples.size() >= 5) {
    const std::size_t n = samples.size();
    const auto &a = samples[n - 5];
    const auto &low = samples[n - 3];
    const auto &b = samples[n - 1];
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
      ModelState pre;
      pre.position = low.position;
      pre.velocity = (low.position - a.position) / ta;
      pre.spin = shot_.spin();
      shot_.confirm(
          PhysicalEventKind::TABLE, low.source_time_ns, b.source_time_ns, pre,
          "source_reversal_bracket",
          static_cast<std::int64_t>(std::round(std::max(ta, tb) * 1e9)));
      if (!shot_.active()) {
        tracker_.reset();
        return false;
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
