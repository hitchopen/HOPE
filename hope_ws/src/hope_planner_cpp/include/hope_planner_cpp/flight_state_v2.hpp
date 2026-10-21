#pragma once
#include "hope_planner_cpp/hit_plane_manager.hpp"
#include "hope_planner_cpp/nominal_event_forecast.hpp"
#include "hope_planner_cpp/probabilistic_ball_tracker.hpp"
#include "hope_planner_cpp/return_region.hpp"
#include "hope_planner_cpp/trusted_spin.hpp"
#include <condition_variable>
#include <deque>
#include <mutex>

namespace hope_planner_cpp {
// V2 public states. Legacy PlannerBallState is a compatibility-only interface.
enum class PlayState { INVALID, OPPONENT_RALLY, RALLY, OPPONENT_SERVE, SERVE };
const char *name(PlayState);
enum class ClassificationStatus { PROVISIONAL, CONFIRMED };
enum class MatchContext { UNKNOWN, NORMAL_RALLY, SERVE };
enum class ForecastStatus {
  READY,
  WARMING_UP,
  HORIZON_EXHAUSTED,
  UNCERTAIN,
  REVOKED
};
enum class EventStatus {
  PREDICTED,
  CONFIRMED,
  SUPERSEDED,
  CANCELLED,
  AMBIGUOUS
};
enum class BoundaryEvidence {
  ACQUISITION,
  RACKET_CONTACT,
  FREE_FLIGHT_LAUNCH,
  RESTART,
  VERIFIED_CORRECTION
};
struct ShotToken {
  std::uint64_t generation = 0, context_revision = 0, solve_sequence = 0;
  std::int64_t source_ns = 0;
};
struct ShotEvent {
  std::uint64_t id = 0, generation = 0;
  PhysicalEventKind kind = PhysicalEventKind::TABLE;
  EventStatus status = EventStatus::PREDICTED;
  std::int64_t event_time_ns = 0, confirmed_at_ns = 0;
  std::int64_t time_uncertainty_ns = 0;
  ContactSide side = ContactSide::kAmbiguous;
  ModelState pre, post;
  std::string source;
  std::optional<std::int64_t> last_prediction_reference_ns;
};
struct ShotSettings {
  std::size_t observation_capacity = 192, candidate_capacity = 32,
              event_capacity = 128;
  double observation_window_s = .30, qualification_span_s = .025;
  double direction_speed_mps = .5, launch_speed_mps = 1.,
         ballistic_residual_mps2 = 8.;
  double contact_association_s = .030, contact_association_m = .10;
  double contact_exclusion_s = .015, held_span_s = .12,
         held_displacement_m = .005;
  double max_source_age_s = .030, supported_sigma = 3.,
         supported_time_margin_s = .001;
  bool service_armed = false;
  // Application opt-in: retain game-illegal evidence without retiring the
  // physical track. Standalone forecasting retains its established default.
  bool game_legality_advisory = false;
  void validate() const;
};
inline constexpr const char *kFlightPhysicsProfile =
    "thin-shell-forward-v3-receipt-20261010";
std::string flight_model_config_id(const ProbabilisticConfig &,
                                   const ShotSettings &,
                                   const std::string &frame = "table");
void set_flight_parameter(ProbabilisticConfig &, ShotSettings &,
                          const std::string &key, double value);
bool checked_arrival(std::int64_t reference, std::int64_t duration,
                     std::int64_t &arrival);
std::optional<std::int64_t> duration_ns(double seconds);
struct PredictedViolationBound {
  // True only when the bound covers auxiliary spin/model, geometry and clock
  // omissions too.
  bool all_relevant_uncertainty_bounded = false, finite_net_geometry = false;
  double forbidden_latest_s = 0, target_earliest_s = 0,
         net_clearance_upper_m = 0;
};
struct ShotSnapshot {
  std::uint64_t generation = 0, revision = 0;
  std::int64_t boundary_ns = 0;
  bool boundary_qualified = false;
  int direction = 0;
  PlayState state = PlayState::INVALID;
  MatchContext match = MatchContext::UNKNOWN;
  HistoryContinuity continuity = HistoryContinuity::kUnknown;
  ClassificationStatus classification = ClassificationStatus::PROVISIONAL;
  std::string context_source;
  std::int64_t context_time_ns = 0;
  std::deque<BallSample> observations;
  std::vector<ShotEvent> events;
  EffectiveSpin spin;
};
class ShotContext {
public:
  ShotContext(ProbabilisticConfig, ShotSettings = {});
  void start(std::int64_t boundary_ns, int direction, BoundaryEvidence,
             HistoryContinuity,
             const std::string &provenance = "qualified_boundary");
  void invalidate(const std::string &);
  void revoke_outputs(const std::string &reason) { changed(reason); }
  bool reject_supported_prediction(PhysicalEventKind,
                                   const PredictedViolationBound &);
  bool observe(BallSample);
  bool qualify_candidate();
  bool context(MatchContext, std::int64_t time_ns, const std::string &source,
               std::uint64_t generation);
  bool confirm(PhysicalEventKind, std::int64_t occurrence_ns,
               std::int64_t confirmed_ns, const ModelState &pre,
               const std::string &source, std::int64_t uncertainty_ns = 0);
  void predict_events(const std::vector<PhysicalEvent> &,
                      std::int64_t reference_ns);
  ShotToken snapshot();
  ShotSnapshot export_snapshot() const;
  void restore_snapshot(const ShotSnapshot &);
  bool publishable(const ShotToken &, std::int64_t now_source_ns);
  PlayState state() const { return state_; }
  PlayState route() const { return route_; }
  ClassificationStatus classification() const { return classification_; }
  HistoryContinuity continuity() const { return continuity_; }
  MatchContext match_context() const { return match_; }
  bool active() const { return state_ != PlayState::INVALID; }
  bool net_crossed() const { return net_crossed_; }
  int direction() const { return direction_; }
  std::uint64_t generation() const { return generation_; }
  std::uint64_t revision() const { return revision_; }
  const std::deque<BallSample> &samples() const { return samples_; }
  const std::deque<BallSample> &candidates() const { return candidates_; }
  const std::vector<ShotEvent> &events() const { return events_; }
  const EffectiveSpin &spin() const { return spin_.value(); }
  const std::string &reason() const { return reason_; }
  int observed_contacts() const;
  bool route_supported() const;
  void set_history_continuity(HistoryContinuity);
  const std::string &context_source() const { return match_source_; }
  std::int64_t context_time_ns() const { return match_time_ns_; }

private:
  void reclassify();
  void changed(const std::string &);
  bool compatible(const BallSample &, const BallSample &) const;
  ProbabilisticConfig config_;
  ShotSettings settings_;
  TrustedSpin spin_;
  PlayState state_ = PlayState::INVALID, route_ = PlayState::INVALID;
  ClassificationStatus classification_ = ClassificationStatus::PROVISIONAL;
  HistoryContinuity continuity_ = HistoryContinuity::kUnknown;
  MatchContext match_ = MatchContext::UNKNOWN;
  std::string match_source_, reason_ = "startup";
  std::int64_t match_time_ns_ = 0, boundary_ns_ = 0;
  std::uint64_t generation_ = 0, revision_ = 0, next_event_id_ = 1,
                solve_sequence_ = 0, published_sequence_ = 0;
  int direction_ = 0;
  bool net_crossed_ = false, rejected_latch_ = false,
       boundary_qualified_ = false;
  std::deque<BallSample> samples_, candidates_;
  std::vector<ShotEvent> events_;
};
struct RouteTarget {
  bool valid = false, incoming = false, historical = false,
       known_forbidden = false;
  ForecastStatus status = ForecastStatus::WARMING_UP;
  std::optional<PhysicalEvent> event;
  std::string reason = "warming_up";
};
// Evaluates observed+remaining events only through the selected route target.
RouteTarget select_route_target(const ShotContext &,
                                const std::vector<PhysicalEvent> &,
                                const ProbabilisticConfig &,
                                bool plane_available);
struct FlightPrediction {
  ShotToken token;
  PlayState state = PlayState::INVALID, route = PlayState::INVALID;
  ForecastStatus status = ForecastStatus::REVOKED;
  bool physical_available = false;
  RouteTarget target;
  StateBelief belief;
  NominalBallForecast nominal;
  std::vector<PhysicalEvent> events;
  HitPlaneSnapshot plane;
  std::int64_t reference_ns = 0, arrival_dt_ns = 0, arrival_ns = 0,
               expires_ns = 0;
  std::string reason, model_config_id;
};
class FlightPipeline {
public:
  FlightPipeline(ProbabilisticConfig, ShotSettings = {});
  const ProbabilisticConfig &config() const { return config_; }
  const ShotSettings &settings() const { return settings_; }
  ShotContext &shot() { return shot_; }
  const ShotContext &shot() const { return shot_; }
  bool observe(BallSample);
  void start(std::int64_t, int, BoundaryEvidence, HistoryContinuity);
  void invalidate(const std::string &);
  FlightPrediction forecast(const HitPlaneSnapshot &,
                            std::int64_t now_source_ns);
  FullTrackerCheckpoint checkpoint() const {
    return tracker_.export_checkpoint();
  }
  std::int64_t source_origin_ns() const { return origin_.value_or(0); }
  void restore(const ShotSnapshot &, const FullTrackerCheckpoint &,
               std::int64_t origin_ns);
  bool accept(const ShotToken &token, std::int64_t now_source_ns) {
    return shot_.publishable(token, now_source_ns);
  }

private:
  ProbabilisticConfig config_;
  ShotSettings settings_;
  ShotContext shot_;
  ProbabilisticBallTracker tracker_;
  std::optional<std::int64_t> origin_;
  std::uint64_t tracker_generation_ = 0;
};
// One running solve plus one replaceable latest input. Close wakes the worker.
template <class T> class LatestWork {
public:
  bool push(T value) {
    std::lock_guard<std::mutex> guard(mutex_);
    if (closed_)
      return false;
    if (pending_)
      ++superseded_;
    pending_ = std::move(value);
    cv_.notify_one();
    return true;
  }
  std::optional<T> take() {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [&] { return closed_ || pending_.has_value(); });
    if (closed_)
      return {};
    auto value = std::move(pending_);
    pending_.reset();
    return value;
  }
  void close() {
    std::lock_guard<std::mutex> guard(mutex_);
    closed_ = true;
    pending_.reset();
    cv_.notify_all();
  }
  std::uint64_t superseded() const {
    std::lock_guard<std::mutex> guard(mutex_);
    return superseded_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<T> pending_;
  bool closed_ = false;
  std::uint64_t superseded_ = 0;
};
} // namespace hope_planner_cpp
