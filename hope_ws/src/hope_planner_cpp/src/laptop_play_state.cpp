#include "hope_planner_cpp/laptop_play_state.hpp"
#include "hope_planner_cpp/ball_dynamics.hpp"

#include <Eigen/QR>
#include <algorithm>
#include <iterator>
#include <limits>

namespace hope_planner_cpp {
namespace {
constexpr double kNet = 1.37, kLength = 2.74, kWidth = 1.525, kRadius = 0.02;
constexpr double kNetHeight = 0.1525;
constexpr double kSpeedWindowS = 0.040, kSpeedPairSpanS = 0.020;
constexpr double kLowReturnRiseM = 0.025;
// Retain the Score Board's contact display interval.
constexpr double kBounceDisplayS = 0.225;
// Calibrated on 244 saved ChingMu bags. The p99 low-motion height span was
// 15.55 mm and the p99.5 height interpolation residual was 5.25 mm.
constexpr double kRepeatClearanceM = 0.016, kRepeatMotionFloorM = 0.006;
constexpr double kRepeatClearanceSpanS = 0.006, kRepeatMotionSpanS = 0.012;
// Source seconds are epoch doubles; contemporary Unix timestamps have ~0.24 us
// resolution. Do not require an exact 40 ms subtraction at that magnitude.
constexpr double kSourceTimeToleranceS = 1e-6;
bool is_serve(PlayState s) { return s == PlayState::SERVE || s == PlayState::OPPONENT_SERVE; }

double median(std::vector<double>& values) {
  auto mid = values.begin() + values.size()/2;
  std::nth_element(values.begin(), mid, values.end());
  if (values.size() % 2) return *mid;
  return (*mid + *std::max_element(values.begin(), mid))/2;
}

bool has_vertical_motion(const std::deque<BallSample>& samples,
                         std::size_t first, std::size_t last, int direction) {
  const double start = samples[first].source_time_s;
  const double span = samples[last].source_time_s - start;
  if (last - first < 2 || span < kRepeatMotionSpanS - kSourceTimeToleranceS) return false;

  // Fit a median secant slope without the putative contact sample. A lone
  // bad exposure cannot provide both approach and rebound evidence.
  std::vector<double> slopes;
  for (std::size_t i = first; i < last; ++i) {
    for (std::size_t j = i + 1; j <= last; ++j) {
      const double dt = samples[j].source_time_s - samples[i].source_time_s;
      if (dt >= 0.004)
        slopes.push_back((samples[j].position.z() - samples[i].position.z())/dt);
    }
  }
  const double slope = median(slopes);
  const double displacement = direction * slope * span;
  if (displacement < kRepeatMotionFloorM) return false;
  std::vector<double> residuals;
  for (std::size_t i = first; i <= last; ++i)
    residuals.push_back(samples[i].position.z() - slope*(samples[i].source_time_s - start));
  const double center = median(residuals);
  for (auto& residual : residuals) residual = std::abs(residual - center);
  const double noise_sigma = 1.4826 * median(residuals);
  return displacement >= 3 * noise_sigma;
}

}  // namespace

const char* play_state_name(PlayState state) noexcept {
  switch (state) {
    case PlayState::OPPONENT_RALLY: return "OPPONENT_RALLY";
    case PlayState::RALLY: return "RALLY";
    case PlayState::OPPONENT_SERVE: return "OPPONENT_SERVE";
    case PlayState::SERVE: return "SERVE";
    default: return "INVALID";
  }
}

LaptopPlayState::LaptopPlayState(PlayStateConfig config) : config_(config) {}

const char* LaptopPlayState::substate_name() const {
  if (input_missing_) return "";
  if (state_ == PlayState::INVALID) {
    switch (fault_) {
      case Fault::DOUBLE_BOUNCE: return "Double bounce";
      case Fault::OUT_OF_BOUNDS: return "Out of bounds";
      case Fault::MISSED_BOUNCE: return "Missed bounce";
      case Fault::NET_NOT_CLEARED: return "Failed to clear the net";
      default:
        return reason_ == "held_ball" || reason_ == "rally_speed_below_threshold" ? "Ball stopped" : "";
    }
  }
  const bool at_bounce = last_source_ - last_bounce_confirmation_ < kBounceDisplayS;
  const bool serve = is_serve(state_);
  if (receiver_bounces_ > 0) {
    if (!at_bounce) return "Post-flight";
    return serve ? "Second Bounce" : "Bounce";
  }
  if (serve && striker_bounces_ > 0)
    return at_bounce ? "First Bounce" : "Mid-flight";
  return "Pre-Flight";
}

bool LaptopPlayState::incoming_flight() const {
  return !local_stopped_ && !input_missing_ && low_ball_since_ == 0 &&
      !waiting_for_launch() &&
      (state_ == PlayState::OPPONENT_RALLY || state_ == PlayState::OPPONENT_SERVE);
}

bool LaptopPlayState::may_forward() const {
  return incoming_flight();
}

void LaptopPlayState::clear_shot() {
  reset_slow_rally();
  forecast_advisory_.clear();
  history_.clear();
  striker_bounces_ = receiver_bounces_ = 0;
  crossed_net_ = history_known_ = target_reached_ = false;
  reached_receiving_half_ = false;
  return_flight_ = false;
  net_time_ = 0;
  low_ball_since_ = 0;
  last_bounce_ = -1e9;
  last_bounce_height_ = 0;
  last_bounce_confirmation_ = -1e9;
  fault_ = Fault::NONE;
  direction_ = 0;
  ++generation_;
}

void LaptopPlayState::invalidate(const char* why) {
  if (state_ != PlayState::INVALID) {
    rejected_direction_ = direction_;
    rejected_receiver_history_ = crossed_net_ || receiver_bounces_ > 0;
    nonflight_since_ = std::numeric_limits<double>::quiet_NaN();
    neutral_seen_ = false;
    clear_shot();
    candidate_.clear();
    candidate_direction_ = 0;
  }
  serve_armed_ = false;
  receive_armed_ = false;
  state_ = PlayState::INVALID;
  reason_ = why;
}

void LaptopPlayState::pause_observations(const char* why) {
  sample_usable_ = false;
  if (!input_missing_ || !candidate_.empty() || !history_.empty()) {
    // Preserve local intent and already observed contacts, but never fit across
    // an explicit tracking loss or retransmit a packet built before that loss.
    candidate_.clear(); history_.clear(); candidate_direction_ = 0;
    candidate_since_ = 0;
    forecast_advisory_.clear(); reset_slow_rally();
    low_ball_since_ = 0;
    boundary_time_ = last_source_ + kSourceTimeToleranceS;
    ++generation_;
  }
  input_missing_ = true;
  reacquiring_ = !waiting_for_launch();
  history_known_ = false; // Missing exposures cannot prove unobserved contacts.
  reason_ = why;
}

void LaptopPlayState::input_unavailable(const char* why, std::int64_t now) {
  clear_clock_probe();
  reset_slow_rally();
  if (local_stopped_) return;
  // Bad or missing sensor data rejects observations, not the logical point.
  poll(now);
  if (state_ != PlayState::INVALID)
    pause_observations(waiting_for_launch() ? "waiting_for_ball" : why);
}

void LaptopPlayState::reset(const char* why) {
  clear_clock_probe();
  invalidate(why);
  fault_ = Fault::NONE;
  point_ended_ = false;
  input_missing_ = false;
  reacquiring_ = false;
  candidate_.clear();
  candidate_direction_ = rejected_direction_ = 0;
  candidate_since_ = 0;
  neutral_seen_ = false;
  neutral_source_time_ = 0;
  last_source_ = 0;
  last_receipt_ = 0;
}

void LaptopPlayState::end_point(const char* why, Fault fault) {
  if (state_ == PlayState::INVALID) return;
  invalidate(why);
  fault_ = fault;
  // Keep a late swing in the same point out. A separate track or a new
  // launch after sustained non-flight can qualify without a button click.
  point_ended_ = true;
}

bool LaptopPlayState::on_table(const Vec3& p) const {
  return p.x() >= 0 && p.x() <= kLength && p.y() >= -kWidth && p.y() <= 0;
}

bool LaptopPlayState::striker_side(double x, int d) const {
  return d > 0 ? x < kNet - config_.net_margin_m : x > kNet + config_.net_margin_m;
}

double LaptopPlayState::slope() const {
  if (candidate_.size() < 3) return 0;
  const double end = candidate_.back().source_time_s;
  double t = 0, x = 0, tt = 0, tx = 0, n = 0, first = end;
  for (const auto& s : candidate_) {
    if (end - s.source_time_s > config_.direction_window_s) continue;
    const double dt = s.source_time_s - end;
    first = std::min(first, s.source_time_s);
    t += dt; x += s.position.x(); tt += dt * dt; tx += dt * s.position.x(); ++n;
  }
  const double denominator = n * tt - t * t;
  return end - first >= config_.direction_window_s * 0.6 && denominator > 1e-10
      ? (n * tx - t * x) / denominator : 0;
}

void LaptopPlayState::reset_slow_rally() {
  slow_since_ = std::numeric_limits<double>::quiet_NaN();
  slow_window_start_ = last_source_;
}

bool LaptopPlayState::observe_rally_speed(const BallSample& s) {
  if (state_ != PlayState::RALLY && state_ != PlayState::OPPONENT_RALLY) {
    reset_slow_rally();
    return false;
  }
  // Observed XYZ only, after P1/P2 normalization. Take the largest 3D secant
  // speed over >=20 ms baselines in a full 40 ms window (including its preceding
  // exposure when sampling is irregular). A net displacement
  // across an apex/bounce can cancel; one slow axis is not a slow ball. Using
  // the maximum also lets noisy samples defer rejection instead of providing
  // false certainty. This is an operational near-stationary detector, not an
  // assumed lower bound on physically possible instantaneous flight speed.
  const double cutoff = std::max(slow_window_start_, s.source_time_s - kSpeedWindowS);
  auto first = std::find_if(candidate_.begin(), candidate_.end(),
      [cutoff](const BallSample& p) { return p.source_time_s >= cutoff - kSourceTimeToleranceS; });
  if (first != candidate_.begin() && first != candidate_.end() &&
      first->source_time_s > cutoff + kSourceTimeToleranceS &&
      std::prev(first)->source_time_s >= slow_window_start_ - kSourceTimeToleranceS)
    --first;
  if (first == candidate_.end() ||
      s.source_time_s - first->source_time_s < kSpeedWindowS - kSourceTimeToleranceS) {
    slow_since_ = std::numeric_limits<double>::quiet_NaN();
    return false;
  }
  double speed = 0;
  for (auto a = first; a != candidate_.end(); ++a) {
    for (auto b = std::next(a); b != candidate_.end(); ++b) {
      const double dt = b->source_time_s - a->source_time_s;
      if (dt >= kSpeedPairSpanS - kSourceTimeToleranceS)
        speed = std::max(speed, (b->position - a->position).norm() / dt);
    }
  }
  if (speed >= config_.rally_min_speed_mps - 1e-6) {
    slow_since_ = std::numeric_limits<double>::quiet_NaN();
    return false;
  }
  if (!std::isfinite(slow_since_)) slow_since_ = s.source_time_s;
  if (s.source_time_s - slow_since_ < config_.rally_slow_confirm_s - kSourceTimeToleranceS) return false;

  // This measured exception applies to ego RALLY too, unlike forecast/legality
  // advisories. Dump the shot, fits/retries (via generation), and candidates.
  // Only a new observed launch or accepted UI context can rearm this point.
  invalidate("rally_speed_below_threshold");
  point_ended_ = true;
  candidate_.clear();
  candidate_direction_ = 0;
  neutral_seen_ = true;
  neutral_source_time_ = s.source_time_s;
  sample_usable_ = false;
  return true;
}

bool LaptopPlayState::launch_evidence(int receiving_direction) const {
  // A translated held ball is not a launch. Use source-time curvature, not a
  // frame count; an apex retains downward acceleration and is admissible.
  if (candidate_.size() < 6) return false;
  const bool post_net = receiving_direction != 0;
  const double span = post_net ? config_.direction_window_s : config_.launch_span_s;
  const double end = candidate_.back().source_time_s;
  std::vector<const BallSample*> points;
  for (const auto& s : candidate_) {
    if (end - s.source_time_s > span * 1.5 ||
        (neutral_seen_ && s.source_time_s <= neutral_source_time_)) continue;
    // A short arc suffices when all its samples are already over the receiving
    // half. There is no requirement to have witnessed the hit or net crossing.
    if (post_net && (!on_table(s.position) || s.position.z() <= kRadius ||
                     (s.position.x() - kNet) * receiving_direction <= 0)) return false;
    points.push_back(&s);
  }
  if (points.size() < 6 || end - points.front()->source_time_s < span) return false;
  Eigen::MatrixXd a(points.size(), 3);
  Eigen::VectorXd z(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double dt = points[i]->source_time_s - end;
    a.row(i) << 1.0, dt, 0.5 * dt * dt;
    z(i) = points[i]->position.z();
  }
  const Eigen::Vector3d fit = a.colPivHouseholderQr().solve(z);
  return fit.allFinite() && fit.z() < -1.0 && fit.z() > -30.0 &&
      (a * fit - z).norm() / std::sqrt(points.size()) < 0.006;
}

void LaptopPlayState::start(int direction, bool serve, bool continuing_shot) {
  const int observed_bounces = receiver_bounces_;
  const int observed_striker_bounces = striker_bounces_;
  const double observed_bounce_time = last_bounce_;
  const double observed_bounce_height = last_bounce_height_;
  const bool reached_receiving_half = reached_receiving_half_;
  clear_shot();
  if (continuing_shot) {
    striker_bounces_ = observed_striker_bounces;
    receiver_bounces_ = observed_bounces;
    last_bounce_ = observed_bounce_time;
    last_bounce_height_ = observed_bounce_height;
    reached_receiving_half_ = reached_receiving_half;
  }
  point_ended_ = false;
  input_missing_ = reacquiring_ = false;
  direction_ = direction;
  state_ = serve ? (direction > 0 ? PlayState::SERVE : PlayState::OPPONENT_SERVE)
                 : (direction > 0 ? PlayState::RALLY : PlayState::OPPONENT_RALLY);
  serve_armed_ = false;
  receive_armed_ = false;
  history_ = candidate_;
  boundary_time_ = candidate_.front().source_time_s;
  history_known_ = striker_side(candidate_.front().position.x(), direction);
  reason_ = history_known_ ? "qualified_launch" : "missing_pre_net_history";
  rejected_direction_ = 0;
  neutral_seen_ = false;
}

void LaptopPlayState::contact(const BallSample& s) {
  if (s.source_time_s - last_bounce_ < config_.contact_refractory_s) return;
  reset_slow_rally();
  last_bounce_ = s.source_time_s;
  last_bounce_height_ = s.position.z();
  forecast_advisory_.clear();
  if (striker_side(s.position.x(), direction_)) {
    if (return_flight_) {
      end_point("return_bounced_on_striker_side",
                reached_receiving_half_ ? Fault::MISSED_BOUNCE : Fault::NET_NOT_CLEARED);
      return;
    }
    // An opening shot may be a serve with a bounce on the striker's half.
    if (crossed_net_ && s.source_time_s >= net_time_) {
      end_point("striker_contact_after_net", Fault::MISSED_BOUNCE);
      return;
    }
    if (++striker_bounces_ > 1) {
      end_point("second_striker_bounce",
                reached_receiving_half_ ? Fault::MISSED_BOUNCE : Fault::NET_NOT_CLEARED);
      return;
    }
    last_bounce_confirmation_ = last_source_;
    state_ = direction_ > 0 ? PlayState::SERVE : PlayState::OPPONENT_SERVE;
    reason_ = "observed_striker_bounce";
  } else if (std::abs(s.position.x() - kNet) <= config_.net_margin_m) {
    forecast_advisory_ = "ambiguous_net_contact";
  } else {
    // The bounce is direct evidence even when the crossing was outside the capture.
    ++receiver_bounces_;
    if (receiver_bounces_ > 1) {
      end_point(is_serve(state_) ? "third_serve_bounce" : "second_receiver_bounce", Fault::DOUBLE_BOUNCE);
    } else {
      last_bounce_confirmation_ = last_source_;
      reason_ = "observed_receiver_bounce";
    }
  }
}

bool LaptopPlayState::local_intent(const std::string& action) {
  if (action != "READY" && action != "SERVE" && action != "STOP") return false;
  // PREPARE and READY TO SERVE are separate local button presses for the same
  // pending serve. Do not erase its held-ball evidence while it is still armed.
  if (action == "SERVE" && serve_armed_ && !local_stopped_) return true;
  reset(action == "STOP" ? "local_stop" : action == "READY" ? "local_ready" : "local_serve");
  local_stopped_ = action == "STOP";
  if (action == "READY") {
    state_ = PlayState::OPPONENT_SERVE; direction_ = -1; receive_armed_ = true;
  } else if (action == "SERVE") {
    state_ = PlayState::SERVE; direction_ = 1; serve_armed_ = true;
  }
  return true;
}

void LaptopPlayState::poll(std::int64_t steady_ns) {
  if (last_receipt_ <= 0 ||
      (steady_ns - last_receipt_) * 1e-9 <= config_.input_timeout_s) return;
  // Observation timeout invalidates cached fits, never the logical ball state.
  if (state_ != PlayState::INVALID)
    pause_observations(waiting_for_launch() ? "waiting_for_ball" : "ball_observation_missing");
}

void LaptopPlayState::clear_clock_probe() {
  clock_probe_first_source_ = clock_probe_last_source_ = 0;
  clock_probe_first_receipt_ = clock_probe_last_receipt_ = 0;
}

bool LaptopPlayState::accept_source_time(const BallSample& s) {
  if (s.source_time_s > last_source_) { clear_clock_probe(); return true; }
  // A duplicate/reordered sample is not a new epoch and cannot renew freshness.
  // Only recover after the old stream expires and lower timestamps advance for
  // 60 ms in source AND monotonic receipt time, with no gap over the watchdog.
  // The ROS boundary separately requires every exposure to be wall-clock fresh.
  if (s.source_time_s == last_source_ || s.receipt_steady_ns <= last_receipt_) {
    clear_clock_probe();
    return false;
  }
  const double source_delta = s.source_time_s - clock_probe_last_source_;
  const double receipt_delta = (s.receipt_steady_ns - clock_probe_last_receipt_) * 1e-9;
  if (!clock_probe_last_receipt_ || source_delta <= 0 || receipt_delta <= 0 ||
      source_delta > config_.input_timeout_s || receipt_delta > config_.input_timeout_s ||
      std::abs(source_delta - receipt_delta) > 0.020) {
    clock_probe_first_source_ = s.source_time_s;
    clock_probe_first_receipt_ = s.receipt_steady_ns;
  }
  clock_probe_last_source_ = s.source_time_s;
  clock_probe_last_receipt_ = s.receipt_steady_ns;
  poll(s.receipt_steady_ns);
  if ((s.receipt_steady_ns - last_receipt_) * 1e-9 <= config_.input_timeout_s ||
      s.source_time_s - clock_probe_first_source_ < .060 - kSourceTimeToleranceS ||
      s.receipt_steady_ns - clock_probe_first_receipt_ < 60'000'000LL) return false;

  const auto old_generation = generation_;
  if (state_ != PlayState::INVALID) pause_observations("source_clock_rebased");
  if (generation_ == old_generation) clear_shot();
  candidate_.clear(); candidate_direction_ = rejected_direction_ = 0;
  candidate_since_ = 0; point_ended_ = neutral_seen_ = false;
  neutral_source_time_ = 0; last_source_ = 0; last_receipt_ = 0;
  boundary_time_ = 0;
  reset_slow_rally(); clear_clock_probe();
  // Clock recovery does not synthesize local button intent or release local STOP.
  return true;
}

bool LaptopPlayState::observe(const BallSample& s) {
  sample_usable_ = false;
  sample_rejection_reason_ = "";
  if (local_stopped_) { sample_rejection_reason_ = "local_stop"; return false; }
  if (!std::isfinite(s.source_time_s) || s.source_time_s <= 0 || !s.position.allFinite()) {
    sample_rejection_reason_ = "invalid_sample";
    input_unavailable("invalid_sample", s.receipt_steady_ns);
    return false;
  }
  if (!accept_source_time(s)) { sample_rejection_reason_ = "source_not_advancing_or_clock_unconfirmed"; return false; }
  // Bursty timestamps cannot support an adjacent-point velocity. Reject the
  // exposure without poisoning the current flight or renewing freshness.
  if (last_source_ > 0 && s.source_time_s - last_source_ < 0.001) {
    sample_rejection_reason_ = "source_interval_below_1ms"; return false;
  }
  // Sparse exposures cannot prove continuously slow motion. Require a new
  // complete speed window after any gap over 20 ms, even below the loss timeout.
  if (last_source_ > 0 && s.source_time_s - last_source_ > kSpeedPairSpanS + kSourceTimeToleranceS) {
    reset_slow_rally();
    slow_window_start_ = s.source_time_s;
    // VRPN may simply stop publishing rather than emit an empty PoseArray.
    // On reappearance, do not fit or infer contacts across the missing span.
    if (state_ != PlayState::INVALID)
      pause_observations(waiting_for_launch() ? "waiting_for_ball" : "ball_tracking_gap");
  }
  if (last_source_ > 0 &&
      (s.source_time_s - last_source_ > config_.input_timeout_s ||
       (s.receipt_steady_ns - last_receipt_) * 1e-9 > config_.input_timeout_s)) {
    if (state_ != PlayState::INVALID)
      pause_observations(waiting_for_launch() ? "waiting_for_ball" : "ball_observation_missing");
    else { rejected_direction_ = 0; point_ended_ = false; }
    candidate_.clear(); candidate_direction_ = 0;
  }
  if (!candidate_.empty() && (s.position - candidate_.back().position).norm() /
      (s.source_time_s - candidate_.back().source_time_s) > 35.0) {
    // Quarantine the outlier. Reacquire from a fresh directional arc instead
    // of latching every subsequent valid ball behind one bad exposure.
    if (state_ != PlayState::INVALID) pause_observations("position_discontinuity");
    else { rejected_direction_ = 0; point_ended_ = false; }
    sample_rejection_reason_ = "position_discontinuity";
    candidate_.clear(); candidate_direction_ = 0; return false;
  }
  if (input_missing_ && !reacquiring_) {
    input_missing_ = false;
    reason_ = waiting_for_launch() ? "waiting_for_launch" : "tracking_reacquired";
  }
  last_source_ = s.source_time_s;
  last_receipt_ = s.receipt_steady_ns;
  candidate_.push_back(s);
  while (candidate_.size() > kMaxEstimatorSamples ||
         s.source_time_s - candidate_.front().source_time_s > 0.25) candidate_.pop_front();
  if (state_ == PlayState::INVALID && point_ended_ && !neutral_seen_) {
    // A caught ball may keep moving in the player's hand. Require sustained
    // non-flight / neutral-X evidence, then a new directional ballistic launch;
    // a vertical preparation bounce is not an already incoming shot. A bounce's
    // short fit discontinuity must not resurrect an already invalid point.
    if (launch_evidence() && std::abs(slope()) >= config_.direction_speed_mps) nonflight_since_ = std::numeric_limits<double>::quiet_NaN();
    else {
      if (!std::isfinite(nonflight_since_)) nonflight_since_ = s.source_time_s;
      if (s.source_time_s - nonflight_since_ >= config_.held_span_s) {
        neutral_seen_ = true; neutral_source_time_ = s.source_time_s;
      }
    }
  }
  if (observe_rally_speed(s)) return true;
  if (s.position.z() < -0.04 && receiver_bounces_ == 0) {
    if (!waiting_for_launch())
      end_point("under_table", on_table(s.position) ? Fault::MISSED_BOUNCE : Fault::OUT_OF_BOUNDS);
    return true;
  }
  // A ball beside the table can still land legally; only an observed miss
  // at table height proves it is out.
  const double vx = slope();
  const int d = vx < -config_.direction_speed_mps ? -1 : vx > config_.direction_speed_mps ? 1 : 0;
  sample_usable_ = !receive_armed_ || d <= 0;
  if (d != candidate_direction_) { candidate_direction_ = d; candidate_since_ = s.source_time_s; }
  const bool confirmed = d != 0 && s.source_time_s - candidate_since_ >= config_.direction_confirm_s;
  if (candidate_.size() > 2 && s.source_time_s - candidate_.front().source_time_s >= config_.held_span_s) {
    double displacement = 0;
    for (const auto& p : candidate_) displacement = std::max(displacement, (s.position - p.position).norm());
    if (displacement < config_.held_distance_m) {
      neutral_seen_ = true;
      neutral_source_time_ = s.source_time_s;
      sample_usable_ = false;
      if (!waiting_for_launch() && (is_serve(state_) || state_ == PlayState::OPPONENT_RALLY)) {
        // An observed serve ends when the ball comes to rest, even after a tracking gap.
        invalidate("held_ball");
        input_missing_ = reacquiring_ = false;
        neutral_seen_ = true;
      }
      return true;
    }
  }
  if (reacquiring_) {
    sample_usable_ = false;
    if (!confirmed) return true;
    const bool post_net = launch_evidence(d);
    if (!post_net && !launch_evidence()) return true;
    // If an old shot already reached the receiver, an arc starting back on
    // the striker's side is a new shot, even when its direction is unchanged.
    const bool continuing = d == direction_ &&
        !(reached_receiving_half_ && striker_side(candidate_.front().position.x(), d));
    const bool same_serve = !post_net && is_serve(state_) && continuing;
    start(d, same_serve, continuing);
    if (continuing) history_known_ = false;
    reason_ = "tracking_reacquired";
    sample_usable_ = true;
    record_events(s);
    return true;
  }
  if (ego_flight() && !serve_armed_ && neutral_seen_) {
    // A held outgoing ball may start a new independently observed shot.
    if (!confirmed || !launch_evidence()) return true;
    const double clean_boundary = std::max(
        candidate_since_ - config_.direction_window_s, neutral_source_time_ + 1e-6);
    while (candidate_.size() > 2 && candidate_.front().source_time_s < clean_boundary)
      candidate_.pop_front();
    start(d, false); // Clears return flag, contact ledger and downstream fits.
    reason_ = "observed_launch_after_hold";
    record_events(s);
    return true;
  }
  if (state_ == PlayState::INVALID || waiting_for_launch()) {
    if (!confirmed || (serve_armed_ && d != 1) || (receive_armed_ && d != -1)) return true;
    const bool post_net = launch_evidence(d);
    const bool clear_striker_side = striker_side(s.position.x(), d) &&
        std::abs(s.position.x() - kNet) > 0.25 && s.position.z() > 0.10;
    // A reversal after a witnessed crossing or receiving bounce can still be
    // part of the rejected point. When the old track lacks that history,
    // a confirmed launch on the striker's side can establish a new shot.
    const bool separate_reversal = d == -rejected_direction_ &&
        fault_ != Fault::NET_NOT_CLEARED &&
        (point_ended_ ? clear_striker_side && !rejected_receiver_history_ :
                        clear_striker_side || post_net);
    const bool rearm = rejected_direction_ == 0 || neutral_seen_ ||
        separate_reversal;
    if (!rearm || (!post_net && !launch_evidence())) return true;
    // Keep only the newly confirmed directional arc, never the previous shot.
    const bool returning_after_rejection = rejected_direction_ != 0 && d == -rejected_direction_;
    const double clean_boundary = std::max(candidate_since_ -
        (returning_after_rejection ? 0.0 : config_.direction_window_s),
        neutral_seen_ ? neutral_source_time_ + 1e-6 : 0.0);
    while (candidate_.size() > 2 && candidate_.front().source_time_s < clean_boundary)
      candidate_.pop_front();
    // READY requests reception; it is not evidence of an opponent serve.
    start(d, serve_armed_);
  } else {
    history_.push_back(s);
    while (history_.size() > kMaxEstimatorSamples || s.source_time_s - history_.front().source_time_s > 0.4)
      history_.pop_front();
  }

  // Confirm contacts before evaluating the return. Otherwise the last receive
  // bounce can disappear when an opposite-direction shot resets its ledger.
  record_events(s);
  if (state_ == PlayState::INVALID) return true;
  const bool low_ball = low_ball_since_ > 0;
  if (confirmed && d == -direction_ && reached_receiving_half_ && receiver_bounces_ == 0) {
    end_point("return_before_receiver_bounce", Fault::MISSED_BOUNCE);
    return true;
  }
  if (confirmed && d == -direction_) {
    const bool ego = ego_flight();
    const auto& flight = ego ? candidate_ : history_;
    // Locate the actual X turnaround, not the later direction-confirmation
    // sample: a spin-driven table reversal must not masquerade as a racket hit.
    const auto turn = *std::max_element(flight.begin(), flight.end(),
        [this](const BallSample& a, const BallSample& b) {
          return a.position.x()*direction_ < b.position.x()*direction_;
        });
    // A low reversal at the net without a crossing belongs to the same shot.
    if (!reached_receiving_half_ && receiver_bounces_ == 0 && on_table(turn.position) &&
        std::abs(turn.position.x() - kNet) <= config_.net_margin_m + kRadius &&
        turn.position.z() <= kNetHeight + kRadius) {
      end_point("net_rebound", Fault::NET_NOT_CLEARED);
      return true;
    }
    const bool receiver_turn = striker_side(turn.position.x(), d) &&
        std::abs(turn.position.x()-kNet) > .25 && turn.source_time_s > last_bounce_;
    bool returned = ego ? on_table(s.position) && s.position.z() >= kRadius
                        : receiver_turn && turn.position.z() > .10;
    if (low_ball) {
      // A rescue can start below the playing surface. Require a confirmed
      // reversal on the receiver's side and a rise back above table height.
      returned = receiver_turn && s.position.z() >= kRadius &&
                 s.position.z() - turn.position.z() >= kLowReturnRiseM;
    }
    if (returned) {
      while (candidate_.size() > 2 && candidate_.front().source_time_s < turn.source_time_s)
        candidate_.pop_front();
      const bool witnessed_return = receiver_bounces_ == 1;
      start(d, false);
      return_flight_ = !ego || witnessed_return;
      reason_ = ego ? "observed_incoming_return" : "observed_racket_return";
      if (ego || low_ball) {
        history_known_ = witnessed_return;
        record_events(s);
      }
      return true;
    }
    if (!ego && !low_ball) {
      pause_observations("unconfirmed_racket_reversal");
      return true;
    }
  }
  if (low_ball && s.source_time_s - low_ball_since_ >= config_.low_return_window_s)
    end_point("no_return_after_low_ball", Fault::OUT_OF_BOUNDS);
  return true;
}

bool LaptopPlayState::distinct_repeat_contact(std::size_t left, std::size_t index,
                                             std::size_t right) const {
  const double clearance = std::max(last_bounce_height_, history_[index].position.z()) + kRepeatClearanceM;
  double raised_since = 0;
  int raised_samples = 0;
  for (std::size_t i = 0; i < index; ++i) {
    const auto& sample = history_[i];
    if (sample.source_time_s <= last_bounce_ || sample.position.z() < clearance) {
      raised_since = 0;
      raised_samples = 0;
    } else {
      if (raised_since == 0) raised_since = sample.source_time_s;
      if (++raised_samples >= 3 &&
          sample.source_time_s - raised_since >= kRepeatClearanceSpanS - kSourceTimeToleranceS)
        return has_vertical_motion(history_, left, index - 1, -1) &&
               has_vertical_motion(history_, index + 1, right, 1);
    }
  }
  return false;
}

void LaptopPlayState::record_events(const BallSample& s) {
  if ((s.position.x() - kNet) * direction_ > config_.net_margin_m)
    reached_receiving_half_ = true;
  // A shot still needs its receiving bounce. After a confirmed bounce, falling
  // below the table only starts the window for the receiver to return the ball.
  if (receiver_bounces_ > 0 && low_ball_since_ == 0 && s.position.z() < kRadius &&
      (!on_table(s.position) || s.position.z() < -0.04))
    low_ball_since_ = s.source_time_s;
  for (std::size_t i = 1; receiver_bounces_ == 0 && i < history_.size(); ++i) {
    const auto& a=history_[i-1]; const auto& b=history_[i];
    if (a.position.z()>kRadius && b.position.z()<=kRadius) {
      const double f=(a.position.z()-kRadius)/(a.position.z()-b.position.z());
      if (!on_table(a.position+f*(b.position-a.position))) {
        end_point("missed_table_or_return", Fault::OUT_OF_BOUNDS); return;
      }
    }
  }
  if (!crossed_net_ && history_known_ && (s.position.x() - kNet) * direction_ > config_.net_margin_m) {
    crossed_net_ = true; net_time_ = s.source_time_s;
    // The estimated crossing height alone cannot decide whether the shot lands.
    for (std::size_t i = 1; i < history_.size(); ++i) {
      const auto& a = history_[i-1]; const auto& b = history_[i];
      if ((a.position.x()-kNet)*direction_ <= 0 && (b.position.x()-kNet)*direction_ > 0) {
        const double f = (kNet-a.position.x())/(b.position.x()-a.position.x());
        net_time_ = a.source_time_s + f*(b.source_time_s-a.source_time_s);
        break;
      }
    }
  }
  // Confirm a distinct local minimum using exposure-time brackets. The ledger
  // survives window trimming and rally -> serve reclassification.
  for (std::size_t i = 2; i + 2 < history_.size(); ++i) {
    const auto event = history_[i];
    if (event.source_time_s - last_bounce_ < config_.contact_refractory_s ||
        event.position.z() > history_[i-1].position.z() ||
        event.position.z() > history_[i+1].position.z()) continue;
    std::size_t left=i-1, right=i+1;
    while (left > 0 && event.source_time_s-history_[left-1].source_time_s <= .025) --left;
    while (right+1 < history_.size() && history_[right+1].source_time_s-event.source_time_s <= .025) ++right;
    if (i-left >= 2 && right-i >= 2 && event.position.z() >= -0.005 &&
        event.position.z() <= config_.contact_height_m && on_table(event.position) &&
        history_[left].position.z()-event.position.z() >= config_.contact_excursion_m &&
        history_[right].position.z()-event.position.z() >= config_.contact_excursion_m) {
      const bool repeat = striker_side(event.position.x(), direction_) ? striker_bounces_ > 0 :
          receiver_bounces_ > 0 && std::abs(event.position.x() - kNet) > config_.net_margin_m;
      if (repeat && !distinct_repeat_contact(left, i, right)) continue;
      contact(event);
      if (state_ == PlayState::INVALID) return;
    }
  }
  if (is_serve(state_) && crossed_net_ && striker_bounces_ == 0 && s.source_time_s - net_time_ > 0.05) {
    end_point("serve_crossed_net_before_first_bounce", Fault::MISSED_BOUNCE); return;
  }
  if (reached_receiving_half_ && receiver_bounces_ == 0 && history_.size() >= 3) {
    // Allow the two later samples needed to confirm a bounce on the end line.
    const double x = history_[history_.size() - 3].position.x();
    if (direction_ > 0 ? x > kLength : x < 0) {
      end_point("no_receiver_bounce", Fault::MISSED_BOUNCE);
      return;
    }
  }
  const double endpoint = direction_ < 0 ? hit_x_ : kLength;
  if (crossed_net_ && receiver_bounces_ >= 1 && (s.position.x() - endpoint) * direction_ >= 0)
    target_reached_ = true;
}

void LaptopPlayState::forecast(const BallState& fit, const BallPhysics& physics, const SpinEstimate& spin) {
  forecast_advisory_.clear();
  const auto advise = [this](const char* reason) { forecast_advisory_ = reason; };
  if (!incoming_flight() || !history_known_ || target_reached_ ||
      !fit.valid || !fit.position.allFinite() || !fit.velocity.allFinite() ||
      fit.source_time_s < last_bounce_ + 0.035 || std::abs(fit.source_time_s - last_source_) > 0.02 ||
      !std::isfinite(fit.residual_rms_m) || fit.residual_rms_m > 0.004) return;
  if (direction_ < 0 && !std::isfinite(hit_x_)) return;
  // Reuse the production drag/Magnus/contact equations, but keep all forecast
  // events local. Position-only prediction is a hypothesis, not evidence of
  // measured zero spin. v1 has no bound on all relevant uncertainty (even with
  // a spin estimate), so no forecast or repeated agreement may close the gate.
  // Only observed events and real-input/session handling decide validity.
  // The caller applies the selected ball profile's coherence/scale settings.
  const bool spin_valid = spin.valid && spin.omega_rad_s.allFinite();
  Vec3 omega = spin_valid ? spin.omega_rad_s : Vec3::Zero();
  Vec3 p = fit.position, v = fit.velocity;
  int own = striker_bounces_, other = receiver_bounces_;
  bool net = crossed_net_, serve = is_serve(state_);
  const double endpoint = direction_ < 0 ? hit_x_ : kLength;
  const double dt = 0.001;
  for (int i = 0; i < 2000; ++i) {
    const Vec3 a = ball_acceleration(physics, v, spin_valid ? omega : Vec3::Zero());
    const Vec3 next = p + v * dt + 0.5 * a * dt * dt;
    // Order all events within the step; an earlier plane crossing wins over
    // a later bounce. Never count a predicted contact as observed evidence.
    double bounce_f = 2, plane_f = 2, net_f = 2;
    if (p.z() > kRadius && next.z() <= kRadius && v.z() < 0)
      bounce_f = (p.z() - kRadius) / (p.z() - next.z());
    if ((p.x() - endpoint) * direction_ < 0 && (next.x() - endpoint) * direction_ >= 0)
      plane_f = (endpoint - p.x()) / (next.x() - p.x());
    if (!net && (p.x() - kNet) * direction_ <= 0 && (next.x() - kNet) * direction_ > 0)
      net_f = (kNet - p.x()) / (next.x() - p.x());
    if (plane_f <= std::min(bounce_f, net_f) && plane_f <= 1) {
      if (other != 1 || (serve && own != 1)) advise("predicted_target_without_required_bounce");
      return;
    }
    if (net_f <= bounce_f && net_f <= 1) {
      if ((p + net_f * (next - p)).z() < kNetHeight + kRadius) { advise("predicted_net_collision"); return; }
      if (serve && own != 1) { advise("predicted_serve_net_before_bounce"); return; }
      net = true;
    }
    if (bounce_f <= 1) {
      Vec3 impact = p + bounce_f * (next - p);
      if (!on_table(impact)) { advise("predicted_floor_before_target"); return; }
      if (striker_side(impact.x(), direction_)) {
        if (return_flight_) { advise("predicted_return_bounced_on_striker_side"); return; }
        if (net || ++own > 1) { advise("predicted_second_striker_bounce"); return; }
        serve = true;  // Hypothesis only; public label awaits observation.
      } else {
        if (!net) { advise("predicted_wrong_contact_order"); return; }
        if (++other > 1) {
          advise(serve ? "predicted_third_serve_bounce" : "predicted_second_receiver_bounce"); return;
        }
      }
      v += a * (dt * bounce_f);
      if (spin_valid) table_contact(physics, v, omega);
      else {
        v.x() *= physics.restitution_h; v.y() *= physics.restitution_h;
        v.z() *= -physics.restitution_v;
      }
      p = impact; p.z() = kRadius + 1e-8;
      // Contact remainder is integrated on the next iteration; <= 1 ms timing
      // discretization only, not a returned arrival timestamp.
    } else { p = next; v += a * dt; }
    if (p.y() < -kWidth || p.y() > 0) { advise("predicted_side_boundary_exit"); return; }
    if (p.z() < -0.04) { advise("predicted_under_table"); return; }
  }
  // Horizon exhaustion is uncertainty, not evidence of an illegal shot.
}

}  // namespace hope_planner_cpp
