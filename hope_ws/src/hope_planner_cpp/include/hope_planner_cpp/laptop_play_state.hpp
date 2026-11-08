#pragma once

#include "hope_planner_cpp/types.hpp"
#include <deque>
#include <cmath>
#include <string>
#include <vector>

namespace hope_planner_cpp {

// Laptop only. The five public labels are independent of Runner modes.
enum class PlayState { INVALID, OPPONENT_RALLY, RALLY, OPPONENT_SERVE, SERVE };
const char* play_state_name(PlayState state) noexcept;

struct PlayStateConfig {
  double direction_speed_mps = 0.25;
  double direction_window_s = 0.035;
  double direction_confirm_s = 0.020;
  double launch_span_s = 0.060;
  double input_timeout_s = 0.100;
  double rally_min_speed_mps = 0.10;
  double rally_slow_confirm_s = 0.100;
  double held_span_s = 0.180;
  double held_distance_m = 0.008;
  double contact_height_m = 0.045;
  double contact_excursion_m = 0.001;
  double contact_refractory_s = 0.060;
  double net_margin_m = 0.025;
  double low_return_window_s = 0.350;
};

class LaptopPlayState {
 public:
  explicit LaptopPlayState(PlayStateConfig config = {});
  // False means the exposure was rejected; it must not enter downstream fits.
  bool observe(const BallSample& sample);
  void poll(std::int64_t steady_ns);
  // Explicit local Console intent only; never Runner feedback or acknowledgements.
  bool local_intent(const std::string& action);
  bool local_stopped() const { return local_stopped_; }
  void invalidate(const char* reason);
  void input_unavailable(const char* reason, std::int64_t steady_ns);
  void reset(const char* reason);
  void set_hit_plane(double x) { if (std::isfinite(x)) hit_x_ = x; }
  void clear_hit_plane() { hit_x_ = std::numeric_limits<double>::quiet_NaN(); }

  PlayState state() const { return state_; }
  const char* substate_name() const;
  // Qualified incoming evidence is independent of robot operating mode.
  bool incoming_flight() const;
  bool may_forward() const;
  bool ego_flight() const { return state_ == PlayState::RALLY || state_ == PlayState::SERVE; }
  bool return_confirmed() const { return return_flight_ && direction_ < 0; }
  std::uint64_t generation() const { return generation_; }
  const std::string& reason() const { return reason_; }
  const std::string& forecast_advisory() const { return forecast_advisory_; }
  bool history_known() const { return history_known_; }
  bool serve_armed() const { return serve_armed_; }
  bool receive_armed() const { return receive_armed_; }
  bool waiting_for_launch() const { return serve_armed_ || receive_armed_; }
  const char* sample_rejection_reason() const { return sample_rejection_reason_; }
  bool observation_missing() const { return input_missing_; }
  bool sample_usable() const { return sample_usable_; }
  int striker_bounces() const { return striker_bounces_; }
  int receiver_bounces() const { return receiver_bounces_; }
  double boundary_time() const { return boundary_time_; }
  double latest_time() const { return last_source_; }
  double last_bounce_time() const { return last_bounce_; }
  std::size_t retained_samples() const { return history_.size(); }
  const std::deque<BallSample>& samples() const { return history_; }
  // Evaluate only a qualified, current free-flight fit. No target is generated
  // or sent to A3 here. All v1 forecast violations are advisory: they never
  // mutate validity, shot identity, forwarding or the observed contact ledger.
  void forecast(const BallState& fit, const BallPhysics& physics,
                const SpinEstimate& spin);

 private:
  enum class Fault { NONE, DOUBLE_BOUNCE, OUT_OF_BOUNDS, MISSED_BOUNCE, NET_NOT_CLEARED };

  void start(int direction, bool serve, bool continuing_shot = false);
  void contact(const BallSample& sample);
  bool distinct_repeat_contact(std::size_t left, std::size_t index, std::size_t right) const;
  void record_events(const BallSample& sample);
  void end_point(const char* reason, Fault fault);
  bool on_table(const Vec3& p) const;
  bool striker_side(double x, int direction) const;
  bool launch_evidence(int receiving_direction = 0) const;
  double slope() const;
  bool observe_rally_speed(const BallSample& sample);
  void reset_slow_rally();
  bool accept_source_time(const BallSample& sample);
  void clear_clock_probe();
  void clear_shot();
  void pause_observations(const char* reason);

  PlayStateConfig config_;
  PlayState state_ = PlayState::INVALID;
  Fault fault_ = Fault::NONE;
  std::string reason_ = "startup";
  std::string forecast_advisory_;
  std::deque<BallSample> candidate_, history_;
  std::uint64_t generation_ = 0;
  int direction_ = 0, candidate_direction_ = 0, rejected_direction_ = 0;
  int striker_bounces_ = 0, receiver_bounces_ = 0;
  bool crossed_net_ = false, history_known_ = false, target_reached_ = false;
  bool reached_receiving_half_ = false;
  bool return_flight_ = false, point_ended_ = false;
  bool rejected_receiver_history_ = false;
  double nonflight_since_ = std::numeric_limits<double>::quiet_NaN();
  bool serve_armed_ = false;
  bool receive_armed_ = false, sample_usable_ = false;
  bool local_stopped_ = false;
  bool input_missing_ = false;
  bool reacquiring_ = false;
  const char* sample_rejection_reason_ = "";
  bool neutral_seen_ = false;
  double candidate_since_ = 0, boundary_time_ = 0, last_source_ = 0;
  double last_bounce_ = -1e9, hit_x_ = std::numeric_limits<double>::quiet_NaN();
  double last_bounce_height_ = 0;
  double last_bounce_confirmation_ = -1e9;
  double net_time_ = 0;
  double low_ball_since_ = 0;
  double neutral_source_time_ = 0;
  double slow_window_start_ = 0;
  double slow_since_ = std::numeric_limits<double>::quiet_NaN();
  std::int64_t last_receipt_ = 0;
  double clock_probe_first_source_ = 0, clock_probe_last_source_ = 0;
  std::int64_t clock_probe_first_receipt_ = 0, clock_probe_last_receipt_ = 0;
};

}  // namespace hope_planner_cpp
