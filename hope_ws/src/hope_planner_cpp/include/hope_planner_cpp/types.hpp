#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

namespace hope_planner_cpp {

using Vec3 = Eigen::Vector3d;

constexpr std::size_t kMaxEstimatorSamples = 192;
constexpr std::size_t kInputRingCapacity = 1024;

struct BallPhysics {
  double drag_k = 0.1261;
  double magnus_k = 0.00444;
  double restitution_h = 0.64;
  double restitution_v = 0.9215;
  // Nakashima/Ace Coulomb prior (not venue-fit).
  double nakashima_friction_mu = 0.25;
  // Retained OptiTrack grip law from ball_physics_venue.yaml.  This is a
  // separate candidate from the Nakashima rolling-cap law above.
  double table_tangential_gain = 0.369;
  double table_friction_cap_mu = 2.0;
  Vec3 gravity{0.0, 0.0, -9.81};
  double radius = 0.02;
};

struct TableParams {
  double length = 2.74;
  double width = 1.525;
  double y_max = 0.0;
  double net_x = 1.37;
  double net_height = 0.1525;
  double net_overhang = 0.15;
};

struct PlannerConfig {
  double x_hit = 0.15;
  Vec3 target_land{2.055, -0.7625, 0.0};
  double delta_t_flight = 0.50;
  double restitution_racket = 0.654;
  double restitution_exp_g1 = 0.759;
  double restitution_exp_g2 = -0.0441;
  double integrate_dt_s = 0.001;
  double max_predict_time_s = 2.0;
  bool adaptive_predict_horizon = false;
  double max_predict_time_cap_s = 3.0;
};

struct EstimatorConfig {
  double window_s = 0.18;
  double min_span_s = 0.08;
  std::size_t min_samples = 12;
  double huber_delta_m = 0.003;
  // Optional audit-tuned temporal weighting. Zero keeps uniform window
  // weights; a positive value is the half-life for older raw samples.
  double recency_half_life_s = 0.0;
  int robust_iterations = 3;
  double integration_dt_s = 0.001;
  double bounce_center_z_max_m = 0.20;
  // Causal contact reversal evidence and physical re-arm interval. These
  // select the piecewise estimator model; they are not command gates.
  double bounce_min_reversal_m = 0.00005;
  double bounce_min_excursion_m = 0.001;
  std::size_t bounce_confirmation_samples = 5;
  double bounce_confirmation_max_span_s = 0.05;
  double bounce_sparse_confirmation_min_span_s = 0.012;
  double bounce_sparse_confirmation_excursion_m = 0.005;
  double bounce_refractory_s = 0.12;
};

struct SpinEstimatorConfig {
  double window_s = 0.10;
  double min_span_s = 0.05;
  double max_gap_s = 0.05;
  double max_rev_s = 20.0;
  double huber_delta_rev_s = 2.0;
  std::size_t min_increments = 3;
  int robust_iterations = 5;
};

struct BallSample {
  double source_time_s = 0.0;
  Vec3 position = Vec3::Zero();
  // Motive/ROS orientation of the tracked ball, normalized in the callback.
  // Position estimation never consumes this field.  Invalid or absent
  // orientation therefore cannot suppress the legacy command path.
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  bool orientation_valid = false;
  std::int64_t receipt_steady_ns = 0;
  std::int64_t receipt_wall_ns = 0;
  std::uint64_t sequence = 0;
};

struct SpinEstimate {
  Vec3 omega_rad_s = Vec3::Zero();
  double sample_span_s = 0.0;
  double retained_time_fraction = 0.0;
  double coherence = 0.0;
  std::size_t retained_increments = 0;
  std::size_t rejected_increments = 0;
  bool valid = false;
  std::string reason = "spin_not_ready";
};

enum class SpinPhysicsMode {
  kLegacyNoSpin,
  kNakashimaBounce,
  kNakashimaBounceAndMagnus,
  kVenueGripBounce,
  kVenueGripBounceAndMagnus,
};

struct BallState {
  Vec3 position = Vec3::Zero();
  Vec3 velocity = Vec3::Zero();
  double source_time_s = 0.0;
  std::size_t sample_count = 0;
  double sample_span_s = 0.0;
  double residual_rms_m = std::numeric_limits<double>::quiet_NaN();
  double residual_max_m = std::numeric_limits<double>::quiet_NaN();
  // Bounce-transition fields are estimator audit only. They are not packed
  // into schema-2 and never participate in command admission.
  bool bounce_transition_used = false;
  bool bounce_epoch_active = false;
  double bounce_source_time_s = std::numeric_limits<double>::quiet_NaN();
  std::size_t pre_bounce_samples = 0;
  std::size_t post_bounce_samples = 0;
  bool valid = false;
  std::string reason = "estimator_not_ready";
};

struct StrikeTarget {
  Vec3 ball_position = Vec3::Zero();
  Vec3 ball_velocity = Vec3::Zero();
  double strike_source_time_s = 0.0;
  int predicted_bounces = 0;
  bool valid = false;
  std::string reason = "not_run";
};

struct RacketCommand {
  Vec3 position = Vec3::Zero();
  Vec3 velocity = Vec3::Zero();
  Vec3 normal{1.0, 0.0, 0.0};
  Vec3 outgoing_ball_velocity = Vec3::Zero();
  Vec3 target_land = Vec3::Zero();
  double strike_source_time_s = 0.0;
  int predicted_bounces = 0;
  bool clears_net = false;
  bool bypasses_net_posts = false;
  bool valid = false;
  std::string reason = "not_run";
};

struct SolveAudit {
  double estimator_ms = 0.0;
  double stage2_ms = 0.0;
  double stage3_ms = 0.0;
  double total_ms = 0.0;
  std::size_t input_samples_consumed = 0;
  std::size_t input_samples_coalesced = 0;
  std::uint64_t trajectory_epoch = 0;
  std::uint64_t snapshot_sequence = 0;
  double segment_start_source_time_s =
      std::numeric_limits<double>::quiet_NaN();
  double previous_segment_last_source_time_s =
      std::numeric_limits<double>::quiet_NaN();
  std::string segment_boundary_reason = "none";
  std::string reason = "not_run";
};

struct SpinShadowAudit {
  SpinEstimate spin;
  StrikeTarget strike;
  std::string mode = "disabled";
  bool enabled = false;
};

}  // namespace hope_planner_cpp
