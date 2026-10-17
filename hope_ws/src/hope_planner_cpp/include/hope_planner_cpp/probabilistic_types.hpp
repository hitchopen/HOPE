#pragma once
#include "hope_planner_cpp/types.hpp"
#include <Eigen/Core>
#include <cstdint>
#include <string>
#include <vector>

namespace hope_planner_cpp {
using State6 = Eigen::Matrix<double, 6, 1>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;
using Matrix3 = Eigen::Matrix3d;
enum class RallyContext { kUnknown, kServeReturn, kNormalReturn };
enum class BallMotionState { kUncertain, kIncoming, kOutgoing };
enum class PlannerBallState { kServe, kReturnServe, kIncoming, kOutgoing };
const char *name(RallyContext);
const char *name(BallMotionState);
const char *name(PlannerBallState);

struct ProbabilisticConfig {
  BallPhysics physics;
  TableParams table;
  Vec3 measurement_std_m{.003, .003, .003};
  Vec3 measurement_bias_std_m = Vec3::Zero();
  double measurement_bias_tau_s = .060;
  // Broad, direction-neutral prior. Strike-time incoming statistics are not a
  // valid prior for every phase, particularly serves and outgoing balls.
  Vec3 velocity_prior_mean = Vec3::Zero();
  Vec3 velocity_prior_std{4., 2., 3.};
  double acceleration_spectral_density = .10; // m^2/s^3
  Vec3 bounce_velocity_std{.65, .30, .45};    // m/s; uncertain nominal impulse
  double integration_dt_s = .001;
  double trajectory_dt_s = .005;
  double horizon_s = 1.5;
  double x_hit = .15;
  std::size_t ensemble_size = 48;
  double max_tracking_gap_s = .35;
  double gap_warning_s = .025;
  double clock_reset_backstep_s = .1;
  double innovation_gate_squared = 25.;
  // Four independent rejected measurements permit explicit track reacquisition.
  std::size_t reacquire_rejections = 4;
  double direction_speed_mps = .25;
  double direction_sigma = 2.5;
  std::size_t direction_confirmations = 2;
  double direction_window_s = .045;
  // Table-surface frame, tracked point is currently ball center. This threshold
  // is an activity criterion, not the physical contact height (ball radius).
  double min_tracking_z_m = 0.;
  int normal_expected_bounces = 1;
  int serve_expected_bounces = 2;
  std::size_t max_rollout_bounces = 8;
  void validate() const;
};

struct StateBelief {
  std::int64_t source_time_origin_ns = 0;
  std::int64_t source_time_ns = 0, last_measurement_time_ns = 0;
  std::int64_t receipt_steady_ns = 0, receipt_wall_ns = 0;
  std::string frame_id;
  std::size_t numerical_cross_repairs = 0;
  bool initialized = false;
  State6 mean = State6::Zero();
  Matrix6 covariance = Matrix6::Identity();
  double source_time_s = 0.;
  double first_measurement_time_s = 0.;
  double last_measurement_time_s = 0.;
  double last_measurement_gap_s = 0.;
  std::size_t observation_count = 0;
  std::size_t rejected_measurements = 0;
  double innovation_squared = 0.;
  double innovation_rms_m = 0.;
  double innovation_max_m = 0.;
};
struct TrackerUpdate {
  bool changed = false;
  bool accepted = false;
  bool new_track = false;
  bool terminated = false;
  bool measurement_rejected = false;
  std::string reason = "none";
};
struct FlightState {
  std::uint64_t rally_id = 0, trajectory_epoch = 0, revision_id = 0;
  RallyContext rally_context = RallyContext::kUnknown;
  BallMotionState motion_state = BallMotionState::kUncertain;
  PlannerBallState planner_state = PlannerBallState::kOutgoing;
  bool active_incoming = false;
  bool ready_stand_requested = true;
  int expected_bounces = -1;
  int observed_bounces = 0;
  int prior_bounce_hint = -1; // unknown, not zero, when acquisition is partial
  double incoming_motion_confidence = .5;
  double last_observed_bounce_time_s = -1.;
};
} // namespace hope_planner_cpp
