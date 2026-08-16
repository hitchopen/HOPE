#include "hope_planner_cpp/batch_physics_estimator.hpp"
#include "hope_planner_cpp/incoming_trajectory.hpp"
#include "hope_planner_cpp/racket_target_planner.hpp"
#include "hope_planner_cpp/spin_estimator.hpp"
#include "hope_planner_cpp/trajectory_predictor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

using hope_planner_cpp::BallPhysics;
using hope_planner_cpp::BallSample;
using hope_planner_cpp::BatchPhysicsEstimator;
using hope_planner_cpp::EstimatorConfig;
using hope_planner_cpp::IncomingTrajectory;
using hope_planner_cpp::IncomingTrajectoryConfig;
using hope_planner_cpp::PlannerConfig;
using hope_planner_cpp::PostNetOneShotEvent;
using hope_planner_cpp::RacketCommand;
using hope_planner_cpp::RacketTargetPlanner;
using hope_planner_cpp::SpinEstimate;
using hope_planner_cpp::SpinEstimator;
using hope_planner_cpp::SpinEstimatorConfig;
using hope_planner_cpp::SpinPhysicsMode;
using hope_planner_cpp::StrikeTarget;
using hope_planner_cpp::TableParams;
using hope_planner_cpp::TrajectoryPredictor;
using hope_planner_cpp::Vec3;

struct Arguments {
  std::string input;
  std::string output;
  double x_hit = 0.15;
  double x_hit_bh_delta = 0.0;
  double solve_period_s = 0.033;
  bool post_net_one_shot = false;
  double post_net_delay_s = 0.05;
  double post_net_future_bounce_tangential_gain = 0.369;
  double net_x = 1.37;
  double incoming_opponent_side_margin_m = 0.05;
  double incoming_speed_threshold_mps = 0.25;
  double outgoing_speed_threshold_mps = 0.25;
  double incoming_source_gap_reset_s = 0.25;
  int incoming_direction_fit_samples = 4;
  int incoming_direction_confirmations = 2;
  int incoming_pre_roll_samples = 24;
  double estimator_window_s = 0.18;
  double estimator_min_span_s = 0.08;
  int estimator_min_samples = 12;
  double estimator_huber_delta_m = 0.003;
  double estimator_recency_half_life_s = 0.0;
  int estimator_iterations = 3;
  bool adaptive_horizon = false;
  double max_predict_time_cap_s = 3.0;
  double drag_k = 0.1261;
  double restitution_h = 0.64;
  double restitution_v = 0.9215;
  double bounce_min_reversal_m = 0.00005;
  double bounce_min_excursion_m = 0.001;
  int bounce_confirmation_samples = 5;
  double bounce_confirmation_max_span_s = 0.05;
  double bounce_sparse_confirmation_min_span_s = 0.012;
  double bounce_sparse_confirmation_excursion_m = 0.005;
  double bounce_refractory_s = 0.12;
  std::string spin_mode = "legacy";
  bool control_zero_spin = false;
  double spin_window_s = 0.10;
  double spin_min_span_s = 0.05;
  double spin_max_gap_s = 0.05;
  double spin_max_rev_s = 20.0;
  double spin_huber_delta_rev_s = 2.0;
  double magnus_k = 0.00444;
  double nakashima_friction_mu = 0.25;
  double table_tangential_gain = 0.369;
  double table_friction_cap_mu = 2.0;
};

SpinPhysicsMode parse_spin_mode(const std::string& value) {
  if (value == "legacy") return SpinPhysicsMode::kLegacyNoSpin;
  if (value == "nakashima") return SpinPhysicsMode::kNakashimaBounce;
  if (value == "nakashima-magnus") {
    return SpinPhysicsMode::kNakashimaBounceAndMagnus;
  }
  if (value == "venue-grip") return SpinPhysicsMode::kVenueGripBounce;
  if (value == "venue-grip-magnus") {
    return SpinPhysicsMode::kVenueGripBounceAndMagnus;
  }
  throw std::invalid_argument(
      "--spin-mode must be legacy, nakashima, nakashima-magnus, "
      "venue-grip, or venue-grip-magnus");
}

std::vector<std::string> parse_csv_line(const std::string& line) {
  std::vector<std::string> fields;
  std::string field;
  bool quoted = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char character = line[i];
    if (quoted) {
      if (character == '"') {
        if (i + 1 < line.size() && line[i + 1] == '"') {
          field.push_back('"');
          ++i;
        } else {
          quoted = false;
        }
      } else {
        field.push_back(character);
      }
    } else if (character == '"') {
      quoted = true;
    } else if (character == ',') {
      fields.push_back(std::move(field));
      field.clear();
    } else {
      field.push_back(character);
    }
  }
  fields.push_back(std::move(field));
  return fields;
}

double number(const std::vector<std::string>& row, std::size_t index) {
  if (index >= row.size()) return std::numeric_limits<double>::quiet_NaN();
  try {
    return std::stod(row[index]);
  } catch (...) {
    return std::numeric_limits<double>::quiet_NaN();
  }
}

std::int64_t integer(const std::vector<std::string>& row, std::size_t index) {
  if (index >= row.size()) return 0;
  try {
    return std::stoll(row[index]);
  } catch (...) {
    return 0;
  }
}

double quantile(std::vector<double> values, double q) {
  values.erase(
      std::remove_if(values.begin(), values.end(), [](double value) {
        return !std::isfinite(value);
      }),
      values.end());
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
  std::sort(values.begin(), values.end());
  const double position = q * static_cast<double>(values.size() - 1);
  const auto low = static_cast<std::size_t>(std::floor(position));
  const auto high = static_cast<std::size_t>(std::ceil(position));
  const double fraction = position - static_cast<double>(low);
  return values[low] * (1.0 - fraction) + values[high] * fraction;
}

std::string json_number(double value) {
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream stream;
  stream << std::setprecision(17) << value;
  return stream.str();
}

Arguments parse_arguments(int argc, char** argv) {
  Arguments args;
  for (int index = 1; index < argc; ++index) {
    const std::string option(argv[index]);
    auto next = [&]() -> std::string {
      if (++index >= argc) throw std::invalid_argument("missing value for " + option);
      return argv[index];
    };
    if (option == "--input" || option == "--mocap-raw-csv") {
      args.input = next();
    } else if (option == "--output" || option == "--output-csv") {
      args.output = next();
    } else if (option == "--x-hit") {
      args.x_hit = std::stod(next());
    } else if (option == "--x-hit-bh-delta") {
      args.x_hit_bh_delta = std::stod(next());
    } else if (option == "--solve-period") {
      args.solve_period_s = std::stod(next());
    } else if (option == "--post-net-one-shot") {
      args.post_net_one_shot = true;
    } else if (option == "--post-net-delay") {
      args.post_net_delay_s = std::stod(next());
    } else if (option == "--post-net-future-bounce-tangential-gain") {
      args.post_net_future_bounce_tangential_gain = std::stod(next());
    } else if (option == "--net-x") {
      args.net_x = std::stod(next());
    } else if (option == "--incoming-opponent-side-margin") {
      args.incoming_opponent_side_margin_m = std::stod(next());
    } else if (option == "--incoming-speed-threshold") {
      args.incoming_speed_threshold_mps = std::stod(next());
    } else if (option == "--outgoing-speed-threshold") {
      args.outgoing_speed_threshold_mps = std::stod(next());
    } else if (option == "--incoming-source-gap-reset") {
      args.incoming_source_gap_reset_s = std::stod(next());
    } else if (option == "--incoming-direction-fit-samples") {
      args.incoming_direction_fit_samples = std::stoi(next());
    } else if (option == "--incoming-direction-confirmations") {
      args.incoming_direction_confirmations = std::stoi(next());
    } else if (option == "--incoming-pre-roll-samples") {
      args.incoming_pre_roll_samples = std::stoi(next());
    } else if (option == "--window") {
      args.estimator_window_s = std::stod(next());
    } else if (option == "--min-span") {
      args.estimator_min_span_s = std::stod(next());
    } else if (option == "--min-samples") {
      args.estimator_min_samples = std::stoi(next());
    } else if (option == "--huber-delta") {
      args.estimator_huber_delta_m = std::stod(next());
    } else if (option == "--recency-half-life") {
      args.estimator_recency_half_life_s = std::stod(next());
    } else if (option == "--iterations") {
      args.estimator_iterations = std::stoi(next());
    } else if (option == "--adaptive-horizon") {
      args.adaptive_horizon = true;
    } else if (option == "--max-predict-time-cap") {
      args.max_predict_time_cap_s = std::stod(next());
    } else if (option == "--drag-k") {
      args.drag_k = std::stod(next());
    } else if (option == "--restitution-h") {
      args.restitution_h = std::stod(next());
    } else if (option == "--restitution-v") {
      args.restitution_v = std::stod(next());
    } else if (option == "--bounce-min-reversal") {
      args.bounce_min_reversal_m = std::stod(next());
    } else if (option == "--bounce-min-excursion") {
      args.bounce_min_excursion_m = std::stod(next());
    } else if (option == "--bounce-confirmation-samples") {
      args.bounce_confirmation_samples = std::stoi(next());
    } else if (option == "--bounce-confirmation-max-span") {
      args.bounce_confirmation_max_span_s = std::stod(next());
    } else if (option == "--bounce-sparse-confirmation-min-span") {
      args.bounce_sparse_confirmation_min_span_s = std::stod(next());
    } else if (option == "--bounce-sparse-confirmation-excursion") {
      args.bounce_sparse_confirmation_excursion_m = std::stod(next());
    } else if (option == "--bounce-refractory") {
      args.bounce_refractory_s = std::stod(next());
    } else if (option == "--spin-mode") {
      args.spin_mode = next();
      static_cast<void>(parse_spin_mode(args.spin_mode));
    } else if (option == "--control-zero-spin") {
      args.control_zero_spin = true;
    } else if (option == "--spin-window") {
      args.spin_window_s = std::stod(next());
    } else if (option == "--spin-min-span") {
      args.spin_min_span_s = std::stod(next());
    } else if (option == "--spin-max-gap") {
      args.spin_max_gap_s = std::stod(next());
    } else if (option == "--spin-max-rev") {
      args.spin_max_rev_s = std::stod(next());
    } else if (option == "--spin-huber-delta-rev") {
      args.spin_huber_delta_rev_s = std::stod(next());
    } else if (option == "--magnus-k") {
      args.magnus_k = std::stod(next());
    } else if (option == "--table-friction-mu" ||
               option == "--nakashima-friction-mu") {
      args.nakashima_friction_mu = std::stod(next());
    } else if (option == "--table-tangential-gain") {
      args.table_tangential_gain = std::stod(next());
    } else if (option == "--table-friction-cap-mu") {
      args.table_friction_cap_mu = std::stod(next());
    } else if (option == "--help" || option == "-h") {
      std::cout
          << "usage: hope_planner_cpp_replay --input mocap_raw.csv --output replay.csv "
             "[--x-hit 0.15] [--x-hit-bh-delta 0] [--solve-period 0.033] "
             "[--post-net-one-shot] [--post-net-delay 0.05] "
             "[--post-net-future-bounce-tangential-gain 0.369] [--net-x 1.37] "
             "[--incoming-opponent-side-margin 0.05] "
             "[--incoming-speed-threshold 0.25] "
             "[--outgoing-speed-threshold 0.25] "
             "[--incoming-direction-fit-samples 4] "
             "[--incoming-direction-confirmations 2] "
             "[--incoming-pre-roll-samples 24] "
             "[--incoming-source-gap-reset 0.25] "
             "[--window 0.18] [--min-span 0.08] [--min-samples 12] "
             "[--huber-delta 0.003] [--recency-half-life 0] [--iterations 3] "
             "[--adaptive-horizon] [--max-predict-time-cap 3.0] "
             "[--drag-k 0.1261] [--restitution-h 0.64] "
             "[--restitution-v 0.9215] "
             "[--bounce-min-reversal 0.00005] [--bounce-min-excursion 0.001] "
             "[--bounce-confirmation-samples 5] "
             "[--bounce-confirmation-max-span 0.05] "
             "[--bounce-sparse-confirmation-min-span 0.012] "
             "[--bounce-sparse-confirmation-excursion 0.005] "
             "[--bounce-refractory 0.12] "
             "[--spin-mode legacy|nakashima|nakashima-magnus|venue-grip|venue-grip-magnus] "
             "[--control-zero-spin] "
             "[--spin-window 0.10] [--spin-min-span 0.05] "
             "[--spin-max-gap 0.05] [--spin-max-rev 20] "
             "[--spin-huber-delta-rev 2] [--magnus-k 0.00444] "
             "[--nakashima-friction-mu 0.25] "
             "[--table-tangential-gain 0.369] "
             "[--table-friction-cap-mu 2.0]\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown option: " + option);
    }
  }
  if (args.input.empty() || args.output.empty()) {
    throw std::invalid_argument("--input and --output are required");
  }
  return args;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Arguments args = parse_arguments(argc, argv);
    std::ifstream input(args.input);
    if (!input) throw std::runtime_error("cannot open input: " + args.input);
    std::ofstream output(args.output, std::ios::out | std::ios::trunc);
    if (!output) throw std::runtime_error("cannot open output: " + args.output);

    std::string line;
    if (!std::getline(input, line)) throw std::runtime_error("input CSV is empty");
    const auto header = parse_csv_line(line);
    std::unordered_map<std::string, std::size_t> column;
    for (std::size_t i = 0; i < header.size(); ++i) column[header[i]] = i;
    for (const char* required : {
             "object_key", "pose_accepted", "ros_stamp_ns",
             "normalized_x", "normalized_y", "normalized_z"}) {
      if (column.find(required) == column.end()) {
        throw std::runtime_error(std::string("missing input column: ") + required);
      }
    }
    if (args.spin_mode != "legacy") {
      for (const char* required : {"qx", "qy", "qz", "qw"}) {
        if (column.find(required) == column.end()) {
          throw std::runtime_error(std::string("missing spin column: ") + required);
        }
      }
    }

    BallPhysics physics;
    physics.drag_k = args.drag_k;
    physics.magnus_k = args.magnus_k;
    physics.restitution_h = args.restitution_h;
    physics.restitution_v = args.restitution_v;
    physics.nakashima_friction_mu = args.nakashima_friction_mu;
    physics.table_tangential_gain = args.table_tangential_gain;
    physics.table_friction_cap_mu = args.table_friction_cap_mu;
    PlannerConfig planner_config;
    planner_config.x_hit = args.x_hit;
    planner_config.adaptive_predict_horizon = args.adaptive_horizon;
    planner_config.max_predict_time_cap_s = args.max_predict_time_cap_s;
    EstimatorConfig estimator_config;
    estimator_config.window_s = args.estimator_window_s;
    estimator_config.min_span_s = args.estimator_min_span_s;
    estimator_config.min_samples = static_cast<std::size_t>(
        std::max(6, args.estimator_min_samples));
    estimator_config.huber_delta_m = args.estimator_huber_delta_m;
    estimator_config.recency_half_life_s = args.estimator_recency_half_life_s;
    estimator_config.robust_iterations = args.estimator_iterations;
    estimator_config.bounce_min_reversal_m = args.bounce_min_reversal_m;
    estimator_config.bounce_min_excursion_m = args.bounce_min_excursion_m;
    estimator_config.bounce_confirmation_samples = static_cast<std::size_t>(
        std::max(1, args.bounce_confirmation_samples));
    estimator_config.bounce_confirmation_max_span_s =
        args.bounce_confirmation_max_span_s;
    estimator_config.bounce_sparse_confirmation_min_span_s =
        args.bounce_sparse_confirmation_min_span_s;
    estimator_config.bounce_sparse_confirmation_excursion_m =
        args.bounce_sparse_confirmation_excursion_m;
    estimator_config.bounce_refractory_s = args.bounce_refractory_s;
    SpinEstimatorConfig spin_config;
    spin_config.window_s = args.spin_window_s;
    spin_config.min_span_s = args.spin_min_span_s;
    spin_config.max_gap_s = args.spin_max_gap_s;
    spin_config.max_rev_s = args.spin_max_rev_s;
    spin_config.huber_delta_rev_s = args.spin_huber_delta_rev_s;
    TableParams table;
    table.net_x = args.net_x;
    BatchPhysicsEstimator estimator(physics, estimator_config);
    SpinEstimator spin_estimator(spin_config);
    TrajectoryPredictor predictor(physics, planner_config, table);
    BallPhysics post_net_physics = physics;
    post_net_physics.table_tangential_gain = std::max(
        0.0, args.post_net_future_bounce_tangential_gain);
    TrajectoryPredictor post_net_predictor(
        post_net_physics, planner_config, table);
    RacketTargetPlanner target_planner(physics, planner_config, table);
    IncomingTrajectoryConfig incoming_config;
    incoming_config.net_x = args.net_x;
    incoming_config.estimator_window_s = args.estimator_window_s;
    incoming_config.commit_delay_s = args.post_net_delay_s;
    incoming_config.opponent_side_margin_m =
        args.incoming_opponent_side_margin_m;
    incoming_config.incoming_speed_threshold_mps =
        args.incoming_speed_threshold_mps;
    incoming_config.outgoing_speed_threshold_mps =
        args.outgoing_speed_threshold_mps;
    incoming_config.source_gap_reset_s = args.incoming_source_gap_reset_s;
    incoming_config.direction_fit_samples = static_cast<std::size_t>(
        std::max(3, args.incoming_direction_fit_samples));
    incoming_config.direction_confirmations = static_cast<std::size_t>(
        std::max(1, args.incoming_direction_confirmations));
    incoming_config.pre_roll_samples = static_cast<std::size_t>(
        std::max(6, args.incoming_pre_roll_samples));
    IncomingTrajectory incoming_trajectory(incoming_config);

    output
        << "kind,sample_seq,source_time_s,reason,valid,ball_x,ball_y,ball_z,"
           "est_x,est_y,est_z,est_vx,est_vy,est_vz,estimator_samples,estimator_span_s,"
           "fit_rms_m,fit_max_m,strike_time_s,strike_x,strike_y,strike_z,"
           "strike_vx,strike_vy,strike_vz,racket_vx,racket_vy,racket_vz,swing_sign,"
           "estimator_ms,stage2_ms,stage3_ms,total_ms,actual_crossing_y,"
           "actual_crossing_z,source_gap_s,racket_nx,racket_ny,racket_nz,"
           "outgoing_vx,outgoing_vy,outgoing_vz,clears_net,bypasses_net_posts,"
           "target_land_x,target_land_y,target_land_z,estimate_valid,spin_mode,"
           "spin_valid,spin_reason,spin_wx_rad_s,spin_wy_rad_s,spin_wz_rad_s,"
           "spin_magnitude_rev_s,spin_coherence,spin_retained_time_fraction,"
           "spin_retained_increments,spin_rejected_increments,"
           "bounce_transition_used,bounce_source_time_s,pre_bounce_samples,"
           "post_bounce_samples,bounce_epoch_active,one_shot_enabled,"
           "one_shot_flight_seq,net_cross_source_time_s,commit_source_time_s,"
           "post_net_delay_s,post_net_future_bounce_tangential_gain,"
           "trajectory_epoch,snapshot_sequence,segment_boundary_reason,"
           "segment_start_source_time_s,previous_segment_last_source_time_s,"
           "previous_tail_to_commit_ms\n";
    output << std::setprecision(17);

    std::uint64_t raw_rows = 0;
    std::uint64_t accepted_rows = 0;
    std::uint64_t solve_rows = 0;
    std::uint64_t valid_rows = 0;
    std::uint64_t crossing_rows = 0;
    std::uint64_t spin_valid_solve_rows = 0;
    std::uint64_t bounce_transition_solve_rows = 0;
    std::uint64_t bounce_transition_valid_rows = 0;
    std::uint64_t one_shot_net_crossings = 0;
    std::uint64_t one_shot_commits = 0;
    std::uint64_t trajectory_source_resets = 0;
    double last_solve_time = -std::numeric_limits<double>::infinity();
    double last_swing_sign = 0.0;
    bool have_previous = false;
    double previous_time = 0.0;
    Vec3 previous_position = Vec3::Zero();
    std::vector<double> estimator_times;
    std::vector<double> stage2_times;
    std::vector<double> stage3_times;
    std::vector<double> total_times;
    std::vector<double> valid_estimator_times;
    std::vector<double> valid_stage2_times;
    std::vector<double> valid_stage3_times;
    std::vector<double> valid_total_times;
    std::map<std::string, std::uint64_t> reasons;
    std::map<std::string, std::uint64_t> spin_reasons;
    const SpinPhysicsMode spin_mode = parse_spin_mode(args.spin_mode);

    while (std::getline(input, line)) {
      const auto row = parse_csv_line(line);
      const auto& object_key = row[column.at("object_key")];
      if (object_key != "ball" && object_key != "Ball") continue;
      ++raw_rows;
      if (row[column.at("pose_accepted")] != "1") continue;
      const auto stamp_ns = integer(row, column.at("ros_stamp_ns"));
      const double source_time = static_cast<double>(stamp_ns) * 1.0e-9;
      const Vec3 position(
          number(row, column.at("normalized_x")),
          number(row, column.at("normalized_y")),
          number(row, column.at("normalized_z")));
      if (stamp_ns <= 0 || !position.allFinite()) continue;
      ++accepted_rows;

      if (have_previous) {
        const double gap = source_time - previous_time;
        if (gap > 0.0 && gap <= 0.05 &&
            previous_position.x() > args.x_hit && position.x() <= args.x_hit) {
          const double fraction = std::clamp(
              (previous_position.x() - args.x_hit) /
                  (previous_position.x() - position.x()),
              0.0, 1.0);
          const double crossing_time = previous_time + fraction * gap;
          const Vec3 crossing_position =
              previous_position + fraction * (position - previous_position);
          output << "crossing,0," << crossing_time
                 << ",measured_crossing,1,"
                 << args.x_hit << ',' << crossing_position.y() << ','
                 << crossing_position.z()
                 << ",nan,nan,nan,nan,nan,nan,0,0,nan,nan,"
                 << crossing_time << ',' << args.x_hit << ','
                 << crossing_position.y() << ',' << crossing_position.z()
                 << ",nan,nan,nan,nan,nan,nan,0,nan,nan,nan,nan,"
                 << crossing_position.y() << ',' << crossing_position.z() << ','
                 << gap
                 << ",nan,nan,nan,nan,nan,nan,0,0,nan,nan,nan,0,"
                 << args.spin_mode
                 << ",0,measured_crossing,nan,nan,nan,nan,nan,nan,0,0,"
                 << "0,nan,0,0,0," << (args.post_net_one_shot ? 1 : 0)
                 << ",0,nan,nan," << args.post_net_delay_s << ','
                 << args.post_net_future_bounce_tangential_gain
                 << ",0,0,measured_crossing,nan,nan,nan\n";
          ++crossing_rows;
        }
      }
      have_previous = true;
      previous_time = source_time;
      previous_position = position;

      BallSample sample;
      sample.source_time_s = source_time;
      sample.position = position;
      sample.sequence = accepted_rows;
      const bool have_quaternion_columns =
          column.find("qx") != column.end() &&
          column.find("qy") != column.end() &&
          column.find("qz") != column.end() &&
          column.find("qw") != column.end();
      bool orientation_declared_valid = true;
      const auto status_column = column.find("orientation_status");
      if (status_column != column.end()) {
        orientation_declared_valid = row[status_column->second] == "valid";
      }
      const auto sanitized_column = column.find("orientation_sanitized");
      if (sanitized_column != column.end() &&
          row[sanitized_column->second] == "1") {
        orientation_declared_valid = false;
      }
      if (have_quaternion_columns && orientation_declared_valid) {
        sample.orientation = Eigen::Quaterniond(
            number(row, column.at("qw")), number(row, column.at("qx")),
            number(row, column.at("qy")), number(row, column.at("qz")));
        sample.orientation_valid = sample.orientation.coeffs().allFinite() &&
                                   sample.orientation.norm() > 1.0e-9;
      }
      PostNetOneShotEvent one_shot_event;
      std::uint64_t trajectory_epoch = 0;
      std::uint64_t snapshot_sequence = 0;
      double segment_start_source_time_s =
          std::numeric_limits<double>::quiet_NaN();
      double previous_segment_last_source_time_s =
          std::numeric_limits<double>::quiet_NaN();
      std::string segment_boundary_reason = "none";
      std::uint64_t solve_sample_sequence = accepted_rows;
      double solve_source_time = source_time;
      Vec3 solve_position = position;
      if (args.post_net_one_shot) {
        const auto update = incoming_trajectory.observe(sample);
        if (update.source_epoch_reset) ++trajectory_source_resets;
        if (update.net_crossed) {
          ++one_shot_net_crossings;
        }
        if (!update.snapshot_ready || update.snapshot.sample_count == 0 ||
            update.snapshot.latest_sample() == nullptr) {
          continue;
        }
        ++one_shot_commits;
        estimator.reset();
        spin_estimator.reset();
        for (std::size_t i = 0; i < update.snapshot.sample_count; ++i) {
          estimator.push(update.snapshot.samples[i]);
          spin_estimator.push(update.snapshot.samples[i]);
        }
        const BallSample& solve_sample = *update.snapshot.latest_sample();
        solve_sample_sequence = solve_sample.sequence;
        solve_source_time = solve_sample.source_time_s;
        solve_position = solve_sample.position;
        one_shot_event = update.snapshot.one_shot;
        trajectory_epoch = update.snapshot.trajectory_epoch;
        snapshot_sequence = update.snapshot.snapshot_sequence;
        segment_start_source_time_s =
            update.snapshot.segment_start_source_time_s;
        previous_segment_last_source_time_s =
            update.snapshot.previous_segment_last_source_time_s;
        segment_boundary_reason = update.snapshot.segment_boundary_reason;
      } else {
        estimator.push(sample);
        spin_estimator.push(sample);
        if (source_time - last_solve_time + 1.0e-12 < args.solve_period_s) continue;
        last_solve_time = source_time;
      }
      ++solve_rows;

      const auto total_start = std::chrono::steady_clock::now();
      const auto estimator_start = std::chrono::steady_clock::now();
      const auto state = estimator.estimate();
      if (state.bounce_transition_used) {
        ++bounce_transition_solve_rows;
        if (state.valid) ++bounce_transition_valid_rows;
      }
      const SpinEstimate spin = spin_estimator.estimate();
      const auto estimator_end = std::chrono::steady_clock::now();
      ++spin_reasons[spin.reason];
      if (spin.valid) ++spin_valid_solve_rows;
      StrikeTarget strike;
      strike.reason = state.reason;
      const auto stage2_start = std::chrono::steady_clock::now();
      if (state.valid) {
        // Match the hardware node exactly when requested: the venue contact
        // law remains active, while orientation-derived omega is audit only.
        const Vec3 control_omega = args.control_zero_spin
            ? Vec3::Zero()
            : (spin.valid ? spin.omega_rad_s : Vec3::Zero());
        TrajectoryPredictor& control_predictor =
            args.post_net_one_shot ? post_net_predictor : predictor;
        strike = control_predictor.predict_with_spin(
            state, args.x_hit, control_omega, spin_mode);
        if (strike.valid) {
          const double relative_y = strike.ball_position.y();
          if (last_swing_sign > 0.5) {
            if (relative_y > -0.21) last_swing_sign = -1.0;
          } else if (last_swing_sign < -0.5) {
            if (relative_y < -0.29) last_swing_sign = 1.0;
          } else {
            last_swing_sign = relative_y < -0.25 ? 1.0 : -1.0;
          }
          if (last_swing_sign < 0.0 && args.x_hit_bh_delta != 0.0) {
            strike = control_predictor.predict_with_spin(
                state, args.x_hit + args.x_hit_bh_delta, control_omega,
                spin_mode);
          }
        }
      }
      const auto stage2_end = std::chrono::steady_clock::now();
      const auto stage3_start = std::chrono::steady_clock::now();
      const RacketCommand command = target_planner.plan(
          strike, planner_config.target_land, planner_config.delta_t_flight);
      const auto stage3_end = std::chrono::steady_clock::now();
      const auto total_end = stage3_end;
      const auto milliseconds = [](auto start, auto end) {
        return std::chrono::duration<double, std::milli>(end - start).count();
      };
      const double estimator_ms = milliseconds(estimator_start, estimator_end);
      const double stage2_ms = milliseconds(stage2_start, stage2_end);
      const double stage3_ms = milliseconds(stage3_start, stage3_end);
      const double total_ms = milliseconds(total_start, total_end);
      estimator_times.push_back(estimator_ms);
      stage2_times.push_back(stage2_ms);
      stage3_times.push_back(stage3_ms);
      total_times.push_back(total_ms);
      const std::string reason = command.valid ? "command_valid" :
          (strike.valid ? command.reason : strike.reason);
      ++reasons[reason];
      if (command.valid) {
        ++valid_rows;
        valid_estimator_times.push_back(estimator_ms);
        valid_stage2_times.push_back(stage2_ms);
        valid_stage3_times.push_back(stage3_ms);
        valid_total_times.push_back(total_ms);
      }

      output << "solve," << solve_sample_sequence << ',' << solve_source_time << ','
             << reason << ','
             << (command.valid ? 1 : 0) << ','
             << solve_position.x() << ',' << solve_position.y() << ','
             << solve_position.z() << ','
             << state.position.x() << ',' << state.position.y() << ',' << state.position.z() << ','
             << state.velocity.x() << ',' << state.velocity.y() << ',' << state.velocity.z() << ','
             << state.sample_count << ',' << state.sample_span_s << ','
             << state.residual_rms_m << ',' << state.residual_max_m << ','
             << strike.strike_source_time_s << ',' << strike.ball_position.x() << ','
             << strike.ball_position.y() << ',' << strike.ball_position.z() << ','
             << strike.ball_velocity.x() << ',' << strike.ball_velocity.y() << ','
             << strike.ball_velocity.z() << ',' << command.velocity.x() << ','
             << command.velocity.y() << ',' << command.velocity.z() << ','
             << last_swing_sign << ',' << estimator_ms << ',' << stage2_ms << ','
             << stage3_ms << ',' << total_ms << ",nan,nan,nan,"
             << command.normal.x() << ',' << command.normal.y() << ','
             << command.normal.z() << ',' << command.outgoing_ball_velocity.x() << ','
             << command.outgoing_ball_velocity.y() << ','
             << command.outgoing_ball_velocity.z() << ','
             << (command.clears_net ? 1 : 0) << ','
             << (command.bypasses_net_posts ? 1 : 0) << ','
             << command.target_land.x() << ',' << command.target_land.y() << ','
             << command.target_land.z() << ',' << (state.valid ? 1 : 0) << ','
             << args.spin_mode << ',' << (spin.valid ? 1 : 0) << ','
             << spin.reason << ',' << spin.omega_rad_s.x() << ','
             << spin.omega_rad_s.y() << ',' << spin.omega_rad_s.z() << ','
             << spin.omega_rad_s.norm() / (2.0 * 3.14159265358979323846) << ','
             << spin.coherence << ',' << spin.retained_time_fraction << ','
             << spin.retained_increments << ',' << spin.rejected_increments << ','
             << (state.bounce_transition_used ? 1 : 0) << ','
             << state.bounce_source_time_s << ',' << state.pre_bounce_samples << ','
             << state.post_bounce_samples << ','
             << (state.bounce_epoch_active ? 1 : 0) << ','
             << (args.post_net_one_shot ? 1 : 0) << ','
             << one_shot_event.flight_sequence << ','
             << one_shot_event.net_cross_source_time_s << ','
             << one_shot_event.commit_source_time_s << ','
             << args.post_net_delay_s << ','
             << args.post_net_future_bounce_tangential_gain << ','
             << trajectory_epoch << ',' << snapshot_sequence << ','
             << segment_boundary_reason << ',' << segment_start_source_time_s << ','
             << previous_segment_last_source_time_s << ','
             << (std::isfinite(previous_segment_last_source_time_s) &&
                         std::isfinite(one_shot_event.commit_source_time_s)
                     ? (one_shot_event.commit_source_time_s -
                        previous_segment_last_source_time_s) * 1.0e3
                     : std::numeric_limits<double>::quiet_NaN())
             << '\n';
    }

    output.flush();
    std::cout << std::setprecision(6)
              << "{\n"
              << "  \"input\": \"" << args.input << "\",\n"
              << "  \"output\": \"" << args.output << "\",\n"
              << "  \"adaptive_horizon\": "
              << (args.adaptive_horizon ? "true" : "false") << ",\n"
              << "  \"post_net_one_shot\": "
              << (args.post_net_one_shot ? "true" : "false") << ",\n"
              << "  \"post_net_delay_s\": " << args.post_net_delay_s << ",\n"
              << "  \"post_net_future_bounce_tangential_gain\": "
              << args.post_net_future_bounce_tangential_gain << ",\n"
              << "  \"net_x\": " << args.net_x << ",\n"
              << "  \"incoming_opponent_side_margin_m\": "
              << args.incoming_opponent_side_margin_m << ",\n"
              << "  \"incoming_speed_threshold_mps\": "
              << args.incoming_speed_threshold_mps << ",\n"
              << "  \"outgoing_speed_threshold_mps\": "
              << args.outgoing_speed_threshold_mps << ",\n"
              << "  \"incoming_direction_fit_samples\": "
              << args.incoming_direction_fit_samples << ",\n"
              << "  \"incoming_direction_confirmations\": "
              << args.incoming_direction_confirmations << ",\n"
              << "  \"incoming_pre_roll_samples\": "
              << args.incoming_pre_roll_samples << ",\n"
              << "  \"incoming_source_gap_reset_s\": "
              << args.incoming_source_gap_reset_s << ",\n"
              << "  \"drag_k\": " << args.drag_k << ",\n"
              << "  \"restitution_h\": " << args.restitution_h << ",\n"
              << "  \"restitution_v\": " << args.restitution_v << ",\n"
              << "  \"bounce_min_reversal_m\": "
              << args.bounce_min_reversal_m << ",\n"
              << "  \"bounce_min_excursion_m\": "
              << args.bounce_min_excursion_m << ",\n"
              << "  \"bounce_confirmation_samples\": "
              << args.bounce_confirmation_samples << ",\n"
              << "  \"bounce_confirmation_max_span_s\": "
              << args.bounce_confirmation_max_span_s << ",\n"
              << "  \"bounce_sparse_confirmation_min_span_s\": "
              << args.bounce_sparse_confirmation_min_span_s << ",\n"
              << "  \"bounce_sparse_confirmation_excursion_m\": "
              << args.bounce_sparse_confirmation_excursion_m << ",\n"
              << "  \"bounce_refractory_s\": "
              << args.bounce_refractory_s << ",\n"
              << "  \"spin_mode\": \"" << args.spin_mode << "\",\n"
              << "  \"control_zero_spin\": "
              << (args.control_zero_spin ? "true" : "false") << ",\n"
              << "  \"spin_window_s\": " << args.spin_window_s << ",\n"
              << "  \"spin_max_rev_s\": " << args.spin_max_rev_s << ",\n"
              << "  \"magnus_k\": " << args.magnus_k << ",\n"
              << "  \"nakashima_friction_mu\": "
              << args.nakashima_friction_mu << ",\n"
              << "  \"table_tangential_gain\": "
              << args.table_tangential_gain << ",\n"
              << "  \"table_friction_cap_mu\": "
              << args.table_friction_cap_mu << ",\n"
              << "  \"raw_ball_rows\": " << raw_rows << ",\n"
              << "  \"estimator_kind\": \"batch_physics_cpp_no_ekf_persistent_bounce\",\n"
              << "  \"accepted_ball_rows\": " << accepted_rows << ",\n"
              << "  \"raw_file_acceptance\": "
              << (raw_rows ? static_cast<double>(accepted_rows) / raw_rows : 0.0) << ",\n"
              << "  \"solve_rows\": " << solve_rows << ",\n"
              << "  \"valid_rows\": " << valid_rows << ",\n"
              << "  \"spin_valid_solve_rows\": " << spin_valid_solve_rows << ",\n"
              << "  \"bounce_transition_solve_rows\": "
              << bounce_transition_solve_rows << ",\n"
              << "  \"bounce_transition_valid_rows\": "
              << bounce_transition_valid_rows << ",\n"
              << "  \"one_shot_net_crossings\": "
              << one_shot_net_crossings << ",\n"
              << "  \"one_shot_commits\": " << one_shot_commits << ",\n"
              << "  \"trajectory_epochs\": "
              << incoming_trajectory.trajectory_epoch() << ",\n"
              << "  \"trajectory_source_resets\": "
              << trajectory_source_resets << ",\n"
              << "  \"measured_crossings\": " << crossing_rows << ",\n"
              << "  \"estimator_p50_ms\": " << json_number(quantile(estimator_times, 0.50)) << ",\n"
              << "  \"estimator_p95_ms\": " << json_number(quantile(estimator_times, 0.95)) << ",\n"
              << "  \"stage2_p50_ms\": " << json_number(quantile(stage2_times, 0.50)) << ",\n"
              << "  \"stage2_p95_ms\": " << json_number(quantile(stage2_times, 0.95)) << ",\n"
              << "  \"stage3_p50_ms\": " << json_number(quantile(stage3_times, 0.50)) << ",\n"
              << "  \"stage3_p95_ms\": " << json_number(quantile(stage3_times, 0.95)) << ",\n"
              << "  \"total_p50_ms\": " << json_number(quantile(total_times, 0.50)) << ",\n"
              << "  \"total_p95_ms\": " << json_number(quantile(total_times, 0.95)) << ",\n"
              << "  \"valid_estimator_p50_ms\": "
              << json_number(quantile(valid_estimator_times, 0.50)) << ",\n"
              << "  \"valid_estimator_p95_ms\": "
              << json_number(quantile(valid_estimator_times, 0.95)) << ",\n"
              << "  \"valid_stage2_p50_ms\": "
              << json_number(quantile(valid_stage2_times, 0.50)) << ",\n"
              << "  \"valid_stage2_p95_ms\": "
              << json_number(quantile(valid_stage2_times, 0.95)) << ",\n"
              << "  \"valid_stage3_p50_ms\": "
              << json_number(quantile(valid_stage3_times, 0.50)) << ",\n"
              << "  \"valid_stage3_p95_ms\": "
              << json_number(quantile(valid_stage3_times, 0.95)) << ",\n"
              << "  \"valid_total_p50_ms\": "
              << json_number(quantile(valid_total_times, 0.50)) << ",\n"
              << "  \"valid_total_p95_ms\": "
              << json_number(quantile(valid_total_times, 0.95)) << ",\n"
              << "  \"reasons\": {";
    bool first = true;
    for (const auto& item : reasons) {
      if (!first) std::cout << ',';
      first = false;
      std::cout << "\n    \"" << item.first << "\": " << item.second;
    }
    if (!first) std::cout << '\n';
    std::cout << "  },\n  \"spin_reasons\": {";
    first = true;
    for (const auto& item : spin_reasons) {
      if (!first) std::cout << ',';
      first = false;
      std::cout << "\n    \"" << item.first << "\": " << item.second;
    }
    if (!first) std::cout << '\n';
    std::cout << "  }\n}\n";
    return 0;
  } catch (const std::exception& exception) {
    std::cerr << "ERROR: " << exception.what() << '\n';
    return 2;
  }
}
