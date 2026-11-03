// Evaluation-only ball forecasts from existing progressive/fixed one-shot APIs.
// No inverse planner, racket normal/velocity, command packing or dispatch.
#include "hope_planner_cpp/batch_physics_estimator.hpp"
#include "hope_planner_cpp/flight_replay_input.hpp"
#include "hope_planner_cpp/flight_state_v2.hpp"
#include "hope_planner_cpp/future_contact_predictor.hpp"
#include "hope_planner_cpp/incoming_trajectory.hpp"
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
using namespace hope_planner_cpp;
int main(int argc, char **argv) {
  try {
    if (argc != 3 || std::string(argv[1]) != "--method" ||
        (std::string(argv[2]) != "one-shot" &&
         std::string(argv[2]) != "progressive"))
      throw std::invalid_argument("Usage: --method one-shot|progressive < "
                                  "CSV(source_ns,x,y,z,omega_valid,wx,wy,wz)");
    const bool one_shot = std::string(argv[2]) == "one-shot";
    BallPhysics physics;
    EstimatorConfig estimator;
    estimator.spin_enabled = false;
    FutureContactConfig future;
    future.spin_enabled = false;
    future.magnus_enabled = false;
    PlannerConfig planner;
    planner.x_hit = .15;
    planner.adaptive_predict_horizon = true;
    planner.max_predict_time_cap_s = 3.;
    FutureContactPredictor predictor(physics, planner, {}, future);
    BatchPhysicsEstimator progressive(physics, estimator);
    IncomingTrajectoryConfig incoming;
    incoming.adaptive.enabled = false;
    IncomingTrajectory segmenter(incoming);
    std::optional<std::int64_t> origin, previous;
    std::string line;
    std::cout << std::setprecision(17);
    while (std::getline(std::cin, line)) {
      if (line.empty() || line[0] == '#')
        continue;
      BallSample s = parse_flight_observation_csv(line);
      if (previous && s.source_time_ns <= *previous)
        throw std::invalid_argument("source order");
      previous = s.source_time_ns;
      if (!origin)
        origin = s.source_time_ns;
      s.source_time_s = double(
          (static_cast<long double>(s.source_time_ns) - *origin) * 1e-9L);
      s.frame_id = "table";
      s.clock_domain = "recorded_registered";
      const auto started = std::chrono::steady_clock::now();
      BallState state;
      bool actual_trigger = !one_shot;
      if (one_shot) {
        const auto update = segmenter.observe(s);
        if (!update.snapshot_ready)
          continue; // Never invent intermediate one-shot predictions.
        actual_trigger = true;
        BatchPhysicsEstimator frozen(physics, estimator);
        for (std::size_t i = 0; i < update.snapshot.sample_count; ++i)
          frozen.push(update.snapshot.samples[i]);
        state = frozen.estimate();
      } else {
        progressive.push(s);
        state = progressive.estimate();
      }
      const auto target =
          predictor.predict(state, planner.x_hit, state.physics_spin);
      std::int64_t arrival = 0;
      const long double dt =
          (target.strike_source_time_s - state.source_time_s) * 1e9L;
      bool valid_time = target.valid && std::isfinite(dt) && dt >= 0 &&
                        dt < std::numeric_limits<std::int64_t>::max();
      if (valid_time)
        valid_time =
            checked_arrival(s.source_time_ns, std::llround(dt), arrival);
      const double compute_ms = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - started)
                                    .count();
      std::cout
          << "{\"method\":\""
          << (one_shot ? "legacy_one_shot" : "progressive_batch")
          << "\",\"implementation_pin\":\"8d599e7:upstream_batch_"
             "incoming_and_fixed_trigger\",\"model_config_id\":\""
          << ("baseline-upstream-v1:sha256:" +
              sha256(
                  return_region_config_identity({}, {}, "table") +
                  ";8d599e7;window=.18;min_span=.08;min_samples=12;spin=false;"
                  "commit=.05;adaptive=false;plane=.15;horizon_cap=3;" +
                  std::string(one_shot ? "one-shot" : "progressive")))
          << "\",\"source_ns\":" << s.source_time_ns
          << ",\"actual_trigger\":" << (actual_trigger ? "true" : "false")
          << ",\"physical_available\":" << (state.valid ? "true" : "false")
          << ",\"target_valid\":"
          << (target.valid && valid_time ? "true" : "false") << ",\"reason\":\""
          << target.reason << "\",\"arrival_ns\":" << arrival
          << ",\"compute_ms\":" << compute_ms;
      auto vector = [&](const char *name, const Vec3 &v) {
        std::cout << ",\"" << name << "\":[" << v.x() << ',' << v.y() << ','
                  << v.z() << ']';
      };
      vector("position", target.ball_position);
      vector("velocity", target.ball_velocity);
      // Legacy Stage 2 exposes a receiving-plane target, not a full event path.
      // Do not invent bounce/apex events with V2's different physical kernel.
      std::cout << ",\"spin_mode\":\"LEGACY_ZERO_PRIOR_CONTACT_NO_MAGNUS\",\"supported_events\":[\"receiving_plane\"],\"future_events\":[";
      std::cout << "]}\n";
    }
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
