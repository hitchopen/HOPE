#include "hope_planner_cpp/flight_replay_input.hpp"
#include "hope_planner_cpp/flight_state_v2.hpp"
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
using namespace hope_planner_cpp;
namespace {
void vector(const Vec3 &v) {
  std::cout << '[' << v.x() << ',' << v.y() << ',' << v.z() << ']';
}
void write_spin(const EffectiveSpin &spin) {
  const char *mode = spin.mode == EffectiveSpinMode::MEASURED ? "MEASURED"
                     : spin.mode == EffectiveSpinMode::HELD   ? "HELD"
                     : spin.mode == EffectiveSpinMode::CONTACT_PROPAGATED
                         ? "CONTACT_PROPAGATED"
                         : "POSITION_ONLY";
  std::cout << ",\"spin_mode\":\"" << mode
            << "\",\"spin_measurement_time_ns\":" << spin.measurement_time_ns
            << ",\"omega_rad_s\":";
  if (spin.usable())
    vector(*spin.omega);
  else
    std::cout << "null";
}
} // namespace
int main(int argc, char **argv) {
  try {
    std::cout << std::setprecision(17);
    ProbabilisticConfig config;
    ShotSettings settings;
    std::vector<std::string> args;
    for (int i = 1; i < argc; ++i) {
      if (std::string(argv[i]) == "--set") {
        if (++i >= argc)
          throw std::invalid_argument("--set requires key=value");
        const std::string parameter = argv[i];
        const auto equal = parameter.find('=');
        if (equal == std::string::npos)
          throw std::invalid_argument("--set requires key=value");
        std::size_t used = 0;
        const double value = std::stod(parameter.substr(equal + 1), &used);
        if (used != parameter.size() - equal - 1)
          throw std::invalid_argument("invalid parameter value");
        set_flight_parameter(config, settings, parameter.substr(0, equal),
                             value);
      } else
        args.emplace_back(argv[i]);
    }
    if (args.size() == 1 && args[0] == "--profile") {
      std::cout << "{\"physics_profile\":\"" << kFlightPhysicsProfile
                << "\",\"model_config_id\":\""
                << flight_model_config_id(config, settings) << "\"}\n";
      return 0;
    }
    if (args.size() == 6 && args[0] == "--rollout") {
      const double h = std::stod(args[1]);
      const int direction = std::stoi(args[2]);
      const double speed = std::stod(args[3]), spin = std::stod(args[4]),
                   duration = std::stod(args[5]);
      if (!std::isfinite(h) || h <= 0 || h > .01 ||
          (direction != 1 && direction != -1) || !std::isfinite(duration) ||
          duration <= 0 || duration > 10)
        return 2;
      ModelState model;
      model.position = Vec3(direction > 0 ? .15 : 2.59, -.7, .35);
      model.velocity = Vec3(direction * speed, .15, -.7);
      if (spin != 0) {
        model.spin.omega = Vec3(5, spin, 8);
        model.spin.mode = EffectiveSpinMode::MEASURED;
      }
      config.integration_dt_s = h;
      const auto physics = config.physics;
      EventGeometry g;
      g.receiving_x = .15;
      std::cout << "{\"profile\":\"" << kFlightPhysicsProfile
                << "\",\"model_config_id\":\""
                << flight_model_config_id(config, settings)
                << "\",\"events\":[";
      bool comma = false;
      for (double t = 0; t < duration - 1e-12;) {
        double step_size = std::min(h, duration - t);
        auto step = advance_ball(model, step_size, physics, g);
        for (const auto &e : step.events) {
          if (comma)
            std::cout << ',';
          comma = true;
          std::cout << "{\"kind\":\"" << name(e.kind)
                    << "\",\"time\":" << t + e.dt_s << ",\"position\":";
          vector(e.pre.position);
          write_spin(e.pre.spin);
          std::cout << ",\"velocity\":";
          vector(e.pre.velocity);
          std::cout << ",\"spin_available\":"
                    << (e.pre.spin.usable() ? "true" : "false") << '}';
        }
        if (step.stopped)
          break;
        t += step_size;
      }
      std::cout << "],\"position\":";
      vector(model.position);
      write_spin(model.spin);
      std::cout << ",\"velocity\":";
      vector(model.velocity);
      std::cout << "}\n";
      return 0;
    }
    if ((args.size() != 1 && args.size() != 3) || args[0] != "--observations") {
      std::cerr
          << "Usage: --rollout step_s direction speed spin_y duration_s | "
             "--observations [direction complete|unknown] < "
             "CSV(source_ns,x,y,z,omega_valid,wx,wy,wz); --profile; optional "
             "--set key=value\n";
      return 2;
    }
    FlightPipeline pipeline(config, settings);
    HitPlaneSnapshot plane;
    plane.calibrated = true;
    plane.x_hit_m = config.x_hit;
    plane.frame_id = "table";
    std::string line;
    bool first = true;
    std::optional<std::int64_t> previous;
    while (std::getline(std::cin, line)) {
      if (line.empty() || line[0] == '#')
        continue;
      BallSample s = parse_flight_observation_csv(line);
      if (previous && s.source_time_ns <= *previous)
        throw std::runtime_error("duplicate/out-of-order CSV source stamp");
      previous = s.source_time_ns;
      s.frame_id = "table";
      s.clock_domain = "simulation";
      s.ball_id = "fixture";
      if (first) {
        first = false;
        if (args.size() == 3) {
          const int direction = std::stoi(args[1]);
          const std::string continuity = args[2];
          if ((direction != 1 && direction != -1) ||
              (continuity != "complete" && continuity != "unknown"))
            throw std::invalid_argument("invalid annotated boundary");
          pipeline.start(s.source_time_ns, direction, BoundaryEvidence::RESTART,
                         continuity == "complete"
                             ? HistoryContinuity::kComplete
                             : HistoryContinuity::kUnknown);
        }
      }
      const auto started = std::chrono::steady_clock::now();
      pipeline.observe(s);
      auto prediction = pipeline.forecast(plane, s.source_time_ns);
      std::cout << "{\"source_ns\":" << s.source_time_ns << ",\"compute_ms\":"
                << std::chrono::duration<double, std::milli>(
                       std::chrono::steady_clock::now() - started)
                       .count()
                << ",\"model_config_id\":\""
                << flight_model_config_id(config, settings)
                << "\",\"generation\":" << prediction.token.generation
                << ",\"state\":\"" << name(prediction.state)
                << "\",\"physical_available\":"
                << (prediction.physical_available ? "true" : "false")
                << ",\"target_valid\":"
                << (prediction.target.valid ? "true" : "false")
                << ",\"arrival_ns\":" << prediction.arrival_ns
                << ",\"reason\":\"" << prediction.reason
                << "\",\"target_kind\":\""
                << (prediction.target.event
                        ? name(prediction.target.event->kind)
                        : "unavailable")
                << "\",\"position\":";
      if (prediction.target.event)
        vector(prediction.target.event->pre.position);
      else
        std::cout << "null";
      write_spin(prediction.target.event ? prediction.target.event->pre.spin
                                         : prediction.belief.effective_spin);
      std::cout << ",\"velocity\":";
      if (prediction.target.event)
        vector(prediction.target.event->pre.velocity);
      else
        std::cout << "null";
      std::cout << ",\"covariance\":";
      const ForecastPhysicsEvent *target = nullptr;
      if (prediction.target.event)
        for (const auto &event : prediction.nominal.events)
          if (event.kind == name(prediction.target.event->kind) &&
              std::abs(event.dt_s - prediction.target.event->dt_s) < 1e-6) {
            target = &event;
            break;
          }
      if (target && target->covariance.allFinite()) {
        std::cout << '[';
        for (int row = 0; row < 7; ++row) {
          if (row)
            std::cout << ',';
          std::cout << '[';
          for (int col = 0; col < 7; ++col) {
            if (col)
              std::cout << ',';
            std::cout << target->covariance(row, col);
          }
          std::cout << ']';
        }
        std::cout << ']';
      } else
        std::cout << "null";
      std::cout << ",\"future_events\":[";
      bool event_comma = false;
      for (const auto &event : prediction.nominal.events) {
        const auto dt = duration_ns(event.dt_s);
        std::int64_t arrival;
        if (!dt || !checked_arrival(prediction.reference_ns, *dt, arrival))
          continue;
        if (event_comma)
          std::cout << ',';
        event_comma = true;
        std::cout << "{\"kind\":\"" << event.kind
                  << "\",\"arrival_ns\":" << arrival << ",\"position\":";
        vector(event.pre.head<3>());
        std::cout << ",\"velocity\":";
        vector(event.pre.tail<3>());
        std::cout << ",\"post_velocity\":";
        vector(event.post.tail<3>());
        write_spin(event.pre_spin);
        std::cout << ",\"covariance\":";
        if (event.covariance.allFinite()) {
          std::cout << '[';
          for (int r = 0; r < 7; ++r) {
            if (r)
              std::cout << ',';
            std::cout << '[';
            for (int c = 0; c < 7; ++c) {
              if (c)
                std::cout << ',';
              std::cout << event.covariance(r, c);
            }
            std::cout << ']';
          }
          std::cout << ']';
        } else
          std::cout << "null";
        std::cout << '}';
      }
      std::cout << ']';
      std::cout << "}\n";
    }
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
