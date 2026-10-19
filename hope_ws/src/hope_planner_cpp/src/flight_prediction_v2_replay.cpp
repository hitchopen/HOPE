#include "hope_planner_cpp/flight_state_v2.hpp"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
using namespace hope_planner_cpp;
namespace {
void vector(const Vec3 &v) {
  std::cout << '[' << v.x() << ',' << v.y() << ',' << v.z() << ']';
}
} // namespace
int main(int argc, char **argv) {
  try {
    std::cout << std::setprecision(17);
    if (argc == 7 && std::string(argv[1]) == "--rollout") {
      const double h = std::stod(argv[2]);
      const int direction = std::stoi(argv[3]);
      const double speed = std::stod(argv[4]), spin = std::stod(argv[5]),
                   duration = std::stod(argv[6]);
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
      BallPhysics physics;
      EventGeometry g;
      g.receiving_x = .15;
      std::cout
          << "{\"profile\":\"thin_shell_v2_receipt_20261010\",\"events\":[";
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
      std::cout << ",\"velocity\":";
      vector(model.velocity);
      std::cout << "}\n";
      return 0;
    }
    if ((argc != 2 && argc != 4) || std::string(argv[1]) != "--observations") {
      std::cerr
          << "Usage: --rollout step_s direction speed spin_y duration_s | "
             "--observations [direction complete|unknown] < "
             "CSV(source_ns,x,y,z,omega_valid,wx,wy,wz)\n";
      return 2;
    }
    ProbabilisticConfig config;
    FlightPipeline pipeline(config);
    HitPlaneSnapshot plane;
    plane.calibrated = true;
    plane.x_hit_m = .15;
    plane.frame_id = "table";
    std::string line;
    bool first = true;
    std::optional<std::int64_t> previous;
    while (std::getline(std::cin, line)) {
      if (line.empty() || line[0] == '#')
        continue;
      std::replace(line.begin(), line.end(), ',', ' ');
      std::istringstream in(line);
      BallSample s;
      int omega_valid;
      if (!(in >> s.source_time_ns >> s.position.x() >> s.position.y() >>
            s.position.z() >> omega_valid >> s.producer_omega_rad_s.x() >>
            s.producer_omega_rad_s.y() >> s.producer_omega_rad_s.z()))
        throw std::runtime_error("invalid observation CSV row");
      if (previous && s.source_time_ns <= *previous)
        throw std::runtime_error("duplicate/out-of-order CSV source stamp");
      previous = s.source_time_ns;
      s.producer_omega_valid = omega_valid == 1;
      s.frame_id = "table";
      s.clock_domain = "simulation";
      s.ball_id = "fixture";
      if (first) {
        first = false;
        if (argc == 4) {
          const int direction = std::stoi(argv[2]);
          const std::string continuity = argv[3];
          if ((direction != 1 && direction != -1) ||
              (continuity != "complete" && continuity != "unknown"))
            throw std::invalid_argument("invalid annotated boundary");
          pipeline.start(s.source_time_ns, direction, BoundaryEvidence::RESTART,
                         continuity == "complete"
                             ? HistoryContinuity::kComplete
                             : HistoryContinuity::kUnknown);
        }
      }
      pipeline.observe(s);
      auto prediction = pipeline.forecast(plane, s.source_time_ns);
      std::cout << "{\"source_ns\":" << s.source_time_ns
                << ",\"generation\":" << prediction.token.generation
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
      std::cout << "}\n";
    }
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
