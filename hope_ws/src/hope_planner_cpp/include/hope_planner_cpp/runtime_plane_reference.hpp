#pragma once
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>

namespace hope_planner_cpp {
enum class PlaneMode { Fixed = 0, PerBall = 1, StablePerBall = 2 };
inline const char* plane_mode_name(PlaneMode mode) {
  switch (mode) {
    case PlaneMode::Fixed: return "fixed";
    case PlaneMode::PerBall: return "per_ball";
    case PlaneMode::StablePerBall: return "stable_per_ball";
  }
  return "unknown";
}
inline PlaneMode parse_plane_mode(const std::string& value) {
  if (value == "fixed") return PlaneMode::Fixed;
  if (value == "per_ball") return PlaneMode::PerBall;
  if (value == "stable_per_ball") return PlaneMode::StablePerBall;
  throw std::invalid_argument("mode must be fixed, per_ball or stable_per_ball");
}
struct PlaneSelection {
  PlaneMode mode = PlaneMode::Fixed;
  std::uint64_t epoch = 0, flight = 0;
  double reference_x = NAN, plane_x = NAN;
  bool valid = false, stable = false;
  std::string reason = "waiting_for_ball";
};
// Service callbacks only change requested mode. A solver snapshots it once per
// physical flight; neither later pose packets nor another service call retarget it.
class RuntimePlaneReference {
 public:
  void request(PlaneMode mode) { std::lock_guard<std::mutex> lock(mutex_); requested_ = mode; }
  PlaneMode requested() const { std::lock_guard<std::mutex> lock(mutex_); return requested_; }
  PlaneSelection last_command() const { std::lock_guard<std::mutex> lock(mutex_); return committed_; }
  PlaneSelection last_selection() const { std::lock_guard<std::mutex> lock(mutex_); return selected_; }
  PlaneSelection select(std::uint64_t epoch, std::uint64_t flight, double home_x,
                        double offset, double base_x, bool body_fresh,
                        bool runtime_supported, bool posture_stable) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (selected_.epoch == epoch && selected_.flight == flight && flight != 0) return selected_;
    if (epoch != epoch_) { epoch_ = epoch; held_reference_x_ = home_x; committed_ = {}; }
    PlaneSelection next;
    next.mode = requested_; next.epoch = epoch; next.flight = flight;
    next.stable = posture_stable;
    next.reference_x = requested_ == PlaneMode::Fixed ? home_x : held_reference_x_;
    if (epoch == 0 || flight == 0 || !std::isfinite(home_x) || !std::isfinite(offset) || offset <= 0) {
      next.reason = "invalid_session_reference";
    } else if (requested_ != PlaneMode::Fixed && !runtime_supported) {
      next.reason = "runner_does_not_support_runtime_planes";
    } else if (requested_ != PlaneMode::Fixed && (!body_fresh || !std::isfinite(base_x))) {
      next.reason = "runtime_plane_requires_fresh_body_pose";
    } else {
      next.valid = true;
      if (requested_ == PlaneMode::PerBall || (requested_ == PlaneMode::StablePerBall && posture_stable)) {
        next.reference_x = base_x;
        next.reason = requested_ == PlaneMode::PerBall ? "current_position" : "posture_stable_update";
      } else next.reason = requested_ == PlaneMode::Fixed ? "session_home" : "posture_unstable_keep_previous";
      next.plane_x = next.reference_x + offset;
    }
    selected_ = next;
    return next;
  }
  void commit(const PlaneSelection& selection) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (selection.valid && selection.epoch == epoch_) {
      held_reference_x_ = selection.reference_x;
      committed_ = selection;
    }
  }
 private:
  mutable std::mutex mutex_;
  PlaneMode requested_ = PlaneMode::Fixed;
  std::uint64_t epoch_ = 0;
  double held_reference_x_ = NAN;
  PlaneSelection selected_, committed_;
};
} // namespace hope_planner_cpp
