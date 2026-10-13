#pragma once

#include <array>
#include <cmath>
#include <stdexcept>

namespace motion_capture_tracking::detail {

// Apply the venue's Z-up Motive -> canonical table transform once, at the
// publication boundary. Tracker inputs and ModelDef local coordinates stay
// in their original frames. P2's table-centre rotation remains downstream.
class WorldFrameTransform {
 public:
  WorldFrameTransform(double yaw_degrees, const std::array<double, 3>& translation)
      : translation_(translation) {
    if (!std::isfinite(yaw_degrees)) {
      throw std::invalid_argument("world transform yaw must be finite");
    }
    for (double value : translation) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("world transform translation must be finite");
      }
    }
    const double yaw = yaw_degrees * std::acos(-1.0) / 180.0;
    c_ = std::cos(yaw);
    s_ = std::sin(yaw);
    half_c_ = std::cos(yaw / 2.0);
    half_s_ = std::sin(yaw / 2.0);
  }

  template <class Point>
  void ApplyPosition(Point& point) const {
    const double x = point.x, y = point.y;
    point.x = c_ * x - s_ * y + translation_[0];
    point.y = s_ * x + c_ * y + translation_[1];
    point.z += translation_[2];
  }

  template <class Quaternion>
  void ApplyOrientation(Quaternion& q) const {
    // Hamilton product q_world_from_motive * q_motive_from_body.
    // Do not replace invalid input with a valid-looking identity quaternion.
    const double x = q.x, y = q.y, z = q.z, w = q.w;
    q.x = half_c_ * x - half_s_ * y;
    q.y = half_s_ * x + half_c_ * y;
    q.z = half_c_ * z + half_s_ * w;
    q.w = half_c_ * w - half_s_ * z;
  }

  template <class Pose>
  void ApplyPose(Pose& pose) const {
    ApplyPosition(pose.position);
    ApplyOrientation(pose.orientation);
  }

  template <class MarkerArray>
  void ApplyMarkerArray(MarkerArray& message) const {
    ApplyPose(message.rigid_body_pose);
    for (auto& marker : message.markers) {
      if (marker.has_live_sample) ApplyPosition(marker.position);
      // model_position belongs to the body frame, not the Motive world.
    }
  }

 private:
  std::array<double, 3> translation_;
  double c_, s_, half_c_, half_s_;
};

}  // namespace motion_capture_tracking::detail
