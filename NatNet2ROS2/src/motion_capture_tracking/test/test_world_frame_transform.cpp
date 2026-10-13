#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include "motion_capture_tracking/world_frame_transform.hpp"

struct Point { double x, y, z; };
struct Quaternion { double x, y, z, w; };
struct Pose { Point position; Quaternion orientation; };
struct Marker { bool has_live_sample; Point position, model_position; };
struct MarkerArray { Pose rigid_body_pose; std::vector<Marker> markers; };

void near(double actual, double expected) {
  if (!std::isfinite(actual) || std::abs(actual - expected) > 1e-10) {
    throw std::runtime_error("world-frame transform mismatch");
  }
}

int main() {
  using motion_capture_tracking::detail::WorldFrameTransform;
  const WorldFrameTransform transform(90.0, {0.0, 0.0, 0.0});
  // Venue geometry: -Y in Motive is forward, +X is left, +Z is up.
  Point forward{0.0, -2.74, 0.0}, left{1.525, 0.0, 0.0};
  transform.ApplyPosition(forward);
  transform.ApplyPosition(left);
  near(forward.x, 2.74); near(forward.y, 0.0);
  near(left.x, 0.0); near(left.y, 1.525);

  // The observed READY pose becomes approximately +10.4 degrees, not -169.6.
  const double yaw = -79.6 * std::acos(-1.0) / 180.0;
  Pose robot{{-.784, .621, .327}, {0.0, 0.0, std::sin(yaw/2), std::cos(yaw/2)}};
  transform.ApplyPose(robot);
  near(robot.position.x, -.621); near(robot.position.y, -.784);
  near(robot.position.z, .327);
  near(2*std::atan2(robot.orientation.z, robot.orientation.w)*180/std::acos(-1.0), 10.4);

  // A -90-degree body and its world marker must still reconstruct exactly
  // from the unchanged local ModelDef coordinates after the world rotation.
  const double h = std::sqrt(.5);
  MarkerArray markers{{{1, -2, .3}, {0, 0, -h, h}},
      {{true, {1.2, -2.1, .6}, {.1, .2, .3}},
       {false, {NAN, NAN, NAN}, {.4, .5, .6}}}};
  transform.ApplyMarkerArray(markers);
  near(markers.rigid_body_pose.position.x, 2);
  near(markers.rigid_body_pose.orientation.z, 0);
  near(markers.rigid_body_pose.orientation.w, 1);
  near(markers.markers[0].position.x, 2.1);
  near(markers.markers[0].position.y, 1.2);
  near(markers.markers[0].position.z, .6);
  near(markers.markers[0].model_position.x, .1);
  near(markers.markers[0].model_position.y, .2);
  near(markers.markers[1].model_position.z, .6);
  if (!std::isnan(markers.markers[1].position.x)) return 1;

  // Left multiplication rotates world heading; local pitch/roll stay intact.
  Quaternion tilted{.1, .2, .3, std::sqrt(.86)};
  const double gravity_z = 2*(tilted.x*tilted.x+tilted.y*tilted.y)-1;
  transform.ApplyOrientation(tilted);
  near(2*(tilted.x*tilted.x+tilted.y*tilted.y)-1, gravity_z);
  const WorldFrameTransform inverse(-90, {0, 0, 0});
  inverse.ApplyOrientation(tilted);
  near(tilted.x, .1); near(tilted.y, .2); near(tilted.z, .3);
  const WorldFrameTransform shifted(90, {1, 2, 3});
  Point point{4, 5, 6}; shifted.ApplyPosition(point);
  near(point.x, -4); near(point.y, 6); near(point.z, 9);
  const WorldFrameTransform identity(0, {0, 0, 0});
  identity.ApplyPosition(point);
  near(point.x, -4); near(point.y, 6); near(point.z, 9);
  for (bool bad_yaw : {false, true}) {
    bool rejected = false;
    try {
      const WorldFrameTransform bad(bad_yaw ? NAN : 0,
          {0, bad_yaw ? 0 : std::numeric_limits<double>::infinity(), 0});
    } catch (const std::invalid_argument&) { rejected = true; }
    if (!rejected) return 1;
  }
  std::cout << "world frame geometry, quaternion, marker consistency and validation PASS\n";
}
