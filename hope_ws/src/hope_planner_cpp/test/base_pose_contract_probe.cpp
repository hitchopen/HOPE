#include "hope_planner_cpp/base_pose_orientation.hpp"
#include "hope_planner_cpp/hit_plane_manager.hpp"
#include "hope_planner_cpp/target_turn_guard.hpp"
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
int main() {
  using namespace hope_planner_cpp;
  HitPlaneManager plane({}, "table", "survey", 1);
  std::cout << std::setprecision(17);
  std::string line;
  std::int64_t receipt = 1;
  while (std::getline(std::cin, line)) {
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream input(line);
    std::vector<double> w(16);
    for (auto &v : w)
      if (!(input >> v))
        return 2;
    Vec3 target;
    for (int i = 0; i < 3; ++i)
      if (!(input >> target[i]))
        return 2;
    auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
                   .count();
    if (!plane.observe(w, receipt++, now, true)) {
      std::cout << "{\"valid\":false,\"reason\":\"" << plane.reason()
                << "\"}\n";
      continue;
    }
    auto q = base_orientation_wxyz(w);
    if (!q)
      return 3;
    auto d = check_target_turn(target, Vec3(w[5], w[6], w[7] - .76), *q, true);
    auto f = *q * Vec3::UnitX();
    std::cout << "{\"valid\":true,\"q\":[" << q->w() << ',' << q->x() << ','
              << q->y() << ',' << q->z() << "],\"forward\":[" << f.x() << ','
              << f.y() << ',' << f.z() << "],\"yaw_deg\":" << d.base_yaw_deg
              << ",\"turn_deg\":" << d.turn_deg
              << ",\"allowed\":" << (d.allowed ? "true" : "false") << "}\n";
  }
}
