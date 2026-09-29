#pragma once
#include "robot_io/robot_io_backend.hpp"
#include <memory>
#include <string>

namespace a3_pingpong {
// Single-threaded HumanLike 25-action adapter. Caller supplies 500 Hz state;
// actor+encoder run at 50 Hz. This object never accesses HAL or ROS.
class PpHumanLikePolicy {
  public:
    explicit PpHumanLikePolicy(const std::string& directory);
    ~PpHumanLikePolicy();
    void Reset(const robot_io::RobotState&, const robot_io::RobotCommand& entry);
    robot_io::RobotCommand Step(const robot_io::RobotState&, const Eigen::Vector3d& target);
    bool settled() const;
    Eigen::Vector3d filtered_velocity() const;
  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace a3_pingpong
