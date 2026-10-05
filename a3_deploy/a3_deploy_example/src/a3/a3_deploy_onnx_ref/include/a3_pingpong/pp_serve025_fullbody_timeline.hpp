// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// Strict, transport-free reader for the original 100 Hz serve025 execution
// timeline.
// The CSV owns all 31 desired joint positions and velocities during the
// action.  It deliberately does not contain controller gains or transport
// fields; those remain properties of the existing RobotCommand path.
#pragma once

#include "robot_io/robot_io_backend.hpp"

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace a3_pingpong {

inline constexpr std::size_t kServe025FullbodyDof = 31;
inline constexpr std::size_t kServe025FullbodyPreambleRows = 5;
inline constexpr std::size_t kServe025FullbodyFrames = 468;
inline constexpr std::size_t kServe025FullbodyTimelineColumns = 8;
inline constexpr std::size_t kServe025FullbodyCsvColumns =
    kServe025FullbodyTimelineColumns + 3 * kServe025FullbodyDof;
inline constexpr double kServe025FullbodyCsvHz = 100.0;
inline constexpr double kServe025FullbodyRunnerHz = 100.0;
// Preserve the existing approximately five-second smooth transition from
// official stand to CSV frame 0 on the same 100 Hz publisher clock.
inline constexpr std::size_t kServe025FullbodyTransitionTicks = 501;
inline constexpr std::size_t kServe025FullbodyCommandTicks =
    kServe025FullbodyFrames;

inline constexpr std::size_t kServe025SpaceFrame = 0;
inline constexpr std::size_t kServe025ReleaseFrame = 48;
inline constexpr std::size_t kServe025DetachFrame = 52;
inline constexpr std::size_t kServe025StrikeFrame = 60;
inline constexpr std::size_t kServe025ContactFrame = 78;
inline constexpr std::size_t kServe025FollowthroughFrame = 166;
inline constexpr std::size_t kServe025RecoveryFrame = 167;
inline constexpr std::size_t kServe025CompleteFrame = 467;

struct Serve025FullbodyFrame {
  std::size_t frame_index{0};
  double time_s{0.0};
  double source_time_s{0.0};
  std::string phase;
  std::string event;
  Eigen::VectorXd q_sdk;
  Eigen::VectorXd qd_sdk;
  Eigen::VectorXd qdd_sdk;
};

struct Serve025FullbodyTimelineStats {
  std::size_t frames{0};
  double duration_s{0.0};
  double max_q_step_rad{0.0};
  std::size_t max_q_step_frame{0};
  int max_q_step_sdk_index{-1};
  double max_command_q_step_rad{0.0};
  std::size_t max_command_q_step_from_frame{0};
  std::size_t max_command_q_step_to_frame{0};
  int max_command_q_step_sdk_index{-1};
  double max_command_step_velocity_ratio{0.0};
  double max_abs_qd_rad_s{0.0};
  std::size_t max_abs_qd_frame{0};
  int max_abs_qd_sdk_index{-1};
};

class PpServe025FullbodyTimeline final {
 public:
  // No whole-file digest or embedded artifact ID participates in acceptance.
  // A replacement file is accepted whenever its named schema and numeric
  // safety contract pass.
  bool LoadCsv(const std::string& path, std::string& error);

  const Serve025FullbodyFrame& At(std::size_t frame) const noexcept;
  const Serve025FullbodyFrame& ready() const noexcept { return At(0); }
  const Serve025FullbodyFrame& complete() const noexcept {
    return At(kServe025CompleteFrame);
  }
  const Serve025FullbodyTimelineStats& stats() const noexcept {
    return stats_;
  }
  const std::string& csv_path() const noexcept { return csv_path_; }
  std::size_t size() const noexcept { return frames_.size(); }
  bool empty() const noexcept { return frames_.empty(); }

 private:
  std::string csv_path_;
  Serve025FullbodyTimelineStats stats_{};
  Serve025FullbodyFrame fallback_{};
  std::vector<Serve025FullbodyFrame> frames_;
};

}  // namespace a3_pingpong
