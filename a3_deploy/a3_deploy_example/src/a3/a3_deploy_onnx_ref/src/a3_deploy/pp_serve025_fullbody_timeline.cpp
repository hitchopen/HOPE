#include "a3_pingpong/pp_serve025_fullbody_timeline.hpp"

#include "a3_deploy/a3_joint_limits.hpp"
#include "a3_pingpong/pp_joint_map.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace a3_pingpong {
namespace {

constexpr double kNumberTolerance = 1.0e-9;
constexpr double kEndpointTolerance = 1.0e-12;
constexpr double kPositionReserveRad = 0.02;
constexpr double kReadyRecoveryStepLimitRad = 0.03;
constexpr double kServeStepLimitRad = 0.07;
constexpr double kVelocityLimitFraction = 0.5;

std::vector<std::string> SplitCsv(const std::string& line) {
  std::vector<std::string> fields;
  std::size_t begin = 0;
  while (true) {
    const std::size_t end = line.find(',', begin);
    fields.emplace_back(
        line.substr(begin, end == std::string::npos ? std::string::npos
                                                    : end - begin));
    if (end == std::string::npos) break;
    begin = end + 1;
  }
  if (!fields.empty() && !fields.back().empty() &&
      fields.back().back() == '\r') {
    fields.back().pop_back();
  }
  return fields;
}

double ParseDouble(const std::string& text, const std::string& label) {
  if (text.empty()) throw std::runtime_error(label + " is empty");
  errno = 0;
  char* end = nullptr;
  const double value = std::strtod(text.c_str(), &end);
  if (errno != 0 || end != text.c_str() + text.size() ||
      !std::isfinite(value)) {
    throw std::runtime_error(label + " is not a finite number");
  }
  return value;
}

std::size_t ParseSize(const std::string& text, const std::string& label) {
  if (text.empty()) throw std::runtime_error(label + " is empty");
  std::uint64_t value = 0;
  const auto parsed =
      std::from_chars(text.data(), text.data() + text.size(), value);
  if (parsed.ec != std::errc{} ||
      parsed.ptr != text.data() + text.size() ||
      value > std::numeric_limits<std::size_t>::max()) {
    throw std::runtime_error(label + " is not an unsigned integer");
  }
  return static_cast<std::size_t>(value);
}

std::vector<std::string> ExpectedHeader() {
  std::vector<std::string> result = {
      "sequence_index", "row_kind", "event", "action_tick",
      "frame_index", "time_s", "phase", "source_time_s"};
  for (const std::string_view prefix :
       {"q_rad", "qd_rad_s", "qdd_rad_s2"}) {
    for (const std::string& joint : backend_joint_order()) {
      result.emplace_back(std::string(prefix) + "::" + joint);
    }
  }
  return result;
}

std::string ExpectedEvent(std::size_t frame) {
  switch (frame) {
    case kServe025SpaceFrame: return "SPACE";
    case kServe025ReleaseFrame: return "RELEASE";
    case kServe025DetachFrame: return "DETACH_TARGET";
    case kServe025StrikeFrame: return "STRIKE_MOTION_START";
    case kServe025ContactFrame: return "NOMINAL_CONTACT";
    case kServe025FollowthroughFrame: return "FOLLOWTHROUGH_END";
    case kServe025RecoveryFrame: return "RECOVERY_START";
    case kServe025CompleteFrame: return "COMPLETE";
    default: return "NONE";
  }
}

std::string ExpectedPhase(std::size_t frame) {
  if (frame <= 50) return "READY";
  if (frame <= kServe025FollowthroughFrame) return "SERVE_FOLLOW";
  return "RECOVERY";
}

void RequireHeader(const std::vector<std::string>& actual) {
  const std::vector<std::string> expected = ExpectedHeader();
  if (actual.size() != expected.size()) {
    throw std::runtime_error(
        "serve025 fullbody CSV must contain exactly 101 columns");
  }
  for (std::size_t column = 0; column < expected.size(); ++column) {
    if (actual[column] != expected[column]) {
      throw std::runtime_error(
          "serve025 fullbody CSV header mismatch at column " +
          std::to_string(column) + ": expected " + expected[column] +
          ", got " + actual[column]);
    }
  }
}

void RequirePreamble(const std::vector<std::string>& fields,
                     std::size_t row) {
  static constexpr std::array<std::string_view,
                              kServe025FullbodyPreambleRows>
      kEvents = {"WAIT_LOAD_CLEAR", "GRAB", "WAIT_GRAB_ACK",
                 "WAIT_SECURE_CLEAR", "WAIT_SPACE"};
  if (fields.size() != kServe025FullbodyCsvColumns ||
      ParseSize(fields[0], "preamble sequence_index") != row ||
      fields[1] != "PREAMBLE" || fields[2] != kEvents[row]) {
    throw std::runtime_error(
        "serve025 fullbody preamble order/shape mismatch at row " +
        std::to_string(row));
  }
  for (std::size_t column = 3; column < fields.size(); ++column) {
    if (!fields[column].empty()) {
      throw std::runtime_error(
          "serve025 fullbody preamble carries action data at row " +
          std::to_string(row));
    }
  }
}

void RequireLowerBodySymmetry(const Serve025FullbodyFrame& frame) {
  static constexpr std::array<std::pair<int, int>, 3> kEqualPairs = {
      std::pair{19, 25}, std::pair{22, 28}, std::pair{23, 29}};
  static constexpr std::array<std::pair<int, int>, 3> kOppositePairs = {
      std::pair{20, 26}, std::pair{21, 27}, std::pair{24, 30}};
  for (const auto& [left, right] : kEqualPairs) {
    if (std::abs(frame.q_sdk[left] - frame.q_sdk[right]) >
        kNumberTolerance) {
      throw std::runtime_error(
          "serve025 fullbody lower-body equal-pair symmetry mismatch");
    }
  }
  for (const auto& [left, right] : kOppositePairs) {
    if (std::abs(frame.q_sdk[left] + frame.q_sdk[right]) >
        kNumberTolerance) {
      throw std::runtime_error(
          "serve025 fullbody lower-body mirrored-pair symmetry mismatch");
    }
  }
}

}  // namespace

bool PpServe025FullbodyTimeline::LoadCsv(const std::string& path,
                                         std::string& error) {
  try {
    std::ifstream input(path);
    if (!input) {
      throw std::runtime_error("cannot open serve025 fullbody CSV: " + path);
    }
    std::string line;
    if (!std::getline(input, line)) {
      throw std::runtime_error("serve025 fullbody CSV is empty");
    }
    RequireHeader(SplitCsv(line));

    for (std::size_t row = 0; row < kServe025FullbodyPreambleRows; ++row) {
      if (!std::getline(input, line) || line.empty()) {
        throw std::runtime_error("serve025 fullbody preamble is truncated");
      }
      RequirePreamble(SplitCsv(line), row);
    }

    std::vector<Serve025FullbodyFrame> parsed_frames;
    parsed_frames.reserve(kServe025FullbodyFrames);
    Serve025FullbodyTimelineStats parsed_stats;
    while (std::getline(input, line)) {
      if (line.empty()) {
        throw std::runtime_error("blank row in serve025 fullbody CSV");
      }
      const std::size_t frame_index = parsed_frames.size();
      if (frame_index >= kServe025FullbodyFrames) {
        throw std::runtime_error("serve025 fullbody CSV has too many frames");
      }
      const std::vector<std::string> fields = SplitCsv(line);
      if (fields.size() != kServe025FullbodyCsvColumns) {
        throw std::runtime_error(
            "serve025 fullbody frame width mismatch at frame " +
            std::to_string(frame_index));
      }
      if (ParseSize(fields[0], "sequence_index") !=
              frame_index + kServe025FullbodyPreambleRows ||
          fields[1] != "FRAME" ||
          ParseSize(fields[3], "action_tick") != frame_index ||
          ParseSize(fields[4], "frame_index") != frame_index) {
        throw std::runtime_error(
            "serve025 fullbody frame numbering mismatch at frame " +
            std::to_string(frame_index));
      }

      Serve025FullbodyFrame frame;
      frame.frame_index = frame_index;
      frame.event = fields[2];
      frame.time_s = ParseDouble(fields[5], "time_s");
      frame.phase = fields[6];
      frame.source_time_s = ParseDouble(fields[7], "source_time_s");
      frame.q_sdk = Eigen::VectorXd(kServe025FullbodyDof);
      frame.qd_sdk = Eigen::VectorXd(kServe025FullbodyDof);
      frame.qdd_sdk = Eigen::VectorXd(kServe025FullbodyDof);
      if (std::abs(frame.time_s -
                   static_cast<double>(frame_index) /
                       kServe025FullbodyCsvHz) > kNumberTolerance ||
          frame.event != ExpectedEvent(frame_index) ||
          frame.phase != ExpectedPhase(frame_index)) {
        throw std::runtime_error(
            "serve025 fullbody time/phase/event mismatch at frame " +
            std::to_string(frame_index));
      }

      for (std::size_t sdk = 0; sdk < kServe025FullbodyDof; ++sdk) {
        const double q = ParseDouble(
            fields[kServe025FullbodyTimelineColumns + sdk], "q_rad");
        const double qd = ParseDouble(
            fields[kServe025FullbodyTimelineColumns +
                   kServe025FullbodyDof + sdk],
            "qd_rad_s");
        const double qdd = ParseDouble(
            fields[kServe025FullbodyTimelineColumns +
                   2 * kServe025FullbodyDof + sdk],
            "qdd_rad_s2");
        if (q < a3_deploy::kA3SdkJointPosLo[sdk] + kPositionReserveRad ||
            q > a3_deploy::kA3SdkJointPosHi[sdk] - kPositionReserveRad) {
          throw std::runtime_error(
              "serve025 fullbody q violates reserved SDK limit at frame " +
              std::to_string(frame_index) + ", joint " +
              backend_joint_order()[sdk]);
        }
        if (std::abs(qd) >
            kVelocityLimitFraction *
                    a3_deploy::kA3SdkJointVelocityLimit[sdk] +
                kNumberTolerance) {
          throw std::runtime_error(
              "serve025 fullbody qd violates velocity envelope at frame " +
              std::to_string(frame_index) + ", joint " +
              backend_joint_order()[sdk]);
        }
        frame.q_sdk[static_cast<Eigen::Index>(sdk)] = q;
        frame.qd_sdk[static_cast<Eigen::Index>(sdk)] = qd;
        frame.qdd_sdk[static_cast<Eigen::Index>(sdk)] = qdd;
        const double abs_qd = std::abs(qd);
        if (abs_qd > parsed_stats.max_abs_qd_rad_s) {
          parsed_stats.max_abs_qd_rad_s = abs_qd;
          parsed_stats.max_abs_qd_frame = frame_index;
          parsed_stats.max_abs_qd_sdk_index = static_cast<int>(sdk);
        }
      }
      RequireLowerBodySymmetry(frame);

      if (!parsed_frames.empty()) {
        for (std::size_t sdk = 0; sdk < kServe025FullbodyDof; ++sdk) {
          const double step = std::abs(
              frame.q_sdk[static_cast<Eigen::Index>(sdk)] -
              parsed_frames.back().q_sdk[static_cast<Eigen::Index>(sdk)]);
          const double limit =
              frame_index <= kServe025DetachFrame ||
                      frame_index >= kServe025RecoveryFrame
                  ? kReadyRecoveryStepLimitRad
                  : kServeStepLimitRad;
          if (step > limit + kNumberTolerance) {
            throw std::runtime_error(
                "serve025 fullbody q step envelope exceeded at frame " +
                std::to_string(frame_index) + ", joint " +
                backend_joint_order()[sdk]);
          }
          if (step > parsed_stats.max_q_step_rad) {
            parsed_stats.max_q_step_rad = step;
            parsed_stats.max_q_step_frame = frame_index;
            parsed_stats.max_q_step_sdk_index = static_cast<int>(sdk);
          }
        }
      }
      parsed_frames.push_back(std::move(frame));
    }

    if (parsed_frames.size() != kServe025FullbodyFrames) {
      throw std::runtime_error(
          "serve025 fullbody CSV frame count mismatch: expected 468, got " +
          std::to_string(parsed_frames.size()));
    }

    for (std::size_t tick = 1;
         tick < kServe025FullbodyCommandTicks; ++tick) {
      const std::size_t from_frame = tick - 1;
      const std::size_t to_frame = tick;
      for (std::size_t sdk = 0; sdk < kServe025FullbodyDof; ++sdk) {
        const double step = std::abs(
            parsed_frames[to_frame].q_sdk[static_cast<Eigen::Index>(sdk)] -
            parsed_frames[from_frame].q_sdk[static_cast<Eigen::Index>(sdk)]);
        const double velocity_ratio =
            step * kServe025FullbodyRunnerHz /
            a3_deploy::kA3SdkJointVelocityLimit[sdk];
        if (velocity_ratio > kVelocityLimitFraction + kNumberTolerance) {
          throw std::runtime_error(
              "serve025 100 Hz command step violates velocity envelope from "
              "frame " + std::to_string(from_frame) + " to " +
              std::to_string(to_frame) + ", joint " +
              backend_joint_order()[sdk]);
        }
        parsed_stats.max_command_step_velocity_ratio = std::max(
            parsed_stats.max_command_step_velocity_ratio, velocity_ratio);
        if (step > parsed_stats.max_command_q_step_rad) {
          parsed_stats.max_command_q_step_rad = step;
          parsed_stats.max_command_q_step_from_frame = from_frame;
          parsed_stats.max_command_q_step_to_frame = to_frame;
          parsed_stats.max_command_q_step_sdk_index = static_cast<int>(sdk);
        }
      }
    }

    for (std::size_t sdk = 0; sdk < kServe025FullbodyDof; ++sdk) {
      for (const Serve025FullbodyFrame* endpoint :
           {&parsed_frames.front(), &parsed_frames.back()}) {
        if (std::abs(endpoint->qd_sdk[static_cast<Eigen::Index>(sdk)]) >
                kEndpointTolerance ||
            std::abs(endpoint->qdd_sdk[static_cast<Eigen::Index>(sdk)]) >
                kEndpointTolerance) {
          throw std::runtime_error(
              "serve025 fullbody endpoint qd/qdd must be zero");
        }
      }
    }

    parsed_stats.frames = parsed_frames.size();
    parsed_stats.duration_s = parsed_frames.back().time_s;
    csv_path_ = path;
    stats_ = parsed_stats;
    frames_ = std::move(parsed_frames);
    error.clear();
    return true;
  } catch (const std::exception& exception) {
    csv_path_.clear();
    stats_ = {};
    frames_.clear();
    error = exception.what();
    return false;
  }
}

const Serve025FullbodyFrame& PpServe025FullbodyTimeline::At(
    std::size_t frame) const noexcept {
  return frame < frames_.size() ? frames_[frame] : fallback_;
}

}  // namespace a3_pingpong
