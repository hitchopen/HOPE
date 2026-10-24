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

ServeSegmentBundle ReadSegments(std::istream& input, const std::string& marker,
                                const std::vector<Serve025FullbodyFrame>& frames) {
  ServeSegmentBundle result;
  auto meta = SplitCsv(marker);
  if (frames.size() != kServe025FullbodyFrames || (meta.size() != 5 && meta.size() != 6) ||
      meta[0] != "#HOPE_SERVE_SEGMENTS_V1")
    throw std::runtime_error("invalid serve segment bundle marker");
  result.branch_frame = ParseSize(meta[1], "branch_frame");
  result.policy_blend_s = ParseDouble(meta[2], "policy_blend_s");
  result.policy_sha256 = meta[3]; result.deploy_sha256 = meta[4];
  if (meta.size() == 6 && meta[5] != "frame_event=SERVE_TO_POLICY")
    throw std::runtime_error("unknown serve branch event contract");
  std::size_t branch_events = 0;
  for (const auto& frame : frames) {
    if (frame.event != "SERVE_TO_POLICY") continue;
    ++branch_events;
    if (frame.frame_index != result.branch_frame)
      throw std::runtime_error("serve branch event does not match segment branch_frame");
  }
  if ((meta.size() == 6 && branch_events != 1) || branch_events > 1)
    throw std::runtime_error("serve bundle requires exactly one SERVE_TO_POLICY frame event");
  const auto sha = [](const std::string& value) {
    return value.size() == 64 && value.find_first_not_of("0123456789abcdef") == std::string::npos;
  };
  if (result.branch_frame <= kServe025ContactFrame || result.branch_frame >= kServe025RecoveryFrame ||
      result.policy_blend_s < .05 || result.policy_blend_s > 1. ||
      !sha(meta[3]) || !sha(meta[4]))
    throw std::runtime_error("invalid serve segment branch/blend/policy identity");
  std::vector<std::string> header = {"segment", "frame_index", "time_s", "event"};
  for (const auto* prefix : {"q_rad", "qd_rad_s", "tau_ff_nm", "kp", "kd"})
    for (const auto& joint : backend_joint_order()) header.push_back(std::string(prefix) + "::" + joint);
  std::string line;
  if (!std::getline(input, line) || SplitCsv(line) != header)
    throw std::runtime_error("serve segment header mismatch");
  std::size_t segment = 0;
  bool ready_seen = false;
  while (std::getline(input, line)) {
    const auto fields = SplitCsv(line);
    if (segment >= 3 || fields.size() != header.size() || fields[0] != kServeSegmentNames[segment])
      throw std::runtime_error("serve segment order/width mismatch");
    auto& samples = result.commands[segment];
    const auto index = samples.size();
    if (index > 1000 || ParseSize(fields[1], "segment frame") != index ||
        std::abs(ParseDouble(fields[2], "segment time") - index / 100.) > 1e-9)
      throw std::runtime_error("serve segment frame/time mismatch");
    const auto& event = fields[3];
    if ((index == 0 && event != "BEGIN") ||
        (index != 0 && event != "SAMPLE" && event != "END" && event != "POLICY_READY"))
      throw std::runtime_error("serve segment event mismatch");
    if (event == "POLICY_READY") {
      if (segment != 2 || ready_seen) throw std::runtime_error("duplicate/misplaced POLICY_READY");
      ready_seen = true; result.policy_ready_frame = index;
    }
    robot_io::RobotCommand c;
    std::array<Eigen::VectorXd*, 5> vectors = {&c.q_des, &c.dq_des, &c.tau_ff, &c.kp, &c.kd};
    for (std::size_t field = 0; field < 5; ++field) {
      vectors[field]->resize(31);
      for (int j = 0; j < 31; ++j) (*vectors[field])[j] = ParseDouble(fields[4 + field*31 + j], header[4 + field*31 + j]);
    }
    for (int j = 0; j < 31; ++j) {
      if (c.q_des[j] < a3_deploy::kA3SdkJointPosLo[j] + kPositionReserveRad ||
          c.q_des[j] > a3_deploy::kA3SdkJointPosHi[j] - kPositionReserveRad ||
          std::abs(c.dq_des[j]) > .5 * a3_deploy::kA3SdkJointVelocityLimit[j] + 1e-9 ||
          std::abs(c.tau_ff[j]) > a3_deploy::kA3SdkJointEffortLimit[j] ||
          c.kp[j] <= 0 || c.kp[j] > 5000 || c.kd[j] <= 0 || c.kd[j] > 100 ||
          (!samples.empty() && std::abs(c.q_des[j] - samples.back().q_des[j]) * 100 >
              .5 * a3_deploy::kA3SdkJointVelocityLimit[j] + 1e-9))
        throw std::runtime_error("serve segment command violates joint envelope: " + fields[0] + " frame " + fields[1]);
    }
    samples.push_back(std::move(c));
    if (event == "END") {
      if (samples.size() < 2) throw std::runtime_error("empty serve segment");
      ++segment;
    }
  }
  if (segment != 3 || !ready_seen) throw std::runtime_error("incomplete three-segment bundle");
  const auto equal = [](const auto& a, const auto& b) { return (a-b).cwiseAbs().maxCoeff() < 1e-6; };
  const auto& prepare = result.commands[0];
  const auto& stand = result.commands[1];
  const auto& policy = result.commands[2];
  if (!equal(prepare.front().q_des, frames.back().q_sdk) || prepare.front().dq_des.norm() > 1e-8 ||
      !equal(prepare.back().q_des, frames.front().q_sdk) || prepare.back().dq_des.norm() > 1e-8 ||
      !equal(stand.back().q_des, frames.back().q_sdk) || stand.back().dq_des.norm() > 1e-8)
    throw std::runtime_error("serve segment Stand/Ready endpoint mismatch");
  for (const auto* tail : {&stand, &policy})
    if (!equal(tail->front().q_des, frames[result.branch_frame].q_sdk) ||
        !equal(tail->front().dq_des, frames[result.branch_frame].qd_sdk))
      throw std::runtime_error("serve segment branch q/dq mismatch");
  for (std::size_t i = result.policy_ready_frame; i < policy.size(); ++i)
    if (!equal(policy[i].q_des, policy.back().q_des) || policy[i].dq_des.norm() > 1e-8 ||
        !equal(policy[i].tau_ff, policy.back().tau_ff) || !equal(policy[i].kp, policy.back().kp) || !equal(policy[i].kd, policy.back().kd))
      throw std::runtime_error("POLICY_READY must begin an exact final-command hold");
  // The stand branch remains the selected CSV's complete original recovery.
  if (stand.size() != frames.size() - result.branch_frame)
    throw std::runtime_error("SERVE_TO_STAND length differs from original recovery");
  for (std::size_t i = 0; i < stand.size(); ++i)
    if (!equal(stand[i].q_des, frames[result.branch_frame+i].q_sdk) ||
        !equal(stand[i].dq_des, frames[result.branch_frame+i].qd_sdk))
      throw std::runtime_error("SERVE_TO_STAND differs from original recovery");
  result.present = true;
  return result;
}

}  // namespace

void ServeSegmentBundle::RequirePolicy(const std::string& policy, const std::string& deploy) const {
  if (!present || policy != policy_sha256 || deploy != deploy_sha256)
    throw std::invalid_argument("SERVE_TO_POLICY requires its exact policy and deploy SHA256");
}

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
    ServeSegmentBundle parsed_segments;
    while (std::getline(input, line)) {
      if (line.empty()) {
        throw std::runtime_error("blank row in serve025 fullbody CSV");
      }
      const std::size_t frame_index = parsed_frames.size();
      if (line.starts_with("#HOPE_SERVE_SEGMENTS_V1")) {
        parsed_segments = ReadSegments(input, line, parsed_frames);
        break;
      }
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
          (frame.event != ExpectedEvent(frame_index) &&
           !(frame.event == "SERVE_TO_POLICY" && ExpectedEvent(frame_index) == "NONE")) ||
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
    if (!parsed_segments.present && std::any_of(parsed_frames.begin(), parsed_frames.end(),
        [](const auto& frame) { return frame.event == "SERVE_TO_POLICY"; }))
      throw std::runtime_error("SERVE_TO_POLICY event requires an annotated segment bundle");

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
    segments_ = std::move(parsed_segments);
    csv_path_ = path;
    stats_ = parsed_stats;
    frames_ = std::move(parsed_frames);
    error.clear();
    return true;
  } catch (const std::exception& exception) {
    segments_ = {};
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
