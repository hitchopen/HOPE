// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// High-level A3 arm-only serve preparation.
//
// This executable deliberately uses the installed motion-control interface
// rather than the body-drive interface.  It never publishes waist, leg, neck
// or low-level actuator commands.  The default is offline validation; real
// publication requires two literal command-line confirmations.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <termios.h>
#include <unistd.h>

namespace {

using ArmVector = std::array<double, 14>;
using Clock = std::chrono::steady_clock;
using JointState = sensor_msgs::msg::JointState;

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kSourceHz = 200.0;
constexpr double kCommandHz = 100.0;
constexpr auto kCommandPeriod = std::chrono::milliseconds(10);
constexpr std::size_t kExpectedSourceFrames = 3878;
constexpr std::size_t kReadySourceFrame = 1600;
constexpr std::size_t kStrokeStartSourceFrame = 1848;
constexpr std::size_t kNominalStrikeSourceFrame = 1860;
constexpr std::size_t kLastStrokeSourceFrame = 3876;
constexpr std::size_t kSourceStride = 2;
constexpr std::size_t kPrepareTicks = 300;
constexpr std::size_t kTriggerHoldTicks = 100;
constexpr std::size_t kReadySettleTicks = 50;
constexpr double kMaxStepRad = 0.03;
constexpr double kMaxPrepareVelocityRadS = 3.0;
constexpr double kMaxSourceStrokeStepRad = 0.026;
constexpr double kMaxSourceStrokeVelocityRadS = 5.20;
constexpr double kStrokeSpeedScale = 1.00;
constexpr double kStrokeReachScale = 1.00;
constexpr std::size_t kReachHoldEndSourceFrame = 2200;
constexpr std::size_t kReachFadeEndSourceFrame = 3000;
constexpr double kMaxScaledStrokeVelocityLimitRatio = 0.50;
constexpr auto kStateFreshness = std::chrono::milliseconds(100);
constexpr auto kInitialStateTimeout = std::chrono::seconds(15);
constexpr auto kHandoffSignalTimeout = std::chrono::seconds(30);
constexpr auto kHandoffGraphTimeout = std::chrono::milliseconds(500);
constexpr auto kHandoffStateFreshness = std::chrono::seconds(1);

constexpr std::array<std::string_view, 14> kArmJointNames = {
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
};

// The A3 v3.0 high-level motion-control limits.  These are intentionally not
// the broader low-level URDF limits.
constexpr ArmVector kArmPositionLo = {
    -2.967, -1.588, -2.793, -1.047, -0.576, -1.623, -2.793,
    -2.967, -1.588, -2.793, -1.047, -0.576, -1.623, -2.793,
};
constexpr ArmVector kArmPositionHi = {
    2.967, 1.588, 2.793, 2.444, 0.576, 1.623, 2.793,
    2.967, 1.588, 2.793, 2.444, 0.576, 1.623, 2.793,
};
// URDF velocity limits in the same 14-axis order. The original-timing stroke
// remains below half of every active joint's stored actuator limit.
constexpr ArmVector kArmVelocityLimit = {
    13.613568165555769, 13.613568165555769, 15.707963267948966,
    15.707963267948966, 15.707963267948966, 12.775810124598491,
    12.775810124598491, 13.613568165555769, 13.613568165555769,
    15.707963267948966, 15.707963267948966, 15.707963267948966,
    12.775810124598491, 12.775810124598491,
};

// The source left-wrist roll exceeds the high-level API limit before READY.
// Freeze left-wrist roll at its measured entry value.  The remaining thirteen
// arm axes follow the source.
constexpr std::array<bool, 14> kSourceActive = {
    true, true, true, true, false, true, true,
    true, true, true, true, true, true, true,
};

constexpr std::array<std::string_view, 6> kRootColumns = {
    "root_translateX", "root_translateY", "root_translateZ",
    "root_rotateX", "root_rotateY", "root_rotateZ",
};

constexpr std::array<std::string_view, 31> kCsvJointNames = {
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "head_yaw_joint",
    "head_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
};

struct Arguments {
  std::string motion_csv{"motions/serve_policy.csv"};
  std::string state_topic{"/motion/control/arm_joint_state"};
  std::string command_topic{"/motion/control/arm_joint_command"};
  std::string handoff_ready_file;
  std::string mode{"prepare-only"};
  double hold_seconds{3.0};
  bool offline_validate{false};
  bool allow_publish{false};
  bool confirm_real_commands{false};
};

struct Trajectory {
  std::vector<ArmVector> ready_frames;
  std::vector<ArmVector> stroke_frames;
  ArmVector source_min{};
  ArmVector source_max{};
  double max_step_rad{0.0};
  double max_speed_rad_s{0.0};
  std::size_t max_step_source_frame{0};
  std::size_t max_step_joint{0};
  double max_stroke_step_rad{0.0};
  double max_stroke_speed_rad_s{0.0};
  std::size_t max_stroke_step_source_frame{0};
  std::size_t max_stroke_step_joint{0};
  double max_scaled_stroke_velocity_ratio{0.0};
  std::size_t max_scaled_stroke_velocity_ratio_joint{0};
};

struct ArmSample {
  ArmVector q{};
  Clock::time_point received{};
  std::uint64_t sequence{0};
  bool valid{false};
};

volatile std::sig_atomic_t g_stop = 0;
volatile std::sig_atomic_t g_handoff_requested = 0;

void OnSignal(int) { g_stop = 1; }
void OnHandoffSignal(int) { g_handoff_requested = 1; }

void PrintUsage(const char* argv0) {
  std::cout
      << "Usage:\n"
      << "  " << argv0
      << " --motion-csv PATH --offline-validate\n"
      << "  " << argv0
      << " --motion-csv PATH --allow-publish --confirm-real-commands"
         " --handoff-ready-file PATH"
         " --mode hold-only [--hold-seconds 3]\n"
      << "  " << argv0
      << " --motion-csv PATH --allow-publish --confirm-real-commands"
         " --handoff-ready-file PATH --mode prepare-only\n"
      << "  " << argv0
      << " --motion-csv PATH --allow-publish --confirm-real-commands"
         " --handoff-ready-file PATH --mode serve-only\n"
      << "\nDefault behavior is offline validation with no ROS publisher.\n";
}

bool ParseFiniteDouble(const std::string& text, double& value) {
  if (text.empty()) return false;
  errno = 0;
  char* end = nullptr;
  value = std::strtod(text.c_str(), &end);
  return errno == 0 && end == text.c_str() + text.size() &&
         std::isfinite(value);
}

bool ParseArguments(int argc, char** argv, Arguments& args,
                    std::string& error) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto take_value = [&](std::string& output) -> bool {
      if (++i >= argc) {
        error = arg + " requires a value";
        return false;
      }
      output = argv[i];
      return true;
    };
    if (arg == "--motion-csv" || arg == "--motion") {
      if (!take_value(args.motion_csv)) return false;
    } else if (arg == "--state-topic") {
      if (!take_value(args.state_topic)) return false;
    } else if (arg == "--command-topic") {
      if (!take_value(args.command_topic)) return false;
    } else if (arg == "--handoff-ready-file") {
      if (!take_value(args.handoff_ready_file)) return false;
    } else if (arg == "--mode") {
      if (!take_value(args.mode)) return false;
    } else if (arg == "--hold-seconds") {
      std::string text;
      if (!take_value(text)) return false;
      if (!ParseFiniteDouble(text, args.hold_seconds) ||
          args.hold_seconds <= 0.0 || args.hold_seconds > 60.0) {
        error = "--hold-seconds must be finite and in (0, 60]";
        return false;
      }
    } else if (arg == "--offline-validate" || arg == "--validate-only") {
      args.offline_validate = true;
    } else if (arg == "--allow-publish") {
      args.allow_publish = true;
    } else if (arg == "--confirm-real-commands") {
      args.confirm_real_commands = true;
    } else if (arg == "--help" || arg == "-h") {
      PrintUsage(argv[0]);
      std::exit(0);
    } else {
      error = "unknown argument: " + arg;
      return false;
    }
  }
  if (args.motion_csv.empty() || args.state_topic.empty() ||
      args.command_topic.empty()) {
    error = "motion path and topic names must be non-empty";
    return false;
  }
  if (args.mode != "hold-only" && args.mode != "prepare-only" &&
      args.mode != "serve-only") {
    error = "--mode must be hold-only, prepare-only, or serve-only";
    return false;
  }
  if (args.offline_validate &&
      (args.allow_publish || args.confirm_real_commands)) {
    error = "--offline-validate cannot be combined with real-command flags";
    return false;
  }
  if (args.allow_publish != args.confirm_real_commands) {
    error = "real publication requires both --allow-publish and "
            "--confirm-real-commands";
    return false;
  }
  if (args.allow_publish && args.handoff_ready_file.empty()) {
    error = "real publication requires --handoff-ready-file PATH";
    return false;
  }
  return true;
}

std::vector<std::string> SplitCsvLine(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    if (!field.empty() && field.back() == '\r') field.pop_back();
    fields.push_back(std::move(field));
  }
  if (!line.empty() && line.back() == ',') fields.emplace_back();
  return fields;
}

bool InHardLimits(const ArmVector& q, std::string& error,
                  std::string_view context) {
  for (std::size_t i = 0; i < q.size(); ++i) {
    if (!std::isfinite(q[i])) {
      error = std::string(context) + ": non-finite " +
              std::string(kArmJointNames[i]);
      return false;
    }
    if (q[i] < kArmPositionLo[i] || q[i] > kArmPositionHi[i]) {
      std::ostringstream stream;
      stream << context << ": " << kArmJointNames[i] << '=' << q[i]
             << " is outside high-level API limit [" << kArmPositionLo[i]
             << ", " << kArmPositionHi[i] << ']';
      error = stream.str();
      return false;
    }
  }
  return true;
}

double SmoothUnitInterval(double value) {
  const double u = std::clamp(value, 0.0, 1.0);
  return u * u * (3.0 - 2.0 * u);
}

double StrokeReachEnvelope(double source_frame) {
  if (source_frame <=
      static_cast<double>(kNominalStrikeSourceFrame)) {
    return SmoothUnitInterval(
        (source_frame -
         static_cast<double>(kStrokeStartSourceFrame)) /
        static_cast<double>(
            kNominalStrikeSourceFrame - kStrokeStartSourceFrame));
  }
  if (source_frame <=
      static_cast<double>(kReachHoldEndSourceFrame)) {
    return 1.0;
  }
  if (source_frame >=
      static_cast<double>(kReachFadeEndSourceFrame)) {
    return 0.0;
  }
  return 1.0 - SmoothUnitInterval(
                   (source_frame -
                    static_cast<double>(kReachHoldEndSourceFrame)) /
                   static_cast<double>(
                       kReachFadeEndSourceFrame -
                       kReachHoldEndSourceFrame));
}

bool ScaleStrokeTimeline(Trajectory& trajectory, std::string& error) {
  if (trajectory.stroke_frames.size() < 2) {
    error = "stroke trajectory requires at least two frames";
    return false;
  }

  const std::vector<ArmVector> source =
      std::move(trajectory.stroke_frames);
  const double last_source_index =
      static_cast<double>(source.size() - 1);
  const double source_frames_per_command =
      static_cast<double>(kSourceStride) * kStrokeSpeedScale;
  const std::size_t output_frames =
      static_cast<std::size_t>(
          std::ceil(last_source_index / source_frames_per_command)) +
      1;
  trajectory.stroke_frames.clear();
  trajectory.stroke_frames.reserve(output_frames);
  const ArmVector& stroke_origin = source.front();

  for (std::size_t output_frame = 0;; ++output_frame) {
    const double source_index =
        std::min(last_source_index,
                 static_cast<double>(output_frame) *
                     source_frames_per_command);
    const std::size_t lower =
        static_cast<std::size_t>(std::floor(source_index));
    const std::size_t upper = std::min(lower + 1, source.size() - 1);
    const double blend = source_index - static_cast<double>(lower);
    ArmVector command{};
    for (std::size_t joint = 0; joint < command.size(); ++joint) {
      command[joint] =
          source[lower][joint] +
          blend * (source[upper][joint] - source[lower][joint]);
    }
    const double absolute_source_frame =
        static_cast<double>(kStrokeStartSourceFrame) + source_index;
    const double reach_gain =
        1.0 + (kStrokeReachScale - 1.0) *
                  StrokeReachEnvelope(absolute_source_frame);
    for (std::size_t joint = 7; joint < command.size(); ++joint) {
      command[joint] =
          stroke_origin[joint] +
          reach_gain * (command[joint] - stroke_origin[joint]);
    }
    if (source_index >= last_source_index) {
      command = source.back();
    }
    for (std::size_t joint = 0; joint < command.size(); ++joint) {
      if (!kSourceActive[joint]) continue;
      if (command[joint] < kArmPositionLo[joint] ||
          command[joint] > kArmPositionHi[joint]) {
        std::ostringstream stream;
        stream << "scaled stroke exceeds high-level position limit at "
               << "command frame " << output_frame << ", "
               << kArmJointNames[joint] << '=' << command[joint];
        error = stream.str();
        return false;
      }
    }
    trajectory.stroke_frames.push_back(command);
    if (source_index >= last_source_index) break;
  }

  trajectory.max_stroke_step_rad = 0.0;
  trajectory.max_stroke_speed_rad_s = 0.0;
  trajectory.max_stroke_step_source_frame = 0;
  trajectory.max_stroke_step_joint = 0;
  trajectory.max_scaled_stroke_velocity_ratio = 0.0;
  trajectory.max_scaled_stroke_velocity_ratio_joint = 0;
  for (std::size_t frame = 1;
       frame < trajectory.stroke_frames.size(); ++frame) {
    for (std::size_t joint = 0; joint < kArmJointNames.size(); ++joint) {
      if (!kSourceActive[joint]) continue;
      const double step = std::abs(
          trajectory.stroke_frames[frame][joint] -
          trajectory.stroke_frames[frame - 1][joint]);
      const double velocity = step * kCommandHz;
      const double velocity_ratio =
          velocity / kArmVelocityLimit[joint];
      if (step > trajectory.max_stroke_step_rad) {
        trajectory.max_stroke_step_rad = step;
        trajectory.max_stroke_speed_rad_s = velocity;
        trajectory.max_stroke_step_source_frame = frame;
        trajectory.max_stroke_step_joint = joint;
      }
      if (velocity_ratio >
          trajectory.max_scaled_stroke_velocity_ratio) {
        trajectory.max_scaled_stroke_velocity_ratio = velocity_ratio;
        trajectory.max_scaled_stroke_velocity_ratio_joint = joint;
      }
    }
  }
  if (trajectory.max_scaled_stroke_velocity_ratio >
      kMaxScaledStrokeVelocityLimitRatio + 1.0e-12) {
    std::ostringstream stream;
    stream << "scaled stroke velocity ratio exceeds "
           << kMaxScaledStrokeVelocityLimitRatio << " at "
           << kArmJointNames[
                  trajectory.max_scaled_stroke_velocity_ratio_joint]
           << ": " << trajectory.max_scaled_stroke_velocity_ratio;
    error = stream.str();
    return false;
  }
  return true;
}

bool LoadTrajectory(const std::string& path, Trajectory& trajectory,
                    std::string& error) {
  std::ifstream input(path);
  if (!input) {
    error = "cannot open motion CSV: " + path;
    return false;
  }

  std::string line;
  if (!std::getline(input, line)) {
    error = "motion CSV is empty";
    return false;
  }
  const auto header = SplitCsvLine(line);
  if (header.size() != kRootColumns.size() + kCsvJointNames.size()) {
    error = "motion CSV must have exactly 37 columns";
    return false;
  }
  for (std::size_t i = 0; i < kRootColumns.size(); ++i) {
    if (header[i] != kRootColumns[i]) {
      error = "root header mismatch at column " + std::to_string(i);
      return false;
    }
  }
  for (std::size_t i = 0; i < kCsvJointNames.size(); ++i) {
    if (header[kRootColumns.size() + i] != kCsvJointNames[i]) {
      error = "joint header mismatch at SDK index " + std::to_string(i);
      return false;
    }
  }

  trajectory = Trajectory{};
  trajectory.source_min.fill(std::numeric_limits<double>::infinity());
  trajectory.source_max.fill(-std::numeric_limits<double>::infinity());
  std::size_t source_frame = 0;
  while (std::getline(input, line)) {
    if (line.empty()) {
      error = "blank row at source frame " + std::to_string(source_frame);
      return false;
    }
    const auto fields = SplitCsvLine(line);
    if (fields.size() != header.size()) {
      error = "row width mismatch at source frame " +
              std::to_string(source_frame);
      return false;
    }

    for (std::size_t column = 0; column < fields.size(); ++column) {
      double ignored = 0.0;
      if (!ParseFiniteDouble(fields[column], ignored)) {
        error = "non-finite CSV value at source frame " +
                std::to_string(source_frame) + ", column " +
                std::to_string(column);
        return false;
      }
    }

    ArmVector arm{};
    for (std::size_t arm_index = 0; arm_index < arm.size(); ++arm_index) {
      const std::size_t sdk_index =
          arm_index < 7 ? 5 + arm_index : 12 + (arm_index - 7);
      double degrees = 0.0;
      if (!ParseFiniteDouble(
              fields[kRootColumns.size() + sdk_index], degrees)) {
        error = "invalid arm value at source frame " +
                std::to_string(source_frame);
        return false;
      }
      arm[arm_index] = degrees * kDegToRad;
      trajectory.source_min[arm_index] =
          std::min(trajectory.source_min[arm_index], arm[arm_index]);
      trajectory.source_max[arm_index] =
          std::max(trajectory.source_max[arm_index], arm[arm_index]);
      if (kSourceActive[arm_index] &&
          (arm[arm_index] < kArmPositionLo[arm_index] ||
           arm[arm_index] > kArmPositionHi[arm_index])) {
        std::ostringstream stream;
        stream << "active CSV joint " << kArmJointNames[arm_index]
               << " exceeds high-level API limit at source frame "
               << source_frame << ": " << arm[arm_index] << " not in ["
               << kArmPositionLo[arm_index] << ", "
               << kArmPositionHi[arm_index] << ']';
        error = stream.str();
        return false;
      }
    }

    if (source_frame <= kReadySourceFrame &&
        source_frame % kSourceStride == 0) {
      if (!trajectory.ready_frames.empty()) {
        const auto& previous = trajectory.ready_frames.back();
        for (std::size_t i = 0; i < arm.size(); ++i) {
          if (!kSourceActive[i]) continue;
          const double step = std::abs(arm[i] - previous[i]);
          if (step > trajectory.max_step_rad) {
            trajectory.max_step_rad = step;
            trajectory.max_step_source_frame = source_frame;
            trajectory.max_step_joint = i;
          }
        }
      }
      trajectory.ready_frames.push_back(arm);
    }

    if (source_frame >= kStrokeStartSourceFrame &&
        source_frame <= kLastStrokeSourceFrame) {
      const auto& previous = trajectory.stroke_frames.empty()
                                 ? trajectory.ready_frames.back()
                                 : trajectory.stroke_frames.back();
      for (std::size_t i = 0; i < arm.size(); ++i) {
        if (!kSourceActive[i]) continue;
        const double step = std::abs(arm[i] - previous[i]);
        if (step > trajectory.max_stroke_step_rad) {
          trajectory.max_stroke_step_rad = step;
          trajectory.max_stroke_step_source_frame = source_frame;
          trajectory.max_stroke_step_joint = i;
        }
      }
      trajectory.stroke_frames.push_back(arm);
    }
    ++source_frame;
  }

  if (source_frame != kExpectedSourceFrames) {
    error = "source frame count mismatch: expected " +
            std::to_string(kExpectedSourceFrames) + ", got " +
            std::to_string(source_frame);
    return false;
  }
  const std::size_t expected_ready_frames =
      kReadySourceFrame / kSourceStride + 1;
  if (trajectory.ready_frames.size() != expected_ready_frames) {
    error = "internal ready-frame count mismatch";
    return false;
  }
  const std::size_t expected_stroke_frames =
      kLastStrokeSourceFrame - kStrokeStartSourceFrame + 1;
  if (trajectory.stroke_frames.size() != expected_stroke_frames) {
    error = "internal stroke-frame count mismatch";
    return false;
  }
  trajectory.max_speed_rad_s = trajectory.max_step_rad * kCommandHz;
  trajectory.max_stroke_speed_rad_s =
      trajectory.max_stroke_step_rad * kSourceHz;
  if (trajectory.max_step_rad > kMaxStepRad) {
    error = "ready trajectory step exceeds 0.03 rad";
    return false;
  }
  if (trajectory.max_speed_rad_s > kMaxPrepareVelocityRadS) {
    error = "ready trajectory velocity exceeds 3 rad/s";
    return false;
  }
  if (trajectory.max_stroke_step_rad >
      kMaxSourceStrokeStepRad + 1.0e-12) {
    error = "200 Hz source stroke step exceeds 0.026 rad";
    return false;
  }
  if (trajectory.max_stroke_speed_rad_s >
      kMaxSourceStrokeVelocityRadS + 1.0e-12) {
    error = "200 Hz source stroke velocity exceeds 5.20 rad/s";
    return false;
  }
  return ScaleStrokeTimeline(trajectory, error);
}

double MinimumJerk(double u) {
  u = std::clamp(u, 0.0, 1.0);
  return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
}

bool BuildPreparePlan(const ArmVector& measured,
                      const Trajectory& trajectory,
                      std::vector<ArmVector>& plan, std::string& error,
                      double& max_prepare_velocity) {
  if (trajectory.ready_frames.empty()) {
    error = "ready trajectory is empty";
    return false;
  }
  if (!InHardLimits(measured, error, "measured arm state")) return false;

  ArmVector target = trajectory.ready_frames.front();
  for (std::size_t i = 0; i < target.size(); ++i) {
    if (!kSourceActive[i]) target[i] = measured[i];
  }
  if (!InHardLimits(target, error, "prepare target")) return false;

  max_prepare_velocity = 0.0;
  constexpr double duration_s =
      static_cast<double>(kPrepareTicks) / kCommandHz;
  for (std::size_t i = 0; i < measured.size(); ++i) {
    if (!kSourceActive[i]) continue;
    // max(d/du minimum-jerk) == 1.875 at u=0.5.
    max_prepare_velocity =
        std::max(max_prepare_velocity,
                 1.875 * std::abs(target[i] - measured[i]) / duration_s);
  }
  if (max_prepare_velocity > kMaxPrepareVelocityRadS) {
    error = "measured-to-source-frame0 prepare velocity would exceed 3 rad/s";
    return false;
  }

  plan.clear();
  plan.reserve(kPrepareTicks + trajectory.ready_frames.size());
  ArmVector previous = measured;
  for (std::size_t tick = 1; tick <= kPrepareTicks; ++tick) {
    const double u =
        static_cast<double>(tick) / static_cast<double>(kPrepareTicks);
    const double blend = MinimumJerk(u);
    ArmVector command = measured;
    for (std::size_t i = 0; i < command.size(); ++i) {
      if (kSourceActive[i]) {
        command[i] = measured[i] + (target[i] - measured[i]) * blend;
      }
    }
    plan.push_back(command);
  }
  for (const ArmVector& source : trajectory.ready_frames) {
    ArmVector command = source;
    for (std::size_t i = 0; i < command.size(); ++i) {
      if (!kSourceActive[i]) command[i] = measured[i];
    }
    plan.push_back(command);
  }

  for (std::size_t frame = 0; frame < plan.size(); ++frame) {
    if (!InHardLimits(plan[frame], error,
                      "generated command frame " + std::to_string(frame))) {
      return false;
    }
    for (std::size_t i = 0; i < previous.size(); ++i) {
      const double step = std::abs(plan[frame][i] - previous[i]);
      if (step > kMaxStepRad + 1.0e-12) {
        std::ostringstream stream;
        stream << "generated command step exceeds 0.03 rad at frame "
               << frame << ", joint " << kArmJointNames[i] << ": " << step;
        error = stream.str();
        return false;
      }
      if (!kSourceActive[i] &&
          std::abs(plan[frame][i] - measured[i]) > 1.0e-12) {
        error = "internal error: frozen left-wrist roll changed";
        return false;
      }
    }
    previous = plan[frame];
  }
  return true;
}

class ArmIo final {
 public:
  ArmIo(std::string state_topic, std::string command_topic)
      : node_(std::make_shared<rclcpp::Node>(
            "a3_serve_vendor_arm_runner")),
        state_topic_(std::move(state_topic)),
        command_topic_(std::move(command_topic)) {
    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    state_qos.best_effort();
    state_qos.durability_volatile();
    state_sub_ = node_->create_subscription<JointState>(
        state_topic_, state_qos,
        [this](const JointState::SharedPtr message) {
          HandleState(*message);
        });

    // Request the weakest QoS so this monitor is compatible with both the
    // installed reliable/transient-local publisher and any unexpected
    // best-effort/volatile publisher that appears after handoff.
    auto command_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    command_qos.best_effort();
    command_qos.durability_volatile();
    rclcpp::SubscriptionOptions command_options;
    command_options.ignore_local_publications = true;
    command_monitor_ = node_->create_subscription<JointState>(
        command_topic_, command_qos,
        [this](const JointState::SharedPtr message) {
          HandleCommandMonitor(*message);
        },
        command_options);
    command_message_.header.frame_id = "a3_serve_vendor_arm_runner";
    command_message_.name.reserve(kArmJointNames.size());
    for (std::string_view name : kArmJointNames) {
      command_message_.name.emplace_back(name);
    }
    command_message_.position.resize(kArmJointNames.size());
    command_message_.velocity.assign(kArmJointNames.size(), 0.0);
    command_message_.effort.assign(kArmJointNames.size(), 0.0);
    executor_.add_node(node_);
  }

  ~ArmIo() { StopRuntimeMonitoring(); }

  void SpinSome() {
    // Runtime callbacks only copy the latest 14 positions and set collision
    // flags.  Bound executor work so the 100 Hz command thread cannot be
    // delayed by a callback backlog.
    executor_.spin_some(std::chrono::milliseconds(1));
  }

  void DrainCallbacks() {
    // spin_some() drains the currently executable work.  Repeat it so command
    // samples already delivered by DDS during graph teardown are consumed
    // before the expected pre-handoff latch is reset.
    for (unsigned pass = 0; pass < 4; ++pass) {
      executor_.spin_some();
      std::this_thread::yield();
    }
  }

  std::optional<ArmSample> LatestState(std::string& protocol_error) const {
    std::lock_guard<std::mutex> lock(mutex_);
    protocol_error = protocol_error_;
    if (!sample_.valid) return std::nullopt;
    return sample_;
  }

  void MarkHandoffComplete() {
    std::lock_guard<std::mutex> lock(mutex_);
    handoff_complete_ = true;
  }

  std::size_t ExistingCommandPublisherCount() const {
    return node_->count_publishers(command_topic_);
  }

  void OpenPublisher() {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliable();
    qos.transient_local();
    publisher_ = node_->create_publisher<JointState>(command_topic_, qos);
  }

  void StartRuntimeMonitoring() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      runtime_started_ = true;
      runtime_closing_ = false;
      runtime_failure_.clear();
    }
    callback_thread_ = std::thread([this]() {
      try {
        executor_.spin();
      } catch (const std::exception& exception) {
        LatchRuntimeFailure(
            "runtime callback executor failed: " +
            std::string(exception.what()));
      } catch (...) {
        LatchRuntimeFailure(
            "runtime callback executor failed with an unknown exception");
      }
    });
  }

  void StopRuntimeMonitoring() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      runtime_closing_ = true;
    }
    publisher_.reset();
    executor_.cancel();
    if (callback_thread_.joinable()) callback_thread_.join();
    std::lock_guard<std::mutex> lock(mutex_);
    runtime_started_ = false;
  }

  bool RuntimeMonitorHealthy(std::string& detail) const {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!runtime_started_) {
        detail = "runtime monitor is not running";
        return false;
      }
      if (!runtime_failure_.empty()) {
        detail = runtime_failure_;
        return false;
      }
      if (foreign_command_collision_) {
        detail = foreign_collision_detail_;
        return false;
      }
    }
    return true;
  }

  void Publish(const ArmVector& q) {
    command_message_.header.stamp = node_->now();
    std::copy(q.begin(), q.end(), command_message_.position.begin());
    publisher_->publish(command_message_);
  }

 private:
  bool DecodeArmMessage(const JointState& message, ArmVector& output,
                        std::string& error) const {
    if (message.name.size() != kArmJointNames.size() ||
        message.position.size() != kArmJointNames.size()) {
      error = "JointState must contain exactly 14 names and 14 positions";
      return false;
    }
    std::unordered_map<std::string, std::size_t> index;
    for (std::size_t i = 0; i < message.name.size(); ++i) {
      if (!index.emplace(message.name[i], i).second) {
        error = "JointState contains duplicate joint name: " +
                message.name[i];
        return false;
      }
    }
    if (index.size() != kArmJointNames.size()) {
      error = "JointState name set is not the exact 14-axis arm contract";
      return false;
    }
    for (std::size_t i = 0; i < kArmJointNames.size(); ++i) {
      const auto it = index.find(std::string(kArmJointNames[i]));
      if (it == index.end()) {
        error = "JointState is missing joint: " +
                std::string(kArmJointNames[i]);
        return false;
      }
      output[i] = message.position[it->second];
      if (!std::isfinite(output[i])) {
        error = "JointState contains non-finite position";
        return false;
      }
    }
    return true;
  }

  void HandleState(const JointState& message) {
    ArmVector decoded{};
    std::string error;
    if (!DecodeArmMessage(message, decoded, error)) {
      std::lock_guard<std::mutex> lock(mutex_);
      protocol_error_ = std::move(error);
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    sample_.q = decoded;
    sample_.received = Clock::now();
    ++sample_.sequence;
    sample_.valid = true;
  }

  void HandleCommandMonitor(const JointState&) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!handoff_complete_) return;
    // Local publications are excluded at subscription creation, so every
    // callback after handoff is from a foreign publisher. The hot command
    // loop only reads this latch; it performs no graph query or decoding.
    foreign_command_collision_ = true;
    foreign_collision_detail_ =
        "received a foreign arm command after runner publication began";
  }

  void LatchRuntimeFailure(std::string detail) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!runtime_closing_ && runtime_failure_.empty()) {
      runtime_failure_ = std::move(detail);
    }
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::string state_topic_;
  std::string command_topic_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<JointState>::SharedPtr state_sub_;
  rclcpp::Subscription<JointState>::SharedPtr command_monitor_;
  rclcpp::Publisher<JointState>::SharedPtr publisher_;
  JointState command_message_;
  mutable std::mutex mutex_;
  ArmSample sample_{};
  std::string protocol_error_;
  bool handoff_complete_{false};
  bool foreign_command_collision_{false};
  std::string foreign_collision_detail_;
  std::thread callback_thread_;
  bool runtime_started_{false};
  bool runtime_closing_{true};
  std::string runtime_failure_;
};

bool WaitForFreshState(ArmIo& io, ArmSample& sample, std::string& error) {
  const auto deadline = Clock::now() + kInitialStateTimeout;
  while (!g_stop && Clock::now() < deadline) {
    io.SpinSome();
    std::string protocol_error;
    const auto candidate = io.LatestState(protocol_error);
    if (!protocol_error.empty()) {
      error = "invalid arm state: " + protocol_error;
      return false;
    }
    if (candidate &&
        Clock::now() - candidate->received <= kStateFreshness) {
      sample = *candidate;
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  error = "timed out waiting for a fresh exact-name 14-axis arm state";
  return false;
}

bool CheckRuntimeMonitoring(ArmIo& io, std::string& error) {
  std::string detail;
  if (!io.RuntimeMonitorHealthy(detail)) {
    error = "runtime monitor: " + detail;
    return false;
  }
  return true;
}

bool WriteHandoffMarker(const std::string& path, std::string_view marker,
                        std::string& error) {
  std::ofstream output(path, std::ios::out | std::ios::trunc);
  output << marker << '\n';
  output.flush();
  if (!output) {
    error = "cannot write handoff marker " + std::string(marker) + ": " +
            path;
    return false;
  }
  return true;
}

void PrintValidation(const Arguments& args, const Trajectory& trajectory) {
  const std::size_t nominal_strike_command_frame =
      static_cast<std::size_t>(std::ceil(
          (static_cast<double>(
               kNominalStrikeSourceFrame - kStrokeStartSourceFrame) /
           static_cast<double>(kSourceStride)) /
          kStrokeSpeedScale));
  const double trigger_to_nominal_strike_s =
      static_cast<double>(
          kTriggerHoldTicks + nominal_strike_command_frame) /
      kCommandHz;
  std::cout << std::fixed << std::setprecision(6)
            << "[vendor-arm] validation PASS: " << args.motion_csv << '\n'
            << "[vendor-arm] source=200Hz frames="
            << kExpectedSourceFrames << " ready_source_frame="
            << kReadySourceFrame << " command=100Hz ready_frames="
            << trajectory.ready_frames.size() << '\n'
            << "[vendor-arm] active joints: "
               "left_shoulder_pitch_joint,left_shoulder_roll_joint,"
               "left_shoulder_yaw_joint,left_elbow_joint,"
               "left_wrist_pitch_joint,left_wrist_yaw_joint,"
               "right_shoulder_pitch_joint,right_shoulder_roll_joint,"
               "right_shoulder_yaw_joint,right_elbow_joint,"
               "right_wrist_roll_joint,right_wrist_pitch_joint,"
               "right_wrist_yaw_joint\n"
            << "[vendor-arm] filtered/frozen-at-entry joint: "
               "left_wrist_roll_joint\n"
            << "[vendor-arm] max_ready_step="
            << trajectory.max_step_rad << " rad max_ready_velocity="
            << trajectory.max_speed_rad_s << " rad/s source_frame="
            << trajectory.max_step_source_frame << " joint="
            << kArmJointNames[trajectory.max_step_joint] << '\n'
            << "[vendor-arm] stroke_start_source_frame="
            << kStrokeStartSourceFrame << " nominal_strike_source_frame="
            << kNominalStrikeSourceFrame << " stroke_frames="
            << trajectory.stroke_frames.size() << " speed_scale="
            << kStrokeSpeedScale << " reach_scale="
            << kStrokeReachScale << " trigger_to_nominal_strike="
            << trigger_to_nominal_strike_s << "s max_stroke_step="
            << trajectory.max_stroke_step_rad
            << " rad max_stroke_velocity="
            << trajectory.max_stroke_speed_rad_s
            << " rad/s command_frame="
            << trajectory.max_stroke_step_source_frame << " joint="
            << kArmJointNames[trajectory.max_stroke_step_joint]
            << " max_velocity_limit_ratio="
            << trajectory.max_scaled_stroke_velocity_ratio << " joint="
            << kArmJointNames[
                   trajectory.max_scaled_stroke_velocity_ratio_joint]
            << '\n';
  for (std::size_t i = 0; i < kArmJointNames.size(); ++i) {
    if (!kSourceActive[i]) {
      std::cout << "[vendor-arm] ignored/frozen CSV range "
                << kArmJointNames[i] << "=[" << trajectory.source_min[i]
                << ", " << trajectory.source_max[i] << "] rad\n";
    }
  }
}

class ScopedServeTerminal final {
 public:
  ~ScopedServeTerminal() {
    if (enabled_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_);
    }
  }

  bool Enable(std::string& error) {
    if (tcgetattr(STDIN_FILENO, &original_) != 0) {
      error = "cannot read interactive terminal settings";
      return false;
    }
    termios raw = original_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
      error = "cannot enable non-blocking Space input";
      return false;
    }
    tcflush(STDIN_FILENO, TCIFLUSH);
    enabled_ = true;
    return true;
  }

  bool ConsumeSpace(bool& pressed, std::string& error) {
    pressed = false;
    char input[32];
    while (true) {
      const ssize_t count = read(STDIN_FILENO, input, sizeof(input));
      if (count == 0) return true;
      if (count < 0) {
        if (errno == EINTR) return true;
        error = "interactive terminal read failed";
        return false;
      }
      for (ssize_t i = 0; i < count; ++i) {
        if (input[i] == ' ') pressed = true;
      }
      if (count < static_cast<ssize_t>(sizeof(input))) return true;
    }
  }

 private:
  termios original_{};
  bool enabled_{false};
};

int RunReal(const Arguments& args, const Trajectory& trajectory) {
  if (args.mode == "serve-only" &&
      ::isatty(STDIN_FILENO) != 1) {
    std::cerr << "[vendor-arm] serve-only requires an interactive TTY\n";
    return 4;
  }

  ScopedServeTerminal serve_terminal;
  std::string terminal_error;
  if (args.mode == "serve-only" &&
      !serve_terminal.Enable(terminal_error)) {
    std::cerr << "[vendor-arm SAFETY] " << terminal_error << '\n';
    return 4;
  }

  rclcpp::init(0, nullptr);
  // rclcpp installs signal handlers during init; replace them with the
  // runner's minimal async-signal-safe stop flag.
  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);
  std::signal(SIGHUP, OnSignal);
  std::signal(SIGUSR1, OnHandoffSignal);
  ArmIo io(args.state_topic, args.command_topic);
  const auto shutdown = [&io]() {
    io.StopRuntimeMonitoring();
    rclcpp::shutdown();
  };
  ArmSample entry;
  std::string error;
  if (!WaitForFreshState(io, entry, error)) {
    std::cerr << "[vendor-arm SAFETY] " << error << '\n';
    shutdown();
    return 5;
  }
  if (!InHardLimits(entry.q, error, "entry arm state")) {
    std::cerr << "[vendor-arm SAFETY] " << error << '\n';
    shutdown();
    return 5;
  }

  std::cout << "[vendor-arm] prearming while motion_player still owns the "
               "command topic\n";
  std::size_t command_publishers = 0;
  const auto publisher_deadline =
      Clock::now() + kHandoffGraphTimeout;
  while (!g_stop && Clock::now() < publisher_deadline) {
    command_publishers = io.ExistingCommandPublisherCount();
    if (command_publishers != 0) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  if (command_publishers != 1) {
    std::cerr
        << "[vendor-arm SAFETY] REFUSED: prearm requires exactly one "
           "vendor command publisher; graph="
        << command_publishers << '\n';
    shutdown();
    return 6;
  }

  if (!WriteHandoffMarker(args.handoff_ready_file, "READY", error)) {
    std::cerr << "[vendor-arm SAFETY] " << error << '\n';
    shutdown();
    return 6;
  }
  std::cout << "[vendor-arm] PREARMED: cached fresh 14-axis state; no "
               "publisher created; waiting for SIGUSR1 handoff\n"
            << std::flush;

  const auto handoff_signal_deadline =
      Clock::now() + kHandoffSignalTimeout;
  while (!g_stop && !g_handoff_requested &&
         Clock::now() < handoff_signal_deadline) {
    io.SpinSome();
    std::string protocol_error;
    const auto latest = io.LatestState(protocol_error);
    if (!protocol_error.empty()) {
      error = "invalid arm state while prearmed: " + protocol_error;
      break;
    }
    if (latest) entry = *latest;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  if (error.empty() && !g_stop && !g_handoff_requested) {
    error = "timed out waiting 30 s for wrapper handoff signal";
  }
  if (!error.empty() || g_stop) {
    if (!error.empty()) {
      std::cerr << "[vendor-arm SAFETY] " << error << '\n';
    }
    shutdown();
    return 6;
  }

  const auto handoff_signal_time = Clock::now();
  bool graph_clear = false;
  const auto graph_deadline = handoff_signal_time + kHandoffGraphTimeout;
  while (!g_stop && Clock::now() < graph_deadline) {
    io.SpinSome();
    std::string protocol_error;
    const auto latest = io.LatestState(protocol_error);
    if (!protocol_error.empty()) {
      error = "invalid arm state during handoff: " + protocol_error;
      break;
    }
    if (latest) entry = *latest;
    const std::size_t publishers = io.ExistingCommandPublisherCount();
    if (publishers == 0) {
      graph_clear = true;
      break;
    }
    if (publishers > 1) {
      error = "multiple command publishers remained during handoff";
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (error.empty() && !graph_clear) {
    error = "vendor command publisher did not disappear within 500 ms";
  }
  io.DrainCallbacks();
  std::string protocol_error;
  const auto latest = io.LatestState(protocol_error);
  if (error.empty() && !protocol_error.empty()) {
    error = "invalid cached arm state at handoff: " + protocol_error;
  }
  if (latest) entry = *latest;
  const auto cached_state_age = Clock::now() - entry.received;
  if (error.empty() && cached_state_age > kHandoffStateFreshness) {
    error = "cached arm state exceeded 1 s at command handoff";
  }
  if (error.empty() &&
      !InHardLimits(entry.q, error, "cached handoff arm state")) {
    // InHardLimits populated error.
  }
  if (!error.empty() || g_stop) {
    if (!error.empty()) {
      std::cerr << "[vendor-arm SAFETY] " << error << '\n';
    }
    shutdown();
    return 6;
  }
  io.MarkHandoffComplete();

  std::vector<ArmVector> plan;
  double max_prepare_velocity = 0.0;
  if (args.mode != "hold-only" &&
      !BuildPreparePlan(entry.q, trajectory, plan, error,
                        max_prepare_velocity)) {
    std::cerr << "[vendor-arm SAFETY] " << error << '\n';
    shutdown();
    return 7;
  }
  if (args.mode != "hold-only") {
    std::cout << std::fixed << std::setprecision(6)
              << "[vendor-arm] measured->frame0 minimum-jerk=3.000s "
                 "max_velocity="
              << max_prepare_velocity << " rad/s\n"
              << "[vendor-arm] plan ticks=" << plan.size()
              << "; READY will be held until SIGINT/SIGTERM\n";
  } else {
    std::cout << "[vendor-arm] hold-only duration=" << args.hold_seconds
              << "s\n";
  }

  const double cached_state_age_ms =
      std::chrono::duration<double, std::milli>(cached_state_age).count();
  const std::uint64_t handoff_state_sequence = entry.sequence;
  std::cout << std::fixed << std::setprecision(3)
            << "[vendor-arm] handoff graph clear; cached_state_age="
            << cached_state_age_ms << " ms; opening command publisher\n";
  io.OpenPublisher();
  io.StartRuntimeMonitoring();
  ArmVector last_command = entry.q;
  io.Publish(last_command);
  const auto first_publish_time = Clock::now();
  auto next_tick = first_publish_time + kCommandPeriod;

  // This A3 firmware stops arm_joint_state when motion_player is stopped.
  // Re-send the exact cached pose until feedback resumes. No trajectory step
  // is taken during this one-time handoff bridge.
  bool state_reacquired = false;
  const auto state_reacquire_deadline =
      first_publish_time + kStateFreshness;
  while (!g_stop && !state_reacquired) {
    std::this_thread::sleep_until(next_tick);
    if (!CheckRuntimeMonitoring(io, error)) break;
    std::string state_error;
    const auto resumed = io.LatestState(state_error);
    if (!state_error.empty()) {
      error = "invalid arm state while reacquiring feedback: " + state_error;
      break;
    }
    const bool resumed_now =
        resumed && resumed->sequence > handoff_state_sequence &&
        resumed->received >= first_publish_time &&
        Clock::now() - resumed->received <= kStateFreshness;
    io.Publish(last_command);
    const auto publish_finished = Clock::now();
    next_tick = publish_finished + kCommandPeriod;
    state_reacquired = state_reacquired || resumed_now;
    if (!state_reacquired &&
        publish_finished >= state_reacquire_deadline) {
      error = "arm state did not resume within 100 ms after first command";
      break;
    }
  }
  if (!error.empty() || g_stop) {
    if (!error.empty()) {
      std::cerr << "[vendor-arm SAFETY] STOP: " << error << '\n';
    }
    shutdown();
    return error.empty() ? 0 : 8;
  }
  std::cout << "[vendor-arm] arm state resumed after cached-pose takeover\n";

  if (!WriteHandoffMarker(args.handoff_ready_file, "RUNNING", error)) {
    std::cerr << "[vendor-arm SAFETY] STOP: " << error << '\n';
    shutdown();
    return 8;
  }
  // Marker I/O is outside the command loop. Re-send the cached pose and start
  // the 100 Hz schedule from this actual publication.
  io.Publish(last_command);
  next_tick = Clock::now() + kCommandPeriod;

  const auto publish_tick = [&](const ArmVector& command) -> bool {
    std::this_thread::sleep_until(next_tick);
    if (g_stop) return false;
    // The hot path intentionally contains no state/tracking/gap gate. It only
    // observes stop/foreign-command latches and publishes the fixed plan.
    if (!CheckRuntimeMonitoring(io, error)) return false;
    io.Publish(command);
    last_command = command;
    next_tick = Clock::now() + kCommandPeriod;
    return true;
  };

  if (args.mode == "hold-only") {
    const std::size_t ticks = static_cast<std::size_t>(
        std::ceil(args.hold_seconds * kCommandHz));
    for (std::size_t tick = 0;
         tick < ticks && !g_stop; ++tick) {
      if (!publish_tick(entry.q)) break;
    }
  } else {
    for (const ArmVector& command : plan) {
      if (!publish_tick(command)) break;
    }
    if (error.empty() && !g_stop) {
      // Preserve the successful field path's normal 0.5 s READY settling
      // time without reintroducing a feedback/quality failure gate.
      for (std::size_t tick = 0;
           tick < kReadySettleTicks && error.empty() && !g_stop;
           ++tick) {
        if (!publish_tick(last_command)) break;
      }
    }
    if (error.empty() && !g_stop) {
      if (args.mode == "prepare-only") {
        while (!g_stop) {
          if (!error.empty()) break;
          if (!publish_tick(last_command)) break;
        }
      } else if (error.empty() && !g_stop) {
        std::cout
            << "[vendor-arm] READY HOLD: press Space at the physical "
               "ball-release instant\n";
        // Reject rather than queue any Space typed before READY.
        bool discarded_space = false;
        serve_terminal.ConsumeSpace(discarded_space, error);
        bool triggered = false;
        while (error.empty() && !g_stop && !triggered) {
          if (!publish_tick(last_command)) break;
          if (!serve_terminal.ConsumeSpace(triggered, error)) break;
        }

        // A READY command has just been published at physical t=0. Publish
        // another 99 READY ticks, then publish source frame 1848 at +1.000 s.
        // The original stride-two timeline reaches source frame 1860 six
        // command ticks later, so nominal strike remains +1.060 s.
        for (std::size_t tick = 1;
             tick < kTriggerHoldTicks && error.empty() && !g_stop;
             ++tick) {
          if (!publish_tick(last_command)) break;
        }
        for (const ArmVector& source : trajectory.stroke_frames) {
          if (!error.empty() || g_stop) break;
          ArmVector command = source;
          for (std::size_t i = 0; i < command.size(); ++i) {
            if (!kSourceActive[i]) command[i] = entry.q[i];
          }
          if (!publish_tick(command)) break;
        }
        while (error.empty() && !g_stop) {
          if (!publish_tick(last_command)) break;
        }
      }
    }
  }

  if (!error.empty()) {
    std::cerr << "[vendor-arm SAFETY] STOP: " << error << '\n';
  } else {
    std::cout << "[vendor-arm] stopping publisher; agibot_pm and motion "
                 "control were not modified\n";
  }
  shutdown();
  return error.empty() ? 0 : 8;
}

}  // namespace

int main(int argc, char** argv) {
  Arguments args;
  std::string error;
  if (!ParseArguments(argc, argv, args, error)) {
    std::cerr << "ERROR: " << error << '\n';
    PrintUsage(argv[0]);
    return 2;
  }

  Trajectory trajectory;
  if (!LoadTrajectory(args.motion_csv, trajectory, error)) {
    std::cerr << "[vendor-arm SAFETY] offline validation failed: " << error
              << '\n';
    return 3;
  }
  PrintValidation(args, trajectory);

  const bool publish =
      args.allow_publish && args.confirm_real_commands;
  if (!publish) {
    std::cout << "[vendor-arm] validation-only: no ROS node or publisher was "
                 "created\n";
    return 0;
  }

  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);
  std::signal(SIGHUP, OnSignal);
  return RunReal(args, trajectory);
}
