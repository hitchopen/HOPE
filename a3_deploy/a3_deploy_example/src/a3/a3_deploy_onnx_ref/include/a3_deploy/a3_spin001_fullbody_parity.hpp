// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// Strict, transport-free loader for the Spin001 three-segment full-body
// deployment-parity command artifact.  The CSV already contains the five
// vectors consumed by robot_io::RobotCommand; this loader validates and
// scatters them but never derives gains, scales commands, or owns transport.
#pragma once

#include "robot_io/robot_io_backend.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace a3_deploy {

inline constexpr char kA3Spin001FullbodyProfileId[] =
    "a3p_op3_spin001_deploy_parity_v1";
inline constexpr char kA3Spin001FullbodyManifestType[] =
    "a3p_op3_spin001_deploy_parity_segment_manifest_v1";
inline constexpr char kA3Spin001FullbodyStorageOrderId[] =
    "a3_runtime_articulation_joint_order_v1";
inline constexpr char kA3Spin001FullbodyAbiName[] =
    "A3_RUNTIME_ORDER31_50HZ_ROBOTCOMMAND_FIVE_VECTOR_CSV_V1";

inline constexpr std::size_t kA3Spin001FullbodyDof = 31;
inline constexpr std::size_t kA3Spin001FullbodyMetadataColumns = 5;
inline constexpr std::size_t kA3Spin001FullbodyVectorGroups = 5;
inline constexpr std::size_t kA3Spin001FullbodyCsvColumns =
    kA3Spin001FullbodyMetadataColumns +
    kA3Spin001FullbodyVectorGroups * kA3Spin001FullbodyDof;
inline constexpr std::size_t kA3Spin001FullbodySegmentCount = 3;
inline constexpr std::size_t kA3Spin001FullbodyTotalFrames = 485;
inline constexpr double kA3Spin001FullbodyHz = 50.0;
inline constexpr double kA3Spin001FullbodyDtS = 0.02;
inline constexpr int kA3Spin001FullbodyHardwareBlockedExit = 77;

enum class A3Spin001FullbodySegmentKind : std::uint8_t {
  kEntry = 0,
  kTimedServe = 1,
  kRecovery = 2,
};

const char* A3Spin001FullbodySegmentName(
    A3Spin001FullbodySegmentKind kind) noexcept;
std::string_view A3Spin001FullbodyArtifactId(
    A3Spin001FullbodySegmentKind kind) noexcept;
std::size_t A3Spin001FullbodyExpectedFrameCount(
    A3Spin001FullbodySegmentKind kind) noexcept;

// Source/storage order is the exact Isaac/PhysX articulation order emitted by
// the producer.  RobotCommand vectors use the backend order returned by
// robot_io::MakeA3Layout31(); the loader scatters by this fixed bijection.
const std::array<std::string_view, kA3Spin001FullbodyDof>&
A3Spin001FullbodyStorageJointNames() noexcept;
const std::array<double, kA3Spin001FullbodyDof>&
A3Spin001FullbodyExpectedKpSdk() noexcept;
const std::array<double, kA3Spin001FullbodyDof>&
A3Spin001FullbodyExpectedKdSdk() noexcept;
const std::array<double, kA3Spin001FullbodyDof>&
A3Spin001FullbodyCanonicalStandQSdk() noexcept;

struct A3Spin001FullbodyFrame {
  std::size_t frame_index{0};
  double time_s{0.0};
  std::string phase;
  std::int64_t source_fullbody_global_frame{0};
  std::uint16_t event_mask{0};
  robot_io::RobotCommand command;
};

struct A3Spin001FullbodySegmentStats {
  std::size_t frames{0};
  double duration_s{0.0};
  double full_tick_occupancy_s{0.0};
  double max_q_step_rad{0.0};
  std::size_t max_q_step_frame{0};
  int max_q_step_sdk_index{-1};
  double max_abs_dq_des_rad_s{0.0};
  std::size_t max_abs_dq_frame{0};
  int max_abs_dq_sdk_index{-1};
};

class A3Spin001FullbodySegment final {
 public:
  // expected_csv_sha256 must be the lowercase digest from the independently
  // validated segment manifest.  Load is transactional: a rejection leaves
  // this object empty and exposes no partially validated RobotCommand.
  bool LoadCsv(const std::string& path,
               A3Spin001FullbodySegmentKind kind,
               const std::string& expected_csv_sha256,
               std::string& error);

  const A3Spin001FullbodyFrame& At(std::size_t frame) const noexcept;
  A3Spin001FullbodySegmentKind kind() const noexcept { return kind_; }
  const std::string& csv_sha256() const noexcept { return csv_sha256_; }
  const A3Spin001FullbodySegmentStats& stats() const noexcept {
    return stats_;
  }
  std::size_t size() const noexcept { return frames_.size(); }
  bool empty() const noexcept { return frames_.empty(); }

 private:
  A3Spin001FullbodySegmentKind kind_{
      A3Spin001FullbodySegmentKind::kEntry};
  std::string csv_sha256_;
  A3Spin001FullbodySegmentStats stats_{};
  A3Spin001FullbodyFrame fallback_{};
  std::vector<A3Spin001FullbodyFrame> frames_;
};

struct A3Spin001FullbodyProgramStats {
  std::size_t frames{0};
  double full_tick_occupancy_s{0.0};
  double max_join_dq_jump_rad_s{0.0};
  std::size_t max_join_index{0};
  int max_join_dq_sdk_index{-1};
};

struct A3Spin001FullbodySegmentContract {
  A3Spin001FullbodySegmentKind kind{
      A3Spin001FullbodySegmentKind::kEntry};
  std::string artifact_id;
  std::string manifest_path;
  std::string manifest_sha256;
  std::string csv_path;
  std::string csv_sha256;
  std::string profile_path;
  std::string profile_sha256;
  std::string producer_path;
  std::string producer_sha256;
  std::size_t frame_count{0};
  double timestamp_span_s{0.0};
  double full_tick_occupancy_s{0.0};
  bool hardware_authorized{false};
};

class A3Spin001FullbodyProgram final {
 public:
  using Paths = std::array<std::string, kA3Spin001FullbodySegmentCount>;
  using Sha256s = std::array<std::string, kA3Spin001FullbodySegmentCount>;

  // Segment array order is ENTRY, TIMED_SERVE, RECOVERY.  q_des, tau_ff, kp,
  // and kd must be exactly equal at both joins.  dq_des is intentionally
  // segment-local (one-sided at endpoints), so its finite jump is measured
  // rather than hidden behind a tolerance.  The sole exception is exact-zero
  // ARM14 dq_des on the final recovery hold.  Recovery must terminate at the
  // exact canonical stand pose.
  bool Load(const Paths& csv_paths,
            const Sha256s& expected_csv_sha256,
            std::string& error);

  // Strict offline entry point.  Each manifest itself is pinned by a caller-
  // supplied SHA-256, then binds the exact profile, producer, dependencies,
  // CSV path and CSV digest below repository_root.  It rejects every publish
  // capability and never constructs a robot backend or transport object.
  bool LoadContentBound(const std::string& repository_root,
                        const Paths& manifest_paths,
                        const Sha256s& expected_manifest_sha256,
                        std::string& error);

  const A3Spin001FullbodySegment& Segment(
      A3Spin001FullbodySegmentKind kind) const noexcept;
  const A3Spin001FullbodyFrame& At(std::size_t global_frame) const noexcept;
  const A3Spin001FullbodyProgramStats& stats() const noexcept {
    return stats_;
  }
  const A3Spin001FullbodySegmentContract& Contract(
      A3Spin001FullbodySegmentKind kind) const noexcept;
  std::size_t size() const noexcept { return stats_.frames; }
  bool empty() const noexcept { return stats_.frames == 0; }

 private:
  std::array<A3Spin001FullbodySegment,
             kA3Spin001FullbodySegmentCount>
      segments_{};
  std::array<A3Spin001FullbodySegmentContract,
             kA3Spin001FullbodySegmentCount>
      contracts_{};
  A3Spin001FullbodyProgramStats stats_{};
  A3Spin001FullbodyFrame fallback_{};
};

// Pure argv preflight used by the offline runner before path normalization,
// existence checks, manifest reads, ROS/AimRT construction, or any other I/O.
// It recognizes both direct real-command flags and --mode[=] real aliases.
bool A3Spin001FullbodyRequestsRealCommands(
    const std::vector<std::string>& arguments) noexcept;

}  // namespace a3_deploy
