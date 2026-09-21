// Copyright (c) 2026, AgiBot Inc. All rights reserved.
#include "a3_deploy/a3_spin001_fullbody_parity.hpp"

#include "a3_deploy/a3_joint_limits.hpp"
#include "a3_pingpong/pp_sha256.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace a3_deploy {
namespace {

using SegmentKind = A3Spin001FullbodySegmentKind;
namespace fs = std::filesystem;

constexpr std::array<std::string_view, kA3Spin001FullbodyDof>
    kStorageJointNames = {
        "left_hip_pitch_joint",       "right_hip_pitch_joint",
        "waist_yaw_joint",            "left_hip_roll_joint",
        "right_hip_roll_joint",       "waist_roll_joint",
        "left_hip_yaw_joint",         "right_hip_yaw_joint",
        "waist_pitch_joint",          "left_knee_joint",
        "right_knee_joint",           "head_yaw_joint",
        "left_shoulder_pitch_joint",  "right_shoulder_pitch_joint",
        "left_ankle_pitch_joint",     "right_ankle_pitch_joint",
        "head_pitch_joint",           "left_shoulder_roll_joint",
        "right_shoulder_roll_joint",  "left_ankle_roll_joint",
        "right_ankle_roll_joint",     "left_shoulder_yaw_joint",
        "right_shoulder_yaw_joint",   "left_elbow_joint",
        "right_elbow_joint",          "left_wrist_roll_joint",
        "right_wrist_roll_joint",     "left_wrist_pitch_joint",
        "right_wrist_pitch_joint",    "left_wrist_yaw_joint",
        "right_wrist_yaw_joint"};

// storage index -> A3AimrtBackend RobotCommand slot.  This is the exact
// name-bijection between kStorageJointNames and MakeA3Layout31().
constexpr std::array<int, kA3Spin001FullbodyDof> kStorageToSdk = {
    19, 25, 0,  20, 26, 1,  21, 27, 2,  22, 28, 3, 5, 12, 23, 29,
    4,  6,  13, 24, 30, 7,  14, 8,  15, 9, 16, 10, 17, 11, 18};

// These are validation values, not runtime scale factors.  The producer has
// already materialized them in every CSV row:
//   waist + non-balanced leg joints: official PD_STAND 1.0x
//   bilateral hip_pitch/knee/ankle_pitch: official PD_STAND 1.5x
//   ARM14: current a3_kps/a3_kds 1.5x
//   head: explicit deployment fill 40/2
constexpr std::array<double, kA3Spin001FullbodyDof> kExpectedKpSdk = {
    400.0, 500.0, 500.0, 40.0, 40.0,
    60.0,  60.0,  45.0,  45.0, 45.0, 30.0, 30.0,
    60.0,  60.0,  45.0,  45.0, 45.0, 30.0, 30.0,
    2250.0, 400.0, 300.0, 3000.0, 750.0, 500.0,
    2250.0, 400.0, 300.0, 3000.0, 750.0, 500.0};

constexpr std::array<double, kA3Spin001FullbodyDof> kExpectedKdSdk = {
    4.0, 4.0, 4.0, 2.0, 2.0,
    4.5, 4.5, 3.0, 3.0, 3.0, 3.0, 3.0,
    4.5, 4.5, 3.0, 3.0, 3.0, 3.0, 3.0,
    12.0, 7.0, 7.0, 12.0, 7.5, 5.0,
    12.0, 7.0, 7.0, 12.0, 7.5, 5.0};

constexpr std::array<double, kA3Spin001FullbodyDof> kCanonicalStandQSdk = {
    0.0, 0.0, 0.0, 0.0, 0.0,
    0.3, 0.12, 0.0, 0.8, 0.0, 0.0, 0.0,
    0.3, -0.12, 0.0, 0.8, 0.0, 0.0, 0.0,
    -0.1311, 0.0056, -0.0348, 0.2468, -0.1204, -0.0078,
    -0.1311, -0.0056, 0.0348, 0.2468, -0.1204, 0.0078};

constexpr std::array<std::size_t, kA3Spin001FullbodySegmentCount>
    kFrameCounts = {250, 84, 151};
constexpr std::array<std::string_view, kA3Spin001FullbodySegmentCount>
    kArtifactIds = {
        "a3p_op3_spin001_deploy_parity_entry_v1",
        "a3p_op3_spin001_deploy_parity_timed_serve_v1",
        "a3p_op3_spin001_deploy_parity_recovery_v1"};
constexpr std::array<std::string_view, kA3Spin001FullbodyVectorGroups>
    kVectorPrefixes = {"q_des_rad", "dq_des_rad_s", "tau_ff_nm",
                       "kp_nm_per_rad", "kd_nm_s_per_rad"};
constexpr std::array<std::string_view, kA3Spin001FullbodyMetadataColumns>
    kMetadataColumns = {"frame_index", "time_s", "phase",
                        "source_fullbody_global_frame", "event_mask"};
constexpr std::array<std::string_view, kA3Spin001FullbodyVectorGroups>
    kVectorGroupNames = {"q_des", "dq_des", "tau_ff", "kp", "kd"};

constexpr double kNumericTolerance = 1.0e-12;
constexpr double kDqConsistencyTolerance = 1.0e-10;
constexpr double kArmMaxStepRad = 0.12;

std::size_t KindIndex(SegmentKind kind) noexcept {
  return static_cast<std::size_t>(kind);
}

bool ValidKind(SegmentKind kind) noexcept {
  return KindIndex(kind) < kA3Spin001FullbodySegmentCount;
}

bool IsLowerHexSha256(const std::string& value) noexcept;
std::string ExpectedHeader(std::string_view prefix,
                           std::string_view joint);

std::string ReadBytes(const std::string& path, std::string& error) {
  std::error_code status_error;
  const fs::file_status status = fs::symlink_status(fs::path(path), status_error);
  if (status_error || fs::is_symlink(status) || !fs::is_regular_file(status)) {
    error = "Spin001 content-bound input is missing, non-regular, or a symlink: " +
            path;
    return {};
  }
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    error = "cannot open Spin001 deployment-parity CSV: " + path;
    return {};
  }
  std::string bytes((std::istreambuf_iterator<char>(input)),
                    std::istreambuf_iterator<char>());
  if (input.bad()) {
    error = "cannot read Spin001 deployment-parity CSV: " + path;
    return {};
  }
  return bytes;
}

template <typename T>
T Required(const YAML::Node& parent,
           const char* key,
           const std::string& where) {
  if (!parent.IsMap()) throw std::runtime_error(where + " must be a map");
  const YAML::Node value = parent[key];
  if (!value || !value.IsScalar()) {
    throw std::runtime_error(where + "." + key + " must be a scalar");
  }
  try {
    return value.as<T>();
  } catch (const YAML::Exception&) {
    throw std::runtime_error(where + "." + key + " has the wrong type");
  }
}

YAML::Node RequiredMap(const YAML::Node& parent,
                       const char* key,
                       const std::string& where) {
  if (!parent.IsMap()) throw std::runtime_error(where + " must be a map");
  const YAML::Node value = parent[key];
  if (!value || !value.IsMap()) {
    throw std::runtime_error(where + "." + key + " must be a map");
  }
  return value;
}

YAML::Node RequiredSequence(const YAML::Node& parent,
                            const char* key,
                            const std::string& where) {
  if (!parent.IsMap()) throw std::runtime_error(where + " must be a map");
  const YAML::Node value = parent[key];
  if (!value || !value.IsSequence()) {
    throw std::runtime_error(where + "." + key + " must be a sequence");
  }
  return value;
}

template <typename T>
void RequireEqual(const T& actual,
                  const T& expected,
                  const std::string& where) {
  if (actual != expected) throw std::runtime_error(where + " mismatch");
}

void RequireFalse(const YAML::Node& parent,
                  const char* key,
                  const std::string& where) {
  if (Required<bool>(parent, key, where)) {
    throw std::runtime_error(where + "." + key + " must remain false");
  }
}

void RequireTrue(const YAML::Node& parent,
                 const char* key,
                 const std::string& where) {
  if (!Required<bool>(parent, key, where)) {
    throw std::runtime_error(where + "." + key + " must be true");
  }
}

template <std::size_t N>
void RequireStringSequence(
    const YAML::Node& parent,
    const char* key,
    const std::array<std::string_view, N>& expected,
    const std::string& where) {
  const YAML::Node values = RequiredSequence(parent, key, where);
  if (values.size() != expected.size()) {
    throw std::runtime_error(where + "." + key + " length mismatch");
  }
  for (std::size_t index = 0; index < expected.size(); ++index) {
    RequireEqual(values[index].as<std::string>(), std::string(expected[index]),
                 where + "." + key + "[" + std::to_string(index) + "]");
  }
}

fs::path ResolveRepoFile(const fs::path& repository_root,
                         const std::string& raw,
                         const std::string& where) {
  const fs::path relative(raw);
  if (raw.empty() || relative.is_absolute()) {
    throw std::runtime_error(where + " must be a nonempty repository-relative path");
  }
  const fs::path normalized = relative.lexically_normal();
  for (const fs::path& component : normalized) {
    if (component == "..") {
      throw std::runtime_error(where + " escapes repository_root");
    }
  }

  std::error_code canonical_error;
  const fs::path root = fs::canonical(repository_root, canonical_error);
  if (canonical_error || !fs::is_directory(root)) {
    throw std::runtime_error("repository_root is not an existing directory");
  }
  fs::path current = root;
  for (const fs::path& component : normalized) {
    if (component == ".") continue;
    current /= component;
    std::error_code status_error;
    const fs::file_status status = fs::symlink_status(current, status_error);
    if (!status_error && fs::is_symlink(status)) {
      throw std::runtime_error(where + " traverses a symlink");
    }
  }
  return current;
}

std::string HashRegularFile(const fs::path& path, const std::string& where) {
  std::string error;
  const std::string bytes = ReadBytes(path.string(), error);
  if (!error.empty()) throw std::runtime_error(where + ": " + error);
  return a3_pingpong::PpSha256::String(bytes);
}

struct DependencyBinding {
  std::string path;
  std::string sha256;
};

using DependencyBindings = std::map<std::string, DependencyBinding>;

DependencyBindings ValidateDependencies(const YAML::Node& dependencies,
                                        const fs::path& repository_root,
                                        const std::string& where) {
  if (!dependencies.IsMap() || dependencies.size() == 0) {
    throw std::runtime_error(where + " must be a nonempty map");
  }
  DependencyBindings result;
  for (const auto& item : dependencies) {
    const std::string name = item.first.as<std::string>();
    const YAML::Node binding = item.second;
    const std::string path = Required<std::string>(binding, "path", where + "." + name);
    const std::string sha = Required<std::string>(binding, "sha256", where + "." + name);
    if (!IsLowerHexSha256(sha)) {
      throw std::runtime_error(where + "." + name + ".sha256 is malformed");
    }
    const fs::path resolved = ResolveRepoFile(repository_root, path,
                                              where + "." + name + ".path");
    RequireEqual(HashRegularFile(resolved, where + "." + name), sha,
                 where + "." + name + ".sha256");
    if (!result.emplace(name, DependencyBinding{path, sha}).second) {
      throw std::runtime_error(where + " contains a duplicate dependency");
    }
  }
  return result;
}

void RequireSameDependencies(const DependencyBindings& left,
                             const DependencyBindings& right) {
  if (left.size() != right.size()) {
    throw std::runtime_error("manifest/profile dependency set mismatch");
  }
  for (const auto& [name, binding] : left) {
    const auto found = right.find(name);
    if (found == right.end() || found->second.path != binding.path ||
        found->second.sha256 != binding.sha256) {
      throw std::runtime_error("manifest/profile dependency binding mismatch: " +
                               name);
    }
  }
}

DependencyBindings ValidateProfile(const std::string& bytes,
                                   const fs::path& repository_root) {
  const YAML::Node profile = YAML::Load(bytes);
  if (!profile.IsMap()) throw std::runtime_error("profile root must be a map");
  RequireEqual(Required<int>(profile, "schema_version", "profile"), 1,
               "profile.schema_version");
  RequireEqual(Required<std::string>(profile, "manifest_type", "profile"),
               std::string("a3p_op3_spin001_deploy_parity_profile_v1"),
               "profile.manifest_type");
  RequireEqual(Required<std::string>(profile, "profile_id", "profile"),
               std::string(kA3Spin001FullbodyProfileId),
               "profile.profile_id");

  const YAML::Node wire = RequiredMap(profile, "wire_abi", "profile");
  RequireEqual(Required<double>(wire, "command_hz", "profile.wire_abi"),
               kA3Spin001FullbodyHz, "profile.wire_abi.command_hz");
  RequireEqual(Required<std::size_t>(wire, "joint_count", "profile.wire_abi"),
               kA3Spin001FullbodyDof, "profile.wire_abi.joint_count");
  RequireEqual(Required<std::size_t>(wire, "column_count", "profile.wire_abi"),
               kA3Spin001FullbodyCsvColumns,
               "profile.wire_abi.column_count");
  RequireStringSequence(wire, "metadata_columns", kMetadataColumns,
                        "profile.wire_abi");
  RequireStringSequence(wire, "vector_group_order", kVectorGroupNames,
                        "profile.wire_abi");
  RequireStringSequence(wire, "csv_vector_prefixes", kVectorPrefixes,
                        "profile.wire_abi");
  RequireTrue(wire, "all_five_robot_command_vectors_explicit_per_row",
              "profile.wire_abi");
  RequireTrue(wire, "runtime_must_not_scale_or_override_csv_vectors",
              "profile.wire_abi");

  const YAML::Node materialization =
      RequiredMap(profile, "materialization_contract", "profile");
  const YAML::Node materialized_dq = RequiredMap(
      materialization, "dq_des", "profile.materialization_contract");
  RequireEqual(
      Required<std::string>(materialized_dq, "recovery_terminal_override",
                            "profile.materialization_contract.dq_des"),
      std::string("arm14_dq_des_exact_zero_at_final_row"),
      "profile.materialization_contract.dq_des.recovery_terminal_override");
  RequireFalse(materialization, "hidden_gain_override_permitted",
               "profile.materialization_contract");
  RequireFalse(materialization, "runtime_gain_rescaling_permitted",
               "profile.materialization_contract");

  const YAML::Node capabilities =
      RequiredMap(profile, "capabilities", "profile");
  RequireTrue(capabilities, "full_robot_command_wire_payload_materialized",
              "profile.capabilities");
  RequireTrue(capabilities, "runtime_order31_materialized",
              "profile.capabilities");
  RequireTrue(capabilities,
              "position_velocity_feedforward_and_gains_explicit",
              "profile.capabilities");
  RequireTrue(capabilities, "runtime_loader_implemented",
              "profile.capabilities");
  RequireFalse(capabilities, "hardware_authorized", "profile.capabilities");
  RequireEqual(Required<int>(capabilities, "commands_sent", "profile.capabilities"),
               0, "profile.capabilities.commands_sent");

  return ValidateDependencies(RequiredMap(profile, "dependencies", "profile"),
                              repository_root, "profile.dependencies");
}

struct ParsedManifest {
  A3Spin001FullbodySegmentContract contract;
  std::array<std::string, kA3Spin001FullbodySegmentCount> output_paths;
  std::array<std::string, kA3Spin001FullbodySegmentCount> output_sha256;
};

ParsedManifest ParseManifest(const fs::path& repository_root,
                             const std::string& manifest_path,
                             const std::string& expected_manifest_sha256,
                             SegmentKind kind) {
  if (!IsLowerHexSha256(expected_manifest_sha256)) {
    throw std::runtime_error("expected manifest SHA-256 is malformed");
  }
  std::string read_error;
  const std::string bytes = ReadBytes(manifest_path, read_error);
  if (!read_error.empty()) throw std::runtime_error(read_error);
  const std::string manifest_sha = a3_pingpong::PpSha256::String(bytes);
  RequireEqual(manifest_sha, expected_manifest_sha256,
               "manifest SHA-256");

  const YAML::Node root = YAML::Load(bytes);
  if (!root.IsMap()) throw std::runtime_error("manifest root must be a map");
  RequireEqual(Required<int>(root, "schema_version", "manifest"), 1,
               "manifest.schema_version");
  RequireEqual(Required<std::string>(root, "manifest_type", "manifest"),
               std::string(kA3Spin001FullbodyManifestType),
               "manifest.manifest_type");
  RequireEqual(Required<std::string>(root, "profile_id", "manifest"),
               std::string(kA3Spin001FullbodyProfileId),
               "manifest.profile_id");
  RequireEqual(Required<std::string>(root, "artifact_id", "manifest"),
               std::string(A3Spin001FullbodyArtifactId(kind)),
               "manifest.artifact_id");
  RequireEqual(Required<std::string>(root, "segment_name", "manifest"),
               std::string(A3Spin001FullbodySegmentName(kind)),
               "manifest.segment_name");
  RequireFalse(root, "hardware_authorized", "manifest");

  const YAML::Node hashes = RequiredMap(root, "content_hashes", "manifest");
  const std::string csv_sha =
      Required<std::string>(hashes, "csv_sha256", "manifest.content_hashes");
  const std::string config_sha =
      Required<std::string>(hashes, "config_sha256", "manifest.content_hashes");
  const std::string producer_sha =
      Required<std::string>(hashes, "producer_sha256", "manifest.content_hashes");
  if (!IsLowerHexSha256(csv_sha) || !IsLowerHexSha256(config_sha) ||
      !IsLowerHexSha256(producer_sha)) {
    throw std::runtime_error("manifest content hash is malformed");
  }

  const YAML::Node profile_binding = RequiredMap(root, "profile", "manifest");
  const std::string profile_path = Required<std::string>(
      profile_binding, "path", "manifest.profile");
  RequireEqual(Required<std::string>(profile_binding, "sha256", "manifest.profile"),
               config_sha, "manifest.profile.sha256");
  const fs::path resolved_profile = ResolveRepoFile(
      repository_root, profile_path, "manifest.profile.path");
  std::string profile_read_error;
  const std::string profile_bytes =
      ReadBytes(resolved_profile.string(), profile_read_error);
  if (!profile_read_error.empty()) throw std::runtime_error(profile_read_error);
  RequireEqual(a3_pingpong::PpSha256::String(profile_bytes), config_sha,
               "manifest profile file SHA-256");
  const DependencyBindings profile_dependencies =
      ValidateProfile(profile_bytes, repository_root);

  const YAML::Node producer_binding = RequiredMap(root, "producer", "manifest");
  const std::string producer_path = Required<std::string>(
      producer_binding, "path", "manifest.producer");
  RequireEqual(Required<std::string>(producer_binding, "sha256", "manifest.producer"),
               producer_sha, "manifest.producer.sha256");
  const fs::path resolved_producer = ResolveRepoFile(
      repository_root, producer_path, "manifest.producer.path");
  RequireEqual(HashRegularFile(resolved_producer, "manifest producer"),
               producer_sha, "manifest producer file SHA-256");

  const DependencyBindings manifest_dependencies = ValidateDependencies(
      RequiredMap(root, "dependencies", "manifest"), repository_root,
      "manifest.dependencies");
  RequireSameDependencies(profile_dependencies, manifest_dependencies);

  const YAML::Node abi = RequiredMap(root, "abi", "manifest");
  RequireEqual(Required<std::string>(abi, "name", "manifest.abi"),
               std::string(kA3Spin001FullbodyAbiName), "manifest.abi.name");
  RequireEqual(Required<double>(abi, "command_hz", "manifest.abi"),
               kA3Spin001FullbodyHz, "manifest.abi.command_hz");
  RequireEqual(Required<std::size_t>(abi, "joint_count", "manifest.abi"),
               kA3Spin001FullbodyDof, "manifest.abi.joint_count");
  RequireEqual(Required<std::string>(abi, "storage_order_id", "manifest.abi"),
               std::string(kA3Spin001FullbodyStorageOrderId),
               "manifest.abi.storage_order_id");
  RequireStringSequence(abi, "storage_joint_names", kStorageJointNames,
                        "manifest.abi");
  RequireStringSequence(abi, "metadata_columns", kMetadataColumns,
                        "manifest.abi");
  RequireStringSequence(abi, "vector_group_order", kVectorGroupNames,
                        "manifest.abi");
  RequireStringSequence(abi, "csv_vector_prefixes", kVectorPrefixes,
                        "manifest.abi");
  RequireEqual(Required<std::size_t>(abi, "csv_column_count", "manifest.abi"),
               kA3Spin001FullbodyCsvColumns,
               "manifest.abi.csv_column_count");
  RequireTrue(abi, "all_five_vectors_materialized_per_row", "manifest.abi");
  const YAML::Node csv_columns =
      RequiredSequence(abi, "csv_columns", "manifest.abi");
  if (csv_columns.size() != kA3Spin001FullbodyCsvColumns) {
    throw std::runtime_error("manifest.abi.csv_columns length mismatch");
  }
  for (std::size_t column = 0; column < kMetadataColumns.size(); ++column) {
    RequireEqual(csv_columns[column].as<std::string>(),
                 std::string(kMetadataColumns[column]),
                 "manifest.abi.csv_columns metadata");
  }
  for (std::size_t group = 0; group < kVectorPrefixes.size(); ++group) {
    for (std::size_t joint = 0; joint < kStorageJointNames.size(); ++joint) {
      const std::size_t column = kA3Spin001FullbodyMetadataColumns +
                                 group * kA3Spin001FullbodyDof + joint;
      RequireEqual(csv_columns[column].as<std::string>(),
                   ExpectedHeader(kVectorPrefixes[group],
                                  kStorageJointNames[joint]),
                   "manifest.abi.csv_columns vector");
    }
  }

  const YAML::Node trajectory = RequiredMap(root, "trajectory", "manifest");
  const YAML::Node output = RequiredMap(trajectory, "output", "manifest.trajectory");
  const std::string csv_path =
      Required<std::string>(output, "path", "manifest.trajectory.output");
  RequireEqual(Required<std::string>(output, "sha256", "manifest.trajectory.output"),
               csv_sha, "manifest.trajectory.output.sha256");
  RequireEqual(Required<std::size_t>(output, "rows_excluding_header",
                                     "manifest.trajectory.output"),
               A3Spin001FullbodyExpectedFrameCount(kind),
               "manifest.trajectory.output.rows_excluding_header");
  RequireEqual(Required<std::size_t>(output, "columns", "manifest.trajectory.output"),
               kA3Spin001FullbodyCsvColumns,
               "manifest.trajectory.output.columns");
  const std::size_t frames = A3Spin001FullbodyExpectedFrameCount(kind);
  RequireEqual(Required<std::size_t>(trajectory, "frame_count", "manifest.trajectory"),
               frames, "manifest.trajectory.frame_count");
  RequireEqual(Required<std::size_t>(trajectory, "first_frame", "manifest.trajectory"),
               std::size_t{0}, "manifest.trajectory.first_frame");
  RequireEqual(Required<std::size_t>(trajectory, "last_frame", "manifest.trajectory"),
               frames - 1, "manifest.trajectory.last_frame");
  const double expected_span = static_cast<double>(frames - 1) /
                               kA3Spin001FullbodyHz;
  const double expected_occupancy = static_cast<double>(frames) /
                                    kA3Spin001FullbodyHz;
  RequireEqual(Required<double>(trajectory, "timestamp_span_s", "manifest.trajectory"),
               expected_span, "manifest.trajectory.timestamp_span_s");
  RequireEqual(Required<double>(trajectory, "full_tick_occupancy_s",
                                "manifest.trajectory"),
               expected_occupancy,
               "manifest.trajectory.full_tick_occupancy_s");

  ParsedManifest parsed;
  const YAML::Node outputs = RequiredMap(
      RequiredMap(root, "split_set", "manifest"), "outputs",
      "manifest.split_set");
  for (std::size_t index = 0; index < kA3Spin001FullbodySegmentCount; ++index) {
    const SegmentKind output_kind = static_cast<SegmentKind>(index);
    const std::string name = A3Spin001FullbodySegmentName(output_kind);
    const YAML::Node binding = RequiredMap(outputs, name.c_str(),
                                           "manifest.split_set.outputs");
    RequireEqual(Required<std::string>(binding, "artifact_id",
                                       "manifest.split_set.outputs." + name),
                 std::string(A3Spin001FullbodyArtifactId(output_kind)),
                 "manifest.split_set.outputs." + name + ".artifact_id");
    parsed.output_paths[index] = Required<std::string>(
        binding, "csv_path", "manifest.split_set.outputs." + name);
    parsed.output_sha256[index] = Required<std::string>(
        binding, "csv_sha256", "manifest.split_set.outputs." + name);
    if (!IsLowerHexSha256(parsed.output_sha256[index])) {
      throw std::runtime_error("manifest split output SHA-256 is malformed");
    }
    RequireEqual(Required<std::size_t>(binding, "frame_count",
                                       "manifest.split_set.outputs." + name),
                 A3Spin001FullbodyExpectedFrameCount(output_kind),
                 "manifest.split_set output frame_count");
  }
  RequireEqual(parsed.output_paths[KindIndex(kind)], csv_path,
               "manifest own split/output CSV path");
  RequireEqual(parsed.output_sha256[KindIndex(kind)], csv_sha,
               "manifest own split/output CSV SHA-256");

  const YAML::Node continuity = RequiredMap(
      RequiredMap(root, "split_set", "manifest"), "continuity",
      "manifest.split_set");
  const YAML::Node recovery_terminal = RequiredMap(
      continuity, "recovery_terminal", "manifest.split_set.continuity");
  RequireTrue(recovery_terminal, "terminal_dq_des_exact_zero",
              "manifest.split_set.continuity.recovery_terminal");

  const YAML::Node controller =
      RequiredMap(root, "controller_payload", "manifest");
  RequireTrue(controller, "all_fields_explicit_in_csv",
              "manifest.controller_payload");
  RequireTrue(controller, "runtime_must_not_scale_or_override_csv_vectors",
              "manifest.controller_payload");
  RequireFalse(controller, "hidden_gain_override_permitted",
               "manifest.controller_payload");
  const YAML::Node controller_dq = RequiredMap(
      controller, "dq_des", "manifest.controller_payload");
  const bool expected_terminal_override = kind == SegmentKind::kRecovery;
  RequireEqual(
      Required<bool>(controller_dq, "recovery_terminal_zero_override",
                     "manifest.controller_payload.dq_des"),
      expected_terminal_override,
      "manifest.controller_payload.dq_des.recovery_terminal_zero_override");
  RequireEqual(
      Required<std::string>(controller_dq, "last_row_rule",
                            "manifest.controller_payload.dq_des"),
      expected_terminal_override
          ? std::string("terminal_zero_override_exact_zero")
          : std::string("(q[-1]-q[-2])/0.02"),
      "manifest.controller_payload.dq_des.last_row_rule");
  const YAML::Node runtime =
      RequiredMap(root, "runtime_compatibility", "manifest");
  RequireTrue(runtime, "runtime_loader_implemented",
              "manifest.runtime_compatibility");
  RequireFalse(runtime, "existing_q_only_loader_compatible",
               "manifest.runtime_compatibility");
  RequireTrue(runtime, "requires_exact_five_vector_31d_loader",
              "manifest.runtime_compatibility");
  RequireTrue(runtime, "loader_must_verify_csv_config_and_producer_hashes",
              "manifest.runtime_compatibility");
  RequireTrue(runtime, "loader_must_not_apply_hidden_gain_scaling",
              "manifest.runtime_compatibility");

  const YAML::Node capabilities =
      RequiredMap(root, "capabilities", "manifest");
  RequireTrue(capabilities, "runtime_loader_implemented",
              "manifest.capabilities");
  RequireTrue(capabilities, "full_robot_command_wire_payload_materialized",
              "manifest.capabilities");
  RequireTrue(capabilities, "strict_named_runtime_order31_mapping_validated",
              "manifest.capabilities");
  RequireFalse(capabilities, "hardware_authorized", "manifest.capabilities");
  RequireEqual(Required<int>(capabilities, "commands_sent", "manifest.capabilities"),
               0, "manifest.capabilities.commands_sent");
  const YAML::Node safety = RequiredMap(root, "safety_boundary", "manifest");
  RequireFalse(safety, "hardware_authorized", "manifest.safety_boundary");
  RequireFalse(safety, "body_publish_permitted", "manifest.safety_boundary");
  RequireFalse(safety, "arm_publish_permitted", "manifest.safety_boundary");
  RequireFalse(safety, "gripper_publish_permitted", "manifest.safety_boundary");
  RequireEqual(Required<int>(safety, "commands_sent_by_this_tool",
                             "manifest.safety_boundary"),
               0, "manifest.safety_boundary.commands_sent_by_this_tool");

  const fs::path resolved_csv = ResolveRepoFile(
      repository_root, csv_path, "manifest.trajectory.output.path");
  parsed.contract.kind = kind;
  parsed.contract.artifact_id = std::string(A3Spin001FullbodyArtifactId(kind));
  parsed.contract.manifest_path = manifest_path;
  parsed.contract.manifest_sha256 = manifest_sha;
  parsed.contract.csv_path = resolved_csv.string();
  parsed.contract.csv_sha256 = csv_sha;
  parsed.contract.profile_path = resolved_profile.string();
  parsed.contract.profile_sha256 = config_sha;
  parsed.contract.producer_path = resolved_producer.string();
  parsed.contract.producer_sha256 = producer_sha;
  parsed.contract.frame_count = frames;
  parsed.contract.timestamp_span_s = expected_span;
  parsed.contract.full_tick_occupancy_s = expected_occupancy;
  parsed.contract.hardware_authorized = false;
  return parsed;
}

std::vector<std::string> SplitCsv(const std::string& line) {
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

bool IsLowerHexSha256(const std::string& value) noexcept {
  return value.size() == 64 &&
         std::all_of(value.begin(), value.end(), [](char character) {
           return (character >= '0' && character <= '9') ||
                  (character >= 'a' && character <= 'f');
         });
}

bool ParseFiniteDouble(const std::string& text, double& value) noexcept {
  if (text.empty()) return false;
  bool saw_digit = false;
  for (const char character : text) {
    if (character >= '0' && character <= '9') {
      saw_digit = true;
      continue;
    }
    if (character != '+' && character != '-' && character != '.' &&
        character != 'e' && character != 'E') {
      return false;
    }
  }
  if (!saw_digit) return false;
  errno = 0;
  char* end = nullptr;
  value = std::strtod(text.c_str(), &end);
  return errno == 0 && end == text.c_str() + text.size() &&
         std::isfinite(value);
}

bool ParseCanonicalUnsigned(const std::string& text,
                            std::uint64_t& value) noexcept {
  if (text.empty() ||
      !std::all_of(text.begin(), text.end(), [](char character) {
        return character >= '0' && character <= '9';
      })) {
    return false;
  }
  errno = 0;
  char* end = nullptr;
  const unsigned long long parsed = std::strtoull(text.c_str(), &end, 10);
  if (errno != 0 || end != text.c_str() + text.size()) return false;
  value = static_cast<std::uint64_t>(parsed);
  return text == std::to_string(value);
}

std::string ExpectedHeader(std::string_view prefix,
                           std::string_view joint) {
  std::string value(prefix);
  value += "::";
  value += joint;
  return value;
}

std::string_view ExpectedPhase(SegmentKind kind,
                               std::size_t frame) noexcept {
  switch (kind) {
    case SegmentKind::kEntry:
      if (frame <= 74) return "BALANCE_PREP";
      if (frame <= 99) return "BALANCED_STANCE_HOLD";
      if (frame <= 224) return "READY_TRANSITION";
      return "BALANCED_READY_HOLD";
    case SegmentKind::kTimedServe:
      if (frame <= 22) return "SPIN001_PREP";
      if (frame <= 34) return "SPIN001_RELEASE_HOLD";
      if (frame <= 39) return "SPIN001_STRIKE";
      if (frame <= 46) return "SPIN001_BRAKE";
      return "SPIN001_POST_STRIKE_DWELL";
    case SegmentKind::kRecovery:
      return frame <= 149 ? "FULLBODY_RECOVERY"
                          : "PDSTAND_TERMINAL_HOLD";
  }
  return {};
}

std::int64_t ExpectedSourceFrame(SegmentKind kind,
                                 std::size_t frame) noexcept {
  switch (kind) {
    case SegmentKind::kEntry:
      return static_cast<std::int64_t>(4 * frame);
    case SegmentKind::kTimedServe:
      return 1000 + static_cast<std::int64_t>(2 * frame);
    case SegmentKind::kRecovery:
      return 1167 + static_cast<std::int64_t>(2 * frame);
  }
  return -1;
}

std::uint16_t ExpectedEventMask(SegmentKind kind,
                                std::size_t frame) noexcept {
  switch (kind) {
    case SegmentKind::kEntry:
      if (frame == 0) return 1;
      if (frame == 75) return 2;
      if (frame == 100) return 4;
      if (frame == 225) return 8;
      if (frame == 249) return 16;
      return 0;
    case SegmentKind::kTimedServe:
      if (frame == 0) return 1;
      if (frame == 23) return 6;
      if (frame == 25) return 8;
      if (frame == 35) return 16;
      if (frame == 39) return 32;
      if (frame == 47) return 64;
      if (frame == 83) return 384;
      return 0;
    case SegmentKind::kRecovery:
      if (frame == 0) return 1;
      if (frame == 150) return 6;
      return 0;
  }
  return 0;
}

robot_io::RobotCommand ZeroCommand() {
  robot_io::RobotCommand command;
  command.q_des = Eigen::VectorXd::Zero(robot_io::kA3Dof);
  command.dq_des = Eigen::VectorXd::Zero(robot_io::kA3Dof);
  command.tau_ff = Eigen::VectorXd::Zero(robot_io::kA3Dof);
  command.kp = Eigen::VectorXd::Zero(robot_io::kA3Dof);
  command.kd = Eigen::VectorXd::Zero(robot_io::kA3Dof);
  return command;
}

bool IsArmSdkSlot(int sdk) noexcept {
  return sdk >= robot_io::kA3ArmStart &&
         sdk < robot_io::kA3ArmStart + robot_io::kA3ArmCount;
}

bool ValidateStorageToRobotCommandScatter(std::string& error) {
  const robot_io::JointLayout& layout = robot_io::MakeA3Layout31();
  if (layout.names.size() != kA3Spin001FullbodyDof) {
    error = "Spin001 backend RobotCommand layout is not 31D";
    return false;
  }
  std::array<bool, kA3Spin001FullbodyDof> seen{};
  for (std::size_t storage = 0; storage < kStorageToSdk.size(); ++storage) {
    const int sdk = kStorageToSdk[storage];
    if (sdk < 0 || sdk >= robot_io::kA3Dof ||
        seen[static_cast<std::size_t>(sdk)] ||
        layout.names[static_cast<std::size_t>(sdk)] !=
            kStorageJointNames[storage]) {
      error = "Spin001 storage-order to RobotCommand scatter is not the exact named 31D bijection";
      return false;
    }
    seen[static_cast<std::size_t>(sdk)] = true;
  }
  if (!std::all_of(seen.begin(), seen.end(), [](bool value) { return value; })) {
    error = "Spin001 storage-order scatter leaves a RobotCommand slot unmapped";
    return false;
  }
  return true;
}

bool ValidateCommandRow(const robot_io::RobotCommand& command,
                        std::size_t frame,
                        A3Spin001FullbodySegmentStats& stats,
                        std::string& error) {
  for (int sdk = 0; sdk < robot_io::kA3Dof; ++sdk) {
    const double q = command.q_des[sdk];
    const double dq = command.dq_des[sdk];
    const double tau = command.tau_ff[sdk];
    const double kp = command.kp[sdk];
    const double kd = command.kd[sdk];
    if (!std::isfinite(q) || !std::isfinite(dq) || !std::isfinite(tau) ||
        !std::isfinite(kp) || !std::isfinite(kd)) {
      error = "Spin001 command contains a nonfinite RobotCommand value";
      return false;
    }
    if (q < kA3SdkJointPosLo[static_cast<std::size_t>(sdk)] ||
        q > kA3SdkJointPosHi[static_cast<std::size_t>(sdk)]) {
      error = "Spin001 q_des exceeds authoritative SDK position limits at frame " +
              std::to_string(frame) + " sdk_index " + std::to_string(sdk);
      return false;
    }
    const double abs_dq = std::abs(dq);
    if (abs_dq >
        kA3SdkJointVelocityLimit[static_cast<std::size_t>(sdk)] +
            kNumericTolerance) {
      error = "Spin001 dq_des exceeds authoritative SDK velocity limits at frame " +
              std::to_string(frame) + " sdk_index " + std::to_string(sdk);
      return false;
    }
    if (abs_dq > stats.max_abs_dq_des_rad_s) {
      stats.max_abs_dq_des_rad_s = abs_dq;
      stats.max_abs_dq_frame = frame;
      stats.max_abs_dq_sdk_index = sdk;
    }
    if (tau != 0.0 ||
        std::abs(tau) >
            kA3SdkJointEffortLimit[static_cast<std::size_t>(sdk)]) {
      error = "Spin001 tau_ff must be exactly zero at frame " +
              std::to_string(frame) + " sdk_index " + std::to_string(sdk);
      return false;
    }
    if (kp != kExpectedKpSdk[static_cast<std::size_t>(sdk)] ||
        kd != kExpectedKdSdk[static_cast<std::size_t>(sdk)]) {
      error = "Spin001 embedded Kp/Kd contract mismatch at frame " +
              std::to_string(frame) + " sdk_index " + std::to_string(sdk);
      return false;
    }
  }
  return true;
}

bool ValidateDqSemantics(const std::vector<A3Spin001FullbodyFrame>& frames,
                         SegmentKind kind,
                         std::string& error) {
  if (frames.size() < 2) {
    error = "Spin001 segment needs at least two frames for dq validation";
    return false;
  }
  for (std::size_t frame = 0; frame < frames.size(); ++frame) {
    for (int sdk = 0; sdk < robot_io::kA3Dof; ++sdk) {
      const double actual = frames[frame].command.dq_des[sdk];
      if (!IsArmSdkSlot(sdk)) {
        if (actual != 0.0) {
          error = "Spin001 non-ARM dq_des must be exactly zero at frame " +
                  std::to_string(frame) + " sdk_index " +
                  std::to_string(sdk);
          return false;
        }
        continue;
      }
      double expected = 0.0;
      if (kind == SegmentKind::kRecovery && frame + 1 == frames.size()) {
        // An intentional terminal-hold ABI exception: do not preserve a
        // one-sided numerical derivative after reaching canonical PD_STAND.
        expected = 0.0;
      } else if (frame == 0) {
        expected = (frames[1].command.q_des[sdk] -
                    frames[0].command.q_des[sdk]) /
                   kA3Spin001FullbodyDtS;
      } else if (frame + 1 == frames.size()) {
        expected = (frames[frame].command.q_des[sdk] -
                    frames[frame - 1].command.q_des[sdk]) /
                   kA3Spin001FullbodyDtS;
      } else {
        expected = (frames[frame + 1].command.q_des[sdk] -
                    frames[frame - 1].command.q_des[sdk]) /
                   (2.0 * kA3Spin001FullbodyDtS);
      }
      if (std::abs(actual - expected) > kDqConsistencyTolerance) {
        error = "Spin001 ARM14 dq_des does not match the segment-local finite-difference/terminal-zero ABI at frame " +
                std::to_string(frame) + " sdk_index " +
                std::to_string(sdk);
        return false;
      }
    }
  }
  return true;
}

bool ExactJoinVector(const Eigen::VectorXd& left,
                     const Eigen::VectorXd& right,
                     std::string_view field,
                     std::size_t join,
                     std::string& error) {
  if (left.size() != robot_io::kA3Dof ||
      right.size() != robot_io::kA3Dof) {
    error = "Spin001 join has a non-31D RobotCommand vector";
    return false;
  }
  for (int sdk = 0; sdk < robot_io::kA3Dof; ++sdk) {
    if (left[sdk] != right[sdk]) {
      error = "Spin001 join " + std::to_string(join) + " requires exact " +
              std::string(field) + " equality at sdk_index " +
              std::to_string(sdk);
      return false;
    }
  }
  return true;
}

bool OfflineModeName(std::string_view value) noexcept {
  return value == "offline" || value == "offline-validate" ||
         value == "validate" || value == "self-check" ||
         value == "dry-run" || value == "no-publish";
}

}  // namespace

const char* A3Spin001FullbodySegmentName(SegmentKind kind) noexcept {
  switch (kind) {
    case SegmentKind::kEntry: return "entry";
    case SegmentKind::kTimedServe: return "timed_serve";
    case SegmentKind::kRecovery: return "recovery";
  }
  return "invalid";
}

std::string_view A3Spin001FullbodyArtifactId(SegmentKind kind) noexcept {
  return ValidKind(kind) ? kArtifactIds[KindIndex(kind)] : std::string_view{};
}

std::size_t A3Spin001FullbodyExpectedFrameCount(
    SegmentKind kind) noexcept {
  return ValidKind(kind) ? kFrameCounts[KindIndex(kind)] : 0;
}

const std::array<std::string_view, kA3Spin001FullbodyDof>&
A3Spin001FullbodyStorageJointNames() noexcept {
  return kStorageJointNames;
}

const std::array<double, kA3Spin001FullbodyDof>&
A3Spin001FullbodyExpectedKpSdk() noexcept {
  return kExpectedKpSdk;
}

const std::array<double, kA3Spin001FullbodyDof>&
A3Spin001FullbodyExpectedKdSdk() noexcept {
  return kExpectedKdSdk;
}

const std::array<double, kA3Spin001FullbodyDof>&
A3Spin001FullbodyCanonicalStandQSdk() noexcept {
  return kCanonicalStandQSdk;
}

bool A3Spin001FullbodySegment::LoadCsv(
    const std::string& path,
    SegmentKind kind,
    const std::string& expected_csv_sha256,
    std::string& error) {
  error.clear();
  frames_.clear();
  csv_sha256_.clear();
  stats_ = A3Spin001FullbodySegmentStats{};
  fallback_ = A3Spin001FullbodyFrame{};
  fallback_.command = ZeroCommand();

  if (!ValidKind(kind)) {
    error = "Spin001 segment kind is invalid";
    return false;
  }
  if (!ValidateStorageToRobotCommandScatter(error)) return false;
  if (!IsLowerHexSha256(expected_csv_sha256)) {
    error = "Spin001 segment contract requires a lowercase CSV SHA-256";
    return false;
  }

  const std::string bytes = ReadBytes(path, error);
  if (!error.empty()) return false;
  const std::string actual_sha = a3_pingpong::PpSha256::String(bytes);
  if (actual_sha != expected_csv_sha256) {
    error = "Spin001 segment CSV SHA-256 mismatch";
    return false;
  }

  std::istringstream input(bytes);
  std::string line;
  if (!std::getline(input, line)) {
    error = "Spin001 segment CSV is empty";
    return false;
  }
  const std::vector<std::string> header = SplitCsv(line);
  if (header.size() != kA3Spin001FullbodyCsvColumns) {
    error = "Spin001 segment CSV must have exactly 160 columns";
    return false;
  }
  static constexpr std::array<std::string_view,
                              kA3Spin001FullbodyMetadataColumns>
      kMetadata = {"frame_index", "time_s", "phase",
                   "source_fullbody_global_frame", "event_mask"};
  for (std::size_t column = 0; column < kMetadata.size(); ++column) {
    if (header[column] != kMetadata[column]) {
      error = "Spin001 segment metadata header/order mismatch";
      return false;
    }
  }
  for (std::size_t group = 0; group < kVectorPrefixes.size(); ++group) {
    for (std::size_t joint = 0; joint < kStorageJointNames.size(); ++joint) {
      const std::size_t column = kA3Spin001FullbodyMetadataColumns +
                                 group * kA3Spin001FullbodyDof + joint;
      if (header[column] !=
          ExpectedHeader(kVectorPrefixes[group], kStorageJointNames[joint])) {
        error = "Spin001 segment named vector header/order mismatch";
        return false;
      }
    }
  }

  const std::size_t expected_frames =
      A3Spin001FullbodyExpectedFrameCount(kind);
  std::vector<A3Spin001FullbodyFrame> loaded;
  loaded.reserve(expected_frames);
  A3Spin001FullbodySegmentStats loaded_stats;
  std::size_t row = 0;
  while (std::getline(input, line)) {
    if (line.empty()) {
      error = "Spin001 segment CSV contains a blank row";
      return false;
    }
    if (row >= expected_frames) {
      error = "Spin001 segment CSV has too many rows";
      return false;
    }
    const std::vector<std::string> fields = SplitCsv(line);
    if (fields.size() != kA3Spin001FullbodyCsvColumns) {
      error = "Spin001 segment CSV row width mismatch at frame " +
              std::to_string(row);
      return false;
    }

    std::uint64_t parsed_unsigned = 0;
    if (!ParseCanonicalUnsigned(fields[0], parsed_unsigned) ||
        parsed_unsigned != row) {
      error = "Spin001 frame_index must be a contiguous canonical integer";
      return false;
    }
    double time_s = 0.0;
    if (!ParseFiniteDouble(fields[1], time_s) ||
        std::abs(time_s - static_cast<double>(row) /
                              kA3Spin001FullbodyHz) > kNumericTolerance) {
      error = "Spin001 time_s is not the exact 50 Hz local grid";
      return false;
    }
    if (fields[2] != ExpectedPhase(kind, row)) {
      error = "Spin001 phase ABI mismatch at frame " + std::to_string(row);
      return false;
    }
    if (!ParseCanonicalUnsigned(fields[3], parsed_unsigned) ||
        parsed_unsigned >
            static_cast<std::uint64_t>(
                std::numeric_limits<std::int64_t>::max()) ||
        static_cast<std::int64_t>(parsed_unsigned) !=
            ExpectedSourceFrame(kind, row)) {
      error = "Spin001 source_fullbody_global_frame sequence mismatch at frame " +
              std::to_string(row);
      return false;
    }
    const std::uint16_t expected_event = ExpectedEventMask(kind, row);
    if (!ParseCanonicalUnsigned(fields[4], parsed_unsigned) ||
        parsed_unsigned != expected_event) {
      error = "Spin001 event_mask ABI mismatch at frame " +
              std::to_string(row);
      return false;
    }

    A3Spin001FullbodyFrame frame;
    frame.frame_index = row;
    frame.time_s = time_s;
    frame.phase = fields[2];
    frame.source_fullbody_global_frame = ExpectedSourceFrame(kind, row);
    frame.event_mask = expected_event;
    frame.command = ZeroCommand();
    std::array<Eigen::VectorXd*, kA3Spin001FullbodyVectorGroups> vectors = {
        &frame.command.q_des, &frame.command.dq_des, &frame.command.tau_ff,
        &frame.command.kp, &frame.command.kd};
    for (std::size_t group = 0; group < vectors.size(); ++group) {
      for (std::size_t source = 0; source < kStorageJointNames.size();
           ++source) {
        const std::size_t column = kA3Spin001FullbodyMetadataColumns +
                                   group * kA3Spin001FullbodyDof + source;
        double value = 0.0;
        if (!ParseFiniteDouble(fields[column], value)) {
          error = "Spin001 vector contains a nonfinite or malformed number at frame " +
                  std::to_string(row);
          return false;
        }
        (*vectors[group])[kStorageToSdk[source]] = value;
      }
    }
    if (!ValidateCommandRow(frame.command, row, loaded_stats, error))
      return false;
    if (!loaded.empty()) {
      const robot_io::RobotCommand& previous = loaded.back().command;
      for (int sdk = 0; sdk < robot_io::kA3Dof; ++sdk) {
        const double step =
            std::abs(frame.command.q_des[sdk] - previous.q_des[sdk]);
        const double velocity_step_limit =
            kA3SdkJointVelocityLimit[static_cast<std::size_t>(sdk)] *
            kA3Spin001FullbodyDtS;
        const double step_limit =
            IsArmSdkSlot(sdk)
                ? std::min(kArmMaxStepRad, velocity_step_limit)
                : velocity_step_limit;
        if (step > step_limit + kNumericTolerance) {
          error = "Spin001 adjacent q_des step exceeds the deployment limit at frame " +
                  std::to_string(row) + " sdk_index " +
                  std::to_string(sdk);
          return false;
        }
        if (step > loaded_stats.max_q_step_rad) {
          loaded_stats.max_q_step_rad = step;
          loaded_stats.max_q_step_frame = row;
          loaded_stats.max_q_step_sdk_index = sdk;
        }
      }
    }
    loaded.push_back(std::move(frame));
    ++row;
  }
  if (row != expected_frames) {
    error = "Spin001 segment CSV row count mismatch: expected " +
            std::to_string(expected_frames) + ", got " +
            std::to_string(row);
    return false;
  }
  if (!ValidateDqSemantics(loaded, kind, error)) return false;

  loaded_stats.frames = loaded.size();
  loaded_stats.duration_s =
      static_cast<double>(loaded.size() - 1) / kA3Spin001FullbodyHz;
  loaded_stats.full_tick_occupancy_s =
      static_cast<double>(loaded.size()) / kA3Spin001FullbodyHz;
  kind_ = kind;
  csv_sha256_ = actual_sha;
  stats_ = loaded_stats;
  frames_ = std::move(loaded);
  return true;
}

const A3Spin001FullbodyFrame& A3Spin001FullbodySegment::At(
    std::size_t frame) const noexcept {
  if (frames_.empty()) return fallback_;
  return frames_[std::min(frame, frames_.size() - 1)];
}

bool A3Spin001FullbodyProgram::Load(
    const Paths& csv_paths,
    const Sha256s& expected_csv_sha256,
    std::string& error) {
  error.clear();
  segments_ = {};
  contracts_ = {};
  stats_ = A3Spin001FullbodyProgramStats{};
  fallback_ = A3Spin001FullbodyFrame{};
  fallback_.command = ZeroCommand();

  std::array<A3Spin001FullbodySegment,
             kA3Spin001FullbodySegmentCount>
      loaded;
  for (std::size_t index = 0; index < loaded.size(); ++index) {
    const SegmentKind kind = static_cast<SegmentKind>(index);
    if (!loaded[index].LoadCsv(csv_paths[index], kind,
                               expected_csv_sha256[index], error)) {
      error = std::string(A3Spin001FullbodySegmentName(kind)) +
              " segment rejected: " + error;
      return false;
    }
  }

  A3Spin001FullbodyProgramStats loaded_stats;
  for (std::size_t join = 0; join + 1 < loaded.size(); ++join) {
    const robot_io::RobotCommand& left =
        loaded[join].At(loaded[join].size() - 1).command;
    const robot_io::RobotCommand& right = loaded[join + 1].At(0).command;
    if (!ExactJoinVector(left.q_des, right.q_des, "q_des", join, error) ||
        !ExactJoinVector(left.tau_ff, right.tau_ff, "tau_ff", join, error) ||
        !ExactJoinVector(left.kp, right.kp, "kp", join, error) ||
        !ExactJoinVector(left.kd, right.kd, "kd", join, error)) {
      return false;
    }
    for (int sdk = 0; sdk < robot_io::kA3Dof; ++sdk) {
      const double jump = std::abs(right.dq_des[sdk] - left.dq_des[sdk]);
      if (jump > loaded_stats.max_join_dq_jump_rad_s) {
        loaded_stats.max_join_dq_jump_rad_s = jump;
        loaded_stats.max_join_index = join;
        loaded_stats.max_join_dq_sdk_index = sdk;
      }
    }
  }

  const robot_io::RobotCommand& terminal =
      loaded[KindIndex(SegmentKind::kRecovery)]
          .At(kFrameCounts[KindIndex(SegmentKind::kRecovery)] - 1)
          .command;
  for (int sdk = 0; sdk < robot_io::kA3Dof; ++sdk) {
    if (terminal.q_des[sdk] !=
        kCanonicalStandQSdk[static_cast<std::size_t>(sdk)]) {
      error = "Spin001 recovery does not terminate at exact canonical stand q_des at sdk_index " +
              std::to_string(sdk);
      return false;
    }
    if (terminal.dq_des[sdk] != 0.0) {
      error = "Spin001 recovery terminal dq_des must be exactly zero at sdk_index " +
              std::to_string(sdk);
      return false;
    }
  }

  loaded_stats.frames = kA3Spin001FullbodyTotalFrames;
  loaded_stats.full_tick_occupancy_s =
      static_cast<double>(kA3Spin001FullbodyTotalFrames) /
      kA3Spin001FullbodyHz;
  segments_ = std::move(loaded);
  stats_ = loaded_stats;
  return true;
}

bool A3Spin001FullbodyProgram::LoadContentBound(
    const std::string& repository_root,
    const Paths& manifest_paths,
    const Sha256s& expected_manifest_sha256,
    std::string& error) {
  error.clear();
  segments_ = {};
  contracts_ = {};
  stats_ = A3Spin001FullbodyProgramStats{};
  fallback_ = A3Spin001FullbodyFrame{};
  fallback_.command = ZeroCommand();

  try {
    std::array<ParsedManifest, kA3Spin001FullbodySegmentCount> parsed;
    Paths csv_paths;
    Sha256s csv_sha256;
    for (std::size_t index = 0; index < parsed.size(); ++index) {
      const SegmentKind kind = static_cast<SegmentKind>(index);
      parsed[index] = ParseManifest(repository_root, manifest_paths[index],
                                    expected_manifest_sha256[index], kind);
      csv_paths[index] = parsed[index].contract.csv_path;
      csv_sha256[index] = parsed[index].contract.csv_sha256;
    }
    for (std::size_t manifest = 1; manifest < parsed.size(); ++manifest) {
      if (parsed[manifest].output_paths != parsed[0].output_paths ||
          parsed[manifest].output_sha256 != parsed[0].output_sha256) {
        throw std::runtime_error(
            "Spin001 split-set bindings differ across segment manifests");
      }
      RequireEqual(parsed[manifest].contract.profile_path,
                   parsed[0].contract.profile_path,
                   "Spin001 segment profile path");
      RequireEqual(parsed[manifest].contract.profile_sha256,
                   parsed[0].contract.profile_sha256,
                   "Spin001 segment profile SHA-256");
      RequireEqual(parsed[manifest].contract.producer_path,
                   parsed[0].contract.producer_path,
                   "Spin001 segment producer path");
      RequireEqual(parsed[manifest].contract.producer_sha256,
                   parsed[0].contract.producer_sha256,
                   "Spin001 segment producer SHA-256");
    }
    for (std::size_t index = 0; index < parsed.size(); ++index) {
      RequireEqual(parsed[0].output_sha256[index], csv_sha256[index],
                   "Spin001 split-set CSV SHA-256");
      const fs::path embedded = ResolveRepoFile(
          repository_root, parsed[0].output_paths[index],
          "Spin001 split-set CSV path");
      RequireEqual(embedded.string(), csv_paths[index],
                   "Spin001 split-set CSV path");
    }

    if (!Load(csv_paths, csv_sha256, error)) return false;
    for (std::size_t index = 0; index < parsed.size(); ++index) {
      contracts_[index] = std::move(parsed[index].contract);
    }
    return true;
  } catch (const YAML::Exception& exception) {
    error = std::string("Spin001 manifest/profile JSON parse failed: ") +
            exception.what();
  } catch (const std::exception& exception) {
    error = exception.what();
  }
  segments_ = {};
  contracts_ = {};
  stats_ = A3Spin001FullbodyProgramStats{};
  return false;
}

const A3Spin001FullbodySegment& A3Spin001FullbodyProgram::Segment(
    SegmentKind kind) const noexcept {
  return ValidKind(kind) ? segments_[KindIndex(kind)] : segments_[0];
}

const A3Spin001FullbodySegmentContract&
A3Spin001FullbodyProgram::Contract(SegmentKind kind) const noexcept {
  return ValidKind(kind) ? contracts_[KindIndex(kind)] : contracts_[0];
}

const A3Spin001FullbodyFrame& A3Spin001FullbodyProgram::At(
    std::size_t global_frame) const noexcept {
  if (empty()) return fallback_;
  std::size_t remaining = std::min(global_frame, size() - 1);
  for (const A3Spin001FullbodySegment& segment : segments_) {
    if (remaining < segment.size()) return segment.At(remaining);
    remaining -= segment.size();
  }
  return segments_.back().At(segments_.back().size() - 1);
}

bool A3Spin001FullbodyRequestsRealCommands(
    const std::vector<std::string>& arguments) noexcept {
  for (std::size_t index = 0; index < arguments.size(); ++index) {
    const std::string_view argument = arguments[index];
    if (argument == "--real" || argument == "--live" ||
        argument == "--execute" || argument == "--publish" ||
        argument == "--publish=true" || argument == "--send-commands" ||
        argument == "--hardware" || argument == "--hardware-authorized" ||
        argument == "--confirm-real-commands" ||
        argument == "--serve-only" || argument == "--full" ||
        argument == "--release-only" || argument == "--no-ball-dry-run") {
      return true;
    }
    static constexpr std::array<std::string_view, 8> kRealPrefixes = {
        "--real=", "--live=", "--execute=", "--publish=",
        "--send-commands=", "--hardware=", "--hardware-authorized=",
        "--confirm-real-commands="};
    if (std::any_of(kRealPrefixes.begin(), kRealPrefixes.end(),
                    [argument](std::string_view prefix) {
                      return argument.substr(0, prefix.size()) == prefix;
                    })) {
      return true;
    }
    if (argument == "--mode") {
      if (index + 1 >= arguments.size() ||
          !OfflineModeName(arguments[index + 1])) {
        return true;
      }
      ++index;
      continue;
    }
    constexpr std::string_view kModePrefix = "--mode=";
    if (argument.substr(0, kModePrefix.size()) == kModePrefix &&
        !OfflineModeName(argument.substr(kModePrefix.size()))) {
      return true;
    }
  }
  return false;
}

}  // namespace a3_deploy
