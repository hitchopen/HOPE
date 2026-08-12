#pragma once

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "a3_deploy/numeric_safety.hpp"
#include "a3_pingpong/pp_joint_limits.hpp"
#include "a3_pingpong/pp_joint_map.hpp"
#include "a3_pingpong/pp_sha256.hpp"

namespace a3_pingpong {

constexpr int kServeDof = 31;
constexpr double kServePolicyHz = 50.0;
constexpr double kServeDt = 1.0 / kServePolicyHz;
constexpr double kServeArmKpScale = 2.5;
constexpr double kServeArmKdScale = 1.25;
constexpr double kServeRightArmKpBoost = 1.0;
constexpr double kServeRightArmKdBoost = 1.0;
// Despite the historical manifest key, this qualified boost applies to the
// four proximal left-arm joints: shoulder pitch/roll/yaw and elbow.
constexpr double kServeLeftProximalArmKpBoost = 1.35;
constexpr double kServeLeftProximalArmKdBoost = 1.15;
constexpr int kServeHandoffGainTransitionTicks = 25;
constexpr double kServeBalanceCompensationRad = 0.05;
constexpr double kServePalmTossPitchCompensationRad =
    0.1832595714594046;
constexpr double kServePalmTossRollCompensationRad = 0.0;
constexpr int kServePalmTossDurationTicks = 16;
constexpr double kServePalmReleaseVerticalVelocityRequestMps = 2.95;
constexpr int kServePalmDropDurationTicks = 7;
constexpr double kServePalmDropAccelerationZMps2 = -45.0;
constexpr double kServePalmDropMinVerticalVelocityMps = -2.0;
constexpr double kServeMaxActualReleasePalmTiltDeg = 10.0;
constexpr std::array<double, 3> kServeFixedContactVelocityMps = {
    1.450, 0.200, 1.000};
constexpr const char* kServeProductionBranchMode =
    "observed_fixed_clip";
constexpr double kServeGate3StationXM = -0.500;
constexpr double kServeGate3StationYM = -0.7625;
constexpr double kServeGate3StationZM = 1.070;
constexpr double kServeGate3HeadingRad = 0.0;
constexpr double kServeGate3StationXyToleranceM = 0.050;
constexpr double kServeGate3HeadingToleranceRad = 0.100;
constexpr const char* kServeAnatomicalPalmSurface =
    "B_confirmed_minus_obb_z";
constexpr double kServePalmUpBranchWristRollMaxRad = -1.20;
constexpr double kServePalmUpElbowMinRad = 0.60;
constexpr const char* kServePalmFingerDirectionContract =
    "smooth_in_plane_rotation_from_forward_inboard_30deg_to_45deg";
constexpr int kServeRightSafeHoldFrame = 145;
constexpr int kServeBranchSelectionFrame = 172;
constexpr int kServeBranchSharedPrefixEndFrame = 171;
constexpr const char* kServeBranchSelector =
    "fresh_ball_envelope_fixed_clip";
constexpr const char* kServeAmbiguousPolicy =
    "complete_toss_only_then_safe_return_no_handoff";
constexpr double kServeWristDqReferenceCapRadS = 2.50;
constexpr int kServeLeftWristDqCapStartFrame = 171;
constexpr int kServeRightWristDqCapStartFrame = 173;

enum class ServePhase : int {
  kApproachReady = 0,
  kServeReady = 1,
  kPreSwing = 2,
  kTossCommit = 3,
  kReleaseVerify = 4,
  kSwingCommit = 5,
  kFollowThrough = 6,
  kActiveRecovery = 7,
  kHandoffReady = 8,
};

inline const char* ServePhaseName(ServePhase phase) {
  switch (phase) {
    case ServePhase::kApproachReady: return "APPROACH_READY";
    case ServePhase::kServeReady: return "SERVE_READY";
    case ServePhase::kPreSwing: return "PRE_SWING";
    case ServePhase::kTossCommit: return "TOSS_COMMIT";
    case ServePhase::kReleaseVerify: return "RELEASE_VERIFY";
    case ServePhase::kSwingCommit: return "SWING_COMMIT";
    case ServePhase::kFollowThrough: return "FOLLOW_THROUGH";
    case ServePhase::kActiveRecovery: return "ACTIVE_RECOVERY";
    case ServePhase::kHandoffReady: return "HANDOFF_READY";
  }
  return "UNKNOWN";
}

inline ServePhase ParseServePhase(const std::string& value) {
  if (value == "APPROACH_READY") return ServePhase::kApproachReady;
  if (value == "SERVE_READY") return ServePhase::kServeReady;
  if (value == "PRE_SWING") return ServePhase::kPreSwing;
  if (value == "TOSS_COMMIT") return ServePhase::kTossCommit;
  if (value == "RELEASE_VERIFY") return ServePhase::kReleaseVerify;
  if (value == "SWING_COMMIT") return ServePhase::kSwingCommit;
  if (value == "FOLLOW_THROUGH") return ServePhase::kFollowThrough;
  if (value == "ACTIVE_RECOVERY") return ServePhase::kActiveRecovery;
  if (value == "HANDOFF_READY") return ServePhase::kHandoffReady;
  throw std::runtime_error("unknown serve phase: " + value);
}

struct ServeEventFrames {
  int ready_pose = -1;
  int clip_motion_start = -1;
  int toss_commit = -1;
  int release = -1;
  int swing_commit = -1;
  int contact = -1;
  int follow_end = -1;
  int recovery_start = -1;
  int handoff_begin = -1;
  int handoff_complete = -1;
};

struct ServeStrikeSelection {
  std::string name;
  std::string selector;
  std::string invalid_observation_policy;
  int selection_frame = -1;
  int shared_prefix_end_frame = -1;
  double contact_x_m = 0.0;
  std::array<double, 3> right_retract_position_m{};
  double right_retract_duration_ticks = 0.0;
  int right_retract_lead_ticks = -1;
  int right_retract_anchor_frame = -1;
};

struct ServeClipFrame {
  double time_s = 0.0;
  ServePhase phase = ServePhase::kApproachReady;
  std::uint32_t event_mask = 0;
  Eigen::VectorXd q_source;
  Eigen::VectorXd q_sdk;
  Eigen::VectorXd dq_source;
  Eigen::VectorXd dq_sdk;
};

class PpServeClip {
 public:
  static PpServeClip Load(const std::string& csv_path,
                          const std::string& manifest_path,
                          const Eigen::VectorXd& expected_default_q_source) {
    PpServeClip clip;
    clip.csv_path_ = csv_path;
    clip.manifest_path_ = manifest_path;
    if (expected_default_q_source.size() != kServeDof ||
        !AllFinite_(expected_default_q_source)) {
      throw std::runtime_error(
          "serve clip expected_default_q_source must be finite 31-D");
    }

    const YAML::Node manifest = YAML::LoadFile(manifest_path);
    RequireScalar_(manifest, "schema", "pp_serve_clip_v1");
    RequireScalar_(manifest, "status", "OFFLINE_PASS");
    if (!manifest["dof"] || manifest["dof"].as<int>() != kServeDof) {
      throw std::runtime_error("serve manifest dof must be 31");
    }
    if (!manifest["policy_hz"] ||
        std::abs(manifest["policy_hz"].as<double>() - kServePolicyHz) >
            1.0e-9) {
      throw std::runtime_error("serve manifest policy_hz must be exactly 50");
    }
    const YAML::Node hand = manifest["left_end_effector"];
    if (!hand || hand["type"].as<std::string>("") !=
                     "rigid_palm_no_grasp" ||
        hand["controlled_hand_dof"].as<int>(-1) != 0 ||
        hand["release_mechanism"].as<std::string>("") !=
            "arm_motion_and_drop_away_only" ||
        hand["anatomical_palm_surface"].as<std::string>("") !=
            kServeAnatomicalPalmSurface ||
        hand["palm_fingers_direction"].as<std::string>("") !=
            kServePalmFingerDirectionContract ||
        std::abs(
            hand["palm_up_branch_wrist_roll_max_rad"].as<double>(1.0) -
            kServePalmUpBranchWristRollMaxRad) > 1.0e-12 ||
        std::abs(hand["min_toss_elbow_rad"].as<double>(-1.0) -
                 kServePalmUpElbowMinRad) > 1.0e-12) {
      throw std::runtime_error(
          "serve manifest does not carry the confirmed-B "
          "rigid-palm/no-grasp contract");
    }
    const YAML::Node station = manifest["station_contract"];
    const YAML::Node station_position =
        station ? station["pelvis_position_m"] : YAML::Node();
    if (!station ||
        station["frame"].as<std::string>("") != "HOPE_world" ||
        station["source"].as<std::string>("") != "rally_v17_gate3" ||
        !station_position || !station_position.IsSequence() ||
        station_position.size() != 3 ||
        std::abs(station_position[0].as<double>(1.0) -
                 kServeGate3StationXM) > 1.0e-12 ||
        std::abs(station_position[1].as<double>(1.0) -
                 kServeGate3StationYM) > 1.0e-12 ||
        std::abs(station_position[2].as<double>(-1.0) -
                 kServeGate3StationZM) > 1.0e-12 ||
        std::abs(station["heading_rad"].as<double>(1.0) -
                 kServeGate3HeadingRad) > 1.0e-12 ||
        std::abs(station["xy_tolerance_m"].as<double>(-1.0) -
                 kServeGate3StationXyToleranceM) > 1.0e-12 ||
        std::abs(station["heading_tolerance_rad"].as<double>(-1.0) -
                 kServeGate3HeadingToleranceRad) > 1.0e-12) {
      throw std::runtime_error(
          "serve manifest station differs from RallyV17 Gate3 "
          "(-0.50,-0.7625,+X)");
    }
    const YAML::Node failures = manifest["failures"];
    if (!failures || !failures.IsSequence() || failures.size() != 0) {
      throw std::runtime_error("serve manifest contains offline failures");
    }

    const YAML::Node provenance = manifest["provenance"];
    if (!provenance || !provenance["clip_sha256"]) {
      throw std::runtime_error("serve manifest missing clip SHA256");
    }
    clip.clip_sha256_ = provenance["clip_sha256"].as<std::string>();
    if (!IsSha256_(clip.clip_sha256_)) {
      throw std::runtime_error("serve manifest clip_sha256 is malformed");
    }
    const std::string actual_sha = PpSha256::File(csv_path);
    if (actual_sha != clip.clip_sha256_) {
      throw std::runtime_error(
          "serve clip SHA256 mismatch: manifest=" + clip.clip_sha256_ +
          " actual=" + actual_sha);
    }

    const YAML::Node joint_order = manifest["joint_order"];
    if (!joint_order || !joint_order.IsSequence() ||
        joint_order.size() != kServeDof) {
      throw std::runtime_error("serve manifest joint_order must contain 31 names");
    }
    clip.joint_names_.reserve(kServeDof);
    for (const auto& node : joint_order) {
      clip.joint_names_.push_back(node.as<std::string>());
    }
    if (!build_src_to_sdk(clip.joint_names_, clip.src_to_sdk_)) {
      throw std::runtime_error(
          "serve manifest joint_order is not a 31-D backend bijection");
    }

    clip.events_ = ParseEvents_(manifest["events"]);
    ValidateEventOrder_(clip.events_);
    const YAML::Node controller = manifest["controller_contract"];
    if (!controller ||
        controller["ball_load_ready_frame"].as<int>(-1) !=
            clip.events_.toss_commit - 1 ||
        controller["balance_compensation"].as<std::string>("") !=
            "symmetric_hip_pitch_minus_ankle_pitch_plus" ||
        std::abs(controller["balance_compensation_rad"].as<double>(-1.0) -
                 kServeBalanceCompensationRad) > 1.0e-12 ||
        std::abs(controller["arm_kp_scale"].as<double>(-1.0) -
                 kServeArmKpScale) > 1.0e-12 ||
        std::abs(controller["arm_kd_scale"].as<double>(-1.0) -
                 kServeArmKdScale) > 1.0e-12 ||
        std::abs(controller["right_arm_kp_boost"].as<double>(-1.0) -
                 kServeRightArmKpBoost) > 1.0e-12 ||
        std::abs(controller["right_arm_kd_boost"].as<double>(-1.0) -
                 kServeRightArmKdBoost) > 1.0e-12 ||
        std::abs(
            controller["left_shoulder_pitch_kp_boost"].as<double>(-1.0) -
            kServeLeftProximalArmKpBoost) > 1.0e-12 ||
        std::abs(
            controller["left_shoulder_pitch_kd_boost"].as<double>(-1.0) -
            kServeLeftProximalArmKdBoost) > 1.0e-12 ||
        controller["handoff_gain_transition_ticks"].as<int>(-1) !=
            kServeHandoffGainTransitionTicks ||
        controller["handoff_gain_target"].as<std::string>("") !=
            "a3_pd_stand_static" ||
        std::abs(
            controller["palm_toss_pitch_compensation_rad"].as<double>(-1.0) -
            kServePalmTossPitchCompensationRad) > 1.0e-12 ||
        std::abs(
            controller["palm_toss_roll_compensation_rad"].as<double>(-1.0) -
            kServePalmTossRollCompensationRad) > 1.0e-12 ||
        controller["palm_toss_duration_ticks"].as<int>(-1) !=
            kServePalmTossDurationTicks ||
        controller["palm_in_plane_orientation"].as<std::string>("") !=
            kServePalmFingerDirectionContract ||
        std::abs(
            controller["palm_release_vertical_velocity_request_mps"]
                    .as<double>(-1.0) -
            kServePalmReleaseVerticalVelocityRequestMps) > 1.0e-12 ||
        controller["production_branch_mode"].as<std::string>("") !=
            kServeProductionBranchMode ||
        controller["post_release_elbow_mode"].as<std::string>("") !=
            "hold_through_apex_then_smooth5_resync_before_contact" ||
        controller["right_swing_type"].as<std::string>("") !=
            "dedicated_serve_cartesian_continuous_ik" ||
        controller["joint_velocity_reference"].as<std::string>("") !=
            "centered_difference_50hz_with_phase_wrist_caps" ||
        std::abs(
            controller["wrist_dq_reference_cap_rad_s"].as<double>(-1.0) -
            kServeWristDqReferenceCapRadS) > 1.0e-12 ||
        controller["left_wrist_dq_cap_start_frame"].as<int>(-1) !=
            kServeLeftWristDqCapStartFrame ||
        controller["right_wrist_dq_cap_start_frame"].as<int>(-1) !=
            kServeRightWristDqCapStartFrame ||
        std::abs(
            controller["palm_drop_acceleration_z_mps2"].as<double>(0.0) -
            kServePalmDropAccelerationZMps2) > 1.0e-12 ||
        controller["palm_drop_accel_delay_ticks"].as<int>(-1) != 0 ||
        std::abs(
            controller["palm_drop_min_vertical_velocity_mps"].as<double>(
                0.0) -
            kServePalmDropMinVerticalVelocityMps) > 1.0e-12) {
      throw std::runtime_error(
          "serve manifest controller contract differs from qualified "
          "late-load/balance/gain contract");
    }
    const YAML::Node physics = manifest["physics_contract"];
    if (!physics ||
        physics["racket_face_normal"].as<std::string>("") !=
            "right_racket_site_local_y" ||
        physics["mesh_geom_frame_for_face_normal"].as<bool>(true) ||
        physics["ball_release_state_injection"].as<bool>(true)) {
      throw std::runtime_error(
          "serve manifest physics contract differs from site-normal/no-injection");
    }
    const YAML::Node selection = manifest["strike_selection"];
    if (!selection) {
      throw std::runtime_error(
          "serve manifest missing strike_selection contract");
    }
    clip.strike_selection_.name =
        selection["name"].as<std::string>("");
    clip.strike_selection_.selector =
        selection["selector"].as<std::string>("");
    clip.strike_selection_.invalid_observation_policy =
        selection["invalid_observation_policy"].as<std::string>("");
    clip.strike_selection_.selection_frame =
        selection["selection_frame"].as<int>(-1);
    clip.strike_selection_.shared_prefix_end_frame =
        selection["shared_prefix_end_frame"].as<int>(-1);
    clip.strike_selection_.contact_x_m =
        selection["contact_x_m"].as<double>(-1.0);
    const YAML::Node retract_position =
        selection["right_retract_position_m"];
    if (!retract_position || !retract_position.IsSequence() ||
        retract_position.size() != 3) {
      throw std::runtime_error(
          "serve manifest missing 3-D right retract position");
    }
    for (int axis = 0; axis < 3; ++axis) {
      clip.strike_selection_.right_retract_position_m[axis] =
          retract_position[axis].as<double>();
    }
    clip.strike_selection_.right_retract_duration_ticks =
        selection["right_retract_duration_ticks"].as<double>(-1.0);
    clip.strike_selection_.right_retract_lead_ticks =
        selection["right_retract_lead_ticks"].as<int>(-1);
    clip.strike_selection_.right_retract_anchor_frame =
        selection["right_retract_anchor_frame"].as<int>(-1);
    constexpr double kExpectedRetractDurationTicks = 2.0;
    if (clip.strike_selection_.name != "fixed" ||
        clip.strike_selection_.selector != kServeBranchSelector ||
        clip.strike_selection_.invalid_observation_policy !=
            kServeAmbiguousPolicy ||
        clip.strike_selection_.selection_frame !=
            kServeBranchSelectionFrame ||
        clip.strike_selection_.shared_prefix_end_frame !=
            kServeBranchSharedPrefixEndFrame ||
        std::abs(clip.strike_selection_.contact_x_m - 0.520) > 1.0e-12 ||
        std::abs(
            clip.strike_selection_.right_retract_position_m[0] - 0.400) >
            1.0e-12 ||
        std::abs(
            clip.strike_selection_.right_retract_position_m[1] + 0.260) >
            1.0e-12 ||
        std::abs(
            clip.strike_selection_.right_retract_position_m[2] - 1.360) >
            1.0e-12 ||
        std::abs(clip.strike_selection_.right_retract_duration_ticks -
                 kExpectedRetractDurationTicks) > 1.0e-12 ||
        clip.strike_selection_.right_retract_lead_ticks != 1 ||
        clip.strike_selection_.right_retract_anchor_frame !=
            kServeBranchSharedPrefixEndFrame ||
        clip.strike_selection_.selection_frame <= clip.events_.swing_commit ||
        clip.strike_selection_.selection_frame >= clip.events_.contact ||
        clip.strike_selection_.shared_prefix_end_frame + 1 !=
            clip.strike_selection_.selection_frame) {
      throw std::runtime_error(
          "serve manifest strike selection differs from qualified "
          "single-clip/fail-closed contract");
    }
    const YAML::Node expected = manifest["expected"];
    const YAML::Node contact_velocity =
        expected ? expected["serve_contact_velocity_mps"] : YAML::Node();
    const YAML::Node release_acceleration =
        expected ? expected["palm_release_accel_after_mps2"] : YAML::Node();
    if (!expected || !contact_velocity ||
        !contact_velocity.IsSequence() || contact_velocity.size() != 3 ||
        !release_acceleration || !release_acceleration.IsSequence() ||
        release_acceleration.size() != 3 ||
        expected["palm_drop_duration_ticks"].as<int>(-1) !=
            kServePalmDropDurationTicks) {
      throw std::runtime_error(
          "serve manifest missing contact-velocity/drop-away contract");
    }
    for (int axis = 0; axis < 3; ++axis) {
      if (std::abs(contact_velocity[axis].as<double>() -
                   kServeFixedContactVelocityMps[axis]) > 1.0e-12) {
        throw std::runtime_error(
            "serve manifest contact velocity differs from qualified branch");
      }
    }
    if (std::abs(release_acceleration[0].as<double>()) > 1.0e-12 ||
        std::abs(release_acceleration[1].as<double>()) > 1.0e-12 ||
        std::abs(release_acceleration[2].as<double>() -
                 kServePalmDropAccelerationZMps2) > 1.0e-12) {
      throw std::runtime_error(
          "serve manifest drop-away acceleration differs from qualification");
    }
    const double expected_drop_min_vertical_velocity =
        expected["palm_drop_min_vertical_velocity_mps"].as<double>(
            std::numeric_limits<double>::quiet_NaN());
    if (!std::isfinite(expected_drop_min_vertical_velocity) ||
        std::abs(expected_drop_min_vertical_velocity -
                 kServePalmDropMinVerticalVelocityMps) > 1.0e-12) {
      throw std::runtime_error(
          "serve manifest drop-away speed cap differs from qualification");
    }
    clip.right_safe_hold_frame_ =
        expected ? expected["right_safe_hold_frame"].as<int>(-1) : -1;
    if (clip.right_safe_hold_frame_ != kServeRightSafeHoldFrame ||
        clip.right_safe_hold_frame_ >= clip.events_.toss_commit ||
        clip.right_safe_hold_frame_ >
            clip.strike_selection_.shared_prefix_end_frame) {
      throw std::runtime_error(
          "serve manifest right safe-hold frame differs from qualification");
    }
    const YAML::Node limits = manifest["limits"];
    if (!limits) throw std::runtime_error("serve manifest missing limits");
    clip.max_step_limit_ = limits["max_qdes_step_rad"].as<double>();
    clip.max_second_limit_ =
        limits["max_qdes_second_difference_rad"].as<double>();
    clip.max_dq_limit_ = limits["max_dqdes_rad_s"].as<double>();
    if (!(std::abs(clip.max_step_limit_ - 0.08) <= 1.0e-12 &&
          std::abs(clip.max_second_limit_ - 0.10) <= 1.0e-12 &&
          std::abs(clip.max_dq_limit_ - 4.0) <= 1.0e-12 &&
          std::abs(
              limits["max_actual_release_palm_tilt_deg"].as<double>(-1.0) -
              kServeMaxActualReleasePalmTiltDeg) <= 1.0e-12)) {
      throw std::runtime_error(
          "serve manifest q_des/dq_des or actual-palm limits differ");
    }

    clip.LoadCsv_(csv_path);
    if (clip.frames_.empty()) {
      throw std::runtime_error("serve CSV has no frames");
    }
    if (clip.events_.handoff_complete !=
        static_cast<int>(clip.frames_.size()) - 1) {
      throw std::runtime_error(
          "serve handoff_complete must be the final CSV frame");
    }
    clip.BuildVelocityReferences_();
    clip.ValidateTrajectory_(expected_default_q_source);
    return clip;
  }

  const ServeClipFrame& frame(std::size_t index) const {
    if (index >= frames_.size()) {
      throw std::out_of_range("serve clip frame index");
    }
    return frames_[index];
  }
  std::size_t size() const { return frames_.size(); }
  double dt() const { return kServeDt; }
  const ServeEventFrames& events() const { return events_; }
  const std::array<int, kServeDof>& src_to_sdk() const {
    return src_to_sdk_;
  }
  const std::vector<std::string>& joint_names() const { return joint_names_; }
  const std::string& clip_sha256() const { return clip_sha256_; }
  const ServeStrikeSelection& strike_selection() const {
    return strike_selection_;
  }
  double measured_max_step() const { return measured_max_step_; }
  double measured_max_second() const { return measured_max_second_; }
  double measured_max_dq() const { return measured_max_dq_; }
  int right_safe_hold_frame() const { return right_safe_hold_frame_; }

 private:
  static bool AllFinite_(const Eigen::VectorXd& values) {
    for (int i = 0; i < values.size(); ++i) {
      if (!a3_deploy::numeric_safety::IsFinite(values[i])) return false;
    }
    return true;
  }

  static std::string Trim_(std::string value) {
    while (!value.empty() &&
           std::isspace(static_cast<unsigned char>(value.front()))) {
      value.erase(value.begin());
    }
    while (!value.empty() &&
           std::isspace(static_cast<unsigned char>(value.back()))) {
      value.pop_back();
    }
    if (value.size() >= 2 && value.front() == '"' && value.back() == '"') {
      value = value.substr(1, value.size() - 2);
    }
    return value;
  }

  static std::vector<std::string> SplitCsv_(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) fields.push_back(Trim_(field));
    if (!line.empty() && line.back() == ',') fields.emplace_back();
    return fields;
  }

  static double ParseDouble_(const std::string& value,
                             const std::string& context) {
    try {
      std::size_t parsed = 0;
      const double result = std::stod(value, &parsed);
      if (parsed != value.size() ||
          !a3_deploy::numeric_safety::IsFinite(result)) {
        throw std::runtime_error("");
      }
      return result;
    } catch (const std::exception&) {
      throw std::runtime_error("invalid finite double at " + context +
                               ": " + value);
    }
  }

  static long long ParseInteger_(const std::string& value,
                                 const std::string& context) {
    try {
      std::size_t parsed = 0;
      const long long result = std::stoll(value, &parsed);
      if (parsed != value.size()) throw std::runtime_error("");
      return result;
    } catch (const std::exception&) {
      throw std::runtime_error("invalid integer at " + context + ": " + value);
    }
  }

  static bool IsSha256_(const std::string& value) {
    if (value.size() != 64) return false;
    for (char c : value) {
      if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
    }
    return true;
  }

  static void RequireScalar_(const YAML::Node& root, const char* key,
                             const char* expected) {
    if (!root[key] || root[key].as<std::string>() != expected) {
      throw std::runtime_error(std::string("serve manifest ") + key +
                               " must be " + expected);
    }
  }

  static int Event_(const YAML::Node& events, const char* key) {
    if (!events || !events[key]) {
      throw std::runtime_error(std::string("serve manifest missing event ") +
                               key);
    }
    return events[key].as<int>();
  }

  static ServeEventFrames ParseEvents_(const YAML::Node& events) {
    ServeEventFrames out;
    out.ready_pose = Event_(events, "ready_pose");
    out.clip_motion_start = Event_(events, "clip_motion_start");
    out.toss_commit = Event_(events, "toss_commit");
    out.release = Event_(events, "release");
    out.swing_commit = Event_(events, "swing_commit");
    out.contact = Event_(events, "contact");
    out.follow_end = Event_(events, "follow_end");
    out.recovery_start = Event_(events, "recovery_start");
    out.handoff_begin = Event_(events, "handoff_begin");
    out.handoff_complete = Event_(events, "handoff_complete");
    return out;
  }

  static void ValidateEventOrder_(const ServeEventFrames& e) {
    const std::array<int, 10> ordered = {
        e.ready_pose,       e.clip_motion_start, e.toss_commit, e.release,
        e.swing_commit,     e.contact,           e.follow_end,  e.recovery_start,
        e.handoff_begin,    e.handoff_complete};
    if (ordered.front() < 0) {
      throw std::runtime_error("serve events cannot be negative");
    }
    for (std::size_t i = 1; i < ordered.size(); ++i) {
      if (ordered[i] <= ordered[i - 1]) {
        throw std::runtime_error("serve events are not strictly ordered");
      }
    }
  }

  static std::uint32_t ExpectedEventMask_(int frame,
                                          const ServeEventFrames& e) {
    std::uint32_t mask = 0;
    if (frame == e.ready_pose) mask |= (1U << 0U);
    if (frame == e.clip_motion_start) mask |= (1U << 1U);
    if (frame == e.toss_commit) mask |= (1U << 2U);
    if (frame == e.release) mask |= (1U << 3U);
    if (frame == e.swing_commit) mask |= (1U << 4U);
    if (frame == e.contact) mask |= (1U << 5U);
    if (frame == e.follow_end) mask |= (1U << 6U);
    if (frame == e.recovery_start) mask |= (1U << 7U);
    if (frame == e.handoff_begin) mask |= (1U << 8U);
    if (frame == e.handoff_complete) mask |= (1U << 9U);
    return mask;
  }

  static ServePhase ExpectedPhase_(int frame, const ServeEventFrames& e) {
    if (frame < e.ready_pose) return ServePhase::kApproachReady;
    if (frame < e.clip_motion_start) return ServePhase::kServeReady;
    if (frame < e.toss_commit) return ServePhase::kPreSwing;
    if (frame < e.release) return ServePhase::kTossCommit;
    if (frame < e.swing_commit) return ServePhase::kReleaseVerify;
    if (frame < e.contact) return ServePhase::kSwingCommit;
    if (frame <= e.follow_end) return ServePhase::kFollowThrough;
    if (frame < e.handoff_begin) return ServePhase::kActiveRecovery;
    return ServePhase::kHandoffReady;
  }

  void LoadCsv_(const std::string& path) {
    std::ifstream stream(path);
    if (!stream) throw std::runtime_error("cannot open serve CSV: " + path);
    std::string line;
    if (!std::getline(stream, line) ||
        SplitCsv_(line) !=
            std::vector<std::string>({"schema", "pp_serve_clip_v1"})) {
      throw std::runtime_error("serve CSV schema row mismatch");
    }
    if (!std::getline(stream, line)) {
      throw std::runtime_error("serve CSV missing policy_hz row");
    }
    const auto hz = SplitCsv_(line);
    if (hz.size() != 2 || hz[0] != "policy_hz" ||
        std::abs(ParseDouble_(hz[1], "policy_hz") - kServePolicyHz) >
            1.0e-9) {
      throw std::runtime_error("serve CSV policy_hz must be 50");
    }
    if (!std::getline(stream, line)) {
      throw std::runtime_error("serve CSV missing header");
    }
    const auto header = SplitCsv_(line);
    if (header.size() != 4U + kServeDof || header[0] != "frame" ||
        header[1] != "time_s" || header[2] != "phase" ||
        header[3] != "event_mask") {
      throw std::runtime_error("serve CSV header shape mismatch");
    }
    for (int i = 0; i < kServeDof; ++i) {
      if (header[4 + i] != joint_names_[i]) {
        throw std::runtime_error(
            "serve CSV joint header differs from manifest at column " +
            std::to_string(i));
      }
    }

    int expected_frame = 0;
    while (std::getline(stream, line)) {
      if (!line.empty() && line.back() == '\r') line.pop_back();
      if (line.empty()) continue;
      const auto fields = SplitCsv_(line);
      if (fields.size() != 4U + kServeDof) {
        throw std::runtime_error("serve CSV field count mismatch at frame " +
                                 std::to_string(expected_frame));
      }
      const long long frame = ParseInteger_(fields[0], "frame");
      if (frame != expected_frame) {
        throw std::runtime_error("serve CSV non-contiguous frame sequence");
      }
      ServeClipFrame sample;
      sample.time_s = ParseDouble_(fields[1], "time_s");
      if (std::abs(sample.time_s - expected_frame * kServeDt) > 1.0e-9) {
        throw std::runtime_error("serve CSV nonuniform 50 Hz time at frame " +
                                 std::to_string(expected_frame));
      }
      sample.phase = ParseServePhase(fields[2]);
      const long long event_mask = ParseInteger_(fields[3], "event_mask");
      if (event_mask < 0 || event_mask > 0x3ffLL) {
        throw std::runtime_error("serve CSV event mask out of range");
      }
      sample.event_mask = static_cast<std::uint32_t>(event_mask);
      if (sample.event_mask !=
          ExpectedEventMask_(expected_frame, events_)) {
        throw std::runtime_error("serve CSV event mask mismatch at frame " +
                                 std::to_string(expected_frame));
      }
      if (sample.phase != ExpectedPhase_(expected_frame, events_)) {
        throw std::runtime_error("serve CSV phase mismatch at frame " +
                                 std::to_string(expected_frame));
      }
      sample.q_source = Eigen::VectorXd::Zero(kServeDof);
      for (int i = 0; i < kServeDof; ++i) {
        sample.q_source[i] =
            ParseDouble_(fields[4 + i], "q_des frame " +
                                           std::to_string(expected_frame));
      }
      sample.q_sdk = to_sdk_order(sample.q_source, src_to_sdk_);
      frames_.push_back(std::move(sample));
      ++expected_frame;
    }
  }

  void ValidateTrajectory_(
      const Eigen::VectorXd& expected_default_q_source) {
    if (frames_.size() < 3U) {
      throw std::runtime_error("serve clip requires at least three frames");
    }
    measured_max_step_ = 0.0;
    measured_max_second_ = 0.0;
    measured_max_dq_ = 0.0;
    for (std::size_t frame = 1; frame < frames_.size(); ++frame) {
      measured_max_step_ = std::max(
          measured_max_step_,
          (frames_[frame].q_source - frames_[frame - 1].q_source)
              .cwiseAbs()
              .maxCoeff());
    }
    for (std::size_t frame = 2; frame < frames_.size(); ++frame) {
      measured_max_second_ = std::max(
          measured_max_second_,
          (frames_[frame].q_source -
           2.0 * frames_[frame - 1].q_source +
           frames_[frame - 2].q_source)
              .cwiseAbs()
              .maxCoeff());
    }
    for (std::size_t frame = 0; frame < frames_.size(); ++frame) {
      measured_max_dq_ = std::max(
          measured_max_dq_, frames_[frame].dq_source.cwiseAbs().maxCoeff());
      for (int sdk = 0; sdk < kServeDof; ++sdk) {
        if (frames_[frame].q_sdk[sdk] < kSdkJointPosLo[sdk] ||
            frames_[frame].q_sdk[sdk] > kSdkJointPosHi[sdk]) {
          throw std::runtime_error(
              "serve CSV q_des exceeds backend hard limit at frame " +
              std::to_string(frame) + " sdk_joint " + std::to_string(sdk));
        }
      }
    }
    if (measured_max_step_ > max_step_limit_ + 5.0e-7) {
      throw std::runtime_error("serve CSV q_des step exceeds manifest limit");
    }
    if (measured_max_second_ > max_second_limit_ + 5.0e-7) {
      throw std::runtime_error(
          "serve CSV q_des second difference exceeds manifest limit");
    }
    if (measured_max_dq_ > max_dq_limit_ + 5.0e-7) {
      throw std::runtime_error("serve derived dq_des exceeds manifest limit");
    }
    if (!frames_.front().dq_source.isZero(0.0) ||
        !frames_[events_.toss_commit - 1].dq_source.isZero(0.0)) {
      throw std::runtime_error(
          "serve dq_des must be zero at start and rigid-palm load pause");
    }
    const double start_error =
        (frames_.front().q_source - expected_default_q_source)
            .cwiseAbs()
            .maxCoeff();
    if (start_error > 1.0e-8) {
      throw std::runtime_error(
          "serve CSV first frame is not exact policy default_q");
    }
    for (int frame = events_.handoff_begin;
         frame <= events_.handoff_complete; ++frame) {
      const double error =
          (frames_[frame].q_source - expected_default_q_source)
              .cwiseAbs()
              .maxCoeff();
      if (error > 1.0e-8) {
        throw std::runtime_error(
            "serve CSV handoff hold is not exact policy default_q");
      }
      if (!frames_[frame].dq_source.isZero(0.0)) {
        throw std::runtime_error(
            "serve handoff hold dq_des must be exactly zero");
      }
    }
  }

  void BuildVelocityReferences_() {
    if (frames_.size() < 3U) {
      throw std::runtime_error("serve clip requires at least three frames");
    }
    for (auto& frame : frames_) {
      frame.dq_source = Eigen::VectorXd::Zero(kServeDof);
      frame.dq_sdk = Eigen::VectorXd::Zero(kServeDof);
    }
    for (std::size_t frame = 1; frame + 1 < frames_.size(); ++frame) {
      frames_[frame].dq_source =
          (frames_[frame + 1].q_source - frames_[frame - 1].q_source) /
          (2.0 * kServeDt);
    }
    constexpr int kLeftWristPitchSource = 27;
    constexpr int kLeftWristYawSource = 29;
    constexpr int kRightWristPitchSource = 28;
    constexpr int kRightWristYawSource = 30;
    const auto cap_wrist = [this](int start_frame, int joint) {
      for (int frame = start_frame;
           frame < events_.handoff_begin; ++frame) {
        frames_[frame].dq_source[joint] = std::clamp(
            frames_[frame].dq_source[joint],
            -kServeWristDqReferenceCapRadS,
            kServeWristDqReferenceCapRadS);
      }
    };
    cap_wrist(kServeLeftWristDqCapStartFrame, kLeftWristPitchSource);
    cap_wrist(kServeLeftWristDqCapStartFrame, kLeftWristYawSource);
    cap_wrist(kServeRightWristDqCapStartFrame, kRightWristPitchSource);
    cap_wrist(kServeRightWristDqCapStartFrame, kRightWristYawSource);
    // Runtime can wait indefinitely at this frame for an operator to place the
    // ball on the rigid palm.  Handoff is likewise a true static hold.
    frames_[events_.toss_commit - 1].dq_source.setZero();
    for (int frame = events_.handoff_begin;
         frame <= events_.handoff_complete; ++frame) {
      frames_[frame].dq_source.setZero();
    }
    for (auto& frame : frames_) {
      frame.dq_sdk = to_sdk_order(frame.dq_source, src_to_sdk_);
    }
  }

  std::string csv_path_;
  std::string manifest_path_;
  std::string clip_sha256_;
  std::vector<std::string> joint_names_;
  std::array<int, kServeDof> src_to_sdk_{};
  std::vector<ServeClipFrame> frames_;
  ServeEventFrames events_{};
  ServeStrikeSelection strike_selection_{};
  int right_safe_hold_frame_ = -1;
  double max_step_limit_ = 0.0;
  double max_second_limit_ = 0.0;
  double max_dq_limit_ = 0.0;
  double measured_max_step_ = 0.0;
  double measured_max_second_ = 0.0;
  double measured_max_dq_ = 0.0;
};

}  // namespace a3_pingpong
