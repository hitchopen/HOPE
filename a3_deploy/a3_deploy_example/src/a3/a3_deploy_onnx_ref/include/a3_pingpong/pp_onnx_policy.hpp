// ONNX session wrapper for model_15200. Ported from
// hope_ws/.../hope_wbc_runner/onnx_policy.py. Inputs: obs[1,180], time_step[1,1].
// Outputs: actions[1,31] + reference side-outputs joint_pos[31], joint_vel[31],
// body_pos_w[14,3], body_quat_w[14,4] (clip baked in -> obs-independent refs).
// Metadata: joint_names, default_joint_pos, action_scale, joint_stiffness(kp),
// joint_damping(kd), body_names. Decode: target_q = default_q + action*scale.
#pragma once

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "a3_pingpong/pp_obs_builder.hpp"
#include "a3_pingpong/pp_deploy_config.hpp"
#include "a3_pingpong/pp_deploy_contract.hpp"
#include "a3_pingpong/pp_qdes_contract.hpp"
#include "a3_pingpong/pp_runtime_contract.hpp"
#include "a3_pingpong/pp_stationary_replay.hpp"
#include "a3_pingpong/pp_velocity_gate.hpp"

namespace a3_pingpong {

class PpOnnxPolicy {
 public:
  explicit PpOnnxPolicy(
      const std::string& model_path,
      PpOnnxLoadProfile load_profile = PpOnnxLoadProfile::kProductionStrict,
      const std::string& deploy_cfg_path = {})
      : env_(ORT_LOGGING_LEVEL_WARNING, "pp_onnx"),
        mem_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
        load_profile_(load_profile) {
    Ort::SessionOptions so;
    so.SetIntraOpNumThreads(1);
    session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), so);

    // detect obs input dim: 180 (full), 175 (deploy_parity), 177 (hitter_footwork),
    // 110 (legacy hitter_pure), 112 (legacy Schema27 reach permissions),
    // 127 (Schema28 append-only frozen-core shadow feedback),
    // 113 (legacy V15 position-mocap hitter_pure), or
    // 118 (rewritten V15 position-mocap + finite HUGWBC gait command).
    auto in0 = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (in0.size() != 2 ||
        (in0[1] != kObsDim && in0[1] != kObsDim175 && in0[1] != kObsDim177 &&
         in0[1] != kObsDim110 && in0[1] != kObsDim112 &&
         in0[1] != kObsDim127 && in0[1] != 324 &&
         in0[1] != kObsDim113 && in0[1] != kObsDim118))
      throw std::runtime_error(
          "ONNX obs input is not [1,324], [1,180], [1,177], [1,175], [1,127], [1,118], [1,113], [1,112] or [1,110]");
    obs_dim_ = static_cast<int>(in0[1]);

    Ort::AllocatorWithDefaultOptions alloc;
    for (std::size_t index = 0; index < session_->GetOutputCount(); ++index) {
      const std::string name(
          session_->GetOutputNameAllocated(index, alloc).get());
      const auto shape = session_->GetOutputTypeInfo(index)
                             .GetTensorTypeAndShapeInfo()
                             .GetShape();
      if (name == "actions") {
        actions_output_valid_ =
            shape.size() == 2 && shape[0] == 1 && shape[1] == kNumJoints;
      } else if (name == "core_action_shadow_owned") {
        core_action_shadow_output_present_ = true;
        core_action_shadow_output_valid_ =
            shape.size() == 2 && shape[0] == 1 &&
            shape[1] == kCoreActionShadowDim;
      }
    }
    auto md = session_->GetModelMetadata();
    joint_names_ = SplitCsv(LookupMeta(md, alloc, "joint_names"));
    body_names_ = SplitCsv(LookupMeta(md, alloc, "body_names"));
    default_q_ = ToVec(LookupMeta(md, alloc, "default_joint_pos"));
    action_scale_ = ToVec(LookupMeta(md, alloc, "action_scale"));
    kp_ = ToVec(LookupMeta(md, alloc, "joint_stiffness"));
    kd_ = ToVec(LookupMeta(md, alloc, "joint_damping"));
    if ((int)joint_names_.size() != kNumJoints || default_q_.size() != kNumJoints)
      throw std::runtime_error("ONNX metadata joint count != 31");

    // Optional q_des contract. V17 names the historical V11 affine decoder explicitly;
    // bounded/feasible contracts additionally activate the stateful projector below. All
    // numerical arrays were resolved from the selected training YAML in action-column order.
    qdes_action_contract_ = LookupMetaOptional(md, alloc, "qdes_action_contract");
    qdes_actual_q_guard_contract_ =
        LookupMetaOptional(md, alloc, "qdes_actual_q_guard_contract");
    const std::string qdes_actual_q_hard_tolerance_s =
        LookupMetaOptional(
            md, alloc, "qdes_actual_q_hard_tolerance_rad");
    if (!qdes_actual_q_hard_tolerance_s.empty())
      qdes_actual_q_hard_tolerance_rad_ =
          std::stod(qdes_actual_q_hard_tolerance_s);
    qdes_actual_q_hard_audit_mode_ =
        LookupMetaOptional(md, alloc, "qdes_actual_q_hard_audit_mode");
    if (qdes_actual_q_hard_audit_mode_.empty())
      qdes_actual_q_hard_audit_mode_ = "termination";
    if (qdes_actual_q_hard_audit_mode_ != "termination" &&
        qdes_actual_q_hard_audit_mode_ != "telemetry")
      throw std::runtime_error(
          "ONNX qdes_actual_q_hard_audit_mode must be 'termination' or "
          "'telemetry'");
    if (!qdes_actual_q_guard_contract_.empty())
      qdes_actual_q_guard_horizon_s_ = std::stod(
          LookupMeta(md, alloc, "qdes_actual_q_guard_horizon_s"));
    if (!qdes_action_contract_.empty() &&
        qdes_action_contract_ != "v11_affine_safe_qdes_v1" &&
        qdes_action_contract_ != kV12AffineSafeSlewQdesContract &&
        qdes_action_contract_ != "bounded_qdes_v1" &&
        qdes_action_contract_ != "bounded_qdes_v2" &&
        qdes_action_contract_ != "feasible_qdes_v3" &&
        qdes_action_contract_ != "absolute_feasible_qdes_v5")
      throw std::runtime_error(
          "ONNX has unsupported qdes_action_contract='" + qdes_action_contract_ + "'");
    if (!qdes_actual_q_guard_contract_.empty() &&
        qdes_actual_q_guard_contract_ !=
            "predictive_safe_boundary_brake_v1")
      throw std::runtime_error(
          "ONNX has unsupported qdes_actual_q_guard_contract='" +
          qdes_actual_q_guard_contract_ + "'");
    if (!qdes_actual_q_guard_contract_.empty() &&
        qdes_action_contract_ != "feasible_qdes_v3")
      throw std::runtime_error(
          "ONNX actual-q guard requires feasible_qdes_v3");
    if (qdes_action_contract_ == "v11_affine_safe_qdes_v1" ||
        qdes_action_contract_ == kV12AffineSafeSlewQdesContract) {
      // V12 (affine-safe + per-tick slew) reuses every V11 limit key verbatim; its
      // extra slew metadata is parsed and validated fail-closed right after this block.
      const auto qdes_joint_names =
          SplitCsv(LookupMeta(md, alloc, "qdes_joint_names"));
      if (qdes_joint_names != joint_names_)
        throw std::runtime_error(
            "ONNX qdes_joint_names do not exactly match joint_names/order");
      qdes_safe_lo_ = ToVec(
          LookupMeta(md, alloc, "qdes_safe_lower_rad"));
      qdes_safe_hi_ = ToVec(
          LookupMeta(md, alloc, "qdes_safe_upper_rad"));
      qdes_hard_lo_ = ToVec(
          LookupMeta(md, alloc, "qdes_hard_lower_rad"));
      qdes_hard_hi_ = ToVec(
          LookupMeta(md, alloc, "qdes_hard_upper_rad"));
      const std::array<const Eigen::VectorXd*, 4> arrays = {
          &qdes_safe_lo_, &qdes_safe_hi_, &qdes_hard_lo_, &qdes_hard_hi_};
      for (const Eigen::VectorXd* values : arrays)
        if (values->size() != kNumJoints)
          throw std::runtime_error(
              "ONNX " + qdes_action_contract_ + " limit array length != 31");
      if (!std::isfinite(qdes_actual_q_hard_tolerance_rad_) ||
          qdes_actual_q_hard_tolerance_rad_ < 0.0)
        throw std::runtime_error(
            "ONNX " + qdes_action_contract_ + " hard tolerance is invalid");
      for (int i = 0; i < kNumJoints; ++i) {
        if (!std::isfinite(qdes_safe_lo_[i]) ||
            !std::isfinite(qdes_safe_hi_[i]) ||
            !std::isfinite(qdes_hard_lo_[i]) ||
            !std::isfinite(qdes_hard_hi_[i]) ||
            !(qdes_hard_lo_[i] <= qdes_safe_lo_[i] &&
              qdes_safe_lo_[i] < default_q_[i] &&
              default_q_[i] < qdes_safe_hi_[i] &&
              qdes_safe_hi_[i] <= qdes_hard_hi_[i]))
          throw std::runtime_error(
              "ONNX " + qdes_action_contract_ + " has invalid hard/safe limits for '" +
              joint_names_[i] + "'");
      }
      if (qdes_action_contract_ == kV12AffineSafeSlewQdesContract) {
        // v12_affine_safe_slew_qdes_v1: every list is a comma-separated 31-vector in
        // action order (same encoding as action_scale / qdes_safe_lower_rad); scalars
        // are plain numeric strings.  Missing key -> LookupMeta throws.  Any non-finite,
        // negative, zero-for-executable slew, malformed leg mask, inconsistent scale
        // ramp or wrong feedback contract -> ValidateV12SlewContractMetadata throws.
        qdes_slew_limit_rad_per_tick_ =
            ToVec(LookupMeta(md, alloc, "qdes_slew_limit_rad_per_tick"));
        qdes_slew_leg_mask_ = ToVec(LookupMeta(md, alloc, "qdes_slew_leg_mask"));
        qdes_slew_scale_params_.tilt_full_scale_rad = ParseMetaDouble(
            LookupMeta(md, alloc, "qdes_slew_tilt_full_scale_rad"),
            "qdes_slew_tilt_full_scale_rad");
        qdes_slew_scale_params_.tilt_min_scale_rad = ParseMetaDouble(
            LookupMeta(md, alloc, "qdes_slew_tilt_min_scale_rad"),
            "qdes_slew_tilt_min_scale_rad");
        qdes_slew_scale_params_.speed_full_scale_mps = ParseMetaDouble(
            LookupMeta(md, alloc, "qdes_slew_speed_full_scale_mps"),
            "qdes_slew_speed_full_scale_mps");
        qdes_slew_scale_params_.speed_min_scale_mps = ParseMetaDouble(
            LookupMeta(md, alloc, "qdes_slew_speed_min_scale_mps"),
            "qdes_slew_speed_min_scale_mps");
        qdes_slew_scale_params_.min_scale = ParseMetaDouble(
            LookupMeta(md, alloc, "qdes_slew_min_scale"), "qdes_slew_min_scale");
        qdes_v12_last_action_feedback_ =
            LookupMeta(md, alloc, "hitter_pure_last_action_feedback");
        ValidateV12SlewContractMetadata(
            joint_names_, ToStdVector(action_scale_),
            ToStdVector(qdes_slew_limit_rad_per_tick_),
            ToStdVector(qdes_slew_leg_mask_), qdes_slew_scale_params_,
            qdes_v12_last_action_feedback_);
      }
    } else if (qdes_action_contract_ == "bounded_qdes_v1" ||
        qdes_action_contract_ == "bounded_qdes_v2" ||
        qdes_action_contract_ == "feasible_qdes_v3" ||
        qdes_action_contract_ == "absolute_feasible_qdes_v5") {
      const auto qdes_joint_names =
          SplitCsv(LookupMeta(md, alloc, "qdes_action_joint_names"));
      if (qdes_joint_names != joint_names_)
        throw std::runtime_error(
            "ONNX qdes_action_joint_names do not exactly match joint_names/order");
      qdes_safe_lo_ = ToVec(LookupMeta(md, alloc, "qdes_safe_lo"));
      qdes_safe_hi_ = ToVec(LookupMeta(md, alloc, "qdes_safe_hi"));
      // The legacy affine bias/gain parameterize the bounded_qdes_v1/v2 full-span tanh only.
      // V5 deliberately does NOT read them: its decode is fully determined by
      // (qdes_default_q, qdes_action_scale, qdes_safe_lo/hi) below.
      qdes_tanh_bias_ = (qdes_action_contract_ == "feasible_qdes_v3" ||
                         qdes_action_contract_ == "absolute_feasible_qdes_v5")
          ? Eigen::VectorXd::Zero(kNumJoints)
          : ToVec(LookupMeta(md, alloc, "qdes_tanh_bias"));
      qdes_tanh_input_gain_ = qdes_action_contract_ == "bounded_qdes_v2"
          ? ToVec(LookupMeta(md, alloc, "qdes_tanh_input_gain"))
          : Eigen::VectorXd::Ones(kNumJoints);
      if (qdes_action_contract_ == "absolute_feasible_qdes_v5") {
        // v5 decode metadata (default-anchored per-side tanh). Both arrays are REQUIRED —
        // fail closed (LookupMeta throws) rather than fall back to the retired v4 affine
        // parameterization.
        qdes_default_q_ = ToVec(LookupMeta(md, alloc, "qdes_default_q"));
        qdes_action_scale_ = ToVec(LookupMeta(md, alloc, "qdes_action_scale"));
        if (qdes_default_q_.size() != kNumJoints ||
            qdes_action_scale_.size() != kNumJoints)
          throw std::runtime_error(
              "ONNX absolute_feasible_qdes_v5 qdes_default_q/qdes_action_scale "
              "length != 31");
      }
      qdes_rate_limit_ = ToVec(LookupMeta(md, alloc, "qdes_rate_limit_rad_s"));
      qdes_tracking_error_limit_ =
          ToVec(LookupMeta(md, alloc, "qdes_tracking_error_limit_rad"));
      qdes_projector_kp_ = ToVec(LookupMeta(md, alloc, "qdes_projector_kp"));
      qdes_projector_kd_ = ToVec(LookupMeta(md, alloc, "qdes_projector_kd"));
      qdes_projector_effort_limit_ =
          ToVec(LookupMeta(md, alloc, "qdes_projector_effort_limit"));
      qdes_feedback_scale_ = ToVec(LookupMeta(md, alloc, "qdes_feedback_scale"));
      qdes_torque_headroom_fraction_ =
          std::stod(LookupMeta(md, alloc, "qdes_torque_headroom_fraction"));
      qdes_projector_dt_s_ = std::stod(LookupMeta(md, alloc, "qdes_projector_dt_s"));
      const std::array<const Eigen::VectorXd*, 10> arrays = {
          &qdes_safe_lo_, &qdes_safe_hi_, &qdes_tanh_bias_, &qdes_tanh_input_gain_,
          &qdes_rate_limit_,
          &qdes_tracking_error_limit_, &qdes_projector_kp_, &qdes_projector_kd_,
          &qdes_projector_effort_limit_, &qdes_feedback_scale_};
      for (const Eigen::VectorXd* values : arrays)
        if (values->size() != kNumJoints)
          throw std::runtime_error(
              "ONNX " + qdes_action_contract_ + " metadata array length != 31");
      if (!(qdes_torque_headroom_fraction_ > 0.0 &&
            qdes_torque_headroom_fraction_ <= 1.0) ||
          !(qdes_projector_dt_s_ > 0.0) ||
          !std::isfinite(qdes_torque_headroom_fraction_) ||
          !std::isfinite(qdes_projector_dt_s_) ||
          (!qdes_actual_q_guard_contract_.empty() &&
           (!std::isfinite(qdes_actual_q_guard_horizon_s_) ||
            qdes_actual_q_guard_horizon_s_ < qdes_projector_dt_s_)))
        throw std::runtime_error(
            "ONNX " + qdes_action_contract_ +
            " has invalid torque headroom, projector dt, or actual-q guard horizon");
      for (int i = 0; i < kNumJoints; ++i) {
        const double finite_values[] = {
            default_q_[i], qdes_safe_lo_[i], qdes_safe_hi_[i],
            qdes_rate_limit_[i], qdes_tracking_error_limit_[i],
            qdes_projector_kp_[i], qdes_projector_kd_[i],
            qdes_projector_effort_limit_[i], qdes_feedback_scale_[i],
            qdes_tanh_bias_[i], qdes_tanh_input_gain_[i]};
        bool all_finite = true;
        for (double value : finite_values) all_finite &= std::isfinite(value);
        if (!all_finite ||
            !(qdes_safe_lo_[i] < default_q_[i] && default_q_[i] < qdes_safe_hi_[i]) ||
            qdes_rate_limit_[i] < 0.0 || qdes_tracking_error_limit_[i] < 0.0 ||
            (qdes_action_contract_ != "feasible_qdes_v3" &&
             qdes_tanh_input_gain_[i] <= 0.0) ||
            (qdes_action_contract_ == "absolute_feasible_qdes_v5" &&
             (qdes_action_scale_[i] <= 0.0 ||
              !(qdes_safe_lo_[i] < qdes_default_q_[i] &&
                qdes_default_q_[i] < qdes_safe_hi_[i]))) ||
            qdes_projector_kp_[i] <= 0.0 || qdes_projector_kd_[i] < 0.0 ||
            qdes_projector_effort_limit_[i] <= 0.0 || qdes_feedback_scale_[i] <= 0.0)
          throw std::runtime_error(
              "ONNX " + qdes_action_contract_ + " has invalid joint metadata for '" +
              joint_names_[i] + "'");
      }
    }

    // ZERO-GAIN GUARD. These gains go verbatim into the published RobotCommand; a non-positive
    // kp/kd means that joint receives NO PD torque (limp). The 2026-07-02 explicitpd_ft export
    // baked kp=kd=0 for all IdealPD (arms/waist/ankle) joints because the exporter read PhysX
    // drive gains, which explicit actuators null -> the free-base robot collapsed. Fail fast
    // instead of silently deploying a torqueless robot.
    for (int i = 0; i < kNumJoints; ++i) {
      if (kp_[i] <= 0.0 || kd_[i] <= 0.0)
        throw std::runtime_error(
            "ONNX metadata has non-positive PD gain for joint '" + joint_names_[i] +
            "' (kp=" + std::to_string(kp_[i]) + ", kd=" + std::to_string(kd_[i]) +
            "): broken export (exporter must bake NOMINAL actuator gains, "
            "data.default_joint_stiffness — not the PhysX drive gains). Re-export or patch "
            "the model metadata; refusing to run.");
    }

    // OPTIONAL per-clip reference-clock layout (clip_seg_lengths / clip_strike_phases). New
    // exports carry these so the runner's ClipLayout matches the BAKED clips; legacy models
    // (model_15200, v1 clips) predate the keys and fall back to the hardcoded v1 layout.
    const std::string seg_s = LookupMetaOptional(md, alloc, "clip_seg_lengths");
    const std::string pha_s = LookupMetaOptional(md, alloc, "clip_strike_phases");
    if (!seg_s.empty() && !pha_s.empty()) {
      const Eigen::VectorXd seg = ToVec(seg_s);
      const Eigen::VectorXd pha = ToVec(pha_s);
      if (seg.size() == pha.size() && seg.size() >= 1) {
        clip_seg_lengths_.assign(seg.data(), seg.data() + seg.size());
        clip_strike_phases_.assign(pha.data(), pha.data() + pha.size());
      }
    }

    // OPTIONAL per-clip reference base->racket reach offset at the strike frame
    // (dx0,dy0,dx1,dy1,...). Baked by utils/exporter.py since 2026-07-06 for 177-D
    // hitter_footwork models: the runner derives the deploy-time base STATION from the
    // racket target as station_xy = target_xy - reach_offset_xy[clip] (training
    // base_couple_mode=reference_reach). PpPolicy computes a refs-based fallback for
    // metadata-less 177 exports.
    const std::string reach_s = LookupMetaOptional(md, alloc, "ref_reach_offset_xy");
    if (!reach_s.empty()) {
      const Eigen::VectorXd v = ToVec(reach_s);
      for (int i = 0; i + 1 < v.size(); i += 2) reach_offsets_.push_back(Vec2(v[i], v[i + 1]));
    }

    // OPTIONAL hitter_pure sampling geometry (110-D models, 2026-07-07). Baked by
    // utils/exporter.py when the training task ran target_mode=hitter_pure:
    //   hitter_pure_pos_range_per_clip: "x_lo,x_hi,y_lo,y_hi,z_lo,z_hi;..." per clip,
    //     STATION-RELATIVE x/y (x degenerate = the fixed striking plane, y = the swing
    //     band), z ABSOLUTE above the floor.
    //   hitter_pure_vel_range_per_clip: same 6-tuple format, world-frame velocity box.
    //   hitter_pure_base_target_range: "x_lo,x_hi,y_lo,y_hi" — the independent trained
    //     station box around the spawn (informational; deploy stations come from balls).
    // PpPolicy derives the station from a racket target as
    //   station_xy = target_xy − (x_plane, y_band_center)[clip]
    // and gates engage against the trained z/vel envelopes.
    const std::string hp_pos_s = LookupMetaOptional(md, alloc, "hitter_pure_pos_range_per_clip");
    const std::string hp_vel_s = LookupMetaOptional(md, alloc, "hitter_pure_vel_range_per_clip");
    const std::string hp_vel_core_s =
        LookupMetaOptional(md, alloc, "hitter_pure_vel_core_range_per_clip");
    const std::string hp_vel_planner_s =
        LookupMetaOptional(md, alloc, "hitter_pure_vel_planner_range_per_clip");
    const std::string hp_vel_planner_mix_s =
        LookupMetaOptional(md, alloc, "hitter_pure_vel_planner_mix_prob");
    const std::string hp_vel_ramp_s =
        LookupMetaOptional(md, alloc, "hitter_pure_vel_range_ramp_steps");
    const std::string hp_base_s = LookupMetaOptional(md, alloc, "hitter_pure_base_target_range");
    const std::string hp_step_s = LookupMetaOptional(md, alloc, "hitter_pure_station_y_step_range");
    const std::string hp_mix_s = LookupMetaOptional(md, alloc, "hitter_pure_station_y_mixture");
    const std::string hp_side_s = LookupMetaOptional(md, alloc, "hitter_pure_station_side_explicit");
    runtime_contract_ = LookupMetaOptional(md, alloc, "hitter_pure_runtime_contract");
    training_recipe_ = LookupMetaOptional(md, alloc, "hitter_pure_training_recipe");
    training_recipe_version_ =
        LookupMetaOptional(md, alloc, "hitter_pure_training_recipe_version");
    actor_obs_contract_ = LookupMetaOptional(md, alloc, "actor_obs_contract");
    last_action_head_mode_ =
        LookupMetaOptional(md, alloc, "hitter_pure_last_action_head");
    if (!last_action_head_mode_.empty() && last_action_head_mode_ != "zero" &&
        last_action_head_mode_ != "base_vxy_world_filtered")
      throw std::runtime_error(
          "ONNX has unsupported hitter_pure_last_action_head='" +
          last_action_head_mode_ + "'");
    hitter_pingpong_command_contract_ =
        LookupMetaOptional(md, alloc, "hitter_pingpong_command_contract");
    if (!hitter_pingpong_command_contract_.empty() &&
        hitter_pingpong_command_contract_ != "small_station_external_schedule_precommit_settle_v1" &&
        hitter_pingpong_command_contract_ != "late_reveal_raw_tts_v1" &&
        hitter_pingpong_command_contract_ != "late_reveal_raw_tts_v2" &&
        hitter_pingpong_command_contract_ != "home_preempt_raw_tts_v3" &&
        hitter_pingpong_command_contract_ !=
            "strike_followthrough_home_external_commit_v1" &&
        hitter_pingpong_command_contract_ !=
            "strike_followthrough_home_external_commit_v2" &&
        hitter_pingpong_command_contract_ !=
            "strike_followthrough_home_external_commit_v3" &&
        hitter_pingpong_command_contract_ !=
            "strike_followthrough_home_external_commit_v4" &&
        hitter_pingpong_command_contract_ !=
            "native_clip_end_immediate_resample_v1" &&
        hitter_pingpong_command_contract_ !=
            "postcontact_fresh_flight_preempt_v1" &&
        hitter_pingpong_command_contract_ !=
            "fixed_home_fresh_flight_preempt_v1") {
      throw std::runtime_error(
          "ONNX has unsupported hitter_pingpong_command_contract='" +
          hitter_pingpong_command_contract_ + "'");
    }
    validate_hitter_pingpong_command_recipe_contract(
        training_recipe_, hitter_pingpong_command_contract_);
    hitter_pingpong_planner_commit_contract_ = LookupMetaOptional(
        md, alloc, "hitter_pingpong_planner_commit_contract");
    hitter_pingpong_common_ready_contract_ = LookupMetaOptional(
        md, alloc, "hitter_pingpong_common_ready_contract");
    hitter_pingpong_home_rally_contract_ = LookupMetaOptional(
        md, alloc, "hitter_pingpong_home_rally_contract");
    hitter_pingpong_wait_clock_contract_ = LookupMetaOptional(
        md, alloc, "hitter_pingpong_wait_clock_contract");
    if (hitter_pingpong_command_contract_ == "small_station_external_schedule_precommit_settle_v1" ||
        hitter_pingpong_planner_commit_contract_ == "fresh_schema2_absolute_small_station_v1" ||
        obs_dim_ == 324 || actor_obs_contract_ == "hitter_compact_execution_324_v1" ||
        training_recipe_ == "small_station_compact_execution_candidate_v1" ||
        training_recipe_ == "hitter_small_station_independent_recovery_v2" ||
        training_recipe_ == "hitter_small_station_support_heading_recovery_v3" ||
        training_recipe_ == "hitter_small_station_matched_plant_recovery_v4" ||
        training_recipe_ == "hitter_small_station_hold_resume_recovery_v5") {
      LoadSmallStationContract_(md, alloc);
    }
    if (uses_late_reveal_raw_tts_contract()) {
      const std::string wait_tts_s =
          LookupMetaOptional(md, alloc, "hitter_pingpong_wait_tts_s");
      const std::string pending_target_s =
          LookupMetaOptional(md, alloc, "hitter_pingpong_pending_target");
      const std::string prefix_cap_s = LookupMetaOptional(
          md, alloc, "hitter_pingpong_entry_prefix_phase_cap");
      const std::string policy_dt_s = LookupMetaOptional(
          md, alloc, "hitter_pingpong_policy_dt_s");
      if (wait_tts_s.empty() || pending_target_s.empty() || prefix_cap_s.empty() ||
          policy_dt_s.empty())
        throw std::runtime_error(
            "late_reveal_raw_tts ONNX is missing WAIT/target/prefix/dt metadata");
      hitter_pingpong_wait_tts_s_ = std::stod(wait_tts_s);
      hitter_pingpong_entry_prefix_phase_cap_ = std::stod(prefix_cap_s);
      hitter_pingpong_policy_dt_s_ = std::stod(policy_dt_s);
      const Eigen::VectorXd pending = ToVec(pending_target_s);
      if (pending.size() != 3 || !pending.allFinite() ||
          !std::isfinite(hitter_pingpong_wait_tts_s_) ||
          !std::isfinite(hitter_pingpong_entry_prefix_phase_cap_) ||
          !std::isfinite(hitter_pingpong_policy_dt_s_) ||
          hitter_pingpong_wait_tts_s_ <= 0.0 ||
          hitter_pingpong_entry_prefix_phase_cap_ < 0.0 ||
          hitter_pingpong_entry_prefix_phase_cap_ > 1.0 ||
          hitter_pingpong_policy_dt_s_ <= 0.0)
        throw std::runtime_error(
            "late_reveal_raw_tts ONNX has invalid WAIT/target/prefix/dt metadata");
      hitter_pingpong_pending_target_ =
          {pending[0], pending[1], pending[2]};
      if (hitter_pingpong_command_contract_ == "late_reveal_raw_tts_v2" ||
          hitter_pingpong_command_contract_ == "home_preempt_raw_tts_v3" ||
          hitter_pingpong_command_contract_ ==
              "strike_followthrough_home_external_commit_v1" ||
          hitter_pingpong_command_contract_ ==
              "strike_followthrough_home_external_commit_v2" ||
          hitter_pingpong_command_contract_ ==
              "strike_followthrough_home_external_commit_v3" ||
          hitter_pingpong_command_contract_ ==
              "strike_followthrough_home_external_commit_v4" ||
          hitter_pingpong_command_contract_ ==
              "native_clip_end_immediate_resample_v1" ||
          hitter_pingpong_command_contract_ ==
              "postcontact_fresh_flight_preempt_v1" ||
          hitter_pingpong_command_contract_ ==
              "fixed_home_fresh_flight_preempt_v1") {
        const std::string build_contract = LookupMetaOptional(
            md, alloc, "hitter_pingpong_build_contract");
        const std::string support_s = LookupMetaOptional(
            md, alloc,
            "hitter_pingpong_actor_entry_tts_support_per_clip_s");
        const std::string recipe_version = LookupMetaOptional(
            md, alloc, "hitter_pure_training_recipe_version");
        const std::string tts_buckets = LookupMetaOptional(
            md, alloc, "hitter_pingpong_entry_tts_buckets_s");
        const bool continuous_rally_v1 =
            hitter_pingpong_command_contract_ ==
            "strike_followthrough_home_external_commit_v1";
        const bool continuous_rally_v2 =
            hitter_pingpong_command_contract_ ==
            "strike_followthrough_home_external_commit_v2";
        const bool continuous_rally_v3 =
            hitter_pingpong_command_contract_ ==
            "strike_followthrough_home_external_commit_v3";
        const bool continuous_rally_v4 =
            hitter_pingpong_command_contract_ ==
            "strike_followthrough_home_external_commit_v4";
        const bool build2_minimal_rootfix =
            hitter_pingpong_command_contract_ ==
            "native_clip_end_immediate_resample_v1";
        const bool build2_rapid_preempt =
            hitter_pingpong_command_contract_ ==
            "postcontact_fresh_flight_preempt_v1";
        const bool build2_fixed_home =
            hitter_pingpong_command_contract_ ==
            "fixed_home_fresh_flight_preempt_v1";
        const bool build2_home_support_frame =
            training_recipe_ == "hitter_pingpong_build2_home_support_frame_v1";
        const bool build2_schema27 =
            training_recipe_ ==
            "hitter_pingpong_build2_torso_optional_reach_v2";
        const bool build2_schema28 =
            training_recipe_ ==
            "hitter_pingpong_build2_frozen_core_optional_reach_v1";
        const bool build2_schema29 =
            training_recipe_ ==
            "hitter_pingpong_build2_trainable_home_optional_reach_v1";
        const bool build2_schema31 =
            training_recipe_ == kSchema31TrainingRecipe;
        const bool build2_schema32 =
            training_recipe_ == kSchema32TrainingRecipe;
        const bool build2_schema33 =
            training_recipe_ == kSchema33TrainingRecipe;
        const bool build2_schema34 =
            training_recipe_ == kSchema34TrainingRecipe;
        const bool build2_schema32_slew_family =
            build2_schema32 || build2_schema33 || build2_schema34;
        const bool build2_optional_reach =
            build2_schema27 || build2_schema28 || build2_schema29 ||
            build2_schema31 || build2_schema32_slew_family;
        const bool build2_support_frame_family =
            build2_home_support_frame || build2_optional_reach;
        const bool build2_external_preempt =
            build2_rapid_preempt || build2_fixed_home;
        const std::string training_initialization_contract =
            LookupMetaOptional(
                md, alloc,
                "hitter_pingpong_training_initialization_contract");
        const std::string actor_warm_start_receipt = LookupMetaOptional(
            md, alloc, "hitter_pingpong_actor_warm_start_receipt");
        const bool continuous_rally =
            continuous_rally_v1 || continuous_rally_v2 || continuous_rally_v3 ||
            continuous_rally_v4;
        const bool home_preempt_v3 =
            hitter_pingpong_command_contract_ == "home_preempt_raw_tts_v3";
        const bool home_preempt = home_preempt_v3 || continuous_rally;
        const std::string rearm_s = LookupMetaOptional(
            md, alloc, "hitter_pingpong_post_contact_rearm_s");
        if (home_preempt && !rearm_s.empty())
          hitter_pingpong_post_contact_rearm_s_ = std::stod(rearm_s);
        const Eigen::VectorXd support = ToVec(support_s);
        const std::string replay_contract = LookupMetaOptional(
            md, alloc, "hitter_pingpong_replay_contract");
        const std::string schedule_contract = LookupMetaOptional(
            md, alloc, "hitter_pingpong_schedule_contract");
        const std::string stale_contract = LookupMetaOptional(
            md, alloc, "base_mocap_stale_burst_contract");
        const std::string stale_receipt_sha = LookupMetaOptional(
            md, alloc, "base_mocap_stale_burst_receipt_sha256");
        const std::string stale_start_prob = LookupMetaOptional(
            md, alloc, "base_mocap_stale_burst_start_prob");
        const std::string stale_weights = LookupMetaOptional(
            md, alloc, "base_mocap_stale_burst_length_weights");
        const std::string preposition_contract = LookupMetaOptional(
            md, alloc, "hitter_pingpong_preposition_contract");
        const Eigen::VectorXd preposition_leads = ToVec(
            LookupMetaOptional(md, alloc, "hitter_pingpong_preposition_lead_s"));
        const std::string next_arrival_contract = LookupMetaOptional(
            md, alloc, "hitter_pingpong_next_command_arrival_contract");
        const std::string side_rehearsal_contract = LookupMetaOptional(
            md, alloc, "hitter_pingpong_side_rehearsal_contract");
        const std::string teacher_contract = LookupMetaOptional(
            md, alloc, "hitter_pingpong_teacher_contract");
        bool invalid_repair_metadata =
            !hitter_pingpong_entry_tts_buckets_metadata_compatible(
                hitter_pingpong_command_contract_, tts_buckets) ||
            support.size() != 4 || !support.allFinite() ||
            support[0] <= 0.0 || support[1] < support[0] ||
            support[2] <= 0.0 || support[3] < support[2] ||
            std::fabs(
                hitter_pingpong_wait_tts_s_ -
                ((continuous_rally_v3 || continuous_rally_v4) ? 0.85 : 1.0)) >
                1.0e-12 ||
            std::fabs(hitter_pingpong_entry_prefix_phase_cap_ - 1.0) >
                1.0e-12 ||
            std::fabs(hitter_pingpong_policy_dt_s_ - 0.02) > 1.0e-12;
        if (build2_minimal_rootfix || build2_external_preempt) {
          validate_hitter_pingpong_initialization_contract(
              training_recipe_, training_initialization_contract,
              actor_warm_start_receipt);
          if (build2_schema34) {
            const bool step_v3 = training_recipe_version_ == "3";
            const bool balance_v2 = training_recipe_version_ == "2";
            if ((training_initialization_contract == kSchema34StepInitializationContract) != step_v3 ||
                (training_initialization_contract == kSchema34BalanceInitializationContract) != balance_v2 ||
                LookupMetaOptional(md, alloc, "hitter_pingpong_reward_contract") !=
                    (step_v3 ? kSchema34StepRewardContract : balance_v2 ? kSchema34BalanceRewardContract : kSchema34RewardContract) ||
                LookupMetaOptional(md, alloc, "hitter_pingpong_recovery_phase_contract") !=
                    (step_v3 ? kSchema34StepRecoveryContract : balance_v2 ? kSchema34BalanceRecoveryContract : kSchema34RecoveryPhaseContract))
              throw std::runtime_error("Schema34 recipe revision, source and reward identity disagree");
          }
          const Eigen::VectorXd pending_velocity = ToVec(LookupMetaOptional(
              md, alloc, "hitter_pingpong_pending_velocity"));
          const Eigen::VectorXd pending_normal = ToVec(LookupMetaOptional(
              md, alloc, "hitter_pingpong_pending_normal"));
          if (pending_velocity.size() != 3 || !pending_velocity.allFinite() ||
              pending_normal.size() != 3 || !pending_normal.allFinite()) {
            throw std::runtime_error(
                "Build2 HitterPingPong ONNX is missing its finite neutral pending command");
          }
          hitter_pingpong_pending_velocity_ = {
              pending_velocity[0], pending_velocity[1], pending_velocity[2]};
          hitter_pingpong_trajectory_teacher_contract_ = LookupMetaOptional(
              md, alloc, "hitter_pingpong_trajectory_teacher_contract");
          hitter_pingpong_motion_phase_contract_ = LookupMetaOptional(
              md, alloc, "hitter_pingpong_motion_phase_contract");
          hitter_pingpong_racket_velocity_contract_ = LookupMetaOptional(
              md, alloc, "hitter_pingpong_racket_velocity_contract");
          const std::array<double, 3> pending_normal_contract = {
              pending_normal[0], pending_normal[1], pending_normal[2]};
          if (build2_minimal_rootfix) {
            validate_hitter_pingpong_build2_minimal_rootfix_metadata(
                training_recipe_, recipe_version, build_contract,
                hitter_pingpong_command_contract_,
                hitter_pingpong_planner_commit_contract_, actor_obs_contract_,
                hitter_pingpong_trajectory_teacher_contract_,
                hitter_pingpong_motion_phase_contract_,
                hitter_pingpong_racket_velocity_contract_,
                hitter_pingpong_wait_clock_contract_,
                hitter_pingpong_pending_target_, hitter_pingpong_pending_velocity_,
                pending_normal_contract,
                LookupMetaOptional(
                    md, alloc, "hitter_pingpong_entry_hide_command_during_hold"),
                LookupMetaOptional(
                    md, alloc, "hitter_pingpong_contact_early_wrap_enabled"),
                LookupMetaOptional(
                    md, alloc, "hitter_pingpong_joint_cost_aggregation_contract"),
                LookupMetaOptional(
                    md, alloc, "hitter_pingpong_native_wrap_hold_steps_range"),
                LookupMetaOptional(
                    md, alloc, "hitter_pingpong_post_contact_home_contract"),
                LookupMetaOptional(
                    md, alloc, "hitter_pingpong_home_return_reward_contract"),
                LookupMetaOptional(
                    md, alloc, "hitter_pingpong_lifecycle_contract"),
                replay_contract);
          } else {
            hitter_pingpong_transition_contract_ = LookupMetaOptional(
                md, alloc, "hitter_pingpong_transition_contract");
            const std::string no_preempt_s = LookupMetaOptional(
                md, alloc, "hitter_pingpong_preempt_no_preempt_s");
            const Eigen::VectorXd preempt_tts_support = ToVec(LookupMetaOptional(
                md, alloc, "hitter_pingpong_preempt_entry_tts_support_s"));
            const Eigen::VectorXd preempt_delay_support = ToVec(LookupMetaOptional(
                md, alloc, "hitter_pingpong_preempt_commit_delay_support_s"));
            hitter_pingpong_preempt_commit_delay_clock_contract_ =
                LookupMetaOptional(
                    md, alloc,
                    "hitter_pingpong_preempt_commit_delay_clock_contract");
            if (no_preempt_s.empty() || preempt_tts_support.size() != 2 ||
                !preempt_tts_support.allFinite() || preempt_delay_support.size() != 2 ||
                !preempt_delay_support.allFinite()) {
              throw std::runtime_error(
                  "build2-rapid-preempt ONNX is missing finite preempt timing metadata");
            }
            hitter_pingpong_preempt_no_preempt_s_ = std::stod(no_preempt_s);
            hitter_pingpong_preempt_entry_tts_support_s_ = {
                preempt_tts_support[0], preempt_tts_support[1]};
            hitter_pingpong_preempt_commit_delay_support_s_ = {
                preempt_delay_support[0], preempt_delay_support[1]};
            std::array<double, 4> actor_entry_support{};
            if (support.size() == 4) {
              for (std::size_t i = 0; i < actor_entry_support.size(); ++i)
                actor_entry_support[i] = support[static_cast<Eigen::Index>(i)];
            }
            const std::string entry_hide_contract = LookupMetaOptional(
                md, alloc, "hitter_pingpong_entry_hide_command_during_hold");
            const std::string contact_early_wrap_contract = LookupMetaOptional(
                md, alloc, "hitter_pingpong_contact_early_wrap_enabled");
            const std::string joint_cost_contract = LookupMetaOptional(
                md, alloc, "hitter_pingpong_joint_cost_aggregation_contract");
            const std::string native_wrap_hold_contract = LookupMetaOptional(
                md, alloc, "hitter_pingpong_native_wrap_hold_steps_range");
            const std::string post_contact_home_contract = LookupMetaOptional(
                md, alloc, "hitter_pingpong_post_contact_home_contract");
            const std::string home_return_reward_contract = LookupMetaOptional(
                md, alloc, "hitter_pingpong_home_return_reward_contract");
            const std::string lifecycle_contract = LookupMetaOptional(
                md, alloc, "hitter_pingpong_lifecycle_contract");
            const std::string post_hit_arrival_contract = LookupMetaOptional(
                md, alloc, "hitter_pingpong_post_hit_arrival_contract");
            validate_hitter_pingpong_build2_rapid_preempt_metadata(
                training_recipe_, recipe_version, build_contract,
                hitter_pingpong_command_contract_,
                hitter_pingpong_planner_commit_contract_,
                hitter_pingpong_transition_contract_,
                hitter_pingpong_preempt_no_preempt_s_,
                hitter_pingpong_preempt_entry_tts_support_s_,
                hitter_pingpong_preempt_commit_delay_support_s_,
                hitter_pingpong_preempt_commit_delay_clock_contract_,
                actor_entry_support, preposition_contract,
                next_arrival_contract, replay_contract,
                entry_hide_contract, contact_early_wrap_contract,
                joint_cost_contract, native_wrap_hold_contract,
                post_contact_home_contract, home_return_reward_contract,
                lifecycle_contract, post_hit_arrival_contract,
                LookupMetaOptional(
                    md, alloc, "hitter_pingpong_station_mode"),
                LookupMetaOptional(
                    md, alloc, "hitter_pingpong_station_contract"),
                LookupMetaOptional(
                    md, alloc, "hitter_pingpong_base_target_semantics"));
            if (build2_optional_reach) {
              optional_reach_runtime_mode_ =
                  validate_hitter_pingpong_schema27_nonprivileged_metadata(
                      training_recipe_, actor_obs_contract_,
                      LookupMetaOptional(
                          md, alloc,
                          "hitter_pingpong_actor_architecture_contract"),
                      hitter_pingpong_planner_commit_contract_,
                      LookupMetaOptional(
                          md, alloc,
                          "hitter_pingpong_actor_observation_reach_indices"),
                      LookupMetaOptional(
                          md, alloc,
                          "hitter_pingpong_actor_observation_reach_sources"),
                      LookupMetaOptional(
                          md, alloc,
                          "hitter_pingpong_actor_observation_nonprivileged_contract"),
                      LookupMetaOptional(
                          md, alloc,
                          "hitter_pingpong_reach_permission_contract"),
                      LookupMetaOptional(
                          md, alloc,
                          "hitter_pingpong_optional_reach_training_enabled"),
                      LookupMetaOptional(
                          md, alloc,
                          "hitter_pingpong_optional_reach_contract"),
                      LookupMetaOptional(
                          md, alloc, "hitter_pingpong_reward_contract"),
                      LookupMetaOptional(
                          md, alloc,
                          "hitter_pingpong_recovery_phase_contract"));
              if (build2_schema27) {
                validate_hitter_schema27_initialization_runtime_pair(
                    training_initialization_contract,
                    actor_warm_start_receipt,
                    optional_reach_runtime_mode_);
              } else if (build2_schema28) {
                validate_hitter_pingpong_schema28_shadow_metadata(
                    training_recipe_, hitter_pingpong_transition_contract_,
                    training_initialization_contract,
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_actor_architecture_contract"),
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_core_shadow_feedback_contract"),
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_core_shadow_action_indices"),
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_core_shadow_observation_indices"),
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_onnx_output_contract"),
                    actions_output_valid_, core_action_shadow_output_valid_);
                if (
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_target_tuple_prefilter_contract") !=
                        "fixed_home_staged_support_intent_bank_v3_axis_envelope_"
                        "prefilter_not_certification_v1" ||
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_target_tuple_frame_contract") !=
                        "table_world_xy_minus_immutable_session_home_xy_"
                        "actor_world_z_v1" ||
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_target_tuple_relative_x_sensor_tolerance_m") !=
                        "0.0015" ||
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_target_tuple_normal_range_per_clip") !=
                        "0.74,0.92,-0.10,0.29,0.38,0.64;"
                        "0.78,0.93,-0.21,0.18,0.35,0.62" ||
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_target_tuple_continuous_correlated_qualification") !=
                        "NOT_PROVEN_axis_prefilter_only") {
                  throw std::runtime_error(
                      "Schema28 requires the exact immutable-HOME target-tuple "
                      "axis prefilter metadata and must not claim correlated "
                      "deployment certification");
                }
              } else if (build2_schema29) {
                validate_hitter_pingpong_schema29_single_output_metadata(
                    training_recipe_, hitter_pingpong_transition_contract_,
                    training_initialization_contract,
                    actor_obs_contract_,
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_actor_architecture_contract"),
                    LookupMetaOptional(
                        md, alloc, "hitter_pingpong_onnx_output_contract"),
                    actions_output_valid_, core_action_shadow_output_present_);
                validate_hitter_pingpong_schema29_candidate_qualification_metadata(
                    training_recipe_,
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_target_tuple_prefilter_contract"),
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_target_tuple_continuous_correlated_qualification"),
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_question_bank_artifact_contract"),
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_question_bank_sha256"),
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_question_bank_receipt_sha256"),
                    LookupMetaOptional(
                        md, alloc, "hitter_pingpong_tuple_contract"),
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_question_bank_qualification_scope"),
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_deployment_qualification"),
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_entry_tts_execution_qualification"),
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_rapid_transition_qualification"),
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_optional_reach_trajectory_qualification"),
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_optional_reach_deployment_qualification"));
                validate_hitter_pingpong_schema29_retention_metadata(
                    training_recipe_, side_rehearsal_contract,
                    teacher_contract);
                validate_hitter_pingpong_schema29_mocap_stale_metadata(
                    training_recipe_, stale_contract, stale_receipt_sha,
                    stale_start_prob, stale_weights,
                    LookupMetaOptional(
                        md, alloc,
                        "base_mocap_stale_burst_curriculum_contract"));
              } else if (build2_schema31 || build2_schema32_slew_family) {
                const std::string actor_architecture_contract =
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_actor_architecture_contract");
                const std::string onnx_output_contract = LookupMetaOptional(
                    md, alloc, "hitter_pingpong_onnx_output_contract");
                if (build2_schema32_slew_family) {
                  validate_hitter_pingpong_schema32_single_output_metadata(
                      training_recipe_, hitter_pingpong_transition_contract_,
                      training_initialization_contract, actor_obs_contract_,
                      actor_architecture_contract, onnx_output_contract,
                      actions_output_valid_, core_action_shadow_output_present_);
                } else {
                  validate_hitter_pingpong_schema31_single_output_metadata(
                      training_recipe_, hitter_pingpong_transition_contract_,
                      training_initialization_contract, actor_obs_contract_,
                      actor_architecture_contract, onnx_output_contract,
                      actions_output_valid_, core_action_shadow_output_present_);
                }
                validate_hitter_pingpong_schema31_candidate_qualification_metadata(
                    training_recipe_,
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_optional_reach_training_enabled"),
                    LookupMetaOptional(
                        md, alloc, "hitter_pingpong_optional_reach_contract"),
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_optional_reach_trajectory_qualification"),
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_optional_reach_deployment_qualification"));
                validate_hitter_pingpong_schema31_question_bank_metadata(
                    training_recipe_,
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_target_tuple_prefilter_contract"),
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_target_tuple_continuous_correlated_qualification"),
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_question_bank_artifact_contract"),
                    LookupMetaOptional(
                        md, alloc, "hitter_pingpong_question_bank_sha256"),
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_question_bank_receipt_sha256"),
                    LookupMetaOptional(
                        md, alloc, "hitter_pingpong_tuple_contract"),
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_question_bank_qualification_scope"),
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_deployment_qualification"),
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_entry_tts_execution_qualification"),
                    LookupMetaOptional(
                        md, alloc,
                        "hitter_pingpong_rapid_transition_qualification"));
                validate_hitter_pingpong_schema29_mocap_stale_metadata(
                    training_recipe_, stale_contract, stale_receipt_sha,
                    stale_start_prob, stale_weights,
                    LookupMetaOptional(
                        md, alloc,
                        "base_mocap_stale_burst_curriculum_contract"));
              }
            }
            auto exact3 = [](const std::array<double, 3>& actual,
                             const std::array<double, 3>& expected) {
              for (std::size_t i = 0; i < actual.size(); ++i) {
                if (!std::isfinite(actual[i]) ||
                    std::fabs(actual[i] - expected[i]) > 1.0e-12)
                  return false;
              }
              return true;
            };
            invalid_repair_metadata = invalid_repair_metadata ||
                actor_obs_contract_ !=
                    (build2_schema28
                         ? "hitter_pure_127_headslots_vxy_reach_core_shadow_v1"
                     : build2_schema34
                         ? (training_recipe_version_ == "3" ? kSchema34StepActorObsContract : kSchema34ActorObsContract)
                     : (build2_schema27 || build2_schema29 || build2_schema31 ||
                        build2_schema32_slew_family)
                         ? "hitter_pure_112_headslots_vxy_reach_v1"
                     : build2_home_support_frame
                         ? "hitter_pure_110_headslots_vxy_v1"
                         : "hitter_pure") ||
                last_action_head_mode_ !=
                    (build2_support_frame_family
                         ? "base_vxy_world_filtered"
                         : "zero") ||
                (build2_support_frame_family &&
                 !hitter_pure_v4_velocity_slot_joint_order_matches(joint_names_)) ||
                hitter_pingpong_trajectory_teacher_contract_ !=
                    "contact_linear_actor_visible_v1" ||
                hitter_pingpong_motion_phase_contract_ !=
                    "signed_tts_contact_frame_v1" ||
                hitter_pingpong_racket_velocity_contract_ !=
                    "sim_world_body_v1" ||
                hitter_pingpong_wait_clock_contract_ !=
                    "cold_hold_neutral_hidden_until_release_v1" ||
                !exact3(hitter_pingpong_pending_target_,
                        {0.58, -0.265, 1.075}) ||
                !exact3(hitter_pingpong_pending_velocity_, {0.0, 0.0, 0.0}) ||
                !exact3(pending_normal_contract, {1.0, 0.0, 0.0});
          }
          invalid_repair_metadata = invalid_repair_metadata ||
              hitter_pingpong_common_ready_contract_ !=
                  "cold_hold_neutral_no_ready_gate_v1" ||
              hitter_pingpong_home_rally_contract_ !=
                  (build2_fixed_home
                       ? (build2_support_frame_family
                              ? "immutable_home_support_frame_continuous_v1"
                              : "immutable_session_home_post_strike_drift_only_v1")
                       : "native_tail_toward_home_no_settle_gate_v1") ||
              (build2_minimal_rootfix &&
               preposition_contract != "disabled_until_native_clip_end") ||
              (build2_minimal_rootfix && !next_arrival_contract.empty()) ||
              side_rehearsal_contract !=
                  (build2_fixed_home
                       ? (build2_schema29
                         ? kSchema29SideRehearsalContract
                         : (build2_schema31 || build2_schema32_slew_family)
                              ? kSchema31SideRehearsalContract
                              : "build2_side_prior,fixed_home,no_build1_donor")
                       : "build2_side_prior,small_y,no_build1_donor") ||
              teacher_contract !=
                  (build2_schema29 ? kSchema29TeacherRetentionContract
                   : (build2_schema31 || build2_schema32_slew_family)
                         ? kSchema31TeacherContract
                                   : "disabled_standard_ppo") ||
              (build2_schema32_slew_family &&
               (!has_v12_affine_safe_slew_qdes_contract() ||
                qdes_v12_last_action_feedback_ !=
                    kV12ExecutedQdesRawFeedbackContract)) ||
              (build2_schema31 && !has_v11_affine_safe_qdes_contract()) ||
              LookupMetaOptional(
                  md, alloc, "hitter_pingpong_qdes_delay_contract") !=
                  "deployment_parity_no_fifo,0..0,actor_observes_previous_raw_action" ||
              LookupMetaOptional(
                  md, alloc, "hitter_pingpong_training_push_contract") !=
                  "disabled_in_core_training" ||
              LookupMetaOptional(
                  md, alloc, "hitter_pingpong_post_hit_arrival_contract") !=
                  (build2_fixed_home
                       ? "fixed_home_fresh_flight_preempt_v1"
                       : build2_rapid_preempt
                       ? "postcontact_fresh_flight_preempt_v1"
                       : "disabled");
          if (!invalid_repair_metadata) {
            hitter_pingpong_preposition_lead_s_ = {0.0, 0.0, 0.0};
          }
        } else if (continuous_rally) {
          validate_hitter_pingpong_initialization_contract(
              training_recipe_, training_initialization_contract,
              actor_warm_start_receipt);
          if (continuous_rally_v3 || continuous_rally_v4) {
            const std::string pending_velocity_s = LookupMetaOptional(
                md, alloc, "hitter_pingpong_pending_velocity");
            const std::string arrival_contract = LookupMetaOptional(
                md, alloc, "hitter_pingpong_arrival_contract");
            const std::string arrival_hazard_s = LookupMetaOptional(
                md, alloc, "hitter_pingpong_arrival_hazard_per_tick");
            const Eigen::VectorXd pending_velocity =
                ToVec(pending_velocity_s);
            if (pending_velocity.size() != 3 ||
                !pending_velocity.allFinite() || arrival_hazard_s.empty()) {
              throw std::runtime_error(
                  "continuous-rally-v3/v4 ONNX is missing a finite pending "
                  "velocity or arrival hazard");
            }
            hitter_pingpong_pending_velocity_ = {
                pending_velocity[0], pending_velocity[1],
                pending_velocity[2]};
            hitter_pingpong_arrival_contract_ = arrival_contract;
            hitter_pingpong_arrival_hazard_per_tick_ =
                std::stod(arrival_hazard_s);
            if (continuous_rally_v3) {
              validate_hitter_pingpong_continuous_v3_metadata(
                  training_recipe_, recipe_version, build_contract,
                  hitter_pingpong_command_contract_,
                  hitter_pingpong_wait_clock_contract_,
                  hitter_pingpong_wait_tts_s_,
                  hitter_pingpong_pending_target_,
                  hitter_pingpong_pending_velocity_,
                  hitter_pingpong_arrival_contract_,
                  next_arrival_contract,
                  hitter_pingpong_arrival_hazard_per_tick_,
                  hitter_pingpong_policy_dt_s_);
            } else {
              hitter_pingpong_trajectory_teacher_contract_ = LookupMetaOptional(
                  md, alloc, "hitter_pingpong_trajectory_teacher_contract");
              hitter_pingpong_motion_phase_contract_ = LookupMetaOptional(
                  md, alloc, "hitter_pingpong_motion_phase_contract");
              hitter_pingpong_racket_velocity_contract_ = LookupMetaOptional(
                  md, alloc, "hitter_pingpong_racket_velocity_contract");
              validate_hitter_pingpong_continuous_v4_metadata(
                  training_recipe_, recipe_version, build_contract,
                  hitter_pingpong_command_contract_,
                  hitter_pingpong_planner_commit_contract_, actor_obs_contract_,
                  hitter_pingpong_trajectory_teacher_contract_,
                  hitter_pingpong_motion_phase_contract_,
                  hitter_pingpong_racket_velocity_contract_,
                  hitter_pingpong_wait_clock_contract_,
                  hitter_pingpong_wait_tts_s_,
                  hitter_pingpong_pending_target_,
                  hitter_pingpong_pending_velocity_,
                  hitter_pingpong_arrival_contract_, next_arrival_contract,
                  hitter_pingpong_arrival_hazard_per_tick_,
                  hitter_pingpong_policy_dt_s_);
            }
          }
          invalid_repair_metadata = invalid_repair_metadata ||
              training_recipe_ !=
                  (continuous_rally_v4
                       ? "hitter_pingpong_continuous_rally_v4"
                       : continuous_rally_v3
                       ? "hitter_pingpong_continuous_rally_v3"
                       : continuous_rally_v2
                       ? "hitter_pingpong_continuous_rally_v2"
                       : "hitter_pingpong_continuous_rally_v1") ||
              recipe_version !=
                  (continuous_rally_v4
                       ? "4"
                       : continuous_rally_v3 ? "3" : continuous_rally_v2 ? "2" : "1") ||
              build_contract !=
                  (continuous_rally_v4
                       ? "continuous_rally_schema20"
                       : continuous_rally_v3
                       ? "continuous_rally_schema19"
                       : "continuous_rally_schema18") ||
              replay_contract != "disabled_in_core_training" ||
              hitter_pingpong_planner_commit_contract_ !=
                  ((continuous_rally_v2 || continuous_rally_v3 ||
                    continuous_rally_v4)
                       ? "fresh_shot_external_event_policy_independent_v2"
                       : "fixed_contact_plus_0p40_policy_independent_v1") ||
              hitter_pingpong_common_ready_contract_ !=
                  "no_ready_gate_external_commit" ||
              LookupMetaOptional(
                  md, alloc, "hitter_pingpong_repository_base_commit") !=
                  "7b88a0213fe2f360195b49b704fd7a8b7681858d" ||
              LookupMetaOptional(
                  md, alloc, "hitter_pingpong_design_base_commit") !=
                  "f1ec5e7b98f876779485f886fc12b32d65de7e92" ||
              hitter_pingpong_home_rally_contract_ !=
                  "immutable_home_return_preemptible_by_external_ball_v1" ||
              (continuous_rally_v2 &&
               hitter_pingpong_wait_clock_contract_ != "visible_elapsed_v1") ||
              ((continuous_rally_v3 || continuous_rally_v4) &&
               hitter_pingpong_wait_clock_contract_ !=
                   "constant_pending_wait_v1") ||
              ((continuous_rally_v3 || continuous_rally_v4) &&
               LookupMetaOptional(
                   md, alloc,
                   "hitter_pingpong_station_mix_home_relative") !=
                   "same=0.80,small=0.20,range=0.02..0.06,"
                   "absolute_y=-0.06..0.06,symmetric,"
                   "tuple_positions_rigidly_translated") ||
              ((continuous_rally_v3 || continuous_rally_v4) &&
               LookupMetaOptional(
                   md, alloc,
                   "hitter_pingpong_phase_owner_contract") !=
                   "active_strike_and_zero_upper_style_debt;"
                   "wait_single_max_cvar_idle_posture_debt_v1") ||
              ((continuous_rally_v3 || continuous_rally_v4) &&
               LookupMetaOptional(
                   md, alloc,
                   "hitter_pingpong_hidden_virtual_reward_contract") !=
                   "net_and_landing_telemetry_only") ||
              preposition_contract != "disabled_until_atomic_commit" ||
              (!(continuous_rally_v3 || continuous_rally_v4) &&
               next_arrival_contract !=
                   (continuous_rally_v2
                        ? "contact_relative_external_0p12_1p20_all_station_classes_v2"
                        : "contact_relative_fixed_0p40_all_station_classes_v1")) ||
              side_rehearsal_contract !=
                  "fh_bh_50_50,no_build1_donor,venue=0.25" ||
              teacher_contract != "disabled_standard_ppo" ||
              LookupMetaOptional(
                  md, alloc, "hitter_pingpong_qdes_delay_contract") !=
                  "deployment_parity_no_fifo,0..0,actor_observes_previous_raw_action" ||
              LookupMetaOptional(
                  md, alloc, "hitter_pingpong_training_push_contract") !=
                  "disabled_in_core_training" ||
              LookupMetaOptional(
                  md, alloc, "hitter_pingpong_mocap_outage_contract") !=
                  "model7800_iid_dropout=0.01,burst=off,propagate=0.05" ||
              std::fabs(support[0] -
                        ((continuous_rally_v2 || continuous_rally_v3 ||
                          continuous_rally_v4)
                             ? 0.40
                             : 0.82)) > 1.0e-12 ||
              std::fabs(support[1] - 0.82) > 1.0e-12 ||
              std::fabs(support[2] - 0.40) > 1.0e-12 ||
              std::fabs(support[3] - 0.55) > 1.0e-12 ||
              preposition_leads.size() != 3 ||
              !preposition_leads.allFinite() ||
              preposition_leads.cwiseAbs().maxCoeff() > 1.0e-12 ||
              !std::isfinite(hitter_pingpong_post_contact_rearm_s_) ||
              std::fabs(hitter_pingpong_post_contact_rearm_s_ - 0.12) >
                  1.0e-12;
          if (!invalid_repair_metadata) {
            hitter_pingpong_preposition_lead_s_ = {0.0, 0.0, 0.0};
          }
        } else if (home_preempt_v3) {
          invalid_repair_metadata = invalid_repair_metadata ||
              training_recipe_ != "hitter_pingpong_recovery_tts_v2" ||
              recipe_version != "2" ||
              build_contract != "home_rally_balance_v4" ||
              hitter_pingpong_planner_commit_contract_ !=
                  "postcontact_home_preposition_actor_entry_v3" ||
              hitter_pingpong_common_ready_contract_ !=
                  "real_model21800_synchronized_wait_v1" ||
              LookupMetaOptional(
                  md, alloc, "hitter_pingpong_repository_base_commit") !=
                  "7b88a0213fe2f360195b49b704fd7a8b7681858d" ||
              LookupMetaOptional(
                  md, alloc, "hitter_pingpong_design_base_commit") !=
                  "f1ec5e7b98f876779485f886fc12b32d65de7e92" ||
              LookupMetaOptional(
                  md, alloc,
                  "hitter_pingpong_training_initialization_contract") !=
                  "build1_model21800_actor_only_fresh_critic_optimizer_v1" ||
              LookupMetaOptional(
                  md, alloc, "hitter_pingpong_actor_warm_start_receipt") !=
                  "model_21800.pt,"
                  "sha256=daca62616178177647cf9949128582785529dbe6b7ba080fc96d86da23916561,"
                  "exported_onnx_sha256=6bf1a2418f8538e23577a0153f2fe6a1e78dee91f41650a232259432a84a4dc8" ||
              hitter_pingpong_home_rally_contract_ !=
                  "immutable_motion_entry_home_preposition_preemptible_v2" ||
              preposition_contract !=
                  "home_first_then_base_target_only_wait_v1" ||
              next_arrival_contract !=
                  "contact_relative_by_transition_class_v2,"
                  "same=0.14..3.00,small=0.50..3.00,main=0.90..2.20" ||
              side_rehearsal_contract !=
                  "fh_bh_50_50,build1_core=0.125_per_side,"
                  "venue=0.25_disjoint,current_side" ||
              teacher_contract !=
                  "safe_projected_build1_actor_mean,home_and_all_fh_upper16,"
                  "right_wrist_roll_weight=0.25,inset=0.08,huber=0.10,"
                  "coefficient=0.75,max=0.75,gradient_ratio_max=0.05" ||
              std::fabs(support[0] - 0.82) > 1.0e-12 ||
              std::fabs(support[1] - 0.82) > 1.0e-12 ||
              std::fabs(support[2] - 0.40) > 1.0e-12 ||
              std::fabs(support[3] - 0.55) > 1.0e-12 ||
              preposition_leads.size() != 3 ||
              !preposition_leads.allFinite() ||
              std::fabs(preposition_leads[0]) > 1.0e-12 ||
              std::fabs(preposition_leads[1] - 0.55) > 1.0e-12 ||
              std::fabs(preposition_leads[2] - 0.75) > 1.0e-12 ||
              !std::isfinite(hitter_pingpong_post_contact_rearm_s_) ||
              std::fabs(hitter_pingpong_post_contact_rearm_s_ - 0.12) >
                  1.0e-12;
          if (!invalid_repair_metadata) {
            hitter_pingpong_preposition_lead_s_ = {
                preposition_leads[0], preposition_leads[1],
                preposition_leads[2]};
          }
        } else {
          // Preserve Build3 recovery-v3 exports while Build4 uses a distinct HOME/preemption
          // command contract. Both paths remain fail-closed on their own frozen metadata.
          invalid_repair_metadata = invalid_repair_metadata ||
              training_recipe_ != "hitter_pingpong_recovery_tts_v3" ||
              recipe_version != "3" ||
              build_contract != "recovery_tts_markov_v3" ||
              replay_contract !=
                  "markov_side_phase_severity_v3,7,3,"
                  "capture=0.08+0.30+0.80+1.80+3.60+5.40+wrap,"
                  "clock=post_contact_age_across_wrap_same_strike_hold_v1,"
                  "fixed_prob=0.20,min_fill=256" ||
              schedule_contract != "fixed_from_first_actor_update_v1" ||
              hitter_pingpong_planner_commit_contract_ !=
                  "actor_entry_support_v1" ||
              hitter_pingpong_common_ready_contract_ !=
                  "right_arm_neutral_wait_20pct_default_body_v1" ||
              stale_contract != "localization_fresh_outage_empirical_v1" ||
              stale_receipt_sha !=
                  "ed805160905888fddc12a388f0ed27aaa11951283301c10496b0925a1401c82c" ||
              stale_start_prob != "0.015090824406148" ||
              stale_weights !=
                  "3,1,4,12,9,15,11,12,8,7,16,27,19,15,3" ||
              LookupMetaOptional(md, alloc, "base_mocap_delay_steps") != "1" ||
              LookupMetaOptional(md, alloc, "base_mocap_update_interval_steps") !=
                  "1" ||
              LookupMetaOptional(md, alloc, "base_mocap_position_noise_std") !=
                  "0.003,0.003,0.005" ||
              LookupMetaOptional(
                  md, alloc, "base_mocap_orientation_noise_std_rad") !=
                  "0.003491,0.003491,0.005236" ||
              LookupMetaOptional(
                  md, alloc,
                  "base_mocap_extrinsic_residual_rpy_std_rad") !=
                  "0.005236,0.005236,0.008727" ||
              LookupMetaOptional(md, alloc, "base_mocap_dropout_prob") != "0" ||
              LookupMetaOptional(md, alloc, "base_mocap_max_age_s") != "0.05" ||
              LookupMetaOptional(md, alloc, "base_mocap_max_propagation_s") !=
                  "0.05";
        }
        if (invalid_repair_metadata) {
          throw std::runtime_error(
              "ONNX HitterPingPong repair has incomplete or invalid "
              "recipe/WAIT/actor-entry support metadata");
        }
        hitter_pingpong_actor_entry_tts_support_[0] =
            {support[0], support[1]};
        hitter_pingpong_actor_entry_tts_support_[1] =
            {support[2], support[3]};
      }
    }
    base_localization_contract_ =
        LookupMetaOptional(md, alloc, "base_localization_contract");
    base_pose_source_ = LookupMetaOptional(md, alloc, "base_pose_source");
    base_pose_schema_ = LookupMetaOptional(md, alloc, "base_pose_schema");
    orientation_contract_ =
        LookupMetaOptional(md, alloc, "orientation_contract");
    angular_velocity_contract_ =
        LookupMetaOptional(md, alloc, "angular_velocity_contract");
    yaw_align_contract_ =
        LookupMetaOptional(md, alloc, "yaw_align_contract");
    world_frame_contract_ =
        LookupMetaOptional(md, alloc, "world_frame_contract");
    calibration_contract_ =
        LookupMetaOptional(md, alloc, "calibration_contract");
    const std::string base_velocity_alpha_s =
        LookupMetaOptional(md, alloc, "base_mocap_velocity_ema_alpha");
    const std::string base_max_age_s =
        LookupMetaOptional(md, alloc, "base_mocap_max_age_s");
    const std::string base_max_propagation_s =
        LookupMetaOptional(md, alloc, "base_mocap_max_propagation_s");
    if (!base_velocity_alpha_s.empty())
      base_velocity_ema_alpha_ = std::stod(base_velocity_alpha_s);
    if (!base_max_age_s.empty()) base_localization_max_age_s_ = std::stod(base_max_age_s);
    if (!base_max_propagation_s.empty())
      base_localization_max_propagation_s_ =
          std::stod(base_max_propagation_s);
    locomotion_contract_ =
        LookupMetaOptional(md, alloc, "hitter_pure_locomotion_contract");
    const std::string gait_frequency_s =
        LookupMetaOptional(md, alloc, "hitter_pure_gait_frequency_hz");
    const std::string gait_duty_s =
        LookupMetaOptional(md, alloc, "hitter_pure_gait_duty_factor");
    const std::string gait_deadband_s =
        LookupMetaOptional(md, alloc, "hitter_pure_gait_move_deadband");
    const std::string gait_step_distance_s =
        LookupMetaOptional(md, alloc, "hitter_pure_gait_step_distance");
    const std::string gait_max_cycles_s =
        LookupMetaOptional(md, alloc, "hitter_pure_gait_max_cycles");
    const std::string gait_velocity_max_s =
        LookupMetaOptional(md, alloc, "hitter_pure_gait_velocity_max");
    intervention_contract_ =
        LookupMetaOptional(md, alloc, "hitter_pure_intervention_contract");
    intervention_deploy_value_ = LookupMetaOptional(
        md, alloc, "hitter_pure_intervention_deploy_value");
    if (!gait_frequency_s.empty()) gait_frequency_hz_ = std::stod(gait_frequency_s);
    if (!gait_duty_s.empty()) gait_duty_factor_ = std::stod(gait_duty_s);
    if (!gait_deadband_s.empty()) gait_move_deadband_ = std::stod(gait_deadband_s);
    if (!gait_step_distance_s.empty())
      gait_step_distance_ = std::stod(gait_step_distance_s);
    if (!gait_max_cycles_s.empty()) gait_max_cycles_ = std::stoi(gait_max_cycles_s);
    if (!gait_velocity_max_s.empty()) gait_velocity_max_ = std::stod(gait_velocity_max_s);
    const std::string recipe_version_s =
        LookupMetaOptional(md, alloc, "hitter_pure_training_recipe_version");
    const std::string ready_hold_s =
        LookupMetaOptional(md, alloc, "hitter_pure_ready_hold_steps_range");
    const std::string motion_fh_sha =
        LookupMetaOptional(md, alloc, "hitter_pure_motion_forehand_sha256");
    const std::string motion_bh_sha =
        LookupMetaOptional(md, alloc, "hitter_pure_motion_backhand_sha256");
    const std::string receipt_sha =
        LookupMetaOptional(md, alloc, "hitter_pure_validator_receipt_sha256");
    const std::string env_cfg_sha =
        LookupMetaOptional(md, alloc, "hitter_pure_env_cfg_sha256");
    const std::string checkpoint_sha =
        LookupMetaOptional(md, alloc, "hitter_pure_checkpoint_sha256");
    const std::string task_recipe_sha =
        LookupMetaOptional(md, alloc, "hitter_pure_task_recipe_sha256");
    const std::string hitter_action_contract =
        LookupMetaOptional(md, alloc, "hitter_pure_action_contract");
    const std::string recovery_recipe =
        LookupMetaOptional(md, alloc, "hitter_pure_v17_recovery_recipe");
    const std::string curriculum_gates =
        LookupMetaOptional(md, alloc, "hitter_pure_v17_curriculum_gates");
    const std::string termination_contract =
        LookupMetaOptional(md, alloc, "hitter_pure_termination_contract");
    const std::string validator_profile =
        LookupMetaOptional(md, alloc, "hitter_pure_validator_profile");
    const std::string deployment_status =
        LookupMetaOptional(md, alloc, "hitter_pure_deployment_status");
    const std::string qualification_status =
        LookupMetaOptional(md, alloc, "hitter_pure_qualification_status");
    const std::string resolved_task_sha =
        LookupMetaOptional(md, alloc, "hitter_pure_resolved_task_sha256");
    const std::string deploy_manifest_schema =
        LookupMetaOptional(md, alloc, "a3_deploy_manifest_schema");
    const std::string deploy_manifest_status =
        LookupMetaOptional(md, alloc, "a3_deploy_manifest_status");
    const std::string deploy_hardware_authorized =
        LookupMetaOptional(md, alloc, "a3_deploy_hardware_authorized");
    const std::string deploy_contract_fingerprint_sha =
        LookupMetaOptional(
            md, alloc, "a3_deploy_contract_fingerprint_sha256");
    const std::string deploy_manifest_json =
        LookupMetaOptional(md, alloc, "a3_deploy_manifest_json");
    const std::string deploy_manifest_sha =
        LookupMetaOptional(md, alloc, "a3_deploy_manifest_sha256");
    const std::string qdes_parity_csv_sha =
        LookupMetaOptional(md, alloc, "a3_qdes_parity_csv_sha256");
    const std::string v17_fixed_station_contract = LookupMetaOptional(
        md, alloc, "hitter_pure_v17_fixed_station_contract");
    const std::string v17_release_contract = LookupMetaOptional(
        md, alloc, "hitter_pure_v17_release_contract");
    const std::string v17_target_stream_contract = LookupMetaOptional(
        md, alloc, "hitter_pure_v17_target_stream_contract");
    const std::string planner_schema = LookupMetaOptional(
        md, alloc, "hitter_pure_planner_schema");
    const std::string planner_stability_contract = LookupMetaOptional(
        md, alloc, "hitter_pure_planner_stability_contract");
    const std::string fixed_hit_plane_relative_x_m = LookupMetaOptional(
        md, alloc, "hitter_pure_fixed_hit_plane_relative_x_m");
    if (load_profile_ == PpOnnxLoadProfile::kV17R1StationaryMujocoReplay) {
      const std::string error = ValidateV17R1StationaryReplayMetadata({
          .training_recipe = training_recipe_,
          .recipe_version = recipe_version_s,
          .runtime_contract = runtime_contract_,
          .actor_obs_contract = actor_obs_contract_,
          .qdes_action_contract = qdes_action_contract_,
          .hitter_action_contract = hitter_action_contract,
          .checkpoint_sha256 = checkpoint_sha,
          .env_cfg_sha256 = env_cfg_sha,
          .task_recipe_sha256 = task_recipe_sha,
          .motion_forehand_sha256 = motion_fh_sha,
          .motion_backhand_sha256 = motion_bh_sha,
          .recovery_recipe = recovery_recipe,
          .curriculum_gates = curriculum_gates,
          .termination_contract = termination_contract,
      });
      if (!error.empty()) throw std::runtime_error(error);
      v17_r1_stationary_replay_ = true;
    }
    if (training_recipe_ == "rally_v17" && !v17_r1_stationary_replay_) {
      if (recipe_version_s == "10") {
        if (load_profile_ != PpOnnxLoadProfile::kV17R10P0Gate3) {
          throw std::runtime_error(
              "V17-r10 is a P0 contract-only artifact "
              "(hardware_authorized=false); only the explicit x86 Gate3 "
              "profile may load it");
        }
        DeployFingerprintValues fingerprint_values;
        fingerprint_values.reserve(kV17R10P0FingerprintKeys.size());
        for (const std::string_view key : kV17R10P0FingerprintKeys) {
          const std::string key_string(key);
          fingerprint_values.emplace_back(
              key_string, LookupMeta(md, alloc, key_string.c_str()));
        }
        const std::string error = ValidateV17R10P0ContractMetadata({
            .training_recipe = training_recipe_,
            .recipe_version = recipe_version_s,
            .runtime_contract = runtime_contract_,
            .deployment_status = deployment_status,
            .qualification_status = qualification_status,
            .manifest_schema = deploy_manifest_schema,
            .manifest_status = deploy_manifest_status,
            .hardware_authorized = deploy_hardware_authorized,
            .fixed_station_contract = v17_fixed_station_contract,
            .release_contract = v17_release_contract,
            .target_stream_contract = v17_target_stream_contract,
            .planner_schema = planner_schema,
            .planner_stability_contract = planner_stability_contract,
            .fixed_hit_plane_relative_x_m = fixed_hit_plane_relative_x_m,
            .contract_fingerprint_sha256 = deploy_contract_fingerprint_sha,
            .manifest_json = deploy_manifest_json,
            .manifest_sha256 = deploy_manifest_sha,
            .fingerprint_values = std::move(fingerprint_values),
        });
        if (!error.empty()) throw std::runtime_error(error);
        v17_r10_p0_gate3_ = true;
      } else if (recipe_version_s == "12") {
        if (load_profile_ != PpOnnxLoadProfile::kProductionStrict) {
          throw std::runtime_error(
              "V17-r12 uses the production V11 runtime profile; a retired "
              "V17 P0/replay load profile is not allowed");
        }
        v17_r12_v11_qdes_tuple_hardware_ = true;
      } else if (recipe_version_s == "6") {
        if (load_profile_ != PpOnnxLoadProfile::kV17R6P0ContractAudit) {
          throw std::runtime_error(
              "V17-r6 is a P0 contract-only artifact "
              "(hardware_authorized=false); production loading is forbidden");
        }
        DeployFingerprintValues fingerprint_values;
        fingerprint_values.reserve(kV17R6P0FingerprintKeys.size());
        for (const std::string_view key : kV17R6P0FingerprintKeys) {
          const std::string key_string(key);
          fingerprint_values.emplace_back(
              key_string, LookupMeta(md, alloc, key_string.c_str()));
        }
        const std::string error = ValidateV17R6P0ContractMetadata({
            .training_recipe = training_recipe_,
            .recipe_version = recipe_version_s,
            .deployment_status = deployment_status,
            .qualification_status = qualification_status,
            .manifest_schema = deploy_manifest_schema,
            .manifest_status = deploy_manifest_status,
            .hardware_authorized = deploy_hardware_authorized,
            .contract_fingerprint_sha256 =
                deploy_contract_fingerprint_sha,
            .manifest_json = deploy_manifest_json,
            .manifest_sha256 = deploy_manifest_sha,
            .fingerprint_values = std::move(fingerprint_values),
        });
        if (!error.empty()) throw std::runtime_error(error);
        if (const std::string joint_error =
                ValidateA3PolicyJointBijection(joint_names_);
            !joint_error.empty())
          throw std::runtime_error(
              "V17-r6 P0 deploy contract mismatch: " + joint_error);
        const Eigen::VectorXd training_kd = ToVec(
            LookupMeta(md, alloc, "a3_training_joint_damping"));
        const Eigen::VectorXd passive_kd = ToVec(
            LookupMeta(md, alloc, "a3_passive_joint_damping"));
        const Eigen::VectorXd effort_limit = ToVec(
            LookupMeta(md, alloc, "a3_joint_effort_limit"));
        if (training_kd.size() != kNumJoints ||
            passive_kd.size() != kNumJoints ||
            effort_limit.size() != kNumJoints)
          throw std::runtime_error(
              "V17-r6 P0 deploy contract actuator arrays must contain 31 values");
        for (int index = 0; index < kNumJoints; ++index) {
          const double expected_scale =
              0.25 * effort_limit[index] / kp_[index];
          if (!std::isfinite(training_kd[index]) ||
              !std::isfinite(passive_kd[index]) ||
              !std::isfinite(effort_limit[index]) ||
              effort_limit[index] <= 0.0 ||
              std::fabs(training_kd[index] -
                        (kd_[index] + passive_kd[index])) > 1.0e-5 ||
              std::fabs(action_scale_[index] - expected_scale) > 2.0e-6)
            throw std::runtime_error(
                "V17-r6 P0 deploy contract actuator/scale invariant failed "
                "for joint " +
                joint_names_[index]);
        }
        const double physics_dt_s = std::stod(
            LookupMeta(md, alloc, "a3_control_physics_dt_s"));
        const int control_decimation = std::stoi(
            LookupMeta(md, alloc, "a3_control_decimation"));
        const double policy_dt_s = std::stod(
            LookupMeta(md, alloc, "a3_control_policy_dt_s"));
        if (!std::isfinite(physics_dt_s) || physics_dt_s <= 0.0 ||
            control_decimation <= 0 ||
            std::fabs(
                policy_dt_s - physics_dt_s * control_decimation) > 1.0e-12 ||
            std::fabs(policy_dt_s - 0.02) > 1.0e-12 ||
            LookupMeta(md, alloc, "qdes_policy_feedback_contract") !=
                "legacy_applied_raw_v1" ||
            !IsDeployLowercaseSha256(qdes_parity_csv_sha))
          throw std::runtime_error(
              "V17-r6 P0 deploy contract timing/feedback/parity invariant failed");
        const Eigen::VectorXd observation_term_dims = ToVec(
            LookupMeta(md, alloc, "actor_obs_term_dims"));
        double observation_dim_sum = 0.0;
        for (int index = 0; index < observation_term_dims.size(); ++index) {
          const double value = observation_term_dims[index];
          if (!std::isfinite(value) || value <= 0.0 ||
              std::fabs(value - std::round(value)) > 1.0e-12)
            throw std::runtime_error(
                "V17-r6 P0 observation term dimensions are invalid");
          observation_dim_sum += value;
        }
        if (obs_dim_ != kObsDim110 ||
            LookupMeta(md, alloc, "actor_obs_total_dim") != "110" ||
            std::fabs(observation_dim_sum - 110.0) > 1.0e-12)
          throw std::runtime_error(
              "V17-r6 P0 observation dimensions do not resolve to 110");
        v17_r6_p0_contract_audit_ = true;
        deploy_manifest_status_ = deploy_manifest_status;
        deploy_manifest_sha256_ = deploy_manifest_sha;
        deploy_contract_fingerprint_sha256_ =
            deploy_contract_fingerprint_sha;
        qdes_parity_csv_sha256_ = qdes_parity_csv_sha;
      } else {
        if (load_profile_ == PpOnnxLoadProfile::kV17R6P0ContractAudit ||
            load_profile_ == PpOnnxLoadProfile::kV17R10P0Gate3) {
          throw std::runtime_error(
              "V17 P0 load profile refuses a mismatched recipe revision");
        }
        const std::string error = ValidateV17R5QualificationMetadata({
            .training_recipe = training_recipe_,
            .recipe_version = recipe_version_s,
            .deployment_status = deployment_status,
            .validator_profile = validator_profile,
            .qualification_status = qualification_status,
            .validator_receipt_sha256 = receipt_sha,
            .checkpoint_sha256 = checkpoint_sha,
            .resolved_task_sha256 = resolved_task_sha,
        });
        if (!error.empty()) throw std::runtime_error(error);
      }
    } else if (
        load_profile_ == PpOnnxLoadProfile::kV17R6P0ContractAudit ||
        load_profile_ == PpOnnxLoadProfile::kV17R10P0Gate3) {
      throw std::runtime_error(
          "V17 P0 load profile requires training_recipe=rally_v17");
    }
    const bool schema24_training_screen_recipe =
        training_recipe_ ==
        "hitter_pingpong_build2_fixed_home_feasible_v1";
    if (schema24_training_screen_recipe) {
      if (load_profile_ !=
          PpOnnxLoadProfile::kSchema24TrainingScreenGate3) {
        throw std::runtime_error(
            "Schema24 TRAINING_SCREENED artifact is not deployable; only "
            "the explicit x86 MuJoCo Gate3-screen profile may load it");
      }
      const std::string error = ValidateSchema24TrainingScreenGate3Metadata({
          .training_recipe = training_recipe_,
          .recipe_version = recipe_version_s,
          .runtime_contract = runtime_contract_,
          .deployment_status = deployment_status,
          .validator_profile = validator_profile,
          .qualification_status = qualification_status,
          .hardware_authorized = deploy_hardware_authorized,
          .gate3_screen_contract = LookupMetaOptional(
              md, alloc, "hitter_pingpong_gate3_screen_contract"),
          .reward_contract = LookupMetaOptional(
              md, alloc, "hitter_pingpong_reward_contract"),
          .question_bank_contract = LookupMetaOptional(
              md, alloc, "hitter_pingpong_question_bank_contract"),
          .question_bank_certification_contract = LookupMetaOptional(
              md, alloc,
              "hitter_pingpong_question_bank_certification_contract"),
          .table_clearance_contract = LookupMetaOptional(
              md, alloc, "hitter_pingpong_table_clearance_contract"),
          .training_admissibility = LookupMetaOptional(
              md, alloc, "hitter_pingpong_training_admissibility"),
          .deployment_qualification = LookupMetaOptional(
              md, alloc, "hitter_pingpong_deployment_qualification"),
          .source_tree_matches_training = LookupMetaOptional(
              md, alloc,
              "hitter_pingpong_export_source_tree_matches_training"),
          .checkpoint_sha256 = checkpoint_sha,
          .training_source_tree_sha256 = LookupMetaOptional(
              md, alloc, "hitter_pingpong_training_source_tree_sha256"),
          .export_source_tree_sha256 = LookupMetaOptional(
              md, alloc, "hitter_pingpong_export_source_tree_sha256"),
          .frozen_env_sha256 = LookupMetaOptional(
              md, alloc, "hitter_pure_frozen_env_sha256"),
          .motion_forehand_sha256 = motion_fh_sha,
          .motion_backhand_sha256 = motion_bh_sha,
          .question_bank_sha256 = LookupMetaOptional(
              md, alloc, "hitter_pingpong_question_bank_sha256"),
          .question_bank_receipt_sha256 = LookupMetaOptional(
              md, alloc, "hitter_pingpong_question_bank_receipt_sha256"),
      });
      if (!error.empty()) throw std::runtime_error(error);
      schema24_training_screen_gate3_ = true;
    } else if (load_profile_ ==
               PpOnnxLoadProfile::kSchema24TrainingScreenGate3) {
      throw std::runtime_error(
          "Schema24 Gate3-screen profile requires the exact fixed-home "
          "feasible training recipe");
    }
    if (v17_r10_p0_gate3_) {
      if (const std::string joint_error =
              ValidateA3PolicyJointBijection(joint_names_);
          !joint_error.empty())
        throw std::runtime_error(
            "V17-r10 P0 deploy contract mismatch: " + joint_error);
      const Eigen::VectorXd training_kd = ToVec(
          LookupMeta(md, alloc, "a3_training_joint_damping"));
      const Eigen::VectorXd passive_kd = ToVec(
          LookupMeta(md, alloc, "a3_passive_joint_damping"));
      const Eigen::VectorXd effort_limit = ToVec(
          LookupMeta(md, alloc, "a3_joint_effort_limit"));
      if (training_kd.size() != kNumJoints ||
          passive_kd.size() != kNumJoints ||
          effort_limit.size() != kNumJoints)
        throw std::runtime_error(
            "V17-r10 P0 actuator arrays must contain 31 values");
      for (int index = 0; index < kNumJoints; ++index) {
        const double expected_scale =
            0.25 * effort_limit[index] / kp_[index];
        if (!std::isfinite(training_kd[index]) ||
            !std::isfinite(passive_kd[index]) ||
            !std::isfinite(effort_limit[index]) ||
            effort_limit[index] <= 0.0 ||
            std::fabs(training_kd[index] -
                      (kd_[index] + passive_kd[index])) > 1.0e-5 ||
            std::fabs(action_scale_[index] - expected_scale) > 2.0e-6)
          throw std::runtime_error(
              "V17-r10 P0 actuator/scale invariant failed for joint " +
              joint_names_[index]);
      }
      const double physics_dt_s = std::stod(
          LookupMeta(md, alloc, "a3_control_physics_dt_s"));
      const int control_decimation = std::stoi(
          LookupMeta(md, alloc, "a3_control_decimation"));
      const double policy_dt_s = std::stod(
          LookupMeta(md, alloc, "a3_control_policy_dt_s"));
      if (!std::isfinite(physics_dt_s) || physics_dt_s <= 0.0 ||
          control_decimation <= 0 ||
          std::fabs(policy_dt_s - physics_dt_s * control_decimation) >
              1.0e-12 ||
          std::fabs(policy_dt_s - 0.02) > 1.0e-12 ||
          LookupMeta(md, alloc, "qdes_policy_feedback_contract") !=
              "legacy_applied_raw_v1" ||
          !IsDeployLowercaseSha256(qdes_parity_csv_sha))
        throw std::runtime_error(
            "V17-r10 P0 timing/feedback/parity invariant failed");
      const Eigen::VectorXd observation_term_dims = ToVec(
          LookupMeta(md, alloc, "actor_obs_term_dims"));
      double observation_dim_sum = 0.0;
      for (int index = 0; index < observation_term_dims.size(); ++index) {
        const double value = observation_term_dims[index];
        if (!std::isfinite(value) || value <= 0.0 ||
            std::fabs(value - std::round(value)) > 1.0e-12)
          throw std::runtime_error(
              "V17-r10 P0 observation term dimensions are invalid");
        observation_dim_sum += value;
      }
      if (obs_dim_ != kObsDim110 ||
          LookupMeta(md, alloc, "actor_obs_total_dim") != "110" ||
          std::fabs(observation_dim_sum - 110.0) > 1.0e-12)
        throw std::runtime_error(
            "V17-r10 P0 observation dimensions do not resolve to 110");
      deploy_manifest_status_ = deploy_manifest_status;
      deploy_manifest_sha256_ = deploy_manifest_sha;
      deploy_contract_fingerprint_sha256_ =
          deploy_contract_fingerprint_sha;
      qdes_parity_csv_sha256_ = qdes_parity_csv_sha;
    }
    auto parse_boxes = [](const std::string& s, std::vector<std::array<double, 6>>& out) {
      std::stringstream ss(s);
      std::string clip;
      while (std::getline(ss, clip, ';')) {
        const Eigen::VectorXd v = ToVec(clip);
        if (v.size() == 6) out.push_back({v[0], v[1], v[2], v[3], v[4], v[5]});
      }
    };
    if (!hp_pos_s.empty()) parse_boxes(hp_pos_s, hp_pos_boxes_);
    if (!hp_vel_s.empty()) parse_boxes(hp_vel_s, hp_vel_boxes_);
    if (!hp_vel_core_s.empty()) parse_boxes(hp_vel_core_s, hp_vel_core_boxes_);
    if (!hp_vel_planner_s.empty()) parse_boxes(hp_vel_planner_s, hp_vel_planner_boxes_);
    if (training_recipe_ ==
        "hitter_pingpong_build2_frozen_core_optional_reach_v1") {
      const std::array<std::array<double, 6>, 2> expected_pos = {{
          {0.58, 0.58, -0.35, -0.25, 0.98, 1.26},
          {0.58, 0.58, -0.14, 0.18, 0.98, 1.26},
      }};
      const std::array<std::array<double, 6>, 2> expected_vel = {{
          {1.26, 2.25, -0.21, 0.65, 0.62, 1.60},
          {1.50, 2.17, -0.47, 0.39, 0.70, 1.35},
      }};
      auto exact_boxes = [](
          const std::vector<std::array<double, 6>>& actual,
          const std::array<std::array<double, 6>, 2>& expected) {
        if (actual.size() != expected.size()) return false;
        for (std::size_t clip = 0; clip < expected.size(); ++clip) {
          for (std::size_t axis = 0; axis < expected[clip].size(); ++axis) {
            if (!std::isfinite(actual[clip][axis]) ||
                std::fabs(actual[clip][axis] - expected[clip][axis]) >
                    1.0e-12) {
              return false;
            }
          }
        }
        return true;
      };
      const Eigen::VectorXd planner_mix = ToVec(hp_vel_planner_mix_s);
      const Eigen::VectorXd ramp_steps = ToVec(hp_vel_ramp_s);
      if (!exact_boxes(hp_pos_boxes_, expected_pos) ||
          !exact_boxes(hp_vel_boxes_, expected_vel) ||
          !exact_boxes(hp_vel_core_boxes_, expected_vel) ||
          !exact_boxes(hp_vel_planner_boxes_, expected_vel) ||
          planner_mix.size() != 1 || ramp_steps.size() != 1 ||
          std::fabs(planner_mix[0]) > 1.0e-12 ||
          std::fabs(ramp_steps[0]) > 1.0e-12 ||
          LookupMetaOptional(md, alloc, "hitter_pure_vel_curriculum") !=
              "coherent_question_bank_rows_fixed_v1") {
        throw std::runtime_error(
            "Schema28 target/component velocity metadata does not match the "
            "receipt-bound production prefilter support or still grants "
            "legacy velocity-curriculum authority");
      }
      hp_vel_planner_mix_prob_ = 0.0;
      hp_vel_range_ramp_steps_ = 0;
    }
    if (!hp_base_s.empty()) {
      const Eigen::VectorXd v = ToVec(hp_base_s);
      if (v.size() == 4) {
        hp_base_range_ = {v[0], v[1], v[2], v[3]};
        has_hp_base_range_ = true;
      }
    }
    if (!hp_step_s.empty()) {
      const Eigen::VectorXd v = ToVec(hp_step_s);
      if (v.size() != 2 || v[0] <= 0.0 || v[1] < v[0])
        throw std::runtime_error(
            "ONNX hitter_pure_station_y_step_range must be two values 0 < lo <= hi");
      hp_station_y_step_range_ = {v[0], v[1]};
      has_hp_station_y_step_range_ = true;
    }
    const HitterPureRuntimeContract runtime_contract =
        small_station_ ? HitterPureRuntimeContract::kRallyFinalV2 :
        validate_hitter_pure_runtime_contract(runtime_contract_, training_recipe_);
    const bool runtime_v1 = runtime_contract == HitterPureRuntimeContract::kRallyFinalV1;
    const bool runtime_v2 = runtime_contract == HitterPureRuntimeContract::kRallyFinalV2;
    const bool runtime_v15 = runtime_contract == HitterPureRuntimeContract::kRallyV15;
    const bool runtime_v17_fixed =
        runtime_contract ==
        HitterPureRuntimeContract::kRallyV17FixedStationBallClockV1;
    const bool build2_fixed_home = uses_build2_fixed_home_contract();
    const bool schema27_optional_reach =
        training_recipe_ ==
        "hitter_pingpong_build2_torso_optional_reach_v2";
    const bool schema28_optional_reach =
        training_recipe_ ==
        "hitter_pingpong_build2_frozen_core_optional_reach_v1";
    const bool schema29_optional_reach =
        training_recipe_ ==
        "hitter_pingpong_build2_trainable_home_optional_reach_v1";
    const bool schema31_optional_reach =
        training_recipe_ == kSchema31TrainingRecipe;
    const bool schema32_slew_optional_reach =
        hitter_schema32_slew_family_recipe(training_recipe_);
    const int expected_fixed_home_obs_dim =
        small_station_ ? obs_dim_ : schema28_optional_reach
            ? kObsDim127
            : (schema27_optional_reach || schema29_optional_reach ||
               schema31_optional_reach || schema32_slew_optional_reach)
                  ? kObsDim112
                  : kObsDim110;
    if (has_hp_station_y_step_range_ && !runtime_v1 && !runtime_v2 &&
        !runtime_v15)
      throw std::runtime_error(
          "ONNX carries RallyFinal station-step metadata but not "
          "hitter_pure_runtime_contract=rally_final_v1/v2/rally_v15; refusing an ambiguous runtime");
    if ((runtime_v1 || runtime_v2) &&
        (obs_dim_ != expected_fixed_home_obs_dim ||
         clip_seg_lengths_.size() != 2 ||
         clip_strike_phases_.size() != 2 || hp_pos_boxes_.size() != 2 ||
         hp_vel_boxes_.size() != 2 || !has_hp_base_range_ ||
         (!build2_fixed_home && !small_station_ && !has_hp_station_y_step_range_)))
      throw std::runtime_error(
          "ONNX RallyFinal runtime requires its exact 110-D/112-D/127-D actor, exactly two clip clocks/pos/vel boxes, "
          "and complete base metadata; schema23/24 fixed HOME omits station-step metadata");
    if (uses_build2_schema22_rapid_preempt_contract() &&
        (!has_hp_base_range_ || !has_hp_station_y_step_range_ ||
         std::fabs(hp_base_range_[0]) > 1.0e-12 ||
         std::fabs(hp_base_range_[1]) > 1.0e-12 ||
         std::fabs(hp_base_range_[2] + 0.06) > 1.0e-12 ||
         std::fabs(hp_base_range_[3] - 0.06) > 1.0e-12 ||
         std::fabs(hp_station_y_step_range_[0] - 0.02) > 1.0e-12 ||
         std::fabs(hp_station_y_step_range_[1] - 0.06) > 1.0e-12)) {
      throw std::runtime_error(
          "build2-rapid-preempt requires exact HOME-relative station x=[0,0], "
          "y=[-0.06,0.06], and command-step support [0.02,0.06]");
    }
    if (build2_fixed_home &&
        (!has_hp_base_range_ || has_hp_station_y_step_range_ ||
         std::fabs(hp_base_range_[0]) > 1.0e-12 ||
         std::fabs(hp_base_range_[1]) > 1.0e-12 ||
         std::fabs(hp_base_range_[2]) > 1.0e-12 ||
         std::fabs(hp_base_range_[3]) > 1.0e-12)) {
      throw std::runtime_error(
          "schema23/24 fixed HOME requires zero base-target range and no station-step metadata");
    }
    if (runtime_v17_fixed &&
        (obs_dim_ != kObsDim110 || clip_seg_lengths_.size() != 2 ||
         clip_strike_phases_.size() != 2 || hp_pos_boxes_.size() != 2 ||
         hp_vel_boxes_.size() != 2 || !has_hp_base_range_ ||
         has_hp_station_y_step_range_ ||
         std::fabs(hp_base_range_[0]) > 1.0e-12 ||
         std::fabs(hp_base_range_[1]) > 1.0e-12 ||
         std::fabs(hp_base_range_[2]) > 1.0e-12 ||
         std::fabs(hp_base_range_[3]) > 1.0e-12))
      throw std::runtime_error(
          "ONNX V17-r10 fixed-station runtime requires 110-D/two clips, "
          "zero base target range, and no station-step metadata");
    // "6" = the v5 default-anchored per-side decode. Recipe "5" was the RETIRED v4
    // full-span single-tanh decode: no v4 checkpoint survives and it is rejected here.
    const bool v15_absolute_gait = runtime_v15 && recipe_version_s == "6";
    const bool v15_feasible_gait = runtime_v15 && recipe_version_s == "4";
    const bool v15_guarded_feasible_gait =
        runtime_v15 && recipe_version_s == "7";
    const bool v15_terminated_feasible_gait =
        runtime_v15 && recipe_version_s == "8";
    const bool v15_finite_gait = runtime_v15 && recipe_version_s == "3";
    const bool v15_legacy = runtime_v15 && recipe_version_s == "2";
    const bool v15_gait_contract_ok =
        locomotion_contract_ == "finite_lateral_gait_v1" &&
        intervention_contract_ ==
            (v15_absolute_gait
                 ? "hugwbc_upper_action_replacement_train_only_v2"
                 : "hugwbc_upper_action_replacement_train_only_v1") &&
        intervention_deploy_value_ == "0" && gait_frequency_hz_ > 0.0 &&
        gait_duty_factor_ > 0.0 && gait_duty_factor_ < 1.0 &&
        gait_move_deadband_ > 0.0 && gait_step_distance_ > 0.0 &&
        gait_max_cycles_ > 0 && gait_velocity_max_ > 0.0;
    if (runtime_v15 &&
        ((!v15_absolute_gait && !v15_feasible_gait &&
          !v15_guarded_feasible_gait && !v15_terminated_feasible_gait &&
          !v15_finite_gait && !v15_legacy) ||
         (v15_absolute_gait &&
          (obs_dim_ != kObsDim118 || !v15_gait_contract_ok ||
           qdes_action_contract_ != "absolute_feasible_qdes_v5")) ||
         (v15_feasible_gait &&
          (obs_dim_ != kObsDim118 || !v15_gait_contract_ok ||
           qdes_action_contract_ != "feasible_qdes_v3")) ||
         (v15_guarded_feasible_gait &&
          (obs_dim_ != kObsDim118 || !v15_gait_contract_ok ||
           qdes_action_contract_ != "feasible_qdes_v3" ||
           qdes_actual_q_guard_contract_ !=
               "predictive_safe_boundary_brake_v1" ||
           std::fabs(qdes_actual_q_guard_horizon_s_ - 0.20) > 1e-12)) ||
         (v15_terminated_feasible_gait &&
          (obs_dim_ != kObsDim118 || !v15_gait_contract_ok ||
           qdes_action_contract_ != "feasible_qdes_v3" ||
           !qdes_actual_q_guard_contract_.empty() ||
           std::fabs(qdes_actual_q_guard_horizon_s_) > 1e-12 ||
           std::fabs(
               qdes_actual_q_hard_tolerance_rad_ - 5.0e-4) > 1e-12)) ||
         (v15_finite_gait && (obs_dim_ != kObsDim118 || !v15_gait_contract_ok)) ||
         (v15_legacy && obs_dim_ != kObsDim113) ||
         clip_seg_lengths_.size() != 2 ||
         clip_strike_phases_.size() != 2 || hp_pos_boxes_.size() != 2 ||
         hp_vel_boxes_.size() != 2 || hp_vel_core_boxes_.size() != 2 ||
         hp_vel_planner_boxes_.size() != 2 || !has_hp_base_range_ ||
         !has_hp_station_y_step_range_ ||
         ((!v15_absolute_gait && !v15_feasible_gait &&
           !v15_guarded_feasible_gait && !v15_terminated_feasible_gait &&
           qdes_action_contract_ != "bounded_qdes_v2") ||
          ((v15_feasible_gait || v15_guarded_feasible_gait ||
            v15_terminated_feasible_gait) &&
           qdes_action_contract_ != "feasible_qdes_v3") ||
          (v15_absolute_gait &&
           qdes_action_contract_ != "absolute_feasible_qdes_v5")) ||
         actor_obs_contract_ != "hitter_pure_v15" ||
         base_localization_contract_ != "position_receipt_v1" ||
         !(base_velocity_ema_alpha_ > 0.0 && base_velocity_ema_alpha_ <= 1.0) ||
         !(base_localization_max_age_s_ > 0.0)))
      throw std::runtime_error(
          "ONNX rally_v15 requires either legacy recipe-v2/113-D or finite-gait "
          "recipe-v3/118-D bounded-qdes, recipe-v4/118-D incremental feasible-qdes, "
          "recipe-v6/118-D absolute feasible-qdes-v5, or "
          "recipe-v7/118-D guarded feasible-qdes, or "
          "recipe-v8/118-D feasible-qdes with per-environment physical termination "
          "hitter_pure_v15, complete two-clip/component geometry and "
          "position_receipt_v1 metadata");
    if (training_recipe_ == "rally_final_v3") {
      const Eigen::VectorXd mix = ToVec(hp_mix_s);
      const Eigen::VectorXd ready_hold = ToVec(ready_hold_s);
      if (mix.size() != 6 || mix[0] < 0.0 || mix[1] < 0.0 || mix[0] + mix[1] > 1.0 ||
          mix[2] <= 0.0 || mix[3] < mix[2] || mix[4] <= 0.0 || mix[5] < mix[4])
        throw std::runtime_error(
            "ONNX rally_final_v3 requires valid station mixture metadata "
            "same_prob,small_prob,small_lo,small_hi,main_lo,main_hi");
      if (hp_side_s != "true")
        throw std::runtime_error(
            "ONNX rally_final_v3 requires hitter_pure_station_side_explicit=true");
      if (validator_profile != "strict_final_v3" && validator_profile != "v7_approved")
        throw std::runtime_error(
            "ONNX rally_final_v3 requires validator_profile=strict_final_v3 or v7_approved");
      const bool strict_profile = validator_profile == "strict_final_v3";
      const double min_y_separation = strict_profile ? 0.20 : 0.10;
      if (hp_pos_boxes_.size() != 2 ||
          hp_pos_boxes_[1][2] - hp_pos_boxes_[0][2] < min_y_separation - 1e-9)
        throw std::runtime_error(
            "ONNX rally_final_v3 FH/BH station-relative y separation violates validator profile");
      if (hp_pos_boxes_.size() != 2 || hp_pos_boxes_[0][4] < 0.65 - 1e-9 ||
          hp_pos_boxes_[0][5] > 1.35 + 1e-9 || hp_pos_boxes_[1][4] < 0.65 - 1e-9 ||
          hp_pos_boxes_[1][5] > 1.35 + 1e-9)
        throw std::runtime_error(
            "ONNX rally_final_v3 requires both target z boxes inside [0.65,1.35] m");
      const double min_x_low = strict_profile ? 0.5 : -0.05;
      if (hp_vel_boxes_.size() != 2 || hp_vel_boxes_[0][0] < min_x_low - 1e-9 ||
          hp_vel_boxes_[1][0] < min_x_low - 1e-9)
        throw std::runtime_error(
            "ONNX rally_final_v3 velocity-box x low violates validator profile");
      for (const auto& box : hp_vel_boxes_) {
        const double max_x = std::max(std::fabs(box[0]), std::fabs(box[1]));
        const double max_y = std::max(std::fabs(box[2]), std::fabs(box[3]));
        const double max_z = std::max(std::fabs(box[4]), std::fabs(box[5]));
        if (box[4] < -1e-9 || std::sqrt(max_x * max_x + max_y * max_y + max_z * max_z) >
                                  3.5 + 1e-9)
          throw std::runtime_error(
              "ONNX rally_final_v3 velocity box has z_lo<0 or max corner norm >3.5 m/s");
      }
      if (recipe_version_s != "3" || ready_hold.size() != 2 ||
          std::fabs(ready_hold[0] - 40.0) > 1e-9 ||
          std::fabs(ready_hold[1] - 60.0) > 1e-9)
        throw std::runtime_error(
            "ONNX rally_final_v3 requires recipe_version=3 and ready hold 40,60");
      auto is_sha256 = [](const std::string& value) {
        if (value.size() != 64) return false;
        for (const char ch : value)
          if (!((ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f'))) return false;
        return true;
      };
      if (!is_sha256(motion_fh_sha) || !is_sha256(motion_bh_sha) ||
          !is_sha256(receipt_sha) || !is_sha256(env_cfg_sha))
        throw std::runtime_error(
            "ONNX rally_final_v3 requires immutable motion/validator/env sha256 provenance");
      if (LookupMetaOptional(md, alloc, "hitter_pure_passive_head") !=
              "head_yaw_joint,head_pitch_joint" ||
          last_action_head_mode_ != "zero" ||
          LookupMetaOptional(md, alloc, "hitter_pure_termination_contract") !=
              "physical_fall_only")
        throw std::runtime_error(
            "ONNX rally_final_v3 is missing passive-head or physical-termination metadata");
      station_y_mixture_ = {mix[0], mix[1], mix[2], mix[3], mix[4], mix[5]};
      station_side_explicit_ = true;
    }
    if (training_recipe_ == "hitter_pingpong_continuous_rally_v1" ||
        training_recipe_ == "hitter_pingpong_continuous_rally_v2" ||
        training_recipe_ == "hitter_pingpong_continuous_rally_v3" ||
        training_recipe_ == "hitter_pingpong_continuous_rally_v4" ||
        training_recipe_ == "hitter_pingpong_build2_minimal_rootfix_v1" ||
        training_recipe_ == "hitter_pingpong_build2_rapid_preempt_v1") {
      const Eigen::VectorXd mix = ToVec(hp_mix_s);
      const Eigen::VectorXd ready_hold = ToVec(ready_hold_s);
      const bool continuous_rally_v3 =
          training_recipe_ == "hitter_pingpong_continuous_rally_v3";
      const bool continuous_rally_v4 =
          training_recipe_ == "hitter_pingpong_continuous_rally_v4";
      const bool build2_minimal_rootfix =
          training_recipe_ == "hitter_pingpong_build2_minimal_rootfix_v1";
      const bool build2_rapid_preempt =
          training_recipe_ == "hitter_pingpong_build2_rapid_preempt_v1";
      const bool memoryless_continuous =
          continuous_rally_v3 || continuous_rally_v4;
      const bool small_y_contract =
          memoryless_continuous || build2_minimal_rootfix ||
          build2_rapid_preempt;
      const std::array<double, 6> expected_mix =
          small_y_contract
              ? std::array<double, 6>{0.80, 0.20, 0.02, 0.06, 0.02, 0.06}
              : std::array<double, 6>{0.70, 0.30, 0.03, 0.12, 0.03, 0.12};
      bool mix_ok = mix.size() == static_cast<Eigen::Index>(expected_mix.size());
      if (mix_ok) {
        for (std::size_t i = 0; i < expected_mix.size(); ++i) {
          mix_ok = mix_ok &&
              std::fabs(mix[static_cast<Eigen::Index>(i)] - expected_mix[i]) <=
                  1.0e-12;
        }
      }
      const bool base_range_ok = has_hp_base_range_ &&
          std::fabs(hp_base_range_[0]) <= 1.0e-12 &&
          std::fabs(hp_base_range_[1]) <= 1.0e-12 &&
          std::fabs(
              hp_base_range_[2] + (small_y_contract ? 0.06 : 0.12)) <=
              1.0e-12 &&
          std::fabs(
              hp_base_range_[3] - (small_y_contract ? 0.06 : 0.12)) <=
              1.0e-12;
      const bool station_step_ok = has_hp_station_y_step_range_ &&
          std::fabs(
              hp_station_y_step_range_[0] -
              (small_y_contract ? 0.02 : 0.03)) <= 1.0e-12 &&
          std::fabs(
              hp_station_y_step_range_[1] -
              (small_y_contract ? 0.06 : 0.12)) <= 1.0e-12;
      const bool fixed_plane = hp_pos_boxes_.size() == 2 &&
          std::fabs(hp_pos_boxes_[0][0] - 0.58) <= 1.0e-9 &&
          std::fabs(hp_pos_boxes_[0][1] - 0.58) <= 1.0e-9 &&
          std::fabs(hp_pos_boxes_[1][0] - 0.58) <= 1.0e-9 &&
          std::fabs(hp_pos_boxes_[1][1] - 0.58) <= 1.0e-9;
      const std::string expected_recipe_version =
          (build2_minimal_rootfix || build2_rapid_preempt)
              ? "1"
              : training_recipe_ == "hitter_pingpong_continuous_rally_v4"
              ? "4"
              : training_recipe_ == "hitter_pingpong_continuous_rally_v3"
              ? "3"
              : training_recipe_ == "hitter_pingpong_continuous_rally_v2"
                  ? "2"
                  : "1";
      const std::array<double, 2> expected_ready_hold =
          hitter_pingpong_ready_hold_steps_contract(training_recipe_);
      const std::string expected_actor_obs_contract = continuous_rally_v4
          ? "hitter_pure_110_headslots_vxy_v1" : "hitter_pure";
      const bool v4_headslot_order_ok = !continuous_rally_v4 ||
          hitter_pure_v4_velocity_slot_joint_order_matches(joint_names_);
      if (recipe_version_s != expected_recipe_version || !mix_ok ||
          hp_side_s != "true" ||
          ready_hold.size() != 2 ||
          std::fabs(ready_hold[0] - expected_ready_hold[0]) > 1.0e-12 ||
          std::fabs(ready_hold[1] - expected_ready_hold[1]) > 1.0e-12 ||
          obs_dim_ != kObsDim110 ||
          actor_obs_contract_ != expected_actor_obs_contract ||
          !v4_headslot_order_ok ||
          !(has_v11_affine_safe_qdes_contract() ||
            has_v12_affine_safe_slew_qdes_contract()) ||
          qdes_safe_lo_.size() != kNumJoints ||
          qdes_safe_hi_.size() != kNumJoints ||
          qdes_hard_lo_.size() != kNumJoints ||
          qdes_hard_hi_.size() != kNumJoints ||
          !base_range_ok || !station_step_ok || !fixed_plane ||
          hp_vel_boxes_.size() != 2 ||
          LookupMetaOptional(md, alloc, "hitter_pure_passive_head") !=
              "head_yaw_joint,head_pitch_joint" ||
          last_action_head_mode_ != "zero" ||
          LookupMetaOptional(md, alloc, "hitter_pure_termination_contract") !=
              "physical_fall_qhard_telemetry_mocap_stale_nonterminal_v2" ||
          LookupMetaOptional(md, alloc, "hitter_pure_reach_x_augmented") !=
              "true" ||
          LookupMetaOptional(md, alloc, "hitter_pure_ready_deadline") !=
              "disabled" ||
          (!(build2_minimal_rootfix || build2_rapid_preempt) &&
           LookupMetaOptional(md, alloc, "hitter_pure_heading_debt") !=
               "disabled") ||
          LookupMetaOptional(md, alloc, "hitter_pure_right_elbow_extension") !=
              "disabled" ||
          (!(build2_minimal_rootfix || build2_rapid_preempt) &&
           LookupMetaOptional(md, alloc, "hitter_pure_post_swing_xlock") !=
               "disabled") ||
          base_localization_contract_ != "calibrated_pose_receipt_v2" ||
          base_pose_source_ != "mocap_authoritative" ||
          base_pose_schema_ != "2" ||
          orientation_contract_ != "calibrated_world_base_v1" ||
          angular_velocity_contract_ != "mocap_anchored_pelvis_gyro_v1" ||
          yaw_align_contract_ != "disabled_for_mocap_authoritative" ||
          world_frame_contract_ != "table_p1_to_p2_v1" ||
          calibration_contract_ != "runtime_receipt_required" ||
          (continuous_rally_v4 &&
           std::fabs(base_velocity_ema_alpha_ - 0.25) > 1.0e-12) ||
          std::fabs(base_localization_max_age_s_ - 0.05) > 1.0e-12 ||
          std::fabs(base_localization_max_propagation_s_ - 0.05) > 1.0e-12) {
        throw std::runtime_error(
            "ONNX continuous-rally HitterPingPong has drifted from its "
            "110-D affine-q_des, small-y, balanced-side, or calibrated-mocap contract");
      }
      station_y_mixture_ = {
          mix[0], mix[1], mix[2], mix[3], mix[4], mix[5]};
      station_side_explicit_ = true;
    }
    if (training_recipe_ == "rally_v15") {
      const Eigen::VectorXd mix = ToVec(hp_mix_s);
      const Eigen::VectorXd ready_hold = ToVec(ready_hold_s);
      const Eigen::VectorXd planner_mix = ToVec(hp_vel_planner_mix_s);
      const Eigen::VectorXd ramp_steps = ToVec(hp_vel_ramp_s);
      const bool small_range_ok = mix.size() == 6 &&
          ((mix[1] <= 0.0 && std::fabs(mix[2]) <= 1e-9 && std::fabs(mix[3]) <= 1e-9) ||
           (mix[1] > 0.0 && mix[2] > 0.0 && mix[3] >= mix[2]));
      if ((recipe_version_s != "2" && recipe_version_s != "3" &&
           recipe_version_s != "4" && recipe_version_s != "6" &&
           recipe_version_s != "7" && recipe_version_s != "8") ||
          ready_hold.size() != 2 || ready_hold[0] < 0.0 ||
          ready_hold[1] < ready_hold[0] || mix.size() != 6 || mix[0] < 0.0 ||
          mix[1] < 0.0 || mix[0] + mix[1] > 1.0 || !small_range_ok ||
          mix[4] <= 0.0 || mix[5] < mix[4] ||
          planner_mix.size() != 1 || planner_mix[0] < 0.0 || planner_mix[0] > 1.0 ||
          ramp_steps.size() != 1 || ramp_steps[0] < 0.0 ||
          (recipe_version_s == "7" &&
           qdes_actual_q_guard_contract_ !=
               "predictive_safe_boundary_brake_v1") ||
          (recipe_version_s == "7" &&
           std::fabs(qdes_actual_q_guard_horizon_s_ - 0.20) > 1e-12))
        throw std::runtime_error(
            "ONNX rally_v15 has incomplete or malformed YAML-derived hold/station/velocity metadata");
      const std::string expected_qdes_projector = recipe_version_s == "6"
          ? "default_anchored_per_side_tanh+feasible_projection+rate+tracking+pd_headroom"
          : (recipe_version_s == "7"
                 ? "feasible_interval_tanh+safe+rate+tracking+pd_headroom"
                   "+predictive_safe_boundary_brake"
                 : ((recipe_version_s == "4" || recipe_version_s == "8")
                        ? "feasible_interval_tanh+safe+rate+tracking+pd_headroom"
                        : "local_scale_safe_tanh+rate+tracking+pd_headroom"));
      const std::string expected_termination_contract =
          recipe_version_s == "7"
              ? "physical_fall_plus_q_hard_audit_v1"
              : (recipe_version_s == "8"
                     ? "physical_fall_plus_q_hard_termination_v2"
                     : "physical_fall_only");
      if (hp_side_s != "true" ||
          LookupMetaOptional(md, alloc, "hitter_pure_passive_head") !=
              "head_yaw_joint,head_pitch_joint" ||
          last_action_head_mode_ != "zero" ||
          LookupMetaOptional(md, alloc, "hitter_pure_last_action_feedback") !=
              "executed_qdes_normalized" ||
          LookupMetaOptional(md, alloc, "hitter_pure_termination_contract") !=
              expected_termination_contract ||
          LookupMetaOptional(md, alloc, "hitter_pure_qdes_projector") !=
              expected_qdes_projector)
        throw std::runtime_error(
            "ONNX rally_v15 is missing executed-q_des, passive-head, position-mocap or "
            "physical-termination contract metadata");
      auto contained_by = [](const std::array<double, 6>& inner,
                             const std::array<double, 6>& outer) {
        return inner[0] >= outer[0] - 1e-9 && inner[1] <= outer[1] + 1e-9 &&
               inner[2] >= outer[2] - 1e-9 && inner[3] <= outer[3] + 1e-9 &&
               inner[4] >= outer[4] - 1e-9 && inner[5] <= outer[5] + 1e-9;
      };
      for (std::size_t c = 0; c < 2; ++c) {
        if (!contained_by(hp_vel_core_boxes_[c], hp_vel_boxes_[c]) ||
            !contained_by(hp_vel_planner_boxes_[c], hp_vel_boxes_[c]))
          throw std::runtime_error(
              "ONNX rally_v15 core/planner velocity component escapes its YAML safety union");
      }
      hp_vel_planner_mix_prob_ = planner_mix[0];
      hp_vel_range_ramp_steps_ = static_cast<int>(ramp_steps[0]);
      station_y_mixture_ = {mix[0], mix[1], mix[2], mix[3], mix[4], mix[5]};
      station_side_explicit_ = true;
    }
    if (v17_r10_p0_gate3_) {
      auto is_sha256 = [](const std::string& value) {
        if (value.size() != 64) return false;
        for (const char ch : value)
          if (!((ch >= '0' && ch <= '9') ||
                (ch >= 'a' && ch <= 'f')))
            return false;
        return true;
      };
      const bool fixed_plane = hp_pos_boxes_.size() == 2 &&
          std::fabs(hp_pos_boxes_[0][0] - 0.58) <= 1.0e-9 &&
          std::fabs(hp_pos_boxes_[0][1] - 0.58) <= 1.0e-9 &&
          std::fabs(hp_pos_boxes_[1][0] - 0.58) <= 1.0e-9 &&
          std::fabs(hp_pos_boxes_[1][1] - 0.58) <= 1.0e-9;
      const bool sensor_contract =
          LookupMetaOptional(md, alloc, "hitter_pure_v17_sensor_contract") ==
              "mocap_authoritative,2,calibrated_world_base_v1,"
              "mocap_anchored_pelvis_gyro_v1,"
              "disabled_for_mocap_authoritative,"
              "table_p1_to_p2_v1,runtime_receipt_required" &&
          base_localization_contract_ == "calibrated_pose_receipt_v2" &&
          base_pose_source_ == "mocap_authoritative" &&
          base_pose_schema_ == "2" &&
          orientation_contract_ == "calibrated_world_base_v1" &&
          angular_velocity_contract_ ==
              "mocap_anchored_pelvis_gyro_v1" &&
          yaw_align_contract_ == "disabled_for_mocap_authoritative" &&
          world_frame_contract_ == "table_p1_to_p2_v1" &&
          calibration_contract_ == "runtime_receipt_required" &&
          std::fabs(base_localization_max_age_s_ - 0.05) <= 1.0e-12 &&
          std::fabs(
              base_localization_max_propagation_s_ -
              (uses_home_preempt_rally_contract()
                   ? 0.0 : 0.05)) <=
              1.0e-12;
      if (!fixed_plane || !sensor_contract ||
          actor_obs_contract_ != "hitter_pure" ||
          qdes_action_contract_ != "v11_affine_safe_qdes_v1" ||
          std::fabs(qdes_actual_q_hard_tolerance_rad_ - 0.002) > 1.0e-12 ||
          qdes_safe_lo_.size() != kNumJoints ||
          qdes_safe_hi_.size() != kNumJoints ||
          qdes_hard_lo_.size() != kNumJoints ||
          qdes_hard_hi_.size() != kNumJoints ||
          hitter_action_contract != "v11_affine_safe_qdes_v1" ||
          LookupMetaOptional(md, alloc, "hitter_pure_v17_recipe_revision") !=
              "10" ||
          LookupMetaOptional(md, alloc, "hitter_pure_v17_runtime_handoff") !=
              "static_to_policy_blend_v2,0.0500,50,150,0.5000" ||
          !is_sha256(checkpoint_sha))
        throw std::runtime_error(
            "ONNX rally_v17-r10 P0 has drifted from its fixed-station, "
            "110-D affine-q_des, or calibrated-mocap contract");
    }
    if (!v17_r10_p0_gate3_ &&
        (training_recipe_ == "rally_v8" || training_recipe_ == "rally_v9" ||
        training_recipe_ == "rally_v10" || training_recipe_ == "rally_v11" ||
        training_recipe_ == "rally_v12" || training_recipe_ == "rally_v13" ||
        training_recipe_ == "rally_v14" ||
        training_recipe_ == "hitter_pingpong_recovery_tts_v2" ||
        training_recipe_ == "hitter_pingpong_recovery_tts_v3" ||
        training_recipe_ == "rally_v17")) {
      const bool is_v9 = training_recipe_ == "rally_v9";
      const bool is_v10 = training_recipe_ == "rally_v10";
      const bool is_v11 = training_recipe_ == "rally_v11";
      const bool is_v12 = training_recipe_ == "rally_v12";
      const bool is_v13 = training_recipe_ == "rally_v13";
      const bool is_v14 = training_recipe_ == "rally_v14";
      const bool is_hitter_repair =
          training_recipe_ == "hitter_pingpong_recovery_tts_v2" ||
          training_recipe_ == "hitter_pingpong_recovery_tts_v3";
      const bool is_v17 = training_recipe_ == "rally_v17";
      // The repair keeps V14's deploy geometry/action/sensor contract. Its
      // distinct identity changes WAIT/TTS/recovery semantics only, so it must
      // still pass every V14-derived fail-closed check below.
      const bool is_v14_like = is_v14 || is_hitter_repair;
      const bool is_v13_plus = is_v13 || is_v14_like;
      const bool is_component_recipe =
          is_v10 || is_v11 || is_v12 || is_v13_plus || is_v17;
      const bool is_augmented = is_v9 || is_component_recipe;
      const char* recipe_label =
          is_v17 ? "rally_v17" :
          (is_hitter_repair ? training_recipe_.c_str() :
          (is_v14 ? "rally_v14" :
          (is_v13 ? "rally_v13" :
          (is_v12 ? "rally_v12" :
          (is_v11 ? "rally_v11" :
           (is_v10 ? "rally_v10" : (is_v9 ? "rally_v9" : "rally_v8")))))));
      // RallyV8--V13: the merged Pure->V3->V4 recipe on the v13 clips. Contract:
      // v7 plane 0.70 + ARM-REACH y bands (mostly-stationary hitter — the runner keeps the
      // station when the target is in-band), station mixture metadata, explicit planner side,
      // true training hold 25,125 (+ a long tail stamped in hitter_pure_hold_long).
      const Eigen::VectorXd mix = ToVec(hp_mix_s);
      const Eigen::VectorXd ready_hold = ToVec(ready_hold_s);
      if (mix.size() != 6 || mix[0] < 0.0 || mix[1] < 0.0 || mix[0] + mix[1] > 1.0 ||
          mix[2] <= 0.0 || mix[3] < mix[2] || mix[4] <= 0.0 || mix[5] < mix[4])
        throw std::runtime_error(
            std::string("ONNX ") + recipe_label +
            " requires valid station mixture metadata "
            "same_prob,small_prob,small_lo,small_hi,main_lo,main_hi");
      if (mix[0] < 0.5 - 1e-9)
        throw std::runtime_error(
            "ONNX rally_v8 mixture same_prob < 0.5 — the mostly-stationary contract drifted");
      if (hp_side_s != "true")
        throw std::runtime_error(
            "ONNX rally_v8 requires hitter_pure_station_side_explicit=true");
      const std::string expected_version = is_hitter_repair
          ? (uses_home_preempt_rally_contract() ? "2" : "3") :
          (is_v10 ? "7" :
          (is_v17 ? (v17_r12_v11_qdes_tuple_hardware_ ? "12" :
                     (v17_r1_stationary_replay_ ? "1" :
                     (v17_r6_p0_contract_audit_ ? "6" : "5"))) :
          ((is_v11 || is_v12 || is_v13_plus) ? "1" :
                                                     (is_v9 ? "5" : "4"))));
      const double expected_hold_lo =
          (is_v11 || is_v12 || is_v13_plus || is_v17) ? 45.0 : 25.0;
      const double expected_hold_hi =
          (is_v11 || is_v12 || is_v13_plus || is_v17) ? 60.0 : 125.0;
      if (recipe_version_s != expected_version || ready_hold.size() != 2 ||
          std::fabs(ready_hold[0] - expected_hold_lo) > 1e-9 ||
          std::fabs(ready_hold[1] - expected_hold_hi) > 1e-9)
        throw std::runtime_error(
            std::string("ONNX ") + recipe_label + " requires recipe_version=" +
            expected_version + " and recipe-matched training hold");
      if (hp_pos_boxes_.size() != 2 ||
          hp_pos_boxes_[1][2] - hp_pos_boxes_[0][2] < 0.10 - 1e-9 ||
          hp_pos_boxes_[1][2] - hp_pos_boxes_[0][3] < 0.03 - 1e-9)
        throw std::runtime_error(
            "ONNX rally_v8 FH/BH arm-reach y bands must be disjoint around the side split");
      if (hp_pos_boxes_[0][4] < 0.65 - 1e-9 || hp_pos_boxes_[0][5] > 1.35 + 1e-9 ||
          hp_pos_boxes_[1][4] < 0.65 - 1e-9 || hp_pos_boxes_[1][5] > 1.35 + 1e-9)
        throw std::runtime_error(
            "ONNX rally_v8 requires both target z boxes inside [0.65,1.35] m");
      for (const auto& box : hp_vel_boxes_) {
        const double max_x = std::max(std::fabs(box[0]), std::fabs(box[1]));
        const double max_y = std::max(std::fabs(box[2]), std::fabs(box[3]));
        const double max_z = std::max(std::fabs(box[4]), std::fabs(box[5]));
        if (box[0] < -0.05 - 1e-9 || box[4] < -1e-9 ||
            std::sqrt(max_x * max_x + max_y * max_y + max_z * max_z) > 3.5 + 1e-9)
          throw std::runtime_error(
              "ONNX rally_v8 velocity box violates x_lo>=-0.05 / z_lo>=0 / corner<=3.5 m/s");
      }
      if (is_augmented) {
        const std::array<double, 4> want_x =
            is_component_recipe ? std::array<double, 4>{0.58, 0.58, 0.58, 0.58}
                   : std::array<double, 4>{0.60, 0.70, 0.45, 0.55};
        if (hp_pos_boxes_.size() != 2 ||
            std::fabs(hp_pos_boxes_[0][0] - want_x[0]) > 1e-9 ||
            std::fabs(hp_pos_boxes_[0][1] - want_x[1]) > 1e-9 ||
            std::fabs(hp_pos_boxes_[1][0] - want_x[2]) > 1e-9 ||
            std::fabs(hp_pos_boxes_[1][1] - want_x[3]) > 1e-9)
          throw std::runtime_error(
              std::string("ONNX ") + recipe_label +
              " has wrong reach-x boxes (V9 FH [0.60,0.70] BH [0.45,0.55]; "
              "V10/V11/V12/V13 FH/BH [0.58,0.58])");
        if (LookupMetaOptional(md, alloc, "hitter_pure_passive_head") !=
                "head_yaw_joint,head_pitch_joint" ||
            last_action_head_mode_ != "zero" ||
            LookupMetaOptional(md, alloc, "hitter_pure_termination_contract") !=
                (uses_home_preempt_rally_contract()
                     ? "physical_fall_qhard_telemetry_mocap_stale_nonterminal_v2"
                     : (uses_late_reveal_raw_tts_contract()
                     ? "physical_fall_mocap_outage_continue_qhard_telemetry_v2"
                     : "rally_v8_reference_feet_no_wrist")) ||
            LookupMetaOptional(md, alloc, "hitter_pure_reach_x_augmented") != "true")
          throw std::runtime_error(
              "ONNX rally_v9/v10/v11/v12/v13 is missing passive-head, no-wrist or reach-x metadata");
      }
      if (is_component_recipe &&
          (LookupMetaOptional(md, alloc, "hitter_pure_left_wrist_reference") !=
               (uses_home_preempt_rally_contract()
                    ? "real_model21800_synchronized_wait_v1"
                    : "hold_frame0_swing_current") ||
           LookupMetaOptional(md, alloc, "hitter_pure_pre_settle_t_max") != "1.1000" ||
           LookupMetaOptional(md, alloc, "hitter_pure_yaw_settle") != "first_order" ||
           LookupMetaOptional(md, alloc, "hitter_pure_strike_x_drift") !=
               "0.0200,0.0500,1.1000,1.5500,huber" ||
           LookupMetaOptional(md, alloc, "hitter_pure_right_elbow_extension") !=
               (is_v14_like ? "1.2500,0.1200,0.3000,0.2000,forehand_only" :
                (is_v13 ? "1.2500,0.1200,0.3000,0.1200,forehand_only" :
                          "1.3000,0.1500,0.2500,0.1000")) ||
           LookupMetaOptional(md, alloc, "hitter_pure_fixed_plane_x") != "0.5800" ||
           LookupMetaOptional(md, alloc, "hitter_pure_vel_curriculum") !=
               (is_hitter_repair && !uses_home_preempt_rally_contract()
                    ? "v13_core25+planner_q05_q95_75,fixed_final"
                    : "v13_core25+planner_q05_q95_75,96000") ||
           (is_hitter_repair && !uses_home_preempt_rally_contract() &&
            LookupMetaOptional(
                md, alloc, "hitter_pure_vel_curriculum_progress_override") !=
                "1.0000"))) {
        throw std::runtime_error(
            "ONNX rally_v10/v11/v12/v13 is missing fixed-plane/wrist/yaw/timing/x-drift/elbow/velocity-curriculum metadata");
      }
      if (is_v10 &&
          LookupMetaOptional(md, alloc, "hitter_pure_joint_qdes_max_blend") != "0.5000")
        throw std::runtime_error("ONNX rally_v10 requires joint_qdes_max_blend=0.5000");
      const std::string expected_candidate_status =
          (is_v17 && v17_r6_p0_contract_audit_)
              ? "p0_contract_candidate"
              : "gate3_candidate";
      if ((is_v11 || is_v12 || is_v17) &&
          (LookupMetaOptional(md, alloc, "hitter_pure_deployment_status") !=
               expected_candidate_status ||
           LookupMetaOptional(md, alloc, "hitter_pure_joint_qdes_max_blend") != "1.0000" ||
           LookupMetaOptional(md, alloc, "hitter_pure_ready_deadline") !=
               "0.1000,0.1000,0.2000,0.1200,class3" ||
           LookupMetaOptional(md, alloc, "hitter_pure_station_y_positive_main") !=
               (uses_home_preempt_rally_contract()
                    ? "0.0000,symmetric_home_relative"
                    : "0.8000,0.1900,0.2400") ||
           LookupMetaOptional(md, alloc, "hitter_pure_ready_stance") !=
               "0.1000,0.1500,0.2000,0.2500,0.3500" ||
           LookupMetaOptional(md, alloc, "hitter_pure_left_wrist_debt") !=
               (uses_home_preempt_rally_contract()
                    ? "disabled_canonical_synchronized_wait"
                    : "0.0800,0.7500") ||
           LookupMetaOptional(md, alloc, "hitter_pure_heading_debt") !=
               "0.1000,0.5000,1.5500"))
        throw std::runtime_error(
            "ONNX rally_v11/v12/v17 is missing the Gate3 "
            "or P0 candidate/deadline/station/stance metadata");
      if (is_v17) {
        const std::string expected_recovery =
            "markov_side_phase_severity_v3,0.1000,12288,384,4,3,"
            "0.1800,0.3200,0.1200,0.0700,0.1000,1.1000,"
            "16,100,100,50";
        const std::string expected_gates =
            "200,400;500,1000,200,1000,500;"
            "0.7500,0.7000,0.7500,0.7500,0.6000,0.8000,0.8500,"
            "0.8500,0.7500,0.6500,0.0300,0.0000,500;"
            "0.6500,0.5500,0.6000,0.6000,0.4500,0.7000,0.7500,"
            "0.7500,0.6500,0.5000,0.0500,0.0050,150;"
            "8000,8000,4000,12000,6000;250,500;"
            "release_eligible_completion_v1";
        auto is_sha256 = [](const std::string& value) {
          if (value.size() != 64) return false;
          for (const char ch : value)
            if (!((ch >= '0' && ch <= '9') ||
                  (ch >= 'a' && ch <= 'f')))
              return false;
          return true;
        };
        if (actor_obs_contract_ != "hitter_pure" ||
            qdes_action_contract_ != "v11_affine_safe_qdes_v1" ||
            std::fabs(qdes_actual_q_hard_tolerance_rad_ - 0.002) > 1e-12 ||
            qdes_safe_lo_.size() != kNumJoints ||
            qdes_safe_hi_.size() != kNumJoints ||
            qdes_hard_lo_.size() != kNumJoints ||
            qdes_hard_hi_.size() != kNumJoints ||
            LookupMetaOptional(md, alloc, "hitter_pure_action_contract") !=
                "v11_affine_safe_qdes_v1")
          throw std::runtime_error(
              "ONNX rally_v17 is missing the exact V11 affine-safe action contract");
        if (v17_r12_v11_qdes_tuple_hardware_ &&
            (LookupMetaOptional(
                 md, alloc, "hitter_pure_v17_recipe_revision") != "12" ||
             LookupMetaOptional(
                 md, alloc, "hitter_pure_v17_qdes_training") !=
                 "v11_action_rate_joint_limit_saturation_plus_"
                 "unitree_joint_acc_v1,-0.100000,-0.000000250" ||
             LookupMetaOptional(
                 md, alloc, "hitter_pure_v17_runtime_handoff") !=
                 "static_to_policy_blend_v2,0.0500,50,150,0.5000" ||
             LookupMetaOptional(
                 md, alloc, "hitter_pure_v17_sensor_contract") !=
                 "mocap_authoritative,2,calibrated_world_base_v1,"
                 "mocap_anchored_pelvis_gyro_v1,"
                 "disabled_for_mocap_authoritative,"
                 "table_p1_to_p2_v1,runtime_receipt_required" ||
             LookupMetaOptional(md, alloc, "hitter_pure_planner_schema") !=
                 "2" ||
             LookupMetaOptional(
                 md, alloc, "hitter_pure_planner_stability_contract") !=
                 "three_revisions_v1,0.0300,0.2500,0.0300" ||
             LookupMetaOptional(
                 md, alloc, "hitter_pure_v17_target_stream_contract") !=
                 "freeze_at_engage_v1" ||
             LookupMetaOptional(
                 md, alloc, "hitter_pure_v17_venue_tuple") !=
                 "fixed_balanced_high_fidelity_bank_v1,0.2500,50000,50000,"
                 "bc202d6335473f15c2233de7ceece42795dd18b12acf557a21bd611969fc5d03,"
                 "90fdba0a631f96a89ebc970403b7b49345fc94e9678ac4f075e9df963eb28005,"
                 "unconditional_swing_denominator_v1" ||
             base_localization_contract_ != "calibrated_pose_receipt_v2" ||
             base_pose_source_ != "mocap_authoritative" ||
             base_pose_schema_ != "2" ||
             orientation_contract_ != "calibrated_world_base_v1" ||
             angular_velocity_contract_ !=
                 "mocap_anchored_pelvis_gyro_v1" ||
             yaw_align_contract_ !=
                 "disabled_for_mocap_authoritative" ||
             world_frame_contract_ != "table_p1_to_p2_v1" ||
             calibration_contract_ != "runtime_receipt_required" ||
             std::fabs(base_localization_max_age_s_ - 0.05) > 1e-12 ||
             std::fabs(base_localization_max_propagation_s_ - 0.05) >
                 1e-12 ||
             deployment_status != "gate3_candidate" ||
             validator_profile != "v17_r12_v11_qdes_tuple_hardware_v1" ||
             qualification_status != "strong_stability_candidate" ||
             !is_sha256(receipt_sha) ||
             !is_sha256(checkpoint_sha) ||
             task_recipe_sha !=
                 "624b458dfc3d3403c79f0d50902c0468b0c0b0dcdc8fe5567efc36c1d738488c" ||
             resolved_task_sha !=
                 "4395851f3a97336043fb0ed3aaf0fd026ab4f327cc61a4540c06325a24db3dfa"))
          throw std::runtime_error(
              "ONNX rally_v17-r12 is missing its exact V11 q_des, physical tuple, "
              "full-mocap, handoff, planner, qualification, or provenance contract");
        if (!v17_r12_v11_qdes_tuple_hardware_ &&
            !v17_r1_stationary_replay_ &&
            !v17_r6_p0_contract_audit_ &&
            (LookupMetaOptional(
                 md, alloc, "hitter_pure_v17_recipe_revision") != "5" ||
             LookupMetaOptional(
                 md, alloc, "hitter_pure_v17_recovery_recipe") !=
                 expected_recovery ||
             LookupMetaOptional(
                 md, alloc, "hitter_pure_v17_curriculum_gates") !=
                 expected_gates ||
             LookupMetaOptional(
                 md, alloc, "hitter_pure_v17_ready_release") !=
                 "scale_sampled_ready_latch_v1,0.3000,1.5000,0.1000,"
                 "0.1000,0.2000,0.1500,0.2000,0.1000,0.3500,0.0300,5" ||
             LookupMetaOptional(
                 md, alloc, "hitter_pure_v17_venue_tuple") !=
                 "correlated_mirror_law_reference_hemisphere_v2,"
                 "0.7500,3.5000,8,2.1700,"
                 "3.0400,-0.5000,0.5000" ||
             LookupMetaOptional(
                 md, alloc, "hitter_pure_v17_recovery_safe_set") !=
                 "bounded_max_top3_safe_set_ready_hold_rational_v3,-0.3500,"
                 "0.1000,1.5500,3,0.5000,1" ||
             LookupMetaOptional(
                 md, alloc, "hitter_pure_v17_runtime_handoff") !=
                 "static_to_policy_blend_v2,0.0500,50,150,0.5000" ||
             LookupMetaOptional(
                 md, alloc, "hitter_pure_v17_sensor_contract") !=
                 "mocap_authoritative,2,calibrated_world_base_v1,"
                 "mocap_anchored_pelvis_gyro_v1,"
                 "disabled_for_mocap_authoritative,"
                 "table_p1_to_p2_v1,runtime_receipt_required" ||
             base_localization_contract_ != "calibrated_pose_receipt_v2" ||
             base_pose_source_ != "mocap_authoritative" ||
             base_pose_schema_ != "2" ||
             orientation_contract_ != "calibrated_world_base_v1" ||
             angular_velocity_contract_ !=
                 "mocap_anchored_pelvis_gyro_v1" ||
             yaw_align_contract_ !=
                 "disabled_for_mocap_authoritative" ||
             world_frame_contract_ != "table_p1_to_p2_v1" ||
             calibration_contract_ != "runtime_receipt_required" ||
             std::fabs(base_localization_max_age_s_ - 0.05) > 1e-12 ||
             std::fabs(base_localization_max_propagation_s_ - 0.05) >
                 1e-12 ||
             LookupMetaOptional(
                 md, alloc, "hitter_pure_v17_projection_aux") !=
                 "executable_equivalence_class_v1,0.0100,0.0500,0.1000" ||
             !is_sha256(checkpoint_sha) || !is_sha256(env_cfg_sha) ||
             motion_fh_sha !=
                 "a6c68513720b12b168379cd6fa13f8b77607b4fa0bf7e828c4e1d81eda6f2094" ||
             motion_bh_sha !=
                 "67d04e13deeed068bdb003e379e18330dcd29210d280188fab7af26c0764eaac" ||
             task_recipe_sha !=
                 "b24a5b1a2749bc32722cbdc038d68bc003e15b641b1b69e8df73ccd8856691ea"))
          throw std::runtime_error(
              "ONNX rally_v17 is missing the exact reversible-recovery "
              "curriculum or immutable checkpoint/task/motion provenance");
        if (v17_r6_p0_contract_audit_ &&
            (LookupMetaOptional(
                 md, alloc, "hitter_pure_v17_recipe_revision") != "6" ||
             LookupMetaOptional(
                 md, alloc, "hitter_pure_v17_sensor_contract") !=
                 "mocap_authoritative,2,calibrated_world_base_v1,"
                 "mocap_anchored_pelvis_gyro_v1,"
                 "disabled_for_mocap_authoritative,"
                 "table_p1_to_p2_v1,runtime_receipt_required" ||
             base_localization_contract_ != "calibrated_pose_receipt_v2" ||
             base_pose_source_ != "mocap_authoritative" ||
             base_pose_schema_ != "2" ||
             orientation_contract_ != "calibrated_world_base_v1" ||
             angular_velocity_contract_ !=
                 "mocap_anchored_pelvis_gyro_v1" ||
             yaw_align_contract_ !=
                 "disabled_for_mocap_authoritative" ||
             world_frame_contract_ != "table_p1_to_p2_v1" ||
             calibration_contract_ != "runtime_receipt_required" ||
             std::fabs(base_localization_max_age_s_ - 0.05) > 1e-12 ||
             std::fabs(base_localization_max_propagation_s_ - 0.05) >
                 1e-12 ||
             !is_sha256(checkpoint_sha)))
          throw std::runtime_error(
              "ONNX rally_v17-r6 P0 manifest carries the wrong "
              "sensor/localization/checkpoint contract");
      }
      if (is_v12 &&
          (LookupMetaOptional(md, alloc, "hitter_pure_strike_x_gate_margin") !=
               "0.0150,0.0200,0.0400" ||
           LookupMetaOptional(md, alloc, "hitter_pure_idle_left_wrist_debt") !=
               (uses_home_preempt_rally_contract()
                    ? "disabled_canonical_synchronized_wait"
                    : "0.0200,0.1200,0.0500,0.2500,0.2500,0.7500,outward") ||
           LookupMetaOptional(md, alloc, "hitter_pure_waist_qdes_saturation") !=
               "soft_limit,0.1000,1.0000"))
        throw std::runtime_error(
            "ONNX rally_v12 is missing contact-x/idle-wrist/waist-qdes residual metadata");
      if (is_v13_plus &&
          (LookupMetaOptional(md, alloc, "hitter_pure_deployment_status") !=
               "gate3_candidate" ||
           LookupMetaOptional(md, alloc, "hitter_pure_ready_deadline") !=
               "0.1000,0.1000,0.2000,0.1200,classes1+2+3" ||
           LookupMetaOptional(md, alloc, "hitter_pure_station_y_positive_main") !=
               (uses_home_preempt_rally_contract()
                    ? "0.0000,symmetric_home_relative"
                    : "0.8000,0.1900,0.2400") ||
           LookupMetaOptional(md, alloc, "hitter_pure_ready_stance") !=
               "0.1000,0.1500,0.2000,0.2500,0.3500" ||
           LookupMetaOptional(md, alloc, "hitter_pure_left_wrist_debt") !=
               (uses_home_preempt_rally_contract()
                    ? "disabled_canonical_synchronized_wait"
                    : "0.0800,0.7500") ||
           LookupMetaOptional(md, alloc, "hitter_pure_heading_debt") !=
               (is_v14_like ? "0.1000,0.5000,1.5500,1.2500,1.5000" :
                         "0.1000,0.5000,1.5500,1.0000,1.5000") ||
           LookupMetaOptional(md, alloc, "hitter_pure_strike_x_gate_margin") !=
               (is_v14_like ? "0.0150,0.0200,0.0400,0.7500,1.2500" :
                         "0.0150,0.0200,0.0400,0.5000,1.2500") ||
           LookupMetaOptional(md, alloc, "hitter_pure_idle_left_wrist_debt") !=
               (uses_home_preempt_rally_contract()
                    ? "disabled_canonical_synchronized_wait"
                    : "0.0200,0.1200,0.0500,0.2500,0.2500,0.7500,outward") ||
           LookupMetaOptional(md, alloc, "hitter_pure_all_joint_qdes_barrier") !=
               (is_v14_like ? "official_hard,0.0800,0.0300,4,0.9000,all31" :
                         "official_hard,0.0500,0.0300,4,0.7500,all31") ||
           LookupMetaOptional(md, alloc, "hitter_pure_post_swing_xlock") !=
               "0.0300,0.0400,0.1000,1.5500,1.0000,1.2500" ||
           !LookupMetaOptional(md, alloc, "hitter_pure_joint_qdes_max_blend").empty() ||
           !LookupMetaOptional(md, alloc, "hitter_pure_waist_qdes_saturation").empty()))
        throw std::runtime_error(
            "ONNX rally_v13/v14 is missing its all-joint/side-specific Gate3 candidate metadata or carries obsolete V11/V12 qdes metadata");
      if (is_v14_like &&
          LookupMetaOptional(md, alloc, "hitter_pure_v14_stability_recipe") !=
              (is_hitter_repair
                   ? (uses_home_preempt_rally_contract()
                          ? "home_rally_push_v3,-0.6500,-3.0000,-0.3500,"
                            "-0.4000,0.6000,3.0000,"
                            "hold_default=-0.2500,all_hold=0,"
                            "qdes_hold_return=-0.1000"
                          : "recovery_tts_markov_v3,-0.6500,0.0000,-0.3500,"
                            "-0.4000,0.6000,5.4000,"
                            "hold_default=-0.2500,all_hold=1")
                   : (uses_late_reveal_raw_tts_contract()
                          ? "recovery_tts_markov_v1,-0.6500,-2.0000,-0.3500,"
                            "-0.4000,0.6000,3.0000"
                          : "0.3500,-0.6500,0.0800,0.0300,4,0.9000,-0.5000,0.2000,"
                            "0.7500,-0.4500,-0.5000,1.2500,-0.4000,-0.3500,-0.3500,-0.7000")))
        throw std::runtime_error(
            std::string("ONNX ") + recipe_label +
            " is missing the YAML-derived stability recipe metadata");
      if (is_component_recipe) {
        const std::array<std::array<double, 6>, 2> expected_union = {{
            {1.24, 2.60, -0.31, 0.69, 0.40, 1.66},
            {1.50, 2.60, -0.66, 0.40, 0.00, 1.35},
        }};
        const std::array<std::array<double, 6>, 2> expected_core = {{
            {1.24, 2.24, -0.31, 0.69, 0.66, 1.66},
            {1.60, 2.60, -0.66, 0.34, 0.00, 0.54},
        }};
        const std::array<std::array<double, 6>, 2> expected_planner = {{
            {1.57, 2.55, 0.10, 0.52, 0.41, 1.35},
            {1.55, 2.52, -0.18, 0.29, 0.40, 1.32},
        }};
        auto boxes_equal = [](const std::vector<std::array<double, 6>>& actual,
                              const std::array<std::array<double, 6>, 2>& expected) {
          if (actual.size() != expected.size()) return false;
          for (std::size_t c = 0; c < expected.size(); ++c)
            for (std::size_t i = 0; i < expected[c].size(); ++i)
              if (std::fabs(actual[c][i] - expected[c][i]) > 1e-9) return false;
          return true;
        };
        if (!boxes_equal(hp_vel_boxes_, expected_union) ||
            !boxes_equal(hp_vel_core_boxes_, expected_core) ||
            !boxes_equal(hp_vel_planner_boxes_, expected_planner))
          throw std::runtime_error(
              "ONNX rally_v10/v11/v12 velocity union/core/planner boxes drifted from recipe-7");

        const Eigen::VectorXd planner_mix = ToVec(hp_vel_planner_mix_s);
        const Eigen::VectorXd ramp_steps = ToVec(hp_vel_ramp_s);
        if (planner_mix.size() != 1 || ramp_steps.size() != 1 ||
            std::fabs(planner_mix[0] - 0.75) > 1e-9 ||
            std::fabs(ramp_steps[0] - 96000.0) > 1e-9)
          throw std::runtime_error(
              "ONNX rally_v10/v11/v12 requires planner mix 0.75 and velocity ramp 96000");
        hp_vel_planner_mix_prob_ = planner_mix[0];
        hp_vel_range_ramp_steps_ = static_cast<int>(ramp_steps[0]);

        // The final union is retained as a coarse speed-safety envelope only.  Every sampled
        // component must be contained by it, but deploy membership is core OR planner (never
        // arbitrary Cartesian corners of the bounding union).
        auto contained_by = [](const std::array<double, 6>& inner,
                               const std::array<double, 6>& outer) {
          return inner[0] >= outer[0] - 1e-9 && inner[1] <= outer[1] + 1e-9 &&
                 inner[2] >= outer[2] - 1e-9 && inner[3] <= outer[3] + 1e-9 &&
                 inner[4] >= outer[4] - 1e-9 && inner[5] <= outer[5] + 1e-9;
        };
        for (std::size_t c = 0; c < 2; ++c) {
          if (!contained_by(hp_vel_core_boxes_[c], hp_vel_boxes_[c]) ||
              !contained_by(hp_vel_planner_boxes_[c], hp_vel_boxes_[c]))
            throw std::runtime_error(
                "ONNX rally_v10/v11 velocity component escapes the final safety union");
          const auto& u = hp_vel_boxes_[c];
          const auto& p = hp_vel_planner_boxes_[c];
          const std::array<double, 3> center = {
              0.5 * (u[0] + u[1]), 0.5 * (u[2] + u[3]), 0.5 * (u[4] + u[5])};
          if (!velocity_in_box(
                  p, center[0], center[1], center[2], kVelocityBoxContractTolerance))
            throw std::runtime_error(
                "ONNX rally_v10/v11 --demo union center is outside the planner component");
        }
      }
      station_y_mixture_ = {mix[0], mix[1], mix[2], mix[3], mix[4], mix[5]};
      station_side_explicit_ = true;
    }
    const auto rally_end = LookupMetaOptional(md, alloc, "hitter_pingpong_rally_end_contract");
    if (!rally_end.empty()) {
      if (rally_end != "ball_clock_followthrough_v1" ||
          runtime_contract_ != "rally_final_v2" || obs_dim_ != kObsDim110 ||
          actor_obs_contract_ != "hitter_pure" ||
          qdes_action_contract_ != "v11_affine_safe_qdes_v1" ||
          !hitter_pingpong_command_contract_.empty())
        throw std::runtime_error("unsupported rally end/observation/action contract");
      ball_clock_followthrough_s_ = std::stod(
          LookupMetaOptional(md, alloc, "hitter_pingpong_rally_post_strike_s"));
      if (!std::isfinite(ball_clock_followthrough_s_) || ball_clock_followthrough_s_ <= 0.0)
        throw std::runtime_error("rally post-strike duration must be finite and positive");
    }
    if (!deploy_cfg_path.empty()) {
      BindDeployConfig_(model_path, deploy_cfg_path);
    }
  }

  bool uses_compact_execution_obs() const { return compact_execution_; }
  bool uses_small_station_contract() const { return small_station_; }
  // Runtime identity is independent of training provenance. Shared by the
  // offline inspector and the live Runner/report receipt.
  const char* policy_abi() const {
    return compact_execution_ ? "small_station_324_v1" :
           small_station_ ? "small_station_112_v1" :
           uses_ball_clock_followthrough() ? "ball_clock_110_v1" : "legacy";
  }
  bool uses_ball_clock_followthrough() const { return ball_clock_followthrough_s_ > 0.0; }
  double ball_clock_followthrough_s() const { return ball_clock_followthrough_s_; }
  int obs_dim() const { return obs_dim_; }
  // Per-clip layout baked by new exports (empty on legacy models -> caller keeps its default).
  bool has_clip_layout() const { return !clip_seg_lengths_.empty(); }
  const std::vector<double>& clip_seg_lengths() const { return clip_seg_lengths_; }
  const std::vector<double>& clip_strike_phases() const { return clip_strike_phases_; }
  // Per-clip reference reach offsets (empty when the export predates the metadata key).
  bool has_reach_offsets() const { return !reach_offsets_.empty(); }
  const std::vector<Vec2>& reach_offsets() const { return reach_offsets_; }
  // hitter_pure sampling geometry (empty on non-pure exports). Box layout per clip:
  // {x_lo, x_hi, y_lo, y_hi, z_lo, z_hi} — pos boxes station-relative x/y + absolute z.
  bool has_hitter_pure_boxes() const { return !hp_pos_boxes_.empty(); }
  const std::vector<std::array<double, 6>>& hp_pos_boxes() const { return hp_pos_boxes_; }
  const std::vector<std::array<double, 6>>& hp_vel_boxes() const { return hp_vel_boxes_; }
  const std::vector<std::array<double, 6>>& hp_vel_core_boxes() const {
    return hp_vel_core_boxes_;
  }
  const std::vector<std::array<double, 6>>& hp_vel_planner_boxes() const {
    return hp_vel_planner_boxes_;
  }
  double hp_vel_planner_mix_prob() const { return hp_vel_planner_mix_prob_; }
  int hp_vel_range_ramp_steps() const { return hp_vel_range_ramp_steps_; }
  const std::array<double, 4>& hp_base_range() const { return hp_base_range_; }
  bool has_hitter_pure_base_range() const { return has_hp_base_range_; }
  bool has_hitter_pure_station_y_step_range() const { return has_hp_station_y_step_range_; }
  bool is_rally_final_contract() const {
    return runtime_contract_ == "rally_final_v1" || runtime_contract_ == "rally_final_v2" ||
           runtime_contract_ == "rally_v15" ||
           runtime_contract_ == "rally_v17_fixed_station_ball_clock_v1";
  }
  bool requires_component_velocity_gate() const {
    if (small_station_) return false;  // one hash-bound SmallStation bank
    return runtime_contract_ == "rally_final_v2" ||
           runtime_contract_ == "rally_v15" ||
           runtime_contract_ == "rally_v17_fixed_station_ball_clock_v1";
  }
  bool is_hitter_pure_obs() const {
    return compact_execution_ || obs_dim_ == kObsDim110 || obs_dim_ == kObsDim112 ||
           obs_dim_ == kObsDim127 || obs_dim_ == kObsDim113 ||
           obs_dim_ == kObsDim118;
  }
  bool has_finite_lateral_gait() const {
    return obs_dim_ == kObsDim118 && locomotion_contract_ == "finite_lateral_gait_v1";
  }
  bool uses_position_mocap_obs() const {
    return obs_dim_ == kObsDim113 || obs_dim_ == kObsDim118;
  }
  bool uses_authoritative_mocap_pose() const {
    // Full-pose mocap is an observation contract, not a training-recipe
    // property.  HitterPingPong deliberately keeps the V14 control recipe
    // while changing only the provenance of four 110-D actor terms.  Keying
    // this path on rally_v17 would therefore accept the metadata at export but
    // silently feed the deployed policy IMU/perfect-tracking observations.
    return is_hitter_pure_obs() &&
           base_localization_contract_ == "calibrated_pose_receipt_v2" &&
           base_pose_source_ == "mocap_authoritative" &&
           base_pose_schema_ == "2" &&
           orientation_contract_ == "calibrated_world_base_v1" &&
           angular_velocity_contract_ ==
               "mocap_anchored_pelvis_gyro_v1" &&
           yaw_align_contract_ == "disabled_for_mocap_authoritative" &&
           world_frame_contract_ == "table_p1_to_p2_v1" &&
           calibration_contract_ == "runtime_receipt_required" &&
           base_localization_max_age_s_ > 0.0 &&
           base_localization_max_propagation_s_ >= 0.0 &&
           base_localization_max_propagation_s_ <=
               base_localization_max_age_s_;
  }
  bool has_bounded_qdes_contract() const {
    return qdes_action_contract_ == "bounded_qdes_v1" ||
           qdes_action_contract_ == "bounded_qdes_v2" ||
           qdes_action_contract_ == "feasible_qdes_v3" ||
           qdes_action_contract_ == "absolute_feasible_qdes_v5";
  }
  bool has_v11_affine_safe_qdes_contract() const {
    return qdes_action_contract_ == "v11_affine_safe_qdes_v1";
  }
  // v12 = V11 affine-safe decode + runner-side per-tick slew (leg budget scaled by
  // tilt/speed) + executed-q_des raw-action feedback.  Never runs the bounded projector.
  bool has_v12_affine_safe_slew_qdes_contract() const {
    return qdes_action_contract_ == kV12AffineSafeSlewQdesContract;
  }
  bool has_safe_qdes_interval_contract() const {
    return has_v11_affine_safe_qdes_contract() ||
           has_v12_affine_safe_slew_qdes_contract() ||
           has_bounded_qdes_contract();
  }
  const Eigen::VectorXd& qdes_slew_limit_rad_per_tick() const {
    return qdes_slew_limit_rad_per_tick_;
  }
  const Eigen::VectorXd& qdes_slew_leg_mask() const { return qdes_slew_leg_mask_; }
  const V12SlewScaleParams& qdes_slew_scale_params() const {
    return qdes_slew_scale_params_;
  }
  bool has_feasible_qdes_contract() const {
    return qdes_action_contract_ == "feasible_qdes_v3";
  }
  bool has_absolute_feasible_qdes_contract() const {
    return qdes_action_contract_ == "absolute_feasible_qdes_v5";
  }
  bool is_rally_final_v3_recipe() const { return training_recipe_ == "rally_final_v3"; }
  bool is_rally_v8_recipe() const { return training_recipe_ == "rally_v8"; }
  bool is_rally_v9_recipe() const { return training_recipe_ == "rally_v9"; }
  bool is_rally_v10_recipe() const { return training_recipe_ == "rally_v10"; }
  bool is_rally_v11_recipe() const { return training_recipe_ == "rally_v11"; }
  bool is_rally_v12_recipe() const { return training_recipe_ == "rally_v12"; }
  bool is_rally_v13_recipe() const { return training_recipe_ == "rally_v13"; }
  bool is_rally_v14_recipe() const { return training_recipe_ == "rally_v14"; }
  bool is_rally_v15_recipe() const { return training_recipe_ == "rally_v15"; }
  bool is_rally_v17_recipe() const { return training_recipe_ == "rally_v17"; }
  bool is_hitter_pingpong_recovery_tts_recipe() const {
    return training_recipe_ == "hitter_pingpong_recovery_tts_v2" ||
           training_recipe_ == "hitter_pingpong_recovery_tts_v3" ||
           training_recipe_ == "hitter_pingpong_continuous_rally_v1" ||
           training_recipe_ == "hitter_pingpong_continuous_rally_v2" ||
           training_recipe_ == "hitter_pingpong_continuous_rally_v3" ||
           training_recipe_ == "hitter_pingpong_continuous_rally_v4" ||
           training_recipe_ == "hitter_pingpong_build2_minimal_rootfix_v1" ||
           training_recipe_ == "hitter_pingpong_build2_rapid_preempt_v1" ||
           training_recipe_ ==
               "hitter_pingpong_build2_fixed_home_recovery_v1" ||
           training_recipe_ ==
               "hitter_pingpong_build2_fixed_home_feasible_v1" ||
           training_recipe_ ==
               "hitter_pingpong_build2_fixed_home_phase_recovery_v1" ||
           training_recipe_ ==
               "hitter_pingpong_build2_home_support_frame_v1" ||
           training_recipe_ ==
               "hitter_pingpong_build2_torso_optional_reach_v2" ||
           training_recipe_ ==
               "hitter_pingpong_build2_frozen_core_optional_reach_v1" ||
           hitter_schema31_candidate_family_recipe(training_recipe_);
  }
  bool uses_late_reveal_raw_tts_contract() const {
    return uses_small_station_contract() ||
           hitter_pingpong_command_contract_ == "late_reveal_raw_tts_v1" ||
           hitter_pingpong_command_contract_ == "late_reveal_raw_tts_v2" ||
           hitter_pingpong_command_contract_ == "home_preempt_raw_tts_v3" ||
           hitter_pingpong_command_contract_ ==
               "strike_followthrough_home_external_commit_v1" ||
           hitter_pingpong_command_contract_ ==
               "strike_followthrough_home_external_commit_v2" ||
           hitter_pingpong_command_contract_ ==
               "strike_followthrough_home_external_commit_v3" ||
           hitter_pingpong_command_contract_ ==
               "strike_followthrough_home_external_commit_v4" ||
           hitter_pingpong_command_contract_ ==
               "native_clip_end_immediate_resample_v1" ||
           hitter_pingpong_command_contract_ ==
               "postcontact_fresh_flight_preempt_v1" ||
           hitter_pingpong_command_contract_ ==
               "fixed_home_fresh_flight_preempt_v1";
  }
  bool uses_build2_minimal_rootfix_contract() const {
    return training_recipe_ == "hitter_pingpong_build2_minimal_rootfix_v1" &&
           hitter_pingpong_command_contract_ ==
               "native_clip_end_immediate_resample_v1" &&
           hitter_pingpong_planner_commit_contract_ ==
               "native_clip_end_immediate_resample_v1" &&
           hitter_pingpong_wait_clock_contract_ ==
               "cold_hold_neutral_hidden_until_release_v1" &&
           hitter_pingpong_trajectory_teacher_contract_ ==
               "contact_linear_actor_visible_v1" &&
           actor_obs_contract_ == "hitter_pure";
  }
  bool uses_build2_schema22_rapid_preempt_contract() const {
    return training_recipe_ == "hitter_pingpong_build2_rapid_preempt_v1" &&
           training_recipe_version_ == "1" &&
           hitter_pingpong_command_contract_ ==
               "postcontact_fresh_flight_preempt_v1" &&
           hitter_pingpong_planner_commit_contract_ ==
               "fresh_schema2_flight_after_protected_followthrough_v1" &&
           hitter_pingpong_transition_contract_ ==
               "policy_owned_110d_markov_no_qdes_override_v1" &&
           hitter_pingpong_wait_clock_contract_ ==
               "cold_hold_neutral_hidden_until_release_v1" &&
           hitter_pingpong_trajectory_teacher_contract_ ==
               "contact_linear_actor_visible_v1" &&
           hitter_pingpong_motion_phase_contract_ ==
               "signed_tts_contact_frame_v1" &&
           actor_obs_contract_ == "hitter_pure";
  }
  bool uses_build2_fixed_home_contract() const {
    const bool home_support_frame =
        training_recipe_ == "hitter_pingpong_build2_home_support_frame_v1";
    const bool torso_optional_reach =
        training_recipe_ == "hitter_pingpong_build2_torso_optional_reach_v2";
    const bool frozen_core_shadow =
        training_recipe_ ==
        "hitter_pingpong_build2_frozen_core_optional_reach_v1";
    const bool trainable_schema29 =
        training_recipe_ ==
        "hitter_pingpong_build2_trainable_home_optional_reach_v1";
    const bool trainable_schema31 =
        training_recipe_ == kSchema31TrainingRecipe;
    const bool trainable_schema32_slew_family =
        hitter_schema32_slew_family_recipe(training_recipe_);
    const bool trainable_schema34 = training_recipe_ == kSchema34TrainingRecipe;
    const bool optional_reach_family =
        torso_optional_reach || frozen_core_shadow || trainable_schema29 ||
        trainable_schema31 || trainable_schema32_slew_family;
    const bool support_frame_family =
        home_support_frame || optional_reach_family;
    const bool fixed_home_recipe =
        training_recipe_ ==
            "hitter_pingpong_build2_fixed_home_recovery_v1" ||
        training_recipe_ ==
            "hitter_pingpong_build2_fixed_home_feasible_v1" ||
        training_recipe_ ==
            "hitter_pingpong_build2_fixed_home_phase_recovery_v1" ||
        support_frame_family;
    return fixed_home_recipe &&
           (training_recipe_version_ == (torso_optional_reach ? "2" : "1") ||
            (trainable_schema34 && (training_recipe_version_ == "2" || training_recipe_version_ == "3"))) &&
           hitter_pingpong_command_contract_ ==
               "fixed_home_fresh_flight_preempt_v1" &&
           hitter_pingpong_planner_commit_contract_ ==
               (optional_reach_family &&
                        hitter_optional_reach_mode_uses_schema3(
                            optional_reach_runtime_mode_)
                    ? "fresh_schema3_flight_shared_session_home_support_intent_v2"
                    : "fresh_schema2_flight_fixed_session_home_v1") &&
           hitter_pingpong_transition_contract_ ==
               (trainable_schema34
                    ? kSchema34TransitionContract
                : trainable_schema32_slew_family
                    ? kSchema32TransitionContract
                : (trainable_schema29 || trainable_schema31)
                    ? "policy_owned_112d_markov_trainable_no_qdes_override_v1"
                : frozen_core_shadow
                    ? "policy_owned_127d_markov_frozen_core_shadow_residual_no_qdes_override_v1"
                : torso_optional_reach
                    ? "policy_owned_112d_markov_frozen_core_residual_no_qdes_override_v3"
                : home_support_frame
                    ? "policy_owned_110d_markov_headslots_vxy_no_qdes_override_v2"
                    : "policy_owned_110d_markov_no_qdes_override_v1") &&
           hitter_pingpong_wait_clock_contract_ ==
               "cold_hold_neutral_hidden_until_release_v1" &&
           hitter_pingpong_trajectory_teacher_contract_ ==
               "contact_linear_actor_visible_v1" &&
           hitter_pingpong_motion_phase_contract_ ==
               "signed_tts_contact_frame_v1" &&
           actor_obs_contract_ ==
               (frozen_core_shadow
                    ? "hitter_pure_127_headslots_vxy_reach_core_shadow_v1"
                : trainable_schema34
                    ? (training_recipe_version_ == "3" ? kSchema34StepActorObsContract : kSchema34ActorObsContract)
                : (torso_optional_reach || trainable_schema29 ||
                   trainable_schema31 || trainable_schema32_slew_family)
                    ? "hitter_pure_112_headslots_vxy_reach_v1"
                : home_support_frame
                    ? "hitter_pure_110_headslots_vxy_v1"
                    : "hitter_pure");
  }
  bool uses_schema27_optional_reach_obs() const {
    const bool schema27 =
        training_recipe_ ==
            "hitter_pingpong_build2_torso_optional_reach_v2" &&
        obs_dim_ == kObsDim112;
    const bool schema28 =
        training_recipe_ ==
            "hitter_pingpong_build2_frozen_core_optional_reach_v1" &&
        obs_dim_ == kObsDim127;
    const bool schema29 =
        training_recipe_ ==
            "hitter_pingpong_build2_trainable_home_optional_reach_v1" &&
        obs_dim_ == kObsDim112;
    const bool schema31 =
        training_recipe_ == kSchema31TrainingRecipe &&
        obs_dim_ == kObsDim112;
    const bool schema32_slew_family =
        hitter_schema32_slew_family_recipe(training_recipe_) &&
        obs_dim_ == kObsDim112;
    return (schema27 || schema28 || schema29 || schema31 ||
            schema32_slew_family) &&
           uses_build2_fixed_home_contract();
  }
  bool uses_schema34_step_task_obs() const {
    return training_recipe_ == kSchema34TrainingRecipe && training_recipe_version_ == "3" &&
           actor_obs_contract_ == kSchema34StepActorObsContract && obs_dim_ == kObsDim112 && uses_build2_fixed_home_contract();
  }
  bool uses_schema34_signed_support_obs() const {
    return training_recipe_ == kSchema34TrainingRecipe &&
           actor_obs_contract_ == kSchema34ActorObsContract &&
           obs_dim_ == kObsDim112 && uses_build2_fixed_home_contract();
  }
  bool uses_schema28_core_shadow_obs() const {
    return training_recipe_ ==
               "hitter_pingpong_build2_frozen_core_optional_reach_v1" &&
           uses_build2_fixed_home_contract() && obs_dim_ == kObsDim127;
  }
  bool optional_reach_runtime_enabled() const {
    return uses_schema27_optional_reach_obs() &&
           optional_reach_runtime_mode_ ==
               HitterOptionalReachRuntimeMode::kEnabledSchema3;
  }
  bool is_schema31_training_candidate() const noexcept {
    return hitter_schema31_candidate_family_recipe(training_recipe_);
  }
  void enable_schema31_sim_optional_reach() {
    if (!is_schema31_training_candidate() ||
        optional_reach_runtime_mode_ !=
            HitterOptionalReachRuntimeMode::kCandidateSchema3Level0Only) {
      throw std::runtime_error(
          "Schema31/32 simulator optional-reach override requires the exact "
          "non-deployment-qualified training-candidate artifact");
    }
    optional_reach_runtime_mode_ =
        HitterOptionalReachRuntimeMode::kEnabledSchema3;
  }
  bool optional_reach_schema3_wire_required() const {
    return uses_schema27_optional_reach_obs() &&
           hitter_optional_reach_mode_uses_schema3(
               optional_reach_runtime_mode_);
  }
  bool planner_reach_wire_is_compatible(
      int wire_schema, double reach_level,
      double swing_foot_sign) const noexcept {
    const bool schema27_actor = uses_schema27_optional_reach_obs();
    return hitter_optional_reach_wire_is_compatible(
        schema27_actor, optional_reach_runtime_mode_, wire_schema,
        reach_level, swing_foot_sign);
  }
  // Both schemas share the external preempt clock. Callers that need station semantics must
  // branch on uses_build2_fixed_home_contract() rather than infer them from this lifecycle bit.
  bool uses_build2_rapid_preempt_contract() const {
    return small_station_ || uses_build2_schema22_rapid_preempt_contract() ||
           uses_build2_fixed_home_contract();
  }
  bool uses_continuous_rally_v2_contract() const {
    return (hitter_pingpong_command_contract_ ==
                "strike_followthrough_home_external_commit_v2" &&
            hitter_pingpong_planner_commit_contract_ ==
                "fresh_shot_external_event_policy_independent_v2") ||
           uses_build2_minimal_rootfix_contract();
  }
  bool uses_continuous_rally_v3_contract() const {
    return hitter_pingpong_command_contract_ ==
               "strike_followthrough_home_external_commit_v3" &&
           hitter_pingpong_planner_commit_contract_ ==
               "fresh_shot_external_event_policy_independent_v2" &&
           hitter_pingpong_wait_clock_contract_ ==
               "constant_pending_wait_v1";
  }
  bool uses_continuous_rally_v4_contract() const {
    return hitter_pingpong_command_contract_ ==
               "strike_followthrough_home_external_commit_v4" &&
           hitter_pingpong_planner_commit_contract_ ==
               "fresh_shot_external_event_policy_independent_v2" &&
           hitter_pingpong_wait_clock_contract_ ==
               "constant_pending_wait_v1" &&
           actor_obs_contract_ == "hitter_pure_110_headslots_vxy_v1" &&
           hitter_pingpong_trajectory_teacher_contract_ ==
               "contact_linear_actor_visible_v1" &&
           hitter_pingpong_motion_phase_contract_ ==
               "signed_tts_contact_frame_v1" &&
           hitter_pingpong_racket_velocity_contract_ ==
               "mocap_vxy_world_racket_velocity_v1";
  }
  bool uses_signed_tts_contact_frame_contract() const {
    return uses_continuous_rally_v4_contract() ||
           uses_build2_rapid_preempt_contract() ||
           (uses_build2_minimal_rootfix_contract() &&
            hitter_pingpong_motion_phase_contract_ ==
                "signed_tts_contact_frame_v1");
  }
  bool uses_hitter_pure_headslots_vxy_obs() const {
    return small_station_ || uses_continuous_rally_v4_contract() ||
           (training_recipe_ == "hitter_pingpong_build2_home_support_frame_v1" &&
            uses_build2_fixed_home_contract()) ||
           uses_schema27_optional_reach_obs();
  }
  bool uses_mocap_velocity_observation() const {
    return uses_position_mocap_obs() || uses_hitter_pure_headslots_vxy_obs();
  }
  bool uses_fresh_shot_external_commit_contract() const {
    return uses_continuous_rally_v2_contract() ||
           uses_continuous_rally_v3_contract() ||
           uses_continuous_rally_v4_contract() ||
           uses_build2_rapid_preempt_contract();
  }
  bool uses_visible_wait_clock_contract() const {
    return uses_continuous_rally_v2_contract() &&
           hitter_pingpong_wait_clock_contract_ == "visible_elapsed_v1";
  }
  bool uses_actor_entry_support_commit_contract() const {
    return (hitter_pingpong_command_contract_ == "late_reveal_raw_tts_v2" &&
            hitter_pingpong_planner_commit_contract_ ==
                "actor_entry_support_v1") ||
           (hitter_pingpong_command_contract_ == "home_preempt_raw_tts_v3" &&
            hitter_pingpong_planner_commit_contract_ ==
                "postcontact_home_preposition_actor_entry_v3") ||
           (hitter_pingpong_command_contract_ ==
                "strike_followthrough_home_external_commit_v1" &&
            hitter_pingpong_planner_commit_contract_ ==
                "fixed_contact_plus_0p40_policy_independent_v1") ||
           (hitter_pingpong_command_contract_ ==
                "strike_followthrough_home_external_commit_v2" &&
            hitter_pingpong_planner_commit_contract_ ==
                "fresh_shot_external_event_policy_independent_v2") ||
           (hitter_pingpong_command_contract_ ==
                "strike_followthrough_home_external_commit_v3" &&
            hitter_pingpong_planner_commit_contract_ ==
                "fresh_shot_external_event_policy_independent_v2") ||
           (hitter_pingpong_command_contract_ ==
                "strike_followthrough_home_external_commit_v4" &&
            hitter_pingpong_planner_commit_contract_ ==
                "fresh_shot_external_event_policy_independent_v2") ||
           uses_build2_rapid_preempt_contract();
  }
  bool uses_home_preempt_rally_contract() const {
    return (hitter_pingpong_command_contract_ == "home_preempt_raw_tts_v3" &&
            hitter_pingpong_home_rally_contract_ ==
                "immutable_motion_entry_home_preposition_preemptible_v2") ||
           (hitter_pingpong_command_contract_ ==
                "strike_followthrough_home_external_commit_v1" &&
            hitter_pingpong_home_rally_contract_ ==
                "immutable_home_return_preemptible_by_external_ball_v1") ||
           (hitter_pingpong_command_contract_ ==
                "strike_followthrough_home_external_commit_v2" &&
            hitter_pingpong_home_rally_contract_ ==
                "immutable_home_return_preemptible_by_external_ball_v1") ||
           (hitter_pingpong_command_contract_ ==
                "strike_followthrough_home_external_commit_v3" &&
            hitter_pingpong_home_rally_contract_ ==
                "immutable_home_return_preemptible_by_external_ball_v1") ||
           (hitter_pingpong_command_contract_ ==
                "strike_followthrough_home_external_commit_v4" &&
            hitter_pingpong_home_rally_contract_ ==
                "immutable_home_return_preemptible_by_external_ball_v1");
  }
  bool uses_native_tail_home_rally_contract() const {
    return (uses_build2_minimal_rootfix_contract() ||
            uses_build2_rapid_preempt_contract()) &&
           hitter_pingpong_home_rally_contract_ ==
               "native_tail_toward_home_no_settle_gate_v1";
  }
  double hitter_pingpong_native_tail_home_s() const {
    if (!uses_native_tail_home_rally_contract()) {
      throw std::runtime_error(
          "native-tail HOME timing requested for an incompatible HITTER contract");
    }
    // The Build2 schema21/schema22 post-contact HOME metadata names this contact+0.12 edge.
    // It is not the native clip-completion edge and does not itself admit the next flight.
    return 0.12;
  }

  double hitter_pingpong_preempt_no_preempt_s() const {
    if (!uses_build2_rapid_preempt_contract()) {
      throw std::runtime_error(
          "rapid-preempt timing requested for an incompatible HITTER contract");
    }
    return hitter_pingpong_preempt_no_preempt_s_;
  }
  const std::array<double, 2>& hitter_pingpong_preempt_entry_tts_support_s() const {
    if (!uses_build2_rapid_preempt_contract()) {
      throw std::runtime_error(
          "rapid-preempt TTS support requested for an incompatible HITTER contract");
    }
    return hitter_pingpong_preempt_entry_tts_support_s_;
  }
  const std::array<double, 2>&
  hitter_pingpong_preempt_commit_delay_support_s() const {
    if (!uses_build2_rapid_preempt_contract()) {
      throw std::runtime_error(
          "rapid-preempt commit-delay support requested for an incompatible "
          "HITTER contract");
    }
    return hitter_pingpong_preempt_commit_delay_support_s_;
  }
  const std::string&
  hitter_pingpong_preempt_commit_delay_clock_contract() const {
    if (!uses_build2_rapid_preempt_contract()) {
      throw std::runtime_error(
          "rapid-preempt commit-delay clock requested for an incompatible "
          "HITTER contract");
    }
    return hitter_pingpong_preempt_commit_delay_clock_contract_;
  }

  double hitter_pingpong_preposition_lead_s(int station_class) const {
    if (station_class < 0 || station_class >= 3)
      throw std::out_of_range("station class outside same/small/main");
    return hitter_pingpong_preposition_lead_s_[station_class];
  }
  double hitter_pingpong_post_contact_rearm_s() const {
    return hitter_pingpong_post_contact_rearm_s_;
  }
  const std::array<double, 2>& hitter_pingpong_actor_entry_tts_support(
      int clip) const {
    if (clip < 0 || clip >= 2)
      throw std::out_of_range(
          "HitterPingPong actor-entry support clip must be 0 or 1");
    return hitter_pingpong_actor_entry_tts_support_[clip];
  }
  double hitter_pingpong_wait_tts_s() const {
    return hitter_pingpong_wait_tts_s_;
  }
  double hitter_pingpong_entry_prefix_phase_cap() const {
    return hitter_pingpong_entry_prefix_phase_cap_;
  }
  double hitter_pingpong_policy_dt_s() const {
    return hitter_pingpong_policy_dt_s_;
  }
  const std::array<double, 3>& hitter_pingpong_pending_target() const {
    return hitter_pingpong_pending_target_;
  }
  const std::array<double, 3>& hitter_pingpong_pending_velocity() const {
    return hitter_pingpong_pending_velocity_;
  }
  bool is_v17_r1_stationary_replay() const {
    return v17_r1_stationary_replay_;
  }
  bool is_v17_r6_p0_contract_audit() const {
    return v17_r6_p0_contract_audit_;
  }
  bool is_v17_r10_p0_gate3() const { return v17_r10_p0_gate3_; }
  bool is_schema24_training_screen_gate3() const {
    return schema24_training_screen_gate3_;
  }
  bool is_v17_r12_v11_qdes_tuple_hardware() const {
    return v17_r12_v11_qdes_tuple_hardware_;
  }
  const std::string& deploy_manifest_status() const {
    return deploy_manifest_status_;
  }
  const std::string& deploy_manifest_sha256() const {
    return deploy_manifest_sha256_;
  }
  const std::string& deploy_contract_fingerprint_sha256() const {
    return deploy_contract_fingerprint_sha256_;
  }
  const std::string& qdes_parity_csv_sha256() const {
    return qdes_parity_csv_sha256_;
  }
  // Operational observation contract: empty metadata means legacy/raw feedback; "zero"
  // means passive head outputs must never recur through the last_action observation.
  bool last_action_head_is_zero() const { return last_action_head_mode_ == "zero"; }
  bool is_rally_station_recipe() const {
    return is_rally_v8_recipe() || is_rally_v9_recipe() || requires_component_velocity_gate();
  }
  const std::string& runtime_contract() const { return runtime_contract_; }
  const std::string& training_recipe() const { return training_recipe_; }
  const std::string& training_recipe_version() const {
    return training_recipe_version_;
  }
  bool uses_deploy_config() const { return !deploy_cfg_path_.empty(); }
  const std::string& deploy_cfg_path() const { return deploy_cfg_path_; }
  const std::string& deploy_policy_sha256() const {
    return deploy_policy_sha256_;
  }
  const std::array<double, 2>& hp_station_y_step_range() const {
    return hp_station_y_step_range_;
  }

  // Reference base->racket reach offset at a given time_step, computed from the baked refs:
  // blade world xy (ref pelvis pose + racket FK on the ref joints) minus ref pelvis xy. Same
  // arithmetic as training's _ensure_reference_strike_state — the fallback for 177 models
  // whose export predates the ref_reach_offset_xy metadata key.
  Vec2 reach_offset_from_refs(int time_step) {
    auto out = Run(Eigen::VectorXd::Zero(obs_dim_), time_step,
                   {"joint_pos", "body_pos_w", "body_quat_w"});
    const Eigen::VectorXd ref_q = Map(out[0], kNumJoints);
    const float* bp = out[1].GetTensorData<float>();  // [1,14,3]; tracked body 0 = pelvis
    const float* bq = out[2].GetTensorData<float>();  // [1,14,4]
    const Vec3 pelvis_pos(bp[0], bp[1], bp[2]);
    const Vec4 pelvis_quat(bq[0], bq[1], bq[2], bq[3]);
    const Vec3 blade_w = pelvis_pos + mat_from_quat(pelvis_quat) * racket_pos_pelvis(ref_q);
    return Vec2(blade_w[0] - pelvis_pos[0], blade_w[1] - pelvis_pos[1]);
  }
  const std::vector<std::string>& joint_names() const { return joint_names_; }
  const std::vector<std::string>& body_names() const { return body_names_; }
  const Eigen::VectorXd& default_q() const { return default_q_; }
  const Eigen::VectorXd& action_scale() const { return action_scale_; }
  const Eigen::VectorXd& kp() const { return kp_; }
  const Eigen::VectorXd& kd() const { return kd_; }
  const Eigen::VectorXd& qdes_safe_lo() const { return qdes_safe_lo_; }
  const Eigen::VectorXd& qdes_safe_hi() const { return qdes_safe_hi_; }
  const Eigen::VectorXd& qdes_hard_lo() const { return qdes_hard_lo_; }
  const Eigen::VectorXd& qdes_hard_hi() const { return qdes_hard_hi_; }
  double qdes_actual_q_hard_tolerance_rad() const {
    return qdes_actual_q_hard_tolerance_rad_;
  }
  bool qdes_actual_q_hard_audit_only() const {
    return qdes_actual_q_hard_audit_mode_ == "telemetry";
  }
  const std::string& qdes_actual_q_hard_audit_mode() const {
    return qdes_actual_q_hard_audit_mode_;
  }
  const Eigen::VectorXd& qdes_rate_limit() const { return qdes_rate_limit_; }
  const Eigen::VectorXd& qdes_tracking_error_limit() const {
    return qdes_tracking_error_limit_;
  }
  const Eigen::VectorXd& qdes_projector_kp() const { return qdes_projector_kp_; }
  const Eigen::VectorXd& qdes_projector_kd() const { return qdes_projector_kd_; }
  const Eigen::VectorXd& qdes_projector_effort_limit() const {
    return qdes_projector_effort_limit_;
  }
  double qdes_torque_headroom_fraction() const {
    return qdes_torque_headroom_fraction_;
  }
  double qdes_projector_dt_s() const { return qdes_projector_dt_s_; }
  bool has_actual_q_guard_contract() const {
    return qdes_actual_q_guard_contract_ ==
           "predictive_safe_boundary_brake_v1";
  }
  double qdes_actual_q_guard_horizon_s() const {
    return qdes_actual_q_guard_horizon_s_;
  }
  double base_velocity_ema_alpha() const { return base_velocity_ema_alpha_; }
  double base_localization_max_age_s() const { return base_localization_max_age_s_; }
  double base_localization_max_propagation_s() const {
    return base_localization_max_propagation_s_;
  }
  double gait_frequency_hz() const { return gait_frequency_hz_; }
  double gait_duty_factor() const { return gait_duty_factor_; }
  double gait_move_deadband() const { return gait_move_deadband_; }
  double gait_step_distance() const { return gait_step_distance_; }
  int gait_max_cycles() const { return gait_max_cycles_; }
  double gait_velocity_max() const { return gait_velocity_max_; }

  // Reference motion at time_step (obs-independent side-outputs).
  PpRefs refs(int time_step) {
    Eigen::VectorXd zero = Eigen::VectorXd::Zero(obs_dim_);
    auto out = Run(zero, time_step, {"joint_pos", "joint_vel", "body_pos_w", "body_quat_w"});
    PpRefs r;
    r.joint_pos = Map(out[0], kNumJoints);
    r.joint_vel = Map(out[1], kNumJoints);
    const float* bp = out[2].GetTensorData<float>();  // [1,14,3]
    const float* bq = out[3].GetTensorData<float>();  // [1,14,4]
    r.anchor_pos_w = Vec3(bp[kAnchorTrackedIdx * 3 + 0], bp[kAnchorTrackedIdx * 3 + 1],
                          bp[kAnchorTrackedIdx * 3 + 2]);
    r.anchor_quat_w = Vec4(bq[kAnchorTrackedIdx * 4 + 0], bq[kAnchorTrackedIdx * 4 + 1],
                           bq[kAnchorTrackedIdx * 4 + 2], bq[kAnchorTrackedIdx * 4 + 3]);
    // reference pelvis/root = tracked body index 0 (perfect-tracking mode only)
    r.ref_pelvis_pos_w = Vec3(bp[0], bp[1], bp[2]);
    return r;
  }

  // Deterministic mean action (31), Isaac order.
  Eigen::VectorXd mean_action(const Eigen::VectorXd& obs, int time_step) {
    auto out = Run(obs, time_step, {"actions"});
    return Map(out[0], kNumJoints);
  }

  // Schema28's second named output is not another actuator command. It is the
  // frozen core's 15 policy-owned action columns for the *next* observation's
  // append-only shadow feedback. The composed 31-D output remains the sole
  // q_des owner.
  std::pair<Eigen::VectorXd, Eigen::VectorXd>
  mean_action_and_core_shadow(const Eigen::VectorXd& obs, int time_step) {
    if (!uses_schema28_core_shadow_obs() || !actions_output_valid_ ||
        !core_action_shadow_output_valid_) {
      throw std::runtime_error(
          "dual action/core-shadow inference requires the exact Schema28 "
          "named output contract");
    }
    auto out = Run(
        obs, time_step, {"actions", "core_action_shadow_owned"});
    return {Map(out[0], kNumJoints),
            Map(out[1], kCoreActionShadowDim)};
  }

  // Decode the network output into the nominal target. Legacy V3 needs measured state and the
  // previous executable q_des, so its incremental decoder runs in ProjectBoundedQdes_. V5
  // decodes one ABSOLUTE posture request anchored at the default posture with an independent
  // tanh room per side (q = default + room * tanh(scale * a / room), room = the side's
  // default-to-safe-rail distance); a single full-span curve fitted at a=0 explodes for
  // near-rail defaults (A3 shoulder_roll/knee — the retired v4's twisted-swing-arm root
  // cause). V2/V1 keep the legacy full-span affine tanh. The downstream
  // safe/rate/tracking/PD projector is unchanged for every contract.
  Eigen::VectorXd target_q(const Eigen::VectorXd& action) const {
    if (has_feasible_qdes_contract()) return action;
    if (has_absolute_feasible_qdes_contract()) {
      Eigen::VectorXd q(kNumJoints);
      for (int i = 0; i < kNumJoints; ++i) {
        const double room_pos = std::max(qdes_safe_hi_[i] - qdes_default_q_[i], 1.0e-6);
        const double room_neg = std::max(qdes_default_q_[i] - qdes_safe_lo_[i], 1.0e-6);
        const double room = action[i] >= 0.0 ? room_pos : room_neg;
        q[i] = qdes_default_q_[i] +
               room * std::tanh(qdes_action_scale_[i] * action[i] / room);
      }
      return q;
    }
    if (has_bounded_qdes_contract()) {
      const Eigen::VectorXd mid = 0.5 * (qdes_safe_lo_ + qdes_safe_hi_);
      const Eigen::VectorXd half = 0.5 * (qdes_safe_hi_ - qdes_safe_lo_);
      const Eigen::VectorXd tanh_input =
          qdes_tanh_input_gain_.cwiseProduct(action) + qdes_tanh_bias_;
      return mid + half.cwiseProduct(tanh_input.array().tanh().matrix());
    }
    Eigen::VectorXd q = default_q_ + action.cwiseProduct(action_scale_);
    if (has_v11_affine_safe_qdes_contract() ||
        has_v12_affine_safe_slew_qdes_contract()) {
      // v12 returns the pre-slew nominal here; PpPolicy applies the stateful slew.
      for (int index = 0; index < kNumJoints; ++index) {
        q[index] = ComputeV11AffineSafeQdes(
            action[index], default_q_[index], action_scale_[index],
            qdes_safe_lo_[index], qdes_safe_hi_[index]);
      }
    }
    return q;
  }

  Eigen::VectorXd qdes_feedback(const Eigen::VectorXd& executed_qdes) const {
    if (!has_bounded_qdes_contract())
      throw std::runtime_error("qdes_feedback requires a bounded_qdes contract");
    Eigen::VectorXd feedback =
        (executed_qdes - default_q_).cwiseQuotient(qdes_feedback_scale_);
    return feedback.array().max(-1.0).min(1.0).matrix();
  }

  // Training-parity engage seed for the executed-q_des feedback obs: before the projector's
  // first tick, training reports the MEASURED posture in the same normalized coordinates —
  // (clamp(q, safe_lo, safe_hi) - default_q) / feedback_scale, clamped to [-1, 1] — instead
  // of a zero action history (hope_actions.BoundedJointPositionAction.executed_qdes_feedback).
  Eigen::VectorXd measured_qdes_feedback(const Eigen::VectorXd& measured_q) const {
    return qdes_feedback(measured_q.cwiseMax(qdes_safe_lo_).cwiseMin(qdes_safe_hi_));
  }

 private:
  void LoadSmallStationContract_(Ort::ModelMetadata& md, Ort::AllocatorWithDefaultOptions& alloc) {
    auto require = [&](const char* key, const char* expected) {
      if (LookupMeta(md, alloc, key) != expected)
        throw std::runtime_error(std::string("SmallStation metadata mismatch: ") + key);
    };
    const bool compact = actor_obs_contract_ == "hitter_compact_execution_324_v1";
    require("actor_obs_contract", compact ? "hitter_compact_execution_324_v1" : "hitter_pure_112_headslots_vxy_reach_v1");
    require("actor_obs_total_dim", compact ? "324" : "112");
    // Reward, initialization, build label and training revision are provenance;
    // the executable observation/action/clock contracts below select behavior.
    require("hitter_pure_runtime_contract", "rally_final_v2");
    require("hitter_pingpong_command_contract", "small_station_external_schedule_precommit_settle_v1");
    require("hitter_pingpong_planner_commit_contract", "fresh_schema2_absolute_small_station_v1");
    require("qdes_action_contract", "v12_affine_safe_slew_qdes_v1");
    if (compact) {
      require("qdes_delivery_contract", "post_clamp_latest_eligible_target_v1");
      require("compact_history_contract", "current114_past3x70_policy_tick_repeat_reset_v1");
      require("compact_sent_feedback_contract", "last_successfully_sent_absolute_action_order_rad_v1");
    }
    require("hitter_pingpong_optional_reach_training_enabled", "false");
    require("hitter_pingpong_wait_clock_contract", "constant_pending_wait_v1");
    require("hitter_pingpong_motion_phase_contract", "signed_tts_contact_frame_v1");
    require("base_pose_source", "mocap_authoritative");
    if (obs_dim_ != (compact ? 324 : 112) || !actions_output_valid_ || core_action_shadow_output_present_)
      throw std::runtime_error("SmallStation requires its declared observation dimension and actions[1,31]");
    const auto support = ToVec(LookupMeta(md, alloc, "hitter_pingpong_actor_entry_tts_support_per_clip_s"));
    const auto delays = ToVec(LookupMeta(md, alloc, compact ? "compact_commit_delay_support_s" : "small_station_commit_delay_support_s"));
    const auto pending_velocity = ToVec(LookupMeta(md, alloc, compact ? "compact_pending_velocity" : "small_station_pending_velocity"));
    if (support.size() != 4 || !support.allFinite() || support[0] <= 0 || support[1] < support[0] ||
        support[2] <= 0 || support[3] < support[2] || delays.size() != 2 || !delays.allFinite() ||
        delays[0] < 0 || delays[1] < delays[0] || pending_velocity.size() != 3 || !pending_velocity.allFinite())
      throw std::runtime_error("SmallStation invalid clock/WAIT metadata");
    hitter_pingpong_actor_entry_tts_support_ = {{{support[0], support[1]}, {support[2], support[3]}}};
    hitter_pingpong_preempt_entry_tts_support_s_ = {std::min(support[0], support[2]), std::max(support[1], support[3])};
    hitter_pingpong_preempt_commit_delay_support_s_ = {delays[0], delays[1]};
    hitter_pingpong_preempt_commit_delay_clock_contract_ = "previous_contact_to_next_commit_v1";
    hitter_pingpong_preempt_no_preempt_s_ = 0.0;
    hitter_pingpong_pending_velocity_ = {pending_velocity[0], pending_velocity[1], pending_velocity[2]};
    hitter_pingpong_motion_phase_contract_ = "signed_tts_contact_frame_v1";
    compact_execution_ = compact;
    small_station_ = true;
  }
  bool compact_execution_ = false;
  bool small_station_ = false;
  std::vector<Ort::Value> Run(const Eigen::VectorXd& obs, int time_step,
                              const std::vector<const char*>& out_names) {
    obs_f_.resize(obs_dim_);
    for (int i = 0; i < obs_dim_; ++i) obs_f_[i] = static_cast<float>(obs[i]);
    float ts = static_cast<float>(time_step);
    std::array<int64_t, 2> obs_shape{1, obs_dim_};
    std::array<int64_t, 2> ts_shape{1, 1};
    std::array<Ort::Value, 2> ins{
        Ort::Value::CreateTensor<float>(mem_, obs_f_.data(), obs_dim_, obs_shape.data(), 2),
        Ort::Value::CreateTensor<float>(mem_, &ts, 1, ts_shape.data(), 2)};
    static const char* in_names[2] = {"obs", "time_step"};
    return session_->Run(Ort::RunOptions{nullptr}, in_names, ins.data(), 2,
                         out_names.data(), out_names.size());
  }

  static Eigen::VectorXd Map(const Ort::Value& v, int n) {
    const float* p = v.GetTensorData<float>();
    Eigen::VectorXd out(n);
    for (int i = 0; i < n; ++i) out[i] = p[i];
    return out;
  }
  void BindDeployConfig_(const std::string& model_path,
                         const std::string& deploy_cfg_path) {
    const PpDeployConfig deploy =
        PpDeployConfig::Load(deploy_cfg_path, model_path);
    if (deploy.task_plugin != "a3_pingpong_hitter_pure") {
      throw std::runtime_error(
          "deploy.yaml task_plugin is not a3_pingpong_hitter_pure");
    }
    if (deploy.observation_dim != obs_dim_ ||
        deploy.action_dim != kNumJoints) {
      throw std::runtime_error(
          "deploy.yaml observation/action dimensions do not match the ONNX");
    }
    if (deploy.joint_names != joint_names_) {
      throw std::runtime_error(
          "deploy.yaml JointPositionAction joint order does not match ONNX joint_names");
    }
    auto require_vector = [](const Eigen::VectorXd& deploy_values,
                             const Eigen::VectorXd& metadata_values,
                             const char* label) {
      if (deploy_values.size() != metadata_values.size() ||
          (deploy_values - metadata_values).cwiseAbs().maxCoeff() > 1.0e-6) {
        throw std::runtime_error(
            std::string("deploy.yaml/ONNX mismatch: ") + label);
      }
    };
    require_vector(deploy.default_q, default_q_, "default_joint_pos");
    require_vector(deploy.action_scale, action_scale_, "action scale");
    require_vector(deploy.kp, kp_, "stiffness");
    require_vector(deploy.kd, kd_, "damping");
    require_vector(deploy.safe_lower, qdes_safe_lo_, "q_des safe lower");
    require_vector(deploy.safe_upper, qdes_safe_hi_, "q_des safe upper");
    require_vector(deploy.hard_lower, qdes_hard_lo_, "q_des hard lower");
    require_vector(deploy.hard_upper, qdes_hard_hi_, "q_des hard upper");
    validate_deploy_training_recipe_pair(
        training_recipe_, training_recipe_version_, deploy.training_recipe,
        deploy.training_recipe_version);
    if (deploy.action_contract != qdes_action_contract_ ||
        deploy.actor_observation_contract != actor_obs_contract_ ||
        deploy.runtime_contract != runtime_contract_ ||
        std::fabs(deploy.actual_q_hard_tolerance_rad -
                  qdes_actual_q_hard_tolerance_rad_) > 1.0e-12) {
      throw std::runtime_error(
          "deploy.yaml policy contracts do not match ONNX metadata");
    }

    // deploy.yaml is authoritative at runtime.  Equality checks above ensure
    // that moving these values out of ONNX metadata cannot silently change the
    // frozen policy semantics.
    default_q_ = deploy.default_q;
    action_scale_ = deploy.action_scale;
    kp_ = deploy.kp;
    kd_ = deploy.kd;
    qdes_safe_lo_ = deploy.safe_lower;
    qdes_safe_hi_ = deploy.safe_upper;
    qdes_hard_lo_ = deploy.hard_lower;
    qdes_hard_hi_ = deploy.hard_upper;
    qdes_actual_q_hard_tolerance_rad_ =
        deploy.actual_q_hard_tolerance_rad;
    deploy_cfg_path_ = deploy_cfg_path;
    deploy_policy_sha256_ = deploy.policy_sha256;
  }

  static std::string LookupMeta(Ort::ModelMetadata& md, Ort::AllocatorWithDefaultOptions& a,
                                const char* key) {
    auto s = md.LookupCustomMetadataMapAllocated(key, a);
    if (!s) throw std::runtime_error(std::string("ONNX missing metadata: ") + key);
    return std::string(s.get());
  }
  static std::string LookupMetaOptional(Ort::ModelMetadata& md,
                                        Ort::AllocatorWithDefaultOptions& a, const char* key) {
    auto s = md.LookupCustomMetadataMapAllocated(key, a);
    return s ? std::string(s.get()) : std::string{};
  }
  static std::vector<std::string> SplitCsv(const std::string& s) {
    std::vector<std::string> out; std::stringstream ss(s); std::string tok;
    while (std::getline(ss, tok, ',')) out.push_back(tok);
    return out;
  }
  static Eigen::VectorXd ToVec(const std::string& s) {
    auto t = SplitCsv(s); Eigen::VectorXd v(t.size());
    for (size_t i = 0; i < t.size(); ++i) v[i] = std::stod(t[i]);
    return v;
  }
  static std::vector<double> ToStdVector(const Eigen::VectorXd& v) {
    return std::vector<double>(v.data(), v.data() + v.size());
  }
  // Strict scalar parse: the whole string must be one number (std::stod alone
  // accepts trailing garbage).  "nan"/"inf" parse but fail the finite checks
  // downstream.
  static double ParseMetaDouble(const std::string& s, const char* key) {
    std::size_t consumed = 0;
    double value = 0.0;
    try {
      value = std::stod(s, &consumed);
    } catch (const std::exception&) {
      throw std::runtime_error(
          std::string("ONNX metadata ") + key + " is not a number: '" + s + "'");
    }
    while (consumed < s.size() &&
           (s[consumed] == ' ' || s[consumed] == '\t' || s[consumed] == '\n' ||
            s[consumed] == '\r'))
      ++consumed;
    if (consumed != s.size())
      throw std::runtime_error(
          std::string("ONNX metadata ") + key + " has trailing characters: '" + s + "'");
    return value;
  }

  Ort::Env env_;
  Ort::MemoryInfo mem_;
  std::unique_ptr<Ort::Session> session_;
  PpOnnxLoadProfile load_profile_ = PpOnnxLoadProfile::kProductionStrict;
  bool v17_r1_stationary_replay_ = false;
  bool v17_r6_p0_contract_audit_ = false;
  bool v17_r10_p0_gate3_ = false;
  bool schema24_training_screen_gate3_ = false;
  bool v17_r12_v11_qdes_tuple_hardware_ = false;
  bool actions_output_valid_ = false;
  bool core_action_shadow_output_present_ = false;
  bool core_action_shadow_output_valid_ = false;
  std::vector<std::string> joint_names_, body_names_;
  std::vector<double> clip_seg_lengths_, clip_strike_phases_;  // empty on legacy exports
  std::vector<Vec2> reach_offsets_;  // per-clip ref base->racket reach; empty on old exports
  std::vector<std::array<double, 6>> hp_pos_boxes_, hp_vel_boxes_;  // hitter_pure geometry
  std::vector<std::array<double, 6>> hp_vel_core_boxes_, hp_vel_planner_boxes_;
  double hp_vel_planner_mix_prob_ = 0.0;
  int hp_vel_range_ramp_steps_ = 0;
  std::array<double, 4> hp_base_range_ = {0.0, 0.0, 0.0, 0.0};
  bool has_hp_base_range_ = false;
  std::array<double, 2> hp_station_y_step_range_ = {0.0, 0.0};
  bool has_hp_station_y_step_range_ = false;
  double ball_clock_followthrough_s_ = 0.0;
  std::string runtime_contract_;
  std::string training_recipe_;
  std::string training_recipe_version_;
  std::string deploy_cfg_path_;
  std::string deploy_policy_sha256_;
  std::string actor_obs_contract_;
  std::string hitter_pingpong_command_contract_;
  std::string hitter_pingpong_planner_commit_contract_;
  std::string hitter_pingpong_common_ready_contract_;
  std::string hitter_pingpong_home_rally_contract_;
  std::string hitter_pingpong_wait_clock_contract_;
  std::string hitter_pingpong_arrival_contract_;
  std::string hitter_pingpong_trajectory_teacher_contract_;
  std::string hitter_pingpong_motion_phase_contract_;
  std::string hitter_pingpong_racket_velocity_contract_;
  std::string hitter_pingpong_transition_contract_;
  HitterOptionalReachRuntimeMode optional_reach_runtime_mode_ =
      HitterOptionalReachRuntimeMode::kDisabledSchema2Zero;
  std::string hitter_pingpong_preempt_commit_delay_clock_contract_;
  double hitter_pingpong_post_contact_rearm_s_ = 0.0;
  double hitter_pingpong_preempt_no_preempt_s_ = 0.0;
  double hitter_pingpong_wait_tts_s_ = 0.0;
  double hitter_pingpong_entry_prefix_phase_cap_ = 0.0;
  double hitter_pingpong_policy_dt_s_ = 0.0;
  double hitter_pingpong_arrival_hazard_per_tick_ = 0.0;
  std::array<double, 3> hitter_pingpong_pending_target_ = {0.0, 0.0, 0.0};
  std::array<double, 3> hitter_pingpong_pending_velocity_ = {0.0, 0.0, 0.0};
  std::array<double, 3> hitter_pingpong_preposition_lead_s_ = {0.0, 0.0, 0.0};
  std::array<std::array<double, 2>, 2>
      hitter_pingpong_actor_entry_tts_support_ = {
          std::array<double, 2>{0.0, 0.0},
          std::array<double, 2>{0.0, 0.0}};
  std::array<double, 2> hitter_pingpong_preempt_entry_tts_support_s_ = {
      0.0, 0.0};
  std::array<double, 2> hitter_pingpong_preempt_commit_delay_support_s_ = {
      0.0, 0.0};
  std::string base_localization_contract_;
  std::string base_pose_source_;
  std::string base_pose_schema_;
  std::string orientation_contract_;
  std::string angular_velocity_contract_;
  std::string yaw_align_contract_;
  std::string world_frame_contract_;
  std::string calibration_contract_;
  std::string locomotion_contract_;
  std::string intervention_contract_;
  std::string intervention_deploy_value_;
  std::string last_action_head_mode_;
  std::string qdes_action_contract_;
  std::string qdes_actual_q_guard_contract_;
  std::string qdes_actual_q_hard_audit_mode_;
  std::string deploy_manifest_status_;
  std::string deploy_manifest_sha256_;
  std::string deploy_contract_fingerprint_sha256_;
  std::string qdes_parity_csv_sha256_;
  std::array<double, 6> station_y_mixture_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  bool station_side_explicit_ = false;
  Eigen::VectorXd default_q_, action_scale_, kp_, kd_;
  Eigen::VectorXd qdes_safe_lo_, qdes_safe_hi_, qdes_hard_lo_, qdes_hard_hi_;
  // v12_affine_safe_slew_qdes_v1 only (empty / zero otherwise)
  Eigen::VectorXd qdes_slew_limit_rad_per_tick_, qdes_slew_leg_mask_;
  V12SlewScaleParams qdes_slew_scale_params_{0.0, 0.0, 0.0, 0.0, 0.0};
  std::string qdes_v12_last_action_feedback_;
  Eigen::VectorXd qdes_tanh_bias_, qdes_tanh_input_gain_;
  Eigen::VectorXd qdes_default_q_, qdes_action_scale_;  // v5 decode (empty otherwise)
  Eigen::VectorXd qdes_rate_limit_, qdes_tracking_error_limit_;
  Eigen::VectorXd qdes_projector_kp_, qdes_projector_kd_;
  Eigen::VectorXd qdes_projector_effort_limit_, qdes_feedback_scale_;
  double qdes_torque_headroom_fraction_ = 0.0;
  double qdes_projector_dt_s_ = 0.0;
  double qdes_actual_q_guard_horizon_s_ = 0.0;
  double qdes_actual_q_hard_tolerance_rad_ = 0.0;
  double base_velocity_ema_alpha_ = 0.0;
  double base_localization_max_age_s_ = 0.0;
  double base_localization_max_propagation_s_ = 0.0;
  double gait_frequency_hz_ = 0.0;
  double gait_duty_factor_ = 0.0;
  double gait_move_deadband_ = 0.0;
  double gait_step_distance_ = 0.0;
  int gait_max_cycles_ = 0;
  double gait_velocity_max_ = 0.0;
  std::vector<float> obs_f_;
  int obs_dim_ = kObsDim;  // detected from the model input at load (180 full / 175 deploy_parity)
};

}  // namespace a3_pingpong
