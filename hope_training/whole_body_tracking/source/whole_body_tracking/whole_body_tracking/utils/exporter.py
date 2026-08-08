# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import os
import json
import hashlib
from typing import Any, Mapping
import torch

import onnx

from isaaclab.envs import ManagerBasedRLEnv
from isaaclab_rl.rsl_rl.exporter import _OnnxPolicyExporter

from whole_body_tracking.tasks.tracking.mdp import MotionCommand
from whole_body_tracking.tasks.tracking.actor_observation_contract import (
    infer_actor_observation_contract,
    removed_terms_vs_full,
)
from whole_body_tracking.utils.a3_deploy_manifest import (
    A3_DEPLOYMENT_STATUS,
    A3_QUALIFICATION_STATUS,
    attach_v17_r6_p0_manifest_metadata,
    attach_v17_r10_p0_manifest_metadata,
    build_v11_qdes_parity_csv,
    canonical_json,
    metadata_wire_value,
    qdes_parity_csv_sha256,
    write_deploy_manifest_sidecar,
    write_qdes_parity_sidecar,
)


def export_motion_policy_as_onnx(
    env: ManagerBasedRLEnv,
    actor_critic: object,
    path: str,
    normalizer: object | None = None,
    filename="policy.onnx",
    verbose=False,
):
    if not os.path.exists(path):
        os.makedirs(path, exist_ok=True)
    policy_exporter = _OnnxMotionPolicyExporter(env, actor_critic, normalizer, verbose)
    policy_exporter.export(path, filename)


class _OnnxMotionPolicyExporter(_OnnxPolicyExporter):
    def __init__(self, env: ManagerBasedRLEnv, actor_critic, normalizer=None, verbose=False):
        super().__init__(actor_critic, normalizer, verbose)
        cmd: MotionCommand = env.command_manager.get_term("motion")

        self.joint_pos = cmd.motion.joint_pos.to("cpu")
        self.joint_vel = cmd.motion.joint_vel.to("cpu")
        self.body_pos_w = cmd.motion.body_pos_w.to("cpu")
        self.body_quat_w = cmd.motion.body_quat_w.to("cpu")
        self.body_lin_vel_w = cmd.motion.body_lin_vel_w.to("cpu")
        self.body_ang_vel_w = cmd.motion.body_ang_vel_w.to("cpu")
        self.time_step_total = self.joint_pos.shape[0]

    def forward(self, x, time_step):
        time_step_clamped = torch.clamp(time_step.long().squeeze(-1), max=self.time_step_total - 1)
        return (
            self.actor(self.normalizer(x)),
            self.joint_pos[time_step_clamped],
            self.joint_vel[time_step_clamped],
            self.body_pos_w[time_step_clamped],
            self.body_quat_w[time_step_clamped],
            self.body_lin_vel_w[time_step_clamped],
            self.body_ang_vel_w[time_step_clamped],
        )

    def export(self, path, filename):
        self.to("cpu")
        obs = torch.zeros(1, self.actor[0].in_features)
        time_step = torch.zeros(1, 1)
        torch.onnx.export(
            self,
            (obs, time_step),
            os.path.join(path, filename),
            export_params=True,
            opset_version=11,
            verbose=self.verbose,
            input_names=["obs", "time_step"],
            output_names=[
                "actions",
                "joint_pos",
                "joint_vel",
                "body_pos_w",
                "body_quat_w",
                "body_lin_vel_w",
                "body_ang_vel_w",
            ],
            dynamic_axes={},
        )

def _sha256_file(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def attach_onnx_metadata(
    env: ManagerBasedRLEnv,
    run_path: str,
    path: str,
    filename="policy.onnx",
    checkpoint_path: str | None = None,
    task_identity: Mapping[str, Any] | None = None,
) -> None:
    onnx_path = os.path.join(path, filename)
    deploy_manifest = None
    deploy_manifest_sha256 = None
    qdes_parity_content = None

    observation_names = env.observation_manager.active_terms["policy"]
    observation_history_lengths: list[int] = []

    if env.observation_manager.cfg.policy.history_length is not None:
        observation_history_lengths = [env.observation_manager.cfg.policy.history_length] * len(observation_names)
    else:
        for name in observation_names:
            term_cfg = env.observation_manager.cfg.policy.to_dict()[name]
            history_length = term_cfg["history_length"]
            observation_history_lengths.append(1 if history_length == 0 else history_length)

    metadata = {
        "run_path": run_path,
        "joint_names": env.scene["robot"].data.joint_names,
        # NOMINAL PD gains — must come from default_joint_stiffness/damping, NOT joint_stiffness.
        # data.joint_stiffness holds the LIVE PhysX drive gains: explicit actuators
        # (IdealPDActuatorCfg) null them (their PD is applied by the actuator model, not the PhysX
        # drive) and randomize_actuator_gains DR rewrites them per env at reset. Exporting
        # data.joint_stiffness[0] therefore baked kp=kd=0 for every explicit joint (limp
        # arms/waist/ankles on deploy) and a random env-0 DR draw for the implicit ones.
        # data.default_joint_stiffness is written once from the actuator configs at init
        # ("for implicit and explicit actuators" — articulation.py) and DR only reads it.
        "joint_stiffness": env.scene["robot"].data.default_joint_stiffness[0].cpu().tolist(),
        "joint_damping": env.scene["robot"].data.default_joint_damping[0].cpu().tolist(),
        "default_joint_pos": env.scene["robot"].data.default_joint_pos_nominal.cpu().tolist(),
        "command_names": env.command_manager.active_terms,
        "observation_names": observation_names,
        "observation_history_lengths": observation_history_lengths,
        "action_scale": env.action_manager.get_term("joint_pos")._scale[0].cpu().tolist(),
        "anchor_body_name": env.command_manager.get_term("motion").cfg.anchor_body_name,
        "body_names": env.command_manager.get_term("motion").cfg.body_names,
    }
    if checkpoint_path is not None:
        checkpoint_path = os.path.abspath(os.path.expanduser(checkpoint_path))
        if not os.path.isfile(checkpoint_path):
            raise FileNotFoundError(
                f"attach_onnx_metadata checkpoint does not exist: {checkpoint_path}"
            )
        metadata["hitter_pure_checkpoint_sha256"] = _sha256_file(checkpoint_path)

    # Stateful action contracts export their fully resolved action-order arrays.  The selected task
    # YAML remains the sole numerical source; the C++ runner reads these values instead of carrying
    # another per-version table.  Use enough precision for the Python/C++ projector parity test.
    _action_term = env.action_manager.get_term("joint_pos")
    _resolved_action_metadata = getattr(_action_term, "resolved_contract_metadata", None)
    if callable(_resolved_action_metadata):
        for _key, _value in _resolved_action_metadata().items():
            if isinstance(_value, list):
                if _value and isinstance(_value[0], str):
                    metadata[_key] = ",".join(str(v) for v in _value)
                else:
                    metadata[_key] = ",".join(f"{float(v):.9g}" for v in _value)
            else:
                metadata[_key] = _value

    # DEPLOY kd CONTRACT GUARD (2026-07-09 plant-alignment fix): the A3 TRAINING actuator
    # damping now includes the plant's passive viscous damping (robots/agibot_a3.py — Isaac was
    # under-damped vs the AGI-MuJoCo/hardware plant, the measured cause of the ×2.5 follow-through
    # amplification). The runner must SEND only the original message kd — the deploy plant adds
    # its own passive damping physically; baking the raised training kd would DOUBLE-damp on
    # deploy. Resolve the wire kd per joint from the A3 contract table. HARDENED 2026-07-09 audit:
    # no try/except and no silent fallback — a3_deploy_joint_kd returns None only for a genuinely
    # non-A3 robot (positive head-joint identity; G1 names would otherwise pattern-match) and
    # RAISES on an A3 with unmatched joint names; and the contract invariant
    #   training_damping == deploy_kd + passive   (per joint)
    # is asserted right here, so a retune of any one of the three tables fails the next export.
    from whole_body_tracking.robots.agibot_a3 import a3_deploy_joint_kd, a3_passive_joint_damping

    _deploy_kd = a3_deploy_joint_kd(list(metadata["joint_names"]))
    if _deploy_kd is None:
        print(
            "[exporter] A3 deploy-kd contract NOT applied (robot not identified as the A3); baking "
            "default_joint_damping as-is.",
            flush=True,
        )
    if _deploy_kd is not None:
        _train_kd = metadata["joint_damping"]
        _passive = a3_passive_joint_damping(list(metadata["joint_names"]))
        _bad = [
            f"{n}: train={t:.4g} != wire={d:.4g} + passive={p:.4g}"
            for n, t, d, p in zip(metadata["joint_names"], _train_kd, _deploy_kd, _passive)
            if abs(t - (d + p)) > 1e-4
        ]
        if _bad:
            raise ValueError(
                "attach_onnx_metadata: A3 deploy-kd contract violated (training_damping != "
                "deploy_kd + passive):\n  " + "\n  ".join(_bad) + "\nThe three tables in "
                "robots/agibot_a3.py (A3_CFG actuator damping / A3_DEPLOY_JOINT_KD_PATTERNS / "
                "A3_PASSIVE_JOINT_DAMPING_PATTERNS) must be retuned TOGETHER."
            )
        metadata["a3_training_joint_damping"] = _train_kd
        metadata["a3_passive_joint_damping"] = _passive
        metadata["a3_joint_effort_limit"] = (
            env.scene["robot"].data.joint_effort_limits[0].cpu().tolist()
        )
        metadata["joint_damping"] = _deploy_kd
        print(
            "[exporter] A3 deploy-kd contract applied + invariant train==wire+passive verified "
            "per joint: baked message kd (wire values), NOT the passive-inclusive training "
            "damping. e.g. knee train "
            f"{_train_kd[metadata['joint_names'].index('left_knee_joint')]:.1f} -> wire "
            f"{_deploy_kd[metadata['joint_names'].index('left_knee_joint')]:.1f}",
            flush=True,
        )

    # kp=kd=0 bug that felled the 2026-07-02 explicitpd_ft bring-up). Refuse to bake it.
    _kp = metadata["joint_stiffness"]
    _kd = metadata["joint_damping"]
    if min(_kp) <= 0.0 or min(_kd) <= 0.0:
        bad = [
            f"{n}(kp={p:.3g},kd={d:.3g})"
            for n, p, d in zip(metadata["joint_names"], _kp, _kd)
            if p <= 0.0 or d <= 0.0
        ]
        raise ValueError(
            "attach_onnx_metadata: non-positive nominal PD gain(s) — deploy would command zero "
            "torque on: " + ", ".join(bad)
        )

    # Per-clip layout for the deploy reference clock (pp_reference_clock.hpp). The C++ runner
    # falls back to a hardcoded legacy layout when these keys are absent, which drove the baked
    # v2 clips at v1 frame indices (strike served ~0.6 s early; "backhand" spliced across the
    # clip boundary). seg_len comes from the baked MotionLoader segments; strike phases from the
    # racket_target command (per-clip when configured).
    motion_cmd = env.command_manager.get_term("motion")
    metadata["clip_seg_lengths"] = ",".join(str(int(n)) for n in motion_cmd.motion.seg_len.cpu().tolist())
    _task_identity = dict(task_identity or {})
    _hitter_pingpong_build = (
        str(_task_identity.get("name", "")) == "HitterPingPong"
        and str(_task_identity.get("gym_task", ""))
        == "HOPE-HitterPingPong-AgibotA3-v0"
    )
    try:
        rt_cfg = env.command_manager.get_term("racket_target").cfg
        if bool(getattr(rt_cfg, "base_mocap_enabled", False)):
            full_pose = bool(
                getattr(rt_cfg, "base_mocap_orientation_enabled", False)
            )
            metadata["base_localization_contract"] = (
                "calibrated_pose_receipt_v2" if full_pose
                else "position_receipt_v1"
            )
            metadata["base_mocap_delay_steps"] = str(
                int(getattr(rt_cfg, "base_mocap_delay_steps"))
            )
            metadata["base_mocap_update_interval_steps"] = str(
                int(getattr(rt_cfg, "base_mocap_update_interval_steps"))
            )
            metadata["base_mocap_position_noise_std"] = ",".join(
                f"{float(v):.9g}"
                for v in getattr(rt_cfg, "base_mocap_position_noise_std")
            )
            metadata["base_mocap_dropout_prob"] = (
                f"{float(rt_cfg.base_mocap_dropout_prob):.9g}"
            )
            metadata["base_mocap_velocity_ema_alpha"] = (
                f"{float(rt_cfg.base_mocap_velocity_ema_alpha):.9g}"
            )
            metadata["base_mocap_max_age_s"] = (
                f"{float(rt_cfg.base_mocap_max_age_s):.9g}"
            )
            if _hitter_pingpong_build and not full_pose:
                raise RuntimeError(
                    "Refusing HitterPingPong export without full-pose mocap metadata"
                )
            if full_pose:
                metadata["base_pose_source"] = "mocap_authoritative"
                metadata["base_pose_schema"] = "2"
                metadata["orientation_contract"] = "calibrated_world_base_v1"
                metadata["angular_velocity_contract"] = (
                    "mocap_anchored_pelvis_gyro_v1"
                )
                metadata["yaw_align_contract"] = (
                    "disabled_for_mocap_authoritative"
                )
                metadata["world_frame_contract"] = "table_p1_to_p2_v1"
                metadata["calibration_contract"] = "runtime_receipt_required"
                metadata["base_mocap_orientation_noise_std_rad"] = ",".join(
                    f"{float(v):.9g}"
                    for v in getattr(
                        rt_cfg, "base_mocap_orientation_noise_std_rad"
                    )
                )
                metadata[
                    "base_mocap_extrinsic_residual_rpy_std_rad"
                ] = ",".join(
                    f"{float(v):.9g}"
                    for v in getattr(
                        rt_cfg, "base_mocap_extrinsic_residual_rpy_std_rad"
                    )
                )
                metadata["base_mocap_max_propagation_s"] = (
                    f"{float(rt_cfg.base_mocap_max_propagation_s):.9g}"
                )
            if _hitter_pingpong_build:
                build_contract = _task_identity.get("build_contract") or {}
                friction_contract = _task_identity.get("friction_contract") or {}
                expected_forbidden = [
                    "runtime_handoff",
                    "qdes_projector",
                    "qdes_rate_limiter",
                    "executed_qdes_feedback",
                    "ready_recovery_curriculum",
                ]
                material = env.cfg.events.physics_material.params
                pd_event = env.cfg.events.randomize_pd_gains
                pd_params = pd_event.params
                action_cfg = env.cfg.actions.joint_pos
                motion_cfg = env.cfg.commands.motion
                build_ok = (
                    int(build_contract.get("actor_observation_dim", -1)) == 110
                    and int(build_contract.get("action_dim", -1)) == 31
                    and list(build_contract.get("forbidden_features", ()))
                    == expected_forbidden
                    and bool(getattr(rt_cfg, "venue_tuple_enabled", False))
                    and abs(
                        float(getattr(rt_cfg, "venue_tuple_final_mix_prob", -1.0))
                        - 0.25
                    )
                    <= 1.0e-12
                    and str(getattr(rt_cfg, "venue_tuple_mix_mode", ""))
                    == "fixed_balanced_bank_v1"
                    and bool(
                        getattr(rt_cfg, "venue_tuple_unconditional_outcomes", False)
                    )
                    and bool(getattr(rt_cfg, "virtual_ball", False))
                    and not bool(getattr(rt_cfg, "physical_ball", True))
                    and not bool(getattr(rt_cfg, "physical_ball_impulse", True))
                    and bool(material.get("make_consistent", False))
                    and bool(friction_contract.get("make_consistent", False))
                    and str(friction_contract.get("joint_friction", ""))
                    == "legacy_v14_unidentified"
                    and str(getattr(action_cfg, "action_contract", ""))
                    == "v11_affine_safe_qdes_v1"
                    and str(
                        getattr(action_cfg, "actual_q_hard_audit_mode", "")
                    )
                    == "telemetry"
                    and int(getattr(action_cfg, "qdes_delay_min_steps", -1)) == 0
                    and int(getattr(action_cfg, "qdes_delay_max_steps", -1)) == 0
                    and abs(
                        float(
                            getattr(
                                action_cfg,
                                "qdes_delay_nominal_fraction",
                                -1.0,
                            )
                        )
                        - 1.0
                    )
                    <= 1.0e-12
                    and str(
                        getattr(motion_cfg, "post_swing_replay_contract", "")
                    )
                    == "legacy_state_v1"
                    and int(getattr(motion_cfg, "post_swing_buffer_size", -1))
                    == 8192
                    and int(getattr(motion_cfg, "post_swing_min_fill", -1))
                    == 256
                    and int(
                        getattr(motion_cfg, "post_swing_capture_phase_bins", -1)
                    )
                    == 1
                    and int(
                        getattr(
                            motion_cfg,
                            "post_swing_capture_severity_bins",
                            -1,
                        )
                    )
                    == 1
                    and not bool(
                        getattr(motion_cfg, "post_swing_risk_edge_capture", True)
                    )
                    and not bool(
                        getattr(motion_cfg, "post_swing_ability_gate_enabled", True)
                    )
                    and tuple(
                        getattr(rt_cfg, "post_strike_capture_delays_s", ())
                    )
                    == ()
                    and tuple(
                        float(value)
                        for value in pd_params.get("alpha_range", ())
                    )
                    == (0.85, 1.15)
                    and tuple(
                        float(value)
                        for value in pd_params.get("beta_range", ())
                    )
                    == (0.85, 1.15)
                    and abs(float(pd_params.get("nominal_fraction", -1.0)) - 0.25)
                    <= 1.0e-12
                )
                if not build_ok:
                    raise RuntimeError(
                        "Refusing HitterPingPong export: the saved V14 recovery "
                        "contract drifted"
                    )
                metadata.update(
                    {
                        "hitter_pingpong_build_contract": (
                            "v14_strike_legacy_recovery_a3_plant_v4"
                        ),
                        "hitter_pingpong_replay_contract": (
                            "legacy_state_v1,1,1,capture=wrap_only,min_fill=256"
                        ),
                        "hitter_pingpong_qdes_delay_contract": (
                            "post_clamp_direct,0..0,nominal_fraction=1.0"
                        ),
                        "hitter_pingpong_pd_contract": (
                            "a3_message_kd_plus_fixed_passive,kp=0.85..1.15,"
                            "kd_message=0.85..1.15,nominal_fraction=0.25"
                        ),
                        "hitter_pingpong_tuple_contract": (
                            "fixed_balanced_bank_v1,0.2500,"
                            f"{getattr(rt_cfg, 'venue_tuple_bank_sha256', '')},"
                            f"{getattr(rt_cfg, 'venue_tuple_bank_receipt_sha256', '')}"
                        ),
                        "hitter_pingpong_physical_ball_contract": (
                            "formal_training_disabled,low_env_audit_only_v1"
                        ),
                        "hitter_pingpong_friction_contract": (
                            "consistent_feet_contact_material_v2,"
                            "legacy_v14_unidentified"
                        ),
                    }
                )
        if bool(getattr(rt_cfg, "locomotion_enabled", False)):
            # V15's station command is not a perpetual position servo.  These YAML-owned
            # values fully specify the finite STAND/STEP scheduler reproduced by the C++ and
            # MuJoCo runners.  The intervention indicator is an actor input but action
            # replacement is deliberately training-only, hence deploy always supplies zero.
            metadata["hitter_pure_locomotion_contract"] = "finite_lateral_gait_v1"
            metadata["hitter_pure_gait_frequency_hz"] = (
                f"{float(rt_cfg.gait_frequency_hz):.9g}"
            )
            metadata["hitter_pure_gait_duty_factor"] = (
                f"{float(rt_cfg.gait_duty_factor):.9g}"
            )
            metadata["hitter_pure_gait_move_deadband"] = (
                f"{float(rt_cfg.gait_move_deadband):.9g}"
            )
            metadata["hitter_pure_gait_step_distance"] = (
                f"{float(rt_cfg.gait_step_distance):.9g}"
            )
            metadata["hitter_pure_gait_max_cycles"] = str(int(rt_cfg.gait_max_cycles))
            metadata["hitter_pure_gait_velocity_max"] = (
                f"{float(rt_cfg.gait_velocity_max):.9g}"
            )
            metadata["hitter_pure_gait_contact_smoothing"] = (
                f"{float(rt_cfg.gait_contact_smoothing):.9g}"
            )
            action_cfg = getattr(getattr(env.cfg, "actions", None), "joint_pos", None)
            action_contract = str(getattr(action_cfg, "action_contract", ""))
            metadata["hitter_pure_intervention_contract"] = (
                "hugwbc_upper_action_replacement_train_only_v2"
                if action_contract == "absolute_feasible_qdes_v5"
                else "hugwbc_upper_action_replacement_train_only_v1"
            )
            metadata["hitter_pure_intervention_deploy_value"] = "0"
        phases = getattr(rt_cfg, "strike_phase_per_clip", None)
        if phases is None:
            phases = [rt_cfg.strike_phase] * motion_cmd.motion.num_segments
        metadata["clip_strike_phases"] = ",".join(f"{float(p):.4f}" for p in phases)
        # Per-clip reference base->racket reach offset at the strike frame (dx0,dy0,dx1,dy1,...).
        # The 177-D hitter_footwork deploy path derives its base station from the frozen racket
        # target as station_xy = racket_target_xy - reach_offset_xy[clip] (base_couple_mode
        # reference_reach) — the C++ runner refuses a 177 model without this key rather than
        # silently feeding a zero station.
        rt_cmd = env.command_manager.get_term("racket_target")
        if hasattr(rt_cmd, "_ensure_reference_strike_state"):
            rt_cmd._ensure_reference_strike_state()
            reach = getattr(rt_cmd, "_ref_reach_offset_xy_per_clip", None)
            if reach is not None:
                metadata["ref_reach_offset_xy"] = ",".join(
                    f"{float(v):.6f}" for v in reach.reshape(-1).cpu().tolist()
                )
        # HITTER-PURE sampling geometry (2026-07-07): the 110-D deploy path derives the base
        # station from the racket target the paper's way (§V-B-3 fh/bh heuristic ONLY computes
        # p̂_base) as station_xy = racket_target_xy − (fixed_plane_x, band_center_y)[clip], and
        # its engage/gate logic needs the TRAINED distribution, so bake the station-relative
        # per-clip boxes + the independent station box. Format: per-clip flat
        # "x_lo,x_hi,y_lo,y_hi,z_lo,z_hi" joined by ';' (clip 0 = forehand, 1 = backhand).
        if getattr(rt_cfg, "target_mode", "") == "hitter_pure":
            _ppc = getattr(rt_cfg, "racket_pos_range_per_clip", None)
            _vpc = getattr(rt_cfg, "racket_vel_range_per_clip", None)
            _vcore = getattr(rt_cfg, "racket_vel_start_range_per_clip", None)
            _vplanner = getattr(rt_cfg, "racket_vel_planner_range_per_clip", None)
            if _ppc is not None:
                metadata["hitter_pure_pos_range_per_clip"] = ";".join(
                    ",".join(f"{float(lo):.4f},{float(hi):.4f}" for (lo, hi) in clip_rng)
                    for clip_rng in _ppc
                )
            if _vpc is not None:
                metadata["hitter_pure_vel_range_per_clip"] = ";".join(
                    ",".join(f"{float(lo):.4f},{float(hi):.4f}" for (lo, hi) in clip_rng)
                    for clip_rng in _vpc
                )
            if _vcore is not None:
                metadata["hitter_pure_vel_core_range_per_clip"] = ";".join(
                    ",".join(f"{float(lo):.4f},{float(hi):.4f}" for (lo, hi) in clip_rng)
                    for clip_rng in _vcore
                )
            if _vplanner is not None:
                metadata["hitter_pure_vel_planner_range_per_clip"] = ";".join(
                    ",".join(f"{float(lo):.4f},{float(hi):.4f}" for (lo, hi) in clip_rng)
                    for clip_rng in _vplanner
                )
                metadata["hitter_pure_vel_planner_mix_prob"] = (
                    f"{float(getattr(rt_cfg, 'racket_vel_planner_mix_prob', 0.0)):.4f}"
                )
                metadata["hitter_pure_vel_range_ramp_steps"] = str(
                    int(getattr(rt_cfg, "racket_vel_range_ramp_steps", 0))
                )
            metadata["hitter_pure_base_target_range"] = ",".join(
                f"{float(v):.4f}"
                for v in (*rt_cfg.base_target_x_range, *rt_cfg.base_target_y_range)
            )
            _step_y = getattr(rt_cfg, "station_y_step_range", None)
            # Recipe identity is independent of station-step metadata.  In
            # particular, R10 intentionally removes station_y_step_range, so
            # placing recipe detection solely under ``_step_y is not None``
            # makes the exact fixed-station recipe impossible to export.
            _v17_contract = _task_identity.get("v17_contract") or {}
            v17_r10_declared = (
                str(_task_identity.get("name", ""))
                == "HOPEPingPongHitterPureRallyV17"
                and str(_task_identity.get("gym_task", ""))
                == "HOPE-PingPong-HitterPureRallyV17-AgibotA3-v0"
                and int(_task_identity.get("recipe_revision", 0) or 0) == 10
                and int(_v17_contract.get("recipe_revision", 0) or 0) == 10
                and str(_v17_contract.get("curriculum", ""))
                == "disabled_fixed_station_survival_v1"
                and str(_v17_contract.get("ready_release", ""))
                == "telemetry_only_ball_clock_v1"
            )
            v17_r12_declared = (
                str(_task_identity.get("name", ""))
                == "HOPEPingPongHitterPureRallyV17"
                and str(_task_identity.get("gym_task", ""))
                == "HOPE-PingPong-HitterPureRallyV17-AgibotA3-v0"
                and int(_task_identity.get("recipe_revision", 0) or 0) == 12
                and int(_v17_contract.get("recipe_revision", 0) or 0) == 12
                and str(_v17_contract.get("baseline", ""))
                == "exact_rally_v11_behavior_plus_25pct_physical_tuple_v1"
                and str(_v17_contract.get("training_delta", ""))
                == "unitree_joint_acc_plus_physical_tuple_bank_v1"
                and str(_v17_contract.get("question_bank", ""))
                == "fixed_75pct_v11_25pct_physical_balanced_v1"
                and str(_v17_contract.get("tuple_outcome", ""))
                == "telemetry_only_unconditional_swing_denominator_v1"
                and str(_v17_contract.get("planner_contract", ""))
                == "schema2_three_stable_revisions_freeze_at_engage_v1"
            )
            if _step_y is not None or v17_r10_declared or v17_r12_declared:
                if _step_y is not None:
                    metadata["hitter_pure_station_y_step_range"] = ",".join(
                        f"{float(v):.4f}" for v in _step_y
                    )
                    _same_p = float(
                        getattr(rt_cfg, "station_y_same_prob", 0.0)
                    )
                    _small_p = float(
                        getattr(rt_cfg, "station_y_small_step_prob", 0.0)
                    )
                    _small_r = getattr(
                        rt_cfg, "station_y_small_step_range", None
                    )
                    if _same_p > 0.0 or _small_p > 0.0:
                        if _small_r is None:
                            _small_r = (0.0, 0.0)
                        metadata["hitter_pure_station_y_mixture"] = ",".join(
                            f"{float(v):.4f}"
                            for v in (_same_p, _small_p, *_small_r, *_step_y)
                        )
                        metadata["hitter_pure_station_side_explicit"] = str(
                            bool(getattr(rt_cfg, "station_side_explicit", False))
                        ).lower()
                # Most RallyFinal descendants retain the v1 wire/runtime contract. RallyV10
                # overrides this to v2 below: its exact core-or-planner command support requires a
                # loader capability that old v1 binaries do not have and therefore must reject.
                metadata["hitter_pure_runtime_contract"] = "rally_final_v1"
                rewards_cfg = getattr(env.cfg, "rewards", None)
                action_cfg = getattr(getattr(env.cfg, "actions", None), "joint_pos", None)
                passive_names = tuple(getattr(action_cfg, "passive_joint_names", ()))
                v3_marker = hasattr(rewards_cfg, "passive_head_raw_action") or bool(passive_names)
                v9_marker = hasattr(rewards_cfg, "windup_x_recovery")
                v10_marker = hasattr(rewards_cfg, "left_wrist_reference_debt")
                # V11 inherits every V10 reward, so left_wrist_reference_debt alone cannot
                # distinguish them. ready_deadline is the first V11-only runtime term.
                v11_marker = getattr(rewards_cfg, "ready_deadline", None) is not None
                # A configclass field remains present when a successor explicitly disables it.
                # Inspect live terms rather than using hasattr, otherwise V13's disabled V12
                # waist term would cause a false rally_v12 stamp.
                v13_marker = getattr(rewards_cfg, "post_swing_xlock", None) is not None
                # RallyV14 inherits the complete V13 command/wire contract.  Its unique live
                # marker is the YAML-owned 35% completed-swing recovery population; the exact
                # stability-reward values are checked below before the distinct recipe is stamped.
                v14_marker = (
                    v13_marker
                    and abs(float(getattr(motion_cmd.cfg, "post_swing_start_prob", 0.0)) - 0.35)
                    < 1.0e-8
                )
                # RallyV17-r5 returns to the V11 110-D/action/runtime contract and adds a
                # task-complete, reversible recovery curriculum.  It is shape-compatible with
                # V11, so the side/phase/severity replay contract is the first-class marker:
                # accepting either the retired r1 string or an arbitrary future string would
                # silently stamp a shape-compatible model with the wrong deployment recipe.
                v17_marker = (
                    v11_marker
                    and not v13_marker
                    and str(getattr(action_cfg, "action_contract", ""))
                    == "v11_affine_safe_qdes_v1"
                    and bool(getattr(rt_cfg, "recovery_curriculum_enabled", False))
                    and str(getattr(motion_cmd.cfg, "post_swing_replay_contract", ""))
                    == "markov_side_phase_severity_v3"
                )
                # A non-null runtime ground receipt uniquely identifies the current r6
                # plant.  It is deliberately exported only as a P0 contract candidate,
                # never under the previously qualified r5 status.
                v17_r6_marker = (
                    v17_marker
                    and getattr(env.cfg, "v17_ground_plant_contract", None)
                    is not None
                )
                # R10 is intentionally NOT inferred from a reward term: it is
                # shape-compatible with V11 but changes the runtime question to
                # one immutable station and a ball-owned release clock.  play.py
                # passes the selected Hydra task identity and we cross-check it
                # against the fully resolved environment below.
                v17_r10_marker = (
                    v17_r10_declared
                    and v11_marker
                    and not v13_marker
                    and str(getattr(action_cfg, "action_contract", ""))
                    == "v11_affine_safe_qdes_v1"
                    and getattr(env.cfg, "v17_ground_plant_contract", None)
                    is not None
                )
                if v17_r10_declared and not v17_r10_marker:
                    raise RuntimeError(
                        "Refusing RallyV17-r10 metadata stamp: the exact Hydra "
                        "identity was selected, but the resolved V11 affine/action, "
                        "reward-lineage, or ground-plant prerequisites drifted"
                    )
                v17_r12_marker = (
                    v17_r12_declared
                    and v11_marker
                    and not v13_marker
                    and str(getattr(action_cfg, "action_contract", ""))
                    == "v11_affine_safe_qdes_v1"
                    and getattr(rewards_cfg, "joint_acc", None) is not None
                    and getattr(
                        getattr(rewards_cfg, "joint_acc", None).func,
                        "__name__",
                        "",
                    )
                    == "joint_acc_l2"
                    and abs(
                        float(getattr(rewards_cfg, "joint_acc").weight)
                        - (-2.5e-7)
                    )
                    < 1.0e-12
                    and bool(getattr(rt_cfg, "base_mocap_enabled", False))
                    and bool(
                        getattr(rt_cfg, "base_mocap_orientation_enabled", False)
                    )
                    and abs(
                        float(
                            getattr(
                                motion_cmd.cfg,
                                "runtime_handoff_start_prob",
                                0.0,
                            )
                        )
                        - 0.05
                    )
                    < 1.0e-12
                    and tuple(
                        int(v)
                        for v in getattr(
                            motion_cmd.cfg,
                            "runtime_handoff_hold_steps_range",
                            (),
                        )
                    )
                    == (50, 150)
                    and bool(getattr(rt_cfg, "venue_tuple_enabled", False))
                    and bool(getattr(rt_cfg, "virtual_ball", False))
                    and bool(
                        getattr(
                            rt_cfg,
                            "venue_tuple_unconditional_outcomes",
                            False,
                        )
                    )
                    and str(
                        getattr(rt_cfg, "venue_tuple_mix_mode", "")
                    )
                    == "fixed_balanced_bank_v1"
                    and abs(
                        float(
                            getattr(
                                rt_cfg, "venue_tuple_final_mix_prob", 0.0
                            )
                        )
                        - 0.25
                    )
                    < 1.0e-12
                    and str(
                        getattr(rt_cfg, "venue_tuple_bank_sha256", "")
                    )
                    == "bc202d6335473f15c2233de7ceece42795dd18b12acf557a21bd611969fc5d03"
                    and str(
                        getattr(
                            rt_cfg,
                            "venue_tuple_bank_receipt_sha256",
                            "",
                        )
                    )
                    == "13e39d1a76d664da29e5ffe0f8df3d6962de981cd0ee83ec87e28043082170c8"
                )
                if v17_r12_declared and not v17_r12_marker:
                    raise RuntimeError(
                        "Refusing RallyV17-r12 metadata stamp: the selected "
                        "task is not exact V11 behavior plus the pinned 25% "
                        "physical bank and standard hardware repair"
                    )
                v12_marker = (
                    getattr(rewards_cfg, "waist_qdes_saturation", None) is not None
                    and not v13_marker
                )
                action_term = env.action_manager.get_term("joint_pos")
                qdes_contract = str(getattr(action_cfg, "action_contract", ""))
                v15_marker = (
                    qdes_contract in (
                        "bounded_qdes_v2",
                        "feasible_qdes_v3",
                        "absolute_feasible_qdes_v5",
                    )
                    and callable(getattr(action_term, "resolved_contract_metadata", None))
                )
                if v17_r10_marker:
                    expected_passive = ("head_yaw_joint", "head_pitch_joint")
                    runtime_passive = tuple(
                        getattr(action_term, "passive_joint_names", ())
                    )
                    terminations_cfg = getattr(env.cfg, "terminations", None)
                    physical_missing = [
                        name
                        for name in ("base_fell_tilt", "base_too_low")
                        if getattr(terminations_cfg, name, None) is None
                    ]
                    fixed_plane_ok = (
                        _ppc is not None
                        and len(_ppc) == 2
                        and all(
                            abs(float(_ppc[clip][0][0]) - 0.58) < 1.0e-8
                            and abs(float(_ppc[clip][0][1]) - 0.58) < 1.0e-8
                            for clip in range(2)
                        )
                    )
                    base_range_ok = (
                        tuple(float(v) for v in rt_cfg.base_target_x_range)
                        == (0.0, 0.0)
                        and tuple(float(v) for v in rt_cfg.base_target_y_range)
                        == (0.0, 0.0)
                        and getattr(rt_cfg, "station_y_step_range", None) is None
                    )
                    resolved_r10_ok = (
                        passive_names == expected_passive
                        and runtime_passive == expected_passive
                        and not physical_missing
                        and fixed_plane_ok
                        and base_range_ok
                        and not bool(
                            getattr(rt_cfg, "recovery_curriculum_enabled", True)
                        )
                        and not bool(getattr(rt_cfg, "ready_release_enabled", True))
                        and not bool(getattr(rt_cfg, "venue_tuple_enabled", True))
                        and abs(
                            float(
                                getattr(motion_cmd.cfg, "post_swing_start_prob", -1.0)
                            )
                        )
                        < 1.0e-12
                        and str(
                            getattr(
                                motion_cmd.cfg, "post_swing_replay_contract", ""
                            )
                        )
                        == "legacy_state_v1"
                        and abs(
                            float(getattr(rt_cfg, "midswing_resample_prob", 0.0))
                        )
                        < 1.0e-12
                    )
                    if not resolved_r10_ok:
                        raise RuntimeError(
                            "Refusing RallyV17-r10 metadata stamp: the resolved "
                            "fixed-station/ball-clock/A3 contract drifted "
                            f"(passive_cfg={passive_names}, passive_runtime={runtime_passive}, "
                            f"physical_missing={physical_missing}, fixed_plane={fixed_plane_ok}, "
                            f"base_range={base_range_ok})"
                        )
                    metadata.update(
                        {
                            "hitter_pure_training_recipe": "rally_v17",
                            "hitter_pure_training_recipe_version": "10",
                            "hitter_pure_runtime_contract": (
                                "rally_v17_fixed_station_ball_clock_v1"
                            ),
                            "hitter_pure_deployment_status": A3_DEPLOYMENT_STATUS,
                            "hitter_pure_qualification_status": (
                                A3_QUALIFICATION_STATUS
                            ),
                            "hitter_pure_validator_profile": "p0_contract_only",
                            "hitter_pure_action_contract": (
                                "v11_affine_safe_qdes_v1"
                            ),
                            "hitter_pure_v17_recipe_revision": "10",
                            "hitter_pure_v17_fixed_station_contract": (
                                "session_anchor_xy_with_10cm_recovery_v1"
                            ),
                            "hitter_pure_v17_release_contract": (
                                "telemetry_only_ball_clock_v1"
                            ),
                            "hitter_pure_v17_target_stream_contract": (
                                "freeze_at_engage_v1"
                            ),
                            "hitter_pure_planner_schema": "2",
                            "hitter_pure_planner_stability_contract": (
                                "three_revisions_v1,0.0300,0.2500,0.0300"
                            ),
                            "hitter_pure_fixed_hit_plane_relative_x_m": "0.5800",
                            "hitter_pure_v17_runtime_handoff": (
                                "static_to_policy_blend_v2,0.0500,50,150,0.5000"
                            ),
                            "hitter_pure_v17_sensor_contract": (
                                "mocap_authoritative,2,"
                                "calibrated_world_base_v1,"
                                "mocap_anchored_pelvis_gyro_v1,"
                                "disabled_for_mocap_authoritative,"
                                "table_p1_to_p2_v1,"
                                "runtime_receipt_required"
                            ),
                        }
                    )
                elif v15_marker:
                    expected_passive = ("head_yaw_joint", "head_pitch_joint")
                    runtime_passive = tuple(getattr(action_term, "passive_joint_names", ()))
                    actual_q_guard_contract = str(
                        getattr(action_cfg, "actual_q_guard_contract", "")
                    )
                    actual_q_guard_horizon_s = float(
                        getattr(action_cfg, "actual_q_guard_horizon_s", 0.0)
                    )
                    actual_q_hard_tolerance_rad = float(
                        getattr(
                            action_cfg,
                            "actual_q_hard_tolerance_rad",
                            0.0,
                        )
                    )
                    terminations_cfg = getattr(env.cfg, "terminations", None)
                    forbidden_live = [
                        name for name in ("anchor_pos", "anchor_ori", "ee_body_pos")
                        if getattr(terminations_cfg, name, None) is not None
                    ]
                    physical_missing = [
                        name for name in ("base_fell_tilt", "base_too_low")
                        if getattr(terminations_cfg, name, None) is None
                    ]
                    if (
                        passive_names != expected_passive
                        or runtime_passive != expected_passive
                        or forbidden_live
                        or physical_missing
                        or (
                            qdes_contract == "feasible_qdes_v3"
                            and actual_q_guard_contract != ""
                        )
                        or (
                            qdes_contract == "feasible_qdes_v3"
                            and abs(actual_q_guard_horizon_s) > 1.0e-12
                        )
                        or (
                            qdes_contract == "feasible_qdes_v3"
                            and abs(
                                actual_q_hard_tolerance_rad - 5.0e-4
                            )
                            > 1.0e-12
                        )
                    ):
                        raise RuntimeError(
                            "Refusing bounded-q_des metadata stamp: structural action/termination "
                            f"contract mismatch cfg={passive_names}, runtime={runtime_passive}, "
                            f"forbidden_live={forbidden_live}, physical_missing={physical_missing}"
                            f", actual_q_guard={actual_q_guard_contract!r}"
                            f", guard_horizon_s={actual_q_guard_horizon_s!r}"
                            f", hard_tolerance_rad={actual_q_hard_tolerance_rad!r}"
                        )
                    metadata["hitter_pure_training_recipe"] = "rally_v15"
                    locomotion_enabled = bool(getattr(rt_cfg, "locomotion_enabled", False))
                    if qdes_contract == "absolute_feasible_qdes_v5":
                        # "6" = default-anchored per-side decode (v5); "5" was the retired v4
                        # full-span single-tanh decode and is never stamped again.
                        recipe_version = "6"
                    elif qdes_contract == "feasible_qdes_v3":
                        # "8" removes the production-size-audit-disproven predictive brake.
                        # Actual-q hard crossings are training episode terminations; q_des and
                        # numeric contract violations remain fatal.
                        recipe_version = "8"
                    else:
                        recipe_version = "3" if locomotion_enabled else "2"
                    metadata["hitter_pure_training_recipe_version"] = recipe_version
                    metadata["hitter_pure_runtime_contract"] = "rally_v15"
                    metadata["hitter_pure_deployment_status"] = "gate3_candidate"
                    metadata["hitter_pure_ready_hold_steps_range"] = ",".join(
                        str(int(v)) for v in motion_cmd.cfg.hold_steps_range
                    )
                    metadata["hitter_pure_hold_long"] = "%.4f,%d,%d" % (
                        float(motion_cmd.cfg.hold_long_prob),
                        int(motion_cmd.cfg.hold_long_steps_range[0]),
                        int(motion_cmd.cfg.hold_long_steps_range[1]),
                    )
                    metadata["hitter_pure_passive_head"] = ",".join(runtime_passive)
                    metadata["hitter_pure_last_action_head"] = "zero"
                    metadata["hitter_pure_last_action_feedback"] = (
                        "executed_qdes_normalized"
                    )
                    metadata["hitter_pure_termination_contract"] = (
                        "physical_fall_plus_q_hard_termination_v2"
                    )
                    metadata["hitter_pure_reach_x_augmented"] = "true"
                    metadata["hitter_pure_qdes_projector"] = (
                        "default_anchored_per_side_tanh+feasible_projection+rate+tracking+pd_headroom"
                        if qdes_contract == "absolute_feasible_qdes_v5"
                        else (
                            "feasible_interval_tanh+safe+rate+tracking+pd_headroom"
                            if qdes_contract == "feasible_qdes_v3"
                            else "local_scale_safe_tanh+rate+tracking+pd_headroom"
                        )
                    )
                elif v9_marker or v10_marker or v11_marker or v12_marker or v13_marker:
                    expected_passive = ("head_yaw_joint", "head_pitch_joint")
                    action_term = env.action_manager.get_term("joint_pos")
                    runtime_passive = tuple(getattr(action_term, "passive_joint_names", ()))
                    terminations_cfg = getattr(env.cfg, "terminations", None)
                    physical_missing = [
                        name for name in ("base_fell_tilt", "base_too_low")
                        if getattr(terminations_cfg, name, None) is None
                    ]
                    wrist_live = getattr(terminations_cfg, "ee_wrist_pos", None) is not None
                    feet_guard_missing = getattr(terminations_cfg, "ee_body_pos", None) is None
                    if (
                        passive_names != expected_passive
                        or runtime_passive != expected_passive
                        or not hasattr(rewards_cfg, "passive_head_raw_action")
                        or not hasattr(rewards_cfg, "rally_joint_qdes_saturation")
                        or wrist_live
                        or feet_guard_missing
                        or physical_missing
                    ):
                        raise RuntimeError(
                            "Refusing RallyV9/V10/V11/V12/V13/V14 metadata stamp: passive-head/termination contract "
                            f"mismatch cfg={passive_names}, runtime={runtime_passive}, "
                            f"wrist_live={wrist_live}, feet_guard_missing={feet_guard_missing}, "
                            f"physical_missing={physical_missing}"
                        )
                    if v10_marker or v11_marker or v12_marker or v13_marker:
                        heading = rewards_cfg.rally_heading_debt
                        windup = rewards_cfg.windup_x_recovery
                        pre_settle = rewards_cfg.pre_strike_station_settle
                        strike_x = rewards_cfg.strike_x_drift
                        wrist = rewards_cfg.left_wrist_reference_debt
                        elbow = rewards_cfg.right_elbow_extension_debt
                        qdes = rewards_cfg.rally_joint_qdes_saturation
                        fixed_plane_ok = (
                            _ppc is not None
                            and len(_ppc) == 2
                            and all(
                                abs(float(_ppc[c][0][0]) - 0.58) < 1.0e-8
                                and abs(float(_ppc[c][0][1]) - 0.58) < 1.0e-8
                                for c in range(2)
                            )
                        )
                        planner_vel_final = (
                            ((1.24, 2.60), (-0.31, 0.69), (0.40, 1.66)),
                            ((1.50, 2.60), (-0.66, 0.40), (0.00, 1.35)),
                        )
                        planner_vel_start = (
                            ((1.24, 2.24), (-0.31, 0.69), (0.66, 1.66)),
                            ((1.60, 2.60), (-0.66, 0.34), (0.00, 0.54)),
                        )
                        planner_vel_distribution = (
                            ((1.57, 2.55), (0.10, 0.52), (0.41, 1.35)),
                            ((1.55, 2.52), (-0.18, 0.29), (0.40, 1.32)),
                        )
                        vel_curriculum_ok = (
                            _vpc is not None
                            and tuple(tuple(tuple(float(v) for v in axis) for axis in clip)
                                      for clip in _vpc) == planner_vel_final
                            and getattr(rt_cfg, "racket_vel_start_range_per_clip", None)
                            is not None
                            and tuple(
                                tuple(tuple(float(v) for v in axis) for axis in clip)
                                for clip in rt_cfg.racket_vel_start_range_per_clip
                            ) == planner_vel_start
                            and getattr(rt_cfg, "racket_vel_planner_range_per_clip", None)
                            is not None
                            and tuple(
                                tuple(tuple(float(v) for v in axis) for axis in clip)
                                for clip in rt_cfg.racket_vel_planner_range_per_clip
                            ) == planner_vel_distribution
                            and abs(float(getattr(rt_cfg, "racket_vel_planner_mix_prob", 0.0))
                                    - 0.75) < 1.0e-8
                            and int(getattr(rt_cfg, "racket_vel_range_ramp_steps", 0)) == 96000
                        )
                        component_common_ok = (
                            fixed_plane_ok
                            and vel_curriculum_ok
                            and getattr(heading.func, "__name__", "")
                            == "rally_heading_settle_debt"
                            and getattr(elbow.func, "__name__", "") == "right_elbow_extension_debt"
                            and abs(float(windup.params.get("x_margin", 0.0)) - 0.02) < 1.0e-8
                            and abs(float(windup.params.get("t_hi", 0.0)) - 1.10) < 1.0e-8
                            and abs(float(pre_settle.params.get("t_max", 0.0)) - 1.10) < 1.0e-8
                            and abs(float(strike_x.params.get("margin", 0.0)) - 0.02) < 1.0e-8
                            and abs(float(strike_x.params.get("std", 0.0)) - 0.05) < 1.0e-8
                            and abs(float(strike_x.params.get("t_pre", 0.0)) - 1.10) < 1.0e-8
                            and abs(float(strike_x.params.get("t_post", 0.0)) - 1.55) < 1.0e-8
                            and bool(strike_x.params.get("huber_tail", False))
                            and abs(float(heading.params.get("yaw_rate_gain", 0.0)) - 2.0) < 1.0e-8
                            and abs(float(heading.params.get("yaw_rate_max", 0.0)) - 0.15) < 1.0e-8
                        )
                        legacy_elbow_ok = (
                            abs(float(elbow.params.get("extension_start", 0.0)) - 1.30) < 1.0e-8
                            and abs(float(elbow.params.get("std", 0.0)) - 0.15) < 1.0e-8
                            and abs(float(elbow.params.get("t_pre", 0.0)) - 0.25) < 1.0e-8
                            and abs(float(elbow.params.get("t_post", 0.0)) - 0.10) < 1.0e-8
                            and not bool(elbow.params.get("forehand_only", False))
                        )
                        v10_common_ok = component_common_ok and legacy_elbow_ok
                        if v11_marker:
                            ready_deadline = rewards_cfg.ready_deadline
                            ready_stance_width = rewards_cfg.ready_stance_width
                            ready_foot_alignment = rewards_cfg.ready_foot_alignment
                            ready_leg_settle = rewards_cfg.ready_leg_settle
                            v11_ok = (
                                component_common_ok
                                and (v13_marker or legacy_elbow_ok)
                                and getattr(ready_deadline.func, "__name__", "")
                                == "rally_ready_deadline_debt"
                                and getattr(ready_stance_width.func, "__name__", "")
                                == "rally_ready_stance_width_debt"
                                and getattr(ready_foot_alignment.func, "__name__", "")
                                == "rally_ready_foot_alignment_debt"
                                and getattr(ready_leg_settle.func, "__name__", "")
                                == "rally_ready_leg_settle_debt"
                                and abs(float(heading.weight) - (-0.50 if v14_marker else -0.40))
                                < 1.0e-8
                                and abs(float(heading.params.get("yaw_margin", 0.0)) - 0.10)
                                < 1.0e-8
                                and abs(float(heading.params.get("heading_blend", 0.0)) - 0.50)
                                < 1.0e-8
                                and abs(float(heading.params.get("post_t_hi", 0.0)) - 1.55)
                                < 1.0e-8
                                and abs(float(wrist.weight) - (-0.40 if v14_marker else -0.30))
                                < 1.0e-8
                                and abs(float(wrist.params.get("margin", 0.0)) - 0.08) < 1.0e-8
                                and abs(float(wrist.params.get("max_blend", 0.0)) - 0.75)
                                < 1.0e-8
                                and (
                                    v13_marker
                                    or (
                                        abs(float(qdes.weight) - (-0.25)) < 1.0e-8
                                        and abs(float(qdes.params.get("max_blend", 0.0)) - 1.0)
                                        < 1.0e-8
                                    )
                                )
                                and abs(float(pre_settle.weight) - (-0.35)) < 1.0e-8
                                and abs(float(ready_deadline.weight) - (-0.35)) < 1.0e-8
                                and abs(float(ready_deadline.params.get("x_margin", 0.0)) - 0.10)
                                < 1.0e-8
                                and abs(float(ready_deadline.params.get("y_margin", 0.0)) - 0.10)
                                < 1.0e-8
                                and abs(float(ready_deadline.params.get("speed_margin", 0.0)) - 0.20)
                                < 1.0e-8
                                and abs(float(ready_deadline.params.get("final_window_s", 0.0)) - 0.12)
                                < 1.0e-8
                                and int(ready_deadline.params.get("target_step_class", -1)) == 3
                                and tuple(int(v) for v in motion_cmd.cfg.hold_steps_range) == (45, 60)
                                and int(motion_cmd.cfg.stand_start_min_hold) == 45
                                and abs(float(getattr(rt_cfg, "station_y_positive_main_prob", 0.0))
                                        - 0.80) < 1.0e-8
                                and tuple(float(v) for v in getattr(
                                    rt_cfg, "station_y_positive_main_step_range", ()))
                                == (0.19, 0.24)
                            )
                            if not v11_ok:
                                raise RuntimeError(
                                    "Refusing RallyV11 metadata stamp: training recipe drifted "
                                    "(the same base contract is required by RallyV12/V13)"
                                )
                            if v17_marker:
                                safe_set = getattr(
                                    rewards_cfg, "recovery_safe_set_cost", None
                                )
                                v17_gate_values = (
                                    ("recovery_min_environment_steps", 24000.0),
                                    ("recovery_stage1_min_exact_samples_per_side", 200.0),
                                    ("recovery_stage1_min_swing_starts_per_side", 400.0),
                                    ("recovery_stage2_min_exact_samples_per_side", 500.0),
                                    ("recovery_stage2_min_swing_starts_per_side", 1000.0),
                                    ("recovery_stage2_min_virtual_samples_per_side", 200.0),
                                    (
                                        "recovery_stage2_min_actual_q_window_starts_per_side",
                                        1000.0,
                                    ),
                                    ("recovery_actual_q_window_steps", 500.0),
                                    ("recovery_stage1_ready_dwell_steps", 250.0),
                                    (
                                        "recovery_stage1_acquisition_timeout_steps",
                                        500.0,
                                    ),
                                    ("recovery_stage2_enter_completion", 0.75),
                                    ("recovery_stage2_enter_position", 0.70),
                                    ("recovery_stage2_enter_velocity", 0.75),
                                    ("recovery_stage2_enter_normal", 0.75),
                                    ("recovery_stage2_enter_composite", 0.60),
                                    ("recovery_stage2_enter_ready", 0.80),
                                    ("recovery_stage2_enter_safe_recovery", 0.85),
                                    ("recovery_stage2_enter_virtual_contact", 0.85),
                                    ("recovery_stage2_enter_virtual_over_net", 0.75),
                                    ("recovery_stage2_enter_virtual_legal", 0.65),
                                    ("recovery_stage2_enter_post_fall_max", 0.03),
                                    ("recovery_stage2_enter_actual_q_fault_max", 0.0),
                                    ("recovery_stage2_enter_dwell_steps", 500.0),
                                    ("recovery_stage2_exit_completion", 0.65),
                                    ("recovery_stage2_exit_position", 0.55),
                                    ("recovery_stage2_exit_velocity", 0.60),
                                    ("recovery_stage2_exit_normal", 0.60),
                                    ("recovery_stage2_exit_composite", 0.45),
                                    ("recovery_stage2_exit_ready", 0.70),
                                    ("recovery_stage2_exit_safe_recovery", 0.75),
                                    ("recovery_stage2_exit_virtual_contact", 0.75),
                                    ("recovery_stage2_exit_virtual_over_net", 0.65),
                                    ("recovery_stage2_exit_virtual_legal", 0.50),
                                    ("recovery_stage2_exit_post_fall_max", 0.05),
                                    ("recovery_stage2_exit_actual_q_fault_max", 0.005),
                                    ("recovery_stage2_exit_dwell_steps", 150.0),
                                    ("recovery_stage0_to_1_ramp_steps", 8000.0),
                                    ("recovery_stage1_coverage_ramp_steps", 8000.0),
                                    ("recovery_stage1_to_0_ramp_steps", 4000.0),
                                    ("recovery_stage1_to_2_ramp_steps", 12000.0),
                                    ("recovery_stage2_to_1_ramp_steps", 6000.0),
                                )
                                v17_gates_ok = all(
                                    abs(
                                        float(getattr(rt_cfg, name, float("nan")))
                                        - expected
                                    )
                                    < 1.0e-8
                                    for name, expected in v17_gate_values
                                )
                                v17_ok = (
                                    safe_set is not None
                                    and v17_gates_ok
                                    and tuple(
                                        float(v) for v in getattr(
                                            rt_cfg, "post_strike_capture_delays_s", ()
                                        )
                                    ) == (0.08, 0.30, 0.80)
                                    and abs(float(getattr(
                                        rt_cfg, "post_strike_capture_max_tilt", 0.0
                                    )) - 0.45) < 1.0e-8
                                    and bool(getattr(
                                        rt_cfg, "ready_release_enabled", False
                                    ))
                                    and abs(float(getattr(
                                        rt_cfg, "ready_release_arm_tts_s", 0.0
                                    )) - 0.30) < 1.0e-8
                                    and abs(float(getattr(
                                        rt_cfg, "ready_release_timeout_s", 0.0
                                    )) - 1.50) < 1.0e-8
                                    and int(getattr(
                                        rt_cfg, "ready_monitor_dwell_ticks", 0
                                    )) == 5
                                    and bool(getattr(
                                        rt_cfg, "venue_tuple_enabled", False
                                    ))
                                    and bool(getattr(rt_cfg, "virtual_ball", False))
                                    and abs(float(getattr(
                                        rt_cfg, "venue_tuple_final_mix_prob", 0.0
                                    )) - 0.75) < 1.0e-8
                                    and abs(float(getattr(
                                        rt_cfg, "venue_tuple_speed_limit_mps", 0.0
                                    )) - 3.50) < 1.0e-8
                                    and abs(float(getattr(
                                        motion_cmd.cfg, "post_swing_start_prob", 0.0
                                    )) - 0.10) < 1.0e-8
                                    and int(getattr(
                                        motion_cmd.cfg, "post_swing_buffer_size", 0
                                    )) == 12288
                                    and int(getattr(
                                        motion_cmd.cfg, "post_swing_min_fill", 0
                                    )) == 384
                                    and int(getattr(
                                        motion_cmd.cfg,
                                        "post_swing_capture_phase_bins",
                                        0,
                                    )) == 4
                                    and int(getattr(
                                        motion_cmd.cfg,
                                        "post_swing_capture_severity_bins",
                                        0,
                                    )) == 3
                                    and bool(getattr(
                                        motion_cmd.cfg,
                                        "post_swing_risk_edge_capture",
                                        False,
                                    ))
                                    and int(getattr(
                                        motion_cmd.cfg,
                                        "post_swing_min_fill_per_bucket",
                                        0,
                                    )) == 16
                                    and abs(float(getattr(
                                        motion_cmd.cfg,
                                        "runtime_handoff_start_prob",
                                        0.0,
                                    )) - 0.05) < 1.0e-8
                                    and tuple(int(v) for v in getattr(
                                        motion_cmd.cfg,
                                        "runtime_handoff_hold_steps_range",
                                        (),
                                    )) == (50, 150)
                                    and int(getattr(
                                        motion_cmd.cfg, "post_swing_min_hold", 0
                                    )) == 100
                                    and int(getattr(
                                        motion_cmd.cfg, "post_wrap_min_hold", 0
                                    )) == 100
                                    and int(getattr(
                                        motion_cmd.cfg,
                                        "post_wrap_hold_jitter_steps",
                                        0,
                                    )) == 50
                                    and int(getattr(rt_cfg, "target_delay_steps", 0)) == 1
                                    and abs(float(getattr(
                                        rt_cfg, "target_noise_white", 0.0
                                    )) - 0.0019) < 1.0e-8
                                    and abs(float(getattr(
                                        rt_cfg, "target_noise_ar1_sigma", 0.0
                                    )) - 0.0030) < 1.0e-8
                                    and abs(float(getattr(
                                        rt_cfg, "target_noise_ar1_rho", 0.0
                                    )) - 0.717) < 1.0e-8
                                    and abs(float(getattr(
                                        rt_cfg, "target_dropout_prob", 0.0
                                    )) - 0.005) < 1.0e-8
                                    and abs(float(getattr(
                                        rt_cfg, "target_post_strike_dropout_s", 0.0
                                    )) - 0.02) < 1.0e-8
                                    and getattr(safe_set.func, "__name__", "")
                                    == "recovery_safe_set_cost"
                                    and abs(float(safe_set.weight) - (-0.35)) < 1.0e-8
                                    and bool(safe_set.params.get(
                                        "curriculum_scaled", False
                                    ))
                                    and bool(safe_set.params.get(
                                        "include_replay", False
                                    ))
                                    and bool(safe_set.params.get(
                                        "include_ready_hold", False
                                    ))
                                    and int(safe_set.params.get("topk", 0)) == 3
                                    and abs(float(safe_set.params.get(
                                        "max_blend", 0.0
                                    )) - 0.50) < 1.0e-8
                                )
                                if not v17_ok:
                                    raise RuntimeError(
                                        "Refusing RallyV17-r5 metadata stamp: READY, venue, "
                                        "safe-set, replay, or reversible curriculum drifted"
                                    )
                            if v12_marker or v13_marker:
                                strike_x_gate = rewards_cfg.strike_x_gate_margin
                                idle_wrist = rewards_cfg.idle_left_wrist_debt
                                residual_common_ok = (
                                    getattr(strike_x_gate.func, "__name__", "")
                                    == "rally_strike_x_margin_debt"
                                    and getattr(idle_wrist.func, "__name__", "")
                                    == "rally_idle_left_wrist_debt"
                                    and abs(float(strike_x_gate.weight) - (-1.00)) < 1.0e-8
                                    and abs(float(strike_x_gate.params.get("margin", 0.0)) - 0.015)
                                    < 1.0e-8
                                    and abs(float(strike_x_gate.params.get("std", 0.0)) - 0.020)
                                    < 1.0e-8
                                    and abs(float(strike_x_gate.params.get("half_window_s", 0.0))
                                            - 0.040) < 1.0e-8
                                    and abs(float(idle_wrist.weight) - (-0.40)) < 1.0e-8
                                    and abs(float(idle_wrist.params.get("position_margin", 0.0))
                                            - 0.020) < 1.0e-8
                                    and abs(float(idle_wrist.params.get("position_std", 0.0))
                                            - 0.120) < 1.0e-8
                                    and abs(float(idle_wrist.params.get("velocity_margin", 0.0))
                                            - 0.050) < 1.0e-8
                                    and abs(float(idle_wrist.params.get("velocity_std", 0.0))
                                            - 0.250) < 1.0e-8
                                    and abs(float(idle_wrist.params.get("velocity_blend", 0.0))
                                            - 0.250) < 1.0e-8
                                    and abs(float(idle_wrist.params.get("max_blend", 0.0))
                                            - 0.750) < 1.0e-8
                                )
                                if v12_marker:
                                    waist_qdes = rewards_cfg.waist_qdes_saturation
                                    v12_ok = (
                                        residual_common_ok
                                        and getattr(waist_qdes.func, "__name__", "")
                                        == "rally_waist_qdes_saturation_penalty"
                                        and abs(float(waist_qdes.weight) - (-0.50)) < 1.0e-8
                                        and abs(float(waist_qdes.params.get("std", 0.0)) - 0.100)
                                        < 1.0e-8
                                        and abs(float(waist_qdes.params.get("max_blend", 0.0))
                                                - 1.000) < 1.0e-8
                                    )
                                else:
                                    post_xlock = rewards_cfg.post_swing_xlock
                                    post_base = rewards_cfg.post_swing_base_quiet
                                    root_height = rewards_cfg.rally_ready_root_height_debt
                                    post_tilt = rewards_cfg.post_swing_tilt_debt
                                    v13_ok = (
                                        residual_common_ok
                                        and getattr(qdes.func, "__name__", "")
                                        == "rally_all_joint_qdes_barrier"
                                        and qdes.params.get("command_name") == "racket_target"
                                        and qdes.params.get("action_name") == "joint_pos"
                                        and abs(float(qdes.weight) - (
                                            -0.65 if v14_marker else -0.45)) < 1.0e-8
                                        and abs(float(qdes.params.get("safe_margin_fraction", 0.0))
                                                - (0.08 if v14_marker else 0.05)) < 1.0e-8
                                        and abs(float(qdes.params.get("std_fraction", 0.0)) - 0.03)
                                        < 1.0e-8
                                        and int(qdes.params.get("topk", 0)) == 4
                                        and abs(float(qdes.params.get("topk_blend", 0.0)) -
                                                (0.90 if v14_marker else 0.75)) < 1.0e-8
                                        and getattr(rewards_cfg, "waist_qdes_saturation", None)
                                        is None
                                        and abs(float(elbow.params.get("extension_start", 0.0))
                                                - 1.25) < 1.0e-8
                                        and abs(float(elbow.params.get("std", 0.0)) - 0.12)
                                        < 1.0e-8
                                        and abs(float(elbow.weight) -
                                                (-0.50 if v14_marker else -0.35)) < 1.0e-8
                                        and abs(float(elbow.params.get("t_pre", 0.0)) - 0.30)
                                        < 1.0e-8
                                        and abs(float(elbow.params.get("t_post", 0.0)) -
                                                (0.20 if v14_marker else 0.12))
                                        < 1.0e-8
                                        and bool(elbow.params.get("forehand_only", False))
                                        and elbow.params.get("command_name") == "racket_target"
                                        and tuple(getattr(
                                            elbow.params.get("asset_cfg"), "joint_names", ()))
                                        == ("right_elbow_joint",)
                                        and strike_x_gate.params.get("command_name")
                                        == "racket_target"
                                        and abs(float(strike_x_gate.params.get(
                                            "forehand_scale", 0.0)) -
                                            (0.75 if v14_marker else 0.50)) < 1.0e-8
                                        and abs(float(strike_x_gate.params.get(
                                            "backhand_scale", 0.0)) - 1.25) < 1.0e-8
                                        and getattr(post_xlock.func, "__name__", "")
                                        == "rally_post_swing_xlock_debt"
                                        and post_xlock.params.get("command_name")
                                        == "racket_target"
                                        and abs(float(post_xlock.weight) -
                                                (-0.45 if v14_marker else -0.35)) < 1.0e-8
                                        and abs(float(post_xlock.params.get("margin", 0.0)) - 0.03)
                                        < 1.0e-8
                                        and abs(float(post_xlock.params.get("std", 0.0)) - 0.04)
                                        < 1.0e-8
                                        and abs(float(post_xlock.params.get("t_lo", 0.0)) - 0.10)
                                        < 1.0e-8
                                        and abs(float(post_xlock.params.get("t_hi", 0.0)) - 1.55)
                                        < 1.0e-8
                                        and abs(float(post_xlock.params.get(
                                            "forehand_scale", 0.0)) - 1.00) < 1.0e-8
                                        and abs(float(post_xlock.params.get(
                                            "backhand_scale", 0.0)) - 1.25) < 1.0e-8
                                        and abs(float(heading.params.get(
                                            "forehand_scale", 0.0)) -
                                            (1.25 if v14_marker else 1.00)) < 1.0e-8
                                        and abs(float(heading.params.get(
                                            "backhand_scale", 0.0)) - 1.50) < 1.0e-8
                                        and heading.params.get("command_name") == "racket_target"
                                        and ready_deadline.params.get("command_name")
                                        == "racket_target"
                                        and ready_deadline.params.get("motion_command_name")
                                        == "motion"
                                        and tuple(int(v) for v in ready_deadline.params.get(
                                            "target_step_classes", ())) == (1, 2, 3)
                                        and abs(float(post_base.weight) -
                                                (-0.35 if v14_marker else -0.25)) < 1.0e-8
                                        and abs(float(root_height.weight) -
                                                (-0.35 if v14_marker else -0.20)) < 1.0e-8
                                        and abs(float(post_tilt.weight) -
                                                (-0.70 if v14_marker else -0.50)) < 1.0e-8
                                        and abs(float(getattr(
                                            motion_cmd.cfg, "post_swing_start_prob", 0.0)) -
                                            (0.35 if v14_marker else 0.25)) < 1.0e-8
                                        and int(getattr(
                                            motion_cmd.cfg, "post_swing_buffer_size", 0)) == 8192
                                        and int(getattr(
                                            motion_cmd.cfg, "post_swing_min_fill", 0)) == 256
                                        and int(getattr(
                                            motion_cmd.cfg, "post_swing_min_hold", 0)) == 45
                                    )
                                if v12_marker and not v12_ok:
                                    raise RuntimeError(
                                        "Refusing RallyV12 metadata stamp: residual recipe drifted"
                                    )
                                if v13_marker and not v13_ok:
                                    raise RuntimeError(
                                        "Refusing RallyV13/V14 metadata stamp: residual recipe drifted"
                                    )
                                metadata["hitter_pure_training_recipe"] = (
                                    "rally_v14" if v14_marker else
                                    ("rally_v13" if v13_marker else "rally_v12")
                                )
                                metadata["hitter_pure_training_recipe_version"] = "1"
                            else:
                                metadata["hitter_pure_training_recipe"] = (
                                    "rally_v17"
                                    if (v17_marker or v17_r12_marker)
                                    else "rally_v11"
                                )
                                metadata["hitter_pure_training_recipe_version"] = (
                                    "12"
                                    if v17_r12_marker
                                    else (
                                        ("6" if v17_r6_marker else "5")
                                        if v17_marker
                                        else "1"
                                    )
                                )
                            # Preserve the actual v2 wire capability, but give the model a
                            # distinct recipe. Gate3-capable loaders validate every V11--V13-only
                            # field below before accepting the model; this is a candidate marker,
                            # not a real-hardware approval.
                            metadata["hitter_pure_runtime_contract"] = "rally_final_v2"
                            metadata["hitter_pure_deployment_status"] = (
                                A3_DEPLOYMENT_STATUS
                                if v17_r6_marker
                                else "gate3_candidate"
                            )
                            if not v13_marker:
                                metadata["hitter_pure_joint_qdes_max_blend"] = "1.0000"
                            metadata["hitter_pure_ready_deadline"] = (
                                "0.1000,0.1000,0.2000,0.1200,classes1+2+3"
                                if v13_marker else
                                "0.1000,0.1000,0.2000,0.1200,class3"
                            )
                            metadata["hitter_pure_station_y_positive_main"] = (
                                "0.8000,0.1900,0.2400"
                            )
                            metadata["hitter_pure_ready_stance"] = (
                                "0.1000,0.1500,0.2000,0.2500,0.3500"
                            )
                            metadata["hitter_pure_left_wrist_debt"] = "0.0800,0.7500"
                            metadata["hitter_pure_heading_debt"] = (
                                ("0.1000,0.5000,1.5500,1.2500,1.5000"
                                 if v14_marker else
                                 "0.1000,0.5000,1.5500,1.0000,1.5000")
                                if v13_marker else "0.1000,0.5000,1.5500"
                            )
                            if v17_marker or v17_r12_marker:
                                metadata["hitter_pure_action_contract"] = (
                                    "v11_affine_safe_qdes_v1"
                                )
                                metadata["hitter_pure_v17_recipe_revision"] = (
                                    "12"
                                    if v17_r12_marker
                                    else ("6" if v17_r6_marker else "5")
                                )
                                if v17_r6_marker:
                                    # P0 closes the numerical deploy contract without
                                    # changing the trained actor.  This status is consumed
                                    # by a separate no-publish C++ audit profile; the
                                    # production loader rejects it until later qualification
                                    # supplies a new immutable receipt.
                                    metadata["hitter_pure_qualification_status"] = (
                                        A3_QUALIFICATION_STATUS
                                    )
                                    metadata["hitter_pure_validator_profile"] = (
                                        "p0_contract_only"
                                    )
                                if v17_r12_marker:
                                    metadata["hitter_pure_v17_qdes_training"] = (
                                        "v11_action_rate_joint_limit_saturation_plus_"
                                        "unitree_joint_acc_v1,-0.100000,-0.000000250"
                                    )
                                    metadata["hitter_pure_planner_schema"] = "2"
                                    metadata[
                                        "hitter_pure_planner_stability_contract"
                                    ] = "three_revisions_v1,0.0300,0.2500,0.0300"
                                    metadata[
                                        "hitter_pure_v17_target_stream_contract"
                                    ] = "freeze_at_engage_v1"
                                    metadata["hitter_pure_v17_venue_tuple"] = (
                                        "fixed_balanced_high_fidelity_bank_v1,"
                                        "0.2500,50000,50000,"
                                        "bc202d6335473f15c2233de7ceece42795dd18b12acf557a21bd611969fc5d03,"
                                        "13e39d1a76d664da29e5ffe0f8df3d6962de981cd0ee83ec87e28043082170c8,"
                                        "unconditional_swing_denominator_v1"
                                    )
                                else:
                                    # Exact V17-r5 training-only recipe receipts. The runner
                                    # validates these values so a shape-compatible V11 actor
                                    # cannot enter Gate3 under a renamed file.
                                    metadata["hitter_pure_v17_recovery_recipe"] = (
                                        "markov_side_phase_severity_v3,0.1000,12288,"
                                        "384,4,3,0.1800,0.3200,0.1200,0.0700,"
                                        "0.1000,1.1000,16,100,100,50"
                                    )
                                    metadata["hitter_pure_v17_curriculum_gates"] = (
                                        (
                                            "reversible_ready_acquisition_eligible_"
                                            "completion_coverage_v5"
                                        )
                                        if v17_r6_marker
                                        else (
                                            "200,400;500,1000,200,1000,500;"
                                            "0.7500,0.7000,0.7500,0.7500,0.6000,"
                                            "0.8000,0.8500,0.8500,0.7500,0.6500,"
                                            "0.0300,0.0000,500;"
                                            "0.6500,0.5500,0.6000,0.6000,0.4500,"
                                            "0.7000,0.7500,0.7500,0.6500,0.5000,"
                                            "0.0500,0.0050,150;"
                                            "8000,8000,4000,12000,6000;250,500;"
                                            "release_eligible_completion_v1"
                                        )
                                    )
                                    metadata["hitter_pure_v17_ready_release"] = (
                                        "scale_sampled_ready_latch_v1,0.3000,1.5000,"
                                        "0.1000,0.1000,0.2000,0.1500,0.2000,"
                                        "0.1000,0.3500,0.0300,5"
                                    )
                                    metadata["hitter_pure_v17_venue_tuple"] = (
                                        "correlated_mirror_law_reference_hemisphere_v2,"
                                        "0.7500,3.5000,8,"
                                        "2.1700,3.0400,-0.5000,0.5000"
                                    )
                                    metadata["hitter_pure_v17_recovery_safe_set"] = (
                                        "bounded_max_top3_safe_set_ready_hold_rational_v3,"
                                        "-0.3500,0.1000,1.5500,3,0.5000,1"
                                    )
                                metadata["hitter_pure_v17_runtime_handoff"] = (
                                    "static_to_policy_blend_v2,0.0500,50,150,0.5000"
                                )
                                metadata["hitter_pure_v17_sensor_contract"] = (
                                    "mocap_authoritative,2,"
                                    "calibrated_world_base_v1,"
                                    "mocap_anchored_pelvis_gyro_v1,"
                                    "disabled_for_mocap_authoritative,"
                                    "table_p1_to_p2_v1,"
                                    "runtime_receipt_required"
                                )
                            if v12_marker or v13_marker:
                                metadata["hitter_pure_strike_x_gate_margin"] = (
                                    ("0.0150,0.0200,0.0400,0.7500,1.2500"
                                     if v14_marker else
                                     "0.0150,0.0200,0.0400,0.5000,1.2500")
                                    if v13_marker else "0.0150,0.0200,0.0400"
                                )
                                metadata["hitter_pure_idle_left_wrist_debt"] = (
                                    "0.0200,0.1200,0.0500,0.2500,0.2500,0.7500,outward"
                                )
                                if v12_marker:
                                    metadata["hitter_pure_waist_qdes_saturation"] = (
                                        "soft_limit,0.1000,1.0000"
                                    )
                                else:
                                    metadata["hitter_pure_all_joint_qdes_barrier"] = (
                                        "official_hard,0.0800,0.0300,4,0.9000,all31"
                                        if v14_marker else
                                        "official_hard,0.0500,0.0300,4,0.7500,all31"
                                    )
                                    metadata["hitter_pure_post_swing_xlock"] = (
                                        "0.0300,0.0400,0.1000,1.5500,1.0000,1.2500"
                                    )
                                    if v14_marker:
                                        metadata["hitter_pure_v14_stability_recipe"] = (
                                            "0.3500,-0.6500,0.0800,0.0300,4,0.9000,"
                                            "-0.5000,0.2000,0.7500,-0.4500,-0.5000,"
                                            "1.2500,-0.4000,-0.3500,-0.3500,-0.7000"
                                        )
                        else:
                            v10_ok = (
                                v10_common_ok
                                and abs(float(heading.params.get("heading_blend", 0.0)) - 0.35)
                                < 1.0e-8
                                and abs(float(wrist.params.get("max_blend", 0.0)) - 0.50)
                                < 1.0e-8
                                and abs(float(qdes.params.get("max_blend", 0.0)) - 0.50)
                                < 1.0e-8
                            )
                            if not v10_ok:
                                raise RuntimeError(
                                    "Refusing RallyV10 metadata stamp: recovery recipe drifted"
                                )
                            metadata["hitter_pure_training_recipe"] = "rally_v10"
                            metadata["hitter_pure_training_recipe_version"] = "7"
                            metadata["hitter_pure_runtime_contract"] = "rally_final_v2"
                            metadata["hitter_pure_joint_qdes_max_blend"] = "0.5000"
                        metadata["hitter_pure_left_wrist_reference"] = (
                            "hold_frame0_swing_current"
                        )
                        metadata["hitter_pure_pre_settle_t_max"] = "1.1000"
                        metadata["hitter_pure_yaw_settle"] = "first_order"
                        metadata["hitter_pure_strike_x_drift"] = (
                            "0.0200,0.0500,1.1000,1.5500,huber"
                        )
                        metadata["hitter_pure_right_elbow_extension"] = (
                            ("1.2500,0.1200,0.3000,0.2000,forehand_only"
                             if v14_marker else
                             "1.2500,0.1200,0.3000,0.1200,forehand_only")
                            if v13_marker else "1.3000,0.1500,0.2500,0.1000"
                        )
                        metadata["hitter_pure_fixed_plane_x"] = "0.5800"
                        metadata["hitter_pure_vel_curriculum"] = (
                            "v13_core25+planner_q05_q95_75,96000"
                        )
                    else:
                        metadata["hitter_pure_training_recipe"] = "rally_v9"
                        metadata["hitter_pure_training_recipe_version"] = "5"
                    metadata["hitter_pure_ready_hold_steps_range"] = ",".join(
                        str(int(v)) for v in motion_cmd.cfg.hold_steps_range
                    )
                    metadata["hitter_pure_hold_long"] = "%.4f,%d,%d" % (
                        float(motion_cmd.cfg.hold_long_prob),
                        int(motion_cmd.cfg.hold_long_steps_range[0]),
                        int(motion_cmd.cfg.hold_long_steps_range[1]),
                    )
                    metadata["hitter_pure_passive_head"] = ",".join(runtime_passive)
                    metadata["hitter_pure_last_action_head"] = "zero"
                    metadata["hitter_pure_termination_contract"] = (
                        "rally_v8_reference_feet_no_wrist"
                    )
                    metadata["hitter_pure_reach_x_augmented"] = "true"
                elif v3_marker:
                    expected_passive = ("head_yaw_joint", "head_pitch_joint")
                    action_term = env.action_manager.get_term("joint_pos")
                    runtime_passive = tuple(getattr(action_term, "passive_joint_names", ()))
                    terminations_cfg = getattr(env.cfg, "terminations", None)
                    forbidden_live = [
                        name for name in ("anchor_pos", "anchor_ori", "ee_body_pos")
                        if getattr(terminations_cfg, name, None) is not None
                    ]
                    physical_missing = [
                        name for name in ("base_fell_tilt", "base_too_low")
                        if getattr(terminations_cfg, name, None) is None
                    ]
                    if (
                        passive_names != expected_passive
                        or runtime_passive != expected_passive
                        or not hasattr(rewards_cfg, "passive_head_raw_action")
                        or forbidden_live
                        or physical_missing
                    ):
                        raise RuntimeError(
                            "Refusing RallyFinalV3 metadata stamp: actual passive-head/termination "
                            f"contract mismatch cfg={passive_names}, runtime={runtime_passive}, "
                            f"forbidden_live={forbidden_live}, physical_missing={physical_missing}"
                        )
                    metadata["hitter_pure_training_recipe"] = "rally_final_v3"
                    metadata["hitter_pure_training_recipe_version"] = "3"
                    metadata["hitter_pure_ready_hold_steps_range"] = ",".join(
                        str(int(v)) for v in motion_cmd.cfg.hold_steps_range
                    )
                    metadata["hitter_pure_passive_head"] = ",".join(runtime_passive)
                    metadata["hitter_pure_last_action_head"] = "zero"
                    metadata["hitter_pure_termination_contract"] = "physical_fall_only"
                elif float(getattr(motion_cmd.cfg, "hold_long_prob", 0.0) or 0.0) > 0.0:
                    # RallyV8 (2026-07-12): the merged Pure→V3→V4 recipe. Its unique training
                    # marker is the LONG-HOLD TAIL (hold_long_prob>0 — no other lineage has it);
                    # it also borrows FinalV2's rally_ankle_qdes_saturation term, so this branch
                    # MUST come before the v2 sniff or V8 exports get mislabeled rally_final_v2
                    # (whose consumers assert plane 0.51 / hold 40,60 → gate3 report goes fatal).
                    metadata["hitter_pure_training_recipe"] = "rally_v8"
                    metadata["hitter_pure_training_recipe_version"] = "4"
                    metadata["hitter_pure_ready_hold_steps_range"] = ",".join(
                        str(int(v)) for v in motion_cmd.cfg.hold_steps_range
                    )
                    metadata["hitter_pure_hold_long"] = "%.4f,%d,%d" % (
                        float(motion_cmd.cfg.hold_long_prob),
                        int(motion_cmd.cfg.hold_long_steps_range[0]),
                        int(motion_cmd.cfg.hold_long_steps_range[1]),
                    )
                elif hasattr(rewards_cfg, "rally_ankle_qdes_saturation"):
                    metadata["hitter_pure_training_recipe"] = "rally_final_v2"
                    metadata["hitter_pure_training_recipe_version"] = "2"
                    metadata["hitter_pure_ready_hold_steps_range"] = ",".join(
                        str(int(v)) for v in motion_cmd.cfg.hold_steps_range
                    )
                elif hasattr(rewards_cfg, "rally_heading_debt"):
                    metadata["hitter_pure_training_recipe"] = "rally_final_v1"
                    metadata["hitter_pure_training_recipe_version"] = "1"
                else:
                    metadata["hitter_pure_training_recipe"] = "legacy_station_step"
                    metadata["hitter_pure_training_recipe_version"] = "0"
    except (KeyError, ValueError):
        pass  # task without a racket_target command (plain tracking) — clock keys not needed
    actor_contract = infer_actor_observation_contract(env)
    if actor_contract is not None:
        metadata.update(
            {
                "actor_obs_contract": actor_contract.name,
                "actor_obs_mode": actor_contract.obs_mode,
                "actor_obs_total_dim": actor_contract.total_dim,
                "actor_obs_term_dims": [term.dim for term in actor_contract.terms],
                "actor_obs_term_sources_json": json.dumps(
                    {term.name: term.deploy_source for term in actor_contract.terms},
                    separators=(",", ":"),
                ),
                "actor_obs_removed_vs_full_json": json.dumps(
                    [term.name for term in removed_terms_vs_full(actor_contract)],
                    separators=(",", ":"),
                ),
            }
        )

    _v17_p0_version = metadata.get("hitter_pure_training_recipe_version")
    if (
        metadata.get("hitter_pure_training_recipe") == "rally_v17"
        and _v17_p0_version in ("6", "10")
    ):
        ground_contract = getattr(env.cfg, "v17_ground_plant_contract", None)
        if ground_contract is None:
            raise RuntimeError(
                f"Refusing RallyV17-r{_v17_p0_version} P0 export without the "
                "resolved ground-plant receipt"
            )
        if "hitter_pure_checkpoint_sha256" not in metadata:
            raise RuntimeError(
                f"Refusing RallyV17-r{_v17_p0_version} P0 export without checkpoint_path; the "
                "contract candidate must be bound to exact checkpoint bytes"
            )
        physics_dt_s = float(env.cfg.sim.dt)
        control_decimation = int(env.cfg.decimation)
        metadata.update(
            {
                "a3_control_physics_dt_s": f"{physics_dt_s:.17g}",
                "a3_control_decimation": str(control_decimation),
                "a3_control_policy_dt_s": (
                    f"{physics_dt_s * control_decimation:.17g}"
                ),
                "v17_ground_plant_contract_json": canonical_json(
                    ground_contract
                ),
            }
        )
        qdes_parity_content = build_v11_qdes_parity_csv(metadata)
        metadata["a3_qdes_parity_csv_sha256"] = qdes_parity_csv_sha256(
            qdes_parity_content
        )
        attach_manifest = (
            attach_v17_r10_p0_manifest_metadata
            if _v17_p0_version == "10"
            else attach_v17_r6_p0_manifest_metadata
        )
        (
            deploy_manifest,
            _deploy_manifest_json,
            deploy_manifest_sha256,
        ) = attach_manifest(metadata)

    model = onnx.load(onnx_path)
    # Preserve unrelated exporter metadata while replacing every contract key
    # exactly once.  Duplicate ONNX custom keys are parser-dependent and would
    # make Python/C++ consumers disagree about which value is authoritative.
    merged_metadata = {entry.key: entry.value for entry in model.metadata_props}
    merged_metadata.update(
        {key: metadata_wire_value(value) for key, value in metadata.items()}
    )
    del model.metadata_props[:]
    for k, v in merged_metadata.items():
        entry = onnx.StringStringEntryProto()
        entry.key = k
        entry.value = v
        model.metadata_props.append(entry)

    onnx.save(model, onnx_path)
    if deploy_manifest is not None:
        parity_sidecar = write_qdes_parity_sidecar(
            onnx_path,
            qdes_parity_content,
            metadata["a3_qdes_parity_csv_sha256"],
        )
        sidecar = write_deploy_manifest_sidecar(
            onnx_path, deploy_manifest, deploy_manifest_sha256
        )
        print(
            f"[exporter] V17-r{_v17_p0_version} P0 deploy manifest written: "
            f"{sidecar} (hardware_authorized=false, "
            f"sha256={deploy_manifest_sha256})",
            flush=True,
        )
        print(
            f"[exporter] V17-r{_v17_p0_version} q_des parity vectors written: "
            f"{parity_sidecar} "
            f"(sha256={metadata['a3_qdes_parity_csv_sha256']})",
            flush=True,
        )
