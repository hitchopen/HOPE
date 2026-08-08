"""Fail-closed upload bounds for HOPE training runs.

W&B's scalar API creates one history key per tag.  The detailed stability and
strike matrices belong in focused offline evaluation, not thousands of
independent remote time series that make the project UI slow and create
multi-GB local W&B streams.  This module deliberately has no torch or wandb
import so the policy can be tested on a host without either training
dependency.
"""

from __future__ import annotations


_CORE_INSTRUMENTATION_PREFIXES = (
    "Instrumentation/qdes_safety/",
    "Instrumentation/chain/all/",
    "Instrumentation/chain/fh/all/",
    "Instrumentation/chain/bh/all/",
    "Instrumentation/one_step/all/",
)

_HITTER_STANDARD_PREFIXES = (
    "Train/",
    "Loss/",
    "Perf/",
    "Episode_Termination/",
)

_HITTER_EPISODE_REWARDS = {
    "motion_global_anchor_ori",
    "motion_body_pos",
    "motion_body_ori",
    "motion_body_lin_vel",
    "motion_body_ang_vel",
    "racket_position",
    "racket_velocity",
    "racket_normal",
    "action_rate_l2",
    "joint_acc",
    "joint_torques",
    "joint_limit",
    "rally_joint_qdes_saturation",
    "post_strike_leg_quiet",
    "post_swing_xlock",
    "post_swing_base_quiet",
    "post_swing_leg_quiet",
    "settle_foot_slip",
    "rally_ready_root_height_debt",
    "post_swing_tilt_debt",
}

_HITTER_LIVE_EXACT = {
    "Live/Env/episode_length",
    "Live/Env/common_step_counter",
    "Live/Reward/total",
    "Live/Action/mean_abs",
    "Live/Action/max_abs",
    "Live/Action/delta_mean_abs",
    "Live/Action/delta_max_abs",
    "Live/Policy/physical_noise_std_parameter_mean",
    "Live/Policy/effective_noise_std_mean",
    "Live/Policy/executed_entropy_mean",
    "Live/Termination/done_rate",
    "Live/Termination/terminated_rate",
    "Live/Termination/timeout_rate",
    "Live/Termination/base_fell_tilt",
    "Live/Termination/base_too_low",
    "Live/Termination/base_mocap_stale",
}

_HITTER_MOTION_METRICS = {
    "in_hold",
    "motion_phase",
    "post_swing_capture_enabled",
    "post_swing_replay_probability_effective",
    "post_swing_replay_buffer_fill",
    "post_swing_replay_ready",
    "post_swing_replay_reset",
}

_HITTER_RACKET_METRICS = {
    # One-way admission and full-pose localization.
    "ability_gate_condition",
    "ability_gate_enough_samples",
    "ability_gate_completion_fh",
    "ability_gate_completion_bh",
    "ability_gate_position_fh",
    "ability_gate_position_bh",
    "ability_gate_composite",
    "ability_gate_post_fall",
    "ability_gate_dwell",
    "ability_unlocked",
    "base_mocap_robustness_scale",
    "base_mocap_age_s",
    "base_mocap_stale",
    "base_mocap_delay_steps_in_effect",
    "base_mocap_orientation_error_rad",
    "base_mocap_velocity_error",
    # Fixed startup plant cohort, republished after resets.
    "pd_nominal_env",
    "pd_kp_multiplier_mean",
    "pd_kd_message_multiplier_mean",
    # Executed affine q_des by task phase; these are diagnostics, not a filter.
    "qdes_step_rms_rad_hold",
    "qdes_step_rms_rad_strike",
    "qdes_step_rms_rad_recovery",
    "qdes_second_difference_rms_rad_hold",
    "qdes_second_difference_rms_rad_strike",
    "qdes_second_difference_rms_rad_recovery",
    "qdes_reversal_hz_hold",
    "qdes_reversal_hz_strike",
    "qdes_reversal_hz_recovery",
    # End-to-end strike and fall health.
    "swing_completion_rate",
    "swing_completion_rate_forehand",
    "swing_completion_rate_backhand",
    "pre_strike_fall_rate",
    "post_strike_fall_rate",
    "post_strike_fall_rate_forehand",
    "post_strike_fall_rate_backhand",
    "strike_composite_success_exact",
    "strike_composite_success_exact_forehand",
    "strike_composite_success_exact_backhand",
    "strike_pos_pass_exact",
    "strike_pos_pass_exact_forehand",
    "strike_pos_pass_exact_backhand",
    "strike_vel_pass_exact",
    "strike_normal_pass_exact",
    "exact_strike_sample_count_decayed",
    # Recovery and virtual-ball outcome summaries. Rigid physical-ball truth is
    # restricted to the low-env offline audit and never enters training W&B.
    "post_swing_base_speed",
    "post_swing_base_tilt_deg",
    "post_swing_foot_slip",
    "post_swing_leg_speed",
    "base_drift_per_swing",
    "virtual_hit_rate",
    "virtual_net_clear_rate",
    "virtual_land_inbounds_rate",
    "virtual_sample_count_forehand",
    "virtual_sample_count_backhand",
    "virtual_contact_rate_forehand",
    "virtual_contact_rate_backhand",
    "virtual_over_net_rate_forehand",
    "virtual_over_net_rate_backhand",
    "virtual_legal_rate_forehand",
    "virtual_legal_rate_backhand",
    "virtual_landing_error_m_forehand",
    "virtual_landing_error_m_backhand",
}

_HITTER_QDES_INSTRUMENTATION = {
    "qdes_isfinite",
    "q_isfinite",
    "numeric_or_qdes_safety_fault_latched",
    "physical_q_hard_fault_latched",
    "actual_q_audit_sample_count",
    "actual_q_hard_exceedance_count",
    "actual_q_hard_exceedance_env_count",
    "actual_q_hard_exceedance_rate",
    "actual_q_hard_excess_max_rad",
    "qdes_clamp_fraction",
    "qdes_step_rms_rad",
    "qdes_second_difference_rms_rad",
    "qdes_reversal_fraction",
    "qdes_delay_mean_steps",
    "qdes_delay_zero_fraction",
}


def wandb_metric_allowed(tag: str, profile: str = "core_v1") -> bool:
    """Return whether ``tag`` belongs in the remote W&B scalar history."""

    normalized_profile = str(profile).strip().lower()
    if normalized_profile == "all":
        return True
    if normalized_profile not in {"core_v1", "hitter_pingpong_v1"}:
        raise ValueError(
            f"Unknown W&B metric profile {profile!r}; expected "
            "'core_v1', 'hitter_pingpong_v1', or 'all'"
        )

    tag = str(tag)
    if normalized_profile == "hitter_pingpong_v1":
        if tag.startswith(_HITTER_STANDARD_PREFIXES):
            return True
        if tag == "Policy/mean_noise_std":
            return True
        reward_prefix = "Episode_Reward/"
        if tag.startswith(reward_prefix):
            return tag[len(reward_prefix) :] in _HITTER_EPISODE_REWARDS
        if tag in _HITTER_LIVE_EXACT:
            return True
        motion_prefix = "Live/motion/"
        if tag.startswith(motion_prefix):
            return tag[len(motion_prefix) :] in _HITTER_MOTION_METRICS
        racket_prefix = "Live/racket_target/"
        if tag.startswith(racket_prefix):
            return tag[len(racket_prefix) :] in _HITTER_RACKET_METRICS
        qdes_prefix = "Instrumentation/qdes_safety/"
        if tag.startswith(qdes_prefix):
            return tag[len(qdes_prefix) :] in _HITTER_QDES_INSTRUMENTATION
        return False
    if not tag.startswith("Instrumentation/"):
        return True
    return tag.startswith(_CORE_INSTRUMENTATION_PREFIXES)


class WandbScalarUploadLimiter:
    """Proxy a W&B summary writer while bounding its distinct scalar keys.

    Dropped metrics never reach the underlying writer, which prevents both the
    remote history schema and the duplicate TensorBoard event stream produced
    by rsl_rl's W&B writer from growing with the full matrix cardinality.
    """

    def __init__(
        self,
        writer,
        *,
        profile: str = "core_v1",
        max_distinct_keys: int = 2000,
    ) -> None:
        # Validate the profile at construction instead of waiting for the first
        # scalar after an expensive simulator launch.
        wandb_metric_allowed("_policy_validation", profile)
        max_distinct_keys = int(max_distinct_keys)
        if max_distinct_keys < 0:
            raise ValueError("W&B max_distinct_keys must be >= 0")

        self._writer = writer
        self.profile = str(profile).strip().lower()
        self.max_distinct_keys = max_distinct_keys
        self.accepted_tags: set[str] = set()
        self.dropped_tags: set[str] = set()
        self._drop_reported = False

    def __getattr__(self, name):
        return getattr(self._writer, name)

    def add_scalar(self, tag: str, scalar_value, *args, **kwargs) -> None:
        tag = str(tag)
        allowed = wandb_metric_allowed(tag, self.profile)
        within_cap = (
            tag in self.accepted_tags
            or self.max_distinct_keys == 0
            or len(self.accepted_tags) < self.max_distinct_keys
        )
        if allowed and within_cap:
            self.accepted_tags.add(tag)
            self._writer.add_scalar(tag, scalar_value, *args, **kwargs)
            return

        self.dropped_tags.add(tag)
        if not self._drop_reported:
            reason = (
                "detailed instrumentation profile"
                if not allowed
                else f"{self.max_distinct_keys}-key cap"
            )
            print(
                "[W&B upload policy] suppressing non-core scalar series "
                f"({reason}); core training curves remain enabled",
                flush=True,
            )
            self._drop_reported = True
