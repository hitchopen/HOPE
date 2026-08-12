import hashlib
import json
import math
import os
import pathlib
import random
import signal
import time
from collections import deque
from dataclasses import asdict

import numpy as np
import torch
from rsl_rl.env import VecEnv
from rsl_rl.runners.on_policy_runner import OnPolicyRunner
from tensordict import TensorDict

from isaaclab_rl.rsl_rl import export_policy_as_onnx

from whole_body_tracking.utils.exporter import attach_onnx_metadata, export_motion_policy_as_onnx
from whole_body_tracking.utils.wandb_logging import WandbScalarUploadLimiter


class MyOnPolicyRunner(OnPolicyRunner):
    def save(self, path: str, infos=None):
        """Save the model and training information."""
        super().save(path, infos)
        if self.logger_type in ["wandb"]:
            import wandb

            policy_path = path.split("model")[0]
            filename = policy_path.split("/")[-2] + ".onnx"
            export_policy_as_onnx(
                self.alg.policy,
                normalizer=getattr(self.alg.policy, "actor_obs_normalizer", None),
                path=policy_path,
                filename=filename,
            )
            attach_onnx_metadata(self.env.unwrapped, wandb.run.name, path=policy_path, filename=filename)
            wandb.save(policy_path + filename, base_path=os.path.dirname(policy_path))


class MotionOnPolicyRunner(OnPolicyRunner):
    def __init__(self, env: VecEnv, train_cfg: dict, log_dir: str | None = None, device="cpu", registry_name=None):
        # The resolved action term is authoritative for which fixed actor outputs execute.  Bind
        # its passive columns before rsl_rl constructs the policy, so entropy optimizes the 29
        # active A3 channels while preserving the 31-D network/export shape.
        policy_cfg = train_cfg.get("policy", {})
        if policy_cfg.get("class_name") == "BoundedStdActorCritic":
            action_term = env.unwrapped.action_manager.get_term("joint_pos")
            policy_cfg["entropy_action_transform"] = (
                "tanh"
                if getattr(action_term, "_action_contract", None) == "feasible_qdes_v3"
                else "identity"
            )
            passive_cols = getattr(action_term, "_passive_action_cols", None)
            if passive_cols is not None:
                policy_cfg["entropy_excluded_action_indices"] = [
                    int(index) for index in passive_cols.detach().cpu().tolist()
                ]
        super().__init__(env, train_cfg, log_dir, device)
        bind_projection = getattr(
            self.alg, "bind_executable_action_projection", None
        )
        if callable(bind_projection):
            action_term = env.unwrapped.action_manager.get_term("joint_pos")
            required = (
                "_decoder_parameter_rows",
                "_safe_lo",
                "_safe_hi",
                "_offset",
                "_scale",
                "_passive_action_cols",
            )
            missing = [name for name in required if not hasattr(action_term, name)]
            if missing:
                raise RuntimeError(
                    "ProjectionPPO requires the V11 affine-clamp action term; "
                    f"missing={missing}"
                )
            env_ids = torch.arange(
                int(env.num_envs), device=action_term.device
            )
            offset = action_term._decoder_parameter_rows(
                action_term._offset, env_ids, "offset"
            )
            scale = action_term._decoder_parameter_rows(
                action_term._scale, env_ids, "scale"
            )
            if bool((torch.abs(scale) <= 1.0e-12).any()):
                raise RuntimeError(
                    "ProjectionPPO cannot invert a zero V11 action scale"
                )
            bound_a = (action_term._safe_lo - offset) / scale
            bound_b = (action_term._safe_hi - offset) / scale
            lower = torch.minimum(bound_a, bound_b)
            upper = torch.maximum(bound_a, bound_b)
            active = torch.ones(
                int(env.num_actions),
                dtype=torch.bool,
                device=action_term.device,
            )
            passive_cols = action_term._passive_action_cols
            if passive_cols.numel() > 0:
                active[passive_cols] = False
            bind_projection(lower, upper, active)
        self.registry_name = registry_name

    def _prepare_logging_writer(self) -> None:
        """Create a bounded W&B writer and preserve exact-resume configuration."""
        logger_type = str(self.cfg.get("logger", "tensorboard")).lower()
        exact_wandb_resume = (
            bool(getattr(self, "checkpoint_exact_resume", False))
            and logger_type == "wandb"
        )
        if logger_type != "wandb":
            super()._prepare_logging_writer()
            return
        if self.log_dir is None or self.writer is not None or self.disable_logs:
            return

        # W&B otherwise captures the full Isaac/PPO terminal stream into output.log and uploads it
        # on finish.  Training diagnostics remain visible in the terminal and local launch log;
        # remote runs receive scalar curves and small provenance files only.
        if not bool(getattr(self, "wandb_capture_console", False)):
            os.environ["WANDB_CONSOLE"] = "off"

        from rsl_rl.utils.wandb_utils import WandbSummaryWriter

        self.logger_type = "wandb"
        raw_writer = WandbSummaryWriter(log_dir=self.log_dir, flush_secs=10, cfg=self.cfg)
        self.writer = WandbScalarUploadLimiter(
            raw_writer,
            profile=str(getattr(self, "wandb_metric_profile", "core_v1")),
            max_distinct_keys=int(getattr(self, "wandb_max_metric_keys", 2000)),
        )

        if not exact_wandb_resume:
            self.writer.log_config(
                self.env.cfg,
                self.cfg,
                self.alg_cfg,
                self.policy_cfg,
            )
            print(
                "[MotionOnPolicyRunner] bounded W&B logging active: "
                f"profile={self.writer.profile}, "
                f"max_distinct_keys={self.writer.max_distinct_keys}, "
                "console_capture=off",
                flush=True,
            )
            return

        # rsl_rl writes the resolved config again whenever its W&B writer is constructed. On a
        # resumed run W&B treats that config as immutable. Its first serialization can omit empty
        # nested dictionaries (for example actor_obs_normalization={}), so writing the same saved
        # runner config again may look like a value change and abort before the first PPO update.
        # Exact resume deliberately reconnects to the same run and frozen params, therefore allow
        # those serialization-only differences here. Fresh and ordinary warm-start runs retain the
        # upstream fail-closed behavior.
        import wandb

        try:
            env_cfg = self.env.cfg.to_dict()
        except Exception:
            env_cfg = asdict(self.env.cfg)
        for key, value in (
            ("runner_cfg", self.cfg),
            ("policy_cfg", self.policy_cfg),
            ("alg_cfg", self.alg_cfg),
            ("env_cfg", env_cfg),
        ):
            wandb.config.update({key: value}, allow_val_change=True)
        print(
            "[MotionOnPolicyRunner] exact W&B resume attached; saved configs restored with "
            "allow_val_change=True; bounded scalar/file upload remains active",
            flush=True,
        )

    @staticmethod
    def _is_scalar_tree(value) -> bool:
        """Return whether a value is cheap, device-independent command-manager state."""
        if value is None or isinstance(value, (bool, int, float, str)):
            return True
        if isinstance(value, (list, tuple)):
            return all(MotionOnPolicyRunner._is_scalar_tree(item) for item in value)
        if isinstance(value, dict):
            return all(
                MotionOnPolicyRunner._is_scalar_tree(key)
                and MotionOnPolicyRunner._is_scalar_tree(item)
                for key, item in value.items()
            )
        return False

    def _capture_legacy_environment_resume_state(self) -> dict:
        """Capture the schema-v2 distribution state used by legacy checkpoints."""
        env = self.env.unwrapped
        state = {
            "schema_version": 1,
            "common_step_counter": int(getattr(env, "common_step_counter", 0)),
            "command_terms": {},
        }
        manager = getattr(env, "command_manager", None)
        if manager is None:
            return state

        for term_name in getattr(manager, "active_terms", ()):
            term = manager.get_term(term_name)
            term_state = {
                "scalars": {},
                "tensors": {},
                "tensor_dicts": {},
            }

            # Preserve scalar EMA/curriculum drivers. These control target-distribution progress
            # and the long-horizon success/fall statistics, but are not part of the policy state.
            for attr, value in vars(term).items():
                is_resume_scalar = (
                    attr == "_curr_perturb_scale"
                    # Reversible strike-velocity curriculum: preserve the live interpolation
                    # progress and consecutive healthy-gate dwell across an exact resume. Keep the
                    # legacy latch field for checkpoints written before the reversible curriculum.
                    or attr in {
                        "_vel_weight_latched",
                        "_vel_weight_progress",
                        "_vel_weight_gate_dwell",
                        "_velocity_stage",
                        "_velocity_current_weight",
                        "_velocity_target_weight",
                        "_velocity_stage1_enter_dwell",
                        "_velocity_stage1_exit_dwell",
                        "_velocity_stage2_enter_dwell",
                        "_velocity_stage2_exit_dwell",
                        "_recovery_stage",
                        "_recovery_current_scale",
                        "_recovery_target_scale",
                        "_recovery_ramp_rate_per_step",
                        "_recovery_coverage_scale",
                        "_recovery_coverage_target_scale",
                        "_recovery_coverage_ramp_rate_per_step",
                        "_recovery_stage1_coverage_unlocked",
                        "_recovery_stage1_acquisition_rung",
                        "_recovery_stage1_acquisition_failures",
                        "_recovery_stage1_ready_dwell",
                        "_recovery_stage1_acquisition_age",
                        "_recovery_stage1_enter_dwell",
                        "_recovery_stage1_exit_dwell",
                        "_recovery_stage2_enter_dwell",
                        "_recovery_stage2_exit_dwell",
                        "_recovery_curriculum_scale",
                        "_last_target_robustness_scale",
                        "_ability_unlocked",
                        "_ability_gate_dwell",
                        "_ability_unlock_step",
                        "_base_mocap_robustness_scale",
                        "_post_swing_ability_unlocked",
                        "_post_swing_replay_ready_step",
                    }
                    or attr.startswith("_adaptive_sigma_")
                    or attr.endswith("_acc")
                    or attr.endswith("_acc_c")
                    # 2026-07-24 audit: the adaptive-sigma DRIVERS (_exact_pos_err_sum /
                    # _exact_vel_err_sum, _exact_normal_dot_sum_c, per-clip dicts) were not
                    # captured, so every exact resume restarted the error EMAs at 0 and
                    # whipsawed the sigma schedule.  _is_scalar_tree still gates the value.
                    or attr.endswith("_sum")
                    or attr.endswith("_sum_c")
                )
                if is_resume_scalar and self._is_scalar_tree(value):
                    term_state["scalars"][attr] = value

            # RallyV14 samples 35% of true resets from a ring of completed follow-through states.
            # Keeping that ring avoids silently dropping the recovery population after a restart.
            for attr in (
                "bin_failed_count",
                "_current_bin_failed",
                "_post_swing_root",
                "_post_swing_joint_pos",
                "_post_swing_joint_vel",
                # RallyV17 Markov-stratified replay: physical state plus every action-history
                # tensor that affects the actor observation/action-rate transition.
                "_v17_replay_root",
                "_v17_replay_joint_pos",
                "_v17_replay_joint_vel",
                "_v17_replay_manager_action",
                "_v17_replay_manager_prev_action",
                "_v17_replay_action_raw",
                "_v17_replay_action_applied_raw",
                "_v17_replay_action_unclamped_qdes",
                "_v17_replay_action_processed_qdes",
                "_v17_replay_action_executed_qdes",
                "_v17_replay_action_previous_executed_qdes",
                "_v17_replay_action_qdes_delta",
                "_v17_replay_action_previous_qdes_delta",
                "_v17_replay_action_qdes_second_difference",
                "_v17_replay_action_decoder_offset",
                "_v17_replay_action_decoder_scale",
                "_v17_replay_source_env",
                "_v17_replay_local_latest_slot",
                "_v17_replay_joint_stiffness",
                "_v17_replay_joint_damping",
                "_v17_replay_motion_time_steps",
                "_v17_replay_motion_clip_id",
                "_v17_replay_motion_hold_counter",
                "_v17_replay_bucket_count",
                "_v17_replay_bucket_ptr",
                "_v17_replay_failure_score",
                "_v17_replay_pending_failure_count",
                "_v17_replay_sampling_probability",
                # RallyV17-r3 finite-window actual-q admission proof. Restoring only the
                # lifetime counters would silently change the exact-zero Stage-2 gate.
                "_actual_q_window_faults",
                "_actual_q_window_starts",
                "_actual_q_window_pending_faults",
                "_actual_q_window_pending_starts",
                # V15 HUGWBC adaptive intervention curriculum: without these, exact resume
                # silently restarted every env at intervention_curriculum_start (2026-07-23 audit).
                "_intervention_strength",
                "_locomotion_tracking_sum",
                "_locomotion_tracking_count",
                # Training-only strike/q_des instrumentation accumulators.  Exact resume keeps
                # their cumulative denominators continuous, but they never feed the task.
                "_strike_audit_count",
                "_strike_audit_pos_abs_sum",
                "_strike_audit_pos_signed_sum",
                "_strike_audit_base_abs_sum",
                "_strike_audit_base_signed_sum",
                "_strike_audit_pass_sum",
                "_strike_audit_start_count",
                "_strike_audit_postfall_count",
                "_position_guidance_prev_active",
                "_position_guidance_event_acc",
                "_exact_position_debt_prev_active",
                "_exact_position_debt_event_acc",
                "_strike_local_position_event_count",
                "_strike_local_position_event_sum",
                "_strike_local_exact_debt_event_count",
                "_strike_local_exact_debt_event_sum",
                "_qdes_joint_audit_count",
                "_qdes_joint_audit_sum",
                "_qdes_joint_audit_max",
                "_qdes_joint_tightest_count",
            ):
                value = getattr(term, attr, None)
                if torch.is_tensor(value):
                    term_state["tensors"][attr] = value.detach().cpu().clone()
            for attr in ("_v17_replay_target_state",):
                value = getattr(term, attr, None)
                if isinstance(value, dict) and all(
                    isinstance(name, str) and torch.is_tensor(tensor)
                    for name, tensor in value.items()
                ):
                    term_state["tensor_dicts"][attr] = {
                        name: tensor.detach().cpu().clone()
                        for name, tensor in value.items()
                    }
            for attr in (
                "_post_swing_ptr",
                "_post_swing_count",
                "_v17_replay_bucket_capacity",
                "_v17_replay_total_count",
                "_actual_q_window_ptr",
                "_actual_q_window_filled_steps",
                "_ability_gate_dwell",
                "_ability_unlock_step",
                "_post_swing_replay_ready_step",
            ):
                value = getattr(term, attr, None)
                if isinstance(value, int):
                    term_state["scalars"][attr] = value

            if (
                term_state["scalars"]
                or term_state["tensors"]
                or term_state["tensor_dicts"]
            ):
                state["command_terms"][str(term_name)] = term_state
        return state

    def _restore_legacy_environment_resume_state(
        self, resume_state: dict
    ) -> tuple[int, str]:
        """Restore the distribution-only state written by schema-v1/v2 checkpoints."""
        env = self.env.unwrapped
        saved = resume_state.get("environment_resume_state")
        if isinstance(saved, dict) and "common_step_counter" in saved:
            common_step_counter = int(saved["common_step_counter"])
            source = "checkpoint"
        else:
            # Checkpoints written before environment state was added still have an exact next PPO
            # iteration. Each completed PPO iteration advances every env by num_steps_per_env control
            # steps, so this reconstructs the schedule counter exactly for the existing V14 run.
            common_step_counter = int(resume_state["next_learning_iteration"]) * int(
                self.num_steps_per_env
            )
            source = "derived-from-iteration"
        env.common_step_counter = common_step_counter

        manager = getattr(env, "command_manager", None)
        restored_terms = []
        if isinstance(saved, dict) and manager is not None:
            for term_name, term_state in saved.get("command_terms", {}).items():
                try:
                    term = manager.get_term(term_name)
                except ValueError:
                    continue
                restored = False
                for attr, value in term_state.get("scalars", {}).items():
                    if hasattr(term, attr):
                        setattr(term, attr, value)
                        restored = True
                for attr, value in term_state.get("tensors", {}).items():
                    if hasattr(term, attr) and torch.is_tensor(value):
                        current = getattr(term, attr)
                        device = current.device if torch.is_tensor(current) else term.device
                        setattr(term, attr, value.to(device=device))
                        restored = True
                for attr, mapping in term_state.get(
                    "tensor_dicts", {}
                ).items():
                    if hasattr(term, attr) and isinstance(mapping, dict):
                        setattr(
                            term,
                            attr,
                            {
                                name: value.to(device=term.device)
                                for name, value in mapping.items()
                                if torch.is_tensor(value)
                            },
                        )
                        restored = True
                if restored:
                    restored_terms.append(str(term_name))
                # 2026-07-24 audit: restored _adaptive_sigma_* values only live inside the
                # command term — the reward-term params they mirror stay at YAML maxima until
                # the next sigma_update_every boundary AND a full stats window.  Re-apply them
                # immediately so the first post-resume rollouts score with the same kernels the
                # checkpoint trained on. (The velocity curriculum weight self-heals from restored
                # progress on its next compute.)
                if (
                    getattr(term, "cfg", None) is not None
                    and bool(getattr(term.cfg, "adaptive_sigma", False))
                    and hasattr(term, "_adaptive_sigma_pos")
                ):
                    try:
                        rm = env.reward_manager
                        rm.get_term_cfg("racket_position").params["std"] = float(
                            term._adaptive_sigma_pos
                        )
                        rm.get_term_cfg("racket_velocity").params["std"] = float(
                            term._adaptive_sigma_vel
                        )
                        succ = rm.get_term_cfg("racket_strike_success").params
                        succ["std_pos"] = float(term._adaptive_sigma_pos)
                        succ["std_vel"] = float(term._adaptive_sigma_vel)
                    except (ValueError, AttributeError):
                        pass  # variant task without these terms
        print(
            "[MotionOnPolicyRunner] exact environment progress restored: "
            f"common_step_counter={common_step_counter} ({source}), "
            f"command_terms={restored_terms}",
            flush=True,
        )
        return common_step_counter, source

    @staticmethod
    def _clone_resume_tree(value):
        """Clone a tensor/scalar container to CPU, returning ``(supported, clone)``."""
        if torch.is_tensor(value):
            return True, value.detach().cpu().clone()
        if value is None or isinstance(value, (bool, int, float, str)):
            return True, value
        if isinstance(value, dict):
            cloned = {}
            for key, item in value.items():
                if not isinstance(key, (bool, int, float, str)):
                    return False, None
                supported, packed = MotionOnPolicyRunner._clone_resume_tree(item)
                if not supported:
                    return False, None
                cloned[key] = packed
            return True, cloned
        if isinstance(value, list):
            cloned = []
            for item in value:
                supported, packed = MotionOnPolicyRunner._clone_resume_tree(item)
                if not supported:
                    return False, None
                cloned.append(packed)
            return True, cloned
        if isinstance(value, tuple):
            cloned = []
            for item in value:
                supported, packed = MotionOnPolicyRunner._clone_resume_tree(item)
                if not supported:
                    return False, None
                cloned.append(packed)
            return True, tuple(cloned)
        return False, None

    @staticmethod
    def _resume_tree_contains_tensor(value) -> bool:
        if torch.is_tensor(value):
            return True
        if isinstance(value, dict):
            return any(
                MotionOnPolicyRunner._resume_tree_contains_tensor(item)
                for item in value.values()
            )
        if isinstance(value, (list, tuple)):
            return any(
                MotionOnPolicyRunner._resume_tree_contains_tensor(item)
                for item in value
            )
        return False

    @staticmethod
    def _owner_label(owner) -> str:
        cls = type(owner)
        return f"{cls.__module__}.{cls.__qualname__}"

    @classmethod
    def _capture_owner_resume_state(cls, owner) -> dict:
        """Capture direct mutable tensor fields and private scalar state of one owner."""
        fields = {}
        for name, value in vars(owner).items():
            supported, packed = cls._clone_resume_tree(value)
            if not supported:
                continue
            # Every direct/nested tensor is state. Private scalar trees additionally contain
            # delay pointers, curriculum stages, dwell counters and reset edge latches.
            if cls._resume_tree_contains_tensor(packed) or name.startswith("_"):
                fields[name] = packed
        return {
            "class": cls._owner_label(owner),
            "fields": fields,
        }

    @classmethod
    def _restore_resume_tree(cls, current, saved, *, device, path: str):
        if torch.is_tensor(saved):
            value = saved.to(device=device)
            if current is None:
                return value.clone()
            if not torch.is_tensor(current):
                raise RuntimeError(
                    f"Stateful exact resume field {path} changed from tensor to "
                    f"{type(current).__name__}"
                )
            if tuple(current.shape) != tuple(value.shape) or current.dtype != value.dtype:
                raise RuntimeError(
                    f"Stateful exact resume tensor contract mismatch at {path}: "
                    f"checkpoint={tuple(value.shape)}/{value.dtype}, "
                    f"runtime={tuple(current.shape)}/{current.dtype}"
                )
            current.copy_(value)
            return current
        if isinstance(saved, dict):
            if current is None:
                current = {}
            if not isinstance(current, dict):
                raise RuntimeError(
                    f"Stateful exact resume field {path} changed from dict to "
                    f"{type(current).__name__}"
                )
            for key, value in saved.items():
                current[key] = cls._restore_resume_tree(
                    current.get(key),
                    value,
                    device=device,
                    path=f"{path}.{key}",
                )
            return current
        if isinstance(saved, list):
            if current is None:
                current = [None] * len(saved)
            if not isinstance(current, list) or len(current) != len(saved):
                raise RuntimeError(
                    f"Stateful exact resume list contract mismatch at {path}: "
                    f"checkpoint_len={len(saved)}, "
                    f"runtime_len={None if not isinstance(current, list) else len(current)}"
                )
            return [
                cls._restore_resume_tree(
                    active,
                    value,
                    device=device,
                    path=f"{path}[{index}]",
                )
                for index, (active, value) in enumerate(zip(current, saved))
            ]
        if isinstance(saved, tuple):
            if current is None:
                current = (None,) * len(saved)
            if not isinstance(current, tuple) or len(current) != len(saved):
                raise RuntimeError(
                    f"Stateful exact resume tuple contract mismatch at {path}: "
                    f"checkpoint_len={len(saved)}, "
                    f"runtime_len={None if not isinstance(current, tuple) else len(current)}"
                )
            return tuple(
                cls._restore_resume_tree(
                    active,
                    value,
                    device=device,
                    path=f"{path}[{index}]",
                )
                for index, (active, value) in enumerate(zip(current, saved))
            )
        return saved

    @classmethod
    def _restore_owner_resume_state(
        cls, owner, saved: dict, *, device, path: str
    ) -> None:
        expected_class = str(saved.get("class", ""))
        active_class = cls._owner_label(owner)
        if expected_class != active_class:
            raise RuntimeError(
                f"Stateful exact resume owner mismatch at {path}: "
                f"checkpoint={expected_class!r}, runtime={active_class!r}"
            )
        for name, value in saved.get("fields", {}).items():
            restored = cls._restore_resume_tree(
                getattr(owner, name, None),
                value,
                device=device,
                path=f"{path}.{name}",
            )
            setattr(owner, name, restored)

    @staticmethod
    def _manager_term_owners(manager) -> dict[str, object]:
        """Return stateful class terms without depending on one IsaacLab manager layout."""
        result = {}
        terms = getattr(manager, "_terms", None)
        names = list(
            getattr(manager, "active_terms", ())
            or getattr(manager, "_term_names", ())
        )
        if isinstance(terms, dict):
            iterable = terms.items()
        elif isinstance(terms, (list, tuple)):
            iterable = (
                (
                    names[index] if index < len(names) else str(index),
                    term,
                )
                for index, term in enumerate(terms)
            )
        else:
            iterable = ()
        for name, term in iterable:
            if hasattr(term, "__dict__"):
                result[str(name)] = term
        return result

    def _capture_physics_properties(self) -> dict:
        env = self.env.unwrapped
        properties = {}
        for asset_name, asset in getattr(env.scene, "articulations", {}).items():
            view = asset.root_physx_view
            articulation = {
                "material_properties": view.get_material_properties().detach().cpu().clone(),
                "masses": view.get_masses().detach().cpu().clone(),
                "inertias": view.get_inertias().detach().cpu().clone(),
                "coms": view.get_coms().detach().cpu().clone(),
                "data": {},
                "actuators": {},
            }
            for name in (
                "default_root_state",
                "default_joint_pos",
                "default_joint_vel",
                "default_joint_stiffness",
                "default_joint_damping",
                "default_mass",
                "default_inertia",
                "joint_stiffness",
                "joint_damping",
            ):
                value = getattr(asset.data, name, None)
                if torch.is_tensor(value):
                    articulation["data"][name] = value.detach().cpu().clone()
            for actuator_name, actuator in asset.actuators.items():
                articulation["actuators"][str(actuator_name)] = (
                    self._capture_owner_resume_state(actuator)
                )
            properties[str(asset_name)] = articulation
        return properties

    def _restore_physics_properties(self, saved: dict) -> None:
        env = self.env.unwrapped
        expected_assets = set(getattr(env.scene, "articulations", {}))
        if set(saved) != expected_assets:
            raise RuntimeError(
                "Stateful exact resume articulation set mismatch: "
                f"checkpoint={sorted(saved)}, runtime={sorted(expected_assets)}"
            )
        env_ids_cpu = torch.arange(int(env.num_envs), device="cpu")
        for asset_name, articulation in saved.items():
            asset = env.scene.articulations[asset_name]
            view = asset.root_physx_view
            view.set_material_properties(
                articulation["material_properties"].cpu(), env_ids_cpu
            )
            view.set_masses(articulation["masses"].cpu(), env_ids_cpu)
            view.set_inertias(articulation["inertias"].cpu(), env_ids_cpu)
            view.set_coms(articulation["coms"].cpu(), env_ids_cpu)
            for name, value in articulation.get("data", {}).items():
                current = getattr(asset.data, name, None)
                if not torch.is_tensor(current):
                    raise RuntimeError(
                        f"Stateful exact resume articulation data field disappeared: "
                        f"{asset_name}.{name}"
                    )
                restored = value.to(device=current.device)
                if tuple(restored.shape) != tuple(current.shape) or restored.dtype != current.dtype:
                    raise RuntimeError(
                        f"Stateful exact resume articulation data mismatch: "
                        f"{asset_name}.{name}"
                    )
                current.copy_(restored)
            expected_actuators = set(asset.actuators)
            if set(articulation.get("actuators", {})) != expected_actuators:
                raise RuntimeError(
                    f"Stateful exact resume actuator set mismatch for {asset_name}: "
                    f"checkpoint={sorted(articulation.get('actuators', {}))}, "
                    f"runtime={sorted(expected_actuators)}"
                )
            for actuator_name, actuator_state in articulation.get(
                "actuators", {}
            ).items():
                self._restore_owner_resume_state(
                    asset.actuators[actuator_name],
                    actuator_state,
                    device=asset.device,
                    path=f"articulation.{asset_name}.actuator.{actuator_name}",
                )
            if "joint_stiffness" in articulation.get("data", {}):
                asset.write_joint_stiffness_to_sim(asset.data.joint_stiffness)
            if "joint_damping" in articulation.get("data", {}):
                asset.write_joint_damping_to_sim(asset.data.joint_damping)

    def _capture_environment_resume_state(self) -> dict:
        """Capture the complete logical MDP boundary needed by the next rollout."""
        env = self.env.unwrapped
        scene_state = env.scene.get_state(is_relative=True)
        supported, scene_state = self._clone_resume_tree(scene_state)
        if not supported:
            raise RuntimeError("Isaac scene state is not tensor-serializable")

        env_buffers = {}
        for name in (
            "_sim_step_counter",
            "common_step_counter",
            "episode_length_buf",
            "reset_buf",
            "reset_terminated",
            "reset_time_outs",
            "reward_buf",
        ):
            if hasattr(env, name):
                supported, value = self._clone_resume_tree(getattr(env, name))
                if not supported:
                    raise RuntimeError(
                        f"Environment resume field is not serializable: {name}"
                    )
                env_buffers[name] = value

        managers = {}
        manager_names = (
            "action_manager",
            "command_manager",
            "reward_manager",
            "termination_manager",
            "curriculum_manager",
            "event_manager",
            "observation_manager",
        )
        for manager_name in manager_names:
            manager = getattr(env, manager_name, None)
            if manager is None:
                continue
            entry = {
                "owner": self._capture_owner_resume_state(manager),
                "terms": {},
            }
            if manager_name in {"action_manager", "command_manager"}:
                for term_name in getattr(manager, "active_terms", ()):
                    entry["terms"][str(term_name)] = (
                        self._capture_owner_resume_state(
                            manager.get_term(term_name)
                        )
                    )
            else:
                for term_name, term in self._manager_term_owners(manager).items():
                    entry["terms"][term_name] = (
                        self._capture_owner_resume_state(term)
                    )
            managers[manager_name] = entry

        sensors = {}
        for sensor_name, sensor in getattr(env.scene, "sensors", {}).items():
            entry = {"owner": self._capture_owner_resume_state(sensor)}
            data = getattr(sensor, "_data", None)
            if data is not None and hasattr(data, "__dict__"):
                entry["data"] = self._capture_owner_resume_state(data)
            sensors[str(sensor_name)] = entry

        obs_source = getattr(env, "obs_buf", None)
        supported, observations = self._clone_resume_tree(obs_source)
        if not supported or not isinstance(observations, dict):
            raise RuntimeError(
                "Stateful exact resume requires the last environment observation buffer"
            )
        return {
            "schema_version": 2,
            "fidelity": "stateful_mdp_boundary_v1",
            "num_envs": int(env.num_envs),
            "decimation": int(env.cfg.decimation),
            "physics_dt": float(env.physics_dt),
            "step_dt": float(env.step_dt),
            "scene_state_relative": scene_state,
            "physics_properties": self._capture_physics_properties(),
            "env_buffers": env_buffers,
            "managers": managers,
            "sensors": sensors,
            "initial_observations": observations,
        }

    def _restore_environment_resume_state(self, resume_state: dict) -> None:
        """Restore a schema-v3 logical MDP boundary without calling ``env.reset()``."""
        env = self.env.unwrapped
        saved = resume_state.get("environment_resume_state")
        if not isinstance(saved, dict) or int(saved.get("schema_version", 0)) != 2:
            raise RuntimeError(
                "Schema-v3 exact resume is missing stateful environment metadata"
            )
        contracts = {
            "num_envs": int(env.num_envs),
            "decimation": int(env.cfg.decimation),
            "physics_dt": float(env.physics_dt),
            "step_dt": float(env.step_dt),
        }
        for name, active in contracts.items():
            checkpoint = saved.get(name)
            if checkpoint != active:
                raise RuntimeError(
                    f"Stateful exact resume environment contract mismatch for {name}: "
                    f"checkpoint={checkpoint!r}, runtime={active!r}"
                )

        self._restore_physics_properties(saved["physics_properties"])
        scene_state = self._restore_resume_tree(
            None,
            saved["scene_state_relative"],
            device=env.device,
            path="scene_state_relative",
        )
        env.scene.reset_to(scene_state, is_relative=True)

        for name, value in saved.get("env_buffers", {}).items():
            restored = self._restore_resume_tree(
                getattr(env, name, None),
                value,
                device=env.device,
                path=f"environment.{name}",
            )
            setattr(env, name, restored)

        for manager_name, entry in saved.get("managers", {}).items():
            manager = getattr(env, manager_name, None)
            if manager is None:
                raise RuntimeError(
                    f"Stateful exact resume manager disappeared: {manager_name}"
                )
            self._restore_owner_resume_state(
                manager,
                entry["owner"],
                device=env.device,
                path=manager_name,
            )
            active_terms = {}
            if manager_name in {"action_manager", "command_manager"}:
                for term_name in getattr(manager, "active_terms", ()):
                    active_terms[str(term_name)] = manager.get_term(term_name)
            else:
                active_terms = self._manager_term_owners(manager)
            if set(entry.get("terms", {})) != set(active_terms):
                raise RuntimeError(
                    f"Stateful exact resume term set mismatch for {manager_name}: "
                    f"checkpoint={sorted(entry.get('terms', {}))}, "
                    f"runtime={sorted(active_terms)}"
                )
            for term_name, term_state in entry.get("terms", {}).items():
                term = active_terms[term_name]
                self._restore_owner_resume_state(
                    term,
                    term_state,
                    device=getattr(term, "device", env.device),
                    path=f"{manager_name}.{term_name}",
                )
                rebind = getattr(
                    term, "restore_exact_resume_runtime_references", None
                )
                if callable(rebind):
                    rebind()

        active_sensors = getattr(env.scene, "sensors", {})
        if set(saved.get("sensors", {})) != set(active_sensors):
            raise RuntimeError(
                "Stateful exact resume sensor set mismatch: "
                f"checkpoint={sorted(saved.get('sensors', {}))}, "
                f"runtime={sorted(active_sensors)}"
            )
        for sensor_name, entry in saved.get("sensors", {}).items():
            sensor = active_sensors[sensor_name]
            self._restore_owner_resume_state(
                sensor,
                entry["owner"],
                device=sensor.device,
                path=f"sensor.{sensor_name}",
            )
            if "data" in entry:
                self._restore_owner_resume_state(
                    sensor._data,
                    entry["data"],
                    device=sensor.device,
                    path=f"sensor.{sensor_name}.data",
                )

        observations = self._restore_resume_tree(
            None,
            saved["initial_observations"],
            device=env.device,
            path="initial_observations",
        )
        env.obs_buf = observations
        self._exact_resume_initial_observations = TensorDict(
            observations, batch_size=[int(env.num_envs)]
        )
        print(
            "[MotionOnPolicyRunner] STATEFUL exact environment boundary restored "
            f"without env.reset(): common_step_counter={env.common_step_counter}, "
            f"sim_step_counter={env._sim_step_counter}",
            flush=True,
        )

    @staticmethod
    def _capture_rng_state() -> dict:
        return {
            "python_random_state": random.getstate(),
            "numpy_random_state": np.random.get_state(),
            "torch_random_state": torch.get_rng_state(),
            "torch_cuda_random_states": (
                torch.cuda.get_rng_state_all()
                if torch.cuda.is_available()
                else []
            ),
        }

    @staticmethod
    def _restore_rng_state(state: dict) -> None:
        random.setstate(state["python_random_state"])
        np.random.set_state(state["numpy_random_state"])
        torch.set_rng_state(state["torch_random_state"])
        cuda_states = state.get("torch_cuda_random_states", [])
        if cuda_states and torch.cuda.is_available():
            if len(cuda_states) != torch.cuda.device_count():
                raise RuntimeError(
                    "Exact resume CUDA device count mismatch: "
                    f"checkpoint={len(cuda_states)} runtime={torch.cuda.device_count()}"
                )
            torch.cuda.set_rng_state_all(cuda_states)

    @staticmethod
    def _cpu_tensor(value):
        return value.detach().cpu().clone()

    def _capture_rollout_bookkeeping(self, locs: dict) -> None:
        """Persist rsl_rl's iteration-local episode/logging accumulators."""
        state = {
            "rewbuffer": list(locs.get("rewbuffer", ())),
            "lenbuffer": list(locs.get("lenbuffer", ())),
            "cur_reward_sum": self._cpu_tensor(locs["cur_reward_sum"]),
            "cur_episode_length": self._cpu_tensor(
                locs["cur_episode_length"]
            ),
        }
        if getattr(self.alg, "rnd", None):
            state.update(
                {
                    "erewbuffer": list(locs.get("erewbuffer", ())),
                    "irewbuffer": list(locs.get("irewbuffer", ())),
                    "cur_ereward_sum": self._cpu_tensor(
                        locs["cur_ereward_sum"]
                    ),
                    "cur_ireward_sum": self._cpu_tensor(
                        locs["cur_ireward_sum"]
                    ),
                }
            )
        self._rollout_bookkeeping_state = state

    def _restore_bookkeeping_tensor(
        self, state: dict | None, name: str, shape: tuple[int, ...]
    ) -> torch.Tensor:
        if state is None or name not in state:
            return torch.zeros(shape, dtype=torch.float, device=self.device)
        value = state[name]
        if not torch.is_tensor(value) or tuple(value.shape) != shape:
            raise RuntimeError(
                f"Exact resume bookkeeping tensor mismatch for {name}: "
                f"checkpoint={None if not torch.is_tensor(value) else tuple(value.shape)}, "
                f"runtime={shape}"
            )
        return value.to(device=self.device).clone()

    def _configure_transfer_actor_update(self, iteration: int) -> None:
        """Keep an audited migrated actor fixed while its fresh critic learns R9 returns."""

        freeze_until = int(
            getattr(self, "v17_actor_freeze_until_iteration", 0)
        )
        enabled = int(iteration) >= freeze_until
        if freeze_until > 0 and not hasattr(self.alg, "actor_updates_enabled"):
            # ProjectionPPO accepts this runtime switch without changing its serialized model
            # contract.  Other algorithms must opt in explicitly instead of silently ignoring a
            # requested migration guard.
            if self.alg.__class__.__name__ != "ProjectionPPO":
                raise RuntimeError(
                    "V17 critic-only migration warm-up requires ProjectionPPO"
                )
        self.alg.actor_updates_enabled = bool(enabled)
        previous = getattr(self, "_last_actor_updates_enabled", None)
        if previous is None or bool(previous) != bool(enabled):
            if enabled:
                finetune_lr = float(
                    getattr(
                        self,
                        "v17_actor_finetune_learning_rate",
                        0.0,
                    )
                )
                if freeze_until > 0:
                    if not math.isfinite(finetune_lr) or finetune_lr <= 0.0:
                        raise RuntimeError(
                            "V17 migration actor fine-tune LR must be finite and positive"
                        )
                    # Set this *before* the first actor minibatch. Letting fresh AdamW begin at
                    # 1e-3 and trusting the post-hoc KL reduction already moved every actor layer
                    # by ~2.8e-3 and destroyed the transferred behavior in one iteration.
                    self.alg.learning_rate = finetune_lr
                    self.alg.adaptive_learning_rate_max = finetune_lr
                    for param_group in self.alg.optimizer.param_groups:
                        param_group["lr"] = finetune_lr
                print(
                    "[MotionOnPolicyRunner] V17 migration actor/std updates ENABLED "
                    f"at iteration {iteration}; critic-only warm-up is complete; "
                    f"fine-tune LR/cap={finetune_lr:.1e}",
                    flush=True,
                )
            else:
                print(
                    "[MotionOnPolicyRunner] V17 migration critic-only warm-up ACTIVE: "
                    f"actor/std frozen for iterations 0..{freeze_until - 1}",
                    flush=True,
                )
            self._last_actor_updates_enabled = bool(enabled)

    def _learn_loop(
        self, num_learning_iterations: int, init_at_random_ep_len: bool
    ) -> None:
        """Current rsl_rl loop with serializable boundary-local bookkeeping."""
        from rsl_rl.utils import store_code_state

        # W&B/TensorBoard initialization is allowed to touch host RNGs. Rewind to the checkpoint
        # only after the writer is attached, immediately before the first resumed observation/action.
        self._prepare_logging_writer()
        pending_rng = getattr(self, "_pending_exact_rng_state", None)
        if pending_rng is not None:
            self._restore_rng_state(pending_rng)
            del self._pending_exact_rng_state

        stateful_resume = bool(
            getattr(self, "_stateful_exact_resume_active", False)
        )
        if stateful_resume and init_at_random_ep_len:
            print(
                "[MotionOnPolicyRunner] stateful exact resume preserves checkpointed "
                "episode_length_buf; init_at_random_ep_len ignored",
                flush=True,
            )
            init_at_random_ep_len = False
        if init_at_random_ep_len:
            self.env.episode_length_buf = torch.randint_like(
                self.env.episode_length_buf,
                high=int(self.env.max_episode_length),
            )

        initial_observations = getattr(
            self, "_exact_resume_initial_observations", None
        )
        if initial_observations is not None:
            obs = initial_observations.to(self.device)
            del self._exact_resume_initial_observations
        else:
            obs = self.env.get_observations().to(self.device)
        self.train_mode()

        restored = getattr(self, "_restored_rollout_bookkeeping", None)
        if hasattr(self, "_restored_rollout_bookkeeping"):
            del self._restored_rollout_bookkeeping
        ep_infos = []
        rewbuffer = deque(
            () if restored is None else restored.get("rewbuffer", ()),
            maxlen=100,
        )
        lenbuffer = deque(
            () if restored is None else restored.get("lenbuffer", ()),
            maxlen=100,
        )
        vector_shape = (int(self.env.num_envs),)
        cur_reward_sum = self._restore_bookkeeping_tensor(
            restored, "cur_reward_sum", vector_shape
        )
        cur_episode_length = self._restore_bookkeeping_tensor(
            restored, "cur_episode_length", vector_shape
        )

        if self.alg.rnd:
            erewbuffer = deque(
                () if restored is None else restored.get("erewbuffer", ()),
                maxlen=100,
            )
            irewbuffer = deque(
                () if restored is None else restored.get("irewbuffer", ()),
                maxlen=100,
            )
            cur_ereward_sum = self._restore_bookkeeping_tensor(
                restored, "cur_ereward_sum", vector_shape
            )
            cur_ireward_sum = self._restore_bookkeeping_tensor(
                restored, "cur_ireward_sum", vector_shape
            )

        if self.is_distributed:
            print(
                f"Synchronizing parameters for rank {self.gpu_global_rank}..."
            )
            self.alg.broadcast_parameters()

        start_iter = self.current_learning_iteration
        tot_iter = start_iter + num_learning_iterations
        for it in range(start_iter, tot_iter):
            self._configure_transfer_actor_update(it)
            start = time.time()
            with torch.inference_mode():
                for _ in range(self.num_steps_per_env):
                    actions = self.alg.act(obs)
                    obs, rewards, dones, extras = self.env.step(
                        actions.to(self.env.device)
                    )
                    obs, rewards, dones = (
                        obs.to(self.device),
                        rewards.to(self.device),
                        dones.to(self.device),
                    )
                    self.alg.process_env_step(obs, rewards, dones, extras)
                    intrinsic_rewards = (
                        self.alg.intrinsic_rewards if self.alg.rnd else None
                    )
                    if self.log_dir is not None:
                        if "episode" in extras:
                            ep_infos.append(extras["episode"])
                        elif "log" in extras:
                            ep_infos.append(extras["log"])
                        if self.alg.rnd:
                            cur_ereward_sum += rewards
                            cur_ireward_sum += intrinsic_rewards
                            cur_reward_sum += rewards + intrinsic_rewards
                        else:
                            cur_reward_sum += rewards
                        cur_episode_length += 1
                        new_ids = (dones > 0).nonzero(as_tuple=False)
                        rewbuffer.extend(
                            cur_reward_sum[new_ids][:, 0]
                            .cpu()
                            .numpy()
                            .tolist()
                        )
                        lenbuffer.extend(
                            cur_episode_length[new_ids][:, 0]
                            .cpu()
                            .numpy()
                            .tolist()
                        )
                        cur_reward_sum[new_ids] = 0
                        cur_episode_length[new_ids] = 0
                        if self.alg.rnd:
                            erewbuffer.extend(
                                cur_ereward_sum[new_ids][:, 0]
                                .cpu()
                                .numpy()
                                .tolist()
                            )
                            irewbuffer.extend(
                                cur_ireward_sum[new_ids][:, 0]
                                .cpu()
                                .numpy()
                                .tolist()
                            )
                            cur_ereward_sum[new_ids] = 0
                            cur_ireward_sum[new_ids] = 0

                stop = time.time()
                collection_time = stop - start
                start = stop
                self.alg.compute_returns(obs)

            loss_dict = self.alg.update()
            stop = time.time()
            learn_time = stop - start
            self.current_learning_iteration = it

            if self.log_dir is not None and not self.disable_logs:
                self.log(locals())
                if it % self.save_interval == 0:
                    self.save(
                        os.path.join(self.log_dir, f"model_{it}.pt")
                    )

            ep_infos.clear()
            if (
                it == start_iter
                and not self.disable_logs
                and not bool(getattr(self, "checkpoint_exact_resume", False))
            ):
                git_file_paths = store_code_state(
                    self.log_dir, self.git_status_repos
                )
                if (
                    self.logger_type in ["wandb", "neptune"]
                    and git_file_paths
                ):
                    for path in git_file_paths:
                        self.writer.save_file(path)

        if self.log_dir is not None and not self.disable_logs:
            self.save(
                os.path.join(
                    self.log_dir,
                    f"model_{self.current_learning_iteration}.pt",
                )
            )

    def learn(self, num_learning_iterations: int, init_at_random_ep_len: bool = False) -> None:
        """Defer signals and save a complete logical MDP state at an iteration boundary."""
        pending_signal = None
        previous_handlers = {}

        def request_stop(signum, _frame):
            nonlocal pending_signal
            if pending_signal is None:
                pending_signal = signum
                print(
                    "\n[MotionOnPolicyRunner] interrupt received; finishing the current PPO "
                    "iteration before saving",
                    flush=True,
                )

        try:
            for signum in (signal.SIGINT, signal.SIGTERM):
                previous_handlers[signum] = signal.getsignal(signum)
                signal.signal(signum, request_stop)
        except ValueError:
            # signal.signal is restricted to the main thread. Training normally runs there; retain
            # upstream behavior if an embedding invokes learn() from another thread.
            previous_handlers.clear()

        self._boundary_stop_requested = lambda: pending_signal is not None
        try:
            self._learn_loop(
                num_learning_iterations=num_learning_iterations,
                init_at_random_ep_len=init_at_random_ep_len,
            )
        finally:
            self._boundary_stop_requested = None
            for signum, handler in previous_handlers.items():
                signal.signal(signum, handler)

    def save(self, path: str, infos=None):
        """Atomically save a checkpoint that can continue at the next PPO iteration."""
        provenance = getattr(self, "checkpoint_provenance", None)
        logger_type = str(
            getattr(
                self,
                "logger_type",
                self.cfg.get("logger", "tensorboard"),
            )
        ).lower()
        sidecar = pathlib.Path(f"{path}.final_v3.json") if provenance is not None else None
        # Never leave a stale sidecar beside a failed/partial overwrite.  A successful save writes
        # the replacement atomically below; export independently re-hashes the checkpoint.
        if sidecar is not None:
            sidecar.unlink(missing_ok=True)
        checkpoint = pathlib.Path(path)
        checkpoint.parent.mkdir(parents=True, exist_ok=True)
        wandb_run_id = None
        wandb_run_name = None
        if logger_type == "wandb" and not self.disable_logs:
            try:
                import wandb

                if wandb.run is not None:
                    wandb_run_id = wandb.run.id
                    wandb_run_name = wandb.run.name
            except Exception:
                pass
        rng_state = self._capture_rng_state()
        resume_state = {
            "schema_version": 3,
            "resume_fidelity": "stateful_mdp_boundary_v1",
            # Base rsl_rl saves after completing iteration N but stores iter=N. Exact resume must
            # begin at N+1, otherwise it silently performs one duplicate PPO update.
            "next_learning_iteration": int(self.current_learning_iteration) + 1,
            "target_learning_iterations": int(self.cfg.get("max_iterations", 0)),
            "tot_timesteps": int(self.tot_timesteps),
            "tot_time": float(self.tot_time),
            "algorithm_learning_rate": float(self.alg.learning_rate),
            **rng_state,
            "log_dir": str(self.log_dir) if self.log_dir is not None else None,
            "wandb_run_id": wandb_run_id,
            "wandb_run_name": wandb_run_name,
            "environment_resume_state": self._capture_environment_resume_state(),
            "rollout_bookkeeping_state": getattr(
                self, "_rollout_bookkeeping_state", None
            ),
            "runtime_manifest": dict(
                getattr(self, "checkpoint_resume_manifest", {})
            ),
            "policy_exploration_contract": getattr(
                self.alg.policy, "exploration_contract", None
            ),
            **dict(getattr(self, "checkpoint_resume_context", {})),
        }
        saved_dict = {
            "model_state_dict": self.alg.policy.state_dict(),
            "optimizer_state_dict": self.alg.optimizer.state_dict(),
            "iter": self.current_learning_iteration,
            "infos": infos,
            "hope_exact_resume_state": resume_state,
        }
        if hasattr(self.alg, "rnd") and self.alg.rnd:
            saved_dict["rnd_state_dict"] = self.alg.rnd.state_dict()
            saved_dict["rnd_optimizer_state_dict"] = self.alg.rnd_optimizer.state_dict()
        temporary_checkpoint = checkpoint.with_name(
            f".{checkpoint.name}.tmp-{os.getpid()}"
        )
        try:
            torch.save(saved_dict, temporary_checkpoint)
            os.replace(temporary_checkpoint, checkpoint)
        finally:
            temporary_checkpoint.unlink(missing_ok=True)
        if logger_type == "neptune" and not self.disable_logs:
            self.writer.save_model(str(checkpoint), self.current_learning_iteration)
        elif (
            logger_type == "wandb"
            and not self.disable_logs
            and bool(getattr(self, "wandb_upload_checkpoints", False))
        ):
            self.writer.save_model(str(checkpoint), self.current_learning_iteration)
        elif (
            logger_type == "wandb"
            and not self.disable_logs
            and not getattr(self, "_wandb_checkpoint_skip_reported", False)
        ):
            print(
                "[MotionOnPolicyRunner] W&B checkpoint upload disabled; "
                "model_*.pt remains in the local run directory",
                flush=True,
            )
            self._wandb_checkpoint_skip_reported = True
        if provenance is not None:
            digest = hashlib.sha256()
            with checkpoint.open("rb") as stream:
                for block in iter(lambda: stream.read(1024 * 1024), b""):
                    digest.update(block)
            payload = {
                "schema_version": 1,
                "checkpoint_name": checkpoint.name,
                "checkpoint_sha256": digest.hexdigest(),
                **dict(provenance),
            }
            temporary = pathlib.Path(f"{sidecar}.tmp")
            temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
            temporary.replace(sidecar)
        if logger_type == "wandb":
            import wandb

            if getattr(self, "checkpoint_export_onnx", True):
                policy_path = path.split("model")[0]
                filename = policy_path.split("/")[-2] + ".onnx"
                # rsl_rl moved obs normalization onto the policy (actor_obs_normalizer); the runner no
                # longer has self.obs_normalizer. Fall back to None (-> Identity in the exporter) if absent.
                export_motion_policy_as_onnx(
                    self.env.unwrapped,
                    self.alg.policy,
                    normalizer=getattr(self.alg.policy, "actor_obs_normalizer", None),
                    path=policy_path,
                    filename=filename,
                )
                attach_onnx_metadata(
                    self.env.unwrapped, wandb.run.name, path=policy_path, filename=filename
                )
                if bool(getattr(self, "wandb_upload_onnx", False)):
                    wandb.save(
                        policy_path + filename,
                        base_path=os.path.dirname(policy_path),
                    )
                elif not getattr(self, "_wandb_onnx_skip_reported", False):
                    print(
                        "[MotionOnPolicyRunner] W&B ONNX upload disabled; "
                        "periodic ONNX remains local",
                        flush=True,
                    )
                    self._wandb_onnx_skip_reported = True
            elif not getattr(self, "_checkpoint_export_skip_reported", False):
                print(
                    "[MotionOnPolicyRunner] periodic ONNX export disabled by task YAML; "
                    "resumable PT checkpoints remain enabled",
                    flush=True,
                )
                self._checkpoint_export_skip_reported = True
            if sidecar is not None:
                # Keep remote checkpoint downloads exportable: the FinalV3 exporter requires this
                # hash binding and must not silently degrade to a local-run-only workflow.
                wandb.save(str(sidecar), base_path=os.path.dirname(str(sidecar)))
            provenance_files = getattr(self, "checkpoint_provenance_files", ())
            if provenance_files and not getattr(self, "_checkpoint_provenance_files_uploaded", False):
                for provenance_file in provenance_files:
                    provenance_file = pathlib.Path(provenance_file)
                    # Preserve params/<name> under the W&B run rather than flattening three files
                    # with checkpoint artifacts at the run root.
                    wandb.save(
                        str(provenance_file),
                        base_path=str(provenance_file.parents[1]),
                    )
                self._checkpoint_provenance_files_uploaded = True

            # Link the input motion artifact(s) to this run (lineage bookkeeping only — a W&B API
            # failure here must not kill the training run). W&B expects registry refs to include an
            # alias (for example, collection:latest).
            if self.registry_name is not None:
                registry_names = (
                    self.registry_name if isinstance(self.registry_name, (list, tuple)) else [self.registry_name]
                )
                for registry_name in registry_names:
                    try:
                        wandb.run.use_artifact(registry_name)
                    except Exception as e:
                        print(f"[MotionOnPolicyRunner] WARNING: use_artifact({registry_name!r}) failed: {e}")
                self.registry_name = None

    def _restore_yaml_optimizer_hyperparameters(self, optimizer_state: dict) -> None:
        """Keep a loaded optimizer's moments while restoring YAML-owned hyperparameters."""
        checkpoint_param_groups = optimizer_state.get("param_groups", [])
        checkpoint_weight_decay = (
            float(checkpoint_param_groups[0].get("weight_decay", 0.0))
            if checkpoint_param_groups
            else 0.0
        )
        restore_optimizer_hparams = getattr(
            self.alg, "restore_yaml_optimizer_hyperparameters", None
        )
        if callable(restore_optimizer_hparams):
            restore_optimizer_hparams()
            active_weight_decay = float(self.alg.optimizer.param_groups[0]["weight_decay"])
            migration_label = (
                "intentional Adam->AdamW migration"
                if checkpoint_weight_decay != active_weight_decay
                else "AdamW resume"
            )
            print(
                f"[MotionOnPolicyRunner] optimizer state restored with {migration_label}: "
                "moments retained, "
                f"checkpoint_weight_decay={checkpoint_weight_decay:.9g} -> "
                f"yaml_weight_decay={active_weight_decay:.9g}",
                flush=True,
            )

    def load(self, path: str, load_optimizer: bool = True, map_location=None):
        """Load a checkpoint and retain YAML optimizer settings after state restoration."""
        loaded = torch.load(path, map_location=map_location, weights_only=False)
        infos = super().load(path, load_optimizer=load_optimizer, map_location=map_location)
        if load_optimizer and "optimizer_state_dict" in loaded:
            self._restore_yaml_optimizer_hyperparameters(loaded["optimizer_state_dict"])
        return infos

    def _validate_runtime_manifest(self, saved: dict) -> None:
        active = dict(getattr(self, "checkpoint_resume_manifest", {}))
        if not saved or not active:
            raise RuntimeError(
                "Schema-v3 exact resume requires a non-empty runtime manifest on both "
                "the checkpoint and current launch"
            )
        if saved == active:
            return
        keys = sorted(set(saved) | set(active))
        mismatches = [
            key
            for key in keys
            if saved.get(key) != active.get(key)
        ]
        details = {
            key: {
                "checkpoint": saved.get(key),
                "runtime": active.get(key),
            }
            for key in mismatches[:8]
        }
        raise RuntimeError(
            "Stateful exact resume runtime manifest mismatch; source/config/motion "
            f"drift would change the MDP. mismatches={details}"
        )

    def load_exact(self, path: str):
        """Restore PPO plus the exact logical MDP boundary when the checkpoint supports it."""
        loaded = torch.load(path, map_location="cpu", weights_only=False)
        if "optimizer_state_dict" not in loaded:
            raise RuntimeError(
                f"Exact resume requires optimizer_state_dict, but checkpoint has none: {path}"
            )
        state = loaded.get("hope_exact_resume_state")
        schema_version = (
            int(state.get("schema_version", 0))
            if isinstance(state, dict)
            else 0
        )
        if not isinstance(state, dict) or schema_version not in (1, 2, 3):
            raise RuntimeError(
                "Exact resume metadata is absent. Use a checkpoint written after exact-resume "
                f"support was enabled, or use ordinary checkpoint_path warm-start semantics: {path}"
            )
        if schema_version == 3:
            self._validate_runtime_manifest(state.get("runtime_manifest", {}))

        # Exact means exact: load the optimizer parameter groups stored in PT verbatim. The ordinary
        # ``load`` method intentionally reapplies YAML optimizer settings for migration/warm-start,
        # which is a different operation and must not run on this path.
        infos = super().load(path, load_optimizer=True, map_location="cpu")
        saved_exploration_contract = state.get("policy_exploration_contract")
        active_exploration_contract = getattr(
            self.alg.policy, "exploration_contract", None
        )
        if saved_exploration_contract != active_exploration_contract:
            raise RuntimeError(
                "Exact resume policy-exploration contract mismatch: "
                f"checkpoint={saved_exploration_contract!r}, "
                f"runtime={active_exploration_contract!r}. A checkpoint from the old bounded-logit "
                "implementation must use ordinary checkpoint_path migration so actor/critic are "
                "retained while std and optimizer are reset."
            )
        self.current_learning_iteration = int(state["next_learning_iteration"])
        self.tot_timesteps = int(state.get("tot_timesteps", 0))
        self.tot_time = float(state.get("tot_time", 0.0))
        saved_learning_rate = float(state["algorithm_learning_rate"])
        optimizer_learning_rates = [
            float(param_group["lr"])
            for param_group in self.alg.optimizer.param_groups
        ]
        if any(
            learning_rate != saved_learning_rate
            for learning_rate in optimizer_learning_rates
        ):
            raise RuntimeError(
                "Exact resume learning-rate metadata disagrees with the "
                "checkpointed optimizer groups: "
                f"algorithm={saved_learning_rate}, "
                f"optimizer={optimizer_learning_rates}"
            )
        self.alg.learning_rate = saved_learning_rate
        rng_state = {
            name: state[name]
            for name in (
                "python_random_state",
                "numpy_random_state",
                "torch_random_state",
                "torch_cuda_random_states",
            )
        }
        if schema_version == 3:
            self._restore_environment_resume_state(state)
            self._restored_rollout_bookkeeping = state.get(
                "rollout_bookkeeping_state"
            )
            self._restore_rng_state(rng_state)
            self._pending_exact_rng_state = self._capture_rng_state()
            self._stateful_exact_resume_active = True
            print(
                "[MotionOnPolicyRunner] exact resume fidelity="
                "stateful_mdp_boundary_v1; optimizer/config/MDP boundary restored",
                flush=True,
            )
        else:
            # Irreversible compatibility bridge: old PTs never contained simulator/per-env state.
            # Restore their best available curriculum/RNG state, sample one fresh boundary, and
            # make every subsequently written checkpoint schema-v3/stateful.
            self._restore_legacy_environment_resume_state(state)
            self._restore_rng_state(rng_state)
            self.env.reset()
            self._pending_exact_rng_state = self._capture_rng_state()
            self._stateful_exact_resume_active = False
            print(
                "[MotionOnPolicyRunner] WARNING: LEGACY SCHEMA-"
                f"{schema_version} DISTRIBUTIONAL BRIDGE. This one resume cannot reconstruct "
                "the discarded PhysX/per-env boundary. The next checkpoint will be schema-3 "
                "and stateful; no actor or optimizer state was reset.",
                flush=True,
            )
        return infos, state

    def log(self, locs: dict, width: int = 80, pad: int = 35) -> None:
        # ``save`` runs immediately after ``log`` (and the interrupt path saves from inside it),
        # so snapshot rsl_rl's otherwise-local episode accumulators before either path writes PT.
        self._capture_rollout_bookkeeping(locs)
        super().log(locs, width=width, pad=pad)
        if not self.disable_logs and self.writer is not None:
            self._log_live_metrics(
                locs["it"],
                executed_entropy=locs.get("loss_dict", {}).get("entropy"),
            )
        stop_requested = getattr(self, "_boundary_stop_requested", None)
        if callable(stop_requested) and stop_requested():
            checkpoint = pathlib.Path(self.log_dir) / f"model_{self.current_learning_iteration}.pt"
            self.save(str(checkpoint))
            print(
                "[MotionOnPolicyRunner] interrupt checkpoint saved at a completed PPO boundary: "
                f"{checkpoint}",
                flush=True,
            )
            raise KeyboardInterrupt

    def _log_live_metrics(self, step: int, *, executed_entropy=None) -> None:
        """Log current manager state means every PPO iteration for richer dashboards."""
        env = self.env.unwrapped

        if hasattr(env, "command_manager"):
            for term_name in env.command_manager.active_terms:
                term = env.command_manager.get_term(term_name)
                for metric_name, metric_value in term.metrics.items():
                    self._log_scalar(f"Live/{term_name}/{metric_name}", self._mean_tensor(metric_value), step)
                if hasattr(term, "command_counter"):
                    self._log_scalar(
                        f"Live/{term_name}/command_counter", self._mean_tensor(term.command_counter), step
                    )
                # The full FH/BH x source x speed x z matrix and per-joint q_des diagnostics are
                # cumulative.  Emit them every ten PPO iterations and transfer the packed tensor
                # once, rather than synchronizing the GPU independently for ~1k scalar tags.
                instrumentation = getattr(term, "instrumentation_scalars", None)
                if callable(instrumentation) and step % 10 == 0:
                    self._log_scalar_mapping(
                        f"Instrumentation/{term_name}",
                        instrumentation(),
                        step,
                    )
                foundation_instrumentation = getattr(
                    term, "foundation_instrumentation_scalars", None
                )
                if callable(foundation_instrumentation) and step % 50 == 0:
                    self._log_scalar_mapping(
                        "Instrumentation",
                        foundation_instrumentation(),
                        step,
                    )

        if hasattr(env, "action_manager") and step % 10 == 0:
            try:
                action_term = env.action_manager.get_term("joint_pos")
            except (AttributeError, ValueError):
                action_term = None
            safety_instrumentation = getattr(
                action_term, "safety_instrumentation_scalars", None
            )
            if callable(safety_instrumentation):
                self._log_scalar_mapping(
                    "Instrumentation/qdes_safety",
                    safety_instrumentation(),
                    step,
                )

        if hasattr(env, "reward_manager"):
            self._log_scalar("Live/Reward/total", self._mean_tensor(getattr(env, "reward_buf", None)), step)
            for idx, term_name in enumerate(env.reward_manager.active_terms):
                self._log_scalar(
                    f"Live/Reward/{term_name}", self._mean_tensor(env.reward_manager._step_reward[:, idx]), step
                )

        if hasattr(env, "termination_manager"):
            tm = env.termination_manager
            self._log_scalar("Live/Termination/done_rate", self._mean_tensor(tm.dones), step)
            self._log_scalar("Live/Termination/terminated_rate", self._mean_tensor(tm.terminated), step)
            self._log_scalar("Live/Termination/timeout_rate", self._mean_tensor(tm.time_outs), step)
            for term_name in tm.active_terms:
                self._log_scalar(f"Live/Termination/{term_name}", self._mean_tensor(tm.get_term(term_name)), step)

        if hasattr(env, "action_manager"):
            action = getattr(env.action_manager, "action", None)
            prev_action = getattr(env.action_manager, "prev_action", None)
            if action is not None:
                action_abs = torch.abs(action)
                self._log_scalar("Live/Action/mean_abs", self._mean_tensor(action_abs), step)
                self._log_scalar("Live/Action/max_abs", self._mean_tensor(torch.max(action_abs, dim=-1).values), step)
            if action is not None and prev_action is not None:
                action_delta_abs = torch.abs(action - prev_action)
                self._log_scalar("Live/Action/delta_mean_abs", self._mean_tensor(action_delta_abs), step)
                self._log_scalar(
                    "Live/Action/delta_max_abs",
                    self._mean_tensor(torch.max(action_delta_abs, dim=-1).values),
                    step,
                )

        policy = getattr(getattr(self, "alg", None), "policy", None)
        min_noise_std = getattr(policy, "min_noise_std", None)
        max_noise_std = getattr(policy, "max_noise_std", None)
        std_parameter = getattr(policy, "std", None)
        effective_std = getattr(policy, "effective_noise_std", None)
        has_noise_bounds = min_noise_std is not None and max_noise_std is not None
        if effective_std is None and std_parameter is not None:
            # Standard rsl_rl ActorCritic intentionally has no min/max bounds. RallyV16 restores
            # that V11 policy class, so its learned std is already the effective std and must not
            # be passed through float(None). BoundedStdActorCritic keeps the clamped path below.
            if has_noise_bounds:
                effective_std = torch.clamp(
                    std_parameter,
                    min=float(min_noise_std),
                    max=float(max_noise_std),
                )
            else:
                effective_std = std_parameter
        if effective_std is not None:
            effective_std = effective_std.detach()
            active_mask = getattr(policy, "_entropy_active_action_mask", None)
            executable_std = effective_std[active_mask] if active_mask is not None else effective_std
            if std_parameter is not None:
                std_parameter = std_parameter.detach()
                executable_parameter = std_parameter[active_mask] if active_mask is not None else std_parameter
                self._log_scalar(
                    "Live/Policy/physical_noise_std_parameter_mean",
                    executable_parameter.mean(),
                    step,
                )
                self._log_scalar(
                    "Live/Policy/physical_noise_std_parameter_max",
                    executable_parameter.max(),
                    step,
                )
            self._log_scalar("Live/Policy/effective_noise_std_mean", executable_std.mean(), step)
            self._log_scalar("Live/Policy/effective_noise_std_max", executable_std.max(), step)
        if effective_std is not None and has_noise_bounds:
            std_span = float(max_noise_std) - float(min_noise_std)
            bound_fraction = (executable_std - float(min_noise_std)) / std_span
            self._log_scalar("Live/Policy/noise_std_bound_fraction_mean", bound_fraction.mean(), step)
            self._log_scalar("Live/Policy/noise_std_bound_fraction_max", bound_fraction.max(), step)
            tolerance = max(std_span * 1.0e-6, 1.0e-8)
            self._log_scalar(
                "Live/Policy/noise_std_at_lower_bound_fraction",
                (executable_std <= float(min_noise_std) + tolerance).float().mean(),
                step,
            )
            self._log_scalar(
                "Live/Policy/noise_std_at_upper_bound_fraction",
                (executable_std >= float(max_noise_std) - tolerance).float().mean(),
                step,
            )

        distribution = getattr(policy, "distribution", None)
        if distribution is not None:
            executable_gaussian_entropy = getattr(
                policy, "executable_gaussian_entropy", None
            )
            all_action_gaussian_entropy = getattr(
                policy, "all_action_gaussian_entropy", None
            )
            if executed_entropy is not None:
                self._log_scalar(
                    "Live/Policy/executed_entropy_mean",
                    self._mean_tensor(executed_entropy),
                    step,
                )
            if executable_gaussian_entropy is not None:
                self._log_scalar(
                    "Live/Policy/executable_gaussian_entropy_mean",
                    executable_gaussian_entropy.mean(),
                    step,
                )
            if all_action_gaussian_entropy is not None:
                self._log_scalar(
                    "Live/Policy/all_action_gaussian_entropy_mean",
                    all_action_gaussian_entropy.mean(),
                    step,
                )
                if executable_gaussian_entropy is not None:
                    self._log_scalar(
                        "Live/Policy/passive_gaussian_entropy_mean",
                        (
                            all_action_gaussian_entropy
                            - executable_gaussian_entropy
                        ).mean(),
                        step,
                    )

        self._log_scalar("Live/Env/episode_length", self._mean_tensor(env.episode_length_buf), step)
        self._log_scalar("Live/Env/common_step_counter", float(getattr(env, "common_step_counter", 0)), step)

    @staticmethod
    def _mean_tensor(value):
        if value is None:
            return None
        if isinstance(value, torch.Tensor):
            if value.numel() == 0:
                return None
            return value.float().mean().item()
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _log_scalar(self, tag: str, value, step: int) -> None:
        if value is None or not math.isfinite(float(value)):
            return
        self.writer.add_scalar(tag, float(value), step)

    def _log_scalar_mapping(self, prefix: str, values: dict, step: int) -> None:
        """Log a mapping of scalar tensors with one device synchronization."""
        tensor_items = [
            (str(name), value.reshape(()))
            for name, value in values.items()
            if torch.is_tensor(value) and value.numel() == 1
        ]
        if tensor_items:
            packed = torch.stack([value for _, value in tensor_items])
            host_values = packed.detach().float().cpu().tolist()
            for (name, _), value in zip(tensor_items, host_values):
                self._log_scalar(f"{prefix}/{name}", value, step)
        for name, value in values.items():
            if not torch.is_tensor(value):
                self._log_scalar(f"{prefix}/{name}", value, step)


# Public alias: the training/eval/export scripts construct the runner under this
# name (signature: env, train_cfg dict, log_dir, device).
HOPEOnPolicyRunner = MotionOnPolicyRunner
