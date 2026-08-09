from __future__ import annotations

import math
import re

import torch
from typing import TYPE_CHECKING, Literal

import isaaclab.utils.math as math_utils
from isaaclab.assets import Articulation
from isaaclab.actuators import ImplicitActuator
from isaaclab.envs.mdp.events import _randomize_prop_by_op
from isaaclab.managers import EventTermCfg, ManagerTermBase, SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


class randomize_structured_pd_gains(ManagerTermBase):
    """Randomize A3 PD gains with one correlated latent per actuator family.

    Unlike Isaac Lab's generic gain randomizer, this term keeps left/right
    homologous joints correlated and couples damping to stiffness:

    ``Kp' = alpha * Kp`` and ``Kd' = beta * sqrt(alpha) * Kd``.

    A deterministic fraction of environments stays exactly nominal.  The
    default joint gains are read as the immutable anchor on every reset, so
    repeated resets never compound the randomization and action scale remains
    independent of the sampled plant.
    """

    def __init__(self, cfg: EventTermCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)
        self.asset_cfg: SceneEntityCfg = cfg.params["asset_cfg"]
        self.asset: Articulation = env.scene[self.asset_cfg.name]
        self.group_patterns = {
            str(name): tuple(str(pattern) for pattern in patterns)
            for name, patterns in dict(cfg.params["groups"]).items()
        }
        self.group_names = tuple(self.group_patterns)
        if not self.group_names:
            raise ValueError("structured PD randomization requires actuator groups")

        self.alpha_range = self._validate_range(
            cfg.params["alpha_range"], "alpha_range"
        )
        self.beta_range = self._validate_range(
            cfg.params["beta_range"], "beta_range"
        )
        self.nominal_fraction = float(cfg.params["nominal_fraction"])
        if (
            not math.isfinite(self.nominal_fraction)
            or self.nominal_fraction < 0.0
            or self.nominal_fraction > 1.0
        ):
            raise ValueError("nominal_fraction must be finite and in [0, 1]")

        self.activation_command_name = str(
            cfg.params.get("activation_command_name", "")
        )
        self.activation_recovery_scale = float(
            cfg.params.get("activation_recovery_scale", 0.0)
        )
        if (
            not math.isfinite(self.activation_recovery_scale)
            or not 0.0 <= self.activation_recovery_scale <= 1.0
        ):
            raise ValueError("activation_recovery_scale must be finite and in [0, 1]")
        self.activation_requires_coverage_unlocked = bool(
            cfg.params.get("activation_requires_coverage_unlocked", False)
        )
        self.randomization_enabled = self.activation_command_name == ""

        excluded = set(str(name) for name in cfg.params.get("excluded_joint_names", ()))
        assignments: dict[str, list[int]] = {name: [] for name in self.group_names}
        for joint_id, joint_name in enumerate(self.asset.joint_names):
            if joint_name in excluded:
                continue
            matches = [
                group_name
                for group_name, patterns in self.group_patterns.items()
                if any(re.fullmatch(pattern, joint_name) for pattern in patterns)
            ]
            if len(matches) != 1:
                raise RuntimeError(
                    "structured PD groups must match every non-excluded joint exactly once: "
                    f"joint={joint_name!r}, matches={matches}"
                )
            assignments[matches[0]].append(joint_id)
        empty = [name for name, ids in assignments.items() if not ids]
        if empty:
            raise RuntimeError(f"structured PD groups matched no joints: {empty}")
        self.group_joint_ids = tuple(
            torch.tensor(assignments[name], dtype=torch.long, device=self.asset.device)
            for name in self.group_names
        )

        num_envs = int(env.scene.num_envs)
        num_groups = len(self.group_names)
        self.alpha = torch.ones(num_envs, num_groups, device=self.asset.device)
        self.beta = torch.ones_like(self.alpha)
        self.nominal_mask = torch.ones(
            num_envs, dtype=torch.bool, device=self.asset.device
        )

    @staticmethod
    def _validate_range(value, label: str) -> tuple[float, float]:
        if not isinstance(value, (tuple, list)) or len(value) != 2:
            raise ValueError(f"{label} must be a [lo, hi] pair")
        lo, hi = float(value[0]), float(value[1])
        if (
            not math.isfinite(lo)
            or not math.isfinite(hi)
            or lo <= 0.0
            or hi < lo
        ):
            raise ValueError(f"{label} must satisfy 0 < lo <= hi")
        return lo, hi

    def _select_nominal(self, env_ids: torch.Tensor) -> torch.Tensor:
        if self.nominal_fraction <= 0.0:
            return torch.zeros_like(env_ids, dtype=torch.bool)
        if self.nominal_fraction >= 1.0:
            return torch.ones_like(env_ids, dtype=torch.bool)
        reciprocal = 1.0 / self.nominal_fraction
        stride = int(round(reciprocal))
        if math.isclose(
            reciprocal, float(stride), rel_tol=0.0, abs_tol=1.0e-9
        ):
            # Stable anchor population: env 0,4,8,... stays nominal for 25%.
            return torch.remainder(env_ids, stride) == 0
        return (
            torch.rand(env_ids.numel(), device=self.asset.device)
            < self.nominal_fraction
        )

    def _publish_metrics(self, env: ManagerBasedEnv) -> None:
        command_manager = getattr(env, "command_manager", None)
        if command_manager is None:
            return
        try:
            command = command_manager.get_term("racket_target")
        except (AttributeError, KeyError, ValueError):
            return
        command.metrics["pd_nominal_env"] = self.nominal_mask.float().clone()
        command.metrics["pd_randomization_enabled"] = torch.full(
            (env.scene.num_envs,),
            float(self.randomization_enabled),
            device=self.asset.device,
        )
        for group_index, group_name in enumerate(self.group_names):
            command.metrics[f"pd_alpha_{group_name}"] = (
                self.alpha[:, group_index].clone()
            )
            command.metrics[f"pd_beta_{group_name}"] = (
                self.beta[:, group_index].clone()
            )

    def __call__(
        self,
        env: ManagerBasedEnv,
        env_ids: torch.Tensor | None,
        asset_cfg: SceneEntityCfg,
        groups: dict,
        alpha_range: tuple[float, float],
        beta_range: tuple[float, float],
        nominal_fraction: float,
        excluded_joint_names: tuple[str, ...] = (),
        activation_command_name: str = "",
        activation_recovery_scale: float = 0.0,
        activation_requires_coverage_unlocked: bool = False,
    ):
        del asset_cfg, groups, alpha_range, beta_range, nominal_fraction
        del excluded_joint_names, activation_command_name
        del activation_recovery_scale, activation_requires_coverage_unlocked
        if env_ids is None:
            env_ids = torch.arange(
                env.scene.num_envs, device=self.asset.device, dtype=torch.long
            )
        else:
            env_ids = env_ids.to(device=self.asset.device, dtype=torch.long)

        enabled = self.activation_command_name == ""
        if not enabled:
            try:
                command = env.command_manager.get_term(
                    self.activation_command_name
                )
            except (AttributeError, KeyError, ValueError) as exc:
                raise RuntimeError(
                    "structured PD activation command is unavailable: "
                    f"{self.activation_command_name!r}"
                ) from exc
            enabled = float(
                getattr(command, "_recovery_current_scale", 0.0)
            ) >= self.activation_recovery_scale - 1.0e-12
            if self.activation_requires_coverage_unlocked:
                enabled = enabled and bool(
                    getattr(
                        command,
                        "_recovery_stage1_coverage_unlocked",
                        False,
                    )
                )
        self.randomization_enabled = bool(enabled)

        if self.randomization_enabled:
            nominal = self._select_nominal(env_ids)
            sampled_alpha = torch.empty(
                env_ids.numel(), len(self.group_names), device=self.asset.device
            ).uniform_(*self.alpha_range)
            sampled_beta = torch.empty_like(sampled_alpha).uniform_(*self.beta_range)
            sampled_alpha[nominal] = 1.0
            sampled_beta[nominal] = 1.0
        else:
            nominal = torch.ones_like(env_ids, dtype=torch.bool)
            sampled_alpha = torch.ones(
                env_ids.numel(), len(self.group_names), device=self.asset.device
            )
            sampled_beta = torch.ones_like(sampled_alpha)
        self.alpha[env_ids] = sampled_alpha
        self.beta[env_ids] = sampled_beta
        self.nominal_mask[env_ids] = nominal

        stiffness = self.asset.data.default_joint_stiffness[env_ids].clone()
        damping = self.asset.data.default_joint_damping[env_ids].clone()
        for group_index, joint_ids in enumerate(self.group_joint_ids):
            alpha = sampled_alpha[:, group_index].unsqueeze(-1)
            beta = sampled_beta[:, group_index].unsqueeze(-1)
            stiffness[:, joint_ids] *= alpha
            damping[:, joint_ids] *= beta * torch.sqrt(alpha)

        self.asset.write_joint_stiffness_to_sim(
            stiffness, env_ids=env_ids
        )
        self.asset.write_joint_damping_to_sim(damping, env_ids=env_ids)
        for actuator in self.asset.actuators.values():
            joint_ids = actuator.joint_indices
            actuator.stiffness[env_ids] = stiffness[:, joint_ids]
            actuator.damping[env_ids] = damping[:, joint_ids]
            if not isinstance(actuator, ImplicitActuator):
                raise RuntimeError(
                    "RallyV17 structured PD contract requires implicit actuators"
                )
        self._publish_metrics(env)


class randomize_a3_message_pd_gains(ManagerTermBase):
    """HKUST-style A3 gain DR without randomizing passive viscous damping.

    The A3 Isaac drive stores ``Kd_message + damping_passive``.  Consequently the
    randomized drive must be ``beta * Kd_message + damping_passive`` rather than
    ``beta * default_joint_damping``.  A deterministic env-id cohort stays exactly
    nominal; robust environments draw per-joint log-uniform multipliers once at
    startup.  The affine action scale is intentionally untouched.
    """

    def __init__(self, cfg: EventTermCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)
        self.asset_cfg: SceneEntityCfg = cfg.params["asset_cfg"]
        self.asset: Articulation = env.scene[self.asset_cfg.name]
        self.alpha_range = randomize_structured_pd_gains._validate_range(
            cfg.params["alpha_range"], "alpha_range"
        )
        self.beta_range = randomize_structured_pd_gains._validate_range(
            cfg.params["beta_range"], "beta_range"
        )
        self.nominal_fraction = float(cfg.params["nominal_fraction"])
        if not (
            math.isfinite(self.nominal_fraction)
            and 0.0 <= self.nominal_fraction <= 1.0
        ):
            raise ValueError("nominal_fraction must be finite and in [0, 1]")

        from whole_body_tracking.robots.agibot_a3 import (
            a3_deploy_joint_kd,
            a3_passive_joint_damping,
        )

        message_kd = a3_deploy_joint_kd(list(self.asset.joint_names))
        if message_kd is None:
            raise RuntimeError(
                "A3 message/passive PD randomization was attached to a non-A3 asset"
            )
        passive = a3_passive_joint_damping(list(self.asset.joint_names))
        self.message_kd = torch.tensor(
            message_kd, dtype=torch.float, device=self.asset.device
        )
        self.passive_damping = torch.tensor(
            passive, dtype=torch.float, device=self.asset.device
        )
        expected_total = self.message_kd + self.passive_damping
        nominal_total = self.asset.data.default_joint_damping[0]
        if not bool(
            torch.allclose(
                nominal_total,
                expected_total,
                rtol=0.0,
                atol=1.0e-6,
            )
        ):
            raise RuntimeError(
                "A3 damping contract drift: Isaac default must equal "
                "deploy message Kd plus passive damping"
            )

        num_envs = int(env.scene.num_envs)
        num_joints = len(self.asset.joint_names)
        self.alpha = torch.ones(
            num_envs, num_joints, device=self.asset.device
        )
        self.beta = torch.ones_like(self.alpha)
        env_ids = torch.arange(
            num_envs, dtype=torch.long, device=self.asset.device
        )
        self.nominal_mask = self._select_nominal(env_ids)
        # CommandTerm.reset() zeros command metrics, while this startup event does not run again.
        # Keep one authoritative, read-only telemetry view on the articulation so the racket
        # command can republish the fixed cohort and gains every policy step.
        self.asset._hope_a3_pd_telemetry = {
            "nominal_mask": self.nominal_mask,
            "alpha": self.alpha,
            "beta": self.beta,
        }

    def _select_nominal(self, env_ids: torch.Tensor) -> torch.Tensor:
        if self.nominal_fraction <= 0.0:
            return torch.zeros_like(env_ids, dtype=torch.bool)
        if self.nominal_fraction >= 1.0:
            return torch.ones_like(env_ids, dtype=torch.bool)
        reciprocal = 1.0 / self.nominal_fraction
        stride = int(round(reciprocal))
        if not math.isclose(
            reciprocal, float(stride), rel_tol=0.0, abs_tol=1.0e-9
        ):
            raise RuntimeError(
                "A3 nominal fraction must be the reciprocal of an integer"
            )
        return torch.remainder(env_ids, stride) == 0

    @staticmethod
    def _sample_log_uniform(
        shape: tuple[int, ...], value_range: tuple[float, float], device
    ) -> torch.Tensor:
        lo, hi = value_range
        return torch.empty(shape, device=device).uniform_(
            math.log(lo), math.log(hi)
        ).exp_()

    def _publish_metrics(self, env: ManagerBasedEnv) -> None:
        command_manager = getattr(env, "command_manager", None)
        if command_manager is None:
            return
        try:
            command = command_manager.get_term("racket_target")
        except (AttributeError, KeyError, ValueError):
            return
        values = {
            "pd_nominal_env": self.nominal_mask.float(),
            "pd_kp_multiplier_mean": self.alpha.mean(dim=-1),
            "pd_kd_message_multiplier_mean": self.beta.mean(dim=-1),
        }
        for name, value in values.items():
            existing = command.metrics.get(name)
            if torch.is_tensor(existing) and existing.shape == value.shape:
                existing.copy_(value)
            else:
                command.metrics[name] = value.clone()

    def __call__(
        self,
        env: ManagerBasedEnv,
        env_ids: torch.Tensor | None,
        asset_cfg: SceneEntityCfg,
        alpha_range: tuple[float, float],
        beta_range: tuple[float, float],
        nominal_fraction: float,
    ):
        del asset_cfg, alpha_range, beta_range, nominal_fraction
        if env_ids is None:
            env_ids = torch.arange(
                env.scene.num_envs,
                dtype=torch.long,
                device=self.asset.device,
            )
        else:
            env_ids = env_ids.to(device=self.asset.device, dtype=torch.long)

        shape = (env_ids.numel(), len(self.asset.joint_names))
        alpha = self._sample_log_uniform(
            shape, self.alpha_range, self.asset.device
        )
        beta = self._sample_log_uniform(
            shape, self.beta_range, self.asset.device
        )
        nominal = self._select_nominal(env_ids)
        alpha[nominal] = 1.0
        beta[nominal] = 1.0
        self.alpha[env_ids] = alpha
        self.beta[env_ids] = beta
        self.nominal_mask[env_ids] = nominal

        stiffness = self.asset.data.default_joint_stiffness[env_ids] * alpha
        damping = (
            self.message_kd.unsqueeze(0) * beta
            + self.passive_damping.unsqueeze(0)
        )
        self.asset.write_joint_stiffness_to_sim(stiffness, env_ids=env_ids)
        self.asset.write_joint_damping_to_sim(damping, env_ids=env_ids)
        for actuator in self.asset.actuators.values():
            if not isinstance(actuator, ImplicitActuator):
                raise RuntimeError(
                    "A3 message/passive PD contract requires implicit actuators"
                )
            joint_ids = actuator.joint_indices
            actuator.stiffness[env_ids] = stiffness[:, joint_ids]
            actuator.damping[env_ids] = damping[:, joint_ids]
        self._publish_metrics(env)


def assert_material_randomization_scope(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    expected_body_names: tuple[str, ...],
):
    """Fail closed if the material event resolves beyond the two foot bodies."""

    del env_ids
    asset: Articulation = env.scene[asset_cfg.name]
    if isinstance(asset_cfg.body_ids, slice):
        resolved = tuple(asset.body_names[asset_cfg.body_ids])
    else:
        resolved = tuple(asset.body_names[int(index)] for index in asset_cfg.body_ids)
    expected = tuple(expected_body_names)
    if set(resolved) != set(expected) or len(resolved) != len(expected):
        raise RuntimeError(
            "RallyV17 foot material scope mismatch: "
            f"resolved={resolved}, expected={expected}. Refusing to randomize "
            "wrist/racket or other robot collision shapes."
        )


def randomize_joint_default_pos(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    pos_distribution_params: tuple[float, float] | None = None,
    operation: Literal["add", "scale", "abs"] = "abs",
    distribution: Literal["uniform", "log_uniform", "gaussian"] = "uniform",
):
    """
    Randomize the joint default positions which may be different from URDF due to calibration errors.
    """
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]

    # save nominal value for export
    asset.data.default_joint_pos_nominal = torch.clone(asset.data.default_joint_pos[0])

    # resolve environment ids
    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=asset.device)

    # resolve joint indices
    if asset_cfg.joint_ids == slice(None):
        joint_ids = slice(None)  # for optimization purposes
    else:
        joint_ids = torch.tensor(asset_cfg.joint_ids, dtype=torch.int, device=asset.device)

    if pos_distribution_params is not None:
        pos = asset.data.default_joint_pos.to(asset.device).clone()
        pos = _randomize_prop_by_op(
            pos, pos_distribution_params, env_ids, joint_ids, operation=operation, distribution=distribution
        )[env_ids][:, joint_ids]

        if env_ids != slice(None) and joint_ids != slice(None):
            env_ids = env_ids[:, None]
        asset.data.default_joint_pos[env_ids, joint_ids] = pos
        # update the offset in action since it is not updated automatically
        env.action_manager.get_term("joint_pos")._offset[env_ids, joint_ids] = pos


def randomize_rigid_body_com(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    com_range: dict[str, tuple[float, float]],
    asset_cfg: SceneEntityCfg,
):
    """Randomize the center of mass (CoM) of rigid bodies by adding a random value sampled from the given ranges.

    .. note::
        This function uses CPU tensors to assign the CoM. It is recommended to use this function
        only during the initialization of the environment.
    """
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    # resolve environment ids
    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device="cpu")
    else:
        env_ids = env_ids.cpu()

    # resolve body indices
    if asset_cfg.body_ids == slice(None):
        body_ids = torch.arange(asset.num_bodies, dtype=torch.int, device="cpu")
    else:
        body_ids = torch.tensor(asset_cfg.body_ids, dtype=torch.int, device="cpu")

    # sample random CoM values
    range_list = [com_range.get(key, (0.0, 0.0)) for key in ["x", "y", "z"]]
    ranges = torch.tensor(range_list, device="cpu")
    rand_samples = math_utils.sample_uniform(ranges[:, 0], ranges[:, 1], (len(env_ids), 3), device="cpu").unsqueeze(1)

    # get the current com of the bodies (num_assets, num_bodies)
    coms = asset.root_physx_view.get_coms().clone()

    # Randomize the com in range
    coms[:, body_ids, :3] += rand_samples

    # Set the new coms
    asset.root_physx_view.set_coms(coms, env_ids)
