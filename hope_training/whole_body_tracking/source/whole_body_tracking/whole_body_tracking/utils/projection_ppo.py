"""PPO with a bounded auxiliary loss for V11 affine-clamp equivalence classes.

The environment still executes the exact V11 action contract::

    q_raw = q0 + scale * action
    q_des = clamp(q_raw, soft_lo, soft_hi)

For an action outside the raw interval corresponding to the soft limits, replacing it with its
interval projection produces exactly the same executed ``q_des``.  The auxiliary below therefore
selects a compact representative of an existing executable-action equivalence class; it does not
add an action limiter or change the plant command.
"""

from __future__ import annotations

import math

import torch
from torch import nn

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlPpoAlgorithmCfg
from rsl_rl.algorithms import PPO

from .gaussian_action_projection import gaussian_projection_debt
from .settled_policy_refinement import SettledPolicyRefinement


class ProjectionPPO(PPO):
    """Adam PPO plus a gradient-capped mean or Gaussian-tail projection cost."""

    def __init__(
        self,
        *args,
        projection_aux_enabled: bool = False,
        projection_aux_lambda: float = 0.0,
        projection_aux_lambda_max: float = 0.0,
        projection_aux_gradient_ratio_max: float = 0.0,
        projection_aux_gaussian_tails: bool = False,
        settled_aux_enabled: bool = False,
        settled_aux_start_steps: int = 448000,
        settled_aux_ramp_steps: int = 128000,
        settled_aux_smooth_coef: float = 0.01,
        settled_aux_tracking_coef: float = 0.0,
        settled_aux_gradient_ratio_max: float = 0.05,
        settled_aux_include_upper: bool = False,
        settled_aux_step_margin: float = 0.002,
        settled_aux_step_scale: float = 0.01,
        settled_aux_worst_joint_blend: float = 0.0,
        settled_aux_ready_only: bool = False,
        settled_aux_bounded_smooth: bool = True,
        update_diagnostics: bool = False,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.projection_aux_enabled = bool(projection_aux_enabled)
        self.update_diagnostics = bool(update_diagnostics)
        self.settled_aux_enabled = bool(settled_aux_enabled)
        self.settled_aux_start_steps = int(settled_aux_start_steps)
        self.settled_aux_ramp_steps = int(settled_aux_ramp_steps)
        self.settled_aux_smooth_coef = float(settled_aux_smooth_coef)
        self.settled_aux_tracking_coef = float(settled_aux_tracking_coef)
        if self.settled_aux_tracking_coef != 0.0:
            raise ValueError("q_des-q auxiliary tracking was removed: use actual task-pose PPO rewards; settled_aux_tracking_coef must be 0")
        self.settled_aux_gradient_ratio_max = float(settled_aux_gradient_ratio_max)
        self.settled_aux_include_upper = bool(settled_aux_include_upper)
        self.settled_aux_step_margin = float(settled_aux_step_margin)
        self.settled_aux_step_scale = float(settled_aux_step_scale)
        self.settled_aux_worst_joint_blend = float(settled_aux_worst_joint_blend)
        self.settled_aux_ready_only = bool(settled_aux_ready_only)
        self.settled_aux_bounded_smooth = bool(settled_aux_bounded_smooth)
        if not 0 <= self.settled_aux_worst_joint_blend <= 1:
            raise ValueError("settled_aux_worst_joint_blend must be in [0, 1]")
        self._settled_refinement = None
        if self.settled_aux_enabled and not self.projection_aux_enabled:
            raise ValueError("settled refinement requires the V11 ProjectionPPO path")
        if self.settled_aux_start_steps < 0 or self.settled_aux_ramp_steps <= 0:
            raise ValueError("invalid settled auxiliary curriculum")
        for value in (self.settled_aux_smooth_coef, self.settled_aux_tracking_coef,
                      self.settled_aux_gradient_ratio_max):
            if not math.isfinite(value) or value < 0:
                raise ValueError("settled auxiliary coefficients must be finite and non-negative")
        if self.settled_aux_gradient_ratio_max > 1:
            raise ValueError("settled auxiliary gradient ratio must be <= 1")
        self.projection_aux_gaussian_tails = bool(projection_aux_gaussian_tails)
        self.projection_aux_lambda = float(projection_aux_lambda)
        self.projection_aux_lambda_max = float(projection_aux_lambda_max)
        self.projection_aux_gradient_ratio_max = float(
            projection_aux_gradient_ratio_max
        )
        for name, value in (
            ("projection_aux_lambda", self.projection_aux_lambda),
            ("projection_aux_lambda_max", self.projection_aux_lambda_max),
            (
                "projection_aux_gradient_ratio_max",
                self.projection_aux_gradient_ratio_max,
            ),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative, got {value}")
        if self.projection_aux_lambda > self.projection_aux_lambda_max:
            raise ValueError(
                "projection_aux_lambda must not exceed projection_aux_lambda_max"
            )
        if self.projection_aux_gradient_ratio_max > 1.0:
            raise ValueError(
                "projection_aux_gradient_ratio_max must be in [0, 1]"
            )

        self._projection_lower: torch.Tensor | None = None
        self._projection_upper: torch.Tensor | None = None
        self._projection_active: torch.Tensor | None = None
        # AdamWPPO used to be the only optimizer installing this constraint.
        # ProjectionPPO owns an upstream Adam optimizer, so bound it as well,
        # including the disabled-auxiliary path that delegates to upstream PPO.
        project_std = getattr(self.policy, "project_noise_std_", None)
        if callable(project_std):
            self.optimizer.register_step_post_hook(
                lambda _optimizer, _args, _kwargs: project_std()
            )
        if self.projection_aux_enabled:
            if self.policy.is_recurrent:
                raise RuntimeError(
                    "ProjectionPPO currently requires a non-recurrent actor"
                )
            if self.symmetry:
                raise RuntimeError(
                    "ProjectionPPO equivalence bounds are not defined for symmetry augmentation"
                )
            if self.rnd:
                raise RuntimeError(
                    "ProjectionPPO is intentionally restricted to the V17 PPO contract without RND"
                )

    def bind_settled_refinement(self, env) -> None:
        if self.settled_aux_enabled:
            self._settled_refinement = SettledPolicyRefinement(
                env, self.storage, start_steps=self.settled_aux_start_steps,
                ramp_steps=self.settled_aux_ramp_steps,
                include_upper=self.settled_aux_include_upper,
                step_margin=self.settled_aux_step_margin, step_scale=self.settled_aux_step_scale,
                worst_joint_blend=self.settled_aux_worst_joint_blend,
                ready_only=self.settled_aux_ready_only,
                bounded_smooth=self.settled_aux_bounded_smooth,
            )
            print("[ProjectionPPO] deterministic HOLD refinement bound: "
                  f"start={self.settled_aux_start_steps}, ramp={self.settled_aux_ramp_steps}, "
                  f"gradient_ratio_max={self.settled_aux_gradient_ratio_max}", flush=True)

    def act(self, obs):
        if self.settled_aux_enabled:
            if self._settled_refinement is None:
                raise RuntimeError("settled refinement context was not bound")
            self._settled_refinement.capture(self.storage.step)
        return super().act(obs)

    @torch.no_grad()
    def bind_executable_action_projection(
        self,
        lower: torch.Tensor,
        upper: torch.Tensor,
        active: torch.Tensor,
    ) -> None:
        """Bind per-environment raw-action intervals after startup calibration.

        ``lower`` and ``upper`` are ``[num_envs, action_dim]`` because V11's startup calibration
        randomizes the affine decoder offset per environment.  Keeping those rows aligned with the
        rollout's ``[time, env]`` layout is necessary for the equivalence proof; a global
        intersection would incorrectly penalize some locally executable actions.
        """

        if not self.projection_aux_enabled:
            raise RuntimeError("cannot bind projection bounds when the auxiliary is disabled")
        lower = torch.as_tensor(lower, device=self.device, dtype=torch.float32)
        upper = torch.as_tensor(upper, device=self.device, dtype=torch.float32)
        active = torch.as_tensor(active, device=self.device, dtype=torch.bool)
        expected = (self.storage.num_envs, self.storage.actions.shape[-1])
        if tuple(lower.shape) != expected or tuple(upper.shape) != expected:
            raise RuntimeError(
                "projection bounds shape mismatch: "
                f"lower={tuple(lower.shape)}, upper={tuple(upper.shape)}, expected={expected}"
            )
        if tuple(active.shape) != (expected[1],):
            raise RuntimeError(
                f"projection active mask must have shape {(expected[1],)}, got {tuple(active.shape)}"
            )
        if int(active.sum()) <= 0:
            raise RuntimeError("projection auxiliary has no active action channels")
        if not bool(torch.isfinite(lower).all() and torch.isfinite(upper).all()):
            raise RuntimeError("projection bounds contain NaN/Inf")
        if bool((lower[:, active] >= upper[:, active]).any()):
            raise RuntimeError("projection lower bound must be below upper bound")
        self._projection_lower = lower.detach().clone()
        self._projection_upper = upper.detach().clone()
        self._projection_active = active.detach().clone()
        print(
            "[ProjectionPPO] executable-equivalence auxiliary ACTIVE: "
            f"lambda={self.projection_aux_lambda:.6g}, "
            f"lambda_max={self.projection_aux_lambda_max:.6g}, "
            f"gradient_ratio_max={self.projection_aux_gradient_ratio_max:.6g}, "
            f"active_actions={int(active.sum())}/{active.numel()}, "
            f"per_env_bounds={expected[0]}",
            flush=True,
        )

    def _projection_loss(
        self,
        actor_mean: torch.Tensor,
        lower: torch.Tensor,
        upper: torch.Tensor,
        actor_std: torch.Tensor | None = None,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        active = self._projection_active
        if active is None:
            raise RuntimeError("ProjectionPPO bounds were not bound before update")
        mean = actor_mean[:, active]
        projected = torch.minimum(
            torch.maximum(mean, lower[:, active]), upper[:, active]
        )
        delta = mean - projected
        if actor_std is None:
            loss = torch.square(delta).mean()
        else:
            loss = gaussian_projection_debt(
                mean, actor_std[:, active], lower[:, active], upper[:, active]
            ).mean()
        distance = torch.sqrt(torch.square(delta).mean(dim=-1))
        outside = (delta != 0.0).float().mean()
        return loss, distance.mean(), outside

    @staticmethod
    def _grad_norm(
        parameters: list[torch.nn.Parameter],
        gradients: list[torch.Tensor | None] | None = None,
    ) -> torch.Tensor:
        terms = []
        for index, parameter in enumerate(parameters):
            grad = parameter.grad if gradients is None else gradients[index]
            if grad is not None:
                terms.append(torch.sum(torch.square(grad)))
        if not terms:
            return torch.zeros((), device=parameters[0].device)
        return torch.sqrt(torch.stack(terms).sum())

    def update(self) -> dict[str, float]:
        """Run the upstream PPO math and add the projection gradient under a hard ratio cap."""

        if not self.projection_aux_enabled:
            return super().update()
        if (
            self._projection_lower is None
            or self._projection_upper is None
            or self._projection_active is None
        ):
            raise RuntimeError(
                "ProjectionPPO auxiliary is enabled but executable bounds were never bound"
            )

        actor_updates_enabled = bool(
            getattr(self, "actor_updates_enabled", True)
        )
        mean_value_loss = 0.0
        mean_surrogate_loss = 0.0
        mean_entropy = 0.0
        mean_projection_loss = 0.0
        mean_projection_distance = 0.0
        mean_projection_outside = 0.0
        mean_projection_lambda = 0.0
        mean_projection_gradient_ratio = 0.0
        mean_projection_sample_distance = 0.0
        mean_projection_sample_outside = 0.0
        mean_actor_abs_ge_one = 0.0
        mean_sample_abs_ge_one = 0.0
        mean_policy_std = 0.0
        max_actor_mean_abs = 0.0
        max_sample_abs = 0.0
        max_policy_std = 0.0
        # Disabled refinement still reports its zero added gradient, without
        # fabricating a zero q_des-step measurement for an unmeasured signal.
        settled_totals = {"gradient_ratio": 0.0}
        run_settled = (self.settled_aux_enabled and
                       self._settled_refinement.rollout_has_curriculum)
        if self.settled_aux_enabled and not run_settled:
            # Exactly zero before the curriculum. Avoid an extra actor forward
            # and autograd pass for every minibatch; keep zero diagnostic keys.
            settled_totals = dict.fromkeys(("smooth", "qdes_step_rad", "active_fraction",
                                           "pair_fraction", "curriculum_scale", "gradient_ratio",
                                           "gradient_multiplier"), 0.0)

        storage = self.storage
        diagnostics = {}
        gradient_rows = []
        diagnostic_kl = []
        minibatch_statistics = []
        if self.update_diagnostics:
            from .ppo_diagnostics import value_statistics, gradient_statistics
            diagnostics = value_statistics(storage.values, storage.returns)
            diagnostic_critic = list(self.policy.critic.parameters())
            diagnostic_all = list(self.policy.parameters())
        batch_size = storage.num_envs * storage.num_transitions_per_env
        mini_batch_size = batch_size // self.num_mini_batches
        if mini_batch_size * self.num_mini_batches != batch_size:
            raise RuntimeError(
                "ProjectionPPO requires rollout size divisible by num_mini_batches"
            )
        indices = torch.randperm(batch_size, requires_grad=False, device=self.device)

        observations = storage.observations.flatten(0, 1)
        actions = storage.actions.flatten(0, 1)
        target_values = storage.values.flatten(0, 1)
        returns = storage.returns.flatten(0, 1)
        old_actions_log_prob = storage.actions_log_prob.flatten(0, 1)
        advantages = storage.advantages.flatten(0, 1)
        old_mu = storage.mu.flatten(0, 1)
        old_sigma = storage.sigma.flatten(0, 1)
        projection_lower = (
            self._projection_lower.unsqueeze(0)
            .expand(storage.num_transitions_per_env, -1, -1)
            .flatten(0, 1)
        )
        projection_upper = (
            self._projection_upper.unsqueeze(0)
            .expand(storage.num_transitions_per_env, -1, -1)
            .flatten(0, 1)
        )
        mean_parameters = list(self.policy.actor.parameters())
        actor_parameters = list(mean_parameters)
        if self.projection_aux_gaussian_tails:
            std_parameter = getattr(self.policy, "std", None)
            if not isinstance(std_parameter, torch.nn.Parameter):
                raise RuntimeError("Gaussian projection requires a trainable physical policy.std")
            actor_parameters.append(std_parameter)
        if not actor_parameters:
            raise RuntimeError("ProjectionPPO could not resolve actor parameters")

        for _epoch in range(self.num_learning_epochs):
            for mini_batch in range(self.num_mini_batches):
                start = mini_batch * mini_batch_size
                stop = (mini_batch + 1) * mini_batch_size
                batch_idx = indices[start:stop]
                obs_batch = observations[batch_idx]
                actions_batch = actions[batch_idx]
                target_values_batch = target_values[batch_idx]
                returns_batch = returns[batch_idx]
                old_actions_log_prob_batch = old_actions_log_prob[batch_idx]
                advantages_batch = advantages[batch_idx]
                old_mu_batch = old_mu[batch_idx]
                old_sigma_batch = old_sigma[batch_idx]
                lower_batch = projection_lower[batch_idx]
                upper_batch = projection_upper[batch_idx]

                if self.normalize_advantage_per_mini_batch:
                    with torch.no_grad():
                        advantages_batch = (
                            advantages_batch - advantages_batch.mean()
                        ) / (advantages_batch.std() + 1.0e-8)

                self.policy.act(obs_batch)
                actions_log_prob_batch = self.policy.get_actions_log_prob(
                    actions_batch
                )
                value_batch = self.policy.evaluate(obs_batch)
                mu_batch = self.policy.action_mean
                sigma_batch = self.policy.action_std
                entropy_batch = self.policy.entropy

                if (
                    actor_updates_enabled
                    and self.desired_kl is not None
                    and self.schedule == "adaptive"
                ):
                    with torch.inference_mode():
                        kl = torch.sum(
                            torch.log(sigma_batch / old_sigma_batch + 1.0e-5)
                            + (
                                torch.square(old_sigma_batch)
                                + torch.square(old_mu_batch - mu_batch)
                            )
                            / (2.0 * torch.square(sigma_batch))
                            - 0.5,
                            dim=-1,
                        )
                        kl_mean = torch.mean(kl)
                        if self.is_multi_gpu:
                            torch.distributed.all_reduce(
                                kl_mean, op=torch.distributed.ReduceOp.SUM
                            )
                            kl_mean /= self.gpu_world_size
                        if self.gpu_global_rank == 0:
                            if kl_mean > self.desired_kl * 2.0:
                                self.learning_rate = max(
                                    1.0e-5, self.learning_rate / 1.5
                                )
                            elif (
                                kl_mean < self.desired_kl / 2.0
                                and kl_mean > 0.0
                            ):
                                self.learning_rate = min(
                                    float(
                                        getattr(
                                            self,
                                            "adaptive_learning_rate_max",
                                            1.0e-2,
                                        )
                                    ),
                                    self.learning_rate * 1.5,
                                )
                        if self.is_multi_gpu:
                            lr_tensor = torch.tensor(
                                self.learning_rate, device=self.device
                            )
                            torch.distributed.broadcast(lr_tensor, src=0)
                            self.learning_rate = lr_tensor.item()
                        for param_group in self.optimizer.param_groups:
                            param_group["lr"] = self.learning_rate

                ratio = torch.exp(
                    actions_log_prob_batch
                    - torch.squeeze(old_actions_log_prob_batch)
                )
                surrogate = -torch.squeeze(advantages_batch) * ratio
                surrogate_clipped = -torch.squeeze(
                    advantages_batch
                ) * torch.clamp(
                    ratio, 1.0 - self.clip_param, 1.0 + self.clip_param
                )
                surrogate_loss = torch.max(
                    surrogate, surrogate_clipped
                ).mean()

                if self.use_clipped_value_loss:
                    value_clipped = target_values_batch + (
                        value_batch - target_values_batch
                    ).clamp(-self.clip_param, self.clip_param)
                    value_losses = torch.square(value_batch - returns_batch)
                    value_losses_clipped = torch.square(
                        value_clipped - returns_batch
                    )
                    value_loss = torch.max(
                        value_losses, value_losses_clipped
                    ).mean()
                else:
                    value_loss = torch.square(
                        returns_batch - value_batch
                    ).mean()

                ppo_loss = (
                    surrogate_loss
                    + self.value_loss_coef * value_loss
                    - self.entropy_coef * entropy_batch.mean()
                )
                projection_loss, projection_distance, projection_outside = (
                    self._projection_loss(
                        mu_batch, lower_batch, upper_batch,
                        sigma_batch if self.projection_aux_gaussian_tails else None,
                    )
                )
                settled_loss = None
                settled_stats = {}
                if run_settled:
                    smooth, tracking, diagnostics = self._settled_refinement.losses(
                        self.policy, observations, mu_batch, batch_idx, storage.dones,
                    )
                    settled_loss = self.settled_aux_smooth_coef * smooth
                    settled_stats = {"smooth": smooth.item(),
                                     **{k: v.item() for k, v in diagnostics.items()
                                        if not k.startswith("waist_qdes_offset_")}}
                    # Physical PD offsets are logged by the state/command/torque
                    # chain, not under rsl_rl's misleading Loss/* namespace.
                # Preserve separate mean and rollout-sample diagnostics even when
                # the auxiliary integrates the full current Gaussian distribution.
                with torch.no_grad():
                    (
                        _sample_projection_loss,
                        sample_projection_distance,
                        sample_projection_outside,
                    ) = self._projection_loss(
                        actions_batch, lower_batch, upper_batch
                    )
                    active = self._projection_active
                    if active is None:
                        raise RuntimeError(
                            "ProjectionPPO active mask disappeared during update"
                        )
                    active_mean = mu_batch[:, active]
                    active_samples = actions_batch[:, active]
                    active_sigma = sigma_batch[:, active]
                    actor_abs_ge_one = (
                        torch.abs(active_mean) >= 1.0
                    ).float().mean()
                    sample_abs_ge_one = (
                        torch.abs(active_samples) >= 1.0
                    ).float().mean()

                self.optimizer.zero_grad()
                ppo_loss.backward(retain_graph=actor_updates_enabled)
                if actor_updates_enabled:
                    base_actor_grad_norm = self._grad_norm(actor_parameters)
                    if settled_loss is not None:
                        base_mean_grad_norm = self._grad_norm(mean_parameters)
                    projection_gradients = list(
                        torch.autograd.grad(
                            projection_loss,
                            actor_parameters,
                            retain_graph=settled_loss is not None,
                            allow_unused=True,
                        )
                    )
                    raw_projection_grad_norm = self._grad_norm(
                        actor_parameters, projection_gradients
                    )
                    requested_lambda = min(
                        self.projection_aux_lambda,
                        self.projection_aux_lambda_max,
                    )
                    base_norm_value, raw_norm_value = torch.stack((
                        base_actor_grad_norm.detach(), raw_projection_grad_norm.detach()
                    )).cpu().tolist()
                    if raw_norm_value <= 1.0e-20:
                        effective_lambda = requested_lambda
                        gradient_ratio = 0.0
                    elif base_norm_value <= 1.0e-20:
                        effective_lambda = 0.0
                        gradient_ratio = 0.0
                    else:
                        cap = (
                            self.projection_aux_gradient_ratio_max
                            * base_norm_value
                            / raw_norm_value
                        )
                        effective_lambda = min(requested_lambda, cap)
                        gradient_ratio = (
                            effective_lambda
                            * raw_norm_value
                            / base_norm_value
                        )
                    for parameter, projection_gradient in zip(
                        actor_parameters, projection_gradients
                    ):
                        if projection_gradient is None or effective_lambda == 0.0:
                            continue
                        addition = projection_gradient * effective_lambda
                        if parameter.grad is None:
                            parameter.grad = addition
                        else:
                            parameter.grad.add_(addition)
                    if settled_loss is not None:
                        # The cap references the original PPO actor gradient, excluding std.
                        # Both new objectives differentiate deterministic means only.
                        gradients = list(torch.autograd.grad(
                            settled_loss, mean_parameters, allow_unused=True,
                        ))
                        raw_norm = float(self._grad_norm(mean_parameters, gradients).detach())
                        base_norm = float(base_mean_grad_norm.detach())
                        # Ramp the ceiling too: clipping must not undo the late curriculum.
                        ratio_cap = (self.settled_aux_gradient_ratio_max
                                     * settled_stats["curriculum_scale"])
                        multiplier = min(1.0, ratio_cap
                                         * base_norm / max(raw_norm, 1e-20))
                        for parameter, gradient in zip(mean_parameters, gradients):
                            if gradient is not None:
                                addition = gradient * multiplier
                                if parameter.grad is None:
                                    parameter.grad = addition
                                else:
                                    parameter.grad.add_(addition)
                        settled_stats["gradient_ratio"] = multiplier * raw_norm / max(base_norm, 1e-20)
                        settled_stats["gradient_multiplier"] = multiplier
                else:
                    # A migrated actor must not be rewritten by advantages from a freshly
                    # initialized critic.  ``grad=None`` also prevents AdamW weight decay and
                    # optimizer-state creation for actor/std, while critic gradients remain.
                    critic_ids = {
                        id(parameter)
                        for parameter in self.policy.critic.parameters()
                    }
                    for parameter in self.policy.parameters():
                        if id(parameter) not in critic_ids:
                            parameter.grad = None
                    effective_lambda = 0.0
                    gradient_ratio = 0.0

                if self.is_multi_gpu:
                    self.reduce_parameters()
                if self.update_diagnostics:
                    gradient_rows.append(gradient_statistics(
                        mean_parameters, diagnostic_critic, diagnostic_all, self.max_grad_norm))
                    with torch.no_grad():
                        diagnostic_kl.append((torch.log(sigma_batch.detach()/old_sigma_batch)
                            +(old_sigma_batch.square()+(old_mu_batch-mu_batch.detach()).square())
                            /(2*sigma_batch.detach().square())-.5).sum(-1).mean())
                nn.utils.clip_grad_norm_(
                    self.policy.parameters(), self.max_grad_norm
                )
                self.optimizer.step()
                for key, value in settled_stats.items():
                    settled_totals[key] = settled_totals.get(key, 0.0) + value

                # Diagnostics do not feed the next minibatch. Retain only
                # detached scalars, then transfer once after all updates instead
                # of synchronizing CUDA for each of 14 .item() calls per batch.
                with torch.no_grad():
                    minibatch_statistics.append(torch.stack((
                        value_loss.detach(), surrogate_loss.detach(), entropy_batch.mean(),
                        projection_loss.detach(), projection_distance, projection_outside,
                        sample_projection_distance, sample_projection_outside,
                        actor_abs_ge_one, sample_abs_ge_one, active_sigma.mean(),
                        torch.abs(active_mean).max(), torch.abs(active_samples).max(),
                        active_sigma.max(),
                    )))
                mean_projection_lambda += effective_lambda
                mean_projection_gradient_ratio += gradient_ratio

        # Python sums/maxima in the original minibatch order retain the old
        # double-precision accumulation semantics exactly.
        for row in torch.stack(minibatch_statistics).cpu().tolist():
            mean_value_loss += row[0]
            mean_surrogate_loss += row[1]
            mean_entropy += row[2]
            mean_projection_loss += row[3]
            mean_projection_distance += row[4]
            mean_projection_outside += row[5]
            mean_projection_sample_distance += row[6]
            mean_projection_sample_outside += row[7]
            mean_actor_abs_ge_one += row[8]
            mean_sample_abs_ge_one += row[9]
            mean_policy_std += row[10]
            max_actor_mean_abs = max(max_actor_mean_abs, row[11])
            max_sample_abs = max(max_sample_abs, row[12])
            max_policy_std = max(max_policy_std, row[13])

        num_updates = self.num_learning_epochs * self.num_mini_batches
        if self.update_diagnostics:
            for key, value in zip(("actor_grad_norm", "critic_grad_norm", "total_grad_norm", "grad_clip_multiplier"),
                                  torch.stack(gradient_rows).mean(0)):
                diagnostics[key] = value
            diagnostics["policy_kl"] = torch.stack(diagnostic_kl).mean()
            values = torch.stack(list(diagnostics.values())).cpu().tolist()
            diagnostics = dict(zip(diagnostics, values))
        storage.clear()
        return {
            **diagnostics,
            **{f"settled_mean_{k}": v / num_updates for k, v in settled_totals.items()},
            "settled_aux_enabled": float(self.settled_aux_enabled),
            "value_function": mean_value_loss / num_updates,
            "surrogate": mean_surrogate_loss / num_updates,
            "entropy": mean_entropy / num_updates,
            "projection_aux": mean_projection_loss / num_updates,
            "projection_distance_mean": mean_projection_distance / num_updates,
            "projection_outside_fraction": mean_projection_outside / num_updates,
            "projection_sample_distance_mean": (
                mean_projection_sample_distance / num_updates
            ),
            "projection_sample_outside_fraction": (
                mean_projection_sample_outside / num_updates
            ),
            "projection_actor_mean_abs_max": max_actor_mean_abs,
            "projection_sample_abs_max": max_sample_abs,
            "projection_policy_std_mean": mean_policy_std / num_updates,
            "projection_policy_std_max": max_policy_std,
            "projection_actor_mean_abs_ge_one_fraction": (
                mean_actor_abs_ge_one / num_updates
            ),
            "projection_sample_abs_ge_one_fraction": (
                mean_sample_abs_ge_one / num_updates
            ),
            "projection_effective_lambda": mean_projection_lambda / num_updates,
            "projection_gradient_ratio": (
                mean_projection_gradient_ratio / num_updates
            ),
            "actor_updates_enabled": float(actor_updates_enabled),
        }


@configclass
class RslRlProjectionPpoAlgorithmCfg(RslRlPpoAlgorithmCfg):
    """Isaac Lab runner configuration for :class:`ProjectionPPO`."""

    class_name: str = "ProjectionPPO"
    projection_aux_enabled: bool = False
    projection_aux_lambda: float = 0.0
    projection_aux_lambda_max: float = 0.0
    projection_aux_gradient_ratio_max: float = 0.0
    projection_aux_gaussian_tails: bool = False
    settled_aux_enabled: bool = False
    settled_aux_start_steps: int = 448000
    settled_aux_ramp_steps: int = 128000
    settled_aux_smooth_coef: float = 0.01
    settled_aux_tracking_coef: float = 0.0
    settled_aux_gradient_ratio_max: float = 0.05
    settled_aux_include_upper: bool = False
    settled_aux_step_margin: float = 0.002
    settled_aux_step_scale: float = 0.01
    settled_aux_worst_joint_blend: float = 0.0
    settled_aux_ready_only: bool = False
    settled_aux_bounded_smooth: bool = True
    update_diagnostics: bool = False


def register_with_rsl_rl_runner() -> None:
    """Expose the repository algorithm to rsl_rl's class-name factory."""

    import rsl_rl.runners.on_policy_runner as runner_module

    runner_module.ProjectionPPO = ProjectionPPO
