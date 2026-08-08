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


class ProjectionPPO(PPO):
    """Upstream Adam PPO plus a gradient-capped actor-mean projection auxiliary."""

    def __init__(
        self,
        *args,
        projection_aux_enabled: bool = False,
        projection_aux_lambda: float = 0.0,
        projection_aux_lambda_max: float = 0.0,
        projection_aux_gradient_ratio_max: float = 0.0,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.projection_aux_enabled = bool(projection_aux_enabled)
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
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        active = self._projection_active
        if active is None:
            raise RuntimeError("ProjectionPPO bounds were not bound before update")
        mean = actor_mean[:, active]
        projected = torch.minimum(
            torch.maximum(mean, lower[:, active]), upper[:, active]
        )
        delta = mean - projected
        loss = torch.square(delta).mean()
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

        storage = self.storage
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
        actor_parameters = list(self.policy.actor.parameters())
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
                    self._projection_loss(mu_batch, lower_batch, upper_batch)
                )
                # The auxiliary deliberately acts on the differentiable actor mean. The plant,
                # however, executes sampled rollout actions. Report both populations so a small
                # mean penalty cannot hide large exploration tails.
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
                    projection_gradients = list(
                        torch.autograd.grad(
                            projection_loss,
                            actor_parameters,
                            retain_graph=False,
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
                    if float(raw_projection_grad_norm.detach()) <= 1.0e-20:
                        effective_lambda = requested_lambda
                        gradient_ratio = 0.0
                    elif float(base_actor_grad_norm.detach()) <= 1.0e-20:
                        effective_lambda = 0.0
                        gradient_ratio = 0.0
                    else:
                        cap = (
                            self.projection_aux_gradient_ratio_max
                            * float(base_actor_grad_norm.detach())
                            / float(raw_projection_grad_norm.detach())
                        )
                        effective_lambda = min(requested_lambda, cap)
                        gradient_ratio = (
                            effective_lambda
                            * float(raw_projection_grad_norm.detach())
                            / float(base_actor_grad_norm.detach())
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
                nn.utils.clip_grad_norm_(
                    self.policy.parameters(), self.max_grad_norm
                )
                self.optimizer.step()

                mean_value_loss += value_loss.item()
                mean_surrogate_loss += surrogate_loss.item()
                mean_entropy += entropy_batch.mean().item()
                mean_projection_loss += projection_loss.item()
                mean_projection_distance += projection_distance.item()
                mean_projection_outside += projection_outside.item()
                mean_projection_lambda += effective_lambda
                mean_projection_gradient_ratio += gradient_ratio
                mean_projection_sample_distance += (
                    sample_projection_distance.item()
                )
                mean_projection_sample_outside += (
                    sample_projection_outside.item()
                )
                mean_actor_abs_ge_one += actor_abs_ge_one.item()
                mean_sample_abs_ge_one += sample_abs_ge_one.item()
                mean_policy_std += active_sigma.mean().item()
                max_actor_mean_abs = max(
                    max_actor_mean_abs,
                    torch.abs(active_mean).max().item(),
                )
                max_sample_abs = max(
                    max_sample_abs,
                    torch.abs(active_samples).max().item(),
                )
                max_policy_std = max(
                    max_policy_std, active_sigma.max().item()
                )

        num_updates = self.num_learning_epochs * self.num_mini_batches
        storage.clear()
        return {
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


def register_with_rsl_rl_runner() -> None:
    """Expose the repository algorithm to rsl_rl's class-name factory."""

    import rsl_rl.runners.on_policy_runner as runner_module

    runner_module.ProjectionPPO = ProjectionPPO
