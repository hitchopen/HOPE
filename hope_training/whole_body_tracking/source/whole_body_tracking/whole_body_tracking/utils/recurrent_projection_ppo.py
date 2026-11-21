"""Recurrent PPO for the StationaryRally balanced-hold recipe.

The upstream recurrent generator groups whole environments and preserves each
trajectory's initial hidden state. The projection loss uses those same
time/env rows; no feed-forward flattening or hidden-state reset is introduced.
The professor's balanced-hold recipe disables direct temporal actor losses.
"""

from __future__ import annotations

import torch
from torch import nn

from isaaclab.utils import configclass

from .projection_ppo import ProjectionPPO, RslRlProjectionPpoAlgorithmCfg
class RecurrentProjectionPPO(ProjectionPPO):
    """Apply baseline action projection to RSL-RL recurrent PPO minibatches."""

    _supports_recurrent = True

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        if not self.policy.is_recurrent or not self.projection_aux_enabled:
            raise ValueError("RecurrentProjectionPPO requires recurrent policy and projection auxiliary")
        if self.settled_aux_enabled:
            raise ValueError("StationaryRally balanced-hold PPO disables direct temporal actor loss")
        if self.rnd or self.symmetry or self.is_multi_gpu:
            raise ValueError("recurrent auxiliary PPO has no RND, symmetry, or multi-GPU contract")

    @torch.no_grad()
    def compute_returns(self, obs) -> None:
        """Peek at the next value without consuming the next critic observation.

        Upstream PPO.evaluate advances Memory in inference mode. Its bootstrap
        must not become part of the rollout history: the next act() evaluates
        the same observation again. Keep the existing GAE calculation, then
        restore the GRU state, including an uninitialized or just-reset state.
        """
        memory = self.policy.memory_c
        hidden = memory.hidden_state
        saved_hidden = hidden.detach().clone() if hidden is not None else None
        try:
            super().compute_returns(obs)
        finally:
            memory.reset(hidden_state=saved_hidden)

    def update(self) -> dict[str, float]:
        if (self.storage is None or self._projection_lower is None
                or self._projection_upper is None or self._projection_active is None):
            raise RuntimeError("recurrent projection bounds and rollout storage must be bound")
        if not getattr(self, "actor_updates_enabled", True):
            raise RuntimeError("recurrent auxiliary PPO does not support frozen actor updates")
        if self.storage.num_envs % self.num_mini_batches:
            raise ValueError("recurrent PPO requires num_envs divisible by num_mini_batches")

        storage = self.storage
        envs_per_batch = storage.num_envs // self.num_mini_batches
        actor_mean_parameters = list(self.policy.memory_a.parameters()) + list(self.policy.actor.parameters())
        projection_parameters = list(actor_mean_parameters)
        if self.projection_aux_gaussian_tails:
            projection_parameters.append(self.policy.std)
        totals = {key: 0.0 for key in (
            "value_function", "surrogate", "entropy", "projection_aux",
            "projection_distance_mean", "projection_outside_fraction",
            "projection_effective_lambda", "projection_gradient_ratio",
        )}
        generator = storage.recurrent_mini_batch_generator(
            self.num_mini_batches, self.num_learning_epochs
        )
        for update_index, batch in enumerate(generator):
            (obs, actions, old_values, advantages, returns, old_log_prob,
             old_mean, old_std, hidden, masks) = batch
            if self.normalize_advantage_per_mini_batch:
                with torch.no_grad():
                    advantages = (advantages - advantages.mean()) / (advantages.std() + 1.0e-8)
            start = (update_index % self.num_mini_batches) * envs_per_batch
            stop = start + envs_per_batch
            self.policy.act(obs, masks=masks, hidden_state=hidden[0])
            log_prob = self.policy.get_actions_log_prob(actions)
            values = self.policy.evaluate(obs, masks=masks, hidden_state=hidden[1])
            mean = self.policy.action_mean
            std = self.policy.action_std
            entropy = self.policy.entropy
            if mean.shape[:2] != (storage.num_transitions_per_env, envs_per_batch):
                raise RuntimeError("recurrent minibatch rows do not match rollout time/env order")

            if self.desired_kl is not None and self.schedule == "adaptive":
                with torch.no_grad():
                    kl = torch.sum(
                        torch.log(std / old_std + 1.0e-5)
                        + (old_std.square() + (old_mean - mean).square()) / (2.0 * std.square())
                        - 0.5, dim=-1,
                    ).mean()
                    if kl > self.desired_kl * 2.0:
                        self.learning_rate = max(1.0e-5, self.learning_rate / 1.5)
                    elif 0.0 < kl < self.desired_kl / 2.0:
                        self.learning_rate = min(1.0e-2, self.learning_rate * 1.5)
                    for group in self.optimizer.param_groups:
                        group["lr"] = self.learning_rate

            ratio = torch.exp(log_prob - old_log_prob.squeeze(-1))
            surrogate = -advantages.squeeze(-1) * ratio
            surrogate_clipped = -advantages.squeeze(-1) * ratio.clamp(
                1.0 - self.clip_param, 1.0 + self.clip_param
            )
            surrogate_loss = torch.maximum(surrogate, surrogate_clipped).mean()
            if self.use_clipped_value_loss:
                clipped = old_values + (values - old_values).clamp(-self.clip_param, self.clip_param)
                value_loss = torch.maximum((values - returns).square(), (clipped - returns).square()).mean()
            else:
                value_loss = (returns - values).square().mean()
            ppo_loss = surrogate_loss + self.value_loss_coef * value_loss - self.entropy_coef * entropy.mean()

            lower = self._projection_lower[start:stop].unsqueeze(0).expand_as(mean)
            upper = self._projection_upper[start:stop].unsqueeze(0).expand_as(mean)
            projection_loss, distance, outside = self._projection_loss(
                mean.reshape(-1, mean.shape[-1]),
                lower.reshape(-1, mean.shape[-1]),
                upper.reshape(-1, mean.shape[-1]),
                std.reshape(-1, mean.shape[-1]) if self.projection_aux_gaussian_tails else None,
            )
            self.optimizer.zero_grad()
            ppo_loss.backward(retain_graph=True)
            base_actor_norm = float(self._grad_norm(projection_parameters).detach())
            projection_grads = torch.autograd.grad(
                projection_loss, projection_parameters,
                allow_unused=True,
            )
            raw_projection_norm = float(self._grad_norm(projection_parameters, projection_grads).detach())
            requested = min(self.projection_aux_lambda, self.projection_aux_lambda_max)
            coefficient = min(
                requested,
                self.projection_aux_gradient_ratio_max * base_actor_norm / max(raw_projection_norm, 1.0e-20),
            ) if base_actor_norm > 0.0 else 0.0
            for parameter, gradient in zip(projection_parameters, projection_grads):
                if gradient is not None and coefficient > 0.0:
                    if parameter.grad is None:
                        parameter.grad = gradient * coefficient
                    else:
                        parameter.grad.add_(gradient, alpha=coefficient)
            totals["projection_effective_lambda"] += coefficient
            totals["projection_gradient_ratio"] += coefficient * raw_projection_norm / max(base_actor_norm, 1.0e-20)

            nn.utils.clip_grad_norm_(self.policy.parameters(), self.max_grad_norm)
            self.optimizer.step()
            totals["value_function"] += float(value_loss.detach())
            totals["surrogate"] += float(surrogate_loss.detach())
            totals["entropy"] += float(entropy.mean().detach())
            totals["projection_aux"] += float(projection_loss.detach())
            totals["projection_distance_mean"] += float(distance.detach())
            totals["projection_outside_fraction"] += float(outside.detach())

        storage.clear()
        count = self.num_learning_epochs * self.num_mini_batches
        return {
            **{key: value / count for key, value in totals.items()},
            "settled_aux_enabled": 0.0,
            "settled_mean_smooth": 0.0,
            "settled_mean_gradient_ratio": 0.0,
        }


@configclass
class RslRlRecurrentProjectionPpoAlgorithmCfg(RslRlProjectionPpoAlgorithmCfg):
    class_name: str = "RecurrentProjectionPPO"


def register_with_rsl_rl_runner() -> None:
    import rsl_rl.runners.on_policy_runner as runner_module

    runner_module.RecurrentProjectionPPO = RecurrentProjectionPPO
