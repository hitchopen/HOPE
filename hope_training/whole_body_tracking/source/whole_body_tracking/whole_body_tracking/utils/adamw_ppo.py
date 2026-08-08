"""YAML-configured AdamW PPO matching the released HUGWBC optimizer contract."""

from __future__ import annotations

import torch

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlPpoAlgorithmCfg
from rsl_rl.algorithms import PPO


class _ProjectedAdamW(torch.optim.AdamW):
    """AdamW followed by projection of policy-owned constrained parameters."""

    def __init__(self, *args, project_after_step=None, **kwargs):
        self._project_after_step = project_after_step
        super().__init__(*args, **kwargs)

    def step(self, closure=None):
        loss = super().step(closure)
        if self._project_after_step is not None:
            self._project_after_step()
        return loss


class AdamWPPO(PPO):
    """Use AdamW and project the learned physical action std after every minibatch."""

    def __init__(self, *args, weight_decay: float = 0.01, **kwargs):
        super().__init__(*args, **kwargs)
        self.weight_decay = float(weight_decay)
        project_noise_std = getattr(self.policy, "project_noise_std_", None)
        # The exploration std is a CONSTRAINED distribution parameter (projected into the YAML
        # interval after every step), not a network weight: decaying it adds a permanent hidden
        # pull toward zero exploration that silently fights the entropy bonus (2026-07-23 audit).
        # Network weights/biases keep the released HUGWBC AdamW decay.
        std_param = getattr(self.policy, "std", None)
        std_param_ids = {id(std_param)} if isinstance(std_param, torch.nn.Parameter) else set()
        decay_params = [p for p in self.policy.parameters() if id(p) not in std_param_ids]
        no_decay_params = [p for p in self.policy.parameters() if id(p) in std_param_ids]
        param_groups = [
            {"params": decay_params, "weight_decay": self.weight_decay, "hope_group": "decay"}
        ]
        if no_decay_params:
            param_groups.append(
                {"params": no_decay_params, "weight_decay": 0.0, "hope_group": "no_decay"}
            )
        self.optimizer = _ProjectedAdamW(
            param_groups,
            lr=self.learning_rate,
            project_after_step=project_noise_std if callable(project_noise_std) else None,
        )
        print(
            "[AdamWPPO] AdamW + post-step physical-std projection ACTIVE: "
            f"lr={self.learning_rate:.9g} weight_decay={self.weight_decay:.9g} "
            f"(std excluded from decay: {bool(no_decay_params)})",
            flush=True,
        )

    def restore_yaml_optimizer_hyperparameters(self) -> None:
        """Reapply values that an older Adam optimizer state can overwrite on load."""
        for param_group in self.optimizer.param_groups:
            param_group["weight_decay"] = (
                0.0 if param_group.get("hope_group") == "no_decay" else self.weight_decay
            )

    @torch.inference_mode()
    def _post_update_diagnostics(self) -> dict[str, float]:
        """Compute PPO health metrics from the retained rollout after the update.

        RolloutStorage.clear() only resets its write index; the tensors remain valid until the
        next collection.  This pass calls the actor/critic directly and updates no normalizer,
        optimizer, random state or task buffer.
        """
        storage = self.storage
        if self.policy.is_recurrent:
            return {}
        observations = storage.observations.flatten(0, 1)
        actions = storage.actions.flatten(0, 1)
        old_log_prob = storage.actions_log_prob.flatten(0, 1).squeeze(-1)
        old_mu = storage.mu.flatten(0, 1)
        old_sigma = storage.sigma.flatten(0, 1)
        returns = storage.returns.flatten(0, 1).squeeze(-1)
        total = int(actions.shape[0])
        if total == 0:
            return {}

        kl_sum = torch.zeros((), device=self.device)
        clipped_count = torch.zeros((), device=self.device)
        return_sum = torch.zeros((), device=self.device)
        return_sq_sum = torch.zeros((), device=self.device)
        residual_sum = torch.zeros((), device=self.device)
        residual_sq_sum = torch.zeros((), device=self.device)
        chunk_size = 16384
        for start in range(0, total, chunk_size):
            stop = min(start + chunk_size, total)
            obs_batch = observations[start:stop]
            actor_obs = self.policy.get_actor_obs(obs_batch)
            actor_obs = self.policy.actor_obs_normalizer(actor_obs)
            self.policy._update_distribution(actor_obs)
            new_mu = self.policy.action_mean
            new_sigma = self.policy.action_std
            kl = torch.sum(
                torch.log(new_sigma / old_sigma[start:stop] + 1.0e-5)
                + (
                    torch.square(old_sigma[start:stop])
                    + torch.square(old_mu[start:stop] - new_mu)
                )
                / (2.0 * torch.square(new_sigma))
                - 0.5,
                dim=-1,
            )
            kl_sum += kl.sum()
            new_log_prob = self.policy.get_actions_log_prob(actions[start:stop])
            ratio = torch.exp(new_log_prob - old_log_prob[start:stop])
            clipped_count += (
                torch.abs(ratio - 1.0) > float(self.clip_param)
            ).sum()

            new_values = self.policy.evaluate(obs_batch).squeeze(-1)
            return_batch = returns[start:stop]
            residual = return_batch - new_values
            return_sum += return_batch.sum()
            return_sq_sum += torch.square(return_batch).sum()
            residual_sum += residual.sum()
            residual_sq_sum += torch.square(residual).sum()

        count = float(total)
        return_variance = return_sq_sum / count - torch.square(return_sum / count)
        residual_variance = (
            residual_sq_sum / count - torch.square(residual_sum / count)
        )
        explained_variance = torch.where(
            return_variance > 1.0e-12,
            1.0 - residual_variance / return_variance,
            torch.zeros_like(return_variance),
        )
        packed = torch.stack(
            (
                kl_sum / count,
                clipped_count / count,
                explained_variance,
            )
        ).detach().float().cpu().tolist()
        return {
            "kl": float(packed[0]),
            "clip_fraction": float(packed[1]),
            "explained_variance": float(packed[2]),
        }

    def update(self) -> dict[str, float]:
        """Run the upstream PPO update and append behavior-neutral optimizer diagnostics."""
        original_clip_grad_norm = torch.nn.utils.clip_grad_norm_
        grad_norms = []

        def recorded_clip_grad_norm(*args, **kwargs):
            norm = original_clip_grad_norm(*args, **kwargs)
            grad_norms.append(norm.detach())
            return norm

        torch.nn.utils.clip_grad_norm_ = recorded_clip_grad_norm
        try:
            loss_dict = super().update()
        finally:
            torch.nn.utils.clip_grad_norm_ = original_clip_grad_norm

        if grad_norms:
            loss_dict["grad_norm"] = float(
                torch.stack(grad_norms).mean().detach().float().cpu()
            )
        try:
            loss_dict.update(self._post_update_diagnostics())
        except Exception as exc:
            if not getattr(self, "_diagnostic_warning_reported", False):
                print(
                    "[AdamWPPO] WARNING: post-update PPO diagnostics unavailable: "
                    f"{type(exc).__name__}: {exc}",
                    flush=True,
                )
                self._diagnostic_warning_reported = True
        return loss_dict


@configclass
class RslRlAdamWPpoAlgorithmCfg(RslRlPpoAlgorithmCfg):
    """Isaac Lab runner configuration for :class:`AdamWPPO`."""

    class_name: str = "AdamWPPO"
    weight_decay: float = 0.01


def register_with_rsl_rl_runner() -> None:
    """Expose the repository algorithm to rsl_rl's class-name based factory."""
    import rsl_rl.runners.on_policy_runner as runner_module

    runner_module.AdamWPPO = AdamWPPO
