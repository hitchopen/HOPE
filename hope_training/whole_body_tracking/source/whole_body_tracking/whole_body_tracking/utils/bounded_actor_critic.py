"""Deploy-action-aware exploration for the upstream rsl_rl actor-critic."""

from __future__ import annotations

import math

import torch
from torch.distributions import Normal

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlPpoActorCriticCfg
from rsl_rl.modules import ActorCritic


def exploration_contract_for_transform(action_transform: str) -> str:
    """Return the checkpoint contract for the action executed by the task."""
    transform = str(action_transform).lower()
    if transform not in {"identity", "tanh"}:
        raise ValueError(
            "entropy_action_transform must be 'identity' or 'tanh', "
            f"got {action_transform!r}"
        )
    return f"projected_physical_std_{transform}_entropy_v1"


class BoundedStdActorCritic(ActorCritic):
    """Optimize action std inside YAML-owned bounds and score executable-channel entropy.

    Legacy V15-v3 uses change-of-variables entropy for its explicit ``tanh(raw_action)`` dynamic
    interval decoder.  V15-v4 uses ordinary Gaussian entropy because each actor output denotes an
    absolute posture request; its separate safe tanh is part of the q_des scale mapping rather than
    the action-space contract optimized by PPO.

    ``self.std`` remains a physical standard deviation.  AdamW projects it back into the YAML
    interval after every optimizer step, so a parameter cannot escape beyond the bound while the
    distribution still has a valid inward task gradient at either boundary.  Some A3 outputs are
    structurally passive; those columns remain in the 31-D actor/export contract but are excluded
    from PPO's entropy bonus and held at zero in sampled/deterministic actions.
    """

    def __init__(
        self,
        *args,
        min_noise_std: float = 0.1,
        max_noise_std: float = 1.2,
        entropy_excluded_action_indices: tuple[int, ...] | list[int] = (),
        entropy_action_transform: str = "identity",
        reset_noise_std_on_warm_start: bool = False,
        **kwargs,
    ):
        self.min_noise_std = float(min_noise_std)
        self.max_noise_std = float(max_noise_std)
        self.entropy_action_transform = str(entropy_action_transform).lower()
        self.exploration_contract = exploration_contract_for_transform(
            self.entropy_action_transform
        )
        self.reset_noise_std_on_warm_start = bool(reset_noise_std_on_warm_start)
        super().__init__(*args, **kwargs)

        # Preserve the upstream checkpoint key and semantics: ``std`` is the physical Gaussian
        # scale.  The persistent optimizer projection is installed by AdamWPPO.
        initial_physical_std = self.std.detach().clone()
        self.register_buffer("_initial_physical_std", initial_physical_std, persistent=False)

        excluded = tuple(sorted({int(index) for index in entropy_excluded_action_indices}))
        active_mask = torch.ones(self.std.numel(), dtype=torch.bool, device=self.std.device)
        if excluded:
            active_mask[list(excluded)] = False
        self.register_buffer("_entropy_active_action_mask", active_mask, persistent=False)
        self.entropy_excluded_action_indices = excluded
        self.project_noise_std_()
        print(
            "[BoundedStdActorCritic] projected physical std ACTIVE: "
            f"min_std={self.min_noise_std} max_std={self.max_noise_std} "
            f"entropy_actions={int(active_mask.sum())}/{active_mask.numel()} "
            f"excluded={list(excluded)} action_transform={self.entropy_action_transform} "
            f"contract={self.exploration_contract}",
            flush=True,
        )

    def _update_distribution(self, obs) -> None:
        # ``ppo.yaml`` selects the upstream state-independent scalar-noise policy.
        actor_mean = self.actor(obs)
        active_mask = self._entropy_active_action_mask
        mean = torch.where(active_mask, actor_mean, torch.zeros_like(actor_mean))
        physical_std = torch.where(active_mask, self.effective_noise_std, self._initial_physical_std)
        self.distribution = Normal(
            mean,
            physical_std.expand_as(mean),
        )

    def act(self, obs, **kwargs) -> torch.Tensor:
        """Sample only executable channels; passive raw actions stay exactly zero."""
        actions = super().act(obs, **kwargs)
        return torch.where(
            self._entropy_active_action_mask,
            actions,
            torch.zeros_like(actions),
        )

    def act_inference(self, obs) -> torch.Tensor:
        """Keep deterministic simulation/evaluation on the same passive-channel contract."""
        actions = super().act_inference(obs)
        return torch.where(
            self._entropy_active_action_mask,
            actions,
            torch.zeros_like(actions),
        )

    @torch.no_grad()
    def project_noise_std_(self) -> None:
        """Project the learned physical std after an optimizer step."""
        self.std.clamp_(min=self.min_noise_std, max=self.max_noise_std)

    @torch.no_grad()
    def reset_noise_std_(self) -> None:
        """Reset a deliberate warm-start to ppo.yaml's physical initial std."""
        self.std.copy_(self._initial_physical_std)
        self.project_noise_std_()

    @property
    def effective_noise_std(self) -> torch.Tensor:
        """Physical action std used by executable channels."""
        return self.std

    @property
    def executable_gaussian_entropy(self) -> torch.Tensor:
        """Pre-tanh Gaussian entropy, retained only as a diagnostic."""
        return self.distribution.entropy()[..., self._entropy_active_action_mask].sum(dim=-1)

    @property
    def entropy(self) -> torch.Tensor:
        """Entropy of the active normalized action actually decoded by the task.

        For ``u=tanh(x)``, ``H(u)=H(x)+E[log(1-tanh(x)^2)]``.  ``rsample`` keeps this estimator
        differentiable with respect to both actor mean and physical std.  PPO minibatches contain
        enough samples that one reparameterized draw per state is a low-noise estimate.  Tasks
        without a tanh action decoder retain the upstream Gaussian entropy.
        """
        if self.entropy_action_transform == "identity":
            return self.executable_gaussian_entropy
        raw_sample = self.distribution.rsample()
        # Numerically stable log(1 - tanh(x)^2), used by tanh-squashed Gaussian policies.
        log_abs_det_jacobian = 2.0 * (
            math.log(2.0)
            - raw_sample
            - torch.nn.functional.softplus(-2.0 * raw_sample)
        )
        entropy_per_action = self.distribution.entropy() + log_abs_det_jacobian
        return entropy_per_action[..., self._entropy_active_action_mask].sum(dim=-1)

    @property
    def all_action_gaussian_entropy(self) -> torch.Tensor:
        """Diagnostic pre-tanh entropy before passive channels are removed."""
        return self.distribution.entropy().sum(dim=-1)


@configclass
class RslRlBoundedStdActorCriticCfg(RslRlPpoActorCriticCfg):
    """Isaac Lab runner config for :class:`BoundedStdActorCritic`."""

    class_name: str = "BoundedStdActorCritic"
    min_noise_std: float = 0.1
    max_noise_std: float = 1.2
    entropy_excluded_action_indices: list[int] = []
    entropy_action_transform: str = "identity"
    reset_noise_std_on_warm_start: bool = False


def register_with_rsl_rl_runner() -> None:
    """Expose the repository policy to rsl_rl's class-name based runner factory."""
    import rsl_rl.runners.on_policy_runner as runner_module

    runner_module.BoundedStdActorCritic = BoundedStdActorCritic
