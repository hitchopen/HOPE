"""Single source of truth for the Agibot A3 PPO runner cfg, built from ``cfg/algo/ppo.yaml``.

The PPO hyperparameters live in ``cfg/algo/ppo.yaml`` at the training-repo root (the file the user
tunes, 106B-Final-Project style). Two places build the rsl_rl runner cfg from it via
:func:`runner_kwargs`, so they never drift:

* the Hydra entry ``scripts/train.py`` (passes the Hydra-merged ``cfg.algo`` dict), and
* the gym-registry class ``config/.../agents/ppo.py`` (used by the legacy ``scripts/rsl_rl/train.py``;
  loads the YAML via :func:`load_ppo_params`).

Set ``WBT_AGIBOT_A3_PPO_CFG=/abs/path.yaml`` to point at a different file.
"""

import os
from dataclasses import fields

import yaml

from isaaclab_rl.rsl_rl import RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg

from whole_body_tracking.utils.adamw_ppo import (
    RslRlAdamWPpoAlgorithmCfg,
    register_with_rsl_rl_runner as register_adamw_ppo,
)
from whole_body_tracking.utils.bounded_actor_critic import (
    RslRlBoundedStdActorCriticCfg,
    register_with_rsl_rl_runner as register_bounded_actor_critic,
)
from whole_body_tracking.utils.projection_ppo import (
    RslRlProjectionPpoAlgorithmCfg,
    register_with_rsl_rl_runner as register_projection_ppo,
)


# rsl_rl resolves the policy through ``eval(class_name)`` in its runner module. Register the
# repository-owned implementation whenever this shared YAML loader is imported; every HOPE
# train/play/eval entry point imports this module before constructing its runner.
register_bounded_actor_critic()
register_adamw_ppo()
register_projection_ppo()


def find_ppo_yaml() -> str | None:
    """Locate cfg/algo/ppo.yaml: env-var override, else walk up from this file to the repo root."""
    env = os.environ.get("WBT_AGIBOT_A3_PPO_CFG")
    if env:
        return env
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(12):
        cand = os.path.join(d, "cfg", "algo", "ppo.yaml")
        if os.path.isfile(cand):
            return cand
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return None


def load_ppo_params(path: str | None = None) -> dict:
    """Load the PPO hyperparameter dict from ppo.yaml."""
    path = path or find_ppo_yaml()
    if path is None or not os.path.isfile(path):
        raise FileNotFoundError(
            "Could not locate cfg/algo/ppo.yaml (training-repo root). "
            "Set WBT_AGIBOT_A3_PPO_CFG to its absolute path."
        )
    with open(path) as f:
        return yaml.safe_load(f)


def bind_optimizer(algorithm_cfg: RslRlPpoAlgorithmCfg, params: dict):
    """Bind a resolved PPO cfg to the optimizer selected by the shared YAML.

    Exact resume freezes the original agent pickle.  Rebinding only the optimizer class and its
    weight decay lets an intentional Adam-to-AdamW repair keep every other saved PPO value.
    """
    algorithm_values = {
        field.name: getattr(algorithm_cfg, field.name)
        for field in fields(RslRlPpoAlgorithmCfg)
        if field.name != "class_name"
    }
    optimizer_cfg = params["algorithm"]
    if str(optimizer_cfg.get("optimizer", "adam")).lower() == "adamw":
        return RslRlAdamWPpoAlgorithmCfg(
            weight_decay=float(optimizer_cfg["weight_decay"]),
            **algorithm_values,
        )
    return RslRlPpoAlgorithmCfg(**algorithm_values)


def bind_projection_auxiliary(
    algorithm_cfg: RslRlPpoAlgorithmCfg,
    *,
    enabled: bool,
    coefficient: float,
    coefficient_max: float,
    gradient_ratio_max: float,
):
    """Select repository-owned ProjectionPPO without changing base PPO hyperparameters."""

    algorithm_values = {
        field.name: getattr(algorithm_cfg, field.name)
        for field in fields(RslRlPpoAlgorithmCfg)
        if field.name != "class_name"
    }
    if not enabled:
        return RslRlPpoAlgorithmCfg(**algorithm_values)
    return RslRlProjectionPpoAlgorithmCfg(
        projection_aux_enabled=True,
        projection_aux_lambda=float(coefficient),
        projection_aux_lambda_max=float(coefficient_max),
        projection_aux_gradient_ratio_max=float(gradient_ratio_max),
        **algorithm_values,
    )


def runner_kwargs(params: dict, experiment_name: str) -> dict:
    """Map a parsed ppo.yaml dict to RslRlOnPolicyRunnerCfg constructor kwargs."""
    r, p, a = params["runner"], params["policy"], params["algorithm"]
    algorithm_kwargs = dict(
        value_loss_coef=float(a["value_loss_coef"]),
        use_clipped_value_loss=bool(a["use_clipped_value_loss"]),
        clip_param=float(a["clip_param"]),
        entropy_coef=float(a["entropy_coef"]),
        num_learning_epochs=int(a["num_learning_epochs"]),
        num_mini_batches=int(a["num_mini_batches"]),
        learning_rate=float(a["learning_rate"]),
        schedule=str(a["schedule"]),
        gamma=float(a["gamma"]),
        lam=float(a["lam"]),
        desired_kl=float(a["desired_kl"]),
        max_grad_norm=float(a["max_grad_norm"]),
    )
    if str(a.get("optimizer", "adam")).lower() == "adamw":
        algorithm_cfg = RslRlAdamWPpoAlgorithmCfg(
            weight_decay=float(a["weight_decay"]),
            **algorithm_kwargs,
        )
    else:
        algorithm_cfg = RslRlPpoAlgorithmCfg(**algorithm_kwargs)

    common_policy_kwargs = dict(
        init_noise_std=float(p["init_noise_std"]),
        actor_hidden_dims=[int(x) for x in p["actor_hidden_dims"]],
        critic_hidden_dims=[int(x) for x in p["critic_hidden_dims"]],
        activation=str(p["activation"]),
    )
    has_min_noise = "min_noise_std" in p
    has_max_noise = "max_noise_std" in p
    if has_min_noise != has_max_noise:
        raise ValueError(
            "policy.min_noise_std and policy.max_noise_std must either both be "
            "present (bounded actor) or both be absent (legacy ActorCritic)"
        )
    if has_min_noise:
        policy_cfg = RslRlBoundedStdActorCriticCfg(
            min_noise_std=float(p["min_noise_std"]),
            max_noise_std=float(p["max_noise_std"]),
            entropy_action_transform=str(
                p.get("entropy_action_transform", "identity")
            ),
            reset_noise_std_on_warm_start=bool(
                p.get("reset_noise_std_on_warm_start", False)
            ),
            **common_policy_kwargs,
        )
    else:
        unsupported = {
            key: p[key]
            for key in (
                "entropy_action_transform",
                "reset_noise_std_on_warm_start",
            )
            if key in p
        }
        if unsupported:
            raise ValueError(
                "legacy ActorCritic cannot consume bounded-actor-only policy "
                f"fields: {unsupported}"
            )
        policy_cfg = RslRlPpoActorCriticCfg(**common_policy_kwargs)

    return dict(
        num_steps_per_env=int(r["num_steps_per_env"]),
        max_iterations=int(r["max_iterations"]),
        save_interval=int(r["save_interval"]),
        experiment_name=experiment_name,
        empirical_normalization=bool(r["empirical_normalization"]),
        policy=policy_cfg,
        algorithm=algorithm_cfg,
    )
