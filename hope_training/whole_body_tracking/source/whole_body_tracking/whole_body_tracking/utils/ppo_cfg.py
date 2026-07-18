"""Single source of truth for the Agibot A3 PPO runner cfg, built from ``cfg/algo/ppo.yaml``.

The PPO hyperparameters live in ``cfg/algo/ppo.yaml`` at the training-repo root (the file the user
tunes). Two places build the rsl_rl runner cfg from it via :func:`runner_kwargs`, so they never drift:

* the Hydra entry ``scripts/train.py`` (passes the Hydra-merged ``cfg.algo`` dict), and
* the gym task's runner-cfg class ``tasks/tracking/config/agibot_a3/agents/ppo.py`` (builds the runner
  cfg exposed as the task's ``rsl_rl_cfg_entry_point``; loads the YAML via :func:`load_ppo_params`).

Set ``WBT_PPO_CFG=/abs/path.yaml`` to point at a different file.
"""

from __future__ import annotations

import os

import yaml

from isaaclab_rl.rsl_rl import RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg


def find_ppo_yaml() -> str | None:
    """Locate cfg/algo/ppo.yaml: env-var override, else walk up from this file to the repo root."""
    env = os.environ.get("WBT_PPO_CFG")
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
            "Could not locate cfg/algo/ppo.yaml (training-repo root). Set WBT_PPO_CFG to its "
            "absolute path."
        )
    with open(path) as f:
        return yaml.safe_load(f)


def runner_kwargs(params: dict, experiment_name: str) -> dict:
    """Map a parsed ppo.yaml dict to RslRlOnPolicyRunnerCfg constructor kwargs."""
    r, p, a = params["runner"], params["policy"], params["algorithm"]
    return dict(
        num_steps_per_env=int(r["num_steps_per_env"]),
        max_iterations=int(r["max_iterations"]),
        save_interval=int(r["save_interval"]),
        experiment_name=experiment_name,
        empirical_normalization=bool(r["empirical_normalization"]),
        policy=RslRlPpoActorCriticCfg(
            init_noise_std=float(p["init_noise_std"]),
            actor_hidden_dims=[int(x) for x in p["actor_hidden_dims"]],
            critic_hidden_dims=[int(x) for x in p["critic_hidden_dims"]],
            activation=str(p["activation"]),
        ),
        algorithm=RslRlPpoAlgorithmCfg(
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
        ),
    )
