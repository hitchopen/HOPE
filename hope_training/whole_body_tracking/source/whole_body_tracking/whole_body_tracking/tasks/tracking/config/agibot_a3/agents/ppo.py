"""Gym-registry PPO runner cfg classes for the Agibot A3 WBC tasks.

These back the registry ``rsl_rl_cfg_entry_point`` used by the legacy ``scripts/rsl_rl/train.py``.
The hyperparameters come from ``cfg/algo/ppo.yaml`` (the same file the Hydra ``scripts/train.py``
uses) via :mod:`whole_body_tracking.utils.ppo_cfg`, so there is one source of truth. Tune by
editing ``cfg/algo/ppo.yaml`` (or set ``WBT_AGIBOT_A3_PPO_CFG``).
"""

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg

from whole_body_tracking.utils.ppo_cfg import load_ppo_params, runner_kwargs

# NOTE: experiment_name is the logs/rsl_rl/<name>/ directory. It is hardcoded here for the legacy
# scripts/rsl_rl/train.py path and ALSO set in cfg/task/*.yaml (experiment_name:) for the Hydra
# scripts/train.py path. Keep the two in sync — "agibot_a3_flat" / "agibot_a3_hope" — or the two
# entry points will write checkpoints into different directories.
_KW = runner_kwargs(load_ppo_params(), "agibot_a3_flat")


@configclass
class AgibotA3FlatPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = _KW["num_steps_per_env"]
    max_iterations = _KW["max_iterations"]
    save_interval = _KW["save_interval"]
    experiment_name = _KW["experiment_name"]
    empirical_normalization = _KW["empirical_normalization"]
    policy = _KW["policy"]
    algorithm = _KW["algorithm"]


@configclass
class HOPEAgibotA3PPORunnerCfg(AgibotA3FlatPPORunnerCfg):
    experiment_name = "agibot_a3_hope"
