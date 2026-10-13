"""The single A3 training task corresponding to model_14600."""

import gymnasium as gym

from . import agents, hope_env_cfg

gym.register(
    id="HOPE-HitterPingPong-AgibotA3-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": hope_env_cfg.HitterPingPongAgibotA3EnvCfg,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.ppo:HOPEAgibotA3PPORunnerCfg",
    },
)
