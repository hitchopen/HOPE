import gymnasium as gym

from . import agents, hope_env_cfg

##
# Register the single public HOPE task.
##
gym.register(
    id="HOPE-PingPong-AgibotA3-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": hope_env_cfg.HOPEPingPongEnvCfg,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.ppo:HOPEPingPongPPORunnerCfg",
    },
)
