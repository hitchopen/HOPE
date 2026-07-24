import gymnasium as gym

from . import agents, hope_env_cfg

##
# Register the G1 HOPE task (parallel to the A3 task).
##
gym.register(
    id="HOPE-PingPong-UnitreeG1-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": hope_env_cfg.G1HOPEPingPongEnvCfg,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.ppo:G1HOPEPingPongPPORunnerCfg",
    },
)
