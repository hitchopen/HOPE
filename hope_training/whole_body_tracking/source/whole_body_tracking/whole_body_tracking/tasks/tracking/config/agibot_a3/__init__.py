"""Official Agibot A3 environment registrations for the Build branch."""

import gymnasium as gym

from . import agents, flat_env_cfg, hope_env_cfg


# Common non-ping-pong A3 motion-tracking baseline retained for robot/asset diagnostics.
gym.register(
    id="Tracking-Flat-AgibotA3-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": flat_env_cfg.AgibotA3FlatEnvCfg,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.ppo:AgibotA3FlatPPORunnerCfg",
    },
)


# The only formal A3 ping-pong task on this branch. Historical task YAMLs/classes remain in the
# repository solely for provenance and regression comparison; they are intentionally unregistered.
gym.register(
    id="HOPE-HitterPingPong-AgibotA3-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": hope_env_cfg.HitterPingPongAgibotA3EnvCfg,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.ppo:HOPEAgibotA3PPORunnerCfg",
    },
)
