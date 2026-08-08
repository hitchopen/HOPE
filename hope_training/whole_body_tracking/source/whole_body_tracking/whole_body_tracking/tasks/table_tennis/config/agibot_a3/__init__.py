import gymnasium as gym

from . import table_tennis_env_cfg

##
# Register the Agibot A3 table-tennis match environment.
##

gym.register(
    id="HOPE-TableTennis-AgibotA3-v0",
    entry_point="whole_body_tracking.tasks.table_tennis.table_tennis_env:TableTennisEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": table_tennis_env_cfg.AgibotA3TableTennisEnvCfg,
    },
)
