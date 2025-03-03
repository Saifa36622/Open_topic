import gymnasium as gym

from . import agents, test1_env_cfg

##
# Register Gym environments.
##

gym.register(
    id="Template-Isaac-Actuator-Robot-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": test1_env_cfg.robottest1EnvCfg,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:robottest1PPORunnerCfg",
    },
)

gym.register(
    id="Template-Isaac-Actuator-Robot-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": test1_env_cfg.robottest1EnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:robottest1PPORunnerCfg",
    },
)