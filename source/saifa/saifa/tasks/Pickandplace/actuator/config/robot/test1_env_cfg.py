from isaaclab.utils import configclass

from saifa.tasks.Pickandplace.actuator.actuator_env_cfg import ActuatorPickandplacetest1EnvCfg

from isaaclab_assets import UR10_CFG  # isort: skip

import math

@configclass
class robottest1EnvCfg(ActuatorPickandplacetest1EnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        self.scene.robot = UR10_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")


@configclass
class robottest1EnvCfg_PLAY(robottest1EnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5

        self.observations.policy.enable_corruption = False