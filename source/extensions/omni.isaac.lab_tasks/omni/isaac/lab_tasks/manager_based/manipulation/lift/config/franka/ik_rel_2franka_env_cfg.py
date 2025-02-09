from omni.isaac.lab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from omni.isaac.lab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from omni.isaac.lab.utils import configclass

#from . import joint_pos_env_cfg
from . import joint_pos_2franka_lift_env_cfg

##
# Pre-defined configs
##
from omni.isaac.lab_assets.franka import FRANKA_PANDA_HIGH_PD_CFG  # isort: skip


@configclass
class ik_twoFrankaCubeEnvCfg(joint_pos_2franka_lift_env_cfg.TwoFrankaCubeEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        """ # Set Franka as robot
        # We switch here to a stiffer PD controller for IK tracking to be better.
        self.scene.robot = FRANKA_PANDA_HIGH_PD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Set actions for the specific robot type (franka)
        self.actions.arm_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=["panda_joint.*"],
            body_name="panda_hand",
            controller=DifferentialIKControllerCfg(command_type="pose", use_relative_mode=True, ik_method="dls"),
            scale=0.5,
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=[0.0, 0.0, 0.107]),
        ) """
        # 设置两个 Franka 机器人
        self.scene.robots = [
            FRANKA_PANDA_HIGH_PD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot_0"),
            FRANKA_PANDA_HIGH_PD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot_1"),
        ]

        # 为两个 Franka 机器人分别设置 IK 控制
        self.actions.arm_action = [
            DifferentialInverseKinematicsActionCfg(
                asset_name="robot_0",
                joint_names=["panda_joint.*"],
                body_name="panda_hand_0",
                controller=DifferentialIKControllerCfg(
                    command_type="pose", use_relative_mode=True, ik_method="dls"
                ),
                scale=0.5,
                body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(
                    pos=[0.0, 0.0, 0.107]
                ),
            ),
            DifferentialInverseKinematicsActionCfg(
                asset_name="robot_1",
                joint_names=["panda_joint.*"],
                body_name="panda_hand_1",
                controller=DifferentialIKControllerCfg(
                    command_type="pose", use_relative_mode=True, ik_method="dls"
                ),
                scale=0.5,
                body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(
                    pos=[0.0, 0.0, 0.107]
                ),
            ),
        ]


@configclass
class ik_twoFrankaCubeEnvCfg_PLAY(ik_twoFrankaCubeEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False