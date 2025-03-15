from omni.isaac.lab.sim import sim as sim_utils
from omni.isaac.lab.assets import RigidObjectCfg
from omni.isaac.lab.assets import ArticulationCfg
from omni.isaac.lab.sensors import FrameTransformerCfg
from omni.isaac.lab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from omni.isaac.lab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from omni.isaac.lab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR

from omni.isaac.lab_tasks.manager_based.manipulation.lift import mdp
from omni.isaac.lab_tasks.manager_based.manipulation.lift.parallel_basic_cfg import ParallelEnvCfg

##
# Pre-defined configs
##
from omni.isaac.lab.markers.config import FRAME_MARKER_CFG  # isort: skip
from omni.isaac.lab_assets.franka import FRANKA_PANDA_CFG, FRANKA_PANDA_HIGH_PD_CFG  # isort: skip

# 为 IK 控制导入相关配置
from omni.isaac.lab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from omni.isaac.lab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg


@configclass
class ParallelDualFrankaIKEnvCfg(ParallelEnvCfg):
    def __post_init__(self):
        # 调用父类后初始化其他配置
        super().__post_init__()

        # 设置左右两台机械臂均采用逆运动学对应的高刚度 PD 控制配置
        self.scene.robot = FRANKA_PANDA_HIGH_PD_CFG.replace(
            prim_path="{ENV_REGEX_NS}/Robot",
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(-0.7, -0.5, 0.0),  # 左侧机械臂初始位置
                rot=(1.0, 0.0, 0.0, 0.0),  # 单位旋转
                joint_pos={
                    "panda_joint1": 0.0,
                    "panda_joint2": -0.569,
                    "panda_joint3": 0.0,
                    "panda_joint4": -2.810,
                    "panda_joint5": 0.0,
                    "panda_joint6": 3.037,
                    "panda_joint7": 0.741,
                    "panda_finger_joint.*": 0.04,
                },
            )
        )
        self.scene.robot_1 = FRANKA_PANDA_HIGH_PD_CFG.replace(
            prim_path="{ENV_REGEX_NS}/Robot_1",
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(-0.7, 0.5, 0.0),  # 右侧机械臂初始位置
                rot=(1.0, 0.0, 0.0, 0.0),
                joint_pos={
                    "panda_joint1": 0.0,
                    "panda_joint2": -0.569,
                    "panda_joint3": 0.0,
                    "panda_joint4": -2.810,
                    "panda_joint5": 0.0,
                    "panda_joint6": 3.037,
                    "panda_joint7": 0.741,
                    "panda_finger_joint.*": 0.04,
                },
            )
        )

        # 设置左右臂的逆运动学动作配置
        self.actions.arm_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=["panda_joint.*"],
            body_name="panda_hand",  # 注意：此处名称需与机器人资源中末端执行器名称一致
            controller=DifferentialIKControllerCfg(
                command_type="pose", use_relative_mode=True, ik_method="dls"
            ),
            scale=0.5,
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=[0.0, 0.0, 0.107]),
        )
        
        self.actions.arm_action_1 = DifferentialInverseKinematicsActionCfg(
            asset_name="robot_1",
            joint_names=["panda_joint.*"],
            body_name="panda_hand",  # 如果右侧机器人末端执行器名称相同，也可用相同名称
            controller=DifferentialIKControllerCfg(
                command_type="pose", use_relative_mode=True, ik_method="dls"
            ),
            scale=0.5,
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=[0.0, 0.0, 0.107]),
        )


@configclass
class ParallelDualFrankaIKEnvCfg_PLAY(ParallelDualFrankaIKEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        # 设置较小的环境用于演示
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # 禁用观察随机扰动
        self.observations.policy.enable_corruption = False
