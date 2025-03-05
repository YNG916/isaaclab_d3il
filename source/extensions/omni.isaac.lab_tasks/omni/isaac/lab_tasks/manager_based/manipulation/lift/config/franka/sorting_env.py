# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import RigidObjectCfg, ArticulationCfg
from omni.isaac.lab.sensors import FrameTransformerCfg
from omni.isaac.lab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from omni.isaac.lab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from omni.isaac.lab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR

from omni.isaac.lab_tasks.manager_based.manipulation.lift import mdp
from omni.isaac.lab_tasks.manager_based.manipulation.lift.sorting_basic_cfg import SortingBasicEnvCfg

##
# Pre-defined configs
##
from omni.isaac.lab.markers.config import FRAME_MARKER_CFG  # isort: skip
from omni.isaac.lab_assets.franka import FRANKA_PANDA_CFG  # isort: skip


@configclass
class FrankaSortingBasicEnvCfg(SortingBasicEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set Franka as robot
        # self.scene.robot = FRANKA_PANDA_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot = FRANKA_PANDA_CFG.replace(
            prim_path="{ENV_REGEX_NS}/Robot",
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(-0.2, 0, 0.0),  # Set different initial position
                rot=(1.0, 0.0, 0.0, 0.0),  # Identity rotation
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
        # Set actions for the specific robot type (franka)
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=["panda_joint.*"], scale=0.5, use_default_offset=True
        )
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["panda_finger.*"],
            open_command_expr={"panda_finger_.*": 0.04},
            close_command_expr={"panda_finger_.*": 0.0},
        )
        # Set the body name for the end effector
        self.commands.object_pose.body_name = "panda_hand"

        # Rigid body properties of each cube
        cube_properties = RigidBodyPropertiesCfg(
            solver_position_iteration_count=16,
            solver_velocity_iteration_count=1,
            max_angular_velocity=1000.0,
            max_linear_velocity=1000.0,
            max_depenetration_velocity=5.0,
            disable_gravity=False,
        )

        # Set  Cubes as object  
        self.scene.object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object1",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.1, -0.1, 0.055], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/blue_block.usd",
                scale=(0.8, 0.8, 0.8),
                rigid_props=cube_properties
            ),
        )

        self.scene.object2 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object2",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.1, 0, 0.055], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/blue_block.usd",
                scale=(0.8, 0.8, 0.8),
                rigid_props=cube_properties
            ),
        )

        self.scene.object3 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object3",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.1, 0.1, 0.055], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/blue_block.usd",
                scale=(0.8, 0.8, 0.8),
                rigid_props=cube_properties
            ),
        )

        self.scene.object4 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object4",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.2, -0.1, 0.055], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/blue_block.usd",
                scale=(0.8, 0.8, 0.8),
                rigid_props=cube_properties
            ),
        )

        self.scene.object5 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object5",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.2, 0, 0.055], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/blue_block.usd",
                scale=(0.8, 0.8, 0.8),
                rigid_props=cube_properties
            ),
        )

        self.scene.object6 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object6",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.2, 0.1, 0.055], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/red_block.usd",
                scale=(0.8, 0.8, 0.8),
                rigid_props=cube_properties
            ),
        )   

        self.scene.object7 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object7",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.3, -0.1, 0.055], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/red_block.usd",
                scale=(0.8, 0.8, 0.8),
                rigid_props=cube_properties
            ),
        )   

        self.scene.object8 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object8",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.3, 0.0, 0.055], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/red_block.usd",
                scale=(0.8, 0.8, 0.8),
                rigid_props=cube_properties
            ),
        )   

        self.scene.object9 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object9",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.3, 0.1, 0.055], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/red_block.usd",
                scale=(0.8, 0.8, 0.8),
                rigid_props=cube_properties
            ),
        )   

        self.scene.object10 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object10",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.4, -0.1, 0.055], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/red_block.usd",
                scale=(0.8, 0.8, 0.8),
                rigid_props=cube_properties
            ),
        )   

        self.scene.object11 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object11",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.4, 0.0, 0.055], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/red_block.usd",
                scale=(0.8, 0.8, 0.8),
                rigid_props=cube_properties
            ),
        )  

        self.scene.object12 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object12",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.4, 0.1, 0.055], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/blue_block.usd",
                scale=(0.8, 0.8, 0.8),
                rigid_props=cube_properties
            ),
        )  


        #Set Cube as destination  
        # Class UsdFileCfg, spawning an asset from a USD file
        # Class CollisionPropertiesCfg, collision_enabled: Whether to enable or disable collisions. 
        self.scene.cube_Goal = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Cube_Goal",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.2, 0.5, 0.0], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/blue_block.usd",
                scale=(10.0, 10.0, 0.1),
                rigid_props=cube_properties,
                collision_props=sim_utils.CollisionPropertiesCfg(
                    collision_enabled=None,),
                #collision_enabled=False,
            ),
        )
        
        self.scene.cube_Goal_2 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Cube_Goal_2",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.2, -0.5, 0.0], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/red_block.usd",
                scale=(10.0, 10.0, 0.1),
                rigid_props=cube_properties,
                collision_props=sim_utils.CollisionPropertiesCfg(
                    collision_enabled=None,),
                #collision_enabled=False,
            ),
        )

        # self.scene.cube_Goal = RigidObjectCfg(
        #     prim_path="{ENV_REGEX_NS}/Cube_Goal",
        #     init_state=RigidObjectCfg.InitialStateCfg(pos=[0.2, 0.5, 0.0], rot=[1, 0, 0, 0]),
        #     spawn=UsdFileCfg(
        #         usd_path=f"/home/i53/student/jdu/Downloads/Kitchen_set/assets/SoapDish/SoapDish_payload.usd",
        #         rigid_props=cube_properties,
        #         collision_props=sim_utils.CollisionPropertiesCfg(
        #             collision_enabled=True,),
        #         #collision_enabled=False,
        #     ),
        # )
        
        # self.scene.cube_Goal_2 = RigidObjectCfg(
        #     prim_path="{ENV_REGEX_NS}/Cube_Goal_2",
        #     init_state=RigidObjectCfg.InitialStateCfg(pos=[0.2, -0.5, 0.0], rot=[1, 0, 0, 0]),
        #     spawn=UsdFileCfg(
        #         usd_path=f"/home/i53/student/jdu/Downloads/Kitchen_set/assets/SoapDish/SoapDish_payload.usd",
        #         rigid_props=cube_properties,
        #         collision_props=sim_utils.CollisionPropertiesCfg(
        #             collision_enabled=True,),
        #         #collision_enabled=False,
        #     ),
        # )


            # boxes for sorting
        # self.scene.box_1 = RigidObjectCfg(
        #         prim_path="{ENV_REGEX_NS}/Box",
        #         init_state=RigidObjectCfg.InitialStateCfg(pos=[0.3, 0, 0.0], rot=[1, 0, 0, 0]),
        #         spawn=UsdFileCfg(
        #             usd_path=f"/home/i53/student/jdu/Downloads/Kitchen_set/assets/PaperBagCrumpled/PaperBagCrumpled.usd",
        #             rigid_props=cube_properties,
        #             collision_props=sim_utils.CollisionPropertiesCfg(
        #                 collision_enabled=True,)
        #         ),
        #     )

        # self.scene.box_2 = RigidObjectCfg(
        #     prim_path="{ENV_REGEX_NS}/Box_2",
        #     init_state=RigidObjectCfg.InitialStateCfg(pos=[0.2, 0, 0.0], rot=[1, 0, 0, 0]),
        #     spawn=UsdFileCfg(
        #         usd_path=f"/home/i53/student/jdu/Downloads/Kitchen_set/assets/Spoon/Spoon.usd",
        #         rigid_props=cube_properties,
        #         collision_props=sim_utils.CollisionPropertiesCfg(
        #             collision_enabled=True,)
        #     ),
        # )

        # Listens to the required transforms
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/panda_link0",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/panda_hand",
                    name="end_effector",
                    offset=OffsetCfg(
                        pos=[0.0, 0.0, 0.1034],
                    ),
                ),
            ],
        )


@configclass
class FrankaSortingBasicEnvCfg_PLAY(FrankaSortingBasicEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
