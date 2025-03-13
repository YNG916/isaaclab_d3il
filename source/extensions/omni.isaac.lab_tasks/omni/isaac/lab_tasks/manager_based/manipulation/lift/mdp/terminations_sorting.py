# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Common functions that can be used to activate certain terminations for the lift task.

The functions can be passed to the :class:`omni.isaac.lab.managers.TerminationTermCfg` object to enable
the termination introduced by the function.
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from omni.isaac.lab.assets import Articulation, RigidObject
from omni.isaac.lab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from omni.isaac.lab.envs import ManagerBasedRLEnv


def cubes_sorted(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    object2_cfg: SceneEntityCfg = SceneEntityCfg("object2"),
    object3_cfg: SceneEntityCfg = SceneEntityCfg("object3"),
    object4_cfg: SceneEntityCfg = SceneEntityCfg("object4"),
    object5_cfg: SceneEntityCfg = SceneEntityCfg("object5"),
    object6_cfg: SceneEntityCfg = SceneEntityCfg("object6"),
    object7_cfg: SceneEntityCfg = SceneEntityCfg("object7"),
    object8_cfg: SceneEntityCfg = SceneEntityCfg("object8"),
    object9_cfg: SceneEntityCfg = SceneEntityCfg("object9"),
    object10_cfg: SceneEntityCfg = SceneEntityCfg("object10"),
    object11_cfg: SceneEntityCfg = SceneEntityCfg("object11"),
    object12_cfg: SceneEntityCfg = SceneEntityCfg("object12"),
    cube_Goal_cfg: SceneEntityCfg = SceneEntityCfg("cube_Goal"),
    cube_Goal_2_cfg: SceneEntityCfg = SceneEntityCfg("cube_Goal_2"),
    
    xy_threshold: float = 0.4,
    height_threshold: float = 0.005,
    height_diff: float = 0.06,
    gripper_open_val: torch.tensor = torch.tensor([0.04]),
    atol=0.0001,
    rtol=0.0001,
):
    robot: Articulation = env.scene[robot_cfg.name]
    object: RigidObject = env.scene[object_cfg.name]
    object2: RigidObject = env.scene[object2_cfg.name]
    object3: RigidObject = env.scene[object3_cfg.name]
    object4: RigidObject = env.scene[object4_cfg.name]
    object5: RigidObject = env.scene[object5_cfg.name]
    object6: RigidObject = env.scene[object6_cfg.name]
    object7: RigidObject = env.scene[object7_cfg.name]
    object8: RigidObject = env.scene[object8_cfg.name]
    object9: RigidObject = env.scene[object9_cfg.name]
    object10: RigidObject = env.scene[object10_cfg.name]
    object11: RigidObject = env.scene[object11_cfg.name]
    object12: RigidObject = env.scene[object12_cfg.name]
    cube_Goal: RigidObject = env.scene[cube_Goal_cfg.name]
    cube_Goal_2:RigidObject = env.scene[cube_Goal_2_cfg.name]
    pos_diff_c11 = object.data.root_pos_w - cube_Goal.data.root_pos_w
    pos_diff_c21 = object2.data.root_pos_w - cube_Goal.data.root_pos_w
    pos_diff_c31 = object3.data.root_pos_w - cube_Goal.data.root_pos_w
    pos_diff_c41 = object4.data.root_pos_w - cube_Goal.data.root_pos_w
    pos_diff_c51 = object5.data.root_pos_w - cube_Goal.data.root_pos_w
    pos_diff_c61 = object6.data.root_pos_w - cube_Goal.data.root_pos_w
    pos_diff_c72 = object7.data.root_pos_w - cube_Goal_2.data.root_pos_w
    pos_diff_c82 = object8.data.root_pos_w - cube_Goal_2.data.root_pos_w
    pos_diff_c92 = object9.data.root_pos_w - cube_Goal_2.data.root_pos_w
    pos_diff_c102 = object10.data.root_pos_w - cube_Goal_2.data.root_pos_w
    pos_diff_c112 = object11.data.root_pos_w - cube_Goal_2.data.root_pos_w
    pos_diff_c122 = object12.data.root_pos_w - cube_Goal_2.data.root_pos_w

    # Compute cube position difference to Goal in x-y plane
    xy_dist_c11 = torch.norm(pos_diff_c11[:, :2], dim=1)
    xy_dist_c21 = torch.norm(pos_diff_c21[:, :2], dim=1)
    xy_dist_c31 = torch.norm(pos_diff_c31[:, :2], dim=1)
    xy_dist_c41 = torch.norm(pos_diff_c41[:, :2], dim=1)
    xy_dist_c51 = torch.norm(pos_diff_c51[:, :2], dim=1)
    xy_dist_c61 = torch.norm(pos_diff_c61[:, :2], dim=1)
    xy_dist_c72 = torch.norm(pos_diff_c72[:, :2], dim=1)
    xy_dist_c82 = torch.norm(pos_diff_c82[:, :2], dim=1)
    xy_dist_c92 = torch.norm(pos_diff_c92[:, :2], dim=1)
    xy_dist_c102 = torch.norm(pos_diff_c102[:, :2], dim=1)
    xy_dist_c112 = torch.norm(pos_diff_c112[:, :2], dim=1)
    xy_dist_c122 = torch.norm(pos_diff_c122[:, :2], dim=1)

    # Compute cube height difference to Goal
    h_dist_c11 = torch.norm(pos_diff_c11[:, 2:], dim=1)
    h_dist_c21 = torch.norm(pos_diff_c21[:, 2:], dim=1)
    h_dist_c31 = torch.norm(pos_diff_c31[:, 2:], dim=1)
    h_dist_c41 = torch.norm(pos_diff_c41[:, 2:], dim=1)
    h_dist_c51 = torch.norm(pos_diff_c51[:, 2:], dim=1)
    h_dist_c61 = torch.norm(pos_diff_c61[:, 2:], dim=1)
    h_dist_c72 = torch.norm(pos_diff_c72[:, 2:], dim=1)
    h_dist_c82 = torch.norm(pos_diff_c82[:, 2:], dim=1)
    h_dist_c92 = torch.norm(pos_diff_c92[:, 2:], dim=1)
    h_dist_c102 = torch.norm(pos_diff_c102[:, 2:], dim=1)
    h_dist_c112 = torch.norm(pos_diff_c112[:, 2:], dim=1)
    h_dist_c122 = torch.norm(pos_diff_c122[:, 2:], dim=1)
    

    # Check cube positions
    sorted = torch.logical_and(xy_dist_c11 < xy_threshold, xy_dist_c21 < xy_threshold)
    sorted = torch.logical_and(xy_dist_c31 < xy_threshold, sorted)
    sorted = torch.logical_and(xy_dist_c41 < xy_threshold, sorted)
    sorted = torch.logical_and(xy_dist_c51 < xy_threshold, sorted)
    sorted = torch.logical_and(xy_dist_c61 < xy_threshold, sorted)
    sorted = torch.logical_and(xy_dist_c72 < xy_threshold, sorted)
    sorted = torch.logical_and(xy_dist_c82 < xy_threshold, sorted)
    sorted = torch.logical_and(xy_dist_c92 < xy_threshold, sorted)
    sorted = torch.logical_and(xy_dist_c102 < xy_threshold, sorted)
    sorted = torch.logical_and(xy_dist_c112 < xy_threshold, sorted)
    sorted = torch.logical_and(xy_dist_c122 < xy_threshold, sorted)    
    sorted = torch.logical_and(h_dist_c11 - height_diff < height_threshold, sorted)
    sorted = torch.logical_and(h_dist_c21 - height_diff < height_threshold, sorted)
    sorted = torch.logical_and(h_dist_c31 - height_diff < height_threshold, sorted)
    sorted = torch.logical_and(h_dist_c41 - height_diff < height_threshold, sorted)
    sorted = torch.logical_and(h_dist_c51 - height_diff < height_threshold, sorted)
    sorted = torch.logical_and(h_dist_c61 - height_diff < height_threshold, sorted)
    sorted = torch.logical_and(h_dist_c72 - height_diff < height_threshold, sorted)
    sorted = torch.logical_and(h_dist_c82 - height_diff < height_threshold, sorted)
    sorted = torch.logical_and(h_dist_c92 - height_diff < height_threshold, sorted)
    sorted = torch.logical_and(h_dist_c102 - height_diff < height_threshold, sorted)
    sorted = torch.logical_and(h_dist_c112 - height_diff < height_threshold, sorted)
    sorted = torch.logical_and(h_dist_c122 - height_diff < height_threshold, sorted)

    # Check gripper positions
    sorted = torch.logical_and(
        torch.isclose(robot.data.joint_pos[:, -1], gripper_open_val.to(env.device), atol=atol, rtol=rtol), sorted
    )
    sorted = torch.logical_and(
        torch.isclose(robot.data.joint_pos[:, -2], gripper_open_val.to(env.device), atol=atol, rtol=rtol), sorted
    )

    return sorted
