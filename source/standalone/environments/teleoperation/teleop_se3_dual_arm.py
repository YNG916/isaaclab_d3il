# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to run a teleoperation with Isaac Lab manipulation environments,
using keyboard for one manipulator and gamepad for the other."""

import argparse
import os

from isaaclab.app import AppLauncher

# 添加命令行参数
parser = argparse.ArgumentParser(
    description="Teleoperation for Isaac Lab environments with dual-arm control: "
                "keyboard controls one arm and gamepad controls the other."
)
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--teleop_device", type=str, default="keyboard", help="Device for interacting with environment")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--sensitivity", type=float, default=1.0, help="Sensitivity factor.")
# 开启双臂控制
parser.add_argument("--dual_arm", action="store_true", default=True, help="Enable dual-arm control.")
# 追加 AppLauncher 参数
AppLauncher.add_app_launcher_args(parser)
# 解析参数
args_cli = parser.parse_args()

app_launcher_args = vars(args_cli)
# 当手部跟踪设备使用时，指定体验
if args_cli.teleop_device.lower() == "handtracking":
    app_launcher_args["experience"] = f'{os.environ["ISAACLAB_PATH"]}/apps/isaaclab.python.xr.openxr.kit'
# 启动 Omniverse 应用
app_launcher = AppLauncher(app_launcher_args)
simulation_app = app_launcher.app

import gymnasium as gym
import torch

import omni.log

from isaaclab.devices import Se3Gamepad, Se3Keyboard, Se3HandTracking, Se3SpaceMouse
from isaaclab.envs import ViewerCfg
from isaaclab.envs.ui import ViewportCameraController
from isaaclab.managers import TerminationTermCfg as DoneTerm

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.manager_based.manipulation.lift import mdp
from isaaclab_tasks.utils import parse_env_cfg


def pre_process_actions(delta_pose: torch.Tensor, gripper_command: bool) -> torch.Tensor:
    """预处理单个机械臂的动作指令"""
    if "Reach" in args_cli.task:
        return delta_pose
    else:
        gripper_vel = torch.zeros(delta_pose.shape[0], 1, device=delta_pose.device)
        gripper_vel[:] = -1.0 if gripper_command else 1.0
        return torch.concat([delta_pose, gripper_vel], dim=1)


def main():
    """运行使用键盘和 gamepad 分别控制双机械臂的遥操作程序"""
    # 解析环境配置，假设环境支持双臂控制，例如通过设置 env_cfg.num_arms=2
    env_cfg = parse_env_cfg(
        args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs, use_fabric=not args_cli.disable_fabric
    )
    if args_cli.dual_arm:
        env_cfg.num_arms = 2

    env_cfg.terminations.time_out = None
    if "Lift" in args_cli.task:
        env_cfg.commands.object_pose.resampling_time_range = (1.0e9, 1.0e9)
        env_cfg.terminations.object_reached_goal = DoneTerm(func=mdp.object_reached_goal)
    # 创建环境
    env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
    if "Reach" in args_cli.task:
        omni.log.warn(
            f"The environment '{args_cli.task}' does not support gripper control. The device command will be ignored."
        )

    # 创建控制器
    # 左侧机械臂使用键盘控制，右侧机械臂使用 gamepad 控制
    if args_cli.dual_arm:
        teleop_interface_left = Se3Keyboard(
            pos_sensitivity=0.05 * args_cli.sensitivity,
            rot_sensitivity=0.05 * args_cli.sensitivity
        )
        teleop_interface_right = Se3Gamepad(
            pos_sensitivity=0.1 * args_cli.sensitivity,
            rot_sensitivity=0.1 * args_cli.sensitivity,
            gamepad_index=0  # 使用默认的 gamepad 设备
        )
    else:
        # 单臂控制模式（按需使用）
        teleop_interface = Se3Keyboard(
            pos_sensitivity=0.05 * args_cli.sensitivity,
            rot_sensitivity=0.05 * args_cli.sensitivity
        )

    # 添加重置回调
    should_reset_recording_instance = False

    def reset_recording_instance():
        nonlocal should_reset_recording_instance
        should_reset_recording_instance = True

    if args_cli.dual_arm:
        teleop_interface_left.add_callback("R", reset_recording_instance)
        teleop_interface_right.add_callback("R", reset_recording_instance)
    else:
        teleop_interface.add_callback("R", reset_recording_instance)

    # 打印控制器信息
    if args_cli.dual_arm:
        print("Keyboard-controlled arm:", teleop_interface_left)
        print("Gamepad-controlled arm:", teleop_interface_right)
    else:
        print(teleop_interface)

    # 重置环境和控制器
    env.reset()
    if args_cli.dual_arm:
        teleop_interface_left.reset()
        teleop_interface_right.reset()
    else:
        teleop_interface.reset()

    # 主仿真循环
    while simulation_app.is_running():
        with torch.inference_mode():
            if args_cli.dual_arm:
                # 分别获取两个控制器的指令
                delta_pose_left, gripper_command_left = teleop_interface_left.advance()
                delta_pose_right, gripper_command_right = teleop_interface_right.advance()
                delta_pose_left = torch.tensor(delta_pose_left.astype("float32"), device=env.device).repeat(env.num_envs, 1)
                delta_pose_right = torch.tensor(delta_pose_right.astype("float32"), device=env.device).repeat(env.num_envs, 1)
                actions_left = pre_process_actions(delta_pose_left, gripper_command_left)
                actions_right = pre_process_actions(delta_pose_right, gripper_command_right)
                # 环境动作空间要求两个机械臂动作拼接为一个向量，例如 [arm1, arm2]
                actions = torch.concat([actions_left, actions_right], dim=1)
            else:
                delta_pose, gripper_command = teleop_interface.advance()
                delta_pose = torch.tensor(delta_pose.astype("float32"), device=env.device).repeat(env.num_envs, 1)
                actions = pre_process_actions(delta_pose, gripper_command)

            env.step(actions)

            if should_reset_recording_instance:
                env.reset()
                should_reset_recording_instance = False

    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
