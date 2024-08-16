# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations



from omni.isaac.lab.assets import AssetBaseCfg
from omni.isaac.lab.utils import configclass
import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.managers import TerminationTermCfg as DoneTerm

import os
import sys

from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR, NVIDIA_NUCLEUS_DIR
from dataclasses import MISSING
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import lab_tasks.reach_obs.mdp as mdp
from lab_tasks.reach_obs.reach_env_cfg import ReachEnvCfg

##
# Pre-defined configs
##
# from omni.isaac.lab_assets import UR10_CFG  # isort: skip
from lab_assets import UR10_CFG_contact  # isort: skip


##
# Environment configuration
##


@configclass
class UR10ReachEnvCfg(ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # switch robot to ur10
        self.scene.robot = UR10_CFG_contact.replace(prim_path="{ENV_REGEX_NS}/Robot") # type: ignore
        # override events
        # self.events.reset_robot_joints.params["position_range"] = (0.75, 1.25)
        # override rewards
        self.rewards.end_effector_position_tracking.params["asset_cfg"].body_names = ["ee_link"]
        self.rewards.end_effector_orientation_tracking.params["asset_cfg"].body_names = ["ee_link"]
        # self.terminations.time_out = DoneTerm(func=mdp.time_out, time_out=True)
        # override actions
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=[".*"], scale=0.5, use_default_offset=False
        )
        # override command generator body
        # end-effector is along x-direction
        self.commands.ee_pose.body_name = "ee_link"
        # self.commands.ee_pose.ranges.pitch = (math.pi / 2, math.pi / 2)
        
@configclass
class UR10ReachEnvCfg_PLAY(UR10ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 1
        self.scene.env_spacing = 2.5
        # disable randomization for play
        # self.observations.policy.enable_corruption = False
        # self.terminations.time_out = DoneTerm(func=mdp.time_out, time_out=True)
        self.terminations.illegal_contact = None
        self.scene.environment1 = AssetBaseCfg(
            prim_path="/World/Environment1",
            spawn=sim_utils.UsdFileCfg(
                    usd_path=f"/home/songwoo/Desktop/IsaacLab/source/work_dir/safe_rl/assets_custom/husky.usd",
                    # usd_path="/home/songwoo/Desktop/IsaacLab/source/work_dir/safe_rl/assets_custom/simple_room.usd",
                    scale=(0.007, 0.007, 0.007),
            ),
            
            init_state=AssetBaseCfg.InitialStateCfg(pos=(5.16, 0.67, -0.3))
        )
        # self.scene.environment2 = AssetBaseCfg(
        #     prim_path="/World/Environment2",
        #     spawn=sim_utils.UsdFileCfg(
        #             usd_path=f"https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Commercial/Storage/Standard/Standard_LargeUnit.usd",
        #             scale=(0.02,0.02,0.02),
        #     ),
        #     init_state=AssetBaseCfg.InitialStateCfg(pos=[0.72, 0.13, -0.73], rot=[0.707, 0., 0., -0.707]),
        # )
        self.scene.environment2 = None
        self.scene.ground = None
        self.scene.light = None 
        self.scene.table = None
        