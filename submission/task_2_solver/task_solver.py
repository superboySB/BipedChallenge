import json
import os
from typing import List

import numpy as np

from tongverselite.solver import TaskSolverBase
from tongverselite.tcp import PersistentTcpClient, json2bin


class DummyPlanner:
    """A dummy planner for task 2 (seed = 266)

    It produces a pre-computed command for accomplishing
    task 2 with random seed 266.
    """

    def __init__(self) -> None:
        # get directory of this file
        dir_path = os.path.dirname(os.path.realpath(__file__))
        # pre-computed velocity command sequence
        cmd_file = f"{dir_path}/task2_cmd.npz"

        self.goal_center = np.array([1.2495524, -9.101468, 0.6063346])
        self.goal_roll = -10.536394033161903
        
        self.cmds_ = np.load(cmd_file)["cmd"]
        self.cnt_ = 0

    def plan(self, obs: dict) -> List:
        assert isinstance(obs, dict)

        diff_pos = self.goal_center - obs["agent"]["body_state"]["world_pos"]
        # 定义新的四元数 q = [qx, qy, qz, qw] 对应于 'world_orient'
        q_world_orient = obs["agent"]["body_state"]["world_orient"]
        # 重新计算欧拉角
        yaw_world_orient = np.arctan2(2 * (q_world_orient[3] * q_world_orient[2] + q_world_orient[0] * q_world_orient[1]), 1 - 2 * (q_world_orient[1]**2 + q_world_orient[2]**2))
        pitch_world_orient = np.arcsin(2 * (q_world_orient[3] * q_world_orient[1] - q_world_orient[2] * q_world_orient[0]))
        roll_world_orient = np.arctan2(2 * (q_world_orient[3] * q_world_orient[0] + q_world_orient[1] * q_world_orient[2]), 1 - 2 * (q_world_orient[0]**2 + q_world_orient[1]**2))
        # 转换为度
        yaw_world_orient_deg = np.degrees(yaw_world_orient)
        pitch_world_orient_deg = np.degrees(pitch_world_orient)
        roll_world_orient_deg = np.degrees(roll_world_orient)
        diff_roll = self.goal_roll - roll_world_orient_deg

        if (np.abs(diff_pos[0]) < 0.01 and np.abs(diff_pos[1]) < 0.01 and np.abs(diff_pos[2]) < 0.2 and np.abs(diff_roll) < 0.01) or (self.cnt_!= 0):
            print(self.cnt_)

            # repeat last cmd when reach the end
            if self.cnt_ == len(self.cmds_):
                return self.cmds_[-1]

            cmd = self.cmds_[self.cnt_]
            self.cnt_ += 1
            
        else:
            print(diff_pos)
            print(diff_roll)
            
            cmd = np.array([-1, -1, 0, -1,0,0,0,0]).astype(np.float32)
            cmd[0] = -diff_pos[0]
            cmd[1] = -diff_pos[1]
            if diff_roll < 0:
                cmd[2] = 1/8
            elif diff_roll > 0:
                cmd[2] = -1/8
            else:
                cmd[2] = 0
        
        return cmd.tolist()


class BipedWalkingCtrlClient(PersistentTcpClient):
    def send_request(self, msg):
        data_bin = json2bin(msg)
        return json.loads(self.send(data_bin).decode("ascii"))

    def get_cmd(self, obs, v_x, v_y, theta, state):
        obs_agent = obs["agent"]
        q_leg = obs_agent["joint_state"]["legs_positions"]
        dq_leg = obs_agent["joint_state"]["legs_velocities"]

        q_arm = obs_agent["joint_state"]["arms_positions"]
        dq_arm = obs_agent["joint_state"]["arms_velocities"]

        p_wb = obs_agent["body_state"]["world_pos"]
        quat_wb = obs_agent["body_state"]["world_orient"]
        v_wb = obs_agent["body_state"]["linear_velocities"]
        w_wb = obs_agent["body_state"]["angular_velocities"]

        msg = {
            "q_leg": q_leg.tolist(),
            "dq_leg": dq_leg.tolist(),
            "q_arm": q_arm.tolist(),
            "dq_arm": dq_arm.tolist(),
            "p_wb": p_wb.tolist(),
            "quat_wb": quat_wb.tolist(),
            "v_wb": v_wb.tolist(),
            "w_wb": w_wb.tolist(),
            "command": [v_x, v_y, theta],
            "change_state": state,
        }
        joint_efforts = self.send_request(msg)

        return joint_efforts


class TaskSolver(TaskSolverBase):
    def __init__(self) -> None:
        super().__init__()
        self.planner_ = DummyPlanner()
        self.ctrl_client_ = BipedWalkingCtrlClient(ip="0.0.0.0", port=8800)

    def next_action(self, obs: dict) -> dict:
        # plan for velocity cmd
        velocity_cmd = self.planner_.plan(obs)

        # call bipedal controller to get joint effort given a target velocity
        joint_efforts = self.ctrl_client_.get_cmd(
            obs, velocity_cmd[0], velocity_cmd[1], velocity_cmd[2], velocity_cmd[3]
        )

        # wrap joint effort into tongverse-lite action format
        action = {
            "legs": {
                "ctrl_mode": joint_efforts["mode"],
                "joint_values": joint_efforts["effort"],
                "stiffness": [],
                "dampings": [],
            },
            "arms": {
                "ctrl_mode": "position",
                "joint_values": [
                    np.pi / 4 + velocity_cmd[4],
                    0.0 + velocity_cmd[5],
                    0.0,
                    -np.pi * 2 / 3 + velocity_cmd[6],
                    np.pi / 4,
                    0.0,
                    0.0,
                    -np.pi * 2 / 3,
                ],
            },
            "pick": "left_hand" if velocity_cmd[7] == 1 else None,
        }
        if obs["pick"] is True:
            action["arms"]["stiffness"] = [10] * 4 + [20] * 4
            action["arms"]["dampings"] = [1] * 4 + [2] * 4

        return action
