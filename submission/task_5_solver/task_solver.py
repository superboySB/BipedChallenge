import json
import os
from typing import List

import numpy as np

from tongverselite.solver import TaskSolverBase
from tongverselite.tcp import PersistentTcpClient, json2bin
import time


class OurPlanner:
    """A dummy planner for task 1 (seed = 66)

    It produces a pre-computed velocity command for accomplishing
    task 1 with random seed 66.
    """

    def __init__(self) -> None:
        self.goal_center = np.array([0.23457, -1.44135, 0.593])
        self.cnt_ = 0

    def plan(self, obs: dict) -> List:
        assert isinstance(obs, dict)
        
        cmd = np.array([0, 0, 0, -1]).astype(np.float32)

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
        if self.cnt_ % 10 ==0:
            print(diff_pos)
            print(yaw_world_orient_deg)
            print(pitch_world_orient_deg)
            print(roll_world_orient_deg)

        cmd[1] = -diff_pos[0]/4.5
        
        if diff_pos[0] > 0.1 and roll_world_orient_deg < 85:
            cmd[0] = 0.2
            cmd[2] = -1/4
        elif diff_pos[0] < -0.1 and roll_world_orient_deg > 95:
            cmd[0] = 0.2
            cmd[2] = 1/4
        else:
            cmd[0] = 0.27
            cmd[2] = 0

        if self.cnt_ > 4000:
            cmd[0] = 0.35

        self.cnt_ += 1
        print(self.cnt_)

        return cmd.tolist()


class BipedWalkingCtrlClient(PersistentTcpClient):
    def send_request(self, msg):
        data_bin = json2bin(msg)
        return json.loads(self.send(data_bin).decode("ascii"))

    def get_cmd(self, obs, vx, vy, theta, state):
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
            "command": [vx, vy, theta],
            "change_state": state,
        }
        joint_efforts = self.send_request(msg)

        return joint_efforts


class TaskSolver(TaskSolverBase):
    def __init__(self) -> None:
        super().__init__()
        self.planner_ = OurPlanner()
        self.ctrl_client_ = BipedWalkingCtrlClient(ip="0.0.0.0", port=8800)

    def next_action(self, obs: dict) -> dict:
        # time.sleep(0.1)
        
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
            }
        }

        return action
