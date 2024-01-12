# pylint: skip-file
import tongverselite as tv  # noqa # pylint: disable=wrong-import-order # isort: skip

import copy

import numpy as np
import omni.isaac.core.utils.prims as prim_utils
import torch
from biped_ctrl_client import BipedCtrlClient
from omni.isaac.core.objects.ground_plane import GroundPlane
from pxr import Gf

from tongverselite.agent import Agent


class KuavoSimulator:
    def __init__(self):
        # robot info
        self.q_arm, self.dq_arm = np.zeros(8), np.zeros(8)
        self.q_leg, self.dq_leg = np.zeros(10), np.zeros(10)

        self.p_wb = np.zeros(3)  # body pos in world frame
        self.quat_wb = np.zeros(4)  # body orientation in world frame (w, x, y, z)
        self.v_wb = np.zeros(3)  # body velocity in world frame
        self.w_wb = np.zeros(3)  # body angular velocity in world frame
        self.foot_contacts = np.zeros(4)  # boolean contact info

        self.default_pos = np.zeros(18)  # initial pose of all joints

        self.kps = torch.FloatTensor([[100.0] * 18])
        self.kds = torch.FloatTensor([[0.5] * 18])

        # the usd joint order
        # ['l_shoulder_y', 'l_shoulder_z', 'l_shoulder_x', 'l_elbow',
        #  'r_shoulder_y', 'r_shoulder_z', 'r_shoulder_x', 'r_elbow']
        self.arm_idx = [1, 5, 9, 13, 3, 7, 11, 15]
        # ['l_hip_z', 'l_hip_x', 'l_hip_y', 'l_knee', 'l_ankle',
        #  'r_hip_z', 'r_hip_x', 'r_hip_y', 'r_knee', 'r_ankle']
        self.leg_idx = [0, 4, 8, 12, 16, 2, 6, 10, 14, 17]

        # simulator info
        self.agent = None
        self.msg = None

    def initialize_socket(self):
        try:
            self.client = BipedCtrlClient(ip="0.0.0.0", port=8800)
        except Exception as e:  # noqa
            raise ValueError("Please run C++ to open the socket first") from e

    def initialize_simulator(self):
        # Add ground
        GroundPlane(prim_path="/World/groundPlane", name="ground_plane", visible=True)

        # Add light
        prim_utils.create_prim(
            "/World/Light/DistantLight",
            "DistantLight",
            translation=(0, 0, 0),
            attributes={"intensity": 1000.0, "color": (1.0, 1.0, 1.0)},
        )

        tv.sim.reset()
        # Add agent
        self.agent = Agent(
            position=[0, 0, 1.0],
            orientation=[1, 0, 0, 0],
        )

        # Add contact sensors
        self.agent.add_contact_sensor(
            "toe_r_cs", "r_foot", sensor_offset=Gf.Vec3d(0.065, 0, -0.042)
        )
        self.agent.add_contact_sensor(
            "heel_r_cs", "r_foot", sensor_offset=Gf.Vec3d(-0.065, 0, -0.042)
        )
        self.agent.add_contact_sensor(
            "toe_l_cs", "l_foot", sensor_offset=Gf.Vec3d(0.065, 0, -0.042)
        )
        self.agent.add_contact_sensor(
            "heel_l_cs", "l_foot", sensor_offset=Gf.Vec3d(-0.065, 0, -0.042)
        )

        tv.sim.step()
        # Initialization
        # tv.sim.reset()
        self.agent.initialize()
        self.agent.set_gains(kps=self.kps, kds=self.kds, save_to_usd=True)
        self.agent.switch_control_mode("position")
        self.agent.set_joints_default_state(
            positions=torch.FloatTensor(self.default_pos)
        )
        # have to call it for setting the default state
        self.agent.post_reset()
        self.agent.initialize()
        tv.sim.step()
        print("TongVerse Initialization Completed!")

    def write_position(self, leg_positions=None, arm_positions=None):
        """
        Send goal positions to the simulator.
        """
        if leg_positions is not None:
            self.agent.set_gains(
                kps=self.kps[:, self.leg_idx],
                kds=self.kds[:, self.leg_idx],
                joint_indices=torch.Tensor(self.leg_idx),
            )
            self.agent.switch_control_mode(
                "position", joint_indices=torch.Tensor(self.leg_idx)
            )
            leg_pos_cmd = copy.deepcopy(leg_positions)
            self.agent.set_joint_position_targets(
                torch.FloatTensor(leg_pos_cmd), joint_indices=self.leg_idx
            )

        if arm_positions is not None:
            self.agent.set_gains(
                kps=self.kps[:, self.arm_idx],
                kds=self.kds[:, self.arm_idx],
                joint_indices=torch.Tensor(self.arm_idx),
            )
            self.agent.switch_control_mode(
                "position", joint_indices=torch.Tensor(self.arm_idx)
            )
            arm_pos_cmd = copy.deepcopy(arm_positions)
            self.agent.set_joint_position_targets(
                torch.FloatTensor(arm_pos_cmd), joint_indices=self.arm_idx
            )

    def write_torque(self, leg_torques=None, arm_torques=None):
        """
        Send goal torques to the simulator.
        """
        if leg_torques is not None:
            self.agent.switch_control_mode(
                "effort", joint_indices=torch.Tensor(self.leg_idx)
            )
            leg_torque_cmd = copy.deepcopy(leg_torques)
            self.agent.set_joint_efforts(
                torch.FloatTensor(leg_torque_cmd), joint_indices=self.leg_idx
            )

        if arm_torques is not None:
            self.agent.switch_control_mode(
                "effort", joint_indices=torch.Tensor(self.arm_idx)
            )
            arm_torque_cmd = copy.deepcopy(arm_torques)
            self.agent.set_joint_efforts(
                torch.FloatTensor(arm_torque_cmd), joint_indices=self.arm_idx
            )

    def update_robot_info(self):
        """
        Update robot info from the simulator.
        """
        # get joint states
        q = self.agent.get_joint_positions(clone=False).cpu().numpy()[0]
        dq = self.agent.get_joint_velocities(clone=False).cpu().numpy()[0]

        self.q_leg = q[self.leg_idx]
        self.dq_leg = dq[self.leg_idx]

        self.q_arm = q[self.arm_idx]
        self.dq_arm = dq[self.arm_idx]

        # get body states
        p_wb, quat_wb = self.agent.get_world_poses(clone=False)
        self.p_wb = p_wb.cpu().numpy()[0]
        self.quat_wb = quat_wb.cpu().numpy()[0]
        self.v_wb = self.agent.get_linear_velocities(clone=False).cpu().numpy()[0]
        self.w_wb = self.agent.get_angular_velocities(clone=False).cpu().numpy()[0]

        self.foot_contacts = np.array(
            [
                self.agent.get_sensor_readings("toe_r_cs", "r_foot"),
                self.agent.get_sensor_readings("heel_r_cs", "r_foot"),
                self.agent.get_sensor_readings("toe_l_cs", "l_foot"),
                self.agent.get_sensor_readings("heel_l_cs", "l_foot"),
            ]
        )

    def get_cmd(self):
        """
        get the command from the controller.
        """
        self.msg = {
            "q_leg": self.q_leg.tolist(),
            "dq_leg": self.dq_leg.tolist(),
            "q_arm": self.q_arm.tolist(),
            "dq_arm": self.dq_arm.tolist(),
            "p_wb": self.p_wb.tolist(),
            "quat_wb": self.quat_wb.tolist(),
            "v_wb": self.v_wb.tolist(),
            "w_wb": self.w_wb.tolist(),
        }
        cmd = self.client.get_command(self.msg)
        return cmd


def main():
    # Sim Init.
    sim_env = KuavoSimulator()
    # sim_env.initialize_socket()
    sim_env.initialize_simulator()

    tv.sim.pause()
    while True:
        if tv.sim.is_playing():
            sim_env.update_robot_info()

            # cmd = sim_env.get_cmd()
            # print(cmd)

            # leg = [0, np.pi/4, 0, np.pi/2 , np.pi/4 , 0, np.pi/4, 0, np.pi/2 , np.pi/4]  # noqa
            # arm = [0, 0 , 0 , -np.pi/2, 0 , 0 , -np.pi/2 , -np.pi/2]

            # sim_env.write_position(
            #         leg_positions=leg,
            #         arm_positions=arm,
            #     )

            # if cmd['mode'] == 'position':  # position control
            #     sim_env.write_position(
            #         leg_positions=cmd,
            #         arm_positions=None,
            #     )
            # elif cmd['mode'] == 'effort':  # torque control
            #     sim_env.write_torque(
            #         leg_torques=cmd,
            #         arm_torques=None
            #     )

        tv.sim.step()

    # app.shutdown()


if __name__ == "__main__":
    main()
