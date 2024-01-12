import tongverselite as tv  # noqa # pylint: disable=wrong-import-order # isort: skip
import copy
import json
import time

from keyboardinput import KeyboardCmd

from tongverselite.env import Env
from tongverselite.tcp import PersistentTcpClient, json2bin


class BipedCtrlClient(PersistentTcpClient):
    def get_command(self, msg):
        data_bin = json2bin(msg)
        return json.loads(self.send(data_bin).decode("ascii"))


class TaskOneController:
    def __init__(self):
        self.msg = None
        self.client = None

    def initialize_socket(self):
        try:
            self.client = BipedCtrlClient(ip="10.1.100.217", port=8800)
        except Exception as e:
            self.client = None
            raise ValueError("Please run C++ to open the socket first") from e

    def reconnect(self):
        print("waiting for server...\r")
        while self.client is None:
            try:
                self.initialize_socket()
            except Exception:  # pylint: disable=broad-exception-caught
                time.sleep(0.05)
            tv.sim.step()
        print("connected to server!")
        # task = self.env.task
        # self.agent.post_reset()
        # self.agent.set_world_poses(task.start_pos, task.start_ori)
        # tv.sim.step()
        # tv.sim.pause()

    def get_cmd(self, obs, x=0, y=0, theta=0, state=-1):  # pylint: disable=invalid-name
        """
        get the command from the controller.
        """
        obs_agent = copy.deepcopy(obs["agent"])
        q_leg = obs_agent["joint_state"]["legs_positions"]
        dq_leg = obs_agent["joint_state"]["legs_velocities"]

        q_arm = obs_agent["joint_state"]["arms_positions"]
        dq_arm = obs_agent["joint_state"]["arms_velocities"]

        p_wb = obs_agent["body_state"]["world_pos"]
        quat_wb = obs_agent["body_state"]["world_orient"]
        v_wb = obs_agent["body_state"]["linear_velocities"]
        w_wb = obs_agent["body_state"]["angular_velocities"]

        self.msg = {
            "q_leg": q_leg.tolist(),
            "dq_leg": dq_leg.tolist(),
            "q_arm": q_arm.tolist(),
            "dq_arm": dq_arm.tolist(),
            "p_wb": p_wb.tolist(),
            "quat_wb": quat_wb.tolist(),
            "v_wb": v_wb.tolist(),
            "w_wb": w_wb.tolist(),
            "command": [x, y, theta],
            "change_state": state,
        }
        cmd = self.client.get_command(self.msg)
        return cmd


def main():
    env = Env(task_num=1, seed=66)
    action = {}

    # Task Controller Init.
    ctrl = TaskOneController()
    # keyboard input
    kb_input = KeyboardCmd()
    kb_input.initialize()

    # velocity_cmd = []
    # Reset the environment before calling step and get the first obs
    # Important: Please do not remove this line.
    done, success, obs = env.reset()

    while True:
        # If simulation is stopped, then exit.
        # Important: Please do not remove this line.
        if tv.sim.is_stopped():
            break

        # If simulation is paused, then skip.
        # Important: Please do not remove this line.
        if not tv.sim.is_playing():
            tv.sim.render()
            continue

        if ctrl.client is None:
            ctrl.reconnect()
            env.reset()
            kb_input.reset()
            done, success, obs = env.step(action)
            tv.sim.pause()
        try:
            if tv.sim.is_playing():
                kb_input.get_keyboard_cmd()
                # velocity_cmd.append([kb_input.vx, kb_input.vy, kb_input.rot])
                cmd = ctrl.get_cmd(
                    obs, kb_input.vx, kb_input.vy, kb_input.rot, kb_input.state
                )

                action = {
                    "legs": {
                        "ctrl_mode": cmd["mode"],
                        "joint_values": cmd["effort"],
                        "stiffness": [],
                        "dampings": [],
                    },
                }
        except Exception as e:  # pylint: disable=broad-exception-caught
            print(e)
            ctrl.client = None

        done, success, obs = env.step(action)

        if done:
            if success:
                print("Task passed")
            else:
                print("Task failed")
            break

    # with open("velocity_cmd.json", "w", encoding="utf-8") as fout:
    #     fout.write(json.dumps(velocity_cmd, indent=4))

    tv.shutdown()


if __name__ == "__main__":
    main()
