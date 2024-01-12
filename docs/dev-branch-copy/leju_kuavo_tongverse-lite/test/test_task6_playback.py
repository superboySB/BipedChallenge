import tongverselite as tv  # noqa # pylint: disable=wrong-import-order # isort: skip
import json
import os
from typing import List

import numpy as np
from test_task1 import TaskOneController

from tongverselite.env import Env


class DummyPlanner:
    def __init__(self) -> None:
        dir_path = os.path.dirname(os.path.realpath(__file__))
        # pre-computed  command sequence
        cmd_file = f"{dir_path}/kb_cmd_task6.json"

        with open(cmd_file, "r", encoding="utf-8") as fin:
            self.cmds_ = json.load(fin)

        self.cnt_ = 0

    def plan(self, obs: dict) -> List:
        assert isinstance(obs, dict)
        cmd = self.cmds_[self.cnt_]
        self.cnt_ += 1

        return cmd


def main():
    env = Env(task_num=6, seed=666)
    action = {}

    # Task Controller Init.
    ctrl = TaskOneController()

    planner = DummyPlanner()

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
            done, success, obs = env.step(action)
            print(obs["goal"])
            tv.sim.pause()
        try:
            if tv.sim.is_playing():
                kb_input = planner.plan(obs)
                # print(kb_input)
                cmd = ctrl.get_cmd(
                    obs, kb_input[0], kb_input[1], kb_input[2], kb_input[3]
                )

                action = {
                    "legs": {
                        "ctrl_mode": cmd["mode"],
                        "joint_values": cmd["effort"],
                    },
                }
                if kb_input[3] == 1:
                    action["arms"] = {
                        "ctrl_mode": "position",
                        "joint_values": [
                            np.pi / 4,
                            0.0,
                            0.0,
                            -np.pi * 2 / 3,
                            np.pi / 4 + kb_input[4],
                            0.0 + kb_input[5],
                            0.0,
                            -np.pi * 2 / 3 + kb_input[6],
                        ],
                    }
                    action["pick"] = "right_hand" if kb_input[7] == 1 else None
                    action["release"] = True if kb_input[8] == 1 else None
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

    tv.shutdown()


if __name__ == "__main__":
    main()
