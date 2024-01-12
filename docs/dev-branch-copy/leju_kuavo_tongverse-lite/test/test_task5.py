import tongverselite as tv  # noqa # pylint: disable=wrong-import-order # isort: skip
import json

import numpy as np
from keyboardinput import KeyboardCmd
from test_task1 import TaskOneController

from tongverselite.env import Env


def main():
    env = Env(task_num=5, seed=666)
    action = {}

    # Task Controller Init.
    ctrl = TaskOneController()
    # keyboard input
    kb_input = KeyboardCmd()
    kb_input.initialize()
    kb_cmd = []
    lift_hand_status = 3

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
            print(obs["agent"]["start_pos"])
            tv.sim.pause()
        try:
            if tv.sim.is_playing():
                kb_input.get_keyboard_cmd()
                kb_cmd.append([kb_input.vx, kb_input.vy, kb_input.rot, kb_input.state])
                cmd = ctrl.get_cmd(
                    obs, kb_input.vx, kb_input.vy, kb_input.rot, kb_input.state
                )

                action = {
                    "legs": {
                        "ctrl_mode": cmd["mode"],
                        "joint_values": cmd["effort"],
                    },
                }
                if kb_input.state == lift_hand_status:
                    action["arms"] = {
                        "ctrl_mode": "position",
                        "joint_values": [
                            -np.pi * 120 / 180,
                            -np.pi * 30 / 180,
                            0.0,
                            -np.pi * 45 / 180,
                            np.pi / 4,
                            0.0,
                            0.0,
                            -np.pi * 2 / 3,
                        ],
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

    with open("kb_cmd_task5.json", "w", encoding="utf-8") as fout:
        fout.write(json.dumps(kb_cmd, indent=4))

    tv.shutdown()


if __name__ == "__main__":
    main()
