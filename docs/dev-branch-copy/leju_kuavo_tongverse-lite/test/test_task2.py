import tongverselite as tv  # noqa # pylint: disable=wrong-import-order # isort: skip

# import json

import numpy as np
from keyboardinput import KeyboardCmd
from test_task1 import TaskOneController

from tongverselite.env import Env


def main():
    env = Env(task_num=2, seed=266)
    action = {}
    obs = {"pick": False}

    # Task Controller Init.
    ctrl = TaskOneController()
    # keyboard input
    kb_input = KeyboardCmd()
    kb_input.initialize()
    kb_cmd = []

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
            tv.sim.step()
            tv.sim.pause()
        try:
            if tv.sim.is_playing():
                kb_input.get_keyboard_cmd()
                kb_cmd.append(
                    [
                        kb_input.vx,
                        kb_input.vy,
                        kb_input.rot,
                        kb_input.state,
                        kb_input.arm1,
                        kb_input.arm2,
                        kb_input.arm3,
                        1 if kb_input.pick == "left_hand" else 0,
                    ]
                )
                cmd = ctrl.get_cmd(
                    obs, kb_input.vx, kb_input.vy, kb_input.rot, kb_input.state
                )
                action = {
                    "legs": {
                        "ctrl_mode": cmd["mode"],
                        "joint_values": cmd["effort"],
                    },
                    "arms": {
                        "ctrl_mode": "position",
                        "joint_values": [
                            np.pi / 4 + kb_input.arm1,
                            0.0 + kb_input.arm2,
                            0.0,
                            -np.pi * 2 / 3 + kb_input.arm3,
                            np.pi / 4,
                            0.0,
                            0.0,
                            -np.pi * 2 / 3,
                        ],
                    },
                    "pick": kb_input.pick,
                    "release": kb_input.release,
                }
                if obs["pick"] is True:
                    action["arms"]["stiffness"] = [5] * 4 + [20] * 4
                    action["arms"]["dampings"] = [1] * 4 + [2] * 4

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

    np.savez_compressed("task2_cmd.npz", cmd=kb_cmd)

    tv.shutdown()


if __name__ == "__main__":
    main()
