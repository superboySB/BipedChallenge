import tongverselite as tv  # noqa # pylint: disable=wrong-import-order # isort: skip
import numpy as np
from keyboardinput import KeyboardCmd
from test_task1 import TaskOneController

from tongverselite.env import Env


def main():
    env = Env(task_num=6, seed=666)
    action = {}

    # Task Controller Init.
    ctrl = TaskOneController()
    # keyboard input
    kb_input = KeyboardCmd()
    kb_input.initialize()
    action_cmd = []

    # Reset the environment before calling step and get the first obs
    # Important: Please do not remove this line.
    # done, success, obs = env.reset()

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
            print(obs["goal"])
            tv.sim.pause()
        try:
            if tv.sim.is_playing():
                kb_input.get_keyboard_cmd()
                cmd = ctrl.get_cmd(
                    obs, kb_input.vx, kb_input.vy, kb_input.rot, kb_input.state
                )
                action_cmd.append(
                    cmd["effort"]
                    + [
                        kb_input.arm1,
                        kb_input.arm2,
                        kb_input.arm3,
                        1 if kb_input.pick == "right_hand" else 0,
                        1 if kb_input.release else 0,
                    ]
                )
                action = {
                    "legs": {
                        "ctrl_mode": cmd["mode"],
                        "joint_values": cmd["effort"],
                        "stiffness": [],
                        "dampings": [],
                    },
                }
                if kb_input.state == 1:
                    action["arms"] = {
                        "ctrl_mode": "position",
                        "joint_values": [
                            np.pi / 4,
                            0.0,
                            0.0,
                            -np.pi * 2 / 3,
                            np.pi / 4 + kb_input.arm1,
                            0.0 + kb_input.arm2,
                            0.0,
                            -np.pi * 2 / 3 + kb_input.arm3,
                        ],
                    }
                    action["pick"] = kb_input.pick
                    action["release"] = kb_input.release
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

    # with open("a_cmd_task6.json", "w", encoding="utf-8") as fout:
    #     fout.write(json.dumps(action_cmd, indent=4))
    np.savez_compressed("task6_cmd2.npz", cmd=action_cmd)

    tv.shutdown()


if __name__ == "__main__":
    main()
