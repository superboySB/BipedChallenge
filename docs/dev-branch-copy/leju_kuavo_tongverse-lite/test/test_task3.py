import tongverselite as tv  # noqa # pylint: disable=wrong-import-order # isort: skip

from keyboardinput import KeyboardCmd
from test_task1 import TaskOneController

from tongverselite.env import Env


def main():
    env = Env(task_num=3, seed=88)
    action = None
    # tv.sim.step()

    # Task Controller Init.
    ctrl = TaskOneController()
    # keyboard input
    kb_input = KeyboardCmd()
    kb_input.initialize()

    d_th = 0.65

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
                d_check = obs["obstacle"]["position"][1] - obs["agent"]["start_pos"][1]
                state = 2 if d_check <= d_th else -1
                cmd = ctrl.get_cmd(obs, kb_input.vx, kb_input.vy, kb_input.rot, state)

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

    tv.shutdown()


if __name__ == "__main__":
    main()
