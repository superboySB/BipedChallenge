import sys

sys.path.append("/TongVerse/dist/")

import tongverselite as tv  # noqa # pylint: disable=wrong-import-order # isort: skip
from tongverselite.env import Env

# pylint: disable=pointless-string-statement
"""
NOTE:
call env.reset() to reset both the task and agent to their initial states.

action(dict): control command from the controller
  1. Control mode can only be position or effort.
  2. The order of values in the 'joint_values' corresponds to the order of elements
  in the 'agent_cfg' arm_idx and leg_idx.
  3. Ensure the key in action remains unchanged.
  4. For Task2 and Task6, set the value in the "pick" key to either "left_hand"
  or "right_hand" to specify the hand you are using if you are ready to attach
  the valve or object. To release, set the "release" key to True if you are
  ready to release the valve or object.

  example:
  action = {
            "arms":{"ctrl_mode":"position",
                    "joint_values":None,
                    "stiffness":None,
                    "dampings":[],
                    },
            "legs": {"ctrl_mode":"effort",
                    "joint_values":[10]*10,
                    "stiffness":[],
                    "dampings":[],
                    },
            "pick": None,
            "release": False

        }

observations(dict):  dictionary containing information about the current state
    1. In Task6, the observation(`obs`) can include RGBD information from the camera,
    stored in the key "cam1" and the task goal which is randomly generated.
    2. In both Task2 and Task6, the observation(`obs`) will indicate whether the object
    or valve is attached to the hand, with the status stored in the key "pick".
    If the attachment is successful, the value will be True, otherwise,
    it will be False.
    3. In Task3, the observation(`obs`) includes obstacle pose information

    example:
    obs = {
        "agent":{
            "joint_state":{
                "arms_positions": np.ndarray,
                "arms_velocities":  np.ndarray,
                "arms_applied_effort" :   np.ndarray,
                "legs_positions":  np.ndarray,
                "legs_velocities":  np.ndarray,
                "legs_applied_effort":  np.ndarray
            },
            "body_state": {
                "world_pos": np.ndarray,
                "world_orient": np.ndarray,
                "linear_velocities": np.ndarray,
                "angular_velocities": np.ndarray
            },
            "stiffness": np.ndarray,
            "dampings": np.ndarray,
            "start_pos":np.ndarray,
            "start_orient": np.ndarray
        },
        'cam1': {
            'rgb': np.ndarray,
            "distance_to_image_plane": np.ndarray,
        },
        "goal": str,
        "pick": bool,
        "obstacle":{
            'position':np.ndarray,
            'orientation': np.ndarray
        }
    }
"""  # pylint: disable=pointless-string-statement


def main(task_id):
    # The seed will be randomly generated in the actual challenge.
    # If you assign 'None' to the seed, the task will be configured randomly.
    # You can test the robutness of your model by having `seed = None`.
    env = Env(task_num=task_id, seed=None)
    action = {}

    # create solver for task 1
    # pylint: disable=import-outside-toplevel,exec-used
    exec(f"from task_{task_id}_solver.task_solver import TaskSolver", globals())
    # pylint: disable=undefined-variable
    solver = TaskSolver()  # noqa: F821

    # Reset the environment before calling step
    # Important: Please do not remove this line.
    env.reset()

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

        # an empty action will be applied at the very beginning
        # stage to get an observation
        done, success, obs = env.step(action)
        print("\n\ndone:",done)
        print("success:",success)
        print(obs)

        if done:
            if success:
                print("Task passed")
            else:
                print("Task failed")
            break

        # pass observation to solver and get action for next step
        action = solver.next_action(obs)

    tv.shutdown()


if __name__ == "__main__":
    main(int(sys.argv[1]))
