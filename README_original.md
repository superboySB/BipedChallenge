# 人形机器人创新挑战赛

Bipedal Robot Challenge powered by TongVerse-Lite.

## System Requirements

- Docker
- [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

## Development Environment Setup

### Setting up Docker

1. **Download Docker Image**
   URL: `https://roban.lejurobot.com/humanoid-tongverselite/tongverselite-release-docker_20240104.tar.gz`
   Use your webbrowser or wget command to download this docker image to your computer.
   wget command:
   ```bash
   wget https://roban.lejurobot.com/humanoid-tongverselite/tongverselite-release-docker_20240104.tar.gz -O tongverselite-release-docker.tar.gz
   ```

2. **Load the Docker Image**
   Run the following command:
   ```bash
   docker load -i tongverselite-release-docker.tar.gz
   ```

3. **Launch the Docker Image**
   To set up the TongVerse-Lite environment, execute:
   ```bash
   bash docker-run-release.sh
   ```
   This script binds your local repository directory to `/BipedChallenge/` inside Docker and initiates a new bash shell.

## Running a Demo

Once you are in the Docker environment, you can start a demo with:

```bash
bash examples/launch_task.sh <task-id>
```
Replace `<task-id>` with an integer between 1 and 6 to select the specific demo task you want to present.

## Development & Submission Guidelines

### Getting Started

1. **Familiarize with Examples**
   Explore the [examples](./examples/) directory to understand how the environment works.
2. **Implement Your TaskSolver**
   Create your custom `TaskSolver` for each task in `submission/task_<id>_solver`. Please note:
   - Do not modify any code outside the `task_<id>_solver` directories.
   - Modifications to `task_launcher.py` or other core files are strictly prohibited.

### Implementing Your Solver

In the [submission](./submission/) folder, we provide a solver template for each task. Implement your solver within the respective `task_<id>_solver/` folder. Below is the template provided:

```python
class TaskSolver(TaskSolverBase):
    def __init__(self) -> None:
        super().__init__()
        # Your TaskSolver implementation goes here
        raise NotImplementedError("Implement your own TaskSolver here")

    def next_action(self, obs: dict) -> dict:
        # Determine the next action based on the current observation (`obs`)
        # action = plan(obs)
        # return action
        raise NotImplementedError("Implement your own TaskSolver here")
```

- Use the `__init__()` function to initialize your solver with any necessary modules.
- Implement the `next_action()` function to determine and return the robot's next action based on the current observation `obs`. The specific formats for observations and actions are detailed in the sections that follow.

### Observation Space

The observation `obs` provided in the `next_action()` function is a dictionary detailing the current state:

```json
obs = {
    "agent":{
        "joint_state":{
            "arms_positions": np.ndarray (n_arm_joints,),
            "arms_velocities":  np.ndarray (n_arm_joints,),
            "arms_applied_effort" :   np.ndarray (n_arm_joints,),
            "legs_positions":  np.ndarray (n_arm_joints,),
            "legs_velocities":  np.ndarray (n_arm_joints,),
            "legs_applied_effort":  np.ndarray (n_arm_joints,)
        },
        "body_state": {
            "world_pos": np.ndarray (3,) <x, y, z>,
            "world_orient": np.ndarray (4,) <w, x, y, z>,
            "linear_velocities": np.ndarray (3,) <x, y, z>,
            "angular_velocities": np.ndarray (3,) <wx, wy, wz>
        },
        "stiffness": np.ndarray (n_links, ),
        "dampings": np.ndarray (n_links, ),
        "start_pos":np.ndarray (3,) <x, y, z>,
        "start_orient": np.ndarray (4,) <w, x, y, z>
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
```

Of note,

1. In Task6, the observation(`obs`) will include RGBD information from the camera, stored in the key `cam1` and the task goal is store in the key `goal`.
2. In both Task2 and Task6, the observation(`obs`) will indicate whether the object or valve is attached to the hand, with the status stored in the key `pick`. If the attachment is successful, the value will be True, otherwise, it will be False.
3. In Task3, the observation(`obs`) will includes obstacle pose information

### Action Space

The action indicate how you want to control the robot, the action format is given as follows:

```json
action = {
    "arms":{
    	"ctrl_mode":"position",
        "joint_values": np.ndarray (8,),
        "stiffness":None,
        "dampings":[],
    },
    "legs": {
        "ctrl_mode":"effort",
        "joint_values": np.ndarray (10, ),
        "stiffness":[],
        "dampings":[],
    },
    "pick": <None/left_hand/right_hand>,
    "release": False/True
}
```
Of note:
1. Control mode can only be *position* or *effort*.
2. The order of values in the `joint_values` should correspond to the order of elements
in the `arm_idx` and `leg_idx`.
3. Ensure the key in action remains unchanged.
4. For Task2 and Task6, set the value in the "pick" key to either "left_hand"
or "right_hand" to specify the hand you are using if you are ready to attach
the valve or object. To release, set the "release" key to True if you are
ready to release the valve or object.

**Arm and Leg Index**

The `arm_idx` and `leg_idx` is given as follows:

```python
# NOTE: DO NOT modify the order of elements in the list
# ('l_shoulder_y', 'l_shoulder_z', 'l_shoulder_x', 'l_elbow',
#  'r_shoulder_y', 'r_shoulder_z', 'r_shoulder_x', 'r_elbow')
arm_idx: List = [1, 5, 9, 13, 3, 7, 11, 15]
# ('l_hip_z', 'l_hip_x', 'l_hip_y', 'l_knee', 'l_ankle',
#  'r_hip_z', 'r_hip_x', 'r_hip_y', 'r_knee', 'r_ankle')
leg_idx: List = [0, 4, 8, 12, 16, 2, 6, 10, 14, 17]
dof_names: List = [
    "l_hip_z",
    "l_shoulder_y",
    "r_hip_z",
    "r_shoulder_y",
    "l_hip_x",
    "l_shoulder_z",
    "r_hip_x",
    "r_shoulder_z",
    "l_hip_y",
    "l_shoulder_x",
    "r_hip_y",
    "r_shoulder_x",
    "l_knee",
    "l_elbow",
    "r_knee",
    "r_elbow",
    "l_ankle",
    "r_ankle",
]
```

For example, The 0th element in `arm_idx` is `dof_names[arm_idx[0]]: "l_shoulder_y"`.

Therefore, when set the joint_values for `arms`: the joint order is given by:
```python
['l_shoulder_y', 'l_shoulder_z', 'l_shoulder_x', 'l_elbow', 'r_shoulder_y', 'r_shoulder_z', 'r_shoulder_x', 'r_elbow']
```

The joint order for leg is given by

```python
['l_hip_z', 'l_hip_x', 'l_hip_y', 'l_knee', 'l_ankle', 'r_hip_z', 'r_hip_x', 'r_hip_y', 'r_knee', 'r_ankle']
```

### Using Robot Locomotion Controller

We integrate a bipedal locomotion controller in our platform. Below is an example to use the provided controller to get the joint effort given a desire velocity command.


```python
from tongverselite.tcp import PersistentTcpClient, json2bin
from tongverselite.solver import TaskSolverBase


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
        self.ctrl_client_ = BipedWalkingCtrlClient(ip="0.0.0.0", port=8800)

    def next_action(self, obs: dict) -> dict:
        # v_x: 0.2 m/s, v_y: 0.3 m/s, v_theta: 0.561 rad/s
        velocity_cmd = [0.2, 0.3, 0.561]

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
```

### Task Specification

**Task 1**

Goal center position <x, y, z>: [1.94795, -9.1162, 0.1]

**Task 4**

Goal center position <x, y, z>: [0.23457, -3.53851, 0.296]

**Task 5**

Goal center position <x, y, z>: [0.23457, -1.44135, 0.593]

**Tasks 2, 3, and 6**

No further information provided.

As the scene is static, you can hard-code any necessary information to ensure the robot can accomplish the task from a random initial state.

**Camera Configuration**

The camera intrinsic matrix is given as follows:

```python
K = [
    [1154.47387, 0, 540],
    [0, 1154.47387, 360],
    [0, 0, 1]
]
```

In _Task 6_, the camera is mounted on the torso of the robot, meaning the camera moves along with the robot. The transformation from the torso link to the camera link is given as follows:

```python
tf_torso_camera = [
    [-0.7071068, 0.0000000, -0.7071068, -0.41],
    [-0.0000000, -1.0000000, 0.0000000, 0.00],
    [-0.7071068, 0.0000000, 0.7071068, 1.20],
    [0, 0, 0, 1]
]
```

### Testing Your Solution

To test your `TaskSolver`, execute:
```bash
bash submission/launch_task.sh <task-id>
```
Replace `<task-id>` with an integer from 1 to 6 corresponding to the task you are testing.

### Preparing for Submission

1. **Compress Your Work**
   Compress the entire [submission](./submission/) folder.

2. **Rename the File**
   Name the compressed file as `submission_<team-id>`.

3. **Submit to the Committee**
   Send your renamed submission file to our committee group for evaluation.
