from abc import ABC, abstractmethod


class TaskSolverBase(ABC):
    @abstractmethod
    def next_action(self, obs: dict) -> dict:
        """Generate next step action given current observation

        Args:
            obs (dict):
            {
                "agent":{
                    "joint_state":
                        {"arms_positions": np.ndarray,
                        "arms_velocities": np.ndarray,
                        "arms_applied_effort" : np.ndarray,
                        "legs_positions": np.ndarray,
                        "legs_velocities": np.ndarray,
                        "legs_applied_effort": np.ndarray
                    },
                    "body_state": {
                        "world_pos": np.ndarray,
                        "world_orient": np.ndarray,
                        "linear_velocities": np.ndarray,
                        "angular_velocities": np.ndarray
                    },
                    "stiffness": np.ndarray,
                    "dampings": np.ndarray,
                    'cam1': {
                        'rgb': np.ndarray,
                        "distance_to_image_plane": np.ndarray,
                    },
                    "obstacle":{
                        'position':np.ndarray,
                        'orientation': np.ndarray
                    }
                }
            }

        Returns:
            dict:
            {
                "arms":{
                    "ctrl_mode":"position",
                    "joint_values":None,
                    "stiffness":None,
                    "dampings":[]
                },
                "legs": {
                    "ctrl_mode":"effort",
                    "joint_values":[10]*10,
                    "stiffness":[],
                    "dampings":[],
                }
            }
        """
        return {}
