import gzip
import json
import os
from typing import List

from tongverselite.solver import TaskSolverBase


def decompress_data(data):
    # Decompress
    decompressed = gzip.decompress(data)
    # Convert from bytes
    decoded = decompressed.decode("utf-8")
    # Convert to JSON
    json_data = json.loads(decoded)

    return json_data


class DummyPlanner:
    """A dummy planner for task 1 (seed = 66)  现在seed不是66了

    It produces a pre-computed velocity command for accomplishing
    task 1 with random seed 66.
    """

    def __init__(self) -> None:
        # get directory of this file
        dir_path = os.path.dirname(os.path.realpath(__file__))
        # pre-computed velocity command sequence
        cmd_file = f"{dir_path}/task1_cmd.json.gz"

        with open(cmd_file, "rb") as fin:
            self.cmds_ = decompress_data(fin.read())

        self.cnt_ = 0

    def plan(self, obs: dict) -> List:
        assert isinstance(obs, dict)

        # repeat last cmd when reach the end
        if self.cnt_ == len(self.cmds_):
            return self.cmds_[-1]

        cmd = self.cmds_[self.cnt_]
        self.cnt_ += 1

        return cmd


class TaskSolver(TaskSolverBase):
    def __init__(self) -> None:
        super().__init__()
        self.planner_ = DummyPlanner()

    def next_action(self, obs: dict) -> dict:
        return self.planner_.plan(obs)
