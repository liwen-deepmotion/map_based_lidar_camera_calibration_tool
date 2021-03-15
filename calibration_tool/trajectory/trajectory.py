

from typing import List

from trajectory.trajectory_node import TrajectoryNode


class Trajectory(object):

    def __init__(self):
        self._trajectory_nodes = []  # type: List[TrajectoryNode]

    def trajectory_nodes(self) -> List[TrajectoryNode]:
        return self._trajectory_nodes

    def load(self, file_path: str):
        pass
