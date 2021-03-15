

import csv

import numpy as np
import pyquaternion

from trajectory.trajectory import Trajectory
from trajectory.trajectory_node import TrajectoryNode


class CameraTrajectory(Trajectory):

    def load(self, file_path: str):
        with open(file_path, 'r') as f:
            csv_rows = csv.reader(f)
            for (timestamp, _, _, _,
                 tx, ty, tz, qw, qx, qy, qz, _, _, _, _, _, _, _, _, _, _, _,
                 cam_to_road_height) in csv_rows:
                node = TrajectoryNode()
                node.set_timestamp(int(timestamp))

                T_camera_to_world = pyquaternion.Quaternion(
                    qw, qx, qy, qz).transformation_matrix
                T_camera_to_world[:3, 3] = np.array([tx, ty, tz])
                node.set_camera_pose(T_camera_to_world)
                self._trajectory_nodes.append(node)
