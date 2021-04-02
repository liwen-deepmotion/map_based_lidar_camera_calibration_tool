# Author: Liwen Liu (liwenliu@deepmotion.ai)
import json
import os
from typing import List

import numpy as np

from trajectory.camera_config import CameraConfig
from trajectory.camera_trajectory import CameraTrajectory
from trajectory.trajectory_node import TrajectoryNode
from layer.base_layer import BaseLayer


class TrajectoryLayer(BaseLayer):
    """
    This layer is used contain all (camera) trajectory nodes of a camera
    trajectory to exhibit images of trajectory nodes.
    The camera trajectory nodes can be generated from camera trajectory
    file or lidar trajectory file.
    """
    def __init__(self, editor: 'MapBasedCalibrator'):
        super().__init__(editor)

        self._keyframes_dir = str()

        # The lidar trajectory is used not only for generating camera
        # trajectory nodes but also for optimization, so we store the
        # path here. If it is empty, we will force to assign it before
        # optimization.
        self._lidar_trajectory_path = str()

        # When we use lidar trajectory to generate camera trajectory
        # nodes, this path can be empty.
        self._cam_trajectory_path = str()

        self._trajectory_nodes = []  # type: List[TrajectoryNode]

        self._camera_config = CameraConfig()

        # This is the pre-calibrated lidar to camera transform, which
        # can be used as a reference when taking optimized lidar to
        # camera transform into computation.
        self._base_lidar_to_camera_transform = np.zeros((4, 4))

    def trajectory_nodes(self) -> List[TrajectoryNode]:
        return self._trajectory_nodes

    def get_node_by_timestamp(self, timestamp: int) -> TrajectoryNode:
        for node in self.trajectory_nodes():
            if node.timestamp() == timestamp:
                return node

    def camera_config(self) -> CameraConfig:
        return self._camera_config

    def T_lidar_to_camera(self) -> np.ndarray(shape=(4,4)):
        return self._base_lidar_to_camera_transform

    def set_base_lidar_to_camera_transform(self, matrix: np.ndarray(shape=(4,4))):
        self._base_lidar_to_camera_transform = matrix

    def keyframes_dir(self) -> str:
        return self._keyframes_dir

    def set_keyframes_dir(self, keyframes_dir: str):
        self._keyframes_dir = keyframes_dir
        return self

    def set_lidar_trajectory_path(self, lidar_trajectory_path: str):
        self._lidar_trajectory_path = lidar_trajectory_path
        return self

    def set_cam_trajectory_path(self, cam_trajectory_path: str):
        self._cam_trajectory_path = cam_trajectory_path
        return self

    def load(self):
        # FIXME: Load sensor calibrations, make it general later.
        sensor_calibrations_path = os.path.join(os.path.dirname(
            self._keyframes_dir), 'sensor_calibrations.json')
        print(sensor_calibrations_path)
        self.load_camera_config(sensor_calibrations_path)
        self.load_base_lidar_to_camera_transform(sensor_calibrations_path)
        if not os.path.exists(sensor_calibrations_path):
            return

        if self._cam_trajectory_path != '':
            self._load_from_camera_trajectory(self._cam_trajectory_path)
            self.file_path = self._cam_trajectory_path
        elif self._lidar_trajectory_path != '':
            self._load_from_lidar_trajectory(self._lidar_trajectory_path)
            self.file_path = self._lidar_trajectory_path
        else:
            return

    def load_camera_config(self, camera_config_path: str):
        self._camera_config.from_json_file(camera_config_path)

    def load_base_lidar_to_camera_transform(self, config_path: str):
        # FIXME: choose a better method to hold the logic.
        with open(config_path, 'r') as f:
            sensor_calibrations = json.load(f)
        lidar_name = ''
        for dm_device in sensor_calibrations['dm_device']['sensors']:
            if dm_device['type'] == 'lidar':
                lidar_name = dm_device['name']
        T_lidar_to_camera = None
        # Load pre-calibrated relative pose.
        for sensor_calib in sensor_calibrations['calibration']:
            if sensor_calib['sensor1'] == lidar_name:
                T_lidar_to_camera = np.array(
                    sensor_calib['T_sensor1_to_sensor2']).reshape(4, 4)
        if T_lidar_to_camera is not None:
            self._base_lidar_to_camera_transform = T_lidar_to_camera
            return

        # Calculate init pose from device to DM300 calibrations.
        T_lidar_to_device = None
        T_camera_to_device = None
        for dm_device in sensor_calibrations['dm_device']['sensors']:
            if dm_device['type'] == 'lidar':
                T_lidar_to_device = np.array(
                    dm_device['sensor_to_device']).reshape(4, 4)
            elif dm_device['type'] == 'camera':
                T_camera_to_device = np.array(
                    dm_device['sensor_to_device']).reshape(4, 4)
        if T_lidar_to_device is not None and T_camera_to_device is not None:
            self._base_lidar_to_camera_transform = \
                np.linalg.inv(T_camera_to_device).dot(T_lidar_to_device)

    def _load_from_camera_trajectory(self, cam_trajectory_path: str):
        trajectory = CameraTrajectory()
        trajectory.load(cam_trajectory_path)
        self._trajectory_nodes = trajectory.trajectory_nodes()
        for trajectory_node in self._trajectory_nodes:
            trajectory_node.set_image_dir(self._keyframes_dir)
            trajectory_node.set_camera_config(self._camera_config)

    def _load_from_lidar_trajectory(self, lidar_trajectory_path: str):
        pass
