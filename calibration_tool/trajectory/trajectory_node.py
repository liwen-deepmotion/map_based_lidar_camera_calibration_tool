# Author: Liwen Liu (liwenliu@deepmotion.ai)


import os

import numpy as np
from PyQt5.QtCore import QRect
from PyQt5.QtGui import QImage

from actor.image_actor import ImageActor
from trajectory.camera_config import CameraConfig


class TrajectoryNode(object):
    """
    This class is used to store trajectory (or trajectory path) and related camera
    pose.
    """
    def __init__(self):
        self._image_dir = str()

        self._timestamp = int()

        self._camera_pose = np.zeros((4, 4))

        self._camera_config = CameraConfig()

    def set_image_dir(self, image_dir: str):
        self._image_dir = image_dir

    def set_timestamp(self, timestamp: int):
        self._timestamp = timestamp

    def timestamp(self) -> int:
        return self._timestamp

    def image_path(self) -> str:
        return os.path.join(self._image_dir, str(self._timestamp) + '.jpg')

    def image_actor(self) -> ImageActor:
        image = QImage()
        image.load(self.image_path())
        image_actor = ImageActor()
        image_actor.geometry().set_qt_geometry(
            QRect(0, 0, image.width(), image.height()))
        image_actor.set_image(image)
        return image_actor

    def T_camera_to_world(self):
        return self._camera_pose

    def set_camera_pose(self, matrix: np.ndarray(shape=(4,4))):
        self._camera_pose = matrix

    def camera_config(self):
        return self._camera_config

    def set_camera_config(self, camera_config: CameraConfig):
        self._camera_config = camera_config
