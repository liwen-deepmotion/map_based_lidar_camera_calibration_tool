

import numpy as np
from PyQt5.QtCore import QPointF, QRect


class Geometry(object):

    def __init__(self,
                 qt_geometry=None,
                 coords=np.zeros((0, 2)),
                 extra_coords=np.zeros((0, 2))):
        # FIXME: inherit to avoid coupling.
        self._qt_geometry = qt_geometry

        # Records the image coordinates of shape points, which could
        # also be canvas coordinates.
        self._coords = coords

        # FIXME: Temporarily for correspondences.
        self._extra_coords = extra_coords

    def set_qt_geometry(self, qt_geometry):
        self._qt_geometry = qt_geometry

    def set_image_coords(self, image_coords: np.ndarray(shape=(0, 2))):
        self._coords = image_coords

    def set_extra_coords(self, extra_coords: np.ndarray(shape=(0, 2))):
        self._extra_coords = extra_coords

    def extra_coords(self) -> np.ndarray(shape=(0, 2)):
        return self._extra_coords

    def data(self):
        if self._qt_geometry is not None:
            return self._qt_geometry
        else:
            return self._coords
