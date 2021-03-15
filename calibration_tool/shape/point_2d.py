

import numpy as np

from shape.shape import Shape


class Point2D(Shape):

    def __init__(self, coords=np.zeros((0, 2))):
        super().__init__(coords)

    def set_position(self, position: np.zeros((2,))):
        self._coords = position.reshape(1, -1)

    def set_origin_position(self, position: np.zeros((3, ))):
        self._origin_vertices = position.reshape(1, -1)

    def position(self) -> np.ndarray(shape=(2,)):
        return self._coords[0]

    def origin_position(self) -> np.ndarray(shape=(3,)):
        return self._origin_vertices[0]
