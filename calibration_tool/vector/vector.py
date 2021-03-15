

import numpy as np


class Vector(object):
    """
    This class is used to represent a 3D vector element.
    """
    def __init__(self, vertices=np.zeros((0, 3))):
        self._vertices = vertices

    def vertices(self) -> np.zeros((0, 3)):
        return self._vertices

    def size(self) -> int:
        return len(self._vertices)
