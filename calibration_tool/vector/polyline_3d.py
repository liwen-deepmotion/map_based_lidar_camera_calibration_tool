

import numpy as np

from vector.vector import Vector


class Polyline3D(Vector):

    def __init__(self, vertices=np.zeros((0, 3))):
        super().__init__(vertices)
