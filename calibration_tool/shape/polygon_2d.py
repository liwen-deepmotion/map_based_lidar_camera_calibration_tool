

import numpy as np

from actor.shape_actor import ShapeActor
from shape.shape import Shape


class Polygon2D(Shape):

    def __init__(self, coord=np.zeros((0, 2))):
        super().__init__(coord)

    def build_actor(self):
        shape_actor = ShapeActor()
        shape_actor.geometry().set_image_coords(
            np.vstack([self._coords, self._coords[0]]))
        return shape_actor
