

import numpy as np

from actor.shape_actor import ShapeActor
from shape.shape import Shape


class Polyline2D(Shape):

    def __init__(self, coords=np.zeros((0, 2))):
        super().__init__(coords)

    def build_actor(self):
        shape_actor = ShapeActor()
        shape_actor.geometry().set_image_coords(self._coords)
        return shape_actor
