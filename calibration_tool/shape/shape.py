

import numpy as np

from actor.shape_actor import ShapeActor


class Shape(object):
    """
    This class is used to represent a 2D shape in image space, and
    generate the shape actor.
    """
    def __init__(self, coords=np.zeros((0, 2))):
        self._coords = coords

        # This field contains relative 3D vector vertices if has any.
        self._origin_vertices = np.zeros((0, 3))

        self.color = (255, 255, 255)

    def coords(self) -> np.ndarray(shape=(0, 2)):
        return self._coords

    def set_coords(self, coords: np.ndarray(shape=(0, 2))):
        self._coords = coords

    def origin_vertices(self) -> np.ndarray(shape=(0, 3)):
        return self._origin_vertices

    def set_origin_vertices(self, vector_vertices: np.ndarray(shape=(0, 3))):
        self._origin_vertices = vector_vertices

    def build_actor(self) -> ShapeActor:
        pass

    def size(self) -> int:
        return len(self._coords)

    def empty(self) -> bool:
        return self.size() == 0

    def clear(self):
        self._coords = np.zeros((0, 2))

    def add_vertex(self, vertex: np.ndarray(shape=(2,))):
        self._coords = np.vstack([self._coords, vertex])

    def pop_vertex(self):
        if not self.empty():
            self._coords = self._coords[:-1, :]

    def head_vertex(self) -> np.ndarray(shape=(2,)):
        return self._coords[0, :]

    def tail_vertex(self) -> np.ndarray(shape=(2,)):
        return self._coords[-1, :]

    def length(self) -> float:
        return np.linalg.norm(self._coords[-1] - self._coords[0])

    def direction(self) -> np.ndarray(shape=(2,)):
        vec = self.tail_vertex() - self.head_vertex()
        return vec / np.linalg.norm(vec)

    def normal(self) -> np.ndarray(shape=(2,)):
        vec = self.direction()
        return np.array([vec[1], -vec[0]])
