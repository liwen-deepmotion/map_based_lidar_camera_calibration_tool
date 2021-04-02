

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

    def head_vertex(self) -> np.ndarray(shape=(3, )):
        return self._vertices[0]

    def tail_vertex(self) -> np.ndarray(shape=(3, )):
        return self._vertices[-1]

    def direction(self) -> np.ndarray(shape=(3,)):
        vec = self.tail_vertex() - self.head_vertex()
        return vec / np.linalg.norm(vec)

    def length(self) -> float:
        return np.linalg.norm(self.tail_vertex() - self.head_vertex())

    def resample(self, gap: float = 1.0):
        sample_step = gap * self.direction()
        num_sampled_points = int(self.length() / gap)
        new_vertices = [self.head_vertex() + sample_step * idx
                        for idx in range(num_sampled_points)]
        new_vertices.append(self.tail_vertex())
        self._vertices = np.vstack(new_vertices)
