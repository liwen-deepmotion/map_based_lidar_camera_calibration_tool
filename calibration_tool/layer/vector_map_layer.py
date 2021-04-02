from typing import List, TYPE_CHECKING

from layer.base_layer import BaseLayer
from layer.hd_map_adapter import HdMapAdapter
from shape.shape import Shape
from vector.polyline_3d import Polyline3D
from vector.vector import Vector

if TYPE_CHECKING:
    from map_based_calibrator import MapBasedCalibrator


class VectorMapLayer(BaseLayer):

    def __init__(self, editor: 'MapBasedCalibrator'):
        super().__init__(editor)

        self._vectors = []  # type: List[Vector]

        self._hd_map_adapter = HdMapAdapter()

        self._shapes = []  # type: List[Shape]

    def vectors(self) -> List[Vector]:
        return self._vectors

    def polylines(self) -> List[Polyline3D]:
        return [vector for vector in self._vectors
                if isinstance(vector, Polyline3D)]

    def reprojected_shapes(self) -> List[Shape]:
        return self._shapes

    def set_shapes(self, shapes: List[Shape]):
        self._shapes = shapes

    def load(self, vector_map_file_path: str):
        self._vectors = self._hd_map_adapter.load(vector_map_file_path)
