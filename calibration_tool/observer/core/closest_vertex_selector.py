from typing import Union

import numpy as np

from observer.base_observer import BaseObserver
from shape.point_2d import Point2D


class ClosestVertexSelector(BaseObserver):
    def __init__(self, editor: 'MapBasedCalibrator'):
        super().__init__(editor)

    def find_closest_vertex_to_mouse(
            self, mouse_position: np.ndarray(shape=(2,)) ) \
            -> Union[Point2D, None]:
        if self.editor.layer_manager.vector_map_layer() is None:
            return
        shapes = \
            self.editor.layer_manager.vector_map_layer().reprojected_shapes()
        if len(shapes) == 0:
            return

        shape_coords = np.vstack([shape.coords() for shape in shapes])
        shape_origin_vertices = \
            np.vstack([shape.origin_vertices() for shape in shapes])

        closest_vertex_idx = np.argmin(np.linalg.norm(
            mouse_position - shape_coords, axis=1))

        closest_vertex = Point2D()
        closest_vertex.set_position(shape_coords[closest_vertex_idx, :])
        closest_vertex.set_origin_position(
            shape_origin_vertices[closest_vertex_idx, :])
        return closest_vertex
