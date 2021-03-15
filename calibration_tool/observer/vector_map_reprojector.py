# Author: Liwen Liu (liwenliu@deepmotion.ai)


from typing import List, Type, Union

import numpy as np

from actor.shape_actor import ShapeActor
from observer.event.events import CustomEvent
from observer.base_observer import BaseObserver
from shape.point_2d import Point2D
from shape.polygon_2d import Polygon2D
from shape.polyline_2d import Polyline2D
from shape.shape import Shape
from trajectory.camera_config import CameraConfig
from vector.point_3d import Point3D
from vector.polygon_3d import Polygon3D
from vector.polyline_3d import Polyline3D
from vector.vector import Vector


class VectorMapReprojector(BaseObserver):

    def __init__(self, editor: 'MapBasedCalibrator'):
        super().__init__(editor)

        self._shape_actors = []  # type: List[ShapeActor]

        self.QT_EVENT_CALLBACK_PRIORITY_TUPLES = [
            (CustomEvent.VectorMapLoadedEvent,
             self.on_vector_map_loaded, 0),
            (CustomEvent.TrajectoryNodeChangedEvent,
             self.on_trajectory_node_changed, 0),
            (CustomEvent.CalibrationOptimizedEvent,
             self.on_calibration_optimized, 1),
        ]

        self.QT_SIGNAL_CALLBACK_TUPLES = [
            (self.editor.side_bar_widget.show_vector_map_checkbox.stateChanged,
             self.toggle_vector_map_visibility),
        ]

        self._vector_to_color_map = {}

    def toggle_vector_map_visibility(self, is_checked: bool):
        if is_checked:
            self._reproject()
        else:
            self._clear_shape_actors()
        self.update()

    def on_vector_map_loaded(self):
        self._prepare_vector_random_colors()
        # self._reproject()
        # self.update()
        self.editor.side_bar_widget.show_vector_map_checkbox.setChecked(True)

    def on_trajectory_node_changed(self):
        if not self.editor.side_bar_widget \
                .show_vector_map_checkbox.isChecked():
            return
        self._reproject()
        # self.update()

    def on_calibration_optimized(self):
        if not self.editor.side_bar_widget \
                .show_vector_map_checkbox.isChecked():
            return
        self._reproject()
        self.update()

    def _reproject(self):
        if self.editor.layer_manager.vector_map_layer() is None:
            return
        if self.editor.layer_manager.trajectory_layer() is None:
            return
        if self.editor.trajectory_navigator.current_trajectory_node() is None:
            return

        camera_extrinsic = self.editor.trajectory_navigator \
            .current_trajectory_node().T_camera_to_world()
        camera_intrinsics = \
            self.editor.layer_manager.trajectory_layer().camera_config()

        reprojected_shapes = self._reproject_vectors(
            self.editor.layer_manager.vector_map_layer().vectors(),
            camera_extrinsic, camera_intrinsics)

        self._build_shape_actors(reprojected_shapes)

        self.editor.layer_manager.vector_map_layer().set_shapes(
            reprojected_shapes)

    def _prepare_vector_random_colors(self):
        if self.editor.layer_manager.vector_map_layer() is None:
            return
        vectors = self.editor.layer_manager.vector_map_layer().vectors()
        for vector in vectors:
            self._vector_to_color_map[vector] = \
                (255 * np.random.random((3,))).astype(int)

    def _reproject_vectors(
            self,
            vectors: List[Vector],
            camera_extrinsic: np.ndarray(shape=(4, 4)),
            camera_intrinsics: CameraConfig) -> List[Shape]:
        vector_reprojector = VectorReprojector()
        vector_reprojector.set_intrinsics(camera_intrinsics)
        vector_reprojector.set_extrinsic(camera_extrinsic)
        reprojected_shapes = []
        for vector in vectors:
            reprojected_shape = vector_reprojector.reproject(vector)
            if reprojected_shape is not None:
                reprojected_shape.color = self._vector_to_color_map[vector]
                reprojected_shapes.append(reprojected_shape)
        return reprojected_shapes

    def _build_shape_actors(self, shapes: List[Shape]):
        self._clear_shape_actors()
        for shape in shapes:
            shape_actor = shape.build_actor()
            if shape_actor is not None:
                shape_actor.property().set_color(0, 255, 127)
                shape_actor.property().set_line_width(5)
                shape_actor.set_point_color(shape.color)
                self.renderer.add_actor(shape_actor)
                self._shape_actors.append(shape_actor)

    def _clear_shape_actors(self):
        for shape_actor in self._shape_actors:
            self.renderer.remove_actor(shape_actor)
        self._shape_actors = []


class VectorReprojector(object):

    def __init__(self):
        self._T_world_to_camera = np.eye(4)

        self._K = np.eye(3)

        self._image_width = int()

        self._image_height = int()

    def set_intrinsics(self, camera_config: CameraConfig):
        self._K = camera_config.intrinsic_matrix()
        self._image_width = camera_config.w
        self._image_height = camera_config.h

    def set_extrinsic(self, T_camera_to_world: np.ndarray(shape=(4,4))):
        self._T_world_to_camera = np.linalg.inv(T_camera_to_world)

    def reproject(self, vector: Vector) -> Union[Shape, None]:
        original_vertices = vector.vertices().copy()
        # Convert to camera coord.
        projected_vertices = \
            original_vertices.dot(self._T_world_to_camera[:3, :3].T) + \
            self._T_world_to_camera[:3, 3].T

        # Filter backward vertices.
        forward_mask = projected_vertices[:, 2] > 0.0
        projected_vertices = projected_vertices[forward_mask]
        original_vertices = original_vertices[forward_mask]

        # Project to image coord.
        projected_vertices = projected_vertices.dot(self._K.T)
        projected_vertices = \
            projected_vertices[:, :2] / projected_vertices[:, [2]]

        # Clipping.
        clipping_mask = \
            (projected_vertices[:, 0] >= -500) & \
            (projected_vertices[:, 1] >= -500) & \
            (projected_vertices[:, 0] < self._image_width + 500) & \
            (projected_vertices[:, 1] < self._image_height + 500)
        projected_vertices = projected_vertices[clipping_mask]
        original_vertices = original_vertices[clipping_mask]

        if isinstance(vector, (Polyline3D, Polygon3D)) and \
                len(projected_vertices) < 2:
            # Make sure at least show a segment of polyline and polygon
            # vector.
            return
        elif isinstance(vector, Point3D) and len(projected_vertices) == 0:
            # Make sure the point correspondence in the screen.
            return

        shape_class = self.get_shape_class(vector.__class__)
        shape = shape_class(projected_vertices)
        shape.set_origin_vertices(original_vertices)
        return shape


    @staticmethod
    def get_shape_class(vector_class: Type[Vector]):
        vector_class_to_shape_class_map = {
            Vector: Shape,
            Polyline3D: Polyline2D,
            Polygon3D: Polygon2D,
            Point3D: Point2D
        }
        return vector_class_to_shape_class_map[vector_class]
