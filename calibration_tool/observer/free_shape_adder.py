from typing import Callable, List

import numpy as np
from PyQt5.QtCore import QPoint
from PyQt5.QtGui import QColor

from actor.shape_actor import ShapeActor
from observer.base_observer import BaseObserver
from observer.event.events import CustomEvent
from shape.point_2d import Point2D
from shape.polyline_2d import Polyline2D
from shape.shape import Shape


class FreeShapeAdder(BaseObserver):

    def __init__(self, editor: 'MapBasedCalibrator'):
        super().__init__(editor)

        self.QT_EVENT_CALLBACK_PRIORITY_TUPLES = [
            (CustomEvent.LeftButtonPressedEvent,
             self.on_left_button_press, 0),
            (CustomEvent.RightButtonPressedEvent,
             self.on_right_button_press, 0),
            (CustomEvent.MouseMoveEvent, self.on_mouse_move, 0),
        ]

        self.deactivate()

        self._in_progress_shape = None  # type: Shape

        self._anchor_actor = ShapeActor()

        # TODO: Make actor register to renderer more consistent.
        self._in_progress_shape_actor = None  # type: ShapeActor

        self._keypoint_count = np.inf  # type: int

        self._keypoint_count_reached_callback = None  # type: Callable

        # Store correspondence points temporarily.
        self._correspondence_points = []  # type: List[Point2D]

        self._correspondence_actors = []  # type: List[ShapeActor]

        # TODO: Try to remove this variable by more proper
        #  correspondence rendering method.
        self._tmp_correspondence_point = None  # type: Point2D

        self._tmp_correspondence_actor = None  # type: ShapeActor

    def on_left_button_press(self, global_pos: QPoint):
        if not self.is_activated():
            return

        # Judge if mouse position in the canvas.
        mouse_position = self.renderer.canvas().mapFromGlobal(global_pos)
        if not self.renderer.canvas().rect().contains(mouse_position):
            return
        mouse_position_point = \
            self.renderer.camera().get_image_coord_at_mouse(mouse_position)
        mouse_position_in_img = np.array([
            mouse_position_point.x(), mouse_position_point.y()])
        # Snap to closest vector vertex.
        closest_vertex = None
        if self.editor.keyboard_observer.is_shift_pressed():
            closest_vertex = self.editor.closest_vertex_selector \
                .find_closest_vertex_to_mouse(mouse_position_in_img)

        if closest_vertex is not None:
            self._correspondence_points.append(closest_vertex)
            if len(self._correspondence_points) == self._keypoint_count:
                if callable(self._keypoint_count_reached_callback):
                    self._keypoint_count_reached_callback()
                    return
        else:
            if self._in_progress_shape.size() - \
                    len(self._correspondence_points) > 1:
                print('There is point without correspondence.')
                return
            if not self._in_progress_shape.empty():
                tail_vertex = self._in_progress_shape.tail_vertex()
                self._in_progress_shape.pop_vertex()
                self._in_progress_shape.add_vertex(mouse_position_in_img)
                # if self._in_progress_shape.size() == self._keypoint_count:
                #     if callable(self._keypoint_count_reached_callback):
                #         self._keypoint_count_reached_callback()
                # else:
                self._in_progress_shape.add_vertex(tail_vertex)
            else:
                self._in_progress_shape.add_vertex(mouse_position_in_img)

        if self._in_progress_shape is not None:
            # The `self._in_progress_shape` may be cleared in
            # `self._key_point_count_reached_callback`, so we need test
            # it before calling `self._build_shape_actor()`.
            self._build_shape_actor()
            self._build_correspondence_actors()

        self._show_anchor_actor(mouse_position_in_img)

        self.update()

    def on_right_button_press(self, global_pos: QPoint):
        if not self.is_activated():
            return

        # Judge if mouse position in the canvas.
        mouse_position = self.renderer.canvas().mapFromGlobal(global_pos)
        if not self.renderer.canvas().rect().contains(mouse_position):
            return
        mouse_position_point = \
            self.renderer.camera().get_image_coord_at_mouse(mouse_position)
        mouse_position_in_img = np.array([
            mouse_position_point.x(), mouse_position_point.y()])

        if self.editor.keyboard_observer.is_control_pressed():
            self._in_progress_shape.clear()
            self._correspondence_points.clear()
        else:
            if self._in_progress_shape.size() > 2:
                last_vertex = self._in_progress_shape.tail_vertex()
                self._in_progress_shape.pop_vertex()
                self._in_progress_shape.pop_vertex()
                self._in_progress_shape.add_vertex(last_vertex)
                if self._in_progress_shape == len(self._correspondence_points):
                    self._correspondence_points.pop()
            else:
                self._in_progress_shape.clear()
                self._correspondence_points.clear()

        self._build_shape_actor()
        self._build_correspondence_actors()
        self._show_anchor_actor(mouse_position_in_img)

        self.update()

    def on_mouse_move(self, global_pos: QPoint):
        if not self.is_activated():
            return

        # Judge if mouse position in the canvas.
        mouse_position = self.renderer.canvas().mapFromGlobal(global_pos)
        if not self.renderer.canvas().rect().contains(mouse_position):
            return

        mouse_position_point = \
            self.renderer.camera().get_image_coord_at_mouse(mouse_position)
        mouse_position_in_img = np.array([
            mouse_position_point.x(), mouse_position_point.y()])

        if not self._in_progress_shape.empty():
            # Snap to closest vector vertex.
            if self.editor.keyboard_observer.is_shift_pressed():
                closest_vertex = self.editor.closest_vertex_selector \
                    .find_closest_vertex_to_mouse(mouse_position_in_img)
                if closest_vertex is not None:
                    self._tmp_correspondence_point = closest_vertex
            else:
                self._tmp_correspondence_point = None

            if self._in_progress_shape.size() == 1:
                self._in_progress_shape.add_vertex(mouse_position_in_img)
            else:
                self._in_progress_shape.pop_vertex()
                self._in_progress_shape.add_vertex(mouse_position_in_img)

            self._build_shape_actor()
            self._build_correspondence_actors()
        self._show_anchor_actor(mouse_position_in_img)

        self.update()

    def _build_shape_actor(self):
        if self._in_progress_shape is None:
            return
        self.renderer.remove_actor(self._in_progress_shape_actor)
        if self._tmp_correspondence_point is not None and not self._in_progress_shape.empty():
            tail_point = self._in_progress_shape.tail_vertex()
            self._in_progress_shape.pop_vertex()
        self._in_progress_shape_actor = \
            self._in_progress_shape.build_actor()
        if self._tmp_correspondence_point is not None and not self._in_progress_shape.empty():
            self._in_progress_shape.add_vertex(tail_point)
        self._in_progress_shape_actor.property().set_color(255, 0, 0)
        self._in_progress_shape_actor.set_point_color((255, 0, 0))
        self._in_progress_shape_actor.set_point_size(2.0)
        self.renderer.add_actor(self._in_progress_shape_actor)

    def _build_correspondence_actors(self):
        for actor in self._correspondence_actors:
            self.renderer.remove_actor(actor)
        self._correspondence_actors = []
        for idx, point in enumerate(self._correspondence_points):
            src_point = self._in_progress_shape.coords()[idx, :]
            dst_point = point.position()
            line = Polyline2D(np.vstack([[src_point], [dst_point]]))
            correspondence_actor = line.build_actor()
            correspondence_actor.property().set_color(255, 255, 0)
            correspondence_actor.set_point_size(0.01)
            self._correspondence_actors.append(correspondence_actor)
            self.renderer.add_actor(correspondence_actor)

        self.renderer.remove_actor(self._tmp_correspondence_actor)
        if self._tmp_correspondence_point is not None:
            src_index = -2 if self._in_progress_shape.size() > 1 else -1
            src_point = self._in_progress_shape.coords()[src_index, :]
            dst_point = self._tmp_correspondence_point.position()
            line = Polyline2D(np.vstack([[src_point], [dst_point]]))
            self._tmp_correspondence_actor = line.build_actor()
            self._tmp_correspondence_actor.property().set_color(255, 255, 0)
            self._tmp_correspondence_actor.set_point_size(0.01)
            self.renderer.add_actor(self._tmp_correspondence_actor)
        else:
            self._tmp_correspondence_actor = None

    def set_keypoint_count(self,
                           num_keypoints: int,
                           keypoint_count_reached_callback: Callable = None):
        self._keypoint_count = num_keypoints
        self._keypoint_count_reached_callback = keypoint_count_reached_callback

    def clear(self):
        self.renderer.remove_actor(self._in_progress_shape_actor)
        for actor in [*self._correspondence_actors,
                      self._tmp_correspondence_actor]:
            self.renderer.remove_actor(actor)
        self.renderer.remove_actor(self._anchor_actor)
        self._anchor_actor = ShapeActor()
        self._in_progress_shape = None
        self._in_progress_shape_actor = None
        self._correspondence_points = []
        self._correspondence_actors = []
        self._tmp_correspondence_point = None
        self._tmp_correspondence_actor = None
        self._keypoint_count = np.inf
        self._keypoint_count_reached_callback = None

    def set_shape(self, shape: Shape):
        self._in_progress_shape = shape
        self._build_shape_actor()

    def shape(self) -> Shape:
        return self._in_progress_shape

    def gen_correspondence_shape(self) -> Shape:
        correspondence_shape = self._in_progress_shape.__class__()
        correspondence_shape.set_coords(np.vstack([
            point.position() for point in self._correspondence_points
        ]))
        correspondence_shape.set_origin_vertices(np.vstack([
            point.origin_position() for point in self._correspondence_points
        ]))
        return correspondence_shape

    def _show_anchor_actor(self, image_coord: np.ndarray(shape=(2,))):
        self.renderer.remove_actor(self._anchor_actor)
        self._anchor_actor.geometry().set_image_coords(
            image_coord.reshape(1, -1))
        self._anchor_actor.set_point_size(5)
        self._anchor_actor.set_point_color((54, 191, 153))
        self._anchor_actor.set_point_filling(False)
        self._anchor_actor.set_point_thickness(3)
        self.renderer.add_actor(self._anchor_actor)
