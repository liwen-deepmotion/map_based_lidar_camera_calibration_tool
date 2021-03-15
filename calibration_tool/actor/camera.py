

from contextlib import contextmanager

import numpy as np
from PyQt5.QtCore import QPointF, QRect, QPoint, QRectF

from actor.actor import Actor
from actor.geometry import Geometry


class Camera(object):
    """
    The camera class is used to achieve 2D object transformation and
    looking at representations.
    """

    ZOOM_STEP_SIZE = 1.6
    MAX_ZOOM_FACTOR = 20
    MIN_ZOOM_FACTOR = 0.5

    def __init__(self):
        self.translation = QPointF(0, 0)

        self.scale = 1.0

        # This is used to convert coordinate between image and canvas.
        self.image_size = (0, 0)

    @contextmanager
    def view_actor(self, actor: 'Actor') -> 'Actor':
        # Transform geometry.
        actor.backup_geometry()
        actor.set_geometry(self.transform_geometry(actor.geometry()))

        # Change properties.
        origin_point_size = actor.property().point_size
        actor.set_point_size(self.scale * origin_point_size)

        yield actor

        # Restore geometry.
        actor.restore_geometry()

        # Restore properties.
        actor.set_point_size(origin_point_size)

    def transform_geometry(self, geometry: Geometry) -> Geometry:
        old_data = geometry.data()
        if isinstance(old_data, QRect):
            return Geometry(qt_geometry=QRect(
                old_data.x() + self.translation.x(),
                old_data.y() + self.translation.y(),
                old_data.width() * self.scale,
                old_data.height() * self.scale))
        elif isinstance(old_data, (QPointF, QRectF)):
            # Static text.
            return Geometry(qt_geometry=old_data)
        elif isinstance(old_data, np.ndarray):
            new_geometry = Geometry(
                coords=old_data * self.scale +
                       np.array([self.translation.x(), self.translation.y()]))
            extra_coords = geometry.extra_coords()
            if len(extra_coords) > 0:
                new_geometry.set_extra_coords(
                    extra_coords * self.scale +
                    np.array([self.translation.x(), self.translation.y()])
                )
            return new_geometry

    def zoom(self, mouse_position: QPoint, num_steps: int):
        new_scale = self.scale * pow(self.ZOOM_STEP_SIZE, num_steps)
        new_scale = min(max(
            new_scale, self.MIN_ZOOM_FACTOR), self.MAX_ZOOM_FACTOR)
        self.translation = mouse_position + (
                self.translation - mouse_position) * new_scale / self.scale
        self.scale = new_scale

    def get_image_coord_at_mouse(self, canvas_position: QPoint) -> QPointF:
        return QPointF(
            (canvas_position.x() - self.translation.x()) / self.scale,
            (canvas_position.y() - self.translation.y()) / self.scale
        )
