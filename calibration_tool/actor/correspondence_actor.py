from typing import Tuple

import numpy as np
from PyQt5.QtCore import QPointF
from PyQt5.QtGui import QColor, QPainter, QPolygon, QPainterPath

from actor.shape_actor import ShapeActor


class CorrespondenceActor(ShapeActor):
    # TODO: Remove this actor and refactor shape actor.
    def __init__(self):
        super().__init__()

        self._correspondence_color = QColor(255, 255, 0)

        self._filled = True

    def set_correspondence_coords(self, coords: np.ndarray(shape=(0, 2))):
        self._geometry.set_extra_coords(coords)

    def set_correspondence_color(self, color: Tuple[int, int, int]):
        self._correspondence_color.setRgb(*color)

    def set_filled(self, is_filled: bool):
        self._filled = is_filled

    def render(self, painter: QPainter):
        if painter is None:
            return
        super().render(painter)

        vertices = self.geometry().data()

        self.set_boundary(painter, color=self._correspondence_color)
        self.set_filling(painter)
        for idx, correspondence_coord in \
                enumerate(self._geometry.extra_coords()):
            src_point = QPointF(*vertices[idx, :])
            dst_point = QPointF(*correspondence_coord)
            painter.drawLine(src_point, dst_point)
        # self.set_boundary(painter, color=QColor(155,0,191,255))
        # self.set_filling(painter, color=QColor(155, 0, 191, 255))
        if self._filled:
            for idx in range(len(self._geometry.extra_coords())-1):
                path = QPainterPath()
                point_0 = QPointF(*vertices[idx, :])
                path.moveTo(point_0)
                point_1 = QPointF(*vertices[idx + 1, :])
                path.lineTo(point_1)
                point_2 = QPointF(*self._geometry.extra_coords()[idx + 1, :])
                path.lineTo(point_2)
                point_3 = QPointF(*self._geometry.extra_coords()[idx, :])
                path.lineTo(point_3)
                path.lineTo(point_0)
                painter.fillPath(path, QColor(0, 191, 255, 100))

