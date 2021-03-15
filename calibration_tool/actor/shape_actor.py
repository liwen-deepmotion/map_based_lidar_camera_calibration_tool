from typing import Tuple

import numpy as np
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QPainter, QBrush, QPen, QPainterPath, QColor

from actor.actor import Actor


class ShapeActor(Actor):

    def __init__(self):
        super().__init__()

        self._point_color = QColor(255, 255, 255)

        self._point_filling = True

        self._point_thickness = 1.0

    def set_point_color(self, color: Tuple[int, int, int]):
        self._point_color.setRgb(*color)

    def set_point_filling(self, is_filling: bool):
        self._point_filling = is_filling

    def set_point_thickness(self, thickness: float):
        self._point_thickness = thickness

    def render(self, painter: QPainter):
        if painter is None:
            return
        vertices = self._geometry.data()

        if len(vertices) == 0:
            return

        # We draw first point alone previously because we want to modify
        # painter style after path created.
        path = QPainterPath()
        first_point = QPointF(vertices[0, 0], vertices[0, 1])
        path.moveTo(first_point)

        # Set points style.
        self.set_boundary(painter, color=self._point_color,
                          line_width=self._point_thickness)
        if self._point_filling:
            self.set_filling(painter,
                             color=self._point_color,
                             style=Qt.SolidPattern)
        else:
            self.set_filling(painter)
        painter.drawEllipse(first_point,
                            self.property().point_size,
                            self.property().point_size)

        for idx, vertex in enumerate(vertices[1:]):
            point = QPointF(vertex[0], vertex[1])
            path.lineTo(point)

            painter.drawEllipse(point,
                                self.property().point_size,
                                self.property().point_size)

        # path.closeSubpath()
        self.set_boundary(painter,
                          self.property().color,
                          self.property().line_width)
        self.set_filling(painter)

        painter.drawPath(path)
