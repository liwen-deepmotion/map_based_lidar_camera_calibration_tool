import copy
from typing import Tuple

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush
from PyQt5.QtWidgets import QStyle

from actor.geometry import Geometry
from actor.property import Property


class Actor(object):

    def __init__(self):
        # TODO: Re-design the Geometry and Property.
        self._geometry = Geometry()

        self._property = Property()

        self._backup_geometry = None

    def geometry(self) -> Geometry:
        return self._geometry

    def property(self) -> Property:
        return self._property

    def build(self):
        pass

    def set_painter(self, painter: QPainter):
        painter.setRenderHint(QPainter.Antialiasing)
        self._set_painter_by_properties(painter)
        return self

    def _set_painter_by_properties(self, painter: QPainter):
        self.set_filling(painter)
        self.set_boundary(
            painter,
            color=self.property().color,
            line_width=self.property().line_width,
            style=Qt.SolidLine)

    def set_boundary(self,
                     painter: QPainter,
                     color: QColor = None,
                     line_width: float = None,
                     style: QStyle = None):
        pen_line_width = line_width if line_width is not None \
            else painter.pen().width()
        pen_style = style if style is not None else painter.pen().style()
        if color is not None:
            pen = QPen(color, pen_line_width, pen_style)
        else:
            pen = QPen(Qt.NoPen)
        painter.setPen(pen)

    def set_filling(self,
                    painter: QPainter,
                    color: QColor = None,
                    style: QStyle = None):
        brush_style = style if style is not None else painter.brush().style()
        if color is not None:
            brush = QBrush(color, brush_style)
        else:
            brush = QBrush(Qt.NoBrush)
        painter.setBrush(brush)

    def render(self, painter: QPainter):
        pass

    def set_geometry(self, geometry: Geometry):
        self._geometry = geometry

    def backup_geometry(self):
        self._backup_geometry = copy.deepcopy(self._geometry)

    def restore_geometry(self):
        self._geometry = copy.deepcopy(self._backup_geometry)
        self._backup_geometry = None

    def set_point_size(self, ratio: float):
        self._property.set_point_size(ratio)
