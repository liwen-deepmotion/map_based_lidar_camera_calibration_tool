#!/usr/bin/env python3
# Author: Zhijun Xie (zhijunxie@deepmotion.ai)


from PyQt5.QtCore import QPointF, QPoint, QSize, QRect, Qt
from PyQt5.QtGui import (QPixmap, QPaintEvent, QPainter, QPen, QColor)
from PyQt5.QtWidgets import QWidget, QStyleOption, QStyle
from typing import Tuple, TYPE_CHECKING


if TYPE_CHECKING:
    from map_based_calibrator import MapBasedCalibrator


class Canvas(QWidget):

    def __init__(self, editor: 'MapBasedCalibrator'):
        super().__init__()
        self.editor = editor  # type: MapBasedCalibrator
        self.setMinimumSize(100, 100)
        self.setMouseTracking(True)

        # When the paint device is a widget, QPainter can only be used
        # inside a paintEvent() function or in a function called by
        # paintEvent(). In order to draw elements outside paintEvent,
        # we need to draw on a middle object that inherits the
        # QPaintDevice class, such as QImage or QPixmap.
        # https://doc.qt.io/qt-5/qpainter.html#details
        self.pix_map = QPixmap()

        self.image_scale = 1
        self.image_translation = QPointF(0, 0)

    def paintEvent(self, event: QPaintEvent) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.SmoothPixmapTransform)

        # The following code enables the custom Canvas2d to use the
        # StyleSheets.
        # https://wiki.qt.io/How_to_Change_the_Background_Color_of_QWidget
        style_option = QStyleOption()
        style_option.initFrom(self)
        self.style().drawPrimitive(
            QStyle.PE_Widget, style_option, painter, self)

        super(Canvas, self).paintEvent(event)
        if self.pix_map.isNull():
            return
        # Test.
        # painter.setPen(QPen(
        #     QColor(255,255,255), 1.0, Qt.SolidLine))
        # painter.drawRect(
        #     QRect(0, 0, self.width()-1, self.height()-1))

        painter.drawPixmap(self.rect(), self.pix_map)

    def map_to_image(self, global_pos: QPoint) -> QPointF:
        pos = self.mapFromGlobal(global_pos)
        return QPointF(
            (pos.x() - self.image_translation.x()) / self.image_scale,
            (pos.y() - self.image_translation.y()) / self.image_scale)

    def map_to_canvas(self, image_coord: Tuple) -> QPointF:
        return QPointF(
            image_coord[0] * self.image_scale +
            self.image_translation.x(),
            image_coord[1] * self.image_scale +
            self.image_translation.y())

    def get_image_coord(self):
        if not self.underMouse():
            return None

        global_pos = self.editor.mouse_positioner.mouse_position
        image_coord = self.map_to_image(global_pos)
        return image_coord

    def sizeHint(self) -> QSize:
        return QSize(800, 800)
