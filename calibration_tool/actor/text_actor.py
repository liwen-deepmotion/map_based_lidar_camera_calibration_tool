from PyQt5.QtCore import QRect, Qt
from PyQt5.QtGui import QImage, QPainter, QColor, QFont

from actor.actor import Actor


class TextActor(Actor):

    def __init__(self):
        super().__init__()

        self.text = str()

        self.text_size = 20

        self._background_rect = None  # type: QRect

        self._background_color = QColor()

    def set_text(self, text: str):
        self.text = text
        return self

    def set_background(self, rect: QRect, color: QColor):
        self._background_rect = rect
        self._background_color = color

    def render(self, painter: QPainter):
        if painter is None:
            return
        if self._background_rect is not None:
            painter.save()
            self.set_boundary(painter)
            self.set_filling(painter, self._background_color, Qt.SolidPattern)
            painter.drawRect(self._background_rect)
            painter.restore()

        painter.save()
        painter.setPen(self.property().color)
        painter.setFont(QFont('Helvetica', self.text_size, QFont.Bold))
        painter.drawText(self.geometry().data(),
                         Qt.AlignVCenter | Qt.AlignHCenter,
                         self.text)
        painter.restore()
