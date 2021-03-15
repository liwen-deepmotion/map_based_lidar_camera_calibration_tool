from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPainter, QPixmap, QBrush, QPen, QColor

from actor.actor import Actor


class ImageActor(Actor):

    def __init__(self):
        super().__init__()

        self.image = QImage()

    def set_image(self, image: QImage):
        self.image = image
        return self

    def render(self, painter: QPainter):
        if painter is None:
            return
        painter.drawImage(self.geometry().data(), self.image)
        mask = self.image.copy()
        mask = QImage(self.image.size(), QImage.Format_ARGB32)
        mask.fill(QColor(0, 0, 0, 100))
        painter.drawImage(self.geometry().data(), mask)
