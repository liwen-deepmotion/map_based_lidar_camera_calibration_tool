from typing import List

from PyQt5.QtCore import QRect
from PyQt5.QtGui import QPixmap, QPainter

from actor.actor import Actor
from actor.camera import Camera
from actor.image_actor import ImageActor
from widget.canvas import Canvas


class Renderer(object):
    """
    The purpose of this class is to capsule Qt paintEvent related
    operations, and make it more object-oriented, referring to some
    other rendering engines like VTK.
    """
    def __init__(self, canvas: Canvas):
        self._actors = []  # type: List[Actor]

        self._canvas = canvas

        self._camera = Camera()

        self._painter = None  # type: QPainter

    def camera(self) -> Camera:
        return self._camera

    def canvas(self) -> Canvas:
        return self._canvas

    def clear(self):
        self._canvas.pix_map.fill(
            self.canvas().palette().color(self.canvas().backgroundRole()))
        self._painter.drawRect(
            QRect(0, 0, self._canvas.width()-1, self._canvas.height()-1))

    def add_actor(self, actor: Actor):
        if isinstance(actor, ImageActor):
            image_rect = actor.geometry().data()
            self._camera.image_size = (image_rect.width(), image_rect.height())
        if actor not in self._actors:
            self._actors.append(actor)

    def remove_actor(self, actor: Actor):
        if actor in self._actors:
            self._actors.remove(actor)

    def render(self):
        self._canvas.pix_map = QPixmap(self._canvas.size())
        if self._painter is None:
            self._painter = QPainter()
        self._painter.begin(self._canvas.pix_map)
        self.clear()
        for actor in self._actors:
            with self._camera.view_actor(actor) as viewing_actor:
                viewing_actor.set_painter(self._painter).render(self._painter)
        self._painter.end()
