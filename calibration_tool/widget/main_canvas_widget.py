#!/usr/bin/env python3
# Author: Zhijun Xie (zhijunxie@deepmotion.ai)
from PyQt5.QtCore import QEvent
from PyQt5.QtWidgets import QGridLayout, QWidget
from typing import TYPE_CHECKING, List, Dict

from actor.renderer import Renderer
from widget.canvas import Canvas

if TYPE_CHECKING:
    from map_based_calibrator import MapBasedCalibrator


class MainCanvasWidget(QWidget):
    def __init__(self, editor: 'MapBasedCalibrator'):
        super().__init__()
        self.editor = editor  # type: MapBasedCalibrator

        self._canvas = Canvas(editor)
        self._renderer = Renderer(self._canvas)

        self._init_ui()

    def renderer(self) -> Renderer:
        return self._renderer

    def render_actors(self):
        self._renderer.render()

    def _init_ui(self):
        self.setMouseTracking(True)
        self.setLayout(QGridLayout())
        self.layout().addWidget(self._canvas)
        self._canvas.update()

    def custom_update(self):
        self.render_actors()
        self._canvas.update()

    def event(self, event: QEvent) -> bool:
        """
        This function is used to wrap widget mouse move to main window
        as main window 's mouse move missed when put the widget into a
        layout, the reason haven't been found.
        """
        if event.type() in [
            QEvent.MouseMove,
        ]:
            self.editor.event(event)
            # self.editor.event_handler.invoke_event(event.type(), event)

        return super().event(event)
