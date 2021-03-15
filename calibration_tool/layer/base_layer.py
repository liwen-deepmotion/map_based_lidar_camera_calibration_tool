

from typing import TYPE_CHECKING

from actor.renderer import Renderer

if TYPE_CHECKING:
    from map_based_calibrator import MapBasedCalibrator


class BaseLayer(object):

    def __init__(self, editor: 'MapBasedCalibrator'):
        self.editor = editor

        self.file_path = str()

    @property
    def renderer(self) -> Renderer:
        return self.editor.main_canvas_widget.renderer()
