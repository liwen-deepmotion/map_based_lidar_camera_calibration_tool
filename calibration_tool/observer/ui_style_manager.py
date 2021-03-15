# Author: Wei Tan (weitan@deepmotion.ai)


import os

from observer.base_observer import BaseObserver


class UIStyleManager(BaseObserver):
    """
    The UIStyleManager update the editor's UI using the qt style
    file(.qss) on the file changed.
    """

    def __init__(self, editor: 'MapBasedCalibrator'):
        super().__init__(editor)

        self.style_sheet_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
            'ui/resources/stylesheet.qss')

        self.QT_SIGNAL_CALLBACK_TUPLES = [
            (self.editor.file_watcher.fileChanged, self.on_file_changed),
        ]

        self.editor.file_watcher.addPath(self.style_sheet_path)

    def on_file_changed(self, file_path: str):
        if file_path != self.style_sheet_path:
            return
        self.editor.set_stylesheet(self.style_sheet_path)
        self.editor.file_watcher.addPath(self.style_sheet_path)
