# Author: Liwen Liu (liwenliu@deepmotion.ai)


from typing import Iterable, Tuple, Any, Callable, TYPE_CHECKING

if TYPE_CHECKING:
    from map_based_calibrator import MapBasedCalibrator


class BaseObserver(object):
    """
    This is a base class for all observer classes implemented in the project.
    It provides common convenient interfaces for subclasses to
    (1) Reference the Calibrator object `self.editor`;
    (2) Access the Qt encapsulated interactor, renderer, and an update call to
    trigger repaint;
    (3) Register itself to the Qt signals;
    """

    def __init__(self, editor: 'MapBasedCalibrator'):
        self.editor = editor

        self._is_activated = True

        self.QT_EVENT_CALLBACK_PRIORITY_TUPLES = [
        ]  # type: Iterable[Tuple[int, Callable, int]]

        self.QT_SIGNAL_CALLBACK_TUPLES = [
        ]  # type: Iterable[Tuple[Any, Callable]]

    @property
    def renderer(self):
        return self.editor.main_canvas_widget.renderer()

    def update(self):
        self.editor.main_canvas_widget.custom_update()

    def invoke_event(self, event: int, *args, **kwargs):
        self.editor.event_handler.invoke_event(event, *args, **kwargs)

    def connect_myself_to_qt_signals(self):
        for signal, callback in self.QT_SIGNAL_CALLBACK_TUPLES:
            signal.connect(callback)

    def connect_myself_to_vtk_events(self):
        for event, callback, priority in \
                self.QT_EVENT_CALLBACK_PRIORITY_TUPLES:
            self.editor.event_handler.add_observer(
                event, callback, priority)

    def is_activated(self):
        return self._is_activated

    def activate(self):
        self._is_activated = True

    def deactivate(self):
        self._is_activated = False




