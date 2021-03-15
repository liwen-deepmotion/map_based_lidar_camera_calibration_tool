from typing import TYPE_CHECKING

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QWheelEvent, QMouseEvent

from observer.base_observer import BaseObserver
from observer.event.events import CustomEvent

if TYPE_CHECKING:
    from map_based_calibrator import MapBasedCalibrator


class MouseObserver(BaseObserver):

    def __init__(self, editor: 'MapBasedCalibrator'):
        super().__init__(editor)

        self.QT_EVENT_CALLBACK_PRIORITY_TUPLES = [
            (CustomEvent.MouseButtonPress, self.on_mouse_button_click, 0),
            (CustomEvent.MouseButtonRelease, self.on_mouse_button_release, 0),
            (CustomEvent.MouseMove, self.on_mouse_move, 0),
            (CustomEvent.Wheel, self.on_wheel, 0),
        ]

    def on_mouse_button_click(self, event: QMouseEvent):
        if event.button() == Qt.LeftButton:
            self.invoke_event(CustomEvent.LeftButtonPressedEvent,
                              event.globalPos())
        elif event.button() == Qt.RightButton:
            self.invoke_event(CustomEvent.RightButtonPressedEvent,
                              event.globalPos())
        elif event.button() == Qt.MidButton:
            self.invoke_event(CustomEvent.MiddleButtonPressedEvent,
                              event.globalPos())

    def on_mouse_button_release(self, event: QMouseEvent):
        if event.button() == Qt.LeftButton:
            self.invoke_event(CustomEvent.LeftButtonReleasedEvent,
                              event.globalPos())
        elif event.button() == Qt.RightButton:
            self.invoke_event(CustomEvent.RightButtonReleasedEvent,
                              event.globalPos())
        elif event.button() == Qt.MidButton:
            self.invoke_event(CustomEvent.MiddleButtonReleasedEvent,
                              event.globalPos())

    def on_mouse_move(self, event: QMouseEvent):
        self.invoke_event(CustomEvent.MouseMoveEvent, event.globalPos())

    def on_wheel(self, event: QWheelEvent):
        self.invoke_event(
            CustomEvent.MouseWheelEvent,
            global_pos=event.globalPos(),
            num_steps=self.get_wheel_rotated_steps(event))

    @staticmethod
    def get_wheel_rotated_steps(event: QWheelEvent) -> float:
        """
        Most mouse types work in steps of 15 degrees, in which case the
        delta value is a multiple of 120; i.e., 120 units * 1/8 = 15
        degrees. https://doc.qt.io/qt-5/qwheelevent.html#angleDelta
        """
        delta_in_degree = event.angleDelta().y() / 8.
        return delta_in_degree / 15
