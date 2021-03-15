from typing import TYPE_CHECKING

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QKeyEvent, QKeySequence
from PyQt5.QtWidgets import QApplication

from config.hot_key import HotKey, KeyCombo
from observer.base_observer import BaseObserver
from observer.event.events import CustomEvent

if TYPE_CHECKING:
    from map_based_calibrator import MapBasedCalibrator


class KeyboardObserver(BaseObserver):

    def __init__(self, editor: 'MapBasedCalibrator'):
        super().__init__(editor)

        self.QT_EVENT_CALLBACK_PRIORITY_TUPLES = [
            (CustomEvent.KeyPress, self.on_key_press, 0),
        ]

    def on_key_press(self, event: QKeyEvent) -> None:
        key = event.key()
        if key in [
            Qt.Key_Control,
            Qt.Key_Shift,
            Qt.Key_Alt,
            Qt.Key_Meta
        ]:
            return

        for value, modifier in [
            (Qt.CTRL, Qt.ControlModifier),
            (Qt.SHIFT, Qt.ShiftModifier),
            (Qt.ALT, Qt.AltModifier),
            (Qt.META, Qt.MetaModifier),
        ]:
            if event.modifiers() & modifier:
                key += value

        key_sequence = QKeySequence(key).toString()
        self.invoke_event(
            CustomEvent.KeyComboPressedEvent, KeyCombo(key_sequence))

    def is_control_pressed(self):
        return QApplication.queryKeyboardModifiers() == Qt.ControlModifier

    def is_shift_pressed(self):
        return QApplication.queryKeyboardModifiers() == Qt.ShiftModifier

    def is_alt_pressed(self):
        return QApplication.queryKeyboardModifiers() == Qt.AltModifier
