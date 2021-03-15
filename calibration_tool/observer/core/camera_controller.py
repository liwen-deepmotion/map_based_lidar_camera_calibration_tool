

from PyQt5.QtCore import QPointF, QPoint
from PyQt5.QtGui import QWheelEvent, QResizeEvent

from config.hot_key import KeyCombo, HotKey
from observer.base_observer import BaseObserver
from observer.event.events import CustomEvent


class CameraController(BaseObserver):

    def __init__(self, editor: 'MapBasedCalibrator'):
        super().__init__(editor)

        self._last_mouse_position = QPoint()

        self._on_dragging = False

        self.QT_EVENT_CALLBACK_PRIORITY_TUPLES = [
            (CustomEvent.Resize, self.on_window_resize, 0),
            (CustomEvent.MouseWheelEvent, self.on_mouse_wheel, 0),
            (CustomEvent.KeyComboPressedEvent, self.on_key_press, 0),
            (CustomEvent.MiddleButtonPressedEvent, self.start_dragging, 0),
            (CustomEvent.MouseMoveEvent, self.on_mouse_move, 1),
            (CustomEvent.MiddleButtonReleasedEvent, self.end_dragging, 0),
        ]

    def on_window_resize(self, event: QResizeEvent):
        self.reset_camera()
        self.update()

    def on_mouse_wheel(self, global_pos: QPoint, num_steps: int):
        # Judge if mouse position in the canvas.
        mouse_position = \
            self.renderer.canvas().mapFromGlobal(global_pos)
        if not self.renderer.canvas().rect().contains(mouse_position):
            return
        self.renderer.camera().zoom(mouse_position, num_steps)
        self.update()

    def on_key_press(self, key_combo: KeyCombo):
        if key_combo.is_same(HotKey.RESET_CAMERA.value):
            self.reset_camera()
            self.update()

    def on_mouse_move(self, mouse_position: QPoint):
        if self._last_mouse_position.isNull():
            return

        if not self.renderer.canvas().underMouse():
            return

        if self._on_dragging:
            self.renderer.camera().translation += \
                mouse_position - self._last_mouse_position
            self._last_mouse_position = mouse_position
        self.update()

    def start_dragging(self, mouse_position: QPoint):
        self._last_mouse_position = mouse_position
        self._on_dragging = True

    def end_dragging(self, mouse_position: QPoint):
        self._on_dragging = False
        self._last_mouse_position = QPoint()

    def reset_camera(self):
        self._align_image_to_canvas()

    def _align_image_to_canvas(self):
        if not any(self.renderer.camera().image_size):
            return
        image_width, image_height = self.renderer.camera().image_size
        canvas_width, canvas_height = \
            self.renderer.canvas().width(), self.renderer.canvas().height()

        sx = canvas_width / image_width
        sy = canvas_height / image_height

        scale = min(sx, sy)
        self.renderer.camera().scale = scale

        x = (canvas_width - image_width * scale) / 2
        y = (canvas_height - image_height * scale) / 2

        self.renderer.camera().translation = QPointF(x, y)

    @staticmethod
    def get_wheel_rotated_steps(event: QWheelEvent) -> float:
        """
        Most mouse types work in steps of 15 degrees, in which case the
        delta value is a multiple of 120; i.e., 120 units * 1/8 = 15
        degrees. https://doc.qt.io/qt-5/qwheelevent.html#angleDelta
        """
        delta_in_degree = event.angleDelta().y() / 8.
        return delta_in_degree / 15
