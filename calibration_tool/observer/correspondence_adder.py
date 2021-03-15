from PyQt5.QtWidgets import QMessageBox

from config.hot_key import KeyCombo, HotKey
from correspondence.correspondence import Correspondence
from observer.base_observer import BaseObserver
from observer.event.events import CustomEvent
from observer.free_shape_adder import FreeShapeAdder
from shape.polyline_2d import Polyline2D


class CorrespondenceAdder(BaseObserver):

    def __init__(self, editor: 'MapBasedCalibrator'):
        super().__init__(editor)

        self.QT_EVENT_CALLBACK_PRIORITY_TUPLES = [
            (CustomEvent.KeyComboPressedEvent, self.on_key_press, 0),
            (CustomEvent.TrajectoryNodeChangedEvent,
             self.on_trajectory_node_changed, 0),
        ]
        self.QT_SIGNAL_CALLBACK_TUPLES = [
            (self.editor.side_bar_widget.add_correspondence_btn.clicked,
             self.on_add_correspondence_btn_clicked),
        ]

        self.deactivate()

    def shape_adder(self) -> FreeShapeAdder:
        return self.editor.free_shape_adder

    def on_key_press(self, key_combo: KeyCombo):
        if key_combo.is_same(HotKey.ADD_CORRESPONDENCE.value):
            if not self.is_activated():
                self._start_adding()
            else:
                self._finish_adding()
        elif key_combo.is_same(HotKey.CANCEL.value):
            self._cancel_adding()

    def on_trajectory_node_changed(self):
        if self.is_activated():
            self._cancel_adding()

    def on_add_correspondence_btn_clicked(self):
        if not self.is_activated():
            self._start_adding()

    def _start_adding(self):
        self.activate()
        self.shape_adder().clear()
        # As only the point and line correspondences needed, a line
        # shape is enough.
        line = Polyline2D()
        line.color = (255, 0, 0)
        self.shape_adder().set_shape(line)
        self.shape_adder().set_keypoint_count(2, self._finish_adding)

        self.shape_adder().activate()
        self.update()

    def _finish_adding(self):
        if not self.is_activated():
            return

        shape = self.shape_adder().shape()
        shape.pop_vertex()

        correspondence = Correspondence()
        correspondence.set_shape(shape)
        correspondence.set_reprojected_shape(
            self.shape_adder().gen_correspondence_shape())
        correspondence.set_timestamp(
            self.editor.trajectory_navigator
                .current_trajectory_node().timestamp())
        correspondence.set_is_on_road(self._get_is_on_road_from_user())

        if not correspondence.is_valid():
            print('Invalid correspondence.')
            return
        self.editor.layer_manager \
            .correspondence_layer().add_correspondence(correspondence)

        self.shape_adder().clear()
        self.shape_adder().deactivate()
        self.deactivate()
        self.update()

    def _cancel_adding(self):
        if not self.is_activated():
            return

        self.shape_adder().clear()
        self.shape_adder().deactivate()
        self.deactivate()
        self.update()

    def _get_is_on_road_from_user(self):
        # clicked_buton = QMessageBox.question(
        #     self.editor,
        #     '',
        #     'Is correspondence on road ?',
        #     QMessageBox.Yes, QMessageBox.No
        # )
        # if clicked_buton == QMessageBox.Yes:
            return True
        # else:
        #     return False
