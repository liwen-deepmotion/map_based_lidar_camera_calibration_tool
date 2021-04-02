# Author: Liwen Liu (liwenliu@deepmotion.ai)


from typing import Union

from actor.image_actor import ImageActor
from config.hot_key import KeyCombo, HotKey
from observer.event.events import CustomEvent
from trajectory.trajectory_node import TrajectoryNode
from observer.base_observer import BaseObserver


class TrajectoryNavigator(BaseObserver):

    def __init__(self, editor: 'MapBasedCalibrator'):
        super().__init__(editor)

        self._current_trajectory_node_idx = 20

        self._image_actor = ImageActor()

        self.QT_EVENT_CALLBACK_PRIORITY_TUPLES = [
            (CustomEvent.TrajectoryLoadedEvent,
             self.on_trajectory_layer_loaded, 0),
            (CustomEvent.KeyComboPressedEvent,
             self.on_key_press, 0),
        ]
        self.QT_SIGNAL_CALLBACK_TUPLES = [
            (self.editor.side_bar_widget.prev_image_btn.clicked,
             self.goto_prev_node),
            (self.editor.side_bar_widget.next_image_btn.clicked,
             self.goto_next_node),
        ]

    def current_trajectory_node_idx(self) -> int:
        return self._current_trajectory_node_idx

    def set_current_trajectory_node_idx(self, idx: int):
        self._current_trajectory_node_idx = idx

    def current_trajectory_node(self) -> Union[TrajectoryNode, None]:
        trajectory_nodes = self.editor.layer_manager \
            .trajectory_layer().trajectory_nodes()
        if 0 <= self._current_trajectory_node_idx < len(trajectory_nodes):
            return trajectory_nodes[self._current_trajectory_node_idx]

    def prev_trajectory_node(self) -> Union[TrajectoryNode, None]:
        self._current_trajectory_node_idx -= 1
        if self._current_trajectory_node_idx < 0:
            self._current_trajectory_node_idx = 0
        return self.current_trajectory_node()

    def next_trajectory_node(self) -> Union[TrajectoryNode, None]:
        self._current_trajectory_node_idx += 1
        trajectory_nodes = self.editor.layer_manager \
            .trajectory_layer().trajectory_nodes()
        if self._current_trajectory_node_idx > len(trajectory_nodes) - 1:
            self._current_trajectory_node_idx = len(trajectory_nodes) - 1
        return self.current_trajectory_node()

    def on_trajectory_layer_loaded(self):
        if self.current_trajectory_node() is not None:
            self._goto_trajectory_node(self.current_trajectory_node())
            self.editor.camera_controller.reset_camera()
            self.update_index_label()
            self.update()

    def on_key_press(self, key_combo: KeyCombo):
        if key_combo.is_same(HotKey.PREV_FRAME.value):
            self.goto_prev_node()
        elif key_combo.is_same(HotKey.NEXT_FRAME.value):
            self.goto_next_node()

    def goto_prev_node(self):
        prev_node = self.prev_trajectory_node()
        if prev_node is not None:
            self._goto_trajectory_node(prev_node)
            self.invoke_event(CustomEvent.TrajectoryNodeChangedEvent)
            self.editor.camera_controller.reset_camera()
            self.update_index_label()
            self.update()

    def goto_next_node(self):
        next_node = self.next_trajectory_node()
        if next_node is not None:
            self._goto_trajectory_node(next_node)
            self.invoke_event(CustomEvent.TrajectoryNodeChangedEvent)
            self.editor.camera_controller.reset_camera()
            self.update_index_label()
            self.update()

    def _goto_trajectory_node(self, trajectory_node: TrajectoryNode):
        self.renderer.remove_actor(self._image_actor)
        self._image_actor = trajectory_node.image_actor()
        self.renderer.add_actor(self._image_actor)

    def update_index_label(self):
        self.editor.side_bar_widget.image_index_label.setText(
            '{} / {}'.format(
                self._current_trajectory_node_idx + 1,
                len(self.editor.layer_manager
                    .trajectory_layer().trajectory_nodes())))
