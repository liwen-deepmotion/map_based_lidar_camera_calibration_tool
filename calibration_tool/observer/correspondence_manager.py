from typing import List

from PyQt5.QtCore import QRectF

from actor.text_actor import TextActor
from config.hot_key import KeyCombo, HotKey
from observer.base_observer import BaseObserver
from observer.event.events import CustomEvent
from observer.vector_map_reprojector import VectorReprojector
from vector.vector import Vector


class CorrespondenceManager(BaseObserver):
    # TODO: Propose a better class name, and try to clean the logic of
    #  this part.
    """
    This class is mainly used to reproject correspondences loaded from
    old json files in which correspondences are without reprojected
    correspondence coords, and refresh correspondences on changing node.
    """

    def __init__(self, editor: 'MapBasedCalibrator'):
        super().__init__(editor)

        self.QT_EVENT_CALLBACK_PRIORITY_TUPLES = [
            (CustomEvent.CorrespondencesLoadedEvent,
             self.on_correspondences_loaded, 0),
            (CustomEvent.TrajectoryNodeChangedEvent,
             self.on_trajectory_node_changed, 0),
            (CustomEvent.CalibrationOptimizedEvent,
             self.on_calibration_optimized, 0),
            (CustomEvent.KeyComboPressedEvent,
             self.on_key_press, 0),
        ]

        self._correspondence_id_actors = []  # type: List[TextActor]

    def on_key_press(self, key_combo: KeyCombo):
        if key_combo.is_same(HotKey.TOGGLE_CORRESPONDENCE_ID_DISPLAYING.value):
            self._toggle_correspondence_displaying()

    def on_correspondences_loaded(self):
        self._update_correspondence_reprojected_coords()
        self._prepare_correspondence_actors()
        self._enable_current_frame_correspondences()
        self.editor.side_bar_widget \
            .show_correspondences_checkbox.setChecked(True)
        self.update()

    def on_trajectory_node_changed(self):
        self._enable_current_frame_correspondences()
        self.update()

    def on_calibration_optimized(self):
        self._update_correspondence_reprojected_coords()
        self._prepare_correspondence_actors()
        self.update()

    def _update_correspondence_reprojected_coords(self):
        if self.editor.layer_manager.trajectory_layer() is None:
            return

        if self.editor.layer_manager.correspondence_layer(
                create_new_layer=False) is None:
            return
        unreprojected_correspondences = \
            self.editor.layer_manager.correspondence_layer().correspondences()

        if len(unreprojected_correspondences) == 0:
            return
        camera_intrinsics = \
            self.editor.layer_manager.trajectory_layer().camera_config()

        vector_reprojector = VectorReprojector()
        vector_reprojector.set_intrinsics(camera_intrinsics)
        for correspondence in unreprojected_correspondences:
            camera_extrinsic = self.editor.layer_manager.trajectory_layer() \
                .get_node_by_timestamp(correspondence.timestamp()).T_camera_to_world()
            vector_reprojector.set_extrinsic(camera_extrinsic)
            reprojected_shape = vector_reprojector.reproject(
                Vector(correspondence.reprojected_shape().origin_vertices()))
            if reprojected_shape is not None:
                correspondence.set_reprojected_shape(reprojected_shape)

    def _prepare_correspondence_actors(self):
        if self.editor.layer_manager.correspondence_layer(
                create_new_layer=False) is None:
            return
        for correspondence in self.editor.layer_manager \
                .correspondence_layer().correspondences():
            correspondence.build_actor()

    def _enable_current_frame_correspondences(self):
        if self.editor.layer_manager.trajectory_layer() is None:
            return
        if self.editor.trajectory_navigator.current_trajectory_node() is None:
            return
        if self.editor.layer_manager.correspondence_layer(
                create_new_layer=False) is None:
            return

        current_timestamp = self.editor.trajectory_navigator \
            .current_trajectory_node().timestamp()

        correspondences = \
            self.editor.layer_manager.correspondence_layer().correspondences()
        for correspondence in correspondences:
            if correspondence.timestamp() == current_timestamp:
                self.renderer.add_actor(correspondence.actor())
            else:
                self.renderer.remove_actor(correspondence.actor())

    def _toggle_correspondence_displaying(self):
        # FIXME: Support dynamic id displaying in the future.
        if self.editor.layer_manager.correspondence_layer(
                create_new_layer=False) is None:
            return
        if len(self._correspondence_id_actors) > 0:
            for correspondence_id_actor in self._correspondence_id_actors:
                self.renderer.remove_actor(correspondence_id_actor)
            self._correspondence_id_actors.clear()
        else:
            for correspondence in self.editor.layer_manager \
                    .correspondence_layer().correspondences():
                id_actor = TextActor()
                id_actor.set_text(str(correspondence.id()))
                id_actor.property().set_color(54, 191, 153)
                screen_position = \
                    self.renderer.camera().transform_geometry(
                        correspondence.actor().geometry()).data()[0, :]
                id_actor.geometry().set_qt_geometry(
                    QRectF(screen_position[0], screen_position[1], 30, 30)
                )
                self.renderer.add_actor(id_actor)
                self._correspondence_id_actors.append(id_actor)
        self.update()
