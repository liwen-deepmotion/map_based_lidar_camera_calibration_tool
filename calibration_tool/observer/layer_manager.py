# Author: Liwen Liu (liwenliu@deepmotion.ai)


import os

from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QMessageBox

from layer.correspondence_layer import CorrespondenceLayer
from layer.trajectory_layer import TrajectoryLayer
from layer.vector_map_layer import VectorMapLayer
from observer.base_observer import BaseObserver
from observer.event.events import CustomEvent


class LayerManager(BaseObserver):

    def __init__(self, editor: 'MapBasedCalibrator'):
        super().__init__(editor)

        self._trajectory_layer = None  # type: TrajectoryLayer

        self._vector_map_layer = None  # type: VectorMapLayer

        self._correspondence_layer = None  # type: CorrespondenceLayer

        self.QT_SIGNAL_CALLBACK_TUPLES = [
            # (self.editor.actionLoadImages.triggered, self.on_load_images),
            (self.editor.side_bar_widget.load_images.clicked,
             self.on_load_images),
            # (self.editor.actionLoadVectorMap.triggered,
            #  self.on_load_vector_map),
            (self.editor.side_bar_widget.load_vector_map.clicked,
             self.on_load_vector_map),
            (self.editor.actionLoadCorrespondences.triggered,
             self.on_load_correspondences),
            # (self.editor.side_bar_widget.load_correspondences_btn.clicked,
            #  self.on_load_correspondences),
            # (self.editor.actionSaveCorrespondences.triggered,
            #  self.on_save_correspondences),
            (self.editor.side_bar_widget.save_correspondences_btn.clicked,
             self.on_save_correspondences)
        ]

    def trajectory_layer(self) -> TrajectoryLayer:
        return self._trajectory_layer

    def vector_map_layer(self) -> VectorMapLayer:
        return self._vector_map_layer

    def correspondence_layer(self, create_new_layer: bool = True) \
            -> CorrespondenceLayer:
        if self._correspondence_layer is None and create_new_layer:
            self._correspondence_layer = CorrespondenceLayer(self.editor)
            self._correspondence_layer.file_path = \
                os.path.join(os.path.dirname(
                    self._trajectory_layer.keyframes_dir()),
                    '3d_2d_correspondences_new.json')
            self.editor.side_bar_widget.correspondence_path_line_edit.setText(
                self._correspondence_layer.file_path)
        return self._correspondence_layer

    def on_load_images(self):
        keyframes_dir = QtWidgets.QFileDialog.getExistingDirectory(
            self.editor, 'Choose images folder', '',
            options=(QtWidgets.QFileDialog.ShowDirsOnly |
                     QtWidgets.QFileDialog.DontUseNativeDialog)
        )

        if not os.path.isdir(keyframes_dir):
            return

        self.load_trajectory_layer(keyframes_dir)

    def on_load_vector_map(self):
        vector_map_file_paths, _ = QtWidgets.QFileDialog.getOpenFileNames(
            self.editor,
            'Select vector map file',
            '',
            'Vector map files (*.json)',
            options=QtWidgets.QFileDialog.DontUseNativeDialog
        )

        # TODO: For multi-file vector maps, load multiple files to one.
        for vector_map_file_path in vector_map_file_paths:
            self.load_vector_map_layer(vector_map_file_path)

    def on_load_correspondences(self):
        if self.trajectory_layer() is not None:
            default_path = os.path.dirname(self.trajectory_layer().file_path)
        else:
            default_path = ''

        correspondences_file_paths, _ = QtWidgets.QFileDialog.getOpenFileNames(
            self.editor,
            'Select correspondence file',
            default_path,
            'Correspondence files (*.json)',
            options=QtWidgets.QFileDialog.DontUseNativeDialog
        )

        # TODO: For multi-file correspondences, load multiple files to one.
        for correspondences_file_path in correspondences_file_paths:
            self.load_correspondence_layer(correspondences_file_path)

    def on_save_correspondences(self):
        if self._correspondence_layer.file_path == '':
            default_path = '3d_2d_correspondences_new.json'
            if self._trajectory_layer is not None:
                keyframes_dir = self._trajectory_layer.keyframes_dir()
                if os.path.exists(keyframes_dir):
                    default_path = os.path.join(
                        os.path.dirname(keyframes_dir), default_path)
            print(default_path)
            path_to_save, _ = QtWidgets.QFileDialog.getSaveFileName(
                self.editor,
                'Save File',
                default_path,
                'Correspondence File (*.json)',
                options=QtWidgets.QFileDialog.DontUseNativeDialog)

            self._correspondence_layer.file_path = path_to_save

        if self._correspondence_layer.file_path == '':
            return
        else:
            self._correspondence_layer.save(
                self._correspondence_layer.file_path)
            self.editor.side_bar_widget.correspondence_path_line_edit.setText(
                self.hide_user_dir(self._correspondence_layer.file_path))

    def load_trajectory_layer(self, keyframes_dir: str):
        # TODO: Change to flexible input paths.
        self.editor.side_bar_widget.images_dir_line_edit.setText(
            self.hide_user_dir(keyframes_dir))
        self.editor.side_bar_widget.image_loaded_checkbox.setChecked(True)
        camera_trajectory_path = ''
        lidar_trajectory_path = ''
        if self._confirm_loading_lidar_trajectories():
            camera_trajectory_path = os.path.join(
                os.path.dirname(keyframes_dir),
                'cam_trajectory.posegraph.csv')
            if not os.path.exists(camera_trajectory_path):
                camera_trajectory_path = ''
            lidar_trajectory_path = os.path.join(
                os.path.dirname(keyframes_dir),
                'lidar_trajectory.posegraph.csv')
            if not os.path.exists(lidar_trajectory_path):
                lidar_trajectory_path = ''
            self.editor.side_bar_widget \
                .lidar_trajectory_path_line_edit.setText(
                self.hide_user_dir(lidar_trajectory_path))
            self.editor.side_bar_widget \
                .lidar_trajectory_loaded_checkbox.setChecked(True)

        trajectory_layer = TrajectoryLayer(self.editor)
        trajectory_layer \
            .set_keyframes_dir(keyframes_dir) \
            .set_cam_trajectory_path(camera_trajectory_path) \
            .set_lidar_trajectory_path(lidar_trajectory_path) \
            .load()

        self._trajectory_layer = trajectory_layer
        self.invoke_event(CustomEvent.TrajectoryLoadedEvent)

    def load_vector_map_layer(self, vector_map_file_path: str):
        if self._trajectory_layer is None:
            return
        if os.path.exists(vector_map_file_path):
            vector_map_layer = VectorMapLayer(self.editor)
            vector_map_layer.load(vector_map_file_path)
            vector_map_layer.file_path = vector_map_file_path
            self._vector_map_layer = vector_map_layer
            self.editor.side_bar_widget.vector_map_path_line_edit.setText(
                self.hide_user_dir(vector_map_file_path)
            )
            self.editor.side_bar_widget \
                .vector_map_loaded_checkbox.setChecked(True)
            self.invoke_event(CustomEvent.VectorMapLoadedEvent)

    def load_correspondence_layer(self, correspondence_file_path: str):
        if self._trajectory_layer is None:
            return
        if os.path.exists(correspondence_file_path):
            if self._correspondence_layer is None:
                self._correspondence_layer = CorrespondenceLayer(self.editor)
            self._correspondence_layer = CorrespondenceLayer(self.editor)
            self._correspondence_layer.load(correspondence_file_path)
            self._correspondence_layer.file_path = correspondence_file_path
            self.editor.side_bar_widget.correspondence_path_line_edit.setText(
                self.hide_user_dir(correspondence_file_path))
            self.invoke_event(CustomEvent.CorrespondencesLoadedEvent)

    def _confirm_loading_lidar_trajectories(self):
        clicked_buton = QMessageBox.question(
            self.editor,
            '',
            'Load related lidar trajectories ?',
            QMessageBox.Yes, QMessageBox.No
        )
        if clicked_buton == QMessageBox.Yes:
            return True
        else:
            return False

    @staticmethod
    def hide_user_dir(file_path: str):
        user_dir = os.path.expanduser('~')
        file_path.replace(user_dir, '~')
        return file_path
