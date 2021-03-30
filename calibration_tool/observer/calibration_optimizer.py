import json
import os
import subprocess
from typing import Callable

import numpy as np
from PyQt5.QtCore import QRect, QTimer, QRectF, QThread
from PyQt5.QtGui import QColor

from actor.image_actor import ImageActor
from actor.text_actor import TextActor
from config.hot_key import KeyCombo, HotKey
from observer.base_observer import BaseObserver
from observer.event.events import CustomEvent


class MyThread(QThread):

    def __init__(self):
        super().__init__()

        self._execute_str = ''

        self._execute_callback = None  # type: Callable

    def set_execute_str(self, exe_str: str):
        self._execute_str = exe_str

    def set_execute_callback(self, callback: Callable):
        self._execute_callback = callback

    def run(self):
        if callable(self._execute_callback):
            self._execute_callback(self._execute_str)


class CalibrationOptimizer(BaseObserver):

    def __init__(self, editor: 'MapBasedCalibrator'):
        super().__init__(editor)

        self.QT_SIGNAL_CALLBACK_TUPLES = [
            (self.editor.side_bar_widget.execute_optimize_btn.clicked,
             self.start_optimization),
            (self.editor.side_bar_widget.toggle_optimization_checkbox.clicked,
             self.on_toggle_optimization_result)
        ]
        self.QT_EVENT_CALLBACK_PRIORITY_TUPLES = [
            (CustomEvent.KeyComboPressedEvent, self.on_key_press, 0),
            (CustomEvent.TrajectoryNodeChangedEvent,
             self.on_trajectory_node_changed, 0),
        ]

        self._current_T_lidar_to_camera_opt = np.eye(4)

        self._is_optimization_executed = False

        self._is_optimized = False

        self._exe_str = ''

        self._text_actor = TextActor()

        self._blocking_text = TextActor()

        self._blocking_rect = ImageActor()

        self._thread = MyThread()

    def on_trajectory_node_changed(self):
        if not self._is_optimization_executed:
            return
        self._show_optimization_state_text()

    def on_toggle_optimization_result(self):
        success = self.toggle_optimization_result()
        if success:
            self.invoke_event(CustomEvent.CalibrationOptimizedEvent)

    def on_key_press(self, key_combo: KeyCombo):
        if key_combo.is_same(HotKey.TOGGLE_OPTIMIZATION_RESULT.value):
            success = self.toggle_optimization_result()
            if success:
                self.invoke_event(CustomEvent.CalibrationOptimizedEvent)

    def start_optimization(self):
        prepare_success = self._prepare_optimization()
        if not prepare_success:
            return
        if not prepare_success:
            return
        else:
            self._add_blocking_mask()
            self.update()
            self.renderer.canvas().setMouseTracking(False)
            self._thread.set_execute_str(self._exe_str)
            self._thread.set_execute_callback(self._execute)
            self._thread.start()
            self._thread.finished.connect(self.finish_optimization)
        print('start_optimization')

    def finish_optimization(self):
        print('finish_optimization')
        self._remove_blocking_mask()
        self.renderer.canvas().setMouseTracking(True)
        self._load_optimized_calibration()
        if not self._is_optimized:
            success = self.toggle_optimization_result()
            if success:
                self.editor.side_bar_widget \
                    .toggle_optimization_checkbox.setChecked(True)
                self.invoke_event(CustomEvent.CalibrationOptimizedEvent)
                self._is_optimization_executed = True

    def _prepare_optimization(self) -> bool:
        if self.editor.layer_manager.trajectory_layer() is None:
            return False
        sequence_dir = \
            os.path.dirname(self.editor.layer_manager.
                            trajectory_layer().keyframes_dir())
        project_dir = \
            os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        exe_path = os.path.join(
            project_dir, 'external', 'calibrate_lidar_camera')
        print(exe_path)

        correspondence_layer = self.editor.layer_manager.correspondence_layer()
        if len(correspondence_layer.correspondences()) == 0:
            print('Optimization terminated, no correspondence found in '
                  'the scene.')
            return False
        if correspondence_layer.file_path == '':
            self.editor.layer_manager.on_save_correspondences()
        else:
            correspondence_layer.save(correspondence_layer.file_path)

        if correspondence_layer.file_path == '':
            print('Optimization terminated, correspondences not found.')
            return False

        print(self.editor.layer_manager.vector_map_layer().file_path)
        exe_command = [
            exe_path,
            '--seq_dir ' + sequence_dir,
            '--correspondence_json_path ' + correspondence_layer.file_path,
        ]
        print(exe_command)
        exe_str = ''
        for sub_str in exe_command:
            exe_str += ' ' + sub_str
        self._exe_str = exe_str

        return True

    def _load_optimized_calibration(self):
        trajectory_layer = self.editor.layer_manager.trajectory_layer()
        if trajectory_layer is None:
            return
        sequence_dir = \
            os.path.dirname(trajectory_layer.keyframes_dir())
        optimized_transform_file_path = os.path.join(
            sequence_dir, 'optimized_lidar_to_camera_transform.json')
        T_lidar_to_cam_optimzed = self._load_optimized_transform(
            optimized_transform_file_path)

        self._current_T_lidar_to_camera_opt = T_lidar_to_cam_optimzed

    def toggle_optimization_result(self) -> bool:
        trajectory_layer = self.editor.layer_manager.trajectory_layer()
        if trajectory_layer is None:
            return False
        T_lidar_to_cam_optimized = self._current_T_lidar_to_camera_opt
        T_lidar_to_cam_src = trajectory_layer.T_lidar_to_camera()
        if not self._is_optimized:
            T_transform = T_lidar_to_cam_src.dot(np.linalg.inv(
                T_lidar_to_cam_optimized))
        else:
            T_transform = T_lidar_to_cam_optimized.dot(np.linalg.inv(
                T_lidar_to_cam_src))

        for trajectory_node in trajectory_layer.trajectory_nodes():
            trajectory_node.set_camera_pose(
                trajectory_node.T_camera_to_world()
                    .dot(T_transform))
        self._is_optimized = not self._is_optimized
        self._show_optimization_state_text()
        return True

    def _load_optimized_transform(self, file_path: str) -> np.ndarray(shape=(4,4)):
        with open(file_path, 'r+') as f:
            transform_dict = json.load(f)
        T_lidar_to_cam = np.array(transform_dict['T_lidar_to_cam']).reshape((4, 4))
        return T_lidar_to_cam

    def _execute(self, exe_str: str):
        proc = subprocess.Popen(exe_str, shell=True, stdout=subprocess.PIPE)
        proc.wait()
        
    def _show_optimization_state_text(self):
        self.renderer.remove_actor(self._text_actor)
        self._text_actor.geometry().set_qt_geometry(
            QRectF(30, 150, 300, 60)
        )
        self._text_actor.property().set_color(54, 191, 153)
        if self._is_optimized:
            self._text_actor.set_text('After Optimization')
        else:
            self._text_actor.set_text('Before Optimization')

        self._text_actor.set_background(
            QRect(30, 150, 300, 60),
            QColor(50, 50, 50, 200),
        )
        self.renderer.add_actor(self._text_actor)

    def _add_blocking_mask(self):
        self.renderer.remove_actor(self._blocking_text)

        canvas_width, canvas_height = \
            self.renderer.canvas().width(), self.renderer.canvas().height()
        self._blocking_text.property().set_color(54, 191, 153)
        self._blocking_text.text_size = 30
        self._blocking_text.geometry().set_qt_geometry(
            QRectF(0, 0, canvas_width, canvas_height)
        )
        self._blocking_text.set_background(
            QRect(0, 0, canvas_width, canvas_height),
            QColor(30, 30, 30, 200),
        )
        self._blocking_text.set_text("Optimizing...")
        self.renderer.add_actor(self._blocking_text)

    def _remove_blocking_mask(self):
        self.renderer.remove_actor(self._blocking_text)
