#!/usr/bin/env python3


import os

from PyQt5.QtCore import Qt, QEvent, QFileSystemWatcher
from PyQt5.QtWidgets import QMainWindow, QApplication

from observer.calibration_optimizer import CalibrationOptimizer
from observer.core.camera_controller import CameraController
from observer.core.closest_vertex_selector import ClosestVertexSelector
from observer.core.keyboard_observer import KeyboardObserver
from observer.core.mouse_observer import MouseObserver
from observer.correspondence_adder import CorrespondenceAdder
from observer.correspondence_manager import CorrespondenceManager
from observer.correspondence_remover import CorrespondenceRemover
from observer.event.event_handler import EventHandler
from observer.free_shape_adder import FreeShapeAdder
from observer.layer_manager import LayerManager
from observer.trajectory_navigator import TrajectoryNavigator
from observer.vector_map_reprojector import VectorMapReprojector
from ui.main_window import Ui_MainWindow
from widget.main_canvas_widget import MainCanvasWidget
from widget.sidebar_widget import SideBarWidget


class MapBasedCalibrator(QMainWindow, Ui_MainWindow):

    def __init__(self):
        super().__init__()

        self.event_handler = EventHandler(self)
        self.layer_manager = None  # type: LayerManager
        self.trajectory_navigator = None  # type: TrajectoryNavigator
        self.camera_controller = None  # type: CameraController
        self.mouse_observer = None  # type: MouseObserver
        self.keyboard_observer = None  # type: KeyboardObserver
        self.free_shape_adder = None  # type: FreeShapeAdder
        self.closest_vertex_selector = None  # type: ClosestVertexSelector

        self.unnamed_observer_instances = []
        self.file_watcher = QFileSystemWatcher(self)

    def setup_ui(self):
        self.setupUi(self)
        self.setAcceptDrops(True)
        self.setMouseTracking(True)

        self.setFocusPolicy(Qt.StrongFocus)

        self.regularization_toolbar.hide()
        self.main_canvas_widget = MainCanvasWidget(self)
        # self.setCentralWidget(self.main_canvas_widget)
        self.container_horizontal_layout.insertWidget(0, self.main_canvas_widget, stretch=1)

        self.side_bar_widget = SideBarWidget(self)
        self.container_horizontal_layout.insertWidget(1, self.side_bar_widget, stretch=0)

        self.set_stylesheet(os.path.join(os.path.dirname(__file__),
                                         'ui/resources/stylesheet.qss'))

    def set_stylesheet(self, style_sheet_path: str):
        root_path = os.path.dirname(__file__)
        with open(style_sheet_path) as f:
            # Use the `QCoreApplication.setStyleSheet` instead of
            # `self.setStyleSheet` because using the second form will
            # not update the window immediately.
            QApplication.instance().setStyleSheet(
                f.read().replace('{root_path}', root_path))

    def setup_event_callback_connections(self):
        self.layer_manager = LayerManager(self)
        self.trajectory_navigator = TrajectoryNavigator(self)
        self.camera_controller = CameraController(self)
        self.mouse_observer = MouseObserver(self)
        self.keyboard_observer = KeyboardObserver(self)
        self.free_shape_adder = FreeShapeAdder(self)
        self.closest_vertex_selector = ClosestVertexSelector(self)

        for observer in [
            self.layer_manager,
            self.trajectory_navigator,
            self.camera_controller,
            self.mouse_observer,
            self.keyboard_observer,
            self.free_shape_adder,
            self.closest_vertex_selector,
        ]:
            observer.connect_myself_to_qt_signals()
            observer.connect_myself_to_vtk_events()

        for observer_class in [
            VectorMapReprojector,
            CorrespondenceAdder,
            CorrespondenceManager,
            CorrespondenceRemover,
            CalibrationOptimizer,
        ]:
            observer = observer_class(self)
            observer.connect_myself_to_qt_signals()
            observer.connect_myself_to_vtk_events()

            self.unnamed_observer_instances.append(observer)

    def event(self, event: QEvent) -> bool:
        """
        This class is used to wrap main window qt events to custom
        events which can be responded by observers.
        """
        if event.type() in [
            QEvent.KeyPress,
            QEvent.KeyRelease,
            QEvent.MouseButtonPress,
            QEvent.MouseButtonRelease,
            QEvent.MouseMove,
            QEvent.Wheel,
            QEvent.DragEnter,
            QEvent.Drop,
            QEvent.Resize,
        ]:
            self.event_handler.invoke_event(event.type(), event)
            if QEvent.Resize != event.type():

                return True

        return super().event(event)
