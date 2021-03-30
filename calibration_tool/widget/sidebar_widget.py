#!/usr/bin/env python3
# Author: Wei Tan (weitan@deepmotion.ai)
from PyQt5.QtCore import QSize, Qt
from PyQt5.QtGui import QIcon, QFont
from PyQt5.QtWidgets import QWidget, QAction, QSizePolicy

from ui.sidebar import Ui_Sidebar


class SideBarWidget(QWidget, Ui_Sidebar):

    def __init__(self, parent=None):
        super().__init__(parent)

        self.setupUi(self)

        # Loads.
        self.images_dir_line_edit.hide()
        self.image_loaded_checkbox.hide()
        self.load_images_groupbox.setTitle("")
        self.load_images.setText("Load Images")
        self.load_images.setSizePolicy(
            QSizePolicy.Expanding, QSizePolicy.Expanding)
        # self.load_images.setIcon(QIcon(":/icons/images.png"))
        # self.load_images.setIconSize(QSize(80, 80))

        self.lidar_trajectory_path_line_edit.hide()
        self.lidar_trajectory_loaded_checkbox.hide()
        self.load_trajectory_groupbox.setTitle("")
        self.load_lidar_trajectory.setText("Load lidar trajectory")
        self.load_lidar_trajectory.setSizePolicy(
            QSizePolicy.Expanding, QSizePolicy.Expanding)
        # self.load_lidar_trajectory.setIcon(QIcon(":/icons/lidar.png"))
        # self.load_lidar_trajectory.setIconSize(QSize(80, 80))

        self.vector_map_path_line_edit.hide()
        self.show_vector_map_checkbox.hide()
        self.reprojection_vector_map_btn.hide()
        self.vector_map_loaded_checkbox.hide()
        self.load_vector_map_groupbox.setTitle("")
        self.load_vector_map.setText("Load vector map")
        self.load_vector_map.setSizePolicy(
            QSizePolicy.Expanding, QSizePolicy.Expanding)
        # self.load_vector_map.setIcon(QIcon(":/icons/vector_map.png"))
        # self.load_vector_map.setIconSize(QSize(80, 80))

        self.prev_image_btn.setText("")
        self.prev_image_btn.setIcon(QIcon(":/icons/arrow_left.png"))
        self.prev_image_btn.setIconSize(QSize(50, 50))
        self.next_image_btn.setText("")
        self.next_image_btn.setIcon(QIcon(":/icons/arrow_right.png"))
        self.next_image_btn.setIconSize(QSize(50, 50))

        self.load_correspondences_btn.hide()
        self.save_correspondences_btn.hide()
        self.correspondence_path_line_edit.hide()
        self.show_correspondences_checkbox.hide()
        self.darken_image_background_checkbox.hide()
        self.add_correspondence_btn.setText("Add correspondence")
        # self.add_correspondence_btn.setText("")
        # self.add_correspondence_btn.setIcon(QIcon(":/icons/correspondence.png"))
        # self.add_correspondence_btn.setIconSize(QSize(70, 70))
        self.add_correspondence_btn.setSizePolicy(
            QSizePolicy.Minimum, QSizePolicy.Minimum)
        self.clear_correspondence_btn.setText("Clear correspondence")
        # self.clear_correspondence_btn.setText("")
        # self.clear_correspondence_btn.setIcon(QIcon(":/icons/clear.png"))
        # self.clear_correspondence_btn.setIconSize(QSize(70, 70))
        self.clear_correspondence_btn.setSizePolicy(
            QSizePolicy.Minimum, QSizePolicy.Minimum)

        self.toggle_optimization_checkbox.setText("")
        self.execute_optimize_btn.setText("Optimization")
        # self.execute_optimize_btn.setText("")
        # self.execute_optimize_btn.setIcon(QIcon(":/icons/execute.png"))
        # self.execute_optimize_btn.setIconSize(QSize(70, 70))
        self.execute_optimize_btn.setSizePolicy(
            QSizePolicy.Expanding, QSizePolicy.Minimum)

        for button in [
            self.load_images,
            self.load_lidar_trajectory,
            self.load_vector_map,
            self.add_correspondence_btn,
            self.clear_correspondence_btn,
            self.prev_image_btn,
            self.next_image_btn,
            self.execute_optimize_btn]:
            button.setStyleSheet(
                "QToolButton:hover {"
                "background-color: rgba(54, 191, 153, 55);"
                "}"
                "QToolButton:pressed {"
                "   padding: 5px;"
                "}"
            )


if __name__ == '__main__':
    import sys
    from PyQt5.QtWidgets import QApplication

    app = QApplication(sys.argv)
    widget = SideBarWidget()
    widget.show()
    sys.exit(app.exec_())
