#!/usr/bin/env python3


import argparse
import sys

from PyQt5.QtWidgets import QApplication

from map_based_calibrator import MapBasedCalibrator


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="HDMapEditor")
    parser.add_argument("--keyframes_dir", default="")
    parser.add_argument("--vector_map_file_path", default="")
    parser.add_argument("--correspondences_file_path", default="")
    args, argv = parser.parse_known_args(sys.argv)

    app = QApplication([])

    map_based_calibrator = MapBasedCalibrator()
    map_based_calibrator.setup_ui()
    map_based_calibrator.setup_event_callback_connections()

    map_based_calibrator.showMaximized()

    if args.keyframes_dir != '':
        map_based_calibrator.layer_manager.load_trajectory_layer(
            args.keyframes_dir)

    if args.vector_map_file_path != '':
        map_based_calibrator.layer_manager.load_vector_map_layer(
            args.vector_map_file_path
        )

    if args.correspondences_file_path != '':
        map_based_calibrator.layer_manager.load_correspondence_layer(
            args.correspondences_file_path
        )

    sys.exit(app.exec_())
