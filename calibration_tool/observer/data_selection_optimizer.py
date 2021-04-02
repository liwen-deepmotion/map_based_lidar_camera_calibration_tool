import os
from typing import List, Tuple, Union

import cvxpy as cp
import numpy as np
import pyquaternion
# import matplotlib.pyplot as plt
from actor.shape_actor import ShapeActor
from config.hot_key import KeyCombo, HotKey
from observer.base_observer import BaseObserver
from observer.event.events import CustomEvent
from observer.vector_map_reprojector import VectorReprojector
from shape.polyline_2d import Polyline2D
from shape.shape import Shape
from trajectory.trajectory_node import TrajectoryNode
from vector.polyline_3d import Polyline3D
from vector.vector import Vector


class DataSelectionOptimizer(BaseObserver):

    def __init__(self, editor: 'MapBasedCalibrator'):
        super().__init__(editor)

        self.QT_EVENT_CALLBACK_PRIORITY_TUPLES = [
            (CustomEvent.KeyComboPressedEvent,
             self.on_key_press, 0),
        ]

        self.QT_SIGNAL_CALLBACK_TUPLES = [
            (self.editor.side_bar_widget.optimize_data_selection_btn.clicked,
             self.optimize_data_selection),
            (self.editor.side_bar_widget.show_data_selections_checkbox.clicked,
             self.toggle_data_selection_highlights),
            (self.editor.side_bar_widget.prev_image_btn.clicked,
             self.clear_selections),
            (self.editor.side_bar_widget.next_image_btn.clicked,
             self.clear_selections),
        ]

        # Record the recommended reprojected vectors and highlight them.
        self.optimal_selections = []  # type: List[Shape]

        self._shape_actors = []  # type: List[ShapeActor]

    def on_key_press(self, key_combo: KeyCombo):
        if key_combo.is_same(HotKey.PREV_FRAME.value):
            self.clear_selections()
        elif key_combo.is_same(HotKey.NEXT_FRAME.value):
            self.clear_selections()

    def clear_selections(self):
        self._clear_shape_actors()
        self.optimal_selections = []
        self.editor.side_bar_widget.show_data_selections_checkbox \
            .setChecked(False)

    def optimize_data_selection(self):
        camera_pose = \
            self.editor.trajectory_navigator.current_trajectory_node()
        vectors = self.editor.layer_manager.vector_map_layer().polylines()
        if camera_pose is None or len(vectors) == 0:
            return [], []

        # Copy polylines to line segments.
        vectors = [Polyline3D(vector.vertices()) for vector in vectors]

        vectors, shapes = self._collect_possible_correspondence_pairs(
            vectors, camera_pose)
        print("{} shapes generated".format(len(shapes)))

        self.optimal_selections = self._select_correspondences(
            vectors, shapes, camera_pose)
        self.editor.side_bar_widget.show_data_selections_checkbox.setChecked(
            True)
        self.toggle_data_selection_highlights()

    def toggle_data_selection_highlights(self):
        self._clear_shape_actors()
        if self.editor.side_bar_widget.show_data_selections_checkbox.isChecked():
            for shape in self.optimal_selections:
                shape_actor = shape.build_actor()
                shape_actor.property().set_color(255, 255, 127)
                shape_actor.property().set_line_width(15)
                self.renderer.add_actor(shape_actor)
                self._shape_actors.append(shape_actor)
        self.update()

    def _clear_shape_actors(self):
        for shape_actor in self._shape_actors:
            self.renderer.remove_actor(shape_actor)
        self._shape_actors.clear()

    def _collect_possible_correspondence_pairs(
            self, vectors: List[Vector], camera_pose: TrajectoryNode) \
            -> Tuple[List[Vector], List[Shape]]:
        reprojected_vectors, reprojected_shapes = \
            self._reproject_vectors(vectors, camera_pose)
        filtered_vectors, filtered_shapes = self._filter_correspondence_pairs(
            reprojected_vectors, reprojected_shapes)
        return filtered_vectors, filtered_shapes

    def _select_correspondences(
            self,
            vectors: List[Vector],
            shapes: List[Shape],
            camera: TrajectoryNode) -> List[Shape]:
        for vector in vectors:
            vector.resample()
        # Simplify ground truth shape to line segments.
        new_shapes = [Polyline2D(np.vstack([
            shape.head_vertex(), shape.tail_vertex()])) for shape in shapes]
        s_ks, c_ks = self._prepare_params(vectors, new_shapes, camera)
        result_indices = self._apply_optimization(s_ks, c_ks)
        print(result_indices)
        return [shapes[int(index)] for index in result_indices]

    def _reproject_vectors(
            self, vectors: List[Vector], camera: TrajectoryNode) \
            -> Tuple[List[Vector], List[Shape]]:
        # Reproject vectors to fixed camera.
        vector_reprojector = VectorReprojector()
        vector_reprojector.set_intrinsics(camera.camera_config())
        vector_reprojector.set_extrinsic(camera.T_camera_to_world())
        reprojected_shapes = []
        reprojected_vectors = []
        for vector in vectors:
            reprojected_shape = \
                vector_reprojector.reproject(vector, boundary=500)
            if reprojected_shape is not None:
                reprojected_shapes.append(reprojected_shape)
                reprojected_vectors.append(vector)
        return reprojected_vectors, reprojected_shapes

    def _filter_correspondence_pairs(
            self, vectors: List[Vector], shapes: List[Shape]) \
            -> Tuple[List[Vector], List[Shape]]:
        remained_vectors = []
        remained_shapes = []
        for idx, shape in enumerate(shapes):
            min_pixel_length_allowed = 100
            if shape.length() < min_pixel_length_allowed:
                continue
            remained_vectors.append(vectors[idx])
            remained_shapes.append(shape)
        return remained_vectors, remained_shapes

    def _prepare_params(
            self,
            vectors: List[Vector],
            shapes: List[Shape],
            camera: TrajectoryNode) \
            -> Tuple[type(np.ndarray(shape=(6, 0))),
                     type(np.ndarray(shape=(6, 0)))]:
        s_ks_list = []
        c_ks_list = []
        coeffs = [1.2, 1.0, 1.0, 0.8, 0.4, 0.6]
        param_preparer = ParamPreparer()
        param_preparer.camera = camera
        param_preparer.vectors = vectors
        param_preparer.shapes = shapes
        param_preparer.current_pose_id = \
            self.editor.trajectory_navigator.current_trajectory_node_idx()
        idx_to_dof = ["x", "y", "z", "yaw", "pitch", "roll"]
        for dof in range(6):
            print("Calculating along {}".format(idx_to_dof[dof]))
            param_preparer.dof = dof
            c_ks_in_dof = param_preparer.compute_convexity()
            c_ks_list.append(c_ks_in_dof)
            print("Convexity finished.")
            s_ks_in_dof = param_preparer.compute_stability()
            s_ks_list.append(s_ks_in_dof * coeffs[dof])
            print("Instability finished.")
        s_ks_list = np.vstack(s_ks_list)
        c_ks_list = np.vstack(c_ks_list)
        return s_ks_list, c_ks_list

    def _apply_optimization(
            self,
            s_ks_list: np.ndarray(shape=(6, 0)),
            c_ks_list: np.ndarray(shape=(6, 0))) -> List[int]:
        print("Preparing parameters...")
        target_num_correspondences = 4
        C = np.sum(s_ks_list, axis=0)
        num_xs = len(C)
        if num_xs < 2:
            return []
        sum_mat = np.ones(num_xs)
        # Use negative to make upper boundary <= to lower boundary >=.
        A = -c_ks_list
        print(np.sum(A, axis=1))
        convexity_boundaries = np.array([
            -1e4,
            -1e4,
            -1e4,
            -1e4,
            -1e4,
            -1e4,
        ]) * 0.0
        b = convexity_boundaries
        print("C: ", C.shape, C)
        print("A: ", A.shape, A)
        print("b: ", b.shape, b)
        print("x: ", num_xs)
        print("Start optimizing...")
        x = cp.Variable(num_xs, boolean=True)
        problem = cp.Problem(cp.Minimize(C.T @ x), [
            A @ x <= b,
            sum_mat.T @ x == target_num_correspondences
        ])
        problem.solve()
        print("Finished.")
        print("result", x.value)
        print("optimal value", problem.value)
        print("boundary value", A.dot(np.array(x.value)))
        print("count", sum_mat.dot(x.value))

        # Optimization.
        selected_indices = []
        for idx, x in enumerate(x.value):
            if x > 0:
                selected_indices.append(idx)
        return selected_indices


class ParamPreparer(object):

    def __init__(self):
        self.camera = None  # type: TrajectoryNode

        self.vectors = []  # type: List[Vector]

        self.shapes = []  # type: List[Shape]

        self.dof = int()

        self.max_offset = 10

        self.current_pose_id = int()

    def compute_convexity(self):
        if not os.path.exists(self._get_convexity_path()):
            curves = self._compute_cost_curves(self.shapes)
            c_ks = self._compute_c_ks(curves)
            # self._show_curves(self._gen_disturbs(), curves, c_ks)
            np.save(self._get_convexity_path(), c_ks)
        else:
            c_ks = np.load(self._get_convexity_path())
        return c_ks

    def compute_stability(self):
        if not os.path.exists(self._get_stability_path()):
            minima_shift_curves = self.compute_minima_shift_curves()
            # self._show_curves(
            #     np.arange(0, self.max_offset), minima_shift_curves)
            s_ks = self._compute_s_ks(minima_shift_curves)
            np.save(self._get_stability_path(), s_ks)
        else:
            s_ks = np.load(self._get_stability_path())
        return s_ks

    def _compute_cost_curves(self, shapes: List[Shape]):
        # Apply perturbations to cost (how to compute cost)
        # 1. cost = compute_cost(vectors, shapes)
        # 2. for each pertabuance, get reprojected shapes, and compute cost
        disturbs = self._gen_disturbs()
        vector_reprojector = VectorReprojector()
        vector_reprojector.set_intrinsics(self.camera.camera_config())
        costs = np.zeros((len(self.vectors), len(disturbs)))
        for idx, disturb in enumerate(disturbs):
            disturbed_pose = self._disturb_pose(
                self.camera.T_camera_to_world().copy(), disturb)
            vector_reprojector.set_extrinsic(disturbed_pose)
            reprojected_shapes = []
            for repr_idx, vector in enumerate(self.vectors):
                reprojected_shape = vector_reprojector.reproject(
                    vector, boundary=500)
                if reprojected_shape is not None:
                    reprojected_shapes.append(reprojected_shape)
                else:
                    reprojected_shapes.append(shapes[repr_idx])
            costs_at_disturb = \
                self._compute_cost(shapes, reprojected_shapes)
            costs[:, idx] = costs_at_disturb
        return costs

    def compute_minima_shift_curves(self):
        # Apply random offsets (referring to the scripts) (1 to 4 offset
        # is enough) for every offset level, every group, compute
        # cost_curves, get minima shift. get minima shift curve.
        end_level = self.max_offset
        all_minima_shifts = np.zeros((len(self.vectors), end_level + 1))
        for offset_level in range(1, end_level + 1):
            minima_shifts_list = []
            x_s = self._gen_disturbs()
            for rand_perm in range(15):
                offset_shapes = self._offset_shapes(self.shapes, offset_level)
                costs = self._compute_cost_curves(offset_shapes)
                min_cost_positions = np.array([
                    x_s[idx] for idx in np.argmin(costs, axis=1)])
                minima_shifts = np.abs(min_cost_positions)
                minima_shifts_list.append(minima_shifts.flatten())
            mean_minima_shifts_at_level = np.mean(minima_shifts_list, axis=0)
            all_minima_shifts[:, offset_level] = mean_minima_shifts_at_level
        return all_minima_shifts

    def _compute_c_ks(self, curves):
        # Compute 2 order derivative at x = 0 use Taylor extension.
        if self.dof < 3:
            step = 1 / 10
        else:
            step = 2 / 10
        origin_position = curves.shape[1] // 2
        gradients_2_order = \
            np.gradient(np.gradient(curves, axis=1) / step, axis=1) / step
        return gradients_2_order[:, origin_position]

    def _compute_s_ks(self, curves):
        # Fit a line and compute slope.
        x_sticks = np.array(range(0, curves.shape[1]))
        slopes = np.sum(curves * x_sticks, axis=1) / np.sum(x_sticks**2)
        return slopes

    def _gen_disturbs(self):
        if self.dof < 3:
            return np.linspace(-1, 1, 21)
        else:
            return np.linspace(-2, 2, 21)

    def _disturb_pose(self, pose: np.ndarray(shape=(4, 4)), disturb: float):
        disturb_vector = np.zeros(6)
        disturb_vector[self.dof] = disturb
        x, y, z, yaw, pitch, roll = disturb_vector
        translation_disturb = np.array([x, y, z])
        pose[:3, 3] += translation_disturb

        # Additional yaw/pitch/roll should be applied according to the
        # camera axes.
        axes = [
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0]
        ]
        quat = pyquaternion.Quaternion(matrix=pose[:3, :3])
        # Note: could not use `quat *= rot`, as quaternion and rotation
        # matrix do not follow the multiplication commutative law, here
        # should be `quat = rot * quat`.
        for idx, angle in enumerate([yaw, pitch, roll]):
            quat = pyquaternion.Quaternion(
                axis=axes[idx], angle=angle * np.pi / 180) * quat
        pose[:3, :3] = quat.rotation_matrix
        return pose

    def _offset_shapes(self, shapes: List[Shape], offset: int) -> List[Shape]:
        new_shapes = []
        for shape in shapes:
            new_shape = Shape()
            new_shape.set_coords(shape.coords())
            random_thetas = np.random.random(new_shape.size()) * np.pi * 2
            random_vecs = \
                np.vstack([np.cos(random_thetas), np.sin(random_thetas)]).T
            random_offsets = random_vecs * offset
            new_shape.set_coords(new_shape.coords() + random_offsets)
            new_shapes.append(new_shape)
        return new_shapes

    def _compute_cost(self, src_shapes: List[Shape], dst_shapes: List[Shape]) \
            -> np.ndarray(shape=(0,)):
        # src shapes are original shapes (line segment) and dst shapes
        # are reprojected disturbed shapes (sampled by 1m).
        costs = []
        for src_shape, dst_shape in zip(src_shapes, dst_shapes):
            distances = \
                self._project_polyline(src_shape.coords(), dst_shape.coords())
            if distances is None:
                continue
            costs.append(np.sum(distances * distances))
        return np.array(costs)

    def _project_polyline(
            self,
            vertices: np.ndarray(shape=(0, 2)),
            points: np.ndarray(shape=(0, 2))) \
            -> np.ndarray((0,)):
        line_segment_tail_vertices = vertices[1:]
        line_segment_head_vertices = vertices[:-1]

        # Compute point to segments distances in vectorized fashion (for
        # speed).
        projection_coeff = np.sum((points[:, None] -
                                   line_segment_head_vertices) *
                                  (line_segment_tail_vertices -
                                   line_segment_head_vertices),
                                  axis=2) / \
                           np.sum((line_segment_tail_vertices -
                                   line_segment_head_vertices) *
                                  (line_segment_tail_vertices -
                                   line_segment_head_vertices),
                                  axis=1).reshape(1, -1)
        points_projected_points = (
                line_segment_head_vertices +
                projection_coeff.reshape(
                    (len(points), len(line_segment_head_vertices), 1)) *
                (line_segment_tail_vertices - line_segment_head_vertices)
        )

        points_to_segment_dists = np.linalg.norm(
            points[:, None] - points_projected_points,
            axis=2)
        return points_to_segment_dists

    def _get_convexity_path(self):
        if not os.path.exists("./cache/"):
            os.mkdir("./cache")
        return "./cache/convexity_" + \
               str(self.current_pose_id) + "_" + str(self.dof) + ".npy"

    def _get_stability_path(self):
        if not os.path.exists("./cache/"):
            os.mkdir("./cache")
        return "./cache/stability_" + \
               str(self.current_pose_id) + "_" + str(self.dof) + ".npy"

    # def _show_curves(self, x_s, curves, convexity=None):
    #     file_path = "test_" + str(self.dof) + ".png"
    #     fig, ax = plt.subplots()
    #     for idx, curve in enumerate(curves):
    #         if convexity is not None:
    #             ax.plot(x_s, curve, label="{:.1f}".format(convexity[idx]))
    #         else:
    #             ax.plot(x_s, curve)
    #     if convexity is not None:
    #         ax.legend()
    #     plt.savefig(file_path)
