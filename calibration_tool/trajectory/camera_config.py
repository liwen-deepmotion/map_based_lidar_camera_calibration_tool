# Author: Chi Zhang (chizhang@deepmotion.ai)


import json

import numpy as np


class CameraConfig(object):

    def __init__(self):
        self.fx = float()
        self.fy = float()
        self.cx = float()
        self.cy = float()
        self.w = int()
        self.h = int()

    def from_json_file(self, json_file_path: str):
        with open(json_file_path, 'r') as f:
            sensor_calibrations = json.load(f)
            for sensor_config in sensor_calibrations['dm_device']['sensors']:
                if sensor_config['type'] == 'camera':
                    self.w = sensor_config['width']
                    self.h = sensor_config['height']
                    self.fx, self.fy, self.cx, self.cy = \
                        sensor_config['intrinsics']
                    break

    def intrinsic_matrix(self):
        return np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ])

    def get_projection_matrix(self) -> np.ndarray(shape=(4, 4)):
        w, h = self.w, self.h
        fu, fv = self.fx, self.fy
        u0, v0 = self.cx, self.cy
        zNear, zFar = 0.54, 3000
        # http://www.songho.ca/opengl/gl_projectionmatrix.html
        # The following lines are copied from panolin's implementation
        # opengl_render_state.cpp, line 404: ProjectionMatrixRUB_BottomLeft
        L = +(u0) * zNear / -fu
        T = +(v0) * zNear / fv
        R = -(w - u0) * zNear / -fu
        B = -(h - v0) * zNear / fv
        P = np.zeros((4, 4), dtype=np.float32)
        P[0, 0] = 2 * zNear / (R - L)
        P[1, 1] = 2 * zNear / (T - B)
        P[2, 2] = -(zFar + zNear) / (zFar - zNear)
        P[2, 0] = (R + L) / (R - L)
        P[2, 1] = (T + B) / (T - B)
        P[2, 3] = -1.0
        P[3, 2] = -(2 * zFar * zNear) / (zFar - zNear)

        return P
