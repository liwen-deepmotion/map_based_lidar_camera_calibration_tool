

import json
import os
from typing import Dict, Union

import numpy as np

from vector.polygon_3d import Polygon3D
from vector.polyline_3d import Polyline3D
from vector.vector import Vector


class JsonHdMapVectorFactory(object):

    def __init__(self):
        pass

    @staticmethod
    def element_type_to_vector_class_map() -> Dict:
        return {
            'LANE_LINE_SPLINE': Polyline3D,
            'ROAD_MARKER_KEYPOINT': Polygon3D,
            'POLE_SEGMENT': Polyline3D,
            'TRAFFIC_SIGN_KEYPOINT': Polygon3D,
            'TRAFFIC_LIGHT_KEYPOINT': Polygon3D,
        }

    @classmethod
    def from_element_dict(cls, element_dict: Dict) -> Union[Vector, None]:
        element_type = element_dict['element_type']
        vector_class = cls.element_type_to_vector_class_map().get(
            element_type, None)
        if vector_class is None:
            return
        return vector_class(np.array(element_dict['geometry']['vertices']))


class HdMapAdapter(object):
    """
    This class is used to load different specs of HD Maps, such as
    DeepMotion HDMap, shapefiles, geojson etc.
    """
    def __init__(self):
        pass

    def load(self, hd_map_file_path: str):
        # TODO: Implement other loading interfaces.
        if os.path.splitext(hd_map_file_path)[-1] == '.json':
            return self._load_json_file(hd_map_file_path)

    @staticmethod
    def _load_json_file(json_file_path: str):
        with open(json_file_path, 'r+') as fin:
            hd_map_dict = json.load(fin)

        vectors = []
        for element_dict in hd_map_dict['semantic_elements'].values():
            vector = JsonHdMapVectorFactory.from_element_dict(element_dict)
            if vector is not None:
                vectors.append(vector)
        return vectors
