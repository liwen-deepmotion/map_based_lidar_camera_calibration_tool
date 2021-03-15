from typing import Dict

import numpy as np

from actor.correspondence_actor import CorrespondenceActor
from shape.point_2d import Point2D
from shape.polyline_2d import Polyline2D
from shape.shape import Shape


class Correspondence(object):

    def __init__(self, shape: Shape = None,
                 reprojected_shape: Shape = None,
                 correspondence_id: int = 0):
        self._shape = shape

        self._reprojected_shape = reprojected_shape

        self._timestamp = int()

        self._is_on_road = True

        self._actor = CorrespondenceActor()

        # This id is used in calibration_optimizer to index the
        # according correspondences.
        self._id = correspondence_id

    def from_json_dict(self, json_dict: Dict):
        # FIXME: fix the point correspondence loading bug.
        if "annotation_segment" in json_dict:
            shape_coords = json_dict['annotation_segment']
            vector_coords = json_dict['hd_map_segment']

            reprojected_coords = np.zeros(np.array(shape_coords).shape)
            self._shape = Polyline2D(
                np.array(shape_coords).reshape(-1, 2))
            self._reprojected_shape = Polyline2D(
                np.array(reprojected_coords).reshape((-1, 2)))
            self._reprojected_shape.set_origin_vertices(
                np.array(vector_coords).reshape((-1, 3)))
        else:
            shape_coords = json_dict['annotation_point']
            vector_coords = json_dict['hd_map_point']

            reprojected_coords = np.zeros(np.array(shape_coords).shape)
            self._shape = Point2D(
                np.array(shape_coords).reshape(-1, 2))
            self._reprojected_shape = Point2D(
                np.array(reprojected_coords).reshape((-1, 2)))
            self._reprojected_shape.set_origin_vertices(
                np.array(vector_coords).reshape((-1, 3)))
        self._id = int(json_dict["id"])

        self._is_on_road = json_dict['is_on_road']

    def to_json_dict(self) -> Dict:
        # TODO: Maybe inherit to remove the judgements.
        if self.is_line_correspondence():
            shape_coords = self._shape.coords().tolist()
            vector_coords = self._reprojected_shape.origin_vertices().tolist()
            shape_key = "annotation_segment"
            vector_key = "hd_map_segment"
        else:
            shape_coords = self._shape.coords()[0].tolist()
            vector_coords = self._reprojected_shape.origin_vertices()[0].tolist()
            shape_key = "annotation_point"
            vector_key = "hd_map_point"

        json_dict = {
            shape_key: shape_coords,
            vector_key: vector_coords,
            'id': self._id,
            'is_on_road': self.is_on_road(),
        }
        return json_dict

    def is_valid(self):
        return self._shape is not None and \
               self._reprojected_shape is not None and \
               self._shape.size() == self._reprojected_shape.size()

    def is_line_correspondence(self):
        return self._shape.size() == 2

    def shape(self) -> Shape:
        return self._shape

    def reprojected_shape(self) -> Shape:
        return self._reprojected_shape

    def timestamp(self) -> int:
        return self._timestamp

    def is_on_road(self) -> bool:
        return self._is_on_road

    def set_id(self, id_: int):
        self._id = id_

    def id(self) -> int:
        return self._id

    def set_shape(self, shape: Shape):
        self._shape = shape

    def set_reprojected_shape(self, shape: Shape):
        self._reprojected_shape = shape

    def set_timestamp(self, timestamp: int):
        self._timestamp = timestamp

    def set_is_on_road(self, is_on_road: bool):
        self._is_on_road = is_on_road

    def actor(self) -> CorrespondenceActor:
        return self._actor

    def build_actor(self):
        self._actor.geometry().set_image_coords(self._shape.coords())
        self._actor.set_correspondence_coords(
            self._reprojected_shape.coords())
        if self._is_on_road:
            # blue = (0, 191, 255)
            blue = (220, 20, 60)
            self._actor.property().set_color(*blue)
            self._actor.set_point_color(blue)
        else:
            pink = (220, 20, 60)
            self._actor.property().set_color(*pink)
            self._actor.set_point_color(pink)
        self._actor.set_correspondence_color((255, 255, 0))
        self._actor.property().set_line_width(5)
        return self._actor

    def __str__(self):
        return 'Correspondence: shape_coords: {}, vector_coords: {}, vector_vertices:{}, timestamp: {}'.format(
            self._shape.coords(), self._reprojected_shape.coords(), self._reprojected_shape.origin_vertices(), self.timestamp())