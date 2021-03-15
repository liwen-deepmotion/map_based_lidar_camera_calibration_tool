import json
from typing import List, Dict

from correspondence.correspondence import Correspondence
from layer.base_layer import BaseLayer


class CorrespondenceLayer(BaseLayer):

    def __init__(self, editor: 'MapBasedCalibrator'):
        super().__init__(editor)

        self._timestamp_to_correspondences_map = {
        }  # type: Dict[int, List[Correspondence]]

    def load(self, correspondence_file_path: str):
        with open(correspondence_file_path, 'r+') as f:
            correspondences_dict = json.load(f)

        for keyframe_dict in correspondences_dict['keyframes']:
            timestamp = int(keyframe_dict['timestamp'])
            for correspondence_dict in \
                    keyframe_dict['line_line_correspondences'] + \
                    keyframe_dict['point_point_correspondences']:
                correspondence = Correspondence()
                correspondence.from_json_dict(correspondence_dict)
                correspondence.set_timestamp(timestamp)
                self._timestamp_to_correspondences_map \
                    .setdefault(timestamp, []).append(correspondence)
        print('{} correspondences loaded.'.format(len(self.correspondences())))

    def save(self, correspondence_file_path: str):
        keyframes_list = []
        for timestamp, correspondences in \
                self._timestamp_to_correspondences_map.items():
            line_correspondence_dicts = []
            point_correspondence_dicts = []
            for correspondence in correspondences:
                if correspondence.is_line_correspondence():
                    line_correspondence_dicts.append(
                        correspondence.to_json_dict())
                else:
                    point_correspondence_dicts.append(
                        correspondence.to_json_dict())
            keyframe_dict = {
                'timestamp':  int(timestamp),
                'line_line_correspondences': line_correspondence_dicts,
                'point_point_correspondences': point_correspondence_dicts
            }
            keyframes_list.append(keyframe_dict)

        json_dict = {
            'keyframes': keyframes_list,
        }
        with open(correspondence_file_path, 'w+') as f:
            json.dump(json_dict,
                      f,
                      ensure_ascii=False,
                      indent=4,
                      sort_keys=True)

    def correspondences(self) -> List[Correspondence]:
        return [correspondence
                for correspondences in
                self._timestamp_to_correspondences_map.values()
                for correspondence in correspondences]

    def get_correspondences_by_timestamp(self, timestamp: int) \
            -> List[Correspondence]:
        return self._timestamp_to_correspondences_map.get(timestamp, [])

    def add_correspondence(self, correspondence: Correspondence):
        if correspondence.timestamp() == 0:
            timestamp = self.editor.trajectory_navigator \
                .current_trajectory_node().timestamp()
        else:
            timestamp = correspondence.timestamp()
        self._timestamp_to_correspondences_map.setdefault(
            timestamp, []).append(correspondence)
        # Assign id to correspondence.
        correspondence.set_id(
            len(self._timestamp_to_correspondences_map[timestamp]) - 1)
        self.renderer.add_actor(correspondence.build_actor())

    def remove_correspondence(self, correspondence: Correspondence):
        correspondence_list = self._timestamp_to_correspondences_map \
            .setdefault(correspondence.timestamp(), [])
        if correspondence in correspondence_list:
            correspondence_list.remove(correspondence)
            self.renderer.remove_actor(correspondence.actor())

    def clear(self):
        for correspondence in self.correspondences():
            self.renderer.remove_actor(correspondence.actor())
        self._timestamp_to_correspondences_map = {}
