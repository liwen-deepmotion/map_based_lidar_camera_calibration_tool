from enum import Enum


class KeyCombo(object):

    def __init__(self, key_sequence: str, description: str = ''):
        self.key_seq = key_sequence

        self.description = description

    def is_same(self, other: 'KeyCombo') -> bool:
        return self.key_seq == other.key_seq


class HotKey(Enum):
    RESET_CAMERA = KeyCombo('Space', 'Reset Camera')
    PREV_FRAME = KeyCombo('A', 'Prev Frame')
    NEXT_FRAME = KeyCombo('D', 'Next Frame')
    ADD_CORRESPONDENCE = KeyCombo('W', 'Add Correspondence')
    CANCEL = KeyCombo('Q', 'Cancel')
    TOGGLE_OPTIMIZATION_RESULT = KeyCombo('S', 'Toggle Optimization')
    TOGGLE_CORRESPONDENCE_ID_DISPLAYING = \
        KeyCombo('E', 'Toggle Correspondence ID')
