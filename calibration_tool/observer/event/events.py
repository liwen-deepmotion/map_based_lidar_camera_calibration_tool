# Author: Liwen Liu (liwenliu@deepmotion.ai)


from PyQt5.QtCore import QEvent


OFFSET = 0


def next_offset() -> int:
    global OFFSET
    OFFSET += 1
    return OFFSET


class CustomEvent(QEvent):
    LeftButtonPressedEvent = QEvent.User + next_offset()
    RightButtonPressedEvent = QEvent.User + next_offset()
    MiddleButtonPressedEvent = QEvent.User + next_offset()
    LeftButtonReleasedEvent = QEvent.User + next_offset()
    RightButtonReleasedEvent = QEvent.User + next_offset()
    MiddleButtonReleasedEvent = QEvent.User + next_offset()
    MouseMoveEvent = QEvent.User + next_offset()
    MouseWheelEvent = QEvent.User + next_offset()

    KeyComboPressedEvent = QEvent.User + next_offset()

    VectorMapLoadedEvent = QEvent.User + next_offset()
    TrajectoryLoadedEvent = QEvent.User + next_offset()
    CorrespondencesLoadedEvent = QEvent.User + next_offset()

    TrajectoryNodeChangedEvent = QEvent.User + next_offset()

    CalibrationOptimizedEvent = QEvent.User + next_offset()
