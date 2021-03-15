#!/usr/bin/env python3
# Author: Zhijun Xie (zhijunxie@deepmotion.ai)


import traceback

from typing import (TYPE_CHECKING, Callable, List, Dict)

if TYPE_CHECKING:
    from map_based_calibrator import MapBasedCalibrator


class CallbackItem(object):

    def __init__(self,
                 priority: int = -1,
                 event: int = -1,
                 callback: Callable = None):
        self.priority = priority
        self.event = event
        self.callback = callback


class EventHandler(object):
    """
    (1) Keep a list of callbacks sorted by priority in descending order.
    (2) Dispatch and invoke events.
    """

    def __init__(self, editor: 'MapBasedCalibrator'):
        self.editor = editor
        self.event_to_callbacks_map = \
            {}  # type: Dict[int, List[CallbackItem]]

    def add_observer(self,
                     event: int,
                     callback: Callable,
                     priority: int) -> None:
        callback_items = self.event_to_callbacks_map.setdefault(
            event, [])
        callback_items.append(CallbackItem(priority, event, callback))
        callback_items.sort(key=lambda item: item.priority,
                            reverse=True)

    def remove_observer(self, event: int, callback: Callable, priority):
        callback_items = self.event_to_callbacks_map.setdefault(
            event, [])

        for item in reversed(callback_items):
            if item.event == event and item.callback == callback \
                    and item.priority == priority:
                callback_items.remove(item)

    def invoke_event(self, event: int, *args, **kwargs):
        callback_items = self.event_to_callbacks_map.get(event, [])
        for item in callback_items:
            try:
                item.callback(*args, **kwargs)
            except Exception as error:
                traceback.print_exc()
