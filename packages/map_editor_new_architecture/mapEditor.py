# -*- coding: utf-8 -*-
from collections import deque

from map_editor_new_architecture.mapStorage import MapStorage


class MapEditor:
    def __init__(self):
        self.map = MapStorage().get_map_instance()
        self.memento = deque([], maxlen=150)
