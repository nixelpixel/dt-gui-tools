from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Optional

from mapStorage import MapStorage


class AbstractLayer(ABC):
    __layer_name__ = ""

    def __init__(self, layer_name: str) -> None:
        self.__layer_name__ = layer_name
        self._data = MapStorage().map.layers[layer_name]

    def render(self) -> None:
        pass
