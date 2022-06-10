from __future__ import annotations
from abc import ABC, abstractmethod
from dt_maps import MapLayer
from mapStorage import MapStorage


class AbstractLayer(ABC):
    _data: MapLayer = None

    def __init__(self) -> None:
        self.dm = MapStorage().map
        self.data = self.dm.layers[self.layer_name()]

    @abstractmethod
    def layer_name(self) -> str:
        pass

    def render(self) -> None:
        pass
