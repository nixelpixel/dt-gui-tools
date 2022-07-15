from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from dt_maps import MapLayer
from mapStorage import MapStorage
from utils.maps import create_layer
from typing import  Dict, Any


class AbstractLayer(ABC):
    _data: MapLayer = None

    def __init__(self) -> None:
        self.dm = MapStorage().map
        try:
            self.data = self.dm.layers[self.layer_name()]
        except KeyError:
            logging.error(f"Empty layer {self.layer_name()}")
            create_layer(self.dm, self.layer_name(), {})
            self.data = self.dm.layers[self.layer_name()]

    @abstractmethod
    def layer_name(self) -> str:
        pass

    def render(self) -> None:
        pass

    @abstractmethod
    def default_conf(self) -> Dict[str, Any]:
        pass

    def check_config(self, config: Dict[str, Any]) -> bool:
        pass
