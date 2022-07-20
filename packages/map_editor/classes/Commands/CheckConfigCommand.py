from dt_maps import Map, MapLayer
from classes.basic.command import Command
from typing import Dict, Any


class CheckConfigCommand(Command):
    _layer_name: str
    _new_config: Dict[str, Any]

    def __init__(self, layer_name: str, new_config: Dict[str, Any]):
        self._layer_name = layer_name
        self._new_config = new_config

    def execute(self, dm: Map, layer: MapLayer,
                layer_name: str, *args, **kwargs) -> None:
        if layer_name == self._layer_name:
            check_config = kwargs["check_config"]
            return check_config(self._new_config)
