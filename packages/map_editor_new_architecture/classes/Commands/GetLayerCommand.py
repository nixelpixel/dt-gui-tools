from typing import Optional

from dt_maps import Map, MapLayer

from classes.basic.command import Command


class GetLayerCommand(Command):
    _layer_name: str = None

    def __init__(self, layer_name) -> None:
        self._layer_name = layer_name

    def execute(self, dm: Map,
                layer: MapLayer,
                layer_name: str,
                default_conf: dict) -> Optional[MapLayer]:
        if layer_name == self._layer_name:
            return layer
