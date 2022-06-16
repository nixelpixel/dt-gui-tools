from dt_maps import Map, MapLayer
from classes.basic.command import Command


class ChangeTileTypeCommand(Command):
    _new_type: str
    _tile_name: str

    def __init__(self, tile_name: str, new_type: str):
        self._tile_name = tile_name
        self._new_type = new_type

    def execute(self, dm: Map, layer: MapLayer, layer_name: str, default_conf: dict) -> None:
        if layer_name == "tiles":
            dm.layers[layer_name][self._tile_name].type = self._new_type
