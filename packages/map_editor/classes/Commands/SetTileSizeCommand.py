from dt_maps import Map, MapLayer
from classes.basic.command import Command


class SetTileSizeCommand(Command):
    _new_size: tuple = (0.0, 0.0)
    _tile_map_name: str

    def __init__(self, tile_map_name: str, new_size: tuple):
        self._tile_map_name = tile_map_name
        self._new_size = new_size

    def execute(self, dm: Map, layer: MapLayer, layer_name: str, *args, **kwargs) -> None:
        if layer_name == "tile_maps":
            dm.layers[layer_name][self._tile_map_name]["tile_size"]['x'] = self._new_size[0]
            dm.layers[layer_name][self._tile_map_name]["tile_size"]['y'] = self._new_size[1]
