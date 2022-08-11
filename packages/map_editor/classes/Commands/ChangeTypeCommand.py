from dt_maps import Map, MapLayer
from classes.basic.command import Command


class ChangeTypeCommand(Command):
    _new_type: str
    _name: str

    def __init__(self, layer_name: str, name: str, new_type: str):
        self._name = name
        self._new_type = new_type
        self._layer_name = layer_name

    def execute(self, dm: Map, layer: MapLayer, layer_name: str, *args, **kwargs) -> None:
        if layer_name == self._layer_name:
            dm.layers[layer_name][self._name].type = self._new_type
