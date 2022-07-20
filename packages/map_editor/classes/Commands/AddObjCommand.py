from dt_maps import Map, MapLayer
from classes.basic.command import Command
from utils.maps import set_obj


class AddObjCommand(Command):
    _layer_name: str
    _object_name: str

    def __init__(self, layer_name: str, object_name: str):
        self._layer_name = layer_name
        self._object_name = object_name

    def execute(self, dm: Map, layer: MapLayer, layer_name: str, default_conf: dict, *args, **kwargs) -> None:
        if layer_name == self._layer_name:
            set_obj(dm, layer, layer_name, self._object_name, default_conf)
