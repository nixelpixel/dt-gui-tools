from dt_maps import Map, MapLayer
from classes.basic.command import Command


class MoveCommand(Command):
    _new_position: tuple = (0, 0)
    _frame_name: str

    def __init__(self, frame_name: str, new_position: tuple):
        self._frame_name = frame_name
        self._new_position = new_position

    def execute(self, dm: Map, layer: MapLayer, layer_name: str) -> None:
        if layer_name == "frames":
            dm.layers[layer_name][self._frame_name].pose.x = self._new_position[0]
            dm.layers[layer_name][self._frame_name].pose.y = self._new_position[1]
