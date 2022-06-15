from dt_maps import Map, MapLayer
from classes.basic.command import Command


class RotateCommand(Command):
    _new_angle: float = 0.0
    _frame_name: str

    def __init__(self, frame_name: str, new_angle: float):
        self._frame_name = frame_name
        self._new_angle = new_angle

    def execute(self, dm: Map, layer: MapLayer, layer_name: str) -> None:
        if layer_name == "frames":
            dm.layers[layer_name][self._frame_name].pose.yaw = self._new_angle
